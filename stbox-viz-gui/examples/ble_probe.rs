//! Headless BLE diagnostic for the macOS connect-flakiness issue
//! (zdavatz/movement_logger_desktop#1).
//!
//! Replicates the scan + connect + service-discovery + subscribe
//! sequence from `src/ble.rs` without the egui layer, so we can run it
//! repeatedly from a terminal and capture which step dies. Each step
//! prints a wall-clock-stamped line to stdout; btleplug's own log
//! messages go to stderr via env_logger.
//!
//! Run:
//!   cargo build --release --example ble_probe -p movement-logger
//!   RUST_LOG=info ./target/release/examples/ble_probe
//!
//! For deeper trace into btleplug's CoreBluetooth backend:
//!   RUST_LOG=btleplug=debug ./target/release/examples/ble_probe
//!
//! macOS note: the parent terminal (Terminal.app / iTerm.app) needs
//! Bluetooth permission. If the first run shows zero peripherals,
//! check System Settings → Privacy & Security → Bluetooth.

use btleplug::api::{Central, Manager as _, Peripheral as _, ScanFilter};
use btleplug::platform::Manager;
use futures::stream::StreamExt;
use std::time::{Duration, Instant};
use uuid::Uuid;

const BOX_NAMES: &[&str] = &["PumpTsueri", "STBoxFs"];

const FILECMD_UUID:  Uuid = Uuid::from_u128(0x00000080_0010_11e1_ac36_0002a5d5c51b);
const FILEDATA_UUID: Uuid = Uuid::from_u128(0x00000040_0010_11e1_ac36_0002a5d5c51b);
const STREAM_UUID:   Uuid = Uuid::from_u128(0x00000100_0010_11e1_ac36_0002a5d5c51b);

const SCAN_DURATION:     Duration = Duration::from_secs(8);
const NOTIFY_WAIT:       Duration = Duration::from_secs(5);

macro_rules! step {
    ($start:expr, $($arg:tt)*) => {{
        println!("[{:>6.2}s] {}", $start.elapsed().as_secs_f32(), format!($($arg)*))
    }};
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let t0 = Instant::now();

    step!(t0, "probe starting");

    let manager = Manager::new().await?;
    let adapters = manager.adapters().await?;
    let adapter = adapters.into_iter().next().ok_or("no BLE adapter")?;
    step!(t0, "adapter ready: {}", adapter.adapter_info().await.unwrap_or_default());

    // ----- SCAN -----
    step!(t0, "starting scan for {}s", SCAN_DURATION.as_secs());
    adapter.start_scan(ScanFilter::default()).await?;
    tokio::time::sleep(SCAN_DURATION).await;
    adapter.stop_scan().await?;

    let peripherals = adapter.peripherals().await?;
    step!(t0, "scan saw {} peripheral(s)", peripherals.len());

    let mut box_peripheral = None;
    for p in &peripherals {
        let props = p.properties().await.ok().flatten();
        let name  = props.as_ref().and_then(|pp| pp.local_name.clone());
        let rssi  = props.as_ref().and_then(|pp| pp.rssi);
        let addr  = props.as_ref().map(|pp| pp.address.to_string()).unwrap_or_default();
        let matched = name.as_deref()
            .map(|n| BOX_NAMES.iter().any(|w| n.contains(w)))
            .unwrap_or(false);
        step!(t0, "  seen: addr={} name={:?} rssi={:?}{}",
              addr, name, rssi, if matched { "  <-- MATCH" } else { "" });
        if matched && box_peripheral.is_none() {
            box_peripheral = Some(p.clone());
        }
    }

    let Some(p) = box_peripheral else {
        step!(t0, "no box found among scanned peripherals — abort");
        return Ok(());
    };

    // ----- CONNECT -----
    step!(t0, "connecting…");
    if let Err(e) = p.connect().await {
        step!(t0, "connect FAILED: {e}");
        return Ok(());
    }
    step!(t0, "connected");

    // ----- DISCOVER SERVICES -----
    step!(t0, "discover_services…");
    if let Err(e) = p.discover_services().await {
        step!(t0, "discover_services FAILED: {e}");
        let _ = p.disconnect().await;
        return Ok(());
    }
    let chars = p.characteristics();
    step!(t0, "discover_services OK — {} characteristics", chars.len());
    for c in &chars {
        step!(t0, "    char {} props={:?}", c.uuid, c.properties);
    }

    let data_char = match chars.iter().find(|c| c.uuid == FILEDATA_UUID).cloned() {
        Some(c) => { step!(t0, "FileData characteristic present"); c }
        None => {
            step!(t0, "FileData characteristic MISSING — old firmware or stale cache");
            let _ = p.disconnect().await;
            return Ok(());
        }
    };
    if chars.iter().any(|c| c.uuid == FILECMD_UUID) {
        step!(t0, "FileCmd  characteristic present");
    } else {
        step!(t0, "FileCmd  characteristic MISSING");
    }

    // ----- SUBSCRIBE -----
    step!(t0, "subscribe FileData…");
    if let Err(e) = p.subscribe(&data_char).await {
        step!(t0, "subscribe FileData FAILED: {e}");
        let _ = p.disconnect().await;
        return Ok(());
    }
    step!(t0, "subscribe FileData OK");

    if let Some(stream_char) = chars.iter().find(|c| c.uuid == STREAM_UUID).cloned() {
        step!(t0, "subscribe SensorStream…");
        match p.subscribe(&stream_char).await {
            Ok(()) => step!(t0, "subscribe SensorStream OK"),
            Err(e) => step!(t0, "subscribe SensorStream soft-fail: {e}"),
        }
    } else {
        step!(t0, "SensorStream characteristic not advertised (legacy firmware)");
    }

    // ----- WAIT FOR NOTIFIES -----
    step!(t0, "opening notification stream, waiting {}s for any notify…",
          NOTIFY_WAIT.as_secs());
    let mut stream = p.notifications().await?;
    let mut count = 0usize;
    let deadline = Instant::now() + NOTIFY_WAIT;
    while let Some(notif) = tokio::time::timeout(
        deadline.saturating_duration_since(Instant::now()),
        stream.next(),
    ).await.ok().flatten() {
        count += 1;
        if count <= 3 {
            step!(t0, "  notify: uuid={} {} bytes head={:02x?}",
                  notif.uuid, notif.value.len(),
                  &notif.value[..notif.value.len().min(8)]);
        }
        if Instant::now() >= deadline { break; }
    }
    step!(t0, "notify wait done — {} notifies received", count);

    // ----- DISCONNECT -----
    step!(t0, "disconnecting…");
    let _ = p.disconnect().await;
    step!(t0, "done");

    Ok(())
}
