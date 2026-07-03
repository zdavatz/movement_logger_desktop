//! Headless BLE diagnostic (`MovementLogger --ble-debug [seconds]`).
//!
//! The GPS-Debug-style probe for the BLE side: everything the Mac's
//! CoreBluetooth stack delivers is printed live to the terminal so a
//! remote user (Peter) can run one command and send us the dump.
//! Unlike the GUI scan (which enumerates `peripherals()` once at the
//! end of the window), this subscribes to `adapter.events()` and
//! prints every DeviceDiscovered/DeviceUpdated as it happens — so we
//! can see *whether and when* the box's advertisements reach
//! CoreBluetooth at all, versus being seen but name-less, versus
//! being filtered before delivery.
//!
//! Sequence:
//!   1. adapter + saved config (box_id) dump
//!   2. event-driven scan for N seconds (default 20)
//!   3. summary: every distinct device, name/name-source, match flags
//!   4. macOS retrieve-by-id test for the saved box_id (does
//!      CoreBluetooth still resolve the identifier without a scan?)
//!   5. full connect probe against the box (bounded every step, same
//!      timeouts as ble.rs): connect → discover → list characteristics
//!      → subscribe FileData → write LIST → count reply notifies →
//!      0x0F polite disconnect → disconnect.
//!
//! Like `--agent`, `main()` dispatches here before any eframe/egui/
//! winit code — no window is created. Terminal needs Bluetooth
//! permission on macOS (System Settings → Privacy & Security →
//! Bluetooth) — a zero-device scan is the symptom of a missing grant.
//!
//! `RUST_LOG=btleplug=debug` surfaces btleplug's internal CoreBluetooth
//! log lines on stderr for even deeper traces.

use btleplug::api::{
    Central, CentralEvent, Manager as _, Peripheral as _, ScanFilter, WriteType,
};
use btleplug::platform::Manager;
use futures::stream::StreamExt;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use uuid::Uuid;

use crate::agent_config::AgentConfig;

const BOX_NAMES: &[&str] = &["PumpTsueri", "STBoxFs"];

const FILECMD_UUID: Uuid = Uuid::from_u128(0x00000080_0010_11e1_ac36_0002a5d5c51b);
const FILEDATA_UUID: Uuid = Uuid::from_u128(0x00000040_0010_11e1_ac36_0002a5d5c51b);
const STREAM_UUID: Uuid = Uuid::from_u128(0x00000100_0010_11e1_ac36_0002a5d5c51b);

const OP_LIST: u8 = 0x01;
const OP_DISCONNECT: u8 = 0x0F;

/// Same bound as ble.rs `LINK_OP_TIMEOUT` — no CoreBluetooth await may
/// run unbounded (vendored-fork map holes never resolve their reply).
const STEP_TIMEOUT: Duration = Duration::from_secs(12);
const LIST_WAIT: Duration = Duration::from_secs(6);

macro_rules! say {
    ($t0:expr, $($arg:tt)*) => {{
        println!("[{:>7.2}s] {}", $t0.elapsed().as_secs_f32(), format!($($arg)*));
    }};
}

pub fn run(secs: u64) -> i32 {
    // btleplug internal logs → stderr. Default to warn so the probe's
    // own stdout stays readable; RUST_LOG overrides for deep traces.
    let _ = env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("warn"),
    )
    .try_init();
    let rt = match tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
    {
        Ok(rt) => rt,
        Err(e) => {
            eprintln!("[ble-debug] tokio runtime: {e}");
            return 1;
        }
    };
    match rt.block_on(run_async(secs)) {
        Ok(()) => 0,
        Err(e) => {
            eprintln!("[ble-debug] FAILED: {e}");
            1
        }
    }
}

fn fmt_mfr(mfr: &HashMap<u16, Vec<u8>>) -> String {
    if mfr.is_empty() {
        return "-".into();
    }
    mfr.iter()
        .map(|(k, v)| format!("{:04x}:{}", k, hex(v)))
        .collect::<Vec<_>>()
        .join(" ")
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

/// Raw-adv print gate: only the box (by saved id or an already-seen
/// matching name) gets its advertisement payloads echoed.
fn is_box_id(
    ids: &str,
    saved_id: &Option<String>,
    seen: &HashMap<String, (Option<String>, u32)>,
) -> bool {
    saved_id.as_deref() == Some(ids)
        || seen
            .get(ids)
            .and_then(|(n, _)| n.as_deref())
            .map(|n| BOX_NAMES.iter().any(|w| n.contains(w)))
            .unwrap_or(false)
}

async fn run_async(secs: u64) -> Result<(), String> {
    let t0 = Instant::now();
    say!(t0, "MovementLogger --ble-debug v{} ({})",
         env!("CARGO_PKG_VERSION"), std::env::consts::OS);

    let cfg = AgentConfig::load();
    say!(t0, "config.toml: box_id={:?} keep_synced={} log_mode_manual={:?}",
         cfg.box_id, cfg.keep_synced, cfg.log_mode_manual);
    let saved_id = cfg.box_id.clone();

    let manager = Manager::new().await.map_err(|e| format!("manager: {e}"))?;
    let adapters = manager
        .adapters()
        .await
        .map_err(|e| format!("adapters: {e}"))?;
    say!(t0, "{} adapter(s)", adapters.len());
    let adapter = adapters.into_iter().next().ok_or("no BLE adapter")?;
    say!(t0, "adapter: {}", adapter.adapter_info().await.unwrap_or_default());

    // ----- event-driven scan ---------------------------------------------
    // Subscribe BEFORE start_scan so no discovery event is lost.
    let mut events = adapter
        .events()
        .await
        .map_err(|e| format!("events: {e}"))?;
    say!(t0, "scanning for {secs}s (every CoreBluetooth event printed live)…");
    adapter
        .start_scan(ScanFilter::default())
        .await
        .map_err(|e| format!("start_scan: {e}"))?;

    // id → (last printed name, update count)
    let mut seen: HashMap<String, (Option<String>, u32)> = HashMap::new();
    let mut box_id_live: Option<String> = None;
    let deadline = Instant::now() + Duration::from_secs(secs);
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() {
            break;
        }
        let ev = match tokio::time::timeout(left, events.next()).await {
            Ok(Some(ev)) => ev,
            Ok(None) => {
                say!(t0, "event stream closed unexpectedly");
                break;
            }
            Err(_) => break, // window over
        };
        match ev {
            CentralEvent::DeviceDiscovered(id) | CentralEvent::DeviceUpdated(id) => {
                let ids = id.to_string();
                let p = adapter.peripheral(&id).await.ok();
                let props = match &p {
                    Some(p) => p.properties().await.ok().flatten(),
                    None => None,
                };
                let name = props.as_ref().and_then(|pp| pp.local_name.clone());
                let rssi = props.as_ref().and_then(|pp| pp.rssi);
                let name_match = name
                    .as_deref()
                    .map(|n| BOX_NAMES.iter().any(|w| n.contains(w)))
                    .unwrap_or(false);
                let id_match = saved_id.as_deref() == Some(ids.as_str());
                if name_match && box_id_live.is_none() {
                    box_id_live = Some(ids.clone());
                }
                let entry = seen.entry(ids.clone()).or_insert((None, 0));
                entry.1 += 1;
                let first = entry.1 == 1;
                let name_changed = entry.0 != name;
                entry.0 = name.clone();
                // Print: every first sighting, every name change, every
                // event for the box itself (first 5, then every 20th so
                // a 100 ms adv interval doesn't flood the log).
                let is_box = name_match || id_match;
                let print = first
                    || name_changed
                    || (is_box && (entry.1 <= 5 || entry.1 % 20 == 0));
                if print {
                    let svcs = props
                        .as_ref()
                        .map(|pp| pp.services.len())
                        .unwrap_or(0);
                    let mfr = props
                        .as_ref()
                        .map(|pp| fmt_mfr(&pp.manufacturer_data))
                        .unwrap_or_else(|| "-".into());
                    say!(t0, "  {} id={} name={:?} rssi={:?} adv_svcs={} mfr={}{}{}",
                         if first { "NEW" } else { "upd" },
                         ids, name, rssi, svcs, mfr,
                         if name_match { "  <-- BOX (name)" } else { "" },
                         if id_match { "  <-- BOX (saved id)" } else { "" });
                }
            }
            // Raw advertisement-payload events fire at every foreign
            // device's adv interval (a HomeKit hub floods 2×/300 ms) —
            // print them only for the box, everything else is noise.
            CentralEvent::ManufacturerDataAdvertisement { id, manufacturer_data } => {
                let ids = id.to_string();
                if is_box_id(&ids, &saved_id, &seen) {
                    say!(t0, "  adv(mfr) id={} {}", ids, fmt_mfr(&manufacturer_data));
                }
            }
            CentralEvent::ServiceDataAdvertisement { id, service_data } => {
                let ids = id.to_string();
                if is_box_id(&ids, &saved_id, &seen) {
                    say!(t0, "  adv(svc-data) id={} {} entr(y/ies)", ids, service_data.len());
                }
            }
            CentralEvent::ServicesAdvertisement { id, services } => {
                let ids = id.to_string();
                if is_box_id(&ids, &saved_id, &seen) {
                    say!(t0, "  adv(svcs) id={} {:?}", ids, services);
                }
            }
            other => say!(t0, "  event: {other:?}"),
        }
    }
    let _ = adapter.stop_scan().await;

    // ----- summary --------------------------------------------------------
    say!(t0, "scan window over — {} distinct device(s):", seen.len());
    let mut rows: Vec<_> = seen.iter().collect();
    rows.sort_by(|a, b| b.1 .1.cmp(&a.1 .1));
    for (id, (name, count)) in &rows {
        let name_match = name
            .as_deref()
            .map(|n| BOX_NAMES.iter().any(|w| n.contains(w)))
            .unwrap_or(false);
        let id_match = saved_id.as_deref() == Some(id.as_str());
        say!(t0, "  {:>4} events  id={} name={:?}{}{}",
             count, id, name,
             if name_match { "  <-- BOX (name)" } else { "" },
             if id_match { "  <-- BOX (saved id)" } else { "" });
    }
    if seen.is_empty() {
        say!(t0, "ZERO devices — on macOS that almost always means the \
                  terminal app has no Bluetooth permission (System \
                  Settings → Privacy & Security → Bluetooth).");
    }

    // ----- retrieve-by-id test (macOS) -------------------------------------
    // Does CoreBluetooth still resolve the saved identifier without a
    // scan hit? This is the path Reconnect (last box) uses.
    let mut probe_id: Option<String> = box_id_live.clone();
    if let Some(idstr) = saved_id.as_deref() {
        #[cfg(target_os = "macos")]
        {
            say!(t0, "retrieve-by-id test for saved box_id {idstr}…");
            match Uuid::parse_str(idstr) {
                Ok(uuid) => {
                    let pid: btleplug::platform::PeripheralId = uuid.into();
                    match adapter.add_peripheral(&pid).await {
                        Ok(_) => {
                            let known = adapter
                                .peripherals()
                                .await
                                .unwrap_or_default()
                                .into_iter()
                                .any(|x| x.id().to_string() == idstr);
                            say!(t0, "  retrievePeripheralsWithIdentifiers: {}",
                                 if known { "RESOLVED — identifier still known to this Mac" }
                                 else { "NOT RESOLVED — identifier unknown (box re-paired/re-addressed or CB db lost it)" });
                            if known && probe_id.is_none() {
                                probe_id = Some(idstr.to_string());
                            }
                        }
                        Err(e) => say!(t0, "  retrieve-by-id FAILED: {e}"),
                    }
                }
                Err(e) => say!(t0, "  saved box_id is not a UUID ({e}) — skipping"),
            }
        }
        #[cfg(not(target_os = "macos"))]
        {
            let _ = idstr;
            say!(t0, "retrieve-by-id test skipped (macOS-only path)");
        }
    } else {
        say!(t0, "no saved box_id — retrieve-by-id test skipped");
    }

    // ----- connect probe ----------------------------------------------------
    let Some(target) = probe_id else {
        say!(t0, "no box to connect to — probe ends here");
        return Ok(());
    };
    let p = adapter
        .peripherals()
        .await
        .map_err(|e| format!("peripherals: {e}"))?
        .into_iter()
        .find(|x| x.id().to_string() == target)
        .ok_or("box peripheral vanished before connect")?;

    say!(t0, "connect probe → {target} (each step bounded {}s)…",
         STEP_TIMEOUT.as_secs());
    match tokio::time::timeout(STEP_TIMEOUT, p.connect()).await {
        Ok(Ok(())) => say!(t0, "  connect OK"),
        Ok(Err(e)) => {
            say!(t0, "  connect FAILED: {e}");
            return Ok(());
        }
        Err(_) => {
            say!(t0, "  connect TIMED OUT after {}s — cancelling", STEP_TIMEOUT.as_secs());
            let _ = tokio::time::timeout(Duration::from_secs(3), p.disconnect()).await;
            return Ok(());
        }
    }

    match tokio::time::timeout(STEP_TIMEOUT, p.discover_services()).await {
        Ok(Ok(())) => {}
        Ok(Err(e)) => say!(t0, "  discover_services error: {e} (continuing — macOS auto-discovers)"),
        Err(_) => say!(t0, "  discover_services timed out (continuing)"),
    }
    let chars = p.characteristics();
    say!(t0, "  {} characteristic(s):", chars.len());
    for c in &chars {
        say!(t0, "    {} props={:?}", c.uuid, c.properties);
    }
    let cmd_char = chars.iter().find(|c| c.uuid == FILECMD_UUID).cloned();
    let data_char = chars.iter().find(|c| c.uuid == FILEDATA_UUID).cloned();
    say!(t0, "  FileCmd {} / FileData {} / Stream {}",
         if cmd_char.is_some() { "present" } else { "MISSING" },
         if data_char.is_some() { "present" } else { "MISSING" },
         if chars.iter().any(|c| c.uuid == STREAM_UUID) { "present" } else { "absent (legacy fw)" });

    if let (Some(cmd), Some(data)) = (cmd_char.clone(), data_char) {
        match tokio::time::timeout(STEP_TIMEOUT, p.subscribe(&data)).await {
            Ok(Ok(())) => say!(t0, "  subscribe FileData OK"),
            Ok(Err(e)) => say!(t0, "  subscribe FileData FAILED: {e}"),
            Err(_) => say!(t0, "  subscribe FileData timed out"),
        }
        let mut notifs = match p.notifications().await {
            Ok(s) => s,
            Err(e) => return Err(format!("notifications: {e}")),
        };
        say!(t0, "  writing LIST (0x01), waiting {}s for reply rows…",
             LIST_WAIT.as_secs());
        match tokio::time::timeout(
            STEP_TIMEOUT,
            p.write(&cmd, &[OP_LIST], WriteType::WithoutResponse),
        )
        .await
        {
            Ok(Ok(())) => {
                let mut rows = 0usize;
                let ldl = Instant::now() + LIST_WAIT;
                loop {
                    let left = ldl.saturating_duration_since(Instant::now());
                    if left.is_zero() {
                        break;
                    }
                    match tokio::time::timeout(left, notifs.next()).await {
                        Ok(Some(n)) if n.uuid == FILEDATA_UUID => {
                            rows += 1;
                            if rows <= 3 {
                                say!(t0, "    notify {} B head={}", n.value.len(),
                                     hex(&n.value[..n.value.len().min(12)]));
                            }
                        }
                        Ok(Some(_)) => {}
                        _ => break,
                    }
                }
                say!(t0, "  LIST replies: {rows} notif(s) — {}",
                     if rows > 0 { "END-TO-END LINK WORKS" } else { "no reply (box busy or link dead)" });
            }
            Ok(Err(e)) => say!(t0, "  LIST write FAILED: {e}"),
            Err(_) => say!(t0, "  LIST write timed out"),
        }
    }

    // Polite box-side teardown (0x0F, fw v0.0.22+; older fw ignores it),
    // then the local disconnect — same order as ble.rs disconnect_inner.
    if let Some(cmd) = cmd_char {
        match tokio::time::timeout(
            Duration::from_secs(3),
            p.write(&cmd, &[OP_DISCONNECT], WriteType::WithResponse),
        )
        .await
        {
            Ok(Ok(())) => say!(t0, "  0x0F host-disconnect ACKed"),
            Ok(Err(e)) => say!(t0, "  0x0F write failed: {e} (legacy fw is fine)"),
            Err(_) => say!(t0, "  0x0F write timed out"),
        }
    }
    let _ = tokio::time::timeout(Duration::from_secs(3), p.disconnect()).await;
    say!(t0, "probe done");
    Ok(())
}
