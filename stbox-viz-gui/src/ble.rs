//! BLE backend for the FileSync panel.
//!
//! The egui UI is sync, btleplug is async — so we run a tokio
//! current-thread runtime on a dedicated worker thread and shuttle
//! commands and events across two `std::sync::mpsc` channels. The UI
//! drains events on every frame; the worker `select!`s between the
//! command channel and the persistent notification stream.
//!
//! ## Why one notification stream per connection (not per op)
//!
//! btleplug's `Peripheral::notifications()` returns a stream from
//! "subscription start"; opening a fresh one per LIST / READ risks
//! losing the first row / chunk if the box notifies before our `await`
//! is parked. So the stream is opened once on connect and lives until
//! disconnect; the worker loop demuxes incoming notifies into whichever
//! op is currently in-flight (Listing / Reading / Idle).
//!
//! ## Wire protocol — matches `Core/Src/ble_filesync.c`
//!
//!   FileCmd  00000080-0010-11e1-ac36-0002a5d5c51b   write w/o response
//!   FileData 00000040-0010-11e1-ac36-0002a5d5c51b   notify
//!
//!   0x01 LIST                — replies with `name,size\n` rows + `\n`
//!   0x02 READ   <name>       — streams raw file bytes; total == LIST size
//!   0x03 DELETE <name>       — single-byte status
//!   0x04 STOP_LOG            — no FileData reply
//!   0x05 START_LOG <dur32_LE> — box reboots into LOG mode for `dur` s,
//!                               then auto-reboots back to BLE (issue #15)
//!
//!   Status bytes: 0x00 OK, 0xB0 BUSY, 0xE1 NOT_FOUND, 0xE2 IO_ERROR, 0xE3 BAD_REQ.

use btleplug::api::{
    Central, Manager as _, Peripheral as _, ScanFilter, ValueNotification, WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use futures::stream::{Stream, StreamExt};
use std::pin::Pin;
use std::sync::mpsc::{channel, Receiver, Sender, TryRecvError};
use std::thread;
use std::time::{Duration, Instant};
use tokio::sync::mpsc as tokio_mpsc;
use uuid::Uuid;

/// Accepted advertise names. `PumpTsueri` is the legacy SDDataLogFileX
/// firmware; `STBoxFs` is the bare-metal PumpLogger firmware (Peter's
/// PR #18). Match either so a single GUI build handles both during the
/// firmware transition.
///
/// Matched as a **substring**, not exact equality: the BlueNRG-LP SDK
/// historically (pre-firmware-v0.0.3) populated the GAP Device Name
/// characteristic with `BlueNRG [<configured>]`, which macOS Core
/// Bluetooth then cached and replayed on scans — exact matching
/// silently dropped the box. See issue #1.
const BOX_NAMES: &[&str] = &["PumpTsueri", "STBoxFs"];

const FILECMD_UUID:  Uuid = Uuid::from_u128(0x00000080_0010_11e1_ac36_0002a5d5c51b);
const FILEDATA_UUID: Uuid = Uuid::from_u128(0x00000040_0010_11e1_ac36_0002a5d5c51b);
/// SensorStream — 0.5 Hz packed 46-byte all-sensor snapshot. New in
/// PumpLogger; not present in SDDataLogFileX (optional characteristic).
/// See DESIGN.md §3 for the byte layout.
const STREAM_UUID:   Uuid = Uuid::from_u128(0x00000100_0010_11e1_ac36_0002a5d5c51b);

/// Watchdog — if no notification arrives for this long during an active
/// LIST or READ, give up and surface a timeout error so the user isn't
/// left staring at "running…" forever (e.g. after a drop-out the box
/// reconnects but our subscription went stale).
const OP_IDLE_TIMEOUT: Duration = Duration::from_secs(20);
/* If LIST has produced at least one entry and no new bytes have arrived for
   this long, treat LIST as complete and go back to Idle. Defensive: the
   firmware's terminator notify can be missed/merged on flaky BLE links and
   without this fallback the GUI sits in `Listing` state until the 20 s
   OP_IDLE_TIMEOUT, blocking any Download click with "another op is in
   flight". 500 ms is comfortably above the firmware's per-tick processing
   delay (~50 ms per row) so we won't fire prematurely while LIST is still
   producing rows. */
const LIST_INACTIVITY_DONE: Duration = Duration::from_millis(500);

// ---------------------------------------------------------------------------
//  Public command / event API
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub enum BleCmd {
    /// Begin a 5-second scan; emits `Discovered` events as PumpTsueri
    /// peripherals appear, then `ScanStopped`.
    Scan,
    /// Connect by peripheral id (the platform-specific string from
    /// `Discovered.id`).
    Connect(String),
    Disconnect,
    /// Send a LIST opcode to the connected box.
    List,
    /// Send a READ opcode for `name`; expected total length is `size`
    /// from the prior LIST so the worker knows when to stop accumulating.
    Read { name: String, size: u64 },
    /// Side-channel STOP_LOG opcode — no FileData reply.
    StopLog,
    /// START_LOG opcode (issue #15) — box writes BKP1R/BKP2R + reboots
    /// into LOG mode. After `duration_seconds` elapses, box reboots
    /// back to BLE mode automatically. No FileData reply (BLE drops
    /// when box reboots). Caller should expect Disconnected event
    /// shortly after sending.
    StartLog { duration_seconds: u32 },
    /// DELETE opcode — drop `name` from the SD card. Single-byte status
    /// reply: 0x00 OK / 0xB0 BUSY / 0xE1 NOT_FOUND / 0xE2 IO_ERROR / 0xE3 BAD_REQUEST.
    Delete { name: String },
}

#[derive(Clone, Debug)]
pub enum BleEvent {
    Status(String),
    /// One advertising peripheral matching the BOX_NAME filter.
    Discovered { id: String, name: String, rssi: Option<i16> },
    ScanStopped,
    Connected,
    Disconnected,
    /// One entry from the LIST response.
    ListEntry { name: String, size: u64 },
    ListDone,
    ReadStarted { name: String, size: u64 },
    ReadProgress { name: String, bytes_done: u64 },
    ReadDone { name: String, content: Vec<u8> },
    /// DELETE finished successfully (status byte 0x00 received).
    DeleteDone { name: String },
    /// User-facing error — display in the log panel.
    Error(String),
    /// One decoded SensorStream snapshot (0.5 Hz). Only emitted while
    /// connected to a PumpLogger firmware that exposes the SensorStream
    /// characteristic; legacy PumpTsueri builds never produce this event.
    Sample(LiveSample),
}

/// One decoded SensorStream snapshot. Mirrors the 46-byte packed layout
/// from DESIGN.md §3, scaled into convenient SI-ish units so the UI
/// doesn't need to know the wire encoding.
#[derive(Clone, Copy, Debug)]
pub struct LiveSample {
    /// Box-local monotonic milliseconds since boot. Used as the X axis
    /// for time-series plots; not wall-clock time (the firmware has no
    /// RTC).
    pub timestamp_ms: u32,
    /// Linear acceleration, mg per axis (LSM6DSV16X).
    pub acc_mg: [i16; 3],
    /// Angular rate, centi-degrees per second per axis (= raw_LSB ×
    /// 1.75 truncated to i16; ±327.67 dps range). Divide by 100 for °/s.
    pub gyro_cdps: [i16; 3],
    /// Magnetic field, milligauss per axis (LIS2MDL).
    pub mag_mg: [i16; 3],
    /// Barometric pressure, raw Pascal (LPS22DF). Divide by 100 for hPa.
    pub pressure_pa: i32,
    /// Air temperature, 0.01 °C steps. Divide by 100 for °C.
    pub temperature_cc: i16,
    /// GPS latitude × 10⁷ (degrees). `0x7FFFFFFF` = no fix yet.
    pub gps_lat_e7: i32,
    /// GPS longitude × 10⁷.
    pub gps_lon_e7: i32,
    /// GPS altitude, signed metres.
    pub gps_alt_m: i16,
    /// GPS speed, cm/h × 10 (~ km/h × 100). Divide by 36 for km/h …
    pub gps_speed_cmh: i16,
    /// GPS course, centi-degrees (0..35999).
    pub gps_course_cdeg: i16,
    /// Fix quality. 0 = no fix, 1 = GPS, …
    pub gps_fix_q: u8,
    /// Number of satellites used in the current fix.
    pub gps_nsat: u8,
    /// True when the most-recent GPS fix is fresh (≤ 5 s old).
    pub gps_valid: bool,
    /// True when the firmware is also writing this snapshot to SD.
    /// Normally always set in the no-mode design.
    pub logging_active: bool,
    /// True when the firmware has raised the low-battery warning.
    pub low_battery: bool,
}

impl LiveSample {
    /// Decode the 46-byte little-endian wire layout (DESIGN.md §3).
    /// Returns None if `bytes` is the wrong length; the caller has
    /// already reassembled chunked frames into 46 contiguous bytes
    /// before reaching here.
    fn parse(bytes: &[u8]) -> Option<Self> {
        if bytes.len() != 46 { return None; }
        // Length is fixed, so the slice → array conversions can't fail.
        // `unwrap` keeps the call sites readable; the bounds check above
        // is the precondition.
        let u32_at = |o: usize| u32::from_le_bytes(bytes[o..o + 4].try_into().unwrap());
        let i32_at = |o: usize| i32::from_le_bytes(bytes[o..o + 4].try_into().unwrap());
        let i16_at = |o: usize| i16::from_le_bytes(bytes[o..o + 2].try_into().unwrap());
        let flags = bytes[44];
        Some(Self {
            timestamp_ms:    u32_at(0),
            acc_mg:          [i16_at(4),  i16_at(6),  i16_at(8)],
            gyro_cdps:       [i16_at(10), i16_at(12), i16_at(14)],
            mag_mg:          [i16_at(16), i16_at(18), i16_at(20)],
            pressure_pa:     i32_at(22),
            temperature_cc:  i16_at(26),
            gps_lat_e7:      i32_at(28),
            gps_lon_e7:      i32_at(32),
            gps_alt_m:       i16_at(36),
            gps_speed_cmh:   i16_at(38),
            gps_course_cdeg: i16_at(40),
            gps_fix_q:       bytes[42],
            gps_nsat:        bytes[43],
            gps_valid:       flags & 0x01 != 0,
            low_battery:     flags & 0x02 != 0,
            logging_active:  flags & 0x04 != 0,
        })
    }

    /// `lat_e7`/`lon_e7` as float degrees if the fix is valid, else None.
    pub fn lat_lon_deg(&self) -> Option<(f64, f64)> {
        if !self.gps_valid || self.gps_lat_e7 == i32::MAX { return None; }
        Some((self.gps_lat_e7 as f64 / 1.0e7, self.gps_lon_e7 as f64 / 1.0e7))
    }
}

// ---------------------------------------------------------------------------
//  Backend handle held by AppState
// ---------------------------------------------------------------------------

pub struct BleBackend {
    cmd_tx: Sender<BleCmd>,
    evt_rx: Receiver<BleEvent>,
}

impl BleBackend {
    /// Spawns the worker thread and its tokio runtime. Cheap — no BLE
    /// activity until the first command lands.
    pub fn spawn() -> Self {
        let (cmd_tx, cmd_rx) = channel::<BleCmd>();
        let (evt_tx, evt_rx) = channel::<BleEvent>();
        thread::Builder::new()
            .name("ble-worker".into())
            .spawn(move || worker_main(cmd_rx, evt_tx))
            .expect("spawn ble worker");
        Self { cmd_tx, evt_rx }
    }

    pub fn send(&self, cmd: BleCmd) {
        let _ = self.cmd_tx.send(cmd);
    }

    /// Drain pending events without blocking. Caller iterates and
    /// updates UI state per event.
    pub fn try_recv_all(&self) -> Vec<BleEvent> {
        let mut out = Vec::new();
        loop {
            match self.evt_rx.try_recv() {
                Ok(e) => out.push(e),
                Err(TryRecvError::Empty) | Err(TryRecvError::Disconnected) => break,
            }
        }
        out
    }
}

// ---------------------------------------------------------------------------
//  Worker thread + runtime
// ---------------------------------------------------------------------------

fn worker_main(cmd_rx: Receiver<BleCmd>, evt_tx: Sender<BleEvent>) {
    let rt = match tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
    {
        Ok(rt) => rt,
        Err(e) => {
            let _ = evt_tx.send(BleEvent::Error(format!("tokio runtime: {e}")));
            return;
        }
    };
    rt.block_on(worker_loop(cmd_rx, evt_tx));
}

/// The std mpsc receiver is sync-blocking, so we hop into a blocking
/// task to drain it and forward into a tokio channel that the async
/// loop can `select!` against alongside the BLE notification stream.
async fn worker_loop(cmd_rx: Receiver<BleCmd>, evt_tx: Sender<BleEvent>) {
    let (atx, mut arx) = tokio_mpsc::unbounded_channel::<BleCmd>();
    let _bridge = thread::spawn(move || {
        while let Ok(cmd) = cmd_rx.recv() {
            if atx.send(cmd).is_err() { break; }
        }
    });

    let mut state = WorkerState::new(evt_tx);

    loop {
        // The shape of `select!` depends on whether we have a live
        // notification stream — there's no "pending forever" stream
        // primitive in btleplug, so we branch on connection state to
        // keep the borrow checker (and the select macro) happy.
        if state.peripheral.is_some() {
            // Watchdog tick is short — 200 ms — so the loop wakes
            // frequently enough to detect op timeouts in human time.
            let watchdog = tokio::time::sleep(Duration::from_millis(200));
            tokio::pin!(watchdog);

            let Some(stream) = state.notif_stream.as_mut() else {
                state.emit_err("internal: stream missing while connected");
                state.disconnect_inner().await;
                continue;
            };

            tokio::select! {
                biased;

                cmd = arx.recv() => {
                    match cmd {
                        Some(c) => state.handle_command(c).await,
                        None    => break,
                    }
                }
                notif = stream.next() => {
                    match notif {
                        Some(n) => state.handle_notification(n).await,
                        None    => {
                            // The peripheral dropped the stream — treat
                            // as a remote-side disconnect.
                            state.emit_err("notification stream closed");
                            state.disconnect_inner().await;
                        }
                    }
                }
                _ = &mut watchdog => {
                    state.tick_watchdog();
                }
            }
        } else {
            // Not connected: only commands matter. arx.recv() returns
            // None when every BleBackend handle is dropped → process
            // teardown, exit cleanly.
            let Some(cmd) = arx.recv().await else { break };
            state.handle_command(cmd).await;
        }
    }
}

// ---------------------------------------------------------------------------
//  Worker state
// ---------------------------------------------------------------------------

enum CurrentOp {
    Idle,
    Listing { line: Vec<u8>, last_progress: Instant, rows_seen: u32 },
    Reading {
        name: String,
        expected: u64,
        content: Vec<u8>,
        last_emit: u64,
        last_progress: Instant,
        /// True until the first FileData notify for this READ has been
        /// inspected — used to disambiguate "single-byte status reply"
        /// from "first byte of a 1-byte file".
        first_packet: bool,
    },
    /// DELETE in flight — waiting for the single-byte status response.
    Deleting { name: String, last_progress: Instant },
}

type NotifStream = Pin<Box<dyn Stream<Item = ValueNotification> + Send>>;

/// 3-chunk reassembly state for the SensorStream MTU-fallback path
/// (DESIGN.md §3 "MTU negotiation"). When the negotiated MTU is too
/// small for a 46-byte single notify, the firmware splits the snapshot
/// across three sequential notifies with first-byte sequence indices
/// 0x00 / 0x01 / 0x02. Independent from `CurrentOp` because the stream
/// runs concurrently with FileSync ops; a misordered or partial frame
/// is silently dropped on the next 0x00 start byte.
#[derive(Default)]
struct StreamAsm {
    buf: Vec<u8>,
    /// Index of the next expected chunk (0 → wait for start, 1 / 2 →
    /// middle / end).
    next: u8,
}

struct WorkerState {
    evt_tx:       Sender<BleEvent>,
    adapter:      Option<Adapter>,
    peripheral:   Option<Peripheral>,
    notif_stream: Option<NotifStream>,
    op:           CurrentOp,
    /// SensorStream chunked-mode reassembly buffer.
    stream_asm:   StreamAsm,
    /// True iff the connected peripheral exposed the SensorStream
    /// characteristic — kept so we can emit a one-shot status line
    /// telling the user whether the box's firmware supports live data.
    stream_capable: bool,
}

impl WorkerState {
    fn new(evt_tx: Sender<BleEvent>) -> Self {
        Self {
            evt_tx,
            adapter: None,
            peripheral: None,
            notif_stream: None,
            op: CurrentOp::Idle,
            stream_asm: StreamAsm::default(),
            stream_capable: false,
        }
    }

    fn emit(&self, e: BleEvent) {
        let _ = self.evt_tx.send(e);
    }
    fn emit_err<S: Into<String>>(&self, msg: S) {
        self.emit(BleEvent::Error(msg.into()));
    }

    async fn ensure_adapter(&mut self) -> Option<Adapter> {
        if let Some(a) = self.adapter.clone() { return Some(a); }
        let manager = match Manager::new().await {
            Ok(m) => m,
            Err(e) => { self.emit_err(format!("manager: {e}")); return None; }
        };
        let adapters = match manager.adapters().await {
            Ok(a) => a,
            Err(e) => { self.emit_err(format!("adapters: {e}")); return None; }
        };
        let Some(adapter) = adapters.into_iter().next() else {
            self.emit_err("no BLE adapter found");
            return None;
        };
        self.adapter = Some(adapter.clone());
        Some(adapter)
    }

    // ---------------- Command dispatch -------------------------------------

    async fn handle_command(&mut self, cmd: BleCmd) {
        match cmd {
            BleCmd::Scan        => self.scan().await,
            BleCmd::Connect(id) => self.connect(id).await,
            BleCmd::Disconnect  => {
                self.disconnect_inner().await;
                self.emit(BleEvent::Disconnected);
            }
            BleCmd::List        => self.list().await,
            BleCmd::Read { name, size } => self.read(name, size).await,
            BleCmd::StopLog     => self.stop_log().await,
            BleCmd::StartLog { duration_seconds } => self.start_log(duration_seconds).await,
            BleCmd::Delete { name } => self.delete(name).await,
        }
    }

    async fn scan(&mut self) {
        let Some(adapter) = self.ensure_adapter().await else { return; };
        self.emit(BleEvent::Status("scanning…".into()));
        if let Err(e) = adapter.start_scan(ScanFilter::default()).await {
            self.emit_err(format!("start_scan: {e}"));
            return;
        }
        // 5-second window — long enough to see most peripherals on the
        // first advertising rotation; short enough that the user can
        // re-scan without feeling stuck.
        tokio::time::sleep(Duration::from_secs(5)).await;
        let _ = adapter.stop_scan().await;

        let peripherals = adapter.peripherals().await.unwrap_or_default();
        let mut matched = 0usize;
        let mut all_count = 0usize;
        for p in peripherals {
            all_count += 1;
            let props = p.properties().await.ok().flatten();
            let name  = props.as_ref().and_then(|pp| pp.local_name.clone());
            let addr  = props.as_ref().map(|pp| pp.address.to_string()).unwrap_or_default();
            // Debug: log every peripheral btleplug returned so we can see why
            // a known box may be missing (cached as old name, no local_name
            // in adv, etc.). Diagnostic — can be removed once stable.
            self.emit(BleEvent::Status(format!(
                "  seen: addr={} name={:?}", addr, name
            )));
            let name_ok = name.as_deref()
                .map(|n| BOX_NAMES.iter().any(|w| n.contains(w)))
                .unwrap_or(false);
            if name_ok {
                matched += 1;
                self.emit(BleEvent::Discovered {
                    id: p.id().to_string(),
                    name: name.unwrap_or_else(|| BOX_NAMES[0].into()),
                    rssi: props.and_then(|pp| pp.rssi),
                });
            }
        }
        self.emit(BleEvent::Status(format!(
            "scan saw {} peripheral(s), {} matched {:?}", all_count, matched, BOX_NAMES
        )));
        self.emit(BleEvent::ScanStopped);
    }

    async fn connect(&mut self, id: String) {
        if self.peripheral.is_some() {
            self.emit_err("already connected — disconnect first");
            return;
        }
        let Some(adapter) = self.ensure_adapter().await else { return; };
        let peripherals = match adapter.peripherals().await {
            Ok(p) => p,
            Err(e) => { self.emit_err(format!("peripherals: {e}")); return; }
        };
        let Some(p) = peripherals.into_iter().find(|x| x.id().to_string() == id) else {
            self.emit_err("peripheral gone — rescan");
            return;
        };
        self.emit(BleEvent::Status("connecting…".into()));
        if let Err(e) = p.connect().await {
            self.emit_err(format!("connect: {e}"));
            return;
        }
        if let Err(e) = p.discover_services().await {
            self.emit_err(format!("discover_services: {e}"));
            let _ = p.disconnect().await;
            return;
        }
        // Verify FileSync characteristics are present — bail loud if the
        // box is running an old firmware that doesn't expose them.
        let chars = p.characteristics();
        let data_char = match chars.iter().find(|c| c.uuid == FILEDATA_UUID).cloned() {
            Some(c) => c,
            None => {
                self.emit_err("Box firmware doesn't expose FileSync chars — flash a newer build");
                let _ = p.disconnect().await;
                return;
            }
        };
        if !chars.iter().any(|c| c.uuid == FILECMD_UUID) {
            self.emit_err("Box firmware doesn't expose FileSync chars — flash a newer build");
            let _ = p.disconnect().await;
            return;
        }
        if let Err(e) = p.subscribe(&data_char).await {
            self.emit_err(format!("subscribe FileData: {e}"));
            let _ = p.disconnect().await;
            return;
        }
        // SensorStream is optional — only PumpLogger (Peter's PR #18)
        // firmware exposes it. Subscribe if present, log if not, but
        // never fail the connect: legacy SDDataLogFileX firmware (the
        // PumpTsueri name) is FileSync-only and must keep working.
        self.stream_capable = false;
        if let Some(stream_char) = chars.iter().find(|c| c.uuid == STREAM_UUID).cloned() {
            match p.subscribe(&stream_char).await {
                Ok(()) => {
                    self.stream_capable = true;
                    self.emit(BleEvent::Status("SensorStream subscribed (live data at 0.5 Hz)".into()));
                }
                Err(e) => {
                    /* Soft-fail: log it and carry on without live data.
                       The user will still get FileSync. */
                    self.emit(BleEvent::Status(format!(
                        "SensorStream subscribe failed ({e}) — live tab will be empty"
                    )));
                }
            }
        } else {
            self.emit(BleEvent::Status(
                "SensorStream characteristic not advertised — legacy firmware, live tab will be empty".into()
            ));
        }
        // Open the persistent notification stream BEFORE storing the
        // peripheral so a failure here drops everything cleanly.
        let stream: NotifStream = match p.notifications().await {
            Ok(s) => Box::pin(s),
            Err(e) => {
                self.emit_err(format!("notifications: {e}"));
                let _ = p.disconnect().await;
                return;
            }
        };
        self.peripheral   = Some(p);
        self.notif_stream = Some(stream);
        self.op           = CurrentOp::Idle;
        self.emit(BleEvent::Connected);
    }

    /// Tear down peripheral + stream + op state without emitting the
    /// public `Disconnected` event. Used both by user-initiated
    /// disconnect and by failure paths inside the worker.
    async fn disconnect_inner(&mut self) {
        // If a download was in flight when the user disconnected, surface
        // it as an error so the UI doesn't keep saying "reading…" forever.
        if let CurrentOp::Reading { name, content, expected, .. } = &self.op {
            self.emit_err(format!(
                "READ {name} aborted by disconnect at {}/{} B",
                content.len(), expected
            ));
        } else if let CurrentOp::Listing { .. } = &self.op {
            self.emit_err("LIST aborted by disconnect");
        } else if let CurrentOp::Deleting { name, .. } = &self.op {
            self.emit_err(format!("DELETE {name} aborted by disconnect"));
        }
        self.op             = CurrentOp::Idle;
        self.notif_stream   = None;
        self.stream_asm     = StreamAsm::default();
        self.stream_capable = false;
        if let Some(p) = self.peripheral.take() {
            let _ = p.disconnect().await;
        }
    }

    async fn write_cmd(&self, payload: &[u8]) -> Result<(), String> {
        let Some(p) = self.peripheral.as_ref() else {
            return Err("not connected".into());
        };
        let chars = p.characteristics();
        let cmd_char = chars.iter().find(|c| c.uuid == FILECMD_UUID)
            .cloned()
            .ok_or_else(|| "FileCmd not discovered".to_string())?;
        // Write-without-response — the box doesn't ack opcodes; replies
        // come back over FileData.
        p.write(&cmd_char, payload, WriteType::WithoutResponse)
            .await
            .map_err(|e| format!("write FileCmd: {e}"))
    }

    async fn list(&mut self) {
        if !matches!(self.op, CurrentOp::Idle) {
            self.emit_err("another op is in flight — wait or Disconnect");
            return;
        }
        if self.peripheral.is_none() {
            self.emit_err("not connected");
            return;
        }
        if let Err(e) = self.write_cmd(&[0x01]).await {
            self.emit_err(e);
            return;
        }
        self.op = CurrentOp::Listing {
            line: Vec::with_capacity(64),
            last_progress: Instant::now(),
            rows_seen: 0,
        };
        self.emit(BleEvent::Status("LIST sent".into()));
    }

    async fn read(&mut self, name: String, size: u64) {
        if !matches!(self.op, CurrentOp::Idle) {
            self.emit_err("another op is in flight — wait or Disconnect");
            return;
        }
        if self.peripheral.is_none() {
            self.emit_err("not connected");
            return;
        }
        // Build opcode payload: 0x02 + filename bytes (no NUL).
        let mut payload = Vec::with_capacity(1 + name.len());
        payload.push(0x02);
        payload.extend_from_slice(name.as_bytes());
        if let Err(e) = self.write_cmd(&payload).await {
            self.emit_err(e);
            return;
        }
        self.op = CurrentOp::Reading {
            name: name.clone(),
            expected: size,
            content: Vec::with_capacity(size as usize),
            last_emit: 0,
            last_progress: Instant::now(),
            first_packet: true,
        };
        self.emit(BleEvent::ReadStarted { name, size });
    }

    async fn delete(&mut self, name: String) {
        if !matches!(self.op, CurrentOp::Idle) {
            self.emit_err("another op is in flight — wait or Disconnect");
            return;
        }
        if self.peripheral.is_none() {
            self.emit_err("not connected");
            return;
        }
        // Opcode payload: 0x03 + filename bytes (no NUL).
        let mut payload = Vec::with_capacity(1 + name.len());
        payload.push(0x03);
        payload.extend_from_slice(name.as_bytes());
        if let Err(e) = self.write_cmd(&payload).await {
            self.emit_err(e);
            return;
        }
        self.op = CurrentOp::Deleting { name: name.clone(), last_progress: Instant::now() };
        self.emit(BleEvent::Status(format!("DELETE {name} sent")));
    }

    async fn stop_log(&mut self) {
        if let Err(e) = self.write_cmd(&[0x04]).await {
            self.emit_err(e);
            return;
        }
        self.emit(BleEvent::Status("STOP_LOG sent".into()));
    }

    /// Send START_LOG (0x05) + 4-byte LE duration. Box validates duration
    /// in [1, 86400] s, writes BKP1R/BKP2R, and reboots into LOG mode.
    /// BLE connection drops shortly after — host should expect Disconnected.
    async fn start_log(&mut self, duration_seconds: u32) {
        let payload = [
            0x05,
            duration_seconds         as u8,
            (duration_seconds >> 8)  as u8,
            (duration_seconds >> 16) as u8,
            (duration_seconds >> 24) as u8,
        ];
        if let Err(e) = self.write_cmd(&payload).await {
            self.emit_err(e);
            return;
        }
        /* CoreBluetooth's WriteWithoutResponse has no delegate
           callback, so p.write(...).await returns as soon as the bytes
           are queued in the kernel — not when they've actually been
           transmitted. If the GUI queues a Disconnect right behind
           START_LOG (which it does, to free the Mac-side connection
           state for the reconnect after the LOG session ends), the
           Disconnect can tear the link down BEFORE the OP_START_LOG
           bytes ever hit the air, and the box never reboots into LOG
           mode. 500 ms is comfortably above CoreBluetooth's typical
           write-buffer flush time on FS-class peripherals — plenty for
           a 5-byte opcode payload to make it across before the
           subsequent Disconnect command runs. */
        tokio::time::sleep(Duration::from_millis(500)).await;
        self.emit(BleEvent::Status(format!(
            "START_LOG sent ({} s) — box rebooting to LOG mode",
            duration_seconds
        )));
    }

    // ---------------- Notification dispatch --------------------------------

    async fn handle_notification(&mut self, n: ValueNotification) {
        // SensorStream notifies are concurrent with FileSync, so handle
        // them on their own path before touching `op`.
        if n.uuid == STREAM_UUID {
            self.handle_stream_notification(&n.value);
            return;
        }
        if n.uuid != FILEDATA_UUID { return; }

        // Take ownership of the op so we can mutate it freely without
        // running into the borrow checker on `self.emit`.
        let mut op = std::mem::replace(&mut self.op, CurrentOp::Idle);

        match &mut op {
            CurrentOp::Idle => {
                // Stray notify between ops — harmless, ignore.
            }
            CurrentOp::Listing { line, last_progress, rows_seen } => {
                *last_progress = Instant::now();
                for b in n.value.iter() {
                    if *b == b'\n' {
                        if line.is_empty() {
                            self.emit(BleEvent::ListDone);
                            // Op completes; drop back to Idle (don't
                            // restore).
                            self.op = CurrentOp::Idle;
                            return;
                        }
                        if let Some((name, size)) = parse_list_row(line) {
                            self.emit(BleEvent::ListEntry { name, size });
                            *rows_seen += 1;
                        }
                        line.clear();
                    } else {
                        line.push(*b);
                    }
                }
            }
            CurrentOp::Reading { name, expected, content, last_emit, last_progress, first_packet } => {
                *last_progress = Instant::now();

                // Status-byte detection: only on the FIRST notify, only
                // when the packet is exactly one byte AND that byte is a
                // recognised error code. This avoids the false-positive
                // case of a 1-byte file whose lone byte happens to be a
                // status code — files we ship (CSV, log) start with
                // ASCII text well below 0x80 so the membership check is
                // unambiguous in practice.
                if *first_packet && n.value.len() == 1 && is_status_byte(n.value[0]) {
                    let s = n.value[0];
                    let msg = match s {
                        0xB0 => "BUSY (logging in progress, send STOP_LOG first)",
                        0xE1 => "NOT_FOUND",
                        0xE2 => "IO_ERROR",
                        0xE3 => "BAD_REQUEST",
                        _    => "unknown error",
                    };
                    self.emit_err(format!("READ {name}: {msg} (0x{s:02X})"));
                    self.op = CurrentOp::Idle;
                    return;
                }
                *first_packet = false;

                content.extend_from_slice(&n.value);

                // Throttle progress events — every ~4 KB or at EOF.
                // 4 KB is a compromise between channel-spam and a smooth
                // progress bar: at BLE FileSync's real-world ~1-3 KB/s the
                // bar updates every 1-4 s. 16 KB was too coarse (5-16 s).
                let done = content.len() as u64;
                if done - *last_emit >= 4 * 1024 || done >= *expected {
                    *last_emit = done;
                    self.emit(BleEvent::ReadProgress { name: name.clone(), bytes_done: done });
                }

                if done >= *expected {
                    if content.len() as u64 > *expected {
                        content.truncate(*expected as usize);
                    }
                    let final_content = std::mem::take(content);
                    let final_name    = std::mem::take(name);
                    self.emit(BleEvent::ReadDone { name: final_name, content: final_content });
                    self.op = CurrentOp::Idle;
                    return;
                }
            }
            CurrentOp::Deleting { name, .. } => {
                /* Firmware replies with exactly one byte: 0x00 OK or one
                   of the error codes. */
                if n.value.is_empty() {
                    return;  // shouldn't happen, but be tolerant
                }
                let s = n.value[0];
                if s == 0x00 {
                    let final_name = std::mem::take(name);
                    self.emit(BleEvent::DeleteDone { name: final_name });
                } else {
                    let msg = match s {
                        0xB0 => "BUSY (logging in progress, send STOP_LOG first)",
                        0xE1 => "NOT_FOUND",
                        0xE2 => "IO_ERROR",
                        0xE3 => "BAD_REQUEST",
                        _    => "unknown error",
                    };
                    self.emit_err(format!("DELETE {name}: {msg} (0x{s:02X})"));
                }
                self.op = CurrentOp::Idle;
                return;
            }
        }

        // If we didn't already replace `self.op` above (i.e. op didn't
        // complete), put it back.
        if matches!(self.op, CurrentOp::Idle) {
            self.op = op;
        }
    }

    // ---------------- SensorStream dispatch --------------------------------

    /// Handle one notify on the SensorStream characteristic. Resolves
    /// the two transport modes described in DESIGN.md §3:
    ///
    /// - **Single-notify (MTU upgrade accepted)**: payload is exactly
    ///   46 bytes; decode + emit.
    /// - **Chunked fallback (default-MTU host)**: three sequential
    ///   notifies prefixed with sequence byte 0x00 / 0x01 / 0x02 over
    ///   payloads of ~20 bytes each; reassemble into 46 bytes, decode,
    ///   emit. Out-of-sequence chunks silently reset the reassembly to
    ///   wait for the next 0x00 start.
    ///
    /// Malformed packets are dropped silently — the stream auto-resyncs
    /// on the next 0x00, and at 0.5 Hz a lost frame is a 2 s gap that
    /// the user might not even notice.
    fn handle_stream_notification(&mut self, bytes: &[u8]) {
        if bytes.len() == 46 {
            // Single-notify path. Reset any in-flight chunked frame so
            // a mid-frame MTU upgrade doesn't leave the asm in a bad
            // state for the next chunked frame (defensive — should be
            // either-or in practice).
            self.stream_asm = StreamAsm::default();
            if let Some(s) = LiveSample::parse(bytes) {
                self.emit(BleEvent::Sample(s));
            }
            return;
        }

        // Chunked-mode path: 20-ish byte notifies with seq byte first.
        if bytes.is_empty() { return; }
        let seq = bytes[0];
        let body = &bytes[1..];

        match seq {
            0x00 => {
                // Start of a new frame — reset and seed with body bytes.
                self.stream_asm.buf.clear();
                self.stream_asm.buf.extend_from_slice(body);
                self.stream_asm.next = 1;
            }
            0x01 => {
                if self.stream_asm.next != 1 {
                    // Out-of-order; drop and wait for the next 0x00.
                    self.stream_asm = StreamAsm::default();
                    return;
                }
                self.stream_asm.buf.extend_from_slice(body);
                self.stream_asm.next = 2;
            }
            0x02 => {
                if self.stream_asm.next != 2 {
                    self.stream_asm = StreamAsm::default();
                    return;
                }
                self.stream_asm.buf.extend_from_slice(body);
                if self.stream_asm.buf.len() == 46 {
                    if let Some(s) = LiveSample::parse(&self.stream_asm.buf) {
                        self.emit(BleEvent::Sample(s));
                    }
                }
                self.stream_asm = StreamAsm::default();
            }
            _ => {
                // Unknown sequence byte — corrupt frame, reset.
                self.stream_asm = StreamAsm::default();
            }
        }
    }

    // ---------------- Watchdog ---------------------------------------------

    fn tick_watchdog(&mut self) {
        let now = Instant::now();

        /* LIST inactivity-done fallback: if at least one row arrived and no
           new bytes have come in for LIST_INACTIVITY_DONE, assume the firmware
           finished and we just missed the terminator notify. Treat as
           success — emit ListDone and return to Idle so the next op (typically
           Download) doesn't trip the "another op is in flight" guard. */
        if let CurrentOp::Listing { last_progress, rows_seen, .. } = &self.op {
            if *rows_seen > 0 && now.duration_since(*last_progress) > LIST_INACTIVITY_DONE {
                self.op = CurrentOp::Idle;
                self.emit(BleEvent::ListDone);
                return;
            }
        }

        let stale = match &self.op {
            CurrentOp::Listing  { last_progress, .. } => now.duration_since(*last_progress) > OP_IDLE_TIMEOUT,
            CurrentOp::Reading  { last_progress, .. } => now.duration_since(*last_progress) > OP_IDLE_TIMEOUT,
            CurrentOp::Deleting { last_progress, .. } => now.duration_since(*last_progress) > OP_IDLE_TIMEOUT,
            CurrentOp::Idle => false,
        };
        if !stale { return; }

        match std::mem::replace(&mut self.op, CurrentOp::Idle) {
            CurrentOp::Listing { .. } => {
                self.emit_err("LIST timed out — no notifies for 20 s");
            }
            CurrentOp::Reading { name, content, expected, .. } => {
                self.emit_err(format!(
                    "READ {name} timed out at {}/{} B — no notifies for 20 s",
                    content.len(), expected
                ));
            }
            CurrentOp::Deleting { name, .. } => {
                self.emit_err(format!("DELETE {name} timed out — no notify for 20 s"));
            }
            CurrentOp::Idle => {}
        }
    }
}

// ---------------------------------------------------------------------------
//  Parsing helpers
// ---------------------------------------------------------------------------

fn parse_list_row(line: &[u8]) -> Option<(String, u64)> {
    let s = std::str::from_utf8(line).ok()?;
    let comma = s.rfind(',')?;
    let name = s[..comma].to_string();
    let size = s[comma + 1..].parse::<u64>().ok()?;
    Some((name, size))
}

fn is_status_byte(b: u8) -> bool {
    matches!(b, 0xB0 | 0xE1 | 0xE2 | 0xE3)
}
