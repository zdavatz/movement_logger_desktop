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
//!   0x06 SET_MODE <u8>        — 0=auto / 1=manual; single-byte status
//!   0x07 GET_MODE             — replies one byte 0=auto / 1=manual
//!   0x08 SET_TIME <epoch64_LE> — host wall-clock ms; box stamps a `# SYNC`
//!                               anchor into the open Sens/Gps CSVs (no RTC)
//!   0x09 FW_BEGIN  <len32_LE><sha256:32> — open a firmware-update session
//!                               (erases the inactive bank, ~1 s). Reply: 1
//!                               byte 0x00 ready / 0xB0 busy / 0xE6 too-big /
//!                               0xE5 erase-fail / 0xE3 bad-request.
//!   0x0A FW_DATA   <off32_LE><bytes…> — one image chunk. Reply: 4-byte LE
//!                               next-expected offset (ACK) or 1-byte error
//!                               (0xE7 bad-seq / 0xE5 flash-fail).
//!   0x0B FW_COMMIT — verify SHA-256, swap banks, RESET. Reply: 1 byte 0xA0
//!                               OK (link drops ~200 ms later) / 0xE4 hash-
//!                               mismatch / 0xE7 short-image / 0xE5 flash-fail.
//!   0x0C FW_ABORT  — cancel the session. Reply: 1 byte 0x00.
//!
//!   Status bytes: 0x00 OK, 0xB0 BUSY, 0xE1 NOT_FOUND, 0xE2 IO_ERROR, 0xE3 BAD_REQ.

use btleplug::api::{
    Central, Manager as _, Peripheral as _, ScanFilter, ValueNotification, WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use futures::stream::{Stream, StreamExt};
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender, TryRecvError};
use std::sync::Arc;
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

/// Auto-reconnect after an unexpected mid-transfer drop. Two regimes,
/// chosen at runtime by `BleShared::persist_reconnect` (mirrors the
/// GUI's "Keep synced" checkbox):
///
/// - **Auto Mode (Keep synced on):** *unbounded* — keep scanning until
///   the box reappears and reconnect the instant it does.
/// - **Manual mode (Keep synced off):** bounded — `RECONNECT_ATTEMPTS`
///   scan windows, then give up → `Disconnected` → manual reconnect
///   banner (still lossless: the live mirror holds the resume offset).
///
/// **Continuous-scan model (v0.0.35).** The earlier design slept an
/// *exponential backoff* (2 s → ×2, capped 60 s) and then fired a single
/// 3 s scan per round, which routinely missed the advertising box.
///
/// **Retrieve-first model (v0.0.36).** The real root cause of the "box is
/// BT-visible but the Mac never reconnects, have to power-cycle" symptom
/// (which iOS/Android don't have) is that stock btleplug 0.11.8 *purges*
/// the peripheral from its tables on disconnect (`internal.rs`
/// `on_peripheral_disconnect` → `peripherals.remove`) and can only refill
/// them by a fresh **scan** — but macOS CoreBluetooth routinely STOPS
/// returning a recently-connected peripheral in scan results, so the scan
/// finds nothing and reconnect never fires. iOS uses
/// `retrievePeripherals(withIdentifiers:)` + a pending `connect()` (no
/// scan); Android connects by MAC. Stock btleplug exposes neither, so we
/// vendored `btleplug-patched` to add exactly that (`Central::add_peripheral`
/// → `RetrievePeripheral`). `auto_reconnect` now tries **retrieve-by-id
/// first** (macOS) and only falls back to the continuous scan below when
/// retrieve fails (or the box is genuinely silent — see `RETRIEVE_MAX_FAILS`).
///
/// The scan fallback keeps a scan running and connects the moment
/// `peripherals()` shows the id (see `wait_for_peripheral`). No exponential
/// backoff — a short `RECONNECT_INTERVAL` gap between windows keeps it
/// responsive without hammering `connect()`.
///
/// Both regimes poll `BleShared::abort_reconnect` between every window
/// and during every sleep, so a user Disconnect / app-quit / un-ticking
/// "Keep synced" breaks the loop promptly even though the worker can't
/// service its command channel while inside this function.
const RECONNECT_ATTEMPTS: u32 = 10;
const RECONNECT_INTERVAL: Duration = Duration::from_secs(2);
/// How long to keep an active scan running each attempt, polling for the
/// box to re-advertise, before pausing `RECONNECT_INTERVAL` and looping.
const RECONNECT_SCAN_WINDOW: Duration = Duration::from_secs(20);

/// macOS retrieve-by-id (the `btleplug-patched` fast path): after this many
/// consecutive retrieve→connect failures within one `auto_reconnect`
/// invocation, stop attempting the retrieve path and fall back to scan-only
/// for the rest of this invocation. Prevents a genuinely-silent box from
/// paying an extra ~20 s dead pending-connect on top of the scan window
/// every round. A retrieve *success* resets nothing — we only stop after
/// this many failures in a row.
#[cfg(target_os = "macos")]
const RETRIEVE_MAX_FAILS: u32 = 3;

/// Upper bound on every individual GATT step inside `connect_core`
/// (connect / discover / subscribe / notifications). Same rationale as
/// `drop_link`'s bounded `disconnect()`: on macOS CoreBluetooth none of
/// these calls has an intrinsic timeout — after an unclean drop a stale
/// connection state makes `connect()` or `discover_services()` block
/// effectively forever. Unbounded, that froze the worker *inside*
/// `auto_reconnect` on the very first attempt, so the 10-attempt loop
/// never advanced and `Disconnected` was never emitted — the box kept
/// advertising ("visible in BT") while the app sat dead. 12 s is plenty
/// for a healthy connect+discover; past that the attempt fails and the
/// reconnect loop moves on (or falls back to manual). Never drop these
/// timeouts — it's the connect-side twin of the `drop_link` fix.
///
/// Raised 12 → 20 s to match the mobile clients' connect budgets (iOS
/// holds a pending connect 18 s, Android 60 s). The desktop's 12 s was
/// the shortest of the three and, on a flaky link where the box is slow
/// to re-advertise, it false-timed-out perfectly good pending
/// CoreBluetooth connects — the reconnect then looked like "dropped
/// again" even though the link was about to come up. 20 s still sits
/// above a healthy connect+discover and only costs extra patience on a
/// genuinely dead link. (iOS documents this exact footgun.)
const LINK_OP_TIMEOUT: Duration = Duration::from_secs(20);

/// Reconnect-scoped pending-connect hold (v0.0.37). CoreBluetooth's
/// `connect()` has **no** intrinsic timeout — the reply resolves only when
/// the box becomes connectable — so on the reconnect path we let ONE pending
/// connect *ride* the box's fragile post-big-drop self-heal window instead of
/// self-cancelling every `LINK_OP_TIMEOUT` and churning. Cancelling a pending
/// connect while the box isn't advertising is a Mac-side no-op the box never
/// sees, so the old 20 s-cancel-and-retry loop just went scan-blind after 3
/// rounds and never caught the box when it *did* come back. Manual mode holds
/// this long then voluntarily re-issues (Android holds 60 s); Auto mode holds
/// unbounded (`None`) — exits only on connect success, user abort, or
/// Keep-synced turned off. DISTINCT from `LINK_OP_TIMEOUT`, which still bounds
/// discover/subscribe/notifications and the snappy manual first-connect.
const RECONNECT_CONNECT_TIMEOUT: Duration = Duration::from_secs(60);

/// How `connect_core` issues the CoreBluetooth connect.
#[derive(Clone, Copy)]
enum ConnectMode {
    /// First user-initiated connect: snappy — bound at `LINK_OP_TIMEOUT`,
    /// cancel on our own timeout for fast UI feedback. Byte-identical to the
    /// pre-v0.0.37 behaviour.
    Manual,
    /// Auto-reconnect: hold ONE pending connect. `budget: None` = unbounded
    /// (Keep synced on); `Some(d)` = hold `d` then voluntarily re-issue.
    /// Cancel only on a real abort or a Keep-synced-off demotion — never on
    /// our own timeout.
    Reconnect { budget: Option<Duration> },
}

/// Why a held reconnect connect stopped waiting.
#[derive(PartialEq)]
enum ConnectWait { Aborted, Demoted, Budget }

// ---------------------------------------------------------------------------
//  Firmware-update (OTA) tuning
// ---------------------------------------------------------------------------

/// FileCmd opcodes for the firmware-update flow. Same characteristic /
/// reply path as LIST/READ/DELETE — the box keeps the single-op model
/// (a concurrent op is rejected with 0xB0 BUSY). See the module-doc
/// opcode list and `Core/Src/ble_filesync.c`.
const OP_FW_BEGIN:  u8 = 0x09;
const OP_FW_DATA:   u8 = 0x0A;
const OP_FW_COMMIT: u8 = 0x0B;
const OP_FW_ABORT:  u8 = 0x0C;

/// Bytes of image per FW_DATA chunk (the write is `1 + 4 + FW_CHUNK` =
/// 165 B, safely under any negotiated MTU — btleplug doesn't expose the
/// negotiated value, so a fixed conservative size is simplest. The
/// protocol is offset-addressed, so the chunk size doesn't affect
/// correctness, only throughput).
const FW_CHUNK: usize = 160;

/// FW_BEGIN erases the inactive flash bank (~1 s) before replying, so it
/// needs a longer ceiling than a normal op. FW_COMMIT verifies the hash,
/// swaps banks, and resets — its reply (0xA0) may never arrive because
/// the link drops first, which we treat as success.
const FW_BEGIN_TIMEOUT:  Duration = Duration::from_secs(8);
const FW_COMMIT_TIMEOUT: Duration = Duration::from_secs(8);
/// Per-chunk ACK wait. On timeout we resend the SAME offset (the box is
/// idempotent for offset < its write cursor) up to `FW_DATA_RETRIES`
/// times before failing the whole upload.
const FW_DATA_TIMEOUT: Duration = Duration::from_secs(5);
const FW_DATA_RETRIES: u32 = 5;

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
    /// Send a READ opcode for `name`; `size` is the full file length
    /// from the prior LIST so the worker knows when EOF is reached.
    /// `offset` is the byte position to resume from (0 = whole file) —
    /// the firmware seeks there before streaming, so an interrupted
    /// transfer continues instead of restarting (`ble.c` READ handler
    /// `<name>\0<offset:u32-LE>`).
    Read { name: String, size: u64, offset: u64 },
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
    /// SET_MODE opcode `0x06` + `<u8>` (0 = auto, 1 = manual). Sets the
    /// box's *log mode* (not the sync) and persists it on the SD card
    /// (`LOGMODE.CFG`): AUTO = the box opens a session on every power-on;
    /// MANUAL = the box stays idle until a START_LOG. No FileData reply.
    /// Legacy `PumpTsueri` firmware silently ignores this opcode.
    SetLogMode { manual: bool },
    /// GET_MODE opcode `0x07`. Box replies with a single byte over
    /// FileData (0 = auto, 1 = manual) → `BleEvent::LogMode`. Legacy
    /// `PumpTsueri` firmware never replies; the GettingMode op just times
    /// out and drops to Idle (mode stays "unknown", no error surfaced).
    GetLogMode,
    /// SET_TIME opcode `0x08` + `<epoch_ms:u64-LE>` (firmware v0.0.10+).
    /// The box has no RTC, so we push the host's current wall-clock millis
    /// and the firmware stamps a `# SYNC epoch_ms=.. tick_ms=..` anchor into
    /// the open Sens/Gps CSVs, pairing the host epoch with its free-running
    /// ms counter so replay can resolve absolute time without a GPS fix.
    /// Sent on every connect; *fire-and-forget* (no FileData reply tracked —
    /// legacy firmware silently ignores 0x08).
    SetTime { epoch_ms: u64 },
    /// Upload a firmware `.bin` to the box over BLE (dual-bank OTA).
    /// `bytes` is the EXACT file content; the worker computes the
    /// SHA-256, sends FW_BEGIN / a stream of FW_DATA chunks / FW_COMMIT,
    /// and emits `FlashStarted` / `FlashProgress` / `FlashDone` /
    /// `FlashError`. Drives the same single-op `CurrentOp` slot as READ,
    /// so it's rejected while another op is in flight (and rejects other
    /// ops while it runs). On success the box swaps banks + reboots —
    /// the link drops and the user reconnects into the new firmware.
    FlashFirmware { bytes: Arc<Vec<u8>> },
    /// GPS bridge on/off (`0x0D` `<u8>` 1=on/0=off) for the GPS Debug tab's
    /// BLE transport. While ON the box relays raw u-blox UBX frames over
    /// FileData notifies (NMEA stays on the SD logger, undisturbed); the
    /// worker routes those notifies to the GPS data sink instead of the
    /// FileSync FSM. Fire-and-forget like `StopLog`/`SetTime` — no op slot,
    /// no FileData reply (legacy firmware silently ignores 0x0D).
    GpsBridge { on: bool },
    /// Forward raw bytes to the u-blox over the bridge (`0x0E` + bytes).
    /// The survey loop sends UBX poll frames this way; the box writes them
    /// straight to the GPS UART. No-op unless a bridge is active.
    GpsTx { bytes: Vec<u8> },
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
    /// `bytes_done` is absolute (resume base + bytes streamed this
    /// segment), so the progress bar is correct across a resume.
    ReadProgress { name: String, bytes_done: u64 },
    /// A READ segment finished at EOF. `content` is just this segment
    /// (from `base` to EOF); `base` is the offset it was requested at.
    /// The caller appends `content` at `base` into the `.part` file.
    ReadDone { name: String, content: Vec<u8>, base: u64 },
    /// A READ was cut off (link lost / timeout) partway. `content` is
    /// the bytes streamed this segment before the cut, `base` the
    /// offset it started at — the caller appends them to `.part` so
    /// the resume continues from the real break point, not the last
    /// completed segment.
    ReadAborted { name: String, content: Vec<u8>, base: u64 },
    /// DELETE finished successfully (status byte 0x00 received).
    DeleteDone { name: String },
    /// The box's current log mode, from a GET_MODE reply or a confirmed
    /// SET_MODE round-trip. `manual = false` → AUTO (logs on power-on),
    /// `true` → MANUAL (idle until START_LOG). Never emitted on legacy
    /// `PumpTsueri` firmware (no GET_MODE) — the UI stays at "unknown".
    LogMode { manual: bool },
    /// User-facing error — display in the log panel.
    Error(String),
    /// One decoded SensorStream snapshot (0.5 Hz). Only emitted while
    /// connected to a PumpLogger firmware that exposes the SensorStream
    /// characteristic; legacy PumpTsueri builds never produce this event.
    Sample(LiveSample),
    /// A firmware upload has started (FW_BEGIN accepted; the box erased
    /// its inactive bank). `total` is the image byte length.
    FlashStarted { total: u64 },
    /// Firmware-upload progress — `bytes_done` of `total` image bytes
    /// have been written + ACKed by the box.
    FlashProgress { bytes_done: u64, total: u64 },
    /// The firmware upload finished: FW_COMMIT succeeded (0xA0) or the
    /// link dropped right after COMMIT. The box is rebooting into the
    /// new firmware — the user should reconnect in a few seconds. A
    /// `Disconnected` typically follows shortly after.
    FlashDone,
    /// The firmware upload failed (mapped error string). The session was
    /// aborted (FW_ABORT best-effort); the box keeps its current
    /// firmware (the bank swap only happens on a verified COMMIT).
    FlashError { msg: String },
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
    /// Strongest satellite C/N0 in dB-Hz (from GSV); 0 = no data.
    pub cn0_max: u8,
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
            cn0_max:         bytes[45],
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

/// Lock-free state shared between the egui thread and the BLE worker,
/// read by `auto_reconnect` (which can't service the command channel
/// while it's looping). `persist_reconnect` = "Keep synced" is on
/// (→ unbounded reconnect); `abort_reconnect` = the user wants out
/// (Disconnect / quit / un-ticked Keep synced).
#[derive(Clone, Default)]
struct BleShared {
    persist_reconnect: Arc<AtomicBool>,
    abort_reconnect:   Arc<AtomicBool>,
    /// GPS-bridge mode (GPS Debug tab over BLE). While set, the box is
    /// relaying raw u-blox UBX frames over FileData notifies, so the worker
    /// routes those notifies to `gps_data_tx` instead of the FileSync FSM.
    /// Kept here (not a `CurrentOp`) because the survey is a long-lived
    /// unsolicited-notify stream, like SensorStream — not a single op.
    gps_bridge:        Arc<AtomicBool>,
    /// Where raw GPS-bridge bytes go while `gps_bridge` is set. The GUI
    /// installs a sender when a BLE survey starts (`set_gps_data_sink`);
    /// the worker clears it when the bridge is turned off.
    gps_data_tx:       Arc<std::sync::Mutex<Option<Sender<Vec<u8>>>>>,
}

pub struct BleBackend {
    cmd_tx: Sender<BleCmd>,
    evt_rx: Receiver<BleEvent>,
    shared: BleShared,
}

impl BleBackend {
    /// Spawns the worker thread and its tokio runtime. Cheap — no BLE
    /// activity until the first command lands.
    pub fn spawn() -> Self {
        let (cmd_tx, cmd_rx) = channel::<BleCmd>();
        let (evt_tx, evt_rx) = channel::<BleEvent>();
        let shared = BleShared::default();
        let worker_shared = shared.clone();
        thread::Builder::new()
            .name("ble-worker".into())
            .spawn(move || worker_main(cmd_rx, evt_tx, worker_shared))
            .expect("spawn ble worker");
        Self { cmd_tx, evt_rx, shared }
    }

    pub fn send(&self, cmd: BleCmd) {
        let _ = self.cmd_tx.send(cmd);
    }

    /// A clonable command sender, so the in-process BLE GPS survey thread can
    /// push `GpsTx` poll frames without borrowing the whole backend.
    pub fn cmd_sender(&self) -> Sender<BleCmd> {
        self.cmd_tx.clone()
    }

    /// Install (or clear, with `None`) the sink that receives raw GPS-bridge
    /// bytes while `BleCmd::GpsBridge { on: true }` is active. The GUI sets
    /// this when a BLE survey starts; the worker also clears it when the
    /// bridge is turned off.
    pub fn set_gps_data_sink(&self, tx: Option<Sender<Vec<u8>>>) {
        if let Ok(mut g) = self.shared.gps_data_tx.lock() { *g = tx; }
    }

    /// Mirror the GUI's "Keep synced" checkbox into the worker. Called
    /// every frame — a relaxed atomic store, effectively free. When on,
    /// `auto_reconnect` retries unbounded with exponential backoff.
    pub fn set_keep_synced(&self, on: bool) {
        self.shared.persist_reconnect.store(on, Ordering::Relaxed);
    }

    /// Ask any in-progress reconnect loop to stop. Must be sent *before*
    /// `BleCmd::Disconnect` because the worker can't read its command
    /// channel while inside `auto_reconnect` — this flag is the only
    /// signal it polls there.
    pub fn request_abort(&self) {
        self.shared.abort_reconnect.store(true, Ordering::Relaxed);
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

fn worker_main(cmd_rx: Receiver<BleCmd>, evt_tx: Sender<BleEvent>, shared: BleShared) {
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
    rt.block_on(worker_loop(cmd_rx, evt_tx, shared));
}

/// The std mpsc receiver is sync-blocking, so we hop into a blocking
/// task to drain it and forward into a tokio channel that the async
/// loop can `select!` against alongside the BLE notification stream.
async fn worker_loop(cmd_rx: Receiver<BleCmd>, evt_tx: Sender<BleEvent>, shared: BleShared) {
    let (atx, mut arx) = tokio_mpsc::unbounded_channel::<BleCmd>();
    let _bridge = thread::spawn(move || {
        while let Ok(cmd) = cmd_rx.recv() {
            if atx.send(cmd).is_err() { break; }
        }
    });

    let mut state = WorkerState::new(evt_tx, shared);

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

            let Some(notif_rx) = state.notif_rx.as_mut() else {
                state.emit_err("internal: notify channel missing while connected");
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
                notif = notif_rx.recv() => {
                    match notif {
                        Some(n) => state.handle_notification(n).await,
                        None    => {
                            // The peripheral dropped the stream — treat
                            // as a remote-side disconnect.
                            let was_transfer =
                                matches!(state.op, CurrentOp::Reading { .. });
                            state.emit_err("notification stream closed");
                            // disconnect_inner emits ReadAborted with the
                            // partial bytes (→ appended to .part) and
                            // tears the connection down.
                            state.disconnect_inner().await;
                            if was_transfer {
                                if let Some(id) = state.last_id.clone() {
                                    if state.auto_reconnect(id).await {
                                        // Reconnected: Connected emitted,
                                        // main re-lists + resumes from the
                                        // .part offset. Stay in the loop.
                                        continue;
                                    }
                                }
                                // Bounded retries exhausted (or no id):
                                // hand control back to the user. main
                                // shows the manual-reconnect banner; the
                                // .part keeps the resume lossless. We've
                                // stopped trying to hold the link, so let
                                // the machine sleep / App-Nap us again.
                                state.release_power();
                                state.emit(BleEvent::Disconnected);
                            }
                        }
                    }
                }
                _ = &mut watchdog => {
                    if state.tick_watchdog() {
                        // A READ stalled but CoreBluetooth still thinks
                        // it's connected (no stream close). Tear the
                        // half-dead link down ourselves and auto-
                        // reconnect so the .part resume can continue —
                        // this is the case Peter hit where the box
                        // never saw a formal disconnect. tick_watchdog
                        // already emitted ReadAborted (partial saved)
                        // + the timeout error and set op = Idle.
                        let id = state.last_id.clone();
                        state.disconnect_inner().await;
                        if let Some(id) = id {
                            if state.auto_reconnect(id).await {
                                continue;
                            }
                        }
                        state.release_power();
                        state.emit(BleEvent::Disconnected);
                    }
                }
            }

            // A log-mode change requested mid-LIST/READ was deferred so the
            // firmware's SET_MODE OK-status byte couldn't corrupt the
            // in-flight download (see `set_log_mode`). Apply it now that the
            // worker is idle; the UI already shows the chosen mode.
            if let Some(manual) = state.pending_set_mode {
                if matches!(state.op, CurrentOp::Idle) && state.peripheral.is_some() {
                    state.pending_set_mode = None;
                    state.write_set_mode(manual).await;
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
        /// Byte offset this READ was requested at (bytes already on
        /// disk in `.part`). Absolute progress = base + content.len().
        base: u64,
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
    /// GET_MODE in flight — waiting for the single-byte mode response
    /// (0 = auto, 1 = manual). Like Deleting, a stall just drops to Idle
    /// without reconnecting (legacy firmware never replies).
    GettingMode { last_progress: Instant },
    /// Firmware upload in flight. Unlike the other ops, the flash is
    /// driven *inline* by `flash_firmware` (it owns the notification
    /// stream for its whole duration and reads each reply with its own
    /// timeout/retry), so notifications never reach `handle_notification`
    /// while this is set. The variant exists only as the single-op guard
    /// (other commands are rejected with "another op is in flight") and a
    /// `last_progress` slot for symmetry — the watchdog never fires for
    /// it because `handle_command` doesn't return to the `select!` loop
    /// until the flash completes.
    Flashing { last_progress: Instant },
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
    /// Notifications arrive here via a dedicated pump task (`notif_pump`)
    /// that does nothing but `stream.next()` → this *unbounded* channel.
    /// The worker loop / flash reader consume from `notif_rx`. This
    /// decouples draining btleplug's **bounded** internal notify buffer
    /// from the worker's own blocking awaits (`write_cmd`, the SET_TIME /
    /// START_LOG settle sleeps, `drop_link`, `auto_reconnect`): while the
    /// worker is parked in any of those, the pump task keeps draining so
    /// btleplug can't silently drop notifies and stall a READ (→ 20 s
    /// watchdog → needless reconnect). Mirrors iOS's CoreBluetooth-callback
    /// → unbounded-AsyncStream hand-off, which structurally can't back up.
    notif_rx:     Option<tokio_mpsc::UnboundedReceiver<ValueNotification>>,
    notif_pump:   Option<tokio::task::JoinHandle<()>>,
    op:           CurrentOp,
    /// SensorStream chunked-mode reassembly buffer.
    stream_asm:   StreamAsm,
    /// True iff the connected peripheral exposed the SensorStream
    /// characteristic — kept so we can emit a one-shot status line
    /// telling the user whether the box's firmware supports live data.
    stream_capable: bool,
    /// Peripheral id of the last successful connect — the auto-reconnect
    /// target. Set by `connect` / `auto_reconnect`, never cleared (a
    /// stale id just makes a future reconnect fail one round and fall
    /// back to manual).
    last_id: Option<String>,
    /// "Keep synced" + abort flags, shared with the egui thread.
    shared: BleShared,
    /// A log-mode change (SET_MODE) the user requested while the worker was
    /// mid-LIST/READ. Deferred so the firmware's OK-status byte can't be
    /// appended into the in-flight download; applied by the worker loop once
    /// `op` returns to Idle.
    pending_set_mode: Option<bool>,
    /// OS "stay awake / no App Nap" assertion, held for the whole connected
    /// session **including** auto-reconnect windows (mirrors iOS's
    /// background-task assertion / Android's foreground service). `Some`
    /// while we have intent to be connected; taken on connect, released
    /// only on an explicit Disconnect / abort / final give-up — never in
    /// `disconnect_inner`, or the Mac could sleep during a reconnect gap.
    /// See `power.rs`.
    power: Option<crate::power::PowerGuard>,
}

impl WorkerState {
    fn new(evt_tx: Sender<BleEvent>, shared: BleShared) -> Self {
        Self {
            evt_tx,
            adapter: None,
            peripheral: None,
            notif_rx: None,
            notif_pump: None,
            op: CurrentOp::Idle,
            stream_asm: StreamAsm::default(),
            stream_capable: false,
            last_id: None,
            shared,
            pending_set_mode: None,
            power: None,
        }
    }

    /// Take the OS stay-awake / no-App-Nap assertion if we don't already
    /// hold it. Idempotent and cheap — safe to call on every connect /
    /// reconnect success.
    fn hold_power(&mut self) {
        if self.power.is_none() {
            self.power = Some(crate::power::PowerGuard::acquire());
        }
    }

    /// Drop the OS assertion (lets the Mac sleep / App-Nap us again). Called
    /// only when we truly stop trying to hold a link — Disconnect, abort, or
    /// a bounded reconnect giving up.
    fn release_power(&mut self) {
        self.power = None;
    }

    fn persist_reconnect(&self) -> bool {
        self.shared.persist_reconnect.load(Ordering::Relaxed)
    }
    fn abort_requested(&self) -> bool {
        self.shared.abort_reconnect.load(Ordering::Relaxed)
    }
    fn clear_abort(&self) {
        self.shared.abort_reconnect.store(false, Ordering::Relaxed);
    }

    /// Sleep `dur`, but wake early (returning `true`) the moment an
    /// abort is requested. Polls in ≤250 ms slices so a Disconnect /
    /// quit during a long backoff is honoured promptly.
    async fn sleep_or_abort(&self, dur: Duration) -> bool {
        let slice = Duration::from_millis(250);
        let mut left = dur;
        while !left.is_zero() {
            if self.abort_requested() { return true; }
            let step = left.min(slice);
            tokio::time::sleep(step).await;
            left -= step;
        }
        self.abort_requested()
    }

    /// Runs concurrently with a held `p.connect()` on the reconnect path
    /// (see `ConnectMode::Reconnect`). Returns the moment a user abort is
    /// requested, Keep-synced is un-ticked during an unbounded hold, or the
    /// budget elapses. `budget: None` waits forever (for abort/demote only).
    /// 250 ms slices keep Disconnect/quit responsive even though the worker
    /// can't service its command channel while inside `auto_reconnect`.
    async fn connect_wait(&self, budget: Option<Duration>) -> ConnectWait {
        let slice = Duration::from_millis(250);
        let mut left = budget;
        loop {
            if self.abort_requested() { return ConnectWait::Aborted; }
            // Keep-synced un-ticked mid-hold demotes an unbounded hold back to
            // the bounded budget — the checkbox only mirrors the atomic (it
            // does NOT call request_abort), so this poll is the only signal.
            // auto_reconnect re-reads persist_reconnect at its loop top.
            if budget.is_none() && !self.persist_reconnect() {
                return ConnectWait::Demoted;
            }
            match &mut left {
                Some(rem) if rem.is_zero() => return ConnectWait::Budget,
                Some(rem) => {
                    let step = (*rem).min(slice);
                    tokio::time::sleep(step).await;
                    *rem -= step;
                }
                None => tokio::time::sleep(slice).await,
            }
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
                self.release_power();
                self.emit(BleEvent::Disconnected);
            }
            BleCmd::List        => self.list().await,
            BleCmd::Read { name, size, offset } => self.read(name, size, offset).await,
            BleCmd::StopLog     => self.stop_log().await,
            BleCmd::StartLog { duration_seconds } => self.start_log(duration_seconds).await,
            BleCmd::Delete { name } => self.delete(name).await,
            BleCmd::SetLogMode { manual } => self.set_log_mode(manual).await,
            BleCmd::GetLogMode => self.get_log_mode().await,
            BleCmd::SetTime { epoch_ms } => self.set_time(epoch_ms).await,
            BleCmd::FlashFirmware { bytes } => self.flash_firmware(bytes).await,
            BleCmd::GpsBridge { on } => self.gps_bridge(on).await,
            BleCmd::GpsTx { bytes } => self.gps_tx(bytes).await,
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
        // re-scan without feeling stuck. Wake early on an abort (Disconnect
        // / quit) so the command channel isn't stalled behind the scan.
        self.sleep_or_abort(Duration::from_secs(5)).await;
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
        match self.connect_core(&id, ConnectMode::Manual).await {
            Ok(()) => {
                self.last_id = Some(id);
                // Hold the OS awake for the whole session (kept across any
                // later auto-reconnect windows; released on Disconnect).
                self.hold_power();
                self.emit(BleEvent::Connected);
            }
            Err(e) => self.emit_err(e),
        }
    }

    /// The connect work, factored out so the bounded auto-reconnect
    /// loop can reuse it. Sets `peripheral` / `notif_rx` (+ pump) / `op` on
    /// success; emits only informational Status lines (SensorStream
    /// presence). Returns Err(reason) instead of emitting an error, so
    /// the caller decides whether a failed attempt is fatal (manual
    /// Connect) or just one retry (auto-reconnect).
    async fn connect_core(&mut self, id: &str, mode: ConnectMode) -> Result<(), String> {
        let Some(adapter) = self.ensure_adapter().await else {
            return Err("no BLE adapter".into());
        };
        let peripherals = adapter
            .peripherals()
            .await
            .map_err(|e| format!("peripherals: {e}"))?;
        let p = peripherals
            .into_iter()
            .find(|x| x.id().to_string() == id)
            .ok_or_else(|| "peripheral gone — rescan".to_string())?;
        // The connect step. Manual = bounded + cancel-on-timeout (snappy UI).
        // Reconnect = hold ONE pending CoreBluetooth connect and let it ride
        // the box's post-drop self-heal; cancel (via drop_link →
        // cancelPeripheralConnection) ONLY on abort/demote/budget, never on
        // our own timeout. Everything after (discover/subscribe/notifications)
        // stays bounded at LINK_OP_TIMEOUT regardless of mode.
        match mode {
            ConnectMode::Manual => {
                match tokio::time::timeout(LINK_OP_TIMEOUT, p.connect()).await {
                    Ok(Ok(())) => {}
                    Ok(Err(e)) => {
                        drop_link(&p).await;
                        return Err(format!("connect: {e}"));
                    }
                    Err(_) => {
                        drop_link(&p).await;
                        return Err("connect: timed out".into());
                    }
                }
            }
            ConnectMode::Reconnect { budget } => {
                // Disjoint borrows: p.connect() borrows the local p,
                // connect_wait borrows &self. Resolves the instant the box
                // becomes connectable — no self-cancel churn.
                tokio::select! {
                    r = p.connect() => match r {
                        Ok(()) => {}
                        Err(e) => {
                            drop_link(&p).await;
                            return Err(format!("connect: {e}"));
                        }
                    },
                    w = self.connect_wait(budget) => {
                        drop_link(&p).await; // the ONLY cancel path in reconnect mode
                        return Err(match w {
                            ConnectWait::Aborted => "connect: aborted".into(),
                            ConnectWait::Demoted => "connect: keep-synced off".into(),
                            ConnectWait::Budget  => "connect: budget".into(),
                        });
                    }
                }
            }
        }
        match tokio::time::timeout(LINK_OP_TIMEOUT, p.discover_services()).await {
            Ok(Ok(())) => {}
            Ok(Err(e)) => {
                drop_link(&p).await;
                return Err(format!("discover_services: {e}"));
            }
            Err(_) => {
                drop_link(&p).await;
                return Err("discover_services: timed out".into());
            }
        }
        let chars = p.characteristics();
        let data_char = match chars.iter().find(|c| c.uuid == FILEDATA_UUID).cloned() {
            Some(c) => c,
            None => {
                drop_link(&p).await;
                return Err("Box firmware doesn't expose FileSync chars — flash a newer build".into());
            }
        };
        if !chars.iter().any(|c| c.uuid == FILECMD_UUID) {
            drop_link(&p).await;
            return Err("Box firmware doesn't expose FileSync chars — flash a newer build".into());
        }
        match tokio::time::timeout(LINK_OP_TIMEOUT, p.subscribe(&data_char)).await {
            Ok(Ok(())) => {}
            Ok(Err(e)) => {
                drop_link(&p).await;
                return Err(format!("subscribe FileData: {e}"));
            }
            Err(_) => {
                drop_link(&p).await;
                return Err("subscribe FileData: timed out".into());
            }
        }
        // SensorStream is optional — only PumpLogger (Peter's PR #18)
        // firmware exposes it. Subscribe if present, log if not, but
        // never fail the connect: legacy SDDataLogFileX firmware (the
        // PumpTsueri name) is FileSync-only and must keep working.
        self.stream_capable = false;
        if let Some(stream_char) = chars.iter().find(|c| c.uuid == STREAM_UUID).cloned() {
            // Bounded like the rest, but a timeout/error here is
            // non-fatal — just leave the live tab empty.
            match tokio::time::timeout(LINK_OP_TIMEOUT, p.subscribe(&stream_char)).await {
                Ok(Ok(())) => {
                    self.stream_capable = true;
                    self.emit(BleEvent::Status("SensorStream subscribed (live data at 0.5 Hz)".into()));
                }
                Ok(Err(e)) => {
                    self.emit(BleEvent::Status(format!(
                        "SensorStream subscribe failed ({e}) — live tab will be empty"
                    )));
                }
                Err(_) => {
                    self.emit(BleEvent::Status(
                        "SensorStream subscribe timed out — live tab will be empty".into()
                    ));
                }
            }
        } else {
            self.emit(BleEvent::Status(
                "SensorStream characteristic not advertised — legacy firmware, live tab will be empty".into()
            ));
        }
        let stream: NotifStream = match tokio::time::timeout(
            LINK_OP_TIMEOUT, p.notifications()
        ).await {
            Ok(Ok(s)) => Box::pin(s),
            Ok(Err(e)) => {
                drop_link(&p).await;
                return Err(format!("notifications: {e}"));
            }
            Err(_) => {
                drop_link(&p).await;
                return Err("notifications: timed out".into());
            }
        };
        // Hand the raw btleplug stream to a dedicated pump task that only
        // forwards into an unbounded channel, so notifications keep
        // draining while the worker is blocked elsewhere (see `notif_rx`).
        let (ntx, nrx) = tokio_mpsc::unbounded_channel::<ValueNotification>();
        let pump = tokio::spawn(async move {
            let mut stream = stream;
            while let Some(n) = stream.next().await {
                if ntx.send(n).is_err() {
                    break; // consumer gone (disconnect) — stop pumping.
                }
            }
            // stream.next() → None means the peripheral closed the stream;
            // dropping `ntx` here makes the consumer's `recv()` return None,
            // which the worker loop treats as a remote disconnect.
        });
        self.peripheral  = Some(p);
        self.notif_rx    = Some(nrx);
        self.notif_pump  = Some(pump);
        self.op          = CurrentOp::Idle;
        Ok(())
    }

    /// Auto-reconnect after an unexpected mid-transfer drop. Re-scans
    /// then re-connects to the same peripheral id (the box needs a
    /// moment to re-advertise once it's healed / back in range, hence
    /// the scan + delay each round). Returns `true` (and has emitted
    /// `Connected`) on success.
    ///
    /// Regime depends on `persist_reconnect` (the "Keep synced"
    /// checkbox) — see the `RECONNECT_*` consts:
    /// - **Auto Mode:** unbounded, exponential backoff. The only exits
    ///   are success or an explicit abort.
    /// - **Manual mode:** bounded to `RECONNECT_ATTEMPTS`, then return
    ///   `false` → caller emits `Disconnected` → manual reconnect
    ///   banner (lossless via the live mirror).
    ///
    /// Either way an abort (Disconnect / quit / "Keep synced" un-ticked
    /// dropping us back to the bounded budget) breaks the loop promptly.
    /// macOS fast path: retrieve the box by identifier (no scan) and connect,
    /// mirroring iOS `retrievePeripherals(withIdentifiers:)` + a pending
    /// `connect()`. `add_peripheral` (the vendored `btleplug-patched`
    /// addition) repopulates btleplug's maps — purged on disconnect and NOT
    /// refillable by scan when CoreBluetooth has stopped returning the box in
    /// scan results — so `connect_core`'s Map lookup then succeeds without a
    /// scan hit. Returns `Ok(())` only on a full connect+subscribe; any error
    /// → the caller falls back to the scan loop.
    ///
    /// NOTE: retrieve success proves only that CoreBluetooth still *knows* the
    /// id, NOT that the box is reachable — reachability is proven solely by
    /// `connect_core` (a fulfilled pending connect). `connect_core` bounds
    /// `p.connect()` with `LINK_OP_TIMEOUT`, which is our per-round
    /// pending-connect budget.
    #[cfg(target_os = "macos")]
    async fn try_retrieve_connect(
        &mut self,
        adapter: &Adapter,
        id: &str,
        mode: ConnectMode,
    ) -> Result<(), String> {
        let uuid = Uuid::parse_str(id).map_err(|e| format!("bad box id: {e}"))?;
        let pid: btleplug::platform::PeripheralId = uuid.into();
        adapter
            .add_peripheral(&pid)
            .await
            .map_err(|e| format!("retrieve-by-id: {e}"))?;
        self.emit(BleEvent::Status(
            "reconnect: retrieved box by id (no scan) — pending connect…".into(),
        ));
        eprintln!("[reconnect] retrieve-by-id OK for {id}; scan not required");
        self.connect_core(id, mode).await
    }

    async fn auto_reconnect(&mut self, id: String) -> bool {
        // A stale abort from a *previous* link must not kill this fresh
        // recovery — the user pressing Disconnect now is what sets it.
        self.clear_abort();
        let mut attempt: u32 = 0;
        // macOS retrieve-by-id fast path: give up on it after this many
        // consecutive failures this invocation (box is genuinely silent, not
        // just scan-invisible) and let the scan fallback carry the rest.
        #[cfg(target_os = "macos")]
        let mut retrieve_fails: u32 = 0;
        loop {
            attempt += 1;
            let persist = self.persist_reconnect();
            // Connect budget for this round: Auto (Keep synced on) holds one
            // pending connect unbounded; Manual holds it RECONNECT_CONNECT_TIMEOUT
            // then voluntarily re-issues. Rides the box's post-drop self-heal
            // instead of self-cancelling every LINK_OP_TIMEOUT.
            let mode = ConnectMode::Reconnect {
                budget: if persist { None } else { Some(RECONNECT_CONNECT_TIMEOUT) },
            };
            // Manual mode (or Keep-synced just turned off mid-loop):
            // honour the bounded budget and fall back to manual.
            if !persist && attempt > RECONNECT_ATTEMPTS {
                return false;
            }
            if self.abort_requested() {
                self.clear_abort();
                return false;
            }
            let label = if persist {
                format!("link lost — scanning for box (attempt {attempt}, Auto Mode)…")
            } else {
                format!(
                    "link lost — scanning for box (attempt {attempt}/{RECONNECT_ATTEMPTS})…"
                )
            };
            self.emit(BleEvent::Status(label));

            let Some(adapter) = self.ensure_adapter().await else {
                // No adapter yet — brief pause, try again.
                if self.sleep_or_abort(RECONNECT_INTERVAL).await {
                    self.clear_abort();
                    return false;
                }
                continue;
            };

            // ---- macOS retrieve-by-id fast path (no scan) --------------------
            // Make the box connectable WITHOUT a scan (fixes the CoreBluetooth
            // "recently-connected box no longer appears in scan results" case,
            // which is what makes the desktop — unlike iOS/Android — need a
            // box power-cycle). Skip once we've failed RETRIEVE_MAX_FAILS times
            // in a row this invocation — that indicates the box is genuinely
            // silent, not merely scan-invisible, so stop paying the extra
            // ~20 s pending-connect per round and let the scan fallback run.
            #[cfg(target_os = "macos")]
            if retrieve_fails < RETRIEVE_MAX_FAILS && !self.abort_requested() {
                // Diagnostic discriminator: is the box in the scan set right
                // now? "target_present=false" immediately followed by a
                // successful retrieve-connect proves scan-staleness (the bug
                // this fixes) vs. a genuinely silent box.
                if let Ok(ps) = adapter.peripherals().await {
                    let present = ps.iter().any(|p| p.id().to_string() == id);
                    eprintln!(
                        "[reconnect] pre-retrieve scan snapshot: {} peripheral(s), target_present={}",
                        ps.len(), present
                    );
                }
                match self.try_retrieve_connect(&adapter, &id, mode).await {
                    Ok(()) => {
                        self.last_id = Some(id.clone());
                        self.hold_power();
                        self.emit(BleEvent::Status(
                            "auto-reconnected (retrieve-by-id) — resuming transfer".into(),
                        ));
                        eprintln!("[reconnect] connected via retrieve-by-id — scan was NOT used");
                        self.emit(BleEvent::Connected);
                        return true;
                    }
                    Err(e) => {
                        retrieve_fails += 1;
                        eprintln!(
                            "[reconnect] retrieve path failed ({e}) [{retrieve_fails}/{RETRIEVE_MAX_FAILS}] — falling back to scan"
                        );
                    }
                }
                if self.abort_requested() {
                    self.clear_abort();
                    return false;
                }
            }

            // ---- scan fallback ---------------------------------------------
            // If retrieve-by-id didn't connect (non-macOS, retrieve failure, or
            // the box is genuinely silent), fall back to re-discovering it in a
            // live scan. Start a scan and leave it running while we wait for the
            // id to reappear, then connect the instant it does — don't stop the
            // scan first (connecting while scanning is fine on CoreBluetooth and
            // avoids a stop→snapshot race). This replaces the old "one 3 s scan
            // then give up for up to 60 s" that missed the advertising box.
            let _ = adapter.start_scan(ScanFilter::default()).await;
            let found = self
                .wait_for_peripheral(&adapter, &id, RECONNECT_SCAN_WINDOW)
                .await;
            if self.abort_requested() {
                let _ = adapter.stop_scan().await;
                self.clear_abort();
                return false;
            }
            if !found {
                // Box didn't re-advertise within this window — pause and
                // loop (a fresh scan starts next round).
                let _ = adapter.stop_scan().await;
                if self.sleep_or_abort(RECONNECT_INTERVAL).await {
                    self.clear_abort();
                    return false;
                }
                continue;
            }
            // Re-discovered → connect (scan still running), then stop.
            let result = self.connect_core(&id, mode).await;
            let _ = adapter.stop_scan().await;
            match result {
                Ok(()) => {
                    self.last_id = Some(id);
                    // Assertion is normally still held from the original
                    // connect (we don't drop it in disconnect_inner), but
                    // re-take defensively in case a give-up path released it.
                    self.hold_power();
                    self.emit(BleEvent::Status(
                        "auto-reconnected — resuming transfer".into(),
                    ));
                    self.emit(BleEvent::Connected);
                    return true;
                }
                Err(e) => {
                    self.emit(BleEvent::Status(format!(
                        "reconnect attempt {attempt} failed: {e}"
                    )));
                    if self.sleep_or_abort(RECONNECT_INTERVAL).await {
                        self.clear_abort();
                        return false;
                    }
                }
            }
        }
    }

    /// With a scan running on `adapter`, poll btleplug's discovered set
    /// until the target `id` reappears (box re-advertised) or `window`
    /// elapses / an abort is requested. Returns `true` if found.
    ///
    /// This is the reconnect crux (v0.0.35): btleplug on macOS can only
    /// (re)connect to a peripheral it has re-discovered in a scan — it
    /// purges the peripheral on disconnect and offers no pending-connect —
    /// so we keep looking (≈3×/s) rather than firing one short scan and
    /// giving up. btleplug scans with AllowDuplicates set, so a still-
    /// advertising box re-reports promptly; the id (CoreBluetooth UUID) is
    /// stable across re-advertising, so the match is reliable.
    async fn wait_for_peripheral(&self, adapter: &Adapter, id: &str, window: Duration) -> bool {
        let deadline = Instant::now() + window;
        loop {
            if let Ok(ps) = adapter.peripherals().await {
                if ps.iter().any(|p| p.id().to_string() == id) {
                    return true;
                }
            }
            if Instant::now() >= deadline {
                return false;
            }
            if self.sleep_or_abort(Duration::from_millis(300)).await {
                return false;
            }
        }
    }

    /// Tear down peripheral + stream + op state without emitting the
    /// public `Disconnected` event. Used both by user-initiated
    /// disconnect and by failure paths inside the worker.
    async fn disconnect_inner(&mut self) {
        // If a download was in flight, hand the partial bytes back so
        // the caller can append them to `.part` (resume continues from
        // the real break point), then surface the abort so the UI
        // doesn't keep saying "reading…" forever.
        match std::mem::replace(&mut self.op, CurrentOp::Idle) {
            CurrentOp::Reading { name, content, expected, base, .. } => {
                let got = base + content.len() as u64;
                self.emit(BleEvent::ReadAborted {
                    name: name.clone(),
                    content,
                    base,
                });
                self.emit_err(format!(
                    "READ {name} aborted by disconnect at {got}/{expected} B"
                ));
            }
            CurrentOp::Listing { .. } => {
                self.emit_err("LIST aborted by disconnect");
            }
            CurrentOp::Deleting { name, .. } => {
                self.emit_err(format!("DELETE {name} aborted by disconnect"));
            }
            CurrentOp::GettingMode { .. } => {}
            CurrentOp::Flashing { .. } => {
                self.emit(BleEvent::FlashError {
                    msg: "firmware upload aborted by disconnect".into(),
                });
            }
            CurrentOp::Idle => {}
        }
        // Stop the notify pump (drops its stream) before tearing the link
        // down, then clear the receiver.
        if let Some(pump) = self.notif_pump.take() {
            pump.abort();
        }
        self.notif_rx       = None;
        self.stream_asm     = StreamAsm::default();
        self.stream_capable = false;
        if let Some(p) = self.peripheral.take() {
            drop_link(&p).await;
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

    /// GPS bridge on/off (`0x0D <u8>`). Fire-and-forget (no op slot): the
    /// box starts/stops relaying raw u-blox UBX frames over FileData. We
    /// flip `shared.gps_bridge` so `handle_notification` routes those frames
    /// to the GPS data sink, and drop the sink when turning the bridge off.
    async fn gps_bridge(&mut self, on: bool) {
        if self.peripheral.is_none() {
            self.emit_err("not connected");
            return;
        }
        if let Err(e) = self.write_cmd(&[0x0D, on as u8]).await {
            self.emit_err(format!("GPS bridge {}: {e}", if on { "on" } else { "off" }));
            return;
        }
        self.shared.gps_bridge.store(on, Ordering::SeqCst);
        if !on {
            if let Ok(mut g) = self.shared.gps_data_tx.lock() { *g = None; }
        }
    }

    /// Forward raw bytes to the u-blox over the bridge (`0x0E` + bytes) — the
    /// survey loop's UBX poll frames. No reply; the answers arrive as
    /// bridged FileData notifies.
    async fn gps_tx(&mut self, bytes: Vec<u8>) {
        if bytes.is_empty() { return; }
        let mut payload = Vec::with_capacity(1 + bytes.len());
        payload.push(0x0E);
        payload.extend_from_slice(&bytes);
        if let Err(e) = self.write_cmd(&payload).await {
            self.emit_err(format!("GPS tx: {e}"));
        }
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

    async fn read(&mut self, name: String, size: u64, offset: u64) {
        if !matches!(self.op, CurrentOp::Idle) {
            self.emit_err("another op is in flight — wait or Disconnect");
            return;
        }
        if self.peripheral.is_none() {
            self.emit_err("not connected");
            return;
        }
        // Opcode payload: 0x02 + name + NUL + 4-byte LE start offset.
        // The firmware (ble.c READ handler) seeks to `offset` before
        // streaming, so a resumed transfer continues mid-file. offset
        // is u32 on the wire — SD files are well under 4 GiB.
        let mut payload = Vec::with_capacity(name.len() + 6);
        payload.push(0x02);
        payload.extend_from_slice(name.as_bytes());
        payload.push(0x00);
        payload.extend_from_slice(&(offset as u32).to_le_bytes());
        if let Err(e) = self.write_cmd(&payload).await {
            self.emit_err(e);
            return;
        }
        let remaining = size.saturating_sub(offset) as usize;
        self.op = CurrentOp::Reading {
            name: name.clone(),
            expected: size,
            base: offset,
            content: Vec::with_capacity(remaining),
            last_emit: offset,
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

    /// Send SET_TIME (0x08) + 8-byte LE epoch-ms so the box stamps a
    /// `# SYNC` anchor into the open Sens/Gps CSVs. Like `stop_log`, this is
    /// fire-and-forget: it does NOT take a `CurrentOp` slot, so the box's
    /// optional OK byte lands harmlessly while `op == Idle` and legacy
    /// firmware without 0x08 can't stall the worker.
    async fn set_time(&mut self, epoch_ms: u64) {
        let mut payload = Vec::with_capacity(9);
        payload.push(0x08);
        payload.extend_from_slice(&epoch_ms.to_le_bytes());
        if let Err(e) = self.write_cmd(&payload).await {
            self.emit_err(e);
            return;
        }
        /* Space SET_TIME from the auto-LIST that the connect handler queues
           right behind it: the firmware holds only ONE pending FileCmd at a
           time, so a LIST landing in the same connection interval would
           clobber the SET_TIME before the box stamps its marker. The worker
           processes commands sequentially, so this sleep delays the next
           command — same trick `start_log` uses above. */
        tokio::time::sleep(Duration::from_millis(500)).await;
        self.emit(BleEvent::Status("SET_TIME sent — box clock anchored to host".into()));
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

    /// SET_MODE (0x06) + 1-byte mode (0 = auto, 1 = manual), persisted to
    /// LOGMODE.CFG on the box. The firmware honours it regardless of any
    /// in-flight op and replies with a 1-byte OK status on FileData.
    ///
    /// We light the chosen Auto/Manual label *optimistically* (emit `LogMode`
    /// at once) instead of reading the mode back here. The old code re-queried
    /// with GET_MODE, but `get_log_mode` early-returns while the worker is busy
    /// (`op != Idle`), so during a sync — exactly when a user wants Auto so the
    /// box logs on power-on — no confirmation ever arrived and the label looked
    /// dead (Peter's "kann den log mode im UX nicht auf auto umstellen"). The
    /// box persists synchronously so the chosen value is authoritative; the
    /// next connect's GET_MODE (`ListDone` reconcile) corrects the rare case
    /// where the write was lost. Re-reading right after SET_MODE was also
    /// unsafe: the OK-status byte (`0x00`) would be consumed by the
    /// `GettingMode` arm as if it were the mode, so setting Manual read back as
    /// Auto.
    async fn set_log_mode(&mut self, manual: bool) {
        if self.peripheral.is_none() {
            self.emit_err("not connected");
            return;
        }
        self.emit(BleEvent::LogMode { manual });
        if !matches!(self.op, CurrentOp::Idle) {
            // Mid-LIST/READ: defer the 0x06 write so the firmware's OK-status
            // byte can't land in the in-flight download stream. The worker
            // loop applies it the instant `op` returns to Idle.
            self.pending_set_mode = Some(manual);
            return;
        }
        self.write_set_mode(manual).await;
    }

    /// Write SET_MODE (0x06) + mode byte. Fire-and-forget: no `CurrentOp`
    /// slot (like `stop_log` / `set_time`), so the box's 1-byte OK status
    /// lands harmlessly while `op == Idle` and never collides with a
    /// GET_MODE read-back.
    async fn write_set_mode(&mut self, manual: bool) {
        if let Err(e) = self.write_cmd(&[0x06, manual as u8]).await {
            self.emit_err(e);
            return;
        }
        self.emit(BleEvent::Status(format!(
            "SET_MODE {} sent",
            if manual { "manual" } else { "auto" }
        )));
    }

    /// GET_MODE (0x07). Box replies with one byte over FileData
    /// (0 = auto, 1 = manual); handled in the GettingMode op arm.
    async fn get_log_mode(&mut self) {
        if !matches!(self.op, CurrentOp::Idle) {
            // Don't trample an in-flight LIST/READ — GET_MODE is
            // best-effort; the next idle GetLogMode will succeed.
            return;
        }
        if self.peripheral.is_none() {
            self.emit_err("not connected");
            return;
        }
        if let Err(e) = self.write_cmd(&[0x07]).await {
            self.emit_err(e);
            return;
        }
        self.op = CurrentOp::GettingMode { last_progress: Instant::now() };
    }

    // ---------------- Firmware upload (OTA) --------------------------------

    /// Pull the next FileData reply off the notification stream, draining
    /// (and ignoring) any concurrent SensorStream notifies, with a
    /// timeout. Used only by `flash_firmware`, which owns the stream for
    /// the duration of the upload so notifications never reach
    /// `handle_notification` while a flash is in flight. Returns:
    /// - `Ok(Some(bytes))` — a FileData payload arrived,
    /// - `Ok(None)`        — the stream closed (remote disconnect),
    /// - `Err(())`         — `timeout` elapsed with no FileData reply.
    async fn next_filedata(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, ()> {
        let deadline = Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(());
            }
            let Some(notif_rx) = self.notif_rx.as_mut() else {
                return Ok(None);
            };
            match tokio::time::timeout(remaining, notif_rx.recv()).await {
                Ok(Some(n)) => {
                    if n.uuid == FILEDATA_UUID {
                        return Ok(Some(n.value));
                    }
                    if n.uuid == STREAM_UUID {
                        // Live-stream notify landed mid-flash — decode it
                        // so the Live tab keeps ticking, then keep waiting
                        // for the FileData reply we actually want.
                        self.handle_stream_notification(&n.value);
                    }
                    // Any other characteristic — ignore, keep waiting.
                }
                Ok(None) => return Ok(None), // stream closed
                Err(_) => return Err(()),    // timed out
            }
        }
    }

    /// Upload a firmware image to the box (dual-bank OTA). Sends
    /// FW_BEGIN, streams FW_DATA chunks gated on the box's next-expected
    /// offset ACK, then FW_COMMIT. Drives the single-op `CurrentOp` slot
    /// (rejected while another op is in flight; rejects others while it
    /// runs) but reads its replies *inline* off the notification stream
    /// rather than through `handle_notification`, because each FW step is
    /// a strict request→reply round-trip with its own timeout + retry —
    /// the demuxed-notify model the other ops use doesn't fit a flow that
    /// has to resend the same offset on a missed ACK. On any fatal error
    /// it sends FW_ABORT best-effort and emits `FlashError`.
    async fn flash_firmware(&mut self, bytes: Arc<Vec<u8>>) {
        use sha2::{Digest, Sha256};

        if !matches!(self.op, CurrentOp::Idle) {
            self.emit(BleEvent::FlashError {
                msg: "another op is in flight — wait or Disconnect".into(),
            });
            return;
        }
        if self.peripheral.is_none() {
            self.emit(BleEvent::FlashError { msg: "not connected".into() });
            return;
        }
        let image_len = bytes.len();
        if image_len == 0 {
            self.emit(BleEvent::FlashError { msg: "firmware file is empty".into() });
            return;
        }

        // Claim the single-op slot for the whole upload.
        self.op = CurrentOp::Flashing { last_progress: Instant::now() };

        let sha = {
            let mut h = Sha256::new();
            h.update(bytes.as_slice());
            h.finalize()
        };

        // --- FW_BEGIN: opcode + len(u32-LE) + sha256(32) = 37 bytes ----
        let mut begin = Vec::with_capacity(37);
        begin.push(OP_FW_BEGIN);
        begin.extend_from_slice(&(image_len as u32).to_le_bytes());
        begin.extend_from_slice(&sha);
        if let Err(e) = self.write_cmd(&begin).await {
            self.op = CurrentOp::Idle;
            self.emit(BleEvent::FlashError { msg: format!("FW_BEGIN write: {e}") });
            return;
        }
        match self.next_filedata(FW_BEGIN_TIMEOUT).await {
            Ok(Some(v)) if v.first() == Some(&0x00) => { /* ready */ }
            Ok(Some(v)) => {
                let code = v.first().copied().unwrap_or(0);
                self.op = CurrentOp::Idle;
                self.abort_firmware().await;
                self.emit(BleEvent::FlashError { msg: fw_begin_err(code) });
                return;
            }
            Ok(None) => {
                self.op = CurrentOp::Idle;
                self.emit(BleEvent::FlashError {
                    msg: "link dropped during FW_BEGIN".into(),
                });
                return;
            }
            Err(()) => {
                self.op = CurrentOp::Idle;
                self.abort_firmware().await;
                self.emit(BleEvent::FlashError {
                    msg: "FW_BEGIN timed out (bank erase) — try again".into(),
                });
                return;
            }
        }
        self.emit(BleEvent::FlashStarted { total: image_len as u64 });

        // --- FW_DATA: offset-gated chunk loop --------------------------
        let mut offset = 0usize;
        while offset < image_len {
            // Honour a Disconnect / quit promptly instead of running the whole
            // upload to completion: the single-op worker can't service the
            // command channel while this loop owns the stream, so without this
            // a quit mid-OTA looked like a frozen UI until the image finished.
            if self.abort_requested() {
                self.op = CurrentOp::Idle;
                self.abort_firmware().await;
                self.emit(BleEvent::FlashError {
                    msg: format!("firmware upload aborted at {offset} B"),
                });
                return;
            }
            let n = FW_CHUNK.min(image_len - offset);
            let mut chunk = Vec::with_capacity(5 + n);
            chunk.push(OP_FW_DATA);
            chunk.extend_from_slice(&(offset as u32).to_le_bytes());
            chunk.extend_from_slice(&bytes[offset..offset + n]);

            let mut attempt = 0u32;
            let next_off = loop {
                if let Err(e) = self.write_cmd(&chunk).await {
                    self.op = CurrentOp::Idle;
                    self.abort_firmware().await;
                    self.emit(BleEvent::FlashError {
                        msg: format!("FW_DATA write at {offset}: {e}"),
                    });
                    return;
                }
                match self.next_filedata(FW_DATA_TIMEOUT).await {
                    Ok(Some(v)) if v.len() == 4 => {
                        // 4-byte LE next-expected offset ACK.
                        break u32::from_le_bytes([v[0], v[1], v[2], v[3]]) as usize;
                    }
                    Ok(Some(v)) => {
                        // 1-byte error (0xE7 bad-seq / 0xE5 flash-fail).
                        let code = v.first().copied().unwrap_or(0);
                        self.op = CurrentOp::Idle;
                        self.abort_firmware().await;
                        self.emit(BleEvent::FlashError { msg: fw_data_err(code) });
                        return;
                    }
                    Ok(None) => {
                        self.op = CurrentOp::Idle;
                        self.emit(BleEvent::FlashError {
                            msg: format!("link dropped during FW_DATA at {offset} B"),
                        });
                        return;
                    }
                    Err(()) => {
                        // Missed ACK — resend the SAME offset (the box is
                        // idempotent for offset < its write cursor).
                        attempt += 1;
                        if attempt >= FW_DATA_RETRIES {
                            self.op = CurrentOp::Idle;
                            self.abort_firmware().await;
                            self.emit(BleEvent::FlashError {
                                msg: format!(
                                    "FW_DATA timed out at {offset} B after {attempt} retries"
                                ),
                            });
                            return;
                        }
                        // loop → resend this chunk
                    }
                }
            };
            // The box drives the cursor: jump to whatever offset it
            // reports next (normally offset + n; on a resent-after-ACK
            // race it may already be ahead, which is fine).
            offset = next_off.max(offset + n).min(image_len);
            if let CurrentOp::Flashing { last_progress } = &mut self.op {
                *last_progress = Instant::now();
            }
            self.emit(BleEvent::FlashProgress {
                bytes_done: offset as u64,
                total: image_len as u64,
            });
        }

        // --- FW_COMMIT: verify hash, swap banks, RESET -----------------
        if let Err(e) = self.write_cmd(&[OP_FW_COMMIT]).await {
            self.op = CurrentOp::Idle;
            self.emit(BleEvent::FlashError { msg: format!("FW_COMMIT write: {e}") });
            return;
        }
        match self.next_filedata(FW_COMMIT_TIMEOUT).await {
            // 0xA0 OK, or the link dropping right after COMMIT (the box
            // resets ~200 ms after replying) — both mean success.
            Ok(Some(v)) if v.first() == Some(&0xA0) => {
                self.op = CurrentOp::Idle;
                self.emit(BleEvent::FlashDone);
            }
            Ok(None) => {
                self.op = CurrentOp::Idle;
                self.emit(BleEvent::FlashDone);
            }
            Ok(Some(v)) => {
                let code = v.first().copied().unwrap_or(0);
                self.op = CurrentOp::Idle;
                self.emit(BleEvent::FlashError { msg: fw_commit_err(code) });
            }
            Err(()) => {
                // No reply — the box may have already reset (link still
                // nominally up on the host side). Treat as success: the
                // bank swap only happens on a verified image, so a silent
                // COMMIT means the image was accepted and the box rebooted.
                self.op = CurrentOp::Idle;
                self.emit(BleEvent::FlashDone);
            }
        }
    }

    /// Best-effort FW_ABORT (one byte 0x00 reply, which we don't wait
    /// for). Used on every fatal path so a half-open update session on
    /// the box is closed cleanly rather than blocking the next attempt.
    async fn abort_firmware(&self) {
        let _ = self.write_cmd(&[OP_FW_ABORT]).await;
    }

    // ---------------- Notification dispatch --------------------------------

    async fn handle_notification(&mut self, n: ValueNotification) {
        // SensorStream notifies are concurrent with FileSync, so handle
        // them on their own path before touching `op`.
        if n.uuid == STREAM_UUID {
            self.handle_stream_notification(&n.value);
            return;
        }
        // GPS-bridge mode: FileData notifies carry raw u-blox UBX frames,
        // not FileSync payloads. Hand them straight to the survey sink and
        // never touch `op`. (The survey assumes you're not also syncing —
        // both would land on FileData and corrupt each other.)
        if n.uuid == FILEDATA_UUID && self.shared.gps_bridge.load(Ordering::SeqCst) {
            if let Ok(g) = self.shared.gps_data_tx.lock() {
                if let Some(tx) = g.as_ref() { let _ = tx.send(n.value.clone()); }
            }
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
            CurrentOp::Reading { name, expected, base, content, last_emit, last_progress, first_packet } => {
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

                // Absolute position = resume base + bytes this segment.
                // Throttle progress events — every ~4 KB or at EOF.
                let done = *base + content.len() as u64;
                if done - *last_emit >= 4 * 1024 || done >= *expected {
                    *last_emit = done;
                    self.emit(BleEvent::ReadProgress { name: name.clone(), bytes_done: done });
                }

                if done >= *expected {
                    // Trim any over-read so base + content == expected.
                    let want = expected.saturating_sub(*base) as usize;
                    if content.len() > want {
                        content.truncate(want);
                    }
                    let final_content = std::mem::take(content);
                    let final_name    = std::mem::take(name);
                    let final_base    = *base;
                    self.emit(BleEvent::ReadDone {
                        name: final_name,
                        content: final_content,
                        base: final_base,
                    });
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
            CurrentOp::GettingMode { .. } => {
                /* Firmware replies with exactly one byte: 0 = auto,
                   1 = manual. Anything else → ignore (stay "unknown"). */
                if let Some(&b) = n.value.first() {
                    if b == 0 || b == 1 {
                        self.emit(BleEvent::LogMode { manual: b == 1 });
                    }
                }
                self.op = CurrentOp::Idle;
                return;
            }
            CurrentOp::Flashing { .. } => {
                /* The flash drives the stream inline (see `flash_firmware`
                   / `next_filedata`), so a FileData reply never reaches
                   here while a flash is in flight. If one somehow did
                   (stray late notify), ignore it and keep the op so the
                   in-flight upload isn't disturbed. */
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

    /// Returns true iff a READ just stalled (no notifies for
    /// `OP_IDLE_TIMEOUT`) — the caller should tear the half-dead link
    /// down and auto-reconnect. This is the path that matters in
    /// practice: a real-world drop (macOS BLE power-nap, link-
    /// supervision timeout, worker falling behind) often does NOT
    /// close the notification stream, so the stream-closed reconnect
    /// path never fires — only the watchdog sees it.
    fn tick_watchdog(&mut self) -> bool {
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
                return false;
            }
        }

        let stale = match &self.op {
            CurrentOp::Listing  { last_progress, .. } => now.duration_since(*last_progress) > OP_IDLE_TIMEOUT,
            CurrentOp::Reading  { last_progress, .. } => now.duration_since(*last_progress) > OP_IDLE_TIMEOUT,
            CurrentOp::Deleting { last_progress, .. } => now.duration_since(*last_progress) > OP_IDLE_TIMEOUT,
            CurrentOp::GettingMode { last_progress } => now.duration_since(*last_progress) > OP_IDLE_TIMEOUT,
            // A flash is driven inline and never yields to the watchdog
            // (its own per-step timeouts cover stalls), so it's never
            // "stale" here.
            CurrentOp::Flashing { .. } => false,
            CurrentOp::Idle => false,
        };
        if !stale { return false; }

        match std::mem::replace(&mut self.op, CurrentOp::Idle) {
            CurrentOp::Listing { .. } => {
                // A stalled zero-row LIST does NOT reconnect (v0.0.21,
                // reverses v0.0.18). The Android app — same box, same
                // firmware, proven working where this desktop wasn't —
                // does exactly this: on a stalled LIST it just logs the
                // timeout, drops to Idle, and keeps the link; the next
                // periodic LIST (every 30 s under Keep synced) retries
                // on the still-alive link and succeeds. v0.0.18's
                // premise ("a zero-row LIST stall is a genuinely dead
                // link, reconnect it") is wrong: combined with v0.0.19's
                // *unbounded* Auto-Mode reconnect it turned one slow
                // LIST into an infinite teardown→reconnect→re-LIST→stall
                // loop with the box advertising throughout — exactly
                // Peter's "FileList geht nicht mehr, Box ist BT
                // sichtbar". `op` is already Idle (replaced above), so
                // the next op proceeds normally. Don't reinstate
                // reconnect-on-LIST-stall without re-confirming against
                // the Android reference behaviour.
                self.emit_err("LIST timed out — no notifies for 20 s");
                false
            }
            CurrentOp::Reading { name, content, expected, base, .. } => {
                let got = base + content.len() as u64;
                // Hand back the partial so the resume continues from
                // here, not from the last fully-completed segment.
                self.emit(BleEvent::ReadAborted {
                    name: name.clone(),
                    content,
                    base,
                });
                self.emit_err(format!(
                    "READ {name} timed out at {got}/{expected} B — no notifies for 20 s"
                ));
                true // stalled READ → caller reconnects + resumes
            }
            CurrentOp::Deleting { name, .. } => {
                self.emit_err(format!("DELETE {name} timed out — no notify for 20 s"));
                false
            }
            CurrentOp::GettingMode { .. } => {
                // Legacy PumpTsueri firmware never replies to GET_MODE.
                // Silently drop to Idle (mode stays "unknown"); never
                // reconnect — there's nothing wrong with the link.
                false
            }
            // Unreachable in practice (the flash never yields here), but
            // matched for exhaustiveness — no reconnect.
            CurrentOp::Flashing { .. } => false,
            CurrentOp::Idle => false,
        }
    }
}

// ---------------------------------------------------------------------------
//  Parsing helpers
// ---------------------------------------------------------------------------

/// Disconnect a peripheral without letting it wedge the worker.
///
/// When the peer vanished without a clean LL_TERMINATE (box shielded /
/// out of range), macOS CoreBluetooth's `disconnect()` can block until
/// the link-supervision timeout — or effectively forever. That froze
/// the worker *before* it reached the auto-reconnect (the exact
/// "Sync bleibt stehen" Peter saw). Bound it: 3 s is plenty for a
/// healthy disconnect; past that we just drop our handle and move on,
/// which is enough to free our state and let reconnect proceed.
async fn drop_link(p: &Peripheral) {
    let _ = tokio::time::timeout(Duration::from_secs(3), p.disconnect()).await;
}

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

/// Map a FW_BEGIN error byte to a user-facing message.
fn fw_begin_err(code: u8) -> String {
    let why = match code {
        0xB0 => "box busy (logging in progress — STOP_LOG / wait first)",
        0xE6 => "image too big for the firmware bank",
        0xE5 => "flash erase failed",
        0xE3 => "bad request (firmware doesn't support OTA?)",
        _    => "unknown error",
    };
    format!("FW_BEGIN rejected: {why} (0x{code:02X})")
}

/// Map a FW_DATA error byte to a user-facing message.
fn fw_data_err(code: u8) -> String {
    let why = match code {
        0xE7 => "bad sequence/offset",
        0xE5 => "flash write failed",
        _    => "unknown error",
    };
    format!("FW_DATA rejected: {why} (0x{code:02X})")
}

/// Map a FW_COMMIT error byte to a user-facing message.
fn fw_commit_err(code: u8) -> String {
    let why = match code {
        0xE4 => "image rejected (hash mismatch — re-download the .bin)",
        0xE7 => "short image (fewer bytes received than declared)",
        0xE5 => "flash finalize failed",
        _    => "unknown error",
    };
    format!("FW_COMMIT rejected: {why} (0x{code:02X})")
}
