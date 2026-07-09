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
//!   0x0D GPS_BRIDGE <u8>      — relay u-blox UBX over BLE (GPS Debug tab)
//!   0x0E GPS_TX <bytes…>      — forward host UBX poll frames to the u-blox
//!   0x0F DISCONNECT           — box HCI-terminates the link + re-advertises.
//!                               No reply. macOS fix: cancelPeripheralConnection
//!                               leaves the ACL alive controller-side, so we ask
//!                               the box to drop it (firmware v0.0.22+).
//!   0x10 GET_VERSION          — replies with one FileData notify carrying the
//!                               ASCII firmware version (e.g. "0.0.29", no NUL).
//!                               Legacy firmware (≤ v0.0.28) doesn't know it and
//!                               sends no reply → the GettingVersion op times out
//!                               → FirmwareVersion(None). Same single-op/demux
//!                               rules as GET_MODE (0x07).
//!   0x11 GPS_POWER <u8>       — 1=on / 0=off; turns the u-blox receiver on/off
//!                               to save battery (off → UBX-RXM-PMREQ backup,
//!                               ~tens of µA vs ~25 mA). Persisted on the box +
//!                               re-applied at boot. Reply = 1 status byte
//!                               (0x00 OK), exactly like SET_MODE (0x06).
//!   0x12 GPS_GET_POWER        — replies one byte 1=on / 0=off. Twin of
//!                               GET_MODE (0x07). Firmware v0.0.35+; legacy
//!                               firmware ignores both → the op times out and
//!                               the toggle stays "unknown".
//!
//!   Status bytes: 0x00 OK, 0xB0 BUSY, 0xE1 NOT_FOUND, 0xE2 IO_ERROR, 0xE3 BAD_REQ.

use btleplug::api::{
    Central, CentralEvent, CharPropFlags, Characteristic, Manager as _, Peripheral as _,
    ScanFilter, ValueNotification, WriteType,
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

/// FileSync GATT service (firmware `BLE_SVC_UUID`). Needed to synthesize a
/// Characteristic for the polite-teardown 0x0F write when the app-side char
/// cache is empty (raced pending connect) — see `polite_drop_link`.
const FSYNC_SVC_UUID: Uuid = Uuid::from_u128(0x00000000_0001_11e1_9ab4_0002a5d5c51b);
const FILECMD_UUID:  Uuid = Uuid::from_u128(0x00000080_0010_11e1_ac36_0002a5d5c51b);
const FILEDATA_UUID: Uuid = Uuid::from_u128(0x00000040_0010_11e1_ac36_0002a5d5c51b);
/// SensorStream — 0.5 Hz packed 46-byte all-sensor snapshot. New in
/// PumpLogger; not present in SDDataLogFileX (optional characteristic).
/// See DESIGN.md §3 for the byte layout.
const STREAM_UUID:   Uuid = Uuid::from_u128(0x00000100_0010_11e1_ac36_0002a5d5c51b);
/// BatteryStatus — 8-byte packed STC3115 fuel-gauge snapshot; the box
/// notifies once/min + immediately on the low-battery transition, and the
/// value is READ-able on demand. New in PumpLogger; absent on legacy
/// SDDataLogFileX (optional characteristic). NOTE: its flags byte differs
/// from the SensorStream packet — here bit0=low_batt, bit1=logging.
const BATTERY_UUID:  Uuid = Uuid::from_u128(0x00000200_0010_11e1_ac36_0002a5d5c51b);

/// Watchdog — if no notification arrives for this long during an active
/// LIST or READ, give up and surface a timeout error so the user isn't
/// left staring at "running…" forever (e.g. after a drop-out the box
/// reconnects but our subscription went stale).
const OP_IDLE_TIMEOUT: Duration = Duration::from_secs(20);
/// READ ride-out: how long a READ may go with ZERO notify progress before we
/// *note* it (a macOS notify pause). We do NOT tear the link down here — see
/// `READ_RIDE_OUT_LIMIT`. A file READ streams thousands of notifies over
/// minutes; macOS CoreBluetooth routinely *parks* delivery for tens of seconds
/// under sustained load while keeping the LL link fully alive (it time-shares
/// the 2.4 GHz radio across Wi-Fi + every other BLE peripheral and
/// deprioritises a "background" box). iOS/Android don't park like this, so
/// they sync straight through — this is a macOS-only artefact.
///
/// v0.0.62 — RIDE OUT, don't reconnect. The old design tore the link down when
/// this elapsed (the box then saw `reason=0x13`, re-adv=211, minute-long
/// gaps). But the link is NOT dead during a park: the box (fw v0.0.39) holds
/// the segment and resumes the instant macOS drains again, so the SAME stream
/// just continues — no teardown, no reconnect, no 211. A genuinely dead peer
/// is signalled by the `CentralEvent::DeviceDisconnected` sweep (`central_rx`
/// → `lost_link_recover`, ~200 ms after macOS reports it), NOT by silence.
/// This value is now only the point at which we emit a one-shot "riding out"
/// status line for visibility; teardown happens only at `READ_RIDE_OUT_LIMIT`.
const READ_STALL_TIMEOUT: Duration = Duration::from_secs(45);

/// READ last-resort: absolute cap on a notify-parked READ before we give up on
/// the live link and fall back to teardown + auto-reconnect (byte-resume from
/// the mirror). Sits ABOVE the box's own 180 s READ-stall deadline
/// (`FSM_READ_STALL_DEADLINE_MS`, fw v0.0.39), so in the normal case the box
/// recovers first and macOS reports `DeviceDisconnected` — which reconnects us
/// via `central_rx` well before this fires. This only covers the pathological
/// "LL link stays up but data never resumes AND the box never reports a
/// disconnect" corner; without it a wedged READ could hang forever.
const READ_RIDE_OUT_LIMIT: Duration = Duration::from_secs(200);

/// Chunk a long file READ into fixed-size segments instead of one to-EOF
/// stream (v0.0.48). Each segment is a separate capped READ opcode
/// (`READ <name>\0<offset><cap>`, firmware v0.0.24+): the box streams
/// exactly this many bytes, then its READ state machine returns to
/// FSM_IDLE — which resets the firmware's own 45 s `FSM_READ_STALL_DEADLINE`
/// every segment. A single 200 MB READ kept the box in FSM_READ_STREAM for
/// the entire multi-hour transfer, so any macOS BLE pause ≥ 45 s tripped
/// the firmware stall → `ble_recover_lost_peer` failed to re-advertise
/// (rc=211, ACL-TX still saturated) → the box went radio-silent and the
/// desktop's reconnect had nothing to connect to (Peter's overnight
/// freeze; ERRLOG 04.07.2026 shows 13 stalls, all re-adv=211). Cycling to
/// IDLE every ≤ 21 s (64 KB even at the ~3 KB/s worst case) keeps the box
/// out of that trap. Deliberately NOT paired with a lower
/// READ_STALL_TIMEOUT: dropping the desktop side to 20 s was tried and
/// rolled back (see that const's history) — it churned. Segmenting fixes
/// the box side without touching the desktop ride-out. Also bounds worker
/// RAM to one segment: each completed segment is flushed to the mirror via
/// `BleEvent::ReadChunk` and drained, so a healthy 200 MB transfer no
/// longer buffers 200 MB before the terminal ReadDone.
const READ_SEGMENT_BYTES: u64 = 64 * 1024;
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

/// Host-requested disconnect (macOS fix, firmware v0.0.22+). On macOS
/// `cancelPeripheralConnection` tears down only the *app's* view of the link;
/// bluetoothd keeps the ACL alive with LL keepalives, so the box never sees
/// LL_TERMINATE, its supervision timer never fires, and it stays connected-in-
/// limbo advertising non-connectably — the next central (e.g. the iPhone) then
/// can't connect until a box power-cycle. iOS tears the link down for real, so
/// it doesn't hit this. We send this fire-and-forget opcode right before
/// `drop_link` so the box terminates the link from ITS side (real LL_TERMINATE)
/// and re-advertises. Legacy firmware silently ignores it (unknown opcode).
const OP_DISCONNECT: u8 = 0x0F;

/// How long to wait for the `0x0F` ATT write-response before we give up and
/// cancel the connection locally anyway. The box sends the ATT ACK before it
/// acts on the opcode, so this normally resolves in well under a second; the
/// bound just stops a flaky link from hanging the disconnect.
const HOST_DISCONNECT_ACK_TIMEOUT: Duration = Duration::from_secs(3);

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
    /// peripherals appear, then `ScanStopped`. `known_id` is the
    /// previously-connected box's peripheral id (config.toml `box_id`):
    /// a peripheral matching it is surfaced even when its name doesn't
    /// match `BOX_NAMES` — macOS serves scan names from a persistent
    /// per-peripheral cache that can go stale/empty (survives reboots),
    /// which otherwise makes a known box invisible to the name filter.
    Scan { known_id: Option<String> },
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
    /// GET_VERSION opcode `0x10` (firmware v0.0.29+). Box replies with one
    /// FileData notify carrying the ASCII firmware version (e.g. `"0.0.29"`,
    /// no NUL) → `BleEvent::FirmwareVersion(Some(..))`. Mirrors `GetLogMode`
    /// exactly (single-byte command, demuxed reply). Legacy firmware never
    /// replies; the `GettingVersion` op times out and emits
    /// `FirmwareVersion(None)` without reconnecting.
    GetFirmwareVersion,
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
    /// GPS_POWER opcode `0x11` + `<u8>` (1 = on, 0 = off). Turns the box's
    /// u-blox receiver on/off to save battery when GPS is faulty/unused (off →
    /// UBX-RXM-PMREQ backup, ~tens of µA vs ~25 mA). Persisted on the box and
    /// re-applied at boot. Box replies one status byte over FileData (0x00 OK →
    /// the box is now in the requested state) → `BleEvent::GpsPower`. Demuxed by
    /// the single-op `GpsPowerReq` slot, exactly like `SetLogMode`/`GetLogMode`.
    /// Firmware v0.0.35+; legacy firmware silently ignores it (op times out).
    SetGpsPower { on: bool },
    /// GPS_GET_POWER opcode `0x12`. Box replies with a single byte over
    /// FileData (1 = on, 0 = off) → `BleEvent::GpsPower`. Twin of `GetLogMode`;
    /// legacy firmware never replies (the `GpsPowerReq` op times out and drops
    /// to Idle, leaving the toggle "unknown").
    GetGpsPower,
    /// CAL_GET opcode `0x13` (firmware v0.0.37+). Box replies with a 32-byte
    /// calibration blob (nosePlusY, magOffsetMg, angleZeroRef + epoch,
    /// headingBiasDeg — see `calibration.rs`) → `BleEvent::Calibration(Some)`.
    /// Twin of `GetLogMode`; legacy firmware silently ignores it, in which
    /// case the `CalibrationReq` op times out and emits `Calibration(None)`
    /// so the client falls back to its local `AgentConfig` copy.
    GetCalibration,
    /// CAL_SET opcode `0x14` + the 32-byte blob (firmware v0.0.37+). Box
    /// merges per-field into `CAL.CFG` and replies one status byte → the
    /// `CalibrationReq` op emits `Calibration(Some(blob))` on OK (with the
    /// stored copy = what the box will report on next `CAL_GET`) so the
    /// client can mirror it locally without an extra roundtrip. Legacy
    /// firmware silently ignores it; the op times out and the client keeps
    /// its optimistic local update. The caller composes the blob via
    /// `calibration::encode` — only fields whose `valid_mask` bit is set
    /// overwrite the stored ones.
    SetCalibration { blob: [u8; 32] },
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
    /// One intermediate READ segment completed (v0.0.48 segmenting). Unlike
    /// ReadDone this is NOT the end of the file — `content` is just the
    /// bytes of this 64 KB segment, `base` its start offset. The caller
    /// appends them to the mirror immediately (persisting progress and
    /// keeping worker RAM to one segment) but must NOT mark the file done
    /// or advance the queue — more segments follow and the terminal
    /// ReadDone closes the file. Emitted between segments while the box's
    /// READ FSM is briefly idle.
    ReadChunk { name: String, content: Vec<u8>, base: u64 },
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
    /// The box's firmware version, from a GET_VERSION (0x10) reply. `Some`
    /// carries the parsed ASCII version (e.g. `"0.0.29"`); `None` means the
    /// query got no reply (legacy firmware ≤ v0.0.28) and timed out — the
    /// "Check FW" flow treats `None` as "older than latest → update".
    FirmwareVersion(Option<String>),
    /// The box's GPS power state, from a GPS_GET_POWER (0x12) reply or a
    /// confirmed GPS_POWER (0x11) round-trip. `on = true` → u-blox receiver
    /// active; `false` → in backup mode to save battery. Never emitted on
    /// legacy firmware (< v0.0.35, no reply) — the toggle stays "unknown".
    GpsPower { on: bool },
    /// Box's calibration blob, from a CAL_GET (0x13) reply or a confirmed
    /// CAL_SET (0x14) round-trip. `Some(blob)` = the box's current 32-byte
    /// blob; the receiver should `calibration::decode` it to drive the local
    /// `AgentConfig` merge (per-field: bits set on the blob win over local,
    /// bits clear leave the local value alone). `None` = legacy firmware
    /// (< v0.0.37) didn't reply → op timed out; the client keeps its local
    /// `AgentConfig` as before, no cross-device sync available.
    Calibration(Option<[u8; 32]>),
    /// User-facing error — display in the log panel.
    Error(String),
    /// One decoded SensorStream snapshot (0.5 Hz). Only emitted while
    /// connected to a PumpLogger firmware that exposes the SensorStream
    /// characteristic; legacy PumpTsueri builds never produce this event.
    Sample(LiveSample),
    /// One decoded BatteryStatus snapshot (dedicated …0200… characteristic).
    /// Only emitted while connected to a PumpLogger firmware that exposes
    /// the BatteryStatus characteristic; legacy PumpTsueri builds never
    /// produce this event.
    Battery(BatterySample),
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
    /// Angular rate. Firmware ≤ v0.0.26 packed centi-dps (÷100 for °/s,
    /// clamped at ±327 dps); firmware v0.0.27+ packs DECI-dps (÷10 for
    /// °/s, full ±500 dps FS). Field name kept for wire-parse stability;
    /// consumers divide by 10 (see the Live tab + `OrientationFilter`).
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

/// One decoded BatteryStatus snapshot (8-byte LE packed layout, DESIGN.md
/// §4). Distinct from `LiveSample` — its flags byte uses bit0=low_batt,
/// bit1=logging (see firmware `Src/battery.c`), so it does NOT share
/// `LiveSample`'s decode.
#[derive(Clone, Copy, Debug)]
pub struct BatterySample {
    /// Pack voltage, millivolts (STC3115).
    pub voltage_mv: u16,
    /// State-of-charge in 0.1 % steps (÷10 for %).
    pub soc_x10: u16,
    /// Current in 100 µA steps, signed (+ charging / − draining;
    /// ÷10000 for A).
    pub current_x100ua: i16,
    /// Firmware low-battery warning (raised at SoC < 10 %).
    pub low_batt: bool,
    /// Box is actively logging to SD.
    pub logging: bool,
}

impl BatterySample {
    /// Decode the fixed 8-byte little-endian BatteryStatus payload.
    /// `None` if the length is wrong. Single-notify only (8 B is well
    /// under any MTU) — no chunk reassembly, unlike SensorStream.
    fn parse(b: &[u8]) -> Option<Self> {
        if b.len() != 8 { return None; }
        Some(Self {
            voltage_mv:     u16::from_le_bytes([b[0], b[1]]),
            soc_x10:        u16::from_le_bytes([b[2], b[3]]),
            current_x100ua: i16::from_le_bytes([b[4], b[5]]),
            low_batt:       b[6] & 0x01 != 0,
            logging:        b[6] & 0x02 != 0,
            // b[7] reserved.
        })
    }
    /// SoC as a 0..1 fraction for a ProgressBar.
    pub fn soc_frac(&self) -> f32 { (self.soc_x10 as f32 / 10.0 / 100.0).clamp(0.0, 1.0) }
    /// SoC as whole percent (rounded).
    pub fn soc_pct(&self) -> u16 { (self.soc_x10 + 5) / 10 }
    /// Voltage in volts.
    pub fn volts(&self) -> f32 { self.voltage_mv as f32 / 1000.0 }
    /// Current in amps (signed: + charging, − draining).
    pub fn amps(&self) -> f32 { self.current_x100ua as f32 / 10_000.0 }
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

    /// Block up to `timeout` for the worker to emit `BleEvent::Disconnected`
    /// (or for the worker channel to close). Any other events that arrive in
    /// the meantime are drained silently — the caller is presumed to be on
    /// the shutdown/relaunch path where the UI can't consume them anyway.
    /// Returns `true` if we observed a clean disconnect, `false` if we timed
    /// out (in which case the box is likely still connected-in-limbo and only
    /// its 90 s peer-gone watchdog will free it).
    ///
    /// Used by `on_exit` in place of a blind sleep: the fixed 250 ms wait was
    /// too short for a healthy `0x0F WithResponse` round-trip on a loaded
    /// CoreBluetooth stack, so app-quit / app-relaunch routinely left the box
    /// stranded — the exact "still no clean disconnect on restart" symptom.
    pub fn await_disconnected(&self, timeout: std::time::Duration) -> bool {
        let deadline = std::time::Instant::now() + timeout;
        loop {
            let remaining = match deadline.checked_duration_since(std::time::Instant::now()) {
                Some(d) if !d.is_zero() => d,
                _ => return false,
            };
            match self.evt_rx.recv_timeout(remaining) {
                Ok(BleEvent::Disconnected) => return true,
                Ok(_) => continue,
                Err(_) => return false, // timeout or worker gone
            }
        }
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

            let central_dead = {
                // Non-blocking sweep of the DeviceDisconnected pump: did
                // CoreBluetooth report OUR peripheral gone? This is the only
                // remote-disconnect signal on macOS (see `central_rx` doc).
                // Drained here (not a select arm) to keep the borrow set
                // simple; the 200 ms watchdog tick bounds the latency.
                let our_id = state
                    .peripheral
                    .as_ref()
                    .map(|p| p.id().to_string());
                let mut hit = false;
                if let (Some(rx), Some(our)) = (state.central_rx.as_mut(), our_id) {
                    while let Ok(gone) = rx.try_recv() {
                        if gone == our {
                            hit = true;
                        }
                    }
                }
                hit
            };
            if central_dead {
                state
                    .lost_link_recover("link lost (CoreBluetooth disconnect event)")
                    .await;
                continue;
            }

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
                            // The peripheral dropped the stream — remote-side
                            // disconnect (Linux/Windows signal; on macOS the
                            // stream never closes and the central_dead sweep
                            // above is the signal instead). ALWAYS recover —
                            // transfer or idle: the old `if was_transfer`
                            // guard made an idle-time close (gap between
                            // queued files) fall through with NO reconnect
                            // and NO Disconnected event — GUI stuck showing
                            // Connected, box stranded (2026-07-02 deadlock).
                            state.lost_link_recover("notification stream closed").await;
                        }
                    }
                }
                _ = &mut watchdog => {
                    if state.tick_watchdog() {
                        // v0.0.62: tick_watchdog returns true ONLY at the READ
                        // last-resort (READ_RIDE_OUT_LIMIT, 200 s) — a shorter
                        // notify-park is now ridden out on the live link (see
                        // tick_watchdog). Reaching here means data never
                        // resumed AND macOS never surfaced a DeviceDisconnected,
                        // so we tear the half-dead link down ourselves (0x0F
                        // best-effort frees the box if the ACL is only app-dead)
                        // and auto-reconnect so the mirror resume continues.
                        // tick_watchdog already emitted ReadAborted (partial
                        // saved) + the error, op = Idle.
                        state.lost_link_recover("READ ride-out limit — reconnecting").await;
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
        /// Byte offset the CURRENT segment was requested at. Advanced by
        /// one segment each time a segment completes and `content` is
        /// drained (v0.0.48). Absolute progress = base + content.len().
        base: u64,
        /// Absolute byte offset where the in-flight capped segment ends —
        /// i.e. `base + this segment's cap`. When `base + content.len()`
        /// reaches it (but is still < `expected`) the segment is flushed
        /// and the next capped READ is issued. See `READ_SEGMENT_BYTES`.
        segment_end: u64,
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
    /// GET_VERSION in flight — waiting for the ASCII firmware-version reply.
    /// Twin of `GettingMode`: demuxed over FileData, single-op-slot, and a
    /// stall just drops to Idle without reconnecting. Difference: on timeout
    /// it emits `FirmwareVersion(None)` (legacy box = unknown → "update"),
    /// where GettingMode stays silent.
    GettingVersion { last_progress: Instant },
    /// GPS_POWER (SET 0x11) or GPS_GET_POWER (GET 0x12) in flight — waiting for
    /// the single-byte reply, demuxed by `is_set`. For a SET the reply is a
    /// status byte (0x00 OK → the box is now in the `on` state); for a GET it's
    /// the current state (1 = on, 0 = off). Twin of `GettingMode`: a stall just
    /// drops to Idle without reconnecting (legacy firmware < v0.0.35 never
    /// replies → the toggle stays "unknown"). `on` carries the requested state
    /// so the SET reply knows what to confirm.
    GpsPowerReq { is_set: bool, on: bool, last_progress: Instant },
    /// CAL_GET (0x13) or CAL_SET (0x14) in flight — waiting for the FileData
    /// reply (a 32-byte blob for GET, a single status byte for SET). Twin of
    /// `GettingMode` / `GpsPowerReq`. Legacy firmware (< v0.0.37) never
    /// replies to either; a stall drops to Idle without reconnecting and
    /// the client falls back to its local `AgentConfig` copy. `is_set`
    /// disambiguates the two demux paths; on a SET, `blob` carries the
    /// payload so it can be re-emitted as `Calibration` (the box's new
    /// authoritative copy) when the status is OK.
    CalibrationReq { is_set: bool, blob: [u8; 32], last_progress: Instant },
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
    /// CentralEvent::DeviceDisconnected ids, pumped from `adapter.events()`.
    /// THE remote-disconnect signal on macOS: the vendored fork's per-
    /// peripheral notification stream NEVER closes on a CoreBluetooth
    /// disconnect (PeripheralEventInternal::Disconnected is ignored and the
    /// worker's own Peripheral clone keeps the broadcast sender alive), so
    /// the notif-None arm is unreachable there and an idle-time drop (gap
    /// between queued files, post-pass) previously went completely
    /// undetected — no reconnect, no Disconnected event, GUI stuck showing
    /// Connected, box stranded in a zombie ACL. The pump filters to
    /// DeviceDisconnected only (discovery spam never queues up).
    central_rx:   Option<tokio_mpsc::UnboundedReceiver<String>>,
    central_pump: Option<tokio::task::JoinHandle<()>>,
    op:           CurrentOp,
    /// SensorStream chunked-mode reassembly buffer.
    stream_asm:   StreamAsm,
    /// True iff the connected peripheral exposed the SensorStream
    /// characteristic — kept so we can emit a one-shot status line
    /// telling the user whether the box's firmware supports live data.
    stream_capable: bool,
    /// True iff the connected peripheral exposed the BatteryStatus
    /// characteristic. Same rationale as `stream_capable`.
    battery_capable: bool,
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
    /// One-shot guard so the "READ notify-parked — riding out" status line is
    /// emitted once per pause, not every 200 ms watchdog tick. Reset when a
    /// READ makes progress (idle drops below `READ_STALL_TIMEOUT`) or a new
    /// READ starts. See `tick_watchdog`.
    read_ride_out_logged: bool,
}

impl WorkerState {
    fn new(evt_tx: Sender<BleEvent>, shared: BleShared) -> Self {
        Self {
            evt_tx,
            adapter: None,
            peripheral: None,
            notif_rx: None,
            notif_pump: None,
            central_rx: None,
            central_pump: None,
            op: CurrentOp::Idle,
            stream_asm: StreamAsm::default(),
            stream_capable: false,
            battery_capable: false,
            last_id: None,
            shared,
            pending_set_mode: None,
            power: None,
            read_ride_out_logged: false,
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

    /// Shared link-lost recovery: teardown (0x0F best-effort inside
    /// disconnect_inner) → auto-reconnect → on give-up release power and
    /// emit Disconnected so the GUI leaves the Connected state. Used by both
    /// drop signals — the notif-stream close (Linux/Windows) and the
    /// CentralEvent::DeviceDisconnected pump (macOS).
    async fn lost_link_recover(&mut self, why: &str) {
        self.emit_err(why.to_string());
        self.disconnect_inner().await;
        if let Some(id) = self.last_id.clone() {
            if self.auto_reconnect(id).await {
                // Reconnected — Connected emitted; main re-lists and the
                // size-diff resumes any unfinished file from the mirror.
                return;
            }
        }
        // Bounded retries exhausted / abort / no id: hand control back to
        // the user. The mirror keeps the resume lossless; let the machine
        // sleep again.
        self.release_power();
        self.emit(BleEvent::Disconnected);
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
        // Central-event pump: forward DeviceDisconnected ids into a channel
        // the worker loop can select on. See the `central_rx` field doc —
        // this is the only remote-disconnect signal macOS delivers.
        match adapter.events().await {
            Ok(mut events) => {
                let (ctx, crx) = tokio_mpsc::unbounded_channel::<String>();
                let pump = tokio::spawn(async move {
                    while let Some(ev) = events.next().await {
                        if let CentralEvent::DeviceDisconnected(pid) = ev {
                            if ctx.send(pid.to_string()).is_err() {
                                break;
                            }
                        }
                    }
                });
                self.central_rx = Some(crx);
                self.central_pump = Some(pump);
            }
            Err(e) => {
                // Non-fatal: Linux/Windows still have the stream-close arm;
                // macOS degrades to the pre-fix behaviour.
                eprintln!("[ble] adapter.events() failed ({e}) — idle-drop detection disabled");
            }
        }
        Some(adapter)
    }

    // ---------------- Command dispatch -------------------------------------

    async fn handle_command(&mut self, cmd: BleCmd) {
        match cmd {
            BleCmd::Scan { known_id } => self.scan(known_id).await,
            BleCmd::Connect(id) => self.connect(id).await,
            BleCmd::Disconnect  => {
                // disconnect_inner sends the 0x0F host-disconnect (ACK-
                // confirmed, bounded) before the local cancel — see there.
                eprintln!("[disconnect] BleCmd::Disconnect — had_link={}", self.peripheral.is_some());
                self.disconnect_inner().await;
                self.release_power();
                // Every request_abort() caller (Disconnect button, START_LOG
                // teardown, on_exit, agent stop) queues this command right
                // after setting the flag — so THIS is where the abort is
                // consumed. Leaving it set poisoned every later Scan
                // (sleep_or_abort returned instantly → 0-ms scan window →
                // "scan saw N, 0 matched" forever) and let auto_reconnect's
                // old entry-clear swallow a user abort that landed during a
                // teardown. Do not clear it anywhere else on entry paths.
                self.clear_abort();
                eprintln!("[disconnect] teardown complete — worker idle, box released");
                self.emit(BleEvent::Disconnected);
            }
            BleCmd::List        => self.list().await,
            BleCmd::Read { name, size, offset } => self.read(name, size, offset).await,
            BleCmd::StopLog     => self.stop_log().await,
            BleCmd::StartLog { duration_seconds } => self.start_log(duration_seconds).await,
            BleCmd::Delete { name } => self.delete(name).await,
            BleCmd::SetLogMode { manual } => self.set_log_mode(manual).await,
            BleCmd::GetLogMode => self.get_log_mode().await,
            BleCmd::GetFirmwareVersion => self.get_firmware_version().await,
            BleCmd::SetTime { epoch_ms } => self.set_time(epoch_ms).await,
            BleCmd::FlashFirmware { bytes } => self.flash_firmware(bytes).await,
            BleCmd::GpsBridge { on } => self.gps_bridge(on).await,
            BleCmd::GpsTx { bytes } => self.gps_tx(bytes).await,
            BleCmd::SetGpsPower { on } => self.set_gps_power(on).await,
            BleCmd::GetGpsPower => self.get_gps_power().await,
            BleCmd::GetCalibration => self.get_calibration().await,
            BleCmd::SetCalibration { blob } => self.set_calibration(blob).await,
        }
    }

    async fn scan(&mut self, known_id: Option<String>) {
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
            // Also match the saved box by peripheral id: macOS answers scans
            // with its persistent name cache, and a stale/empty cache entry
            // (survives reboots) hides a known box from the name filter —
            // "scan saw N, 0 matched" with the box advertising fine.
            let id_ok = known_id.as_deref() == Some(p.id().to_string().as_str());
            if name_ok || id_ok {
                matched += 1;
                self.emit(BleEvent::Discovered {
                    id: p.id().to_string(),
                    name: name.unwrap_or_else(|| format!("{} (saved box)", BOX_NAMES[0])),
                    rssi: props.and_then(|pp| pp.rssi),
                });
            }
        }
        self.emit(BleEvent::Status(format!(
            "scan saw {} peripheral(s), {} matched {:?} or saved id", all_count, matched, BOX_NAMES
        )));
        self.emit(BleEvent::ScanStopped);
    }

    async fn connect(&mut self, id: String) {
        if self.peripheral.is_some() {
            self.emit_err("already connected — disconnect first");
            return;
        }
        eprintln!("[connect] manual Connect requested for {id}");
        match self.connect_core(&id, ConnectMode::Manual).await {
            Ok(()) => {
                eprintln!("[connect] manual Connect OK for {id}");
                self.last_id = Some(id);
                // Hold the OS awake for the whole session (kept across any
                // later auto-reconnect windows; released on Disconnect).
                self.hold_power();
                self.emit(BleEvent::Connected);
            }
            Err(e) => {
                eprintln!("[connect] manual Connect FAILED: {e}");
                self.emit_err(e);
            }
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
        let p = match peripherals.into_iter().find(|x| x.id().to_string() == id) {
            Some(p) => p,
            None => {
                // macOS: after a disconnect CoreBluetooth stops returning the
                // recently-connected box in scan results, so `peripherals()`
                // (fed by scan) won't list it — a plain manual Connect / Mac→Mac
                // reconnect then fails with "peripheral gone" even though the box
                // is re-advertising and the iPhone can see it. Retrieve it by id
                // (btleplug-patched `add_peripheral` → retrievePeripherals-
                // withIdentifiers) to repopulate btleplug's map, then look again.
                // Same scan-suppression fix `auto_reconnect` already uses, now on
                // the manual path too. No-op on Linux/Windows (scan works there).
                #[cfg(target_os = "macos")]
                {
                    let uuid = Uuid::parse_str(id).map_err(|e| format!("bad box id: {e}"))?;
                    let pid: btleplug::platform::PeripheralId = uuid.into();
                    adapter
                        .add_peripheral(&pid)
                        .await
                        .map_err(|e| format!("retrieve-by-id: {e}"))?;
                    self.emit(BleEvent::Status(
                        "connect: box scan-suppressed — retrieved by id (no scan)".into(),
                    ));
                    adapter
                        .peripherals()
                        .await
                        .map_err(|e| format!("peripherals: {e}"))?
                        .into_iter()
                        .find(|x| x.id().to_string() == id)
                        .ok_or_else(|| "peripheral gone — rescan".to_string())?
                }
                #[cfg(not(target_os = "macos"))]
                {
                    return Err("peripheral gone — rescan".into());
                }
            }
        };
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
                        // The pending connect may have completed at the
                        // CoreBluetooth level right at the timeout boundary;
                        // a bare cancel would then strand the box in a
                        // zombie ACL (see polite_drop_link). Check and tear
                        // down politely if so. The check itself MUST be
                        // bounded: the vendored fork's is_connected never
                        // resolves when a late didDisconnect purged the
                        // peripheral map (internal.rs sets no reply on a
                        // map miss) — unbounded, it froze the whole worker.
                        if link_probably_connected(&p).await {
                            eprintln!("[connect] timeout raced a completed connect — polite teardown");
                            polite_drop_link(&p).await;
                        } else {
                            drop_link(&p).await;
                        }
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
                        // The ONLY cancel path in reconnect mode. The pending
                        // connect may have JUST completed (race with the
                        // user's Disconnect click / budget expiry) — a bare
                        // cancel would strand the box in a zombie ACL no
                        // central can reach (Peter's iPhone+Mac total block,
                        // 2026-07-02). Tear down politely (0x0F) if so.
                        // Bounded probe — see link_probably_connected.
                        if link_probably_connected(&p).await {
                            eprintln!("[reconnect] abort raced a completed connect — polite teardown");
                            polite_drop_link(&p).await;
                        } else {
                            drop_link(&p).await;
                        }
                        return Err(match w {
                            ConnectWait::Aborted => "connect: aborted".into(),
                            ConnectWait::Demoted => "connect: keep-synced off".into(),
                            ConnectWait::Budget  => "connect: budget".into(),
                        });
                    }
                }
            }
        }
        // From here on the link IS established — every failure teardown must
        // be polite (0x0F first) or the box is left in the macOS zombie-ACL
        // limbo (see polite_drop_link).
        match tokio::time::timeout(LINK_OP_TIMEOUT, p.discover_services()).await {
            Ok(Ok(())) => {}
            Ok(Err(e)) => {
                polite_drop_link(&p).await;
                return Err(format!("discover_services: {e}"));
            }
            Err(_) => {
                polite_drop_link(&p).await;
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
                polite_drop_link(&p).await;
                return Err(format!("subscribe FileData: {e}"));
            }
            Err(_) => {
                polite_drop_link(&p).await;
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
        // BatteryStatus is optional too — subscribe if present so the box's
        // ~1/min notifies drive the meter. Never fail the connect on absence
        // (legacy firmware). The one-shot READ that seeds an immediate value
        // runs ONLY on a manual connect: on `auto_reconnect` (a silent mid-
        // sync recovery) the reconnect path must stay as lean as it was
        // before the meter — an extra GATT read there just adds a round-trip
        // (up to LINK_OP_TIMEOUT of latency on a marginal macOS link) to a
        // recovery that needs to be fast, and the value is already on screen
        // from before the drop; the notify repopulates it within a minute.
        let seed_read = matches!(mode, ConnectMode::Manual);
        self.battery_capable = false;
        if let Some(batt_char) = chars.iter().find(|c| c.uuid == BATTERY_UUID).cloned() {
            match tokio::time::timeout(LINK_OP_TIMEOUT, p.subscribe(&batt_char)).await {
                Ok(Ok(())) => {
                    self.battery_capable = true;
                    self.emit(BleEvent::Status("BatteryStatus subscribed".into()));
                    // One-shot read for an immediate value (manual connect
                    // only — see above). Bounded like every GATT step here;
                    // non-fatal on error/timeout.
                    if seed_read {
                        if let Ok(Ok(v)) =
                            tokio::time::timeout(LINK_OP_TIMEOUT, p.read(&batt_char)).await
                        {
                            if let Some(b) = BatterySample::parse(&v) {
                                self.emit(BleEvent::Battery(b));
                            }
                        }
                    }
                }
                Ok(Err(e)) => {
                    self.emit(BleEvent::Status(format!(
                        "BatteryStatus subscribe failed ({e}) — no live battery meter"
                    )));
                }
                Err(_) => {
                    self.emit(BleEvent::Status(
                        "BatteryStatus subscribe timed out — no live battery meter".into()
                    ));
                }
            }
        } else {
            self.emit(BleEvent::Status(
                "BatteryStatus characteristic not advertised — legacy firmware, no battery meter".into()
            ));
        }
        let stream: NotifStream = match tokio::time::timeout(
            LINK_OP_TIMEOUT, p.notifications()
        ).await {
            Ok(Ok(s)) => Box::pin(s),
            Ok(Err(e)) => {
                polite_drop_link(&p).await;
                return Err(format!("notifications: {e}"));
            }
            Err(_) => {
                polite_drop_link(&p).await;
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
        // Drop any DeviceDisconnected events queued for the PREVIOUS link of
        // this same box: without this drain, a late didDisconnect from the
        // drop we just recovered from would match our id on the next sweep
        // and instantly tear the fresh link down again.
        if let Some(rx) = self.central_rx.as_mut() {
            while rx.try_recv().is_ok() {}
        }
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
        // NOTE: no clear_abort() here. Every request_abort() is paired with
        // a queued BleCmd::Disconnect whose handler consumes the flag, so a
        // "stale" abort cannot exist at entry — but a FRESH abort can: the
        // user clicks Disconnect while we're still inside the teardown that
        // precedes this call. The old entry-clear silently swallowed exactly
        // that click and looped an unbounded Auto-Mode reconnect against the
        // user's will (the queued Disconnect starving behind it).
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
            CurrentOp::GettingVersion { .. } => {
                // The link dropped mid-query — report "unknown" so the
                // "Check FW" flow doesn't hang waiting for a reply that
                // will never come (the Disconnected handler also resets it).
                self.emit(BleEvent::FirmwareVersion(None));
            }
            CurrentOp::GpsPowerReq { .. } => {
                // The link dropped mid GPS_POWER/GPS_GET_POWER — nothing to
                // report; the Disconnected handler clears ble_gps_power, so the
                // toggle re-queries on the next Connect (mirrors GettingMode).
            }
            CurrentOp::CalibrationReq { .. } => {
                // Link dropped mid CAL_GET/CAL_SET — the client keeps its local
                // AgentConfig unchanged. The next Connect re-fetches via
                // GET_CAL and any queued SET (from a local calibration change
                // that raced this disconnect) gets sent then. Nothing to
                // report — a status banner about a mid-teardown calibration
                // request would just be noise.
            }
            CurrentOp::Flashing { .. } => {
                self.emit(BleEvent::FlashError {
                    msg: "firmware upload aborted by disconnect".into(),
                });
            }
            CurrentOp::Idle => {}
        }
        // macOS: ask the box to terminate the link from ITS side (0x0F →
        // ble_recover_lost_peer → real HCI_Disconnect + re-advertise) before
        // we cancel locally. cancelPeripheralConnection only detaches the
        // app's view; bluetoothd can keep the ACL alive with LL keepalives,
        // leaving the box connected-in-limbo and invisible to EVERY central
        // (iPhone and Mac) until its 90 s peer-gone watchdog fires. This runs
        // on ALL teardown paths now, not just the Disconnect button: on the
        // drop-recovery paths the link is often only *app-dead* (silent macOS
        // delivery stop) while the ACL still stands — exactly the zombie case
        // where the write still reaches the box and frees it. On a truly dead
        // link the write fails fast / times out (bounded 3 s) — cheap.
        // ACK-confirmed WithResponse: a write-without-response is resolved
        // instantly by btleplug and CoreBluetooth drops the still-queued
        // frame on cancel (see write_cmd_confirmed).
        if self.peripheral.is_some() {
            match self.write_cmd_confirmed(&[OP_DISCONNECT]).await {
                Ok(()) => {
                    eprintln!("[teardown] 0x0F confirmed — box tears down + re-advertises");
                    self.emit(BleEvent::Status(
                        "sent host-disconnect (0x0F) — box will re-advertise".into()));
                }
                Err(e) => {
                    eprintln!("[teardown] 0x0F not delivered ({e}) — local cancel only");
                }
            }
        }
        // Stop the notify pump (drops its stream) before tearing the link
        // down, then clear the receiver.
        if let Some(pump) = self.notif_pump.take() {
            pump.abort();
        }
        self.notif_rx        = None;
        self.stream_asm      = StreamAsm::default();
        self.stream_capable  = false;
        self.battery_capable = false;
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
        // come back over FileData. Bounded: on a peripheral that a late
        // didDisconnect purged from the vendored fork's CB-thread map, the
        // write's reply future is never resolved (write_value silently sets
        // no reply on a map miss) — unbounded, one LIST/SET_TIME issued in
        // that window froze the worker forever.
        match tokio::time::timeout(
            LINK_OP_TIMEOUT,
            p.write(&cmd_char, payload, WriteType::WithoutResponse),
        )
        .await
        {
            Ok(r)  => r.map_err(|e| format!("write FileCmd: {e}")),
            Err(_) => Err("write FileCmd: timed out (link dead?)".into()),
        }
    }

    /// Write a FileCmd opcode with an ATT **write-response** and wait for the
    /// ACK. Needed for OP_DISCONNECT: a write-without-response is queued by
    /// CoreBluetooth and resolved instantly by btleplug (it never waits for
    /// peripheralIsReadyToSendWriteWithoutResponse), so a `cancelPeripheral-
    /// Connection` fired shortly after can drop the still-untransmitted frame
    /// and the box never sees the opcode. WithResponse resolves only on the
    /// ATT ACK, guaranteeing the box received it before we tear the link down.
    /// Bounded by a short timeout: the firmware sends the ATT response before
    /// it acts on the opcode, but if the link is already flaky we don't want to
    /// hang the disconnect — we cancel locally either way.
    async fn write_cmd_confirmed(&self, payload: &[u8]) -> Result<(), String> {
        let Some(p) = self.peripheral.as_ref() else {
            return Err("not connected".into());
        };
        let chars = p.characteristics();
        let cmd_char = chars.iter().find(|c| c.uuid == FILECMD_UUID)
            .cloned()
            .ok_or_else(|| "FileCmd not discovered".to_string())?;
        match tokio::time::timeout(
            HOST_DISCONNECT_ACK_TIMEOUT,
            p.write(&cmd_char, payload, WriteType::WithResponse),
        )
        .await
        {
            Ok(r)  => r.map_err(|e| format!("write FileCmd (confirmed): {e}")),
            Err(_) => Err("write FileCmd (confirmed): ATT ACK timed out".into()),
        }
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
        // Opcode payload: 0x02 + name + NUL + 4-byte LE start offset +
        // 4-byte LE stream-length cap (firmware v0.0.24+). The firmware
        // seeks to `offset` and streams EXACTLY `size - offset` bytes.
        // Without the cap it streamed to EOF-at-read-time — but the ACTIVE
        // session file keeps growing, so the box over-delivered past our
        // LIST-size expectation and the excess bytes bled into the NEXT
        // queued download's content (GPS057/BAT057 full of SENS rows,
        // 02.07.2026). Legacy firmware ignores the extra 4 bytes (it only
        // parses name + offset) and falls back to stream-to-EOF.
        // offset/len are u32 on the wire — SD files are well under 4 GiB.
        // Only request ONE segment now (v0.0.48): cap = min(remaining,
        // READ_SEGMENT_BYTES). handle_notification issues the next segment
        // as each one completes, so the box's READ FSM returns to IDLE
        // between segments and its 45 s stall never accumulates over a big
        // file. `expected` stays the WHOLE file size (the terminal
        // ReadDone still fires at true EOF); `segment_end` tracks this
        // segment's boundary.
        let seg_cap = size.saturating_sub(offset).min(READ_SEGMENT_BYTES);
        let mut payload = Vec::with_capacity(name.len() + 10);
        payload.push(0x02);
        payload.extend_from_slice(name.as_bytes());
        payload.push(0x00);
        payload.extend_from_slice(&(offset as u32).to_le_bytes());
        payload.extend_from_slice(&(seg_cap as u32).to_le_bytes());
        if let Err(e) = self.write_cmd(&payload).await {
            self.emit_err(e);
            return;
        }
        self.op = CurrentOp::Reading {
            name: name.clone(),
            expected: size,
            base: offset,
            segment_end: offset + seg_cap,
            // One segment at a time; drained on each ReadChunk flush.
            content: Vec::with_capacity(READ_SEGMENT_BYTES as usize),
            last_emit: offset,
            last_progress: Instant::now(),
            first_packet: true,
        };
        self.read_ride_out_logged = false;
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
        /* Discard the box's 1-byte OK reply NOW, while op is still Idle.
           The biased select processes queued commands before queued
           notifies, so without this drain the 0x00 reply is handled only
           AFTER the follow-up LIST has set op = Listing — and the byte
           then glues onto the first LIST row's name ("\0ERRLOG.LOG").
           That name is invisible in the UI but every READ of it gets
           BAD_REQUEST from the box (the embedded NUL truncates the name
           to zero length), so the first-listed file silently refused to
           download (Peter: "errorlog will nicht über ble runterladen"). */
        if let Some(rx) = self.notif_rx.as_mut() {
            let mut drained = 0u32;
            while let Ok(n) = rx.try_recv() {
                if n.uuid == FILEDATA_UUID { drained += 1; }
            }
            if drained > 0 {
                eprintln!("[set_time] drained {drained} stale FileData notif(s) (SET_TIME reply)");
            }
        }
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

    /// GET_VERSION (0x10). Box replies with one FileData notify carrying the
    /// ASCII firmware version; handled in the GettingVersion op arm. Exact
    /// twin of `get_log_mode`: self-guards on `op == Idle` (so it can't
    /// trample an in-flight LIST/READ — the reply is demuxed by `op`), writes
    /// the single opcode byte, and parks the op. Legacy firmware (≤ v0.0.28)
    /// never replies → the GettingVersion watchdog emits FirmwareVersion(None).
    async fn get_firmware_version(&mut self) {
        if !matches!(self.op, CurrentOp::Idle) {
            // Don't trample an in-flight LIST/READ — GET_VERSION is
            // best-effort; the caller re-tries when idle.
            return;
        }
        if self.peripheral.is_none() {
            self.emit_err("not connected");
            return;
        }
        if let Err(e) = self.write_cmd(&[0x10]).await {
            self.emit_err(e);
            return;
        }
        self.op = CurrentOp::GettingVersion { last_progress: Instant::now() };
    }

    /// GPS_POWER (0x11 `<u8>`). Turns the box's u-blox on/off (persisted on the
    /// box) to save battery when GPS is faulty/unused. Unlike SET_MODE (which is
    /// fire-and-forget, since re-reading would consume its OK byte), we DO track
    /// the reply: the box answers with one status byte (0x00 OK) and the
    /// `GpsPowerReq` op demuxes it → `BleEvent::GpsPower { on }` on OK. The GUI
    /// reflects the target optimistically the moment it's clicked; this reply
    /// reconciles it. Needs an idle worker (the reply is demuxed by `op`); a
    /// click during a LIST/READ is rejected with "another op is in flight".
    async fn set_gps_power(&mut self, on: bool) {
        if self.peripheral.is_none() {
            self.emit_err("not connected");
            return;
        }
        if !matches!(self.op, CurrentOp::Idle) {
            self.emit_err("another op is in flight — wait or Disconnect");
            return;
        }
        if let Err(e) = self.write_cmd(&[0x11, on as u8]).await {
            self.emit_err(e);
            return;
        }
        self.emit(BleEvent::Status(format!(
            "GPS_POWER {} sent",
            if on { "on" } else { "off" }
        )));
        self.op = CurrentOp::GpsPowerReq { is_set: true, on, last_progress: Instant::now() };
    }

    /// GPS_GET_POWER (0x12). Box replies with one byte over FileData
    /// (1 = on, 0 = off); handled in the `GpsPowerReq` op arm. Exact twin of
    /// `get_log_mode`: self-guards on `op == Idle` (the reply is demuxed by
    /// `op`, so it must not race a LIST/READ), writes the single opcode byte,
    /// and parks the op. Legacy firmware (< v0.0.35) never replies → the
    /// `GpsPowerReq` watchdog drops to Idle and the toggle stays "unknown".
    async fn get_gps_power(&mut self) {
        if !matches!(self.op, CurrentOp::Idle) {
            // Don't trample an in-flight LIST/READ — GPS_GET_POWER is
            // best-effort; the caller re-tries when idle.
            return;
        }
        if self.peripheral.is_none() {
            self.emit_err("not connected");
            return;
        }
        if let Err(e) = self.write_cmd(&[0x12]).await {
            self.emit_err(e);
            return;
        }
        self.op = CurrentOp::GpsPowerReq { is_set: false, on: false, last_progress: Instant::now() };
    }

    /// CAL_GET (0x13). Fetch the box's persisted calibration blob so a
    /// "Zero here" / nosePlusY set on ANY host is visible to this one on
    /// the next connect. Firmware v0.0.37+; legacy silently ignores it →
    /// `CalibrationReq` op times out and emits `Calibration(None)`. Best-
    /// effort like `get_gps_power`: self-guards on `op == Idle`, never
    /// trampling a LIST/READ; the caller re-queues after the current op
    /// completes.
    async fn get_calibration(&mut self) {
        if !matches!(self.op, CurrentOp::Idle) {
            return;
        }
        if self.peripheral.is_none() {
            self.emit_err("not connected");
            return;
        }
        if let Err(e) = self.write_cmd(&[0x13]).await {
            self.emit_err(e);
            return;
        }
        self.op = CurrentOp::CalibrationReq {
            is_set: false,
            blob: [0u8; 32],
            last_progress: Instant::now(),
        };
    }

    /// CAL_SET (0x14 + 32-byte blob). Push the per-field-encoded blob
    /// (see `calibration::encode`) to the box for merge into `CAL.CFG`.
    /// Box replies one status byte; the `CalibrationReq` op demuxes it
    /// and, on OK, re-emits `Calibration(Some(blob))` so the client can
    /// mirror the just-pushed values as authoritative without a second
    /// GET roundtrip. Rejected while another op is in flight — same as
    /// `set_gps_power`.
    async fn set_calibration(&mut self, blob: [u8; 32]) {
        if self.peripheral.is_none() {
            self.emit_err("not connected");
            return;
        }
        if !matches!(self.op, CurrentOp::Idle) {
            self.emit_err("another op is in flight — wait or Disconnect");
            return;
        }
        let mut payload = Vec::with_capacity(1 + blob.len());
        payload.push(0x14);
        payload.extend_from_slice(&blob);
        if let Err(e) = self.write_cmd(&payload).await {
            self.emit_err(e);
            return;
        }
        self.emit(BleEvent::Status(format!(
            "CAL_SET sent (mask=0x{:02X})",
            blob[1]
        )));
        self.op = CurrentOp::CalibrationReq {
            is_set: true,
            blob,
            last_progress: Instant::now(),
        };
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
                    if n.uuid == BATTERY_UUID {
                        // Battery notify landed mid-flash — decode it so the
                        // meter keeps ticking, then keep waiting.
                        self.handle_battery_notification(&n.value);
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
        // BatteryStatus notifies are also concurrent + unsolicited and
        // ride their own characteristic — decode on their own path before
        // touching `op` (same rule as SensorStream), so a once/min battery
        // notify can never collide with an in-flight LIST/READ/DELETE/FLASH.
        if n.uuid == BATTERY_UUID {
            self.handle_battery_notification(&n.value);
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

        // If a READ segment completes mid-file, the request for the NEXT
        // capped segment is stashed here and sent after the match (so the
        // `&mut op` borrow is released before the `&self` write_cmd).
        let mut next_read: Option<Vec<u8>> = None;

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
            CurrentOp::Reading { name, expected, base, segment_end, content, last_emit, last_progress, first_packet } => {
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

                // A lone 0xB0 BUSY on a NON-first packet is never file data
                // (the box streams multi-byte chunks, and CSV/log bytes are
                // < 0x80). It's a spurious reply to a segment-continuation
                // READ that either raced the box's FSM or — the important
                // case — hit a LEGACY box (firmware < v0.0.24) that ignored
                // the `cap` and is still streaming the previous to-EOF read.
                // Appending it would corrupt the mirror by one byte per
                // segment. Drop it and stop segmenting (segment_end =
                // expected): ride the single to-EOF stream out, which still
                // delivers every real byte in order — `base`-tracking
                // assembles the file and the terminal ReadDone fires at EOF.
                // This makes segmenting safe on cap-ignoring firmware.
                if n.value.len() == 1 && n.value[0] == 0xB0 {
                    *segment_end = *expected;
                } else {
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

                    // Segment boundary (v0.0.48): this capped segment
                    // finished but the file has more. Flush its bytes to the
                    // mirror now (persist progress + keep worker RAM to one
                    // segment), drain `content`, advance `base`, and queue
                    // the next capped READ. The box's READ FSM has returned
                    // to IDLE, so it accepts the new READ and its 45 s stall
                    // clock resets.
                    if done >= *segment_end {
                        let seg_bytes = std::mem::take(content);
                        let seg_base  = *base;
                        *base += seg_bytes.len() as u64;   // == done
                        self.emit(BleEvent::ReadChunk {
                            name: name.clone(),
                            content: seg_bytes,
                            base: seg_base,
                        });
                        let next_cap = (*expected - *base).min(READ_SEGMENT_BYTES);
                        *segment_end = *base + next_cap;
                        let mut p = Vec::with_capacity(name.len() + 10);
                        p.push(0x02);
                        p.extend_from_slice(name.as_bytes());
                        p.push(0x00);
                        p.extend_from_slice(&(*base as u32).to_le_bytes());
                        p.extend_from_slice(&(next_cap as u32).to_le_bytes());
                        next_read = Some(p);
                    }
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
            CurrentOp::GettingVersion { .. } => {
                /* Firmware replies with the ASCII version string (no NUL),
                   e.g. "0.0.29". Parse as UTF-8 and trim; a lossy/empty
                   decode yields None so a garbled reply reads as "unknown"
                   (→ update) rather than a bogus version. */
                let v = String::from_utf8_lossy(&n.value).trim().to_string();
                let parsed = if v.is_empty() { None } else { Some(v) };
                self.emit(BleEvent::FirmwareVersion(parsed));
                self.op = CurrentOp::Idle;
                return;
            }
            CurrentOp::GpsPowerReq { is_set, on, .. } => {
                /* One-byte reply, demuxed by `is_set`:
                   - SET (0x11): a status byte. 0x00 OK ⇒ the box is now in the
                     requested `on` state → emit GpsPower { on }. Any other byte
                     is an error (surfaced, toggle left as-is).
                   - GET (0x12): the current state (1 = on, 0 = off) → emit
                     GpsPower { on: b != 0 }. */
                let (is_set, requested) = (*is_set, *on);
                if let Some(&b) = n.value.first() {
                    if is_set {
                        if b == 0x00 {
                            self.emit(BleEvent::GpsPower { on: requested });
                            self.emit(BleEvent::Status(format!(
                                "GPS turned {}",
                                if requested { "on" } else { "off" }
                            )));
                        } else {
                            let msg = match b {
                                0xB0 => "BUSY (logging in progress, send STOP_LOG first)",
                                0xE1 => "NOT_FOUND",
                                0xE2 => "IO_ERROR",
                                0xE3 => "BAD_REQUEST",
                                _    => "unknown error",
                            };
                            self.emit_err(format!("GPS_POWER: {msg} (0x{b:02X})"));
                        }
                    } else {
                        self.emit(BleEvent::GpsPower { on: b != 0 });
                    }
                }
                self.op = CurrentOp::Idle;
                return;
            }
            CurrentOp::CalibrationReq { is_set, blob, .. } => {
                /* Demuxed by `is_set`:
                   - GET (0x13): 32-byte blob → emit Calibration(Some).
                     Anything shorter/longer → ignore + drop to Idle so a
                     stray notify doesn't lock the toggle.
                   - SET (0x14): 1-byte status. 0x00 OK ⇒ re-emit
                     Calibration(Some(blob)) with the just-pushed blob so
                     the receiver mirrors it as authoritative without a
                     second GET roundtrip. Anything else surfaces as an
                     error and the client keeps its optimistic local
                     update (the state on the box is unchanged). */
                let (is_set, sent) = (*is_set, *blob);
                if is_set {
                    if let Some(&b) = n.value.first() {
                        if b == 0x00 {
                            self.emit(BleEvent::Calibration(Some(sent)));
                            self.emit(BleEvent::Status(
                                format!("CAL_SET OK (mask=0x{:02X})", sent[1])
                            ));
                        } else {
                            let msg = match b {
                                0xE2 => "IO_ERROR (SD write failed)",
                                0xE3 => "BAD_REQUEST (wrong-size or wrong-version blob)",
                                _    => "unknown error",
                            };
                            self.emit_err(format!("CAL_SET: {msg} (0x{b:02X})"));
                        }
                    }
                } else {
                    if n.value.len() == 32 {
                        let mut out = [0u8; 32];
                        out.copy_from_slice(&n.value);
                        self.emit(BleEvent::Calibration(Some(out)));
                    } else {
                        // A firmware bug or wire desync — treat as if
                        // the timeout fired (client falls back to local
                        // AgentConfig).
                        self.emit(BleEvent::Calibration(None));
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

        // A READ segment completed mid-file: request the next one now that
        // the `&mut op` borrow is gone. On write failure the link is dying
        // — leave `op` as Reading (restored above, with the flushed
        // segments already on the mirror) so the READ_STALL watchdog /
        // central-disconnect sweep tears down + reconnects, and the mirror
        // length drives a clean byte-resume.
        if let Some(p) = next_read {
            if let Err(e) = self.write_cmd(&p).await {
                self.emit_err(format!("READ next segment failed: {e}"));
            }
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

    /// Decode + emit an 8-byte BatteryStatus notify. Fixed-length,
    /// single-notify — no reassembly (unlike SensorStream). Malformed
    /// frames are dropped silently.
    fn handle_battery_notification(&mut self, bytes: &[u8]) {
        if let Some(b) = BatterySample::parse(bytes) {
            self.emit(BleEvent::Battery(b));
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

        // READ ride-out (v0.0.62). A notify-parked READ is NOT torn down on
        // silence. macOS routinely parks the notify drain for tens of seconds
        // while keeping the LL link fully alive; the box (fw v0.0.39) holds the
        // in-flight segment and resumes the instant macOS drains again, so the
        // SAME stream just continues — no teardown, no reconnect, no
        // re-adv=211. A REAL remote disconnect is caught by the
        // DeviceDisconnected sweep (`central_rx` → `lost_link_recover`), NOT by
        // silence. We only *note* the pause once (for visibility) and keep the
        // Reading op alive; only `READ_RIDE_OUT_LIMIT` (200 s, above the box's
        // 180 s) falls through to the teardown+reconnect below, for the
        // pathological "link up but data dead AND box never reports a
        // disconnect". `read_idle` is Copy so no self.op borrow is held.
        let read_idle: Option<Duration> =
            if let CurrentOp::Reading { last_progress, .. } = &self.op {
                Some(now.duration_since(*last_progress))
            } else {
                None
            };
        if let Some(idle) = read_idle {
            if idle < READ_STALL_TIMEOUT {
                self.read_ride_out_logged = false; // fresh / progressing
                return false;
            }
            if idle < READ_RIDE_OUT_LIMIT {
                if !self.read_ride_out_logged {
                    self.read_ride_out_logged = true;
                    let name = if let CurrentOp::Reading { name, .. } = &self.op {
                        name.clone()
                    } else {
                        String::new()
                    };
                    self.emit(BleEvent::Status(format!(
                        "READ {name}: macOS paused the notify drain — riding out on the live link (no reconnect)"
                    )));
                }
                return false;
            }
            // idle >= READ_RIDE_OUT_LIMIT → last resort: fall through to the
            // teardown below (the box's 180 s should already have recovered).
        }

        let stale = match &self.op {
            CurrentOp::Listing  { last_progress, .. } => now.duration_since(*last_progress) > OP_IDLE_TIMEOUT,
            CurrentOp::Reading  { last_progress, .. } => now.duration_since(*last_progress) > READ_RIDE_OUT_LIMIT,
            CurrentOp::Deleting { last_progress, .. } => now.duration_since(*last_progress) > OP_IDLE_TIMEOUT,
            CurrentOp::GettingMode { last_progress } => now.duration_since(*last_progress) > OP_IDLE_TIMEOUT,
            CurrentOp::GettingVersion { last_progress } => now.duration_since(*last_progress) > OP_IDLE_TIMEOUT,
            CurrentOp::GpsPowerReq { last_progress, .. } => now.duration_since(*last_progress) > OP_IDLE_TIMEOUT,
            CurrentOp::CalibrationReq { last_progress, .. } => now.duration_since(*last_progress) > OP_IDLE_TIMEOUT,
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
                // Last resort only: we rode the pause out for READ_RIDE_OUT_LIMIT
                // (200 s) on a still-"connected" link and data never resumed —
                // the box's own 180 s recovery didn't surface a DeviceDisconnected
                // either. Give up on the live link and reconnect + byte-resume.
                self.emit_err(format!(
                    "READ {name} parked at {got}/{expected} B for {}s — giving up on the live link, reconnecting",
                    READ_RIDE_OUT_LIMIT.as_secs()
                ));
                true // wedged READ → caller reconnects + resumes
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
            CurrentOp::GettingVersion { .. } => {
                // Legacy firmware (≤ v0.0.28) never replies to GET_VERSION.
                // Unlike GettingMode we DO emit a terminal event — None, so
                // the "Check FW" flow learns the box is legacy (→ update)
                // instead of waiting forever. Never reconnect (the link is
                // fine); op already dropped to Idle above.
                self.emit(BleEvent::FirmwareVersion(None));
                false
            }
            CurrentOp::GpsPowerReq { .. } => {
                // Legacy firmware (< v0.0.35) never replies to GPS_POWER /
                // GPS_GET_POWER. Silently drop to Idle (toggle stays
                // "unknown"); never reconnect — the link is fine.
                false
            }
            CurrentOp::CalibrationReq { is_set, .. } => {
                // Legacy firmware (< v0.0.37) never replies to CAL_GET / CAL_SET.
                // Drop to Idle without reconnecting; on a GET-side timeout emit
                // Calibration(None) so the client stops waiting and falls back
                // to its local AgentConfig. On a SET-side timeout stay silent —
                // the client keeps its optimistic local update, and re-sending
                // it later is a normal path (not an error to surface).
                if !is_set {
                    self.emit(BleEvent::Calibration(None));
                }
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

/// Bounded "is this link (still/now) established?" probe. NEVER call the
/// vendored fork's `is_connected` unbounded: on a map-purged peripheral (a
/// late `didDisconnectPeripheral` removed it from the CB-thread map) the
/// fork silently sets no reply and the future pends forever — on our single
/// worker thread that froze the entire BLE backend until app restart.
/// Timeout/err ⇒ treat as "not connected" and fall through to the plain
/// bounded cancel.
async fn link_probably_connected(p: &Peripheral) -> bool {
    matches!(
        tokio::time::timeout(Duration::from_secs(3), p.is_connected()).await,
        Ok(Ok(true))
    )
}

/// Tear down a link that IS (or may just have become) established, politely:
/// best-effort 0x0F host-disconnect first, then the local cancel. Needed on
/// every path that abandons a *connected* peripheral outside the normal
/// `disconnect_inner` flow — most critically the pending-connect abort race:
/// the user clicks Disconnect (or a budget/demote fires) while `p.connect()`
/// is pending, and the connect completes at the CoreBluetooth level a beat
/// before we cancel. A bare `drop_link` then only detaches the app's view;
/// bluetoothd keeps the fresh ACL alive and the box is stranded connected-in-
/// limbo with no app owning it — invisible to iPhone AND Mac until its 90 s
/// peer-gone watchdog (or a Mac BT toggle / box power-cycle).
///
/// On the raced-connect path the app-side characteristics cache is EMPTY:
/// it is populated only by `Peripheral::connect()` consuming its
/// `Connected(services)` reply, and that future was dropped/timed out
/// (`discover_services` is a no-op on the CoreBluetooth backend, so calling
/// it here cannot help). The CB-thread map, however, IS populated by the
/// delegate's auto-discovery on didConnect — and `write` looks the target up
/// by (service_uuid, char_uuid) in THAT map. So when the cache misses we
/// write through a synthetic Characteristic instead. The write is retried
/// briefly: delegate discovery may still be in flight right after the raced
/// connect. Every step bounded; only these rare abort/failure paths pay it.
async fn polite_drop_link(p: &Peripheral) {
    let cmd_char = p
        .characteristics()
        .iter()
        .find(|c| c.uuid == FILECMD_UUID)
        .cloned()
        .unwrap_or_else(|| Characteristic {
            uuid: FILECMD_UUID,
            service_uuid: FSYNC_SVC_UUID,
            properties: CharPropFlags::WRITE,
            descriptors: Default::default(),
        });
    let mut sent = false;
    for attempt in 0..2u8 {
        match tokio::time::timeout(
            HOST_DISCONNECT_ACK_TIMEOUT,
            p.write(&cmd_char, &[OP_DISCONNECT], WriteType::WithResponse),
        )
        .await
        {
            Ok(Ok(())) => {
                eprintln!("[teardown] polite 0x0F confirmed — box re-advertises");
                sent = true;
                break;
            }
            other => eprintln!("[teardown] polite 0x0F attempt {attempt} failed ({other:?})"),
        }
    }
    if !sent {
        eprintln!("[teardown] polite 0x0F undeliverable — plain cancel (box may need its own watchdog)");
    }
    drop_link(p).await;
}

fn parse_list_row(line: &[u8]) -> Option<(String, u64)> {
    let s = std::str::from_utf8(line).ok()?;
    let comma = s.rfind(',')?;
    // Strip control bytes / stray whitespace from the name. A 1-byte status
    // reply from a preceding op (SET_TIME OK = 0x00, GET_MODE = 0x00/0x01)
    // that reaches the demux after `op` flipped to Listing glues itself onto
    // the first row — the polluted name renders invisibly in the UI but the
    // box rejects every READ of it (embedded NUL → zero-length name →
    // BAD_REQUEST). Filenames are plain 8.3 ASCII, so dropping controls and
    // trimming whitespace can never mangle a legitimate name.
    let name: String = s[..comma]
        .chars()
        .filter(|c| !c.is_control())
        .collect::<String>()
        .trim()
        .to_string();
    if name.is_empty() {
        return None;
    }
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
