//! MovementLogger — drag-and-drop GUI front-end for `stbox-viz animate`.
//!
//! Drop a sensor CSV (and optionally its companion `_gps.csv`, a camera
//! `.mov`/`.mp4`, and a `.stl` board mesh) onto the window, fill in the
//! handful of optional fields, hit Generate, and the GUI shells out to
//! the bundled `stbox-viz` CLI to produce a side-by-side MOV.
//!
//! The work is intentionally delegated to the existing CLI binary
//! rather than re-linking plotters + rustfft + gif into the GUI. Both
//! binaries ship in the same release archive (and the same .app bundle
//! on macOS, next to MovementLogger inside Contents/MacOS/), so the
//! GUI finds the CLI by looking next to its own executable first and
//! falling back to PATH.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod agent;
mod agent_config;
mod autostart;
mod ble;
mod ble_debug;
mod coord;
mod errlog_check;
mod installer;
mod power;
mod sync_core;
mod sync_db;
mod update;

use eframe::egui;
use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    mpsc, Arc, Mutex,
};
use std::thread;

use ble::{BatterySample, BleBackend, BleCmd, LiveSample};
use sync_core::{
    default_save_base, is_sensor_data_name, push_log, resolve_save_dir, BleFile, BleState,
    SyncCore, SyncHost,
};

/// Bundled-into-binary 512×512 PNG. The build pipeline already
/// generates this file at the canonical asset path; including it as
/// bytes means the standalone binary doesn't depend on any external
/// file at run time. egui rasterises on demand for whichever pixel
/// size the layout asks for.
const ICON_PNG: &[u8] = include_bytes!("../assets/icon.png");

// ---------------------------------------------------------------------------
//  App state
// ---------------------------------------------------------------------------

/// Top-level navigation. Live first to match the panel order Peter
/// asked for (Live → Sync → Replay); Android/iOS apps will grow a Live
/// tab in the same slot when their BLE support catches up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Tab {
    Live,
    Sync,
    Replay,
    /// Diagnostics: BLE probe (headless --ble-debug as a child) + the
    /// GPS/antenna survey. One tab so every "send me the output" tool
    /// lives in the same place.
    Debug,
    /// Box firmware: manual `.bin` upload + one-click GitHub auto-update
    /// ("Check FW"), plus the flash progress/result.
    Firmware,
}

impl Default for Tab {
    fn default() -> Self { Tab::Live }
}

/// Transport for the GPS Debug survey. **USB** spawns the `gps-debug`
/// sidecar against a serial port (the long-standing path). **BLE** runs the
/// same UBX survey in-process, tunnelling the u-blox UART through the box's
/// existing BLE link (firmware GPS-bridge opcode) — no cable, no opening the
/// box. Defaults to USB so the proven path is the landing default.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
enum GpsTransport {
    #[default]
    Usb,
    Ble,
}

/// Number of recent SensorStream samples held for the Live tab's
/// running time-series chart. At the 0.5 Hz spec cadence this is the
/// last ~4 minutes; plenty to spot drift / oscillation without holding
/// state forever.
const LIVE_HISTORY_LEN: usize = 120;

/// "Keep synced" idle poll interval — re-LIST + fetch any grown tail
/// this often when there's nothing left to pull. A busy backlog runs
/// back-to-back (the gap is measured from the previous trigger, which
/// has long elapsed by the time a big pass finishes).
const SYNC_POLL_INTERVAL: std::time::Duration = std::time::Duration::from_secs(30);

#[derive(Default)]
struct AppState {
    /// Auto-detected sensor CSV (e.g. `Sens005.csv`). One per session.
    sensor_csv: Option<PathBuf>,
    /// Auto-detected GPS CSV (e.g. `Sens005_gps.csv`). Optional — when
    /// missing, animate falls back to pitch-oscillation session
    /// detection only.
    gps_csv: Option<PathBuf>,
    /// Optional camera video paired with the session.
    video: Option<PathBuf>,
    /// Optional board STL mesh (defaults to fingerfoil's
    /// `0_combined.stl` if the user has it).
    board_stl: Option<PathBuf>,

    /// Output directory for the generated GIF + MOV.
    output_dir: PathBuf,

    // ----- Optional flags surfaced in the UI ----------------------------
    /// Wall-clock window start ("Von"), e.g. `10:16:09`, local time.
    /// Empty = auto-detect via pitch-oscillation session detector (no
    /// video alignment). Passed as `stbox-viz animate --at`.
    at: String,
    /// Wall-clock window end ("Bis"), e.g. `10:24:00`, local time.
    /// When set together with `at`, the merge video covers exactly
    /// `at`→`to`: the GUI computes `--duration = to - at` so only that
    /// slice of the sensor data is rendered. Empty = use the Duration
    /// field / video length instead.
    to: String,
    /// Local-time UTC offset in hours. 3 for Greek summer (EEST), 2 for
    /// Swiss summer (CEST), 1 for Swiss winter (CET).
    tz_offset_h: f64,
    /// YYYY-MM-DD recording date. Empty = sensor file mtime.
    date: String,
    /// Window length in seconds. 0 = derive from video length / 60 s
    /// fallback.
    duration_s: f64,
    /// Mast-mount or deck-mount. Picks `R_mount`, camera angle and the
    /// 3D-attitude path inside `stbox-viz animate`.
    mount: Mount,
    /// Dock height above water in metres (Ermioni harbour wall = 0.75).
    dock_height_m: f64,
    /// Skip the carry/transition seconds before sustained pitch
    /// oscillation begins.
    auto_skip: bool,
    /// Title-card overlay text.
    title: String,
    subtitle: String,
    /// Output frame rate. Default 15 fps — anything above 20 makes the
    /// GIF huge without visibly improving the trace.
    fps: u32,

    // ----- Run-time state ----------------------------------------------
    /// Live log lines streamed from the child process.
    log: Arc<Mutex<Vec<String>>>,
    /// Cancellation flag — flipped by the Stop button.
    cancel: Arc<AtomicBool>,
    /// True while a child is alive.
    running: Arc<AtomicBool>,
    /// Last subprocess exit status, displayed in the status bar.
    last_status: Option<RunStatus>,

    // ----- GPS Debug tab (gps-debug sidecar) ---------------------------
    /// USB serial port vs BLE bridge. See [`GpsTransport`].
    gps_dbg_transport: GpsTransport,
    /// Last serial-port auto-detect scan (macOS `/dev/cu.usb*`, Linux
    /// `/dev/ttyACM*` etc.). Filled by the Detect button; drives the port
    /// dropdown so the user rarely has to type `/dev/cu.usbserial-XXXX`.
    gps_dbg_detected_ports: Vec<String>,
    /// Serial port of the u-blox under test (e.g. /dev/cu.usbserial-XXXX).
    gps_dbg_port: String,
    /// Baud as a string so the field is free-typed; parsed on Start
    /// (fallback 38400, the box's MAX-M10S rate).
    gps_dbg_baud: String,
    /// Run label — into the CSV filenames + a column, for A/B antenna runs.
    gps_dbg_label: String,
    /// Live stdout/stderr from the `gps-debug` child. Separate `Arc` from
    /// `self.log` so the antenna readout never interleaves with the
    /// animate/BLE log.
    gps_dbg_log: Arc<Mutex<Vec<String>>>,
    /// Stop flag — flipped by the Stop button, polled by the reaper thread.
    gps_dbg_cancel: Arc<AtomicBool>,
    /// True while a `gps-debug` child is alive.
    gps_dbg_running: Arc<AtomicBool>,

    /// BLE debug probe (Debug tab): scan window in seconds for the
    /// `MovementLogger --ble-debug N` child.
    ble_dbg_secs: String,
    /// Live output from the --ble-debug child; also mirrored to a
    /// timestamped .log file in the download (CSV) folder.
    ble_dbg_log: Arc<Mutex<Vec<String>>>,
    /// Stop flag — flipped by the Stop button, polled by the reaper.
    ble_dbg_cancel: Arc<AtomicBool>,
    /// True while a --ble-debug child is alive.
    ble_dbg_running: Arc<AtomicBool>,

    /// Compass calibration: hard-iron offset (mG) subtracted from the
    /// raw mag. Learned SILENTLY by the sliding-window auto-cal (below);
    /// loaded from config.toml at startup. `None` = uncalibrated.
    mag_offset: Option<[f64; 3]>,
    /// Last offset written to config.toml — persists are throttled to
    /// changes > 10 mG so auto-cal doesn't rewrite the file every 2 s.
    mag_off_persisted: Option<[f64; 3]>,
    /// Sliding window of recent FLAT mag (x, y) samples for the silent
    /// hard-iron auto-calibration (iOS/Android parity — `FileSyncCore`
    /// autoCalibrate). A window, not a session envelope, so one motion
    /// spike can't poison the spans forever. 64 samples @ 0.5 Hz ≈ one
    /// slow flat rotation.
    auto_buf: Vec<(f64, f64)>,
    /// One-tap direction anchor (iOS/Android parity): degrees subtracted
    /// from the box's nose azimuth so "USB-C points SOUTH" reads as 180°.
    /// Set by the "USB-C points SOUTH — set direction" button; every
    /// auto-cal offset refinement folds its heading shift in here so the
    /// set direction never drifts.
    heading_bias_deg: f64,
    /// Which body-axis end is the FRONT (USB-C connector): `Some(true)` =
    /// +Y, `Some(false)` = -Y, `None` = not yet confirmed. From the
    /// "USB-C end is UP — confirm" tap.
    nose_plus_y: Option<bool>,

    /// Gyro + accel complementary attitude filter — the 3D preview's
    /// attitude source (mag-independent; iOS `OrientationFilter` parity).
    /// STATEFUL across samples: fed once per received `LiveSample` in
    /// `GuiSyncHost::on_sample`, read (as `ori_rows`) by the renderer.
    orientation_filter: OrientationFilter,
    /// Latest body-frame world axes (n, e, d) from `orientation_filter`,
    /// snapshotted each sample so the egui redraw reads a plain value.
    /// `None` until the filter has seeded from the first good accel+mag.
    ori_rows: Option<OriRows>,

    /// Cached GPU texture for the top-right logo. Lazily uploaded on
    /// first frame so the egui context is available.
    icon_tex: Option<egui::TextureHandle>,

    // ----- BLE FileSync + Sync engine (issue #14 part B: extracted) -----
    /// All BLE-FileSync + Sync engine state. Lifted into `SyncCore`
    /// (`sync_core.rs`) so the headless `--agent` drives the same
    /// engine the GUI does. The GUI reads/writes `self.sync.<field>`
    /// exactly as it used to access `self.<field>`.
    sync: SyncCore,
    /// Held for the GUI's whole lifetime once BLE is first used. Its
    /// mere existence is the "a GUI is alive" signal the headless
    /// agent polls (`coord::GUI_LOCK`); the OS frees it even on a
    /// crash. `_ble_lock` is the BLE-adapter ownership token so the
    /// agent never scans/connects while the GUI is driving the radio.
    _gui_lock: Option<coord::LockGuard>,
    _ble_lock: Option<coord::LockGuard>,

    // ----- UI top-level tab + Live tab state ---------------------------
    /// Active top-level tab — drives the central panel switch.
    current_tab: Tab,
    /// Most recent SensorStream snapshot. Cleared on disconnect.
    latest_sample: Option<LiveSample>,
    /// Wall-clock instant the latest sample arrived. Used for the
    /// "x s ago" freshness label so the user notices a stalled stream.
    latest_sample_at: Option<std::time::Instant>,
    /// Most recent BatteryStatus snapshot (dedicated …0200… characteristic
    /// — richer than the SensorStream `low_batt` bit: real voltage / SoC%
    /// / current). Cleared on link reset.
    latest_battery: Option<BatterySample>,
    /// Wall-clock instant the latest battery reading arrived. Battery
    /// notifies land ~once/min, so the meter tolerates a much longer
    /// staleness window than the stream.
    latest_battery_at: Option<std::time::Instant>,
    /// Rolling history of acc-magnitude samples (in g) for the small
    /// time-series chart on the Live tab. Tuple = (t_seconds_relative,
    /// acc_g_magnitude). Bounded by LIVE_HISTORY_LEN.
    live_acc_history: VecDeque<(f64, f64)>,
    /// Rolling history of pressure (hPa). Same bound.
    live_pressure_history: VecDeque<(f64, f64)>,
    /// `timestamp_ms` of the first sample in `live_*_history`, used to
    /// rebase the X axis so the plot reads "seconds since start" rather
    /// than "ms since box boot" (which has no user meaning).
    live_t0_ms: Option<u32>,
    /// Total number of SensorStream samples received this connection —
    /// shown in the Live tab as a "n samples" counter so the user can
    /// tell the stream is alive even when the values look static.
    live_sample_count: u64,

    // ----- In-app updater ----------------------------------------------
    /// Receiver for the one-shot startup version-check thread.
    update_rx: Option<mpsc::Receiver<Option<update::UpdateInfo>>>,
    /// Latest version info from the GitHub Releases API. Some = banner
    /// shown; None = up-to-date or check hasn't completed yet.
    update_info: Option<update::UpdateInfo>,
    /// True while the version-check request is in flight.
    update_checking: bool,
    /// One-line status badge ("Update available: vX.Y.Z" / "You're on
    /// the latest version (vA.B.C).") — sticks around so the user has
    /// feedback after manually re-checking.
    update_status_msg: Option<String>,
    /// Receiver for InstallEvent stream from the install worker thread.
    install_rx: Option<mpsc::Receiver<installer::InstallEvent>>,
    /// True while the macOS install pipeline is downloading/swapping.
    installing: bool,
    /// Live phase + progress fraction + detail string for the UI bar.
    install_progress: Arc<Mutex<InstallProgress>>,
    /// Last hard error from the install pipeline. Surfaced under the
    /// banner; cleared on next "Update now" click.
    install_error: Option<String>,
}

#[derive(Clone, Default)]
struct InstallProgress {
    phase: String,
    fraction: f32,
    detail: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mount {
    Mast,
    Deck,
}

impl Default for Mount {
    fn default() -> Self {
        Mount::Mast
    }
}

impl Mount {
    fn flag(self) -> &'static str {
        match self {
            Mount::Mast => "mast",
            Mount::Deck => "deck",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RunStatus {
    Ok,
    Failed,
    Cancelled,
}

// ---------------------------------------------------------------------------
//  File classification
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
enum FileKind {
    Sensor,
    Gps,
    Video,
    Stl,
    Unknown,
}

fn classify(path: &Path) -> FileKind {
    let name = path.file_name().and_then(|s| s.to_str()).unwrap_or("");
    let ext = path
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    match ext.as_str() {
        "csv" => {
            if name.ends_with("_gps.csv") {
                FileKind::Gps
            // _quaternions.csv is a derived stbox-viz output, not an
            // input — silently ignore so the user dropping a result
            // folder doesn't end up with the wrong sensor source.
            } else if name.ends_with("_quaternions.csv") || name.ends_with("_errlog.txt") {
                FileKind::Unknown
            } else {
                FileKind::Sensor
            }
        }
        "mov" | "mp4" | "mkv" | "m4v" | "avi" => FileKind::Video,
        "stl" => FileKind::Stl,
        _ => FileKind::Unknown,
    }
}

/// Best-effort companion-GPS guesser: given `Sens005.csv`, look for
/// `Sens005_gps.csv` next to it.
fn guess_gps_for(sensor: &Path) -> Option<PathBuf> {
    let stem = sensor.file_stem()?.to_str()?.to_string();
    let parent = sensor.parent()?;
    let gps = parent.join(format!("{stem}_gps.csv"));
    if gps.exists() {
        Some(gps)
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
//  Locating the bundled stbox-viz binary
// ---------------------------------------------------------------------------

fn stbox_viz_path() -> PathBuf {
    let exe_name = if cfg!(windows) { "stbox-viz.exe" } else { "stbox-viz" };
    if let Ok(my_exe) = std::env::current_exe() {
        if let Some(dir) = my_exe.parent() {
            let candidate = dir.join(exe_name);
            if candidate.exists() {
                return candidate;
            }
        }
    }
    // Fallback to PATH lookup. Letting `Command` resolve via PATH only
    // works if the binary is in the user's PATH — fine for `cargo run`
    // during development if cargo bin is on PATH, otherwise the user
    // sees the spawn failure in the log.
    PathBuf::from(exe_name)
}

/// Locate the bundled `gps-debug` sidecar (next to MovementLogger), same
/// strategy as `stbox_viz_path`: prefer the binary sitting beside us, fall
/// back to a PATH lookup (works under `cargo run` when target/debug is on PATH).
fn gps_debug_path() -> PathBuf {
    let exe_name = if cfg!(windows) { "gps-debug.exe" } else { "gps-debug" };
    if let Ok(my_exe) = std::env::current_exe() {
        if let Some(dir) = my_exe.parent() {
            let candidate = dir.join(exe_name);
            if candidate.exists() {
                return candidate;
            }
        }
    }
    PathBuf::from(exe_name)
}

// ---------------------------------------------------------------------------
//  Spawning + log pump
// ---------------------------------------------------------------------------

/// Parse a local wall-clock `HH:MM` or `HH:MM:SS` into seconds since
/// midnight. `None` on any malformed/out-of-range field. Used only to
/// derive the Bis−Von window length; the absolute `--at` string is
/// still passed through verbatim so stbox-viz owns the real time math.
fn parse_hms(s: &str) -> Option<i64> {
    let mut it = s.trim().split(':');
    let h: i64 = it.next()?.parse().ok()?;
    let m: i64 = it.next()?.parse().ok()?;
    let sec: i64 = match it.next() {
        Some(x) => x.parse().ok()?,
        None => 0,
    };
    if it.next().is_some() || !(0..24).contains(&h) || !(0..60).contains(&m) || !(0..60).contains(&sec)
    {
        return None;
    }
    Some(h * 3600 + m * 60 + sec)
}

fn spawn_animate(state: &mut AppState) {
    let Some(sensor) = state.sensor_csv.clone() else {
        push_log(&state.log, "error: no sensor CSV — drop a Sens*.csv first.".into());
        return;
    };

    let mut cmd = Command::new(stbox_viz_path());
    cmd.arg("animate")
        .arg(&sensor)
        .arg("-o")
        .arg(&state.output_dir)
        .arg("--fps")
        .arg(state.fps.to_string())
        .arg("--mount")
        .arg(state.mount.flag());

    if let Some(v) = state.video.as_ref() {
        cmd.arg("--video").arg(v);
    }
    if let Some(stl) = state.board_stl.as_ref() {
        cmd.arg("--board-stl").arg(stl);
    }
    if !state.at.trim().is_empty() {
        cmd.arg("--at").arg(state.at.trim());
    }
    if state.tz_offset_h.abs() > f64::EPSILON {
        cmd.arg("--tz-offset-h").arg(format!("{}", state.tz_offset_h));
    }
    if !state.date.trim().is_empty() {
        cmd.arg("--date").arg(state.date.trim());
    }
    /* "Von" → "Bis" window. When the user filled in Bis, the merge
       video must cover exactly that wall-clock slice, so derive the
       duration from Bis − Von and pass it to `--duration` (stbox-viz
       animate has no native end-time flag). Fail loud on a bad/empty
       Von or a non-increasing window instead of silently rendering
       the wrong span. With Bis empty, fall back to the Duration
       field exactly as before. */
    let to = state.to.trim();
    if !to.is_empty() {
        let from = state.at.trim();
        let parsed = parse_hms(from).zip(parse_hms(to));
        match parsed {
            Some((f, t)) if t > f => {
                cmd.arg("--duration").arg(format!("{}", t - f));
            }
            Some((_, _)) => {
                push_log(
                    &state.log,
                    "error: \u{201c}Bis\u{201d} must be later than \u{201c}Von\u{201d} \
                     — fix the window and Generate again."
                        .into(),
                );
                return;
            }
            None => {
                push_log(
                    &state.log,
                    "error: \u{201c}Von\u{201d}/\u{201c}Bis\u{201d} must both be \
                     HH:MM or HH:MM:SS (local time)."
                        .into(),
                );
                return;
            }
        }
    } else if state.duration_s > 0.0 {
        cmd.arg("--duration").arg(format!("{}", state.duration_s));
    }
    if state.dock_height_m.abs() > f64::EPSILON {
        cmd.arg("--dock-height-m")
            .arg(format!("{}", state.dock_height_m));
    }
    if state.auto_skip {
        cmd.arg("--auto-skip");
    }
    if !state.title.trim().is_empty() {
        cmd.arg("--title").arg(&state.title);
    }
    if !state.subtitle.trim().is_empty() {
        cmd.arg("--subtitle").arg(&state.subtitle);
    }

    cmd.stdout(Stdio::piped()).stderr(Stdio::piped());

    push_log(
        &state.log,
        format!(
            "$ {} {}",
            stbox_viz_path().display(),
            cmd.get_args()
                .map(|s| s.to_string_lossy().into_owned())
                .collect::<Vec<_>>()
                .join(" ")
        ),
    );

    let mut child = match cmd.spawn() {
        Ok(c) => c,
        Err(e) => {
            push_log(
                &state.log,
                format!("error: failed to spawn stbox-viz: {e}. Make sure it sits next to MovementLogger or is on PATH."),
            );
            state.last_status = Some(RunStatus::Failed);
            return;
        }
    };

    state.cancel.store(false, Ordering::SeqCst);
    state.running.store(true, Ordering::SeqCst);
    let log = state.log.clone();
    let cancel = state.cancel.clone();
    let running = state.running.clone();

    // One thread per stream so a deadlock on either pipe doesn't stall
    // the other. Both forward into a shared mpsc that the main thread
    // drains into the log vec.
    let (tx, rx) = mpsc::channel::<String>();

    if let Some(out) = child.stdout.take() {
        let tx = tx.clone();
        thread::spawn(move || pump_pipe(out, tx));
    }
    if let Some(err) = child.stderr.take() {
        let tx = tx.clone();
        thread::spawn(move || pump_pipe(err, tx));
    }
    drop(tx);

    thread::spawn(move || {
        for line in rx {
            push_log(&log, line);
        }
        // After both pipes have closed, wait for exit + reap.
        let mut cancelled = false;
        loop {
            match child.try_wait() {
                Ok(Some(status)) => {
                    let msg = if cancelled {
                        "--- cancelled ---".to_string()
                    } else if status.success() {
                        "--- done ---".to_string()
                    } else {
                        format!("--- exited with {status} ---")
                    };
                    push_log(&log, msg);
                    break;
                }
                Ok(None) => {
                    if cancel.load(Ordering::SeqCst) && !cancelled {
                        // Best-effort kill — child carries on if signal
                        // delivery fails on this platform, but the wait
                        // loop keeps going.
                        let _ = child.kill();
                        cancelled = true;
                    }
                    thread::sleep(std::time::Duration::from_millis(100));
                }
                Err(e) => {
                    push_log(&log, format!("error: try_wait failed: {e}"));
                    break;
                }
            }
        }
        running.store(false, Ordering::SeqCst);
    });
}

fn pump_pipe<R: std::io::Read + Send + 'static>(r: R, tx: mpsc::Sender<String>) {
    use std::io::{BufRead, BufReader};
    let buf = BufReader::new(r);
    for line in buf.lines() {
        match line {
            Ok(s) => {
                if tx.send(s).is_err() {
                    break;
                }
            }
            Err(_) => break,
        }
    }
}

/// `Read + Write` adapter that lets `gps_debug::run_core` drive a GPS survey
/// over the box's BLE link instead of a serial port. Writes are forwarded as
/// `BleCmd::GpsTx` (UBX poll frames → box → u-blox UART); reads drain the raw
/// bridged UBX bytes the worker forwards from FileData notifies. `read`
/// blocks up to ~50 ms then returns `Ok(0)`, matching the serial transport's
/// timeout contract so the survey's collect loop stays responsive to its stop
/// flag. A notify can exceed the caller's buffer only in theory (BLE MTU ≪ 2
/// kB), but `leftover` keeps the adapter correct regardless.
struct BleTransport {
    cmd: mpsc::Sender<BleCmd>,
    rx: mpsc::Receiver<Vec<u8>>,
    leftover: Vec<u8>,
}

impl BleTransport {
    fn new(cmd: mpsc::Sender<BleCmd>, rx: mpsc::Receiver<Vec<u8>>) -> Self {
        Self { cmd, rx, leftover: Vec::new() }
    }
}

impl std::io::Read for BleTransport {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        if self.leftover.is_empty() {
            match self.rx.recv_timeout(std::time::Duration::from_millis(50)) {
                Ok(bytes) => self.leftover = bytes,
                // Timeout → "no data this window" (Ok(0), like serial);
                // Disconnected → the worker dropped the sink (bridge off).
                Err(_) => return Ok(0),
            }
        }
        let n = buf.len().min(self.leftover.len());
        buf[..n].copy_from_slice(&self.leftover[..n]);
        self.leftover.drain(..n);
        Ok(n)
    }
}

impl std::io::Write for BleTransport {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.cmd
            .send(BleCmd::GpsTx { bytes: buf.to_vec() })
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::BrokenPipe, e.to_string()))?;
        Ok(buf.len())
    }
    fn flush(&mut self) -> std::io::Result<()> { Ok(()) }
}

// ---------------------------------------------------------------------------
//  egui app
// ---------------------------------------------------------------------------

/// GUI implementation of `SyncHost`: borrows just the Live-tab chart
/// fields + Replay-form slots (disjoint from `AppState::sync`) so
/// `SyncCore` can drive them without aliasing its own state. The
/// headless agent uses a no-op host instead.
struct GuiSyncHost<'a> {
    latest_sample: &'a mut Option<LiveSample>,
    latest_sample_at: &'a mut Option<std::time::Instant>,
    latest_battery: &'a mut Option<BatterySample>,
    latest_battery_at: &'a mut Option<std::time::Instant>,
    live_acc_history: &'a mut VecDeque<(f64, f64)>,
    live_pressure_history: &'a mut VecDeque<(f64, f64)>,
    live_t0_ms: &'a mut Option<u32>,
    live_sample_count: &'a mut u64,
    sensor_csv: &'a mut Option<PathBuf>,
    gps_csv: &'a mut Option<PathBuf>,
    /// Gyro+accel attitude filter (mag-independent) driven once per
    /// sample; `ori_rows` is its snapshot for the renderer. `mag_offset`
    /// only SEEDS the filter's initial heading (iOS parity).
    orientation_filter: &'a mut OrientationFilter,
    ori_rows: &'a mut Option<OriRows>,
    mag_offset: Option<[f64; 3]>,
}

impl SyncHost for GuiSyncHost<'_> {
    fn on_link_reset(&mut self) {
        /* Drop live state — the box can drop the stream mid-connection
           (e.g. flaky link) and the next reconnect should start from a
           clean history, not graft new samples onto stale chart data. */
        *self.latest_sample = None;
        *self.latest_sample_at = None;
        *self.latest_battery = None;
        *self.latest_battery_at = None;
        self.live_acc_history.clear();
        self.live_pressure_history.clear();
        *self.live_t0_ms = None;
        *self.live_sample_count = 0;
        // Re-seed the attitude filter on the next sample (keeps the
        // learned gyro bias); drop the stale render rows.
        self.orientation_filter.reset();
        *self.ori_rows = None;
    }

    fn on_sample(&mut self, s: &LiveSample) {
        *self.latest_sample = Some(*s);
        *self.latest_sample_at = Some(std::time::Instant::now());
        *self.live_sample_count = self.live_sample_count.saturating_add(1);
        // Gyro+accel attitude for the 3D preview (mag-independent). The
        // filter is stateful — updated once here per received sample; the
        // egui redraw reads the snapshotted `ori_rows`.
        self.orientation_filter.update(s, self.mag_offset);
        *self.ori_rows = self.orientation_filter.rows();
        // Rebase time axis on the first sample so the chart X starts at
        // 0 regardless of how long the box has been booted.
        let t0 = *self.live_t0_ms.get_or_insert(s.timestamp_ms);
        let dt = (s.timestamp_ms.wrapping_sub(t0)) as f64 / 1000.0;
        let (ax, ay, az) = (
            s.acc_mg[0] as f64 / 1000.0,
            s.acc_mg[1] as f64 / 1000.0,
            s.acc_mg[2] as f64 / 1000.0,
        );
        let acc_g = (ax * ax + ay * ay + az * az).sqrt();
        let pres_hpa = s.pressure_pa as f64 / 100.0;
        if self.live_acc_history.len() >= LIVE_HISTORY_LEN {
            self.live_acc_history.pop_front();
        }
        if self.live_pressure_history.len() >= LIVE_HISTORY_LEN {
            self.live_pressure_history.pop_front();
        }
        self.live_acc_history.push_back((dt, acc_g));
        self.live_pressure_history.push_back((dt, pres_hpa));
    }

    fn on_battery(&mut self, b: &BatterySample) {
        *self.latest_battery = Some(*b);
        *self.latest_battery_at = Some(std::time::Instant::now());
    }

    fn on_downloaded(&mut self, name: &str, path: &Path) {
        // If the saved file is a Sens*.csv or matching _gps.csv, set it
        // on the top-of-form slots so the user doesn't need to hit Pick…
        let lower = name.to_ascii_lowercase();
        if lower.ends_with("_gps.csv") {
            *self.gps_csv = Some(path.into());
        } else if lower.starts_with("sens") && lower.ends_with(".csv") {
            *self.sensor_csv = Some(path.into());
            if self.gps_csv.is_none() {
                *self.gps_csv = guess_gps_for(path);
            }
        }
    }
}

impl AppState {
    fn new() -> Self {
        let base = default_save_base();
        let output_dir = base.join("gif");
        let ble_out_dir = base.join("csv");
        // One shared log buffer: the GUI's scrollable panel reads
        // `self.log`; the sync engine writes via `self.sync.log`. Same
        // `Arc` so BLE/sync lines land in the same panel as before.
        let log: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let mut s = Self {
            output_dir,
            tz_offset_h: 3.0,
            fps: 15,
            sync: SyncCore {
                ble_out_dir,
                ble_session_duration_s: 1800, // 30-min default
                // Seed the last-box id from the persisted config so the
                // "Reconnect (last box)" button works right after launch,
                // before any connect this session.
                ble_last_box_id: agent_config::AgentConfig::load().box_id,
                log: log.clone(),
                ..Default::default()
            },
            log,
            ble_dbg_secs: "20".to_string(),
            mag_offset: agent_config::AgentConfig::load().mag_offset_mg,
            mag_off_persisted: agent_config::AgentConfig::load().mag_offset_mg,
            heading_bias_deg: agent_config::AgentConfig::load().heading_bias_deg.unwrap_or(0.0),
            nose_plus_y: agent_config::AgentConfig::load().nose_plus_y,
            gps_dbg_baud: "38400".to_string(),
            gps_dbg_label: "antenna".to_string(),
            // Pre-fill the serial port from a one-shot scan so the common
            // single-adapter case needs no typing; the Detect button
            // re-scans on demand.
            gps_dbg_port: {
                let ports = gps_debug::detect_ports();
                ports.first().cloned().unwrap_or_default()
            },
            gps_dbg_detected_ports: gps_debug::detect_ports(),
            update_rx: Some(spawn_update_check()),
            update_checking: true,
            ..Self::default()
        };
        /* Grade the existing ERRLOG mirror once at startup so the Debug
           tab's "Box health" panel is populated before any connect;
           every later sync of ERRLOG.LOG re-runs it automatically. */
        s.sync.run_errlog_check("startup");
        s
    }

    fn trigger_update_check(&mut self) {
        if self.update_checking { return; }
        self.update_status_msg = None;
        self.update_checking = true;
        self.update_rx = Some(spawn_update_check());
    }

    fn start_install(&mut self) {
        if self.installing { return; }
        let Some(info) = self.update_info.clone() else { return };
        let Some(download_url) = info.download_url.clone() else {
            self.install_error = Some(
                "This release has no artifact for the current platform yet.".into(),
            );
            return;
        };
        if !installer::can_in_app_update() {
            self.install_error = Some(
                "In-app update is not supported in this build context (e.g. `cargo run`). \
                 Open the release page to download manually.".into()
            );
            return;
        }

        self.install_error = None;
        if let Ok(mut p) = self.install_progress.lock() { *p = InstallProgress::default(); }
        let (tx, rx) = mpsc::channel::<installer::InstallEvent>();
        self.install_rx = Some(rx);
        self.installing = true;
        push_log(&self.log, format!("Starting in-app update to {}…", info.pretty()));

        thread::spawn(move || {
            match installer::install(&download_url, tx.clone()) {
                Ok(()) => {
                    // Helper script is detached and waiting for our PID
                    // to die. Give the user 600 ms to read the success
                    // line, then exit so the swap can run.
                    std::thread::sleep(std::time::Duration::from_millis(600));
                    std::process::exit(0);
                }
                Err(e) => { let _ = tx.send(installer::InstallEvent::Error(e)); }
            }
        });
    }
}

fn spawn_update_check() -> mpsc::Receiver<Option<update::UpdateInfo>> {
    let (tx, rx) = mpsc::channel::<Option<update::UpdateInfo>>();
    thread::spawn(move || {
        let _ = tx.send(update::check_latest(env!("CARGO_PKG_VERSION")));
    });
    rx
}

impl AppState {

    fn ensure_ble(&mut self) -> &BleBackend {
        if self.sync.ble.is_none() {
            /* Claim adapter ownership before the worker starts. The
               GUI must win over the background agent: it holds
               `gui.lock` for its lifetime (the agent's presence
               signal) and takes `ble.lock` (the adapter token). When
               no agent is running both are free and this is instant;
               the agent (step 6) yields `ble.lock` within ~1 s of
               seeing `gui.lock` contended. Non-blocking so the UI
               thread never freezes — a brief overlap on first launch
               is harmless and self-heals as the agent backs off. */
            if self._gui_lock.is_none() {
                self._gui_lock = coord::try_acquire(coord::GUI_LOCK);
            }
            if self._ble_lock.is_none() {
                self._ble_lock = coord::try_acquire(coord::BLE_LOCK);
            }
            self.sync.ble = Some(BleBackend::spawn());
        }
        self.sync.ble.as_ref().unwrap()
    }

    /// Drain pending BLE events into the visible state. Called once
    /// per frame. Thin GUI adapter: builds a `GuiSyncHost` borrowing
    /// just the Live-tab + replay-routing fields (disjoint from
    /// `self.sync`) and delegates the engine work to `SyncCore`, which
    /// the headless `--agent` drives identically.
    fn pump_ble_events(&mut self) {
        let AppState {
            sync,
            latest_sample,
            latest_sample_at,
            latest_battery,
            latest_battery_at,
            live_acc_history,
            live_pressure_history,
            live_t0_ms,
            live_sample_count,
            sensor_csv,
            gps_csv,
            orientation_filter,
            ori_rows,
            mag_offset,
            ..
        } = self;
        let mut host = GuiSyncHost {
            latest_sample,
            latest_sample_at,
            latest_battery,
            latest_battery_at,
            live_acc_history,
            live_pressure_history,
            live_t0_ms,
            live_sample_count,
            sensor_csv,
            gps_csv,
            orientation_filter,
            ori_rows,
            mag_offset: *mag_offset,
        };
        sync.pump_ble_events(&mut host);
    }

    /// Drain pending update-check + install-pipeline events into the
    /// visible state. Called once per frame.
    fn pump_update_events(&mut self) {
        if let Some(rx) = &self.update_rx {
            if let Ok(result) = rx.try_recv() {
                self.update_checking = false;
                self.update_rx = None;
                match result {
                    Some(info) => {
                        push_log(
                            &self.log,
                            format!(
                                "Update available: v{} -> {}",
                                env!("CARGO_PKG_VERSION"),
                                info.pretty()
                            ),
                        );
                        self.update_status_msg =
                            Some(format!("Update available: {}", info.pretty()));
                        self.update_info = Some(info);
                    }
                    None => {
                        self.update_status_msg = Some(format!(
                            "You're on the latest version (v{}).",
                            env!("CARGO_PKG_VERSION")
                        ));
                    }
                }
            }
        }

        if let Some(rx) = &self.install_rx {
            let mut done = false;
            while let Ok(ev) = rx.try_recv() {
                match ev {
                    installer::InstallEvent::Log(s) => push_log(&self.log, s),
                    installer::InstallEvent::Phase(phase) => {
                        if let Ok(mut p) = self.install_progress.lock() {
                            p.phase = phase;
                        }
                    }
                    installer::InstallEvent::DownloadProgress { bytes, total } => {
                        if let Ok(mut p) = self.install_progress.lock() {
                            p.phase = "Downloading update".into();
                            p.fraction =
                                if total == 0 { 0.0 } else { bytes as f32 / total as f32 };
                            p.detail = if total == 0 {
                                format!("{:.1} MB", bytes as f64 / 1_048_576.0)
                            } else {
                                format!(
                                    "{:.1} / {:.1} MB",
                                    bytes as f64 / 1_048_576.0,
                                    total as f64 / 1_048_576.0,
                                )
                            };
                        }
                    }
                    installer::InstallEvent::Done => {
                        push_log(&self.log, "Update staged. Restarting…".into());
                        if let Ok(mut p) = self.install_progress.lock() {
                            p.phase = "Restarting…".into();
                            p.fraction = 1.0;
                            p.detail.clear();
                        }
                        // The worker thread calls process::exit shortly
                        // after sending Done; nothing else to do here.
                    }
                    installer::InstallEvent::Error(e) => {
                        self.install_error = Some(e.clone());
                        push_log(&self.log, format!("Update failed: {}", e));
                        done = true;
                    }
                }
            }
            if done {
                self.installing = false;
                self.install_rx = None;
                if let Ok(mut p) = self.install_progress.lock() {
                    *p = InstallProgress::default();
                }
            }
        }
    }

    /// Render the "Update available" banner at the top of the central
    /// panel. macOS gets an in-app "Update now" button that downloads
    /// the DMG, swaps the .app, and relaunches; other platforms get an
    /// "Open release page" button that opens the GitHub release in the
    /// default browser.
    fn render_update_banner(&mut self, ui: &mut egui::Ui) {
        let Some(info) = self.update_info.clone() else { return };
        let can_in_app_update =
            info.download_url.is_some() && installer::can_in_app_update();
        egui::Frame::none()
            .fill(egui::Color32::from_rgb(220, 240, 255))
            .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(80, 130, 200)))
            .inner_margin(8.0)
            .rounding(4.0)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.colored_label(
                        egui::Color32::from_rgb(20, 60, 120),
                        format!(
                            "⬆ Update available: {} (you have v{})",
                            info.pretty(),
                            env!("CARGO_PKG_VERSION")
                        ),
                    );
                    ui.with_layout(
                        egui::Layout::right_to_left(egui::Align::Center),
                        |ui| {
                            if !self.installing && ui.button("Dismiss").clicked() {
                                self.update_info = None;
                            }
                            if can_in_app_update {
                                let label = if self.installing { "Updating…" } else { "Update now" };
                                let resp = ui.add_enabled(
                                    !self.installing,
                                    egui::Button::new(label),
                                );
                                if resp.clicked() {
                                    self.start_install();
                                }
                            } else if ui.button("Open release page").clicked() {
                                ui.ctx().open_url(egui::OpenUrl::new_tab(info.url.clone()));
                            }
                        },
                    );
                });
                if self.installing {
                    let p = self.install_progress.lock().unwrap().clone();
                    let bar = if p.fraction > 0.0 {
                        egui::ProgressBar::new(p.fraction).show_percentage().animate(true)
                    } else {
                        egui::ProgressBar::new(0.0).animate(true)
                    };
                    let phase_label = if p.phase.is_empty() {
                        "Working".to_string()
                    } else {
                        p.phase.clone()
                    };
                    ui.add_space(4.0);
                    ui.add(bar.text(phase_label));
                    if !p.detail.is_empty() {
                        ui.label(egui::RichText::new(p.detail).weak());
                    }
                }
                if let Some(err) = self.install_error.clone() {
                    ui.add_space(4.0);
                    ui.colored_label(
                        egui::Color32::from_rgb(160, 30, 30),
                        format!("Update failed: {}", err),
                    );
                }
            });
        ui.add_space(6.0);
    }

    /// Render the "New box firmware available" banner. Twin of
    /// `render_update_banner` but for the *box's* firmware (FOTA over BLE)
    /// rather than this app's own release: shown only while connected and
    /// with a pending update. "Update box" downloads + flashes via
    /// `fw_apply_update`; "✕" dismisses. Same colored strip as the app-update
    /// banner so the two read as one family.
    fn render_fw_banner(&mut self, ui: &mut egui::Ui) {
        if !matches!(self.sync.ble_state, BleState::Connected) {
            return;
        }
        let Some((version, _url)) = self.sync.fw_update_available.clone() else {
            return;
        };
        // A flash in flight takes over the Sync-tab UI; hide the offer while
        // one runs so the banner can't re-trigger it.
        let flashing = self.sync.ble_flash_progress.is_some();
        egui::Frame::none()
            .fill(egui::Color32::from_rgb(220, 240, 255))
            .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(80, 130, 200)))
            .inner_margin(8.0)
            .rounding(4.0)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.colored_label(
                        egui::Color32::from_rgb(20, 60, 120),
                        format!("🔄 New box firmware v{version} available"),
                    );
                    ui.with_layout(
                        egui::Layout::right_to_left(egui::Align::Center),
                        |ui| {
                            if !flashing && ui.button("✕").on_hover_text("Dismiss").clicked() {
                                self.sync.fw_update_available = None;
                            }
                            let label = if flashing { "Updating…" } else { "Update box" };
                            if ui
                                .add_enabled(!flashing, egui::Button::new(label))
                                .on_hover_text(
                                    "Download the firmware image and flash it to the \
                                     connected box over BLE. The box verifies the image, \
                                     swaps flash banks, and reboots into the new firmware.",
                                )
                                .clicked()
                            {
                                self.sync.fw_apply_update();
                            }
                        },
                    );
                });
            });
        ui.add_space(6.0);
    }

}

/// Recency rank for the file list: the trailing per-session counter in
/// `SensNNN.csv` / `GpsNNN.csv` / … — higher = later session = shown
/// first. Names without a number rank lowest (-1).
fn recency_key(name: &str) -> i64 {
    let mut last: Option<i64> = None;
    let mut cur = String::new();
    for c in name.chars() {
        if c.is_ascii_digit() {
            cur.push(c);
        } else if !cur.is_empty() {
            last = cur.parse().ok();
            cur.clear();
        }
    }
    if !cur.is_empty() {
        last = cur.parse().ok();
    }
    last.unwrap_or(-1)
}

/// Names the box firmware can never delete, with the reason (for the
/// disabled trash button's tooltip). Mirrors the firmware limits:
/// `ble.c` rejects names >15 bytes with BAD_REQUEST, and
/// `SDFat_Delete` only matches a real FAT 8.3 short name — so macOS
/// AppleDouble sidecars (`._*`) and the virtual `PUMPTSUE.RI`
/// placeholder always come back NOT_FOUND. Disabling the button for
/// these stops it looking like a no-op click.
fn delete_unsupported(name: &str) -> Option<&'static str> {
    if name.starts_with("._") {
        Some("macOS metadata sidecar — not a real file on the box's SD card, the firmware can't address it")
    } else if name.eq_ignore_ascii_case("PUMPTSUE.RI") {
        Some("virtual placeholder entry, not a real file — nothing to delete")
    } else if name.len() > 15 {
        Some("filename too long for the box's delete command (firmware caps it at 15 characters)")
    } else {
        None
    }
}

/// Render one group ("Sensor" or "Debug") inside the BLE file list.
/// Borrows `files` mutably to flip per-row checkboxes; pushes a delete
/// target name when the user clicks the trash icon so the caller can
/// send the BLE command outside of this borrow. Indices are looked up
/// by position; the caller is responsible for partitioning them.
fn render_file_group(
    ui: &mut egui::Ui,
    title: &str,
    indices: &[usize],
    files: &mut [BleFile],
    delete_target: &mut Option<String>,
) {
    if indices.is_empty() { return; }
    /* Header strip with a "Select all" toggle right next to the title —
       left-packed like the file rows, not pinned to the window's far right
       (where it sat a full window-width away from the list it acts on). The
       toggle reads the common state of the group (all on / all off / mixed)
       so the user can flip the whole section in one click. */
    let all_on  = indices.iter().all(|&i| files[i].selected);
    let none_on = indices.iter().all(|&i| !files[i].selected);
    ui.horizontal(|ui| {
        ui.add(egui::Label::new(
            egui::RichText::new(title).strong().size(13.0)
        ));
        ui.label(egui::RichText::new(format!("({})", indices.len())));
        let label = if all_on { "Untick all" }
                    else if none_on { "Tick all" }
                    else { "Tick all" };
        if ui.small_button(label).clicked() {
            let new_val = !all_on;
            for &i in indices { files[i].selected = new_val; }
        }
    });
    for &i in indices {
        let f = &mut files[i];
        ui.horizontal(|ui| {
            ui.checkbox(&mut f.selected, "");
            // Monospace size so the digits right-align cleanly into a column;
            // the name follows, then the trash button + status sit right next
            // to the name (not pinned to the far right).
            ui.label(egui::RichText::new(format!("{:>10} B", f.size)).monospace());
            ui.label(&f.name);
            let busy = f.bytes_done > 0 && !f.downloaded;
            match delete_unsupported(&f.name) {
                Some(reason) => {
                    // Greyed out: the firmware would just reply
                    // NOT_FOUND / BAD_REQUEST — don't pretend it's
                    // actionable. Tooltip explains why.
                    ui.add_enabled(false, egui::Button::new("🗑").small())
                        .on_disabled_hover_text(format!("Can't delete: {reason}"));
                }
                None => {
                    if ui
                        .add_enabled(!busy, egui::Button::new("🗑").small())
                        .on_hover_text("Delete this file from the SD card")
                        .clicked()
                    {
                        *delete_target = Some(f.name.clone());
                    }
                }
            }
            if f.downloaded {
                /* Clear "downloaded" badge. Earlier code used `✓` U+2713,
                   which is missing from egui's default proportional font
                   and rendered as an empty tofu box. Use ✅ U+2705 (the
                   emoji form) which egui can rasterise from its emoji
                   fallback font — same path as the 🗑 trash button. The
                   plain-text "done" suffix is the guaranteed-readable
                   backup if the user's font config drops emoji entirely. */
                ui.colored_label(
                    egui::Color32::from_rgb(0, 170, 0),
                    egui::RichText::new("✅ downloaded").strong(),
                );
            }
        });
        if !f.downloaded && f.bytes_done > 0 && f.size > 0 {
            let frac = (f.bytes_done as f32 / f.size as f32).clamp(0.0, 1.0);
            let bytes_str = if f.size >= 1024 * 1024 {
                format!("{:.2} / {:.2} MB",
                    f.bytes_done as f64 / 1_048_576.0,
                    f.size as f64 / 1_048_576.0)
            } else {
                format!("{} / {} B", f.bytes_done, f.size)
            };
            // Full-width progress bar (adapts to the window; no fixed 360 px
            // that would overflow a narrow window).
            ui.add(egui::ProgressBar::new(frac).text(bytes_str));
        }
    }
}

impl AppState {
    /// Sync tab — BLE FileSync: scan, connect, list SD files, download
    /// them onto disk. Body identical to the pre-tabbed `ui_ble_panel`
    /// (the CollapsingHeader is dropped because the whole tab is
    /// dedicated to this content). Indentation is preserved verbatim to
    /// keep the diff readable.
    fn ui_sync_tab(&mut self, ui: &mut egui::Ui) {
        // Header strip — tab is dedicated, but a clear top label helps
        // when the user has muscle memory of the older collapsing UI.
        ui.heading("Sync");
        ui.label(
            egui::RichText::new(
                "BLE FileSync — scan, connect (PIN 123456), list SD files. \
                 \"Download selected\" = manual transfer; \"Sync now\" = pull \
                 every new session, tracked in a local DB (never deletes on the box).",
            ),
        );
        ui.add_space(4.0);
        // Block-scoped to keep the body's existing closure-flavoured
        // indentation; the outer `ui` is borrowed mutably below.
        let ui = ui;
        {
            {

                /* Interrupted-transfer banner. Shown while disconnected
                   after a mid-batch link loss (the ble_sync_msg surface
                   only renders inside the connected block, so without
                   this the guidance would be invisible exactly when the
                   user needs it — at the Scan/Connect screen). */
                if self.sync.ble_resume_box.is_some()
                    && !matches!(self.sync.ble_state, BleState::Connected)
                {
                    egui::Frame::group(ui.style())
                        .fill(egui::Color32::from_rgb(255, 244, 204))
                        .stroke(egui::Stroke::new(
                            1.0,
                            egui::Color32::from_rgb(200, 150, 60),
                        ))
                        .show(ui, |ui| {
                            ui.colored_label(
                                egui::Color32::from_rgb(120, 80, 20),
                                "Transfer interrupted (BLE link lost). Scan and \
                                 reconnect to the same box — the sync resumes \
                                 automatically and skips files already saved.",
                            );
                        });
                    ui.add_space(4.0);
                }

                /* Post-flash banner. After a successful firmware upload
                   the box reboots and the link drops, so the result
                   message (and "reconnect in a few seconds" guidance) is
                   only useful here, on the disconnected Scan/Connect
                   screen. Shown whenever a flash result exists and we're
                   not connected; the connected block shows progress
                   instead. */
                if !self.sync.ble_flash_msg.is_empty()
                    && !matches!(self.sync.ble_state, BleState::Connected)
                {
                    let is_err = self.sync.ble_flash_msg.contains("failed")
                        || self.sync.ble_flash_msg.contains("Can't")
                        || self.sync.ble_flash_msg.contains("empty");
                    let (fill, stroke, text) = if is_err {
                        (
                            egui::Color32::from_rgb(255, 230, 230),
                            egui::Color32::from_rgb(200, 80, 80),
                            egui::Color32::from_rgb(170, 30, 30),
                        )
                    } else {
                        (
                            egui::Color32::from_rgb(224, 245, 230),
                            egui::Color32::from_rgb(80, 170, 110),
                            egui::Color32::from_rgb(30, 120, 60),
                        )
                    };
                    egui::Frame::group(ui.style())
                        .fill(fill)
                        .stroke(egui::Stroke::new(1.0, stroke))
                        .show(ui, |ui| {
                            ui.horizontal(|ui| {
                                ui.colored_label(text, &self.sync.ble_flash_msg);
                                if ui.small_button("Dismiss").clicked() {
                                    self.sync.ble_flash_msg.clear();
                                }
                            });
                        });
                    ui.add_space(4.0);
                }

                /* LOG session countdown banner. During a session the box
                   is in LOG mode and doesn't advertise — Scan returns
                   nothing — so the user needs a visible "wait this long"
                   message + ETA. Cleared automatically once the deadline
                   passes (plus a ~8 s bring-up margin), at which point
                   the box should be advertising PumpTsueri again. */
                if let Some((started, dur)) = self.sync.ble_session_running {
                    const BLE_BRINGUP_S: u64 = 8;
                    let elapsed_s = started.elapsed().as_secs();
                    let total_with_bringup = dur as u64 + BLE_BRINGUP_S;
                    if elapsed_s >= total_with_bringup {
                        self.sync.ble_session_running = None;
                        push_log(&self.log, "ble: LOG session deadline reached — box should be advertising again".into());
                    } else {
                        let remaining = total_with_bringup - elapsed_s;
                        let m = remaining / 60;
                        let s = remaining % 60;
                        let total_m = dur / 60;
                        let total_s = dur % 60;
                        let msg = if m > 0 {
                            format!("Logging in progress — {} min {:02} s left (of {} min {:02} s + ~8 s reboot). Box is silent, won't appear in Scan.", m, s, total_m, total_s)
                        } else {
                            format!("Logging in progress — {} s left (of {} min {:02} s + ~8 s reboot). Box is silent, won't appear in Scan.", s, total_m, total_s)
                        };
                        egui::Frame::group(ui.style())
                            .fill(egui::Color32::from_rgb(255, 244, 204))   // soft amber
                            .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(200, 150, 60)))
                            .show(ui, |ui| {
                                ui.colored_label(egui::Color32::from_rgb(120, 80, 20), msg);
                            });
                        /* Force the UI to keep ticking once a second so the
                           countdown actually advances even when the user
                           isn't moving the mouse. */
                        ui.ctx().request_repaint_after(std::time::Duration::from_millis(500));
                    }
                    ui.add_space(4.0);
                }

                // ----- Action buttons -------------------------------
                ui.horizontal(|ui| {
                    let scanning  = matches!(self.sync.ble_state, BleState::Scanning | BleState::Connecting);
                    let connected = matches!(self.sync.ble_state, BleState::Connected);
                    if ui
                        .add_enabled(!scanning && !connected, egui::Button::new("Scan"))
                        .clicked()
                    {
                        self.sync.ble_devices.clear();
                        self.sync.ble_state = BleState::Scanning;
                        // Pass the saved box id so the scan also matches it
                        // when macOS' stale name cache hides the box's name.
                        let known_id = self.sync.ble_last_box_id.clone();
                        let b = self.ensure_ble();
                        b.send(BleCmd::Scan { known_id });
                    }
                    // Reconnect straight to the last box by id — no scan. On
                    // macOS CoreBluetooth suppresses a recently-disconnected
                    // peripheral from scan results, so after Disconnect the box
                    // never reappears in the "Discovered" list and there's no
                    // Connect button to click (the iPhone, a different central,
                    // sees it fine — the classic Mac-only reconnect gap). This
                    // sends Connect(last_id) directly; connect_core's retrieve-
                    // by-id fallback pulls the box in without needing a scan hit.
                    if let Some(last_id) = self.sync.ble_last_box_id.clone() {
                        if ui
                            .add_enabled(!scanning && !connected, egui::Button::new("Reconnect (last box)"))
                            .on_hover_text(
                                "Reconnect to the last box by id without scanning. Use this on \
                                 macOS after Disconnect, when the box doesn't reappear under \
                                 Discovered (CoreBluetooth hides a just-disconnected device).",
                            )
                            .clicked()
                        {
                            self.sync.ble_state = BleState::Connecting;
                            self.sync.ble_status = "reconnecting to last box…".into();
                            self.sync.ble_connected_id = Some(last_id.clone());
                            let b = self.ensure_ble();
                            b.send(BleCmd::Connect(last_id));
                        }
                    }
                    // Disable while ANY worker op is in flight — a big
                    // keep-synced READ holds the BLE worker busy for
                    // minutes, and tap-while-busy would pile a second
                    // LIST behind the in-flight op (worker queues
                    // commands serially so the click eventually fires
                    // out of order, against an already-changed file
                    // list). iOS/Android parity (2d001a4).
                    let worker_busy = self.sync.ble_dl_in_flight
                        || self.sync.ble_sync_pending
                        || !self.sync.ble_dl_queue.is_empty()
                        || self.sync.ble_flash_progress.is_some();
                    if ui
                        .add_enabled(connected && !worker_busy, egui::Button::new("Refresh file list"))
                        .clicked()
                    {
                        self.sync.ble_files.clear();
                        if let Some(b) = self.sync.ble.as_ref() { b.send(BleCmd::List); }
                    }
                    if ui
                        .add_enabled(
                            connected && !worker_busy,
                            egui::Button::new("Sync now"),
                        )
                        .on_hover_text(
                            "Fetch every session file that's larger on the box than \
                             the local copy (only the new tail is pulled, so a \
                             growing log isn't re-downloaded). Additive — never \
                             deletes anything on the box.",
                        )
                        .clicked()
                    {
                        self.sync.start_sync_pass();
                    }
                    let keep_changed = ui
                        .checkbox(&mut self.sync.ble_keep_synced, "Keep synced")
                        .on_hover_text(format!(
                            "Stay connected and re-sync every {} s so growing log \
                             files keep mirroring without clicking Sync now again. \
                             Each pass only fetches what grew.",
                            SYNC_POLL_INTERVAL.as_secs()
                        ))
                        .changed();
                    if keep_changed {
                        // Persist so the agent honours (or stops) the
                        // continuous mirror to match the GUI toggle.
                        self.sync.persist_config();
                    }
                    if keep_changed
                        && self.sync.ble_keep_synced
                        && connected
                        && !self.sync.ble_sync_pending
                        && !self.sync.ble_dl_in_flight
                    {
                        // Kick the first pass immediately on enable.
                        self.sync.start_sync_pass();
                    }
                    /* Box log mode (Auto/Manual) — iOS/Android parity.
                       This is the *box's* SET_MODE/GET_MODE state, not
                       the desktop sync: AUTO = box records on power-on,
                       MANUAL = box idle until Start session. The box is
                       the source of truth — a click sends SET_MODE and
                       the worker re-reads via GET_MODE, so the highlight
                       only flips once the box confirms. `None` (legacy
                       PumpTsueri or not yet answered) = neither lit. */
                    ui.separator();
                    ui.label("Log mode:");
                    let mode = self.sync.ble_log_mode;
                    if ui
                        .add_enabled(
                            connected,
                            egui::SelectableLabel::new(mode == Some(false), "Auto"),
                        )
                        .on_hover_text("Box opens a logging session automatically on every power-on. Persisted on the box's SD card.")
                        .clicked()
                        && mode != Some(false)
                    {
                        if let Some(b) = self.sync.ble.as_ref() {
                            b.send(BleCmd::SetLogMode { manual: false });
                        }
                        push_log(&self.log, "ble: SET_MODE auto sent".into());
                    }
                    if ui
                        .add_enabled(
                            connected,
                            egui::SelectableLabel::new(mode == Some(true), "Manual"),
                        )
                        .on_hover_text("Box stays idle after power-on until Start session (START_LOG). Persisted on the box's SD card.")
                        .clicked()
                        && mode != Some(true)
                    {
                        if let Some(b) = self.sync.ble.as_ref() {
                            b.send(BleCmd::SetLogMode { manual: true });
                        }
                        push_log(&self.log, "ble: SET_MODE manual sent".into());
                    }
                    ui.label(
                        egui::RichText::new(match mode {
                            None => "(unknown)",
                            Some(false) => "records on power-on",
                            Some(true) => "idle until Start session",
                        })
                        .weak(),
                    );
                    ui.separator();
                    if ui
                        .add_enabled(connected, egui::Button::new("STOP_LOG"))
                        .on_hover_text("Tells the box to gracefully close any active session — required before READ if logging is busy")
                        .clicked()
                    {
                        if let Some(b) = self.sync.ble.as_ref() { b.send(BleCmd::StopLog); }
                        /* STOP_LOG is a fire-and-forget side-channel write —
                           the firmware does NOT emit a FileData reply (see
                           `ble_filesync.c::OP_STOP_LOG`). Without an explicit
                           log line the user has no on-screen confirmation
                           the click registered, so push one here. */
                        push_log(&self.log, "ble: STOP_LOG sent".into());
                    }
                    /* Issue #15 — START_LOG triggers a LOG-mode session of
                     * `ble_session_duration_s` seconds. Box reboots into
                     * LOG mode, runs the logger for that long, then auto-
                     * reboots back to BLE mode. Connection drops during
                     * the LOG session — reconnect after to download files. */
                    ui.add_enabled(
                        connected,
                        egui::DragValue::new(&mut self.sync.ble_session_duration_s)
                            .speed(10)
                            .range(1..=86400)
                            .suffix(" s"),
                    ).on_hover_text("Session duration in seconds (1..86400)");
                    if ui
                        .add_enabled(connected, egui::Button::new("Start session"))
                        .on_hover_text("Box reboots into LOG mode and is INVISIBLE to Scan for the configured duration. After it expires the box auto-reboots back to BLE mode (~8 s) and reappears. Short-press the user button on the box to abort a session early.")
                        .clicked()
                    {
                        if let Some(b) = self.sync.ble.as_ref() {
                            b.send(BleCmd::StartLog { duration_seconds: self.sync.ble_session_duration_s });
                            /* The firmware will NVIC_SystemReset within
                               ~50 ms of receiving START_LOG — the BLE
                               controller resets too, so the connection
                               drops abruptly without LL_TERMINATE_IND.
                               CoreBluetooth then needs the full link
                               supervision timeout (6–30 s) to realise
                               the peripheral is gone, and during that
                               window the user can't cleanly reconnect.
                               Send an explicit Disconnect right after
                               START_LOG so the worker tears down the
                               Mac-side state proactively; the write
                               may race with the firmware reset but
                               either way Mac state becomes Idle. */
                            b.request_abort();
                            b.send(BleCmd::Disconnect);
                        }
                        /* Flip the GUI to Idle optimistically (same as
                           the Disconnect button) so Scan / Connect are
                           usable as soon as the box finishes booting
                           back into BLE mode. */
                        self.sync.ble_state = BleState::Idle;
                        self.sync.ble_files.clear();
                        self.sync.ble_dl_queue.clear();
                        self.sync.ble_dl_in_flight = false;
                        self.sync.ble_connected_id = None;
                        self.sync.ble_sync_pending = false;
                        self.sync.ble_resume_box = None; // intentional disconnect — don't auto-resume
                        self.sync.ble_status = format!("LOG session: {} s", self.sync.ble_session_duration_s);
                        /* Start the countdown banner. Stored as
                           (start_instant, duration_seconds) so each
                           frame can compute the remaining time without
                           depending on system clock drift. */
                        self.sync.ble_session_running = Some((
                            std::time::Instant::now(),
                            self.sync.ble_session_duration_s,
                        ));
                        push_log(
                            &self.log,
                            format!("ble: START_LOG sent ({} s) — box rebooting to LOG mode, GUI returning to Idle",
                                self.sync.ble_session_duration_s),
                        );
                    }
                    /* Disconnect button + BLE status moved to the top-right of
                       the tab strip (rendered in update(), under the logo).
                       Firmware upload + "Check FW" moved to the Firmware tab. */
                });

                // ----- Discovered devices --------------------------
                if !self.sync.ble_devices.is_empty()
                    && !matches!(self.sync.ble_state, BleState::Connected)
                {
                    ui.add_space(4.0);
                    ui.label("Discovered:");
                    let devices = self.sync.ble_devices.clone();
                    for d in devices {
                        ui.horizontal(|ui| {
                            ui.label(format!(
                                "{} [{}]",
                                d.name,
                                d.rssi.map(|r| format!("{r} dBm")).unwrap_or_else(|| "?".into())
                            ));
                            if ui.button("Connect").clicked() {
                                self.sync.ble_state = BleState::Connecting;
                                self.sync.ble_status = "connecting…".into();
                                /* Capture the box id now so the sync DB
                                   can key per-box; the worker's Connected
                                   event doesn't echo the id back. */
                                self.sync.ble_connected_id = Some(d.id.clone());
                                // Remember it across disconnect too, so the
                                // "Reconnect (last box)" button can retrieve it
                                // by id when CoreBluetooth scan-suppresses it.
                                self.sync.ble_last_box_id = Some(d.id.clone());
                                // Persist the box id so the headless
                                // agent reconnects to this same box.
                                self.sync.persist_config();
                                if let Some(b) = self.sync.ble.as_ref() {
                                    b.send(BleCmd::Connect(d.id.clone()));
                                }
                            }
                        });
                    }
                }

                /* Surface a bad box boot right where the user syncs: the
                   errlog auto-check (Debug tab has the details) graded
                   the newest boot section of the mirrored log. Rendered
                   regardless of connection state — the report also
                   exists offline (startup check on the local mirror).
                   Quiet when the latest boot is healthy. */
                if let Some(last) = self
                    .sync
                    .errlog_report
                    .as_ref()
                    .and_then(|r| r.latest())
                    .filter(|b| b.verdict() != errlog_check::Severity::Info)
                {
                    ui.add_space(2.0);
                    let col = match last.verdict() {
                        errlog_check::Severity::Fail => egui::Color32::from_rgb(235, 90, 90),
                        _ => egui::Color32::from_rgb(230, 175, 60),
                    };
                    ui.colored_label(
                        col,
                        format!(
                            "Box health: latest boot {} — see Debug tab",
                            last.verdict_label()
                        ),
                    );
                }

                // ----- File list / download ------------------------
                if matches!(self.sync.ble_state, BleState::Connected) {
                    if !self.sync.ble_sync_msg.is_empty() {
                        ui.add_space(4.0);
                        ui.colored_label(
                            egui::Color32::from_rgb(120, 200, 140),
                            &self.sync.ble_sync_msg,
                        );
                    }
                    // Cumulative byte-progress bar for the current sync
                    // pass (iOS/Android parity, 2d001a4). Per-row bars
                    // in the file list below still show individual file
                    // progress; this is the roll-up so the user can see
                    // "8 of 23 files, 142 / 312 MB, 45 %" at a glance.
                    if self.sync.sync_pass_total > 0 {
                        let total = self.sync.sync_pass_total;
                        let remaining = self.sync.ble_dl_queue.len()
                            + (self.sync.ble_dl_in_flight as usize);
                        let done = total.saturating_sub(remaining);
                        let frac = self.sync.sync_cumulative_fraction();
                        let bytes = self.sync.sync_cumulative_bytes();
                        let total_bytes = self.sync.sync_pass_total_bytes;
                        let fmt_b = |b: u64| -> String {
                            if b >= 1024 * 1024 {
                                format!("{:.1} MB", b as f64 / 1_048_576.0)
                            } else if b >= 1024 {
                                format!("{:.0} kB", b as f64 / 1024.0)
                            } else {
                                format!("{b} B")
                            }
                        };
                        ui.add_space(4.0);
                        ui.add(
                            egui::ProgressBar::new(frac)
                                .desired_width(360.0)
                                .text(format!(
                                    "{done} of {total} files · {} / {} ({}%)",
                                    fmt_b(bytes),
                                    fmt_b(total_bytes),
                                    (frac * 100.0) as u32,
                                )),
                        );
                    }
                    /* Firmware-upload progress bar + result line and the
                       "Check FW" status live in the Firmware tab now
                       (ui_firmware_tab). */
                    if let Some(err) = self.sync.ble_delete_err.clone() {
                        ui.add_space(4.0);
                        egui::Frame::group(ui.style())
                            .fill(egui::Color32::from_rgb(255, 230, 230))
                            .stroke(egui::Stroke::new(
                                1.0,
                                egui::Color32::from_rgb(200, 80, 80),
                            ))
                            .show(ui, |ui| {
                                ui.horizontal(|ui| {
                                    ui.colored_label(
                                        egui::Color32::from_rgb(170, 30, 30),
                                        format!("⚠ {err}"),
                                    );
                                    if ui.small_button("Dismiss").clicked() {
                                        self.sync.ble_delete_err = None;
                                    }
                                });
                            });
                    }
                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        ui.label("Save to:");
                        let mut s = self.sync.ble_out_dir.display().to_string();
                        if ui.text_edit_singleline(&mut s).changed() {
                            self.sync.ble_out_dir = PathBuf::from(s);
                            // Drop the stale probe error while the user is
                            // mid-edit; it's re-checked on the next attempt.
                            self.sync.ble_save_err = None;
                        }
                        if ui.button("Browse…").clicked() {
                            if let Some(p) = rfd::FileDialog::new().pick_folder() {
                                self.sync.ble_out_dir = p;
                                self.sync.ble_save_err = None;
                            }
                        }
                    });
                    /* Inline path feedback. resolve_save_dir is pure path
                       math (no FS), cheap to run every frame — it catches
                       empty / relative / ~ paths the instant they're typed.
                       The deeper writability/TCC failure only shows after a
                       Download/Sync attempt (probe needs real FS access);
                       that lands in ble_save_err. */
                    let red = egui::Color32::from_rgb(225, 90, 90);
                    match resolve_save_dir(&self.sync.ble_out_dir) {
                        Err(e) => {
                            ui.colored_label(red, format!("⚠ {e}"));
                        }
                        Ok(abs) => {
                            if let Some(err) = &self.sync.ble_save_err {
                                ui.colored_label(red, format!("⚠ {err}"));
                            } else if abs != self.sync.ble_out_dir {
                                ui.label(
                                    egui::RichText::new(format!("-> {}", abs.display())),
                                );
                            }
                        }
                    }

                    if self.sync.ble_files.is_empty() {
                        ui.label(
                            egui::RichText::new("No file list yet — hit Refresh."),
                        );
                    } else {
                        ui.add_space(4.0);
                        /* `delete_target` is set inside the per-row loop
                           when the user clicks the trash button. We can't
                           send to the BLE backend from inside the row
                           closure because that path borrows self.sync.ble_files
                           mutably; defer the send until after the loop. */
                        let mut delete_target: Option<String> = None;
                        /* Split rows by index into two groups so the user
                           sees session data (Sensor) above noise (Debug)
                           and can mass-tick either header. Indices stay
                           valid for the lifetime of this frame because we
                           don't mutate self.sync.ble_files in between. */
                        let mut sensor_idx: Vec<usize> = Vec::new();
                        let mut debug_idx:  Vec<usize> = Vec::new();
                        for (i, f) in self.sync.ble_files.iter().enumerate() {
                            if is_sensor_data_name(&f.name) {
                                sensor_idx.push(i);
                            } else {
                                debug_idx.push(i);
                            }
                        }
                        // Newest session first so the latest recording is
                        // at the top — no scrolling to the end of the list.
                        let files = &self.sync.ble_files;
                        let by_recency_desc = |a: &usize, b: &usize| {
                            recency_key(&files[*b].name)
                                .cmp(&recency_key(&files[*a].name))
                        };
                        sensor_idx.sort_by(by_recency_desc);
                        debug_idx.sort_by(by_recency_desc);

                        // Fill the remaining vertical space with the file
                        // list, reserving ~40 px for the "Download selected"
                        // button below. Adapts to window height; a floor keeps
                        // it usable when the window is very short.
                        let list_height = (ui.available_height() - 40.0).max(120.0);
                        egui::ScrollArea::vertical()
                            .max_height(list_height)
                            // Use the full window width for the list (don't
                            // shrink to content, which left-hugged it); still
                            // shrink height to content up to max_height.
                            .auto_shrink([false, true])
                            .id_salt("ble-file-list")
                            .show(ui, |ui| {
                                render_file_group(
                                    ui,
                                    "Sensor",
                                    &sensor_idx,
                                    &mut self.sync.ble_files,
                                    &mut delete_target,
                                );
                                ui.add_space(4.0);
                                render_file_group(
                                    ui,
                                    "Debug",
                                    &debug_idx,
                                    &mut self.sync.ble_files,
                                    &mut delete_target,
                                );
                            });
                        /* Defer the BLE send out of the row closure to keep
                           the borrow of self.sync.ble_files local. The firmware
                           rejects DELETE while logging is active (BUSY); the
                           error surfaces via BleEvent::Error in the log. */
                        if let Some(name) = delete_target {
                            if let Some(b) = self.sync.ble.as_ref() {
                                b.send(BleCmd::Delete { name: name.clone() });
                                self.sync.ble_delete_err = None; // clear stale failure on a fresh attempt
                                push_log(&self.log, format!("ble: deleting {name}"));
                            }
                        }

                        ui.add_space(4.0);
                        let queue_len = self.sync.ble_dl_queue.len();
                        let any_in_flight = self.sync.ble_dl_in_flight;
                        let dl_label = if any_in_flight || queue_len > 0 {
                            format!("Download selected ({} queued)", queue_len + (any_in_flight as usize))
                        } else {
                            "Download selected".to_string()
                        };
                        if ui.add_enabled(!any_in_flight, egui::Button::new(dl_label)).clicked() {
                            /* Validate the Save-to folder *before* queuing
                               anything. A bad path used to slip through and
                               every read would fail its save silently; now
                               the user gets a red banner and nothing is
                               enqueued. */
                            if self.sync.validate_save_dir().is_some() {
                                /* Queue all ticked, not-yet-downloaded files
                                   serially. The first one is sent immediately
                                   by advance_download_queue(); the rest start
                                   on each ReadDone / READ-error event. */
                                self.sync.ble_dl_queue.clear();
                                for f in self.sync.ble_files.iter() {
                                    if f.selected && !f.downloaded {
                                        self.sync.ble_dl_queue.push_back((f.name.clone(), f.size));
                                    }
                                }
                                self.sync.advance_download_queue();
                            }
                        }
                    }
                }
            }
        }
    }
}

impl AppState {
    /// Live tab — renders the most recent SensorStream snapshot from
    /// the connected box. Gated on already being connected via the
    /// Sync tab (since Connect is a multi-step flow with scan results
    /// and per-device buttons that don't fit nicely above a 6-row
    /// readout). The 0.5 Hz cadence is fixed in firmware, so this
    /// panel updates exactly twice a second.
    /// "GPS Debug" tab: drive the bundled `gps-debug` sidecar against a
    /// u-blox on a serial port and stream its live readout into a panel.
    /// For antenna selection + mounting evaluation — fix quality,
    /// per-signal C/N0, RF/antenna health. CSVs land in a `gps-debug`
    /// subfolder of the configured save dir.
    /// Intentional disconnect: break any in-progress auto-reconnect (the
    /// worker only polls the abort flag while inside it), send Disconnect,
    /// and optimistically drop to Idle so Scan re-enables immediately (the
    /// worker's later, idempotent Disconnected event confirms it). Shared by
    /// the top-bar Disconnect button.
    fn disconnect_box(&mut self) {
        if let Some(b) = self.sync.ble.as_ref() {
            b.request_abort();
            b.send(BleCmd::Disconnect);
        }
        self.sync.ble_state = BleState::Idle;
        self.sync.ble_files.clear();
        self.sync.ble_dl_queue.clear();
        self.sync.ble_dl_in_flight = false;
        self.sync.ble_connected_id = None;
        self.sync.ble_sync_pending = false;
        self.sync.ble_resume_box = None; // intentional — don't auto-resume
        self.sync.ble_status = "disconnecting…".into();
        push_log(&self.log, "ble: Disconnect requested".into());
    }

    /// Firmware tab: manual `.bin` upload + one-click GitHub auto-update
    /// ("Check FW"), with the flash progress + result. Moved out of the Sync
    /// tab so the box-update tools have their own home. Needs a connected box.
    fn ui_firmware_tab(&mut self, ui: &mut egui::Ui) {
        ui.heading("Box firmware");
        ui.add_space(6.0);
        let connected = matches!(self.sync.ble_state, BleState::Connected);
        if !connected {
            ui.label(
                "Connect to a box in the Sync tab first — the firmware update \
                 tools appear here once a box is connected.",
            );
            return;
        }
        let worker_busy = self.sync.ble_dl_in_flight
            || self.sync.ble_sync_pending
            || !self.sync.ble_dl_queue.is_empty()
            || self.sync.ble_flash_progress.is_some();

        ui.horizontal(|ui| {
            /* One-click auto-update: fetch the latest firmware-vX.Y.Z.bin from
               GitHub, ask the box its version over BLE, flash if the box is
               older (or legacy/unknown). */
            if ui
                .add_enabled(
                    !worker_busy && !self.sync.fw_check_active,
                    egui::Button::new("Check FW"),
                )
                .on_hover_text(
                    "Check GitHub for the latest box firmware and compare it \
                     with the connected box's version. If the box is older (or \
                     its firmware is too old to report a version), the newer \
                     image is downloaded and flashed over BLE automatically.",
                )
                .clicked()
            {
                self.sync.start_fw_check();
            }
            /* Manual firmware upload (dual-bank OTA): pick a `.bin`, stream it
               over the FileCmd characteristic; the box verifies the SHA-256 on
               COMMIT, swaps banks, and reboots. */
            if ui
                .add_enabled(!worker_busy, egui::Button::new("Upload firmware…"))
                .on_hover_text(
                    "Send a firmware .bin to the box over BLE. The box verifies \
                     the image (SHA-256), swaps to the inactive flash bank, and \
                     reboots into the new firmware — the connection drops, then \
                     reconnect. The current firmware is untouched unless the \
                     upload fully succeeds.",
                )
                .clicked()
            {
                if let Some(p) = rfd::FileDialog::new()
                    .add_filter("firmware", &["bin"])
                    .pick_file()
                {
                    self.sync.start_firmware_upload(&p);
                }
            }
        });

        // Firmware-upload progress bar (advances during the multi-minute flash).
        if let Some((done, total)) = self.sync.ble_flash_progress {
            let frac = if total > 0 {
                (done as f32 / total as f32).clamp(0.0, 1.0)
            } else {
                0.0
            };
            ui.add_space(6.0);
            ui.add(
                egui::ProgressBar::new(frac).desired_width(360.0).text(format!(
                    "Firmware {} / {} kB ({}%)",
                    (done + 1023) / 1024,
                    (total + 1023) / 1024,
                    (frac * 100.0) as u32,
                )),
            );
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(200));
        }
        // "Check FW" status ("Updating box to vX.Y.Z…" / "up to date" / error).
        if !self.sync.fw_check_msg.is_empty() {
            ui.add_space(6.0);
            let is_err = self.sync.fw_check_msg.contains("Couldn't")
                || self.sync.fw_check_msg.contains("empty");
            ui.colored_label(
                if is_err {
                    egui::Color32::from_rgb(225, 90, 90)
                } else {
                    egui::Color32::from_rgb(120, 200, 140)
                },
                &self.sync.fw_check_msg,
            );
            if self.sync.fw_check_active {
                ui.ctx().request_repaint_after(std::time::Duration::from_millis(200));
            }
        }
        // Flash result line.
        if !self.sync.ble_flash_msg.is_empty() {
            ui.add_space(2.0);
            let is_err = self.sync.ble_flash_msg.contains("failed")
                || self.sync.ble_flash_msg.contains("Can't")
                || self.sync.ble_flash_msg.contains("empty");
            ui.colored_label(
                if is_err {
                    egui::Color32::from_rgb(225, 90, 90)
                } else {
                    egui::Color32::from_rgb(120, 200, 140)
                },
                &self.sync.ble_flash_msg,
            );
        }
    }

    /// Debug tab: BLE probe section on top, GPS/antenna survey below —
    /// every "run this and send me the output" diagnostic in one place.
    /// "Box health (ERRLOG.LOG)" section at the top of the Debug tab:
    /// renders the automatic per-boot grading of the mirrored box error
    /// log (`errlog_check`). The report refreshes itself — at startup
    /// and after every sync pass that pulls ERRLOG.LOG — so each new
    /// box boot appears here without any manual step.
    fn ui_box_health(&mut self, ui: &mut egui::Ui) {
        fn color(v: errlog_check::Severity) -> egui::Color32 {
            match v {
                errlog_check::Severity::Fail => egui::Color32::from_rgb(235, 90, 90),
                errlog_check::Severity::Warn => egui::Color32::from_rgb(230, 175, 60),
                errlog_check::Severity::Info => egui::Color32::from_rgb(110, 200, 110),
            }
        }
        ui.heading("Box health (ERRLOG.LOG)");
        ui.label(egui::RichText::new(
            "Automatic per-boot check of the box error log — re-runs after every sync.",
        ).weak());
        ui.add_space(4.0);
        // Deferred: the button sits inside the errlog_report borrow, so
        // the actual re-check runs after the match releases it.
        let mut recheck = false;
        match self.sync.errlog_report.as_ref() {
            None => {
                ui.horizontal(|ui| {
                    recheck = ui.button("Re-check now").clicked();
                    if let Some(t) = &self.sync.errlog_checked_at {
                        ui.label(egui::RichText::new(format!("checked {t}")).weak());
                    }
                    ui.label(
                        "No ERRLOG.LOG mirror yet — connect in the Sync tab and run one sync.",
                    );
                });
            }
            Some(rep) => {
                if let Some(last) = rep.latest() {
                    let verdict = last.verdict();
                    ui.horizontal(|ui| {
                        recheck = ui.button("Re-check now").clicked();
                        if let Some(t) = &self.sync.errlog_checked_at {
                            ui.label(egui::RichText::new(format!("checked {t}")).weak());
                        }
                        ui.label("Latest boot:");
                        ui.colored_label(
                            color(verdict),
                            format!("#{}  {}", last.index, last.verdict_label()),
                        );
                        // Boot wall-clock (HH:MM-DD.MM.YYYY) from the host
                        // SET_TIME anchor — the box has no RTC, so this is
                        // blank ("time?") on boots with no host connect.
                        match last.boot_time_label() {
                            Some(t) => {
                                ui.colored_label(egui::Color32::LIGHT_BLUE, t);
                            }
                            None => {
                                ui.label(egui::RichText::new("time?").weak());
                            }
                        }
                        ui.label(
                            egui::RichText::new(format!(
                                "up {:.0} s · reset: {} · {}",
                                last.last_tick_ms as f64 / 1000.0,
                                if last.reset.is_empty() { "?" } else { &last.reset },
                                last.fw,
                            ))
                            .weak(),
                        );
                    });
                    // Findings collapsed behind a header so the section
                    // stays compact; auto-open only when something is
                    // actually wrong with the latest boot.
                    egui::CollapsingHeader::new(format!(
                        "Latest boot findings ({})",
                        last.findings.len()
                    ))
                    .id_salt("errlog_latest_findings")
                    .default_open(verdict != errlog_check::Severity::Info)
                    .show(ui, |ui| {
                        for f in &last.findings {
                            ui.horizontal(|ui| {
                                ui.colored_label(color(f.severity), "•");
                                ui.label(&f.msg);
                            });
                        }
                    });
                }
                egui::CollapsingHeader::new(format!(
                    "Boot history ({} boots)",
                    rep.boots.len()
                ))
                    .id_salt("errlog_boot_history")
                    .show(ui, |ui| {
                        // Newest first; the full log is still available
                        // via --check-errlog for anything older.
                        for b in rep.boots.iter().rev().take(25) {
                            ui.horizontal(|ui| {
                                ui.colored_label(
                                    color(b.verdict()),
                                    format!("#{:<3} {}", b.index, b.verdict_label()),
                                );
                                match b.boot_time_label() {
                                    Some(t) => {
                                        ui.colored_label(egui::Color32::LIGHT_BLUE, t);
                                    }
                                    None => {
                                        ui.label(egui::RichText::new("time?").weak());
                                    }
                                }
                                ui.label(
                                    egui::RichText::new(format!(
                                        "up {:.0} s · {}",
                                        b.last_tick_ms as f64 / 1000.0,
                                        if b.reset.is_empty() { "?" } else { &b.reset },
                                    ))
                                    .weak(),
                                );
                            });
                            for f in b.findings.iter().filter(|f| {
                                f.severity != errlog_check::Severity::Info
                                    || b.verdict() == errlog_check::Severity::Info
                            }) {
                                ui.horizontal(|ui| {
                                    ui.add_space(18.0);
                                    ui.colored_label(color(f.severity), "•");
                                    ui.label(&f.msg);
                                });
                            }
                        }
                    });
            }
        }
        if recheck {
            self.sync.run_errlog_check("manual");
        }
        ui.add_space(10.0);
        ui.separator();
        ui.add_space(6.0);
    }

    fn ui_debug_tab(&mut self, ui: &mut egui::Ui) {
        /* The tab stacks three sections (Box health, BLE Debug, GPS
           Debug) that together outgrow the window — scroll the whole
           tab so no section can clip off-screen (the BLE debug log was
           unreachable once Box health landed above it). The BLE/GPS
           output panels keep their own inner scroll areas; egui routes
           the wheel to the hovered one. */
        egui::ScrollArea::vertical()
            .id_salt("debug_tab_scroll")
            .auto_shrink([false; 2])
            .show(ui, |ui| self.ui_debug_tab_inner(ui));
    }

    fn ui_debug_tab_inner(&mut self, ui: &mut egui::Ui) {
        self.ui_box_health(ui);
        ui.heading("BLE Debug");
        ui.label(
            egui::RichText::new(
                "Live CoreBluetooth diagnostics: prints every discovery event, \
                 checks the saved box id, then runs a bounded connect probe \
                 (connect -> subscribe -> LIST -> reply count). The output is \
                 also saved as a .log file in the download folder — send that \
                 file when reporting connection problems.",
            ),
        );
        ui.add_space(6.0);

        let running = self.ble_dbg_running.load(Ordering::SeqCst);
        let ble_busy = matches!(
            self.sync.ble_state,
            BleState::Connected | BleState::Connecting
        );
        ui.horizontal(|ui| {
            ui.label("Scan window (s)");
            ui.add_enabled(
                !running,
                egui::TextEdit::singleline(&mut self.ble_dbg_secs).desired_width(48.0),
            );
            let run = ui
                .add_enabled(!running && !ble_busy, egui::Button::new("▶ Run BLE debug"))
                .on_disabled_hover_text(if ble_busy {
                    "Disconnect from the box first — the probe needs the box advertising."
                } else {
                    "A probe is already running."
                });
            if run.clicked() {
                self.start_ble_debug();
            }
            if ui
                .add_enabled(running, egui::Button::new("⏹ Stop"))
                .clicked()
            {
                self.ble_dbg_cancel.store(true, Ordering::SeqCst);
            }
            if running {
                ui.spinner();
                ui.colored_label(egui::Color32::LIGHT_GREEN, "probing…");
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui
                    .add_enabled(!running, egui::Button::new("Clear log"))
                    .clicked()
                {
                    if let Ok(mut v) = self.ble_dbg_log.lock() {
                        v.clear();
                    }
                }
                if ui
                    .button("Copy")
                    .on_hover_text("Copy the whole probe output to the clipboard")
                    .clicked()
                {
                    let text = self
                        .ble_dbg_log
                        .lock()
                        .map(|v| v.join("\n"))
                        .unwrap_or_default();
                    ui.output_mut(|o| o.copied_text = text);
                }
            });
        });

        // Fixed-height output panel so the GPS section below keeps room.
        let row_h = ui.text_style_height(&egui::TextStyle::Monospace).max(1.0);
        let panel_h = row_h * 14.0;
        egui::ScrollArea::vertical()
            .id_salt("ble_dbg_scroll")
            .max_height(panel_h)
            .stick_to_bottom(true)
            .show(ui, |ui| {
                let mut owned = self
                    .ble_dbg_log
                    .lock()
                    .map(|v| v.join("\n"))
                    .unwrap_or_default();
                ui.add(
                    egui::TextEdit::multiline(&mut owned)
                        .desired_width(f32::INFINITY)
                        .desired_rows(14)
                        .font(egui::TextStyle::Monospace),
                );
            });
        if running {
            ui.ctx()
                .request_repaint_after(std::time::Duration::from_millis(200));
        }

        ui.add_space(8.0);
        ui.separator();
        ui.add_space(8.0);

        self.ui_gps_debug_tab(ui);
    }

    /// Spawn `MovementLogger --ble-debug <secs>` as a child (same signed
    /// binary → same Bluetooth TCC grant) and pump its stdout/stderr into
    /// `ble_dbg_log` AND a timestamped .log file in the download (CSV)
    /// folder, so the user can send the file as-is. Mirrors the
    /// `start_gps_debug` child pattern.
    fn start_ble_debug(&mut self) {
        if self.ble_dbg_running.load(Ordering::SeqCst) {
            return;
        }
        let secs: u64 = self.ble_dbg_secs.trim().parse().unwrap_or(20);
        self.ble_dbg_secs = secs.to_string();

        let out_dir = resolve_save_dir(&self.sync.ble_out_dir)
            .unwrap_or_else(|_| self.sync.ble_out_dir.clone());
        let _ = std::fs::create_dir_all(&out_dir);
        let stamp = chrono::Local::now().format("%Y-%m-%d_%H%M%S");
        let out_path = out_dir.join(format!("ble_debug_{stamp}.log"));

        let exe = std::env::current_exe()
            .unwrap_or_else(|_| std::path::PathBuf::from("MovementLogger"));
        let mut cmd = Command::new(&exe);
        cmd.arg("--ble-debug")
            .arg(secs.to_string())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());

        push_log(
            &self.ble_dbg_log,
            format!("$ {} --ble-debug {}", exe.display(), secs),
        );
        push_log(&self.ble_dbg_log, format!("saving to {}", out_path.display()));

        let mut child = match cmd.spawn() {
            Ok(c) => c,
            Err(e) => {
                push_log(&self.ble_dbg_log, format!("error: failed to spawn probe: {e}"));
                return;
            }
        };

        self.ble_dbg_cancel.store(false, Ordering::SeqCst);
        self.ble_dbg_running.store(true, Ordering::SeqCst);
        let log = self.ble_dbg_log.clone();
        let cancel = self.ble_dbg_cancel.clone();
        let running = self.ble_dbg_running.clone();

        let (tx, rx) = mpsc::channel::<String>();
        if let Some(out) = child.stdout.take() {
            let tx = tx.clone();
            thread::spawn(move || pump_pipe(out, tx));
        }
        if let Some(err) = child.stderr.take() {
            let tx = tx.clone();
            thread::spawn(move || pump_pipe(err, tx));
        }
        drop(tx);

        thread::spawn(move || {
            use std::io::Write;
            let mut file = std::fs::File::create(&out_path).ok();
            for line in rx {
                if let Some(f) = file.as_mut() {
                    let _ = writeln!(f, "{line}");
                }
                push_log(&log, line);
            }
            if let Some(f) = file.as_mut() {
                let _ = f.flush();
            }
            let mut cancelled = false;
            loop {
                match child.try_wait() {
                    Ok(Some(status)) => {
                        let msg = if cancelled {
                            "--- stopped ---".to_string()
                        } else if status.success() {
                            format!("--- done — saved {} ---", out_path.display())
                        } else {
                            format!("--- exited with {status} ---")
                        };
                        push_log(&log, msg);
                        break;
                    }
                    Ok(None) => {
                        if cancel.load(Ordering::SeqCst) && !cancelled {
                            let _ = child.kill();
                            cancelled = true;
                        }
                        thread::sleep(std::time::Duration::from_millis(100));
                    }
                    Err(e) => {
                        push_log(&log, format!("error: try_wait failed: {e}"));
                        break;
                    }
                }
            }
            running.store(false, Ordering::SeqCst);
        });
    }

    fn ui_gps_debug_tab(&mut self, ui: &mut egui::Ui) {
        ui.heading("GPS Debug");
        ui.label(
            egui::RichText::new(
                "Live u-blox UBX diagnostics for antenna selection + mounting: fix \
                 quality (DOP, accuracy), per-signal C/N0, and RF/antenna health \
                 (antStatus, AGC, jamming). Polls the receiver once a second and \
                 writes CSVs. Read-only — it never reconfigures the receiver.",
            ),
        );
        ui.add_space(6.0);

        let running = self.gps_dbg_running.load(Ordering::SeqCst);

        // Transport: USB serial cable vs the box's BLE link.
        ui.horizontal(|ui| {
            ui.label("Transport");
            ui.add_enabled_ui(!running, |ui| {
                ui.selectable_value(&mut self.gps_dbg_transport, GpsTransport::Usb, "USB serial");
                ui.selectable_value(&mut self.gps_dbg_transport, GpsTransport::Ble, "BLE (box bridge)");
            });
        });
        ui.add_space(4.0);

        egui::Grid::new("gps_dbg_form")
            .num_columns(2)
            .spacing([8.0, 6.0])
            .show(ui, |ui| {
                if self.gps_dbg_transport == GpsTransport::Usb {
                    ui.label("Serial port");
                    ui.horizontal(|ui| {
                        // Dropdown of auto-detected ports; falls back to free
                        // text when none are found or the user wants another.
                        let current = if self.gps_dbg_port.is_empty() {
                            "(none)".to_string()
                        } else {
                            self.gps_dbg_port.clone()
                        };
                        ui.add_enabled_ui(!running, |ui| {
                            egui::ComboBox::from_id_salt("gps_dbg_port_combo")
                                .selected_text(current)
                                .width(300.0)
                                .show_ui(ui, |ui| {
                                    for p in &self.gps_dbg_detected_ports {
                                        ui.selectable_value(&mut self.gps_dbg_port, p.clone(), p);
                                    }
                                });
                            if ui.button("🔄 Detect").on_hover_text(
                                "Re-scan for u-blox serial ports (macOS /dev/cu.usb*, Linux ttyACM*/ttyUSB*)",
                            ).clicked() {
                                self.gps_dbg_detected_ports = gps_debug::detect_ports();
                                match self.gps_dbg_detected_ports.as_slice() {
                                    [] => push_log(&self.gps_dbg_log,
                                        "detect: no serial ports found (is the cable plugged in?)".into()),
                                    [one] => {
                                        self.gps_dbg_port = one.clone();
                                        push_log(&self.gps_dbg_log, format!("detect: {one}"));
                                    }
                                    many => {
                                        if self.gps_dbg_port.is_empty() {
                                            self.gps_dbg_port = many[0].clone();
                                        }
                                        push_log(&self.gps_dbg_log,
                                            format!("detect: {} ports — {}", many.len(), many.join(", ")));
                                    }
                                }
                            }
                        });
                    });
                    ui.end_row();
                    ui.label("");
                    ui.add_enabled(
                        !running,
                        egui::TextEdit::singleline(&mut self.gps_dbg_port)
                            .hint_text("…or type: /dev/cu.usbserial-XXXX · /dev/ttyACM0 · COM3")
                            .desired_width(340.0),
                    );
                    ui.end_row();
                    ui.label("Baud");
                    ui.add_enabled(
                        !running,
                        egui::TextEdit::singleline(&mut self.gps_dbg_baud).desired_width(100.0),
                    );
                    ui.end_row();
                } else {
                    ui.label("Source");
                    ui.label(
                        egui::RichText::new(
                            "the connected box (no cable — bridges the u-blox over BLE)",
                        ),
                    );
                    ui.end_row();
                }
                ui.label("Label");
                ui.add_enabled(
                    !running,
                    egui::TextEdit::singleline(&mut self.gps_dbg_label)
                        .hint_text("antenna-A")
                        .desired_width(200.0),
                );
                ui.end_row();
            });

        ui.add_space(6.0);
        ui.horizontal(|ui| {
            if ui
                .add_enabled(!running, egui::Button::new("▶ Start"))
                .clicked()
            {
                self.start_gps_debug();
            }
            if ui
                .add_enabled(running, egui::Button::new("■ Stop"))
                .clicked()
            {
                self.gps_dbg_cancel.store(true, Ordering::SeqCst);
            }
            if running {
                ui.spinner();
                ui.colored_label(egui::Color32::LIGHT_GREEN, "polling…");
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui
                    .add_enabled(!running, egui::Button::new("Clear log"))
                    .clicked()
                {
                    if let Ok(mut v) = self.gps_dbg_log.lock() {
                        v.clear();
                    }
                }
                // Copy the whole survey output — parity with the BLE
                // Debug panel; the per-second summary lines are exactly
                // what gets pasted into an issue report.
                if ui
                    .button("Copy")
                    .on_hover_text("Copy the whole survey output to the clipboard")
                    .clicked()
                {
                    let text = self
                        .gps_dbg_log
                        .lock()
                        .map(|v| v.join("\n"))
                        .unwrap_or_default();
                    ui.output_mut(|o| o.copied_text = text);
                }
            });
        });

        let hint = match self.gps_dbg_transport {
            GpsTransport::Usb =>
                "USB: 🔄 Detect auto-fills the port (macOS /dev/cu.usb*, Linux ttyACM*/ttyUSB*) \
                 or type it. Baud 38400 matches the box's MAX-M10S; a bare module is often 9600.",
            GpsTransport::Ble =>
                "BLE: connect to the box in the Sync tab first, then Start — the survey tunnels \
                 the u-blox over the box's link (no cable). Needs box firmware with the GPS-bridge \
                 opcode; on older firmware you'll just see \"no NAV-PVT reply\".",
        };
        ui.label(
            egui::RichText::new(hint),
        );

        ui.separator();

        // ----- live output panel (monospace, auto-scroll) ------------
        let row_h = ui.text_style_height(&egui::TextStyle::Monospace).max(1.0);
        let avail_h = ui.available_height().max(row_h * 4.0);
        let rows = (avail_h / row_h) as usize;
        egui::ScrollArea::vertical()
            .max_height(avail_h)
            .stick_to_bottom(true)
            .show(ui, |ui| {
                let lines = self
                    .gps_dbg_log
                    .lock()
                    .map(|v| v.clone())
                    .unwrap_or_default();
                let mut owned = lines.join("\n");
                ui.add(
                    egui::TextEdit::multiline(&mut owned)
                        .desired_width(f32::INFINITY)
                        .desired_rows(rows.max(4))
                        .font(egui::TextStyle::Monospace),
                );
            });

        // Keep the UI ticking while the child streams so new lines render
        // without waiting on a mouse/keyboard event.
        if running {
            ui.ctx()
                .request_repaint_after(std::time::Duration::from_millis(200));
        }
    }

    /// Spawn the `gps-debug` sidecar with the current form values and pump
    /// its stdout/stderr into `gps_dbg_log`. Mirrors the animate-run worker:
    /// two pipe-reader threads → mpsc → log, plus a reaper that honours the
    /// Stop flag (`gps_dbg_cancel`) and clears `gps_dbg_running` on exit.
    fn start_gps_debug(&mut self) {
        if self.gps_dbg_running.load(Ordering::SeqCst) {
            return;
        }
        if self.gps_dbg_transport == GpsTransport::Ble {
            self.start_gps_debug_ble();
            return;
        }
        let port = self.gps_dbg_port.trim().to_string();
        if port.is_empty() {
            push_log(
                &self.gps_dbg_log,
                "error: enter a serial port first \
                 (e.g. /dev/cu.usbserial-XXXX, /dev/ttyACM0, COM3)."
                    .into(),
            );
            return;
        }
        let baud: u32 = self.gps_dbg_baud.trim().parse().unwrap_or(38400);
        let label = {
            let l = self.gps_dbg_label.trim();
            if l.is_empty() { "antenna".to_string() } else { l.to_string() }
        };
        // CSVs go into a `gps-debug` subfolder of the configured save dir.
        let out_dir = match resolve_save_dir(&self.sync.ble_out_dir) {
            Ok(abs) => abs.join("gps-debug"),
            Err(_) => self.sync.ble_out_dir.join("gps-debug"),
        };
        let _ = std::fs::create_dir_all(&out_dir);

        let exe = gps_debug_path();
        let mut cmd = Command::new(&exe);
        cmd.arg("--port").arg(&port)
            .arg("--baud").arg(baud.to_string())
            .arg("--label").arg(&label)
            .arg("--output").arg(&out_dir)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());

        push_log(
            &self.gps_dbg_log,
            format!(
                "$ {} --port {} --baud {} --label {} --output {}",
                exe.display(), port, baud, label, out_dir.display()
            ),
        );

        let mut child = match cmd.spawn() {
            Ok(c) => c,
            Err(e) => {
                push_log(
                    &self.gps_dbg_log,
                    format!(
                        "error: failed to spawn gps-debug: {e}. \
                         Make sure it sits next to MovementLogger or is on PATH."
                    ),
                );
                return;
            }
        };

        self.gps_dbg_cancel.store(false, Ordering::SeqCst);
        self.gps_dbg_running.store(true, Ordering::SeqCst);
        let log = self.gps_dbg_log.clone();
        let cancel = self.gps_dbg_cancel.clone();
        let running = self.gps_dbg_running.clone();

        let (tx, rx) = mpsc::channel::<String>();
        if let Some(out) = child.stdout.take() {
            let tx = tx.clone();
            thread::spawn(move || pump_pipe(out, tx));
        }
        if let Some(err) = child.stderr.take() {
            let tx = tx.clone();
            thread::spawn(move || pump_pipe(err, tx));
        }
        drop(tx);

        thread::spawn(move || {
            for line in rx {
                push_log(&log, line);
            }
            let mut cancelled = false;
            loop {
                match child.try_wait() {
                    Ok(Some(status)) => {
                        let msg = if cancelled {
                            "--- stopped ---".to_string()
                        } else if status.success() {
                            "--- done ---".to_string()
                        } else {
                            format!("--- exited with {status} ---")
                        };
                        push_log(&log, msg);
                        break;
                    }
                    Ok(None) => {
                        if cancel.load(Ordering::SeqCst) && !cancelled {
                            let _ = child.kill();
                            cancelled = true;
                        }
                        thread::sleep(std::time::Duration::from_millis(100));
                    }
                    Err(e) => {
                        push_log(&log, format!("error: try_wait failed: {e}"));
                        break;
                    }
                }
            }
            running.store(false, Ordering::SeqCst);
        });
    }

    /// BLE-transport GPS survey: runs the same `gps_debug::run_core` UBX
    /// poll/parse loop in-process, but tunnels the u-blox UART through the
    /// connected box's BLE link (firmware GPS-bridge opcode `0x0D`/`0x0E`) —
    /// no cable, no opening the box. The box keeps logging NMEA to its SD
    /// card; only UBX poll replies ride the bridge. Requires box firmware
    /// with the bridge opcode; legacy firmware silently ignores it, so the
    /// survey simply shows "no NAV-PVT reply".
    fn start_gps_debug_ble(&mut self) {
        if !matches!(self.sync.ble_state, BleState::Connected) {
            push_log(
                &self.gps_dbg_log,
                "error: connect to the box in the Sync tab first — the BLE \
                 survey tunnels the u-blox over the box's link."
                    .into(),
            );
            return;
        }
        let label = {
            let l = self.gps_dbg_label.trim();
            if l.is_empty() { "antenna".to_string() } else { l.to_string() }
        };
        let out_dir = match resolve_save_dir(&self.sync.ble_out_dir) {
            Ok(abs) => abs.join("gps-debug"),
            Err(_) => self.sync.ble_out_dir.join("gps-debug"),
        };
        let _ = std::fs::create_dir_all(&out_dir);

        let Some(ble) = self.sync.ble.as_ref() else {
            push_log(&self.gps_dbg_log, "error: BLE backend not running.".into());
            return;
        };
        // Point the worker's GPS-bridge sink at this survey, then turn the
        // bridge on. `cmd` drives the survey's UBX poll writes; `cmd_off`
        // tears the bridge down when the loop ends (the worker then drops
        // the sink).
        let (data_tx, data_rx) = mpsc::channel::<Vec<u8>>();
        ble.set_gps_data_sink(Some(data_tx));
        ble.send(BleCmd::GpsBridge { on: true });
        let cmd = ble.cmd_sender();
        let cmd_off = ble.cmd_sender();

        push_log(
            &self.gps_dbg_log,
            format!(
                "BLE GPS survey — bridging the u-blox over the box · label {label} · output {}",
                out_dir.display()
            ),
        );

        self.gps_dbg_cancel.store(false, Ordering::SeqCst);
        self.gps_dbg_running.store(true, Ordering::SeqCst);
        let log = self.gps_dbg_log.clone();
        let stop = self.gps_dbg_cancel.clone();
        let running = self.gps_dbg_running.clone();

        thread::spawn(move || {
            let mut transport = BleTransport::new(cmd, data_rx);
            let res = gps_debug::run_core(
                &mut transport, &out_dir, &label, None, stop,
                &mut |line| push_log(&log, line),
            );
            if let Err(e) = res {
                push_log(&log, format!("error: {e}"));
            }
            // Turn the bridge off — the worker drops the data sink.
            let _ = cmd_off.send(BleCmd::GpsBridge { on: false });
            push_log(&log, "--- stopped ---".into());
            running.store(false, Ordering::SeqCst);
        });
    }

    fn ui_live_tab(&mut self, ui: &mut egui::Ui) {
        ui.heading("Live");
        ui.label(
            egui::RichText::new(
                "SensorStream — 0.5 Hz packed all-sensor snapshot \
                 (IMU + mag + baro + GPS).",
            ),
        );

        // ----- Connection / capability gate ------------------------
        let connected = matches!(self.sync.ble_state, BleState::Connected);
        if !connected {
            ui.add_space(20.0);
            ui.vertical_centered(|ui| {
                ui.label(
                    egui::RichText::new("Not connected.")
                        .size(16.0)
                        .strong(),
                );
                ui.add_space(6.0);
                ui.label(
                    egui::RichText::new(
                        "Open the Sync tab, run Scan, and Connect to a box (PIN 123456). \
                         The live stream starts automatically — no extra button needed.",
                    ),
                );
                ui.add_space(8.0);
                if ui.button("Go to Sync").clicked() {
                    self.current_tab = Tab::Sync;
                }
            });
            return;
        }

        // ----- Status strip ----------------------------------------
        ui.add_space(6.0);
        let freshness = self.latest_sample_at.map(|t| t.elapsed());
        let stale = freshness.map(|d| d.as_secs() > 5).unwrap_or(true);
        ui.horizontal(|ui| {
            ui.label(format!("{} samples received", self.live_sample_count));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                match freshness {
                    Some(d) if !stale => {
                        ui.colored_label(
                            egui::Color32::LIGHT_GREEN,
                            format!("last sample {} ms ago", d.as_millis()),
                        );
                    }
                    Some(d) => {
                        ui.colored_label(
                            egui::Color32::LIGHT_YELLOW,
                            format!(
                                "no sample for {} s — check connection",
                                d.as_secs()
                            ),
                        );
                    }
                    None => {
                        ui.colored_label(
                            egui::Color32::LIGHT_YELLOW,
                            "waiting for first SensorStream notify…",
                        );
                    }
                }
            });
        });
        // Keep ticking at ~4 Hz so the freshness label updates between
        // samples without waiting on user input.
        ui.ctx().request_repaint_after(std::time::Duration::from_millis(250));
        ui.separator();

        // ----- Readouts --------------------------------------------
        let Some(s) = self.latest_sample else {
            return;
        };
        let acc_g = |i: usize| s.acc_mg[i] as f32 / 1000.0;
        // Firmware v0.0.27+ streams the gyro as DECI-dps (÷10 for °/s).
        let gyro_dps = |i: usize| s.gyro_cdps[i] as f32 / 10.0;
        let pres_hpa = s.pressure_pa as f64 / 100.0;
        let temp_c = s.temperature_cc as f32 / 100.0;

        // Tilt-compensated orientation from the gravity + magnetic
        // vectors. Pitch/roll come straight out of the accel reading
        // (assumes the box is roughly static / 1 g, which is true at
        // 0.5 Hz for a board on the water between strokes). Heading is
        // the mag vector projected onto the local horizontal plane
        // after de-rotating by pitch/roll — the standard "eCompass"
        // formula. Result is in degrees; heading is wrapped to 0..360.
        // Silent hard-iron auto-calibration (iOS/Android parity —
        // `FileSyncCore.autoCalibrate`): fold every FLAT, still sample's
        // mag (x, y) into a sliding 64-sample window; once X and Y each
        // span a full flat rotation (>= 180 mG, and the two spans are
        // similar — a circle, not a stray arc) the midpoints become the
        // offset. A window, not a session envelope, so one motion spike
        // can't lock calibration out forever. Every offset refinement
        // folds its heading shift into `heading_bias_deg` so the "set
        // direction" anchor never drifts. No button — the only thing the
        // user provides is the one south reference tap below.
        {
            let ax = s.acc_mg[0] as f64;
            let ay = s.acc_mg[1] as f64;
            let az = s.acc_mg[2] as f64;
            // Flat + still: lid up or down, no big lateral acceleration.
            if az.abs() >= 900.0 && ax.abs() <= 300.0 && ay.abs() <= 300.0 {
                let mx = s.mag_mg[0] as f64;
                let my = s.mag_mg[1] as f64;
                if (mx * mx + my * my).sqrt() < 1500.0 {
                    self.auto_buf.push((mx, my));
                    let n = self.auto_buf.len();
                    if n > 64 { self.auto_buf.drain(0..n - 64); }
                    if self.auto_buf.len() >= 8 {
                        let min_x = self.auto_buf.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
                        let max_x = self.auto_buf.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
                        let min_y = self.auto_buf.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
                        let max_y = self.auto_buf.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
                        let span_x = max_x - min_x;
                        let span_y = max_y - min_y;
                        let ratio = span_x.max(span_y) / span_x.min(span_y).max(1e-6);
                        // Full circle in X and Y, similar spans (a circle,
                        // not a stray arc). Spikes age out of the window.
                        if span_x >= 180.0 && span_y >= 180.0 && ratio <= 2.0 {
                            let nx = (max_x + min_x) / 2.0;
                            let ny = (max_y + min_y) / 2.0;
                            let mut off = self.mag_offset.unwrap_or([0.0; 3]);
                            // Apply only on a real change (don't churn config).
                            if (nx - off[0]).abs() > 10.0 || (ny - off[1]).abs() > 10.0 {
                                off[0] = nx;
                                off[1] = ny;
                                self.mag_offset = Some(off);
                                let mut cfg = agent_config::AgentConfig::load();
                                cfg.mag_offset_mg = Some(off);
                                let _ = cfg.save();
                                self.mag_off_persisted = Some(off);
                                // The offset only SEEDS the gyro filter's
                                // initial heading now — the preview no longer
                                // depends on the magnetometer, so an offset
                                // change must NOT touch the render bias (owned
                                // by "set direction", carried by the gyro).
                                push_log(&self.log, format!(
                                    "compass auto-calibrated: offset [{:+.0} {:+.0} {:+.0}] mG",
                                    off[0], off[1], off[2]));
                            }
                        }
                    }
                }
            }
        }

        let (pitch_deg, roll_deg, heading_deg) = {
            let ax = acc_g(0) as f64;
            let ay = acc_g(1) as f64;
            let az = acc_g(2) as f64;
            let roll  = ay.atan2(az);
            let pitch = (-ax).atan2((ay * ay + az * az).sqrt());
            // Heading = the gyro+accel filter's nose azimuth (matches the
            // preview arrow). Pure gyro carry (mag only seeds it), so the
            // absolute value is anchored by the "USB-C south" tap and drifts
            // slowly. `0.0` until the filter seeds.
            let heading = self
                .orientation_filter
                .nose_azimuth(self.nose_plus_y.unwrap_or(false), self.heading_bias_deg)
                .unwrap_or(0.0);
            (pitch.to_degrees(), roll.to_degrees(), heading)
        };

        egui::Grid::new("live-readouts")
            .num_columns(4)
            .spacing([16.0, 8.0])
            .show(ui, |ui| {
                // Per-axis tilt straight from the gravity projection:
                // 0 g → 0° (axis horizontal), +1 g → +90° (axis points
                // straight down), -1 g → -90° (straight up). arcsin of
                // the clamped g value; clamp guards the >1 g motion
                // spikes that would otherwise produce NaN.
                let tilt_deg = |i: usize| {
                    (acc_g(i).clamp(-1.0, 1.0)).asin().to_degrees()
                };
                ui.label(egui::RichText::new("Accel (g / °)").strong());
                ui.label(format!("X {:+.3}  {:+5.1}°", acc_g(0), tilt_deg(0)));
                ui.label(format!("Y {:+.3}  {:+5.1}°", acc_g(1), tilt_deg(1)));
                ui.label(format!("Z {:+.3}  {:+5.1}°", acc_g(2), tilt_deg(2)));
                ui.end_row();

                ui.label(egui::RichText::new("Gyro (°/s)").strong());
                ui.label(format!("X {:+.2}", gyro_dps(0)));
                ui.label(format!("Y {:+.2}", gyro_dps(1)));
                ui.label(format!("Z {:+.2}", gyro_dps(2)));
                ui.end_row();

                ui.label(egui::RichText::new("Mag (mG)").strong());
                ui.label(format!("X {:+}", s.mag_mg[0]));
                ui.label(format!("Y {:+}", s.mag_mg[1]));
                ui.label(format!("Z {:+}", s.mag_mg[2]));
                ui.end_row();

                ui.label(egui::RichText::new("Orient (°)").strong());
                ui.label(format!("pitch {:+6.1}", pitch_deg));
                ui.label(format!("roll {:+6.1}",  roll_deg));
                ui.label(format!("hdg {:6.1}",    heading_deg));
                ui.end_row();

                ui.label(egui::RichText::new("Baro").strong());
                ui.label(format!("{:.2} hPa", pres_hpa));
                ui.label(format!("{:+.2} °C", temp_c));
                ui.label(""); // pad to 4 cols
                ui.end_row();

                ui.label(egui::RichText::new("GPS").strong());
                if let Some((lat, lon)) = s.lat_lon_deg() {
                    ui.label(format!("{:.6}°", lat));
                    ui.label(format!("{:.6}°", lon));
                    ui.label(format!("{} m", s.gps_alt_m));
                } else {
                    // No fix yet (or no GPS module). Make the empty
                    // state obvious so the user doesn't read "0.0, 0.0"
                    // and panic.
                    ui.colored_label(
                        egui::Color32::LIGHT_YELLOW,
                        "no fix",
                    );
                    ui.label("");
                    ui.label("");
                }
                ui.end_row();

                ui.label(egui::RichText::new("GPS aux").strong());
                ui.label(format!("{} sats", s.gps_nsat));
                ui.label(format!("fix-q {}", s.gps_fix_q));
                ui.label(format!(
                    "{:.2} km/h  ({:.1}°)",
                    s.gps_speed_cmh as f64 / 100.0,
                    s.gps_course_cdeg as f64 / 100.0,
                ));
                ui.end_row();

                ui.label(egui::RichText::new("GPS C/N0").strong());
                if s.cn0_max > 0 {
                    ui.label(format!("{} dB-Hz max", s.cn0_max));
                    let (txt, col) = if s.cn0_max >= 40 {
                        ("good antenna", egui::Color32::LIGHT_GREEN)
                    } else if s.cn0_max >= 30 {
                        ("ok", egui::Color32::LIGHT_YELLOW)
                    } else {
                        ("weak signal", egui::Color32::LIGHT_RED)
                    };
                    ui.colored_label(col, txt);
                    ui.label("");
                } else {
                    ui.colored_label(egui::Color32::LIGHT_YELLOW, "no GSV / no data");
                    ui.label("");
                    ui.label("");
                }
                ui.end_row();

                ui.label(egui::RichText::new("Flags").strong());
                let flag_col = |on: bool| {
                    if on { egui::Color32::LIGHT_GREEN }
                    else  { egui::Color32::from_gray(120) }
                };
                ui.colored_label(flag_col(s.gps_valid),     "gps_valid");
                ui.colored_label(flag_col(s.logging_active), "logging");
                ui.colored_label(flag_col(s.low_battery),    "low_batt");
                ui.end_row();
            });

        // ----- Battery (dedicated BatteryStatus characteristic) --------
        // Desktop-only super-set: iOS/Android show only the low_batt flag
        // (in the grid above). This meter is driven by the box's dedicated
        // …0200… BatteryStatus characteristic (real voltage / SoC% /
        // current from the STC3115 fuel gauge) and has no mobile
        // equivalent — don't delete it in a future "match mobile" pass.
        // `None` on legacy firmware ⇒ section simply hidden. Lets the user
        // watch the pack level directly (GPS acquisition browns out on a
        // low battery — this makes that correlation visible).
        if let Some(b) = self.latest_battery {
            let stale = self
                .latest_battery_at
                .map(|t| t.elapsed().as_secs() > 90)   // notifies ~once/min
                .unwrap_or(true);
            ui.add_space(8.0);
            let pct = b.soc_pct();
            // Same red<20 / yellow<40 / green ramp as the C/N0 row above.
            let fill = if pct < 20 { egui::Color32::from_rgb(200, 70, 70) }
                       else if pct < 40 { egui::Color32::from_rgb(210, 170, 60) }
                       else { egui::Color32::from_rgb(70, 170, 90) };
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("Battery").strong());
                ui.add(
                    egui::ProgressBar::new(b.soc_frac())
                        .desired_width(360.0)
                        .fill(fill)
                        .text(format!(
                            "{}%  ·  {:.2} V  ·  {:+.2} A{}",
                            pct,
                            b.volts(),
                            b.amps(),
                            if stale { "  · stale" } else { "" },
                        )),
                );
            });
            if b.low_batt {
                ui.colored_label(egui::Color32::LIGHT_RED,
                    "⚠ low battery (< 10 %) — charge the box; GPS may lose fix");
            }
        }

        ui.add_space(10.0);
        ui.separator();

        // ----- 3D orientation (gyro + accel filter) ----------------
        // Attitude comes from the gyro+accel complementary filter (NO
        // magnetometer in the render path): the accel gives tilt (which
        // face is up, exact every frame) and the gyro carries the
        // horizontal frame (real rotation, consistent in every pose) with
        // its bias auto-removed at rest. The one thing gyro+accel can't
        // know — where north is — the user supplies with ONE "USB-C south"
        // tap (a render bias). Fixed map preview, screen-up = SOUTH.
        // Mirrors iOS `OrientationFilter` + `OrientationBoxCanvas`.
        let norm360 = |x: f64| { let mut v = x % 360.0; if v < 0.0 { v += 360.0; } v };
        ui.label(egui::RichText::new("Box orientation").strong());

        let flat = (s.acc_mg[2] as f64).abs() >= 900.0;
        ui.label("Lay the box flat, USB-C end pointing SOUTH, then tap:");
        if ui
            .add_enabled(flat, egui::Button::new("USB-C points SOUTH — set direction"))
            .on_hover_text(
                "The box's USB-C end is the FRONT. Point it south (use a compass / \
                 phone), box flat on the table, and tap — this defines the gyro \
                 filter's current yaw as south, so the green arrow then points south.",
            )
            .clicked()
        {
            // Define the gyro filter's current nose yaw as south (nose
            // reads 180°). iOS `setDirectionSouth`.
            if let Some(az) = self
                .orientation_filter
                .nose_azimuth(self.nose_plus_y.unwrap_or(false), 0.0)
            {
                self.heading_bias_deg = norm360(az - 180.0);
                let mut cfg = agent_config::AgentConfig::load();
                cfg.heading_bias_deg = Some(self.heading_bias_deg);
                let _ = cfg.save();
                push_log(&self.log, "compass: direction set (USB-C = south)".to_string());
            }
        }

        let upright = (s.acc_mg[1] as f64).abs() >= 900.0;
        ui.label("Once, for tilt: stand the box upright, USB-C end UP, and tap:");
        if ui
            .add_enabled(upright, egui::Button::new("USB-C end is UP — confirm"))
            .clicked()
        {
            let was = self.nose_plus_y.unwrap_or(false);
            let now = s.acc_mg[1] as f64 > 0.0;
            self.nose_plus_y = Some(now);
            // Flipping the nose end flips its azimuth by exactly 180° in any
            // pose — nudge the bias to keep the set direction valid.
            if now != was {
                self.heading_bias_deg = norm360(self.heading_bias_deg + 180.0);
            }
            let mut cfg = agent_config::AgentConfig::load();
            cfg.nose_plus_y = self.nose_plus_y;
            cfg.heading_bias_deg = Some(self.heading_bias_deg);
            let _ = cfg.save();
        }

        ui.horizontal(|ui| {
            if ui.button("Reset calibration").clicked() {
                self.mag_offset = None;
                self.mag_off_persisted = None;
                self.heading_bias_deg = 0.0;
                self.nose_plus_y = None;
                self.auto_buf.clear();
                self.orientation_filter.reset();
                self.ori_rows = None;
                let mut cfg = agent_config::AgentConfig::load();
                cfg.mag_offset_mg = None;
                cfg.heading_bias_deg = None;
                cfg.nose_plus_y = None;
                cfg.lateral_sign = None;   // clear any stale stored value
                let _ = cfg.save();
                push_log(&self.log, "compass calibration reset".to_string());
            }
            if let Some(off) = self.mag_offset {
                ui.label(format!("offset [{:+.0} {:+.0} {:+.0}] mG", off[0], off[1], off[2]));
            }
        });
        ui.label(
            "Attitude is gyro + accel (no magnetometer): tilt is exact every \
             frame, rotation is tracked by the gyro with its bias auto-removed \
             at rest. You set which way is south once — USB-C south, box flat. \
             The preview is a FIXED map: screen-up is SOUTH, so once set the \
             green nose arrow points down/south. The absolute heading drifts \
             slowly — re-tap USB-C south if it wanders.",
        );
        draw_orientation_box(
            ui,
            self.ori_rows,
            self.heading_bias_deg,
            self.nose_plus_y.unwrap_or(false),
        );
        ui.add_space(10.0);
        ui.separator();

        // ----- Sparklines ------------------------------------------
        // Two thin painter-drawn time-series — acc magnitude (g) and
        // pressure (hPa). At 0.5 Hz × 120 samples ≈ 4 min of history.
        // egui_plot would give axis labels for free but pulls in another
        // crate; for v0.0.3 the bare polylines are enough to spot
        // oscillation / pressure trend at a glance.
        ui.label(egui::RichText::new("Acc magnitude (g)").strong());
        draw_sparkline(
            ui,
            &self.live_acc_history,
            egui::Color32::from_rgb(80, 180, 250),
            (0.5, 1.5),
        );
        ui.add_space(8.0);
        ui.label(egui::RichText::new("Pressure (hPa)").strong());
        draw_sparkline(
            ui,
            &self.live_pressure_history,
            egui::Color32::from_rgb(250, 180, 80),
            (980.0, 1030.0),
        );
    }

    fn ingest_dropped(&mut self, files: &[egui::DroppedFile]) {
        for f in files {
            let Some(path) = f.path.as_ref() else { continue };
            match classify(path) {
                FileKind::Sensor => {
                    self.sensor_csv = Some(path.clone());
                    if self.gps_csv.is_none() {
                        self.gps_csv = guess_gps_for(path);
                    }
                }
                FileKind::Gps => {
                    self.gps_csv = Some(path.clone());
                }
                FileKind::Video => {
                    self.video = Some(path.clone());
                }
                FileKind::Stl => {
                    self.board_stl = Some(path.clone());
                }
                FileKind::Unknown => {
                    push_log(
                        &self.log,
                        format!(
                            "ignored {} (drop a Sens*.csv, *_gps.csv, .mov/.mp4, or .stl)",
                            path.display()
                        ),
                    );
                }
            }
        }
    }
}

impl eframe::App for AppState {
    /// Called by eframe when the window is closed cleanly (Cmd-Q, red
    /// traffic-light, app menu Quit). We send Disconnect to the BLE
    /// worker before egui tears the runtime down so btleplug gets to
    /// emit LL_TERMINATE_IND to the box. Without this, the firmware
    /// keeps the GATT connection alive until the supervision timeout
    /// (~10-30 s on macOS), and a fresh GUI launched in that window
    /// can't see `STBoxFs` because the box is still advertising as
    /// non-connectable to others. A hard `pkill -9` skips this path —
    /// in that case only a box reboot resets the link.
    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        if let Some(b) = self.sync.ble.as_ref() {
            // Quitting mid-reconnect must not hang on the worker being
            // stuck in an (Auto Mode: unbounded) reconnect loop.
            b.request_abort();
            b.send(BleCmd::Disconnect);
            /* Give the worker thread ~250 ms to actually emit the
               disconnect over the air before the process exits. Not
               a guarantee — btleplug's per-platform stack may need
               longer — but enough on macOS Core Bluetooth in
               practice. */
            std::thread::sleep(std::time::Duration::from_millis(250));
        }
        /* Release adapter ownership so a background agent can resume
           the mirror as soon as the GUI is gone. Dropping the guards
           unlocks; the OS would also free them on exit, but doing it
           here narrows the handover gap. */
        self._ble_lock = None;
        self._gui_lock = None;
    }

    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Drag-and-drop handling. Plain files come through
        // `dropped_files`; `hovered_files` lets us highlight the drop
        // zone before the user releases.
        let dropped = ctx.input(|i| i.raw.dropped_files.clone());
        if !dropped.is_empty() {
            self.ingest_dropped(&dropped);
        }
        let hovering = ctx.input(|i| !i.raw.hovered_files.is_empty());

        // Drain BLE worker events into AppState before laying out the
        // UI so the FileSync panel renders the latest state every frame.
        self.pump_ble_events();
        // Same idea for the version-check + install pipeline.
        self.pump_update_events();
        // Drain the "Check FW" GitHub-fetch result (box half arrives via
        // pump_ble_events → FirmwareVersion); fires the compare/flash once
        // both halves are in.
        self.sync.poll_fw_check();

        // Mirror "Keep synced" into the worker every frame (cheap
        // relaxed atomic). It's what flips auto_reconnect between the
        // bounded manual-mode budget and unbounded Auto-Mode retry, and
        // un-ticking it mid-reconnect is what brings an unbounded loop
        // back down to the bounded budget so it stops on its own.
        if let Some(b) = self.sync.ble.as_ref() {
            b.set_keep_synced(self.sync.ble_keep_synced);
        }

        // "Keep synced": while connected and idle, kick a fresh sync
        // pass on the poll interval so a continuously-growing log keeps
        // mirroring (each pass only fetches the new tail). A busy
        // backlog effectively runs back-to-back because the interval is
        // measured from the previous *trigger*, long elapsed by the
        // time a big pass drains.
        //
        // `ble_resume_box.is_some()` means a transfer was cut mid-batch
        // and we're inside the worker's silent auto_reconnect loop.
        // `ble_state` stays Connected for the whole reconnect window
        // (no Disconnected event until the bounded budget is spent), so
        // without this guard the tick would fire `start_sync_pass`,
        // send a LIST through the channel, and pile up duplicate ops
        // behind the reconnect — the .connected handler already re-
        // arms the sync with "Reconnected — resuming sync" so nothing
        // is lost by skipping ticks in between. (iOS 8c90276 parity.)
        if self.sync.ble_keep_synced
            && matches!(self.sync.ble_state, BleState::Connected)
            && !self.sync.ble_sync_pending
            && !self.sync.ble_dl_in_flight
            && self.sync.ble_dl_queue.is_empty()
            && self.sync.ble_resume_box.is_none()
        {
            let due = self
                .sync
                .ble_last_sync_at
                .map(|t| t.elapsed() >= SYNC_POLL_INTERVAL)
                .unwrap_or(true);
            if due {
                self.sync.start_sync_pass();
            }
        }
        if matches!(self.sync.ble_state, BleState::Connected) {
            // Keep ticking so the Keep-synced poll fires without user
            // input — and so the worker's auto-reconnect Status lines
            // (the loop emits one per attempt) render live. ble_state
            // stays Connected for the whole reconnect window in both
            // modes (no Disconnected until the bounded budget is spent),
            // so this one condition covers active sync + reconnecting.
            ctx.request_repaint_after(std::time::Duration::from_secs(1));
        }

        // Lazy-load the in-app logo on the first frame after the egui
        // context becomes available.
        if self.icon_tex.is_none() {
            self.icon_tex = decode_icon().map(|img| {
                ctx.load_texture("movementlogger-icon", img, egui::TextureOptions::LINEAR)
            });
        }

        egui::TopBottomPanel::top("title").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.vertical(|ui| {
                    // Name + version live in the OS title bar (set in
                    // main()); don't repeat them in the in-app header.
                    ui.hyperlink_to(
                        "SensorTile.box pumpfoil session video generator",
                        "https://github.com/zdavatz/movement_logger_firmware",
                    );
                });
                // Right-anchor the logo so it sits in the top-right
                // corner regardless of window width. Clicking it opens
                // a mailto: to support — saves field testers having to
                // hunt for the support address when something goes
                // wrong with a session render.
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if let Some(tex) = self.icon_tex.as_ref() {
                        let size = egui::vec2(40.0, 40.0);
                        let resp = ui
                            .add(
                                egui::ImageButton::new((tex.id(), size))
                                    .frame(false),
                            )
                            .on_hover_text("Email zdavatz@ywesee.com")
                            .on_hover_cursor(egui::CursorIcon::PointingHand);
                        if resp.clicked() {
                            ui.ctx().open_url(egui::OpenUrl::new_tab(
                                "mailto:zdavatz@ywesee.com",
                            ));
                        }
                    }
                });
            });
        });

        // ----- Tab strip ---------------------------------------------
        egui::TopBottomPanel::top("tabs").show(ctx, |ui| {
            ui.add_space(2.0);
            let mut do_disconnect = false;
            // Cloned/copied out so the right-aligned block borrows locals, not
            // `self`, inside the nested layout closure.
            let connected = matches!(self.sync.ble_state, BleState::Connected);
            let status = self.sync.ble_status.clone();
            let batt = self.latest_battery;
            ui.horizontal(|ui| {
                for (tab, label) in [
                    (Tab::Live,     "Live"),
                    (Tab::Sync,     "Sync"),
                    (Tab::Replay,   "Replay"),
                    (Tab::Debug,    "Debug"),
                    (Tab::Firmware, "Firmware"),
                ] {
                    let selected = self.current_tab == tab;
                    // SelectableLabel gives us the standard egui
                    // pressed-look without the heavy CollapsingHeader
                    // chrome. Click switches tabs.
                    if ui.add(egui::SelectableLabel::new(selected, label)).clicked() {
                        self.current_tab = tab;
                    }
                }
                // Disconnect + live BLE status, right-aligned in the tab row
                // (under the top-right logo, same height as the tabs). The
                // status ("connecting…", "disconnecting…", errors) sits just
                // left of the button. Visible on every tab; the button is
                // enabled only while a box is connected.
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui
                        .add_enabled(connected, egui::Button::new("Disconnect"))
                        .clicked()
                    {
                        do_disconnect = true;
                    }
                    if !status.is_empty() {
                        ui.add_space(8.0);
                        ui.add(
                            egui::Label::new(
                                egui::RichText::new(&status)
                                    .color(egui::Color32::LIGHT_BLUE),
                            )
                            .selectable(true),
                        );
                    }
                    // Compact always-visible battery chip while connected —
                    // so the pack level is in view on every tab, not just
                    // Live (issue: GPS browns out on a low battery).
                    if let Some(b) = batt.filter(|_| connected) {
                        ui.add_space(8.0);
                        let pct = b.soc_pct();
                        /* One glyph for all levels: egui's bundled emoji
                           font has 🔋 (U+1F50B) but NOT the newer
                           low-battery 🪫 (U+1FAAB, Unicode 14) — that one
                           rendered as a missing-glyph box. Colour carries
                           the low/mid/ok state; ⚠ (in-font) marks low. */
                        let (col, glyph) = if pct < 20 {
                            (egui::Color32::from_rgb(220, 90, 90), "⚠🔋")
                        } else if pct < 40 {
                            (egui::Color32::from_rgb(210, 170, 60), "🔋")
                        } else {
                            (egui::Color32::from_rgb(90, 190, 110), "🔋")
                        };
                        ui.colored_label(col, format!("{glyph} {pct}%"));
                    }
                });
            });
            if do_disconnect {
                self.disconnect_box();
            }
            ui.add_space(2.0);
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.add_space(6.0);

            // ----- Update banner --------------------------------------
            // Shared across all tabs so a user on Live still sees
            // "Update available" without having to switch panes.
            self.render_update_banner(ui);
            // Box-firmware update banner (FOTA over BLE) — same treatment,
            // only while a box is connected with a pending firmware update.
            self.render_fw_banner(ui);

            match self.current_tab {
                Tab::Live     => { self.ui_live_tab(ui);     return; }
                Tab::Sync     => { self.ui_sync_tab(ui);     return; }
                Tab::Firmware => { self.ui_firmware_tab(ui); return; }
                Tab::Debug    => { self.ui_debug_tab(ui);    return; }
                Tab::Replay   => { /* fall through to original layout */ }
            }

            // ----- Drop zone ------------------------------------------
            let zone_color = if hovering {
                egui::Color32::from_rgb(70, 110, 70)
            } else {
                egui::Color32::from_rgb(40, 40, 50)
            };
            egui::Frame::none()
                .fill(zone_color)
                .stroke(egui::Stroke::new(1.0, egui::Color32::from_gray(120)))
                .rounding(6.0)
                .inner_margin(egui::Margin::symmetric(12.0, 18.0))
                .show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.vertical_centered(|ui| {
                        ui.label(
                            egui::RichText::new("Drop sensor CSV + GPS CSV + video here")
                                .heading()
                                .color(egui::Color32::WHITE),
                        );
                        ui.label(
                            egui::RichText::new(
                                "Sens*.csv • *_gps.csv (auto-paired) • .mov / .mp4 • .stl (optional)",
                            ),
                        );
                    });
                });

            ui.add_space(8.0);

            // ----- Input file summary ---------------------------------
            egui::Grid::new("inputs")
                .num_columns(3)
                .min_col_width(80.0)
                .show(ui, |ui| {
                    file_row(ui, "Sensor", &mut self.sensor_csv, &[("CSV", &["csv"])]);
                    ui.end_row();
                    file_row(ui, "GPS", &mut self.gps_csv, &[("CSV", &["csv"])]);
                    ui.end_row();
                    file_row(
                        ui,
                        "Video",
                        &mut self.video,
                        &[("Video", &["mov", "mp4", "mkv", "m4v", "avi"])],
                    );
                    ui.end_row();
                    file_row(ui, "Board STL", &mut self.board_stl, &[("STL", &["stl"])]);
                    ui.end_row();
                });

            ui.separator();

            // ----- Optional fields ------------------------------------
            egui::CollapsingHeader::new("Animation parameters")
                .default_open(true)
                .show(ui, |ui| {
                    egui::Grid::new("opts")
                        .num_columns(2)
                        .spacing([10.0, 6.0])
                        .show(ui, |ui| {
                            ui.label("Von (HH:MM:SS, local)");
                            ui.text_edit_singleline(&mut self.at)
                                .on_hover_text(
                                    "Start of the window the merge video covers. \
                                     Blank = auto-detect the session.",
                                );
                            ui.end_row();

                            ui.label("Bis (HH:MM:SS, local)");
                            ui.text_edit_singleline(&mut self.to)
                                .on_hover_text(
                                    "End of the window. Set together with Von to \
                                     render exactly that slice (duration is \
                                     computed as Bis − Von). Blank = use the \
                                     Duration field below / video length.",
                                );
                            ui.end_row();

                            ui.label("Timezone offset (hours)");
                            ui.add(egui::DragValue::new(&mut self.tz_offset_h).speed(1.0).range(-12.0..=14.0));
                            ui.end_row();

                            ui.label("Date (YYYY-MM-DD, blank = sensor mtime)");
                            ui.text_edit_singleline(&mut self.date);
                            ui.end_row();

                            ui.label("Duration (s, 0 = video length — ignored if Bis set)");
                            ui.add(egui::DragValue::new(&mut self.duration_s).speed(1.0).range(0.0..=600.0));
                            ui.end_row();

                            ui.label("Mount");
                            ui.horizontal(|ui| {
                                ui.radio_value(&mut self.mount, Mount::Mast, "Mast");
                                ui.radio_value(&mut self.mount, Mount::Deck, "Deck");
                            });
                            ui.end_row();

                            ui.label("Dock height (m)");
                            ui.add(egui::DragValue::new(&mut self.dock_height_m).speed(0.05).range(0.0..=3.0));
                            ui.end_row();

                            ui.label("Auto-skip carry phase");
                            ui.checkbox(&mut self.auto_skip, "");
                            ui.end_row();

                            ui.label("Title");
                            ui.text_edit_singleline(&mut self.title);
                            ui.end_row();

                            ui.label("Subtitle");
                            ui.text_edit_singleline(&mut self.subtitle);
                            ui.end_row();

                            ui.label("FPS");
                            ui.add(egui::DragValue::new(&mut self.fps).speed(1.0).range(5..=60));
                            ui.end_row();

                            ui.label("Output folder");
                            ui.horizontal(|ui| {
                                let mut s = self.output_dir.display().to_string();
                                let resp = ui.text_edit_singleline(&mut s);
                                if resp.changed() {
                                    self.output_dir = PathBuf::from(s);
                                }
                                if ui.button("Browse…").clicked() {
                                    if let Some(p) = rfd::FileDialog::new().pick_folder() {
                                        self.output_dir = p;
                                    }
                                }
                            });
                            ui.end_row();
                        });
                });

            ui.add_space(6.0);

            // ----- Action buttons -------------------------------------
            ui.horizontal(|ui| {
                let running = self.running.load(Ordering::SeqCst);
                let can_run = !running && self.sensor_csv.is_some();
                if ui
                    .add_enabled(can_run, egui::Button::new("▶ Generate"))
                    .clicked()
                {
                    spawn_animate(self);
                }
                if ui
                    .add_enabled(running, egui::Button::new("■ Stop"))
                    .clicked()
                {
                    self.cancel.store(true, Ordering::SeqCst);
                }
                if ui.button("Open output folder").clicked() {
                    let _ = open_in_filer(&self.output_dir);
                }
                if ui.button("Clear log").clicked() {
                    if let Ok(mut v) = self.log.lock() {
                        v.clear();
                    }
                }
                let check_label = if self.update_checking { "Checking…" } else { "Check for updates" };
                if ui
                    .add_enabled(!self.update_checking, egui::Button::new(check_label))
                    .clicked()
                {
                    self.trigger_update_check();
                }
                if let Some(msg) = &self.update_status_msg {
                    if self.update_info.is_none() {
                        ui.label(egui::RichText::new(msg).weak());
                    }
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    match (running, self.last_status) {
                        (true, _) => {
                            ui.spinner();
                            ui.label("running…");
                        }
                        (false, Some(RunStatus::Ok)) => {
                            ui.colored_label(egui::Color32::LIGHT_GREEN, "ok");
                        }
                        (false, Some(RunStatus::Failed)) => {
                            ui.colored_label(egui::Color32::LIGHT_RED, "failed");
                        }
                        (false, Some(RunStatus::Cancelled)) => {
                            ui.colored_label(egui::Color32::LIGHT_YELLOW, "cancelled");
                        }
                        _ => {}
                    }
                });
            });

            ui.separator();

            // ----- Log panel ------------------------------------------
            ui.horizontal(|ui| {
                ui.label("Log");
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.small_button("Copy all").clicked() {
                        let lines = self.log.lock().map(|v| v.clone()).unwrap_or_default();
                        ui.output_mut(|o| o.copied_text = lines.join("\n"));
                    }
                });
            });
            /* Grow the log to fill the remaining window space so resizing
               the window enlarges the log instead of leaving dead grey
               below it. `desired_rows` is computed from the available
               pixel height divided by the monospace row height; the
               ScrollArea still provides stick-to-bottom auto-scroll
               and a horizontal-overflow handler. */
            let row_h = ui.text_style_height(&egui::TextStyle::Monospace).max(1.0);
            let avail_h = ui.available_height().max(row_h * 4.0);
            let rows = (avail_h / row_h) as usize;
            egui::ScrollArea::vertical()
                .max_height(avail_h)
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    let lines = self.log.lock().map(|v| v.clone()).unwrap_or_default();
                    let text = lines.join("\n");
                    let mut owned = text;
                    /* No .interactive(false) → TextEdit is selectable
                       (Cmd-A to select all, Cmd-C to copy). Any typing
                       lands in the local `owned` and is discarded on
                       the next frame when we rebuild from self.log, so
                       the buffer remains effectively read-only without
                       blocking the user's mouse selection — which the
                       `.interactive(false)` path did. */
                    ui.add(
                        egui::TextEdit::multiline(&mut owned)
                            .desired_width(f32::INFINITY)
                            .desired_rows(rows.max(4))
                            .font(egui::TextStyle::Monospace),
                    );
                });
        });

        // Watch for the worker thread setting `running=false` and lift
        // the spinner / set the last_status. Tail-line inspection lets
        // us colour the status badge without the worker thread needing
        // a reference to AppState.
        if !self.running.load(Ordering::SeqCst) && self.last_status.is_none() {
            if let Ok(v) = self.log.lock() {
                if let Some(last) = v.last() {
                    if last == "--- done ---" {
                        self.last_status = Some(RunStatus::Ok);
                    } else if last == "--- cancelled ---" {
                        self.last_status = Some(RunStatus::Cancelled);
                    } else if last.starts_with("--- exited with") {
                        self.last_status = Some(RunStatus::Failed);
                    }
                }
            }
        }
        if self.running.load(Ordering::SeqCst) {
            self.last_status = None;
            ctx.request_repaint_after(std::time::Duration::from_millis(150));
        }

        // Same idea for BLE: while a scan / connect / list / read is in
        // flight or has just emitted progress, keep the UI ticking so
        // the status badge updates without waiting on user input.
        if matches!(
            self.sync.ble_state,
            BleState::Scanning | BleState::Connecting | BleState::Connected
        ) {
            ctx.request_repaint_after(std::time::Duration::from_millis(150));
        }

        // Update-check + install pipeline ticks: keep redrawing while
        // either is in flight so the version banner and download
        // progress bar update without user input.
        if self.update_checking || self.installing {
            ctx.request_repaint_after(std::time::Duration::from_millis(150));
        }
    }
}

fn file_row(
    ui: &mut egui::Ui,
    label: &str,
    target: &mut Option<PathBuf>,
    filters: &[(&str, &[&str])],
) {
    ui.label(label);
    let display = target
        .as_ref()
        .map(|p| {
            p.file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_else(|| p.display().to_string())
        })
        .unwrap_or_else(|| "—".to_string());
    ui.label(display);
    ui.horizontal(|ui| {
        if ui.button("Pick…").clicked() {
            let mut dialog = rfd::FileDialog::new();
            for (name, exts) in filters {
                dialog = dialog.add_filter(*name, exts);
            }
            if let Some(p) = dialog.pick_file() {
                *target = Some(p);
            }
        }
        if target.is_some() && ui.button("✕").clicked() {
            *target = None;
        }
    });
}

/// Render a small painter-drawn time-series of `(t_s, value)` samples
/// into the current row. `default_range` provides initial Y bounds when
/// the data span is tiny (so a near-static value doesn't render as a
/// flat line jittering on noise); when the actual sample range exceeds
/// it, the painter expands to fit. Width fills available space; height
/// is a fixed 60 px so two sparklines stack neatly without resize math.
// --- TRIAD seed helpers + gyro/accel attitude filter (iOS parity) --------
// `triad_rows`/`triad_world` build a full 3D frame straight from gravity +
// magnetic field — no Euler chaining, no gimbal. They are used ONLY to
// SEED the gyro+accel `OrientationFilter` below (initial heading) and to
// project the filter's body-frame axes into the fixed south-up preview.
// The live render attitude is the filter's — mag-independent. Mirrors iOS
// `enum Triad` + `OrientationFilter`.

/// Body-frame world axes (n, e, d) — the render rows, produced by the
/// gyro+accel filter instead of the magnetometer. Mirrors iOS `OriRows`.
#[derive(Clone, Copy)]
struct OriRows {
    n: [f64; 3],
    e: [f64; 3],
    d: [f64; 3],
}

/// Accel + gyro complementary filter with drone-style gyro-bias
/// auto-calibration — the live 3D preview's attitude source. Mirrors iOS
/// `OrientationFilter`.
///
///   • DOWN (tilt) comes from the accelerometer — exact, drift-free, every
///     frame, so which face is up is always right in every pose.
///   • the horizontal frame is carried by the gyroscope (EXACT Rodrigues
///     rotation), so blue-face / on-end / backflip stay consistent.
///   • gyro bias (the slow-yaw-drift culprit) is auto-measured whenever the
///     box rests still and subtracted, so the heading barely drifts.
///   • absolute heading (where north is) is the one thing gyro+accel can't
///     know; the "USB-C south" tap supplies it via the render bias.
struct OrientationFilter {
    n: [f64; 3],       // body-frame world-North
    e: [f64; 3],       // body-frame world-East
    d: [f64; 3],       // body-frame world-Down (flat lid-up)
    gbias: [f64; 3],   // gyro bias, deci-dps
    last_tick: Option<u32>,
    inited: bool,
}

impl Default for OrientationFilter {
    fn default() -> Self {
        Self {
            n: [1.0, 0.0, 0.0],
            e: [0.0, 1.0, 0.0],
            d: [0.0, 0.0, -1.0],
            gbias: [0.0, 0.0, 0.0],
            last_tick: None,
            inited: false,
        }
    }
}

impl OrientationFilter {
    /// Latest body-frame world axes, or `None` before the first seed.
    fn rows(&self) -> Option<OriRows> {
        if self.inited {
            Some(OriRows { n: self.n, e: self.e, d: self.d })
        } else {
            None
        }
    }

    /// Re-seed the attitude on the next sample; keep the learned gyro bias.
    fn reset(&mut self) {
        self.inited = false;
        self.last_tick = None;
    }

    /// Consume one SensorStream snapshot. `mag_offset` only SEEDS the
    /// initial heading (once) — there is NO magnetometer re-anchor.
    fn update(&mut self, s: &LiveSample, mag_offset: Option<[f64; 3]>) {
        let acc = [s.acc_mg[0] as f64, s.acc_mg[1] as f64, s.acc_mg[2] as f64];
        let a_mag = (acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]).sqrt();
        if a_mag <= 100.0 { return; }

        // gyro_cdps carries DECI-dps from firmware v0.0.27+ (÷10 for dps).
        let g_raw = [s.gyro_cdps[0] as f64, s.gyro_cdps[1] as f64, s.gyro_cdps[2] as f64];
        let mut g_corr = [
            g_raw[0] - self.gbias[0],
            g_raw[1] - self.gbias[1],
            g_raw[2] - self.gbias[2],
        ];

        // Drone-style gyro-bias cal: while the box rests (tiny corrected
        // rate AND ~1 g), the raw gyro reading IS the bias — ease toward it.
        // 25 deci-dps = 2.5 dps threshold for "still".
        let g_mag = (g_corr[0] * g_corr[0] + g_corr[1] * g_corr[1] + g_corr[2] * g_corr[2]).sqrt();
        if g_mag < 25.0 && a_mag > 900.0 && a_mag < 1100.0 {
            let k = 0.02;
            for i in 0..3 {
                self.gbias[i] = self.gbias[i] * (1.0 - k) + g_raw[i] * k;
            }
            g_corr = [
                g_raw[0] - self.gbias[0],
                g_raw[1] - self.gbias[1],
                g_raw[2] - self.gbias[2],
            ];
        }

        let dt = match self.last_tick {
            Some(last) => {
                (s.timestamp_ms.wrapping_sub(last) as f64 / 1000.0).clamp(0.005, 0.5)
            }
            None => 0.1,
        };
        self.last_tick = Some(s.timestamp_ms);

        // Seed the frame from accel + a mag-derived heading (once), so the
        // preview doesn't start at a random yaw; the gyro owns it after.
        if !self.inited {
            if let Some((rn, re, rd)) = triad_rows(s.acc_mg, s.mag_mg, mag_offset) {
                self.n = rn;
                self.e = re;
                self.d = rd;
                self.inited = true;
            }
            return;
        }

        // Gyro propagation. A world-fixed vector expressed in the body frame
        // rotates by the body's inverse rotation over dt — EXACT rotation
        // (Rodrigues) by angle -|ω|·dt about ω̂, correct at any angle.
        let scale = std::f64::consts::PI / 1800.0; // deci-dps → rad/s (÷10 ·π/180)
        let w = [g_corr[0] * scale, g_corr[1] * scale, g_corr[2] * scale];
        let w_mag = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
        if w_mag > 1e-9 {
            let ang = -w_mag * dt;
            let axis = [w[0] / w_mag, w[1] / w_mag, w[2] / w_mag];
            self.n = Self::rotate(self.n, axis, ang);
            self.d = Self::rotate(self.d, axis, ang);
        }

        // Tilt correction: nudge DOWN toward measured gravity when the accel
        // is trustworthy (near 1 g). Rate-independent gain (∝ dt, τ ≈ 0.6 s).
        if a_mag > 800.0 && a_mag < 1200.0 {
            let inv = -1.0 / a_mag;
            let d_meas = [acc[0] * inv, acc[1] * inv, acc[2] * inv];
            let k = (dt / 0.6).min(0.15);
            for i in 0..3 {
                self.d[i] = self.d[i] * (1.0 - k) + d_meas[i] * k;
            }
        }

        // Re-orthonormalise: D, then N ⟂ D, then E = D × N (NED: D×N = E).
        self.d = Self::normalized(self.d);
        let nd = self.n[0] * self.d[0] + self.n[1] * self.d[1] + self.n[2] * self.d[2];
        self.n = Self::normalized([
            self.n[0] - nd * self.d[0],
            self.n[1] - nd * self.d[1],
            self.n[2] - nd * self.d[2],
        ]);
        self.e = tri_cross(self.d, self.n);

        // NO magnetometer heading re-anchor (this box's hard-iron pins the
        // computed flat heading ≈ south regardless of orientation). Heading
        // is pure gyro, seeded once from the mag; the user sets the absolute
        // direction with "USB-C south".
        let _ = mag_offset;
    }

    /// Rodrigues rotation of `v` about unit axis `k` by angle `a` (rad).
    fn rotate(v: [f64; 3], k: [f64; 3], a: f64) -> [f64; 3] {
        let (ca, sa) = (a.cos(), a.sin());
        let kv = tri_cross(k, v);
        let kd = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
        let f = kd * (1.0 - ca);
        Self::normalized([
            v[0] * ca + kv[0] * sa + k[0] * f,
            v[1] * ca + kv[1] * sa + k[1] * f,
            v[2] * ca + kv[2] * sa + k[2] * f,
        ])
    }

    /// Nose-end compass azimuth (deg, [0,360)) from the filter, bias
    /// applied. `None` before the filter has seeded.
    fn nose_azimuth(&self, nose_plus_y: bool, bias_deg: f64) -> Option<f64> {
        if !self.inited { return None; }
        let nose = [0.0, if nose_plus_y { 1.0 } else { -1.0 }, 0.0];
        let (wn, we, _) = triad_world(nose, (self.n, self.e, self.d), bias_deg);
        let mut az = we.atan2(wn).to_degrees();
        if az < 0.0 { az += 360.0; }
        Some(az)
    }

    fn normalized(v: [f64; 3]) -> [f64; 3] {
        let m = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        if m > 1e-9 { [v[0] / m, v[1] / m, v[2] / m] } else { v }
    }
}

fn tri_dot(a: [f64; 3], b: [f64; 3]) -> f64 { a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }
fn tri_cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]]
}
fn tri_norm(a: [f64; 3]) -> f64 { (a[0]*a[0] + a[1]*a[1] + a[2]*a[2]).sqrt() }

/// Body-frame unit rows (north, east, down) of the attitude, or `None`
/// when degenerate (zero vectors / field parallel to gravity).
fn triad_rows(acc_mg: [i16; 3], mag_mg: [i16; 3], off: Option<[f64; 3]>)
    -> Option<([f64; 3], [f64; 3], [f64; 3])>
{
    let a = [acc_mg[0] as f64, acc_mg[1] as f64, acc_mg[2] as f64];
    let an = tri_norm(a);
    if an <= 100.0 { return None; }              // < 0.1 g: free-fall/garbage
    let d = [-a[0]/an, -a[1]/an, -a[2]/an];      // accel reads +1 g on the UP axis
    let o = off.unwrap_or([0.0; 3]);
    // The LIS2MDL's Y axis is mirrored relative to the IMU frame on this
    // board (ST "esu" vs "enu" mounting) — the mag-Y flip is a fixed
    // hardware fact, always applied. iOS `Triad.rows` parity.
    let m = [
        mag_mg[0] as f64 - o[0],
        -(mag_mg[1] as f64 - o[1]),
        mag_mg[2] as f64 - o[2],
    ];
    let mn = tri_norm(m);
    if mn <= 20.0 { return None; }               // essentially no field signal
    let mu = [m[0]/mn, m[1]/mn, m[2]/mn];
    let e = tri_cross(d, mu);
    let en = tri_norm(e);
    if en <= 0.05 { return None; }               // field ~parallel to gravity
    let e = [e[0]/en, e[1]/en, e[2]/en];
    let n = tri_cross(e, d);                      // unit by construction
    Some((n, e, d))
}

/// World (north, east, down) coordinates of a body-frame point, with the
/// vertical-axis bias rotation applied. No scene reflection: the mag-Y flip
/// in `triad_rows` already yields the physically-correct handedness (iOS
/// `Triad.world` parity — the user-facing left/right mirror was removed).
fn triad_world(p: [f64; 3], rows: ([f64; 3], [f64; 3], [f64; 3]), bias_deg: f64)
    -> (f64, f64, f64)
{
    let (rn, re, rd) = rows;
    let n0 = tri_dot(rn, p);
    let e0 = tri_dot(re, p);
    let d0 = tri_dot(rd, p);
    let b = bias_deg.to_radians();
    // Rotate the world frame by -bias: azimuths shrink by bias.
    let n1 = n0 * b.cos() + e0 * b.sin();
    let e1 = -n0 * b.sin() + e0 * b.cos();
    (n1, e1, d0)
}

/// Wireframe cuboid rendered from the gyro+accel filter attitude. Fixed
/// map view, screen-up = SOUTH (the box is calibrated with USB-C pointing
/// south, so the green nose arrow then points down/south); N/E/S/W labels
/// (N red); semi-transparent lid; green nose arrow on the confirmed front
/// end. All poses consistent — no Euler chaining. Mirrors iOS
/// `OrientationBoxCanvas`.
fn draw_orientation_box(
    ui: &mut egui::Ui,
    rows: Option<OriRows>,
    bias_deg: f64,
    nose_plus_y: bool,
) {
    let (resp, painter) =
        ui.allocate_painter(egui::vec2(280.0, 200.0), egui::Sense::hover());
    let rect = resp.rect;
    let cx = rect.center().x as f64;
    let cy = rect.center().y as f64;
    let scale = rect.height() as f64 / 3.2;
    let el = 28.0_f64.to_radians();
    let (sel, cel) = (el.sin(), el.cos());

    // Fixed south-up orthographic mapping for a world (north, east, down)
    // point: screen-up = SOUTH, screen-right = WEST.
    let screen = |n: f64, e: f64, d: f64| -> egui::Pos2 {
        egui::pos2(
            (cx + (-e) * scale) as f32,
            (cy - ((-n) * sel - d * cel) * scale) as f32,
        )
    };

    let axis = egui::Stroke::new(1.0, egui::Color32::from_gray(110));
    let font = egui::TextStyle::Body.resolve(ui.style());
    // Ground compass cross (world frame, fixed).
    painter.line_segment([screen(1.35, 0.0, 0.0), screen(-1.35, 0.0, 0.0)], axis);
    painter.line_segment([screen(0.0, 1.35, 0.0), screen(0.0, -1.35, 0.0)], axis);
    painter.text(screen(1.5, 0.0, 0.0), egui::Align2::CENTER_CENTER, "N",
        font.clone(), egui::Color32::from_rgb(211, 47, 47));
    painter.text(screen(0.0, 1.5, 0.0), egui::Align2::CENTER_CENTER, "E",
        font.clone(), egui::Color32::from_gray(160));
    painter.text(screen(-1.5, 0.0, 0.0), egui::Align2::CENTER_CENTER, "S",
        font.clone(), egui::Color32::from_gray(160));
    painter.text(screen(0.0, -1.5, 0.0), egui::Align2::CENTER_CENTER, "W",
        font.clone(), egui::Color32::from_gray(160));

    // Attitude from the gyro+accel filter; flat until it seeds.
    let r = rows.unwrap_or(OriRows {
        n: [1.0, 0.0, 0.0],
        e: [0.0, 1.0, 0.0],
        d: [0.0, 0.0, -1.0],
    });
    let p3 = |p: [f64; 3]| -> egui::Pos2 {
        let (n, e, d) = triad_world(p, (r.n, r.e, r.d), bias_deg);
        screen(n, e, d)
    };

    // Cuboid in the SENSOR frame: z points UP out of the lid (accel reads
    // +1 g on z when flat lid-up), long side = y.
    let (hx, hy, hz) = (0.62, 1.0, 0.28);
    let v: [[f64; 3]; 8] = [
        [hx, hy, hz], [hx, -hy, hz], [-hx, -hy, hz], [-hx, hy, hz],     // lid
        [hx, hy, -hz], [hx, -hy, -hz], [-hx, -hy, -hz], [-hx, hy, -hz], // bottom
    ];
    let pts: Vec<egui::Pos2> = v.iter().map(|&q| p3(q)).collect();

    let side = egui::Stroke::new(1.3, egui::Color32::from_gray(128));
    let lid = egui::Color32::from_rgb(79, 181, 250);
    // Bottom face + verticals.
    for (a, b) in [(4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)] {
        painter.line_segment([pts[a], pts[b]], side);
    }
    // Filled + outlined lid.
    painter.add(egui::Shape::convex_polygon(
        vec![pts[0], pts[1], pts[2], pts[3]],
        egui::Color32::from_rgba_unmultiplied(79, 181, 250, 64),
        egui::Stroke::new(2.0, lid),
    ));
    // Nose arrow on the confirmed front end, on the lid.
    let ny = if nose_plus_y { hy } else { -hy };
    let nose_col = egui::Color32::from_rgb(66, 161, 71);
    let lid_c = p3([0.0, 0.0, hz]);
    let tip = p3([0.0, ny * 1.25, hz]);
    painter.arrow(lid_c, tip - lid_c, egui::Stroke::new(2.0, nose_col));
}

fn draw_sparkline(
    ui: &mut egui::Ui,
    history: &VecDeque<(f64, f64)>,
    color: egui::Color32,
    default_range: (f64, f64),
) {
    let desired = egui::vec2(ui.available_width(), 60.0);
    let (rect, _resp) = ui.allocate_exact_size(desired, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    // Frame
    painter.rect_filled(
        rect,
        4.0,
        egui::Color32::from_rgb(20, 20, 28),
    );
    painter.rect_stroke(
        rect,
        4.0,
        egui::Stroke::new(1.0, egui::Color32::from_gray(60)),
    );
    if history.len() < 2 {
        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            "waiting for samples…",
            egui::FontId::proportional(12.0),
            egui::Color32::from_gray(140),
        );
        return;
    }
    let t_min = history.front().unwrap().0;
    let t_max = history.back().unwrap().0.max(t_min + 1.0);
    let mut v_min = default_range.0;
    let mut v_max = default_range.1;
    for &(_t, v) in history {
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }
    if (v_max - v_min).abs() < 1e-9 {
        v_max = v_min + 1.0;
    }
    // Map sample → rect-local pixel.
    let to_px = |t: f64, v: f64| -> egui::Pos2 {
        let tx = (t - t_min) / (t_max - t_min);
        let vy = (v - v_min) / (v_max - v_min);
        let x = rect.left() + tx as f32 * rect.width();
        // Flip Y so larger values draw higher.
        let y = rect.bottom() - vy as f32 * rect.height();
        egui::pos2(x, y)
    };
    let pts: Vec<egui::Pos2> = history.iter().map(|&(t, v)| to_px(t, v)).collect();
    painter.add(egui::Shape::line(pts, egui::Stroke::new(1.4, color)));
    // Min/max labels in the corners so the user knows the Y scale.
    painter.text(
        egui::pos2(rect.left() + 4.0, rect.top() + 2.0),
        egui::Align2::LEFT_TOP,
        format!("{:.2}", v_max),
        egui::FontId::proportional(10.0),
        egui::Color32::from_gray(180),
    );
    painter.text(
        egui::pos2(rect.left() + 4.0, rect.bottom() - 12.0),
        egui::Align2::LEFT_TOP,
        format!("{:.2}", v_min),
        egui::FontId::proportional(10.0),
        egui::Color32::from_gray(180),
    );
}

#[cfg(target_os = "macos")]
fn open_in_filer(path: &Path) -> std::io::Result<()> {
    Command::new("open").arg(path).status().map(|_| ())
}
#[cfg(target_os = "windows")]
fn open_in_filer(path: &Path) -> std::io::Result<()> {
    Command::new("explorer").arg(path).status().map(|_| ())
}
#[cfg(all(unix, not(target_os = "macos")))]
fn open_in_filer(path: &Path) -> std::io::Result<()> {
    Command::new("xdg-open").arg(path).status().map(|_| ())
}

// ---------------------------------------------------------------------------
//  Entry point
// ---------------------------------------------------------------------------

/// Decode `ICON_PNG` into an egui-friendly RGBA image. Returns `None`
/// on decode failure — the GUI still works, it just shows no logo.
fn decode_icon() -> Option<egui::ColorImage> {
    let img = image::load_from_memory(ICON_PNG).ok()?.into_rgba8();
    let (w, h) = img.dimensions();
    Some(egui::ColorImage::from_rgba_unmultiplied(
        [w as usize, h as usize],
        img.as_raw(),
    ))
}

/// Decode the icon into the format eframe wants for the OS-level
/// window icon (Dock on macOS, taskbar on Windows / Linux).
fn os_window_icon() -> Option<egui::IconData> {
    let img = image::load_from_memory(ICON_PNG).ok()?.into_rgba8();
    let (w, h) = img.dimensions();
    Some(egui::IconData {
        rgba: img.into_raw(),
        width: w,
        height: h,
    })
}

fn main() -> eframe::Result<()> {
    /* argv dispatch (issue #14 part B). The headless agent path must
       return before ANY eframe/egui/winit code so the macOS winit
       patch invariant holds and no window is created. */
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|a| a == "--agent") {
        std::process::exit(agent::run());
    }
    /* Headless one-shot firmware upload (`--flash-firmware <path.bin>`).
       Like --agent, returns before any eframe/egui/winit code so no
       window is created. */
    if let Some(pos) = args.iter().position(|a| a == "--flash-firmware") {
        let Some(path) = args.get(pos + 1) else {
            eprintln!("--flash-firmware: missing <path.bin> argument");
            std::process::exit(2);
        };
        std::process::exit(agent::flash(std::path::Path::new(path)));
    }
    /* Headless per-boot health check of the mirrored box error log
       (`--check-errlog [path]`). Default path prefers the persisted
       save_dir from ~/.movementlogger/config.toml — where the GUI/agent
       actually mirror — over the cwd-derived save base, so a cron
       invocation grades the same file the app writes. Prints one graded
       section per box boot; exit 0/1/2 = latest boot OK/WARN/FAIL,
       3 = unreadable / not an errlog. Like --agent, returns before any
       window code. */
    if let Some(pos) = args.iter().position(|a| a == "--check-errlog") {
        let path = args.get(pos + 1).map(PathBuf::from).unwrap_or_else(|| {
            // config.toml's save_dir IS the download (csv) dir — the
            // agent assigns it to ble_out_dir verbatim (agent.rs).
            let cfg_dir = agent_config::AgentConfig::load().save_dir;
            if cfg_dir.trim().is_empty() {
                default_save_base().join("csv").join("ERRLOG.LOG")
            } else {
                PathBuf::from(cfg_dir).join("ERRLOG.LOG")
            }
        });
        std::process::exit(errlog_check::run_cli(&path));
    }
    /* Headless BLE diagnostic (`--ble-debug [seconds]`) — the GPS-Debug
       analogue for the BLE side: prints every CoreBluetooth discovery
       event live, then a retrieve-by-id test and a bounded connect
       probe. Like --agent, returns before any window code. */
    if let Some(pos) = args.iter().position(|a| a == "--ble-debug") {
        let secs = args
            .get(pos + 1)
            .and_then(|s| s.parse::<u64>().ok())
            .unwrap_or(20);
        std::process::exit(ble_debug::run(secs));
    }
    // Autostart (de)registration entry points — invoked in a clean
    // child by the updater/agent so the login item always points at
    // the right (post-swap) bundle. No window.
    if args.iter().any(|a| a == "--register-autostart") {
        std::process::exit(match autostart::register() {
            Ok(()) => 0,
            Err(e) => {
                eprintln!("register-autostart: {e}");
                1
            }
        });
    }
    if args.iter().any(|a| a == "--unregister-autostart") {
        std::process::exit(match autostart::unregister() {
            Ok(()) => 0,
            Err(e) => {
                eprintln!("unregister-autostart: {e}");
                1
            }
        });
    }

    /* If the box is in AUTO, (re-)install the login item on every GUI
       launch. Idempotent, and crucially it re-points the plist /
       .desktop / Run-key at the *current* bundle after an in-app
       update swapped it — and on macOS bootstraps a fresh agent from
       the new code without waiting for a re-login. Best-effort. */
    if agent_config::AgentConfig::load().log_mode_manual == Some(false) {
        let _ = autostart::register();
    }

    let title = format!("MovementLogger v{}", env!("CARGO_PKG_VERSION"));
    let mut viewport = egui::ViewportBuilder::default()
        .with_inner_size([880.0, 720.0])
        .with_min_inner_size([560.0, 480.0])
        .with_title(&title);
    if let Some(icon) = os_window_icon() {
        viewport = viewport.with_icon(icon);
    }
    let options = eframe::NativeOptions {
        viewport,
        ..Default::default()
    };
    eframe::run_native(
        &title,
        options,
        Box::new(|cc| {
            // Readability: egui's default text styles (12.5 px body) read
            // small on macOS. Scale every style up ~20% once at startup;
            // Ctrl/Cmd +/- zoom still works on top of this baseline.
            let mut style = (*cc.egui_ctx.style()).clone();
            for font in style.text_styles.values_mut() {
                font.size = (font.size * 1.2).round();
            }
            cc.egui_ctx.set_style(style);
            Ok(Box::new(AppState::new()))
        }),
    )
}
