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
mod coord;
mod installer;
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

use ble::{BleBackend, BleCmd, LiveSample};
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
}

impl Default for Tab {
    fn default() -> Self { Tab::Live }
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
    live_acc_history: &'a mut VecDeque<(f64, f64)>,
    live_pressure_history: &'a mut VecDeque<(f64, f64)>,
    live_t0_ms: &'a mut Option<u32>,
    live_sample_count: &'a mut u64,
    sensor_csv: &'a mut Option<PathBuf>,
    gps_csv: &'a mut Option<PathBuf>,
}

impl SyncHost for GuiSyncHost<'_> {
    fn on_link_reset(&mut self) {
        /* Drop live state — the box can drop the stream mid-connection
           (e.g. flaky link) and the next reconnect should start from a
           clean history, not graft new samples onto stale chart data. */
        *self.latest_sample = None;
        *self.latest_sample_at = None;
        self.live_acc_history.clear();
        self.live_pressure_history.clear();
        *self.live_t0_ms = None;
        *self.live_sample_count = 0;
    }

    fn on_sample(&mut self, s: &LiveSample) {
        *self.latest_sample = Some(*s);
        *self.latest_sample_at = Some(std::time::Instant::now());
        *self.live_sample_count = self.live_sample_count.saturating_add(1);
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
        Self {
            output_dir,
            tz_offset_h: 3.0,
            fps: 15,
            sync: SyncCore {
                ble_out_dir,
                ble_session_duration_s: 1800, // 30-min default
                log: log.clone(),
                ..Default::default()
            },
            log,
            update_rx: Some(spawn_update_check()),
            update_checking: true,
            ..Self::default()
        }
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
            live_acc_history,
            live_pressure_history,
            live_t0_ms,
            live_sample_count,
            sensor_csv,
            gps_csv,
            ..
        } = self;
        let mut host = GuiSyncHost {
            latest_sample,
            latest_sample_at,
            live_acc_history,
            live_pressure_history,
            live_t0_ms,
            live_sample_count,
            sensor_csv,
            gps_csv,
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
                                "Update available: v{} → {}",
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
                        ui.label(egui::RichText::new(p.detail).weak().small());
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
    /* Header strip with a "Select all" toggle on the right. The
       toggle reads the common state of the group (all on / all off /
       mixed) so the user can flip the whole section in one click. */
    let all_on  = indices.iter().all(|&i| files[i].selected);
    let none_on = indices.iter().all(|&i| !files[i].selected);
    ui.horizontal(|ui| {
        ui.add(egui::Label::new(
            egui::RichText::new(title).strong().size(13.0)
        ));
        ui.label(egui::RichText::new(format!("({})", indices.len()))
            .small()
            .color(egui::Color32::from_gray(140)));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            let label = if all_on { "Untick all" }
                        else if none_on { "Tick all" }
                        else { "Tick all" };
            if ui.small_button(label).clicked() {
                let new_val = !all_on;
                for &i in indices { files[i].selected = new_val; }
            }
        });
    });
    for &i in indices {
        let f = &mut files[i];
        ui.horizontal(|ui| {
            ui.checkbox(&mut f.selected, "");
            ui.label(format!("{:>10} B  {}", f.size, f.name));
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
            ui.add(
                egui::ProgressBar::new(frac)
                    .desired_width(360.0)
                    .text(bytes_str),
            );
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
            )
            .small()
            .color(egui::Color32::from_gray(180)),
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
                        let b = self.ensure_ble();
                        b.send(BleCmd::Scan);
                    }
                    if ui
                        .add_enabled(connected, egui::Button::new("Refresh file list"))
                        .clicked()
                    {
                        self.sync.ble_files.clear();
                        if let Some(b) = self.sync.ble.as_ref() { b.send(BleCmd::List); }
                    }
                    if ui
                        .add_enabled(
                            connected && !self.sync.ble_dl_in_flight && !self.sync.ble_sync_pending,
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
                    if ui
                        .add_enabled(connected, egui::Button::new("Disconnect"))
                        .clicked()
                    {
                        if let Some(b) = self.sync.ble.as_ref() {
                            // Break any in-progress auto-reconnect loop
                            // first — the worker can't read the command
                            // channel while inside it, so the abort flag
                            // is the only thing it polls there.
                            b.request_abort();
                            b.send(BleCmd::Disconnect);
                        }
                        /* Optimistic state transition: drop straight to
                           Idle without waiting for the worker's eventual
                           Disconnected event. `peripheral.disconnect()`
                           can take a few seconds on macOS CoreBluetooth
                           while the LL_TERMINATE_IND propagates — without
                           this, Scan stays disabled in that window. The
                           worker's Disconnected event will arrive later
                           and is idempotent (sets state to Idle again). */
                        self.sync.ble_state = BleState::Idle;
                        self.sync.ble_files.clear();
                        self.sync.ble_dl_queue.clear();
                        self.sync.ble_dl_in_flight = false;
                        self.sync.ble_connected_id = None;
                        self.sync.ble_sync_pending = false;
                        self.sync.ble_resume_box = None; // intentional disconnect — don't auto-resume
                        self.sync.ble_status = "disconnecting…".into();
                        push_log(&self.log, "ble: Disconnect requested".into());
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if !self.sync.ble_status.is_empty() {
                            /* Selectable so the user can copy error text
                               (egui Labels are non-selectable by default). */
                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new(&self.sync.ble_status)
                                        .small()
                                        .color(egui::Color32::LIGHT_BLUE),
                                )
                                .selectable(true),
                            );
                        }
                    });
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

                // ----- File list / download ------------------------
                if matches!(self.sync.ble_state, BleState::Connected) {
                    if !self.sync.ble_sync_msg.is_empty() {
                        ui.add_space(4.0);
                        ui.colored_label(
                            egui::Color32::from_rgb(120, 200, 140),
                            &self.sync.ble_sync_msg,
                        );
                    }
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
                                    egui::RichText::new(format!("→ {}", abs.display()))
                                        .small()
                                        .color(egui::Color32::from_gray(150)),
                                );
                            }
                        }
                    }

                    if self.sync.ble_files.is_empty() {
                        ui.label(
                            egui::RichText::new("No file list yet — hit Refresh.")
                                .small()
                                .color(egui::Color32::from_gray(170)),
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

                        egui::ScrollArea::vertical()
                            .max_height(220.0)
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
    fn ui_live_tab(&mut self, ui: &mut egui::Ui) {
        ui.heading("Live");
        ui.label(
            egui::RichText::new(
                "SensorStream — 0.5 Hz packed all-sensor snapshot \
                 (IMU + mag + baro + GPS).",
            )
            .small()
            .color(egui::Color32::from_gray(180)),
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
                    )
                    .color(egui::Color32::from_gray(200)),
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
        let gyro_dps = |i: usize| s.gyro_cdps[i] as f32 / 100.0;
        let pres_hpa = s.pressure_pa as f64 / 100.0;
        let temp_c = s.temperature_cc as f32 / 100.0;

        // Tilt-compensated orientation from the gravity + magnetic
        // vectors. Pitch/roll come straight out of the accel reading
        // (assumes the box is roughly static / 1 g, which is true at
        // 0.5 Hz for a board on the water between strokes). Heading is
        // the mag vector projected onto the local horizontal plane
        // after de-rotating by pitch/roll — the standard "eCompass"
        // formula. Result is in degrees; heading is wrapped to 0..360.
        let (pitch_deg, roll_deg, heading_deg) = {
            let ax = acc_g(0) as f64;
            let ay = acc_g(1) as f64;
            let az = acc_g(2) as f64;
            let roll  = ay.atan2(az);
            let pitch = (-ax).atan2((ay * ay + az * az).sqrt());
            let mx = s.mag_mg[0] as f64;
            let my = s.mag_mg[1] as f64;
            let mz = s.mag_mg[2] as f64;
            let (sr, cr) = (roll.sin(), roll.cos());
            let (sp, cp) = (pitch.sin(), pitch.cos());
            let xh = mx * cp + mz * sp;
            let yh = mx * sr * sp + my * cr - mz * sr * cp;
            let mut heading = (-yh).atan2(xh).to_degrees();
            if heading < 0.0 { heading += 360.0; }
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
        if self.sync.ble_keep_synced
            && matches!(self.sync.ble_state, BleState::Connected)
            && !self.sync.ble_sync_pending
            && !self.sync.ble_dl_in_flight
            && self.sync.ble_dl_queue.is_empty()
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
                        "https://github.com/zdavatz/fp-sns-stbox1",
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
            ui.horizontal(|ui| {
                for (tab, label) in [
                    (Tab::Live,   "Live"),
                    (Tab::Sync,   "Sync"),
                    (Tab::Replay, "Replay"),
                ] {
                    let selected = self.current_tab == tab;
                    // SelectableLabel gives us the standard egui
                    // pressed-look without the heavy CollapsingHeader
                    // chrome. Click switches tabs.
                    if ui.add(egui::SelectableLabel::new(selected, label)).clicked() {
                        self.current_tab = tab;
                    }
                }
            });
            ui.add_space(2.0);
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.add_space(6.0);

            // ----- Update banner --------------------------------------
            // Shared across all tabs so a user on Live still sees
            // "Update available" without having to switch panes.
            self.render_update_banner(ui);

            match self.current_tab {
                Tab::Live   => { self.ui_live_tab(ui);   return; }
                Tab::Sync   => { self.ui_sync_tab(ui);   return; }
                Tab::Replay => { /* fall through to original layout */ }
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
                            )
                            .color(egui::Color32::from_gray(200)),
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
                        ui.label(egui::RichText::new(msg).weak().small());
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
        Box::new(|_cc| Ok(Box::new(AppState::new()))),
    )
}
