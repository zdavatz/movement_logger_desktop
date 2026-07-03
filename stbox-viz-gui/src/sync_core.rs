//! UI-independent sync core.
//!
//! Step 1 (issue #14 part B): the pure, egui-free helpers the desktop
//! sync engine is built on, lifted verbatim out of `main.rs` so the
//! upcoming headless `--agent` can reuse the exact same mirror /
//! save-dir / name-classification logic the GUI uses. No behaviour
//! change — these moved without edits. Later steps add `SyncCore`
//! (the driver state + methods) here too.

use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use crate::agent_config::AgentConfig;
use crate::ble::{BleBackend, BleCmd, BleEvent, LiveSample};
use crate::sync_db;

/// Append a line to the shared log buffer (the GUI's scrollable log
/// panel; the headless agent drains the same buffer to stdout/file).
/// Capped so a long-running session can't exhaust RAM.
pub fn push_log(log: &Arc<Mutex<Vec<String>>>, line: String) {
    // Mirror every panel line to stderr so a `MovementLogger 2>log` run is
    // diagnosable from outside the GUI (the in-app panel can't be scraped).
    eprintln!("[log] {line}");
    if let Ok(mut v) = log.lock() {
        const CAP: usize = 10_000;
        if v.len() >= CAP {
            v.drain(..CAP / 4);
        }
        v.push(line);
    }
}

/// GUI-only side-effects the sync engine triggers but `SyncCore` itself
/// must not own (they touch egui Live-tab chart state / the Replay
/// form). The GUI implements this over borrows of those fields; the
/// headless `--agent` uses a no-op impl.
pub trait SyncHost {
    /// A BLE link dropped — discard any live-stream chart history so a
    /// reconnect starts clean (no-op for the agent).
    fn on_link_reset(&mut self);
    /// One decoded SensorStream snapshot arrived (GUI updates the Live
    /// charts; the agent ignores it — it never subscribes to Live).
    fn on_sample(&mut self, s: &LiveSample);
    /// A file finished downloading — auto-route it into the Replay
    /// form slots (no-op for the agent).
    fn on_downloaded(&mut self, name: &str, path: &Path);
}

#[derive(Clone, Debug)]
pub struct BleDevice {
    pub id: String,
    pub name: String,
    pub rssi: Option<i16>,
}

#[derive(Clone, Debug)]
pub struct BleFile {
    pub name: String,
    pub size: u64,
    pub selected: bool,
    pub downloaded: bool,
    /// Bytes received so far on the current read. Reset to 0 on
    /// ReadStarted, advanced on each ReadProgress notify, and bumped to
    /// `size` on ReadDone. Drives the per-row progress bar.
    pub bytes_done: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum BleState {
    #[default]
    Idle,
    Scanning,
    Connecting,
    Connected,
}

/// UI-independent BLE-FileSync + Sync engine state. Lifted verbatim out
/// of `AppState` (issue #14 part B step 2) so the headless `--agent`
/// can drive the same engine the GUI does. `AppState` now owns one of
/// these as `self.sync`; the GUI reads/writes its fields for display
/// exactly as before. Step 3 moves the engine *methods* onto this
/// struct; for now only the fields moved (no behaviour change).
#[derive(Default)]
pub struct SyncCore {
    // ----- BLE FileSync state ------------------------------------------
    /// Worker-thread backend. Lazily spawned on first BLE button click
    /// so the tokio runtime doesn't start unless the user actually opens
    /// the panel.
    pub ble: Option<BleBackend>,
    /// Discovered PumpTsueri peripherals from the most recent scan.
    pub ble_devices: Vec<BleDevice>,
    /// Connection lifecycle state — drives which buttons are enabled.
    pub ble_state: BleState,
    /// File listing returned by the most recent LIST. Each row carries
    /// a checkbox the user can flip before hitting "Download selected".
    pub ble_files: Vec<BleFile>,
    /// One-line status badge (last `Status`/`Error` event from the worker).
    pub ble_status: String,
    /// Where downloaded files land. Defaults to a `csv/` subfolder so
    /// they slot straight into the existing visualisation path.
    pub ble_out_dir: PathBuf,
    /// Session duration (seconds) the user types into the "Start session"
    /// field. Sent as the START_LOG payload (issue #15). Default 1800 s
    /// = 30 minutes.
    pub ble_session_duration_s: u32,
    /// Serial download queue. "Download selected" pushes all ticked files
    /// here; the head is sent to the worker. The next entry is popped and
    /// sent on ReadDone or on a READ-side error event. Required because
    /// the worker has a single in-flight slot and blasting all reads in
    /// one shot rejects all but the first with "another op is in flight".
    pub ble_dl_queue: VecDeque<(String, u64)>,
    /// True iff a Read is in flight in the worker (Download or queue head).
    /// Drives queue advancement when a Read completes or errors out.
    pub ble_dl_in_flight: bool,
    /// `(start_instant, duration_seconds)` while a LOG session is running
    /// on the box. Set when Start session is clicked, cleared when the
    /// remaining time falls to zero (after which the box should be back
    /// in BLE mode and Scan-able). Drives the countdown banner that
    /// replaces the file list while the box is silent.
    pub ble_session_running: Option<(std::time::Instant, u32)>,
    /// The box's persisted log mode, from a GET_MODE reply (queried on
    /// every Connect) or a confirmed SET_MODE. `None` = unknown / not
    /// yet answered / legacy `PumpTsueri` firmware (no GET_MODE);
    /// `Some(false)` = AUTO (box logs on power-on); `Some(true)` =
    /// MANUAL (box idle until START_LOG). Mirrors the Android
    /// `FileSyncUiState.logModeManual`.
    pub ble_log_mode: Option<bool>,

    // ----- Sync (vs. plain transfer) -----------------------------------
    /// Peripheral id of the box we're connected to, captured at Connect
    /// time. Used as the per-box key in the sync DB so two different
    /// boxes don't share sync history. Cleared on disconnect.
    pub ble_connected_id: Option<String>,
    /// Last box id we connected to, **kept across disconnect** (unlike
    /// `ble_connected_id`). Backs the Sync-tab "Reconnect (last box)" button so
    /// a Mac→Mac reconnect works even when CoreBluetooth scan-suppresses the
    /// recently-connected box (the manual connect then goes through the
    /// retrieve-by-id fallback in `ble.rs::connect_core`). Seeded from the
    /// persisted `config.toml` box_id on startup so it survives an app restart.
    pub ble_last_box_id: Option<String>,
    /// Lazily-opened local sync-state DB (`~/.movementlogger/sync.db`).
    /// `None` until the first sync; stays `None` (sync disabled, msg
    /// set) if the DB can't be opened.
    pub sync_db: Option<sync_db::SyncDb>,
    /// True between a "Sync now" click and the LIST it triggers coming
    /// back: the ListDone handler runs the diff only when this is set,
    /// so the auto-LIST on Connect doesn't accidentally start a sync.
    pub ble_sync_pending: bool,
    /// "Keep synced" checkbox: while connected and idle, re-run a sync
    /// pass every `SYNC_POLL_INTERVAL` so a continuously-growing log
    /// keeps mirroring (each pass only fetches the new tail).
    pub ble_keep_synced: bool,
    /// When the last sync pass was *triggered*. The continuous loop
    /// schedules the next pass `SYNC_POLL_INTERVAL` after this — so a
    /// busy backlog runs back-to-back while an idle box is polled
    /// every 30 s, not hammered.
    pub ble_last_sync_at: Option<std::time::Instant>,
    /// One-line sync result/status ("Sync: 3 new, 12 already synced —
    /// downloading…"), shown in the Sync tab. Empty = no sync yet.
    pub ble_sync_msg: String,
    /// Last "Save to" validation failure, shown as a prominent red
    /// banner under the field. `Some` blocks Download/Sync so a bad
    /// path can't silently no-op (the original bug: a relative or
    /// macOS-TCC-blocked path made the app *say* it saved while the
    /// file went nowhere the user expected). Cleared on a good path.
    pub ble_save_err: Option<String>,
    /// Box id of an interrupted transfer/sync to auto-resume on the
    /// next reconnect to that same box. Set when the BLE link drops
    /// mid-batch (timeout / disconnect / shielded radio); consumed in
    /// the `Connected` handler, which re-arms a sync — the SQLite DB
    /// makes that skip everything already saved and re-pull only the
    /// file that was cut off. `None` = nothing to resume.
    pub ble_resume_box: Option<String>,
    /// Name of the file the *sync* (or manual queue) is currently pulling.
    /// Set when [`Self::advance_download_queue`] pops the next file onto
    /// the worker, cleared on ReadDone / arm_sync_resume. Used to find
    /// the per-file `bytes_done` for [`Self::sync_cumulative_bytes`] —
    /// the queue is popped *before* the READ is sent, so we can't look
    /// the in-flight file up there.
    pub sync_in_flight_name: Option<String>,
    /// Total file count of the current sync pass — set at `run_sync_diff`,
    /// reset on completion / abort. Renders "Syncing X of N files" in the
    /// cumulative progress line (iOS/Android parity).
    pub sync_pass_total: usize,
    /// Total byte count of the current sync pass (sum of every queued
    /// file's box-reported size). Constant across the pass — denominator
    /// for the cumulative progress bar.
    pub sync_pass_total_bytes: u64,
    /// Bytes drained — sum of completed files' final on-disk sizes. The
    /// in-flight file's contribution comes from `ble_files[…].bytes_done`
    /// via [`Self::sync_cumulative_bytes`] so the bar moves while the
    /// current READ streams.
    pub sync_pass_completed_bytes: u64,
    /// Last DELETE rejection from the box (NOT_FOUND / BUSY / IO_ERROR
    /// / BAD_REQUEST). Shown as a prominent red banner so a failed
    /// delete isn't only buried in the log. Cleared on a successful
    /// delete, a new delete attempt, or disconnect.
    pub ble_delete_err: Option<String>,

    // ----- Firmware upload (OTA) ---------------------------------------
    /// `Some((bytes_done, total))` while a firmware upload is in flight —
    /// set on `FlashStarted`, advanced on `FlashProgress`, cleared on
    /// `FlashDone` / `FlashError` / disconnect. Drives the flash progress
    /// bar and disables the other BLE actions for its duration.
    pub ble_flash_progress: Option<(u64, u64)>,
    /// Last firmware-upload result line for the Sync tab ("Firmware sent
    /// — box rebooting…" on success, the mapped error on failure). Empty
    /// = no upload attempted yet this session.
    pub ble_flash_msg: String,
    /// Shared log sink. The GUI shares its scrollable-panel `Arc` here
    /// so BLE/sync lines land in the same buffer as before; the agent
    /// points it at a stdout/file drain.
    pub log: Arc<Mutex<Vec<String>>>,
}

/// Live-mirror download assembly.
///
/// The local file `<dir>/<name>` *is* the running mirror — we
/// accumulate straight into it, no `.part`/rename. The box's log files
/// grow continuously while a session runs, so "done" is a moving
/// target; what matters is that the local file is always a valid
/// prefix of the box file (the logs are append-only) and we only ever
/// fetch the bytes we don't have yet. This makes the local size the
/// single source of truth for the resume/grow offset — survives an
/// app restart, a dropped link, and "the file got bigger since last
/// sync" identically.
pub fn mirror_path(dir: &Path, name: &str) -> PathBuf {
    dir.join(name)
}

/// Where to resume `name` from given the box's current size.
/// - local missing            → 0 (fresh)
/// - local smaller than box   → local size (resume / fetch the delta)
/// - local == box             → equal (caller treats as up-to-date)
/// - local larger than box    → the file rotated (session reused the
///   name, box file is now shorter) → truncate local, restart at 0
pub fn mirror_offset(dir: &Path, name: &str, box_size: u64) -> u64 {
    let p = mirror_path(dir, name);
    match std::fs::metadata(&p) {
        Ok(m) if m.len() <= box_size => m.len(),
        Ok(_) => {
            // Rotated/shorter on the box — old local copy is stale.
            let _ = std::fs::remove_file(&p);
            0
        }
        Err(_) => 0,
    }
}

/// Append a streamed segment to `<dir>/<name>` (creating it). `base`
/// is the offset the segment started at; if it doesn't match the
/// current file length the resume is misaligned (offset is derived
/// from this same file's length before the READ) — realign to `base`
/// so we never interleave a corrupt prefix.
pub fn append_mirror(dir: &Path, name: &str, base: u64, bytes: &[u8]) -> std::io::Result<u64> {
    use std::io::{Seek, SeekFrom, Write};
    std::fs::create_dir_all(dir)?;
    let p = mirror_path(dir, name);
    let mut f = std::fs::OpenOptions::new()
        .create(true)
        .read(true)
        .write(true)
        .open(&p)?;
    let cur = f.metadata()?.len();
    if cur != base {
        f.set_len(base.min(cur))?;
    }
    f.seek(SeekFrom::End(0))?;
    f.write_all(bytes)?;
    f.flush()?;
    Ok(f.metadata()?.len())
}

/// Base dir for the default `csv/` (downloads) and `gif/` (renders)
/// folders. A macOS `.app` launched from Finder/Dock starts with
/// `current_dir()` == `/` — a read-only volume — so the historical
/// `cwd.join("csv")` became `/csv` and every download died with
/// `Can't create "/csv": Read-only file system (os error 30)`
/// (Peter, v0.0.20 on Mac). Use cwd only when it's a real writable
/// working dir (the Linux/Windows `./MovementLogger` case — behaviour
/// unchanged there); otherwise anchor under `$HOME/MovementLogger`,
/// same `$HOME` rationale as the sync DB. The user can still override
/// the folder in the Sync tab.
pub fn default_save_base() -> PathBuf {
    if let Ok(cwd) = std::env::current_dir() {
        if cwd != Path::new("/") && probe_save_dir(&cwd).is_ok() {
            return cwd;
        }
    }
    std::env::var_os("HOME")
        .or_else(|| std::env::var_os("USERPROFILE"))
        .map(PathBuf::from)
        .map(|h| h.join("MovementLogger"))
        .unwrap_or_else(|| PathBuf::from("."))
}

/// Expand a leading `~` and require an absolute path. Pure path math,
/// no filesystem access — cheap enough to call every frame for the
/// live indicator. The absolute-path requirement is the core fix for
/// the silent-misplace bug: `PathBuf::from("~/Downloads")` or a bare
/// relative path is otherwise created *relative to the process cwd*,
/// so the app reports `saved <path>` while the file is nowhere the
/// user is looking.
pub fn resolve_save_dir(raw: &Path) -> Result<PathBuf, String> {
    let s = raw.to_string_lossy();
    let s = s.trim();
    if s.is_empty() {
        return Err("Save-to path is empty — pick a folder with Browse….".into());
    }
    let expanded: PathBuf = if s == "~" || s.starts_with("~/") {
        let home = std::env::var_os("HOME")
            .or_else(|| std::env::var_os("USERPROFILE"))
            .map(PathBuf::from)
            .ok_or("can't expand ~ — no HOME / USERPROFILE set")?;
        if s == "~" { home } else { home.join(&s[2..]) }
    } else {
        PathBuf::from(s)
    };
    if !expanded.is_absolute() {
        return Err(format!(
            "\u{201c}{}\u{201d} is not an absolute path. Use a full path \
             like /Users/you/Downloads (Browse… fills one in).",
            expanded.display()
        ));
    }
    Ok(expanded)
}

/// Authoritative writability check: actually create the directory and
/// write+delete a probe file. Called at Download/Sync time (not every
/// frame). Surfaces macOS TCC / permission denials that the path-only
/// check can't see — exactly the failures that used to look like a
/// successful save.
pub fn probe_save_dir(dir: &Path) -> Result<(), String> {
    std::fs::create_dir_all(dir)
        .map_err(|e| format!("Can't create \u{201c}{}\u{201d}: {e}", dir.display()))?;
    let probe = dir.join(".movementlogger_write_test");
    match std::fs::write(&probe, b"ok") {
        Ok(()) => {
            let _ = std::fs::remove_file(&probe);
            Ok(())
        }
        Err(e) => {
            let hint = if e.kind() == std::io::ErrorKind::PermissionDenied {
                " — macOS may be blocking it (TCC): grant the app access under \
                 System Settings → Privacy & Security → Files and Folders / \
                 Full Disk Access, or pick a folder you own."
            } else {
                ""
            };
            Err(format!(
                "Folder \u{201c}{}\u{201d} is not writable: {e}{hint}",
                dir.display()
            ))
        }
    }
}

/// True for the four sensor-data row types the firmware creates per
/// logging session (Sens*.csv, Gps*.csv, Bat*.csv, Mic*.wav). Used both
/// for the default-ticked state in the file list and for splitting the
/// list into the Sensor / Debug groups. macOS AppleDouble sidecars
/// (`._<name>`) match the inner pattern by accident, so guard the
/// prefix explicitly.
pub fn is_sensor_data_name(name: &str) -> bool {
    let lower = name.to_ascii_lowercase();
    if lower.starts_with("._") { return false; }
    (lower.starts_with("sens") && lower.ends_with(".csv"))
        || (lower.starts_with("gps")  && lower.ends_with(".csv"))
        || (lower.starts_with("bat")  && lower.ends_with(".csv"))
        || (lower.starts_with("mic")  && lower.ends_with(".wav"))
}

/// Files the *Sync* pass mirrors: every per-session sensor-data file
/// (`is_sensor_data_name`) **plus** the firmware's single rolling error
/// log `ERRLOG.LOG` (`movement_logger_firmware/Src/errlog.c`). Peter
/// needs the errlog to debug box-side stalls, and it's append-only just
/// like the session logs, so the same live-mirror byte-resume path
/// applies unchanged. Deliberately kept separate from
/// `is_sensor_data_name` so the manual file list's default-tick and the
/// Sensor/Debug split are untouched — `ERRLOG.LOG` still lands in the
/// Debug group and stays un-ticked there; only *Sync now* / Keep-synced
/// pull it. Match is case-insensitive (firmware writes it upper-case;
/// macOS/FAT may surface either case).
pub fn is_synced_name(name: &str) -> bool {
    is_sensor_data_name(name) || name.eq_ignore_ascii_case("ERRLOG.LOG")
}

impl SyncCore {
    /// Live cumulative bytes pulled in the current sync pass — completed
    /// files' final on-disk sizes + the in-flight file's `bytes_done`.
    /// Use this for the overall progress-bar numerator. Matches iOS
    /// `syncCumulativeBytes` / Android `syncCumulativeBytes`.
    pub fn sync_cumulative_bytes(&self) -> u64 {
        let in_flight = self
            .sync_in_flight_name
            .as_ref()
            .and_then(|n| self.ble_files.iter().find(|f| f.name == *n))
            .map(|f| f.bytes_done)
            .unwrap_or(0);
        self.sync_pass_completed_bytes + in_flight
    }

    /// 0…1 overall progress of the in-flight sync pass.
    pub fn sync_cumulative_fraction(&self) -> f32 {
        if self.sync_pass_total_bytes == 0 { 0.0 } else {
            (self.sync_cumulative_bytes() as f32 / self.sync_pass_total_bytes as f32)
                .clamp(0.0, 1.0)
        }
    }

    /// Snapshot the agent-relevant state into `~/.movementlogger/
    /// config.toml`. Called by the GUI whenever box id / save dir /
    /// keep-synced / box log mode changes, so the headless `--agent`
    /// always has a current target. Best-effort; a write failure is
    /// logged, not fatal.
    pub fn persist_config(&self) {
        // Load-modify-save: fields the sync engine doesn't own (e.g.
        // the Live tab's mag_offset_mg calibration) must survive.
        let mut cfg = AgentConfig::load();
        cfg.box_id = self.ble_connected_id.clone();
        cfg.save_dir = self.ble_out_dir.to_string_lossy().into_owned();
        cfg.keep_synced = self.ble_keep_synced;
        cfg.log_mode_manual = self.ble_log_mode;
        if let Err(e) = cfg.save() {
            push_log(&self.log, format!("config: save failed: {e}"));
        }
    }

    /// Drain pending BLE events into the visible state. Called once per
    /// frame; egui repaints on a timer while a BLE op is in progress so
    /// events don't sit in the channel for long.
    pub fn pump_ble_events(&mut self, host: &mut dyn SyncHost) {
        let Some(b) = self.ble.as_ref() else { return; };
        let events = b.try_recv_all();
        for e in events {
            match e {
                BleEvent::Status(s) => {
                    /* Mirror Status events into the Log so the user has a
                       scrollable history of BLE state transitions — the
                       single-line ble_status badge was the only surface
                       before, which made it easy to miss a transient
                       message like "LIST sent" or "STOP_LOG sent". Skip
                       the high-frequency "reading X: N B" progress
                       chatter — that already has the per-row progress
                       bar; tee-ing it would drown out everything else. */
                    if !s.starts_with("reading ") {
                        push_log(&self.log, format!("ble: {s}"));
                    }
                    self.ble_status = s;
                }
                BleEvent::Discovered { id, name, rssi } => {
                    if !self.ble_devices.iter().any(|d| d.id == id) {
                        self.ble_devices.push(BleDevice { id, name, rssi });
                    }
                }
                BleEvent::ScanStopped => {
                    self.ble_state = BleState::Idle;
                    self.ble_status = format!("scan done ({} found)", self.ble_devices.len());
                }
                BleEvent::Connected => {
                    self.ble_state = BleState::Connected;
                    self.ble_status = "connected".into();
                    /* Auto-refresh the file list on every successful
                       Connect (initial or post-reconnect). Previously the
                       user had to click Refresh manually — easy to miss
                       after a Disconnect+Reconnect cycle because the
                       list pane already showed "No file list yet — hit
                       Refresh." which looked like a transient state. */
                    self.ble_files.clear();
                    self.ble_sync_msg.clear();
                    /* Resume an interrupted transfer/sync. If we just
                       reconnected to the same box that was cut off mid-
                       batch, arm a sync: the auto-LIST below produces a
                       ListDone, and with ble_sync_pending set that runs
                       run_sync_diff — which the SQLite DB makes skip
                       everything already saved and re-pull only the
                       file the drop interrupted. Resume only for the
                       same box; a different box clears the flag so a
                       stale resume can't fire later. */
                    if self.ble_resume_box.is_some()
                        && self.ble_resume_box == self.ble_connected_id
                    {
                        self.ble_sync_pending = true;
                        self.ble_sync_msg =
                            "Reconnected — resuming sync (skipping files already saved)…"
                                .into();
                        push_log(&self.log, "ble: reconnect — resuming interrupted sync".into());
                    }
                    self.ble_resume_box = None;
                    if let Some(b) = self.ble.as_ref() {
                        /* Stamp the box's open Sens/Gps CSVs with the host
                           wall clock on every connect (SET_TIME 0x08): the box
                           has no RTC, so this is what lets replay time-align
                           without a GPS fix. Sample the epoch right before
                           sending so it matches the box tick the firmware
                           stamps. The worker sleeps 500 ms after SET_TIME, so
                           the LIST below doesn't clobber it in the firmware's
                           single command buffer. Fire-and-forget — legacy
                           firmware ignores 0x08. */
                        let epoch_ms = std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .map(|d| d.as_millis() as u64)
                            .unwrap_or(0);
                        b.send(BleCmd::SetTime { epoch_ms });
                        b.send(BleCmd::List);
                    }
                }
                BleEvent::Disconnected => {
                    /* If a transfer/sync was live when the link dropped,
                       remember which box so the next reconnect resumes
                       it (via the DB diff). Capture before the field
                       clears below. arm_sync_resume is idempotent with
                       the Error-handler path that may have fired first. */
                    let was_active = self.ble_dl_in_flight
                        || !self.ble_dl_queue.is_empty()
                        || self.ble_sync_pending
                        || self.ble_resume_box.is_some();
                    if was_active {
                        self.arm_sync_resume();
                    }
                    self.ble_state = BleState::Idle;
                    self.ble_files.clear();
                    if !was_active {
                        self.ble_status = "disconnected".into();
                    }
                    self.ble_dl_queue.clear();
                    self.ble_dl_in_flight = false;
                    self.sync_in_flight_name = None;
                    self.sync_pass_total = 0;
                    self.sync_pass_total_bytes = 0;
                    self.sync_pass_completed_bytes = 0;
                    /* Drop live state too — the box can drop the stream
                       mid-connection (e.g. flaky link) and the next
                       reconnect should start from a clean history,
                       not graft new samples onto stale chart data. */
                    host.on_link_reset();
                    self.ble_connected_id = None;
                    self.ble_sync_pending = false;
                    self.ble_delete_err = None;
                    /* Clear any in-flight firmware progress bar (the box
                       reboots after a successful COMMIT, dropping the
                       link). Keep `ble_flash_msg` so the success/error
                       result stays visible on the post-flash screen. */
                    self.ble_flash_progress = None;
                    /* Re-query the box log mode fresh on the next
                       Connect (it could be a different box, or changed
                       from another client) rather than show a stale
                       value while disconnected. */
                    self.ble_log_mode = None;
                }
                BleEvent::ListEntry { name, size } => {
                    /* Default-tick only sensor-data rows (Sens*.csv,
                       Gps*.csv, Bat*.csv, Mic*.wav). Everything else
                       (FW_INFO, CHK, error log, macOS AppleDouble,
                       0-byte phantoms like PUMPTSUE.RI) goes in the
                       Debug group and stays unticked so a bulk
                       Download grabs only what the user wants. */
                    let selected = is_sensor_data_name(&name);
                    self.ble_files.push(BleFile {
                        name, size, selected,
                        downloaded: false,
                        bytes_done: 0,
                    });
                }
                BleEvent::ListDone => {
                    self.ble_status = format!("listing done ({} files)", self.ble_files.len());
                    /* A "Sync now" click clears the list and sends LIST,
                       then waits here: the diff needs the *complete*
                       fresh listing, not whatever was on screen before.
                       The auto-LIST on Connect leaves ble_sync_pending
                       false, so it never triggers a sync by itself. */
                    if self.ble_sync_pending {
                        self.ble_sync_pending = false;
                        self.run_sync_diff();
                    } else if self.ble_log_mode.is_none() {
                        /* Worker op is Idle right after ListDone and no
                           sync is about to queue reads — a safe window to
                           query the box log mode. (GET_MODE's reply is
                           demuxed by the worker's `op`, so it must not
                           race a LIST/READ; worker-side get_log_mode also
                           self-guards on op==Idle.) Only when still
                           unknown, so it's a one-shot per connection on
                           PumpLogger firmware and a harmless no-op on
                           legacy PumpTsueri (never replies → stays None). */
                        if let Some(b) = self.ble.as_ref() {
                            b.send(BleCmd::GetLogMode);
                        }
                    }
                }
                BleEvent::ReadStarted { name, size } => {
                    self.ble_status = format!("reading {name} ({size} B)…");
                    for f in self.ble_files.iter_mut() {
                        if f.name == name { f.bytes_done = 0; }
                    }
                }
                BleEvent::ReadProgress { name, bytes_done } => {
                    self.ble_status = format!("reading {name}: {bytes_done} B");
                    for f in self.ble_files.iter_mut() {
                        if f.name == name { f.bytes_done = bytes_done; }
                    }
                }
                BleEvent::ReadAborted { name, content, base } => {
                    /* The link dropped mid-file. Persist the partial
                       segment into <name>.part so the resume continues
                       from the real break point, not the last completed
                       segment. Not finalized, not marked synced — it's
                       still incomplete. The Error("READ … aborted/timed
                       out") that follows drives the resume/ banner. */
                    if !content.is_empty() {
                        match append_mirror(&self.ble_out_dir, &name, base, &content) {
                            Ok(have) => push_log(
                                &self.log,
                                format!("ble: kept {have} B of {name} for resume"),
                            ),
                            Err(e) => push_log(
                                &self.log,
                                format!("ble: could not save partial {name}: {e}"),
                            ),
                        }
                    }
                    // bytes_done stays where it is so the bar reflects
                    // the real resume point; do NOT advance the queue
                    // here — the link-loss handling does that.
                }
                BleEvent::ReadDone { name, content, base } => {
                    let size = self
                        .ble_files
                        .iter()
                        .find(|f| f.name == name)
                        .map(|f| f.size)
                        .unwrap_or(base + content.len() as u64);
                    let result = append_mirror(&self.ble_out_dir, &name, base, &content)
                        .map(|_| mirror_path(&self.ble_out_dir, &name));
                    match result {
                        Ok(path) => {
                            self.ble_status =
                                format!("saved {} ({} B)", path.display(), size);
                            for f in self.ble_files.iter_mut() {
                                if f.name == name {
                                    f.downloaded = true;
                                    f.bytes_done = f.size;
                                }
                            }
                            // Auto-route into the existing animate
                            // pipeline so the user can immediately hit
                            // Generate without re-dragging.
                            host.on_downloaded(&name, &path);
                            push_log(&self.log, format!("ble: saved {}", path.display()));
                            /* Record into the sync DB regardless of how
                               this read was triggered (Sync now *or* a
                               manual "Download selected"): a file that's
                               already on disk shouldn't be re-pulled by
                               a later sync. Keyed by the LIST size so it
                               matches the diff lookup exactly. */
                            if let Some(box_id) = self.ble_connected_id.clone() {
                                let path_str = path.display().to_string();
                                if self.ensure_sync_db().is_some() {
                                    if let Some(db) = self.sync_db.as_ref() {
                                        if let Err(e) =
                                            db.mark_synced(&box_id, &name, size, &path_str)
                                        {
                                            push_log(
                                                &self.log,
                                                format!("ble: sync DB write failed: {e}"),
                                            );
                                        }
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            self.ble_status = format!("save failed: {e}");
                            push_log(&self.log, format!("ble error: {e}"));
                        }
                    }
                    // Fold this file's final size into the cumulative-
                    // byte counter so the overall progress bar snaps
                    // forward as each file completes, then advance to
                    // the next queue entry (matches iOS/Android
                    // `syncPassCompletedBytes` accounting).
                    if self.sync_in_flight_name.as_deref() == Some(name.as_str())
                        && self.sync_pass_total > 0
                    {
                        self.sync_pass_completed_bytes =
                            self.sync_pass_completed_bytes.saturating_add(size);
                    }
                    self.sync_in_flight_name = None;
                    self.ble_dl_in_flight = false;
                    self.advance_download_queue();
                }
                BleEvent::LogMode { manual } => {
                    self.ble_log_mode = Some(manual);
                    self.ble_status = format!(
                        "box log mode: {}",
                        if manual { "manual" } else { "auto" }
                    );
                    push_log(
                        &self.log,
                        format!("ble: box log mode = {}", if manual { "manual" } else { "auto" }),
                    );
                    // The agent's autostart item is gated on AUTO; keep
                    // config in step so it (de)registers on the next
                    // GUI tick / agent poll.
                    self.persist_config();
                    /* Install the login item on AUTO, remove it on
                       MANUAL. Best-effort — a failed plist/registry
                       write must never break sync. Idempotent, so it's
                       fine that both the GUI and the agent run this. */
                    if let Err(e) = crate::autostart::sync_with_mode(manual) {
                        push_log(&self.log, format!("autostart: {e}"));
                    }
                }
                BleEvent::DeleteDone { name } => {
                    self.ble_status = format!("deleted {name}");
                    self.ble_files.retain(|f| f.name != name);
                    self.ble_delete_err = None;
                    push_log(&self.log, format!("ble: deleted {name}"));
                }
                BleEvent::Sample(s) => {
                    host.on_sample(&s);
                }
                BleEvent::FlashStarted { total } => {
                    self.ble_flash_progress = Some((0, total));
                    self.ble_flash_msg =
                        format!("Uploading firmware — 0 / {} kB…", (total + 1023) / 1024);
                    self.ble_status = "firmware upload started".into();
                    push_log(&self.log, format!("ble: firmware upload started ({total} B)"));
                }
                BleEvent::FlashProgress { bytes_done, total } => {
                    self.ble_flash_progress = Some((bytes_done, total));
                    let pct = if total > 0 {
                        (bytes_done as f64 / total as f64 * 100.0) as u32
                    } else {
                        0
                    };
                    self.ble_flash_msg = format!(
                        "Uploading firmware — {} / {} kB ({pct}%)…",
                        (bytes_done + 1023) / 1024,
                        (total + 1023) / 1024,
                    );
                    self.ble_status = format!("firmware: {bytes_done}/{total} B");
                }
                BleEvent::FlashDone => {
                    self.ble_flash_progress = None;
                    self.ble_flash_msg =
                        "Firmware sent — box is rebooting into the new firmware. \
                         Reconnect in a few seconds."
                            .into();
                    self.ble_status = "firmware sent — box rebooting".into();
                    push_log(&self.log, "ble: firmware upload complete — box rebooting".into());
                    /* The box swaps banks and resets within ~200 ms of
                       COMMIT, so the link is about to drop. Leave the
                       Disconnected handler to clean BLE state up; don't
                       arm a resume (this is an intentional reboot, not a
                       lost transfer). */
                }
                BleEvent::FlashError { msg } => {
                    self.ble_flash_progress = None;
                    self.ble_flash_msg = format!("Firmware upload failed: {msg}");
                    self.ble_status = format!("firmware error: {msg}");
                    push_log(&self.log, format!("ble: firmware upload failed: {msg}"));
                }
                BleEvent::Error(msg) => {
                    self.ble_status = format!("error: {msg}");
                    push_log(&self.log, format!("ble error: {msg}"));
                    if matches!(self.ble_state, BleState::Scanning | BleState::Connecting) {
                        self.ble_state = BleState::Idle;
                    }
                    /* Surface DELETE rejections as a prominent banner.
                       The box refuses some Debug rows (8.3-name miss →
                       NOT_FOUND, name >15 chars → BAD_REQUEST, logging
                       active → BUSY); without this it only shows in the
                       log and looks like the click did nothing. */
                    if msg.starts_with("DELETE ") {
                        self.ble_delete_err = Some(msg.clone());
                    }
                    /* A READ-side error (NOT_FOUND, BUSY, IO_ERROR, BAD_REQUEST,
                       timeout, disconnect mid-op) ends the in-flight read. Advance
                       the queue so the next ticked file gets its turn instead of
                       the whole batch being eaten by one bad row (typical case:
                       the phantom 0-byte `PUMPTSUE.RI` returning NOT_FOUND blocks
                       all subsequent reads). The "another op is in flight" guard
                       triggers when the worker rejects a queued Read — that's not
                       a real READ error, so don't advance on it. */
                    if msg.starts_with("READ ") {
                        self.ble_dl_in_flight = false;
                        /* Link-loss vs. per-file failure. "timed out"
                           (watchdog: no notifies for 20 s) and "aborted
                           by disconnect" mean the radio is gone — don't
                           grind the rest of the batch against a dead
                           link 20 s at a time; park it and resume on
                           reconnect. NOT_FOUND / BUSY / IO_ERROR /
                           BAD_REQUEST are per-file (e.g. the phantom
                           0-byte PUMPTSUE.RI) — skip just that row and
                           keep the batch going as before. */
                        let link_lost = msg.contains("timed out")
                            || msg.contains("aborted by disconnect");
                        if link_lost {
                            self.arm_sync_resume();
                        } else {
                            self.advance_download_queue();
                        }
                    }
                }
            }
        }
    }

    /// Pop the next file off the download queue and send it to the
    /// worker. No-op if queue empty or another read is already in flight.
    /// Called from ReadDone and READ-side error event handlers so each
    /// completed read triggers the next, giving us serial multi-file
    /// downloads instead of the original blast-all-at-once behaviour
    /// (which had the worker reject every read after the first).
    pub fn advance_download_queue(&mut self) {
        if self.ble_dl_in_flight { return; }
        loop {
            let Some((name, size)) = self.ble_dl_queue.pop_front() else {
                // Queue drained. If a sync pass was running, mark it complete
                // and clear the cumulative-progress totals so the UI line
                // disappears between passes.
                if self.sync_pass_total > 0 {
                    self.sync_pass_total = 0;
                    self.sync_pass_total_bytes = 0;
                    self.sync_pass_completed_bytes = 0;
                }
                self.sync_in_flight_name = None;
                return;
            };
            // Resume/grow from whatever is already mirrored locally. The
            // firmware seeks to this offset, so an interrupted file
            // continues and a grown log only fetches its new tail.
            let offset = mirror_offset(&self.ble_out_dir, &name, size);
            // Nothing to fetch: a 0-byte file (the FSEV~128 / SPOTL~99
            // phantoms) or a manual re-download of an already-complete
            // mirror. Issuing a READ here would hang: the box streams zero
            // bytes, no FileData notify ever arrives, and the 45 s READ
            // watchdog then tears the whole link down — one ticked 0-byte
            // row used to cost a stall + reconnect. Mark the row done
            // locally and move on to the next queue entry instead.
            if offset >= size {
                for f in self.ble_files.iter_mut() {
                    if f.name == name {
                        f.downloaded = true;
                        f.bytes_done = f.size;
                    }
                }
                if self.sync_pass_total > 0 {
                    self.sync_pass_completed_bytes =
                        self.sync_pass_completed_bytes.saturating_add(size);
                }
                push_log(
                    &self.log,
                    format!("ble: {name} already complete locally ({size} B) — skipped"),
                );
                continue;
            }
            let Some(b) = self.ble.as_ref() else { return; };
            b.send(BleCmd::Read { name: name.clone(), size, offset });
            self.sync_in_flight_name = Some(name.clone());
            self.ble_dl_in_flight = true;
            if offset > 0 {
                push_log(
                    &self.log,
                    format!("ble: resuming {name} at {offset}/{size} B"),
                );
            } else {
                push_log(&self.log, format!("ble: reading {name} ({size} B)"));
            }
            return;
        }
    }

    /// Lazily open the sync DB. Returns None (and sets `ble_sync_msg`)
    /// if it can't be opened so the caller can bail without panicking —
    /// a broken DB must not take down BLE.
    pub fn ensure_sync_db(&mut self) -> Option<&sync_db::SyncDb> {
        if self.sync_db.is_none() {
            let path = sync_db::default_db_path();
            match sync_db::SyncDb::open(&path) {
                Ok(db) => self.sync_db = Some(db),
                Err(e) => {
                    self.ble_sync_msg =
                        format!("sync disabled — can't open {}: {e}", path.display());
                    push_log(&self.log, format!("ble: sync DB open failed: {e}"));
                    return None;
                }
            }
        }
        self.sync_db.as_ref()
    }

    /// Resolve + probe `ble_out_dir`. On success canonicalises the
    /// field (writes the expanded absolute path back so the user sees
    /// where files really go) and clears the error. On failure sets
    /// `ble_save_err` + status + log and returns None so the caller
    /// aborts instead of silently enqueuing reads that save nowhere.
    pub fn validate_save_dir(&mut self) -> Option<PathBuf> {
        match resolve_save_dir(&self.ble_out_dir)
            .and_then(|d| probe_save_dir(&d).map(|()| d))
        {
            Ok(d) => {
                self.ble_save_err = None;
                self.ble_out_dir = d.clone();
                self.persist_config();
                Some(d)
            }
            Err(e) => {
                self.ble_status = format!("save path invalid: {e}");
                push_log(&self.log, format!("ble: save path invalid: {e}"));
                self.ble_save_err = Some(e);
                None
            }
        }
    }

    /// Park an interrupted transfer/sync so it resumes on reconnect.
    /// Called when the BLE link drops mid-batch (READ timeout, "aborted
    /// by disconnect", or a plain Disconnected while a batch was live).
    /// We don't try to keep the in-memory queue — on reconnect a fresh
    /// LIST + the SQLite diff is the source of truth: completed files
    /// are skipped, the half-pulled one (never marked synced, never
    /// saved) is re-fetched whole. Captures the box id *now* because
    /// the Disconnected handler clears `ble_connected_id` right after.
    pub fn arm_sync_resume(&mut self) {
        if self.ble_resume_box.is_none() {
            if let Some(id) = self.ble_connected_id.clone() {
                self.ble_resume_box = Some(id);
            }
        }
        self.ble_dl_queue.clear();
        self.ble_dl_in_flight = false;
        self.ble_sync_pending = false;
        self.sync_in_flight_name = None;
        self.sync_pass_total = 0;
        self.sync_pass_total_bytes = 0;
        self.sync_pass_completed_bytes = 0;
        self.ble_sync_msg = "⚠ Transfer interrupted (BLE link lost). Reconnect to \
             the box — the sync resumes automatically and skips files already saved."
            .into();
        push_log(
            &self.log,
            "ble: transfer interrupted — armed to resume on reconnect".into(),
        );
    }

    /// Run the diff after a Sync-triggered LIST: enqueue every sensor-
    /// data file on the box not already in the DB for this box. Never
    /// deletes anything (additive-only, by design). Debug/firmware-info
    /// rows are left out — same policy as the default-tick filter, so a
    /// sync mirrors *sessions*, not noise.
    /// Begin one sync pass: fresh LIST, then the diff runs in the
    /// `ListDone` handler (gated on `ble_sync_pending`, so the auto-
    /// LIST on Connect never starts a sync by itself). Used by the
    /// "Sync now" button and the "Keep synced" continuous loop.
    pub fn start_sync_pass(&mut self) {
        if !matches!(self.ble_state, BleState::Connected) { return; }
        self.ble_files.clear();
        self.ble_sync_pending = true;
        self.ble_last_sync_at = Some(std::time::Instant::now());
        self.ble_sync_msg = "Sync: listing SD card…".into();
        if let Some(b) = self.ble.as_ref() { b.send(BleCmd::List); }
        push_log(&self.log, "ble: sync — listing".into());
    }

    /// Read a firmware `.bin` from `path` and start uploading it to the
    /// connected box over BLE (dual-bank OTA). No-op (with a status
    /// message) if not connected, another op is in flight, or the file
    /// can't be read. The worker emits `FlashStarted/Progress/Done/Error`
    /// which `pump_ble_events` folds into `ble_flash_progress` /
    /// `ble_flash_msg`.
    pub fn start_firmware_upload(&mut self, path: &Path) {
        if !matches!(self.ble_state, BleState::Connected) {
            self.ble_flash_msg = "Connect to a box before uploading firmware.".into();
            return;
        }
        if self.ble_dl_in_flight
            || self.ble_sync_pending
            || !self.ble_dl_queue.is_empty()
            || self.ble_flash_progress.is_some()
        {
            self.ble_flash_msg = "Another BLE op is in flight — wait for it to finish.".into();
            return;
        }
        let bytes = match std::fs::read(path) {
            Ok(b) => b,
            Err(e) => {
                self.ble_flash_msg = format!("Can't read firmware file: {e}");
                push_log(&self.log, format!("ble: firmware read failed: {e}"));
                return;
            }
        };
        if bytes.is_empty() {
            self.ble_flash_msg = "Firmware file is empty.".into();
            return;
        }
        let len = bytes.len();
        self.ble_flash_progress = Some((0, len as u64));
        self.ble_flash_msg = format!(
            "Uploading firmware ({}) — {} kB…",
            path.file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_else(|| path.display().to_string()),
            (len as f64 / 1024.0).round() as u64,
        );
        push_log(
            &self.log,
            format!("ble: uploading firmware {} ({len} B)", path.display()),
        );
        if let Some(b) = self.ble.as_ref() {
            b.send(BleCmd::FlashFirmware { bytes: Arc::new(bytes) });
        }
    }

    pub fn run_sync_diff(&mut self) {
        if self.validate_save_dir().is_none() {
            // Bad Save-to path — surface it as the sync result too so
            // the user isn't left wondering why nothing downloaded.
            self.ble_sync_msg =
                format!("Sync aborted — {}", self.ble_save_err.clone().unwrap_or_default());
            return;
        }
        if self.ble_connected_id.is_none() {
            self.ble_sync_msg = "sync: no box id (reconnect and retry)".into();
            return;
        }
        // Decide by local mirror size vs the box's current size, not a
        // "have I ever seen this exact size" DB lookup. That's what
        // makes a continuously-growing log work: each pass fetches only
        // the new tail (offset = local size) instead of re-pulling the
        // whole file, so no single big file can starve GPS/BAT either.
        let mut fetch = 0usize;
        let mut fetch_bytes: u64 = 0;
        let mut up_to_date = 0usize;
        self.ble_dl_queue.clear();
        for f in self.ble_files.iter() {
            if !is_synced_name(&f.name) {
                continue;
            }
            let local = std::fs::metadata(mirror_path(&self.ble_out_dir, &f.name))
                .map(|m| m.len())
                .unwrap_or(0);
            if local == f.size {
                up_to_date += 1;
            } else {
                // local < size → grow/resume; local > size → rotated
                // (mirror_offset resets it). Either way, fetch.
                fetch += 1;
                // Box sizes both sides of the bar: `bytes_done` from the
                // worker is *absolute* (resume base + streamed), so the
                // bar's "already-on-disk bytes count once the file
                // becomes in-flight" — same semantic as iOS/Android.
                fetch_bytes = fetch_bytes.saturating_add(f.size);
                self.ble_dl_queue.push_back((f.name.clone(), f.size));
            }
        }
        // Seed the cumulative progress card (iOS/Android parity). Both
        // totals zero if there's nothing to fetch — UI hides the line.
        self.sync_pass_total = fetch;
        self.sync_pass_total_bytes = fetch_bytes;
        self.sync_pass_completed_bytes = 0;
        self.ble_sync_msg = if fetch == 0 {
            format!("Sync: up to date ({up_to_date} files)")
        } else {
            format!("Sync: fetching {fetch} ({up_to_date} up to date)…")
        };
        push_log(&self.log, format!("ble: {}", self.ble_sync_msg));
        self.advance_download_queue();
    }

}
