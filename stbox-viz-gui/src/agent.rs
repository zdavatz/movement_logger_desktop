//! Headless background sync agent (`MovementLogger --agent`).
//!
//! Issue #14 part B step 6. Zero-click: launched at login (step 7),
//! it mirrors the configured box whenever the box is in AUTO mode and
//! "Keep synced" is on — without an open window. It drives the exact
//! same `SyncCore` engine the GUI does (a no-op `SyncHost`), so the
//! resume/reconnect/byte-mirror guarantees are unchanged.
//!
//! Coordination (issue #14 decided architecture: GUI wins, agent
//! yields): the agent holds `agent.lock` for single-instance, and
//! before/while connected it watches `gui.lock`. When a GUI appears
//! it runs the same abort+Disconnect teardown as `on_exit`, releases
//! `ble.lock`, and idles until the GUI is gone — then resumes (the
//! local mirror makes the resume lossless).
//!
//! No eframe/egui/winit is touched on this path — `main()` returns
//! before any window code, keeping the macOS winit-patch invariant.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::agent_config::AgentConfig;
use crate::ble::{BleBackend, BleCmd};
use crate::coord;
use crate::sync_core::{default_save_base, BleState, SyncCore, SyncHost};

const TICK: Duration = Duration::from_millis(250);
const SYNC_POLL_INTERVAL: Duration = Duration::from_secs(30);
/// Re-issue Scan this often while connected to nothing.
const RESCAN_INTERVAL: Duration = Duration::from_secs(15);
/// Teardown grace — same 250 ms the GUI's on_exit uses, rounded up a
/// touch since the agent has no UI deadline to hit.
const TEARDOWN_GRACE: Duration = Duration::from_millis(400);

/// The agent never renders Live charts or the Replay form, so every
/// `SyncHost` hook is a no-op.
struct NoopHost;
impl SyncHost for NoopHost {
    fn on_link_reset(&mut self) {}
    fn on_sample(&mut self, _s: &crate::ble::LiveSample) {}
    fn on_battery(&mut self, _b: &crate::ble::BatterySample) {}
    fn on_downloaded(&mut self, _name: &str, _path: &std::path::Path) {}
    fn on_calibration(&mut self, _blob: Option<&[u8; 32]>) {}
}

/// Drain new lines from the shared log buffer to stdout (launchd /
/// journald capture it). `seen` tracks how many we've already printed.
fn drain_log(log: &Arc<Mutex<Vec<String>>>, seen: &mut usize) {
    if let Ok(v) = log.lock() {
        for line in v.iter().skip(*seen) {
            println!("[agent] {line}");
        }
        *seen = v.len();
    }
}

/// Pick the device to connect to: the configured box id if we've seen
/// it advertise, else (no id pinned yet) the first one. Scan results
/// are already filtered to box names by the worker, so any entry is a
/// valid SensorTile.box.
fn choose<'a>(
    devices: &'a [crate::sync_core::BleDevice],
    box_id: &Option<String>,
) -> Option<&'a crate::sync_core::BleDevice> {
    match box_id {
        Some(id) => devices.iter().find(|d| &d.id == id),
        None => devices.first(),
    }
}

/// Entry point from `main()` when `--agent` is passed. Returns a
/// process exit code.
pub fn run() -> i32 {
    // Single instance. A second agent (e.g. launchd relaunch racing an
    // old one) just exits — it must never touch BLE.
    let _single = match coord::try_acquire(coord::AGENT_LOCK) {
        Some(g) => g,
        None => {
            eprintln!("[agent] another agent instance is already running — exiting");
            return 0;
        }
    };

    // Record our PID so the in-app updater can stop us before a
    // bundle swap (the post-update GUI restarts a fresh agent).
    coord::write_agent_pid();

    let stop = Arc::new(AtomicBool::new(false));
    {
        let s = stop.clone();
        // SIGINT/SIGTERM (launchd/systemd stop) + Windows console close.
        let _ = ctrlc::set_handler(move || s.store(true, Ordering::SeqCst));
    }

    let log: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let mut seen = 0usize;
    eprintln!(
        "[agent] MovementLogger v{} background sync agent started",
        env!("CARGO_PKG_VERSION")
    );

    while !stop.load(Ordering::SeqCst) {
        let cfg = AgentConfig::load();
        // Only mirror when the user wants continuous sync and the box
        // isn't explicitly MANUAL. Unknown (None) is allowed — the
        // autostart item is only installed for AUTO anyway, and a
        // first run before GET_MODE answered shouldn't sit idle.
        let active = cfg.keep_synced && cfg.log_mode_manual != Some(true);
        if !active {
            sleep_or_stop(&stop, Duration::from_secs(3));
            continue;
        }
        // GUI present → it owns the adapter. Wait.
        if coord::is_held(coord::GUI_LOCK) {
            sleep_or_stop(&stop, Duration::from_secs(1));
            continue;
        }
        // Take the adapter token. If the GUI grabbed it first, back off.
        let ble_lock = match coord::try_acquire(coord::BLE_LOCK) {
            Some(g) => g,
            None => {
                sleep_or_stop(&stop, Duration::from_secs(1));
                continue;
            }
        };
        run_session(&cfg, &log, &stop, &mut seen);
        drop(ble_lock); // release adapter before the next outer iteration
    }

    drain_log(&log, &mut seen);
    coord::clear_agent_pid();
    eprintln!("[agent] stopped");
    0
}

/// One connected mirror session. Runs until stop, a GUI appears, or
/// the config goes inactive; always tears the BLE link down cleanly
/// on the way out (same path as the GUI's on_exit).
fn run_session(
    cfg: &AgentConfig,
    log: &Arc<Mutex<Vec<String>>>,
    stop: &Arc<AtomicBool>,
    seen: &mut usize,
) {
    let mut sc = SyncCore::default();
    sc.log = log.clone();
    sc.ble_keep_synced = true;
    sc.ble_out_dir = if cfg.save_dir.trim().is_empty() {
        default_save_base().join("csv")
    } else {
        std::path::PathBuf::from(&cfg.save_dir)
    };
    sc.ble_connected_id = cfg.box_id.clone();

    let b = BleBackend::spawn();
    // Agent regime: unbounded auto-reconnect (the whole point — leave
    // it running and a multi-hour sync finishes across drops).
    b.set_keep_synced(true);
    b.send(BleCmd::Scan { known_id: sc.ble_connected_id.clone() });
    sc.ble = Some(b);
    sc.ble_state = BleState::Scanning;

    let mut host = NoopHost;
    let mut last_scan = Instant::now();
    // Due immediately once connected.
    let mut last_sync = Instant::now()
        .checked_sub(SYNC_POLL_INTERVAL)
        .unwrap_or_else(Instant::now);

    loop {
        if stop.load(Ordering::SeqCst) || coord::is_held(coord::GUI_LOCK) {
            break;
        }
        // Honour a live config change (Keep synced off / box → MANUAL)
        // without waiting for the next outer pass.
        let now_cfg = AgentConfig::load();
        if !now_cfg.keep_synced || now_cfg.log_mode_manual == Some(true) {
            break;
        }

        sc.pump_ble_events(&mut host);

        match sc.ble_state {
            BleState::Idle | BleState::Scanning if sc.ble.is_some() => {
                if let Some(d) = choose(&sc.ble_devices, &cfg.box_id) {
                    let id = d.id.clone();
                    sc.ble_connected_id = Some(id.clone());
                    sc.persist_config();
                    if let Some(b) = sc.ble.as_ref() {
                        b.send(BleCmd::Connect(id));
                    }
                    sc.ble_state = BleState::Connecting;
                } else if last_scan.elapsed() >= RESCAN_INTERVAL {
                    if let Some(b) = sc.ble.as_ref() {
                        b.send(BleCmd::Scan { known_id: sc.ble_connected_id.clone() });
                    }
                    sc.ble_state = BleState::Scanning;
                    last_scan = Instant::now();
                }
            }
            BleState::Connected => {
                if !sc.ble_sync_pending
                    && !sc.ble_dl_in_flight
                    && last_sync.elapsed() >= SYNC_POLL_INTERVAL
                {
                    sc.start_sync_pass();
                    last_sync = Instant::now();
                }
            }
            _ => {}
        }

        drain_log(log, seen);
        std::thread::sleep(TICK);
    }

    // Clean teardown — identical intent to eframe::App::on_exit:
    // abort any (unbounded) reconnect loop first, then Disconnect,
    // then give the worker a moment to emit it over the air before
    // the BleBackend is dropped.
    if let Some(b) = sc.ble.as_ref() {
        b.request_abort();
        b.send(BleCmd::Disconnect);
        std::thread::sleep(TEARDOWN_GRACE);
    }
    drain_log(log, seen);
}

/// Headless one-shot firmware upload (`MovementLogger --flash-firmware
/// <path.bin>`). Scans for the configured box (or the first box seen),
/// connects, streams the image over BLE, and prints progress to stdout.
/// Returns a process exit code (0 = sent + box rebooting, non-zero =
/// error). Self-contained — drives the same `SyncCore` / `BleBackend`
/// the GUI does (with a no-op `SyncHost`), so the wire protocol and
/// single-op semantics are identical.
pub fn flash(path: &std::path::Path) -> i32 {
    if !path.exists() {
        eprintln!("[flash] firmware file not found: {}", path.display());
        return 2;
    }

    let cfg = AgentConfig::load();
    let log: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let mut seen = 0usize;

    let mut sc = SyncCore::default();
    sc.log = log.clone();
    sc.ble_connected_id = cfg.box_id.clone();

    let b = BleBackend::spawn();
    // Bounded reconnect is fine for a one-shot — a flash is single-op and
    // short; if the link is too flaky to start we just fail out.
    b.send(BleCmd::Scan { known_id: sc.ble_connected_id.clone() });
    sc.ble = Some(b);
    sc.ble_state = BleState::Scanning;

    let mut host = NoopHost;
    let mut last_scan = Instant::now();
    let started = Instant::now();
    // Wall-clock ceiling for the whole pass (scan → connect → upload).
    // Measured throughput on a slow link is ~90 B/s (≈1.7 s per 160 B
    // chunk while each FW_DATA waits on its 4-byte ack), so a ~100 kB
    // image needs ~20 min. 600 s timed out a real flash mid-stream;
    // 2400 s leaves headroom for FW_DATA retries on a flaky link.
    let deadline = Duration::from_secs(2400);
    let mut sent = false;
    let mut last_pct = u64::MAX;
    let mut waiting_logged = false;

    eprintln!("[flash] uploading {} — scanning for box…", path.display());

    loop {
        if started.elapsed() > deadline {
            eprintln!("[flash] timed out");
            teardown(&sc);
            drain_log(&log, &mut seen);
            return 1;
        }
        sc.pump_ble_events(&mut host);
        drain_log(&log, &mut seen);

        match sc.ble_state {
            BleState::Idle | BleState::Scanning if sc.ble.is_some() => {
                if let Some(d) = choose(&sc.ble_devices, &cfg.box_id) {
                    let id = d.id.clone();
                    sc.ble_connected_id = Some(id.clone());
                    if let Some(b) = sc.ble.as_ref() {
                        b.send(BleCmd::Connect(id));
                    }
                    sc.ble_state = BleState::Connecting;
                    eprintln!("[flash] connecting…");
                } else if last_scan.elapsed() >= RESCAN_INTERVAL {
                    if let Some(b) = sc.ble.as_ref() {
                        b.send(BleCmd::Scan { known_id: sc.ble_connected_id.clone() });
                    }
                    sc.ble_state = BleState::Scanning;
                    last_scan = Instant::now();
                }
            }
            BleState::Connected => {
                if !sent {
                    sc.start_firmware_upload(path);
                    sent = true;
                    if sc.ble_flash_progress.is_none() {
                        // start_firmware_upload rejected it (printed via
                        // ble_flash_msg) — bail.
                        eprintln!("[flash] {}", sc.ble_flash_msg);
                        teardown(&sc);
                        drain_log(&log, &mut seen);
                        return 1;
                    }
                }
            }
            _ => {}
        }

        // Progress / terminal-state reporting off the SyncCore fields the
        // event pump maintains.
        if let Some((done, total)) = sc.ble_flash_progress {
            let pct = if total > 0 { done * 100 / total } else { 0 };
            if pct != last_pct {
                last_pct = pct;
                eprintln!("[flash] {pct}% ({done}/{total} B)");
            }
        } else if sent {
            // Progress cleared after we sent → terminal (FlashDone /
            // FlashError) *or* a transient bounce. The shared Connected
            // handler queues SET_TIME + LIST (and, after ListDone, a
            // GET_MODE on a fresh connection) ahead of us; those hold the
            // worker's single-op slot, so the first FlashFirmware can be
            // rejected with "another op is in flight". That's transient —
            // re-arm and retry until the connect-time ops drain (on a
            // legacy box that never answers GET_MODE the slot self-frees
            // after the worker's ~20 s op-idle watchdog). The 600 s
            // deadline above bounds the retry loop.
            if sc.ble_flash_msg.contains("another op is in flight") {
                if !waiting_logged {
                    eprintln!(
                        "[flash] box busy with connect-time ops — waiting for the slot…"
                    );
                    waiting_logged = true;
                }
                sent = false; // let start_firmware_upload re-arm next pass
                last_pct = u64::MAX;
                std::thread::sleep(Duration::from_millis(500));
                continue;
            }
            // Upload finished. The message distinguishes success from failure.
            let ok = !(sc.ble_flash_msg.contains("failed")
                || sc.ble_flash_msg.contains("Can't")
                || sc.ble_flash_msg.contains("empty"));
            eprintln!("[flash] {}", sc.ble_flash_msg);
            teardown(&sc);
            drain_log(&log, &mut seen);
            return if ok { 0 } else { 1 };
        }

        std::thread::sleep(TICK);
    }
}

/// Clean BLE teardown shared by the flash one-shot.
fn teardown(sc: &SyncCore) {
    if let Some(b) = sc.ble.as_ref() {
        b.request_abort();
        b.send(BleCmd::Disconnect);
        std::thread::sleep(TEARDOWN_GRACE);
    }
}

/// Sleep in short slices so a stop signal is honoured promptly.
fn sleep_or_stop(stop: &Arc<AtomicBool>, total: Duration) {
    let mut left = total;
    while left > Duration::ZERO {
        if stop.load(Ordering::SeqCst) {
            return;
        }
        let slice = left.min(Duration::from_millis(250));
        std::thread::sleep(slice);
        left = left.saturating_sub(slice);
    }
}
