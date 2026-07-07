# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Cross-platform desktop build of **MovementLogger** — a drag-and-drop GUI for SensorTile.box pumpfoil session telemetry. Sources were imported from `fp-sns-stbox1/Utilities/rust` so this repo can cut its own release cadence and live separately from the firmware monorepo.

## Workspace layout

Cargo workspace at the repo root with three members and two excluded vendored forks:

- `stbox-viz/` — CLI session visualiser (plotters, rustfft, gif). The GUI shells out to its `animate` subcommand to render side-by-side MOVs.
- `stbox-viz-gui/` — the GUI itself (binary name `MovementLogger`, package name `movement-logger`). Built on `eframe`/`egui`, BLE FileSync via `btleplug` on a tokio worker thread, GitHub-Releases-based in-app updater.
- `usb-console/` — minimal `rusb` debug console for the SensorTile.box PRO USB CDC firmware. Independent; not shipped in the GUI release archive.
- `winit-patched/` — vendored fork of `winit` 0.30.13 with the private `_CGSSetWindowBackgroundBlurRadius` call removed from `Window::set_blur`. Required for Mac App Store binary scanning. Applied workspace-wide via `[patch.crates-io] winit = { path = "winit-patched" }` and **excluded** from the workspace members list (it ships its own workspace, must not nest).
- `btleplug-patched/` — vendored fork of `btleplug` 0.11.8 adding a macOS CoreBluetooth **retrieve-by-identifier + pending-connect** path (`Central::add_peripheral` → a new `RetrievePeripheral` message → `retrievePeripheralsWithIdentifiers:` → `on_discovered_peripheral`). Stock btleplug reconnects *only* by scan re-discovery, but macOS CoreBluetooth stops returning a recently-connected peripheral in scan results — so the desktop (unlike iOS/Android) got stuck and needed a box power-cycle. The fork lets `ble.rs::auto_reconnect` reconnect by id without a scan, like iOS `retrievePeripherals(withIdentifiers:)`. Applied via `[patch.crates-io] btleplug = { path = "btleplug-patched" }` and **excluded** from members. **Version must stay pinned to whatever the graph resolves (0.11.8)** — if a dep bump moves btleplug to a new minor, the `[patch]` silently no-ops and reconnect regresses to scan-only with no compile error. See *BLE auto-reconnect* below.

## Build / run

```sh
cargo build --release -p movement-logger -p stbox-viz
./target/release/MovementLogger
```

Linux build deps: `libxkbcommon-dev libfontconfig1-dev libgl1-mesa-dev libdbus-1-dev`.

On macOS, after every release build verify the patch held:

```sh
nm target/release/MovementLogger | grep CGSSetWindowBackgroundBlur   # must be empty
```

If the symbol returns, the `[patch.crates-io]` substitution didn't apply — usually because `eframe` was bumped to a version that pulls in a different `winit` major than the fork covers.

Also verify the **`btleplug-patched`** substitution is live (no `nm`-style check applies — check the lockfile + tree instead):

```sh
cargo tree -p btleplug | head -1        # must show the local path fork, not a registry version
grep -A3 'name = "btleplug"' Cargo.lock # must have NO `source`/`checksum` lines
```

If those show a registry source, the patch no-oped (usually a dep bump moved btleplug off 0.11.8) and macOS reconnect silently regresses to scan-only — the "must power-cycle the box" bug returns. See *BLE auto-reconnect* below.

## Runtime topology

The GUI is intentionally thin: it composes a `stbox-viz animate …` command line, spawns it as a child, streams stdout/stderr into the live log panel, and lets the user cancel. `stbox-viz` lookup order: next to `current_exe()`, then `PATH`. The release archives ship both binaries side by side, and the macOS `.app` bundles `stbox-viz` inside `Contents/MacOS/`. `ffmpeg` must be on `PATH` for `--video` mode.

The egui UI is organised into three top-level tabs (`enum Tab` in `main.rs`): **Live** (SensorStream readouts), **Sync** (BLE FileSync), **Replay** (drag-and-drop CSV → animated video). Default tab is Live so the box-connected workflow lands there first; the previous single-panel layout collapsed into Replay. The egui top-bar in `update()` switches between them; the update-banner is rendered once above the active tab. The product name + version live **only** in the OS title bar (set in `main()`); the in-app top panel deliberately does **not** repeat them — it carries just the subtitle hyperlink and the top-right logo→mailto. Upstream `fp-sns-stbox1` renders a `MovementLogger v{CARGO_PKG_VERSION}` `ui.heading` there; drop it again on the next pull (removed here in v0.0.16 — it duplicated the title bar).

The Live tab renders a **Board angles** card (`ui_live_tab` → `board_angles` / `BoardAngles`, v0.0.56) — **Pitch** (up / down hill), **Roll** (lean L / R) and **Yaw** (heading) about the box's PHYSICAL axes (nose = Y, up = Z), taken from the gyro+accel orientation filter (`OriRows` / `triad_world`), **not** the raw accel eCompass formulas. Each is a single decoupled physical quantity, so the signs stay individually predictable at the modest angles a foil sees (no gimbal / axis-order pitfalls a matrix→Euler decomposition would reintroduce). **This fixes a swapped-axis bug:** the old accel-only pitch/roll assumed a phone-style frame (long axis = X), so on this box (nose = Y) they SWAPPED pitch and roll — the old "Orient (°)" row was removed. Two sets are shown: **Absolute** (yaw = compass heading, `heading_bias_deg` applied) and **Calibrated** — a *Zero here* button tares to the current pose (`angle_zero_ref` = `[pitch, roll, yaw]`, bias 0), *Clear* resets, and a "zeroed N ago" line reads from `angle_zero_at`. Both persist in `~/.movementlogger/config.toml` (`agent_config::AgentConfig`). Mirrors iOS `BoardAnglesCard` / `BoardAngles.from`.

BLE (`stbox-viz-gui/src/ble.rs`) runs its own tokio current-thread runtime on a worker thread; commands/events shuttle across `std::sync::mpsc` so the egui UI stays sync. Single notification stream demuxed by a `select!` loop on the worker side; the stream now carries both FileSync (FileData UUID `…40…`) and SensorStream (UUID `…100…`) notifies, branched on UUID. SensorStream is subscribed on Connect when the characteristic is present (PumpLogger firmware) and silently skipped on legacy SDDataLogFileX (`PumpTsueri`) builds. Wire spec for the 46-byte packed snapshot lives in upstream `DESIGN.md` §3; `LiveSample::parse` mirrors it.

Two transport modes:

- **Single-notify** (46 B exact) — used when the host accepts the box's MTU upgrade.
- **Chunked fallback** — three sequential notifies prefixed with sequence byte `0x00` / `0x01` / `0x02`; reassembled into 46 bytes in `StreamAsm` and parsed identically. Out-of-order chunks reset the buffer and wait for the next `0x00` start. Sample loss at 0.5 Hz is a 2 s gap — visible in the freshness label but never produces partial decodes.

Accepted BLE advertise names: `PumpTsueri` (legacy SDDataLogFileX) and `STBoxFs` (PumpLogger). Single source of truth is `BOX_NAMES` in `ble.rs`.

## Sync vs. transfer — `sync_db.rs`

The box firmware only speaks LIST / READ / DELETE / STOP_LOG / START_LOG / SET_MODE / GET_MODE / SET_TIME — there is **no sync concept on the device**. "File transfer" (tick files → *Download selected* → serial READ queue → `save_downloaded_file`) is the old path and unchanged. "File **Sync**" (*Sync now* button) is a thin client-side layer on top:

- State lives in a local SQLite DB at `~/.movementlogger/sqlite/sync.db` (Windows `%USERPROFILE%\.movementlogger\sqlite\sync.db`) — `sync_db.rs::default_db_path`, cross-platform via `HOME`/`USERPROFILE`. `rusqlite` with the `bundled` feature so there's no system libsqlite (same self-contained rationale as `rustls-tls`). The `sqlite/` subdir is created by `SyncDb::open` via `create_dir_all`. Anchored to `$HOME`, **not** to the user-selectable "Save to" folder, so changing that folder doesn't orphan history and re-pull everything. (Changed in v0.0.7 from the flat `~/.movementlogger/sync.db`; no migration shim — v0.0.6 was brand-new so a stale flat DB just means one extra re-pull.)
- The **default download/render folders** (`ble_out_dir` = `csv/`, `output_dir` = `gif/`) come from `default_save_base()` (v0.0.22), **not** a bare `current_dir()`. A macOS `.app` launched from Finder/Dock starts with cwd `/` (read-only volume), so the old `cwd.join("csv")` resolved to `/csv` and *every* download failed with `Can't create "/csv": Read-only file system (os error 30)` (Peter, v0.0.20). `default_save_base()` keeps cwd only when it's non-root and passes `probe_save_dir` (the unchanged Linux/Windows `./MovementLogger` case); otherwise it anchors under `$HOME/MovementLogger`. The user can still override the folder in the Sync tab. Don't reintroduce a raw `current_dir()` default.
- The DB is now an **audit log only** (v0.0.14): it records `(box_id, name, size, downloaded_at, local_path)` on each completed pull but is **not consulted** to decide what to fetch. `box_id` is the btleplug peripheral id, captured at Connect-click time into `AppState::ble_connected_id`.
- *Sync now* (`start_sync_pass`) clears the list, sets `ble_sync_pending`, sends a fresh LIST; the diff runs in the `ListDone` handler **only when that flag is set** (so the auto-LIST on Connect never triggers a sync). `run_sync_diff` decides per file by **local mirror size vs the box's LIST size**: `local < box` → fetch the tail from offset `local`; `local == box` → up to date; `local > box` → file rotated, reset and refetch. This is what makes a continuously-growing log fetch only its new bytes instead of re-downloading, and stops a big file starving GPS/BAT in the serial queue.
- **Keep synced** (`ble_keep_synced` checkbox): while connected and idle, `update()` calls `start_sync_pass` every `SYNC_POLL_INTERVAL` (30 s). A busy backlog runs back-to-back because the interval is measured from the previous *trigger* (`ble_last_sync_at`), long elapsed by the time a big pass drains. The box logs grow forever, so a sync that "completes" is never really final — this keeps the local files mirroring. **The tick also guards on `ble_resume_box.is_none()`** (iOS `8c90276` parity) so it doesn't fire during the silent `auto_reconnect` window — `ble_state` stays `Connected` for the whole reconnect window (the worker emits no `Disconnected` until the bounded budget is spent in Manual mode, never in Auto), and without the guard the tick would queue a fresh LIST behind the in-flight reconnect and clobber the "Reconnected — resuming sync" status the `Connected` handler already shows. The `Connected` handler re-runs `start_sync_pass` itself on link return, so nothing is lost by skipping ticks in between. The "Refresh file list" and "Sync now" buttons share a `worker_busy = ble_dl_in_flight || ble_sync_pending || !ble_dl_queue.is_empty()` gate so tap-while-busy can't pile a second LIST behind the in-flight op.
- **Cumulative byte progress.** While a sync pass runs the Sync tab shows an `egui::ProgressBar` above the file list with "N of M files · X / Y MB · Z %" (iOS/Android parity, 2d001a4). Backed by `SyncCore::{sync_pass_total, sync_pass_total_bytes, sync_pass_completed_bytes, sync_in_flight_name}` + the `sync_cumulative_bytes()` / `sync_cumulative_fraction()` helpers. Totals are seeded in `run_sync_diff` (sums box file sizes — `bytes_done` from the worker is *absolute*, resume base + streamed, so the bar's "already-on-disk bytes count once the file becomes in-flight" matches the mobile semantic). Folded forward in `ReadDone` (`completed_bytes += size`), cleared on `arm_sync_resume`, `Disconnected`, and queue-drain in `advance_download_queue`. The per-row bars below the cumulative one still show individual file progress; the cumulative one is the roll-up so the user sees the whole pass at a glance.
- **What Sync mirrors** is gated by `is_synced_name` (v0.0.20), **not** `is_sensor_data_name`. It's the per-session sensor data (`Sens*.csv` / `Gps*.csv` / `Bat*.csv` / `Mic*.wav`) **plus** the firmware's single rolling error log `ERRLOG.LOG` (`movement_logger_firmware/Src/errlog.c`). Before v0.0.20 the diff used `is_sensor_data_name` directly, so `ERRLOG.LOG` was never pulled — Peter's "ein Teil synchronisiert, Errlog fehlt aber". `is_synced_name` is deliberately a *superset* used only in `run_sync_diff`: `is_sensor_data_name` still drives the manual list's default-tick and the Sensor/Debug split, so `ERRLOG.LOG` stays in the Debug group, un-ticked, and is pulled *only* by Sync. The errlog is append-only so the live-mirror byte-resume path needs no changes. Add new box-side debug files to `is_synced_name` (not `is_sensor_data_name`) so Sync picks them up without disturbing the manual UI.
- **Sync never issues DELETE** — additive only, by explicit product decision. Don't add a delete-after-sync path without re-confirming.
- **Host time-sync (`SET_TIME` `0x08`, firmware v0.0.10+)** — the box has no RTC, so on every connect `sync_core.rs`'s shared `BleEvent::Connected` handler (covers GUI, agent, and auto-reconnect) sends `BleCmd::SetTime { epoch_ms }` with `SystemTime::now()` sampled right before the send. `ble.rs::set_time` writes `0x08 + epoch_ms(u64-LE)` and is *fire-and-forget* (no `CurrentOp` slot, like `stop_log`, so legacy firmware that ignores 0x08 can't stall the worker); it sleeps 500 ms after the write so the auto-LIST that the connect handler queues right behind it can't clobber it in the firmware's single command buffer. The box stamps a `# SYNC epoch_ms=.. tick_ms=..` marker into the open Sens/Gps CSVs. **Consuming the anchors:** `io.rs::read_sync_anchors` parses those marker lines into `Vec<SyncAnchor>` (the CSV row loaders skip `#` comment lines via `.comment(Some(b'#'))`), and `io.rs::abs_times_from_sync_anchors` maps row ticks → absolute epoch ms (piecewise-linear; constant 10 ms/tick outside the anchor span). `animate_cmd.rs::resolve_at_window` **prefers** these host anchors (drift-free, no GPS fix needed, same clock domain as the video's `creation_time`) over the GPS-UTC nearest-row anchoring, which stays as the fallback for legacy / never-connected sessions. The `io.rs` loaders were also taught the post-22.4.2026 **compact schema** (`ms,ax_mg,…`, tick ÷10) here, since the markers only ever appear in compact-schema files. Mirrors iOS `CsvParsers.parseSyncAnchors` / `ReplayViewModel.absTimesFromSyncAnchors`.
- **Box log mode (Auto/Manual)** — separate from Sync; it's the *box's* SET_MODE (`0x06` `<u8>` 0=auto/1=manual, persisted to `LOGMODE.CFG`) / GET_MODE (`0x07`, 1-byte reply) state, mirroring iOS/Android (`movement_logger_android` `LogModeSelector`/`OP_SET_MODE`). AUTO = box opens a session on every power-on; MANUAL = box idle until START_LOG. Desktop part A (Auto/Manual UI parity) added the `BleCmd::SetLogMode`/`GetLogMode`, `BleEvent::LogMode`, and `CurrentOp::GettingMode` (waits for the 1-byte reply, demuxed by `op` over FileData like DELETE — **must not** race a LIST/READ). The box is the source of truth: a UI click sends SET_MODE and the worker re-reads via GET_MODE, so the Sync-tab highlight only flips once the box confirms. GET_MODE is queried once per connection from the `ListDone` handler **only when `!ble_sync_pending`** (worker op is Idle then and no reads are about to queue); legacy `PumpTsueri` never replies so `ble_log_mode` stays `None` ("unknown", no error). A stalled `GettingMode` drops to Idle and **never reconnects** (same rationale as Deleting — nothing wrong with the link). This is part A of issue #14.
- **Box GPS power (On/Off, firmware v0.0.35+, v0.0.56)** — a battery-save twin of the log-mode control, sitting next to it in the Sync panel: turns the box's u-blox receiver off (→ UBX-RXM-PMREQ backup, ~tens of µA vs ~25 mA) when GPS is faulty/unused. GPS off ⇒ IMU + baro keep logging and animate/replay still time-aligns via the `# SYNC` anchor (you lose speed + track, keep pitch/roll/height). Two opcodes mirroring SET_MODE/GET_MODE: **`GPS_POWER 0x11`** (`[0x11, on as u8]`, one status-byte reply — 0x00 OK) and **`GPS_GET_POWER 0x12`** (→ 1 byte, 1=on/0=off). `BleCmd::SetGpsPower`/`GetGpsPower`, `BleEvent::GpsPower`, `CurrentOp::GpsPowerReq`, the senders + the reply demux live in `ble.rs` (demuxed by `op` over FileData like DELETE/GET_MODE — **must not** race a LIST/READ). `sync_core.rs` holds `ble_gps_power: Option<bool>` and queries it once per connect: the connect flow is **GET_VERSION → GET_MODE → GET_GPS_POWER**, each chained on an idle worker (its demuxed reply can't overlap the next), so `GetGpsPower` fires from the `LogMode` handler (and as a fallback from `ListDone`) only when `ble_gps_power.is_none() && !ble_sync_pending`. The box persists the state and re-applies it at boot; the app only reflects + sends — a click reflects the target optimistically and the box's `GpsPower` OK reply reconciles it. Legacy firmware < v0.0.35 never answers → `ble_gps_power` stays `None` ("unknown", neither On nor Off lit, no error). Mirrors iOS/Android.

## Background sync agent — `--agent` (issue #14 part B)

Zero-click: the same `MovementLogger` binary invoked as `MovementLogger --agent` runs **headless** (no eframe/egui/winit — `main()` `process::exit`s on the `--agent` arg *before* any window code, so the macOS winit-patch invariant is untouched) and mirrors the configured box at login without an open window. It is **not** a second binary.

- **Shared engine.** The sync engine was lifted out of `AppState` into `sync_core::SyncCore` (state + `pump_ble_events`/`run_sync_diff`/`advance_download_queue`/`start_sync_pass`/`validate_save_dir`/`arm_sync_resume`/`ensure_sync_db`). GUI-only side-effects (Live charts, Replay auto-route) are behind the `SyncHost` trait — the GUI passes a `GuiSyncHost` over disjoint field borrows; the agent passes a no-op host. `AppState` owns one `SyncCore` as `self.sync`; `AppState::pump_ble_events` is a thin adapter. **Touch the engine in `sync_core.rs`, not by duplicating logic in the agent.**
- **Config.** `~/.movementlogger/config.toml` (`agent_config::AgentConfig`, atomic write): `box_id`, `save_dir`, `keep_synced`, `log_mode_manual`. GUI is the writer (`SyncCore::persist_config()` on Connect / save-dir validate / Keep-synced toggle / `LogMode`); agent is the reader. `$HOME`-anchored like the sync DB.
- **Coordination — GUI wins, agent yields (decided architecture, do not change without re-confirming).** `coord.rs` advisory locks under `~/.movementlogger/` (fs4, with std-`File::lock` fast path): `gui.lock` held by the GUI for its lifetime = "a GUI is alive" (OS frees it on crash — no stale PID); `ble.lock` = BLE-adapter ownership; `agent.lock` = agent single-instance. The GUI takes `gui.lock`+`ble.lock` in `ensure_ble` (non-blocking, never freezes the UI) and drops them in `on_exit`. The agent polls `coord::is_held(GUI_LOCK)` every ~250 ms tick and on every outer pass; when a GUI appears it runs the **same abort+Disconnect teardown as `on_exit`**, releases `ble.lock`, and idles until the GUI is gone, then resumes (lossless via the local mirror). ~2–5 s handover gap is by design.
- **Teardown.** The agent has no `on_exit`; `ctrlc` (with the `termination` feature) catches SIGINT/SIGTERM (launchd/systemd stop) and Windows console-close, flips an `AtomicBool` the main loop polls, and runs the identical `request_abort()` → `BleCmd::Disconnect` → grace-sleep path. Never reintroduce a bare disconnect here (same CoreBluetooth footgun as the GUI).
- **Autostart, gated on box log mode.** `autostart.rs` `register()`/`unregister()`/`sync_with_mode(manual)` wired into the `SyncCore` `LogMode` handler: AUTO installs the login item, MANUAL removes it (idempotent, best-effort — a failed write must never break sync). macOS: `~/Library/LaunchAgents/com.ywesee.movementlogger.agent.plist` pointing at the **signed `.app`'s** `Contents/MacOS/MovementLogger` (Bluetooth TCC is keyed to the bundle code-signature — launchd must start that exact Mach-O, not a bare copy), `RunAtLoad=true KeepAlive=false` + `launchctl bootstrap`. Linux: XDG `~/.config/autostart/movementlogger-agent.desktop`. Windows: HKCU `…\Run` (`winreg`, windows-only dep). `--register-autostart`/`--unregister-autostart` argv entry points (also windowless) let the updater/agent re-point post-swap.
- **macOS TCC precondition.** A headless launchd agent can't show the Bluetooth consent prompt. By the time `LogMode` resolves AUTO the GUI has already used BLE (it read the mode over BLE), so the grant is recorded against `com.ywesee.movementlogger` and the agent inherits it. Don't register autostart from a path that hasn't gone through a GUI BLE connect first.
- **Updater.** The agent writes `~/.movementlogger/agent.pid` under `agent.lock`. `installer::stop_running_agent()` (called at the top of every macOS/Linux/Windows swap helper) SIGTERMs/taskkills it before the bundle swap (700 ms grace to drop BLE + free `ble.lock`); the post-update GUI re-runs `autostart::register()` on startup when the box is AUTO, re-pointing the login item at the new bundle and (macOS) bootstrapping a fresh agent without a re-login.

### Resume + auto-reconnect (v0.0.11)

Downloads are resumable at the byte level. The firmware READ opcode takes `<name>\0<offset:u32-LE>` and `SDFat_Seek`s there (`movement_logger_firmware/Src/ble.c` READ handler), so the desktop never restarts a file from 0 after a drop:

- **Live-mirror model (v0.0.14, replaced the `.part`/rename scheme):** every download accumulates straight into the final file `<dir>/<name>` (`mirror_path` / `mirror_offset` / `append_mirror` in `main.rs`); there is no `.part` and no rename. The local file *is* the running mirror — always a valid prefix of the box file (logs are append-only) — and its length is the single source of truth for the resume/grow offset. This unifies three cases into one: first download, resume after a drop, and "the log grew since last sync" are all just "fetch from offset = local size". Survives an app restart identically. `mark_synced` is recorded for audit when a pass completes a file.
- `advance_download_queue` derives the offset from `mirror_offset` and sends `BleCmd::Read { name, size, offset }`. Local larger than the box size ⇒ the box file rotated (name reused) ⇒ local is deleted and refetched from 0.
- **Segmented READ (v0.0.48) — `READ_SEGMENT_BYTES` (64 KB) in `ble.rs`.** A single `BleCmd::Read` no longer streams the whole file in one on-air burst. `ble.rs::read` requests only the first `min(remaining, 64 KB)` via the firmware `cap` (`READ <name>\0<offset><cap>`, firmware v0.0.24+); `handle_notification`'s `Reading` arm flushes each completed segment to the mirror as a new `BleEvent::ReadChunk` (drains `content` → worker RAM bounded to one segment, progress persisted incrementally / crash-safe), advances `base`/`segment_end`, and issues the next capped READ. The terminal `ReadDone` still fires at true EOF (`expected` stays the full LIST size). **Why:** one 200 MB READ kept the box's READ FSM in `FSM_READ_STREAM` for the entire multi-hour transfer, so any macOS BLE pause ≥ 45 s tripped the firmware's `FSM_READ_STALL_DEADLINE` → `ble_recover_lost_peer` failed to re-advertise (rc=211, ACL-TX saturated) → the box went radio-silent and the desktop's reconnect had nothing to connect to (Peter's overnight freeze; ERRLOG 04.07.2026: 13 stalls, all `re-adv=211`). Cycling the box's FSM back to IDLE every ≤ 21 s (64 KB even at ~3 KB/s) keeps it out of that trap. **Deliberately NOT paired with lowering `READ_STALL_TIMEOUT`** (still 45 s) — dropping the desktop side to 20 s was tried and rolled back for churn/non-convergence (see that const's history); segmenting fixes the box side without touching the desktop ride-out. **Legacy-safe:** firmware that ignores `cap` streams to EOF; a lone `0xB0` BUSY on a non-first `Reading` notify (spurious reply to a continuation READ that raced the FSM, or a cap-ignoring box still streaming) is dropped (never legitimate multi-byte data) and `segment_end` is set to `expected`, reverting to the old single-stream behaviour with no mirror corruption. Confirmed against Peter's box, which honours `cap` (ERRLOG: 386 `done sent=<cap> (cap)` markers). This is a *mitigation*, not a cure for the firmware re-advertise wedge — a stall mid-segment can still wedge; the firmware `ble_recover_lost_peer` re-advertise/HCI-reset fix is the real cure.
- On a mid-READ drop the worker emits `BleEvent::ReadAborted { name, content, base }` (from `disconnect_inner` and the watchdog timeout) carrying the partial segment; `main` appends it to the mirror file so the resume continues from the true break point, not the last completed segment.
- The worker does the auto-reconnect itself (`auto_reconnect`, reusing `connect_core` + a short rescan each round). Success → `Connected` (main re-LISTs, the size-diff re-queues the unfinished/grown file, which resumes from the mirror length).
- **Two regimes, chosen at runtime by the "Keep synced" checkbox (v0.0.19).** The checkbox is mirrored every frame into `BleShared::persist_reconnect` (a shared `AtomicBool`):
  - **Auto Mode (Keep synced on):** *unbounded* retries with exponential backoff (`RECONNECT_INTERVAL` 2 s → ×2 each failed round, capped at `RECONNECT_BACKOFF_CAP` 60 s). Only success or an explicit abort exits. This is deliberate: `movement_logger_firmware` #3 builds #57–#65 made the box self-heal (HCI-disconnect + local re-adv, NRST chip-reset, stale-conn detect) and stay discoverable across 20–30 recovery cycles, so the old cap was the *only* thing stopping a multi-hour sync from finishing. "Leave the GUI running, the file is complete at the end." This **reverses** the earlier "bounded then manual" product decision — that decision's premise (box invisible until power-cycle) no longer holds; do not reinstate a hard cap in Auto Mode without re-confirming with Peter/Zeno.
  - **Manual mode (Keep synced off):** bounded to `RECONNECT_ATTEMPTS` (10), then `Disconnected` → manual-reconnect banner (lossless via the live mirror). A big manual sync still completes via byte-resume; it just doesn't retry forever, and the user can abort.
- **Cancellation.** The worker still can't service its command channel while inside `auto_reconnect`, so a bare unbounded loop would make Disconnect/quit hang forever. `auto_reconnect` therefore polls `BleShared::abort_reconnect` between every attempt **and** in ≤250 ms slices during every backoff/scan sleep (`sleep_or_abort`). The GUI sets that flag via `BleBackend::request_abort()` **before** sending `BleCmd::Disconnect` on: the Disconnect button, the START_LOG teardown, and `on_exit`. Un-ticking "Keep synced" mid-loop flips `persist_reconnect` false → the loop falls back to the bounded budget and stops on its own (no separate abort needed). `auto_reconnect` clears a stale abort on entry so a previous link's Disconnect can't kill a fresh recovery.
- `disconnect_inner` (and `connect_core`'s error paths) tear the link down via `drop_link`, which bounds `Peripheral::disconnect()` with a 3 s timeout. On macOS a peer that vanished without a clean LL_TERMINATE makes CoreBluetooth's `disconnect()` block ~indefinitely; unbounded, that froze the worker *before* `auto_reconnect` (the "Sync bleibt stehen" symptom — watchdog fired, no reconnect). Never reintroduce a bare `p.disconnect().await` on the teardown path.
- **`connect_core` is the connect-side twin (v0.0.18).** Every GATT step there — `connect()`, `discover_services()`, `subscribe()`, `notifications()` — is wrapped in `tokio::time::timeout(LINK_OP_TIMEOUT, …)` (12 s). Same CoreBluetooth footgun as `drop_link` but on the connect path: after an unclean drop a stale connection state makes `connect()`/`discover_services()` block ~forever, which froze the worker *inside* `auto_reconnect`'s first attempt — the 10-attempt loop never advanced, `Disconnected` was never emitted, and the box kept advertising ("visible in BT") while the app sat dead. On a connect timeout `drop_link` is also called to cancel the pending CoreBluetooth connect so the next attempt doesn't fight a half-open link. Never drop these timeouts.
- **A stalled LIST does NOT auto-reconnect (v0.0.21 — reverses the v0.0.18 change above).** `tick_watchdog`'s `CurrentOp::Listing` timeout arm returns `false`: it logs `"LIST timed out — no notifies for 20 s"`, leaves `op` at Idle (already replaced), and **keeps the link**; the next periodic LIST (every 30 s under Keep synced) retries on the still-alive link. This matches the **Android app** (`movement_logger_android` `BleClient.tickWatchdog`), which does exactly this and was the proven-working reference against the same box/firmware where this desktop was looping. v0.0.18 had made this arm return `true` (tear down + `auto_reconnect`) on the theory that a zero-row LIST stall is a "genuinely dead link"; that premise is wrong — combined with v0.0.19's *unbounded* Auto-Mode reconnect it turned one slow LIST into an infinite teardown→reconnect→re-LIST→stall loop with the box advertising throughout (Peter's "FileList geht nicht mehr, Box ist BT sichtbar"). The `rows_seen > 0` → `ListDone` inactivity-done early return (a LIST that got rows then missed only the terminator) is unchanged. Stalled **READ** still reconnects (that path is real — the box never saw a formal disconnect mid-transfer); only LIST changed. Don't reinstate reconnect-on-LIST-stall without re-confirming against the Android reference behaviour.
- **Both** drop signals trigger the reconnect, not just one: (a) the notification stream closing (`notif None` arm), and (b) `tick_watchdog` returning `true` for a stalled READ. (b) is the one that matters in practice — a real macOS drop (BLE power-nap, link-supervision timeout, worker falling behind) frequently does **not** close the stream and the box never sees a formal disconnect, so the watchdog (20 s of zero notifies) is the only signal. The GUI's 20 s `OP_IDLE_TIMEOUT` sits deliberately above the firmware's ~15 s stall detector so the firmware handles its side first. Don't add a heartbeat READ during a transfer: the firmware FSM is single-op, so a probe READ mid-transfer returns BUSY — the watchdog→reconnect→mirror-resume is the safe recovery path.

- **macOS reconnect: retrieve-by-id first, then scan (v0.0.36), holding ONE pending connect (v0.0.37).** btleplug on macOS reconnects *only* by scan re-discovery, but CoreBluetooth stops returning a recently-connected peripheral in scan results — so a dropped link left the desktop scanning a box it couldn't see, the "must power-cycle the box on Mac" bug iOS/Android don't have. Fix has two layers: (1) the vendored **`btleplug-patched`** fork adds `retrievePeripherals`-by-identifier + `Central::add_peripheral`, and `auto_reconnect` tries **retrieve-by-id first** (no scan), scan only as fallback; (2) the reconnect connect step holds **one un-timed pending connect** (`ConnectMode::Reconnect`, cancel only on abort / Keep-synced-off / a 60 s Manual budget via `connect_wait`) instead of self-cancelling every `LINK_OP_TIMEOUT` and going scan-blind. Manual first-connect is unchanged (`ConnectMode::Manual`, bounded + cancel-on-timeout). **This is host-side recovery only.** The *large-file* connect-hang (box emits no connectable ADV after a big-file mid-READ drop — retrieve OK but `connect: timed out` repeatedly, only a box power-cycle recovers) is a **firmware** dead-end (`movement_logger_firmware/Src/ble.c`: `ble_aci_cmd` ate the mid-transfer `Disconnection_Complete`; `ble_recover_lost_peer` was a one-shot with no periodic re-advertise re-arm). No desktop change can reach a non-advertising box — the firmware re-advertise re-arm is the real fix for that tail.
- **macOS disconnect: box tears the link down itself (`0x0F DISCONNECT`, firmware v0.0.22+).** On macOS `cancelPeripheralConnection` (btleplug `drop_link`) tears down only the *app's* view of the link — `bluetoothd` keeps the ACL alive with LL keepalives, so the box never sees `LL_TERMINATE`, its supervision timer never fires, and it stays "connected-in-limbo" advertising non-connectably → the **next central (iPhone AND Mac) can't connect until a box power-cycle / Mac-BT toggle / the box's 90 s peer-gone watchdog**. iOS tears the link down for real, so it doesn't hit this. There is **no CoreBluetooth API** to force the physical teardown from the host. Fix: `disconnect_inner` writes opcode **`0x0F`** (`OP_DISCONNECT`) as an **ACK-confirmed `WriteWithResponse`** (`write_cmd_confirmed`, bounded `HOST_DISCONNECT_ACK_TIMEOUT` 3 s) *before* `drop_link`, on **every** teardown path (intentional Disconnect *and* drop-recovery — on a "silent drop" the ACL is often only app-dead, so the write still reaches the box and frees it; on a truly dead link it fails fast/bounded). A `WriteWithoutResponse` is useless here: btleplug resolves it instantly and CoreBluetooth drops the still-queued frame on cancel. The firmware (`Src/ble.c`) routes `0x0F` to `ble_recover_lost_peer()` (`HCI_Disconnect` from the box side → real `LL_TERMINATE` + re-advertise). Legacy firmware silently ignores `0x0F`. Paths that abandon a link *outside* `disconnect_inner` (pending-connect abort race, post-connect failures in `connect_core`) use **`polite_drop_link`** — it writes `0x0F` through a **synthetic Characteristic** (`FSYNC_SVC_UUID`+`FILECMD_UUID`) because the app-side char cache is empty on the raced-connect path (only the CB-thread map is populated, by delegate auto-discovery), retried ×2 bounded.
- **macOS idle-drop detection: `CentralEvent::DeviceDisconnected`, NOT the notify stream.** The vendored fork's per-peripheral notification stream **never closes on a CoreBluetooth remote disconnect** (`PeripheralEventInternal::Disconnected` is ignored, and the worker's own `Peripheral` clone keeps the broadcast sender alive) — so the worker-loop `notif None` arm is unreachable on macOS and is only the Linux/Windows drop signal. A drop while `op == Idle` (the gap between queued files) previously went completely undetected: no reconnect, no `Disconnected` event, GUI stuck showing Connected with Scan/Reconnect greyed, box stranded (total deadlock, 2026-07-02). Fix: `ensure_adapter` spawns a pump forwarding `adapter.events()` → `DeviceDisconnected` ids into `central_rx`; the connected worker-loop branch sweeps it non-blockingly each iteration (≥5 Hz via the 200 ms watchdog tick) and routes a match through the shared `lost_link_recover` (teardown incl. 0x0F → `auto_reconnect` → else `Disconnected`). `connect_core` drains the queue on success so a late didDisconnect from the *previous* link can't kill the fresh one.
- **Worker-freeze traps (vendored-fork map holes).** On a peripheral that a late `didDisconnect` purged from the fork's CB-thread map, `is_connected` and `write` **never resolve their reply future** (internal.rs sets no reply on a map miss). On the single-worker-thread runtime one unbounded await = the whole BLE backend frozen until app restart. Therefore: `is_connected` only via the bounded `link_probably_connected` helper; `write_cmd` is timeout-bounded (`LINK_OP_TIMEOUT`). **Never add an unbounded CoreBluetooth await.**
- **Abort-flag lifecycle: the `BleCmd::Disconnect` handler consumes it.** Every `request_abort()` caller (Disconnect button, START_LOG teardown, `on_exit`, agent stop) queues a `BleCmd::Disconnect` right after; the handler's trailing `clear_abort()` is the *only* consumption point. `auto_reconnect` must **NOT** clear it on entry (the old entry-clear silently swallowed a user's Disconnect click that landed during the preceding teardown → unbounded reconnect against the user's will), and nothing else may leave it set (a lingering flag made every later Scan abort in ~0 ms → "scan saw N, 0 matched" forever — Peter's "Scan zeigt nichts").

## Firmware update over BLE (FOTA) — `--flash-firmware`

The box's firmware can be upgraded over BLE (dual-bank OTA, firmware v0.0.29+). Same FileData characteristic as LIST/READ; opcodes `0x09` FW_BEGIN `<len32_LE><sha256:32>` (erases the inactive bank, ~1 s), `0x0A` FW_DATA `<off32_LE><bytes…>` (one image chunk, 4-byte LE ack), `0x0B` FW_COMMIT (verify SHA-256, swap banks, RESET), `0x0C` FW_ABORT. Wire spec + ack codes are in the `ble.rs` module header. The image is written to the **inactive** bank, so the box stays bootable on its current firmware the whole time and only swaps on COMMIT — an interrupted upload is harmless (the next FW_BEGIN re-erases and restarts from 0; FOTA is *not* byte-resumable like file READ).

- **Driver.** `BleCmd::FlashFirmware { bytes }` → `ble.rs::flash_firmware` reads the `.bin`, computes SHA-256, sends FW_BEGIN, streams FW_DATA in `FW_CHUNK` (160 B) chunks **stop-and-wait** (each chunk waits its ack before the next; FW_DATA idempotent + retried `FW_DATA_RETRIES`×), then FW_COMMIT. Drives the single-op `CurrentOp::Flashing` slot (rejects/rejected-by other ops with "another op is in flight"). Progress surfaces via `BleEvent::FlashStarted/FlashProgress/FlashDone/FlashError` → `SyncCore::ble_flash_progress`/`ble_flash_msg`. The flash is driven *inline* (it never yields to the watchdog — its own per-step timeouts cover stalls), so a FileData notify never reaches the demux while a flash is in flight.
- **Throughput is inherently slow** (~90 B/s ≈ 1.7 s/chunk on a relaxed connection interval): each FW_DATA round-trips the BLE link *and* waits for the STM32 to program that chunk into flash before acking. A ~100 kB image takes **~20 min**. This is normal SensorTile.box OTA — don't "optimise" it into a windowed/pipelined scheme without firmware-side support; the FSM is single-op.
- **GUI path.** A flash button in the Sync tab backs the `ble_flash_*` fields; the user naturally clicks it *after* the connect-time LIST has drained, so it doesn't race.
- **Headless CLI path — `MovementLogger --flash-firmware <path.bin>`** (`agent::flash`). Like `--agent`/`--register-autostart`, it returns from `main()` before any eframe/egui/winit code, so no window opens and the macOS winit-patch invariant is untouched. Scans → connects (box from `config.toml`'s `box_id`, so the box must have been GUI-connected once) → uploads → prints `[flash] N%` to stderr; exit `0` success, `1` flash error/timeout, `2` file missing/bad arg. macOS TCC precondition is the same as `--agent`: the BLE grant must already be recorded from a prior GUI connect (a headless launch can't show the consent prompt).
  - **Connect-time op race (fixed).** The shared `Connected` handler queues SET_TIME → LIST → (after ListDone) GET_MODE, all of which take the single-op slot — on a legacy box that never answers GET_MODE the slot is held for the full 20 s `OP_IDLE_TIMEOUT`. `agent::flash` fires the flash immediately on `Connected`, so its first FW_BEGIN loses that race and the worker bounces it with "another op is in flight". The CLI driver treats *that specific* rejection as transient: it re-arms `start_firmware_upload` and retries until the connect-time ops drain (bounded by the pass deadline), rather than bailing. Don't restore the bail-on-first-rejection behaviour.
  - **Deadline.** The whole pass (scan→connect→upload) is bounded by a wall-clock `deadline` in `agent::flash` — **2400 s** (was 600 s, which timed out a real ~100 kB flash mid-stream at ~56% given the ~90 B/s rate). Keep it comfortably above `image_bytes / 90 B/s` plus FW_DATA-retry slack.

## BLE debug probe — `--ble-debug`

`MovementLogger --ble-debug [seconds]` (`ble_debug.rs`, v0.0.44) is the GPS-Debug analogue for the BLE side: a headless, terminal-only diagnostic a remote user can run with their installed (signed, TCC-granted) app and send us the dump. Like `--agent` it returns from `main()` before any window code. Unlike the GUI scan (which enumerates `peripherals()` once at the end), it subscribes to `adapter.events()` and prints **every CoreBluetooth discovery event live** — so "the box's ADVs never arrive" vs "arrive name-less" vs "name filter misses" are distinguishable (built for Peter's macOS-15.7.4 stale-name-cache case, where the GUI scan showed `0 matched` while nRF/Android saw the box fine). Sequence: adapter+config dump → event-scan (default 20 s; foreign devices' raw adv-payload events are suppressed, box's are printed) → per-device summary with `<-- BOX (name)`/`<-- BOX (saved id)` flags → macOS retrieve-by-id test for the saved `box_id` → bounded connect probe (connect → char list → subscribe FileData → LIST → count reply notifies → "END-TO-END LINK WORKS" → 0x0F → disconnect). Every CoreBluetooth await is bounded (`STEP_TIMEOUT` 12 s — same fork-map-hole rationale as `ble.rs`). `RUST_LOG=btleplug=debug` adds btleplug's internal stderr trace (`env_logger`, default filter `warn`). Zero devices seen ⇒ the terminal app lacks the Bluetooth TCC grant.

## GPS Debug tab — USB serial + BLE bridge

The **GPS Debug** tab (`enum Tab::Gps`) runs the `gps-debug` u-blox UBX survey (antenna/mounting diagnostics: fix quality, per-signal C/N0, MON-RF antStatus/AGC/jamming). The survey engine itself is the `gps-debug` crate, which is now **both a bin and a lib** (`gps-debug/src/lib.rs`): the transport-agnostic poll/parse loop is `survey::run_core<T: Read + Write>` and the GUI depends on the crate (`gps-debug = { path = "../gps-debug" }`). One survey engine, two transports — pick via the `GpsTransport` radio in the tab:

- **USB serial** (default, the long-standing path): the GUI still spawns the standalone `gps-debug` **sidecar binary** as a child process against a serial port (`start_gps_debug` → `gps_debug_path()`), streaming its stdout into the log panel. **Port auto-detect** (`gps_debug::detect_ports()`, std-only, no new dep): macOS `/dev/cu.usbserial*`/`cu.usbmodem*`/SiLabs/CH34x, Linux `ttyACM*`/`ttyUSB*`. Seeded once at startup into `gps_dbg_port`/`gps_dbg_detected_ports`, re-scanned by the **🔄 Detect** button, surfaced as a ComboBox + free-text fallback. The CLI gained `--list-ports` and an optional `--port` (auto-picks when exactly one candidate exists). Windows enumeration is left to manual `COMx` entry.
- **BLE bridge** (no cable, no opening the box — `start_gps_debug_ble`): runs `survey::run_core` **in-process** on a worker thread, tunnelling the u-blox UART through the connected box's existing BLE link. Requires the box to be connected in the Sync tab first; legacy firmware silently ignores the bridge opcode, so the survey just shows "no NAV-PVT reply".

  **Transport plumbing (BLE):** `BleTransport` (main.rs) implements `Read + Write` — `write` → `BleCmd::GpsTx { bytes }` (UBX poll frames → box → u-blox UART); `read` drains raw bridged UBX bytes (≤50 ms block, then `Ok(0)` to match the serial timeout contract). The bytes flow on a **dedicated side-channel**, deliberately *not* through `sync_core`'s shared `BleEvent` pump (which the agent also drives): `BleShared` gained `gps_bridge: Arc<AtomicBool>` + `gps_data_tx: Arc<Mutex<Option<Sender<Vec<u8>>>>>`. The worker's `handle_notification` routes FileData notifies to `gps_data_tx` whenever `gps_bridge` is set (before the FSM sees them); `BleCmd::GpsBridge { on }` writes opcode `0x0D` + flips the flag (clearing the sink on off); `BleCmd::GpsTx` writes `0x0E` + bytes. Both are fire-and-forget (no `CurrentOp` slot, like `StopLog`/`SetTime`). `BleBackend::{cmd_sender, set_gps_data_sink}` expose the channel to the GUI survey thread. **The BLE survey assumes you are not simultaneously syncing** — both ride FileData, and the bridge flag would misroute a LIST/READ's notifies. Keep-synced should be off during a survey; the firmware also only emits bridge notifies while its FileSync FSM is idle.

The firmware half lives in `movement_logger_firmware` (opcodes `0x0D GPS_BRIDGE` / `0x0E GPS_TX` in `ble.c`; raw UBX frame capture + `$PUBX,41` output-protocol toggle in `gps.c`). NMEA keeps flowing to the SD logger the whole time — enabling the bridge only *adds* UBX output on the port. See that repo's docs for the wire details.

## ERRLOG auto-check — `errlog_check.rs` + `--check-errlog`

Automatic per-boot health grading of the box's mirrored `ERRLOG.LOG`. The
firmware appends one section per cold boot (banner `--- Boot … ---`, init
lines, a cumulative `gps_diag: …` counters line every 5 s — see
`movement_logger_firmware/Src/logger.c:345`); `errlog_check::analyze` splits
the mirror into boots and grades each one OK/WARN/FAIL: subsystem `init FAIL`
and `***` markers (with joint-level GPS-wiring diagnosis for `GPS NO NMEA` /
`no baud lock`) → FAIL; IWDG/WWDG reset, GPS stuck at the 9600 fallback
(box→module wire suspect), growing NMEA-checksum / UART-error counters,
mid-run UART silence windows, failed BLE re-advertises, unconfirmed FOTA
trial boots → WARN. **Auto-run, no manual step:** `SyncCore::run_errlog_check`
fires from the `ReadDone` handler whenever a sync pass pulls `ERRLOG.LOG`
(so every new box boot is graded as soon as its log lands — the headless
`--agent` gets this for free via the shared engine) and once at GUI startup
on the existing mirror. Rendered as "Box health (ERRLOG.LOG)" at the top of
the **Debug tab** (latest boot + collapsible history); the Sync tab shows a
one-line WARN/FAIL badge pointing there. `MovementLogger --check-errlog
[path]` (default `<save base>/csv/ERRLOG.LOG`) runs the same analysis
headless — exit 0/1/2 = latest boot OK/WARN/FAIL — for scripting/cron.

Parser hard-won robustness rules (all observed in Peter's real 1.2 MB
mirror and/or confirmed by adversarial review — don't simplify them away):

- **Torn/spliced lines** (sudden power cut mid-write, F-PWR-5): any
  `gps_diag` line failing the strict 8-distinct-keys parse (bitmask, not a
  count — splices can repeat one `k=v` while dropping another) is torn,
  never trusted. A diag line with a **missing or implausible `[N ms]` tick**
  is also torn: > 30 days (`TICK_CAP_MS`) or a > 6 h forward jump
  (`MAX_TICK_JUMP_MS` — digit-fused ticks like `[79642534253 ms]` carry 8
  plausible-looking fields and would otherwise poison uptime + frontier).
  If a fused tick does get accepted, 3 mutually-consistent rejected samples
  **self-heal** the frontier back onto the live stream.
- **Replayed byte ranges**: the live-mirror byte-resume can leave duplicated
  regions (same tick+content twice, or a short rewind). A **monotonic-tick
  frontier** skips any diag sample at/below the highest accepted tick —
  without it, duplicates read as fake "UART went silent" stalls (boot #4
  showed 24 phantom stalls, real count 0). Graded non-diag lines (markers,
  `init FAIL`, BLE failures) with a tick strictly below the frontier are
  skipped too (no double counting). A **replayed boot banner** would mint a
  phantom "latest boot" and flip the verdict/exit code — a short section
  whose raw lines exactly prefix-match the previous boot's head is merged
  away, but **only when ≥ 1 matching line is tick-stamped**: banner/fw/reset
  are constant strings identical across genuine boots, and a tickless
  3-line section is a real rapid power-cycle (magnet flicker), not a replay.
- **Signed rc values**: the firmware prints `re-adv=`/`rc=` with `%d`; a
  dead SPI/BLE chip yields `-10`/`-11` — the exact box-invisible condition.
  Parse them signed (`field_i64`); an unsigned parse + `unwrap_or(0)`
  silently inverted these failures into success. Torn/absent values are
  "not proven", not zero.
- **GPS-survey bridge windows**: between `gps: bridge ON` / `gps: bridge
  off` the UBX/NMEA interleave inflates `lines_bad` (~300–700 per survey in
  real logs). Checksum/error growth in bridge windows — including a window
  that merely *touches* a bridge toggle at either end — is bucketed
  separately and reported as expected Info, never as a wiring WARN.
- Stall detection requires a real tick advance (≥ 2.5 s) between equal-byte
  samples, only outside bridge windows; pair deltas are skipped across
  > 10 min discontinuities.
- Boots with < 6 lines and no GPS-ready are "cut short during init (power
  removed?)" — but **hard evidence is graded before that early-return**:
  a 4-line boot ending in `fuel: init FAIL` or carrying `reset: IWDG` must
  FAIL/WARN, not hide behind "too little data" (bootloops produce exactly
  this shape on every iteration).
- CLI: a file with zero `--- Boot` sections exits 3 (wrong file), never a
  false-healthy 0. The default path prefers `config.toml`'s `save_dir`
  (which IS the csv dir — the agent assigns it to `ble_out_dir` verbatim)
  over the cwd-derived save base.

## Box-sourced board-orientation calibration (v0.0.61+) — `calibration.rs`

The four calibration fields (`nose_plus_y`, `mag_offset_mg`, `angle_zero_ref`
+ `angle_zero_at`, `heading_bias_deg`) live on the BOX in `CAL.CFG`
(firmware v0.0.37+) — no longer just in `~/.movementlogger/config.toml`.
Three of them are physical facts of the specific box (which end is the
nose, hard-iron of *this* magnetometer, the pose the user picked as level
for *this* board), so the box is the source of truth and every host that
connects gets the same values.

- **Wire format** (32-byte blob, per-field `valid_mask`, tenths-of-degree
  fixed point, LE u64 epoch ms): `stbox-viz-gui/src/calibration.rs`
  (`encode` / `decode`). Byte-for-byte matched by iOS
  `MovementLogger/BLE/Calibration.swift` and Android
  `.../ble/Calibration.kt` — the module is the ONLY spec; do not
  re-derive per-field offsets elsewhere.
- **On connect**: `SyncCore` chains `BleCmd::GetCalibration` after
  `GET_GPS_POWER` completes (same self-guarded slot pattern as
  `GET_MODE` / `GET_VERSION`). Reply → `BleEvent::Calibration(Some(blob))`
  → `SyncHost::on_calibration` in `main.rs` merges each Some-field into
  `AppState` + `AgentConfig` (fields the box hasn't set leave the local
  value alone). Legacy firmware (< v0.0.37) times out silently and
  emits `Calibration(None)` — the app keeps its local `config.toml`.
- **On any user-driven tap**: `AppState::push_cal_to_box(EncodeInput)`
  fires `BleCmd::SetCalibration` with ONLY the touched field's bit set,
  so the box's per-field merge leaves the other stored fields alone.
  Call sites: `Zero here` / `Clear` (angle-zero), `USB-C points SOUTH`
  (heading bias), nose-up confirm (nose + nudged bias in one atomic
  blob), `Reset calibration` (all four bits with zero payloads =
  wipe).

**Deliberately not synced**: continuous mag-offset auto-cal. Each host's
convergence loop would push on every tick and churn `CAL.CFG`. Only the
explicit `Reset calibration` tap pushes zeros. iOS + Android make the
same tradeoff.

## In-app updater — sharp edge

`stbox-viz-gui/src/update.rs` hardcodes the repo it polls for new releases. When importing fixes from `fp-sns-stbox1/Utilities/rust`, the upstream version of this file has `const REPO: &str = "zdavatz/fp-sns-stbox1"` — that is the **firmware** repo, not this one. **Always check this constant after a pull from upstream**:

```rust
const REPO: &str = "zdavatz/movement_logger_desktop";  // must match this repo
```

If wrong, the updater will offer "newer" upstream firmware releases as MovementLogger updates and the user will end up downloading the wrong DMG. v0.0.1 of this repo shipped with the bug; v0.0.2+ is correct.

The installer logic (`stbox-viz-gui/src/installer.rs`) verifies `codesign --deep --strict` on the downloaded macOS .app before swapping, so in-app updates on macOS will *fail closed* on an unsigned DMG. Don't ship unsigned macOS builds.

## Releases

Push a `vX.Y.Z` tag — `.github/workflows/release.yml` builds Linux x86_64, macOS arm64 (Apple Silicon) and Windows x86_64, packages each as `MovementLogger-vX.Y.Z-<triple>.{tar.gz,zip}`, builds a macOS `.app` + `.dmg`, signs + notarizes + staples the macOS bundle, and attaches everything to the auto-created GitHub Release.

The Apple signing/notarization secrets are already configured on this repo (`gh secret list -R zdavatz/movement_logger_desktop` to inspect names — values are not visible):

- `MACOS_DEVELOPER_ID_CERTIFICATE` / `MACOS_DEVELOPER_ID_CERTIFICATE_PASSWORD` — Developer ID Application cert (team `4B37356EGR`, ywesee GmbH).
- `APPLE_API_KEY_P8` / `APPLE_API_KEY_ID` / `APPLE_API_ISSUER_ID` — App Store Connect API key for `notarytool`.

The signing & notarization jobs gate on those secrets being non-empty, so they activate automatically — no workflow edit needed.

Mac App Store `.pkg` and Microsoft Store MSIX paths from upstream `fp-sns-stbox1` are intentionally omitted from this repo. Port them from `fp-sns-stbox1/.github/workflows/release.yml` if/when needed.

## Upstream

When pulling fixes from `fp-sns-stbox1/Utilities/rust`, the workspace `Cargo.toml`s here drop the `Utilities/rust` working-directory prefix and use repo-relative paths. Watch for that when copying steps. Also re-check `stbox-viz-gui/src/update.rs` for the REPO constant — see *In-app updater* above.
