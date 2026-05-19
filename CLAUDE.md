# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Cross-platform desktop build of **MovementLogger** — a drag-and-drop GUI for SensorTile.box pumpfoil session telemetry. Sources were imported from `fp-sns-stbox1/Utilities/rust` so this repo can cut its own release cadence and live separately from the firmware monorepo.

## Workspace layout

Cargo workspace at the repo root with three members and one excluded vendored fork:

- `stbox-viz/` — CLI session visualiser (plotters, rustfft, gif). The GUI shells out to its `animate` subcommand to render side-by-side MOVs.
- `stbox-viz-gui/` — the GUI itself (binary name `MovementLogger`, package name `movement-logger`). Built on `eframe`/`egui`, BLE FileSync via `btleplug` on a tokio worker thread, GitHub-Releases-based in-app updater.
- `usb-console/` — minimal `rusb` debug console for the SensorTile.box PRO USB CDC firmware. Independent; not shipped in the GUI release archive.
- `winit-patched/` — vendored fork of `winit` 0.30.13 with the private `_CGSSetWindowBackgroundBlurRadius` call removed from `Window::set_blur`. Required for Mac App Store binary scanning. Applied workspace-wide via `[patch.crates-io] winit = { path = "winit-patched" }` and **excluded** from the workspace members list (it ships its own workspace, must not nest).

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

## Runtime topology

The GUI is intentionally thin: it composes a `stbox-viz animate …` command line, spawns it as a child, streams stdout/stderr into the live log panel, and lets the user cancel. `stbox-viz` lookup order: next to `current_exe()`, then `PATH`. The release archives ship both binaries side by side, and the macOS `.app` bundles `stbox-viz` inside `Contents/MacOS/`. `ffmpeg` must be on `PATH` for `--video` mode.

The egui UI is organised into three top-level tabs (`enum Tab` in `main.rs`): **Live** (SensorStream readouts), **Sync** (BLE FileSync), **Replay** (drag-and-drop CSV → animated video). Default tab is Live so the box-connected workflow lands there first; the previous single-panel layout collapsed into Replay. The egui top-bar in `update()` switches between them; the update-banner is rendered once above the active tab. The product name + version live **only** in the OS title bar (set in `main()`); the in-app top panel deliberately does **not** repeat them — it carries just the subtitle hyperlink and the top-right logo→mailto. Upstream `fp-sns-stbox1` renders a `MovementLogger v{CARGO_PKG_VERSION}` `ui.heading` there; drop it again on the next pull (removed here in v0.0.16 — it duplicated the title bar).

The Live tab derives an "Orient (°)" row (pitch / roll / heading) inline in `ui_live_tab` from the raw accel + mag values — standard eCompass formulas, no fusion filter. Pitch/roll are only meaningful when |acc| ≈ 1 g (between strokes), and heading is mag-only (no gyro integration) so it wobbles visibly when the board moves. If a future change needs smoothed orientation, lift the math out of `ui_live_tab` into a filter that consumes `BleEvent::Sample` in `pump_ble_events` instead.

BLE (`stbox-viz-gui/src/ble.rs`) runs its own tokio current-thread runtime on a worker thread; commands/events shuttle across `std::sync::mpsc` so the egui UI stays sync. Single notification stream demuxed by a `select!` loop on the worker side; the stream now carries both FileSync (FileData UUID `…40…`) and SensorStream (UUID `…100…`) notifies, branched on UUID. SensorStream is subscribed on Connect when the characteristic is present (PumpLogger firmware) and silently skipped on legacy SDDataLogFileX (`PumpTsueri`) builds. Wire spec for the 46-byte packed snapshot lives in upstream `DESIGN.md` §3; `LiveSample::parse` mirrors it.

Two transport modes:

- **Single-notify** (46 B exact) — used when the host accepts the box's MTU upgrade.
- **Chunked fallback** — three sequential notifies prefixed with sequence byte `0x00` / `0x01` / `0x02`; reassembled into 46 bytes in `StreamAsm` and parsed identically. Out-of-order chunks reset the buffer and wait for the next `0x00` start. Sample loss at 0.5 Hz is a 2 s gap — visible in the freshness label but never produces partial decodes.

Accepted BLE advertise names: `PumpTsueri` (legacy SDDataLogFileX) and `STBoxFs` (PumpLogger). Single source of truth is `BOX_NAMES` in `ble.rs`.

## Sync vs. transfer — `sync_db.rs`

The box firmware only speaks LIST / READ / DELETE / STOP_LOG / START_LOG — there is **no sync concept on the device**. "File transfer" (tick files → *Download selected* → serial READ queue → `save_downloaded_file`) is the old path and unchanged. "File **Sync**" (*Sync now* button) is a thin client-side layer on top:

- State lives in a local SQLite DB at `~/.movementlogger/sqlite/sync.db` (Windows `%USERPROFILE%\.movementlogger\sqlite\sync.db`) — `sync_db.rs::default_db_path`, cross-platform via `HOME`/`USERPROFILE`. `rusqlite` with the `bundled` feature so there's no system libsqlite (same self-contained rationale as `rustls-tls`). The `sqlite/` subdir is created by `SyncDb::open` via `create_dir_all`. Anchored to `$HOME`, **not** to the user-selectable "Save to" folder, so changing that folder doesn't orphan history and re-pull everything. (Changed in v0.0.7 from the flat `~/.movementlogger/sync.db`; no migration shim — v0.0.6 was brand-new so a stale flat DB just means one extra re-pull.)
- The DB is now an **audit log only** (v0.0.14): it records `(box_id, name, size, downloaded_at, local_path)` on each completed pull but is **not consulted** to decide what to fetch. `box_id` is the btleplug peripheral id, captured at Connect-click time into `AppState::ble_connected_id`.
- *Sync now* (`start_sync_pass`) clears the list, sets `ble_sync_pending`, sends a fresh LIST; the diff runs in the `ListDone` handler **only when that flag is set** (so the auto-LIST on Connect never triggers a sync). `run_sync_diff` decides per file by **local mirror size vs the box's LIST size**: `local < box` → fetch the tail from offset `local`; `local == box` → up to date; `local > box` → file rotated, reset and refetch. This is what makes a continuously-growing log fetch only its new bytes instead of re-downloading, and stops a big file starving GPS/BAT in the serial queue.
- **Keep synced** (`ble_keep_synced` checkbox): while connected and idle, `update()` calls `start_sync_pass` every `SYNC_POLL_INTERVAL` (30 s). A busy backlog runs back-to-back because the interval is measured from the previous *trigger* (`ble_last_sync_at`), long elapsed by the time a big pass drains. The box logs grow forever, so a sync that "completes" is never really final — this keeps the local files mirroring.
- **What Sync mirrors** is gated by `is_synced_name` (v0.0.20), **not** `is_sensor_data_name`. It's the per-session sensor data (`Sens*.csv` / `Gps*.csv` / `Bat*.csv` / `Mic*.wav`) **plus** the firmware's single rolling error log `ERRLOG.LOG` (`movement_logger_firmware/Src/errlog.c`). Before v0.0.20 the diff used `is_sensor_data_name` directly, so `ERRLOG.LOG` was never pulled — Peter's "ein Teil synchronisiert, Errlog fehlt aber". `is_synced_name` is deliberately a *superset* used only in `run_sync_diff`: `is_sensor_data_name` still drives the manual list's default-tick and the Sensor/Debug split, so `ERRLOG.LOG` stays in the Debug group, un-ticked, and is pulled *only* by Sync. The errlog is append-only so the live-mirror byte-resume path needs no changes. Add new box-side debug files to `is_synced_name` (not `is_sensor_data_name`) so Sync picks them up without disturbing the manual UI.
- **Sync never issues DELETE** — additive only, by explicit product decision. Don't add a delete-after-sync path without re-confirming.

### Resume + auto-reconnect (v0.0.11)

Downloads are resumable at the byte level. The firmware READ opcode takes `<name>\0<offset:u32-LE>` and `SDFat_Seek`s there (`movement_logger_firmware/Src/ble.c` READ handler), so the desktop never restarts a file from 0 after a drop:

- **Live-mirror model (v0.0.14, replaced the `.part`/rename scheme):** every download accumulates straight into the final file `<dir>/<name>` (`mirror_path` / `mirror_offset` / `append_mirror` in `main.rs`); there is no `.part` and no rename. The local file *is* the running mirror — always a valid prefix of the box file (logs are append-only) — and its length is the single source of truth for the resume/grow offset. This unifies three cases into one: first download, resume after a drop, and "the log grew since last sync" are all just "fetch from offset = local size". Survives an app restart identically. `mark_synced` is recorded for audit when a pass completes a file.
- `advance_download_queue` derives the offset from `mirror_offset` and sends `BleCmd::Read { name, size, offset }`. Local larger than the box size ⇒ the box file rotated (name reused) ⇒ local is deleted and refetched from 0.
- On a mid-READ drop the worker emits `BleEvent::ReadAborted { name, content, base }` (from `disconnect_inner` and the watchdog timeout) carrying the partial segment; `main` appends it to the mirror file so the resume continues from the true break point, not the last completed segment.
- The worker does the auto-reconnect itself (`auto_reconnect`, reusing `connect_core` + a short rescan each round). Success → `Connected` (main re-LISTs, the size-diff re-queues the unfinished/grown file, which resumes from the mirror length).
- **Two regimes, chosen at runtime by the "Keep synced" checkbox (v0.0.19).** The checkbox is mirrored every frame into `BleShared::persist_reconnect` (a shared `AtomicBool`):
  - **Auto Mode (Keep synced on):** *unbounded* retries with exponential backoff (`RECONNECT_INTERVAL` 2 s → ×2 each failed round, capped at `RECONNECT_BACKOFF_CAP` 60 s). Only success or an explicit abort exits. This is deliberate: `movement_logger_firmware` #3 builds #57–#65 made the box self-heal (HCI-disconnect + local re-adv, NRST chip-reset, stale-conn detect) and stay discoverable across 20–30 recovery cycles, so the old cap was the *only* thing stopping a multi-hour sync from finishing. "Leave the GUI running, the file is complete at the end." This **reverses** the earlier "bounded then manual" product decision — that decision's premise (box invisible until power-cycle) no longer holds; do not reinstate a hard cap in Auto Mode without re-confirming with Peter/Zeno.
  - **Manual mode (Keep synced off):** bounded to `RECONNECT_ATTEMPTS` (10), then `Disconnected` → manual-reconnect banner (lossless via the live mirror). A big manual sync still completes via byte-resume; it just doesn't retry forever, and the user can abort.
- **Cancellation.** The worker still can't service its command channel while inside `auto_reconnect`, so a bare unbounded loop would make Disconnect/quit hang forever. `auto_reconnect` therefore polls `BleShared::abort_reconnect` between every attempt **and** in ≤250 ms slices during every backoff/scan sleep (`sleep_or_abort`). The GUI sets that flag via `BleBackend::request_abort()` **before** sending `BleCmd::Disconnect` on: the Disconnect button, the START_LOG teardown, and `on_exit`. Un-ticking "Keep synced" mid-loop flips `persist_reconnect` false → the loop falls back to the bounded budget and stops on its own (no separate abort needed). `auto_reconnect` clears a stale abort on entry so a previous link's Disconnect can't kill a fresh recovery.
- `disconnect_inner` (and `connect_core`'s error paths) tear the link down via `drop_link`, which bounds `Peripheral::disconnect()` with a 3 s timeout. On macOS a peer that vanished without a clean LL_TERMINATE makes CoreBluetooth's `disconnect()` block ~indefinitely; unbounded, that froze the worker *before* `auto_reconnect` (the "Sync bleibt stehen" symptom — watchdog fired, no reconnect). Never reintroduce a bare `p.disconnect().await` on the teardown path.
- **`connect_core` is the connect-side twin (v0.0.18).** Every GATT step there — `connect()`, `discover_services()`, `subscribe()`, `notifications()` — is wrapped in `tokio::time::timeout(LINK_OP_TIMEOUT, …)` (12 s). Same CoreBluetooth footgun as `drop_link` but on the connect path: after an unclean drop a stale connection state makes `connect()`/`discover_services()` block ~forever, which froze the worker *inside* `auto_reconnect`'s first attempt — the 10-attempt loop never advanced, `Disconnected` was never emitted, and the box kept advertising ("visible in BT") while the app sat dead. On a connect timeout `drop_link` is also called to cancel the pending CoreBluetooth connect so the next attempt doesn't fight a half-open link. Never drop these timeouts.
- **A stalled LIST auto-reconnects too (v0.0.18).** `tick_watchdog`'s `CurrentOp::Listing` timeout arm returns `true` (not `false`) so the worker tears down + `auto_reconnect`s, identical to the stalled-READ path. Under **Keep synced** the worker spends most idle time in LIST (a fresh LIST every 30 s + the re-LIST after each reconnect), so a drop during LIST is the *common* case — the old `false` left peripheral+stream half-dead with no reconnect, and every subsequent 30 s LIST stalled the same way (sync silently dead, box still visible in BT). The `rows_seen > 0` → `ListDone` early return is unchanged (a LIST that got rows then missed only the terminator is still treated as success); only a zero-row 20 s stall — a genuinely dead link — now reconnects. Lossless: the reconnect re-LISTs and Keep-synced re-arms the pass; the mirror resumes from local size.
- **Both** drop signals trigger the reconnect, not just one: (a) the notification stream closing (`notif None` arm), and (b) `tick_watchdog` returning `true` for a stalled READ. (b) is the one that matters in practice — a real macOS drop (BLE power-nap, link-supervision timeout, worker falling behind) frequently does **not** close the stream and the box never sees a formal disconnect, so the watchdog (20 s of zero notifies) is the only signal. The GUI's 20 s `OP_IDLE_TIMEOUT` sits deliberately above the firmware's ~15 s stall detector so the firmware handles its side first. Don't add a heartbeat READ during a transfer: the firmware FSM is single-op, so a probe READ mid-transfer returns BUSY — the watchdog→reconnect→mirror-resume is the safe recovery path.

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
