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

The egui UI is organised into three top-level tabs (`enum Tab` in `main.rs`): **Live** (SensorStream readouts), **Sync** (BLE FileSync), **Replay** (drag-and-drop CSV → animated video). Default tab is Live so the box-connected workflow lands there first; the previous single-panel layout collapsed into Replay. The egui top-bar in `update()` switches between them; the update-banner is rendered once above the active tab.

The Live tab derives an "Orient (°)" row (pitch / roll / heading) inline in `ui_live_tab` from the raw accel + mag values — standard eCompass formulas, no fusion filter. Pitch/roll are only meaningful when |acc| ≈ 1 g (between strokes), and heading is mag-only (no gyro integration) so it wobbles visibly when the board moves. If a future change needs smoothed orientation, lift the math out of `ui_live_tab` into a filter that consumes `BleEvent::Sample` in `pump_ble_events` instead.

BLE (`stbox-viz-gui/src/ble.rs`) runs its own tokio current-thread runtime on a worker thread; commands/events shuttle across `std::sync::mpsc` so the egui UI stays sync. Single notification stream demuxed by a `select!` loop on the worker side; the stream now carries both FileSync (FileData UUID `…40…`) and SensorStream (UUID `…100…`) notifies, branched on UUID. SensorStream is subscribed on Connect when the characteristic is present (PumpLogger firmware) and silently skipped on legacy SDDataLogFileX (`PumpTsueri`) builds. Wire spec for the 46-byte packed snapshot lives in upstream `DESIGN.md` §3; `LiveSample::parse` mirrors it.

Two transport modes:

- **Single-notify** (46 B exact) — used when the host accepts the box's MTU upgrade.
- **Chunked fallback** — three sequential notifies prefixed with sequence byte `0x00` / `0x01` / `0x02`; reassembled into 46 bytes in `StreamAsm` and parsed identically. Out-of-order chunks reset the buffer and wait for the next `0x00` start. Sample loss at 0.5 Hz is a 2 s gap — visible in the freshness label but never produces partial decodes.

Accepted BLE advertise names: `PumpTsueri` (legacy SDDataLogFileX) and `STBoxFs` (PumpLogger). Single source of truth is `BOX_NAMES` in `ble.rs`.

## Sync vs. transfer — `sync_db.rs`

The box firmware only speaks LIST / READ / DELETE / STOP_LOG / START_LOG — there is **no sync concept on the device**. "File transfer" (tick files → *Download selected* → serial READ queue → `save_downloaded_file`) is the old path and unchanged. "File **Sync**" (*Sync now* button) is a thin client-side layer on top:

- State lives in a local SQLite DB at `~/.movementlogger/sqlite/sync.db` (Windows `%USERPROFILE%\.movementlogger\sqlite\sync.db`) — `sync_db.rs::default_db_path`, cross-platform via `HOME`/`USERPROFILE`. `rusqlite` with the `bundled` feature so there's no system libsqlite (same self-contained rationale as `rustls-tls`). The `sqlite/` subdir is created by `SyncDb::open` via `create_dir_all`. Anchored to `$HOME`, **not** to the user-selectable "Save to" folder, so changing that folder doesn't orphan history and re-pull everything. (Changed in v0.0.7 from the flat `~/.movementlogger/sync.db`; no migration shim — v0.0.6 was brand-new so a stale flat DB just means one extra re-pull.)
- Key is `(box_id, name, size)`. `box_id` is the btleplug peripheral id, captured at Connect-click time into `AppState::ble_connected_id` (the worker's `Connected` event doesn't echo the id). `size` is in the key on purpose: a reused name with a new length counts as new.
- *Sync now* clears the list, sets `ble_sync_pending`, sends a fresh LIST; the diff runs in the `ListDone` handler **only when that flag is set** (so the auto-LIST on Connect never triggers a sync). `run_sync_diff` enqueues every `is_sensor_data_name` row not in the DB onto the existing `ble_dl_queue` and reuses `advance_download_queue` — Sync is bookkeeping + a filter, it does not reimplement transfer.
- Every successful save (`ReadDone`) records into the DB regardless of whether it came from Sync or a manual download, so a later sync skips anything already on disk.
- **Sync never issues DELETE** — additive only, by explicit product decision. Don't add a delete-after-sync path without re-confirming.

### Resume + auto-reconnect (v0.0.11)

Downloads are resumable at the byte level. The firmware READ opcode takes `<name>\0<offset:u32-LE>` and `SDFat_Seek`s there (`movement_logger_firmware/Src/ble.c` READ handler), so the desktop never restarts a file from 0 after a drop:

- Every download streams into a sibling `<name>.part`; only a complete file is renamed to its final name (`part_path` / `append_part` / `finalize_part` in `main.rs`). The `.part` length is the resume offset and the single source of truth — it survives an app restart, so a resume after a crash/quit is as lossless as one after a reconnect. `mark_synced` only fires on the finalized name.
- `advance_download_queue` derives the offset from `.part` and sends `BleCmd::Read { name, size, offset }`. A stale `.part` larger than the LIST size is treated as corrupt → deleted → restart from 0.
- On a mid-READ drop the worker emits `BleEvent::ReadAborted { name, content, base }` (from `disconnect_inner` and the watchdog timeout) carrying the partial segment; `main` appends it to `.part` so the resume continues from the true break point, not the last completed segment.
- The worker does a **bounded** auto-reconnect itself (`auto_reconnect`, `RECONNECT_ATTEMPTS`×`RECONNECT_INTERVAL` + a short rescan each round, reusing `connect_core`). Success → `Connected` (main re-LISTs, the sync diff re-queues the unfinished file, which resumes from its `.part`). Exhaustion → `Disconnected` → the existing manual-reconnect banner; still lossless via `.part`. Caveat: the worker can't process commands (incl. Disconnect) while inside `auto_reconnect` — acceptable because it's bounded (~50 s worst case).

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
