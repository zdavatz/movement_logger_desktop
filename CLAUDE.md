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
