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

BLE FileSync (`stbox-viz-gui/src/ble.rs`) runs its own tokio current-thread runtime on a worker thread; commands/events shuttle across `std::sync::mpsc` so the egui UI stays sync. Single notification stream demuxed by a `select!` loop so notifies never get lost between LIST / READ / DELETE op boundaries.

## Releases

Push a `vX.Y.Z` tag — `.github/workflows/release.yml` builds Linux x86_64, macOS arm64 (Apple Silicon) and Windows x86_64, packages each as `MovementLogger-vX.Y.Z-<triple>.{tar.gz,zip}`, builds a macOS `.app` + `.dmg`, and attaches everything to the auto-created GitHub Release.

macOS Developer ID signing + notarization is gated on these repo secrets — set them to get signed/notarized output, otherwise the .app/DMG ship unsigned and Gatekeeper needs right-click → Open on first launch:

- `MACOS_DEVELOPER_ID_CERTIFICATE` / `MACOS_DEVELOPER_ID_CERTIFICATE_PASSWORD`
- `APPLE_API_KEY_P8` / `APPLE_API_KEY_ID` / `APPLE_API_ISSUER_ID`

Mac App Store `.pkg` and Microsoft Store MSIX paths from upstream `fp-sns-stbox1` are intentionally omitted from this repo. Port them from `fp-sns-stbox1/.github/workflows/release.yml` if/when needed.

## Upstream

When pulling fixes from `fp-sns-stbox1/Utilities/rust`, the workspace `Cargo.toml`s here drop the `Utilities/rust` working-directory prefix and use repo-relative paths. Watch for that when copying steps.
