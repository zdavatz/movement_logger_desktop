# MovementLogger Desktop

Cross-platform (Windows / macOS / Linux) GUI companion for the SensorTile.box pumpfoil rig. Three tabs:

- **Live** — subscribe to the new PumpLogger firmware's SensorStream characteristic and watch real-time IMU + magnetometer + baro + GPS readings update at 0.5 Hz, plus derived pitch / roll / tilt-compensated heading in degrees and running sparklines for acc magnitude and pressure.
- **Sync** — BLE FileSync: scan, connect (PIN 123456), list SD files. Two modes: **Download selected** is a manual one-off transfer of the files you tick; **Sync now** pulls every session file (`Sens*.csv` / `*_gps.csv` / `Bat*.csv` / `Mic*.wav`) that isn't already recorded in a local SQLite DB (`~/.movementlogger/sqlite/sync.db`; Windows `%USERPROFILE%\.movementlogger\sqlite\sync.db`), so re-running it only fetches new sessions. Sync is additive — it **never deletes** anything on the box. Transfers are **resumable**: if the BLE link drops mid-file the app keeps the partial download, bounded-auto-reconnects, and continues from the exact break point (survives an app restart too — no re-downloading from scratch). Downloaded files auto-route into the Replay tab.
- **Replay** — drag-and-drop sensor CSV + optional camera `.mov`/`.mp4` + optional board `.stl`, fill in a few fields, click **Generate**. The GUI shells out to the bundled `stbox-viz animate` CLI to render the annotated session video.

Sources imported from [`fp-sns-stbox1/Utilities/rust`](https://github.com/zdavatz/fp-sns-stbox1) so this repo can ship its own release cadence.

## Supported hardware

This GUI talks to one board only:

- **STMicroelectronics STEVAL-MKBOXPRO** — *SensorTile.box PRO* (Rev_C), running the MovementLogger / PumpLogger firmware ([movement_logger_firmware](https://github.com/zdavatz/movement_logger_firmware)). MCU STM32U585AI; sensors LSM6DSV16X (IMU), LIS2MDL (mag), LPS22DF (baro), STTS22H (temp), STC3115 (fuel gauge), u-blox MAX-M10S (GPS); BLE via BlueNRG-LP.
- **Buy:** [st.com → STEVAL-MKBOXPRO](https://www.st.com/en/evaluation-tools/steval-mkboxpro.html) (the product page lists ST's authorised distributors / "Buy" options).

Older SensorTile.box variants (Rev_A/B, STWIN.box) are **not** supported by this firmware/GUI combination.

## Download

Grab the latest archive for your platform from the [Releases page](https://github.com/zdavatz/movement_logger_desktop/releases).

- **Windows** — `MovementLogger-vX.Y.Z-x86_64-pc-windows-msvc.zip`. Unzip, run `MovementLogger.exe`. (The `.exe` is unsigned; Windows SmartScreen may warn on first launch — click *More info → Run anyway*.)
- **macOS (Apple Silicon)** — `MovementLogger-X.Y.Z-macos-aarch64.dmg`. Drag the `.app` to `/Applications`. From **v0.0.2 onwards** the `.app` is Developer ID-signed, notarized and stapled, so it launches without any Gatekeeper prompt — even offline.
- **Linux x86_64** — `MovementLogger-vX.Y.Z-x86_64-unknown-linux-gnu.tar.gz`. Extract, run `./install-linux.sh` for desktop integration or `./MovementLogger` directly. Needs `libxkbcommon`, `libfontconfig`, `libGL`, `libdbus-1` on the system; `ffmpeg` on `PATH` for video output.

## In-app updates

On launch the GUI polls this repo's Releases API and, if a newer `vX.Y.Z` tag is published, offers an **Update now** button. The platform-specific upgrade:

- **macOS** — downloads the DMG, mounts it, verifies the embedded `.app` is properly codesigned, copies it next to the running one, and a tiny helper script swaps the `.app` once you quit and relaunches.
- **Windows** — downloads the zip, extracts it, and a helper batch swaps `MovementLogger.exe`.
- **Linux** — downloads the tarball, extracts, and a helper shell script swaps the binary.

> **Note for v0.0.1 users:** the in-app updater in v0.0.1 points at the wrong repository (a leftover from when this code lived inside `fp-sns-stbox1`). It will *not* find v0.0.2 automatically. Download v0.0.2 manually once; from that point on every future update is automatic.

## Build from source

```sh
cargo build --release -p movement-logger -p stbox-viz
./target/release/MovementLogger
```

Linux build deps: `libxkbcommon-dev libfontconfig1-dev libgl1-mesa-dev libdbus-1-dev`.

See [`CLAUDE.md`](CLAUDE.md) for the workspace layout, runtime topology, and release process.

## Releasing

```sh
# bump versions in */Cargo.toml, commit
git tag v0.0.2 && git push origin v0.0.2
```

The `.github/workflows/release.yml` workflow builds + packages for all three platforms and publishes a GitHub Release with the artefacts. macOS code-signing + notarization runs automatically on every tag — the Developer ID Application certificate and App Store Connect API key are already configured as repo secrets.

## License

GPL-3.0-only. See [`LICENSE`](LICENSE).
