# MovementLogger Desktop

Cross-platform (Windows / macOS / Linux) drag-and-drop GUI for SensorTile.box pumpfoil session telemetry. Drop a sensor CSV (auto-pairs the matching `_gps.csv`), an optional camera `.mov`/`.mp4`, and an optional board `.stl`; fill in a few optional fields; click **Generate**. The GUI shells out to the bundled `stbox-viz animate` CLI to render the annotated session video.

Sources imported from [`fp-sns-stbox1/Utilities/rust`](https://github.com/zdavatz/fp-sns-stbox1) so this repo can ship its own release cadence.

## Download

Grab the latest archive for your platform from the [Releases page](https://github.com/zdavatz/movement_logger_desktop/releases).

- **Windows** — `MovementLogger-vX.Y.Z-x86_64-pc-windows-msvc.zip`. Unzip, run `MovementLogger.exe`.
- **macOS (Apple Silicon)** — `MovementLogger-vX.Y.Z-macos-aarch64.dmg`. Drag the `.app` to `/Applications`. Unsigned builds need right-click → Open the first time.
- **Linux x86_64** — `MovementLogger-vX.Y.Z-x86_64-unknown-linux-gnu.tar.gz`. Extract, run `./install-linux.sh` for desktop integration or `./MovementLogger` directly. Needs `libxkbcommon`, `libfontconfig`, `libGL`, `libdbus-1` on the system; `ffmpeg` on `PATH` for video output.

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

The `.github/workflows/release.yml` workflow builds + packages for all three platforms and publishes a GitHub Release with the artefacts. macOS code-signing + notarization activates automatically when the Developer ID + App Store Connect API key secrets are present.

## License

GPL-3.0-only. See [`LICENSE`](LICENSE).
