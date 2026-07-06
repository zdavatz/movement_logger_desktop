# MovementLogger Desktop

Cross-platform (Windows / macOS / Linux) GUI companion for the SensorTile.box pumpfoil rig. Four tabs:

- **Live** — subscribe to the new PumpLogger firmware's SensorStream characteristic and watch real-time IMU + magnetometer + baro + GPS readings update at 0.5 Hz, a **Board angles** readout — **Pitch** (up / down hill), **Roll** (lean L / R) and **Yaw** (heading) about the box's physical axes (nose = Y), taken from the gyro+accel orientation filter rather than the raw accel formulas — and running sparklines for acc magnitude and pressure. The board angles come in two flavours: **Absolute** (vs level & north) and **Calibrated** — tap *Zero here* with the board in its reference pose to tare pitch / roll / yaw to zero, *Clear* to reset; the tare is remembered across launches.
- **Sync** — BLE FileSync: scan, connect (PIN 123456), list SD files. Two modes: **Download selected** is a manual one-off transfer of the files you tick; **Sync now** pulls every session file (`Sens*.csv` / `*_gps.csv` / `Bat*.csv` / `Mic*.wav`) plus the box's rolling error log `ERRLOG.LOG` that isn't already recorded in a local SQLite DB (`~/.movementlogger/sqlite/sync.db`; Windows `%USERPROFILE%\.movementlogger\sqlite\sync.db`), so re-running it only fetches new sessions. Sync is additive — it **never deletes** anything on the box. The local file is a live mirror: each pass fetches only the bytes that grew since last time, so a continuously-growing log isn't re-downloaded; if the BLE link drops the app keeps what it has, bounded-auto-reconnects, and continues from the exact break point (survives an app restart too). Tick **Keep synced** to stay connected and re-mirror every 30 s without clicking again; in this mode the app also auto-reconnects *indefinitely* (exponential backoff) across any number of link drops, so you can leave it running and a multi-hour sync finishes on its own even if the box drops out and self-heals dozens of times. While a pass runs, a cumulative progress bar above the file list shows "N of M files · X / Y MB · Z %" so the whole pass is visible at a glance (per-row bars below still show individual file progress). With **Keep synced** off, a dropped link retries a bounded number of times and then waits for a manual reconnect (still lossless — the mirror holds the resume point), and Disconnect always cancels promptly. Downloaded files auto-route into the Replay tab. A **Log mode** selector (Auto / Manual) sets the box's own recording mode (persisted on the box, like the iOS/Android apps): **Auto** = the box opens a session automatically on every power-on; **Manual** = the box stays idle until you press *Start session*. Next to it a **GPS: On / Off** control turns the box's u-blox receiver off to save battery (~tens of µA vs ~25 mA) when it's faulty or unused — IMU + baro keep logging, and Replay still time-aligns from the `# SYNC` anchor (you lose the speed + GPS-track panels but keep pitch / roll / height). Persisted on the box; needs box firmware v0.0.35+. When the box is in **Auto**, MovementLogger also installs a tiny background **sync agent** that starts at login and mirrors the box with **zero clicks** — no window needed. It automatically steps aside whenever you open the app (the GUI always wins the Bluetooth radio) and resumes when you quit; switching the box to **Manual** removes the login item again.
- **Replay** — drag-and-drop sensor CSV + optional camera `.mov`/`.mp4` + optional board `.stl`, fill in a few fields, click **Generate**. The GUI shells out to the bundled `stbox-viz animate` CLI to render the annotated session video. Use the **Von** / **Bis** time fields (HH:MM:SS, local) to render exactly one wall-clock slice of the session — the GUI passes that window's start as `--at` and its length (Bis − Von) as `--duration`. Leave **Bis** blank to fall back to the **Duration** field / video length. The box has no RTC, so on every connect the app now stamps the host's wall clock into the box's open logs (`SET_TIME` `0x08`, firmware v0.0.10+); when those `# SYNC` anchors are present the renderer aligns the window to absolute time straight from them — **drift-free and with no GPS fix required** — instead of leaning on the GPS UTC strings (which stay as the fallback for older recordings).

- **GPS Debug** — live u-blox UBX diagnostics for antenna selection + mounting evaluation: fix quality (DOP, accuracy), per-signal C/N0, and RF/antenna health (antStatus, AGC, jamming), polled once a second into CSVs. Read-only — it never reconfigures the receiver. Two transports: **USB serial** (🔄 **Detect** auto-fills the port — macOS `/dev/cu.usb*`, Linux `ttyACM*`/`ttyUSB*` — or type it), or **BLE** (no cable, no opening the box: connect to the box in the Sync tab first, then the survey tunnels the u-blox over the box's link — needs box firmware with the GPS-bridge support).

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

## Updating the box firmware (FOTA)

MovementLogger can flash new SensorTile.box firmware **over BLE** — no cable, no STM32 programmer. The image is written to the box's *inactive* flash bank and only swapped in once it's fully verified, so the box stays bootable on its current firmware throughout and an interrupted upload is harmless.

You can do it two ways:

- **From the GUI** — connect to the box in the **Sync** tab, then use the firmware-upload button and pick the `.bin`.
- **Headless from the command line:**

  ```sh
  MovementLogger --flash-firmware /path/to/firmware.bin
  ```

  No window opens. It connects to the box remembered in `~/.movementlogger/config.toml` (so the box must have been connected once from the GUI first — on macOS this is also what records the Bluetooth permission the headless run inherits), uploads the image, and prints progress to stderr. Exit code `0` = sent (box reboots into the new firmware), `1` = error/timeout, `2` = file missing.

> **It's slow — that's normal.** BLE OTA is stop-and-wait (160-byte chunks, each waiting for the box to program it to flash and acknowledge before the next), so a ~100 kB image takes **roughly 20 minutes**. Leave it running; the box only switches to the new firmware after the whole image is uploaded and its SHA-256 checks out.

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
