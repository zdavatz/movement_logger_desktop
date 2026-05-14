# stbox-viz

Rust replacement for the Python pumpfoil-session visualisers under
`Utilities/scripts/`. Reads a sensor CSV + auto-detected GPS CSV from an
SD-card recording, runs Madgwick 6DOF fusion, detects rides from sustained
GPS movement, computes board height above water from the temperature-
compensated + GPS-anchored barometer, and emits an interactive Plotly HTML.

## Build

```sh
cd Utilities/rust/stbox-viz
cargo build --release
```

Binary lands at `target/release/stbox-viz`.

## Usage

```sh
./target/release/stbox-viz combined /Volumes/SDCARD/Sens000.csv -o html/
```

The GPS CSV is auto-detected next to the sensor CSV. Two naming forms
are accepted:

- `SensNNN.csv` ↔ `GpsNNN.csv` — the firmware's on-card layout (preferred)
- `<stem>.csv` ↔ `<stem>_gps.csv` — legacy form for renamed/exported CSVs

Output filename is `viz_<stem>.html`.

### Time-axis options (`combined`)

By default the x-axis runs in UTC anchored to the GPS clock. Use:

- `--tz-offset-h <h>` — shift x-axis to local time. `3` for Ermioni / Greek
  summer (EEST), `2` for Swiss summer (CEST), `1` for Swiss winter (CET).
  Axis title and ride-list times reflect the chosen offset.
- `--date YYYY-MM-DD` — override the recording date. Defaults to the
  sensor file's mtime; pass this if `cp`/`mv` reset the mtime to today.

Time-of-day always reads off the GPS clock — the `--date` flag only
affects the rendered date string in datetime tooltips.

Plotly.js is loaded from the CDN (same as the Python version's
`include_plotlyjs='cdn'`), so the HTML needs internet the first time it's
opened in a browser.

### Wall-clock-aligned animation (`animate --at`)

The default `animate` subcommand finds pumping sessions by pitch
oscillation and renders one GIF per session. To instead pin the
animation to an exact wall-clock time (e.g. to align with external
camera footage), use `--at`:

```sh
./target/release/stbox-viz animate ../../../csv/ayano_25.4.2026_s0.csv \
    --at 10:16 --tz-offset-h 3 --date 2026-04-25 \
    --video /path/to/IMG_1851.MOV \
    --title "Ayano · Ermioni · 25.4.2026" \
    --subtitle "Ride 8 · 10:16 EEST"
```

Flags:

- `--at HH:MM[:SS]` — wall-clock start of the GIF window (in local time
  per `--tz-offset-h`). Bypasses pitch-oscillation session detection.
- `--duration <s>` — window length. Defaults to the video's full length
  when `--video` is given, else 60 s.
- `--video <path>` — overlay the camera video next to the GIF via ffmpeg
  hstack. The video's `creation_time` is probed and printed; pass that
  value back as `--at HH:MM:SS` for millisecond-precise alignment.
- `--auto-skip` — advance both video and GIF past the carry/transition
  seconds before sustained pitch oscillation begins. Off by default.
- `--dock-height-m <h>` — height of the launch dock above water in
  metres (e.g. `0.75` for the Ermioni harbour wall). When set, the
  carry phase is plotted as a flat constant at this height, the
  push-off transition crosses 0 m via a 2-sec linear ramp, and the
  foiling phase uses baro re-anchored at the detected push-off
  moment, clamped to the physical [-0.1, 0.9 m] range and outlier-
  rejected (Δp > 1 hPa per sample → NaN gap). Recovers physically-
  correct heights for dock-launched sessions where the bare baro
  algorithm would otherwise treat the dock itself as water
  reference. Default 0 (raw baro).
- `--board-stl <path>` — render a 3D-rotated hydrofoil mesh in the
  side-view panel instead of the 2D line. Canonical model is
  `~/software/fingerfoil/stl/0_combined.stl`. Pure-Rust software
  rasterizer, no GPU.
- `--mount mast|deck` — where the SensorTile.box is physically
  attached on the board. `mast` (default) — strapped to the mast
  with chip +X pointing along the mast (Ayano's Ermioni sessions);
  uses Madgwick `quat_strip_yaw(quat_conj(q))` with `R_mount =
  (-z, x, y)` and a port-side camera. `deck` — on top of the deck,
  long axis nose-tail, chip +Z pointing down through the deck
  (Peter's 28.4.2026 session); uses accel-only tilt with a 180°-X
  pre-flip, `R_mount = identity`, and a tail-camera. Wrong mount
  flag renders the foil on its side.
- `--tz-offset-h`, `--date` — same semantics as `combined`.

The GIF has 6 panels when GPS is available (top row: board side view
+ GPS-track map; below: pump detail, height-over-water, speed,
nasenwinkel), 3 otherwise. Each panel's y-axis grows with the running
max — no wasted vertical space above the data. Phase backgrounds
(gray/yellow/green) shade the time-series panels for Tragen /
Anschieben/Rennen / Foilen so you see at a glance when the rider
pushed off and got on the foil. Push-off is detected by a 1-sec
rolling mean of raw position-derived speed crossing 4 km/h
sustainedly for 0.5 s — short lag (sub-1-sec) compared to the 5-sec
rolling median that drives the speed panel. The push-off-Winkel
flash for 2 s shows the steepest negative pitch in a ±1 s window
around that moment.

The at-window slicing is anchored by the GPS row whose UTC matches
the requested `--at` time, not by linear extrapolation from the
first GPS sample of the session. ThreadX runs on HSI (±1 % accuracy)
and drifts ~7 s over a 21-min session — extrapolation would slice
the wrong sensor range. Frame generation uses float-step indexing so
the encoded GIF length exactly matches the data window length, which
matters when paired with a `--video` of the same wall-clock length.

The board side view scrolls a sinusoidal waterline backward at 0.6
m/s so the board appears to be travelling forward over the water
while it pumps; lift comes from the smoothed baro height. Panel
titles are horizontal (their own 40-px title strip above each
chart). The Nasenwinkel panel uses a 95th-percentile-based y-limit
so single-outlier pumps don't stretch the axis.

Each time-series panel (Pump-Detail, Höhe, Geschwindigkeit,
Nasenwinkel) draws a red dot at the cursor's data point and a red
value label (`±X.X°` / `X.XX m` / `X.X km/h`) just to the right of
the needle so the live value is readable without scanning the
y-axis — useful when the GIF runs paired with camera footage.

## Why Rust

- Single binary vs Python + venv + pandas + plotly + numpy + scipy
- Strong typing at the CSV boundary — the 22.4.2026 `Time [mS]` → `Time
  [10ms]` rename broke both Python scripts; `serde`-strict column parsing
  would have flagged it at compile time
- Faster fusion + rolling-median on long sessions (~10× on 2 h recordings)
- Matches the rest of the `~software/` stack, which is mostly Rust

## Scope

Phase 1 (this version): feature-parity with `visualize_combined.py` minus
the checkbox ride isolation and URL anchors. Future phases will port
`visualize_sensors`, `visualize_pumpfoil`, `animate_board_3d` and then
delete the Python under `Utilities/scripts/`.
