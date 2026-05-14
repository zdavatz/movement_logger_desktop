//! `stbox-viz animate` — animated GIF (and optional combined MOV) of
//! board side-view tilting with nose-angle + pump-detail panels.
//! Port of `Utilities/scripts/animate_board_3d.py`.
//!
//! 3-panel layout:
//!   - Top: side view of the board tilting + translating vertically
//!   - Middle: pump detail at ±5°
//!   - Bottom: full-range nose angle with cursor
//!
//! Butterworth 2 Hz low-pass (4th-order, zero-phase) + 10 s rolling-
//! median baseline before animating. Progressive line build-up: history
//! line grows with the cursor, no future data shown.
//!
//! With `--video <path>`, pipes the resulting GIF through ffmpeg into a
//! side-by-side MOV next to camera footage.

use crate::baro;
use crate::board3d;
use crate::butter::{butter4_lowpass, filtfilt};
use crate::euler::quats_to_euler_deg;
use crate::fusion;
use crate::gps as gps_mod;
use crate::io::{load_gps_csv, load_sensor_csv};
use crate::plot_common::FONT;
use crate::session::detect_sessions;
use anyhow::{Context, Result, anyhow};
use chrono::{Duration as ChronoDuration, NaiveDate, NaiveDateTime, NaiveTime, Timelike, Utc, DateTime};
use plotters::prelude::*;
use std::path::{Path, PathBuf};
use std::process::Command;

pub struct AnimateArgs<'a> {
    pub sensor_csv: &'a Path,
    pub output_dir: &'a Path,
    pub fps: u32,
    pub session: Option<usize>,
    pub video: Option<&'a Path>,
    pub video_offset: f64,
    pub sensor_offset: f64,
    pub title: Option<&'a str>,
    pub subtitle: Option<&'a str>,
    /// Wall-clock start (HH:MM[:SS]) in local time. When set, bypasses
    /// session detection and renders one GIF for that exact window.
    pub at: Option<&'a str>,
    /// Window length in seconds when `at` is set; defaults to the
    /// video's duration if `video` is given, else 60 s.
    pub duration: Option<f64>,
    pub tz_offset_h: f64,
    pub date: Option<&'a str>,
    /// In `--at` mode, skip carry/transition seconds at the start until
    /// sustained pitch oscillation begins. Off by default.
    pub auto_skip: bool,
    /// Dock height above water in metres. Added to all baro-derived
    /// heights so that dock-anchored windows display dock = +dock_h,
    /// water = 0, foiling = actual lift. 0 by default.
    pub dock_height_m: f64,
    /// Optional binary-STL of the board mesh. When provided, the side-
    /// view panel renders a 3D-rotated mesh instead of the 2D line.
    pub board_stl: Option<&'a std::path::Path>,
    /// Where the IMU is physically mounted on the board. Picks
    /// `R_mount`, the camera angle, and the accel-only-tilt
    /// pre-flip path. Default is mast.
    pub mount: board3d::MountKind,
}

pub fn run(args: &AnimateArgs) -> Result<()> {
    std::fs::create_dir_all(args.output_dir)
        .with_context(|| format!("mkdir {}", args.output_dir.display()))?;

    println!("loading {}", args.sensor_csv.file_name().unwrap().to_string_lossy());
    let samples = load_sensor_csv(args.sensor_csv)?;
    if samples.is_empty() {
        anyhow::bail!("sensor CSV is empty");
    }
    let base = args.sensor_csv.file_stem().unwrap().to_string_lossy().to_string();

    println!("running Madgwick 6DOF fusion");
    let quats = fusion::compute_quaternions(&samples, 0.1);
    let (roll, pitch, _yaw) = quats_to_euler_deg(&quats);

    let sample_hz: usize = 100;

    // Optional 3D-board-mesh load. Only used in the side-view panel.
    // We keep it Option<board3d::Mesh> so the line-fallback path
    // works unchanged for users who don't pass --board-stl.
    let board_mesh: Option<board3d::Mesh> = if let Some(stl) = args.board_stl {
        println!("loading board STL {} (3D side-view enabled, mount={:?})",
                 stl.display(), args.mount);
        let m = board3d::Mesh::load_binary_stl(stl, args.mount)?;
        println!("  {} triangles", m.tris.len());
        Some(m)
    } else {
        None
    };

    // Nose angle for animation: 0.7 Hz Butterworth + 10 s baseline.
    // Pump frequency on this hardware is ~0.5 Hz. With a 0.7 Hz cutoff
    // the pump fundamental sits well inside the passband (~5 % loss)
    // while 1+ Hz "shake the board" wiggle is sharply attenuated.
    let nose_raw = nose_angle_deg_raw(&quats);
    let (b, a) = butter4_lowpass(0.7, sample_hz as f64);
    let nose_smooth = filtfilt(&b, &a, &nose_raw);
    let baseline = rolling_median(&nose_smooth, 10 * sample_hz);
    let nose_corrected: Vec<f64> = nose_smooth.iter().zip(baseline.iter())
        .map(|(s, b)| s - b).collect();


    // GPS-derived height + speed (sample-aligned with sensors), plus
    // raw lat/lon/time for the map panel. Loaded once for the whole
    // session; the --at mode slices it later. Fails gracefully if no
    // GPS CSV is next to the sensor file.
    let base_ticks_session = samples[0].ticks;
    let (height_full, speed_at_sensor, gps_lat_full, gps_lon_full, gps_t_full,
         yaw_full_rad):
        (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f32>) = {
        let gps_path = guess_gps_path(args.sensor_csv);
        if let Some(gp) = gps_path {
            let mut gps_rows = load_gps_csv(&gp)?;
            gps_rows.retain(|g| g.fix >= 1);
            // Same speed pipeline as `combined`: position-derived speed,
            // multipath-rejection at 4 m/s² (≈ 15 km/h/s), 5 s rolling
            // median.
            let raw = gps_mod::position_derived_speed_kmh(&gps_rows);
            let gated = gps_mod::reject_acc_outliers(&gps_rows, &raw, 15.0);
            let smooth = gps_mod::smooth_speed_kmh(&gated);
            let height_raw = baro::height_above_water_m(
                &samples, &gps_rows, &smooth, base_ticks_session);
            // Smooth the baro height for visualization. The raw signal
            // has many short pressure-noise spikes (visible as jagged
            // peaks in the height panel and, more importantly, as
            // jitter in the board-view lift). 0.5-sec centred rolling
            // mean kills the jitter while preserving the slower
            // foiling-height changes (mast travel is at most ~0.8 m,
            // pumps are 0.3–0.6 Hz, well-preserved by a 1 Hz-equivalent
            // smoother).
            let height: Vec<f64> = {
                let win = sample_hz / 2;
                let half = win / 2;
                let n = height_raw.len();
                let mut csum = vec![0.0; n + 1];
                let mut ccnt = vec![0usize; n + 1];
                for i in 0..n {
                    let v = height_raw[i];
                    csum[i + 1] = csum[i] + if v.is_finite() { v } else { 0.0 };
                    ccnt[i + 1] = ccnt[i] + if v.is_finite() { 1 } else { 0 };
                }
                let mut out = vec![0.0; n];
                for i in 0..n {
                    let lo = i.saturating_sub(half);
                    let hi = (i + half + 1).min(n);
                    let cnt = ccnt[hi] - ccnt[lo];
                    out[i] = if cnt > 0 {
                        (csum[hi] - csum[lo]) / cnt as f64
                    } else {
                        f64::NAN
                    };
                }
                out
            };
            // Interp GPS speed onto the sensor time grid so the speed
            // panel can use a per-sensor-sample cursor.
            let gps_t_s: Vec<f64> = gps_rows.iter()
                .map(|g| (g.ticks - base_ticks_session) / gps_mod::TICKS_PER_SEC)
                .collect();
            let sensor_t_s: Vec<f64> = samples.iter()
                .map(|s| (s.ticks - base_ticks_session) / gps_mod::TICKS_PER_SEC)
                .collect();
            let speed_aligned = baro::interp_linear(&sensor_t_s, &gps_t_s, &smooth);
            let lats: Vec<f64> = gps_rows.iter().map(|g| g.lat).collect();
            let lons: Vec<f64> = gps_rows.iter().map(|g| g.lon).collect();
            // Note: height re-anchoring (water_set_t-based) happens
            // later inside the --at branch, after phase detection.
            // The legacy session-detection path can use --dock-height-m
            // as a manual override.
            let height_offset: Vec<f64> = if args.dock_height_m != 0.0 {
                height.iter().map(|h| h + args.dock_height_m).collect()
            } else { height };
            // Per-sample yaw from GPS course-over-ground when speed
            // > 4 km/h (Ayano's data: pumping starts at ~4 km/h, foiling
            // sustains 13–20 km/h). Below threshold, hold the last
            // good value — board doesn't significantly rotate while
            // it's being carried/parked. Only used when --board-stl
            // is given.
            let courses: Vec<f64> = gps_rows.iter().map(|g| g.course_deg).collect();
            let yaw_rad = board3d::yaw_from_gps(
                &sensor_t_s, &gps_t_s, &courses, &smooth, 4.0);
            (height_offset, speed_aligned, lats, lons, gps_t_s, yaw_rad)
        } else {
            (Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new())
        }
    };

    // Pump count per sample, gated on GPS speed > 4 km/h so carry/
    // walking/dock impacts don't inflate it. Cumulative count.
    let pump_count_full: Vec<u32> = {
        let n = samples.len();
        if n == 0 { Vec::new() } else {
            let mag: Vec<f64> = samples.iter()
                .map(|s| (s.acc[0]*s.acc[0] + s.acc[1]*s.acc[1] + s.acc[2]*s.acc[2]).sqrt())
                .collect();
            let half = sample_hz / 2;
            let mut csum = vec![0.0; n + 1];
            for i in 0..n { csum[i + 1] = csum[i] + mag[i]; }
            let dyn_acc: Vec<f64> = (0..n).map(|i| {
                let lo = i.saturating_sub(half);
                let hi = (i + half + 1).min(n);
                mag[i] - (csum[hi] - csum[lo]) / (hi - lo) as f64
            }).collect();
            let thresh = 80.0;
            let refractory: i64 = (sample_hz * 3 / 10) as i64;
            let mut count = 0u32;
            let mut last_peak: i64 = -1_000_000;
            let mut out = vec![0u32; n];
            for i in 1..n.saturating_sub(1) {
                let on_water = !speed_at_sensor.is_empty()
                    && speed_at_sensor[i] > 4.0;
                if on_water
                    && dyn_acc[i] > thresh
                    && dyn_acc[i] > dyn_acc[i-1]
                    && dyn_acc[i] >= dyn_acc[i+1]
                    && (i as i64 - last_peak) > refractory
                {
                    count += 1;
                    last_peak = i as i64;
                }
                out[i] = count;
            }
            if n > 0 { out[n - 1] = count; }
            out
        }
    };

    // 3D-board calibration via a single reference quaternion.
    //
    // We pick `q_mount` from a stable foiling moment (GPS speed >
    // 10 km/h sustained) and treat that quaternion as "the IMU's
    // orientation when the board is level". For every frame we
    // compute `q_rel = conj(q_mount) * q_now` — this is the actual
    // 3-axis rotation of the board *relative to its level pose*,
    // independent of any IMU-mount-axis confusion.
    //
    // Earlier attempts decomposed the quaternion into Tait-Bryan
    // pitch/roll/yaw or into per-axis Z-components, but those all
    // require correct assumptions about IMU axis labels — which
    // turn out to be wrong on this hardware (port_offset = 77°
    // proved the IMU is mounted with its sensor axes far from where
    // we expected). Using the full quaternion bypasses the axis-
    // label question entirely.
    let q_mount: Option<[f64; 4]> = {
        let foil_idxs: Vec<usize> = if !speed_at_sensor.is_empty() {
            speed_at_sensor.iter().enumerate()
                .filter(|(_, s)| **s > 10.0).map(|(i, _)| i).collect()
        } else { Vec::new() };
        if foil_idxs.len() >= 100 {
            // Average quaternion over the foiling phase. Single-
            // sample calibration was sensitive to whatever instan-
            // taneous roll/pitch the rider had at that moment (e.g.
            // a slight bank during a carve), which polluted the
            // "level" reference and surfaced as phantom lateral
            // motion during pumping. Averaging across many samples
            // washes the per-stroke variation out.
            //
            // Quaternions live on the double-cover SO(3) so q and
            // -q represent the same rotation; before summing we
            // flip the sign of any sample whose dot-product with
            // the running average is negative. With samples already
            // close in orientation (all foiling) this aligns them
            // to the same hemisphere, so direct component-wise
            // averaging then renormalising yields a clean mean.
            let first = quats[foil_idxs[0]];
            let mut sum = [0.0f64; 4];
            for &i in &foil_idxs {
                let q = quats[i];
                let dot = first[0]*q[0] + first[1]*q[1] + first[2]*q[2] + first[3]*q[3];
                let s = if dot >= 0.0 { 1.0 } else { -1.0 };
                sum[0] += s * q[0]; sum[1] += s * q[1];
                sum[2] += s * q[2]; sum[3] += s * q[3];
            }
            let n = (sum[0]*sum[0] + sum[1]*sum[1] + sum[2]*sum[2] + sum[3]*sum[3]).sqrt();
            let q_avg = if n > 1e-9 {
                [sum[0]/n, sum[1]/n, sum[2]/n, sum[3]/n]
            } else {
                quats[foil_idxs[foil_idxs.len() / 2]]
            };
            println!("  3D calibration: q_mount = mean of {} foiling samples = \
                      [{:.3}, {:.3}, {:.3}, {:.3}]",
                     foil_idxs.len(), q_avg[0], q_avg[1], q_avg[2], q_avg[3]);
            Some(q_avg)
        } else {
            println!("  3D calibration: no foiling phase (<100 samples > 10 km/h), \
                      using identity — board appears at IMU's raw orientation");
            None
        }
    };
    let q_mount_conj = q_mount.map(|q| board3d::quat_conj(&q));

    // --at mode: bypass session detection and render exactly one GIF
    // covering [at, at + duration). The window is computed from the
    // GPS-anchored UTC clock so it lines up with the same wall-clock
    // axis as `combined`.
    if let Some(at_str) = args.at {
        let (mut at_s, at_e, mut start_local) = resolve_at_window(
            args.sensor_csv, &samples, sample_hz, at_str, args.duration,
            args.video, args.tz_offset_h, args.date,
        )?;
        if at_e <= at_s {
            anyhow::bail!("--at window resolves to zero/negative length");
        }

        // Auto-detect water-entry: skip the carry/transition seconds
        // before sustained pitch oscillation begins. While the rider
        // walks the board to the water, pitch flails irregularly; once
        // pumping starts it oscillates at ~0.3–1 Hz with amplitude
        // > 5°. Find the first 3-sec window inside [at_s, at_e) with
        // ≥ 2 zero-crossings of the detrended pitch and amplitude
        // > 5°. If found, advance both the GIF start and (when paired
        // with --video) the video by the same number of seconds, so
        // the side-by-side stays in sync. The carry portion is just
        // dropped from the output.
        let mut video_offset = args.video_offset;
        if args.auto_skip {
            let pitch_at = &pitch[at_s..at_e];
            if let Some(skip) = detect_water_entry(pitch_at, sample_hz) {
                let skip_s = skip as f64 / sample_hz as f64;
                if skip_s >= 0.5 {
                    println!("  auto-skip: {:.1} s carry/transition before pumping",
                             skip_s);
                    at_s += skip;
                    start_local = start_local
                        + ChronoDuration::milliseconds((skip_s * 1000.0).round() as i64);
                    video_offset += skip_s;
                }
            }
        }

        let dur = (at_e - at_s) as f64 / sample_hz as f64;
        println!("  --at window: {:.1} s ({} samples), start {} local",
            dur, at_e - at_s, start_local.format("%H:%M:%S"));
        let label = at_str.replace(':', "");
        let gif_path = args.output_dir.join(
            format!("anim_board_{}_at{}.gif", base, label));
        let height_slice: Option<&[f64]> = if !height_full.is_empty() {
            Some(&height_full[at_s..at_e])
        } else { None };
        let speed_slice: Option<&[f64]> = if !speed_at_sensor.is_empty() {
            Some(&speed_at_sensor[at_s..at_e])
        } else { None };
        // GPS rows that fall within the at-window time range.
        let win_t0 = at_s as f64 / sample_hz as f64;
        let win_t1 = at_e as f64 / sample_hz as f64;
        let gps_indices: Vec<usize> = gps_t_full.iter().enumerate()
            .filter(|(_, t)| **t >= win_t0 && **t < win_t1)
            .map(|(i, _)| i).collect();
        let gps_lat_w: Vec<f64> = gps_indices.iter().map(|&i| gps_lat_full[i]).collect();
        let gps_lon_w: Vec<f64> = gps_indices.iter().map(|&i| gps_lon_full[i]).collect();
        let gps_t_w: Vec<f64> = gps_indices.iter()
            .map(|&i| gps_t_full[i] - win_t0).collect();
        let map_data = if !gps_lat_w.is_empty() {
            Some((gps_lat_w.as_slice(), gps_lon_w.as_slice(), gps_t_w.as_slice()))
        } else { None };

        // Phase detection inside the window (seconds since window start):
        //   "Tragen"  — GPS speed < 1 km/h sustained (stationary on
        //               dock / shore), regardless of board pitch
        //   "Anschieben/Rennen" — speed has risen above 2 km/h but
        //               height is still < 0.15 m (board on water,
        //               paddling/running, not yet on the foil)
        //   "Foilen"  — 3-sec mean height ≥ 0.15 m (sustained foil
        //               lift, not just a single pump peak)
        //
        // GPS speed is the right discriminator for "in water" vs
        // "on land": pitch alone is unreliable because the rider
        // holds the board at varying angles on the dock. Speed jumps
        // from ~0 to a few km/h the moment they actually push off.
        let (water_set_t, foil_start_t): (Option<f64>, Option<f64>) = {
            // Push-off detection: first index where raw position-
            // derived speed (no smoothing!) is ≥ 3 km/h sustained
            // for 0.5 s. Raw speed reacts in < 100 ms when the
            // rider actually leaves the dock; the previous 5 m
            // displacement threshold + smoothed-speed pipeline
            // added ~1-2 s of lag, so the push-off flash triggered
            // after she was already on the foil instead of at
            // feet-on-board.
            let wset = if !gps_lat_full.is_empty() && !gps_t_full.is_empty() {
                let win_t0 = at_s as f64 / sample_hz as f64;
                let win_t1 = at_e as f64 / sample_hz as f64;
                let in_win: Vec<usize> = gps_t_full.iter().enumerate()
                    .filter(|(_, t)| **t >= win_t0 && **t < win_t1)
                    .map(|(i, _)| i).collect();
                if in_win.len() < 12 {
                    None
                } else {
                    let mut raw_kmh: Vec<f64> = vec![0.0; in_win.len()];
                    for k in 1..in_win.len() {
                        let i_prev = in_win[k - 1];
                        let i_curr = in_win[k];
                        let dt = gps_t_full[i_curr] - gps_t_full[i_prev];
                        if dt <= 0.0 { continue; }
                        let lat0 = gps_lat_full[i_prev];
                        let cos_lat = (lat0 * std::f64::consts::PI / 180.0).cos();
                        let dlat = gps_lat_full[i_curr] - gps_lat_full[i_prev];
                        let dlon = (gps_lon_full[i_curr] - gps_lon_full[i_prev]) * cos_lat;
                        let dist_m = ((dlat * dlat + dlon * dlon).sqrt() * 111_000.0).abs();
                        raw_kmh[k] = (dist_m / dt) * 3.6;
                    }
                    // 1-sec rolling mean (≈10 GPS samples) suppresses
                    // multipath jitter (which can spike raw speed to
                    // 5-15 km/h even when stationary) while keeping
                    // much shorter lag than the 5-sec rolling median
                    // used in `speed_at_sensor` (which lags ~2.5 s).
                    let win_g = 10usize;
                    let half_g = win_g / 2;
                    let n = raw_kmh.len();
                    let mut csum = vec![0.0; n + 1];
                    for k in 0..n { csum[k + 1] = csum[k] + raw_kmh[k]; }
                    let smooth_kmh: Vec<f64> = (0..n).map(|k| {
                        let lo = k.saturating_sub(half_g);
                        let hi = (k + half_g + 1).min(n);
                        (csum[hi] - csum[lo]) / (hi - lo) as f64
                    }).collect();
                    // 4 km/h threshold on the 1-sec mean: above the
                    // ~2 km/h GPS noise floor, well below the 6-10 km/h
                    // a moving rider has.
                    let consec = 5usize;
                    let mut found: Option<usize> = None;
                    for k in 0..smooth_kmh.len().saturating_sub(consec) {
                        if smooth_kmh[k..k + consec].iter().all(|&s| s >= 4.0) {
                            found = Some(k); break;
                        }
                    }
                    found.map(|k| {
                        let t_rel = gps_t_full[in_win[k]] - win_t0;
                        (t_rel * sample_hz as f64).round() as usize
                    })
                }
            } else { None };
            let wset_idx = wset.unwrap_or(0);
            // First index after wset_idx where the 3-sec mean height
            // exceeds 0.15 m → sustainedly on the foil.
            let foil_win = 3 * sample_hz;
            let foil = if !height_full.is_empty()
                && height_full.len() >= at_s + foil_win {
                let h_w = &height_full[at_s..at_e];
                let mut found: Option<usize> = None;
                if h_w.len() > foil_win {
                    for i in wset_idx..h_w.len().saturating_sub(foil_win) {
                        let mut sum = 0.0; let mut cnt = 0;
                        for v in &h_w[i..i + foil_win] {
                            if v.is_finite() { sum += *v; cnt += 1; }
                        }
                        if cnt > 0 && sum / cnt as f64 > 0.15 {
                            found = Some(i); break;
                        }
                    }
                }
                found.map(|i| i as f64 / sample_hz as f64)
            } else { None };
            (wset.map(|i| i as f64 / sample_hz as f64), foil)
        };
        if let Some(t) = water_set_t {
            println!("  phase: Push-off ab +{:.1} s (raw Speed ≥ 3 km/h, 0.5 s sustained)", t);
        } else {
            println!("  phase: Push-off — kein Übergang im Fenster gefunden");
        }
        if let Some(t) = foil_start_t {
            println!("  phase: Foilen ab +{:.1} s (3-Sek-Mean Höhe > 0.15 m)", t);
        }

        // Hybrid height construction. Baro alone is too thermal-drift-
        // prone over a 39-sec window to give a meaningful absolute
        // dock-period altitude (we observed the trace crashing to
        // −30 m at the end of a window because of accumulating
        // pressure noise). What the user actually wants:
        //   1. Dock phase  → flat at the harbour-wall height
        //                    (`--dock-height-m`, e.g. 0.75 m at Ermioni)
        //   2. Water-set    → smooth crossing through 0 m
        //   3. Foiling      → real mast lift, baro re-anchored to
        //                     pressure at water_set_t
        // We synthesise the dock-phase as a constant (no baro), and
        // splice in baro for the foiling phase. The water-set
        // moment is filled with a linear ramp from dock_height_m
        // down to 0 over 1 sec to avoid a vertical step.
        let dock_h = args.dock_height_m;
        let height_window: Vec<f64> = if let Some(wt) = water_set_t {
            let n = at_e - at_s;
            let wt_idx = (wt * sample_hz as f64).round() as usize;
            // Anchor pressure: average TC'd pressure ±0.5 s around wt.
            let tk_all: Vec<f64> = samples.iter()
                .map(|s| s.temperature_c + 273.15).collect();
            let mut sorted_tk = tk_all.clone();
            sorted_tk.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let ref_k = sorted_tk[sorted_tk.len() / 2];
            let lo_anchor = (at_s + wt_idx).saturating_sub(sample_hz / 2);
            let hi_anchor = (at_s + wt_idx + sample_hz / 2 + 1).min(samples.len());
            let mut sum = 0.0; let mut cnt = 0;
            for i in lo_anchor..hi_anchor {
                let p = samples[i].pressure_hpa * (ref_k / tk_all[i]);
                if p.is_finite() { sum += p; cnt += 1; }
            }
            let p_water = if cnt > 0 { sum / cnt as f64 } else { 1013.25 };
            println!("  height: P_water = {:.2} hPa @ +{:.1} s, dock = {:.2} m",
                     p_water, wt, dock_h);

            let mut h = vec![0.0; n];
            // 1-sec ramp window centred on water_set_t.
            let ramp_half = sample_hz; // ±1 sec around wt → 2-sec ramp
            let ramp_lo = wt_idx.saturating_sub(ramp_half);
            let ramp_hi = (wt_idx + ramp_half).min(n);
            // Outlier-reject: any pressure sample whose 10-ms delta to
            // its neighbour exceeds 1 hPa is treated as a glitch (we
            // observed a 3.3 hPa one-sample spike at end-of-window
            // that translated into an apparent −25 m drop). Replace
            // with NaN so the trace shows a gap instead.
            let outlier_dp = 1.0_f64;
            let raw_p_tc: Vec<f64> = (0..n).map(|i| {
                samples[at_s + i].pressure_hpa * (ref_k / tk_all[at_s + i])
            }).collect();
            let mut p_clean: Vec<f64> = raw_p_tc.clone();
            for i in 1..n {
                let dp = (raw_p_tc[i] - raw_p_tc[i - 1]).abs();
                if dp > outlier_dp { p_clean[i] = f64::NAN; }
            }
            for i in 0..n {
                if i < ramp_lo {
                    h[i] = dock_h;
                } else if i > ramp_hi {
                    // Foiling phase: baro re-anchored. Clamp to the
                    // physical range [-0.1, 0.9] m (mast = 0.8 m) so
                    // accumulated thermal drift can't push the trace
                    // into unphysical territory (we saw +2.5 m at
                    // end-of-window when the real lift was ~0.5 m).
                    let p = p_clean[i];
                    if p.is_finite() {
                        let baro_h = 8434.0 * (1.0 - p / p_water);
                        h[i] = baro_h.clamp(-0.1, 0.9);
                    } else {
                        h[i] = f64::NAN;
                    }
                } else {
                    // Linear ramp from dock_h to baro foil-phase value.
                    let p = p_clean[i];
                    let baro_h = if p.is_finite() {
                        (8434.0 * (1.0 - p / p_water)).clamp(-0.1, 0.9)
                    } else { 0.0 };
                    let alpha = (i - ramp_lo) as f64 /
                                (ramp_hi - ramp_lo).max(1) as f64;
                    h[i] = dock_h * (1.0 - alpha) + baro_h * alpha;
                }
            }
            // Light 0.5-sec smoothing (NaN-aware).
            let win = sample_hz / 2;
            let half = win / 2;
            let mut csum = vec![0.0; n + 1];
            let mut ccnt = vec![0usize; n + 1];
            for i in 0..n {
                csum[i + 1] = csum[i] + if h[i].is_finite() { h[i] } else { 0.0 };
                ccnt[i + 1] = ccnt[i] + if h[i].is_finite() { 1 } else { 0 };
            }
            let mut out = vec![0.0; n];
            for i in 0..n {
                let lo = i.saturating_sub(half);
                let hi = (i + half + 1).min(n);
                let cnt = ccnt[hi] - ccnt[lo];
                out[i] = if cnt > 0 { (csum[hi] - csum[lo]) / cnt as f64 }
                         else { f64::NAN };
            }
            out
        } else {
            // No water_set detected — use raw baro slice.
            height_full[at_s..at_e].to_vec()
        };
        let height_slice_anchored: Option<&[f64]> = if !height_window.is_empty() {
            Some(&height_window)
        } else { None };

        // Build a slice of "tilt quaternions" — pitch+roll only,
        // derived from a low-pass-filtered accelerometer reading.
        //
        // Accel-only attitude has a known failure mode: when linear
        // acceleration is comparable to gravity (e.g. the rider hits
        // a wave or pump-pushes hard), the accel vector points in a
        // direction unrelated to "down" and the rendered foil flips
        // sideways for a frame or two. Smoothing the accel with a
        // ~0.5 s rolling mean (50 samples at 100 Hz) preserves the
        // ~1 Hz pump-pitch signal but rejects sub-100-ms linear-
        // acceleration spikes.
        //
        // With deck-mount the chip's +Z faces down, so we pre-flip
        // accel Y and Z (180° around X) to get a right-side-up
        // gravity reading before extracting the pitch+roll angles.
        // Mast-mount has the chip oriented differently and uses the
        // Madgwick quaternion path directly (no accel pre-flip), so
        // tilt_quats is only built for the deck branch.
        let tilt_quats: Vec<fusion::Quat> = if args.mount == board3d::MountKind::Deck {
            let smooth_window: usize = (sample_hz / 4).max(1); // 0.25 s
            let half = smooth_window / 2;
            let n_total = samples.len();
            let mut acc_smooth: Vec<[f64; 3]> = Vec::with_capacity(n_total);
            for i in 0..n_total {
                let lo = i.saturating_sub(half);
                let hi = (i + half + 1).min(n_total);
                let mut sx = 0.0; let mut sy = 0.0; let mut sz = 0.0;
                for j in lo..hi {
                    sx += samples[j].acc[0];
                    sy += samples[j].acc[1];
                    sz += samples[j].acc[2];
                }
                let cnt = (hi - lo) as f64;
                acc_smooth.push([sx / cnt, sy / cnt, sz / cnt]);
            }
            // The chip's labelled X is along the box's *short*
            // dimension (lateral on the board), Y is along the long
            // dimension = nose-tail. Pitch (nose-up) → AccY, roll
            // (port-up) → AccX. The unary minuses on ay/az are the
            // 180°-around-X pre-flip for chip-Z-down deck mount.
            acc_smooth.iter().map(|a| {
                let ax = a[0];
                let ay = -a[1];
                let az = -a[2];
                let n = (ax * ax + ay * ay + az * az).sqrt();
                if n > 1e-9 {
                    let (ax, ay, az) = (ax / n, ay / n, az / n);
                    let pitch = (-ay).atan2((ax * ax + az * az).sqrt());
                    let roll  = ax.atan2(az);
                    let cp = (pitch * 0.5).cos();
                    let sp = (pitch * 0.5).sin();
                    let cr = (roll * 0.5).cos();
                    let sr = (roll * 0.5).sin();
                    [cp * cr, cp * sr, sp * cr, -sp * sr]
                } else {
                    [1.0, 0.0, 0.0, 0.0]
                }
            }).collect()
        } else {
            Vec::new()
        };
        // Source of per-frame quaternions for the 3D mesh: tilt-only
        // (deck mount) or full Madgwick (mast mount, where the mesh
        // path uses `quat_strip_yaw(quat_conj(q))` to drop the gyro-
        // drifted yaw).
        let quats_for_3d: &[fusion::Quat] = match args.mount {
            board3d::MountKind::Deck => &tilt_quats,
            board3d::MountKind::Mast => &quats,
        };
        let quats_slice: Option<&[fusion::Quat]> = if board_mesh.is_some() {
            Some(&quats_for_3d[at_s..at_e])
        } else { None };
        let yaw_slice: Option<&[f32]> = if board_mesh.is_some()
            && !yaw_full_rad.is_empty()
        {
            Some(&yaw_full_rad[at_s..at_e])
        } else { None };
        render_session_gif(
            &nose_corrected[at_s..at_e],
            0,
            &base,
            sample_hz,
            args.fps,
            &gif_path,
            Some(start_local),
            height_slice_anchored,
            speed_slice,
            None,
            map_data,
            water_set_t,
            foil_start_t,
            board_mesh.as_ref(),
            quats_slice,
            q_mount_conj,
            yaw_slice,
            // Re-zero the cumulative pump count at at-window start
            // so the displayed counter starts at 0 for this clip.
            {
                let off = pump_count_full[at_s];
                let pc: Vec<u32> = pump_count_full[at_s..at_e].iter()
                    .map(|&v| v - off).collect();
                Some(pc)
            }.as_deref(),
            args.mount,
        )?;
        println!("Saved {}", gif_path.display());

        if let Some(video_path) = args.video {
            let mov_path = args.output_dir.join(
                format!("combined_{}_at{}.mov", base, label));
            combine_with_ffmpeg(
                &gif_path, video_path, video_offset, args.sensor_offset,
                &mov_path, args.title, args.subtitle, args.fps * 2,
            )?;
            println!("Saved {}", mov_path.display());
        }
        return Ok(());
    }

    let sessions = detect_sessions(&pitch, &roll, sample_hz);
    if sessions.is_empty() {
        anyhow::bail!("no pumping sessions detected");
    }
    println!("found {} pumping session(s)", sessions.len());

    for (i, &(s, e)) in sessions.iter().enumerate() {
        let session_num = i + 1;
        if let Some(target) = args.session {
            if session_num != target { continue; }
        }

        let dur = (e - s) as f64 / sample_hz as f64;
        println!("  session {}: {:.0} s ({} samples)", session_num, dur, e - s);

        let gif_path = args.output_dir.join(
            format!("anim_board_{}_session{}.gif", base, session_num));
        let h_slice: Option<&[f64]> = if !height_full.is_empty() {
            Some(&height_full[s..e])
        } else { None };
        let v_slice: Option<&[f64]> = if !speed_at_sensor.is_empty() {
            Some(&speed_at_sensor[s..e])
        } else { None };
        render_session_gif(
            &nose_corrected[s..e],
            session_num,
            &base,
            sample_hz,
            args.fps,
            &gif_path,
            None,
            h_slice,
            v_slice,
            None,
            None,  // legacy session-detection mode skips the map panel
            None,  // and the phase boundaries
            None,
            None, None, None, None, None,  // 3D mesh/quats/q_mount/yaw + pump_count — only --at mode
            args.mount,
        )?;
        println!("Saved {}", gif_path.display());

        if let Some(video_path) = args.video {
            let mov_path = args.output_dir.join(
                format!("combined_session{}_drop.mov", session_num));
            combine_with_ffmpeg(
                &gif_path, video_path, args.video_offset, args.sensor_offset,
                &mov_path, args.title, args.subtitle, args.fps * 2,
            )?;
            println!("Saved {}", mov_path.display());
        }
    }

    Ok(())
}

/// Resolve `--at HH:MM[:SS]` (in local time) + optional `--duration` /
/// video length into a (start, end) sensor-sample-index pair.
///
/// Anchors local time to the sensor sample stream by:
/// 1. Picking a sensor-side reference tick (sensors[0].ticks).
/// 2. Pulling the matching wall-clock UTC from the first GPS fix's
///    UTC time-of-day, back-extrapolated to that reference tick.
/// 3. Combining with the recording date (CLI arg, then file mtime,
///    then today) to get an absolute UTC anchor at sensor t = 0.
/// 4. Adding the local-time offset and computing the requested window.
fn resolve_at_window(
    sensor_path: &Path,
    samples: &[crate::io::SensorRow],
    sample_hz: usize,
    at_str: &str,
    duration_arg: Option<f64>,
    video: Option<&Path>,
    tz_offset_h: f64,
    date_arg: Option<&str>,
) -> Result<(usize, usize, NaiveDateTime)> {
    let gps_path = guess_gps_path(sensor_path)
        .ok_or_else(|| anyhow!("no GPS CSV next to {}; --at needs GPS to anchor wall-clock time",
                               sensor_path.display()))?;
    let mut rows = load_gps_csv(&gps_path)?;
    rows.retain(|g| g.fix >= 1);
    if rows.is_empty() {
        anyhow::bail!("GPS CSV has no valid fixes; --at needs GPS to anchor wall-clock time");
    }

    if let Some(vt) = video.and_then(ffprobe_creation_time) {
        let local = vt + ChronoDuration::milliseconds((tz_offset_h * 3600.0 * 1000.0) as i64);
        println!("  video creation_time: {} local (override --at to align precisely)",
                 local.format("%H:%M:%S"));
    }
    let at_local_sec = parse_hhmm_to_sec(at_str)
        .ok_or_else(|| anyhow!("could not parse --at '{}', expected HH:MM[:SS]", at_str))?;
    let at_utc_sec_of_day = at_local_sec - tz_offset_h * 3600.0;

    // ThreadX runs on HSI (internal RC, ±1 % accuracy) so ticks drift
    // ~7 s over a 21-min session — linear extrapolation from the
    // first GPS sample is unreliable. Instead, find the GPS row whose
    // UTC is closest to the target and use ITS tick directly as the
    // anchor. Both GPS and sensor rows share the same ThreadX tick
    // counter, so they line up locally even when the tick-vs-wall-
    // clock relationship has drifted.
    let mut best_idx = 0usize;
    let mut best_diff = f64::INFINITY;
    for (i, row) in rows.iter().enumerate() {
        if let Some(u) = parse_utc_hhmmss(&row.utc) {
            let d = (u - at_utc_sec_of_day).abs();
            if d < best_diff { best_diff = d; best_idx = i; }
        }
    }
    if best_diff > 5.0 {
        anyhow::bail!("no GPS row within 5 s of --at {} (closest was off by {:.1} s) — \
                       check --tz-offset-h / --date / GPS coverage",
                      at_str, best_diff);
    }
    let anchor_tick = rows[best_idx].ticks;
    let anchor_utc_sec = parse_utc_hhmmss(&rows[best_idx].utc).unwrap_or(at_utc_sec_of_day);

    // Window length: explicit --duration → ffprobe video length → 60 s.
    let dur = duration_arg.or_else(|| {
        video.and_then(|v| ffprobe_duration(v).ok())
    }).unwrap_or(60.0);

    // End-anchor: same GPS-UTC trick as the start. Without this we'd
    // compute the end as anchor_tick + dur*100, which assumes 100 ticks
    // = 1 wall-clock second — but ThreadX runs on the drifting HSI, so
    // by the end of a long window the tick count diverges from real
    // time. Symptom: side-by-side combined.mov where the GIF panel
    // (driven by ticks) lags or leads the camera video (true wall-
    // clock) by several seconds at the end. Anchoring both ends via
    // GPS UTC keeps the data window equal to `dur` real seconds
    // regardless of HSI drift.
    let target_end_utc = anchor_utc_sec + dur;
    let mut end_idx = best_idx;
    let mut end_diff = f64::INFINITY;
    for (i, row) in rows.iter().enumerate() {
        if let Some(u) = parse_utc_hhmmss(&row.utc) {
            let d = (u - target_end_utc).abs();
            if d < end_diff { end_diff = d; end_idx = i; }
        }
    }
    let anchor_end_tick = if end_diff < 5.0 {
        rows[end_idx].ticks
    } else {
        anchor_tick + dur * 100.0  // fallback: no GPS coverage near end
    };
    let s = samples.iter().position(|r| r.ticks >= anchor_tick).unwrap_or(0);
    let e = samples.iter().position(|r| r.ticks >= anchor_end_tick)
        .unwrap_or(samples.len());

    // Local-time anchor for display = the GPS UTC at the anchor row,
    // shifted by tz_offset_h. Date comes from CLI / file mtime / today.
    let date_utc = resolve_session_date(date_arg, sensor_path)?;
    let local_secs = anchor_utc_sec + tz_offset_h * 3600.0;
    let start_local = NaiveDateTime::new(date_utc, NaiveTime::from_hms_opt(0, 0, 0).unwrap())
        + ChronoDuration::milliseconds((local_secs * 1000.0).round() as i64);

    Ok((s.min(samples.len()), e.min(samples.len()), start_local))
}

fn parse_hhmm_to_sec(s: &str) -> Option<f64> {
    let parts: Vec<&str> = s.split(':').collect();
    let h: f64 = parts.first()?.parse().ok()?;
    let m: f64 = parts.get(1)?.parse().ok()?;
    let sec: f64 = parts.get(2).map(|x| x.parse().unwrap_or(0.0)).unwrap_or(0.0);
    Some(h * 3600.0 + m * 60.0 + sec)
}

fn parse_utc_hhmmss(s: &str) -> Option<f64> {
    let s = s.trim();
    if s.len() < 6 { return None; }
    let h: f64 = s[0..2].parse().ok()?;
    let m: f64 = s[2..4].parse().ok()?;
    let sec: f64 = s[4..].parse().ok()?;
    Some(h * 3600.0 + m * 60.0 + sec)
}

fn resolve_session_date(arg: Option<&str>, sensor_path: &Path) -> Result<NaiveDate> {
    if let Some(s) = arg {
        return NaiveDate::parse_from_str(s, "%Y-%m-%d")
            .with_context(|| format!("parse --date {}", s));
    }
    if let Ok(meta) = std::fs::metadata(sensor_path) {
        if let Ok(mtime) = meta.modified() {
            let dt: DateTime<Utc> = mtime.into();
            return Ok(dt.date_naive());
        }
    }
    Ok(Utc::now().date_naive())
}

fn guess_gps_path(sensor_path: &Path) -> Option<PathBuf> {
    let stem = sensor_path.file_stem()?.to_str()?.to_string();
    let parent = sensor_path.parent()?;
    if let Some(rest) = stem.strip_prefix("Sens") {
        let gps = parent.join(format!("Gps{}.csv", rest));
        if gps.exists() { return Some(gps); }
    }
    let gps = parent.join(format!("{}_gps.csv", stem));
    if gps.exists() { Some(gps) } else { None }
}

/// First sample index inside `pitch` where sustained pumping
/// oscillation starts. Carry/transition before water entry is
/// irregular and the board may be in any orientation — so we look for
/// the first 3-second window with ≥ 2 zero-crossings of the detrended
/// pitch and amplitude > 5°. Returns None if the window never settles
/// into pumping (e.g. pure walking, or window too short).
fn detect_water_entry(pitch: &[f64], sample_hz: usize) -> Option<usize> {
    let n = pitch.len();
    let win = 3 * sample_hz;
    if n < win + sample_hz { return None; }

    // 5-second rolling-median baseline → detrended pitch isolates the
    // oscillatory component. Median (not mean) so a few seconds of
    // carry-tilt with constant offset don't bias the baseline.
    let baseline = rolling_median(pitch, 5 * sample_hz);
    let detrended: Vec<f64> = pitch.iter().zip(baseline.iter())
        .map(|(p, b)| p - b).collect();

    for start in 0..n.saturating_sub(win) {
        let w = &detrended[start..start + win];
        let amp = w.iter().fold(0.0f64, |a, &v| a.max(v.abs()));
        if amp < 5.0 { continue; }
        let mut crossings = 0;
        let mut prev = 0.0f64;
        for &v in w {
            if v.signum() != 0.0 && prev.signum() != 0.0 && v.signum() != prev.signum() {
                crossings += 1;
            }
            if v != 0.0 { prev = v; }
        }
        if crossings >= 2 {
            return Some(start);
        }
    }
    None
}

fn ffprobe_duration(video: &Path) -> Result<f64> {
    let probe = Command::new("ffprobe")
        .args(["-v", "error", "-show_entries", "format=duration",
               "-of", "default=noprint_wrappers=1:nokey=1"])
        .arg(video)
        .output()
        .with_context(|| "running ffprobe — is it in PATH?")?;
    let s = String::from_utf8_lossy(&probe.stdout).trim().to_string();
    s.parse::<f64>().with_context(|| format!("parse ffprobe output '{}'", s))
}

/// Read the video's `creation_time` (typically iPhone's wall-clock UTC
/// at the start of recording). Returns None if absent / unparseable.
fn ffprobe_creation_time(video: &Path) -> Option<DateTime<Utc>> {
    let probe = Command::new("ffprobe")
        .args(["-v", "error", "-show_entries", "format_tags=creation_time",
               "-of", "default=noprint_wrappers=1:nokey=1"])
        .arg(video)
        .output().ok()?;
    let s = String::from_utf8_lossy(&probe.stdout).trim().to_string();
    if s.is_empty() { return None; }
    DateTime::parse_from_rfc3339(&s).ok().map(|d| d.with_timezone(&Utc))
}

/// Raw nose elevation in degrees from quaternions.
fn nose_angle_deg_raw(quats: &[fusion::Quat]) -> Vec<f64> {
    quats.iter().map(|q| {
        let (qs, qi, qj, qk) = (q[0], q[1], q[2], q[3]);
        let nose_z = (2.0 * (qj * qk - qs * qi)).clamp(-1.0, 1.0);
        nose_z.asin().to_degrees()
    }).collect()
}

fn rolling_median(x: &[f64], window: usize) -> Vec<f64> {
    let n = x.len();
    let half = window / 2;
    let mut out = vec![0.0; n];
    for i in 0..n {
        let lo = i.saturating_sub(half);
        let hi = (i + half + 1).min(n);
        let mut s: Vec<f64> = x[lo..hi].to_vec();
        s.sort_by(|a, b| a.partial_cmp(b).unwrap());
        out[i] = s[s.len() / 2];
    }
    out
}

fn render_session_gif(
    nose: &[f64],
    session_num: usize,
    base: &str,
    sample_hz: usize,
    fps: u32,
    out_path: &PathBuf,
    _start_local: Option<NaiveDateTime>,
    height_m: Option<&[f64]>,
    speed_kmh: Option<&[f64]>,
    nose_abs: Option<&[f64]>,
    map: Option<(&[f64], &[f64], &[f64])>,  // (lat, lon, time_s) within window
    water_set_t: Option<f64>,   // seconds when board enters water
    foil_start_t: Option<f64>,  // seconds when foiling begins
    // 3D side-view inputs. `mesh` + `quats` engage the 3D path;
    // `q_mount_conj` (when Some) calibrates the mesh to a level
    // reference; otherwise the line fallback is used.
    board3d_mesh: Option<&board3d::Mesh>,
    quats: Option<&[fusion::Quat]>,
    q_mount_conj: Option<[f64; 4]>,
    yaw_rad: Option<&[f32]>,
    // Cumulative pump count per sample, aligned with `nose`. When
    // None, the displayed Pumps counter shows 0.
    pump_count: Option<&[u32]>,
    mount: board3d::MountKind,
) -> Result<()> {
    // Subsample so the GIF's wall-clock playback matches the data
    // window exactly. Two pitfalls to avoid:
    // (1) Integer step (e.g. 100/15 = 6) gives 60 ms of data per
    //     frame at 15 fps target, so the GIF ran 1.117× too long.
    //     Float step fixes that.
    // (2) GIF spec encodes per-frame delay in 1/100-sec units (10 ms
    //     resolution). 1000/15 = 66.67 ms gets rounded by the gif
    //     crate to 70 ms → real playback rate 14.286 fps, not 15.
    //     If we still compute step_f from the 15-fps target, the
    //     GIF plays ~5 % slower than the underlying data window —
    //     in a 41-sec at-window paired with a camera video, that's
    //     ~2 s of drift by the end. Pre-round delay_ms to a 10-ms
    //     multiple, derive effective_fps from THAT, and use it for
    //     step_f so the encoded GIF rate matches the data step.
    let delay_ms_target = 1000.0 / fps as f64;
    let delay_ms = (((delay_ms_target / 10.0).round() * 10.0) as u32).max(10);
    let effective_fps = 1000.0 / delay_ms as f64;
    let step_f = sample_hz as f64 / effective_fps;
    let n_frames = ((nose.len() as f64) / step_f).round() as usize;
    let frame_indices: Vec<usize> = (0..n_frames)
        .map(|i| ((i as f64 * step_f).round() as usize).min(nose.len() - 1))
        .collect();

    let t_sess: Vec<f64> = (0..nose.len()).map(|i| i as f64 / sample_hz as f64).collect();
    let duration_s = *t_sess.last().unwrap_or(&0.0);
    let dm = (duration_s / 60.0) as u32;
    let ds = ((duration_s % 60.0) as u32).min(59);

    // Drop-in flash: in --at mode trigger at water_set_t (= the moment
    // the rider's feet land on the board). The drop angle itself is
    // the steepest negative nose angle in a ±1 s window around that
    // moment — which captures the actual stepping-down motion. When
    // no water_set_t is provided (legacy session-detection mode),
    // fall back to "steepest negative angle in the first 10 s".
    let (drop_idx, drop_time) = if let Some(wt) = water_set_t {
        let center = (wt * sample_hz as f64).round() as usize;
        let lo = center.saturating_sub(sample_hz);
        let hi = (center + sample_hz + 1).min(nose.len());
        let local_min = nose[lo..hi].iter().enumerate()
            .min_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| lo + i).unwrap_or(center.min(nose.len() - 1));
        (local_min, wt)
    } else {
        let first10 = &nose[..nose.len().min(10 * sample_hz)];
        let i = first10.iter().enumerate()
            .min_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i).unwrap_or(0);
        (i, i as f64 / sample_hz as f64)
    };
    let drop_angle = nose[drop_idx];
    let drop_flash_end = drop_time + 2.0;

    // Pump counter consumed from caller. Caller computes peaks in
    // the dynamic-acceleration magnitude (one peak per pump stroke)
    // using the full sensor stream and passes the cumulative count
    // per sample sliced to the at-window.
    let pump_count_at_sample: Vec<u32> = pump_count
        .map(|s| s.to_vec())
        .unwrap_or_else(|| vec![0u32; nose.len()]);

    // Scales are computed PER FRAME inside draw_frame from the
    // history-up-to-now slices, so the y-max grows with the trace.
    // Earlier frames get a tight scale that highlights the small early
    // movements; later frames expand to fit any peaks.

    // Bitmap frame size
    let (w, h) = (1200u32, 900u32);

    let title_line = format!("Session {} — {} (Dauer: {}:{:02})",
        session_num, base, dm, ds);

    let root = BitMapBackend::gif(out_path, (w, h), delay_ms)?
        .into_drawing_area();

    // Push-off frame index + carry-phase yaw offset.
    //
    // After push_off (foiling phase) we strip the Madgwick yaw entirely
    // because the 6DOF fusion has no magnetometer reference and drifts.
    // Before push_off (carry phase) we use the *full* body-to-world
    // quaternion so the rendered foil rotates as Ayano turns the board
    // in his hands while walking.
    //
    // To avoid a yaw discontinuity at push_off, we subtract the
    // body-to-world Z-twist of the quaternion sampled at push_off from
    // every carry frame. That makes the carry-phase yaw equal zero
    // at push_off, smoothly matching the stripped foiling yaw.
    let push_off_idx: Option<usize> = water_set_t.zip(quats).map(|(t, qs)| {
        ((t * sample_hz as f64).round() as usize).min(qs.len().saturating_sub(1))
    });
    let q_twist_pushoff: [f64; 4] = match (push_off_idx, quats) {
        (Some(idx), Some(qs)) => {
            let p = board3d::quat_conj(&qs[idx]);
            let (qw, qz) = (p[0], p[3]);
            let n = (qw * qw + qz * qz).sqrt();
            if n > 1e-9 { [qw / n, 0.0, 0.0, qz / n] } else { [1.0, 0.0, 0.0, 0.0] }
        }
        _ => [1.0, 0.0, 0.0, 0.0],
    };
    let q_twist_pushoff_inv = board3d::quat_conj(&q_twist_pushoff);

    // Carry-phase orientation corrections (applied per-frame
    // depending on whether the rider is in foil-up or foil-down
    // sub-pose; see below for the deck-up direction check):
    //
    // 1. 180° around the board's pitch axis (body frame). Foil-up
    //    only — corrects the inverted mast pose so the rendered
    //    mesh shows deck-up rather than deck-down.
    //    Under the current mount (board_Y → IMU +Z), the board's
    //    pitch axis is IMU +Z, so the quaternion is [0, 0, 0, 1].
    //
    // 2. 180° around the board's roll axis (body frame). Foil-up
    //    only — additional correction the rider observed visually:
    //    after the pitch flip the board still rendered rolled by
    //    180° vs. the camera footage.
    //    Roll axis = board +X = IMU +Y, so the quaternion is
    //    [0, 0, 1, 0].
    //
    // 3. 180° around world yaw (Z). Both sub-poses — the gyro-
    //    integrated yaw is unanchored (GPS course unreliable below
    //    3 km/h), so the rendered nose tends to face 180° off from
    //    the rider's actual heading throughout the carry.
    let q_180_pitch_body: [f64; 4] = [0.0, 0.0, 0.0, 1.0];
    let q_180_roll_body:  [f64; 4] = [0.0, 0.0, 1.0, 0.0];
    let q_180_yaw_world:  [f64; 4] = [0.0, 0.0, 0.0, 1.0];

    println!("  generating {} frames…", n_frames);
    for (frame, &fi) in frame_indices.iter().enumerate() {
        root.fill(&WHITE)?;
        let t_now = fi as f64 / sample_hz as f64;
        let angle = nose_abs.map(|s| s[fi]).unwrap_or(nose[fi]);
        let h_hist = height_m.map(|s| &s[..=fi]);
        let v_hist = speed_kmh.map(|s| &s[..=fi]);
        let n_abs_hist = nose_abs.map(|s| &s[..=fi]);
        let h_now = height_m.and_then(|s| if s[fi].is_finite() { Some(s[fi]) } else { None });
        // Per-frame quaternion → mesh rotation.
        //
        // The mesh is pre-rotated at load time by the mount transform
        // `R_mount` (in board3d::Mesh::recentre_unit), so it lives in
        // the IMU body frame. Madgwick's q is world-to-body, so
        // conj(q) is body-to-world.
        //
        // Phase-dependent yaw handling:
        //   - Carry phase (fi < push_off_idx): use the FULL body-to-
        //     world quaternion, with the Z-twist at push_off
        //     subtracted. This lets the rendered foil yaw with Ayano
        //     as he turns the board in his hands, while still landing
        //     at zero yaw at push_off so the transition into foiling
        //     is seamless.
        //   - Foiling phase (fi >= push_off_idx): strip the yaw via
        //     swing-twist decomposition around world Z. Madgwick 6DOF
        //     yaw drifts (no magnetometer), so we ignore it and let
        //     the board sit with a fixed heading in the rendered
        //     scene.
        let _ = q_mount_conj;
        let yaw_now = yaw_rad.map(|s| s[fi]);
        // 3D mesh quaternion. The side-view panel hides the 3D
        // model entirely until push-off — see the `in_foiling`
        // gating in draw_frame. Before push-off the Madgwick yaw
        // has drifted freely (no magnetometer, no GPS course
        // correction at < 3 km/h), so the rendered carry-phase
        // orientation isn't trustworthy. We compute a quaternion
        // here for every frame anyway because the panel falls back
        // to a "Tragen — keine 3D-Daten" placeholder, and the
        // value is just unused on those frames.
        //
        // For foiling frames: strip the Madgwick yaw via swing-
        // twist decomposition, leaving only pitch+roll. The board
        // sits with a fixed heading in the rendered scene; the
        // side panel cares about pitch/roll, not which way the
        // nose is pointing.
        // 3D mesh quaternion. The caller passes accel-only tilt
        // quaternions (pitch+roll from gravity direction in body
        // frame, no gyro integration → no yaw drift around the
        // mast). On top of that we pre-multiply a 180° world-Z
        // rotation because the STL's nose direction lands at
        // world-X with the identity tilt quaternion, which puts
        // the nose toward the camera at -X (= "view from front").
        // Pre-multiplying by 180°-world-Z rotates the entire
        // scene around the vertical, putting the nose at world+X
        // = into the screen for a camera at -X = "view from
        // behind the tail."
        let _ = (yaw_now, q_twist_pushoff_inv,
                 q_180_pitch_body, q_180_roll_body,
                 push_off_idx, q_mount_conj);
        let q_rel_now: Option<[f64; 4]> = quats.map(|qs| match mount {
            // Deck mount: caller passes accel-only tilt quaternions;
            // pre-multiply 180° world-Z so the STL nose lands at
            // world+X for a camera at world−X (= "view from behind
            // the tail").
            board3d::MountKind::Deck => board3d::quat_mul(&q_180_yaw_world, &qs[fi]),
            // Mast mount: caller passes Madgwick world-to-body
            // quaternions. Conjugate to get body-to-world, then
            // strip the gyro-drifted yaw via swing-twist
            // decomposition so the rendered foil keeps a fixed
            // heading (the side-view panel only cares about
            // pitch+roll).
            board3d::MountKind::Mast => board3d::quat_strip_yaw(&board3d::quat_conj(&qs[fi])),
        });
        let pump_count_now = pump_count_at_sample[fi];
        draw_frame(
            &root,
            &title_line,
            &nose[..=fi],
            &t_sess[..=fi],
            t_now,
            angle,
            drop_time,
            drop_flash_end,
            drop_angle,
            h_hist,
            v_hist,
            n_abs_hist,
            h_now,
            map,
            water_set_t,
            foil_start_t,
            board3d_mesh,
            q_rel_now,
            yaw_now,
            pump_count_now,
            mount,
        )?;
        root.present()?;
        if frame % 100 == 0 {
            print!("\r    frame {}/{}", frame, n_frames);
            use std::io::Write;
            std::io::stdout().flush().ok();
        }
    }
    println!("\r    frame {}/{}  ", n_frames, n_frames);
    Ok(())
}

fn draw_frame<DB: DrawingBackend>(
    root: &DrawingArea<DB, plotters::coord::Shift>,
    title: &str,
    nose_hist: &[f64],
    t_hist: &[f64],
    t_now: f64,
    angle: f64,
    drop_time: f64,
    drop_flash_end: f64,
    drop_angle: f64,
    height_hist: Option<&[f64]>,
    speed_hist: Option<&[f64]>,
    nose_abs_hist: Option<&[f64]>,
    height_now: Option<f64>,
    map: Option<(&[f64], &[f64], &[f64])>,
    water_set_t: Option<f64>,
    foil_start_t: Option<f64>,
    // 3D side-view inputs. When `mesh` is Some AND pitch/roll/yaw are
    // all Some, the board side-view panel renders a 3D-rasterized
    // board mesh instead of the 2D line. Otherwise the legacy line
    // path is used.
    mesh: Option<&board3d::Mesh>,
    q_rel_now: Option<[f64; 4]>,
    yaw_rad_now: Option<f32>,
    pump_count: u32,
    mount: board3d::MountKind,
) -> Result<(), anyhow::Error>
where
    DB::ErrorType: 'static,
{
    // Per-frame scales: y-max equals the running max of the history
    // shown so far. No headroom above. Tiny floors prevent degenerate
    // empty ranges in the very first frames.
    let zoom_lim: f64 = {
        let mut mx = 0f64;
        for &v in nose_hist {
            if v.is_finite() && v.abs() > mx { mx = v.abs(); }
        }
        mx.max(1.0)
    };
    let y_lim: f64 = {
        // 95th-percentile-based scale: a single ±35° outlier pump
        // shouldn't stretch the axis so the typical ±10° activity
        // ends up in the bottom 30 % of the panel. Outliers above
        // p95 are visually clipped at the chart edge, which the
        // user prefers over wasted vertical space.
        let src = nose_abs_hist.unwrap_or(nose_hist);
        let mut abs: Vec<f64> = src.iter()
            .filter(|v| v.is_finite())
            .map(|v| v.abs())
            .collect();
        if abs.is_empty() {
            1.0
        } else {
            abs.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let i = ((abs.len() as f64 * 0.95) as usize).min(abs.len() - 1);
            abs[i].max(1.0).min(180.0)
        }
    };
    let speed_top: f64 = if let Some(s) = speed_hist {
        let m = s.iter().filter(|v| v.is_finite()).copied()
            .fold(0.0f64, |a, b| a.max(b));
        m.max(0.5)
    } else { 30.0 };
    let height_top: f64 = if let Some(s) = height_hist {
        let m = s.iter()
            .filter(|v| v.is_finite())
            .copied()
            .fold(0.0f64, |a, b| a.max(b));
        m.max(0.1)
    } else { 0.9 };
    let height_bot: f64 = if let Some(s) = height_hist {
        let m = s.iter()
            .filter(|v| v.is_finite())
            .copied()
            .fold(f64::INFINITY, |a, b| a.min(b));
        if m.is_finite() { m.min(0.0) } else { -0.1 }
    } else { -0.1 };
    // Title across the top
    root.draw(&Text::new(
        title.to_string(),
        (10, 4),
        (FONT, 18).into_font().color(&BLACK),
    )).map_err(|e| anyhow::anyhow!("title: {e:?}"))?;

    // Layout: 5 panels when height + speed are present, 3 panels
    // otherwise (legacy session-detection mode without GPS). The
    // canvas is 900 px tall and the global title needs ~25 px at top.
    // When `map` is also given, the top row splits horizontally into
    // board view (left half) + GPS-track map (right half).
    let have_extras = height_hist.is_some() && speed_hist.is_some();
    let (top_area, rest) = if have_extras {
        // top=320, rest=580 → split rest into zoom=130, height=145, speed=145, graph=160
        root.split_vertically(320)
    } else {
        root.split_vertically(430)
    };
    let (board_area, map_area) = if map.is_some() {
        let (b, m) = top_area.split_horizontally(600);
        (b, Some(m))
    } else {
        (top_area, None)
    };
    let (zoom_area, after_zoom) = if have_extras {
        rest.split_vertically(130)
    } else {
        rest.split_vertically(225)
    };
    let (height_area, speed_area, graph_area) = if have_extras {
        let (h_a, after_h) = after_zoom.split_vertically(145);
        let (v_a, g_a) = after_h.split_vertically(145);
        (Some(h_a), Some(v_a), g_a)
    } else {
        (None, None, after_zoom)
    };

    // --- Board side view ---
    let rad = angle.to_radians();
    let hl = 0.8f64;        // half-length
    // Lift = real baro-derived height when available, clamped to ±0.7 m
    // so the board stays in the ±0.8 view. Falls back to a small angle-
    // based proxy (1/100 of the angle in radians × hl) when no baro.
    let lift: f64 = match height_now {
        Some(h) if h.is_finite() => h.clamp(-0.1, 0.75),
        _ => angle * 0.005,
    };
    let nose_x = hl * rad.cos();
    let nose_y = hl * rad.sin() + lift;
    let tail_x = -hl * rad.cos();
    let tail_y = -hl * rad.sin() + lift;

    // y=30 clears the global title (FONT 18 at root y=4) below it.
    // The "Nasenwinkel: ±X.X°" overlay is moved further down so the
    // two don't share a row.
    board_area.draw(&Text::new(
        "Höhe [m]".to_string(),
        (12, 30),
        (FONT, 22).into_font().color(&BLACK),
    )).map_err(|e| anyhow::anyhow!("board y-label: {e:?}"))?;
    let mut bc = ChartBuilder::on(&board_area)
        .margin(20)
        .x_label_area_size(30)
        .y_label_area_size(60)
        .build_cartesian_2d(-1.3f64..1.3f64, -0.8f64..0.8f64)
        .map_err(|e| anyhow::anyhow!("board chart: {e:?}"))?;
    bc.configure_mesh()
        .light_line_style(RGBColor(240, 240, 240))
        .draw().map_err(|e| anyhow::anyhow!("mesh: {e:?}"))?;

    // Water — fill rectangle + animated wavy surface line that scrolls
    // backward at ~0.6 m/s, giving the illusion the board is travelling
    // forward over the water while it pumps. Wave amplitude is tiny
    // (2 cm) so it doesn't compete visually with the board's tilt/lift,
    // but the motion is unmistakable.
    bc.draw_series(std::iter::once(Rectangle::new(
        [(-2.0, -0.8), (2.0, 0.0)],
        RGBColor(179, 217, 255).mix(0.4).filled(),
    ))).map_err(|e| anyhow::anyhow!("water: {e:?}"))?;
    {
        use std::f64::consts::PI;
        let wave_amp = 0.02;
        let wavelength = 0.8;
        let scroll_speed = 0.6; // m/s
        let phase = -t_now * scroll_speed * 2.0 * PI / wavelength;
        let n_pts = 240usize;
        let wave_pts: Vec<(f64, f64)> = (0..n_pts).map(|i| {
            let x = -2.0 + 4.0 * (i as f64 / (n_pts - 1) as f64);
            let y = wave_amp * (phase + 2.0 * PI * x / wavelength).sin();
            (x, y)
        }).collect();
        bc.draw_series(LineSeries::new(
            wave_pts,
            RGBColor(0, 119, 190).stroke_width(2),
        )).map_err(|e| anyhow::anyhow!("waterline: {e:?}"))?;
    }

    // Board
    // Board: 3D-rasterized mesh if --board-stl provided, we have
    // pitch/roll/yaw for this frame, AND either push-off has
    // happened (foiling, GPS course reliable) OR a fixed early
    // start time has elapsed (for clips where the rider is
    // already foiling at the start of the window and push-off is
    // outside it). The fixed start gives the visualization a
    // moment to settle (Madgwick gyro convergence + waiting until
    // the board is clearly on the water before drawing).
    const SHOW_3D_FROM_S: f64 = 10.5;
    let after_push_off = water_set_t.map_or(false, |wt| t_now >= wt);
    let elapsed_enough = t_now >= SHOW_3D_FROM_S;
    let show_3d_now = after_push_off || elapsed_enough;
    let render_3d = mesh.is_some() && q_rel_now.is_some() && show_3d_now;
    if render_3d {
        let m = mesh.unwrap();
        // Render the 3D board into a bitmap that fills the entire
        // board_area panel. We blit directly on `board_area` with
        // pixel coordinates rather than going via the chart because
        // plotters' chart-coord-to-pixel mapping for `BitMapElement`
        // doesn't reliably hit our intended rectangle when the panel
        // layout has margins + label areas.
        let (panel_w, panel_h) = board_area.dim_in_pixel();
        // Build the body-to-world rotation matrix directly from the
        // calibrated relative quaternion. q_rel = conj(q_mount) *
        // q_now, so applying it to the level-pose STL mesh produces
        // the actual board orientation regardless of how the IMU is
        // mounted inside the SensorTile box. Yaw drift from
        // Madgwick's 6DOF (no magnetometer) is replaced with GPS
        // course in a future iteration; for now whatever yaw component
        // is in q_rel comes through.
        let q_rel = q_rel_now.unwrap();
        let _ = angle;
        let _ = yaw_rad_now;
        let rot = board3d::quat_to_matrix(&q_rel);
        let cam = board3d::Camera::iso(mount);
        let frame = board3d::render_with_matrix(
            m, rot, &cam, panel_w, panel_h, [34, 102, 170],
        );
        // RGBA → RGB. Transparent pixels become white so the bitmap
        // blends with the surrounding panel background.
        let mut rgb = vec![255u8; (frame.w * frame.h * 3) as usize];
        for i in 0..(frame.w * frame.h) as usize {
            if frame.rgba[i * 4 + 3] != 0 {
                rgb[i * 3]     = frame.rgba[i * 4];
                rgb[i * 3 + 1] = frame.rgba[i * 4 + 1];
                rgb[i * 3 + 2] = frame.rgba[i * 4 + 2];
            }
        }
        if let Some(elem) = plotters::element::BitMapElement::with_owned_buffer(
            (0i32, 0i32), (frame.w, frame.h), rgb)
        {
            board_area.draw(&elem)
                .map_err(|e| anyhow::anyhow!("board3d blit: {e:?}"))?;
        }
    } else if !show_3d_now {
        // Carry phase: no 3D model, no 2D line. Just the water
        // surface (already drawn above) plus a centered placeholder
        // text indicating that 3D data isn't available yet.
        let (panel_w, panel_h) = board_area.dim_in_pixel();
        let txt = "Tragen — keine 3D-Daten";
        board_area.draw(&Text::new(
            txt.to_string(),
            ((panel_w as i32) / 2 - 130, (panel_h as i32) / 2 - 12),
            (FONT, 24).into_font().color(&RGBColor(120, 120, 120)),
        )).map_err(|e| anyhow::anyhow!("carry placeholder: {e:?}"))?;
    } else {
        bc.draw_series(std::iter::once(PathElement::new(
            vec![(tail_x, tail_y), (nose_x, nose_y)],
            RGBColor(34, 102, 170).stroke_width(10),
        ))).map_err(|e| anyhow::anyhow!("board: {e:?}"))?;
        bc.draw_series(std::iter::once(Circle::new(
            (nose_x, nose_y), 7, RGBColor(220, 20, 20).filled(),
        ))).map_err(|e| anyhow::anyhow!("nose: {e:?}"))?;
        bc.draw_series(std::iter::once(Rectangle::new(
            [(tail_x - 0.03, tail_y - 0.03), (tail_x + 0.03, tail_y + 0.03)],
            RGBColor(34, 68, 102).filled(),
        ))).map_err(|e| anyhow::anyhow!("tail: {e:?}"))?;
    }

    // Nose angle / time / pump overlays. The Nasenwinkel and Pumps
    // values are meaningless while the rider is still carrying the
    // board (no foiling motion, the IMU's "pitch" is whatever the
    // board's hold orientation happens to be), so we suppress them
    // until push-off. The Zeit clock stays on always.
    let (bw, _bh) = board_area.dim_in_pixel();
    if show_3d_now {
        board_area.draw(&Text::new(
            format!("Nasenwinkel: {:+.1}°", angle),
            (30, 70),
            (FONT, 22).into_font().color(&RGBColor(220, 20, 20)),
        )).map_err(|e| anyhow::anyhow!("angle_txt: {e:?}"))?;
    }
    let tm = (t_now / 60.0) as u32;
    let ts = ((t_now % 60.0) as u32).min(59);
    let th = ((t_now * 100.0) as u32 % 100).min(99);
    board_area.draw(&Text::new(
        format!("Zeit: {}:{:02}.{:02}", tm, ts, th),
        (bw as i32 - 200, 40),
        (FONT, 20).into_font().color(&BLACK),
    )).map_err(|e| anyhow::anyhow!("time_txt: {e:?}"))?;
    if show_3d_now {
        // Pump counter — sum of dynamic-acc peaks gated on speed
        // > 4 km/h. Hidden during carry so it doesn't read as
        // "0 pumps" while the rider is just walking with the board.
        board_area.draw(&Text::new(
            format!("Pumps: {}", pump_count),
            (bw as i32 - 180, 70),
            (FONT, 22).into_font().color(&RGBColor(34, 102, 170)),
        )).map_err(|e| anyhow::anyhow!("pump_txt: {e:?}"))?;
    }

    // Drop-in flash
    if t_now >= drop_time && t_now <= drop_flash_end {
        let fade = (1.0 - (t_now - drop_time) / 2.0).max(0.0);
        if fade > 0.02 {
            let red = RGBColor(220, 20, 20).mix(fade);
            board_area.draw(&Text::new(
                format!("Push-off-Winkel: {:.1}°", drop_angle),
                (bw as i32 / 2 - 160, 90),
                (FONT, 32).into_font().color(&red),
            )).map_err(|e| anyhow::anyhow!("drop_txt: {e:?}"))?;
        }
    }

    // --- GPS-track map panel (right half of top row) ---
    if let (Some(area), Some((lats, lons, ts))) = (&map_area, map) {
        if !lats.is_empty() {
            let (m_title, m_plot) = area.split_vertically(40);
            m_title.draw(&Text::new(
                "GPS-Track".to_string(),
                (12, 4),
                (FONT, 22).into_font().color(&BLACK),
            )).map_err(|e| anyhow::anyhow!("map title: {e:?}"))?;

            // Equirectangular projection scaled by cos(median_lat) so
            // 1° east covers the same on-screen distance as 1° north.
            // Without this Greece (37 °N) would show ~1.25× horizontal
            // stretch.
            let mut lats_sorted = lats.to_vec();
            lats_sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let med_lat = lats_sorted[lats_sorted.len() / 2];
            let cos_lat = (med_lat * std::f64::consts::PI / 180.0).cos();

            let xs: Vec<f64> = lons.iter().map(|l| l * cos_lat).collect();
            let ys: Vec<f64> = lats.to_vec();

            let x_min = xs.iter().cloned().fold(f64::INFINITY, f64::min);
            let x_max = xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            let y_min = ys.iter().cloned().fold(f64::INFINITY, f64::min);
            let y_max = ys.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            // Square the bounds so distances on map are equal in both
            // axes (the chart is roughly 560 × 280 px after margins,
            // but plotters' build_cartesian_2d will stretch the data
            // to fill regardless — equalising the data range matches
            // the plot pixel ratio approximately).
            let span_x = (x_max - x_min).max(1e-6);
            let span_y = (y_max - y_min).max(1e-6);
            let pad_x = span_x * 0.05;
            let pad_y = span_y * 0.05;

            let mut mc = ChartBuilder::on(&m_plot)
                .margin_top(2).margin_bottom(8).margin_left(10).margin_right(20)
                .x_label_area_size(0)
                .y_label_area_size(0)
                .build_cartesian_2d(
                    (x_min - pad_x)..(x_max + pad_x),
                    (y_min - pad_y)..(y_max + pad_y),
                )
                .map_err(|e| anyhow::anyhow!("map chart: {e:?}"))?;
            mc.configure_mesh()
                .disable_x_axis()
                .disable_y_axis()
                .light_line_style(RGBColor(235, 235, 235))
                .draw().map_err(|e| anyhow::anyhow!("map mesh: {e:?}"))?;

            // Find the GPS index closest to t_now → "current position".
            let mut cur_idx = 0usize;
            for (i, t) in ts.iter().enumerate() {
                if *t > t_now { break; }
                cur_idx = i;
            }

            // Track travelled so far: solid line up to current index.
            mc.draw_series(LineSeries::new(
                xs[..=cur_idx].iter().zip(ys[..=cur_idx].iter())
                    .map(|(x, y)| (*x, *y)),
                RGBColor(31, 119, 180).stroke_width(3),
            )).map_err(|e| anyhow::anyhow!("track line: {e:?}"))?;

            // Current position dot.
            mc.draw_series(std::iter::once(Circle::new(
                (xs[cur_idx], ys[cur_idx]),
                7,
                RGBColor(220, 20, 20).filled(),
            ))).map_err(|e| anyhow::anyhow!("track dot: {e:?}"))?;
        }
    }

    // --- Phase boundaries (used as background-shaded bands across all
    // time-series panels) ---
    let x_right = t_now + 2.0;
    let xr = x_right.max(1.0);
    let p1_x = water_set_t.map(|t| t.clamp(0.0, xr)).unwrap_or(0.0);
    let p2_x = foil_start_t.map(|t| t.clamp(0.0, xr)).unwrap_or(p1_x);
    // Subtle pastels: gray (carry) → yellow (push/run) → green (foiling).
    let phase_carry = RGBColor(225, 225, 225).mix(0.55);
    let phase_run   = RGBColor(255, 230, 120).mix(0.55);
    let phase_foil  = RGBColor(170, 230, 170).mix(0.55);

    // --- Zoom panel (auto-fit) ---
    let (zoom_title, zoom_plot) = zoom_area.split_vertically(40);
    zoom_title.draw(&Text::new(
        format!("Pump-Detail [°] (±{:.0}°)", zoom_lim),
        (12, 4),
        (FONT, 22).into_font().color(&BLACK),
    )).map_err(|e| anyhow::anyhow!("zoom title: {e:?}"))?;
    let mut zc = ChartBuilder::on(&zoom_plot)
        .margin_top(2).margin_bottom(8).margin_left(10).margin_right(20)
        .x_label_area_size(30)
        .y_label_area_size(60)
        .build_cartesian_2d(0.0f64..xr, -zoom_lim..zoom_lim)
        .map_err(|e| anyhow::anyhow!("zoom chart: {e:?}"))?;
    zc.configure_mesh()
        .light_line_style(RGBColor(240, 240, 240))
        .draw().map_err(|e| anyhow::anyhow!("zoom mesh: {e:?}"))?;
    // Phase backgrounds
    zc.draw_series(std::iter::once(Rectangle::new(
        [(0.0, -zoom_lim), (p1_x, zoom_lim)], phase_carry.filled(),
    ))).ok();
    zc.draw_series(std::iter::once(Rectangle::new(
        [(p1_x, -zoom_lim), (p2_x, zoom_lim)], phase_run.filled(),
    ))).ok();
    zc.draw_series(std::iter::once(Rectangle::new(
        [(p2_x, -zoom_lim), (xr, zoom_lim)], phase_foil.filled(),
    ))).ok();
    zc.draw_series(std::iter::once(PathElement::new(
        vec![(0.0, 0.0), (xr, 0.0)],
        RGBColor(128, 128, 128).stroke_width(1),
    ))).map_err(|e| anyhow::anyhow!("zero: {e:?}"))?;
    zc.draw_series(LineSeries::new(
        t_hist.iter().zip(nose_hist.iter()).map(|(&t, &n)| (t, n.clamp(-zoom_lim, zoom_lim))),
        RGBColor(44, 160, 44).stroke_width(2),
    )).map_err(|e| anyhow::anyhow!("zoom line: {e:?}"))?;
    zc.draw_series(std::iter::once(PathElement::new(
        vec![(t_now, -zoom_lim), (t_now, zoom_lim)],
        RGBColor(220, 20, 20).stroke_width(2),
    ))).map_err(|e| anyhow::anyhow!("cursor: {e:?}"))?;
    // Current value label at top of panel, just right of the cursor.
    if let Some(val) = nose_hist.iter().rev().find(|v| v.is_finite()).copied() {
        let plotted = val.clamp(-zoom_lim, zoom_lim);
        zc.draw_series(std::iter::once(Circle::new(
            (t_now, plotted), 4, RGBColor(220, 20, 20).filled(),
        ))).ok();
        zc.draw_series(std::iter::once(Text::new(
            format!("{:+.1}°", val),
            (t_now + 0.2, zoom_lim * 0.78),
            (FONT, 18).into_font().color(&RGBColor(220, 20, 20)),
        ))).ok();
    }

    // --- Height-above-water panel (baro, GPS-anchored, TC-corrected) ---
    if let (Some(area), Some(h_data)) = (&height_area, height_hist) {
        let (h_title, h_plot) = area.split_vertically(40);
        h_title.draw(&Text::new(
            "Höhe über Wasser [m] · Mast 0.80 m".to_string(),
            (12, 4),
            (FONT, 22).into_font().color(&BLACK),
        )).map_err(|e| anyhow::anyhow!("height title: {e:?}"))?;
        // Auto-fit y-axis to actual data range — no headroom above max.
        // Out-of-physical-range values (>0.95 m or <−0.15 m) are still
        // NaN'd in the trace itself.
        let mut hc = ChartBuilder::on(&h_plot)
            .margin_top(2).margin_bottom(8).margin_left(10).margin_right(20)
            .x_label_area_size(30)
            .y_label_area_size(60)
            .build_cartesian_2d(0.0f64..x_right.max(1.0), height_bot..height_top)
            .map_err(|e| anyhow::anyhow!("height chart: {e:?}"))?;
        hc.configure_mesh()
            .light_line_style(RGBColor(240, 240, 240))
            .draw().map_err(|e| anyhow::anyhow!("height mesh: {e:?}"))?;
        // Phase backgrounds
        hc.draw_series(std::iter::once(Rectangle::new(
            [(0.0, height_bot), (p1_x, height_top)], phase_carry.filled(),
        ))).ok();
        hc.draw_series(std::iter::once(Rectangle::new(
            [(p1_x, height_bot), (p2_x, height_top)], phase_run.filled(),
        ))).ok();
        hc.draw_series(std::iter::once(Rectangle::new(
            [(p2_x, height_bot), (xr, height_top)], phase_foil.filled(),
        ))).ok();
        // Mast reference at 0.80 m
        hc.draw_series(std::iter::once(PathElement::new(
            vec![(0.0, 0.80), (xr, 0.80)],
            RGBColor(200, 0, 0).mix(0.5).stroke_width(1),
        ))).map_err(|e| anyhow::anyhow!("mast ref: {e:?}"))?;
        hc.draw_series(LineSeries::new(
            t_hist.iter().zip(h_data.iter()).filter_map(|(&t, &h)| {
                // Only filter NaN; let height_top auto-fit handle
                // any thermal-drift excursions visually.
                if h.is_finite() { Some((t, h)) } else { None }
            }),
            RGBColor(44, 160, 44).stroke_width(2),
        )).map_err(|e| anyhow::anyhow!("height line: {e:?}"))?;
        hc.draw_series(std::iter::once(PathElement::new(
            vec![(t_now, -0.1), (t_now, 0.9)],
            RGBColor(220, 20, 20).stroke_width(2),
        ))).map_err(|e| anyhow::anyhow!("height cursor: {e:?}"))?;
        // Current value label at top of panel, just right of the cursor.
        if let Some(val) = h_data.iter().rev().find(|v| v.is_finite()).copied() {
            let plotted = val.clamp(height_bot, height_top);
            hc.draw_series(std::iter::once(Circle::new(
                (t_now, plotted), 4, RGBColor(220, 20, 20).filled(),
            ))).ok();
            let span = (height_top - height_bot).max(0.1);
            hc.draw_series(std::iter::once(Text::new(
                format!("{:.2} m", val),
                (t_now + 0.2, height_top - span * 0.12),
                (FONT, 18).into_font().color(&RGBColor(220, 20, 20)),
            ))).ok();
        }
    }

    // --- Speed panel (km/h, GPS-derived, smoothed) ---
    if let (Some(area), Some(v_data)) = (&speed_area, speed_hist) {
        let (v_title, v_plot) = area.split_vertically(40);
        v_title.draw(&Text::new(
            "Geschwindigkeit [km/h]".to_string(),
            (12, 4),
            (FONT, 22).into_font().color(&BLACK),
        )).map_err(|e| anyhow::anyhow!("speed title: {e:?}"))?;
        let mut vc = ChartBuilder::on(&v_plot)
            .margin_top(2).margin_bottom(8).margin_left(10).margin_right(20)
            .x_label_area_size(30)
            .y_label_area_size(60)
            .build_cartesian_2d(0.0f64..x_right.max(1.0), 0.0f64..speed_top)
            .map_err(|e| anyhow::anyhow!("speed chart: {e:?}"))?;
        vc.configure_mesh()
            .light_line_style(RGBColor(240, 240, 240))
            .draw().map_err(|e| anyhow::anyhow!("speed mesh: {e:?}"))?;
        // Phase backgrounds
        vc.draw_series(std::iter::once(Rectangle::new(
            [(0.0, 0.0), (p1_x, speed_top)], phase_carry.filled(),
        ))).ok();
        vc.draw_series(std::iter::once(Rectangle::new(
            [(p1_x, 0.0), (p2_x, speed_top)], phase_run.filled(),
        ))).ok();
        vc.draw_series(std::iter::once(Rectangle::new(
            [(p2_x, 0.0), (xr, speed_top)], phase_foil.filled(),
        ))).ok();
        // Phase labels above each band, centred on the band's x-mid.
        let lbl_y = speed_top * 0.92;
        if p1_x > 1.0 {
            vc.draw_series(std::iter::once(Text::new(
                "Tragen", (p1_x * 0.5, lbl_y),
                (FONT, 14).into_font().color(&RGBColor(80, 80, 80)),
            ))).ok();
        }
        if p2_x - p1_x > 1.0 {
            vc.draw_series(std::iter::once(Text::new(
                "Rennen", ((p1_x + p2_x) * 0.5, lbl_y),
                (FONT, 14).into_font().color(&RGBColor(120, 90, 0)),
            ))).ok();
        }
        if xr - p2_x > 1.0 {
            vc.draw_series(std::iter::once(Text::new(
                "Foilen", ((p2_x + xr) * 0.5, lbl_y),
                (FONT, 14).into_font().color(&RGBColor(20, 100, 20)),
            ))).ok();
        }
        vc.draw_series(LineSeries::new(
            t_hist.iter().zip(v_data.iter()).filter_map(|(&t, &v)| {
                if v.is_finite() { Some((t, v)) } else { None }
            }),
            RGBColor(214, 39, 40).stroke_width(2),
        )).map_err(|e| anyhow::anyhow!("speed line: {e:?}"))?;
        vc.draw_series(std::iter::once(PathElement::new(
            vec![(t_now, 0.0), (t_now, speed_top)],
            RGBColor(220, 20, 20).stroke_width(2),
        ))).map_err(|e| anyhow::anyhow!("speed cursor: {e:?}"))?;
        // Current value label at top of panel, just right of the cursor.
        // Y position is below the phase labels (which sit at speed_top * 0.92)
        // so the value badge doesn't collide with them.
        if let Some(val) = v_data.iter().rev().find(|v| v.is_finite()).copied() {
            let plotted = val.clamp(0.0, speed_top);
            vc.draw_series(std::iter::once(Circle::new(
                (t_now, plotted), 4, RGBColor(220, 20, 20).filled(),
            ))).ok();
            vc.draw_series(std::iter::once(Text::new(
                format!("{:.1} km/h", val),
                (t_now + 0.2, speed_top * 0.78),
                (FONT, 18).into_font().color(&RGBColor(220, 20, 20)),
            ))).ok();
        }
    }

    // --- Full-range panel: absolute orientation pitch (un-detrended) ---
    let bottom_label = if nose_abs_hist.is_some() {
        "Brett-Pitch absolut [°]"
    } else {
        "Nasenwinkel [°] (detrended)"
    };
    let (g_title, g_plot) = graph_area.split_vertically(40);
    g_title.draw(&Text::new(
        bottom_label.to_string(),
        (12, 4),
        (FONT, 22).into_font().color(&BLACK),
    )).map_err(|e| anyhow::anyhow!("graph title: {e:?}"))?;
    let mut gc = ChartBuilder::on(&g_plot)
        .margin_top(2).margin_bottom(8).margin_left(10).margin_right(20)
        .x_label_area_size(40)
        .y_label_area_size(60)
        .build_cartesian_2d(0.0f64..x_right.max(1.0), -y_lim..y_lim)
        .map_err(|e| anyhow::anyhow!("graph chart: {e:?}"))?;
    gc.configure_mesh()
        .x_desc("Zeit [s]")
        .light_line_style(RGBColor(240, 240, 240))
        .draw().map_err(|e| anyhow::anyhow!("graph mesh: {e:?}"))?;
    // Phase backgrounds
    gc.draw_series(std::iter::once(Rectangle::new(
        [(0.0, -y_lim), (p1_x, y_lim)], phase_carry.filled(),
    ))).ok();
    gc.draw_series(std::iter::once(Rectangle::new(
        [(p1_x, -y_lim), (p2_x, y_lim)], phase_run.filled(),
    ))).ok();
    gc.draw_series(std::iter::once(Rectangle::new(
        [(p2_x, -y_lim), (xr, y_lim)], phase_foil.filled(),
    ))).ok();
    gc.draw_series(std::iter::once(PathElement::new(
        vec![(0.0, 0.0), (xr, 0.0)],
        RGBColor(128, 128, 128).stroke_width(1),
    ))).map_err(|e| anyhow::anyhow!("zero: {e:?}"))?;
    let abs_or_corrected: &[f64] = nose_abs_hist.unwrap_or(nose_hist);
    gc.draw_series(LineSeries::new(
        t_hist.iter().zip(abs_or_corrected.iter()).map(|(&t, &n)| (t, n)),
        RGBColor(31, 119, 180).stroke_width(2),
    )).map_err(|e| anyhow::anyhow!("graph line: {e:?}"))?;
    gc.draw_series(std::iter::once(PathElement::new(
        vec![(t_now, -y_lim), (t_now, y_lim)],
        RGBColor(220, 20, 20).stroke_width(2),
    ))).map_err(|e| anyhow::anyhow!("cursor: {e:?}"))?;
    // Current value label at top of panel, just right of the cursor.
    if let Some(val) = abs_or_corrected.iter().rev().find(|v| v.is_finite()).copied() {
        let plotted = val.clamp(-y_lim, y_lim);
        gc.draw_series(std::iter::once(Circle::new(
            (t_now, plotted), 4, RGBColor(220, 20, 20).filled(),
        ))).ok();
        gc.draw_series(std::iter::once(Text::new(
            format!("{:+.1}°", val),
            (t_now + 0.2, y_lim * 0.78),
            (FONT, 18).into_font().color(&RGBColor(220, 20, 20)),
        ))).ok();
    }

    Ok(())
}

fn combine_with_ffmpeg(
    gif_path: &PathBuf,
    video_path: &Path,
    video_offset: f64,
    sensor_offset: f64,
    out_path: &PathBuf,
    title: Option<&str>,
    subtitle: Option<&str>,
    fps: u32,
) -> Result<()> {
    // Main clip = camera (left) + GIF (right), scaled to 900px tall.
    let probe = Command::new("ffprobe")
        .args(["-v", "error", "-show_entries", "format=duration",
               "-of", "default=noprint_wrappers=1:nokey=1"])
        .arg(video_path)
        .output()
        .with_context(|| "running ffprobe — is it in PATH?")?;
    let total_s: f64 = String::from_utf8_lossy(&probe.stdout).trim().parse()
        .with_context(|| "parse video duration")?;
    let ride_s = (total_s - video_offset).max(1.0);

    let tmp = tempfile::tempdir()?;
    let main_clip = tmp.path().join("main.mov");
    let status = Command::new("ffmpeg")
        .args(["-y",
               "-ss", &video_offset.to_string(), "-i", video_path.to_str().unwrap(),
               "-ss", &sensor_offset.to_string(), "-i", gif_path.to_str().unwrap(),
               "-filter_complex",
               "[0:v]scale=-1:900[vid];[1:v]scale=-1:900[gif];[vid][gif]hstack=inputs=2",
               "-c:v", "libx264", "-pix_fmt", "yuv420p",
               "-r", &fps.to_string(),
               "-t", &ride_s.to_string(), "-an",
               main_clip.to_str().unwrap()])
        .status()
        .with_context(|| "running ffmpeg for main clip")?;
    if !status.success() { anyhow::bail!("ffmpeg main clip failed"); }

    if title.is_some() || subtitle.is_some() {
        // Probe combined dims
        let probe2 = Command::new("ffprobe")
            .args(["-v", "error", "-select_streams", "v:0",
                   "-show_entries", "stream=width,height",
                   "-of", "csv=p=0"])
            .arg(&main_clip)
            .output()
            .with_context(|| "ffprobe combined")?;
        let dims = String::from_utf8_lossy(&probe2.stdout);
        let mut it = dims.trim().split(',');
        let w: u32 = it.next().unwrap_or("1920").parse().unwrap_or(1920);
        let h: u32 = it.next().unwrap_or("900").parse().unwrap_or(900);

        let title_png = tmp.path().join("title.png");
        render_title_frame(w, h, title, subtitle, &title_png)?;
        let title_clip = tmp.path().join("title.mov");
        Command::new("ffmpeg")
            .args(["-y", "-loop", "1", "-i", title_png.to_str().unwrap(),
                   "-t", "2", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                   "-vf", &format!("scale={}:{}", w, h), "-r", &fps.to_string(),
                   title_clip.to_str().unwrap()])
            .status().with_context(|| "ffmpeg title clip")?;

        let concat_txt = tmp.path().join("concat.txt");
        std::fs::write(&concat_txt, format!("file '{}'\nfile '{}'\n",
            title_clip.display(), main_clip.display()))?;
        Command::new("ffmpeg")
            .args(["-y", "-f", "concat", "-safe", "0",
                   "-i", concat_txt.to_str().unwrap(),
                   "-c:v", "libx264", "-pix_fmt", "yuv420p",
                   "-movflags", "+faststart",
                   out_path.to_str().unwrap()])
            .status().with_context(|| "ffmpeg concat")?;
    } else {
        Command::new("ffmpeg")
            .args(["-y", "-i", main_clip.to_str().unwrap(),
                   "-c:v", "copy", "-movflags", "+faststart",
                   out_path.to_str().unwrap()])
            .status().with_context(|| "ffmpeg copy")?;
    }

    Ok(())
}

fn render_title_frame(
    w: u32, h: u32, title: Option<&str>, subtitle: Option<&str>, out: &PathBuf,
) -> Result<()> {
    let root = BitMapBackend::new(out, (w, h)).into_drawing_area();
    root.fill(&BLACK)?;
    if let Some(t) = title {
        let y = if subtitle.is_some() { h as i32 * 45 / 100 } else { h as i32 / 2 - 30 };
        let approx_w = t.len() as i32 * 28;
        root.draw(&Text::new(
            t.to_string(),
            (w as i32 / 2 - approx_w / 2, y),
            (FONT, 56).into_font().color(&RGBColor(0, 255, 0)),
        ))?;
    }
    if let Some(st) = subtitle {
        let approx_w = st.len() as i32 * 20;
        root.draw(&Text::new(
            st.to_string(),
            (w as i32 / 2 - approx_w / 2, h as i32 * 60 / 100),
            (FONT, 40).into_font().color(&RGBColor(135, 206, 250)),
        ))?;
    }
    root.present()?;
    Ok(())
}

// Minimal tempfile substitute (we don't need the full crate).
mod tempfile {
    use std::path::PathBuf;
    pub struct TempDir { path: PathBuf }
    impl TempDir {
        pub fn path(&self) -> &std::path::Path { &self.path }
    }
    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.path);
        }
    }
    pub fn tempdir() -> std::io::Result<TempDir> {
        let mut p = std::env::temp_dir();
        let pid = std::process::id();
        let ts = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos()).unwrap_or(0);
        p.push(format!("stbox-viz-{}-{}", pid, ts));
        std::fs::create_dir_all(&p)?;
        Ok(TempDir { path: p })
    }
}
// touch 1777116091
// touch 1777116125810961000
// 1777117177953306000
// 1777117609989703000
// 1777119994656819000
// 1777122036278705000
// rebuild 1777122077310373000
// rebuild 1777122255017631000
// rebuild 1777122374250603000
// rebuild 1777124058384934000
// rebuild 1777124194859005000
// rebuild 1777124232926942000
// rebuild 1777124477331970000
// rebuild 1777124858733054000
// rebuild 1777125293256443000
// rebuild 1777125736311109000
// 1777125756543100000
// rebuild 1777126065139384000
// rebuild 1777126856557418000
// rebuild 1777127224116348000
// rebuild 1777127360631995000
// rebuild 1777128088273981000
