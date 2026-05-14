//! GPS post-processing: position-derived speed (haversine) and
//! sustained-movement ride detection.
//!
//! The u-blox MAX-M10S Doppler-based Speed column in GpsNNN.csv is not
//! reliable on this hardware (observed median 0.12 km/h while position
//! deltas showed sustained 10–30 km/h flight). All consumers should use
//! the position-derived speed.

use crate::io::GpsRow;

const EARTH_RADIUS_M: f64 = 6_371_000.0;
pub const TICKS_PER_SEC: f64 = 100.0; // 1 tick = 10 ms

pub fn haversine_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (lat1r, lat2r) = (lat1.to_radians(), lat2.to_radians());
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let a = (dlat / 2.0).sin().powi(2)
        + lat1r.cos() * lat2r.cos() * (dlon / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_M * a.sqrt().asin()
}

/// Position-derived speed per row (km/h). The first row gets 0.
pub fn position_derived_speed_kmh(gps: &[GpsRow]) -> Vec<f64> {
    let n = gps.len();
    let mut out = vec![0.0; n];
    for i in 1..n {
        let dist = haversine_m(gps[i - 1].lat, gps[i - 1].lon, gps[i].lat, gps[i].lon);
        let dt = (gps[i].ticks - gps[i - 1].ticks) / TICKS_PER_SEC;
        if dt > 0.05 {
            out[i] = (dist / dt) * 3.6;
        }
    }
    out
}

/// Reject unphysical jumps by comparing |Δspeed| against a maximum
/// plausible longitudinal acceleration. Pumpfoil/SUPfoil paddle-starts
/// produce ~1–3 m/s² over a stroke — anything above ~4 m/s² is almost
/// certainly a multipath-induced position jump rather than real motion
/// (on a 1 Hz fix interval, 15 km/h/s = 4.16 m/s² = 4.16 m position
/// delta over 1 s, which is inside the module's ~5–10 m horizontal
/// error envelope).
///
/// Returns a copy of `raw` with glitch samples set to NaN. Keeps the
/// previous valid sample as the "baseline" for the next comparison so
/// a glitch doesn't poison everything downstream of it.
pub fn reject_acc_outliers(
    gps: &[GpsRow],
    raw_kmh: &[f64],
    max_accel_kmh_per_s: f64,
) -> Vec<f64> {
    let n = raw_kmh.len();
    let mut out = raw_kmh.to_vec();
    if n == 0 { return out; }

    let mut prev_t: Option<f64> = None;
    let mut prev_v: Option<f64> = None;
    for i in 0..n {
        let t = gps[i].ticks / TICKS_PER_SEC;
        let v = out[i];
        if let (Some(tp), Some(vp)) = (prev_t, prev_v) {
            let dt = (t - tp).max(0.05);
            let accel = (v - vp).abs() / dt;
            // Only flag high-speed jumps — a Δspeed from 0 to 10 km/h in
            // 1 s is 10 km/h/s (2.8 m/s²), plausible for a paddle stroke.
            // The pathological case is Δspeed from 5 to 35 km/h in 1 s.
            if accel > max_accel_kmh_per_s && v > 15.0 {
                out[i] = f64::NAN;
                continue;
            }
        }
        prev_t = Some(t);
        prev_v = Some(v);
    }
    out
}

/// Clamp implausible multipath glitches (>60 km/h on a pumpfoil is always
/// a bad fix) and NaN-marked acceleration outliers, linearly interpolate
/// the rejected values, then apply a 5-sample rolling median to absorb
/// remaining single-sample spikes.
pub fn smooth_speed_kmh(raw: &[f64]) -> Vec<f64> {
    let mut clipped: Vec<Option<f64>> = raw.iter()
        .map(|&v| if !v.is_finite() || v > 60.0 { None } else { Some(v) })
        .collect();
    linear_interpolate(&mut clipped);
    let interpolated: Vec<f64> = clipped.iter().map(|o| o.unwrap_or(0.0)).collect();
    rolling_median(&interpolated, 5)
}

/// In-place linear interpolation of `None` values using surrounding `Some`.
/// Leading/trailing Nones copy the nearest non-None; if everything is None,
/// fills with 0.
fn linear_interpolate(v: &mut [Option<f64>]) {
    let n = v.len();
    if n == 0 {
        return;
    }
    // Forward pass: fill trailing None with last Some
    let mut last: Option<(usize, f64)> = None;
    let mut i = 0;
    while i < n {
        if let Some(val) = v[i] {
            // If we had a gap before this, linearly interpolate over it
            if let Some((pi, pv)) = last {
                let span = i - pi;
                for k in (pi + 1)..i {
                    let t = (k - pi) as f64 / span as f64;
                    v[k] = Some(pv + (val - pv) * t);
                }
            }
            last = Some((i, val));
        }
        i += 1;
    }
    // Backfill leading None
    if let Some((fi, fv)) = (0..n).find_map(|k| v[k].map(|x| (k, x))) {
        for k in 0..fi {
            v[k] = Some(fv);
        }
    } else {
        // All None
        for slot in v.iter_mut() {
            *slot = Some(0.0);
        }
        return;
    }
    // Forward-fill trailing None
    let mut last_val = 0.0;
    for slot in v.iter_mut() {
        if let Some(x) = slot {
            last_val = *x;
        } else {
            *slot = Some(last_val);
        }
    }
}

fn rolling_median(x: &[f64], window: usize) -> Vec<f64> {
    let n = x.len();
    let w = window.max(1);
    let half = w / 2;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let lo = i.saturating_sub(half);
        let hi = (i + half + 1).min(n);
        let mut buf: Vec<f64> = x[lo..hi].to_vec();
        buf.sort_by(|a, b| a.partial_cmp(b).unwrap());
        out.push(buf[buf.len() / 2]);
    }
    out
}

/// A detected ride over water.
#[derive(Debug, Clone)]
#[allow(dead_code)] // gps_end/sensor_end used in per-session rendering (phase 2)
pub struct Ride {
    /// Index into the GPS row array where the ride starts.
    pub gps_start: usize,
    /// Index into the GPS row array where the ride ends (exclusive).
    pub gps_end: usize,
    /// Sensor-sample index of start (aligned to sensor timeline).
    pub sensor_start: usize,
    /// Sensor-sample index of end.
    pub sensor_end: usize,
    /// 90th percentile speed during the ride (km/h).
    pub p90_kmh: f64,
    /// UTC hhmmss.ss at ride start (from GPS).
    pub utc_start: String,
    /// Duration in seconds.
    pub duration_s: f64,
}

/// Detect rides from GPS speed: sustained movement above `speed_threshold_kmh`
/// for at least `min_run_s` seconds, merging gaps shorter than `merge_gap_s`,
/// padded by `pad_s` on each side.
pub fn detect_rides(
    gps: &[GpsRow],
    speed_kmh: &[f64],
    sensor_n_samples: usize,
    sample_hz: usize,
    base_ticks: f64,
    speed_threshold_kmh: f64,
    min_run_s: f64,
    merge_gap_s: f64,
    pad_s: f64,
) -> Vec<Ride> {
    if gps.is_empty() {
        return vec![];
    }

    // Build second-granular active/inactive series
    let last_ticks = gps.last().map(|g| g.ticks).unwrap_or(0.0);
    let total_s = ((last_ticks - base_ticks) / TICKS_PER_SEC).max(0.0) as usize + 1;
    let mut active = vec![false; total_s];
    for (row, spd) in gps.iter().zip(speed_kmh.iter()) {
        let s = ((row.ticks - base_ticks) / TICKS_PER_SEC).round() as isize;
        if s >= 0 && (s as usize) < total_s && *spd >= speed_threshold_kmh {
            active[s as usize] = true;
        }
    }

    // Find runs of active seconds
    let mut runs: Vec<(usize, usize)> = Vec::new();
    let mut i = 0;
    while i < total_s {
        if active[i] {
            let start = i;
            while i < total_s && active[i] {
                i += 1;
            }
            runs.push((start, i));
        } else {
            i += 1;
        }
    }

    // Merge gaps < merge_gap_s
    let mut merged: Vec<(usize, usize)> = Vec::new();
    for (s, e) in runs {
        if let Some(last) = merged.last_mut() {
            if (s as f64) - (last.1 as f64) < merge_gap_s {
                last.1 = e;
                continue;
            }
        }
        merged.push((s, e));
    }

    // Filter min_run_s, pad, and clip to session bounds
    let pad = pad_s as usize;
    let min_run = min_run_s as usize;
    let mut rides: Vec<Ride> = Vec::new();
    for (s, e) in merged {
        if e - s < min_run {
            continue;
        }
        let s_pad = s.saturating_sub(pad);
        let e_pad = (e + pad).min(total_s);

        // Find the GPS row indices corresponding to these seconds
        let gps_start = gps
            .iter()
            .position(|g| (g.ticks - base_ticks) / TICKS_PER_SEC >= s_pad as f64)
            .unwrap_or(0);
        let gps_end = gps
            .iter()
            .rposition(|g| (g.ticks - base_ticks) / TICKS_PER_SEC < e_pad as f64)
            .map(|i| i + 1)
            .unwrap_or(gps.len());

        // 90th percentile speed within the ride
        let mut speeds: Vec<f64> = speed_kmh[gps_start..gps_end].to_vec();
        speeds.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let p90 = if speeds.is_empty() {
            0.0
        } else {
            speeds[(speeds.len() as f64 * 0.9) as usize]
        };

        // Sensor-sample index alignment (samples are at sample_hz)
        let sensor_start = (s_pad * sample_hz).min(sensor_n_samples);
        let sensor_end = (e_pad * sample_hz).min(sensor_n_samples);

        let utc_start = gps.get(gps_start).map(|g| g.utc.clone()).unwrap_or_default();

        rides.push(Ride {
            gps_start,
            gps_end,
            sensor_start,
            sensor_end,
            p90_kmh: p90,
            utc_start,
            duration_s: (e_pad - s_pad) as f64,
        });
    }
    rides
}
