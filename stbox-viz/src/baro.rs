//! Board height above water via GPS-anchored water reference + temperature
//! compensation. See the matching Python `height_above_water_m` docstring
//! in `Utilities/scripts/visualize_combined.py` for why this approach is
//! needed on this hardware.

use crate::gps::TICKS_PER_SEC;
use crate::io::{GpsRow, SensorRow};

const KELVIN_OFFSET: f64 = 273.15;
/// Hypsometric coefficient near sea level — `dh ≈ −8434 × dP/P` is
/// accurate to ~1 % for heights < 100 m, vastly better than the full
/// international formula for this use case (we only look at Δalt of
/// a few metres).
const HYPSOMETRIC_H: f64 = 8434.0;
const STATIONARY_THRESHOLD_KMH: f64 = 3.0;

/// Compute height above water, aligned 1:1 with the sensor rows.
/// If `gps`/`speed_kmh` are empty, falls back to a whole-session rolling
/// min of the TC'd altitude (still bad, but non-NaN).
pub fn height_above_water_m(
    sensors: &[SensorRow],
    gps: &[GpsRow],
    speed_kmh: &[f64],
    base_ticks: f64,
) -> Vec<f64> {
    if sensors.is_empty() {
        return vec![];
    }

    // Temperature compensation: P_tc = P × T_ref / T (Kelvin). Cancels the
    // ideal-gas coupling inside the semi-sealed SensorTile enclosure.
    let tk: Vec<f64> = sensors.iter().map(|s| s.temperature_c + KELVIN_OFFSET).collect();
    let mut sorted_tk = tk.clone();
    sorted_tk.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let ref_k = sorted_tk[sorted_tk.len() / 2];
    let press_tc: Vec<f64> = sensors.iter().zip(tk.iter())
        .map(|(s, t)| s.pressure_hpa * (ref_k / t))
        .collect();

    if gps.is_empty() || speed_kmh.is_empty() {
        // Fallback: session-max pressure as water reference.
        let mut pmax = f64::MIN;
        for &p in &press_tc {
            if p > pmax { pmax = p; }
        }
        return press_tc.iter().map(|p| HYPSOMETRIC_H * (1.0 - p / pmax)).collect();
    }

    // Sensor & GPS time axes in seconds.
    let sens_sec: Vec<f64> = sensors.iter().map(|s| (s.ticks - base_ticks) / TICKS_PER_SEC).collect();
    let gps_sec: Vec<f64> = gps.iter().map(|g| (g.ticks - base_ticks) / TICKS_PER_SEC).collect();

    // Step 1: interpolate TC'd pressure onto GPS time grid.
    let press_at_gps = interp_linear(&gps_sec, &sens_sec, &press_tc);

    // Step 2: anchor water reference where GPS speed < threshold.
    // Collect (gps_sec, press) pairs at stationary samples.
    let mut anchors: Vec<(f64, f64)> = Vec::new();
    for (i, &spd) in speed_kmh.iter().enumerate() {
        if spd < STATIONARY_THRESHOLD_KMH {
            anchors.push((gps_sec[i], press_at_gps[i]));
        }
    }

    if anchors.is_empty() {
        // All-flying session → fall back to max pressure reference (underestimates).
        let mut pmax = f64::MIN;
        for &p in &press_tc {
            if p > pmax { pmax = p; }
        }
        return press_tc.iter().map(|p| HYPSOMETRIC_H * (1.0 - p / pmax)).collect();
    }

    // Build a water-reference series on the GPS grid by linear-interpolating
    // through the anchor points. Edges extend with the nearest anchor value.
    let anchor_times: Vec<f64> = anchors.iter().map(|&(t, _)| t).collect();
    let anchor_press: Vec<f64> = anchors.iter().map(|&(_, p)| p).collect();
    let water_ref_gps = interp_linear(&gps_sec, &anchor_times, &anchor_press);

    // Step 3: map the water reference back to the sensor timeline.
    let water_ref_sens = interp_linear(&sens_sec, &gps_sec, &water_ref_gps);

    // Step 4: height = 8434 × (1 − P_tc / P_ref).
    press_tc.iter().zip(water_ref_sens.iter())
        .map(|(p, pref)| HYPSOMETRIC_H * (1.0 - p / pref))
        .collect()
}

/// Linearly interpolate `y(x)` onto `x_new`. Requires `x` be sorted
/// ascending; out-of-range queries clamp to edge values. O(n + m) via
/// a two-pointer walk.
pub fn interp_linear(x_new: &[f64], x: &[f64], y: &[f64]) -> Vec<f64> {
    assert_eq!(x.len(), y.len());
    let mut out = Vec::with_capacity(x_new.len());
    if x.is_empty() {
        return vec![0.0; x_new.len()];
    }
    if x.len() == 1 {
        return vec![y[0]; x_new.len()];
    }
    let mut j = 0;
    for &xn in x_new {
        if xn <= x[0] {
            out.push(y[0]);
            continue;
        }
        if xn >= *x.last().unwrap() {
            out.push(*y.last().unwrap());
            continue;
        }
        while j + 1 < x.len() && x[j + 1] < xn {
            j += 1;
        }
        // Now x[j] <= xn < x[j+1]
        let x0 = x[j];
        let x1 = x[j + 1];
        let t = if x1 > x0 { (xn - x0) / (x1 - x0) } else { 0.0 };
        out.push(y[j] + (y[j + 1] - y[j]) * t);
    }
    out
}
