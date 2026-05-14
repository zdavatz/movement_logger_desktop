//! Pumping-session detection from quaternion-derived Euler angles.
//!
//! A session is a sustained window of high pitch/roll activity with
//! oscillation frequency ≥ 0.3 Hz (the pumping cadence cutoff — below
//! that is walking on land). Port of the logic in
//! `Utilities/scripts/visualize_sensors.py::plot_quaternions`.

/// Rolling mean with centred window and min_periods=1.
fn rolling_mean(x: &[f64], window: usize) -> Vec<f64> {
    let n = x.len();
    let half = window / 2;
    let mut out = vec![0.0; n];
    for i in 0..n {
        let lo = i.saturating_sub(half);
        let hi = (i + half + 1).min(n);
        let slice = &x[lo..hi];
        out[i] = slice.iter().sum::<f64>() / slice.len() as f64;
    }
    out
}

fn median(x: &[f64]) -> f64 {
    let mut s: Vec<f64> = x.iter().copied().filter(|v| v.is_finite()).collect();
    if s.is_empty() { return 0.0; }
    s.sort_by(|a, b| a.partial_cmp(b).unwrap());
    s[s.len() / 2]
}

fn std_dev(x: &[f64]) -> f64 {
    let finite: Vec<f64> = x.iter().copied().filter(|v| v.is_finite()).collect();
    if finite.is_empty() { return 0.0; }
    let m = finite.iter().sum::<f64>() / finite.len() as f64;
    let var = finite.iter().map(|v| (v - m).powi(2)).sum::<f64>() / finite.len() as f64;
    var.sqrt()
}

/// Detect pumping sessions from the full quaternion-derived pitch/roll
/// series. Returns (start_sample, end_sample) pairs, half-open.
pub fn detect_sessions(pitch: &[f64], roll: &[f64], sample_hz: usize) -> Vec<(usize, usize)> {
    if pitch.is_empty() || pitch.len() != roll.len() {
        return vec![];
    }
    let n = pitch.len();

    // euler_change[i] = |Δpitch| + |Δroll| (prepending row 0)
    let mut ec = Vec::with_capacity(n);
    ec.push(0.0);
    for i in 1..n {
        ec.push((pitch[i] - pitch[i - 1]).abs() + (roll[i] - roll[i - 1]).abs());
    }

    // 1-second rolling mean activity
    let activity = rolling_mean(&ec, sample_hz);

    let threshold = median(&activity) + std_dev(&activity);

    // Boolean active mask, find rising/falling edges
    let active: Vec<bool> = activity.iter().map(|&a| a > threshold).collect();

    let mut segments: Vec<(usize, usize)> = Vec::new();
    let mut run_start: Option<usize> = None;
    for (i, &a) in active.iter().enumerate() {
        match (a, run_start) {
            (true, None) => run_start = Some(i),
            (false, Some(s)) => {
                segments.push((s, i));
                run_start = None;
            }
            _ => {}
        }
    }
    if let Some(s) = run_start {
        segments.push((s, n));
    }

    // Merge segments with gaps < 60 seconds
    let merge_gap = 60 * sample_hz;
    let mut merged: Vec<(usize, usize)> = Vec::new();
    for (s, e) in segments {
        if let Some(last) = merged.last_mut() {
            if s - last.1 < merge_gap {
                last.1 = e;
                continue;
            }
        }
        merged.push((s, e));
    }

    // Filter short bursts (< 30 s) and walking (oscillation < 0.3 Hz)
    let min_duration = 30 * sample_hz;
    merged.into_iter()
        .filter(|&(s, e)| e - s >= min_duration)
        .filter(|&(s, e)| {
            let p = &pitch[s..e];
            let med = median(p);
            let mut crossings = 0usize;
            let mut prev_sign = (p[0] - med).signum();
            for &v in &p[1..] {
                let cur = (v - med).signum();
                if cur != 0.0 && prev_sign != 0.0 && cur != prev_sign {
                    crossings += 1;
                }
                if cur != 0.0 {
                    prev_sign = cur;
                }
            }
            let duration_s = (e - s) as f64 / sample_hz as f64;
            let osc_hz = crossings as f64 / duration_s / 2.0;
            osc_hz >= 0.3
        })
        .collect()
}
