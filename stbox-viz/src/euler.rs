//! Quaternion → Euler angle conversion (Hamilton convention, ZYX order).
//!
//! Takes `[w, x, y, z]` = `[qs, qi, qj, qk]` and returns (roll, pitch, yaw)
//! in degrees. Pitch is clamped to ±90° in the gimbal-lock region; the
//! caller is expected to mask those regions in any downstream visualisation.

pub type Quat = [f64; 4];

/// Returns (roll_deg, pitch_deg, yaw_deg) for the full time series.
pub fn quats_to_euler_deg(quats: &[Quat]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let n = quats.len();
    let mut roll = Vec::with_capacity(n);
    let mut pitch = Vec::with_capacity(n);
    let mut yaw = Vec::with_capacity(n);
    for q in quats {
        let (r, p, y) = quat_to_euler_deg(q);
        roll.push(r);
        pitch.push(p);
        yaw.push(y);
    }
    (roll, pitch, yaw)
}

pub fn quat_to_euler_deg(q: &Quat) -> (f64, f64, f64) {
    let (qs, qi, qj, qk) = (q[0], q[1], q[2], q[3]);

    let sinr_cosp = 2.0 * (qs * qi + qj * qk);
    let cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj);
    let roll = sinr_cosp.atan2(cosr_cosp).to_degrees();

    let sinp = 2.0 * (qs * qj - qk * qi);
    let pitch = if sinp.abs() >= 1.0 {
        (sinp.signum() * std::f64::consts::FRAC_PI_2).to_degrees()
    } else {
        sinp.asin().to_degrees()
    };

    let siny_cosp = 2.0 * (qs * qk + qi * qj);
    let cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk);
    let yaw = siny_cosp.atan2(cosy_cosp).to_degrees();

    (roll, pitch, yaw)
}

/// Contiguous gimbal-lock regions (|pitch| > 85°) as (start_idx, end_idx)
/// pairs where `end_idx` is exclusive. Used by callers to shade those
/// zones red on Euler-angle plots — the sharp roll/yaw flips there are
/// representational artifacts, not real motion.
pub fn gimbal_lock_regions(pitch: &[f64]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut in_region = false;
    let mut start = 0;
    for (i, &p) in pitch.iter().enumerate() {
        let gl = p.abs() > 85.0;
        if gl && !in_region {
            start = i;
            in_region = true;
        } else if !gl && in_region {
            out.push((start, i));
            in_region = false;
        }
    }
    if in_region {
        out.push((start, pitch.len()));
    }
    out
}
