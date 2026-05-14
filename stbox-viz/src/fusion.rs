//! Madgwick AHRS 6DOF quaternion filter (IMU-only, acc + gyro).
//!
//! Port of the classic Madgwick 2010 algorithm: gradient descent on the
//! accelerometer error, weighted by `beta`. Magnetometer is deliberately
//! skipped — the LIS2MDL on this board drifts over time and couples into
//! roll/pitch through the 9DOF gradient, corrupting the quaternion even
//! when heading drift is tolerable.
//!
//! Input units: acc in mg (milli-g), gyro in mdps (milli-degrees-per-second).
//! Internal math: acc normalised (direction only), gyro converted to rad/s.

use crate::io::SensorRow;

const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;

/// Quaternion as [w, x, y, z].
pub type Quat = [f64; 4];

pub struct Madgwick {
    pub q: Quat,
    pub beta: f64,
}

impl Madgwick {
    pub fn new(beta: f64) -> Self {
        Self { q: [1.0, 0.0, 0.0, 0.0], beta }
    }

    /// Advance the filter by one sample. `dt` in seconds, gyro in rad/s,
    /// acc direction (any unit — only the normalised vector is used).
    pub fn update_imu(&mut self, gyro_rad: [f64; 3], acc: [f64; 3], dt: f64) {
        let [mut q0, mut q1, mut q2, mut q3] = self.q;
        let (gx, gy, gz) = (gyro_rad[0], gyro_rad[1], gyro_rad[2]);
        let (mut ax, mut ay, mut az) = (acc[0], acc[1], acc[2]);

        // Rate of change from gyro only
        let mut d_q0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        let mut d_q1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
        let mut d_q2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
        let mut d_q3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

        // Add accelerometer correction if acc vector is non-zero
        let a_norm = (ax * ax + ay * ay + az * az).sqrt();
        if a_norm > 1e-9 {
            ax /= a_norm;
            ay /= a_norm;
            az /= a_norm;

            // Gradient of the objective function (see Madgwick eq. 25-46 IMU variant)
            let _2q0 = 2.0 * q0;
            let _2q1 = 2.0 * q1;
            let _2q2 = 2.0 * q2;
            let _2q3 = 2.0 * q3;
            let _4q0 = 4.0 * q0;
            let _4q1 = 4.0 * q1;
            let _4q2 = 4.0 * q2;
            let _8q1 = 8.0 * q1;
            let _8q2 = 8.0 * q2;
            let q0q0 = q0 * q0;
            let q1q1 = q1 * q1;
            let q2q2 = q2 * q2;
            let q3q3 = q3 * q3;

            let s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            let s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1
                + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            let s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
                + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            let s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;

            // Normalise gradient
            let s_norm = (s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3).sqrt();
            if s_norm > 1e-9 {
                let inv = 1.0 / s_norm;
                d_q0 -= self.beta * s0 * inv;
                d_q1 -= self.beta * s1 * inv;
                d_q2 -= self.beta * s2 * inv;
                d_q3 -= self.beta * s3 * inv;
            }
        }

        // Integrate
        q0 += d_q0 * dt;
        q1 += d_q1 * dt;
        q2 += d_q2 * dt;
        q3 += d_q3 * dt;

        // Normalise quaternion
        let n = (q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3).sqrt();
        if n > 1e-9 {
            let inv = 1.0 / n;
            self.q = [q0 * inv, q1 * inv, q2 * inv, q3 * inv];
        } else {
            self.q = [1.0, 0.0, 0.0, 0.0];
        }
    }
}

/// Run 6DOF fusion across all sensor samples. Returns one quaternion
/// per input row. Sample rate is auto-detected from the tick delta
/// (1 tick = 10 ms) — if median dt is 1 tick, this is 100 Hz sampling
/// and dt for the filter is 0.01 s.
pub fn compute_quaternions(samples: &[SensorRow], beta: f64) -> Vec<Quat> {
    if samples.is_empty() {
        return vec![];
    }

    let dt = detect_dt_seconds(samples);
    let mut f = Madgwick::new(beta);
    let mut out = Vec::with_capacity(samples.len());

    for s in samples {
        let gyro_rad = [
            s.gyro[0] * 0.001 * DEG_TO_RAD,
            s.gyro[1] * 0.001 * DEG_TO_RAD,
            s.gyro[2] * 0.001 * DEG_TO_RAD,
        ];
        f.update_imu(gyro_rad, s.acc, dt);
        out.push(f.q);
    }
    out
}

/// Detect the per-sample dt in seconds from the median tick delta.
/// Ticks are 10 ms on this firmware, so a median dt of 1 tick means 100 Hz.
pub fn detect_dt_seconds(samples: &[SensorRow]) -> f64 {
    if samples.len() < 2 {
        return 0.01;
    }
    let mut deltas: Vec<f64> = samples.windows(2).map(|w| w[1].ticks - w[0].ticks).collect();
    deltas.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = deltas[deltas.len() / 2];
    // Median 1 tick → 100 Hz sampling, dt = 0.01 s
    // Median 10 ticks → maybe a 10 Hz recording, dt = 0.1 s
    (median * 0.01).max(0.0001)
}

/// Board nose elevation (°) from quaternions, following the existing Python
/// convention: the sensor is mounted Breitachse (Y-axis along the board
/// nose direction). Rotating the body-frame Y axis into world frame gives:
///   nose_z = 2·(qj·qk − qs·qi)
/// where the quaternion ordering is [qs, qi, qj, qk] = [w, x, y, z].
/// Returns angles before drift correction; caller does the smoothing +
/// rolling-median baseline subtraction.
pub fn nose_z_component(q: &Quat) -> f64 {
    // Quat layout: [w, x, y, z] = [qs, qi, qj, qk]
    let qs = q[0];
    let qi = q[1];
    let qj = q[2];
    let qk = q[3];
    (2.0 * (qj * qk - qs * qi)).clamp(-1.0, 1.0)
}

pub fn nose_angle_series_deg(quats: &[Quat], sample_hz: usize) -> Vec<f64> {
    let raw: Vec<f64> = quats.iter().map(|q| nose_z_component(q).asin().to_degrees()).collect();

    // 1 s centred median — suppresses magnetometer-correction spikes
    let w1 = sample_hz.max(1);
    let smooth = rolling_median(&raw, w1);

    // 60 s rolling median baseline
    let w60 = 60 * sample_hz;
    let baseline = rolling_median(&smooth, w60);

    smooth.iter().zip(baseline.iter()).map(|(s, b)| s - b).collect()
}

/// Centred rolling median with edge-shortened windows. O(n·w) — fine for
/// w ≤ 6000 and n ≤ 200k (a 33-minute session at 100 Hz).
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
