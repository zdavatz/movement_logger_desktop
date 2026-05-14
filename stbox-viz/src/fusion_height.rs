//! Complementary baro + accelerometer fusion for vertical position.
//!
//! Purpose: make the ~1 Hz pump oscillation (amplitude 5–15 cm) clearly
//! visible. Pure baro after TC-correction has ~10–20 cm noise floor, so
//! pump strokes are just barely above the noise. Integrated vertical
//! acceleration (with gravity removed via the Madgwick-derived body-to-
//! world rotation) has sub-cm noise on short timescales but drifts
//! rapidly from any residual bias. Complementary mixing gives the
//! short-term cleanliness of acc with the long-term absolute-level of
//! baro.
//!
//! Limits of this approach:
//! - Does NOT fix the absolute-level drift in the baro (thermal drift
//!   in the semi-sealed SensorTile enclosure is a sensor-hardware
//!   problem, not an algorithm problem). The fused trace inherits the
//!   baro's low-frequency level, including its drift.
//! - Needs accurate orientation from Madgwick so gravity can be
//!   subtracted correctly. When the fusion itself is bad (e.g. first
//!   ~30 s while the filter converges), the "motion acc" still
//!   contains residual gravity projection and the integrated position
//!   wanders.
//!
//! Algorithm: alpha-beta filter (2nd-order complementary). At each
//! sample:
//!
//!     pos_pred = pos + vel·dt + 0.5·a·dt²      (kinematic prediction)
//!     vel_pred = vel + a·dt
//!     r        = baro_height − pos_pred        (baro residual)
//!     pos      = pos_pred + α·r                (position correction)
//!     vel      = vel_pred + (β/dt)·r           (velocity correction)
//!
//! With α = 0.02, β = α²/2 ≈ 2×10⁻⁴ the crossover frequency is ~0.3 Hz:
//! below, baro dominates; above (including the 1 Hz pump), acc
//! dominates. The β-correction absorbs slow acc bias into the velocity
//! state rather than letting it blow up in the integrated position.

use crate::fusion::Quat;
use crate::io::SensorRow;

const GRAVITY_MS2: f64 = 9.80665;
const MG_TO_MS2: f64 = GRAVITY_MS2 / 1000.0;

/// Rotate a body-frame vector into the world frame using the Madgwick
/// quaternion [w, x, y, z]. Standard q·v·q⁻¹ identity.
fn rotate_body_to_world(q: &Quat, v: [f64; 3]) -> [f64; 3] {
    let (qw, qx, qy, qz) = (q[0], q[1], q[2], q[3]);
    let t = [
        2.0 * (qy * v[2] - qz * v[1]),
        2.0 * (qz * v[0] - qx * v[2]),
        2.0 * (qx * v[1] - qy * v[0]),
    ];
    [
        v[0] + qw * t[0] + (qy * t[2] - qz * t[1]),
        v[1] + qw * t[1] + (qz * t[0] - qx * t[2]),
        v[2] + qw * t[2] + (qx * t[1] - qy * t[0]),
    ]
}

/// Returns a fused-height time series aligned 1:1 with `sensors`.
/// `quats` and `baro_height` must match in length and indexing.
pub fn fused_height_m(
    sensors: &[SensorRow],
    quats: &[Quat],
    baro_height: &[f64],
    sample_hz: f64,
) -> Vec<f64> {
    let n = sensors.len();
    assert_eq!(quats.len(), n);
    assert_eq!(baro_height.len(), n);
    if n == 0 {
        return vec![];
    }

    let dt = 1.0 / sample_hz;
    // Crossover ~0.3 Hz: pumps (~1 Hz) pass through acc, drift (~0.1 Hz)
    // comes from baro. α chosen empirically to match this corner.
    let alpha: f64 = 0.02;
    let beta: f64 = alpha * alpha * 0.5;

    let mut pos = baro_height[0];
    let mut vel: f64 = 0.0;
    let mut out = Vec::with_capacity(n);

    for i in 0..n {
        // Body-frame acc (mg → m/s²)
        let a_body = [
            sensors[i].acc[0] * MG_TO_MS2,
            sensors[i].acc[1] * MG_TO_MS2,
            sensors[i].acc[2] * MG_TO_MS2,
        ];
        // Rotate to world frame; subtract gravity (up = +Z in Madgwick's
        // world convention) to isolate motion acceleration.
        let a_world = rotate_body_to_world(&quats[i], a_body);
        let a_up = a_world[2] - GRAVITY_MS2;

        // Kinematic prediction
        let pos_pred = pos + vel * dt + 0.5 * a_up * dt * dt;
        let vel_pred = vel + a_up * dt;

        // Baro residual + α-β correction
        let r = baro_height[i] - pos_pred;
        pos = pos_pred + alpha * r;
        vel = vel_pred + (beta / dt) * r;

        out.push(pos);
    }
    out
}
