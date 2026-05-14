//! `stbox-viz compass` — validate the LIS2MDL magnetometer by plotting
//! its tilt-compensated heading against the GPS course over ground.
//!
//! Motivation (Peter Schmidlin, 23.4.2026): "Wäre interessant zu wissen
//! wie gut der Kompass ist" — the board has several metal screws near
//! the sensor box that are suspected of causing hard/soft-iron
//! distortion. If the compass and the GPS course line up (modulo a
//! constant magnetic-declination offset), the compass is usable for
//! heading readout. If they drift apart with orientation or position,
//! the iron interference is too strong to trust the mag alone.
//!
//! Method:
//! 1. Run Madgwick 6DOF (acc+gyro only, no mag) to get orientation
//! 2. Rotate the raw body-frame magnetometer vector into world frame
//! 3. World-frame heading = atan2(-mag_world_y, mag_world_x), in degrees
//!    from geographic north (clockwise positive — the same convention
//!    as the GPS course column)
//! 4. GPS course is only meaningful above ~2 km/h — slower than that
//!    and the u-blox module's course-over-ground is noise
//! 5. Residual (mag − GPS) exposes the compass error. A static offset
//!    of a few degrees is just magnetic declination (Ermioni ~+4° E,
//!    Zürich ~+2.5° E in 2026); orientation-dependent residuals are the
//!    interesting signal of iron distortion.

use crate::fusion;
use crate::io::{load_gps_csv, load_sensor_csv};
use crate::plot_common::{fmt_min_sec, yrange, FONT, MARGIN};
use anyhow::{Context, Result};
use plotters::prelude::*;
use std::path::{Path, PathBuf};

const TICKS_PER_SEC: f64 = 100.0;

pub fn run(sensor_csv: &Path, output_dir: &Path) -> Result<()> {
    std::fs::create_dir_all(output_dir)
        .with_context(|| format!("mkdir {}", output_dir.display()))?;

    let stem = sensor_csv.file_stem().unwrap().to_string_lossy().to_string();
    let gps_path = {
        let p = sensor_csv.with_file_name(format!("{}_gps.csv", stem));
        if p.exists() { Some(p) } else { None }
    };
    let gps_path = gps_path.ok_or_else(|| anyhow::anyhow!(
        "GPS CSV not found (expected {}_gps.csv next to the sensor CSV)", stem))?;

    println!("loading {}", sensor_csv.file_name().unwrap().to_string_lossy());
    let sensors = load_sensor_csv(sensor_csv)?;
    if sensors.is_empty() { anyhow::bail!("sensor CSV is empty"); }

    println!("loading {}", gps_path.file_name().unwrap().to_string_lossy());
    let mut gps = load_gps_csv(&gps_path)?;
    gps.retain(|g| g.fix >= 1);
    if gps.is_empty() { anyhow::bail!("no valid GPS fixes"); }

    println!("running Madgwick 6DOF fusion for orientation");
    let quats = fusion::compute_quaternions(&sensors, 0.1);

    let base_ticks = sensors[0].ticks;
    let t_sensor_s: Vec<f64> = sensors.iter()
        .map(|s| (s.ticks - base_ticks) / TICKS_PER_SEC).collect();

    // Tilt-compensated magnetic heading per sensor sample. Madgwick
    // 6DOF has no absolute yaw reference (no magnetometer feedback in
    // the filter), so rotating the mag by the full quaternion would
    // give heading relative to Madgwick's drifting world-X, not true
    // north. Use only roll + pitch from the quaternion, leave yaw
    // alone, and compute heading in the horizontal projection of the
    // body frame — the classic compass-tilt-compensation formula.
    //
    // Convention on this board (per nose-angle code in fusion.rs):
    // body-Y = nose direction (along board), body-X = Breitachse
    // (across board), body-Z = up. Heading is measured CW from
    // magnetic north to the board's nose (body-Y).
    let heading_mag: Vec<f64> = sensors.iter().zip(quats.iter()).map(|(s, q)| {
        let (roll, pitch) = quat_to_tilt_rad(q);
        let (sr, cr) = roll.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (mx, my, mz) = (s.mag[0], s.mag[1], s.mag[2]);
        // Rotate mag_body by (−pitch, −roll) to the level (horizontal)
        // body frame. Standard Honeywell AN203 tilt-comp identities:
        let xh = mx * cp + mz * sp;
        let yh = mx * sr * sp + my * cr - mz * sr * cp;
        // Nose direction in board-body frame is +Y. Heading of nose
        // relative to magnetic north: angle of (xh,yh) from +Y, CW.
        let h = (-xh).atan2(yh).to_degrees();
        if h < 0.0 { h + 360.0 } else { h }
    }).collect();

    // GPS course, masked to samples where ground speed > 2 km/h
    // (course from the u-blox module is noise at lower speeds).
    let mut gps_t_s = Vec::new();
    let mut gps_course = Vec::new();
    let mut gps_mag_at_gps = Vec::new();
    let mut residuals = Vec::new();
    // Position-derived speed reuse
    let raw_speed = crate::gps::position_derived_speed_kmh(&gps);
    for (i, row) in gps.iter().enumerate() {
        if raw_speed[i] < 2.0 { continue; }
        let t = (row.ticks - base_ticks) / TICKS_PER_SEC;
        gps_t_s.push(t);
        let course = row.course_deg;
        gps_course.push(course);
        // Linear-interpolate the mag heading at this GPS time.
        // Heading wraps at 360 so naive interp causes spikes near wrap;
        // unwrap sensor heading into a continuous series first so the
        // interp is smooth, then wrap back for plotting.
        let m = interp_circular(&t_sensor_s, &heading_mag, t);
        gps_mag_at_gps.push(m);

        let mut r = m - course;
        // Wrap residual to [-180, 180]
        while r > 180.0 { r -= 360.0; }
        while r < -180.0 { r += 360.0; }
        residuals.push(r);
    }

    let out = output_dir.join(format!("plot_compass_{}.png", stem));
    render_compass_plot(&t_sensor_s, &heading_mag,
                        &gps_t_s, &gps_course, &gps_mag_at_gps, &residuals,
                        &stem, &out)?;
    println!("Saved {}", out.display());

    // Print a quick summary
    if !residuals.is_empty() {
        let mut s = residuals.clone();
        s.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let med = s[s.len() / 2];
        let p5 = s[(s.len() as f64 * 0.05) as usize];
        let p95 = s[(s.len() as f64 * 0.95) as usize];
        println!("compass − GPS residual over {} moving samples:", s.len());
        println!("  median {:+.1}°  (expected: magnetic declination, ~+2–4° E in Europe)", med);
        println!("  p5–p95 span: {:+.1}° to {:+.1}° ({:.1}° total)", p5, p95, p95 - p5);
        println!("  a span > ~30° suggests the LIS2MDL is being pulled by nearby iron");
    }
    Ok(())
}

/// Extract roll and pitch (in radians) from a [w, x, y, z] quaternion.
/// Ignores yaw — caller uses tilt for compass compensation, and yaw
/// from a 6DOF Madgwick filter has no absolute reference.
fn quat_to_tilt_rad(q: &[f64; 4]) -> (f64, f64) {
    let (qw, qx, qy, qz) = (q[0], q[1], q[2], q[3]);
    let sinr_cosp = 2.0 * (qw * qx + qy * qz);
    let cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    let roll = sinr_cosp.atan2(cosr_cosp);
    let sinp = 2.0 * (qw * qy - qz * qx);
    let pitch = if sinp.abs() >= 1.0 {
        sinp.signum() * std::f64::consts::FRAC_PI_2
    } else {
        sinp.asin()
    };
    (roll, pitch)
}

/// Linear interpolate a circular (0–360°) series at `t`, handling the
/// 360°/0° wrap so interpolation between 350° and 10° gives 0° not 180°.
fn interp_circular(x: &[f64], y: &[f64], xt: f64) -> f64 {
    if x.is_empty() { return 0.0; }
    if xt <= x[0] { return y[0]; }
    if xt >= *x.last().unwrap() { return *y.last().unwrap(); }
    // Binary search for the bracket
    let mut lo = 0; let mut hi = x.len() - 1;
    while lo + 1 < hi {
        let mid = (lo + hi) / 2;
        if x[mid] <= xt { lo = mid; } else { hi = mid; }
    }
    let (a, b) = (y[lo], y[hi]);
    let mut diff = b - a;
    while diff > 180.0 { diff -= 360.0; }
    while diff < -180.0 { diff += 360.0; }
    let frac = (xt - x[lo]) / (x[hi] - x[lo]);
    let mut out = a + diff * frac;
    if out < 0.0 { out += 360.0; }
    if out >= 360.0 { out -= 360.0; }
    out
}

fn render_compass_plot(
    t_sensor: &[f64],
    heading_mag: &[f64],
    gps_t: &[f64],
    gps_course: &[f64],
    gps_mag_at_gps: &[f64],
    residuals: &[f64],
    stem: &str,
    out: &PathBuf,
) -> Result<()> {
    let root = BitMapBackend::new(out, (2200, 1800)).into_drawing_area();
    root.fill(&WHITE)?;
    let areas = root.split_evenly((2, 1));

    let t_min = if let Some(&t0) = t_sensor.first() { t0 } else { 0.0 };
    let t_max = if let Some(&tn) = t_sensor.last() { tn } else { 1.0 };

    // Top panel: both heading series overlaid
    let mut c0 = ChartBuilder::on(&areas[0])
        .caption(format!("Compass vs GPS course — {}\nblue = mag heading (tilt-compensated, Madgwick); red = GPS course over ground (only where v > 2 km/h)", stem),
                 (FONT, 22).into_font())
        .margin(MARGIN)
        .x_label_area_size(40)
        .y_label_area_size(80)
        .build_cartesian_2d(t_min..t_max, 0f64..360.0)
        .map_err(|e| anyhow::anyhow!("chart: {e:?}"))?;
    c0.configure_mesh()
        .y_desc("Heading [°]")
        .x_labels(12)
        .x_label_formatter(&|v| fmt_min_sec(v))
        .draw().map_err(|e| anyhow::anyhow!("mesh: {e:?}"))?;

    // Mag heading as a dense line (with wrap gaps handled by plotters'
    // default behaviour — lines will jump at 0/360 boundaries)
    c0.draw_series(heading_mag.iter().zip(t_sensor.iter()).map(|(&h, &t)| {
        Circle::new((t, h), 1, RGBColor(31, 119, 180).filled())
    })).map_err(|e| anyhow::anyhow!("mag: {e:?}"))?;

    c0.draw_series(gps_course.iter().zip(gps_t.iter()).map(|(&h, &t)| {
        Circle::new((t, h), 3, RGBColor(220, 20, 20).filled())
    })).map_err(|e| anyhow::anyhow!("gps: {e:?}"))?;

    // Bottom panel: residual
    let (r_lo, r_hi) = {
        let (a, b) = yrange(residuals, 0.1);
        // Cap the view to something readable if there are outliers.
        (a.max(-180.0), b.min(180.0))
    };
    let mut c1 = ChartBuilder::on(&areas[1])
        .caption("Residual: mag heading − GPS course. A constant offset is magnetic declination (~+2–4° in Europe); orientation-dependent scatter is iron distortion.",
                 (FONT, 20).into_font())
        .margin(MARGIN)
        .x_label_area_size(40)
        .y_label_area_size(80)
        .build_cartesian_2d(t_min..t_max, r_lo..r_hi)
        .map_err(|e| anyhow::anyhow!("chart: {e:?}"))?;
    c1.configure_mesh()
        .y_desc("Δheading [°]")
        .x_desc("Zeit [min:sek]")
        .x_labels(12)
        .x_label_formatter(&|v| fmt_min_sec(v))
        .draw().map_err(|e| anyhow::anyhow!("mesh: {e:?}"))?;

    // Zero line
    c1.draw_series(std::iter::once(PathElement::new(
        vec![(t_min, 0.0), (t_max, 0.0)],
        RGBColor(128, 128, 128).stroke_width(1),
    ))).map_err(|e| anyhow::anyhow!("zero: {e:?}"))?;

    c1.draw_series(residuals.iter().zip(gps_t.iter()).map(|(&r, &t)| {
        Circle::new((t, r), 3, RGBColor(80, 80, 80).mix(0.6).filled())
    })).map_err(|e| anyhow::anyhow!("resid: {e:?}"))?;

    let _ = gps_mag_at_gps; // reserved for future use (e.g. colour-coded residual)

    root.present()?;
    Ok(())
}
