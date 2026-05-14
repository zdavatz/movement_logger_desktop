//! `stbox-viz pumpfoil` — pump cadence + phase PNGs.
//! Port of `Utilities/scripts/visualize_pumpfoil.py` for SD-format sensor CSV.
//!
//! Produces:
//!   - plot_pump_cadence.png — dynamic acc, spectrogram, dominant-frequency scatter
//!   - plot_pump_phases.png  — rest/active/crash phase + gyro axes + |acc| + pressure

use crate::io::load_sensor_csv;
use crate::plot_common::{yrange, FONT, MARGIN};
use crate::spectrogram::spectrogram;
use anyhow::{Context, Result};
use plotters::prelude::*;
use std::path::{Path, PathBuf};

const SAMPLE_HZ: f64 = 100.0;

pub fn run(sensor_csv: &Path, output_dir: &Path) -> Result<()> {
    std::fs::create_dir_all(output_dir)
        .with_context(|| format!("mkdir {}", output_dir.display()))?;

    println!("loading {}", sensor_csv.file_name().unwrap().to_string_lossy());
    let samples = load_sensor_csv(sensor_csv)?;
    if samples.is_empty() {
        anyhow::bail!("sensor CSV is empty");
    }

    let t0 = samples[0].ticks;
    let t_sec: Vec<f64> = samples.iter().map(|s| (s.ticks - t0) * 0.01).collect();

    let acc_x: Vec<f64> = samples.iter().map(|s| s.acc[0]).collect();
    let acc_y: Vec<f64> = samples.iter().map(|s| s.acc[1]).collect();
    let acc_z: Vec<f64> = samples.iter().map(|s| s.acc[2]).collect();
    let gyro_x: Vec<f64> = samples.iter().map(|s| s.gyro[0]).collect();
    let gyro_y: Vec<f64> = samples.iter().map(|s| s.gyro[1]).collect();
    let gyro_z: Vec<f64> = samples.iter().map(|s| s.gyro[2]).collect();
    let press: Vec<f64> = samples.iter().map(|s| s.pressure_hpa).collect();

    let cadence_png = output_dir.join("plot_pump_cadence.png");
    render_cadence(&t_sec, &acc_x, &acc_y, &acc_z, &cadence_png)?;
    println!("Saved {}", cadence_png.display());

    let phases_png = output_dir.join("plot_pump_phases.png");
    render_phases(&t_sec, &acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z, &press, &phases_png)?;
    println!("Saved {}", phases_png.display());

    Ok(())
}

fn rolling_mean(x: &[f64], window: usize) -> Vec<f64> {
    let n = x.len();
    let half = window / 2;
    let mut out = vec![0.0; n];
    for i in 0..n {
        let lo = i.saturating_sub(half);
        let hi = (i + half + 1).min(n);
        let s: &[f64] = &x[lo..hi];
        out[i] = s.iter().sum::<f64>() / s.len() as f64;
    }
    out
}

fn rolling_rms(x: &[f64], window: usize) -> Vec<f64> {
    let n = x.len();
    let half = window / 2;
    let mut out = vec![0.0; n];
    for i in 0..n {
        let lo = i.saturating_sub(half);
        let hi = (i + half + 1).min(n);
        let s: &[f64] = &x[lo..hi];
        let sum_sq: f64 = s.iter().map(|v| v * v).sum();
        out[i] = (sum_sq / s.len() as f64).sqrt();
    }
    out
}

fn percentile(x: &[f64], p: f64) -> f64 {
    let mut s: Vec<f64> = x.iter().copied().filter(|v| v.is_finite()).collect();
    if s.is_empty() { return 0.0; }
    s.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let idx = ((s.len() - 1) as f64 * p / 100.0).round() as usize;
    s[idx]
}

fn render_cadence(
    t_sec: &[f64],
    acc_x: &[f64], acc_y: &[f64], acc_z: &[f64],
    out: &PathBuf,
) -> Result<()> {
    // Dynamic acceleration = |acc| − rolling mean
    let acc_mag: Vec<f64> = acc_x.iter().zip(acc_y.iter()).zip(acc_z.iter())
        .map(|((x, y), z)| (x * x + y * y + z * z).sqrt())
        .collect();
    let mean100 = rolling_mean(&acc_mag, 100);
    let acc_dyn: Vec<f64> = acc_mag.iter().zip(mean100.iter())
        .map(|(m, r)| m - r).collect();
    let env = rolling_mean(&acc_dyn.iter().map(|v| v.abs()).collect::<Vec<_>>(), 500);

    let spec = spectrogram(&acc_dyn, SAMPLE_HZ);

    let root = BitMapBackend::new(out, (2400, 1800)).into_drawing_area();
    root.fill(&WHITE)?;
    let areas = root.split_evenly((3, 1));

    let t_min = t_sec[0];
    let t_max = *t_sec.last().unwrap();

    // Panel 1: dynamic acc + envelope
    let (y_lo, y_hi) = yrange(&acc_dyn, 0.05);
    let mut c0 = ChartBuilder::on(&areas[0])
        .caption("Pump-Kadenz-Analyse — Dynamische Beschleunigung (Gravitation entfernt)",
                 (FONT, 22).into_font())
        .margin(MARGIN)
        .x_label_area_size(40)
        .y_label_area_size(80)
        .build_cartesian_2d(t_min..t_max, y_lo..y_hi)
        .map_err(|e| anyhow::anyhow!("chart: {e:?}"))?;
    c0.configure_mesh().y_desc("Dyn. Acc [mg]").x_labels(10).draw()
        .map_err(|e| anyhow::anyhow!("mesh: {e:?}"))?;
    c0.draw_series(LineSeries::new(
        t_sec.iter().zip(acc_dyn.iter()).map(|(x, y)| (*x, *y)),
        RGBColor(70, 130, 180).mix(0.5).stroke_width(1),
    )).map_err(|e| anyhow::anyhow!("dyn: {e:?}"))?;
    c0.draw_series(LineSeries::new(
        t_sec.iter().zip(env.iter()).map(|(x, y)| (*x, *y)),
        RGBColor(0, 0, 128).stroke_width(2),
    )).map_err(|e| anyhow::anyhow!("env: {e:?}"))?;

    // Panel 2: spectrogram as rasterised heatmap
    let f_lo = 0.3; let f_hi = 5.0;
    let f_mask: Vec<usize> = spec.freqs.iter().enumerate()
        .filter(|&(_, &f)| f >= f_lo && f <= f_hi)
        .map(|(i, _)| i).collect();

    let mut c1 = ChartBuilder::on(&areas[1])
        .caption("Spektrogramm — Pump-Frequenz über Zeit",
                 (FONT, 22).into_font())
        .margin(MARGIN)
        .x_label_area_size(40)
        .y_label_area_size(80)
        .build_cartesian_2d(t_min..t_max, f_lo..f_hi)
        .map_err(|e| anyhow::anyhow!("chart: {e:?}"))?;
    c1.configure_mesh().y_desc("Frequenz [Hz]").x_labels(10).draw()
        .map_err(|e| anyhow::anyhow!("mesh: {e:?}"))?;

    if !spec.times.is_empty() && !f_mask.is_empty() {
        // Collect dB values + global min/max for normalisation
        let mut db_min = f64::INFINITY;
        let mut db_max = f64::NEG_INFINITY;
        let mut db_grid: Vec<Vec<f64>> = Vec::with_capacity(f_mask.len());
        for &fi in &f_mask {
            let row: Vec<f64> = spec.power[fi].iter()
                .map(|p| 10.0 * (p + 1e-10).log10()).collect();
            for &v in &row {
                if v.is_finite() {
                    if v < db_min { db_min = v; }
                    if v > db_max { db_max = v; }
                }
            }
            db_grid.push(row);
        }
        let dt = if spec.times.len() > 1 { spec.times[1] - spec.times[0] } else { 1.0 };
        let df = if f_mask.len() > 1 {
            spec.freqs[f_mask[1]] - spec.freqs[f_mask[0]]
        } else { 0.1 };

        for (row_i, &_fi) in f_mask.iter().enumerate() {
            let f = spec.freqs[f_mask[row_i]];
            for (col_i, &tm) in spec.times.iter().enumerate() {
                let v = db_grid[row_i][col_i];
                let norm = ((v - db_min) / (db_max - db_min).max(1e-9)).clamp(0.0, 1.0);
                let color = inferno(norm);
                c1.draw_series(std::iter::once(Rectangle::new(
                    [(tm - dt / 2.0, f - df / 2.0), (tm + dt / 2.0, f + df / 2.0)],
                    color.filled(),
                ))).map_err(|e| anyhow::anyhow!("cell: {e:?}"))?;
            }
        }
    }

    // Reference lines 1 Hz + 2 Hz
    c1.draw_series(std::iter::once(PathElement::new(
        vec![(t_min, 1.0), (t_max, 1.0)],
        RGBColor(0, 255, 255).stroke_width(1),
    ))).map_err(|e| anyhow::anyhow!("ln1: {e:?}"))?;
    c1.draw_series(std::iter::once(PathElement::new(
        vec![(t_min, 2.0), (t_max, 2.0)],
        RGBColor(50, 205, 50).stroke_width(1),
    ))).map_err(|e| anyhow::anyhow!("ln2: {e:?}"))?;

    // Panel 3: dominant-frequency scatter (only where energy > 40th pctl)
    let (dom, pumping_mask) = if !spec.times.is_empty() && !f_mask.is_empty() {
        let mut dom = Vec::with_capacity(spec.times.len());
        let mut energy = Vec::with_capacity(spec.times.len());
        for col_i in 0..spec.times.len() {
            let mut best_v = f64::NEG_INFINITY;
            let mut best_f = 0.0;
            for &fi in &f_mask {
                let v = spec.power[fi][col_i];
                if v > best_v { best_v = v; best_f = spec.freqs[fi]; }
            }
            dom.push(best_f);
            energy.push(best_v);
        }
        let thr = percentile(&energy, 40.0);
        let mask: Vec<bool> = energy.iter().map(|&e| e > thr).collect();
        (dom, mask)
    } else {
        (Vec::new(), Vec::new())
    };

    let mut c2 = ChartBuilder::on(&areas[2])
        .caption("Dominante Pump-Frequenz (nur aktive Phasen)",
                 (FONT, 22).into_font())
        .margin(MARGIN)
        .x_label_area_size(40)
        .y_label_area_size(80)
        .build_cartesian_2d(t_min..t_max, 0.3..4.0)
        .map_err(|e| anyhow::anyhow!("chart: {e:?}"))?;
    c2.configure_mesh().y_desc("Kadenz [Hz]").x_desc("Zeit [s]").x_labels(10).draw()
        .map_err(|e| anyhow::anyhow!("mesh: {e:?}"))?;

    for (i, &tm) in spec.times.iter().enumerate() {
        if pumping_mask.get(i).copied().unwrap_or(false) {
            let f = dom[i];
            let norm = ((f - 0.5) / 2.5).clamp(0.0, 1.0);
            c2.draw_series(std::iter::once(Circle::new(
                (tm, f), 3, coolwarm(norm).filled(),
            ))).map_err(|e| anyhow::anyhow!("sc: {e:?}"))?;
        }
    }

    if !dom.is_empty() {
        let active_freqs: Vec<f64> = dom.iter().zip(pumping_mask.iter())
            .filter(|&(_, &m)| m).map(|(&f, _)| f).collect();
        if !active_freqs.is_empty() {
            let mut s = active_freqs.clone();
            s.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let med = s[s.len() / 2];
            c2.draw_series(std::iter::once(PathElement::new(
                vec![(t_min, med), (t_max, med)],
                RGBColor(255, 0, 0).stroke_width(2),
            ))).map_err(|e| anyhow::anyhow!("med: {e:?}"))?;
            println!("  median cadence: {:.2} Hz ({:.0} pumps/min)", med, med * 60.0);
        }
    }

    root.present()?;
    Ok(())
}

fn render_phases(
    t_sec: &[f64],
    acc_x: &[f64], acc_y: &[f64], acc_z: &[f64],
    gyro_x: &[f64], gyro_y: &[f64], gyro_z: &[f64],
    press: &[f64],
    out: &PathBuf,
) -> Result<()> {
    let acc_mag: Vec<f64> = acc_x.iter().zip(acc_y.iter()).zip(acc_z.iter())
        .map(|((x, y), z)| (x * x + y * y + z * z).sqrt()).collect();
    let mean100 = rolling_mean(&acc_mag, 100);
    let acc_dyn: Vec<f64> = acc_mag.iter().zip(mean100.iter())
        .map(|(m, r)| (m - r).abs()).collect();
    let gyro_mag: Vec<f64> = gyro_x.iter().zip(gyro_y.iter()).zip(gyro_z.iter())
        .map(|((x, y), z)| (x * x + y * y + z * z).sqrt()).collect();

    let win_2s = (SAMPLE_HZ * 2.0) as usize;
    let gyro_rms: Vec<f64> = rolling_rms(&gyro_mag, win_2s).iter()
        .map(|v| v / 1000.0).collect();
    let acc_rms = rolling_rms(&acc_dyn, win_2s);

    let thr_active = percentile(&gyro_rms, 60.0);
    let thr_crash = percentile(&gyro_rms, 95.0);
    let thr_acc_crash = percentile(&acc_rms, 95.0);

    // Phase: 0 = rest, 1 = active, 2 = crash (smoothed via rolling median)
    let mut phase: Vec<u8> = vec![0; gyro_rms.len()];
    for i in 0..gyro_rms.len() {
        if gyro_rms[i] > thr_active { phase[i] = 1; }
        if gyro_rms[i] > thr_crash && acc_rms[i] > thr_acc_crash { phase[i] = 2; }
    }
    let phase_smooth = rolling_median_u8(&phase, 200);

    let root = BitMapBackend::new(out, (2400, 1950)).into_drawing_area();
    root.fill(&WHITE)?;
    // Panel heights 2:1:1:1 — approximate with explicit split points
    let (top, rest) = root.split_vertically(780);
    let (mid1, rest2) = rest.split_vertically(390);
    let (mid2, bottom) = rest2.split_vertically(390);

    let t_min = t_sec[0];
    let t_max = *t_sec.last().unwrap();

    // Panel 1: gyro RMS with phase colouring
    let (g_lo, g_hi) = yrange(&gyro_rms, 0.05);
    let mut c0 = ChartBuilder::on(&top)
        .caption("Bewegungsphasen & Intensität — Pumpfoil Session\nBewegungsintensität (Gyro RMS)",
                 (FONT, 22).into_font())
        .margin(MARGIN)
        .x_label_area_size(40)
        .y_label_area_size(80)
        .build_cartesian_2d(t_min..t_max, 0f64..g_hi.max(1.0))
        .map_err(|e| anyhow::anyhow!("chart: {e:?}"))?;
    c0.configure_mesh().y_desc("Rot.intens. [dps]").x_labels(10).draw()
        .map_err(|e| anyhow::anyhow!("mesh: {e:?}"))?;

    let phase_colors = [
        RGBColor(52, 152, 219),   // 0 = blue
        RGBColor(46, 204, 113),   // 1 = green
        RGBColor(231, 76, 60),    // 2 = red
    ];
    // Draw coloured fill for each phase by grouping contiguous regions
    for p_val in 0u8..=2 {
        let mut run_start: Option<usize> = None;
        for i in 0..phase_smooth.len() {
            if phase_smooth[i] == p_val && run_start.is_none() {
                run_start = Some(i);
            } else if (phase_smooth[i] != p_val || i + 1 == phase_smooth.len()) && run_start.is_some() {
                let s = run_start.take().unwrap();
                let e = if phase_smooth[i] != p_val { i } else { i + 1 };
                if e > s + 1 {
                    let points: Vec<(f64, f64)> = (s..e).map(|k| (t_sec[k], gyro_rms[k])).collect();
                    let col = phase_colors[p_val as usize];
                    c0.draw_series(std::iter::once(Polygon::new(
                        points.iter().copied()
                            .chain(std::iter::once((t_sec[e - 1], 0.0)))
                            .chain(std::iter::once((t_sec[s], 0.0)))
                            .collect::<Vec<_>>(),
                        col.mix(0.5).filled(),
                    ))).map_err(|e| anyhow::anyhow!("fill: {e:?}"))?;
                }
            }
        }
    }

    // Panel 2: gyro per-axis (|gx|, |gy|, |gz| in dps, 2s rolling mean)
    let gx_abs: Vec<f64> = gyro_x.iter().map(|v| (v / 1000.0).abs()).collect();
    let gy_abs: Vec<f64> = gyro_y.iter().map(|v| (v / 1000.0).abs()).collect();
    let gz_abs: Vec<f64> = gyro_z.iter().map(|v| (v / 1000.0).abs()).collect();
    let gx_s = rolling_mean(&gx_abs, win_2s);
    let gy_s = rolling_mean(&gy_abs, win_2s);
    let gz_s = rolling_mean(&gz_abs, win_2s);
    let mut all_g = Vec::new();
    all_g.extend_from_slice(&gx_s);
    all_g.extend_from_slice(&gy_s);
    all_g.extend_from_slice(&gz_s);
    let (_, gy_hi) = yrange(&all_g, 0.05);
    let mut c1 = ChartBuilder::on(&mid1)
        .caption("Rotationsachsen — welche Achse dominiert die Pump-Bewegung?",
                 (FONT, 22).into_font())
        .margin(MARGIN)
        .x_label_area_size(40)
        .y_label_area_size(80)
        .build_cartesian_2d(t_min..t_max, 0f64..gy_hi)
        .map_err(|e| anyhow::anyhow!("chart: {e:?}"))?;
    c1.configure_mesh().y_desc("Drehrate [dps]").x_labels(10).draw()
        .map_err(|e| anyhow::anyhow!("mesh: {e:?}"))?;
    let axis_colors = [
        ("Roll (X)", RGBColor(31, 119, 180)),
        ("Pitch (Y)", RGBColor(255, 127, 14)),
        ("Yaw (Z)", RGBColor(44, 160, 44)),
    ];
    for (vals, (label, color)) in [&gx_s, &gy_s, &gz_s].iter().zip(axis_colors.iter()) {
        let col = *color;
        c1.draw_series(LineSeries::new(
            t_sec.iter().zip(vals.iter()).map(|(x, y)| (*x, *y)),
            col.stroke_width(1),
        )).map_err(|e| anyhow::anyhow!("line: {e:?}"))?
          .label(*label)
          .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], col.stroke_width(2)));
    }
    c1.configure_series_labels()
        .background_style(RGBColor(255, 255, 255).mix(0.85))
        .border_style(BLACK.mix(0.2))
        .position(SeriesLabelPosition::UpperRight)
        .draw().map_err(|e| anyhow::anyhow!("legend: {e:?}"))?;

    // Panel 3: |acc| magnitude + 5s envelope
    let acc_env = rolling_mean(&acc_mag, 500);
    let (a_lo, a_hi) = yrange(&acc_mag, 0.05);
    let mut c2 = ChartBuilder::on(&mid2)
        .caption("Beschleunigungs-Betrag (Spitzen = Stürze/Landungen)",
                 (FONT, 22).into_font())
        .margin(MARGIN)
        .x_label_area_size(40)
        .y_label_area_size(80)
        .build_cartesian_2d(t_min..t_max, a_lo..a_hi)
        .map_err(|e| anyhow::anyhow!("chart: {e:?}"))?;
    c2.configure_mesh().y_desc("|Acc| [mg]").x_labels(10).draw()
        .map_err(|e| anyhow::anyhow!("mesh: {e:?}"))?;
    c2.draw_series(LineSeries::new(
        t_sec.iter().zip(acc_mag.iter()).map(|(x, y)| (*x, *y)),
        RGBColor(70, 130, 180).mix(0.4).stroke_width(1),
    )).map_err(|e| anyhow::anyhow!("mag: {e:?}"))?;
    c2.draw_series(LineSeries::new(
        t_sec.iter().zip(acc_env.iter()).map(|(x, y)| (*x, *y)),
        RGBColor(0, 0, 128).stroke_width(2),
    )).map_err(|e| anyhow::anyhow!("env: {e:?}"))?;

    // Panel 4: pressure (inverted — lower = higher)
    let (p_lo, p_hi) = yrange(press, 0.05);
    let mut c3 = ChartBuilder::on(&bottom)
        .caption("Luftdruck (niedriger = höher über Wasser)",
                 (FONT, 22).into_font())
        .margin(MARGIN)
        .x_label_area_size(40)
        .y_label_area_size(80)
        .build_cartesian_2d(t_min..t_max, p_hi..p_lo)  // inverted Y via swapped order
        .map_err(|e| anyhow::anyhow!("chart: {e:?}"))?;
    c3.configure_mesh().y_desc("P [hPa]").x_desc("Zeit [s]").x_labels(10).draw()
        .map_err(|e| anyhow::anyhow!("mesh: {e:?}"))?;
    c3.draw_series(LineSeries::new(
        t_sec.iter().zip(press.iter()).map(|(x, y)| (*x, *y)),
        RGBColor(148, 103, 189).stroke_width(1),
    )).map_err(|e| anyhow::anyhow!("p: {e:?}"))?;

    root.present()?;
    Ok(())
}

fn rolling_median_u8(x: &[u8], window: usize) -> Vec<u8> {
    let n = x.len();
    let half = window / 2;
    let mut out = vec![0u8; n];
    for i in 0..n {
        let lo = i.saturating_sub(half);
        let hi = (i + half + 1).min(n);
        let mut s: Vec<u8> = x[lo..hi].to_vec();
        s.sort();
        out[i] = s[s.len() / 2];
    }
    out
}

// --- simple inferno & coolwarm colour-maps ---
// Rough approximations: we just need usable heat-map colours, not
// matplotlib-exact. Both maps interpolate through 4 control points.

fn inferno(t: f64) -> RGBColor {
    // Control points sampled from the inferno palette.
    let stops = [
        (0.00, (0, 0, 4)),
        (0.25, (87, 15, 109)),
        (0.50, (188, 55, 84)),
        (0.75, (249, 142, 8)),
        (1.00, (252, 255, 164)),
    ];
    interp_stops(&stops, t)
}

fn coolwarm(t: f64) -> RGBColor {
    let stops = [
        (0.00, (58, 76, 192)),
        (0.50, (221, 221, 221)),
        (1.00, (180, 4, 38)),
    ];
    interp_stops(&stops, t)
}

fn interp_stops(stops: &[(f64, (u8, u8, u8))], t: f64) -> RGBColor {
    let t = t.clamp(0.0, 1.0);
    for w in stops.windows(2) {
        let (t0, c0) = w[0];
        let (t1, c1) = w[1];
        if t <= t1 {
            let f = if t1 > t0 { (t - t0) / (t1 - t0) } else { 0.0 };
            return RGBColor(
                (c0.0 as f64 + (c1.0 as f64 - c0.0 as f64) * f) as u8,
                (c0.1 as f64 + (c1.1 as f64 - c0.1 as f64) * f) as u8,
                (c0.2 as f64 + (c1.2 as f64 - c0.2 as f64) * f) as u8,
            );
        }
    }
    let last = stops.last().unwrap().1;
    RGBColor(last.0, last.1, last.2)
}
