//! `stbox-viz sensors` — 5-panel summary PNG of raw sensor data and
//! quaternion/Euler plot. Mirrors `Utilities/scripts/visualize_sensors.py`
//! for the SD-card format only (BLE is legacy — the project's live data
//! is all SD). Per-session detail + FFT overlay is a phase-2b follow-up.

use crate::euler::{gimbal_lock_regions, quats_to_euler_deg};
use crate::fusion::{self, Quat};
use crate::io::load_sensor_csv;
use crate::plot_common::{fmt_min_sec, yrange, FONT, MARGIN};
use anyhow::{Context, Result};
use plotters::prelude::*;
use std::path::{Path, PathBuf};

const SAMPLE_HZ: usize = 100;

pub fn run(sensor_csv: &Path, output_dir: &Path) -> Result<()> {
    std::fs::create_dir_all(output_dir)
        .with_context(|| format!("mkdir {}", output_dir.display()))?;

    println!("loading {}", sensor_csv.file_name().unwrap().to_string_lossy());
    let samples = load_sensor_csv(sensor_csv)?;
    if samples.is_empty() {
        anyhow::bail!("sensor CSV is empty");
    }
    let base = sensor_csv.file_stem().unwrap().to_string_lossy().to_string();

    let t0 = samples[0].ticks;
    let t_sec: Vec<f64> = samples.iter()
        .map(|s| (s.ticks - t0) * 0.01)
        .collect();

    let sensors_png = output_dir.join(format!("plot_sensors_{}.png", base));
    render_sensors_5panel(&samples, &t_sec, &base, &sensors_png)?;
    println!("Saved {}", sensors_png.display());

    println!("running Madgwick 6DOF fusion");
    let quats = fusion::compute_quaternions(&samples, 0.1);

    let quat_png = output_dir.join(format!("plot_quaternions_{}.png", base));
    render_quaternions_euler(&quats, &t_sec, &base, &quat_png)?;
    println!("Saved {}", quat_png.display());

    Ok(())
}

fn render_sensors_5panel(
    samples: &[crate::io::SensorRow],
    t_sec: &[f64],
    base: &str,
    out: &PathBuf,
) -> Result<()> {
    let root = BitMapBackend::new(out, (2100, 2400)).into_drawing_area();
    root.fill(&WHITE)?;
    let areas = root.split_evenly((5, 1));

    let t_min = t_sec[0];
    let t_max = *t_sec.last().unwrap();

    let acc_x: Vec<f64> = samples.iter().map(|s| s.acc[0]).collect();
    let acc_y: Vec<f64> = samples.iter().map(|s| s.acc[1]).collect();
    let acc_z: Vec<f64> = samples.iter().map(|s| s.acc[2]).collect();
    let gyro_x: Vec<f64> = samples.iter().map(|s| s.gyro[0]).collect();
    let gyro_y: Vec<f64> = samples.iter().map(|s| s.gyro[1]).collect();
    let gyro_z: Vec<f64> = samples.iter().map(|s| s.gyro[2]).collect();
    let mag_x: Vec<f64> = samples.iter().map(|s| s.mag[0]).collect();
    let mag_y: Vec<f64> = samples.iter().map(|s| s.mag[1]).collect();
    let mag_z: Vec<f64> = samples.iter().map(|s| s.mag[2]).collect();
    let temp: Vec<f64> = samples.iter().map(|s| s.temperature_c).collect();
    let press: Vec<f64> = samples.iter().map(|s| s.pressure_hpa).collect();

    let xyz_colors = [
        ("X", RGBColor(31, 119, 180)),
        ("Y", RGBColor(255, 127, 14)),
        ("Z", RGBColor(44, 160, 44)),
    ];

    // Row 0: Accel
    draw_xyz_panel(
        &areas[0],
        &format!("Sensordaten (SD-Karte) — {}\nBeschleunigung [mg]", base),
        "Acc [mg]",
        t_sec, (t_min, t_max),
        &[(&acc_x, "X"), (&acc_y, "Y"), (&acc_z, "Z")],
        &xyz_colors,
    )?;

    // Row 1: Gyro
    draw_xyz_panel(
        &areas[1],
        "Drehrate",
        "Gyro [mdps]",
        t_sec, (t_min, t_max),
        &[(&gyro_x, "X"), (&gyro_y, "Y"), (&gyro_z, "Z")],
        &xyz_colors,
    )?;

    // Row 2: Mag
    draw_xyz_panel(
        &areas[2],
        "Magnetfeld",
        "Mag [mGauss]",
        t_sec, (t_min, t_max),
        &[(&mag_x, "X"), (&mag_y, "Y"), (&mag_z, "Z")],
        &xyz_colors,
    )?;

    // Row 3: Temperature
    draw_single_panel(&areas[3], "Temperatur", "T [°C]",
        t_sec, (t_min, t_max), &temp, RGBColor(214, 39, 40))?;

    // Row 4: Pressure (with x-axis labels)
    draw_single_panel(&areas[4], "Luftdruck", "P [hPa]",
        t_sec, (t_min, t_max), &press, RGBColor(148, 103, 189))?;

    root.present()?;
    Ok(())
}

fn draw_xyz_panel<DB: DrawingBackend>(
    area: &DrawingArea<DB, plotters::coord::Shift>,
    title: &str,
    y_label: &str,
    t: &[f64],
    x_range: (f64, f64),
    series: &[(&Vec<f64>, &str)],
    colors: &[(&str, RGBColor); 3],
) -> Result<(), anyhow::Error>
where
    DB::ErrorType: 'static,
{
    let mut all: Vec<f64> = Vec::new();
    for (v, _) in series { all.extend_from_slice(v); }
    let (y_lo, y_hi) = yrange(&all, 0.05);

    let mut chart = ChartBuilder::on(area)
        .caption(title, (FONT, 22).into_font())
        .margin(MARGIN)
        .x_label_area_size(40)
        .y_label_area_size(80)
        .build_cartesian_2d(x_range.0..x_range.1, y_lo..y_hi)
        .map_err(|e| anyhow::anyhow!("chart: {e:?}"))?;

    chart.configure_mesh()
        .y_desc(y_label)
        .x_labels(10)
        .x_label_formatter(&|v| fmt_min_sec(v))
        .light_line_style(RGBColor(230, 230, 230))
        .draw()
        .map_err(|e| anyhow::anyhow!("mesh: {e:?}"))?;

    for (i, (values, name)) in series.iter().enumerate() {
        let color = colors[i].1;
        chart.draw_series(LineSeries::new(
            t.iter().zip(values.iter()).map(|(x, y)| (*x, *y)),
            color.stroke_width(1),
        ))
        .map_err(|e| anyhow::anyhow!("line: {e:?}"))?
        .label(*name)
        .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], color.stroke_width(2)));
    }

    chart.configure_series_labels()
        .background_style(RGBColor(255, 255, 255).mix(0.85))
        .border_style(BLACK.mix(0.2))
        .position(SeriesLabelPosition::UpperRight)
        .draw()
        .map_err(|e| anyhow::anyhow!("legend: {e:?}"))?;

    Ok(())
}

fn draw_single_panel<DB: DrawingBackend>(
    area: &DrawingArea<DB, plotters::coord::Shift>,
    title: &str,
    y_label: &str,
    t: &[f64],
    x_range: (f64, f64),
    values: &[f64],
    color: RGBColor,
) -> Result<(), anyhow::Error>
where
    DB::ErrorType: 'static,
{
    let (y_lo, y_hi) = yrange(values, 0.05);
    let mut chart = ChartBuilder::on(area)
        .caption(title, (FONT, 22).into_font())
        .margin(MARGIN)
        .x_label_area_size(40)
        .y_label_area_size(80)
        .build_cartesian_2d(x_range.0..x_range.1, y_lo..y_hi)
        .map_err(|e| anyhow::anyhow!("chart: {e:?}"))?;

    chart.configure_mesh()
        .y_desc(y_label)
        .x_desc("Zeit [min:sek]")
        .x_labels(10)
        .x_label_formatter(&|v| fmt_min_sec(v))
        .light_line_style(RGBColor(230, 230, 230))
        .draw()
        .map_err(|e| anyhow::anyhow!("mesh: {e:?}"))?;

    chart.draw_series(LineSeries::new(
        t.iter().zip(values.iter()).map(|(x, y)| (*x, *y)),
        color.stroke_width(1),
    )).map_err(|e| anyhow::anyhow!("line: {e:?}"))?;

    Ok(())
}

fn render_quaternions_euler(
    quats: &[Quat],
    t_sec: &[f64],
    base: &str,
    out: &PathBuf,
) -> Result<()> {
    let root = BitMapBackend::new(out, (2100, 1200)).into_drawing_area();
    root.fill(&WHITE)?;
    let areas = root.split_evenly((2, 1));

    let t_min = t_sec[0];
    let t_max = *t_sec.last().unwrap();

    let qi: Vec<f64> = quats.iter().map(|q| q[1]).collect();
    let qj: Vec<f64> = quats.iter().map(|q| q[2]).collect();
    let qk: Vec<f64> = quats.iter().map(|q| q[3]).collect();
    let qs: Vec<f64> = quats.iter().map(|q| q[0]).collect();

    // Row 0: quaternion components
    let q_colors = [
        ("Qi", RGBColor(31, 119, 180)),
        ("Qj", RGBColor(255, 127, 14)),
        ("Qk", RGBColor(44, 160, 44)),
        ("Qs", RGBColor(214, 39, 40)),
    ];
    let mut all = Vec::new();
    all.extend_from_slice(&qi);
    all.extend_from_slice(&qj);
    all.extend_from_slice(&qk);
    all.extend_from_slice(&qs);
    let (y_lo, y_hi) = yrange(&all, 0.05);

    let mut c0 = ChartBuilder::on(&areas[0])
        .caption(format!("Quaternion / Orientierung — {}\nQuaternion-Verlauf", base),
                 (FONT, 22).into_font())
        .margin(MARGIN)
        .x_label_area_size(40)
        .y_label_area_size(80)
        .build_cartesian_2d(t_min..t_max, y_lo..y_hi)
        .map_err(|e| anyhow::anyhow!("chart: {e:?}"))?;
    c0.configure_mesh()
        .y_desc("Quaternion-Komponenten")
        .x_desc("Zeit [min:sek]")
        .x_labels(10)
        .x_label_formatter(&|v| fmt_min_sec(v))
        .light_line_style(RGBColor(230, 230, 230))
        .draw().map_err(|e| anyhow::anyhow!("mesh: {e:?}"))?;

    for (vals, (label, color)) in [&qi, &qj, &qk, &qs].iter().zip(q_colors.iter()) {
        let col = *color;
        c0.draw_series(LineSeries::new(
            t_sec.iter().zip(vals.iter()).map(|(x, y)| (*x, *y)),
            col.stroke_width(1),
        )).map_err(|e| anyhow::anyhow!("line: {e:?}"))?
          .label(*label)
          .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], col.stroke_width(2)));
    }
    c0.configure_series_labels()
        .background_style(RGBColor(255, 255, 255).mix(0.85))
        .border_style(BLACK.mix(0.2))
        .position(SeriesLabelPosition::UpperRight)
        .draw().map_err(|e| anyhow::anyhow!("legend: {e:?}"))?;

    // Row 1: Euler angles with gimbal lock shading
    let (roll, pitch, yaw) = quats_to_euler_deg(quats);
    let mut all_e = Vec::new();
    all_e.extend_from_slice(&roll);
    all_e.extend_from_slice(&pitch);
    all_e.extend_from_slice(&yaw);
    let (e_lo, e_hi) = yrange(&all_e, 0.05);

    let mut c1 = ChartBuilder::on(&areas[1])
        .caption("Euler-Winkel (aus Quaternionen berechnet)",
                 (FONT, 22).into_font())
        .margin(MARGIN)
        .x_label_area_size(40)
        .y_label_area_size(80)
        .build_cartesian_2d(t_min..t_max, e_lo..e_hi)
        .map_err(|e| anyhow::anyhow!("chart: {e:?}"))?;
    c1.configure_mesh()
        .y_desc("Winkel [°]")
        .x_desc("Zeit [min:sek]")
        .x_labels(10)
        .x_label_formatter(&|v| fmt_min_sec(v))
        .light_line_style(RGBColor(230, 230, 230))
        .draw().map_err(|e| anyhow::anyhow!("mesh: {e:?}"))?;

    // Gimbal-lock regions: red translucent vspans
    for (s, e) in gimbal_lock_regions(&pitch) {
        let x0 = t_sec[s];
        let x1 = t_sec[(e - 1).min(t_sec.len() - 1)];
        c1.draw_series(std::iter::once(Rectangle::new(
            [(x0, e_lo), (x1, e_hi)],
            RGBColor(255, 0, 0).mix(0.15).filled(),
        ))).map_err(|e| anyhow::anyhow!("rect: {e:?}"))?;
    }

    let e_colors = [
        ("Roll", RGBColor(31, 119, 180)),
        ("Pitch", RGBColor(255, 127, 14)),
        ("Yaw", RGBColor(44, 160, 44)),
    ];
    for (vals, (label, color)) in [&roll, &pitch, &yaw].iter().zip(e_colors.iter()) {
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

    root.present()?;
    Ok(())
}
