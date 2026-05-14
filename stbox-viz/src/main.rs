//! stbox-viz — SensorTile.box-Pro pumpfoil session visualiser.
//!
//! Replaces all of `Utilities/scripts/*.py` with four subcommands:
//!
//!   combined   interactive Plotly HTML (map + nose angle + height + speed)
//!   sensors    5-panel PNG of raw sensors + quaternion/Euler angles
//!   pumpfoil   pump-cadence spectrogram + movement-phase PNGs
//!   animate    animated GIF of board side view + optional combined MOV

mod animate_cmd;
mod baro;
mod bin_util;
mod board3d;
mod butter;
mod compass_cmd;
mod euler;
mod fusion;
mod fusion_height;
mod gps;
mod html;
mod io;
mod plot_common;
mod pumpfoil_cmd;
mod sensors_cmd;
mod session;
mod spectrogram;

use anyhow::{Context, Result, anyhow};
use clap::{Parser, Subcommand};
use std::path::{Path, PathBuf};

const SAMPLE_HZ: usize = 100;

#[derive(Parser, Debug)]
#[command(name = "stbox-viz", version, about = "SensorTile.box-Pro session visualiser")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand, Debug)]
enum Cmd {
    /// Interactive Plotly HTML combining map + time-series panels.
    Combined {
        sensor_csv: PathBuf,
        #[arg(short, long, default_value = "html")]
        output: PathBuf,
        #[arg(long, default_value_t = 0.1)]
        beta: f64,
        /// UTC offset in hours for the x-axis (e.g. 3 for Greek summer
        /// time, 2 for Swiss summer time). Default 0 = UTC. Determines
        /// the wall-clock time displayed on the time axis.
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        tz_offset_h: f64,
        /// Recording date as YYYY-MM-DD. Used as the date for x-axis
        /// datetimes. If omitted, falls back to the sensor file's
        /// modification time, then to today.
        #[arg(long)]
        date: Option<String>,
    },
    /// Sensor + quaternion PNG plots.
    Sensors {
        sensor_csv: PathBuf,
        #[arg(short, long, default_value = "png")]
        output: PathBuf,
    },
    /// Pump cadence + movement-phase PNGs.
    Pumpfoil {
        sensor_csv: PathBuf,
        #[arg(short, long, default_value = "png")]
        output: PathBuf,
    },
    /// Compass-validity PNG: tilt-compensated magnetometer heading vs
    /// GPS course over ground. Residual exposes hard/soft-iron
    /// interference on the LIS2MDL from nearby metal screws.
    Compass {
        sensor_csv: PathBuf,
        #[arg(short, long, default_value = "png")]
        output: PathBuf,
    },
    /// Animated GIF of board orientation per session, with optional
    /// side-by-side camera-video MOV combine (needs ffmpeg in PATH).
    Animate {
        sensor_csv: PathBuf,
        #[arg(short, long, default_value = "gif")]
        output: PathBuf,
        #[arg(long, default_value_t = 15)]
        fps: u32,
        #[arg(long)]
        session: Option<usize>,
        #[arg(long)]
        video: Option<PathBuf>,
        #[arg(long, default_value_t = 0.0)]
        video_offset: f64,
        #[arg(long, default_value_t = 0.0)]
        sensor_offset: f64,
        #[arg(long)]
        title: Option<String>,
        #[arg(long)]
        subtitle: Option<String>,
        /// Wall-clock start time HH:MM[:SS] in local time. When set,
        /// bypasses session detection and renders one GIF spanning the
        /// `--duration` window (or the video's length if --video given).
        /// Pair with --tz-offset-h and --date so the time anchor matches
        /// the GPS clock.
        #[arg(long)]
        at: Option<String>,
        /// Duration in seconds for the --at window. Defaults to the
        /// video's duration when --video is set, else 60 s.
        #[arg(long)]
        duration: Option<f64>,
        /// UTC offset in hours for --at. Default 0 = UTC.
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        tz_offset_h: f64,
        /// Recording date as YYYY-MM-DD, used with --at. Defaults to
        /// the sensor file's mtime.
        #[arg(long)]
        date: Option<String>,
        /// In --at mode, auto-skip the carry/transition seconds before
        /// sustained pitch oscillation begins. Off by default — the
        /// raw nose-angle stream from --at looks visually right;
        /// auto-skip can land mid-stroke and make the angle trace
        /// look discontinuous.
        #[arg(long, default_value_t = false)]
        auto_skip: bool,
        /// Height of the launch dock above water in metres. Baro
        /// height anchors its "water reference" to stationary periods
        /// (speed < 3 km/h). When the rider starts on a dock, the
        /// only stationary anchor in a short --at window IS the dock,
        /// so heights end up dock-relative instead of water-relative.
        /// Pass `--dock-height-m 0.75` to shift the displayed height
        /// up by 0.75 m → dock shows +0.75 m, water = 0 m, foiling =
        /// actual lift above water. Default 0 (no offset).
        #[arg(long, default_value_t = 0.0)]
        dock_height_m: f64,
        /// Path to a binary STL of the board (fingerfoil's
        /// `1_board.stl` is the canonical one). When set, the side-
        /// view panel renders the board as a 3D rotating mesh driven
        /// by Madgwick pitch+roll plus GPS-course yaw, instead of
        /// the simple 2D line. See issue #4 for the data plan.
        #[arg(long)]
        board_stl: Option<PathBuf>,
        /// Where the SensorTile.box is physically mounted on the
        /// board. `mast` (default) — strapped to the mast with chip
        /// +X pointing along the mast; used for Ayano's Ermioni
        /// sessions. `deck` — on top of the deck, long axis
        /// nose-tail, chip +Z facing down through the deck; used
        /// for Peter's 28.4.2026 session.
        #[arg(long, value_enum, default_value_t = MountArg::Mast)]
        mount: MountArg,
    },
}

#[derive(clap::ValueEnum, Clone, Copy, Debug)]
enum MountArg { Mast, Deck }

impl From<MountArg> for board3d::MountKind {
    fn from(m: MountArg) -> Self {
        match m {
            MountArg::Mast => board3d::MountKind::Mast,
            MountArg::Deck => board3d::MountKind::Deck,
        }
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.cmd {
        Cmd::Combined { sensor_csv, output, beta, tz_offset_h, date } =>
            run_combined(&sensor_csv, &output, beta, tz_offset_h, date.as_deref()),
        Cmd::Sensors { sensor_csv, output } =>
            sensors_cmd::run(&sensor_csv, &output),
        Cmd::Pumpfoil { sensor_csv, output } =>
            pumpfoil_cmd::run(&sensor_csv, &output),
        Cmd::Compass { sensor_csv, output } =>
            compass_cmd::run(&sensor_csv, &output),
        Cmd::Animate { sensor_csv, output, fps, session, video,
                       video_offset, sensor_offset, title, subtitle,
                       at, duration, tz_offset_h, date, auto_skip,
                       dock_height_m, board_stl, mount } => {
            animate_cmd::run(&animate_cmd::AnimateArgs {
                sensor_csv: &sensor_csv,
                output_dir: &output,
                fps,
                session,
                video: video.as_deref(),
                video_offset,
                sensor_offset,
                title: title.as_deref(),
                subtitle: subtitle.as_deref(),
                at: at.as_deref(),
                duration,
                tz_offset_h,
                date: date.as_deref(),
                auto_skip,
                dock_height_m,
                board_stl: board_stl.as_deref(),
                mount: mount.into(),
            })
        }
    }
}

fn run_combined(sensor_path: &Path, output: &Path, beta: f64,
                tz_offset_h: f64, date_arg: Option<&str>) -> Result<()> {
    let sensor_path = sensor_path.canonicalize()
        .with_context(|| format!("canonicalize {}", sensor_path.display()))?;
    let stem = sensor_path.file_stem().and_then(|s| s.to_str())
        .ok_or_else(|| anyhow!("sensor csv has no stem"))?.to_string();

    let gps_path = guess_gps_path(&sensor_path);

    println!("loading {}", sensor_path.file_name().unwrap().to_string_lossy());
    let sensors = io::load_sensor_csv(&sensor_path)?;
    let n = sensors.len();
    let dur_s = n as f64 / SAMPLE_HZ as f64;
    println!("  {} rows at {} Hz ({:.1} min)", n, SAMPLE_HZ, dur_s / 60.0);
    if n == 0 { return Err(anyhow!("sensor CSV is empty")); }

    let base_ticks = sensors[0].ticks;
    let t_sensor_s: Vec<f64> = sensors.iter()
        .map(|s| (s.ticks - base_ticks) / gps::TICKS_PER_SEC)
        .collect();

    println!("running Madgwick 6DOF fusion (beta={})", beta);
    let quats = fusion::compute_quaternions(&sensors, beta);
    let nose_deg = fusion::nose_angle_series_deg(&quats, SAMPLE_HZ);

    let (gps_rows, gps_speed, gps_t_s) = if let Some(p) = gps_path.as_ref() {
        println!("loading {}", p.file_name().unwrap().to_string_lossy());
        let mut rows = io::load_gps_csv(p)?;
        rows.retain(|g| g.fix >= 1);
        println!("  {} GPS fixes", rows.len());
        let raw = gps::position_derived_speed_kmh(&rows);
        // Reject multipath-induced position jumps: anything where the
        // implied longitudinal acceleration exceeds 15 km/h/s (≈4 m/s²)
        // is almost certainly a bad fix, not a real paddle stroke.
        // SUPfoil paddle-starts produce 1–3 m/s² per stroke.
        let gated = gps::reject_acc_outliers(&rows, &raw, 15.0);
        let smooth = gps::smooth_speed_kmh(&gated);
        let t_s: Vec<f64> = rows.iter()
            .map(|g| (g.ticks - base_ticks) / gps::TICKS_PER_SEC).collect();
        (rows, smooth, t_s)
    } else {
        println!("no GPS CSV found — skipping map + ride detection");
        (Vec::new(), Vec::new(), Vec::new())
    };

    let rides = if !gps_rows.is_empty() {
        gps::detect_rides(&gps_rows, &gps_speed, n, SAMPLE_HZ, base_ticks,
                          3.0, 10.0, 30.0, 3.0)
    } else { Vec::new() };
    println!("detected {} rides over water", rides.len());
    for (i, r) in rides.iter().enumerate() {
        let m = (r.duration_s / 60.0) as u32;
        let s = ((r.duration_s - m as f64 * 60.0) as u32).min(59);
        println!("  session {}: UTC {}, duration {:02}:{:02}, p90 {:.1} km/h",
                 i + 1, r.utc_start, m, s, r.p90_kmh);
    }

    let height_m = baro::height_above_water_m(&sensors, &gps_rows, &gps_speed, base_ticks);

    // GPS altitude, zeroed at the median of stationary samples (speed
    // < 3 km/h) so it lines up with the baro height axis. MAX-M10S
    // vertical scatter is ~3–10 m — the trace won't resolve pumps, but
    // makes fix dropouts visible when the board lies flat on water.
    let gps_height_m: Vec<f64> = if !gps_rows.is_empty() {
        let alt: Vec<f64> = gps_rows.iter().map(|g| g.alt_m).collect();
        let mut stationary: Vec<f64> = alt.iter().zip(gps_speed.iter())
            .filter(|&(_, &s)| s < 3.0)
            .map(|(&a, _)| a)
            .collect();
        let baseline = {
            let pool = if !stationary.is_empty() { &mut stationary } else {
                &mut alt.clone()
            };
            pool.sort_by(|a, b| a.partial_cmp(b).unwrap());
            pool[pool.len() / 2]
        };
        alt.iter().map(|a| a - baseline).collect()
    } else {
        Vec::new()
    };

    // Reverted to the simplest form: the GPS-anchored TC-corrected baro
    // height on its own. Earlier iterations added α-β fusion with the
    // accelerometer, 30 s rolling-mean detrending, and 0.5 s
    // post-smoothing — each intended to kill the baro's thermal drift,
    // but they also stripped out the slow-trend component that visibly
    // correlates with the nose angle.
    //
    // One concession: a mild 250 ms rolling mean on the raw baro before
    // binning, to kill the ~10 Hz sensor-noise floor. Without this,
    // when thermal drift pushes the signal near the ±1 m y-axis clip,
    // the noise constantly crosses the boundary and Plotly draws
    // saturated vertical bars at ±1 m ("Zacken"). 250 ms preserves the
    // 1 Hz pump oscillation almost entirely (sinusoidal attenuation
    // ≈ 4 %) and the slow trends that correlate with nose angle.
    let height_m_smooth = rolling_mean(&height_m, 25);
    let fused_m: Vec<f64> = vec![];  // keep the slot, render path skips empty

    let (nose_t, nose_binned) =
        bin_util::bin_to_resolution(&t_sensor_s, &nose_deg, 100, bin_util::Agg::Mean);
    let (_, height_binned_raw) =
        bin_util::bin_to_resolution(&t_sensor_s, &height_m_smooth, 100, bin_util::Agg::Mean);
    // Physical range: the mast is 0.8 m, so board height is in [0, 0.8]
    // (plus a tiny margin to account for noise and momentary
    // submersion). Anything outside that is baro thermal drift — show
    // as a gap rather than a line saturated at the axis boundary, so
    // the graph only plots values that make physical sense. Does not
    // affect bin length (NaN replaces value, slot stays) so the
    // hover interpolation in html.rs still works.
    let height_binned: Vec<f64> = height_binned_raw.iter()
        .map(|&v| if v >= -0.15 && v <= 0.95 { v } else { f64::NAN })
        .collect();
    let fused_binned: Vec<f64> = Vec::new();

    // Build the wall-clock anchor for the time axis. We anchor sensor
    // t = 0 to a real datetime by combining: (a) the date (CLI arg or
    // file mtime or today) with (b) the UTC time-of-day at sensor t = 0,
    // back-extrapolated from the first GPS fix's UTC + tick. Without
    // GPS, we anchor to midnight UTC and the axis still plots correctly
    // — just with a wrong absolute time.
    let date_utc = resolve_session_date(date_arg, &sensor_path)?;
    let utc_at_t0_sec = if let Some(g) = gps_rows.first() {
        let utc_sec = parse_utc_hhmmss(&g.utc).unwrap_or(0.0);
        utc_sec - (g.ticks - base_ticks) / gps::TICKS_PER_SEC
    } else {
        0.0
    };

    let title = format!("{} — {:.1} min · {} ride{}",
        stem, dur_s / 60.0, rides.len(),
        if rides.len() == 1 { "" } else { "s" });
    let page = html::render(&html::PanelData {
        t_sensor_s: &nose_t,
        nose_deg: &nose_binned,
        height_m: &height_binned,
        fused_height_m: &fused_binned,
        gps: &gps_rows,
        gps_speed_kmh: &gps_speed,
        gps_t_s: &gps_t_s,
        gps_height_m: &gps_height_m,
        rides: &rides,
        title: &title,
        date_utc,
        utc_at_t0_sec,
        tz_offset_h,
    });

    std::fs::create_dir_all(output)
        .with_context(|| format!("mkdir {}", output.display()))?;
    let out_path = output.join(format!("viz_{}.html", stem));
    std::fs::write(&out_path, page)
        .with_context(|| format!("write {}", out_path.display()))?;
    println!("  wrote {}", out_path.display());
    Ok(())
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

/// Parse a "hhmmss.ss" GPS UTC string into seconds-since-midnight.
fn parse_utc_hhmmss(s: &str) -> Option<f64> {
    let s = s.trim();
    if s.len() < 6 { return None; }
    let h: f64 = s[0..2].parse().ok()?;
    let m: f64 = s[2..4].parse().ok()?;
    let sec: f64 = s[4..].parse().ok()?;
    Some(h * 3600.0 + m * 60.0 + sec)
}

/// Pick the recording date. Preference order: explicit CLI arg →
/// sensor file's mtime (in UTC) → today (UTC). The date is purely
/// for display — getting it slightly wrong only mislabels the date,
/// the time-of-day still reads correctly off the GPS clock.
fn resolve_session_date(arg: Option<&str>, sensor_path: &Path)
    -> Result<chrono::NaiveDate>
{
    use chrono::{NaiveDate, DateTime, Utc};
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

/// Subtract a centred rolling mean from each sample. Removes slow
/// drift (thermal baro drift) while preserving oscillations. Mean is
/// used over median because median has step-like response when the
/// sample distribution shifts — produces visible sharp corners in
/// the output; mean is C¹-smooth as long as the input is.
fn subtract_rolling_mean(x: &[f64], window: usize) -> Vec<f64> {
    let n = x.len();
    if n == 0 || window == 0 { return x.to_vec(); }
    let half = window / 2;
    // Cumulative sum + finite-count for O(n) rolling mean.
    let mut csum = vec![0.0; n + 1];
    let mut ccount = vec![0usize; n + 1];
    for i in 0..n {
        let v = if x[i].is_finite() { x[i] } else { 0.0 };
        csum[i + 1] = csum[i] + v;
        ccount[i + 1] = ccount[i] + if x[i].is_finite() { 1 } else { 0 };
    }
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let lo = i.saturating_sub(half);
        let hi = (i + half + 1).min(n);
        let cnt = ccount[hi] - ccount[lo];
        if cnt == 0 {
            out.push(x[i]);
        } else {
            let mean = (csum[hi] - csum[lo]) / cnt as f64;
            out.push(x[i] - mean);
        }
    }
    out
}

/// Centred rolling mean (no subtraction) — used as a light post-
/// smoothing pass to take the edge off the alpha-beta filter's
/// high-frequency baro-noise injection.
fn rolling_mean(x: &[f64], window: usize) -> Vec<f64> {
    let n = x.len();
    if n == 0 || window <= 1 { return x.to_vec(); }
    let half = window / 2;
    let mut csum = vec![0.0; n + 1];
    let mut ccount = vec![0usize; n + 1];
    for i in 0..n {
        let v = if x[i].is_finite() { x[i] } else { 0.0 };
        csum[i + 1] = csum[i] + v;
        ccount[i + 1] = ccount[i] + if x[i].is_finite() { 1 } else { 0 };
    }
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let lo = i.saturating_sub(half);
        let hi = (i + half + 1).min(n);
        let cnt = ccount[hi] - ccount[lo];
        out.push(if cnt == 0 { x[i] } else { (csum[hi] - csum[lo]) / cnt as f64 });
    }
    out
}
