//! Session tab — a recorded session drawn straight from the box's
//! `SENSnnn.CSV` + `GPSnnn.CSV`, no video and no stbox-viz render.
//!
//! Left: the GPS track on an OSM / satellite map, coloured by speed,
//! with a dot at the cursor time. Right: stacked time-series charts
//! (GPS speed, acceleration, pitch/roll, baro height, rotation rate)
//! sharing one time axis and one cursor line. Below: a time slider
//! with playback. Clicking the map or a chart moves the cursor.
//!
//! Both files are stamped with the box's own `HAL_GetTick()` ms, so
//! they share one clock and need no alignment. Wall-clock labels come
//! from the host `# SYNC` anchors when present, else from the GPS UTC.
//!
//! The sensor file is ~100 Hz (a 3 h session is ~1 M rows), so the
//! charts never draw raw samples: every frame each series is reduced
//! to min/max pairs over the visible time range (`decimate`).

use std::path::{Path, PathBuf};
use std::io::Read;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use egui_plot::{Corner, Legend, Line, Plot, PlotPoints, VLine};
use stbox_viz::io;
use walkers::{HttpTiles, Map, MapMemory, Plugin, Position, Projector};

use crate::race::{opts, EsriWorldImagery, OverzoomTiles, MAX_MAP_ZOOM};

/// Min/max buckets per chart and frame — two points each, so ~2400
/// vertices per series regardless of session length.
const PLOT_BUCKETS: usize = 1200;

/// GPS hops faster than this between consecutive fixes are multipath
/// jumps, not movement (same clip as the Race tab).
const MAX_JUMP_KMH: f64 = 60.0;

/// Pitch/roll come from the accelerometer alone, smoothed over this
/// many seconds so pumping accelerations don't dominate the angle.
const ANGLE_SMOOTH_S: f64 = 0.5;

/// Read size per syscall — big enough that a USB-mounted SD streams.
const READ_CHUNK: usize = 4 << 20;

const PLAY_SPEEDS: [f64; 5] = [1.0, 5.0, 20.0, 60.0, 300.0];

pub struct SessionData {
    sens_name: String,
    gps_name: Option<String>,
    /// Box ms of time 0 on every axis (first sample of either file).
    t0_ms: f64,
    duration_s: f64,
    /// Wall clock: epoch ms at t = 0 plus drift-free slope from the
    /// `# SYNC` anchors, or the GPS-UTC seconds-of-day offset.
    clock: Clock,

    sens_t: Vec<f64>,
    acc_g: Vec<f64>,
    gyro_dps: Vec<f64>,
    pitch_deg: Vec<f64>,
    roll_deg: Vec<f64>,
    height_m: Vec<f64>,

    gps_t: Vec<f64>,
    lat: Vec<f64>,
    lon: Vec<f64>,
    speed_kmh: Vec<f64>,
    nsat: Vec<f64>,
    /// Indices into the gps_* vectors that are drawable track points
    /// (fix, non-zero position, no multipath jump).
    track: Vec<usize>,
    speed_scale: f64,
    distance_m: f64,
    max_speed_kmh: f64,
}

#[derive(Clone)]
enum Clock {
    /// Host `# SYNC` anchors: tick (10 ms units) → epoch ms.
    Anchors(Vec<io::SyncAnchor>),
    /// GPS UTC: seconds-of-day = box seconds + offset (UTC, no date).
    GpsUtc(f64),
    None,
}

impl Clock {
    /// "HH:MM:SS" local wall time for box ms `ms`, if known.
    fn label(&self, ms: f64, with_tenths: bool) -> Option<String> {
        use chrono::{Local, TimeZone, Timelike};
        let (h, m, s) = match self {
            Clock::Anchors(a) => {
                let epoch = io::abs_times_from_sync_anchors(&[ms / 10.0], a)[0];
                let t = Local.timestamp_millis_opt(epoch).single()?;
                let frac = (epoch.rem_euclid(1000)) as f64 / 1000.0;
                (t.hour(), t.minute(), t.second() as f64 + frac)
            }
            Clock::GpsUtc(off) => {
                let utc_sod = (ms / 1000.0 + off).rem_euclid(86_400.0);
                let local_off = Local::now().offset().local_minus_utc() as f64;
                let sod = (utc_sod + local_off).rem_euclid(86_400.0);
                let whole = sod.floor() as u32;
                (whole / 3600, (whole / 60) % 60, (whole % 60) as f64 + sod.fract())
            }
            Clock::None => return None,
        };
        Some(if with_tenths {
            format!("{h:02}:{m:02}:{s:04.1}")
        } else {
            format!("{h:02}:{m:02}:{:02}", s.floor() as u32)
        })
    }
}

/// Seconds-of-day from the firmware's "hhmmss.ss" UTC string.
fn utc_sod(s: &str) -> Option<f64> {
    let s = s.trim();
    if s.len() < 6 {
        return None;
    }
    let h: f64 = s.get(0..2)?.parse().ok()?;
    let m: f64 = s.get(2..4)?.parse().ok()?;
    let sec: f64 = s.get(4..)?.parse().ok()?;
    (h < 24.0 && m < 60.0 && sec < 61.0).then_some(h * 3600.0 + m * 60.0 + sec)
}

fn haversine_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let r = 6_371_000.0;
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dp = p2 - p1;
    let dl = (lon2 - lon1).to_radians();
    let a = (dp / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.0).sin().powi(2);
    2.0 * r * a.sqrt().asin()
}

/// Centred moving average over `win` samples (prefix sums, O(n)).
fn smooth(y: &[f64], win: usize) -> Vec<f64> {
    if win <= 1 || y.is_empty() {
        return y.to_vec();
    }
    let mut pre = Vec::with_capacity(y.len() + 1);
    pre.push(0.0);
    for v in y {
        pre.push(pre.last().unwrap() + v);
    }
    let half = win / 2;
    (0..y.len())
        .map(|i| {
            let a = i.saturating_sub(half);
            let b = (i + half + 1).min(y.len());
            (pre[b] - pre[a]) / (b - a) as f64
        })
        .collect()
}

/// Loader progress shared with the UI: which file, bytes read so far,
/// file size. Parsing is fast; reading is the slow part when the CSVs
/// sit on the box's SD card mounted over USB (~1 MB/s).
#[derive(Default)]
struct Progress {
    file: Mutex<String>,
    done: AtomicU64,
    total: AtomicU64,
}

/// Each file is read exactly once, in big sequential chunks (a USB
/// mass-storage SD answers small reads slowly), then parsed from RAM.
fn read_all(path: &Path, progress: &Progress) -> Result<Vec<u8>, String> {
    let err = |e: std::io::Error| format!("{}: {e}", path.display());
    let mut f = std::fs::File::open(path).map_err(err)?;
    let total = f.metadata().map(|m| m.len()).unwrap_or(0);
    *progress.file.lock().unwrap() = path.file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
    progress.total.store(total, Ordering::Relaxed);
    progress.done.store(0, Ordering::Relaxed);
    let mut buf = Vec::with_capacity(total as usize);
    let mut chunk = vec![0u8; READ_CHUNK];
    loop {
        let n = f.read(&mut chunk).map_err(err)?;
        if n == 0 {
            break;
        }
        buf.extend_from_slice(&chunk[..n]);
        progress.done.store(buf.len() as u64, Ordering::Relaxed);
    }
    Ok(buf)
}

fn load(sens: &Path, gps: Option<&Path>, progress: &Progress) -> Result<SessionData, String> {
    // GPS first: it's small, so a wrong/missing GPS file fails fast.
    let gps_rows = match gps {
        Some(p) => {
            let bytes = read_all(p, progress)?;
            io::load_gps_csv_from(bytes.as_slice(), p).map_err(|e| format!("{}: {e:#}", p.display()))?
        }
        None => Vec::new(),
    };
    let sens_bytes = read_all(sens, progress)?;
    let rows = io::load_sensor_csv_from(sens_bytes.as_slice(), sens)
        .map_err(|e| format!("{}: {e:#}", sens.display()))?;
    if rows.is_empty() {
        return Err(format!("{}: no sensor rows", sens.display()));
    }

    // Ticks are 10 ms units in both loaders.
    let sens_ms: Vec<f64> = rows.iter().map(|r| r.ticks * 10.0).collect();
    let gps_ms: Vec<f64> = gps_rows.iter().map(|r| r.ticks * 10.0).collect();
    let t0_ms = sens_ms[0].min(gps_ms.first().copied().unwrap_or(f64::MAX));
    let sens_t: Vec<f64> = sens_ms.iter().map(|m| (m - t0_ms) / 1000.0).collect();
    let gps_t: Vec<f64> = gps_ms.iter().map(|m| (m - t0_ms) / 1000.0).collect();
    let duration_s = sens_t
        .last()
        .copied()
        .unwrap_or(0.0)
        .max(gps_t.last().copied().unwrap_or(0.0));

    let acc_g: Vec<f64> = rows
        .iter()
        .map(|r| (r.acc[0].powi(2) + r.acc[1].powi(2) + r.acc[2].powi(2)).sqrt() / 1000.0)
        .collect();
    let gyro_dps: Vec<f64> = rows
        .iter()
        .map(|r| (r.gyro[0].powi(2) + r.gyro[1].powi(2) + r.gyro[2].powi(2)).sqrt() / 1000.0)
        .collect();

    // Box frame: nose = +Y, up = +Z (see CLAUDE.md "Board angles" — the
    // phone-style long-axis-X formulas swap pitch and roll on this box).
    let hz = if sens_t.len() > 1 && duration_s > 0.0 {
        (sens_t.len() as f64 / sens_t.last().unwrap().max(1e-3)).clamp(1.0, 1000.0)
    } else {
        100.0
    };
    let win = (ANGLE_SMOOTH_S * hz).round() as usize;
    let ax = smooth(&rows.iter().map(|r| r.acc[0]).collect::<Vec<_>>(), win);
    let ay = smooth(&rows.iter().map(|r| r.acc[1]).collect::<Vec<_>>(), win);
    let az = smooth(&rows.iter().map(|r| r.acc[2]).collect::<Vec<_>>(), win);
    let pitch_deg: Vec<f64> = (0..rows.len())
        .map(|i| ay[i].atan2((ax[i] * ax[i] + az[i] * az[i]).sqrt()).to_degrees())
        .collect();
    let roll_deg: Vec<f64> = (0..rows.len())
        .map(|i| ax[i].atan2(az[i]).to_degrees())
        .collect();

    // Baro height relative to the start (international barometric
    // formula, reference = median of the first ~second).
    let n0 = rows.len().min(100);
    let mut first: Vec<f64> = rows[..n0].iter().map(|r| r.pressure_hpa).collect();
    first.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let p0 = first[first.len() / 2];
    let height_m = smooth(
        &rows
            .iter()
            .map(|r| 44_330.0 * (1.0 - (r.pressure_hpa / p0).powf(1.0 / 5.255)))
            .collect::<Vec<_>>(),
        win,
    );

    let lat: Vec<f64> = gps_rows.iter().map(|r| r.lat).collect();
    let lon: Vec<f64> = gps_rows.iter().map(|r| r.lon).collect();
    let speed_kmh: Vec<f64> = gps_rows
        .iter()
        .map(|r| if r.fix > 0 { r.speed_kmh_module } else { f64::NAN })
        .collect();
    let nsat: Vec<f64> = gps_rows.iter().map(|r| r.num_sat as f64).collect();

    let mut track: Vec<usize> = Vec::new();
    let mut distance_m = 0.0;
    for (i, r) in gps_rows.iter().enumerate() {
        if r.fix <= 0 || (r.lat == 0.0 && r.lon == 0.0) {
            continue;
        }
        if let Some(&j) = track.last() {
            let d = haversine_m(lat[j], lon[j], lat[i], lon[i]);
            let dt = (gps_t[i] - gps_t[j]).max(0.05);
            if d / dt * 3.6 > MAX_JUMP_KMH {
                continue;
            }
            distance_m += d;
        }
        track.push(i);
    }
    let mut speeds: Vec<f64> = track.iter().map(|&i| speed_kmh[i]).filter(|v| v.is_finite()).collect();
    speeds.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let max_speed_kmh = speeds.last().copied().unwrap_or(0.0);
    // Colour scale tops out at the 95th percentile so one spike
    // doesn't wash the whole track blue.
    let speed_scale = speeds
        .get(speeds.len().saturating_sub(1) * 95 / 100)
        .copied()
        .unwrap_or(0.0)
        .max(5.0);

    let anchors = io::sync_anchors_from_str(&String::from_utf8_lossy(&sens_bytes));
    let clock = if !anchors.is_empty() {
        Clock::Anchors(anchors)
    } else {
        // Median of (UTC seconds-of-day − box seconds) over fixed rows.
        let mut offs: Vec<f64> = gps_rows
            .iter()
            .zip(&gps_ms)
            .filter(|(r, _)| r.fix > 0)
            .filter_map(|(r, &ms)| utc_sod(&r.utc).map(|u| u - ms / 1000.0))
            .take(200)
            .collect();
        offs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        match offs.get(offs.len() / 2) {
            Some(&o) => Clock::GpsUtc(o),
            None => Clock::None,
        }
    };

    let name = |p: &Path| p.file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
    Ok(SessionData {
        sens_name: name(sens),
        gps_name: gps.map(name),
        t0_ms,
        duration_s,
        clock,
        sens_t,
        acc_g,
        gyro_dps,
        pitch_deg,
        roll_deg,
        height_m,
        gps_t,
        lat,
        lon,
        speed_kmh,
        nsat,
        track,
        speed_scale,
        distance_m,
        max_speed_kmh,
    })
}

/// Min/max pairs of `y` over buckets of the visible range `[lo, hi]`
/// (`t` sorted). NaN samples are skipped; a bucket with none is a gap.
fn decimate(t: &[f64], y: &[f64], lo: f64, hi: f64) -> Vec<[f64; 2]> {
    let a = t.partition_point(|&v| v < lo).saturating_sub(1);
    let b = (t.partition_point(|&v| v <= hi) + 1).min(t.len());
    if b <= a {
        return Vec::new();
    }
    let n = b - a;
    if n <= PLOT_BUCKETS * 2 {
        return (a..b).filter(|&i| y[i].is_finite()).map(|i| [t[i], y[i]]).collect();
    }
    let mut out = Vec::with_capacity(PLOT_BUCKETS * 2);
    for k in 0..PLOT_BUCKETS {
        let s = a + n * k / PLOT_BUCKETS;
        let e = a + n * (k + 1) / PLOT_BUCKETS;
        let (mut imin, mut imax) = (usize::MAX, usize::MAX);
        for i in s..e {
            if !y[i].is_finite() {
                continue;
            }
            if imin == usize::MAX || y[i] < y[imin] {
                imin = i;
            }
            if imax == usize::MAX || y[i] > y[imax] {
                imax = i;
            }
        }
        if imin == usize::MAX {
            continue;
        }
        let (p, q) = if imin <= imax { (imin, imax) } else { (imax, imin) };
        out.push([t[p], y[p]]);
        if q != p {
            out.push([t[q], y[q]]);
        }
    }
    out
}

/// Value of `y` at time `x` (nearest sample at or before).
fn at(t: &[f64], y: &[f64], x: f64) -> Option<f64> {
    if t.is_empty() {
        return None;
    }
    let i = t.partition_point(|&v| v <= x).saturating_sub(1);
    y.get(i).copied().filter(|v| v.is_finite())
}

/// Blue (slow) → green → red (fast).
fn speed_color(v: f64, scale: f64) -> egui::Color32 {
    let f = (v / scale).clamp(0.0, 1.0) as f32;
    let (r, g, b) = if f < 0.5 {
        let k = f * 2.0;
        (0.0, k, 1.0 - k)
    } else {
        let k = (f - 0.5) * 2.0;
        (k, 1.0 - k, 0.0)
    };
    egui::Color32::from_rgb((40.0 + 215.0 * r) as u8, (40.0 + 215.0 * g) as u8, (40.0 + 215.0 * b) as u8)
}

struct TrackPlugin<'a> {
    d: &'a SessionData,
    cursor_s: f64,
    /// Set when the user clicked near the track: that fix's time.
    clicked: &'a mut Option<f64>,
}

impl Plugin for TrackPlugin<'_> {
    fn run(self: Box<Self>, ui: &mut egui::Ui, response: &egui::Response, projector: &Projector) {
        let painter = ui.painter().with_clip_rect(response.rect);
        let d = self.d;
        let screen = |i: usize| {
            let v = projector.project(Position::from_lat_lon(d.lat[i], d.lon[i]));
            egui::pos2(v.x, v.y)
        };
        // One polyline per run of the same colour band (16 bands).
        let band = |i: usize| {
            let v = d.speed_kmh[i];
            if v.is_finite() { ((v / d.speed_scale).clamp(0.0, 1.0) * 15.0).round() as i32 } else { 0 }
        };
        let mut run: Vec<egui::Pos2> = Vec::new();
        let mut run_band = -1;
        for (k, &i) in d.track.iter().enumerate() {
            let b = band(i);
            let p = screen(i);
            if b != run_band && !run.is_empty() {
                run.push(p);
                let c = speed_color(run_band as f64 / 15.0 * d.speed_scale, d.speed_scale);
                painter.add(egui::Shape::line(std::mem::take(&mut run), egui::Stroke::new(3.0_f32, c)));
            }
            if k == 0 || b != run_band {
                run_band = b;
            }
            run.push(p);
        }
        if run.len() >= 2 {
            let c = speed_color(run_band as f64 / 15.0 * d.speed_scale, d.speed_scale);
            painter.add(egui::Shape::line(run, egui::Stroke::new(3.0_f32, c)));
        }
        // Start / end markers.
        if let (Some(&s), Some(&e)) = (d.track.first(), d.track.last()) {
            painter.circle_filled(screen(s), 5.0, egui::Color32::from_rgb(30, 160, 60));
            painter.circle_filled(screen(e), 5.0, egui::Color32::from_rgb(200, 40, 40));
        }
        // Cursor dot: last track point at or before the cursor time.
        let k = d.track.partition_point(|&i| d.gps_t[i] <= self.cursor_s);
        if k > 0 {
            let p = screen(d.track[k - 1]);
            painter.circle_filled(p, 8.0, egui::Color32::from_rgb(255, 200, 0));
            painter.circle_stroke(p, 8.0, egui::Stroke::new(2.0_f32, egui::Color32::BLACK));
        }
        // Click → nearest track point within 15 px moves the cursor.
        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                let best = d
                    .track
                    .iter()
                    .map(|&i| (i, screen(i).distance_sq(pos)))
                    .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
                if let Some((i, dist2)) = best {
                    if dist2 < 15.0 * 15.0 {
                        *self.clicked = Some(d.gps_t[i]);
                    }
                }
            }
        }
    }
}

pub struct SessionView {
    loaded_from: Option<(PathBuf, Option<PathBuf>)>,
    loading: Option<Receiver<Result<SessionData, String>>>,
    progress: Arc<Progress>,
    data: Option<SessionData>,
    error: Option<String>,
    cursor_s: f64,
    playing: bool,
    play_speed: usize,
    last_frame: Option<Instant>,
    follow: bool,
    satellite: bool,
    fit_pending: bool,
    map_memory: MapMemory,
    tiles: Option<OverzoomTiles>,
    tiles_sat: Option<OverzoomTiles>,
    /// Visible time range of the charts last frame (drives decimation).
    view: Option<(f64, f64)>,
    show_speed: bool,
    show_acc: bool,
    show_angles: bool,
    show_height: bool,
    show_gyro: bool,
}

impl Default for SessionView {
    fn default() -> Self {
        Self {
            loaded_from: None,
            loading: None,
            progress: Arc::default(),
            data: None,
            error: None,
            cursor_s: 0.0,
            playing: false,
            play_speed: 1,
            last_frame: None,
            follow: false,
            satellite: false,
            fit_pending: false,
            map_memory: MapMemory::default(),
            tiles: None,
            tiles_sat: None,
            view: None,
            show_speed: true,
            show_acc: true,
            show_angles: true,
            show_height: true,
            show_gyro: false,
        }
    }
}

impl SessionView {
    /// `sensor` / `gps` are the Replay tab's file slots, so a file
    /// dropped anywhere in the window lands here too.
    pub fn ui(&mut self, ui: &mut egui::Ui, sensor: &mut Option<PathBuf>, gps: &mut Option<PathBuf>) {
        self.poll_loading(ui.ctx());

        ui.horizontal(|ui| {
            file_pick(ui, "Sensor", sensor);
            ui.separator();
            file_pick(ui, "GPS", gps);
            ui.separator();
            let busy = self.loading.is_some();
            let wanted = sensor.clone().map(|s| (s, gps.clone()));
            let label = if busy { "Loading…" } else { "Show session" };
            if ui
                .add_enabled(!busy && wanted.is_some(), egui::Button::new(label))
                .on_hover_text("Draw the GPS track and the sensor data from these two files")
                .clicked()
            {
                if let Some((s, g)) = wanted {
                    self.start_load(ui.ctx(), s, g);
                }
            }
            if busy {
                ui.spinner();
                let p = &self.progress;
                let (done, total) = (p.done.load(Ordering::Relaxed), p.total.load(Ordering::Relaxed));
                let file = p.file.lock().unwrap().clone();
                if total > 0 && done < total {
                    ui.add(
                        egui::ProgressBar::new(done as f32 / total as f32)
                            .desired_width(220.0)
                            .text(format!("{file} {:.1} / {:.1} MB", done as f64 / 1e6, total as f64 / 1e6)),
                    );
                } else if total > 0 {
                    ui.label("parsing…");
                }
                ui.ctx().request_repaint_after(std::time::Duration::from_millis(200));
            }
        });
        if let Some(e) = &self.error {
            ui.colored_label(egui::Color32::from_rgb(220, 80, 80), e);
        }
        let Some(d) = self.data.as_ref() else {
            ui.add_space(20.0);
            ui.label("Drop a SENSnnn.CSV (and its GPSnnn.CSV) onto the window, or pick them above, then click \u{201c}Show session\u{201d}.");
            return;
        };

        // Summary line.
        let dur = d.duration_s;
        let start = d.clock.label(d.t0_ms, false);
        let end = d.clock.label(d.t0_ms + dur * 1000.0, false);
        ui.horizontal_wrapped(|ui| {
            ui.label(egui::RichText::new(&d.sens_name).strong());
            if let Some(g) = &d.gps_name {
                ui.label(format!("+ {g}"));
            }
            ui.separator();
            if let (Some(s), Some(e)) = (&start, &end) {
                ui.label(format!("{s} – {e}"));
            }
            ui.label(format!("{:.0} min {:02.0} s", (dur / 60.0).floor(), dur % 60.0));
            if !d.track.is_empty() {
                ui.separator();
                ui.label(format!("{:.2} km", d.distance_m / 1000.0));
                ui.label(format!("max {:.1} km/h", d.max_speed_kmh));
                let hz = gps_rate_hz(d);
                if hz > 0.0 {
                    ui.label(format!("GPS {hz:.0} Hz"));
                }
            } else if d.gps_name.is_some() {
                ui.separator();
                ui.label("no GPS fix in this file");
            }
        });

        self.tick_playback(ui.ctx());
        let d = self.data.as_ref().unwrap();

        // Bottom: slider + playback + readout.
        let mut cursor = self.cursor_s;
        let readout = readout_text(d, cursor);
        egui::TopBottomPanel::bottom("session_bottom")
            .resizable(false)
            .show_inside(ui, |ui| {
                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    if ui.button(if self.playing { "Pause" } else { "Play" }).clicked() {
                        self.playing = !self.playing;
                        self.last_frame = None;
                        if self.playing && cursor >= d.duration_s {
                            cursor = 0.0;
                        }
                    }
                    egui::ComboBox::from_id_salt("session_play_speed")
                        .width(60.0)
                        .selected_text(format!("{}×", PLAY_SPEEDS[self.play_speed]))
                        .show_ui(ui, |ui| {
                            for (i, s) in PLAY_SPEEDS.iter().enumerate() {
                                ui.selectable_value(&mut self.play_speed, i, format!("{s}×"));
                            }
                        });
                    let w = (ui.available_width() - 8.0).max(100.0);
                    ui.spacing_mut().slider_width = w;
                    let clock = d.clock.clone();
                    let t0 = d.t0_ms;
                    ui.add(
                        egui::Slider::new(&mut cursor, 0.0..=d.duration_s.max(0.1))
                            .show_value(false)
                            .custom_formatter(move |v, _| {
                                clock.label(t0 + v * 1000.0, true).unwrap_or_else(|| format!("{v:.1} s"))
                            }),
                    );
                });
                ui.label(egui::RichText::new(readout).monospace());
                ui.add_space(2.0);
            });

        // Map left, charts right.
        let has_track = !d.track.is_empty();
        let mut clicked: Option<f64> = None;
        let total_w = ui.available_width();
        egui::SidePanel::left("session_map")
            .resizable(true)
            .default_width(total_w * 0.5)
            .show_inside(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.satellite, "Satellite");
                    ui.checkbox(&mut self.follow, "Follow cursor");
                    if ui.button("Fit track").clicked() {
                        self.fit_pending = true;
                    }
                });
                if !has_track {
                    ui.label("No GPS track to draw.");
                    return;
                }
                self.draw_map(ui, cursor, &mut clicked);
            });
        egui::CentralPanel::default().show_inside(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.checkbox(&mut self.show_speed, "Speed");
                ui.checkbox(&mut self.show_acc, "Acceleration");
                ui.checkbox(&mut self.show_angles, "Pitch / roll");
                ui.checkbox(&mut self.show_height, "Height");
                ui.checkbox(&mut self.show_gyro, "Rotation");
            });
            self.draw_charts(ui, cursor, &mut clicked);
        });

        if let Some(t) = clicked {
            cursor = t;
            self.playing = false;
        }
        self.cursor_s = cursor.clamp(0.0, self.data.as_ref().map_or(0.0, |d| d.duration_s));
    }

    pub fn start_load(&mut self, ctx: &egui::Context, sens: PathBuf, gps: Option<PathBuf>) {
        let (tx, rx) = mpsc::channel();
        let ctx = ctx.clone();
        self.loaded_from = Some((sens.clone(), gps.clone()));
        self.error = None;
        self.progress = Arc::default();
        let progress = self.progress.clone();
        std::thread::spawn(move || {
            let _ = tx.send(load(&sens, gps.as_deref(), &progress));
            ctx.request_repaint();
        });
        self.loading = Some(rx);
    }

    fn poll_loading(&mut self, _ctx: &egui::Context) {
        let Some(rx) = &self.loading else { return };
        match rx.try_recv() {
            Ok(Ok(d)) => {
                self.cursor_s = 0.0;
                self.view = None;
                self.playing = false;
                self.fit_pending = true;
                self.data = Some(d);
                self.loading = None;
            }
            Ok(Err(e)) => {
                self.error = Some(e);
                self.loading = None;
            }
            Err(mpsc::TryRecvError::Empty) => {}
            Err(mpsc::TryRecvError::Disconnected) => {
                self.error = Some("loader thread died".into());
                self.loading = None;
            }
        }
    }

    fn tick_playback(&mut self, ctx: &egui::Context) {
        if !self.playing {
            return;
        }
        let now = Instant::now();
        if let Some(prev) = self.last_frame {
            self.cursor_s += now.duration_since(prev).as_secs_f64() * PLAY_SPEEDS[self.play_speed];
        }
        self.last_frame = Some(now);
        let dur = self.data.as_ref().map_or(0.0, |d| d.duration_s);
        if self.cursor_s >= dur {
            self.cursor_s = dur;
            self.playing = false;
        }
        ctx.request_repaint();
    }

    fn draw_map(&mut self, ui: &mut egui::Ui, cursor: f64, clicked: &mut Option<f64>) {
        let Some(d) = self.data.as_ref() else { return };
        if self.tiles.is_none() {
            self.tiles = Some(OverzoomTiles(HttpTiles::with_options(
                walkers::sources::OpenStreetMap,
                opts("tiles-cache"),
                ui.ctx().clone(),
            )));
        }
        if self.satellite && self.tiles_sat.is_none() {
            self.tiles_sat = Some(OverzoomTiles(HttpTiles::with_options(
                EsriWorldImagery,
                opts("tiles-cache-esri"),
                ui.ctx().clone(),
            )));
        }
        let visible = ui.available_rect_before_wrap().intersect(ui.clip_rect());
        let size = egui::vec2(visible.width(), (visible.height() - 16.0).max(80.0));

        let (mut la0, mut la1, mut lo0, mut lo1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
        for &i in &d.track {
            la0 = la0.min(d.lat[i]);
            la1 = la1.max(d.lat[i]);
            lo0 = lo0.min(d.lon[i]);
            lo1 = lo1.max(d.lon[i]);
        }
        let center = Position::from_lat_lon((la0 + la1) / 2.0, (lo0 + lo1) / 2.0);
        if self.fit_pending {
            self.fit_pending = false;
            self.map_memory.center_at(center);
            // Web-Mercator zoom that fits the bbox with a 15 % margin.
            let lat_c = ((la0 + la1) / 2.0).to_radians();
            let span_x = ((lo1 - lo0) / 360.0).max(1e-7);
            let y = |la: f64| (std::f64::consts::FRAC_PI_4 + la.to_radians() / 2.0).tan().ln();
            let span_y = ((y(la1) - y(la0)) / (2.0 * std::f64::consts::PI)).max(1e-7);
            let _ = lat_c;
            let zx = (size.x as f64 / 256.0 / span_x / 1.15).log2();
            let zy = (size.y as f64 / 256.0 / span_y / 1.15).log2();
            let _ = self.map_memory.set_zoom(zx.min(zy).clamp(2.0, 19.0));
        }
        if self.follow {
            let k = d.track.partition_point(|&i| d.gps_t[i] <= cursor);
            if k > 0 {
                let i = d.track[k - 1];
                self.map_memory.center_at(Position::from_lat_lon(d.lat[i], d.lon[i]));
            }
        }
        if self.map_memory.zoom() > MAX_MAP_ZOOM {
            let _ = self.map_memory.set_zoom(MAX_MAP_ZOOM);
        }
        let tiles = if self.satellite { self.tiles_sat.as_mut() } else { self.tiles.as_mut() };
        let map = Map::new(tiles.map(|t| t as &mut dyn walkers::Tiles), &mut self.map_memory, center)
            .zoom_with_ctrl(false)
            .double_click_to_zoom(true)
            .with_plugin(TrackPlugin { d, cursor_s: cursor, clicked });
        ui.add_sized(size, map);
        ui.small(if self.satellite {
            "© Esri, Maxar, Earthstar Geographics"
        } else {
            "© OpenStreetMap contributors"
        });
    }

    fn draw_charts(&mut self, ui: &mut egui::Ui, cursor: f64, clicked: &mut Option<f64>) {
        let Some(d) = self.data.as_ref() else { return };
        let (lo, hi) = self.view.unwrap_or((0.0, d.duration_s.max(0.1)));
        type Series<'a> = (&'a str, &'a [f64], &'a [f64], egui::Color32);
        let mut charts: Vec<(&str, &str, Vec<Series>)> = Vec::new();
        if self.show_speed && !d.gps_t.is_empty() {
            charts.push(("speed", "km/h", vec![("GPS speed", &d.gps_t, &d.speed_kmh, egui::Color32::from_rgb(80, 160, 255))]));
        }
        if self.show_acc {
            charts.push(("acc", "g", vec![("|acc|", &d.sens_t, &d.acc_g, egui::Color32::from_rgb(255, 140, 60))]));
        }
        if self.show_angles {
            charts.push((
                "angles",
                "°",
                vec![
                    ("pitch (nose up +)", &d.sens_t, &d.pitch_deg, egui::Color32::from_rgb(120, 220, 120)),
                    ("roll", &d.sens_t, &d.roll_deg, egui::Color32::from_rgb(220, 120, 220)),
                ],
            ));
        }
        if self.show_height {
            charts.push(("height", "m", vec![("baro height", &d.sens_t, &d.height_m, egui::Color32::from_rgb(90, 210, 210))]));
        }
        if self.show_gyro {
            charts.push(("gyro", "°/s", vec![("|rotation|", &d.sens_t, &d.gyro_dps, egui::Color32::from_rgb(230, 210, 80))]));
        }
        if charts.is_empty() {
            return;
        }
        let h = ((ui.available_height() - 4.0) / charts.len() as f32 - ui.spacing().item_spacing.y).max(60.0);
        let link = egui::Id::new("session_charts");
        let mut new_view = None;
        let clock = d.clock.clone();
        let t0 = d.t0_ms;
        let n = charts.len();
        for (k, (id, unit, series)) in charts.into_iter().enumerate() {
            let clock = clock.clone();
            let mut plot = Plot::new(("session_plot", id))
                .height(h)
                .link_axis(link, true, false)
                .link_cursor(link, true, false)
                .allow_scroll(false)
                .y_axis_label(unit)
                .y_axis_min_width(40.0)
                .legend(Legend::default().position(Corner::LeftTop))
                .show_axes([k + 1 == n, true])
                .x_axis_formatter(move |m, _| clock.label(t0 + m.value * 1000.0, false).unwrap_or_else(|| format!("{:.0} s", m.value)));
            if self.view.is_none() {
                plot = plot.include_x(0.0).include_x(d.duration_s.max(0.1));
            }
            let resp = plot.show(ui, |pui| {
                for (name, t, y, color) in &series {
                    let pts = decimate(t, y, lo, hi);
                    pui.line(Line::new(PlotPoints::from(pts)).name(*name).color(*color));
                }
                pui.vline(VLine::new(cursor).color(egui::Color32::from_rgb(255, 200, 0)).width(1.5_f32));
                let b = pui.plot_bounds();
                (b.min()[0], b.max()[0])
            });
            new_view = Some(resp.inner);
            if resp.response.clicked() {
                if let Some(p) = resp.response.interact_pointer_pos() {
                    *clicked = Some(resp.transform.value_from_position(p).x);
                }
            }
        }
        if let Some((a, b)) = new_view {
            if b > a {
                self.view = Some((a, b));
            }
        }
    }
}

fn gps_rate_hz(d: &SessionData) -> f64 {
    let mut dts: Vec<f64> = d.track.windows(2).map(|w| d.gps_t[w[1]] - d.gps_t[w[0]]).filter(|v| *v > 0.0).collect();
    if dts.is_empty() {
        return 0.0;
    }
    dts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    1.0 / dts[dts.len() / 2]
}

fn readout_text(d: &SessionData, x: f64) -> String {
    let mut s = d.clock.label(d.t0_ms + x * 1000.0, true).unwrap_or_else(|| format!("{x:.1} s"));
    if let Some(v) = at(&d.gps_t, &d.speed_kmh, x) {
        s += &format!("   {v:5.1} km/h");
    }
    if let (Some(la), Some(lo)) = (at(&d.gps_t, &d.lat, x), at(&d.gps_t, &d.lon, x)) {
        if la != 0.0 || lo != 0.0 {
            s += &format!("   {la:.6}, {lo:.6}");
        }
    }
    if let Some(n) = at(&d.gps_t, &d.nsat, x) {
        s += &format!("   {n:.0} sats");
    }
    if let Some(a) = at(&d.sens_t, &d.acc_g, x) {
        s += &format!("   |acc| {a:.2} g");
    }
    if let (Some(p), Some(r)) = (at(&d.sens_t, &d.pitch_deg, x), at(&d.sens_t, &d.roll_deg, x)) {
        s += &format!("   pitch {p:+.0}°  roll {r:+.0}°");
    }
    if let Some(h) = at(&d.sens_t, &d.height_m, x) {
        s += &format!("   height {h:+.1} m");
    }
    s
}

fn file_pick(ui: &mut egui::Ui, label: &str, target: &mut Option<PathBuf>) {
    ui.label(format!("{label}:"));
    let name = target
        .as_ref()
        .and_then(|p| p.file_name())
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "—".into());
    let resp = ui.label(egui::RichText::new(name).monospace());
    if let Some(p) = target.as_ref() {
        resp.on_hover_text(p.display().to_string());
    }
    if ui.small_button("Browse…").clicked() {
        if let Some(p) = rfd::FileDialog::new().add_filter("CSV", &["csv", "CSV"]).pick_file() {
            *target = Some(p);
        }
    }
    if target.is_some() && ui.small_button("Clear").clicked() {
        *target = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn utc_sod_parses_firmware_format() {
        assert_eq!(utc_sod("161459.20"), Some(16.0 * 3600.0 + 14.0 * 60.0 + 59.2));
        assert_eq!(utc_sod(""), None);
        assert_eq!(utc_sod("256000.00"), None);
    }

    #[test]
    fn decimate_keeps_extremes() {
        let t: Vec<f64> = (0..100_000).map(|i| i as f64 * 0.01).collect();
        let mut y = vec![0.0; t.len()];
        y[54_321] = 9.0;
        y[77_777] = -4.0;
        let pts = decimate(&t, &y, 0.0, 1000.0);
        assert!(pts.len() <= PLOT_BUCKETS * 2);
        assert!(pts.iter().any(|p| p[1] == 9.0));
        assert!(pts.iter().any(|p| p[1] == -4.0));
    }

    /// `SESSION_SENS=…/SENS000.CSV cargo test --release -p movement-logger real_session -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn real_session() {
        let sens = PathBuf::from(std::env::var("SESSION_SENS").expect("SESSION_SENS"));
        let gps = crate::guess_gps_for(&sens);
        let t = Instant::now();
        let d = load(&sens, gps.as_deref(), &Progress::default()).unwrap();
        println!(
            "loaded in {:.1} s: {} sens rows, {} gps rows, {} track pts, {:.0} s, {:.2} km, max {:.1} km/h, GPS {:.1} Hz, start {:?}",
            t.elapsed().as_secs_f64(), d.sens_t.len(), d.gps_t.len(), d.track.len(), d.duration_s,
            d.distance_m / 1000.0, d.max_speed_kmh, gps_rate_hz(&d), d.clock.label(d.t0_ms, false)
        );
        println!("{}", readout_text(&d, d.duration_s / 2.0));
    }

    #[test]
    fn smooth_is_centered_average() {
        assert_eq!(smooth(&[0.0, 3.0, 6.0], 3), vec![1.5, 3.0, 4.5]);
    }
}
