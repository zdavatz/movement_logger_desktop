//! CSV loading for SensorTile.box-Pro SD-card recordings.
//!
//! Sensor CSV columns (22.4.2026+ firmware, with legacy fallbacks):
//!   Time [10ms] | Time [mS]   -- ThreadX ticks, 1 tick = 10 ms
//!   AccX/Y/Z [mg]
//!   GyroX/Y/Z [mdps]
//!   MagX/Y/Z [mgauss]
//!   P [mB]
//!   T ['C]
//!
//! GPS CSV columns:
//!   Time [10ms], UTC, Lat, Lon, Alt [m], Speed [km/h], Course [deg], Fix, NumSat, HDOP

use anyhow::{Context, Result, anyhow};
use std::collections::HashMap;
use std::path::Path;

/// Single 100 Hz sensor sample. Units matches the CSV:
/// acc in mg, gyro in mdps, mag in mgauss, pressure in hPa, temp in °C.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)] // mag is unused in 6DOF fusion (see fusion.rs)
pub struct SensorRow {
    pub ticks: f64,
    pub acc: [f64; 3],
    pub gyro: [f64; 3],
    pub mag: [f64; 3],
    pub pressure_hpa: f64,
    pub temperature_c: f64,
}

/// Single GPS fix row. UTC kept as string ("hhmmss.ss") for display.
#[derive(Debug, Clone)]
#[allow(dead_code)] // module-speed/alt/course/etc read for completeness; used in later phases
pub struct GpsRow {
    pub ticks: f64,
    pub utc: String,
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f64,
    pub speed_kmh_module: f64,
    pub course_deg: f64,
    pub fix: i32,
    pub num_sat: i32,
    pub hdop: f64,
}

/// Header index map for flexible column lookup. Accepts both the old
/// (misnamed) `Time [mS]` and the new `Time [10ms]`.
struct Cols(HashMap<String, usize>);

impl Cols {
    fn from_headers(rec: &csv::StringRecord) -> Self {
        let mut m = HashMap::new();
        for (i, name) in rec.iter().enumerate() {
            m.insert(name.trim().to_string(), i);
        }
        Cols(m)
    }

    fn idx(&self, name: &str) -> Option<usize> {
        self.0.get(name).copied()
    }

    /// Lookup the first matching column name from a list (first match wins).
    fn idx_any(&self, names: &[&str]) -> Result<usize> {
        for n in names {
            if let Some(i) = self.idx(n) {
                return Ok(i);
            }
        }
        Err(anyhow!("missing column, expected one of {:?}", names))
    }
}

/// Resolve the tick column + the divisor that maps it to 10 ms units.
/// Compact schema (`ms`) stores raw milliseconds → ÷10; the legacy spaced
/// `Time [10ms]` / `Time [mS]` column is already in 10 ms ticks → ÷1.
fn tick_col(cols: &Cols) -> Result<(usize, f64)> {
    if let Some(i) = cols.idx("ms") {
        Ok((i, 10.0))
    } else {
        Ok((cols.idx_any(&["Time [10ms]", "Time [mS]"])?, 1.0))
    }
}

fn parse_f64(s: &str) -> Result<f64> {
    s.trim()
        .parse::<f64>()
        .with_context(|| format!("not a float: {:?}", s))
}

fn parse_i32(s: &str) -> Result<i32> {
    s.trim()
        .parse::<i32>()
        .with_context(|| format!("not an int: {:?}", s))
}

pub fn load_sensor_csv(path: &Path) -> Result<Vec<SensorRow>> {
    let mut rdr = csv::ReaderBuilder::new()
        .trim(csv::Trim::All)
        // Skip the firmware's `# SYNC epoch_ms=.. tick_ms=..` marker lines
        // (v0.0.10+) — they're host time-sync anchors, not data rows; pulled
        // separately by `read_sync_anchors`.
        .comment(Some(b'#'))
        .from_path(path)
        .with_context(|| format!("open {}", path.display()))?;

    let hdr = rdr.headers()?.clone();
    let cols = Cols::from_headers(&hdr);

    // Accept BOTH the legacy spaced schema (`Time [10ms]`, `AccX [mg]`, …)
    // and the post-22.4.2026 compact schema (`ms`, `ax_mg`, …). The compact
    // `ms` column is in raw milliseconds, so divide by 10 to keep `ticks` in
    // the 10 ms unit the alignment / fusion code expects.
    let (i_t, tick_div) = tick_col(&cols)?;
    let i_ax = cols.idx_any(&["AccX [mg]", "ax_mg"])?;
    let i_ay = cols.idx_any(&["AccY [mg]", "ay_mg"])?;
    let i_az = cols.idx_any(&["AccZ [mg]", "az_mg"])?;
    let i_gx = cols.idx_any(&["GyroX [mdps]", "gx_mdps"])?;
    let i_gy = cols.idx_any(&["GyroY [mdps]", "gy_mdps"])?;
    let i_gz = cols.idx_any(&["GyroZ [mdps]", "gz_mdps"])?;
    let i_mx = cols.idx_any(&["MagX [mgauss]", "mx_mg"])?;
    let i_my = cols.idx_any(&["MagY [mgauss]", "my_mg"])?;
    let i_mz = cols.idx_any(&["MagZ [mgauss]", "mz_mg"])?;
    let i_p = cols.idx_any(&["P [mB]", "p_hPa"])?;
    let i_t_c = cols.idx_any(&["T ['C]", "t_C"])?;

    let mut out = Vec::new();
    let mut skipped = 0usize;
    for (n, rec) in rdr.records().enumerate() {
        // A torn write on the box — e.g. a power cut mid-row that leaves a
        // stray fragment like a bare "5" — yields a short or non-numeric
        // record. Skip it (with a warning) instead of failing the whole
        // file's replay with `?`.
        let row = (|| -> Option<SensorRow> {
            let rec = rec.as_ref().ok()?;
            Some(SensorRow {
                ticks: parse_f64(rec.get(i_t)?).ok()? / tick_div,
                acc: [
                    parse_f64(rec.get(i_ax)?).ok()?,
                    parse_f64(rec.get(i_ay)?).ok()?,
                    parse_f64(rec.get(i_az)?).ok()?,
                ],
                gyro: [
                    parse_f64(rec.get(i_gx)?).ok()?,
                    parse_f64(rec.get(i_gy)?).ok()?,
                    parse_f64(rec.get(i_gz)?).ok()?,
                ],
                mag: [
                    parse_f64(rec.get(i_mx)?).ok()?,
                    parse_f64(rec.get(i_my)?).ok()?,
                    parse_f64(rec.get(i_mz)?).ok()?,
                ],
                pressure_hpa: parse_f64(rec.get(i_p)?).ok()?,
                temperature_c: parse_f64(rec.get(i_t_c)?).ok()?,
            })
        })();
        match row {
            Some(r) => out.push(r),
            None => {
                skipped += 1;
                if skipped <= 5 {
                    eprintln!("io: {}: skipping malformed row {}", path.display(), n + 2);
                }
            }
        }
    }
    if skipped > 0 {
        eprintln!(
            "io: {}: skipped {} malformed row(s) total",
            path.display(),
            skipped
        );
    }
    Ok(out)
}

pub fn load_gps_csv(path: &Path) -> Result<Vec<GpsRow>> {
    let mut rdr = csv::ReaderBuilder::new()
        .trim(csv::Trim::All)
        .comment(Some(b'#'))
        .from_path(path)
        .with_context(|| format!("open {}", path.display()))?;

    let hdr = rdr.headers()?.clone();
    let cols = Cols::from_headers(&hdr);

    let (i_t, tick_div) = tick_col(&cols)?;
    let i_utc = cols.idx_any(&["UTC", "utc"])?;
    let i_lat = cols.idx_any(&["Lat", "lat"])?;
    let i_lon = cols.idx_any(&["Lon", "lon"])?;
    let i_alt = cols.idx_any(&["Alt [m]", "alt_m"])?;
    let i_spd = cols.idx_any(&["Speed [km/h]", "speed_kmh"])?;
    let i_crs = cols.idx_any(&["Course [deg]", "course_deg"])?;
    let i_fix = cols.idx_any(&["Fix", "fix_q"])?;
    let i_sat = cols.idx_any(&["NumSat", "nsat"])?;
    let i_hdp = cols.idx_any(&["HDOP", "hdop"])?;

    let mut out = Vec::new();
    let mut skipped = 0usize;
    for (n, rec) in rdr.records().enumerate() {
        // Skip torn/garbled rows (see load_sensor_csv) rather than failing
        // the whole file's replay.
        let row = (|| -> Option<GpsRow> {
            let rec = rec.as_ref().ok()?;
            Some(GpsRow {
                ticks: parse_f64(rec.get(i_t)?).ok()? / tick_div,
                utc: rec.get(i_utc)?.to_string(),
                lat: parse_f64(rec.get(i_lat)?).ok()?,
                lon: parse_f64(rec.get(i_lon)?).ok()?,
                alt_m: parse_f64(rec.get(i_alt)?).ok()?,
                speed_kmh_module: parse_f64(rec.get(i_spd)?).ok()?,
                course_deg: parse_f64(rec.get(i_crs)?).ok()?,
                fix: parse_i32(rec.get(i_fix)?).ok()?,
                num_sat: parse_i32(rec.get(i_sat)?).ok()?,
                hdop: parse_f64(rec.get(i_hdp)?).ok()?,
            })
        })();
        match row {
            Some(r) => out.push(r),
            None => {
                skipped += 1;
                if skipped <= 5 {
                    eprintln!("io: {}: skipping malformed row {}", path.display(), n + 2);
                }
            }
        }
    }
    if skipped > 0 {
        eprintln!(
            "io: {}: skipped {} malformed row(s) total",
            path.display(),
            skipped
        );
    }
    Ok(out)
}

/// A host-clock time-sync anchor the firmware stamps into Sens/Gps CSVs on
/// each BLE connect (`SET_TIME` 0x08, v0.0.10+): a `# SYNC epoch_ms=… tick_ms=…`
/// comment line pairing the host's absolute wall-clock millis with the box's
/// free-running ms counter. Because the box has no RTC these are the only
/// drift-free, GPS-independent way to map a logged row's tick to absolute time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SyncAnchor {
    /// Box tick in 10 ms units — the same unit as `SensorRow::ticks` /
    /// `GpsRow::ticks` — so it slots straight into the abs-time interpolation.
    pub ticks: f64,
    /// Host wall-clock epoch milliseconds pushed at this tick.
    pub epoch_ms: i64,
}

/// Extract every `# SYNC epoch_ms=<u64> tick_ms=<u32>` marker from a Sens/Gps
/// CSV. `tick_ms` is the box's raw `HAL_GetTick()` ms (same clock as the
/// `ms` / `Time` column), so we divide by the file's tick divisor to land in
/// the 10 ms row-tick unit. Returns an empty vec for files written by firmware
/// that predates the marker (legacy / never-connected). Mirrors the iOS
/// `CsvParsers.parseSyncAnchors`.
pub fn read_sync_anchors(path: &Path) -> Result<Vec<SyncAnchor>> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("open {}", path.display()))?;
    let mut tick_div = 10.0_f64; // compact `ms` schema → raw ms; ÷10 → 10ms ticks
    let mut saw_header = false;
    let mut out: Vec<SyncAnchor> = Vec::new();
    for raw in text.lines() {
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if !saw_header {
            saw_header = true;
            // Legacy spaced header ("Time [10ms]", …) is already 10ms units.
            if line.to_ascii_lowercase().contains("time [") {
                tick_div = 1.0;
            }
            continue;
        }
        if !line.starts_with('#') || !line.contains("SYNC") {
            continue;
        }
        let mut epoch_ms: Option<i64> = None;
        let mut tick_ms: Option<f64> = None;
        for tok in line.split_whitespace() {
            if let Some(v) = tok.strip_prefix("epoch_ms=") {
                epoch_ms = v.parse::<i64>().ok();
            } else if let Some(v) = tok.strip_prefix("tick_ms=") {
                tick_ms = v.parse::<f64>().ok();
            }
        }
        if let (Some(e), Some(t)) = (epoch_ms, tick_ms) {
            if e > 0 {
                out.push(SyncAnchor { ticks: t / tick_div, epoch_ms: e });
            }
        }
    }
    // Sort + dedupe by tick so the interpolation gets a clean monotone curve.
    out.sort_by(|a, b| a.ticks.partial_cmp(&b.ticks).unwrap_or(std::cmp::Ordering::Equal));
    out.dedup_by(|a, b| a.ticks == b.ticks);
    Ok(out)
}

/// Map row ticks → absolute epoch ms through host-clock `# SYNC` anchors.
/// Piecewise-linear between anchors (drift-free across a session); constant
/// 10 ms/tick extrapolation before the first / after the last anchor. A single
/// anchor degenerates to a fixed 10 ms/tick offset from that one connect.
/// `anchors` must be tick-sorted + deduped (as `read_sync_anchors` returns).
/// Mirrors the iOS `absTimesFromSyncAnchors`.
pub fn abs_times_from_sync_anchors(ticks: &[f64], anchors: &[SyncAnchor]) -> Vec<i64> {
    if ticks.is_empty() || anchors.is_empty() {
        return Vec::new();
    }
    let first = anchors[0];
    let last = anchors[anchors.len() - 1];
    let mut out = Vec::with_capacity(ticks.len());
    let mut j = 0usize;
    for &t in ticks {
        if t <= first.ticks {
            out.push(first.epoch_ms + ((t - first.ticks) * 10.0) as i64);
        } else if t >= last.ticks {
            out.push(last.epoch_ms + ((t - last.ticks) * 10.0) as i64);
        } else {
            while j + 1 < anchors.len() && anchors[j + 1].ticks <= t {
                j += 1;
            }
            let a = anchors[j];
            let b = anchors[j + 1];
            let frac = (t - a.ticks) / (b.ticks - a.ticks);
            out.push(a.epoch_ms + ((b.epoch_ms - a.epoch_ms) as f64 * frac) as i64);
        }
    }
    out
}
