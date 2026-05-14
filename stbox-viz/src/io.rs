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
        .from_path(path)
        .with_context(|| format!("open {}", path.display()))?;

    let hdr = rdr.headers()?.clone();
    let cols = Cols::from_headers(&hdr);

    let i_t = cols.idx_any(&["Time [10ms]", "Time [mS]"])?;
    let i_ax = cols.idx_any(&["AccX [mg]"])?;
    let i_ay = cols.idx_any(&["AccY [mg]"])?;
    let i_az = cols.idx_any(&["AccZ [mg]"])?;
    let i_gx = cols.idx_any(&["GyroX [mdps]"])?;
    let i_gy = cols.idx_any(&["GyroY [mdps]"])?;
    let i_gz = cols.idx_any(&["GyroZ [mdps]"])?;
    let i_mx = cols.idx_any(&["MagX [mgauss]"])?;
    let i_my = cols.idx_any(&["MagY [mgauss]"])?;
    let i_mz = cols.idx_any(&["MagZ [mgauss]"])?;
    let i_p = cols.idx_any(&["P [mB]"])?;
    let i_t_c = cols.idx_any(&["T ['C]"])?;

    let mut out = Vec::new();
    for (n, rec) in rdr.records().enumerate() {
        let rec = rec.with_context(|| format!("row {}", n + 2))?;
        out.push(SensorRow {
            ticks: parse_f64(&rec[i_t])?,
            acc: [parse_f64(&rec[i_ax])?, parse_f64(&rec[i_ay])?, parse_f64(&rec[i_az])?],
            gyro: [parse_f64(&rec[i_gx])?, parse_f64(&rec[i_gy])?, parse_f64(&rec[i_gz])?],
            mag: [parse_f64(&rec[i_mx])?, parse_f64(&rec[i_my])?, parse_f64(&rec[i_mz])?],
            pressure_hpa: parse_f64(&rec[i_p])?,
            temperature_c: parse_f64(&rec[i_t_c])?,
        });
    }
    Ok(out)
}

pub fn load_gps_csv(path: &Path) -> Result<Vec<GpsRow>> {
    let mut rdr = csv::ReaderBuilder::new()
        .trim(csv::Trim::All)
        .from_path(path)
        .with_context(|| format!("open {}", path.display()))?;

    let hdr = rdr.headers()?.clone();
    let cols = Cols::from_headers(&hdr);

    let i_t = cols.idx_any(&["Time [10ms]", "Time [mS]"])?;
    let i_utc = cols.idx_any(&["UTC"])?;
    let i_lat = cols.idx_any(&["Lat"])?;
    let i_lon = cols.idx_any(&["Lon"])?;
    let i_alt = cols.idx_any(&["Alt [m]"])?;
    let i_spd = cols.idx_any(&["Speed [km/h]"])?;
    let i_crs = cols.idx_any(&["Course [deg]"])?;
    let i_fix = cols.idx_any(&["Fix"])?;
    let i_sat = cols.idx_any(&["NumSat"])?;
    let i_hdp = cols.idx_any(&["HDOP"])?;

    let mut out = Vec::new();
    for (n, rec) in rdr.records().enumerate() {
        let rec = rec.with_context(|| format!("row {}", n + 2))?;
        out.push(GpsRow {
            ticks: parse_f64(&rec[i_t])?,
            utc: rec[i_utc].to_string(),
            lat: parse_f64(&rec[i_lat])?,
            lon: parse_f64(&rec[i_lon])?,
            alt_m: parse_f64(&rec[i_alt])?,
            speed_kmh_module: parse_f64(&rec[i_spd])?,
            course_deg: parse_f64(&rec[i_crs])?,
            fix: parse_i32(&rec[i_fix])?,
            num_sat: parse_i32(&rec[i_sat])?,
            hdop: parse_f64(&rec[i_hdp])?,
        });
    }
    Ok(out)
}
