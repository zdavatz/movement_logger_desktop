//! `gps-debug` survey core — live UBX diagnostics readout from a u-blox
//! receiver for antenna selection + mounting evaluation.
//!
//! Connect a u-blox module (MAX-M10S or any M8/M9/M10) to the host over a
//! USB/serial port and this polls it once a second and logs the fields you
//! need to judge an antenna + its mounting spot:
//!
//!   * fix quality      — fixType, gnssFixOK, numSV, hAcc/vAcc/sAcc, p/h/vDOP
//!   * per-signal C/N0  — gnssId, svId, sigId, cno, elev, azim, qualityInd,
//!                        svUsed/prUsed, prRes  (the real antenna-quality signal)
//!   * RF / antenna     — antStatus, antPower, noisePerMS, agcCnt,
//!                        cwSuppression (jamInd), jammingState
//!
//! Two CSVs are written: `<label>_gnss_epoch.csv` (one row per second) and
//! `<label>_gnss_signals.csv` (one row per tracked signal per second). The
//! `--label` ends up in both the filename and a column so you can A/B several
//! antennas/positions and concatenate the runs later.
//!
//! **Non-destructive:** we only ever *poll* (send zero-length UBX requests).
//! The receiver's own message rates / configuration are never written, so this
//! is safe to run against the box's GPS or a bench module without changing how
//! it behaves afterwards. NMEA the receiver may already be emitting is ignored
//! (we only decode UBX frames, sync `B5 62`).

use anyhow::{Context, Result, anyhow};
use std::io::{Read, Write, BufWriter};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

// ---- UBX message identifiers (class, id) --------------------------------
const NAV_PVT: (u8, u8) = (0x01, 0x07);
const NAV_DOP: (u8, u8) = (0x01, 0x04);
const NAV_SAT: (u8, u8) = (0x01, 0x35);
const NAV_SIG: (u8, u8) = (0x01, 0x43);
const MON_RF:  (u8, u8) = (0x0A, 0x38);

// ---- CLI args -----------------------------------------------------------
pub struct SurveyArgs<'a> {
    pub port: &'a str,
    pub baud: u32,
    pub out_dir: &'a Path,
    pub label: &'a str,
    pub duration_s: Option<f64>,
}

// ---- UBX framing --------------------------------------------------------

/// 8-bit Fletcher checksum over `class .. payload-end` (UBX spec).
fn ubx_checksum(body: &[u8]) -> (u8, u8) {
    let (mut a, mut b) = (0u8, 0u8);
    for &x in body {
        a = a.wrapping_add(x);
        b = b.wrapping_add(a);
    }
    (a, b)
}

/// Build a poll request: a frame with the target class/id and empty payload.
/// For the NAV-* and MON-RF messages this asks the receiver to emit the
/// current value once, regardless of its configured periodic rate.
fn poll_frame((cls, id): (u8, u8)) -> [u8; 8] {
    let body = [cls, id, 0x00, 0x00];
    let (ck_a, ck_b) = ubx_checksum(&body);
    [0xB5, 0x62, cls, id, 0x00, 0x00, ck_a, ck_b]
}

/// Incremental UBX frame extractor. Feed it raw bytes; it yields decoded
/// `(class, id, payload)` frames as their checksums verify. Non-UBX bytes
/// (NMEA, noise) are skipped by the sync-word hunt.
#[derive(Default)]
struct UbxParser {
    state: u8,
    cls: u8,
    id: u8,
    len: usize,
    payload: Vec<u8>,
    ck_a: u8,
    ck_b: u8,
}

impl UbxParser {
    fn push(&mut self, byte: u8, out: &mut Vec<(u8, u8, Vec<u8>)>) {
        match self.state {
            0 => if byte == 0xB5 { self.state = 1; },
            // Saw 0xB5. 0x62 completes the sync word; a repeated 0xB5 keeps us
            // armed (the previous byte was a false sync); anything else resets.
            1 => self.state = match byte { 0x62 => 2, 0xB5 => 1, _ => 0 },
            2 => { self.cls = byte; self.state = 3; }
            3 => { self.id = byte; self.state = 4; }
            4 => { self.len = byte as usize; self.state = 5; }
            5 => {
                self.len |= (byte as usize) << 8;
                self.payload.clear();
                // Guard against a corrupt length blowing up memory.
                if self.len > 4096 { self.state = 0; } else { self.state = 6; }
            }
            6 => {
                self.payload.push(byte);
                if self.payload.len() >= self.len { self.state = 7; }
            }
            7 => { self.ck_a = byte; self.state = 8; }
            8 => {
                self.ck_b = byte;
                let mut body = Vec::with_capacity(4 + self.len);
                body.push(self.cls);
                body.push(self.id);
                body.push((self.len & 0xFF) as u8);
                body.push(((self.len >> 8) & 0xFF) as u8);
                body.extend_from_slice(&self.payload);
                let (a, b) = ubx_checksum(&body);
                if a == self.ck_a && b == self.ck_b {
                    out.push((self.cls, self.id, std::mem::take(&mut self.payload)));
                }
                self.state = 0;
            }
            _ => self.state = 0,
        }
    }
}

// ---- little-endian field readers ---------------------------------------
fn u16le(b: &[u8], o: usize) -> u16 { u16::from_le_bytes([b[o], b[o + 1]]) }
fn u32le(b: &[u8], o: usize) -> u32 { u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]) }
fn i16le(b: &[u8], o: usize) -> i16 { i16::from_le_bytes([b[o], b[o + 1]]) }
fn i32le(b: &[u8], o: usize) -> i32 { i32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]) }

// ---- decoded structures -------------------------------------------------
#[derive(Clone, Default)]
struct NavPvt {
    itow: u32,
    year: u16, month: u8, day: u8, hour: u8, min: u8, sec: u8, valid: u8,
    fix_type: u8, gnss_fix_ok: bool, num_sv: u8,
    lon_deg: f64, lat_deg: f64, height_m: f64, hmsl_m: f64,
    hacc_m: f64, vacc_m: f64, sacc_mps: f64, pdop: f64,
}

// pDOP is taken from NAV-PVT; NAV-DOP supplies the h/v components NAV-PVT lacks.
#[derive(Clone, Default)]
struct NavDop { hdop: f64, vdop: f64 }

#[derive(Clone)]
struct SatInfo { gnss: u8, sv: u8, cno: u8, elev: i8, azim: i16, pr_res_m: f64, qual: u8, sv_used: bool }

#[derive(Clone)]
struct SigInfo { gnss: u8, sv: u8, sig: u8, cno: u8, pr_res_m: f64, qual: u8, pr_used: bool }

#[derive(Clone, Default)]
struct MonRf { ant_status: u8, ant_power: u8, noise_per_ms: u16, agc_cnt: u16, jam_ind: u8, jamming_state: u8 }

fn parse_nav_pvt(p: &[u8]) -> Option<NavPvt> {
    if p.len() < 78 { return None; }
    Some(NavPvt {
        itow: u32le(p, 0),
        year: u16le(p, 4), month: p[6], day: p[7], hour: p[8], min: p[9], sec: p[10], valid: p[11],
        fix_type: p[20], gnss_fix_ok: (p[21] & 0x01) != 0, num_sv: p[23],
        lon_deg: i32le(p, 24) as f64 * 1e-7,
        lat_deg: i32le(p, 28) as f64 * 1e-7,
        height_m: i32le(p, 32) as f64 / 1000.0,
        hmsl_m:   i32le(p, 36) as f64 / 1000.0,
        hacc_m:   u32le(p, 40) as f64 / 1000.0,
        vacc_m:   u32le(p, 44) as f64 / 1000.0,
        sacc_mps: u32le(p, 68) as f64 / 1000.0,
        pdop:     u16le(p, 76) as f64 * 0.01,
    })
}

fn parse_nav_dop(p: &[u8]) -> Option<NavDop> {
    if p.len() < 18 { return None; }
    Some(NavDop { vdop: u16le(p, 10) as f64 * 0.01, hdop: u16le(p, 12) as f64 * 0.01 })
}

fn parse_nav_sat(p: &[u8]) -> Vec<SatInfo> {
    let mut v = Vec::new();
    if p.len() < 8 { return v; }
    let n = p[5] as usize;
    for i in 0..n {
        let o = 8 + i * 12;
        if o + 12 > p.len() { break; }
        let flags = u32le(p, o + 8);
        v.push(SatInfo {
            gnss: p[o], sv: p[o + 1], cno: p[o + 2], elev: p[o + 3] as i8,
            azim: i16le(p, o + 4), pr_res_m: i16le(p, o + 6) as f64 * 0.1,
            qual: (flags & 0x07) as u8, sv_used: (flags & 0x08) != 0,
        });
    }
    v
}

fn parse_nav_sig(p: &[u8]) -> Vec<SigInfo> {
    let mut v = Vec::new();
    if p.len() < 8 { return v; }
    let n = p[5] as usize;
    for i in 0..n {
        let o = 8 + i * 16;
        if o + 16 > p.len() { break; }
        let sig_flags = u16le(p, o + 10);
        v.push(SigInfo {
            gnss: p[o], sv: p[o + 1], sig: p[o + 2],
            pr_res_m: i16le(p, o + 4) as f64 * 0.1, cno: p[o + 6], qual: p[o + 7],
            pr_used: (sig_flags & 0x08) != 0,
        });
    }
    v
}

fn parse_mon_rf(p: &[u8]) -> Option<MonRf> {
    if p.len() < 4 { return None; }
    let n = p[1] as usize;
    if n == 0 || p.len() < 4 + 24 { return None; }
    // Report the first RF block (single-band MAX-M10S has one: L1).
    let o = 4;
    Some(MonRf {
        jamming_state: p[o + 1] & 0x03, ant_status: p[o + 2], ant_power: p[o + 3],
        noise_per_ms: u16le(p, o + 12), agc_cnt: u16le(p, o + 14), jam_ind: p[o + 16],
    })
}

// ---- enum → text decoders ----------------------------------------------
fn gnss_name(id: u8) -> &'static str {
    match id { 0 => "GPS", 1 => "SBAS", 2 => "Galileo", 3 => "BeiDou", 4 => "IMES", 5 => "QZSS", 6 => "GLONASS", 7 => "NavIC", _ => "?" }
}
fn sig_name(gnss: u8, sig: u8) -> &'static str {
    match (gnss, sig) {
        (0, 0) => "L1C/A", (0, 3) => "L2CL", (0, 4) => "L2CM",
        (1, 0) => "L1C/A",
        (2, 0) => "E1C", (2, 1) => "E1B", (2, 5) => "E5bI", (2, 6) => "E5bQ",
        (3, 0) => "B1ID1", (3, 1) => "B1ID2", (3, 2) => "B2ID1", (3, 3) => "B2ID2",
        (5, 0) => "L1C/A", (5, 1) => "L1S", (5, 4) => "L2CM", (5, 5) => "L2CL",
        (6, 0) => "L1OF", (6, 2) => "L2OF",
        (7, 0) => "L5A",
        _ => "sig?",
    }
}
fn ant_status_name(s: u8) -> &'static str {
    match s { 0 => "INIT", 1 => "UNKNOWN", 2 => "OK", 3 => "SHORT", 4 => "OPEN", _ => "?" }
}
fn ant_power_name(s: u8) -> &'static str {
    match s { 0 => "OFF", 1 => "ON", 2 => "UNKNOWN", _ => "?" }
}
fn jamming_name(s: u8) -> &'static str {
    match s { 0 => "unknown", 1 => "ok", 2 => "warning", 3 => "critical", _ => "?" }
}

// ---- one polled epoch ---------------------------------------------------
#[derive(Default)]
struct Epoch {
    pvt: Option<NavPvt>,
    dop: Option<NavDop>,
    sats: Vec<SatInfo>,
    sigs: Vec<SigInfo>,
    rf: Option<MonRf>,
}

fn now_iso() -> String {
    chrono::Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string()
}

/// Serial-transport entry point (the `gps-debug` CLI and the GUI's USB mode).
/// Opens the named serial port, installs a Ctrl-C stop, and drives the
/// transport-agnostic [`run_core`] with stdout as the status sink.
pub fn run(args: &SurveyArgs) -> Result<()> {
    let mut port = serialport::new(args.port, args.baud)
        .timeout(Duration::from_millis(50))
        .open()
        .with_context(|| format!(
            "open serial port {} @ {} baud\n\
             hint: list candidates with  ls /dev/cu.* (macOS)  or  ls /dev/ttyACM* /dev/ttyUSB* (Linux)",
            args.port, args.baud))?;

    let stop = Arc::new(AtomicBool::new(false));
    {
        let s = stop.clone();
        let _ = ctrlc::set_handler(move || s.store(true, Ordering::SeqCst));
    }

    println!("gnss-survey: polling {} @ {} baud — label '{}'", args.port, args.baud, args.label);
    run_core(&mut port, args.out_dir, args.label, args.duration_s, stop,
             &mut |line| println!("{line}"))
}

/// Transport-agnostic survey loop. `transport` is anything that reads UBX
/// bytes and accepts UBX poll writes — a `serialport` handle over USB, or a
/// BLE bridge that tunnels the same bytes through the box (see the GUI's
/// `BleTransport`). Writes the two CSVs into `out_dir` and emits one live
/// summary line per epoch through `on_status` (stdout for the CLI, the log
/// panel for the GUI). Runs until `stop` is set or `duration_s` elapses.
///
/// The `transport` read contract matches `serialport`: return `Ok(0)` or a
/// `TimedOut` error when no bytes are ready within a short (~50 ms) window so
/// the collect loop can re-check `stop` instead of blocking indefinitely.
pub fn run_core<T: Read + Write>(
    transport: &mut T,
    out_dir: &Path,
    label: &str,
    duration_s: Option<f64>,
    stop: Arc<AtomicBool>,
    on_status: &mut dyn FnMut(String),
) -> Result<()> {
    std::fs::create_dir_all(out_dir)
        .with_context(|| format!("mkdir {}", out_dir.display()))?;
    let safe_label: String = label.chars()
        .map(|c| if c.is_ascii_alphanumeric() || c == '-' || c == '_' { c } else { '_' })
        .collect();
    let epoch_path: PathBuf = out_dir.join(format!("{safe_label}_gnss_epoch.csv"));
    let sig_path: PathBuf = out_dir.join(format!("{safe_label}_gnss_signals.csv"));

    let mut epoch_w = BufWriter::new(std::fs::File::create(&epoch_path)
        .with_context(|| format!("create {}", epoch_path.display()))?);
    let mut sig_w = BufWriter::new(std::fs::File::create(&sig_path)
        .with_context(|| format!("create {}", sig_path.display()))?);
    writeln!(epoch_w, "label,host_iso,iTOW_ms,utc_date,utc_time,timeValid,fixType,gnssFixOK,numSV_used,lat_deg,lon_deg,height_m,hMSL_m,hAcc_m,vAcc_m,sAcc_mps,pDOP,hDOP,vDOP,antStatus,antPower,noisePerMS,agcCnt,cwSuppression_jamInd,jammingState")?;
    writeln!(sig_w, "label,host_iso,iTOW_ms,gnssId,gnss,svId,sigId,sig,cno_dbhz,elev_deg,azim_deg,qualityInd,svUsed,prUsed,prRes_m")?;

    on_status(format!("epoch  -> {}", epoch_path.display()));
    on_status(format!("signals-> {}", sig_path.display()));

    let polls = [NAV_PVT, NAV_DOP, NAV_SAT, NAV_SIG, MON_RF];
    let mut parser = UbxParser::default();
    let mut buf = [0u8; 2048];
    let start = Instant::now();
    let mut n_epochs: u64 = 0;

    while !stop.load(Ordering::SeqCst) {
        if let Some(d) = duration_s {
            if start.elapsed().as_secs_f64() >= d { break; }
        }

        // Fire all polls for this second.
        for &m in &polls {
            if let Err(e) = transport.write_all(&poll_frame(m)) {
                return Err(anyhow!("transport write failed: {e}"));
            }
        }
        let _ = transport.flush();

        // Collect replies for ~900 ms; keep the latest of each type.
        let mut ep = Epoch::default();
        let mut frames: Vec<(u8, u8, Vec<u8>)> = Vec::new();
        let window = Instant::now() + Duration::from_millis(900);
        while Instant::now() < window {
            match transport.read(&mut buf) {
                Ok(0) => {}
                Ok(n) => for &byte in &buf[..n] { parser.push(byte, &mut frames); },
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => return Err(anyhow!("transport read failed: {e}")),
            }
            for (cls, id, pl) in frames.drain(..) {
                match (cls, id) {
                    NAV_PVT => ep.pvt = parse_nav_pvt(&pl),
                    NAV_DOP => ep.dop = parse_nav_dop(&pl),
                    NAV_SAT => ep.sats = parse_nav_sat(&pl),
                    NAV_SIG => ep.sigs = parse_nav_sig(&pl),
                    MON_RF  => ep.rf  = parse_mon_rf(&pl),
                    _ => {}
                }
            }
            if stop.load(Ordering::SeqCst) { break; }
        }

        write_epoch(&mut epoch_w, &mut sig_w, label, &ep)?;
        epoch_w.flush()?;
        sig_w.flush()?;
        n_epochs += 1;
        on_status(live_summary(&ep));
    }

    on_status(format!("stopped after {n_epochs} epoch(s)."));
    Ok(())
}

fn write_epoch(
    epoch_w: &mut impl Write,
    sig_w: &mut impl Write,
    label: &str,
    ep: &Epoch,
) -> Result<()> {
    let host = now_iso();
    let pvt = ep.pvt.clone().unwrap_or_default();
    let dop = ep.dop.clone().unwrap_or_default();
    let rf = ep.rf.clone().unwrap_or_default();
    let itow = pvt.itow;

    let (date, time) = if ep.pvt.is_some() {
        (format!("{:04}-{:02}-{:02}", pvt.year, pvt.month, pvt.day),
         format!("{:02}:{:02}:{:02}", pvt.hour, pvt.min, pvt.sec))
    } else { (String::new(), String::new()) };

    if ep.pvt.is_some() {
        writeln!(epoch_w,
            "{label},{host},{itow},{date},{time},0x{valid:02X},{fix},{ok},{nsv},\
             {lat:.7},{lon:.7},{h:.3},{hmsl:.3},{hacc:.3},{vacc:.3},{sacc:.3},\
             {pdop:.2},{hdop:.2},{vdop:.2},{astat},{apow},{noise},{agc},{jam},{jstate}",
            valid = pvt.valid, fix = pvt.fix_type, ok = pvt.gnss_fix_ok as u8, nsv = pvt.num_sv,
            lat = pvt.lat_deg, lon = pvt.lon_deg, h = pvt.height_m, hmsl = pvt.hmsl_m,
            hacc = pvt.hacc_m, vacc = pvt.vacc_m, sacc = pvt.sacc_mps,
            pdop = pvt.pdop, hdop = dop.hdop, vdop = dop.vdop,
            astat = ant_status_name(rf.ant_status), apow = ant_power_name(rf.ant_power),
            noise = rf.noise_per_ms, agc = rf.agc_cnt, jam = rf.jam_ind,
            jstate = jamming_name(rf.jamming_state),
        )?;
    }

    // Per-signal rows: NAV-SIG is the per-signal truth (sigId/cno/qual/prUsed);
    // elev/azim/svUsed come from the matching NAV-SAT satellite. Fall back to
    // per-satellite rows if the receiver answered NAV-SAT but not NAV-SIG.
    let sat_of = |g: u8, s: u8| ep.sats.iter().find(|x| x.gnss == g && x.sv == s).cloned();
    if !ep.sigs.is_empty() {
        for s in &ep.sigs {
            let sat = sat_of(s.gnss, s.sv);
            let (elev, azim, sv_used) = match &sat {
                Some(a) => (a.elev.to_string(), a.azim.to_string(), a.sv_used as u8),
                None => (String::new(), String::new(), 0),
            };
            writeln!(sig_w,
                "{label},{host},{itow},{g},{gn},{sv},{sid},{sn},{cno},{elev},{azim},{q},{svu},{pru},{pr:.1}",
                g = s.gnss, gn = gnss_name(s.gnss), sv = s.sv, sid = s.sig, sn = sig_name(s.gnss, s.sig),
                cno = s.cno, q = s.qual, svu = sv_used, pru = s.pr_used as u8, pr = s.pr_res_m,
            )?;
        }
    } else {
        for a in &ep.sats {
            writeln!(sig_w,
                "{label},{host},{itow},{g},{gn},{sv},,,{cno},{elev},{azim},{q},{svu},,{pr:.1}",
                g = a.gnss, gn = gnss_name(a.gnss), sv = a.sv, cno = a.cno,
                elev = a.elev, azim = a.azim, q = a.qual, svu = a.sv_used as u8, pr = a.pr_res_m,
            )?;
        }
    }
    Ok(())
}

/// One-line human-readable summary of an epoch — fed to `on_status` so the
/// CLI can print it and the GUI can append it to the live log panel.
fn live_summary(ep: &Epoch) -> String {
    let max_cno = ep.sigs.iter().map(|s| s.cno).chain(ep.sats.iter().map(|s| s.cno)).max().unwrap_or(0);
    let used = ep.sigs.iter().filter(|s| s.pr_used).count()
        .max(ep.sats.iter().filter(|s| s.sv_used).count());
    match (&ep.pvt, &ep.rf) {
        (Some(p), rf) => {
            let (astat, apow, jam) = rf.as_ref()
                .map(|r| (ant_status_name(r.ant_status), ant_power_name(r.ant_power), jamming_name(r.jamming_state)))
                .unwrap_or(("?", "?", "?"));
            format!(
                "{:02}:{:02}:{:02} fix={} ok={} sv={:2} used={:2} maxCN0={:2} hAcc={:.1}m pDOP={:.1} | ant={}/{} jam={}",
                p.hour, p.min, p.sec, p.fix_type, p.gnss_fix_ok as u8,
                p.num_sv, used, max_cno, p.hacc_m, p.pdop, astat, apow, jam,
            )
        }
        (None, _) => "(no NAV-PVT reply — check port/baud; receiver may be NMEA-only or wrong device)".to_string(),
    }
}

/// Enumerate likely u-blox serial ports for the host, best-guess and
/// non-invasive (no port is opened). macOS exposes USB-CDC/USB-serial
/// adapters as `/dev/cu.usbserial-*` / `/dev/cu.usbmodem-*` (plus the common
/// SiLabs/CH34x vendor names); Linux as `/dev/ttyACM*` / `/dev/ttyUSB*`.
/// Windows enumeration needs the registry and isn't done here — the user
/// types `COMx` (rare to have more than one). Returned paths are sorted and
/// ready to pass straight to `--port`.
pub fn detect_ports() -> Vec<String> {
    let mut out: Vec<String> = Vec::new();

    #[cfg(target_os = "macos")]
    let (dir, matches): (&str, fn(&str) -> bool) = ("/dev", |n| {
        n.starts_with("cu.usbserial")
            || n.starts_with("cu.usbmodem")
            || n.starts_with("cu.SLAB_USBtoUART")
            || n.starts_with("cu.wchusbserial")
    });
    #[cfg(target_os = "linux")]
    let (dir, matches): (&str, fn(&str) -> bool) = ("/dev", |n| {
        n.starts_with("ttyACM") || n.starts_with("ttyUSB")
    });
    #[cfg(not(any(target_os = "macos", target_os = "linux")))]
    let (dir, matches): (&str, fn(&str) -> bool) = ("", |_| false);

    if !dir.is_empty() {
        if let Ok(rd) = std::fs::read_dir(dir) {
            for ent in rd.flatten() {
                if let Some(name) = ent.file_name().to_str() {
                    if matches(name) {
                        out.push(format!("{dir}/{name}"));
                    }
                }
            }
        }
    }
    out.sort();
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a full UBX frame around a class/id/payload.
    fn frame(cls: u8, id: u8, payload: &[u8]) -> Vec<u8> {
        let len = payload.len();
        let mut body = vec![cls, id, (len & 0xFF) as u8, ((len >> 8) & 0xFF) as u8];
        body.extend_from_slice(payload);
        let (a, b) = ubx_checksum(&body);
        let mut f = vec![0xB5, 0x62];
        f.extend_from_slice(&body);
        f.push(a);
        f.push(b);
        f
    }

    #[test]
    fn checksum_known_vector() {
        // UBX-MON-VER poll: B5 62 0A 04 00 00 0E 34 (ck = 0x0E, 0x34).
        assert_eq!(ubx_checksum(&[0x0A, 0x04, 0x00, 0x00]), (0x0E, 0x34));
    }

    #[test]
    fn poll_frame_is_well_formed() {
        let f = poll_frame(NAV_DOP);
        assert_eq!(&f[..6], &[0xB5, 0x62, 0x01, 0x04, 0x00, 0x00]);
        assert_eq!((f[6], f[7]), (0x05, 0x10));
    }

    #[test]
    fn parser_skips_garbage_then_decodes() {
        let f = frame(0x01, 0x04, &[0xAA; 18]);
        // Prepend NMEA-ish noise + a false sync byte to exercise the hunt.
        let mut stream = b"$GPGGA,junk\r\n\xB5".to_vec();
        stream.extend_from_slice(&f);
        let mut p = UbxParser::default();
        let mut out = Vec::new();
        for b in stream { p.push(b, &mut out); }
        assert_eq!(out.len(), 1);
        assert_eq!((out[0].0, out[0].1), (0x01, 0x04));
    }

    #[test]
    fn parser_rejects_bad_checksum() {
        let mut f = frame(0x01, 0x07, &[0u8; 92]);
        let n = f.len();
        f[n - 1] ^= 0xFF; // corrupt ck_b
        let mut p = UbxParser::default();
        let mut out = Vec::new();
        for b in f { p.push(b, &mut out); }
        assert!(out.is_empty());
    }

    #[test]
    fn nav_dop_offsets() {
        let mut pl = [0u8; 18];
        pl[10..12].copy_from_slice(&175u16.to_le_bytes()); // vDOP = 1.75
        pl[12..14].copy_from_slice(&90u16.to_le_bytes());  // hDOP = 0.90
        let d = parse_nav_dop(&pl).unwrap();
        assert!((d.vdop - 1.75).abs() < 1e-9);
        assert!((d.hdop - 0.90).abs() < 1e-9);
    }

    #[test]
    fn nav_pvt_offsets() {
        let mut pl = [0u8; 92];
        pl[20] = 3;                                          // fixType = 3D
        pl[21] = 0x01;                                       // gnssFixOK
        pl[23] = 11;                                         // numSV
        pl[28..32].copy_from_slice(&473_000_000i32.to_le_bytes()); // lat 47.3°
        pl[40..44].copy_from_slice(&1500u32.to_le_bytes());  // hAcc 1.5 m
        pl[68..72].copy_from_slice(&300u32.to_le_bytes());   // sAcc 0.30 m/s
        pl[76..78].copy_from_slice(&140u16.to_le_bytes());   // pDOP 1.40
        let v = parse_nav_pvt(&pl).unwrap();
        assert_eq!(v.fix_type, 3);
        assert!(v.gnss_fix_ok);
        assert_eq!(v.num_sv, 11);
        assert!((v.lat_deg - 47.3).abs() < 1e-6);
        assert!((v.hacc_m - 1.5).abs() < 1e-9);
        assert!((v.sacc_mps - 0.30).abs() < 1e-9);
        assert!((v.pdop - 1.40).abs() < 1e-9);
    }

    #[test]
    fn mon_rf_offsets() {
        let mut pl = vec![0u8; 4 + 24];
        pl[1] = 1;                 // nBlocks
        pl[4 + 1] = 0x02;          // flags: jammingState = 2 (warning)
        pl[4 + 2] = 2;             // antStatus = OK
        pl[4 + 3] = 1;             // antPower = ON
        pl[4 + 12..4 + 14].copy_from_slice(&123u16.to_le_bytes()); // noisePerMS
        pl[4 + 14..4 + 16].copy_from_slice(&4096u16.to_le_bytes()); // agcCnt
        pl[4 + 16] = 7;            // jamInd
        let r = parse_mon_rf(&pl).unwrap();
        assert_eq!(r.jamming_state, 2);
        assert_eq!(ant_status_name(r.ant_status), "OK");
        assert_eq!(ant_power_name(r.ant_power), "ON");
        assert_eq!(r.noise_per_ms, 123);
        assert_eq!(r.agc_cnt, 4096);
        assert_eq!(r.jam_ind, 7);
    }

    #[test]
    fn nav_sat_flags() {
        let mut pl = vec![0u8; 8 + 12];
        pl[5] = 1; // numSvs
        let o = 8;
        pl[o] = 0; pl[o + 1] = 5; pl[o + 2] = 44; // GPS sv5, cno 44
        pl[o + 3] = 30i8 as u8;                   // elev 30
        pl[o + 4..o + 6].copy_from_slice(&120i16.to_le_bytes()); // azim 120
        pl[o + 8..o + 12].copy_from_slice(&((4u32) | (1 << 3)).to_le_bytes()); // qual=4, svUsed
        let s = parse_nav_sat(&pl);
        assert_eq!(s.len(), 1);
        assert_eq!(s[0].cno, 44);
        assert_eq!(s[0].elev, 30);
        assert_eq!(s[0].azim, 120);
        assert_eq!(s[0].qual, 4);
        assert!(s[0].sv_used);
    }
}
