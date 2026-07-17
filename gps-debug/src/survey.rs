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
//!   * L1 spectrum      — UBX-MON-SPAN, the receiver's built-in coarse
//!                        spectrum analyzer (256 bins across the GNSS band).
//!                        Amplitudes aren't calibrated, but A/B comparisons
//!                        (case open vs. closed, subsystem on vs. off) show
//!                        *which frequency* an interferer sits on — the
//!                        fastest way from "GPS is jammed" to "by that clock".
//!                        M8 receivers don't answer the poll; skipped then.
//!
//! Three CSVs are written: `<label>_gnss_epoch.csv` (one row per second),
//! `<label>_gnss_signals.csv` (one row per tracked signal per second) and
//! `<label>_gnss_spectrum.csv` (one row per second per RF block; only created
//! once the receiver actually answers MON-SPAN). The `--label` ends up in both
//! the filename and a column so you can A/B several antennas/positions and
//! concatenate the runs later.
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
const NAV_PVT:  (u8, u8) = (0x01, 0x07);
const NAV_DOP:  (u8, u8) = (0x01, 0x04);
const NAV_SAT:  (u8, u8) = (0x01, 0x35);
const NAV_SIG:  (u8, u8) = (0x01, 0x43);
const MON_RF:   (u8, u8) = (0x0A, 0x38);
const MON_SPAN: (u8, u8) = (0x0A, 0x31);

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
pub(crate) fn ubx_checksum(body: &[u8]) -> (u8, u8) {
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
pub(crate) struct UbxParser {
    state: u8,
    cls: u8,
    id: u8,
    len: usize,
    payload: Vec<u8>,
    ck_a: u8,
    ck_b: u8,
}

impl UbxParser {
    pub(crate) fn push(&mut self, byte: u8, out: &mut Vec<(u8, u8, Vec<u8>)>) {
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

/// One RF block of a UBX-MON-SPAN reply — the receiver's coarse spectrum
/// snapshot: 256 amplitude bins (uncalibrated) covering `span_hz` around
/// `center_hz`. Single-band MAX-M10S reports one block (L1).
#[derive(Clone)]
struct SpanBlock { spectrum: Vec<u8>, span_hz: u32, res_hz: u32, center_hz: u32, pga_db: u8 }

impl SpanBlock {
    /// Frequency of bin `i` in Hz (bin 0 = center - span/2).
    fn bin_freq_hz(&self, i: usize) -> f64 {
        self.center_hz as f64 - self.span_hz as f64 / 2.0 + i as f64 * self.res_hz as f64
    }
    /// (bin index, amplitude) of the strongest bin.
    fn peak(&self) -> (usize, u8) {
        self.spectrum.iter().copied().enumerate()
            .max_by_key(|&(_, a)| a).map(|(i, a)| (i, a)).unwrap_or((0, 0))
    }
}

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

/// UBX-MON-SPAN: version U1, numRfBlocks U1, reserved U1[2], then per block
/// spectrum U1[256] + span U4 + res U4 + center U4 + pga U1 + reserved U1[3]
/// (272 B/block). Not supported on M8 — those receivers simply never reply.
fn parse_mon_span(p: &[u8]) -> Vec<SpanBlock> {
    let mut v = Vec::new();
    if p.len() < 4 { return v; }
    let n = p[1] as usize;
    for i in 0..n {
        let o = 4 + i * 272;
        if o + 272 > p.len() { break; }
        v.push(SpanBlock {
            spectrum: p[o..o + 256].to_vec(),
            span_hz: u32le(p, o + 256),
            res_hz: u32le(p, o + 260),
            center_hz: u32le(p, o + 264),
            pga_db: p[o + 268],
        });
    }
    v
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
    span: Vec<SpanBlock>,
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
    let spec_path: PathBuf = out_dir.join(format!("{safe_label}_gnss_spectrum.csv"));
    // Created lazily on the first MON-SPAN reply — M8 receivers never answer
    // the poll, and an empty spectrum CSV would read as "no interference".
    let mut spec_w: Option<BufWriter<std::fs::File>> = None;

    let mut epoch_w = BufWriter::new(std::fs::File::create(&epoch_path)
        .with_context(|| format!("create {}", epoch_path.display()))?);
    let mut sig_w = BufWriter::new(std::fs::File::create(&sig_path)
        .with_context(|| format!("create {}", sig_path.display()))?);
    writeln!(epoch_w, "label,host_iso,iTOW_ms,utc_date,utc_time,timeValid,fixType,gnssFixOK,numSV_used,lat_deg,lon_deg,height_m,hMSL_m,hAcc_m,vAcc_m,sAcc_mps,pDOP,hDOP,vDOP,antStatus,antPower,noisePerMS,agcCnt,cwSuppression_jamInd,jammingState")?;
    writeln!(sig_w, "label,host_iso,iTOW_ms,gnssId,gnss,svId,sigId,sig,cno_dbhz,elev_deg,azim_deg,qualityInd,svUsed,prUsed,prRes_m")?;

    on_status(format!("epoch  -> {}", epoch_path.display()));
    on_status(format!("signals-> {}", sig_path.display()));

    let polls = [NAV_PVT, NAV_DOP, NAV_SAT, NAV_SIG, MON_RF, MON_SPAN];
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
                    NAV_PVT  => ep.pvt = parse_nav_pvt(&pl),
                    NAV_DOP  => ep.dop = parse_nav_dop(&pl),
                    NAV_SAT  => ep.sats = parse_nav_sat(&pl),
                    NAV_SIG  => ep.sigs = parse_nav_sig(&pl),
                    MON_RF   => ep.rf  = parse_mon_rf(&pl),
                    MON_SPAN => ep.span = parse_mon_span(&pl),
                    _ => {}
                }
            }
            if stop.load(Ordering::SeqCst) { break; }
        }

        write_epoch(&mut epoch_w, &mut sig_w, label, &ep)?;
        epoch_w.flush()?;
        sig_w.flush()?;
        if !ep.span.is_empty() {
            if spec_w.is_none() {
                let mut w = BufWriter::new(std::fs::File::create(&spec_path)
                    .with_context(|| format!("create {}", spec_path.display()))?);
                writeln!(w, "label,host_iso,iTOW_ms,block,center_hz,span_hz,res_hz,pga_db,peak_bin,peak_amp,peak_freq_hz,bins")?;
                on_status(format!("spectrum-> {}", spec_path.display()));
                spec_w = Some(w);
            }
            if let Some(w) = spec_w.as_mut() {
                write_spectrum(w, label, &ep)?;
                w.flush()?;
            }
        }
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

/// Spectrum rows: one per RF block per epoch. `bins` is the raw 256-value
/// amplitude vector, space-separated inside one CSV field, so plotting tools
/// can `split(' ')` it; peak_* pre-computes the strongest bin for quick sorts.
fn write_spectrum(w: &mut impl Write, label: &str, ep: &Epoch) -> Result<()> {
    let host = now_iso();
    let itow = ep.pvt.as_ref().map(|p| p.itow).unwrap_or(0);
    for (bi, b) in ep.span.iter().enumerate() {
        let (pi, pa) = b.peak();
        let bins: Vec<String> = b.spectrum.iter().map(|a| a.to_string()).collect();
        writeln!(w,
            "{label},{host},{itow},{bi},{center},{span},{res},{pga},{pi},{pa},{pf:.0},{bins}",
            center = b.center_hz, span = b.span_hz, res = b.res_hz, pga = b.pga_db,
            pf = b.bin_freq_hz(pi), bins = bins.join(" "),
        )?;
    }
    Ok(())
}

/// Per-satellite C/N0 of the GPS (gnssId 0) and Galileo (gnssId 2)
/// constellations only, strongest first. NAV-SAT already reports one
/// best-signal C/N0 per satellite; when only NAV-SIG answered, its
/// per-signal rows are folded to max-per-satellite so a dual-band sat
/// isn't counted twice. cno == 0 (searched but not tracked) is skipped.
fn gps_gal_sat_cnos(ep: &Epoch) -> Vec<u8> {
    let mut v: Vec<u8> = if !ep.sats.is_empty() {
        ep.sats.iter()
            .filter(|s| (s.gnss == 0 || s.gnss == 2) && s.cno > 0)
            .map(|s| s.cno).collect()
    } else {
        let mut best: std::collections::HashMap<(u8, u8), u8> = std::collections::HashMap::new();
        for s in ep.sigs.iter().filter(|s| (s.gnss == 0 || s.gnss == 2) && s.cno > 0) {
            let e = best.entry((s.gnss, s.sv)).or_insert(0);
            if s.cno > *e { *e = s.cno; }
        }
        best.into_values().collect()
    };
    v.sort_unstable_by(|a, b| b.cmp(a));
    v
}

/// UBX fixType rendered the way Peter reads it (0/2D/3D…), not as a raw enum.
fn fix_type_name(t: u8) -> &'static str {
    match t { 0 => "none", 1 => "DR", 2 => "2D", 3 => "3D", 4 => "3D+DR", 5 => "time", _ => "?" }
}

/// One-line human-readable summary of an epoch — fed to `on_status` so the
/// CLI can print it and the GUI can append it to the live log panel.
/// Peter's EMI assembly metrics (2026-07-17): `avg6`/`min6`/`max6` are the
/// mean/weakest/strongest of the 6 strongest **GPS + Galileo** satellites
/// (other constellations excluded), plus `used` (sats in the nav solution),
/// `jam` (narrowband jamming), `noise` (broadband noise floor) and `agc`
/// (antenna-signal gain). All collapse within a second of closing the case /
/// powering a noisy subsystem, long before the fix itself reacts.
fn live_summary(ep: &Epoch) -> String {
    let cnos = gps_gal_sat_cnos(ep);
    let top6 = &cnos[..cnos.len().min(6)];
    let avg6 = if top6.is_empty() { 0.0 } else { top6.iter().map(|&c| c as f64).sum::<f64>() / top6.len() as f64 };
    let max6 = top6.first().copied().unwrap_or(0);
    let min6 = top6.last().copied().unwrap_or(0);
    let used = ep.sigs.iter().filter(|s| s.pr_used).count()
        .max(ep.sats.iter().filter(|s| s.sv_used).count());
    match &ep.pvt {
        Some(p) => {
            let rfs = match &ep.rf {
                Some(r) => format!(
                    "ant={}/{} jam={}({}) noise={} agc={}",
                    ant_status_name(r.ant_status), ant_power_name(r.ant_power),
                    r.jam_ind, jamming_name(r.jamming_state), r.noise_per_ms, r.agc_cnt),
                None => "ant=?/? jam=? noise=? agc=?".to_string(),
            };
            let span_s = ep.span.first().map(|b| {
                let (pi, pa) = b.peak();
                format!(" | peak {:.1}MHz a={}", b.bin_freq_hz(pi) / 1e6, pa)
            }).unwrap_or_default();
            format!(
                "{:02}:{:02}:{:02} fix={} ok={} sv={:2} used={:2} avg6={:4.1} min6={:2} max6={:2} hAcc={:.1}m pDOP={:.1} | {rfs}{span_s}",
                p.hour, p.min, p.sec, fix_type_name(p.fix_type), p.gnss_fix_ok as u8,
                p.num_sv, used, avg6, min6, max6, p.hacc_m, p.pdop,
            )
        }
        None => "(no NAV-PVT reply — check port/baud; receiver may be NMEA-only or wrong device)".to_string(),
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
    fn gps_gal_top6_metrics() {
        let sat = |gnss, sv, cno| SatInfo { gnss, sv, cno, elev: 0, azim: 0, pr_res_m: 0.0, qual: 0, sv_used: true };
        let mut ep = Epoch::default();
        ep.sats = vec![
            sat(0, 1, 45), sat(0, 2, 40), sat(2, 3, 42),   // GPS + Galileo
            sat(3, 4, 50), sat(6, 5, 49),                  // BeiDou/GLONASS — excluded
            sat(0, 6, 0),                                  // searched, not tracked — excluded
            sat(2, 7, 38), sat(0, 8, 33), sat(0, 9, 30),
            sat(2, 10, 28),                                // 7th-strongest GPS+GAL — outside top6
        ];
        let cnos = gps_gal_sat_cnos(&ep);
        assert_eq!(cnos, vec![45, 42, 40, 38, 33, 30, 28]);
        let top6 = &cnos[..6];
        assert_eq!((top6[0], top6[5]), (45, 30));

        // NAV-SIG fallback folds dual-band rows to one per satellite.
        let sig = |gnss, sv, sig_id, cno| SigInfo { gnss, sv, sig: sig_id, cno, pr_res_m: 0.0, qual: 0, pr_used: true };
        ep.sats.clear();
        ep.sigs = vec![sig(2, 3, 0, 41), sig(2, 3, 1, 44), sig(0, 1, 0, 39), sig(3, 9, 0, 50)];
        assert_eq!(gps_gal_sat_cnos(&ep), vec![44, 39]);
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
    fn mon_span_offsets() {
        let mut pl = vec![0u8; 4 + 272];
        pl[0] = 0; // version
        pl[1] = 1; // numRfBlocks
        let o = 4;
        pl[o + 100] = 200; // strongest bin at index 100
        pl[o + 256..o + 260].copy_from_slice(&96_000_000u32.to_le_bytes());    // span 96 MHz
        pl[o + 260..o + 264].copy_from_slice(&375_000u32.to_le_bytes());       // res 375 kHz
        pl[o + 264..o + 268].copy_from_slice(&1_575_420_000u32.to_le_bytes()); // center L1
        pl[o + 268] = 12;                                                      // pga dB
        let v = parse_mon_span(&pl);
        assert_eq!(v.len(), 1);
        let b = &v[0];
        assert_eq!(b.span_hz, 96_000_000);
        assert_eq!(b.res_hz, 375_000);
        assert_eq!(b.center_hz, 1_575_420_000);
        assert_eq!(b.pga_db, 12);
        let (pi, pa) = b.peak();
        assert_eq!((pi, pa), (100, 200));
        // bin 0 = center - span/2; bin 100 = that + 100*res
        let f0 = 1_575_420_000f64 - 48_000_000f64;
        assert!((b.bin_freq_hz(100) - (f0 + 100.0 * 375_000.0)).abs() < 1e-6);
        // truncated payload → no blocks
        assert!(parse_mon_span(&pl[..200]).is_empty());
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
