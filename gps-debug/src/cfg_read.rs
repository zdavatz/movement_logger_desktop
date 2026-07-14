//! One-shot UBX-CFG-VALGET readout of the receiver's *live* configuration
//! (RAM layer), key-for-key against Peter's known-good u-center 2 chip
//! export (`known_good.rs`, 393 keys) — every value is printed and any
//! divergence from the known-good reference is marked.
//!
//! Runs over the same two transports as the survey: a USB serial port or
//! the box's BLE GPS bridge (`run_core`'s `Read + Write` contract). Like
//! the survey it is strictly **read-only** — VALGET never writes anything
//! to the receiver.
//!
//! Wire detail: keys are polled in chunks of 32 (request ≈ 140 B, worst-
//! case reply ≈ 400 B — comfortably under the box bridge's 1024 B frame
//! cap and BLE FileCmd's 244 B write cap). A receiver on older firmware
//! NAKs a VALGET containing *any* key it doesn't know, so a NAK'd or
//! timed-out chunk falls back to per-key polls — one unsupported key then
//! costs only itself, not its 31 neighbours.

use anyhow::{Result, anyhow};
use std::collections::HashMap;
use std::io::{Read, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use crate::known_good::{Expected, KNOWN_GOOD};
use crate::survey::{UbxParser, ubx_checksum};

const CFG_VALGET: (u8, u8) = (0x06, 0x8B);
const ACK_NAK: (u8, u8) = (0x05, 0x00);

const CHUNK: usize = 32;
const CHUNK_TIMEOUT: Duration = Duration::from_millis(2500);
const ATTEMPTS: usize = 2;

/// Bytes a config value occupies on the VALGET wire, from the key id's
/// storage-size field (bits 28..30). Size 1 (a single bit) still ships as
/// one byte.
fn key_wire_size(id: u32) -> usize {
    match (id >> 28) & 0x7 {
        1 | 2 => 1,
        3 => 2,
        4 => 4,
        5 => 8,
        _ => 0,
    }
}

/// UBX-CFG-VALGET request for `keys` from the given layer (0 = RAM).
fn valget_frame(keys: &[u32], layer: u8) -> Vec<u8> {
    let payload_len = 4 + 4 * keys.len();
    let mut body = Vec::with_capacity(4 + payload_len);
    body.push(CFG_VALGET.0);
    body.push(CFG_VALGET.1);
    body.push((payload_len & 0xFF) as u8);
    body.push((payload_len >> 8) as u8);
    body.extend_from_slice(&[0x00, layer, 0x00, 0x00]); // version, layer, position
    for k in keys {
        body.extend_from_slice(&k.to_le_bytes());
    }
    let (ck_a, ck_b) = ubx_checksum(&body);
    let mut f = Vec::with_capacity(2 + body.len() + 2);
    f.extend_from_slice(&[0xB5, 0x62]);
    f.extend_from_slice(&body);
    f.push(ck_a);
    f.push(ck_b);
    f
}

/// Decode a VALGET response payload (version 1) into `out` (key → raw
/// little-endian value, zero-extended to u64).
fn parse_valget_reply(p: &[u8], out: &mut HashMap<u32, u64>) {
    if p.len() < 4 || p[0] != 0x01 {
        return;
    }
    let mut o = 4;
    while o + 4 <= p.len() {
        let key = u32::from_le_bytes([p[o], p[o + 1], p[o + 2], p[o + 3]]);
        o += 4;
        let sz = key_wire_size(key);
        if sz == 0 || o + sz > p.len() {
            break;
        }
        let mut raw = [0u8; 8];
        raw[..sz].copy_from_slice(&p[o..o + sz]);
        out.insert(key, u64::from_le_bytes(raw));
        o += sz;
    }
}

enum Poll {
    Ok,
    Nak,
    Timeout,
}

/// Send one VALGET request and collect its reply (or NAK) within
/// `CHUNK_TIMEOUT`. Unrelated frames riding the same link (the box's
/// periodic NAV-PVT during a bridge session) are skipped by class/id.
fn poll_chunk<T: Read + Write>(
    transport: &mut T,
    parser: &mut UbxParser,
    keys: &[u32],
    got: &mut HashMap<u32, u64>,
) -> Result<Poll> {
    transport
        .write_all(&valget_frame(keys, 0))
        .map_err(|e| anyhow!("transport write failed: {e}"))?;
    let _ = transport.flush();

    let mut buf = [0u8; 2048];
    let mut frames: Vec<(u8, u8, Vec<u8>)> = Vec::new();
    let deadline = Instant::now() + CHUNK_TIMEOUT;
    while Instant::now() < deadline {
        match transport.read(&mut buf) {
            Ok(0) => {}
            Ok(n) => {
                for &b in &buf[..n] {
                    parser.push(b, &mut frames);
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => return Err(anyhow!("transport read failed: {e}")),
        }
        for (cls, id, pl) in frames.drain(..) {
            if (cls, id) == CFG_VALGET {
                parse_valget_reply(&pl, got);
                return Ok(Poll::Ok);
            }
            if (cls, id) == ACK_NAK && pl.len() >= 2 && (pl[0], pl[1]) == CFG_VALGET {
                return Ok(Poll::Nak);
            }
        }
    }
    Ok(Poll::Timeout)
}

fn sign_extend(raw: u64, size: usize) -> i64 {
    match size {
        1 => raw as u8 as i8 as i64,
        2 => raw as u16 as i16 as i64,
        4 => raw as u32 as i32 as i64,
        _ => raw as i64,
    }
}

/// Format a raw wire value per the key's u-blox value type.
fn fmt_val(vtype: &str, id: u32, raw: u64) -> String {
    let sz = key_wire_size(id);
    match vtype {
        "R4" => format!("{}", f32::from_bits(raw as u32)),
        "R8" => format!("{}", f64::from_bits(raw)),
        t if t.starts_with('I') => format!("{}", sign_extend(raw, sz)),
        t if t.starts_with('X') => format!("0x{:0w$x}", raw, w = sz * 2),
        _ => format!("{raw}"), // L / U* / E*
    }
}

/// Does the chip's raw value match the known-good expectation?
/// Returns (matches, formatted expected value).
fn compare(vtype: &str, id: u32, raw: u64, expected: &Expected) -> (bool, String) {
    let sz = key_wire_size(id);
    match expected {
        Expected::U(e) => (raw == *e, fmt_expected_u(vtype, sz, *e)),
        Expected::I(e) => (sign_extend(raw, sz) == *e, format!("{e}")),
        Expected::F(e) => {
            let v = if vtype == "R4" {
                f32::from_bits(raw as u32) as f64
            } else {
                f64::from_bits(raw)
            };
            ((v - e).abs() <= e.abs() * 1e-6 + 1e-9, format!("{e}"))
        }
    }
}

fn fmt_expected_u(vtype: &str, sz: usize, e: u64) -> String {
    if vtype.starts_with('X') {
        format!("0x{:0w$x}", e, w = sz * 2)
    } else {
        format!("{e}")
    }
}

/// Group part of an official key name — `CFG-MSGOUT-UBX_NAV_PVT_UART1`
/// → `CFG-MSGOUT` (up to the second hyphen).
fn group_of(name: &str) -> &str {
    let mut hyphens = 0;
    for (i, c) in name.char_indices() {
        if c == '-' {
            hyphens += 1;
            if hyphens == 2 {
                return &name[..i];
            }
        }
    }
    name
}

/// Transport-agnostic config readout. Polls every `KNOWN_GOOD` key from the
/// receiver's RAM layer, then prints one line per key through `on_status`
/// (grouped by config group), marking any value that differs from the
/// known-good export with `***`. Honours `stop` between polls.
pub fn read_config_core<T: Read + Write>(
    transport: &mut T,
    stop: Arc<AtomicBool>,
    on_status: &mut dyn FnMut(String),
) -> Result<()> {
    on_status(format!(
        "reading {} config keys from the receiver (UBX-CFG-VALGET, RAM layer)…",
        KNOWN_GOOD.len()
    ));

    let keys: Vec<u32> = KNOWN_GOOD.iter().map(|k| k.id).collect();
    let mut got: HashMap<u32, u64> = HashMap::new();
    let mut parser = UbxParser::default();
    let n_chunks = keys.len().div_ceil(CHUNK);

    for (ci, chunk) in keys.chunks(CHUNK).enumerate() {
        if stop.load(Ordering::SeqCst) {
            return Err(anyhow!("stopped"));
        }
        let mut ok = false;
        for _ in 0..ATTEMPTS {
            match poll_chunk(transport, &mut parser, chunk, &mut got)? {
                Poll::Ok => {
                    ok = true;
                    break;
                }
                Poll::Nak => break, // deterministic — retrying the chunk won't help
                Poll::Timeout => {}
            }
        }
        if !ok {
            // A VALGET is NAK'd wholesale if ANY key in it is unknown to
            // this receiver firmware — retry key-by-key so only the
            // offending key stays unread.
            on_status(format!(
                "  chunk {}/{} refused — retrying its {} keys one by one…",
                ci + 1,
                n_chunks,
                chunk.len()
            ));
            for &k in chunk {
                if stop.load(Ordering::SeqCst) {
                    return Err(anyhow!("stopped"));
                }
                let _ = poll_chunk(transport, &mut parser, &[k], &mut got)?;
            }
        }
        on_status(format!(
            "  … {}/{} keys read",
            (ci * CHUNK + chunk.len()).min(keys.len()),
            keys.len()
        ));
    }

    // ---- report -----------------------------------------------------------
    let (mut n_match, mut n_diff, mut n_miss) = (0usize, 0usize, 0usize);
    let mut cur_group = "";
    for k in KNOWN_GOOD.iter() {
        let group = group_of(k.name);
        if group != cur_group {
            on_status(format!("[{group}]"));
            cur_group = group;
        }
        let item = k.name.get(group.len() + 1..).unwrap_or(k.name);
        match got.get(&k.id) {
            None => {
                n_miss += 1;
                on_status(format!("  {item:<34} = ?            (no reply)"));
            }
            Some(&raw) => {
                let val = fmt_val(k.vtype, k.id, raw);
                let (same, exp) = compare(k.vtype, k.id, raw, &k.expected);
                if same {
                    n_match += 1;
                    on_status(format!("  {item:<34} = {val}"));
                } else {
                    n_diff += 1;
                    on_status(format!(
                        "  {item:<34} = {val:<12} *** known-good: {exp}"
                    ));
                }
            }
        }
    }
    on_status(format!(
        "config readout done: {} keys · {n_match} match · {n_diff} differ (***) · {n_miss} unread — \
         reference: u-center 2 known-good export (01_known_good_before_reset.ucf)",
        KNOWN_GOOD.len()
    ));
    Ok(())
}

/// Serial-transport entry point (the `gps-debug --read-config` CLI).
pub fn read_config_serial(port: &str, baud: u32) -> Result<()> {
    use anyhow::Context;
    let mut p = serialport::new(port, baud)
        .timeout(Duration::from_millis(50))
        .open()
        .with_context(|| format!("open serial port {port} @ {baud} baud"))?;
    let stop = Arc::new(AtomicBool::new(false));
    {
        let s = stop.clone();
        let _ = ctrlc::set_handler(move || s.store(true, Ordering::SeqCst));
    }
    read_config_core(&mut p, stop, &mut |line| println!("{line}"))
}

/// Open a serial port with the survey/config-read timeout contract — for
/// callers (the GUI's USB transport) that drive `read_config_core` or
/// `run_core` themselves.
pub fn open_serial(port: &str, baud: u32) -> Result<Box<dyn serialport::SerialPort>> {
    use anyhow::Context;
    serialport::new(port, baud)
        .timeout(Duration::from_millis(50))
        .open()
        .with_context(|| format!("open serial port {port} @ {baud} baud"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valget_frame_well_formed() {
        let f = valget_frame(&[0x40520001], 0);
        assert_eq!(&f[..2], &[0xB5, 0x62]);
        assert_eq!((f[2], f[3]), CFG_VALGET);
        assert_eq!(u16::from_le_bytes([f[4], f[5]]), 8); // 4 hdr + 1 key
        assert_eq!(&f[6..10], &[0x00, 0x00, 0x00, 0x00]); // version 0, RAM
        assert_eq!(&f[10..14], &0x40520001u32.to_le_bytes());
        let (a, b) = ubx_checksum(&f[2..f.len() - 2]);
        assert_eq!((a, b), (f[f.len() - 2], f[f.len() - 1]));
    }

    #[test]
    fn valget_reply_mixed_sizes() {
        // version 1, layer 0, position 0, then L(1B)=1, U2=0x1234, U4, X8.
        let mut p = vec![0x01, 0x00, 0x00, 0x00];
        p.extend_from_slice(&0x10310021u32.to_le_bytes());
        p.push(1);
        p.extend_from_slice(&0x30210001u32.to_le_bytes());
        p.extend_from_slice(&100u16.to_le_bytes());
        p.extend_from_slice(&0x40520001u32.to_le_bytes());
        p.extend_from_slice(&230400u32.to_le_bytes());
        p.extend_from_slice(&0x50360006u32.to_le_bytes());
        p.extend_from_slice(&469960u64.to_le_bytes());
        let mut got = HashMap::new();
        parse_valget_reply(&p, &mut got);
        assert_eq!(got.get(&0x10310021), Some(&1));
        assert_eq!(got.get(&0x30210001), Some(&100));
        assert_eq!(got.get(&0x40520001), Some(&230400));
        assert_eq!(got.get(&0x50360006), Some(&469960));
    }

    #[test]
    fn wire_sizes() {
        assert_eq!(key_wire_size(0x10310021), 1); // L
        assert_eq!(key_wire_size(0x20910007), 1); // U1
        assert_eq!(key_wire_size(0x30210001), 2); // U2
        assert_eq!(key_wire_size(0x40520001), 4); // U4
        assert_eq!(key_wire_size(0x50360006), 8); // X8
    }

    #[test]
    fn signed_and_float_formatting() {
        // I1 -5 on the wire.
        assert_eq!(fmt_val("I1", 0x201100a4, 0xFBu64), "-5");
        // R8 298.257223563 round-trips bit-exact.
        let bits = 298.257223563f64.to_bits();
        assert_eq!(fmt_val("R8", 0x50110063, bits), "298.257223563");
        let (ok, _) = compare("R8", 0x50110063, bits, &Expected::F(298.257223563));
        assert!(ok);
    }

    #[test]
    fn group_split() {
        assert_eq!(group_of("CFG-MSGOUT-UBX_NAV_PVT_UART1"), "CFG-MSGOUT");
        assert_eq!(group_of("CFG-ANA-USE_ANA"), "CFG-ANA");
    }

    /// Mock receiver: parses VALGET requests and answers every key with its
    /// known-good value (interleaved with an unrelated NAV-PVT frame, like
    /// the box's periodic PVT during a bridge session).
    struct MockChip {
        pending: Vec<u8>,
    }

    impl std::io::Write for MockChip {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            // Decode the request enough to extract the polled key ids.
            let n_keys = (u16::from_le_bytes([buf[4], buf[5]]) as usize - 4) / 4;
            let mut reply_pl = vec![0x01, 0x00, 0x00, 0x00];
            for i in 0..n_keys {
                let o = 10 + i * 4;
                let id = u32::from_le_bytes([buf[o], buf[o + 1], buf[o + 2], buf[o + 3]]);
                let k = KNOWN_GOOD.iter().find(|k| k.id == id).unwrap();
                let raw: u64 = match k.expected {
                    Expected::U(v) => v,
                    Expected::I(v) => v as u64,
                    Expected::F(v) => {
                        if k.vtype == "R4" {
                            (v as f32).to_bits() as u64
                        } else {
                            v.to_bits()
                        }
                    }
                };
                reply_pl.extend_from_slice(&id.to_le_bytes());
                reply_pl.extend_from_slice(&raw.to_le_bytes()[..key_wire_size(id)]);
            }
            // Unrelated traffic first — the reader must skip it.
            self.pending.extend_from_slice(&mock_frame(0x01, 0x07, &[0u8; 92]));
            self.pending.extend_from_slice(&mock_frame(0x06, 0x8B, &reply_pl));
            Ok(buf.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    impl std::io::Read for MockChip {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            let n = buf.len().min(self.pending.len());
            buf[..n].copy_from_slice(&self.pending[..n]);
            self.pending.drain(..n);
            Ok(n)
        }
    }

    fn mock_frame(cls: u8, id: u8, payload: &[u8]) -> Vec<u8> {
        let mut body = vec![
            cls,
            id,
            (payload.len() & 0xFF) as u8,
            (payload.len() >> 8) as u8,
        ];
        body.extend_from_slice(payload);
        let (a, b) = ubx_checksum(&body);
        let mut f = vec![0xB5, 0x62];
        f.extend_from_slice(&body);
        f.push(a);
        f.push(b);
        f
    }

    #[test]
    fn full_readout_matches_known_good() {
        let mut chip = MockChip { pending: Vec::new() };
        let stop = Arc::new(AtomicBool::new(false));
        let mut lines: Vec<String> = Vec::new();
        read_config_core(&mut chip, stop, &mut |l| lines.push(l)).unwrap();
        let summary = lines.last().unwrap();
        assert!(
            summary.contains(&format!("{} keys · {} match · 0 differ", KNOWN_GOOD.len(), KNOWN_GOOD.len())),
            "unexpected summary: {summary}"
        );
        // Spot-check a value line.
        assert!(lines.iter().any(|l| l.contains("BAUDRATE") && l.contains("230400")));
    }
}
