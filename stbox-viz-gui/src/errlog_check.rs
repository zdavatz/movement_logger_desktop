//! Automatic per-boot health check of the box's mirrored `ERRLOG.LOG`.
//!
//! The firmware appends one section per cold boot (banner + init lines +
//! a `gps_diag: …` counters line every 5 s, see
//! `movement_logger_firmware/Src/logger.c` / `Src/errlog.c`). This module
//! splits the mirror into boot sections and grades each one:
//!
//! - **Fail** — a subsystem `init FAIL`, a `*** GPS: no baud lock ***` /
//!   `*** GPS NO NMEA ***` marker, or any other `***` warning the
//!   firmware only emits for real trouble.
//! - **Warn** — watchdog (IWDG/WWDG) reset, GPS stuck at the 9600-baud
//!   fallback (box→module command wire suspect), growing NMEA-checksum /
//!   UART-error counters (marginal solder joint or noisy GND), UART
//!   silence windows mid-run (intermittent joint / VCC), failed BLE
//!   re-advertises, an unconfirmed FOTA trial boot.
//! - Everything else is informational, including the torn/garbled lines
//!   a sudden power cut leaves behind (expected by design — F-PWR-5).
//!
//! Tolerant by construction — all of these shapes exist in real mirrors:
//! torn/spliced lines (power cut mid-write), digit-fused `[N ms]` ticks,
//! replayed byte ranges (live-mirror byte-resume overlap, including
//! replayed *boot banners*), and GPS-survey bridge windows whose
//! UBX/NMEA interleave inflates the NMEA-checksum counter. A line that
//! doesn't parse cleanly is skipped/counted, never trusted; replays are
//! deduplicated by a monotonic-tick frontier with self-healing.
//!
//! Runs automatically after every sync pass that touches `ERRLOG.LOG`
//! (`SyncCore::run_errlog_check`, called from the `ReadDone` handler),
//! when the save dir changes, and once at GUI startup — so every new box
//! boot is checked as soon as its log lands. `MovementLogger
//! --check-errlog [path]` runs the same analysis headless for scripting.

use std::fmt;
use std::path::Path;

/// Severity of one finding. `Ord` so a boot's verdict is just the max.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// Positive/neutral evidence ("GPS link clean", torn-line counts).
    Info,
    Warn,
    Fail,
}

impl fmt::Display for Severity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Severity::Info => write!(f, "ok"),
            Severity::Warn => write!(f, "WARN"),
            Severity::Fail => write!(f, "FAIL"),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Finding {
    pub severity: Severity,
    pub msg: String,
}

/// One well-formed `gps_diag:` counters sample (cumulative since boot).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GpsDiag {
    pub bytes: u64,
    pub lines_good: u64,
    pub lines_bad: u64,
    pub rmc: u64,
    pub gga: u64,
    pub errors: u64,
    pub rx_drop: u64,
    pub ubx_drop: u64,
}

/// Health report for one boot section of the log.
#[derive(Clone, Debug, Default)]
pub struct BootReport {
    /// 1-based boot number within the log (oldest = 1).
    pub index: usize,
    /// 1-based line number of the boot banner in the mirror file.
    pub line_no: usize,
    /// Firmware identification from the banner + `fw:` line.
    pub fw: String,
    /// Decoded reset-reason line ("SOFTWARE+BOR (CSR=0x…)").
    pub reset: String,
    /// Highest sane `[N ms]` stamp seen — approximate section duration.
    pub last_tick_ms: u64,
    /// Boot-start wall-clock in epoch-ms, back-computed from the firmware's
    /// `ble: SET_TIME epoch_ms=<E> tick=<T>` line (`E - T`). The box has no
    /// RTC, so a boot only gets an absolute time once a host (phone/desktop)
    /// connected and pushed SET_TIME during that boot. `None` = no connect
    /// that boot → no time knowable. See `boot_time_label`.
    pub host_epoch_ms: Option<i64>,
    pub findings: Vec<Finding>,
    /// First and last accepted gps_diag sample.
    pub gps_first: Option<GpsDiag>,
    pub gps_last: Option<GpsDiag>,
    /// Count of accepted gps_diag samples in this boot.
    pub diag_samples: usize,
    /// Garbled `gps_diag`-ish lines (power-cut artifacts).
    pub torn_lines: usize,
    /// Raw first lines of the section (banner included), used to detect
    /// replayed banner fragments. Capped at `HEAD_LINES`.
    raw_head: Vec<String>,
    /// Total raw line count of the section (banner included).
    raw_len: usize,
}

impl BootReport {
    /// Worst finding severity; a boot with only Info findings is healthy.
    pub fn verdict(&self) -> Severity {
        self.findings
            .iter()
            .map(|f| f.severity)
            .max()
            .unwrap_or(Severity::Info)
    }

    /// Boot-start wall-clock as `HH:MM-DD.MM.YYYY` in local time, when a
    /// host pushed SET_TIME during this boot (`host_epoch_ms`). `None` when
    /// no host connected that boot — the box has no RTC, so that boot's
    /// absolute time is genuinely unknown.
    pub fn boot_time_label(&self) -> Option<String> {
        let ms = self.host_epoch_ms?;
        let dt = chrono::DateTime::from_timestamp_millis(ms)?
            .with_timezone(&chrono::Local);
        Some(dt.format("%H:%M-%d.%m.%Y").to_string())
    }

    /// Short human label: "OK", "WARN", "FAIL".
    pub fn verdict_label(&self) -> &'static str {
        match self.verdict() {
            Severity::Info => "OK",
            Severity::Warn => "WARN",
            Severity::Fail => "FAIL",
        }
    }

    /// True when EVERY finding at the boot's worst severity is the
    /// "GPS silent at power-on" pattern (no baud lock / gps init FAIL).
    /// Used to suppress the health badge while the box reports its GPS
    /// deliberately powered off — expected silence is not a fault. Any
    /// other co-occurring finding at the same severity (watchdog reset,
    /// BLE trouble, …) keeps the badge visible.
    pub fn fail_is_only_gps_silence(&self) -> bool {
        let v = self.verdict();
        if v == Severity::Info {
            return false;
        }
        self.findings
            .iter()
            .filter(|f| f.severity == v)
            .all(|f| {
                let l = f.msg.to_lowercase();
                // The GPS-init failure path emits a FAMILY of markers, two of
                // which never mention "gps" ("factory-reset the module via
                // u-center…", "module is sending, we can't decode — check
                // UART baud/wiring") — they must count as GPS-silence too or
                // this all() can never be true on a real silent-GPS boot.
                l.contains("gps")
                    || l.contains("no baud lock")
                    || l.contains("nmea")
                    || l.contains("u-center")
                    || l.contains("module is sending")
            })
    }

    /// One-line human-readable reason for a non-OK verdict — what actually
    /// failed, in words a user can act on, not a checker-internal marker
    /// string ("Box health: latest boot FAIL" alone had no value). Maps the
    /// worst finding onto plain language; falls back to the finding's raw
    /// message for cases without a mapping. `None` when the boot is healthy.
    pub fn human_reason(&self) -> Option<String> {
        let worst = self
            .findings
            .iter()
            .filter(|f| f.severity == self.verdict())
            .map(|f| f.msg.as_str())
            .next()?;
        if self.verdict() == Severity::Info {
            return None;
        }
        let lower = worst.to_lowercase();
        let human = if lower.contains("no baud lock")
            || (lower.contains("gps") && lower.contains("init fail"))
        {
            "GPS gave no signal at power-on (receiver off/asleep, or check the GPS wiring)"
                .to_string()
        } else if lower.contains("iwdg") || lower.contains("wwdg") {
            "the box firmware hung and the watchdog auto-restarted it".to_string()
        } else if lower.contains("9600") && lower.contains("fallback") {
            "GPS stuck at its fallback baud rate (box→GPS wire suspect)".to_string()
        } else if lower.contains("checksum") || lower.contains("uart") {
            "GPS data arriving corrupted (wiring/interference suspect)".to_string()
        } else if lower.contains("cut short") || lower.contains("power removed") {
            "the box lost power during startup (battery/magnet?)".to_string()
        } else if lower.contains("re-adv") || lower.contains("ble") {
            "Bluetooth radio had trouble re-advertising".to_string()
        } else if lower.contains("fota") || lower.contains("trial") {
            "a firmware update is still in its unconfirmed trial boot".to_string()
        } else {
            // No mapping — the raw finding is still better than nothing.
            worst.to_string()
        };
        Some(human)
    }
}

#[derive(Clone, Debug, Default)]
pub struct ErrlogReport {
    pub boots: Vec<BootReport>,
    /// One-liner for the log panel / headless output.
    pub summary: String,
}

impl ErrlogReport {
    pub fn latest(&self) -> Option<&BootReport> {
        self.boots.last()
    }
}

/// Strip the firmware's `[N ms] ` prefix, returning (tick, rest).
fn strip_tick(line: &str) -> (Option<u64>, &str) {
    let Some(rest) = line.strip_prefix('[') else {
        return (None, line);
    };
    let Some(end) = rest.find(" ms] ") else {
        return (None, line);
    };
    match rest[..end].parse::<u64>() {
        Ok(ms) => (Some(ms), &rest[end + 5..]),
        Err(_) => (None, line),
    }
}

/// Strict parse of a `gps_diag: k=v …` line: exactly the 8 known keys,
/// each once (bitmask, not a count — a splice that repeats one key while
/// dropping another must not sneak defaulted fields in). Returns `None`
/// for the torn/spliced variants a mid-write power cut produces.
fn parse_gps_diag(rest: &str) -> Option<GpsDiag> {
    let body = rest.strip_prefix("gps_diag: ")?;
    let mut d = GpsDiag::default();
    let mut seen = 0u8;
    for part in body.split_whitespace() {
        let (k, v) = part.split_once('=')?;
        let v: u64 = v.parse().ok()?;
        let (slot, bit): (&mut u64, u8) = match k {
            "bytes" => (&mut d.bytes, 1 << 0),
            "lines_good" => (&mut d.lines_good, 1 << 1),
            "lines_bad" => (&mut d.lines_bad, 1 << 2),
            "rmc" => (&mut d.rmc, 1 << 3),
            "gga" => (&mut d.gga, 1 << 4),
            "errors" => (&mut d.errors, 1 << 5),
            "rx_drop" => (&mut d.rx_drop, 1 << 6),
            "ubx_drop" => (&mut d.ubx_drop, 1 << 7),
            _ => return None,
        };
        if seen & bit != 0 {
            return None; // duplicate key = splice
        }
        *slot = v;
        seen |= bit;
    }
    (seen == 0xFF).then_some(d)
}

/// Extract `key=<i64>` from anywhere in a line ("re-adv=211", "rc=-11").
/// Signed: the firmware prints these with %d and a dead SPI/chip yields
/// negative rc values (-10/-11) — precisely the failures that must not
/// read as success. Returns `None` when the value is torn off.
fn field_i64(line: &str, key: &str) -> Option<i64> {
    let pat = format!("{key}=");
    let start = line.find(&pat)? + pat.len();
    let tail = &line[start..];
    let digits_from = usize::from(tail.starts_with('-'));
    let end = tail[digits_from..]
        .find(|c: char| !c.is_ascii_digit())
        .map(|e| e + digits_from)
        .unwrap_or(tail.len());
    if end == digits_from {
        return None; // no digits at all — torn value
    }
    tail[..end].parse().ok()
}

/// `key=<v>` present and non-zero → a real recorded failure. Torn /
/// missing values (None) count as *not proven*, per the never-trust-torn
/// rule — intact failure lines are frequent enough to still be caught.
fn failed_rc(line: &str, key: &str) -> bool {
    field_i64(line, key).is_some_and(|v| v != 0)
}

/// Ticks above this are torn-line splices (two writes fused mid-line
/// after a power cut) — the box never runs 30 days on one boot.
const TICK_CAP_MS: u64 = 30 * 24 * 3_600_000;

/// Largest believable gap between two consecutive diag samples (they're
/// written every 5 s; even heavy SD-busy skipping never approaches an
/// hour). A forward jump beyond this is a digit-fused tick — accepting
/// it would poison the frontier and discard the rest of the boot.
const MAX_TICK_JUMP_MS: u64 = 6 * 3_600_000;

/// Pair deltas are only graded across plausible windows; a discontinuity
/// larger than this (frontier reset, giant gap) skips the pair so fused
/// counter values can't fabricate growth.
const MAX_DELTA_WINDOW_MS: u64 = 600_000;

/// Minimum tick advance for two diag samples to count as a genuine
/// "UART went silent" window (nominal spacing is 5000 ms).
const STALL_MIN_DT_MS: u64 = 2_500;

/// Rejected-diag ticks within this of each other are "mutually
/// consistent" — three in a row mean the *frontier* is wrong (poisoned
/// by a splice), not the stream, and the frontier self-heals onto them.
const HEAL_WINDOW_MS: u64 = 60_000;
const HEAL_STREAK: u32 = 3;

/// Raw head lines kept per boot for replayed-banner-fragment detection.
const HEAD_LINES: usize = 16;

/// Per-boot scratch accumulated while scanning one section.
#[derive(Default)]
struct BootScan {
    gps_ready_baud: Option<u64>,
    upgrade_ok: bool,
    cfg_ack: bool,
    bridge_valset_ack: bool,
    /// GPS-survey bridge currently active (between `gps: bridge ON` and
    /// `gps: bridge off`). UBX/NMEA interleave during a survey produces
    /// checksum noise that must not be graded as a wiring problem.
    bridge_on: bool,
    /// Any bridge ON/off line seen since the last accepted diag sample —
    /// taints the whole window so boundary-spanning noise stays out of
    /// the clean bucket.
    bridge_touched: bool,
    init_fails: Vec<String>,
    markers: Vec<String>,
    trial_boot: bool,
    fota_confirmed: bool,
    readv_fails: u32,
    peer_recovers: u32,
    fsm_aborts: u32,
    stalls: u32,
    /// lines_bad / errors growth split by bridge state: only the clean
    /// buckets are wiring evidence.
    bad_clean: u64,
    bad_bridge: u64,
    err_clean: u64,
    err_bridge: u64,
    /// `gps_rf:` RF/EMI health lines (firmware v0.0.52+): worst-case
    /// tallies + the last accepted sample. The firmware never `***`s
    /// these by design — wiring/jamming grading happens here.
    rf_samples: u32,
    rf_jam_max: u32,
    rf_jam_crit: u32,
    rf_jam_warn: u32,
    rf_ant_bad: u32,
    rf_noreply: bool,
    rf_last: Option<String>,
    /// Replayed/duplicated log lines (mirror resume overlap) skipped by
    /// the monotonic-tick frontier.
    dup_lines: usize,
    prev_diag: Option<(u64, GpsDiag)>,
    /// Highest accepted diag tick — samples at or below it are replays
    /// of an already-seen region and are ignored.
    tick_frontier: u64,
    /// Self-healing state: consecutive rejected diag ticks that are
    /// mutually consistent (see `HEAL_WINDOW_MS`).
    reject_streak: u32,
    last_reject_tick: u64,
    non_banner_lines: usize,
}

impl BootScan {
    /// A diag sample was rejected (replay or implausible jump). Three
    /// mutually-consistent rejections in a row mean the frontier itself
    /// is poisoned (a fused tick got accepted) — heal onto the live
    /// stream instead of discarding the rest of the boot.
    fn reject_and_maybe_heal(&mut self, ms: u64) -> bool {
        if ms >= self.last_reject_tick && ms - self.last_reject_tick <= HEAL_WINDOW_MS {
            self.reject_streak += 1;
        } else {
            self.reject_streak = 1;
        }
        self.last_reject_tick = ms;
        if self.reject_streak >= HEAL_STREAK {
            self.reject_streak = 0;
            self.tick_frontier = ms;
            self.prev_diag = None; // no pair delta across the reset
            return true;
        }
        false
    }
}

pub fn analyze(text: &str) -> ErrlogReport {
    let mut boots: Vec<BootReport> = Vec::new();
    let mut cur: Option<(BootReport, BootScan)> = None;

    for (i, raw) in text.lines().enumerate() {
        let line_no = i + 1;
        if let Some(banner) = raw.strip_prefix("--- Boot ") {
            if let Some((rep, scan)) = cur.take() {
                push_boot(&mut boots, finish_boot(rep, scan));
            }
            let fw = banner
                .split_once(" --- ")
                .map(|(_, fw)| fw.trim().to_string())
                .unwrap_or_else(|| banner.trim().to_string());
            let mut rep = BootReport {
                index: boots.len() + 1,
                line_no,
                fw,
                ..Default::default()
            };
            rep.raw_head.push(raw.to_string());
            rep.raw_len = 1;
            cur = Some((rep, BootScan::default()));
            continue;
        }
        let Some((rep, scan)) = cur.as_mut() else {
            // Preamble before the first banner (truncated mirror head
            // after a box-side log rotation): nothing to grade.
            continue;
        };
        rep.raw_len += 1;
        if rep.raw_head.len() < HEAD_LINES {
            rep.raw_head.push(raw.to_string());
        }

        let (tick, rest) = strip_tick(raw);
        if let Some(ms) = tick {
            // Same plausibility rules as the diag frontier: a digit-fused
            // tick under the 30-day cap must not become the displayed
            // uptime either. Ticks flow every ≤5 s while the box runs,
            // so a >6 h forward jump is never genuine.
            if ms <= TICK_CAP_MS
                && (rep.last_tick_ms == 0
                    || ms.saturating_sub(rep.last_tick_ms) <= MAX_TICK_JUMP_MS)
            {
                rep.last_tick_ms = rep.last_tick_ms.max(ms);
            }
        }
        if !raw.trim().is_empty() {
            scan.non_banner_lines += 1;
        }

        // Replay gate for graded non-diag lines: a tick strictly below
        // the frontier re-describes an already-seen region (byte-resume
        // overlap) — grading it would double-count markers, init FAILs
        // and BLE failures. Genuine lines are always at/after the last
        // diag tick. (Lines without a tick — banner, `reset:`, `fw:` —
        // are not affected.)
        let replayed = matches!(tick, Some(ms) if ms < scan.tick_frontier);

        // Host wall-clock anchor. On the first connect after a boot the
        // host pushes SET_TIME and the firmware logs
        // `ble: SET_TIME epoch_ms=<E> tick=<T>` (firmware v0.0.33+; older
        // builds log only `tick=` → no anchor, boot stays untimed). Boot-start
        // epoch = E − T, where T is the ErrLog `[N ms]` prefix (the same
        // HAL_GetTick() ms-since-boot clock as `tick=`). First anchor per
        // boot wins — it's the earliest, closest to the true boot start.
        // This is the ONLY absolute time a boot ever gets (no RTC).
        if rep.host_epoch_ms.is_none()
            && !replayed
            && rest.starts_with("ble: SET_TIME ")
        {
            if let Some(epoch) = field_i64(rest, "epoch_ms") {
                let off = tick
                    .map(|t| t as i64)
                    .or_else(|| field_i64(rest, "tick"))
                    .unwrap_or(0);
                if epoch > 0 {
                    rep.host_epoch_ms = Some(epoch - off);
                }
            }
        }

        if let Some(r) = rest.strip_prefix("reset: ") {
            rep.reset = r.trim().to_string();
        } else if let Some(d) = parse_gps_diag(rest) {
            // Every genuine diag line carries a `[N ms] ` prefix; a
            // missing or implausible tick means the line is a splice
            // from a torn write — its payload can't be trusted either.
            let Some(ms) = tick.filter(|&ms| ms <= TICK_CAP_MS) else {
                rep.torn_lines += 1;
                continue;
            };
            // Monotonic-tick frontier with two rejection classes:
            //  - at/below the frontier → replayed region (dup);
            //  - implausible forward jump → digit-fused tick (torn).
            // Both feed the self-healing streak so one bad accepted
            // tick can never discard the rest of the boot.
            if ms <= scan.tick_frontier {
                if !scan.reject_and_maybe_heal(ms) {
                    scan.dup_lines += 1;
                    continue;
                }
            } else if scan.tick_frontier > 0
                && ms.saturating_sub(scan.tick_frontier) > MAX_TICK_JUMP_MS
            {
                if !scan.reject_and_maybe_heal(ms) {
                    rep.torn_lines += 1;
                    continue;
                }
            } else {
                scan.reject_streak = 0;
            }
            if let Some((ptick, prev)) = scan.prev_diag {
                let dt = ms.saturating_sub(ptick);
                // Grade the pair only across a plausible window — a
                // discontinuity means one endpoint is untrustworthy.
                if dt <= MAX_DELTA_WINDOW_MS {
                    let db = d.lines_bad.saturating_sub(prev.lines_bad);
                    let de = d.errors.saturating_sub(prev.errors);
                    if scan.bridge_on || scan.bridge_touched {
                        scan.bad_bridge += db;
                        scan.err_bridge += de;
                    } else {
                        scan.bad_clean += db;
                        scan.err_clean += de;
                        // ≥1 diag window with zero UART bytes after the
                        // link was alive — the signature of an
                        // intermittent joint / VCC brown-out. Only
                        // graded outside survey windows and only across
                        // a real time advance.
                        if dt >= STALL_MIN_DT_MS && d.bytes == prev.bytes && prev.bytes > 0 {
                            scan.stalls += 1;
                        }
                    }
                }
            }
            if rep.gps_first.is_none() {
                rep.gps_first = Some(d);
            }
            rep.gps_last = Some(d);
            rep.diag_samples += 1;
            scan.prev_diag = Some((ms, d));
            scan.tick_frontier = ms;
            scan.bridge_touched = scan.bridge_on;
        } else if rest.contains("gps_diag") {
            rep.torn_lines += 1;
        } else if replayed {
            scan.dup_lines += 1;
        } else if let Some(r) = rest.strip_prefix("gps_rf: ") {
            if r.starts_with("MON-RF poll unanswered") {
                scan.rf_noreply = true;
            } else {
                scan.rf_samples += 1;
                if let Some(j) = field_i64(rest, "jam") {
                    scan.rf_jam_max = scan.rf_jam_max.max(j.max(0) as u32);
                }
                if r.contains("state=crit") {
                    scan.rf_jam_crit += 1;
                } else if r.contains("state=warn") {
                    scan.rf_jam_warn += 1;
                }
                if r.contains("ant=SHORT") || r.contains("ant=OPEN") {
                    scan.rf_ant_bad += 1;
                }
                scan.rf_last = Some(r.trim().to_string());
            }
        } else if rest.contains("***") {
            scan.markers.push(rest.trim().to_string());
        } else if let Some(m) = rest.strip_suffix(": init FAIL") {
            scan.init_fails.push(m.trim().to_string());
        } else if rest.starts_with("gps: ready @") {
            scan.gps_ready_baud = rest
                .split('@')
                .nth(1)
                .and_then(|t| t.split_whitespace().next())
                .and_then(|n| n.parse().ok());
        } else if rest.contains("gps: upgrade") && rest.contains("succeeded") {
            scan.upgrade_ok = true;
        } else if rest.starts_with("gps: cfg-rate") && rest.ends_with("ACK") {
            scan.cfg_ack = true;
        } else if rest.starts_with("gps: bridge ON") {
            scan.bridge_on = true;
            scan.bridge_touched = true;
            if rest.contains("valset in=ACK") {
                scan.bridge_valset_ack = true;
            }
        } else if rest.starts_with("gps: bridge off") {
            scan.bridge_on = false;
            scan.bridge_touched = true;
        } else if rest.starts_with("fwupdate: trial boot") {
            scan.trial_boot = true;
        } else if rest.contains("fwupdate: new image confirmed healthy") {
            scan.fota_confirmed = true;
        } else if rest.starts_with("ble: peer-recover") {
            scan.peer_recovers += 1;
            if failed_rc(rest, "re-adv") {
                scan.readv_fails += 1;
            }
        } else if rest.starts_with("ble: disconnected reason=")
            || rest.starts_with("ble: latched-disconnect")
        {
            if failed_rc(rest, "re-adv") {
                scan.readv_fails += 1;
            }
        } else if rest.starts_with("ble: periodic re-adv") {
            if failed_rc(rest, "rc") {
                scan.readv_fails += 1;
            }
        } else if rest.starts_with("fsm:") && rest.contains("aborted") {
            scan.fsm_aborts += 1;
        }
    }
    if let Some((rep, scan)) = cur.take() {
        push_boot(&mut boots, finish_boot(rep, scan));
    }

    let summary = summarize(&boots);
    ErrlogReport { boots, summary }
}

/// Append a finished boot — unless it is a replayed banner fragment: the
/// byte-resume overlap can re-append a recent boot's banner + first few
/// lines verbatim, which would otherwise become a phantom "latest boot"
/// and flip the verdict / exit code. A short section whose raw lines are
/// an exact prefix of the previous boot's head is such a replay — but
/// only if at least one matching line is tick-stamped: banner/fw/reset
/// are constant strings, identical across *genuine* boots too, so a
/// tickless 3-line section is a real rapid power-cycle (magnet flicker),
/// not a replay, and must stay a boot of its own.
fn push_boot(boots: &mut Vec<BootReport>, rep: BootReport) {
    if rep.raw_len <= HEAD_LINES {
        if let Some(prev) = boots.last_mut() {
            if rep.raw_head.len() <= prev.raw_head.len()
                && rep.raw_head[..] == prev.raw_head[..rep.raw_head.len()]
                && rep
                    .raw_head
                    .iter()
                    .any(|l| strip_tick(l).0.is_some())
            {
                prev.findings.push(Finding {
                    severity: Severity::Info,
                    msg: format!(
                        "replayed boot-banner fragment ({} line(s)) skipped — \
                         sync byte-resume overlap, harmless",
                        rep.raw_len
                    ),
                });
                return;
            }
        }
    }
    boots.push(rep);
}

/// Turn one scanned boot section into graded findings.
fn finish_boot(mut rep: BootReport, scan: BootScan) -> BootReport {
    let f = &mut rep.findings;

    // Hard evidence is graded unconditionally — a boot cut short by the
    // very failure it logged (init FAIL then crash/power cut, watchdog
    // bootloop) must never be waved through as "too little data".
    if rep.reset.contains("IWDG") || rep.reset.contains("WWDG") {
        let ctx = if scan.trial_boot {
            " (during a FOTA trial-boot cycle)"
        } else {
            ""
        };
        f.push(Finding {
            severity: Severity::Warn,
            msg: format!(
                "watchdog reset preceded this boot{ctx} — firmware hung in the previous run \
                 (reset: {})",
                rep.reset
            ),
        });
    }

    for m in &scan.init_fails {
        f.push(Finding {
            severity: Severity::Fail,
            msg: format!("{m}: init FAILED at boot"),
        });
    }

    for m in &scan.markers {
        // The two GPS wiring markers get a joint-level diagnosis; any
        // other `***` line is surfaced verbatim as a failure.
        let msg = if m.contains("GPS NO NMEA") {
            match field_i64(m, "bytes") {
                Some(0) => format!(
                    "{m} — zero UART bytes: GPS unpowered or dead data joint \
                     (check VCC/GND/JP2 pin 13)"
                ),
                Some(_) => format!(
                    "{m} — module sends but nothing decodes: baud/wiring \
                     (TX/RX swap or marginal joint)"
                ),
                None => m.clone(),
            }
        } else if m.contains("no baud lock") {
            format!("{m} — no NMEA at any baud: GPS unpowered or dead data joint")
        } else {
            m.clone()
        };
        f.push(Finding {
            severity: Severity::Fail,
            msg,
        });
    }

    // A handful of lines with no GPS-ready and no diag samples = power
    // was cut during init. Expected with the magnet/case workflow; the
    // hard evidence above (if any) has already been graded.
    if scan.non_banner_lines < 6 && scan.gps_ready_baud.is_none() && rep.diag_samples == 0 {
        f.push(Finding {
            severity: Severity::Info,
            msg: "boot cut short during init (power removed?) — too little data to grade".into(),
        });
        return rep;
    }

    match scan.gps_ready_baud {
        Some(9600) => f.push(Finding {
            severity: Severity::Warn,
            msg: "GPS stuck at fallback 9600 baud / 5 Hz — the $PUBX,41 upgrade never took; \
                  box→module command wire (JP2 pin 14) suspect"
                .into(),
        }),
        Some(_) if scan.upgrade_ok || scan.cfg_ack || scan.bridge_valset_ack => {
            f.push(Finding {
                severity: Severity::Info,
                msg: "box→module command wire verified (UBX/PUBX ACKed)".into(),
            })
        }
        _ => {}
    }

    // Steady-state GPS-link quality from the accepted diag stream.
    // The first sample already contains all boot-time noise (baud hunt
    // garbage), so growth after it is genuine in-flight corruption —
    // except inside GPS-survey bridge windows, whose UBX/NMEA
    // interleave noise is bucketed separately and never blamed on
    // wiring.
    if let (Some(first), Some(last)) = (rep.gps_first, rep.gps_last) {
        if rep.diag_samples >= 2 {
            let d_bad = scan.bad_clean;
            let d_good = last.lines_good.saturating_sub(first.lines_good);
            let d_err = scan.err_clean;
            let d_bytes = last.bytes.saturating_sub(first.bytes);

            if d_bad > 5 && d_bad * 200 > d_good {
                f.push(Finding {
                    severity: Severity::Warn,
                    msg: format!(
                        "{d_bad} NMEA checksum failures during the run ({d_good} good) — \
                         bit errors in flight: marginal GPS data joint or noisy GND"
                    ),
                });
            } else if d_bad > 0 {
                f.push(Finding {
                    severity: Severity::Info,
                    msg: format!("{d_bad} NMEA checksum failures over {d_good} good sentences"),
                });
            }

            if d_err > 5 {
                f.push(Finding {
                    severity: Severity::Warn,
                    msg: format!(
                        "{d_err} UART framing/noise errors during the run — electrical \
                         noise on the GPS wiring (GND joint?)"
                    ),
                });
            } else if d_err > 0 {
                f.push(Finding {
                    severity: Severity::Info,
                    msg: format!("{d_err} UART framing/noise errors during the run"),
                });
            }

            if scan.stalls >= 2 {
                f.push(Finding {
                    severity: Severity::Warn,
                    msg: format!(
                        "GPS UART went silent in {}×~5 s windows mid-run — intermittent \
                         data joint or VCC brown-out cycles",
                        scan.stalls
                    ),
                });
            } else if scan.stalls == 1 {
                f.push(Finding {
                    severity: Severity::Info,
                    msg: "one ~5 s GPS UART silence window mid-run".into(),
                });
            }

            if scan.bad_bridge > 0 || scan.err_bridge > 0 {
                f.push(Finding {
                    severity: Severity::Info,
                    msg: format!(
                        "{} checksum fail(s) / {} UART error(s) during GPS-survey bridge \
                         windows — UBX/NMEA interleave, expected, not a wiring signal",
                        scan.bad_bridge, scan.err_bridge
                    ),
                });
            }

            if d_bad == 0 && d_err == 0 && scan.stalls == 0 && d_bytes > 0 {
                f.push(Finding {
                    severity: Severity::Info,
                    msg: format!(
                        "GPS link clean — {d_bytes} bytes, {d_good} sentences, \
                         0 checksum fails, 0 UART errors"
                    ),
                });
            }
        }
    }

    // RF/EMI health from the periodic `gps_rf:` lines (firmware v0.0.52+).
    // Antenna-supervisor faults and critical jamming are wiring/EMI
    // evidence; everything else is trend data surfaced as Info.
    if scan.rf_ant_bad > 0 {
        f.push(Finding {
            severity: Severity::Warn,
            msg: format!(
                "antenna supervisor reported SHORT/OPEN in {} of {} RF sample(s) — \
                 antenna feed wiring suspect",
                scan.rf_ant_bad, scan.rf_samples
            ),
        });
    }
    if scan.rf_jam_crit > 0 {
        f.push(Finding {
            severity: Severity::Warn,
            msg: format!(
                "RF jamming CRITICAL in {} of {} sample(s) (jam_ind max {}) — \
                 strong interference at the antenna (issue #10 EMI)",
                scan.rf_jam_crit, scan.rf_samples, scan.rf_jam_max
            ),
        });
    } else if scan.rf_jam_warn > 0 {
        f.push(Finding {
            severity: Severity::Info,
            msg: format!(
                "RF jamming 'warn' in {} of {} sample(s) (jam_ind max {})",
                scan.rf_jam_warn, scan.rf_samples, scan.rf_jam_max
            ),
        });
    }
    if let Some(last) = &scan.rf_last {
        f.push(Finding {
            severity: Severity::Info,
            msg: format!("RF health (last sample): {last}"),
        });
    }
    if scan.rf_noreply {
        f.push(Finding {
            severity: Severity::Info,
            msg: "MON-RF poll unanswered — no RF health data this boot".into(),
        });
    }

    if scan.readv_fails > 0 {
        f.push(Finding {
            severity: Severity::Warn,
            msg: format!(
                "{} failed BLE re-advertise attempt(s) — the box may have been \
                 temporarily unreachable",
                scan.readv_fails
            ),
        });
    }
    if scan.peer_recovers > 0 || scan.fsm_aborts > 0 {
        f.push(Finding {
            severity: Severity::Info,
            msg: format!(
                "BLE recoveries: {} peer-recover, {} transfer abort(s)",
                scan.peer_recovers, scan.fsm_aborts
            ),
        });
    }

    if scan.trial_boot {
        f.push(if scan.fota_confirmed {
            Finding {
                severity: Severity::Info,
                msg: "FOTA trial boot confirmed healthy".into(),
            }
        } else {
            Finding {
                severity: Severity::Warn,
                msg: "FOTA trial boot NOT confirmed within this boot — rollback counter \
                      advanced (reverts after 5 unconfirmed boots)"
                    .into(),
            }
        });
    }

    if rep.torn_lines > 0 {
        f.push(Finding {
            severity: Severity::Info,
            msg: format!(
                "{} garbled log line(s) — sudden-power-cut artifacts, expected by design",
                rep.torn_lines
            ),
        });
    }
    if scan.dup_lines > 0 {
        f.push(Finding {
            severity: Severity::Info,
            msg: format!(
                "{} replayed log line(s) skipped — sync byte-resume overlap, harmless",
                scan.dup_lines
            ),
        });
    }

    rep
}

fn summarize(boots: &[BootReport]) -> String {
    let Some(last) = boots.last() else {
        return "no boot sections found".into();
    };
    let earlier = &boots[..boots.len() - 1];
    let warns = earlier
        .iter()
        .filter(|b| b.verdict() == Severity::Warn)
        .count();
    let fails = earlier
        .iter()
        .filter(|b| b.verdict() == Severity::Fail)
        .count();
    let head = format!(
        "{} boots — latest #{}: {}",
        boots.len(),
        last.index,
        last.verdict_label()
    );
    let detail = last
        .findings
        .iter()
        .find(|f| f.severity == last.verdict())
        .map(|f| format!(" ({})", f.msg))
        .unwrap_or_default();
    let tail = match (fails, warns) {
        (0, 0) => String::new(),
        (0, w) => format!("; {w} earlier boot(s) with warnings"),
        (f2, w) => format!("; earlier boots: {f2} FAIL, {w} WARN"),
    };
    format!("{head}{detail}{tail}")
}

pub fn analyze_file(path: &Path) -> std::io::Result<ErrlogReport> {
    // Lossy: the log can contain torn multi-byte sequences after a
    // power cut; analysis must never fail on them.
    let bytes = std::fs::read(path)?;
    Ok(analyze(&String::from_utf8_lossy(&bytes)))
}

/// Headless `--check-errlog` entry point: print the full per-boot
/// report to stdout, exit 0 = latest boot OK, 1 = WARN, 2 = FAIL,
/// 3 = could not read the file / no boot sections in it.
pub fn run_cli(path: &Path) -> i32 {
    let rep = match analyze_file(path) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("check-errlog: cannot read {}: {e}", path.display());
            return 3;
        }
    };
    println!("errlog check: {} — {}", path.display(), rep.summary);
    for b in &rep.boots {
        println!(
            "\nboot #{:<3} line {:<7} [{}] {} — up {:.1} s (reset: {})",
            b.index,
            b.line_no,
            b.verdict_label(),
            b.fw,
            b.last_tick_ms as f64 / 1000.0,
            if b.reset.is_empty() { "?" } else { &b.reset },
        );
        for f in &b.findings {
            println!("    [{}] {}", f.severity, f.msg);
        }
    }
    match rep.latest().map(|b| b.verdict()) {
        Some(Severity::Fail) => 2,
        Some(Severity::Warn) => 1,
        Some(Severity::Info) => 0,
        // A file with zero boot sections is not a healthy box — it's
        // the wrong file (or an empty mirror). Don't report success.
        None => {
            eprintln!(
                "check-errlog: {} contains no '--- Boot' sections — not an ERRLOG.LOG mirror?",
                path.display()
            );
            3
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HEALTHY: &str = "\
--- Boot Error_Log_Pump_Tsueri_04.07.2026 --- MovementLogger 0.0.30 build #1
fw: build Jul  4 2026 15:29:48 | GPS 10Hz | flash~?
reset: SOFTWARE+BOR (CSR=0x0C004400)
[325 ms] fuel: ok (STC3115)
[449 ms] imu: ok (LSM6DSV16X)
[520 ms] mag: ok (LIS2MDL)
[561 ms] baro: ok (LPS22DF)
[3564 ms] gps: locked @9600 (16 newlines in 1500ms), upgrading via $PUBX,41…
[5349 ms] gps: upgrade @9600 → @38400 succeeded (20 newlines)
[5357 ms] gps: cfg-rate 10Hz ACK
[5577 ms] gps: ready @38400 baud, rate=10Hz
[5578 ms] gps: ok (MAX-M10S UART4)
[8402 ms] ble: advertising as PumpTsueri
[8402 ms] usb: ok (MSC)
[10000 ms] gps_diag: bytes=5000 lines_good=100 lines_bad=3 rmc=0 gga=50 errors=2 rx_drop=0 ubx_drop=0
[15000 ms] gps_diag: bytes=10000 lines_good=200 lines_bad=3 rmc=10 gga=100 errors=2 rx_drop=0 ubx_drop=0
[20000 ms] gps_diag: bytes=15000 lines_good=300 lines_bad=3 rmc=60 gga=150 errors=2 rx_drop=0 ubx_drop=0
";

    #[test]
    fn host_set_time_anchor_gives_boot_wall_clock() {
        // 2026-07-06 06:00:00 UTC = 1751781600000 ms, logged at tick 12000
        // ⇒ boot start epoch = 1751781600000 − 12000 = 1751781588000.
        let log = "\
--- Boot Error_Log_Pump_Tsueri_06.07.2026 --- MovementLogger 0.0.55 build #1
reset: SOFTWARE+BOR (CSR=0x0C004400)
[325 ms] fuel: ok (STC3115)
[12000 ms] ble: SET_TIME epoch_ms=1751781600000 tick=12000 wrote=1
[17000 ms] ble: SET_TIME epoch_ms=1751781605000 tick=17000 wrote=1
";
        let rep = analyze(log);
        assert_eq!(rep.boots.len(), 1);
        let b = &rep.boots[0];
        // First anchor wins; both anchors agree on the same boot start.
        assert_eq!(b.host_epoch_ms, Some(1751781588000));
        // Label renders in the shape HH:MM-DD.MM.YYYY (exact value is
        // timezone-dependent, so check the structure, not the digits).
        let t = b.boot_time_label().expect("label");
        let ok = t.len() == 16
            && &t[2..3] == ":"
            && &t[5..6] == "-"
            && &t[8..9] == "."
            && &t[11..12] == "."
            && t.chars().filter(|c| c.is_ascii_digit()).count() == 12;
        assert!(ok, "unexpected label shape: {t}");
    }

    #[test]
    fn boot_without_host_connect_has_no_time() {
        let rep = analyze(HEALTHY);
        assert_eq!(rep.boots[0].host_epoch_ms, None);
        assert!(rep.boots[0].boot_time_label().is_none());
    }

    #[test]
    fn healthy_boot_is_ok() {
        let rep = analyze(HEALTHY);
        assert_eq!(rep.boots.len(), 1);
        let b = &rep.boots[0];
        assert_eq!(b.verdict(), Severity::Info, "findings: {:?}", b.findings);
        assert_eq!(b.diag_samples, 3);
        // Boot-time noise (bad=3, errors=2 in the FIRST sample) must not
        // trigger warnings — only growth after the first sample counts.
        assert!(b
            .findings
            .iter()
            .any(|f| f.msg.contains("GPS link clean")));
        assert!(b
            .findings
            .iter()
            .any(|f| f.msg.contains("command wire verified")));
    }

    #[test]
    fn healthy_gps_rf_lines_stay_ok_and_surface_last_sample() {
        let txt = format!(
            "{HEALTHY}\
             [25000 ms] gps_rf: fix=3D used=12 avg6=41.3 min6=38 max6=45 noise=88 agc=3120 jam=3 state=ok ant=ok\n\
             [85000 ms] gps_rf: fix=3D used=13 avg6=42.0 min6=39 max6=46 noise=90 agc=3100 jam=5 state=ok ant=ok\n"
        );
        let rep = analyze(&txt);
        let b = &rep.boots[0];
        assert_eq!(b.verdict(), Severity::Info, "findings: {:?}", b.findings);
        let last = b
            .findings
            .iter()
            .find(|f| f.msg.starts_with("RF health (last sample):"))
            .expect("last RF sample surfaced");
        assert!(last.msg.contains("used=13"), "wrong sample kept: {}", last.msg);
    }

    #[test]
    fn jamming_critical_and_antenna_fault_warn() {
        let txt = format!(
            "{HEALTHY}\
             [25000 ms] gps_rf: fix=0 used=0 avg6=0.0 min6=0 max6=0 noise=412 agc=7900 jam=201 state=crit ant=SHORT\n"
        );
        let rep = analyze(&txt);
        let b = &rep.boots[0];
        assert_eq!(b.verdict(), Severity::Warn, "findings: {:?}", b.findings);
        assert!(b
            .findings
            .iter()
            .any(|f| f.severity == Severity::Warn && f.msg.contains("jamming CRITICAL")
                && f.msg.contains("jam_ind max 201")));
        assert!(b
            .findings
            .iter()
            .any(|f| f.severity == Severity::Warn && f.msg.contains("SHORT/OPEN")));
    }

    #[test]
    fn no_nmea_marker_is_fail_with_joint_hint() {
        let txt = format!(
            "{HEALTHY}--- Boot X --- MovementLogger 0.0.30 build #1\n\
             reset: SOFTWARE+BOR (CSR=0x0C004400)\n\
             [1 ms] fuel: ok (STC3115)\n\
             [2 ms] imu: ok (LSM6DSV16X)\n\
             [3 ms] mag: ok\n\
             [4 ms] baro: ok\n\
             [5 ms] ble: advertising as PumpTsueri\n\
             [30001 ms] *** GPS NO NMEA: 30s, 0 lines parsed (bytes=0 errors=0) ***\n"
        );
        let rep = analyze(&txt);
        assert_eq!(rep.boots.len(), 2);
        let b = &rep.boots[1];
        assert_eq!(b.verdict(), Severity::Fail);
        assert!(b.findings.iter().any(|f| f.msg.contains("VCC/GND")));
    }

    #[test]
    fn checksum_growth_warns() {
        let txt = HEALTHY.replace(
            "[20000 ms] gps_diag: bytes=15000 lines_good=300 lines_bad=3 rmc=60 gga=150 errors=2 rx_drop=0 ubx_drop=0",
            "[20000 ms] gps_diag: bytes=15000 lines_good=300 lines_bad=50 rmc=60 gga=150 errors=2 rx_drop=0 ubx_drop=0",
        );
        let rep = analyze(&txt);
        let b = &rep.boots[0];
        assert_eq!(b.verdict(), Severity::Warn);
        assert!(b
            .findings
            .iter()
            .any(|f| f.msg.contains("checksum failures during the run")));
    }

    #[test]
    fn stalls_warn_and_torn_lines_are_info() {
        let txt = format!(
            "{HEALTHY}\
             [25000 ms] gps_diag: bytes=15000 lines_good=300 lines_bad=3 rmc=80 gga=200 errors=2 rx_drop=0 ubx_drop=0\n\
             [30000 ms] gps_diag: bytes=15000 lines_good=300 lines_bad=3 rmc=80 gga=200 errors=2 rx_drop=0 ubx_drop=0\n\
             [35000 ms] gps_diag: bytes=15golines_good=300 gga=200\n"
        );
        let rep = analyze(&txt);
        let b = &rep.boots[0];
        assert_eq!(b.torn_lines, 1);
        assert_eq!(b.verdict(), Severity::Warn);
        assert!(b.findings.iter().any(|f| f.msg.contains("went silent")));
    }

    #[test]
    fn truncated_boot_is_not_graded() {
        let txt = "--- Boot X --- MovementLogger 0.0.30 build #1\n\
                   fw: build Jul  4 2026 | GPS 10Hz\n\
                   reset: SOFTWARE+BOR (CSR=0x0C004400)\n\
                   [325 ms] fuel: ok (STC3115)\n";
        let rep = analyze(txt);
        let b = &rep.boots[0];
        assert_eq!(b.verdict(), Severity::Info);
        assert!(b.findings.iter().any(|f| f.msg.contains("cut short")));
    }

    #[test]
    fn truncated_boot_still_fails_on_init_fail() {
        // Power cut right after the failure line (or a crash caused by
        // it) — the hard evidence must survive the cut-short heuristic.
        let txt = "--- Boot X --- MovementLogger 0.0.30 build #1\n\
                   fw: build Jul  4 2026 | GPS 10Hz\n\
                   reset: SOFTWARE+BOR (CSR=0x0C004400)\n\
                   [266 ms] fuel: init FAIL\n";
        let rep = analyze(txt);
        let b = &rep.boots[0];
        assert_eq!(b.verdict(), Severity::Fail, "findings: {:?}", b.findings);
        assert!(b.findings.iter().any(|f| f.msg.contains("fuel: init FAILED")));
        // The cut-short note is still there as context.
        assert!(b.findings.iter().any(|f| f.msg.contains("cut short")));
    }

    #[test]
    fn truncated_boot_still_warns_on_watchdog_reset() {
        let txt = "--- Boot X --- MovementLogger 0.0.30 build #1\n\
                   reset: IWDG+SOFTWARE (CSR=0x06004400)\n\
                   [100 ms] fuel: ok\n";
        let rep = analyze(txt);
        let b = &rep.boots[0];
        assert_eq!(b.verdict(), Severity::Warn);
        assert!(b.findings.iter().any(|f| f.msg.contains("watchdog reset")));
    }

    #[test]
    fn init_fail_and_watchdog_reset() {
        let txt = "--- Boot X --- MovementLogger 0.0.30 build #1\n\
                   reset: IWDG+SOFTWARE (CSR=0x06004400)\n\
                   [100 ms] fuel: ok\n\
                   [200 ms] imu: ok\n\
                   [300 ms] mag: ok\n\
                   [400 ms] gps: init FAIL\n\
                   [500 ms] ble: advertising as PumpTsueri\n\
                   [600 ms] usb: ok (MSC)\n\
                   [10000 ms] gps_diag: bytes=0 lines_good=0 lines_bad=0 rmc=0 gga=0 errors=0 rx_drop=0 ubx_drop=0\n";
        let rep = analyze(txt);
        let b = &rep.boots[0];
        assert_eq!(b.verdict(), Severity::Fail);
        assert!(b.findings.iter().any(|f| f.msg.contains("gps: init FAILED")));
        assert!(b.findings.iter().any(|f| f.msg.contains("watchdog reset")));
    }

    #[test]
    fn fallback_baud_warns_pin14() {
        let txt = "--- Boot X --- MovementLogger 0.0.30 build #1\n\
                   reset: SOFTWARE+BOR (CSR=0x0C004400)\n\
                   [1 ms] fuel: ok\n\
                   [2 ms] imu: ok\n\
                   [3 ms] gps: locked @9600 (16 newlines in 1500ms), upgrading via $PUBX,41…\n\
                   [4 ms] gps: upgrade failed (newlines=0), staying @9600 / 5Hz\n\
                   [5 ms] gps: ready @9600 baud, rate=5Hz\n\
                   [6 ms] ble: advertising as PumpTsueri\n\
                   [10000 ms] gps_diag: bytes=500 lines_good=10 lines_bad=0 rmc=0 gga=5 errors=0 rx_drop=0 ubx_drop=0\n";
        let rep = analyze(txt);
        let b = &rep.boots[0];
        assert_eq!(b.verdict(), Severity::Warn);
        assert!(b.findings.iter().any(|f| f.msg.contains("pin 14")));
    }

    #[test]
    fn readv_failure_warns() {
        let txt = format!(
            "{HEALTHY}[21000 ms] ble: disconnected reason=0x08 re-adv=211\n"
        );
        let rep = analyze(&txt);
        let b = &rep.boots[0];
        assert_eq!(b.verdict(), Severity::Warn);
        assert!(b.findings.iter().any(|f| f.msg.contains("re-advertise")));
    }

    #[test]
    fn negative_rc_is_a_failure_too() {
        // A dead SPI/BLE chip yields rc=-10/-11 (printed with %d by the
        // firmware) — the original u64 parse inverted these into
        // success, hiding the exact box-invisible condition.
        let txt = format!(
            "{HEALTHY}\
             [21000 ms] ble: periodic re-adv rc=-11\n\
             [22000 ms] ble: latched-disconnect re-adv=-10\n\
             [23000 ms] ble: peer-recover hci_disc=0x0000 rc=0 re-adv=-10 why=deadline\n"
        );
        let rep = analyze(&txt);
        let b = &rep.boots[0];
        assert_eq!(b.verdict(), Severity::Warn);
        assert!(b
            .findings
            .iter()
            .any(|f| f.msg.contains("3 failed BLE re-advertise")));
    }

    #[test]
    fn successful_readv_is_quiet() {
        let txt = format!(
            "{HEALTHY}[21000 ms] ble: disconnected reason=0x08 re-adv=0\n"
        );
        let rep = analyze(&txt);
        assert_eq!(rep.boots[0].verdict(), Severity::Info);
    }

    #[test]
    fn replayed_segments_do_not_fake_stalls() {
        // A byte-resume overlap replays two already-seen samples: the
        // monotonic-tick frontier must skip them (no phantom stall, no
        // double-counted growth).
        let txt = format!(
            "{HEALTHY}\
             [15000 ms] gps_diag: bytes=10000 lines_good=200 lines_bad=3 rmc=10 gga=100 errors=2 rx_drop=0 ubx_drop=0\n\
             [20000 ms] gps_diag: bytes=15000 lines_good=300 lines_bad=3 rmc=60 gga=150 errors=2 rx_drop=0 ubx_drop=0\n"
        );
        let rep = analyze(&txt);
        let b = &rep.boots[0];
        assert_eq!(b.verdict(), Severity::Info, "findings: {:?}", b.findings);
        assert!(b.findings.iter().any(|f| f.msg.contains("replayed")));
        assert!(b.findings.iter().any(|f| f.msg.contains("GPS link clean")));
    }

    #[test]
    fn replayed_failure_lines_are_not_double_counted() {
        // One genuine re-adv failure at tick 21000, then the same line
        // replayed (tick below the 25000 frontier) — must count once.
        let txt = format!(
            "{HEALTHY}\
             [21000 ms] ble: disconnected reason=0x08 re-adv=211\n\
             [25000 ms] gps_diag: bytes=20000 lines_good=400 lines_bad=3 rmc=80 gga=200 errors=2 rx_drop=0 ubx_drop=0\n\
             [21000 ms] ble: disconnected reason=0x08 re-adv=211\n"
        );
        let rep = analyze(&txt);
        let b = &rep.boots[0];
        assert!(
            b.findings
                .iter()
                .any(|f| f.msg.contains("1 failed BLE re-advertise")),
            "findings: {:?}",
            b.findings
        );
    }

    #[test]
    fn spliced_giant_tick_is_torn_not_uptime() {
        let txt = format!(
            "{HEALTHY}[56439993999 ms] gps_diag: bytes=99999 lines_good=999 lines_bad=999 rmc=0 gga=9 errors=999 rx_drop=0 ubx_drop=0\n"
        );
        let rep = analyze(&txt);
        let b = &rep.boots[0];
        // Uptime stays at the last sane tick; the spliced line is torn,
        // and its (untrustworthy) counter values produce no warning.
        assert_eq!(b.last_tick_ms, 20000);
        assert_eq!(b.torn_lines, 1);
        assert_eq!(b.verdict(), Severity::Info, "findings: {:?}", b.findings);
    }

    #[test]
    fn frontier_poisoning_self_heals() {
        // A fused tick below the 30-day cap but plausible-looking gets
        // rejected by the 6 h jump guard; if it HAD been accepted, the
        // healing streak re-anchors onto the live stream after 3
        // mutually-consistent rejected samples either way.
        let txt = format!(
            "{HEALTHY}\
             [79992545 ms] gps_diag: bytes=16000 lines_good=320 lines_bad=3 rmc=60 gga=160 errors=2 rx_drop=0 ubx_drop=0\n\
             [25000 ms] gps_diag: bytes=20000 lines_good=400 lines_bad=3 rmc=80 gga=200 errors=2 rx_drop=0 ubx_drop=0\n\
             [30000 ms] gps_diag: bytes=25000 lines_good=500 lines_bad=3 rmc=100 gga=250 errors=2 rx_drop=0 ubx_drop=0\n\
             [35000 ms] gps_diag: bytes=30000 lines_good=600 lines_bad=3 rmc=120 gga=300 errors=2 rx_drop=0 ubx_drop=0\n\
             [40000 ms] gps_diag: bytes=35000 lines_good=700 lines_bad=3 rmc=140 gga=350 errors=2 rx_drop=0 ubx_drop=0\n"
        );
        let rep = analyze(&txt);
        let b = &rep.boots[0];
        // The fused-tick line is rejected (jump guard) and the samples
        // at 25000/30000/35000 re-anchor the frontier; 40000 then flows
        // normally. Last sample must be the 40000 one, uptime sane.
        assert_eq!(b.gps_last.unwrap().bytes, 35000);
        assert_eq!(b.last_tick_ms, 40000);
        assert_eq!(b.verdict(), Severity::Info, "findings: {:?}", b.findings);
    }

    #[test]
    fn duplicate_key_splice_is_rejected() {
        // 8 parts but only 6 distinct keys — a token-boundary splice
        // must not sneak defaulted fields in.
        let txt = format!(
            "{HEALTHY}[25000 ms] gps_diag: bytes=20000 lines_good=400 lines_bad=3 lines_good=401 lines_bad=4 rmc=80 gga=200 errors=2\n"
        );
        let rep = analyze(&txt);
        let b = &rep.boots[0];
        assert_eq!(b.torn_lines, 1);
        assert_eq!(b.diag_samples, 3);
    }

    #[test]
    fn phantom_banner_fragment_does_not_become_latest_boot() {
        // Real latest boot FAILs; a byte-resume overlap then replays its
        // banner + first lines. The fragment must be merged away so the
        // FAIL stays the latest verdict (drives GUI badge + exit code).
        let failing = "--- Boot X --- MovementLogger 0.0.30 build #1\n\
                       fw: build Jul  4 2026 | GPS 10Hz\n\
                       reset: SOFTWARE+BOR (CSR=0x0C004400)\n\
                       [1 ms] fuel: ok (STC3115)\n\
                       [2 ms] imu: ok (LSM6DSV16X)\n\
                       [3 ms] mag: ok\n\
                       [4 ms] baro: ok\n\
                       [5 ms] ble: advertising as PumpTsueri\n\
                       [30001 ms] *** GPS NO NMEA: 30s, 0 lines parsed (bytes=0 errors=0) ***\n";
        let replay_fragment = "--- Boot X --- MovementLogger 0.0.30 build #1\n\
                               fw: build Jul  4 2026 | GPS 10Hz\n\
                               reset: SOFTWARE+BOR (CSR=0x0C004400)\n\
                               [1 ms] fuel: ok (STC3115)\n";
        let txt = format!("{HEALTHY}{failing}{replay_fragment}");
        let rep = analyze(&txt);
        assert_eq!(rep.boots.len(), 2, "fragment must not become a boot");
        assert_eq!(rep.latest().unwrap().verdict(), Severity::Fail);
        assert!(rep
            .latest()
            .unwrap()
            .findings
            .iter()
            .any(|f| f.msg.contains("replayed boot-banner fragment")));
    }

    #[test]
    fn bridge_window_noise_is_not_a_wiring_warning() {
        let txt = format!(
            "{HEALTHY}\
             [21000 ms] gps: bridge ON @38400 baud (valset in=ACK out=ACK, nmea trimmed)\n\
             [25000 ms] gps_diag: bytes=20000 lines_good=350 lines_bad=100 rmc=70 gga=175 errors=2 rx_drop=0 ubx_drop=0\n\
             [30000 ms] gps_diag: bytes=25000 lines_good=400 lines_bad=200 rmc=80 gga=200 errors=2 rx_drop=0 ubx_drop=0\n\
             [31000 ms] gps: bridge off @38400 baud (nmea restored)\n\
             [35000 ms] gps_diag: bytes=30000 lines_good=500 lines_bad=200 rmc=90 gga=250 errors=2 rx_drop=0 ubx_drop=0\n"
        );
        let rep = analyze(&txt);
        let b = &rep.boots[0];
        assert_eq!(b.verdict(), Severity::Info, "findings: {:?}", b.findings);
        assert!(b
            .findings
            .iter()
            .any(|f| f.msg.contains("GPS-survey bridge")));
    }

    #[test]
    fn bridge_off_boundary_window_stays_bridge_bucketed() {
        // Window 20000→25000 spans "bridge off" at 22000: its noise
        // accrued mostly in-bridge and must not hit the clean bucket.
        let txt = format!(
            "{HEALTHY}\
             [18000 ms] gps: bridge ON @38400 baud (valset in=ACK out=ACK, nmea trimmed)\n\
             [22000 ms] gps: bridge off @38400 baud (nmea restored)\n\
             [25000 ms] gps_diag: bytes=20000 lines_good=350 lines_bad=150 rmc=70 gga=175 errors=2 rx_drop=0 ubx_drop=0\n\
             [30000 ms] gps_diag: bytes=25000 lines_good=450 lines_bad=150 rmc=80 gga=225 errors=2 rx_drop=0 ubx_drop=0\n"
        );
        let rep = analyze(&txt);
        let b = &rep.boots[0];
        assert_eq!(b.verdict(), Severity::Info, "findings: {:?}", b.findings);
        assert!(b
            .findings
            .iter()
            .any(|f| f.msg.contains("GPS-survey bridge")));
    }

    #[test]
    fn summary_excludes_latest_from_history() {
        let txt = HEALTHY.replace(
            "[20000 ms] gps_diag: bytes=15000 lines_good=300 lines_bad=3 rmc=60 gga=150 errors=2 rx_drop=0 ubx_drop=0",
            "[20000 ms] gps_diag: bytes=15000 lines_good=300 lines_bad=50 rmc=60 gga=150 errors=2 rx_drop=0 ubx_drop=0",
        );
        let rep = analyze(&txt);
        assert_eq!(rep.boots.len(), 1);
        assert!(
            !rep.summary.contains("earlier"),
            "single WARN boot must not count itself as history: {}",
            rep.summary
        );
    }

    #[test]
    fn empty_input_has_no_boots() {
        let rep = analyze("");
        assert!(rep.boots.is_empty());
        assert_eq!(rep.summary, "no boot sections found");
    }
}
