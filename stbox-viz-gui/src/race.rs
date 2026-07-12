//! Race mode — live multi-rider tracking (issue: wingfoil race, ~50
//! riders, each with an Android/iOS phone + GPS on the board nose).
//!
//! Transport-agnostic UDP listener: every phone fires one small JSON
//! datagram per fix (1–2 Hz) at this machine's `ip:port`. On a venue
//! LAN the phones send directly; over cellular a relay can forward the
//! same datagrams unchanged, so this listener never needs to know.
//!
//! Wire format (one JSON object per datagram, ≤ ~300 bytes; all keys
//! except `rider`, `lat`, `lon` optional — senders: Android
//! `RaceUplink.kt`, iOS `RaceUplink.swift`):
//!
//! ```json
//! {"v":1,"rider":"Zeno","src":"ublox","lat":37.3838,"lon":23.2472,
//!  "kmh":4.2,"deg":181.0,"ts":1783948000123,"batt":85}
//! ```
//!
//! `src` = "ublox" (Android dongle) | "phone" (iPhone GPS) | "watch"
//! (Apple Watch fix relayed through the paired iPhone). `ts` = sender
//! epoch ms (informational; staleness is judged by receive time so
//! clock skew between phones can't grey anyone out).

use eframe::egui;
use serde::Deserialize;
use std::collections::{BTreeMap, VecDeque};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::net::UdpSocket;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender};
use std::sync::Arc;
use std::time::{Duration, Instant};
use walkers::{HttpOptions, HttpTiles, Map, MapMemory, Plugin, Position, Projector};

/// Default listen port — unassigned high port, same constant baked
/// into the Android and iOS senders.
pub const DEFAULT_PORT: u16 = 47777;

/// A rider whose last fix is older than this renders greyed-out (their
/// dot stays on the map — a capsized rider's last position is exactly
/// what the race committee wants to see).
const STALE_AFTER: Duration = Duration::from_secs(5);

/// Trail points kept per rider — 600 ≈ 5 min at the senders' 2 Hz cap.
const TRAIL_LEN: usize = 600;

// ---------------------------------------------------------------------------
//  Wire format
// ---------------------------------------------------------------------------

/// One decoded position datagram.
#[derive(Deserialize, Clone, Debug)]
pub struct RaceFix {
    pub rider: String,
    #[serde(default)]
    pub src: String,
    pub lat: f64,
    pub lon: f64,
    #[serde(default = "nan")]
    pub kmh: f64,
    #[serde(default = "nan")]
    pub deg: f64,
    #[serde(default)]
    pub ts: u64,
    /// Sender battery percent, -1 = unknown.
    #[serde(default = "batt_unknown")]
    pub batt: i32,
}

fn nan() -> f64 {
    f64::NAN
}
fn batt_unknown() -> i32 {
    -1
}

enum RaceEvent {
    Fix(RaceFix),
    /// A datagram that didn't parse — counted so a mis-configured
    /// sender shows up as a climbing number instead of silence.
    Bad,
}

// ---------------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------------

struct Rider {
    last: RaceFix,
    last_rx: Instant,
    trail: VecDeque<Position>,
    color: egui::Color32,
}

struct Listener {
    stop: Arc<AtomicBool>,
    rx: Receiver<RaceEvent>,
    port: u16,
}

impl Drop for Listener {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

pub struct RaceState {
    /// Port text field (kept as text so a half-typed value doesn't
    /// fight the parser).
    port_text: String,
    listener: Option<Listener>,
    riders: BTreeMap<String, Rider>,
    /// Follow mode: keep the camera centred on the riders' centroid.
    follow: bool,
    datagrams: u64,
    bad_datagrams: u64,
    /// This machine's LAN IP, captured at listen start — shown so the
    /// phones know where to point without the user hunting through
    /// ifconfig.
    local_ip: Option<String>,
    /// Everything received is appended here (one CSV row per datagram)
    /// for post-race replay/analysis.
    log_path: Option<PathBuf>,
    log: Option<BufWriter<File>>,
    tiles: Option<HttpTiles>,
    map_memory: MapMemory,
    /// First fix centres + zooms the map once; after that the camera
    /// belongs to the user (or the follow toggle).
    centered_once: bool,
}

impl Default for RaceState {
    fn default() -> Self {
        Self {
            port_text: DEFAULT_PORT.to_string(),
            listener: None,
            riders: BTreeMap::new(),
            follow: true,
            datagrams: 0,
            bad_datagrams: 0,
            local_ip: None,
            log_path: None,
            log: None,
            tiles: None,
            map_memory: MapMemory::default(),
            centered_once: false,
        }
    }
}

// ---------------------------------------------------------------------------
//  Listener thread
// ---------------------------------------------------------------------------

fn spawn_listener(
    port: u16,
    ctx: egui::Context,
) -> Result<Listener, std::io::Error> {
    let socket = UdpSocket::bind(("0.0.0.0", port))?;
    // Half-second poll so the stop flag is honoured promptly without
    // busy-spinning.
    socket.set_read_timeout(Some(Duration::from_millis(500)))?;
    let stop = Arc::new(AtomicBool::new(false));
    let (tx, rx): (Sender<RaceEvent>, Receiver<RaceEvent>) = std::sync::mpsc::channel();
    let stop_thread = stop.clone();
    std::thread::Builder::new()
        .name("race-udp".into())
        .spawn(move || {
            let mut buf = [0u8; 2048];
            loop {
                if stop_thread.load(Ordering::Relaxed) {
                    return;
                }
                match socket.recv_from(&mut buf) {
                    Ok((n, _from)) => {
                        let ev = match serde_json::from_slice::<RaceFix>(&buf[..n]) {
                            Ok(fix) if fix.lat.is_finite() && fix.lon.is_finite() => {
                                RaceEvent::Fix(fix)
                            }
                            _ => RaceEvent::Bad,
                        };
                        if tx.send(ev).is_err() {
                            return; // UI side gone
                        }
                        ctx.request_repaint();
                    }
                    // Timeout / EINTR — loop to re-check the stop flag.
                    Err(_) => {}
                }
            }
        })?;
    Ok(Listener { stop, rx, port })
}

/// This machine's LAN IPv4, via the connected-UDP-socket trick (no
/// packet is actually sent).
fn local_lan_ip() -> Option<String> {
    let s = UdpSocket::bind("0.0.0.0:0").ok()?;
    s.connect("8.8.8.8:80").ok()?;
    Some(s.local_addr().ok()?.ip().to_string())
}

/// Stable per-rider colour: hash the name onto the golden-angle hue
/// wheel so any two riders are far apart and the colour survives
/// restarts.
fn rider_color(name: &str) -> egui::Color32 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    name.hash(&mut h);
    let hue = (h.finish() % 360) as f32 / 360.0;
    egui::ecolor::Hsva::new(hue, 0.85, 0.85, 1.0).into()
}

// ---------------------------------------------------------------------------
//  Map plugin — trails + dots + name labels
// ---------------------------------------------------------------------------

struct RidersPlugin<'r> {
    riders: &'r BTreeMap<String, Rider>,
}

impl Plugin for RidersPlugin<'_> {
    fn run(self: Box<Self>, ui: &mut egui::Ui, _response: &egui::Response, projector: &Projector) {
        let painter = ui.painter();
        for rider in self.riders.values() {
            let stale = rider.last_rx.elapsed() > STALE_AFTER;
            let color = if stale {
                rider.color.gamma_multiply(0.4)
            } else {
                rider.color
            };
            // Trail — one polyline mesh per rider.
            if rider.trail.len() >= 2 {
                let pts: Vec<egui::Pos2> = rider
                    .trail
                    .iter()
                    .map(|p| {
                        let v = projector.project(*p);
                        egui::pos2(v.x, v.y)
                    })
                    .collect();
                painter.add(egui::Shape::line(pts, egui::Stroke::new(3.0, color)));
            }
            // Dot + white ring at the latest fix.
            let v = projector.project(Position::from_lat_lon(rider.last.lat, rider.last.lon));
            let at = egui::pos2(v.x, v.y);
            painter.circle_filled(at, 7.0, color);
            painter.circle_stroke(at, 7.0, egui::Stroke::new(2.0, egui::Color32::WHITE));
            // Name label with a soft plate behind it for readability
            // over any tile colour.
            let galley = painter.layout_no_wrap(
                rider.last.rider.clone(),
                egui::FontId::proportional(13.0),
                egui::Color32::WHITE,
            );
            let text_pos = at + egui::vec2(10.0, -galley.size().y / 2.0);
            let plate = egui::Rect::from_min_size(text_pos, galley.size()).expand(3.0);
            painter.rect_filled(plate, 4.0, egui::Color32::from_black_alpha(140));
            painter.galley(text_pos, galley, egui::Color32::WHITE);
        }
    }
}

// ---------------------------------------------------------------------------
//  Tab UI
// ---------------------------------------------------------------------------

impl RaceState {
    fn start(&mut self, ctx: &egui::Context) {
        let port: u16 = self.port_text.trim().parse().unwrap_or(DEFAULT_PORT);
        self.port_text = port.to_string();
        match spawn_listener(port, ctx.clone()) {
            Ok(l) => {
                self.local_ip = local_lan_ip();
                // New race log per listen session.
                let dir = crate::agent_config::movementlogger_home().join("race");
                let _ = std::fs::create_dir_all(&dir);
                let name = format!(
                    "race_{}.csv",
                    chrono::Local::now().format("%Y%m%d_%H%M%S")
                );
                let path = dir.join(name);
                if let Ok(f) = File::create(&path) {
                    let mut w = BufWriter::new(f);
                    let _ = writeln!(w, "rx_iso,rider,src,lat,lon,kmh,deg,ts,batt");
                    self.log = Some(w);
                    self.log_path = Some(path);
                }
                self.listener = Some(l);
            }
            Err(e) => {
                self.local_ip = None;
                self.port_text = format!("{port}");
                self.bad_datagrams = 0;
                self.datagrams = 0;
                self.log = None;
                self.log_path = Some(PathBuf::from(format!("bind failed: {e}")));
            }
        }
    }

    fn stop(&mut self) {
        self.listener = None; // Drop stops the thread.
        if let Some(w) = self.log.as_mut() {
            let _ = w.flush();
        }
        self.log = None;
    }

    fn drain_events(&mut self) {
        let Some(listener) = self.listener.as_ref() else {
            return;
        };
        // Collect first — logging below needs &mut self.
        let mut fixes: Vec<RaceFix> = Vec::new();
        while let Ok(ev) = listener.rx.try_recv() {
            match ev {
                RaceEvent::Fix(f) => fixes.push(f),
                RaceEvent::Bad => self.bad_datagrams += 1,
            }
        }
        for fix in fixes {
            self.datagrams += 1;
            if let Some(w) = self.log.as_mut() {
                let _ = writeln!(
                    w,
                    "{},{},{},{:.7},{:.7},{:.2},{:.1},{},{}",
                    chrono::Local::now().format("%Y-%m-%dT%H:%M:%S%.3f"),
                    fix.rider.replace(',', " "),
                    fix.src,
                    fix.lat,
                    fix.lon,
                    fix.kmh,
                    fix.deg,
                    fix.ts,
                    fix.batt,
                );
            }
            let pos = Position::from_lat_lon(fix.lat, fix.lon);
            let color = rider_color(&fix.rider);
            let rider = self
                .riders
                .entry(fix.rider.clone())
                .or_insert_with(|| Rider {
                    last: fix.clone(),
                    last_rx: Instant::now(),
                    trail: VecDeque::with_capacity(TRAIL_LEN),
                    color,
                });
            rider.last = fix;
            rider.last_rx = Instant::now();
            if rider.trail.len() == TRAIL_LEN {
                rider.trail.pop_front();
            }
            rider.trail.push_back(pos);
            if !self.centered_once {
                self.map_memory.center_at(pos);
                let _ = self.map_memory.set_zoom(16.0);
                self.centered_once = true;
            }
        }
    }

    /// Riders' centroid — the follow-mode camera target.
    fn centroid(&self) -> Option<Position> {
        if self.riders.is_empty() {
            return None;
        }
        let (mut lat, mut lon) = (0.0, 0.0);
        for r in self.riders.values() {
            lat += r.last.lat;
            lon += r.last.lon;
        }
        let n = self.riders.len() as f64;
        Some(Position::from_lat_lon(lat / n, lon / n))
    }

    pub fn ui(&mut self, ui: &mut egui::Ui) {
        self.drain_events();

        // While listening, ages / staleness must keep ticking even with
        // no incoming datagrams.
        if self.listener.is_some() {
            ui.ctx().request_repaint_after(Duration::from_millis(500));
        }

        ui.horizontal(|ui| {
            ui.heading("Race");
            ui.separator();
            ui.label("UDP port:");
            ui.add_enabled(
                self.listener.is_none(),
                egui::TextEdit::singleline(&mut self.port_text).desired_width(60.0),
            );
            if self.listener.is_none() {
                if ui.button("Start listening").clicked() {
                    self.start(&ui.ctx().clone());
                }
            } else if ui.button("Stop").clicked() {
                self.stop();
            }
            ui.checkbox(&mut self.follow, "Follow riders");
            if ui.button("Clear riders").clicked() {
                self.riders.clear();
                self.centered_once = false;
            }
        });
        ui.horizontal(|ui| {
            if let Some(l) = &self.listener {
                let ip = self.local_ip.as_deref().unwrap_or("this machine's IP");
                ui.label(format!("Phones send to  {ip}:{}", l.port));
                ui.separator();
                ui.label(format!("{} datagrams", self.datagrams));
                if self.bad_datagrams > 0 {
                    ui.colored_label(
                        egui::Color32::YELLOW,
                        format!("{} unparseable", self.bad_datagrams),
                    );
                }
                if let Some(p) = &self.log_path {
                    ui.separator();
                    ui.label(format!("log: {}", p.display()));
                }
            } else {
                ui.label(
                    "Start listening, then enable Race mode on each phone \
                     (same WiFi, or any relay that forwards the datagrams here).",
                );
                if let Some(p) = &self.log_path {
                    // Shows the bind error after a failed start too.
                    ui.separator();
                    ui.label(format!("{}", p.display()));
                }
            }
        });
        ui.separator();

        // Rider list — compact single-line rows above the map.
        if !self.riders.is_empty() {
            egui::ScrollArea::vertical()
                .max_height(120.0)
                .show(ui, |ui| {
                    for r in self.riders.values() {
                        let age = r.last_rx.elapsed().as_secs();
                        let stale = r.last_rx.elapsed() > STALE_AFTER;
                        ui.horizontal(|ui| {
                            let (rect, _) = ui
                                .allocate_exact_size(egui::vec2(12.0, 12.0), egui::Sense::hover());
                            ui.painter().circle_filled(rect.center(), 5.0, r.color);
                            let kmh = if r.last.kmh.is_finite() {
                                format!("{:.1} km/h", r.last.kmh)
                            } else {
                                "—".into()
                            };
                            let batt = if r.last.batt >= 0 {
                                format!("  ·  {} %", r.last.batt)
                            } else {
                                String::new()
                            };
                            let line = format!(
                                "{}  ·  {}  ·  {}{}",
                                r.last.rider, kmh, r.last.src, batt
                            );
                            if stale {
                                ui.colored_label(
                                    egui::Color32::GRAY,
                                    format!("{line}  ·  last seen {age}s ago"),
                                );
                            } else {
                                ui.label(line);
                            }
                        });
                    }
                });
            ui.separator();
        }

        // Map — created lazily so the tiles cache/UA are set up with a
        // live egui context.
        if self.tiles.is_none() {
            let opts = HttpOptions {
                cache: Some(crate::agent_config::movementlogger_home().join("tiles-cache")),
                user_agent: Some(walkers::HeaderValue::from_static(
                    "MovementLogger-Desktop (github.com/zdavatz/movement_logger_desktop)",
                )),
            };
            self.tiles = Some(HttpTiles::with_options(
                walkers::sources::OpenStreetMap,
                opts,
                ui.ctx().clone(),
            ));
        }
        if self.follow {
            if let Some(c) = self.centroid() {
                self.map_memory.center_at(c);
            }
        }
        // `my_position` is only the fallback camera anchor before the
        // first fix arrives; Ermioni harbour is as good a default as any.
        let anchor = self
            .centroid()
            .unwrap_or(Position::from_lat_lon(47.3769, 8.5417));
        let map = Map::new(
            self.tiles.as_mut().map(|t| t as &mut dyn walkers::Tiles),
            &mut self.map_memory,
            anchor,
        )
        .with_plugin(RidersPlugin {
            riders: &self.riders,
        });
        ui.add_sized(ui.available_size(), map);
        ui.small("© OpenStreetMap contributors");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// End-to-end through a real socket: bind, fire datagrams (one per
    /// sender platform + one broken), assert the parsed fixes and the
    /// bad-datagram count. The egui context is headless — repaint
    /// requests go nowhere, which is fine.
    #[test]
    fn listener_parses_all_three_senders() {
        // Port 0 → OS-assigned; read it back for the sender.
        let l = spawn_listener(0, egui::Context::default()).expect("bind");
        // spawn_listener stores the requested port, so query the real
        // one from a probe bind on the same port… instead, re-bind is
        // racy — use a fixed high port unlikely to clash.
        drop(l);
        let port = 47999;
        let l = spawn_listener(port, egui::Context::default()).expect("bind");
        let tx = UdpSocket::bind("127.0.0.1:0").unwrap();
        let to = format!("127.0.0.1:{port}");
        tx.send_to(
            br#"{"v":1,"rider":"Zeno","src":"ublox","lat":37.3838,"lon":23.2472,"kmh":4.2,"deg":181.0,"ts":1783948000123,"batt":85}"#,
            &to,
        )
        .unwrap();
        tx.send_to(
            br#"{"rider":"Kai","src":"watch","lat":47.37,"lon":8.54}"#,
            &to,
        )
        .unwrap();
        tx.send_to(b"not json", &to).unwrap();

        let mut fixes = Vec::new();
        let mut bad = 0;
        let deadline = Instant::now() + Duration::from_secs(3);
        while fixes.len() + bad < 3 && Instant::now() < deadline {
            match l.rx.recv_timeout(Duration::from_millis(200)) {
                Ok(RaceEvent::Fix(f)) => fixes.push(f),
                Ok(RaceEvent::Bad) => bad += 1,
                Err(_) => {}
            }
        }
        assert_eq!(bad, 1);
        assert_eq!(fixes.len(), 2);
        let zeno = fixes.iter().find(|f| f.rider == "Zeno").unwrap();
        assert_eq!(zeno.src, "ublox");
        assert!((zeno.lat - 37.3838).abs() < 1e-9);
        assert_eq!(zeno.batt, 85);
        let kai = fixes.iter().find(|f| f.rider == "Kai").unwrap();
        assert!(kai.kmh.is_nan()); // optional fields default cleanly
        assert_eq!(kai.batt, -1);
    }
}
