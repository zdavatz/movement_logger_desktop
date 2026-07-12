//! race-relay — UDP fan-out relay for MovementLogger Race Mode.
//!
//! Lets riders stream over cellular instead of venue WiFi: phones send
//! their ordinary race datagrams to this relay's public `ip:port`
//! (nothing changes in the apps — it's just a different address in the
//! Race card), and every subscribed viewer (the desktop's Race tab in
//! "via relay" mode) receives a copy. Datagrams are forwarded byte-for-
//! byte; the relay only peeks at two JSON fields:
//!
//!  - `{"v":1,"sub":true,"race":"<token>"}` — a viewer keepalive. The
//!    sender's address is registered for `EXPIRY` (refreshed by every
//!    keepalive, which the desktop sends every ~10 s — that same
//!    keepalive holds the viewer's NAT pinhole open for the return
//!    path, so nobody needs port forwarding).
//!  - anything with a `rider` field — a position fix, forwarded to all
//!    live viewers whose `race` token matches the datagram's (both
//!    default to "", so token-less setups just work).
//!
//! Run on any Linux box with a public IP (fine next to Apache — that's
//! TCP 80/443, this is UDP): `race-relay [--port 47777]`, open the
//! port in the firewall, done. A systemd unit ships next to this file.
//!
//! Capacity maths: 50 riders × 5 Hz × ~200 B ≈ 50 KB/s in, times the
//! handful of viewers out — a single blocking thread is plenty.

use serde::Deserialize;
use std::collections::HashMap;
use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

/// Default listen port — same as the desktop's LAN listener.
const DEFAULT_PORT: u16 = 47777;

/// A viewer that hasn't sent a keepalive for this long is dropped.
const EXPIRY: Duration = Duration::from_secs(30);

/// The only parsing the relay does. Unknown fields are ignored, so the
/// wire format can grow without touching the relay.
#[derive(Deserialize)]
struct Probe {
    #[serde(default)]
    sub: bool,
    #[serde(default)]
    race: String,
    rider: Option<String>,
}

struct Relay {
    socket: UdpSocket,
    /// viewer address → (race token, last keepalive).
    viewers: HashMap<SocketAddr, (String, Instant)>,
    forwarded: u64,
    dropped: u64,
    last_stats: Instant,
}

impl Relay {
    fn new(socket: UdpSocket) -> Self {
        Self {
            socket,
            viewers: HashMap::new(),
            forwarded: 0,
            dropped: 0,
            last_stats: Instant::now(),
        }
    }

    /// Handle one datagram. Returns immediately on anything unparseable
    /// (internet background noise on an open UDP port is a given).
    fn handle(&mut self, buf: &[u8], from: SocketAddr) {
        let Ok(probe) = serde_json::from_slice::<Probe>(buf) else {
            self.dropped += 1;
            return;
        };
        if probe.sub {
            if self
                .viewers
                .insert(from, (probe.race.clone(), Instant::now()))
                .is_none()
            {
                println!("viewer joined: {from} (race '{}')", probe.race);
            }
            return;
        }
        if probe.rider.is_none() {
            self.dropped += 1;
            return;
        }
        self.viewers
            .retain(|addr, (_, seen)| {
                let live = seen.elapsed() < EXPIRY;
                if !live {
                    println!("viewer expired: {addr}");
                }
                live
            });
        for (addr, (race, _)) in &self.viewers {
            if *race == probe.race {
                let _ = self.socket.send_to(buf, addr);
                self.forwarded += 1;
            }
        }
    }

    fn maybe_stats(&mut self) {
        if self.last_stats.elapsed() >= Duration::from_secs(60) {
            println!(
                "stats: {} viewers, {} forwarded, {} dropped",
                self.viewers.len(),
                self.forwarded,
                self.dropped
            );
            self.last_stats = Instant::now();
        }
    }
}

fn main() -> std::io::Result<()> {
    let mut port = DEFAULT_PORT;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--port" => {
                port = args
                    .next()
                    .and_then(|p| p.parse().ok())
                    .unwrap_or(DEFAULT_PORT)
            }
            "--help" | "-h" => {
                println!("race-relay [--port {DEFAULT_PORT}]");
                return Ok(());
            }
            other => eprintln!("ignoring unknown argument {other}"),
        }
    }

    let socket = UdpSocket::bind(("0.0.0.0", port))?;
    socket.set_read_timeout(Some(Duration::from_millis(500)))?;
    println!("race-relay listening on 0.0.0.0:{port}");

    let mut relay = Relay::new(socket.try_clone()?);
    let mut buf = [0u8; 2048];
    loop {
        match socket.recv_from(&mut buf) {
            Ok((n, from)) => relay.handle(&buf[..n], from),
            Err(_) => {} // timeout — fall through to stats
        }
        relay.maybe_stats();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Full fan-out through real sockets: two viewers subscribe (one
    /// with a different race token), a rider datagram arrives, and only
    /// the matching viewer receives the identical bytes.
    #[test]
    fn forwards_to_matching_viewers_only() {
        let relay_sock = UdpSocket::bind("127.0.0.1:0").unwrap();
        let relay_addr = relay_sock.local_addr().unwrap();
        let mut relay = Relay::new(relay_sock.try_clone().unwrap());

        let viewer = UdpSocket::bind("127.0.0.1:0").unwrap();
        viewer
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let other = UdpSocket::bind("127.0.0.1:0").unwrap();
        other
            .set_read_timeout(Some(Duration::from_millis(300)))
            .unwrap();
        let rider = UdpSocket::bind("127.0.0.1:0").unwrap();

        // Subscribe both viewers, then send one rider fix.
        viewer
            .send_to(br#"{"v":1,"sub":true}"#, relay_addr)
            .unwrap();
        other
            .send_to(br#"{"v":1,"sub":true,"race":"other"}"#, relay_addr)
            .unwrap();
        let fix = br#"{"v":1,"rider":"Zeno","src":"ublox","lat":37.38,"lon":23.24,"kmh":4.2}"#;
        rider.send_to(fix, relay_addr).unwrap();

        // Drive the relay loop for the three datagrams.
        let mut buf = [0u8; 2048];
        let recv_sock = relay_sock;
        recv_sock
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        for _ in 0..3 {
            let (n, from) = recv_sock.recv_from(&mut buf).unwrap();
            relay.handle(&buf[..n], from);
        }

        let mut out = [0u8; 2048];
        let (n, _) = viewer.recv_from(&mut out).unwrap();
        assert_eq!(&out[..n], fix, "matching viewer gets identical bytes");
        assert!(
            other.recv_from(&mut out).is_err(),
            "different race token must receive nothing"
        );
        assert_eq!(relay.forwarded, 1);
    }
}
