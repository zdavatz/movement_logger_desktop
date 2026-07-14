//! gps-debug — live u-blox UBX diagnostics for antenna selection + mounting
//! evaluation. Sidecar binary shipped next to `MovementLogger`; the GUI's
//! "GPS Debug" tab spawns it and streams its stdout into a panel, but it's
//! equally usable standalone from a terminal.
//!
//! It polls a u-blox receiver (MAX-M10S or any M8/M9/M10) once a second over a
//! serial port and writes two CSVs — `<label>_gnss_epoch.csv` (per second) and
//! `<label>_gnss_signals.csv` (per tracked signal) — plus a live one-line
//! summary to stdout. Read-only: it only *polls*, never reconfigures the
//! receiver. See `survey.rs` for the field list + wire details.

use anyhow::Result;
use clap::Parser;
use gps_debug::survey;
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(
    name = "gps-debug",
    version,
    about = "Live u-blox UBX diagnostics (fix quality, per-signal C/N0, RF/antenna) → CSV + live console."
)]
struct Cli {
    /// Serial port of the u-blox: e.g. /dev/cu.usbserial-XXXX (macOS),
    /// /dev/ttyACM0 or /dev/ttyUSB0 (Linux), COM3 (Windows). Omit to
    /// auto-detect — works when exactly one candidate port is present.
    #[arg(long)]
    port: Option<String>,
    /// List the auto-detected candidate serial ports and exit.
    #[arg(long)]
    list_ports: bool,
    /// Serial baud rate. The box configures the MAX-M10S at 38400; a bare
    /// module defaults to 9600. Ignored over native USB-CDC.
    #[arg(long, default_value_t = 38400)]
    baud: u32,
    /// Directory for the two CSV outputs.
    #[arg(short, long, default_value = ".")]
    output: PathBuf,
    /// Run label — goes into the filenames and a CSV column so you can A/B
    /// several antennas/positions and concatenate the runs.
    #[arg(long, default_value = "antenna")]
    label: String,
    /// Optional stop after N seconds. Omit to run until Ctrl-C.
    #[arg(long)]
    duration: Option<f64>,
    /// One-shot: read the receiver's live configuration (UBX-CFG-VALGET,
    /// RAM layer) for every key of the known-good u-center 2 chip export,
    /// print each value, and mark divergences with ***. Read-only.
    #[arg(long)]
    read_config: bool,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    let detected = survey::detect_ports();
    if cli.list_ports {
        if detected.is_empty() {
            println!("no candidate serial ports found");
            println!("  hint: ls /dev/cu.* (macOS)  ·  ls /dev/ttyACM* /dev/ttyUSB* (Linux)");
        } else {
            for p in &detected {
                println!("{p}");
            }
        }
        return Ok(());
    }

    // Explicit --port wins; otherwise auto-detect, which only succeeds when
    // there's exactly one candidate (zero or many is ambiguous → ask).
    let port = match cli.port {
        Some(p) => p,
        None => match detected.as_slice() {
            [one] => {
                eprintln!("gps-debug: auto-detected serial port {one}");
                one.clone()
            }
            [] => anyhow::bail!(
                "no --port given and no serial port auto-detected\n\
                 hint: ls /dev/cu.* (macOS)  ·  ls /dev/ttyACM* /dev/ttyUSB* (Linux)"
            ),
            many => anyhow::bail!(
                "no --port given and {} candidate ports found — pass one of:\n  {}",
                many.len(),
                many.join("\n  ")
            ),
        },
    };

    if cli.read_config {
        return gps_debug::read_config_serial(&port, cli.baud);
    }

    survey::run(&survey::SurveyArgs {
        port: &port,
        baud: cli.baud,
        out_dir: &cli.output,
        label: &cli.label,
        duration_s: cli.duration,
    })
}
