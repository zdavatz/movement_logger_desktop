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

mod survey;

use anyhow::Result;
use clap::Parser;
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(
    name = "gps-debug",
    version,
    about = "Live u-blox UBX diagnostics (fix quality, per-signal C/N0, RF/antenna) → CSV + live console."
)]
struct Cli {
    /// Serial port of the u-blox: e.g. /dev/cu.usbserial-XXXX (macOS),
    /// /dev/ttyACM0 or /dev/ttyUSB0 (Linux), COM3 (Windows).
    #[arg(long)]
    port: String,
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
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    survey::run(&survey::SurveyArgs {
        port: &cli.port,
        baud: cli.baud,
        out_dir: &cli.output,
        label: &cli.label,
        duration_s: cli.duration,
    })
}
