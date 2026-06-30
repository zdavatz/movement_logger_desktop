//! `gps-debug` as a library — so the MovementLogger GUI can drive the same
//! UBX survey engine in-process over a BLE transport (the box bridges the
//! u-blox UART over its existing link), while the standalone `gps-debug`
//! binary keeps using it over a USB serial port. One survey engine, two
//! transports. See `survey.rs` for the wire details and `survey::run_core`
//! for the transport-agnostic loop.

pub mod survey;

pub use survey::{detect_ports, run, run_core, SurveyArgs};
