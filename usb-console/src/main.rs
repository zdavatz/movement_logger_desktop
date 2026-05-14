// USB CDC console for the SensorTile.box PRO debug build.
//
// Bypasses macOS's CDC ACM driver (which doesn't fully attach on Sequoia
// 15+ for TinyUSB descriptors) by using libusb directly. Opens the
// bulk-IN endpoint 0x82 and streams the firmware's printf output to
// stdout.
//
// Usage on macOS (kernel CDC partial-attach holds the device, so root is
// required to claim the interface):
//     sudo cargo run --release
// or after a release build:
//     sudo ./target/release/usb-console
//
// On Linux:
//     cargo run --release        # add a udev rule for unprivileged access
//
// Press Ctrl-C to exit cleanly.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use rusb::{Context, DeviceHandle, UsbContext};

// Match the descriptor in Core/Src/usb_descriptors.c.
const VID: u16 = 0xCAFE;
const PID: u16 = 0x4001;

// CDC bulk endpoints (data interface).
const EP_BULK_IN: u8 = 0x82;
const DATA_INTERFACE: u8 = 1;

const READ_SIZE: usize = 256;
const READ_TIMEOUT: Duration = Duration::from_millis(250);

fn main() -> std::process::ExitCode {
    let stop = Arc::new(AtomicBool::new(false));
    {
        let s = Arc::clone(&stop);
        if let Err(e) = ctrlc::set_handler(move || s.store(true, Ordering::SeqCst)) {
            eprintln!("error: failed to install ctrl-c handler: {e}");
            return std::process::ExitCode::from(2);
        }
    }

    let ctx = match Context::new() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("error: libusb init failed: {e}");
            eprintln!("       on macOS install with: brew install libusb");
            return std::process::ExitCode::from(1);
        }
    };

    let handle = match ctx.open_device_with_vid_pid(VID, PID) {
        Some(h) => h,
        None => {
            eprintln!("error: no device found with VID:PID {VID:04x}:{PID:04x}");
            eprintln!("       is the box plugged in and the USB CDC firmware running?");
            return std::process::ExitCode::from(1);
        }
    };

    if let Err(e) = run(handle, stop) {
        eprintln!("error: {e}");
        return std::process::ExitCode::from(3);
    }
    std::process::ExitCode::SUCCESS
}

fn run<T: UsbContext>(mut handle: DeviceHandle<T>, stop: Arc<AtomicBool>) -> Result<(), rusb::Error> {
    // Best-effort device-info read. macOS may refuse string-descriptor
    // reads on a USB device whose kernel driver hasn't fully attached, so
    // we silence those errors and just keep going — VID/PID already
    // confirmed device identity.
    let dev = handle.device();
    let desc = dev.device_descriptor()?;
    let timeout = Duration::from_millis(200);
    let manuf = handle
        .read_manufacturer_string_ascii(&desc)
        .ok()
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "?".into());
    let prod = handle
        .read_product_string_ascii(&desc)
        .ok()
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "?".into());
    let _ = timeout;
    eprintln!("connected: {manuf} / {prod}");

    // On Linux the CDC kernel driver auto-attaches; detach so we can
    // claim. macOS's libusb backend doesn't implement detach (the kernel
    // driver's partial-attach state can't be detached anyway), so we
    // ignore that error.
    let _ = handle.set_auto_detach_kernel_driver(true);
    if let Ok(true) = handle.kernel_driver_active(DATA_INTERFACE) {
        let _ = handle.detach_kernel_driver(DATA_INTERFACE);
    }

    // SET_CONFIGURATION 1. macOS may already have set it; treat
    // ResourceBusy / similar as non-fatal. The bulk read below will
    // surface a real problem.
    if let Err(e) = handle.set_active_configuration(1) {
        eprintln!("set_active_configuration: {e} (carrying on)");
    }

    handle.claim_interface(DATA_INTERFACE)?;

    eprintln!("--- streaming bulk IN (Ctrl-C to exit) ---");
    let stdout = std::io::stdout();
    let mut buf = vec![0u8; READ_SIZE];
    while !stop.load(Ordering::SeqCst) {
        match handle.read_bulk(EP_BULK_IN, &mut buf, READ_TIMEOUT) {
            Ok(0) => continue,
            Ok(n) => {
                use std::io::Write;
                let mut out = stdout.lock();
                out.write_all(&buf[..n]).ok();
                out.flush().ok();
            }
            Err(rusb::Error::Timeout) => continue,
            Err(rusb::Error::Interrupted) => continue,
            Err(e) => return Err(e),
        }
    }
    eprintln!("\n--- exit ---");
    let _ = handle.release_interface(DATA_INTERFACE);
    Ok(())
}
