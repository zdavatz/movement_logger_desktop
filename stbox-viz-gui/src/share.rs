//! Per-file "Share…" in the Sync tab — desktop twin of the iOS
//! `ShareLink` / Android `ACTION_SEND` on a downloaded file.
//!
//! macOS: the native `NSSharingServicePicker` (AirDrop, Mail, Messages,
//! Notes …), anchored at the clicked button. Must run on the main thread,
//! which `eframe::App::update` is. Windows / Linux have no share sheet we
//! can reach without a large platform dependency, so there the button
//! reveals the file in Explorer / the file manager instead.

use std::path::Path;

#[cfg(target_os = "macos")]
thread_local! {
    /// AppKit does not keep the picker alive while its menu is open; drop
    /// it early and the menu vanishes. Hold the last one until the next
    /// share replaces it.
    static PICKER: std::cell::RefCell<Option<objc2::rc::Retained<objc2_app_kit::NSSharingServicePicker>>> =
        const { std::cell::RefCell::new(None) };
}

/// Show the share UI for `path`. `anchor` is the clicked button in egui
/// points (top-left origin), used to position the macOS popover.
#[cfg(target_os = "macos")]
pub fn share_file(frame: &eframe::Frame, path: &Path, anchor: egui::Rect) -> anyhow::Result<()> {
    use anyhow::{anyhow, Context};
    use objc2::rc::Retained;
    use objc2::runtime::AnyObject;
    use objc2::MainThreadMarker;
    use objc2_app_kit::{NSSharingServicePicker, NSView};
    use objc2_foundation::{NSArray, NSPoint, NSRect, NSRectEdge, NSSize, NSString, NSURL};
    use raw_window_handle::{HasWindowHandle, RawWindowHandle};

    let mtm = MainThreadMarker::new().ok_or_else(|| anyhow!("share must run on the main thread"))?;
    let handle = frame.window_handle().context("window handle")?.as_raw();
    let RawWindowHandle::AppKit(h) = handle else {
        return Err(anyhow!("not an AppKit window"));
    };
    // SAFETY: winit hands out the window's content NSView; it lives as
    // long as the window, i.e. for the whole call.
    let view: &NSView = unsafe { h.ns_view.cast::<NSView>().as_ref() };

    let p = path.to_str().ok_or_else(|| anyhow!("path is not UTF-8"))?;
    let url = NSURL::fileURLWithPath(&NSString::from_str(p));
    let item: Retained<AnyObject> = url.into();
    let items = NSArray::from_retained_slice(&[item]);
    // SAFETY: the items are file NSURLs, which the picker accepts.
    let picker = unsafe { NSSharingServicePicker::initWithItems(mtm.alloc(), &items) };

    // egui uses a top-left origin; flip if the view does not.
    let bounds = view.bounds();
    let y = if view.isFlipped() {
        anchor.min.y as f64
    } else {
        bounds.size.height - anchor.max.y as f64
    };
    let rect = NSRect::new(
        NSPoint::new(anchor.min.x as f64, y),
        NSSize::new(anchor.width() as f64, anchor.height() as f64),
    );
    picker.showRelativeToRect_ofView_preferredEdge(rect, view, NSRectEdge::MinY);
    PICKER.with(|c| *c.borrow_mut() = Some(picker));
    Ok(())
}

#[cfg(not(target_os = "macos"))]
pub fn share_file(_frame: &eframe::Frame, path: &Path, _anchor: egui::Rect) -> anyhow::Result<()> {
    reveal(path)
}

#[cfg(target_os = "windows")]
fn reveal(path: &Path) -> anyhow::Result<()> {
    std::process::Command::new("explorer")
        .arg(format!("/select,{}", path.display()))
        .spawn()?;
    Ok(())
}

#[cfg(all(not(target_os = "windows"), not(target_os = "macos")))]
fn reveal(path: &Path) -> anyhow::Result<()> {
    let dir = path.parent().unwrap_or(path);
    std::process::Command::new("xdg-open").arg(dir).spawn()?;
    Ok(())
}

/// Button label: a real share sheet on macOS, "Show" elsewhere.
pub const LABEL: &str = if cfg!(target_os = "macos") { "Share…" } else { "Show" };
pub const HINT: &str = if cfg!(target_os = "macos") {
    "Share the downloaded file (AirDrop, Mail, Messages …)"
} else {
    "Show the downloaded file in the file manager"
};
