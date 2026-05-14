//! Shared plotters helpers: font loading, min:sec time formatter,
//! robust axis-bound computation.

use plotters::prelude::*;

/// Format seconds as `m:ss` for the time axis.
pub fn fmt_min_sec(t: &f64) -> String {
    let total = t.round() as i64;
    let m = total / 60;
    let s = (total % 60).abs();
    format!("{}:{:02}", m, s)
}

/// Robust y-range: min/max ignoring NaN, with padding. Falls back to
/// (-1, 1) for empty / all-NaN inputs so plotters doesn't panic.
pub fn yrange(values: &[f64], pad_frac: f64) -> (f64, f64) {
    let mut mn = f64::INFINITY;
    let mut mx = f64::NEG_INFINITY;
    for &v in values {
        if v.is_finite() {
            if v < mn { mn = v; }
            if v > mx { mx = v; }
        }
    }
    if !mn.is_finite() || !mx.is_finite() {
        return (-1.0, 1.0);
    }
    if (mx - mn).abs() < 1e-9 {
        return (mn - 1.0, mx + 1.0);
    }
    let span = mx - mn;
    (mn - span * pad_frac, mx + span * pad_frac)
}

/// System font pick that should work across macOS / Linux without
/// fontconfig gymnastics. Plotters falls back to DejaVu Sans if it
/// can't find the first choice, which is good enough.
pub const FONT: &str = "sans-serif";

/// Default chart margins (px).
pub const MARGIN: u32 = 8;

/// Draw a simple legend box in the top-right of a chart area. Plotters'
/// built-in `configure_series_labels` works for most charts but draws
/// inside the plotting area; this one is a manual overlay that we can
/// place exactly.
#[allow(dead_code)]
pub fn legend_lines<DB: DrawingBackend>(
    root: &DrawingArea<DB, plotters::coord::Shift>,
    labels: &[(&str, RGBColor)],
) -> Result<(), DrawingAreaErrorKind<DB::ErrorType>> {
    let (w, _h) = root.dim_in_pixel();
    let x = (w as i32) - 140;
    let y0 = 10;
    for (i, (label, color)) in labels.iter().enumerate() {
        let y = y0 + (i as i32) * 18;
        root.draw(&PathElement::new(
            vec![(x, y + 7), (x + 24, y + 7)],
            color.stroke_width(2),
        ))?;
        root.draw(&Text::new(
            label.to_string(),
            (x + 30, y),
            (FONT, 13).into_font().color(&BLACK),
        ))?;
    }
    Ok(())
}
