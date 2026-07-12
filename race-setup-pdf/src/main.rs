//! One-page Race Mode setup PDF — the hand-out for riders and race
//! committees (Desktop + Android + iPhone/Apple Watch).
//!
//! Usage: `cargo run -p race-setup-pdf [output.pdf]`
//! Default output: `MovementLogger_RaceMode_Setup.pdf` in the CWD.
//!
//! Layout is a simple top-down cursor over an A4 page with the built-in
//! Helvetica fonts (no font files, deterministic output). Lines are
//! pre-wrapped by hand — keep body lines under ~105 characters so the
//! page never overflows.

use printpdf::{BuiltinFont, Color, IndirectFontRef, Line, Mm, PdfDocument, Point, Rgb};
use std::fs::File;
use std::io::BufWriter;

const PAGE_W: f32 = 210.0;
const PAGE_H: f32 = 297.0;
const MARGIN: f32 = 14.0;

struct Page {
    layer: printpdf::PdfLayerReference,
    regular: IndirectFontRef,
    bold: IndirectFontRef,
    oblique: IndirectFontRef,
    /// Top-down cursor, in mm from the page bottom (printpdf's origin).
    y: f32,
}

impl Page {
    fn text(&mut self, font: &IndirectFontRef, size: f32, indent: f32, s: &str) {
        // use_text's y is the baseline; step the cursor by the line
        // height first so `y` always means "next free baseline".
        self.y -= size * 0.42; // pt → mm plus a little leading
        self.layer
            .use_text(s, size, Mm(MARGIN + indent), Mm(self.y), font);
    }

    fn heading(&mut self, s: &str) {
        self.y -= 3.2;
        let bold = self.bold.clone();
        self.text(&bold, 11.0, 0.0, s);
        self.rule(0.75);
        self.y -= 1.2;
    }

    fn body(&mut self, s: &str) {
        let regular = self.regular.clone();
        self.text(&regular, 8.8, 2.0, s);
    }

    fn note(&mut self, s: &str) {
        let oblique = self.oblique.clone();
        self.text(&oblique, 8.2, 2.0, s);
    }

    fn gap(&mut self, mm: f32) {
        self.y -= mm;
    }

    fn rule(&mut self, below: f32) {
        let y = self.y - below;
        let line = Line {
            points: vec![
                (Point::new(Mm(MARGIN), Mm(y)), false),
                (Point::new(Mm(PAGE_W - MARGIN), Mm(y)), false),
            ],
            is_closed: false,
        };
        self.layer.set_outline_thickness(0.4);
        self.layer
            .set_outline_color(Color::Rgb(Rgb::new(0.55, 0.55, 0.55, None)));
        self.layer.add_line(line);
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let out = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "MovementLogger_RaceMode_Setup.pdf".into());

    let (doc, page_idx, layer_idx) =
        PdfDocument::new("Movement Logger — Race Mode Setup", Mm(PAGE_W), Mm(PAGE_H), "page");
    let layer = doc.get_page(page_idx).get_layer(layer_idx);
    let mut p = Page {
        layer,
        regular: doc.add_builtin_font(BuiltinFont::Helvetica)?,
        bold: doc.add_builtin_font(BuiltinFont::HelveticaBold)?,
        oblique: doc.add_builtin_font(BuiltinFont::HelveticaOblique)?,
        y: PAGE_H - MARGIN,
    };

    // --- Title ----------------------------------------------------------
    let bold = p.bold.clone();
    p.text(&bold, 17.0, 0.0, "Movement Logger — Race Mode Setup");
    let oblique = p.oblique.clone();
    p.gap(1.5);
    p.text(
        &oblique,
        9.5,
        0.0,
        "Live rider tracking: every rider's phone streams its GPS position to one shared map on the desktop.",
    );
    p.gap(1.0);

    // --- Requirements -----------------------------------------------------
    p.heading("Requirements");
    p.body("- All phones and the desktop computer on the same WiFi network (club/venue WiFi works fine).");
    p.body("- Versions: Desktop app 0.0.67 or newer, Android app 0.0.52 or newer, iOS app 1.0.29 or newer.");
    p.body("- Android riders: a u-blox USB GNSS receiver plugged into the phone. iPhone riders: nothing extra.");

    // --- Desktop ----------------------------------------------------------
    p.heading("1 — Desktop (race committee)");
    p.body("1. Install: github.com/zdavatz/movement_logger_desktop -> Releases (macOS DMG, Windows zip, Linux tar).");
    p.body("2. Open the app -> Race tab -> click \"Start listening\".");
    p.body("3. The header now shows:  Phones send to  <IP>:47777   — riders enter exactly these two values.");
    p.body("4. Map controls: scroll or double-click or the -/+ buttons to zoom; \"Satellite\" switches to imagery;");
    p.body("   \"Follow riders\" keeps everyone in view automatically. macOS firewall prompt: click Allow.");
    p.note("Every received fix is also logged to ~/.movementlogger/race/race_<timestamp>.csv for post-race analysis.");

    // --- Android ----------------------------------------------------------
    p.heading("2 — Android rider (u-blox USB GPS)");
    p.body("1. Install \"Movement Logger\" from Google Play.");
    p.body("2. Plug the u-blox receiver into USB-C -> GPS tab -> Connect (allow USB access).");
    p.body("3. In the \"Race mode\" card: enter your rider name + the desktop IP and port from step 1.3 -> toggle ON.");
    p.body("4. \"Sending — N fixes\" counts up as soon as the receiver has a GPS fix (needs sky view — go outside).");

    // --- iOS --------------------------------------------------------------
    p.heading("3 — iPhone / Apple Watch rider");
    p.body("1. Install \"Movement Logger\" from the App Store.");
    p.body("2. GPS tab -> \"Race mode\" card: rider name + the desktop IP and port, then pick the GPS source:");
    p.body("   - iPhone GPS: toggle ON — done. The phone's own GPS starts automatically.");
    p.body("   - Apple Watch: toggle ON, then start a recording in the watch app. The watch streams its fixes");
    p.body("     through the paired iPhone — keep the iPhone with you (it is the uplink).");

    // --- Reading the map ----------------------------------------------------
    p.heading("Reading the map");
    p.body("- Dot = the rider's latest position; the translucent circle around it is the receiver's own accuracy");
    p.body("  estimate (bigger circle = less precise fix). The coloured trail is the recently travelled path.");
    p.body("- The rider list shows: name, speed, source (ublox / phone / watch), +-accuracy, satellites, battery.");
    p.body("- A rider greyed out has sent nothing for 5 s (out of range, capsized, app closed) — the last");
    p.body("  position stays on the map on purpose.");

    // --- Troubleshooting ----------------------------------------------------
    p.heading("Troubleshooting");
    p.body("- No dot appears: same WiFi? IP and port typed exactly as the desktop shows? GPS fix yet (sky view)?");
    p.body("- Everything jumps or looks imprecise near buildings: watch the accuracy circle — GPS is a 2-5 m");
    p.body("  instrument; on open water it is at its best.");
    p.body("- Two riders must not use the same rider name on the same source type.");

    // --- Footer -------------------------------------------------------------
    p.gap(4.0);
    p.rule(0.0);
    p.gap(3.0);
    let regular = p.regular.clone();
    p.text(
        &regular,
        8.0,
        0.0,
        "github.com/zdavatz/movement_logger_desktop   ·   movement_logger_android   ·   movement_logger_ios",
    );

    doc.save(&mut BufWriter::new(File::create(&out)?))?;
    println!("wrote {out}");
    Ok(())
}
