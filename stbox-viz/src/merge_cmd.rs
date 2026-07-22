//! `stbox-viz merge` — merge multiple session videos into one film.
//!
//! For every input video (sorted chronologically by capture time):
//!   1. a title card (`card_seconds`, default 2.5 s) with the recording
//!      date and start time,
//!   2. the clip itself — NEVER trimmed or cut; when `--sensor-csv` is
//!      given each clip is first rendered through the `animate` pipeline
//!      (camera left, sensor animation right, aligned via the CSV's
//!      `# SYNC` anchors), otherwise the plain clip is used,
//!   3. the clip's last frame frozen and faded to black over
//!      `fade_seconds` (default 3 s).
//! The film OPENS with the "MovementLogger" gradient title over the first
//! clip's first frame (semi-transparent), and CLOSES with the foil logo
//! PUMPING — rocking about its wings + a synced bob + squash over a sky→sea
//! gradient (cadence mapped from the pumping footage), fading in from the
//! last freeze's black and back out. All pieces are normalized to a common
//! canvas (height 900, width = widest segment) and concatenated into a
//! single H.264 .mov.
//!
//! Capture times come from `com.apple.quicktime.creationdate` (survives
//! iOS share/export rewrites of the container `creation_time`, which is
//! only the fallback), else the file mtime. Card times are shown in the
//! host's local timezone unless `--tz-offset-h` is given.

use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{bail, Context, Result};
use chrono::{DateTime, FixedOffset, Local, Offset, TimeZone, Utc};
use plotters::prelude::*;
use plotters::style::text_anchor::{HPos, Pos, VPos};

use crate::animate_cmd::{self, tempfile, AnimateArgs};
use crate::board3d;
use crate::plot_common::FONT;

pub struct MergeArgs<'a> {
    pub videos: &'a [PathBuf],
    pub sensor_csv: Option<&'a Path>,
    pub output: &'a Path,
    /// Display timezone for the cards as UTC offset in hours; None =
    /// host local time.
    pub tz_offset_h: Option<f64>,
    pub mount: board3d::MountKind,
    pub trace_label: Option<&'a str>,
    pub card_seconds: f64,
    pub fade_seconds: f64,
    pub fps: u32,
    /// Seconds of the "MovementLogger" intro card at the film's start
    /// (text in the logo's colors); 0 disables.
    pub intro_seconds: f64,
    /// Seconds of Pump Tsüri logo outro at the film's end; 0 disables.
    pub logo_seconds: f64,
    /// Override the embedded logo image.
    pub logo: Option<&'a Path>,
}

/// The Pump Tsüri foil logo (same asset as the GUI's top-right icon and
/// iOS RideLogo), shown as the film's outro.
const LOGO_PNG: &[u8] = include_bytes!("../assets/logo.png");

struct Clip {
    path: PathBuf,
    /// Capture start in UTC.
    start_utc: DateTime<Utc>,
}

/// Machine-readable progress line for the GUI ("[progress] <0-100>").
/// Plain stdout so the CLI stays scriptable; the GUI's log pump
/// intercepts these lines and feeds its progress bar instead of the
/// log panel.
fn emit_progress(done: usize, total: usize) {
    let pct = ((done as f64 / total.max(1) as f64) * 100.0).round() as u32;
    println!("[progress] {}", pct.min(100));
}

pub fn run(args: &MergeArgs) -> Result<()> {
    if args.videos.is_empty() {
        bail!("no videos given");
    }
    let mut clips: Vec<Clip> = args
        .videos
        .iter()
        .map(|v| {
            let start_utc = probe_capture_time(v)
                .with_context(|| format!("capture time of {}", v.display()))?;
            Ok(Clip { path: v.clone(), start_utc })
        })
        .collect::<Result<_>>()?;
    clips.sort_by_key(|c| c.start_utc);

    let disp_offset: FixedOffset = match args.tz_offset_h {
        Some(h) => FixedOffset::east_opt((h * 3600.0) as i32)
            .context("bad --tz-offset-h")?,
        None => Local::now().offset().fix(),
    };

    let tmp = tempfile::tempdir()?;
    let work = tmp.path();

    // Progress accounting: one step per sensor render (heavy), one per
    // segment encode, one for the final concat.
    let total_steps =
        if args.sensor_csv.is_some() { clips.len() * 2 } else { clips.len() } + 1;
    let mut steps_done = 0usize;
    emit_progress(0, total_steps);

    // Pass 1: build each clip's segment source (plain clip, or the
    // animate side-by-side render when a sensor CSV is present).
    let mut seg_sources: Vec<PathBuf> = Vec::new();
    for (i, c) in clips.iter().enumerate() {
        let src = if let Some(csv) = args.sensor_csv {
            let at = c.start_utc.format("%H:%M:%S").to_string();
            let date = c.start_utc.format("%Y-%m-%d").to_string();
            let out_dir = work.join(format!("anim{}", i));
            std::fs::create_dir_all(&out_dir)?;
            println!(
                "[{}/{}] render {} at {} UTC",
                i + 1,
                clips.len(),
                c.path.file_name().unwrap_or_default().to_string_lossy(),
                at
            );
            animate_cmd::run(&AnimateArgs {
                sensor_csv: csv,
                output_dir: &out_dir,
                fps: args.fps,
                session: None,
                video: Some(&c.path),
                video_offset: 0.0,
                sensor_offset: 0.0,
                title: None,
                subtitle: None,
                trace_label: args.trace_label,
                at: Some(&at),
                duration: None,
                tz_offset_h: 0.0,
                date: Some(&date),
                auto_skip: false,
                dock_height_m: 0.0,
                board_stl: None,
                mount: args.mount,
            })
            .with_context(|| format!("animate render for {}", c.path.display()))?;
            // animate writes combined_<csvstem>_at<HHMMSS>.mov
            let mov = std::fs::read_dir(&out_dir)?
                .filter_map(|e| e.ok())
                .map(|e| e.path())
                .find(|p| {
                    p.extension().map(|e| e == "mov").unwrap_or(false)
                        && p.file_name()
                            .map(|n| n.to_string_lossy().starts_with("combined_"))
                            .unwrap_or(false)
                })
                .context("animate produced no combined .mov")?;
            mov
        } else {
            c.path.clone()
        };
        if args.sensor_csv.is_some() {
            steps_done += 1;
            emit_progress(steps_done, total_steps);
        }
        seg_sources.push(src);
    }

    // Common canvas: height 900, width = widest segment (kept even).
    // Canvas width = the NARROWEST clip. The user films portrait and
    // never wants black side bars; with mixed portrait aspects (9:16
    // camera clips vs 3:4 QuickTake) the narrow 9:16 majority fills the
    // frame edge-to-edge and the wider 3:4 clips letterbox top/bottom
    // instead (cropping them is off the table — footage is never cut).
    let mut canvas_w: u32 = u32::MAX;
    for s in &seg_sources {
        let (w, h) = probe_display_dims(s)?;
        let w900 = ((w as f64) * 900.0 / (h as f64)).round() as u32;
        canvas_w = canvas_w.min(w900 & !1);
    }
    let canvas_w = canvas_w.clamp(2, 4096);

    // Pass 2: cards, normalized segments, freeze-fades → concat list.
    let mut list = String::new();

    // Intro: "MovementLogger" in the logo's colors, over the FIRST clip's
    // first frame (aspect-fit into the canvas), semi-transparent. Falls back
    // to the gradient title on black if the frame can't be extracted.
    if args.intro_seconds > 0.0 {
        let title_color = work.join("intro_title.png");
        let title_mask = work.join("intro_mask.png");
        draw_intro(&title_color, canvas_w, 900, false)?;
        draw_intro(&title_mask, canvas_w, 900, true)?;
        let intro_png = work.join("intro.png");
        let norm_vf = format!(
            "scale=w={cw}:h=900:force_original_aspect_ratio=decrease:force_divisible_by=2,\
             pad={cw}:900:(ow-iw)/2:(oh-ih)/2:color=black,setsar=1",
            cw = canvas_w
        );
        let mut composed = false;
        if let Some(s) = seg_sources.first() {
            let bg = work.join("intro_bg.png");
            if ffmpeg(&[
                "-y", "-ss", "0", "-i", s.to_str().unwrap(),
                "-frames:v", "1", "-vf", &norm_vf,
                bg.to_str().unwrap(),
            ])
            .is_ok()
                && composite_intro(&bg, &title_color, &title_mask, &intro_png, 0.85).is_ok()
            {
                composed = true;
            }
        }
        if !composed {
            draw_intro(&intro_png, canvas_w, 900, false)?;
        }
        let intro_mp4 = work.join("intro.mp4");
        ffmpeg(&[
            "-y", "-loop", "1", "-i", intro_png.to_str().unwrap(),
            "-f", "lavfi", "-i", "anullsrc=r=48000:cl=stereo",
            "-t", &args.intro_seconds.to_string(),
            "-r", "30", "-c:v", "libx264", "-pix_fmt", "yuv420p",
            "-crf", "20", "-c:a", "aac",
            intro_mp4.to_str().unwrap(),
        ])?;
        list.push_str(&format!("file '{}'\n", intro_mp4.display()));
    }
    for (i, (c, src)) in clips.iter().zip(&seg_sources).enumerate() {
        let local = c.start_utc.with_timezone(&disp_offset);
        let date_s = local.format("%d.%m.%Y").to_string();
        let time_s = local.format("%H:%M:%S").to_string();

        let card_png = work.join(format!("card{}.png", i));
        draw_card(&card_png, canvas_w, 900, &date_s, &time_s)?;
        let card_mp4 = work.join(format!("card{}.mp4", i));
        ffmpeg(&[
            "-y", "-loop", "1", "-i", card_png.to_str().unwrap(),
            "-f", "lavfi", "-i", "anullsrc=r=48000:cl=stereo",
            "-t", &args.card_seconds.to_string(),
            "-r", "30", "-c:v", "libx264", "-pix_fmt", "yuv420p",
            "-crf", "20", "-c:a", "aac",
            card_mp4.to_str().unwrap(),
        ])?;

        let seg_mp4 = work.join(format!("seg{}.mp4", i));
        // Fit-into-canvas: scale to touch the canvas from inside (a
        // 9:16 clip fills it exactly; a wider clip fits the width and
        // letterboxes top/bottom), then center-pad to the exact canvas.
        let vf = format!(
            "scale=w={cw}:h=900:force_original_aspect_ratio=decrease:force_divisible_by=2,\
             pad={cw}:900:(ow-iw)/2:(oh-ih)/2:color=black,setsar=1,fps=30",
            cw = canvas_w
        );
        if has_audio(src) {
            ffmpeg(&[
                "-y", "-i", src.to_str().unwrap(), "-vf", &vf,
                "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
                "-c:a", "aac", "-ar", "48000", "-ac", "2",
                seg_mp4.to_str().unwrap(),
            ])?;
        } else {
            ffmpeg(&[
                "-y", "-i", src.to_str().unwrap(),
                "-f", "lavfi", "-i", "anullsrc=r=48000:cl=stereo",
                "-vf", &vf,
                "-map", "0:v", "-map", "1:a", "-shortest",
                "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
                "-c:a", "aac",
                seg_mp4.to_str().unwrap(),
            ])?;
        }

        let last_png = work.join(format!("last{}.png", i));
        ffmpeg(&[
            "-y", "-sseof", "-0.2", "-i", seg_mp4.to_str().unwrap(),
            "-update", "1", "-frames:v", "1", last_png.to_str().unwrap(),
        ])?;
        let fade_mp4 = work.join(format!("fade{}.mp4", i));
        let fade_vf = format!("fade=t=out:st=0:d={}", args.fade_seconds);
        ffmpeg(&[
            "-y", "-loop", "1", "-i", last_png.to_str().unwrap(),
            "-f", "lavfi", "-i", "anullsrc=r=48000:cl=stereo",
            "-t", &args.fade_seconds.to_string(),
            "-r", "30", "-vf", &fade_vf,
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
            "-c:a", "aac",
            fade_mp4.to_str().unwrap(),
        ])?;

        for p in [card_mp4.as_path(), seg_mp4.as_path(), fade_mp4.as_path()] {
            list.push_str(&format!("file '{}'\n", p.display()));
        }
        println!("[{}/{}] segment ready ({} {})", i + 1, clips.len(), date_s, time_s);
        steps_done += 1;
        emit_progress(steps_done, total_steps);
    }

    // Outro: the foil logo PUMPING (rocking about its wings + bob + squash)
    // over a sky→sea gradient, fading in from black and back out. Frames
    // drawn in Rust, encoded by ffmpeg. `logo_seconds` sets its length.
    if args.logo_seconds > 0.0 {
        let logo_bytes: Vec<u8> = match args.logo {
            Some(p) => std::fs::read(p)
                .with_context(|| format!("read logo {}", p.display()))?,
            None => LOGO_PNG.to_vec(),
        };
        let outro = render_pump_outro(work, &logo_bytes, canvas_w, 900, 30, args.logo_seconds)?;
        list.push_str(&format!("file '{}'\n", outro.display()));
    }

    let list_path = work.join("list.txt");
    std::fs::write(&list_path, list)?;
    if let Some(parent) = args.output.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)?;
        }
    }
    ffmpeg(&[
        "-y", "-f", "concat", "-safe", "0", "-i", list_path.to_str().unwrap(),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
        "-c:a", "aac",
        args.output.to_str().unwrap(),
    ])?;
    emit_progress(total_steps, total_steps);
    println!("Saved {}", args.output.display());
    Ok(())
}

/// True when the file has at least one audio stream.
fn has_audio(p: &Path) -> bool {
    Command::new("ffprobe")
        .args(["-v", "error", "-select_streams", "a",
               "-show_entries", "stream=index", "-of", "csv=p=0"])
        .arg(p)
        .output()
        .map(|o| !String::from_utf8_lossy(&o.stdout).trim().is_empty())
        .unwrap_or(false)
}

fn ffmpeg(a: &[&str]) -> Result<()> {
    let out = Command::new("ffmpeg")
        .args(a)
        .output()
        .context("running ffmpeg — is it in PATH?")?;
    if !out.status.success() {
        bail!(
            "ffmpeg failed ({:?}): {}",
            a.first().map(|_| a.join(" ")),
            String::from_utf8_lossy(&out.stderr)
                .lines()
                .rev()
                .take(4)
                .collect::<Vec<_>>()
                .join(" | ")
        );
    }
    Ok(())
}

/// Capture start of a clip in UTC. Prefers the Apple capture-date tag,
/// falls back to the container tag, then the file mtime.
fn probe_capture_time(video: &Path) -> Result<DateTime<Utc>> {
    for tag in ["com.apple.quicktime.creationdate", "creation_time"] {
        let out = Command::new("ffprobe")
            .args([
                "-v", "error", "-show_entries",
                &format!("format_tags={}", tag),
                "-of", "default=noprint_wrappers=1:nokey=1",
            ])
            .arg(video)
            .output()
            .context("running ffprobe — is it in PATH?")?;
        let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
        if s.is_empty() {
            continue;
        }
        if let Ok(dt) = DateTime::parse_from_rfc3339(&s) {
            return Ok(dt.with_timezone(&Utc));
        }
        // Apple writes e.g. 2026-07-18T10:34:07+0300 (no colon in offset).
        if let Ok(dt) = DateTime::parse_from_str(&s, "%Y-%m-%dT%H:%M:%S%z") {
            return Ok(dt.with_timezone(&Utc));
        }
        if let Ok(dt) = DateTime::parse_from_str(&s, "%Y-%m-%dT%H:%M:%S%.f%z") {
            return Ok(dt.with_timezone(&Utc));
        }
    }
    let mtime = std::fs::metadata(video)?.modified()?;
    let secs = mtime
        .duration_since(std::time::UNIX_EPOCH)
        .context("mtime before epoch")?
        .as_secs() as i64;
    Ok(Utc.timestamp_opt(secs, 0).single().context("bad mtime")?)
}

/// Displayed (post-rotation) pixel dimensions of a video.
fn probe_display_dims(video: &Path) -> Result<(u32, u32)> {
    let out = Command::new("ffprobe")
        .args([
            "-v", "error", "-select_streams", "v:0", "-show_entries",
            "stream=width,height:stream_side_data=rotation",
            "-of", "csv=p=0",
        ])
        .arg(video)
        .output()
        .context("running ffprobe — is it in PATH?")?;
    let text = String::from_utf8_lossy(&out.stdout);
    let first = text.lines().next().unwrap_or_default();
    let parts: Vec<&str> = first.split(',').collect();
    let w: u32 = parts.first().and_then(|s| s.trim().parse().ok())
        .with_context(|| format!("probe dims of {}: {:?}", video.display(), first))?;
    let h: u32 = parts.get(1).and_then(|s| s.trim().parse().ok())
        .with_context(|| format!("probe dims of {}: {:?}", video.display(), first))?;
    // The rotation side-data lands at a varying field position (iPhone
    // clips print "w,h,,-90" — an empty field first). Take the first
    // trailing field that parses; treating it as absent turned every
    // portrait clip landscape and baked pillar-box bars into the film.
    let rot: i32 = parts[2.min(parts.len())..]
        .iter()
        .find_map(|s| s.trim().parse().ok())
        .unwrap_or(0);
    if rot.abs() % 180 == 90 {
        Ok((h, w))
    } else {
        Ok((w, h))
    }
}

/// The logo's palette (board orange → mast teal → wing blue → wing
/// purple), lerped across the letters of the intro text.
const INTRO_STOPS: [(u8, u8, u8); 4] =
    [(247, 154, 51), (36, 195, 188), (62, 141, 243), (125, 77, 240)];

fn intro_color(t: f64) -> RGBColor {
    let x = (t.clamp(0.0, 1.0)) * (INTRO_STOPS.len() - 1) as f64;
    let i = (x.floor() as usize).min(INTRO_STOPS.len() - 2);
    let f = x - i as f64;
    let (a, b) = (INTRO_STOPS[i], INTRO_STOPS[i + 1]);
    RGBColor(
        (a.0 as f64 + (b.0 as f64 - a.0 as f64) * f) as u8,
        (a.1 as f64 + (b.1 as f64 - a.1 as f64) * f) as u8,
        (a.2 as f64 + (b.2 as f64 - a.2 as f64) * f) as u8,
    )
}

// ---- Pumping-foil outro (mapped from Ayano's IMG_5266.MOV pumping
// footage): the foil rocks about its wings + a synced vertical bob + squash
// over a sky→sea gradient, fading in from black and back out.
const PUMP_PERIOD_S: f64 = 1.05; // ~3 pumps over 3 s
const PUMP_PITCH_DEG: f64 = 11.0; // rock amplitude (°)
const PUMP_HEAVE_FRAC: f64 = 0.021; // vertical bob (of height)
const PUMP_SQUASH: f64 = 0.035; // compress/extend
const PUMP_LOGO_FRAC: f64 = 0.55; // foil size (of height)
const PUMP_PIVOT_FRAC: f64 = 0.78; // wings pivot (down the foil)
const PUMP_FADE_IN_S: f64 = 0.35;
const PUMP_FADE_OUT_S: f64 = 0.75;
/// Sky → horizon → sea gradient (RGB) for the outro background.
const PUMP_SKY: (f64, f64, f64) = (189.0, 227.0, 245.0);
const PUMP_HORIZON: (f64, f64, f64) = (107.0, 184.0, 212.0);
const PUMP_SEA: (f64, f64, f64) = (28.0, 87.0, 128.0);

/// Vertical sky→sea gradient colour at `t` in [0,1] (0 = top).
fn sea_gradient(t: f64) -> (f64, f64, f64) {
    let lerp = |a: (f64, f64, f64), b: (f64, f64, f64), f: f64| {
        (a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f, a.2 + (b.2 - a.2) * f)
    };
    if t <= 0.52 {
        lerp(PUMP_SKY, PUMP_HORIZON, (t / 0.52).clamp(0.0, 1.0))
    } else {
        lerp(PUMP_HORIZON, PUMP_SEA, ((t - 0.52) / 0.48).clamp(0.0, 1.0))
    }
}

/// Bilinear RGBA sample of `logo` at (fx, fy) in image pixels. Returns
/// (r, g, b) in 0..255 and alpha in 0..1.
fn sample_logo(logo: &image::RgbaImage, fx: f64, fy: f64) -> (f64, f64, f64, f64) {
    let (w, h) = (logo.width() as i64, logo.height() as i64);
    let x0 = fx.floor() as i64;
    let y0 = fy.floor() as i64;
    let dx = fx - x0 as f64;
    let dy = fy - y0 as f64;
    let at = |x: i64, y: i64| -> (f64, f64, f64, f64) {
        let xc = x.clamp(0, w - 1) as u32;
        let yc = y.clamp(0, h - 1) as u32;
        let p = logo.get_pixel(xc, yc).0;
        (p[0] as f64, p[1] as f64, p[2] as f64, p[3] as f64 / 255.0)
    };
    let (a, b, c, d) = (
        at(x0, y0), at(x0 + 1, y0), at(x0, y0 + 1), at(x0 + 1, y0 + 1),
    );
    let mix = |p: f64, q: f64, f: f64| p + (q - p) * f;
    let r = mix(mix(a.0, b.0, dx), mix(c.0, d.0, dx), dy);
    let g = mix(mix(a.1, b.1, dx), mix(c.1, d.1, dx), dy);
    let bl = mix(mix(a.2, b.2, dx), mix(c.2, d.2, dx), dy);
    let al = mix(mix(a.3, b.3, dx), mix(c.3, d.3, dx), dy);
    (r, g, bl, al)
}

/// One frame of the pumping outro (RGB): sky→sea gradient with the foil
/// rocked about its wings + bob + squash, plus fade in/out to black. Frame
/// `i` of `n`. Mirrors the iOS `pumpFrameImage` transform, worked in the
/// image crate's y-down space.
fn render_pump_frame(
    w: u32, h: u32, logo: &image::RgbaImage, i: usize, n: usize, fps: u32,
) -> image::RgbImage {
    use std::f64::consts::PI;
    let (fw, fh) = (w as f64, h as f64);
    let secs = i as f64 / fps as f64;
    let dur = n as f64 / fps as f64;
    let phase = 2.0 * PI / PUMP_PERIOD_S * secs;

    let logo_iw = logo.width() as f64;
    let logo_ih = logo.height() as f64;
    let logo_h = fh * PUMP_LOGO_FRAC;
    let logo_w = logo_h * logo_iw / logo_ih;
    let left = (fw - logo_w) / 2.0;
    let top = (fh - logo_h) / 2.0;
    let piv_x = left + logo_w / 2.0;
    let piv_y = top + PUMP_PIVOT_FRAC * logo_h; // wings, down from the top

    let theta = PUMP_PITCH_DEG * phase.sin() * PI / 180.0;
    let dy = -PUMP_HEAVE_FRAC * fh * phase.sin(); // rise on nose-up
    let sy = 1.0 - PUMP_SQUASH * phase.cos(); // squash at compress
    let (ct, st) = (theta.cos(), theta.sin());

    let cover = if secs < PUMP_FADE_IN_S {
        1.0 - secs / PUMP_FADE_IN_S
    } else if secs > dur - PUMP_FADE_OUT_S {
        (secs - (dur - PUMP_FADE_OUT_S)) / PUMP_FADE_OUT_S
    } else {
        0.0
    }
    .clamp(0.0, 1.0);

    // Forward-transform the foil's rect corners to bound the (expensive)
    // inverse-mapped sampling region; the gradient + fade cover the rest.
    let fwd = |rx: f64, ry: f64| -> (f64, f64) {
        let xs = rx;
        let ys = piv_y + (ry - piv_y) * sy; // scale about pivot
        let ( dx0, dy0) = (xs - piv_x, ys - piv_y);
        let xr = piv_x + dx0 * ct - dy0 * st; // rotate about pivot
        let yr = piv_y + dx0 * st + dy0 * ct;
        (xr, yr + dy) // heave
    };
    let corners = [
        fwd(left, top), fwd(left + logo_w, top),
        fwd(left, top + logo_h), fwd(left + logo_w, top + logo_h),
    ];
    let pad = 2.0;
    let bx0 = corners.iter().map(|c| c.0).fold(f64::MAX, f64::min).floor().max(0.0) - pad;
    let bx1 = corners.iter().map(|c| c.0).fold(f64::MIN, f64::max).ceil().min(fw) + pad;
    let by0 = corners.iter().map(|c| c.1).fold(f64::MAX, f64::min).floor().max(0.0) - pad;
    let by1 = corners.iter().map(|c| c.1).fold(f64::MIN, f64::max).ceil().min(fh) + pad;
    let (bx0, by0) = (bx0.max(0.0) as u32, by0.max(0.0) as u32);
    let (bx1, by1) = ((bx1.min(fw)) as u32, (by1.min(fh)) as u32);

    let mut img = image::RgbImage::new(w, h);
    for y in 0..h {
        let base = sea_gradient(y as f64 / (fh - 1.0).max(1.0));
        for x in 0..w {
            let mut gr = base.0;
            let mut gg = base.1;
            let mut gb = base.2;
            if x >= bx0 && x < bx1 && y >= by0 && y < by1 {
                // Inverse-map this output pixel back into logo space.
                let ox = x as f64 + 0.5;
                let oy = y as f64 + 0.5;
                let y1 = oy - dy; // un-heave
                let (dx0, dy0) = (ox - piv_x, y1 - piv_y);
                let xr = piv_x + dx0 * ct + dy0 * st; // un-rotate
                let yr = piv_y - dx0 * st + dy0 * ct;
                let ys = piv_y + (yr - piv_y) / sy; // un-scale
                let lx = (xr - left) / logo_w * logo_iw;
                let ly = (ys - top) / logo_h * logo_ih;
                if lx >= 0.0 && ly >= 0.0 && lx < logo_iw && ly < logo_ih {
                    let (lr, lg, lb, la) = sample_logo(logo, lx, ly);
                    if la > 0.0 {
                        gr = gr * (1.0 - la) + lr * la;
                        gg = gg * (1.0 - la) + lg * la;
                        gb = gb * (1.0 - la) + lb * la;
                    }
                }
            }
            if cover > 0.0 {
                let k = 1.0 - cover;
                gr *= k;
                gg *= k;
                gb *= k;
            }
            img.put_pixel(
                x, y,
                image::Rgb([
                    gr.round().clamp(0.0, 255.0) as u8,
                    gg.round().clamp(0.0, 255.0) as u8,
                    gb.round().clamp(0.0, 255.0) as u8,
                ]),
            );
        }
    }
    img
}

/// Render the pumping-foil outro to `<work>/outro.mp4` (canvas-sized,
/// `seconds` long at `fps`) and return its path. PNG frames drawn in Rust,
/// encoded by ffmpeg (with a silent audio track for the concat).
fn render_pump_outro(
    work: &Path, logo_bytes: &[u8], w: u32, h: u32, fps: u32, seconds: f64,
) -> Result<PathBuf> {
    let logo = image::load_from_memory(logo_bytes)
        .context("decode outro logo")?
        .to_rgba8();
    let frames_dir = work.join("pump");
    std::fs::create_dir_all(&frames_dir)?;
    let n = ((seconds * fps as f64).round() as usize).max(1);
    for i in 0..n {
        let frame = render_pump_frame(w, h, &logo, i, n, fps);
        frame
            .save(frames_dir.join(format!("f{:04}.png", i)))
            .with_context(|| format!("write pump frame {i}"))?;
    }
    let pattern = frames_dir.join("f%04d.png");
    let outro = work.join("outro.mp4");
    ffmpeg(&[
        "-y", "-framerate", &fps.to_string(), "-i", pattern.to_str().unwrap(),
        "-f", "lavfi", "-i", "anullsrc=r=48000:cl=stereo",
        "-t", &seconds.to_string(),
        "-r", "30", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
        "-c:a", "aac", "-shortest",
        outro.to_str().unwrap(),
    ])?;
    Ok(outro)
}

/// Composite the gradient title (`color_png`) over the first-frame
/// background (`bg_png`) at `alpha`, using the white coverage mask
/// (`mask_png`) so every letter blends at the same strength regardless of
/// its gradient colour. Writes `out_png`.
fn composite_intro(
    bg_png: &Path, color_png: &Path, mask_png: &Path, out_png: &Path, alpha: f64,
) -> Result<()> {
    let mut bg = image::open(bg_png).context("open intro bg")?.to_rgb8();
    let color = image::open(color_png).context("open intro title")?.to_rgb8();
    let mask = image::open(mask_png).context("open intro mask")?.to_rgb8();
    let (w, h) = (bg.width().min(color.width()).min(mask.width()),
                  bg.height().min(color.height()).min(mask.height()));
    for y in 0..h {
        for x in 0..w {
            let m = mask.get_pixel(x, y).0;
            let cov = *m.iter().max().unwrap() as f64 / 255.0;
            if cov <= 0.02 {
                continue;
            }
            let a = (cov * alpha).clamp(0.0, 1.0);
            let c = color.get_pixel(x, y).0;
            let p = bg.get_pixel_mut(x, y);
            for k in 0..3 {
                p.0[k] = (p.0[k] as f64 * (1.0 - a) + c[k] as f64 * a)
                    .round()
                    .clamp(0.0, 255.0) as u8;
            }
        }
    }
    bg.save(out_png).context("write intro composite")?;
    Ok(())
}

/// "MovementLogger" title on black, centered, each letter colored along the
/// logo gradient, sized to ~86% of the canvas width. With `mask = true`
/// every letter is drawn solid WHITE instead — a coverage mask so the
/// composite over the first frame (`composite_intro`) blends every letter at
/// the same strength regardless of its gradient colour.
fn draw_intro(path: &Path, w: u32, h: u32, mask: bool) -> Result<()> {
    const TEXT: &str = "MovementLogger";
    let root = BitMapBackend::new(path, (w, h)).into_drawing_area();
    root.fill(&BLACK)
        .map_err(|e| anyhow::anyhow!("intro fill: {e:?}"))?;

    let measure = |size: f64| -> Result<(i32, Vec<i32>)> {
        let font = (FONT, size).into_font();
        let mut widths = Vec::with_capacity(TEXT.len());
        let mut total = 0i32;
        for ch in TEXT.chars() {
            let cs = ch.to_string();
            let ((x0, _), (x1, _)) = font
                .box_size(&cs)
                .map(|(bw, _)| ((0, 0), (bw as i32, 0)))
                .map_err(|e| anyhow::anyhow!("intro measure: {e:?}"))?;
            let cw = x1 - x0;
            widths.push(cw);
            total += cw;
        }
        Ok((total, widths))
    };
    let (base_total, _) = measure(100.0)?;
    let size = (100.0 * (w as f64 * 0.86) / base_total.max(1) as f64).min(180.0);
    let (total, widths) = measure(size)?;

    let mut x = (w as i32 - total) / 2;
    let y = (h as i32) / 2;
    let n = TEXT.chars().count().max(2);
    for (i, ch) in TEXT.chars().enumerate() {
        let color = if mask { WHITE } else { intro_color(i as f64 / (n - 1) as f64) };
        let style = (FONT, size)
            .into_font()
            .color(&color)
            .pos(Pos::new(HPos::Left, VPos::Center));
        root.draw(&Text::new(ch.to_string(), (x, y), style))
            .map_err(|e| anyhow::anyhow!("intro draw: {e:?}"))?;
        x += widths[i];
    }
    root.present()
        .map_err(|e| anyhow::anyhow!("intro present: {e:?}"))?;
    Ok(())
}

/// Black title card with the date above the start time, both centered.
fn draw_card(path: &Path, w: u32, h: u32, date_s: &str, time_s: &str) -> Result<()> {
    let root = BitMapBackend::new(path, (w, h)).into_drawing_area();
    root.fill(&BLACK)
        .map_err(|e| anyhow::anyhow!("card fill: {e:?}"))?;
    let center = Pos::new(HPos::Center, VPos::Center);
    let date_style = (FONT, 72).into_font().color(&WHITE).pos(center);
    let time_style = (FONT, 110).into_font().color(&WHITE).pos(center);
    root.draw(&Text::new(
        date_s.to_string(),
        ((w / 2) as i32, (h / 2) as i32 - 90),
        date_style,
    ))
    .map_err(|e| anyhow::anyhow!("card date: {e:?}"))?;
    root.draw(&Text::new(
        time_s.to_string(),
        ((w / 2) as i32, (h / 2) as i32 + 30),
        time_style,
    ))
    .map_err(|e| anyhow::anyhow!("card time: {e:?}"))?;
    root.present()
        .map_err(|e| anyhow::anyhow!("card present: {e:?}"))?;
    Ok(())
}
