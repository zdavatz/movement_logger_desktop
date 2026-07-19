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
//! All pieces are normalized to a common canvas (height 900, width =
//! widest segment) and concatenated into a single H.264 .mov.
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

    // Intro: "MovementLogger" in the logo's colors on black, before the
    // first date card.
    if args.intro_seconds > 0.0 {
        let intro_png = work.join("intro.png");
        draw_intro(&intro_png, canvas_w, 900)?;
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

    // Outro: the logo centered on black for logo_seconds.
    if args.logo_seconds > 0.0 {
        let logo_png = match args.logo {
            Some(p) => p.to_path_buf(),
            None => {
                let p = work.join("logo.png");
                std::fs::write(&p, LOGO_PNG)?;
                p
            }
        };
        let outro = work.join("outro.mp4");
        let lavfi = format!("color=black:s={}x900:d={}:r=30", canvas_w, args.logo_seconds);
        let filter =
            "[1:v]scale=-1:420[lg];[0:v][lg]overlay=(W-w)/2:(H-h)/2[v]".to_string();
        ffmpeg(&[
            "-y", "-f", "lavfi", "-i", &lavfi,
            "-i", logo_png.to_str().unwrap(),
            "-f", "lavfi", "-i", "anullsrc=r=48000:cl=stereo",
            "-filter_complex", &filter,
            "-map", "[v]", "-map", "2:a", "-shortest",
            "-t", &args.logo_seconds.to_string(),
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
            "-c:a", "aac",
            outro.to_str().unwrap(),
        ])?;
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

/// Black intro card: "MovementLogger" centered, each letter colored along
/// the logo gradient. Letters are laid out by per-glyph measured widths
/// (no kerning — close enough for a title card), sized to ~86% of the
/// canvas width.
fn draw_intro(path: &Path, w: u32, h: u32) -> Result<()> {
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
        let color = intro_color(i as f64 / (n - 1) as f64);
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
