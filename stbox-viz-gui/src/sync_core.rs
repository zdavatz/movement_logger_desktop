//! UI-independent sync core.
//!
//! Step 1 (issue #14 part B): the pure, egui-free helpers the desktop
//! sync engine is built on, lifted verbatim out of `main.rs` so the
//! upcoming headless `--agent` can reuse the exact same mirror /
//! save-dir / name-classification logic the GUI uses. No behaviour
//! change — these moved without edits. Later steps add `SyncCore`
//! (the driver state + methods) here too.

use std::path::{Path, PathBuf};

/// Live-mirror download assembly.
///
/// The local file `<dir>/<name>` *is* the running mirror — we
/// accumulate straight into it, no `.part`/rename. The box's log files
/// grow continuously while a session runs, so "done" is a moving
/// target; what matters is that the local file is always a valid
/// prefix of the box file (the logs are append-only) and we only ever
/// fetch the bytes we don't have yet. This makes the local size the
/// single source of truth for the resume/grow offset — survives an
/// app restart, a dropped link, and "the file got bigger since last
/// sync" identically.
pub fn mirror_path(dir: &Path, name: &str) -> PathBuf {
    dir.join(name)
}

/// Where to resume `name` from given the box's current size.
/// - local missing            → 0 (fresh)
/// - local smaller than box   → local size (resume / fetch the delta)
/// - local == box             → equal (caller treats as up-to-date)
/// - local larger than box    → the file rotated (session reused the
///   name, box file is now shorter) → truncate local, restart at 0
pub fn mirror_offset(dir: &Path, name: &str, box_size: u64) -> u64 {
    let p = mirror_path(dir, name);
    match std::fs::metadata(&p) {
        Ok(m) if m.len() <= box_size => m.len(),
        Ok(_) => {
            // Rotated/shorter on the box — old local copy is stale.
            let _ = std::fs::remove_file(&p);
            0
        }
        Err(_) => 0,
    }
}

/// Append a streamed segment to `<dir>/<name>` (creating it). `base`
/// is the offset the segment started at; if it doesn't match the
/// current file length the resume is misaligned (offset is derived
/// from this same file's length before the READ) — realign to `base`
/// so we never interleave a corrupt prefix.
pub fn append_mirror(dir: &Path, name: &str, base: u64, bytes: &[u8]) -> std::io::Result<u64> {
    use std::io::{Seek, SeekFrom, Write};
    std::fs::create_dir_all(dir)?;
    let p = mirror_path(dir, name);
    let mut f = std::fs::OpenOptions::new()
        .create(true)
        .read(true)
        .write(true)
        .open(&p)?;
    let cur = f.metadata()?.len();
    if cur != base {
        f.set_len(base.min(cur))?;
    }
    f.seek(SeekFrom::End(0))?;
    f.write_all(bytes)?;
    f.flush()?;
    Ok(f.metadata()?.len())
}

/// Base dir for the default `csv/` (downloads) and `gif/` (renders)
/// folders. A macOS `.app` launched from Finder/Dock starts with
/// `current_dir()` == `/` — a read-only volume — so the historical
/// `cwd.join("csv")` became `/csv` and every download died with
/// `Can't create "/csv": Read-only file system (os error 30)`
/// (Peter, v0.0.20 on Mac). Use cwd only when it's a real writable
/// working dir (the Linux/Windows `./MovementLogger` case — behaviour
/// unchanged there); otherwise anchor under `$HOME/MovementLogger`,
/// same `$HOME` rationale as the sync DB. The user can still override
/// the folder in the Sync tab.
pub fn default_save_base() -> PathBuf {
    if let Ok(cwd) = std::env::current_dir() {
        if cwd != Path::new("/") && probe_save_dir(&cwd).is_ok() {
            return cwd;
        }
    }
    std::env::var_os("HOME")
        .or_else(|| std::env::var_os("USERPROFILE"))
        .map(PathBuf::from)
        .map(|h| h.join("MovementLogger"))
        .unwrap_or_else(|| PathBuf::from("."))
}

/// Expand a leading `~` and require an absolute path. Pure path math,
/// no filesystem access — cheap enough to call every frame for the
/// live indicator. The absolute-path requirement is the core fix for
/// the silent-misplace bug: `PathBuf::from("~/Downloads")` or a bare
/// relative path is otherwise created *relative to the process cwd*,
/// so the app reports `saved <path>` while the file is nowhere the
/// user is looking.
pub fn resolve_save_dir(raw: &Path) -> Result<PathBuf, String> {
    let s = raw.to_string_lossy();
    let s = s.trim();
    if s.is_empty() {
        return Err("Save-to path is empty — pick a folder with Browse….".into());
    }
    let expanded: PathBuf = if s == "~" || s.starts_with("~/") {
        let home = std::env::var_os("HOME")
            .or_else(|| std::env::var_os("USERPROFILE"))
            .map(PathBuf::from)
            .ok_or("can't expand ~ — no HOME / USERPROFILE set")?;
        if s == "~" { home } else { home.join(&s[2..]) }
    } else {
        PathBuf::from(s)
    };
    if !expanded.is_absolute() {
        return Err(format!(
            "\u{201c}{}\u{201d} is not an absolute path. Use a full path \
             like /Users/you/Downloads (Browse… fills one in).",
            expanded.display()
        ));
    }
    Ok(expanded)
}

/// Authoritative writability check: actually create the directory and
/// write+delete a probe file. Called at Download/Sync time (not every
/// frame). Surfaces macOS TCC / permission denials that the path-only
/// check can't see — exactly the failures that used to look like a
/// successful save.
pub fn probe_save_dir(dir: &Path) -> Result<(), String> {
    std::fs::create_dir_all(dir)
        .map_err(|e| format!("Can't create \u{201c}{}\u{201d}: {e}", dir.display()))?;
    let probe = dir.join(".movementlogger_write_test");
    match std::fs::write(&probe, b"ok") {
        Ok(()) => {
            let _ = std::fs::remove_file(&probe);
            Ok(())
        }
        Err(e) => {
            let hint = if e.kind() == std::io::ErrorKind::PermissionDenied {
                " — macOS may be blocking it (TCC): grant the app access under \
                 System Settings → Privacy & Security → Files and Folders / \
                 Full Disk Access, or pick a folder you own."
            } else {
                ""
            };
            Err(format!(
                "Folder \u{201c}{}\u{201d} is not writable: {e}{hint}",
                dir.display()
            ))
        }
    }
}

/// True for the four sensor-data row types the firmware creates per
/// logging session (Sens*.csv, Gps*.csv, Bat*.csv, Mic*.wav). Used both
/// for the default-ticked state in the file list and for splitting the
/// list into the Sensor / Debug groups. macOS AppleDouble sidecars
/// (`._<name>`) match the inner pattern by accident, so guard the
/// prefix explicitly.
pub fn is_sensor_data_name(name: &str) -> bool {
    let lower = name.to_ascii_lowercase();
    if lower.starts_with("._") { return false; }
    (lower.starts_with("sens") && lower.ends_with(".csv"))
        || (lower.starts_with("gps")  && lower.ends_with(".csv"))
        || (lower.starts_with("bat")  && lower.ends_with(".csv"))
        || (lower.starts_with("mic")  && lower.ends_with(".wav"))
}

/// Files the *Sync* pass mirrors: every per-session sensor-data file
/// (`is_sensor_data_name`) **plus** the firmware's single rolling error
/// log `ERRLOG.LOG` (`movement_logger_firmware/Src/errlog.c`). Peter
/// needs the errlog to debug box-side stalls, and it's append-only just
/// like the session logs, so the same live-mirror byte-resume path
/// applies unchanged. Deliberately kept separate from
/// `is_sensor_data_name` so the manual file list's default-tick and the
/// Sensor/Debug split are untouched — `ERRLOG.LOG` still lands in the
/// Debug group and stays un-ticked there; only *Sync now* / Keep-synced
/// pull it. Match is case-insensitive (firmware writes it upper-case;
/// macOS/FAT may surface either case).
pub fn is_synced_name(name: &str) -> bool {
    is_sensor_data_name(name) || name.eq_ignore_ascii_case("ERRLOG.LOG")
}
