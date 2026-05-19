//! Per-OS login item for the background sync agent.
//!
//! Issue #14 part B step 7. Coupled to the box log mode: AUTO →
//! `register()` (agent starts at login and mirrors zero-click),
//! MANUAL → `unregister()`. `sync_with_mode` is the single entry the
//! engine calls whenever GET_MODE / SET_MODE resolves.
//!
//! Registration is idempotent and best-effort: a failure to write the
//! login item must never break BLE/sync, so callers ignore the result
//! (it's logged by the caller if they want).

use std::io;
use std::path::PathBuf;

// Used by the macOS launchd plist; dead on Linux/Windows builds.
#[allow(dead_code)]
const LABEL: &str = "com.ywesee.movementlogger.agent";

/// What the login item should launch. On macOS this is the executable
/// **inside the signed `.app`** (`…/Contents/MacOS/MovementLogger`),
/// not a bare copy — Bluetooth TCC is keyed to the bundle's code
/// signature, so launchd must start that exact Mach-O for the grant
/// the GUI obtained to apply. Outside a bundle (dev) / on other OSes
/// it's just `current_exe()`.
fn launch_exe() -> io::Result<PathBuf> {
    #[cfg(target_os = "macos")]
    {
        if let Some(app) = crate::installer::current_app_bundle() {
            return Ok(app.join("Contents/MacOS/MovementLogger"));
        }
    }
    std::env::current_exe()
}

/// Idempotently make the login item match the box log mode.
/// `manual == false` (AUTO) → install; `true` (MANUAL) → remove.
pub fn sync_with_mode(manual: bool) -> io::Result<()> {
    if manual {
        unregister()
    } else {
        register()
    }
}

// ----------------------------- macOS -----------------------------------

#[cfg(target_os = "macos")]
fn plist_path() -> io::Result<PathBuf> {
    let home = std::env::var_os("HOME")
        .map(PathBuf::from)
        .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "no HOME"))?;
    Ok(home
        .join("Library/LaunchAgents")
        .join(format!("{LABEL}.plist")))
}

#[cfg(target_os = "macos")]
pub fn register() -> io::Result<()> {
    let exe = launch_exe()?;
    let path = plist_path()?;
    if let Some(d) = path.parent() {
        std::fs::create_dir_all(d)?;
    }
    // KeepAlive=false + RunAtLoad=true: launchd starts it at login but
    // does NOT respawn it when it exits cleanly on a GUI handover —
    // the agent's own loop handles waiting/resuming, so a tight
    // launchd respawn would fight our coordination.
    let plist = format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>Label</key><string>{LABEL}</string>
  <key>ProgramArguments</key>
  <array><string>{exe}</string><string>--agent</string></array>
  <key>RunAtLoad</key><true/>
  <key>KeepAlive</key><false/>
  <key>ProcessType</key><string>Background</string>
</dict>
</plist>
"#,
        exe = exe.display()
    );
    std::fs::write(&path, plist)?;
    // Best-effort load so it starts now without a re-login. Ignore
    // failure (e.g. already loaded) — RunAtLoad covers next login.
    let uid = unsafe { libc_getuid() };
    let _ = std::process::Command::new("launchctl")
        .args(["bootout", &format!("gui/{uid}/{LABEL}")])
        .status();
    let _ = std::process::Command::new("launchctl")
        .args(["bootstrap", &format!("gui/{uid}"), &path.to_string_lossy()])
        .status();
    Ok(())
}

#[cfg(target_os = "macos")]
pub fn unregister() -> io::Result<()> {
    let path = plist_path()?;
    let uid = unsafe { libc_getuid() };
    let _ = std::process::Command::new("launchctl")
        .args(["bootout", &format!("gui/{uid}/{LABEL}")])
        .status();
    if path.exists() {
        std::fs::remove_file(&path)?;
    }
    Ok(())
}

#[cfg(target_os = "macos")]
extern "C" {
    #[link_name = "getuid"]
    fn libc_getuid() -> u32;
}

// ----------------------------- Linux -----------------------------------

#[cfg(all(unix, not(target_os = "macos")))]
fn desktop_path() -> io::Result<PathBuf> {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
        .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "no HOME/XDG_CONFIG_HOME"))?;
    Ok(base.join("autostart").join("movementlogger-agent.desktop"))
}

#[cfg(all(unix, not(target_os = "macos")))]
pub fn register() -> io::Result<()> {
    let exe = launch_exe()?;
    let path = desktop_path()?;
    if let Some(d) = path.parent() {
        std::fs::create_dir_all(d)?;
    }
    let entry = format!(
        "[Desktop Entry]\n\
         Type=Application\n\
         Name=MovementLogger Sync Agent\n\
         Comment=Zero-click background sync of the SensorTile.box\n\
         Exec={exe} --agent\n\
         X-GNOME-Autostart-enabled=true\n\
         Hidden=false\n\
         NoDisplay=true\n\
         Terminal=false\n",
        exe = exe.display()
    );
    std::fs::write(&path, entry)?;
    Ok(())
}

#[cfg(all(unix, not(target_os = "macos")))]
pub fn unregister() -> io::Result<()> {
    let path = desktop_path()?;
    if path.exists() {
        std::fs::remove_file(&path)?;
    }
    Ok(())
}

// ---------------------------- Windows ----------------------------------

#[cfg(windows)]
const RUN_KEY: &str = r"Software\Microsoft\Windows\CurrentVersion\Run";
#[cfg(windows)]
const RUN_VALUE: &str = "MovementLoggerAgent";

#[cfg(windows)]
pub fn register() -> io::Result<()> {
    use winreg::enums::HKEY_CURRENT_USER;
    use winreg::RegKey;
    let exe = launch_exe()?;
    let hkcu = RegKey::predef(HKEY_CURRENT_USER);
    let (run, _) = hkcu.create_subkey(RUN_KEY)?;
    run.set_value(RUN_VALUE, &format!("\"{}\" --agent", exe.display()))?;
    Ok(())
}

#[cfg(windows)]
pub fn unregister() -> io::Result<()> {
    use winreg::enums::HKEY_CURRENT_USER;
    use winreg::RegKey;
    let hkcu = RegKey::predef(HKEY_CURRENT_USER);
    if let Ok(run) = hkcu.open_subkey_with_flags(RUN_KEY, winreg::enums::KEY_ALL_ACCESS) {
        // Missing value is fine (already unregistered).
        let _ = run.delete_value(RUN_VALUE);
    }
    Ok(())
}
