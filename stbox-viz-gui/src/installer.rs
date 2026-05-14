//! Cross-platform in-app updater. Three flavours of "download the new
//! release artifact, swap it in next to the running binary, and
//! relaunch":
//!
//! - **macOS**: DMG → `hdiutil attach` → codesign-verify → `ditto`
//!   stage → bash helper waits for our PID to die → `mv` swap the
//!   `.app` bundle → `open` relaunch.
//! - **Linux**: tar.gz → shell out to `tar -xzf` → stage both binaries
//!   (`MovementLogger` + `stbox-viz`) → bash helper waits → `mv` swap
//!   both → `exec` relaunch.
//! - **Windows**: zip → PowerShell `Expand-Archive` → stage both .exes
//!   → PowerShell helper waits for our PID via `Get-Process` → `Move-
//!   Item` swap (Windows lets you rename a running exe) → `Start-
//!   Process` relaunch.
//!
//! Why a helper script instead of doing the swap in-process: macOS
//! gets unhappy when you mv a .app over the running one (dyld cache
//! drift → "killed: 9"). Linux + Windows would technically work
//! in-process for a single binary, but using the same helper-after-
//! exit shape across all three keeps the code paths uniform.

use std::fs;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::mpsc::Sender;

#[derive(Clone)]
pub enum InstallEvent {
    Log(String),
    Phase(String),
    DownloadProgress { bytes: u64, total: u64 },
    Done,
    Error(String),
}

/// Walk up from the running executable to find the enclosing .app.
/// macOS-only — returns None when running outside a bundle (e.g.
/// `cargo run`) or on Linux/Windows.
pub fn current_app_bundle() -> Option<PathBuf> {
    let exe = std::env::current_exe().ok()?;
    let mut cur = exe.as_path();
    while let Some(parent) = cur.parent() {
        if parent.extension().and_then(|e| e.to_str()) == Some("app") {
            return Some(parent.to_path_buf());
        }
        cur = parent;
    }
    None
}

/// True when the platform target string maps to a release artifact and
/// we can find the running executable / .app to swap. Drives the
/// "Update now" vs "Open release page" UI choice.
pub fn can_in_app_update() -> bool {
    if crate::update::target_asset_suffix().is_empty() {
        return false;
    }
    if cfg!(target_os = "macos") {
        current_app_bundle().is_some()
    } else {
        std::env::current_exe().is_ok()
    }
}

/// Catch the easy "user installed into a system dir and the parent
/// isn't writable" case before we waste bandwidth on the download.
pub fn check_writable_parent(target: &Path) -> Result<(), String> {
    let parent = target.parent().ok_or_else(|| "no parent directory".to_string())?;
    let probe = parent.join(".movement_logger_write_test");
    fs::File::create(&probe)
        .map_err(|e| format!("cannot write to {}: {}", parent.display(), e))?;
    let _ = fs::remove_file(&probe);
    Ok(())
}

/// Top-level dispatch. The UI layer only calls this; the per-platform
/// branches own their swap strategy.
pub fn install(url: &str, tx: Sender<InstallEvent>) -> Result<(), String> {
    #[cfg(target_os = "macos")]
    {
        let app = current_app_bundle()
            .ok_or_else(|| "could not locate enclosing .app bundle".to_string())?;
        return install_macos(url, &app, tx);
    }
    #[cfg(target_os = "linux")]
    {
        let exe = std::env::current_exe().map_err(|e| format!("current_exe: {e}"))?;
        return install_linux(url, &exe, tx);
    }
    #[cfg(target_os = "windows")]
    {
        let exe = std::env::current_exe().map_err(|e| format!("current_exe: {e}"))?;
        return install_windows(url, &exe, tx);
    }
    #[cfg(not(any(target_os = "macos", target_os = "linux", target_os = "windows")))]
    {
        let _ = (url, tx);
        Err("in-app update is not supported on this platform".into())
    }
}

// =====================================================================
//   macOS — DMG flow
// =====================================================================

#[cfg(target_os = "macos")]
pub fn install_macos(url: &str, current_app: &Path, tx: Sender<InstallEvent>) -> Result<(), String> {
    check_writable_parent(current_app)?;

    let _ = tx.send(InstallEvent::Phase("Downloading".into()));
    let dmg = download_to_temp(url, "MovementLogger.dmg", &tx)?;
    let _ = tx.send(InstallEvent::Log(format!("Downloaded {}", dmg.display())));

    let _ = tx.send(InstallEvent::Phase("Mounting".into()));
    let mount = mount_dmg(&dmg)?;
    let _ = tx.send(InstallEvent::Log(format!("Mounted at {}", mount.display())));

    let result = stage_from_mount(&mount, current_app, &tx);
    let _ = detach_mount(&mount);
    let _ = fs::remove_file(&dmg);

    let staging = result?;
    let _ = tx.send(InstallEvent::Phase("Scheduling install".into()));
    spawn_macos_swap_helper(current_app, &staging)?;
    let _ = tx.send(InstallEvent::Done);
    Ok(())
}

#[cfg(target_os = "macos")]
fn stage_from_mount(mount: &Path, current_app: &Path, tx: &Sender<InstallEvent>) -> Result<PathBuf, String> {
    let new_app = find_first(mount, |p| p.extension().and_then(|s| s.to_str()) == Some("app"))
        .ok_or_else(|| "no .app inside DMG".to_string())?;

    let _ = tx.send(InstallEvent::Phase("Verifying signature".into()));
    let cs = Command::new("codesign")
        .args(["--verify", "--deep", "--strict"])
        .arg(&new_app)
        .output()
        .map_err(|e| format!("codesign: {}", e))?;
    if !cs.status.success() {
        return Err(format!(
            "codesign verification failed: {}",
            String::from_utf8_lossy(&cs.stderr).trim()
        ));
    }

    let parent = current_app.parent().ok_or("no parent for current .app")?;
    let app_name = current_app.file_name().and_then(|s| s.to_str()).unwrap_or("MovementLogger.app");
    let staging = parent.join(format!(".{}.new.{}", app_name, std::process::id()));
    let _ = fs::remove_dir_all(&staging);

    let _ = tx.send(InstallEvent::Phase("Staging".into()));
    let ditto = Command::new("ditto").arg(&new_app).arg(&staging).status()
        .map_err(|e| format!("ditto: {}", e))?;
    if !ditto.success() {
        return Err(format!("ditto exited with {:?}", ditto.code()));
    }
    Ok(staging)
}

#[cfg(target_os = "macos")]
fn mount_dmg(dmg: &Path) -> Result<PathBuf, String> {
    let mount = std::env::temp_dir().join(format!("movement_logger_mount_{}", std::process::id()));
    let _ = fs::create_dir_all(&mount);
    let out = Command::new("hdiutil")
        .args(["attach", "-nobrowse", "-readonly", "-mountpoint"])
        .arg(&mount)
        .arg(dmg)
        .output()
        .map_err(|e| format!("hdiutil: {}", e))?;
    if !out.status.success() {
        return Err(format!(
            "hdiutil attach failed: {}",
            String::from_utf8_lossy(&out.stderr).trim()
        ));
    }
    Ok(mount)
}

#[cfg(target_os = "macos")]
fn detach_mount(mount: &Path) -> Result<(), String> {
    let out = Command::new("hdiutil").args(["detach", "-quiet"]).arg(mount).output()
        .map_err(|e| format!("hdiutil detach: {}", e))?;
    if !out.status.success() {
        let _ = Command::new("hdiutil").args(["detach", "-force", "-quiet"]).arg(mount).status();
    }
    let _ = fs::remove_dir(mount);
    Ok(())
}

#[cfg(target_os = "macos")]
fn spawn_macos_swap_helper(current_app: &Path, staging: &Path) -> Result<(), String> {
    let pid = std::process::id();
    let helper = std::env::temp_dir().join(format!("movement_logger_install_{}.sh", pid));
    let log = std::env::temp_dir().join(format!("movement_logger_install_{}.log", pid));
    let bak = current_app.with_file_name(format!(
        "{}.bak.{}",
        current_app.file_name().and_then(|s| s.to_str()).unwrap_or("MovementLogger.app"),
        pid
    ));

    let script = format!(
        "#!/bin/bash\n\
         set -u\n\
         exec >{log_q} 2>&1\n\
         echo waiting for parent {pid}\n\
         for i in $(seq 1 100); do\n\
           if ! kill -0 {pid} 2>/dev/null; then break; fi\n\
           sleep 0.1\n\
         done\n\
         echo swapping bundle\n\
         rm -rf {bak_q}\n\
         mv {cur_q} {bak_q} && mv {new_q} {cur_q}\n\
         rc=$?\n\
         if [ $rc -ne 0 ]; then\n\
           echo swap failed: rc=$rc\n\
           if [ -d {bak_q} ] && [ ! -d {cur_q} ]; then mv {bak_q} {cur_q}; fi\n\
           exit $rc\n\
         fi\n\
         rm -rf {bak_q}\n\
         echo relaunching\n\
         /usr/bin/open {cur_q}\n\
         rm -- \"$0\"\n",
        log_q = shell_quote(&log),
        pid = pid,
        cur_q = shell_quote(current_app),
        new_q = shell_quote(staging),
        bak_q = shell_quote(&bak),
    );

    write_executable(&helper, &script)?;
    Command::new("/bin/bash")
        .arg(&helper)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .map_err(|e| format!("spawn helper: {}", e))?;
    Ok(())
}

// =====================================================================
//   Linux — tar.gz flow
// =====================================================================

#[cfg(target_os = "linux")]
pub fn install_linux(url: &str, current_exe: &Path, tx: Sender<InstallEvent>) -> Result<(), String> {
    check_writable_parent(current_exe)?;

    let _ = tx.send(InstallEvent::Phase("Downloading".into()));
    let tgz = download_to_temp(url, "MovementLogger.tar.gz", &tx)?;
    let _ = tx.send(InstallEvent::Log(format!("Downloaded {}", tgz.display())));

    let _ = tx.send(InstallEvent::Phase("Extracting".into()));
    let extract_dir = std::env::temp_dir()
        .join(format!("movement_logger_extract_{}", std::process::id()));
    let _ = fs::remove_dir_all(&extract_dir);
    fs::create_dir_all(&extract_dir).map_err(|e| format!("mkdir extract: {e}"))?;
    let out = Command::new("tar")
        .args(["-xzf"])
        .arg(&tgz)
        .arg("-C")
        .arg(&extract_dir)
        .output()
        .map_err(|e| format!("tar: {e}"))?;
    if !out.status.success() {
        return Err(format!(
            "tar -xzf failed: {}",
            String::from_utf8_lossy(&out.stderr).trim()
        ));
    }
    let _ = fs::remove_file(&tgz);

    // The release tarball contains a single top-level dir with both
    // binaries + the desktop file + icon. Walk one level deep to find
    // the MovementLogger binary; stbox-viz lives next to it.
    let new_main = find_recursive(&extract_dir, "MovementLogger")
        .ok_or_else(|| "no MovementLogger binary in tarball".to_string())?;
    let new_sidecar = new_main.parent().map(|d| d.join("stbox-viz"))
        .filter(|p| p.is_file());

    let _ = tx.send(InstallEvent::Phase("Staging".into()));
    let pid = std::process::id();
    let parent = current_exe.parent().ok_or("no parent for current exe")?;
    let main_name = current_exe.file_name().and_then(|s| s.to_str()).unwrap_or("MovementLogger");
    let main_staging = parent.join(format!(".{main_name}.new.{pid}"));
    let _ = fs::remove_file(&main_staging);
    fs::copy(&new_main, &main_staging).map_err(|e| format!("stage main: {e}"))?;
    set_executable(&main_staging)?;

    let sidecar_pair = if let Some(new_sc) = new_sidecar {
        let cur_sc = parent.join("stbox-viz");
        if cur_sc.is_file() {
            let stage_sc = parent.join(format!(".stbox-viz.new.{pid}"));
            let _ = fs::remove_file(&stage_sc);
            fs::copy(&new_sc, &stage_sc).map_err(|e| format!("stage sidecar: {e}"))?;
            set_executable(&stage_sc)?;
            Some((cur_sc, stage_sc))
        } else {
            None
        }
    } else {
        None
    };

    let _ = fs::remove_dir_all(&extract_dir);

    let _ = tx.send(InstallEvent::Phase("Scheduling install".into()));
    spawn_linux_swap_helper(current_exe, &main_staging, sidecar_pair.as_ref())?;
    let _ = tx.send(InstallEvent::Done);
    Ok(())
}

#[cfg(target_os = "linux")]
fn spawn_linux_swap_helper(
    current_exe: &Path,
    main_staging: &Path,
    sidecar: Option<&(PathBuf, PathBuf)>,
) -> Result<(), String> {
    let pid = std::process::id();
    let helper = std::env::temp_dir().join(format!("movement_logger_install_{}.sh", pid));
    let log = std::env::temp_dir().join(format!("movement_logger_install_{}.log", pid));
    let main_bak = current_exe.with_file_name(format!(
        "{}.bak.{}",
        current_exe.file_name().and_then(|s| s.to_str()).unwrap_or("MovementLogger"),
        pid
    ));

    let sidecar_block = if let Some((cur_sc, stage_sc)) = sidecar {
        let sc_bak = cur_sc.with_file_name(format!("stbox-viz.bak.{pid}"));
        format!(
            "rm -f {sc_bak_q}\n\
             mv {cur_sc_q} {sc_bak_q} && mv {stage_sc_q} {cur_sc_q}\n\
             chmod +x {cur_sc_q} 2>/dev/null || true\n\
             rm -f {sc_bak_q}\n",
            sc_bak_q = shell_quote(&sc_bak),
            cur_sc_q = shell_quote(cur_sc),
            stage_sc_q = shell_quote(stage_sc),
        )
    } else {
        String::new()
    };

    let script = format!(
        "#!/bin/bash\n\
         set -u\n\
         exec >{log_q} 2>&1\n\
         echo waiting for parent {pid}\n\
         for i in $(seq 1 100); do\n\
           if ! kill -0 {pid} 2>/dev/null; then break; fi\n\
           sleep 0.1\n\
         done\n\
         echo swapping binary\n\
         rm -f {main_bak_q}\n\
         mv {cur_q} {main_bak_q} && mv {new_q} {cur_q}\n\
         rc=$?\n\
         if [ $rc -ne 0 ]; then\n\
           echo swap failed: rc=$rc\n\
           if [ -f {main_bak_q} ] && [ ! -f {cur_q} ]; then mv {main_bak_q} {cur_q}; fi\n\
           exit $rc\n\
         fi\n\
         chmod +x {cur_q} 2>/dev/null || true\n\
         rm -f {main_bak_q}\n\
         {sidecar}\
         echo relaunching\n\
         setsid -f {cur_q} </dev/null >/dev/null 2>&1\n\
         rm -- \"$0\"\n",
        log_q = shell_quote(&log),
        pid = pid,
        cur_q = shell_quote(current_exe),
        new_q = shell_quote(main_staging),
        main_bak_q = shell_quote(&main_bak),
        sidecar = sidecar_block,
    );

    write_executable(&helper, &script)?;
    Command::new("/bin/bash")
        .arg(&helper)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .map_err(|e| format!("spawn helper: {}", e))?;
    Ok(())
}

// =====================================================================
//   Windows — zip flow
// =====================================================================

#[cfg(target_os = "windows")]
pub fn install_windows(url: &str, current_exe: &Path, tx: Sender<InstallEvent>) -> Result<(), String> {
    check_writable_parent(current_exe)?;

    let _ = tx.send(InstallEvent::Phase("Downloading".into()));
    let zip = download_to_temp(url, "MovementLogger.zip", &tx)?;
    let _ = tx.send(InstallEvent::Log(format!("Downloaded {}", zip.display())));

    let _ = tx.send(InstallEvent::Phase("Extracting".into()));
    let extract_dir = std::env::temp_dir()
        .join(format!("movement_logger_extract_{}", std::process::id()));
    let _ = fs::remove_dir_all(&extract_dir);
    fs::create_dir_all(&extract_dir).map_err(|e| format!("mkdir extract: {e}"))?;
    let ps = format!(
        "Expand-Archive -LiteralPath '{}' -DestinationPath '{}' -Force",
        zip.display(),
        extract_dir.display()
    );
    let out = Command::new("powershell")
        .args(["-NoProfile", "-ExecutionPolicy", "Bypass", "-Command"])
        .arg(&ps)
        .output()
        .map_err(|e| format!("powershell Expand-Archive: {e}"))?;
    if !out.status.success() {
        return Err(format!(
            "Expand-Archive failed: {}",
            String::from_utf8_lossy(&out.stderr).trim()
        ));
    }
    let _ = fs::remove_file(&zip);

    let new_main = find_recursive(&extract_dir, "MovementLogger.exe")
        .ok_or_else(|| "no MovementLogger.exe in zip".to_string())?;
    let new_sidecar = new_main.parent().map(|d| d.join("stbox-viz.exe"))
        .filter(|p| p.is_file());

    let _ = tx.send(InstallEvent::Phase("Staging".into()));
    let pid = std::process::id();
    let parent = current_exe.parent().ok_or("no parent for current exe")?;
    let main_name = current_exe.file_name().and_then(|s| s.to_str()).unwrap_or("MovementLogger.exe");
    let main_staging = parent.join(format!("{main_name}.new.{pid}"));
    let _ = fs::remove_file(&main_staging);
    fs::copy(&new_main, &main_staging).map_err(|e| format!("stage main: {e}"))?;

    let sidecar_pair = if let Some(new_sc) = new_sidecar {
        let cur_sc = parent.join("stbox-viz.exe");
        if cur_sc.is_file() {
            let stage_sc = parent.join(format!("stbox-viz.exe.new.{pid}"));
            let _ = fs::remove_file(&stage_sc);
            fs::copy(&new_sc, &stage_sc).map_err(|e| format!("stage sidecar: {e}"))?;
            Some((cur_sc, stage_sc))
        } else {
            None
        }
    } else {
        None
    };

    let _ = fs::remove_dir_all(&extract_dir);

    let _ = tx.send(InstallEvent::Phase("Scheduling install".into()));
    spawn_windows_swap_helper(current_exe, &main_staging, sidecar_pair.as_ref())?;
    let _ = tx.send(InstallEvent::Done);
    Ok(())
}

#[cfg(target_os = "windows")]
fn spawn_windows_swap_helper(
    current_exe: &Path,
    main_staging: &Path,
    sidecar: Option<&(PathBuf, PathBuf)>,
) -> Result<(), String> {
    let pid = std::process::id();
    let helper = std::env::temp_dir().join(format!("movement_logger_install_{}.ps1", pid));
    let log = std::env::temp_dir().join(format!("movement_logger_install_{}.log", pid));

    let sidecar_block = if let Some((cur_sc, stage_sc)) = sidecar {
        format!(
            "$scOld = '{}.old'\n\
             if (Test-Path $scOld) {{ Remove-Item -LiteralPath $scOld -Force -ErrorAction SilentlyContinue }}\n\
             Move-Item -LiteralPath '{}' -Destination $scOld -Force\n\
             Move-Item -LiteralPath '{}' -Destination '{}' -Force\n\
             Remove-Item -LiteralPath $scOld -Force -ErrorAction SilentlyContinue\n",
            cur_sc.display(),
            cur_sc.display(),
            stage_sc.display(),
            cur_sc.display(),
        )
    } else {
        String::new()
    };

    // Windows lets you rename a running .exe (the open handle keeps the
    // old name's inode alive), but not delete it. So: rename current →
    // .old, move new → current, then delete .old after the process exits.
    let script = format!(
        "$ErrorActionPreference = 'Continue'\n\
         Start-Transcript -Path '{log}' -Force | Out-Null\n\
         Write-Host \"waiting for parent {pid}\"\n\
         for ($i = 0; $i -lt 200; $i++) {{\n\
           if (-not (Get-Process -Id {pid} -ErrorAction SilentlyContinue)) {{ break }}\n\
           Start-Sleep -Milliseconds 100\n\
         }}\n\
         Write-Host 'swapping binary'\n\
         $mainOld = '{cur}.old'\n\
         if (Test-Path $mainOld) {{ Remove-Item -LiteralPath $mainOld -Force -ErrorAction SilentlyContinue }}\n\
         Move-Item -LiteralPath '{cur}' -Destination $mainOld -Force\n\
         Move-Item -LiteralPath '{new}' -Destination '{cur}' -Force\n\
         Remove-Item -LiteralPath $mainOld -Force -ErrorAction SilentlyContinue\n\
         {sidecar}\
         Write-Host 'relaunching'\n\
         Start-Process -FilePath '{cur}'\n\
         Stop-Transcript | Out-Null\n\
         Remove-Item -LiteralPath $MyInvocation.MyCommand.Path -Force -ErrorAction SilentlyContinue\n",
        log = log.display(),
        pid = pid,
        cur = current_exe.display(),
        new = main_staging.display(),
        sidecar = sidecar_block,
    );

    fs::write(&helper, script).map_err(|e| format!("write helper: {e}"))?;

    Command::new("powershell")
        .args(["-NoProfile", "-ExecutionPolicy", "Bypass", "-WindowStyle", "Hidden", "-File"])
        .arg(&helper)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .map_err(|e| format!("spawn helper: {e}"))?;
    Ok(())
}

// =====================================================================
//   Shared helpers
// =====================================================================

fn download_to_temp(url: &str, filename: &str, tx: &Sender<InstallEvent>) -> Result<PathBuf, String> {
    let client = reqwest::blocking::Client::builder()
        .user_agent("movement-logger-installer")
        .timeout(None)
        .build()
        .map_err(|e| format!("client: {}", e))?;
    let mut resp = client.get(url).send().map_err(|e| format!("GET: {}", e))?;
    if !resp.status().is_success() {
        return Err(format!("download failed: HTTP {}", resp.status()));
    }
    let total = resp.content_length().unwrap_or(0);
    let dir = std::env::temp_dir().join(format!("movement_logger_update_{}", std::process::id()));
    fs::create_dir_all(&dir).map_err(|e| format!("mkdir tmp: {}", e))?;
    let path = dir.join(filename);
    let mut file = fs::File::create(&path).map_err(|e| format!("create dl: {}", e))?;

    let mut buf = vec![0u8; 64 * 1024];
    let mut written: u64 = 0;
    let mut last_emit: u64 = 0;
    loop {
        let n = resp.read(&mut buf).map_err(|e| format!("read: {}", e))?;
        if n == 0 { break; }
        file.write_all(&buf[..n]).map_err(|e| format!("write: {}", e))?;
        written += n as u64;
        if written - last_emit >= 256 * 1024 || (total > 0 && written == total) {
            let _ = tx.send(InstallEvent::DownloadProgress { bytes: written, total });
            last_emit = written;
        }
    }
    Ok(path)
}

#[cfg(any(target_os = "macos"))]
fn find_first(dir: &Path, pred: impl Fn(&Path) -> bool) -> Option<PathBuf> {
    for entry in fs::read_dir(dir).ok()?.flatten() {
        let p = entry.path();
        if pred(&p) { return Some(p); }
    }
    None
}

/// Walk `dir` recursively (depth-first), returning the first regular
/// file whose name exactly matches `target_name`. Used to find
/// `MovementLogger` / `MovementLogger.exe` inside the extracted
/// archive's top-level stem folder without hardcoding the stem.
#[cfg(any(target_os = "linux", target_os = "windows"))]
fn find_recursive(dir: &Path, target_name: &str) -> Option<PathBuf> {
    let mut stack = vec![dir.to_path_buf()];
    while let Some(d) = stack.pop() {
        let Ok(rd) = fs::read_dir(&d) else { continue };
        for entry in rd.flatten() {
            let p = entry.path();
            if p.is_dir() {
                stack.push(p);
            } else if p.file_name().and_then(|s| s.to_str()) == Some(target_name) {
                return Some(p);
            }
        }
    }
    None
}

#[cfg(unix)]
fn set_executable(path: &Path) -> Result<(), String> {
    use std::os::unix::fs::PermissionsExt;
    let mut perms = fs::metadata(path).map_err(|e| format!("perms: {e}"))?.permissions();
    perms.set_mode(0o755);
    fs::set_permissions(path, perms).map_err(|e| format!("chmod: {e}"))
}

#[cfg(not(unix))]
#[allow(dead_code)]
fn set_executable(_: &Path) -> Result<(), String> {
    Ok(())
}

#[cfg(unix)]
fn write_executable(path: &Path, contents: &str) -> Result<(), String> {
    fs::write(path, contents).map_err(|e| format!("write helper: {e}"))?;
    set_executable(path)
}

#[cfg(unix)]
fn shell_quote(p: &Path) -> String {
    let s = p.to_string_lossy();
    format!("'{}'", s.replace('\'', "'\\''"))
}
