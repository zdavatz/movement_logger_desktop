//! GUI ↔ headless-agent coordination via advisory file locks.
//!
//! Issue #14 part B step 5. The GUI and the `--agent` process must
//! never drive the same BLE adapter / mirror folder at once. Three
//! locks under `~/.movementlogger/`:
//!
//! - `gui.lock`  — held by the GUI for its whole lifetime. The agent
//!   *tries* to take it every poll: if it can't, a GUI is alive and
//!   the agent yields the adapter. The OS drops the lock when the GUI
//!   process exits (even on a crash), so there's no stale-PID problem.
//! - `ble.lock`  — the BLE-adapter ownership token. Whoever holds it
//!   may scan/connect. The GUI takes it on first BLE use and holds it;
//!   the agent only takes it while no GUI is present.
//! - `agent.lock`— single-instance guard for the agent itself.
//!
//! Advisory locks are released automatically by the OS on process
//! death, which is exactly the crash-safety we want for adapter
//! ownership.

use std::fs::{File, OpenOptions};
use std::io;
use std::path::PathBuf;

// `File::{lock,try_lock,unlock}` are inherent in std since Rust 1.89;
// on older toolchains they come from this trait instead. Importing it
// is a no-op when std already provides them (hence the allow), but
// keeps the build green on the pre-1.89 fallback path.
#[allow(unused_imports)]
use fs4::FileExt;

use crate::agent_config::movementlogger_home;

pub const GUI_LOCK: &str = "gui.lock";
pub const BLE_LOCK: &str = "ble.lock";
/// Single-instance guard for the agent (wired in step 6).
#[allow(dead_code)]
pub const AGENT_LOCK: &str = "agent.lock";

fn lock_path(name: &str) -> PathBuf {
    movementlogger_home().join(name)
}

/// An exclusive advisory lock held for as long as this guard lives.
/// Dropping it unlocks; the lock file itself is left in place (cheap,
/// and avoids an unlink/recreate race with a waiter).
pub struct LockGuard {
    file: File,
    #[allow(dead_code)]
    name: &'static str,
}

impl Drop for LockGuard {
    fn drop(&mut self) {
        let _ = self.file.unlock();
    }
}

fn open_lock_file(name: &str) -> io::Result<File> {
    let path = lock_path(name);
    if let Some(d) = path.parent() {
        std::fs::create_dir_all(d)?;
    }
    OpenOptions::new()
        .create(true)
        .read(true)
        .write(true)
        .open(&path)
}

/// Try to take `<name>` without blocking. `None` = another process
/// holds it, or any io error (caller treats both as "not mine").
pub fn try_acquire(name: &'static str) -> Option<LockGuard> {
    let file = open_lock_file(name).ok()?;
    match file.try_lock() {
        Ok(()) => Some(LockGuard { file, name }),
        Err(_) => None,
    }
}

/// Block until `<name>` is taken. Used by the GUI, which must win the
/// adapter — it waits for the agent to notice `gui.lock` is contended
/// and release `ble.lock`.
#[allow(dead_code)]
pub fn acquire_blocking(name: &'static str) -> io::Result<LockGuard> {
    let file = open_lock_file(name)?;
    file.lock()?;
    Ok(LockGuard { file, name })
}

/// True iff some process currently holds `<name>` (i.e. a non-blocking
/// acquire would fail). Used by the agent to detect a live GUI.
#[allow(dead_code)]
pub fn is_held(name: &'static str) -> bool {
    match open_lock_file(name) {
        Ok(f) => match f.try_lock() {
            Ok(()) => {
                let _ = f.unlock();
                false
            }
            Err(_) => true,
        },
        // If we can't even open it, conservatively assume "not held"
        // so the agent doesn't deadlock waiting on a phantom GUI.
        Err(_) => false,
    }
}
