//! Persisted GUI/agent config (`~/.movementlogger/config.toml`).
//!
//! Issue #14 part B step 4. The GUI is the writer (it updates this on
//! Connect, save-dir change, keep-synced toggle, and box log-mode
//! change); the headless `--agent` is the reader (it needs to know
//! which box to connect to, where to mirror, whether to keep syncing,
//! and whether AUTO mode is even active). `$HOME`-anchored, same
//! rationale as the sync DB — independent of the user-selectable
//! "Save to" folder so changing that doesn't orphan the agent.

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

/// `~/.movementlogger` (Windows `%USERPROFILE%\.movementlogger`). Same
/// resolution as `sync_db::default_db_path` — kept here too so the
/// agent has one obvious place to look without depending on rusqlite.
pub fn movementlogger_home() -> PathBuf {
    let home = std::env::var_os("HOME")
        .or_else(|| std::env::var_os("USERPROFILE"))
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."));
    home.join(".movementlogger")
}

/// `~/.movementlogger/config.toml`.
pub fn default_path() -> PathBuf {
    movementlogger_home().join("config.toml")
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct AgentConfig {
    /// btleplug peripheral id of the box to mirror, captured by the GUI
    /// at Connect-click time. `None` = the agent scans for the first
    /// `BOX_NAMES` match instead.
    pub box_id: Option<String>,
    /// Where the agent mirrors files — the resolved/validated "Save to"
    /// folder from the GUI. Empty = agent falls back to
    /// `sync_core::default_save_base().join("csv")`.
    #[serde(default)]
    pub save_dir: String,
    /// Mirror of the GUI "Keep synced" checkbox. The agent only runs
    /// the continuous poll when this is true.
    pub keep_synced: bool,
    /// Last known box log mode (GET_MODE result). `Some(false)` = AUTO,
    /// `Some(true)` = MANUAL, `None` = unknown. The autostart login
    /// item is registered only while this is `Some(false)` (AUTO); the
    /// agent self-exits if it sees MANUAL.
    pub log_mode_manual: Option<bool>,
    /// Magnetometer hard-iron offset in mG, captured by the Live tab's
    /// "Calibrate compass" flow (min/max midpoint per axis while the
    /// user tumbles the box). Subtracted from the raw mag reading
    /// before the eCompass heading — without it a box-fixed magnetic
    /// bias bigger than the ~200 mG horizontal earth field pins the
    /// heading regardless of rotation. `None` = uncalibrated.
    pub mag_offset_mg: Option<[f64; 3]>,
    /// One-tap direction anchor (deg) subtracted from the box's nose
    /// azimuth so "USB-C points SOUTH" reads as 180° (iOS/Android
    /// parity). `None`/absent = 0 (no anchor set yet).
    #[serde(default)]
    pub heading_bias_deg: Option<f64>,
    /// Which body-axis end is the FRONT (USB-C): `Some(true)` = +Y,
    /// `Some(false)` = -Y, `None` = not confirmed.
    #[serde(default)]
    pub nose_plus_y: Option<bool>,
    /// Lateral render mirror (+1/-1) from the right-side confirm tap.
    #[serde(default)]
    pub lateral_sign: Option<f64>,
    /// Calibrated board-angle zero reference `[pitch, roll, yaw]` in
    /// degrees, captured at the Live tab's "Zero here" tap (yaw sampled at
    /// bias 0 so the tared heading is direction-cal independent). `None` =
    /// not zeroed. Persisted so a mounted-box tare survives reconnect / app
    /// restart. iOS `AgentConfig.angleZeroRef` parity.
    #[serde(default)]
    pub angle_zero_ref: Option<[f64; 3]>,
    /// Wall-clock (epoch seconds) of the last "Zero here" tap — drives the
    /// "zeroed N ago" note. `None` when never zeroed / after Clear. iOS
    /// `AgentConfig.angleZeroAtEpoch` parity.
    #[serde(default)]
    pub angle_zero_at: Option<f64>,
    /// Last box-confirmed GPS power state (`Some(false)` = the user
    /// deliberately runs the box with GPS off). Used at launch — before a
    /// box has answered GET_GPS_POWER — to keep suppressing the "GPS gave
    /// no signal at power-on" health badge across app restarts. Updated
    /// whenever the box's GpsPower reply arrives.
    #[serde(default)]
    pub gps_power: Option<bool>,
    /// UI zoom factor from the top-bar A+/A− font-size buttons (1.0 =
    /// default). Persisted so the user's chosen size survives a restart.
    /// Clamped to [0.6, 3.0] on both write (button handlers) and read
    /// (startup apply) so a hand-edited config can't render the UI
    /// unusable. GUI-only; the headless agent ignores it.
    #[serde(default)]
    pub ui_zoom: Option<f32>,
    /// Videos last collected in the Replay tab's "Merge videos" list,
    /// restored at startup (entries whose files vanished are dropped).
    /// GUI-only; the headless agent ignores it.
    #[serde(default)]
    pub merge_videos: Option<Vec<String>>,
}

impl AgentConfig {
    /// Load the config, or a default if it's missing / unreadable /
    /// malformed (a corrupt file must never brick the agent or GUI).
    /// (Reader side wired by the headless agent in step 6.)
    #[allow(dead_code)]
    pub fn load() -> Self {
        Self::load_from(&default_path())
    }

    #[allow(dead_code)]
    pub fn load_from(path: &Path) -> Self {
        match std::fs::read_to_string(path) {
            Ok(s) => toml::from_str(&s).unwrap_or_default(),
            Err(_) => Self::default(),
        }
    }

    /// Atomically persist (write a temp file in the same dir, then
    /// rename over the target) so a crash mid-write can't leave a
    /// half-parsed TOML that the agent then chokes on. Best-effort —
    /// returns the io error for logging but callers treat config
    /// persistence as non-fatal.
    pub fn save(&self) -> std::io::Result<()> {
        self.save_to(&default_path())
    }

    pub fn save_to(&self, path: &Path) -> std::io::Result<()> {
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let body = toml::to_string_pretty(self)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        let tmp = path.with_extension("toml.tmp");
        std::fs::write(&tmp, body)?;
        std::fs::rename(&tmp, path)?;
        Ok(())
    }
}
