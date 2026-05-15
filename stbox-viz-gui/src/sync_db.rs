//! Local SQLite sync-state DB.
//!
//! Tracks, per box, which SD-card files have already been pulled to
//! disk so a "Sync now" only fetches the sessions it hasn't seen yet.
//! This layer is deliberately *not* the file-transfer path: transfer
//! (LIST → READ → save) already exists in `ble.rs`/`main.rs`. Sync is
//! the client-side bookkeeping on top of it — the box firmware has no
//! sync concept (LIST/READ/DELETE only), so "what's already mirrored"
//! lives entirely here.
//!
//! Policy (user decision, v0.0.6): sync is **purely additive** — it
//! never issues DELETE. Nothing on the box is ever removed by a sync.

use rusqlite::Connection;
use std::path::{Path, PathBuf};

/// `~/.movementlogger/sqlite/sync.db`
/// (Windows: `%USERPROFILE%\.movementlogger\sqlite\sync.db`).
///
/// Anchored to the home dir, *not* to the user-selectable download
/// folder: if the DB lived next to the files, changing the "Save to"
/// folder would orphan the history and re-pull every session. Home is
/// the one path that's stable across folder changes and app updates.
/// The DB lives in its own `sqlite/` subdir so the data root stays
/// tidy if other state is added next to it later.
pub fn default_db_path() -> PathBuf {
    let home = std::env::var_os("HOME")
        .or_else(|| std::env::var_os("USERPROFILE"))
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."));
    home.join(".movementlogger").join("sqlite").join("sync.db")
}

pub struct SyncDb {
    conn: Connection,
}

impl SyncDb {
    /// Open (creating the file + parent dir + schema if missing).
    pub fn open(path: &Path) -> rusqlite::Result<Self> {
        if let Some(parent) = path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        let conn = Connection::open(path)?;
        // `size` is part of the primary key on purpose: the firmware
        // reuses session-style names, and a file that grew (new
        // session, same name) must count as a *new* file and be
        // re-pulled rather than silently skipped.
        conn.execute_batch(
            "CREATE TABLE IF NOT EXISTS synced_files (
                 box_id        TEXT    NOT NULL,
                 name          TEXT    NOT NULL,
                 size          INTEGER NOT NULL,
                 downloaded_at TEXT    NOT NULL,
                 local_path    TEXT    NOT NULL,
                 PRIMARY KEY (box_id, name, size)
             );",
        )?;
        Ok(Self { conn })
    }

    /// True iff this exact (box, name, size) triple was already pulled.
    pub fn is_synced(&self, box_id: &str, name: &str, size: u64) -> bool {
        self.conn
            .query_row(
                "SELECT 1 FROM synced_files \
                 WHERE box_id = ?1 AND name = ?2 AND size = ?3",
                rusqlite::params![box_id, name, size as i64],
                |_| Ok(()),
            )
            .is_ok()
    }

    /// Record a successfully-saved file. INSERT OR REPLACE so a
    /// re-download of the same triple just refreshes the timestamp /
    /// path instead of erroring on the primary key.
    pub fn mark_synced(
        &self,
        box_id: &str,
        name: &str,
        size: u64,
        local_path: &str,
    ) -> rusqlite::Result<()> {
        self.conn.execute(
            "INSERT OR REPLACE INTO synced_files \
               (box_id, name, size, downloaded_at, local_path) \
             VALUES (?1, ?2, ?3, ?4, ?5)",
            rusqlite::params![
                box_id,
                name,
                size as i64,
                chrono::Utc::now().to_rfc3339(),
                local_path
            ],
        )?;
        Ok(())
    }
}
