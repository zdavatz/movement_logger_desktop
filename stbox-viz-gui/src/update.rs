//! Lightweight GitHub-releases update check. Hits the public Releases
//! API once on startup, finds the newest non-prerelease tag matching
//! `vX.Y.Z`, and reports it back if it's newer than the running
//! `CARGO_PKG_VERSION`. Also pulls out the platform-specific download
//! asset URL so the in-app updater can fetch it without extra round
//! trips.

use serde::Deserialize;

const REPO: &str = "zdavatz/fp-sns-stbox1";
const TAG_PREFIX: &str = "v";

#[derive(Deserialize)]
struct GithubRelease {
    tag_name: String,
    html_url: String,
    #[serde(default)]
    prerelease: bool,
    #[serde(default)]
    assets: Vec<GithubAsset>,
}

#[derive(Deserialize)]
struct GithubAsset {
    name: String,
    browser_download_url: String,
}

#[derive(Clone, Debug)]
pub struct UpdateInfo {
    pub version: (u32, u32, u32),
    pub url: String,
    /// Direct download URL for the platform-specific release artifact:
    /// `-macos-aarch64.dmg` on Apple Silicon, the matching tar.gz on
    /// x86_64 Linux, the matching .zip on x86_64 Windows. None when the
    /// release page hasn't published the artifact for this target yet.
    pub download_url: Option<String>,
}

impl UpdateInfo {
    pub fn pretty(&self) -> String {
        format!("v{}.{}.{}", self.version.0, self.version.1, self.version.2)
    }
}

pub fn parse_version(s: &str) -> Option<(u32, u32, u32)> {
    let mut parts = s.splitn(3, '.');
    let major: u32 = parts.next()?.parse().ok()?;
    let minor: u32 = parts.next()?.parse().ok()?;
    let patch_raw = parts.next()?;
    let patch_str: String = patch_raw.chars().take_while(|c| c.is_ascii_digit()).collect();
    let patch: u32 = patch_str.parse().ok()?;
    Some((major, minor, patch))
}

/// Suffix the platform-specific release asset is expected to end with.
/// Empty string = no in-app update on this target (e.g. aarch64 Linux,
/// which only ships a CLI-only tarball without the GUI).
pub const fn target_asset_suffix() -> &'static str {
    if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
        "-macos-aarch64.dmg"
    } else if cfg!(all(target_os = "linux", target_arch = "x86_64")) {
        "-x86_64-unknown-linux-gnu.tar.gz"
    } else if cfg!(all(target_os = "windows", target_arch = "x86_64")) {
        "-x86_64-pc-windows-msvc.zip"
    } else {
        ""
    }
}

fn find_asset(assets: &[GithubAsset]) -> Option<String> {
    let suffix = target_asset_suffix();
    if suffix.is_empty() {
        return None;
    }
    assets
        .iter()
        .find(|a| a.name.ends_with(suffix))
        .map(|a| a.browser_download_url.clone())
}

pub fn check_latest(current: &str) -> Option<UpdateInfo> {
    let cur = parse_version(current)?;
    let client = reqwest::blocking::Client::builder()
        .user_agent("movement-logger-update-check")
        .timeout(std::time::Duration::from_secs(15))
        .build()
        .ok()?;
    let url = format!("https://api.github.com/repos/{}/releases?per_page=30", REPO);
    let resp = client
        .get(&url)
        .header("Accept", "application/vnd.github+json")
        .send()
        .ok()?;
    if !resp.status().is_success() { return None; }
    let releases: Vec<GithubRelease> = resp.json().ok()?;

    let mut best: Option<UpdateInfo> = None;
    for r in releases {
        if r.prerelease { continue; }
        let Some(stripped) = r.tag_name.strip_prefix(TAG_PREFIX) else { continue };
        let Some(v) = parse_version(stripped) else { continue };
        if v <= cur { continue; }
        if best.as_ref().map_or(true, |b| v > b.version) {
            best = Some(UpdateInfo {
                version: v,
                url: r.html_url,
                download_url: find_asset(&r.assets),
            });
        }
    }
    best
}
