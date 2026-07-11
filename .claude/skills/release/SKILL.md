---
name: release
description: Cut a MovementLogger desktop release — tag-triggered CI that builds, signs, notarizes and publishes Linux/macOS/Windows artifacts. Use when releasing a new version of this repo.
---

# MovementLogger desktop release

Push a `vX.Y.Z` tag — `.github/workflows/release.yml` builds Linux x86_64, macOS arm64 (Apple Silicon) and Windows x86_64, packages each as `MovementLogger-vX.Y.Z-<triple>.{tar.gz,zip}`, builds a macOS `.app` + `.dmg`, signs + notarizes + staples the macOS bundle, and attaches everything to the auto-created GitHub Release.

The Apple signing/notarization secrets are already configured on this repo (`gh secret list -R zdavatz/movement_logger_desktop` to inspect names — values are not visible):

- `MACOS_DEVELOPER_ID_CERTIFICATE` / `MACOS_DEVELOPER_ID_CERTIFICATE_PASSWORD` — Developer ID Application cert (team `4B37356EGR`, ywesee GmbH).
- `APPLE_API_KEY_P8` / `APPLE_API_KEY_ID` / `APPLE_API_ISSUER_ID` — App Store Connect API key for `notarytool`.

The signing & notarization jobs gate on those secrets being non-empty, so they activate automatically — no workflow edit needed.

Mac App Store `.pkg` and Microsoft Store MSIX paths from upstream `fp-sns-stbox1` are intentionally omitted from this repo. Port them from `fp-sns-stbox1/.github/workflows/release.yml` if/when needed.

Before tagging: bump `stbox-viz-gui/Cargo.toml` version, commit, push. After any local macOS release build, verify the winit patch held: `nm target/release/MovementLogger | grep CGSSetWindowBackgroundBlur` must be empty (see the root CLAUDE.md build section).
