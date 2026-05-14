#!/usr/bin/env bash
# Install MovementLogger into the user's desktop environment.
#
# Drops the binaries into ~/.local/bin, the .desktop file into
# ~/.local/share/applications and the icon into the standard hicolor
# theme directory. Run from inside the unpacked release archive.

set -euo pipefail

bin_dir="$HOME/.local/bin"
apps_dir="$HOME/.local/share/applications"
icons_dir="$HOME/.local/share/icons/hicolor/512x512/apps"

mkdir -p "$bin_dir" "$apps_dir" "$icons_dir"

cp -v MovementLogger "$bin_dir/"
# stbox-viz CLI ships next to the GUI so MovementLogger can find it via
# its current_exe() lookup.
cp -v stbox-viz "$bin_dir/" 2>/dev/null || true

cp -v MovementLogger.desktop "$apps_dir/"
cp -v icon.png               "$icons_dir/MovementLogger.png"

if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database "$apps_dir" 2>/dev/null || true
fi
if command -v gtk-update-icon-cache >/dev/null 2>&1; then
    gtk-update-icon-cache "$HOME/.local/share/icons/hicolor" 2>/dev/null || true
fi

echo
echo "Installed to $bin_dir."
echo "Make sure $bin_dir is on your PATH:  export PATH=\"\$HOME/.local/bin:\$PATH\""
echo "Launch from your application menu or run: MovementLogger"
