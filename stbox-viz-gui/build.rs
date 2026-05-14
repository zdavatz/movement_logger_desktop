// Embeds the icon into the Windows .exe so taskbar / explorer / alt-tab
// show the MovementLogger logo without an MSIX wrapper. No-op on macOS
// and Linux — those use the .app's icon.icns / the .desktop launcher.

fn main() {
    #[cfg(target_os = "windows")]
    {
        let ico = std::path::Path::new("assets").join("icon.ico");
        if ico.exists() {
            let mut res = winres::WindowsResource::new();
            res.set_icon(ico.to_str().unwrap());
            // Ignore failures — the icon is cosmetic, build should not
            // fail just because rc.exe can't be located.
            let _ = res.compile();
        }
    }
}
