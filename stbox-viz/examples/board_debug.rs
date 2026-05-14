//! Sanity-check the 3D board renderer by saving a single frame to PPM.
//! Run: cargo run --release --example board_debug -- <stl> [pitch] [roll] [yaw]
//! Output: /tmp/board_debug.ppm (P6 binary PPM, viewable in `open` / Preview)

use stbox_viz::board3d::{Mesh, Camera, MountKind, render};
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let stl = args.get(1).map(|s| s.as_str())
        .unwrap_or("/Users/zdavatz/software/fingerfoil/stl/1_board.stl");
    let pitch_deg: f32 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(0.0);
    let roll_deg:  f32 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(0.0);
    let yaw_deg:   f32 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(0.0);

    let mesh = Mesh::load_binary_stl(Path::new(stl), MountKind::Mast)?;
    eprintln!("loaded {} triangles", mesh.tris.len());

    let cam = Camera::iso(MountKind::Mast);
    eprintln!("camera eye: {:?}, up: {:?}, fov_y: {} deg",
        cam.eye, cam.up, cam.fov_y.to_degrees());

    let (w, h) = (800u32, 600u32);
    let frame = render(
        &mesh,
        pitch_deg.to_radians(),
        roll_deg.to_radians(),
        yaw_deg.to_radians(),
        &cam, w, h, [34, 102, 170],
    );

    let n_pixels = (w * h) as usize;
    let n_drawn = (0..n_pixels).filter(|i| frame.rgba[i*4 + 3] != 0).count();
    eprintln!("rendered {} of {} pixels ({:.1}%)",
        n_drawn, n_pixels, 100.0 * n_drawn as f32 / n_pixels as f32);

    let mut out = Vec::with_capacity(n_pixels * 3 + 32);
    out.extend(format!("P6\n{} {}\n255\n", w, h).as_bytes());
    for i in 0..n_pixels {
        if frame.rgba[i*4 + 3] != 0 {
            out.push(frame.rgba[i*4]);
            out.push(frame.rgba[i*4 + 1]);
            out.push(frame.rgba[i*4 + 2]);
        } else {
            out.extend([255, 255, 255]);
        }
    }
    std::fs::write("/tmp/board_debug.ppm", out)?;
    eprintln!("wrote /tmp/board_debug.ppm");
    Ok(())
}
