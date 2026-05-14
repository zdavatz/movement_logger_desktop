//! Software 3D rasterizer for the fingerfoil board STL.
//!
//! Loads a binary STL once, then renders the board mesh per-frame at a
//! given pitch/roll/yaw orientation into an RGBA buffer that the
//! animator overlays in the side-view panel.
//!
//! No GPU dependency; pure Rust + standard library. ~25 ms per frame
//! for the full 294k-triangle board mesh at 600×400 panel resolution
//! (single-threaded, release build).
//!
//! Coordinate conventions (board body frame):
//!   - +X = nose direction (board points along +X)
//!   - +Y = port (left side of rider)
//!   - +Z = up (deck face)
//! Pitch rotates around Y (nose up = positive), roll rotates around X
//! (port-side up = positive), yaw rotates around Z (counterclockwise
//! viewed from above = positive). Order applied: yaw → pitch → roll.

use anyhow::{Context, Result};
use std::path::Path;

/// 3D vector with the minimal ops the rasterizer needs.
#[derive(Clone, Copy, Debug)]
pub struct Vec3 { pub x: f32, pub y: f32, pub z: f32 }

impl Vec3 {
    pub const fn new(x: f32, y: f32, z: f32) -> Self { Self { x, y, z } }
    pub fn sub(self, o: Vec3) -> Vec3 { Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z) }
    pub fn add(self, o: Vec3) -> Vec3 { Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z) }
    pub fn mul(self, s: f32) -> Vec3 { Vec3::new(self.x * s, self.y * s, self.z * s) }
    pub fn dot(self, o: Vec3) -> f32 { self.x * o.x + self.y * o.y + self.z * o.z }
    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
    pub fn length(self) -> f32 { self.dot(self).sqrt() }
    pub fn normalized(self) -> Vec3 {
        let l = self.length();
        if l > 1e-9 { self.mul(1.0 / l) } else { Vec3::new(0.0, 0.0, 0.0) }
    }
}

/// Triangle in body frame.
#[derive(Clone, Copy)]
pub struct Tri { pub v: [Vec3; 3] }

/// Loaded mesh — re-centred at origin and scaled so the longest body-
/// frame extent fits in [-1, +1]. The animator scales+positions the
/// rendered image to fit the panel area, so absolute mesh size doesn't
/// matter.
pub struct Mesh {
    pub tris: Vec<Tri>,
    /// Per-triangle face normals, precomputed (saves one cross product
    /// per triangle per frame).
    pub normals: Vec<Vec3>,
}

/// Where the SensorTile.box is physically attached on the board.
/// Different mounts need different `R_mount`, camera angle, and accel
/// pre-rotation, so the user picks one at the CLI (`--mount mast|deck`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MountKind {
    /// Box strapped to the mast with the chip's +X pointing along the
    /// mast (down toward the foil at level pose). Original mount used
    /// for Ayano's 22.4.2026 + 25.4.2026 Ermioni sessions.
    Mast,
    /// Box on top of the deck, long axis along the board's nose-tail.
    /// Chip's +Z points down through the deck. Used for Peter's
    /// 28.4.2026 session.
    Deck,
}

impl Mesh {
    /// Load a binary STL file from disk for the given mount.
    ///
    /// Format: 80-byte header (skipped) + uint32 LE triangle count +
    /// repeated (3 floats normal LE, 3×3 floats vertices LE, uint16
    /// attribute byte count). 50 bytes per triangle.
    pub fn load_binary_stl(path: &Path, mount: MountKind) -> Result<Mesh> {
        let bytes = std::fs::read(path)
            .with_context(|| format!("read STL {}", path.display()))?;
        if bytes.len() < 84 {
            anyhow::bail!("STL too short: {} bytes", bytes.len());
        }
        let n_tris = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
        let expected = 84 + n_tris * 50;
        if bytes.len() < expected {
            anyhow::bail!("STL truncated: header says {} triangles ({} bytes) but file is {}",
                n_tris, expected, bytes.len());
        }

        let read_f32 = |off: usize| -> f32 {
            f32::from_le_bytes([bytes[off], bytes[off+1], bytes[off+2], bytes[off+3]])
        };

        let mut tris = Vec::with_capacity(n_tris);
        let mut normals = Vec::with_capacity(n_tris);
        for i in 0..n_tris {
            let base = 84 + i * 50;
            // Skip the 12-byte stored normal — we recompute from
            // vertices to ensure consistent winding.
            let v0 = Vec3::new(read_f32(base + 12), read_f32(base + 16), read_f32(base + 20));
            let v1 = Vec3::new(read_f32(base + 24), read_f32(base + 28), read_f32(base + 32));
            let v2 = Vec3::new(read_f32(base + 36), read_f32(base + 40), read_f32(base + 44));
            tris.push(Tri { v: [v0, v1, v2] });
            let normal = v1.sub(v0).cross(v2.sub(v0)).normalized();
            normals.push(normal);
        }

        Ok(Mesh { tris, normals }.recentre_unit(mount))
    }

    /// Re-centre the mesh at origin and scale so the longest extent in
    /// any axis equals 2 (i.e. [-1, +1]). Lets the camera + animator
    /// work in mesh-independent units. Then apply the mount-specific
    /// `R_mount` rotation to align the mesh body frame with the IMU
    /// body frame.
    fn recentre_unit(mut self, mount: MountKind) -> Mesh {
        if self.tris.is_empty() { return self; }
        let mut min = Vec3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY);
        let mut max = Vec3::new(-f32::INFINITY, -f32::INFINITY, -f32::INFINITY);
        for t in &self.tris {
            for v in &t.v {
                if v.x < min.x { min.x = v.x; } if v.x > max.x { max.x = v.x; }
                if v.y < min.y { min.y = v.y; } if v.y > max.y { max.y = v.y; }
                if v.z < min.z { min.z = v.z; } if v.z > max.z { max.z = v.z; }
            }
        }
        let centre = Vec3::new(
            (min.x + max.x) * 0.5,
            (min.y + max.y) * 0.5,
            (min.z + max.z) * 0.5,
        );
        let extent = (max.x - min.x).max(max.y - min.y).max(max.z - min.z);
        let scale = if extent > 1e-9 { 2.0 / extent } else { 1.0 };
        for t in &mut self.tris {
            for v in &mut t.v {
                v.x = (v.x - centre.x) * scale;
                v.y = (v.y - centre.y) * scale;
                v.z = (v.z - centre.z) * scale;
            }
        }
        // Hard-coded mount transform `R_mount` mapping board frame
        // → IMU frame. Pre-rotating each mesh vertex by R_mount puts
        // the mesh into the IMU body frame, so the per-frame IMU
        // quaternion can be applied directly without a separate
        // calibration step.
        //
        // Mast mount (Ayano's setup):
        //   IMU +X = -Z_board (mast-down — into the foil at level)
        //   IMU +Y = +X_board (nose direction)
        //   IMU +Z = +Y_board (port direction)
        //   ⇒ R_mount(v) = (-v.z, v.x, v.y)
        //
        // Deck mount (Peter's 28.4.2026 setup):
        //   The 3D path uses an accel-only tilt quaternion that is
        //   already pre-flipped on the accelerometer input (in
        //   animate_cmd.rs `tilt_quats`), so the resulting quaternion
        //   is already in a right-side-up body frame. R_mount stays
        //   identity here to avoid double-flipping the mesh.
        let apply_mount: fn(Vec3) -> Vec3 = match mount {
            MountKind::Mast => |v: Vec3| Vec3::new(-v.z, v.x, v.y),
            MountKind::Deck => |v: Vec3| v,
        };
        for t in &mut self.tris {
            for v in &mut t.v {
                *v = apply_mount(*v);
            }
        }
        for n in &mut self.normals {
            *n = apply_mount(*n);
        }
        self
    }
}

/// Camera + light parameters. Camera looks at the world origin; the
/// up vector defines the screen "up" direction.
#[derive(Clone, Copy)]
pub struct Camera {
    pub eye: Vec3,
    pub up: Vec3,
    /// Vertical field of view in radians.
    pub fov_y: f32,
}

impl Camera {
    /// Mount-appropriate side/tail view of the board.
    ///
    /// Mast mount: pure side view from port — eye in the world Y-Z
    /// plane (X = 0), board nose-tail axis (world X) projects
    /// horizontally on screen, so a pitch rotation around the board's
    /// lateral axis (world Y) becomes purely vertical screen motion.
    ///
    /// Deck mount: view from behind the tail — the accel-only tilt
    /// path lands the rendered nose at world −X, so the camera sits
    /// at world +X (= "behind the tail") looking into the screen.
    /// Pump rotation tilts the nose up/down; roll rocks the deck
    /// side-to-side.
    pub fn iso(mount: MountKind) -> Self {
        let eye = match mount {
            MountKind::Mast => Vec3::new(0.0, 3.2, 0.5),
            MountKind::Deck => Vec3::new(-3.0, 0.0, 0.7),
        };
        Camera { eye, up: Vec3::new(0.0, 0.0, 1.0), fov_y: 35f32.to_radians() }
    }
}

/// Rotation matrix Z(yaw) · Y(pitch) · X(roll). Applied as
/// world_pos = Z·Y·X · body_pos so the rotation order matches "first
/// roll, then pitch, then yaw" — the natural order for a board (yaw
/// is heading, pitch is bow up/down, roll is bank).
fn rotation_matrix(pitch: f32, roll: f32, yaw: f32) -> [[f32; 3]; 3] {
    let (cx, sx) = (roll.cos(),  roll.sin());
    let (cy, sy) = (pitch.cos(), pitch.sin());
    let (cz, sz) = (yaw.cos(),   yaw.sin());
    // Z·Y·X
    [
        [cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
        [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
        [-sy,   cy*sx,            cy*cx],
    ]
}

/// Hamilton quaternion product `a * b`. Quat layout `[w, x, y, z]`.
pub fn quat_mul(a: &[f64; 4], b: &[f64; 4]) -> [f64; 4] {
    let (aw, ax, ay, az) = (a[0], a[1], a[2], a[3]);
    let (bw, bx, by, bz) = (b[0], b[1], b[2], b[3]);
    [
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ]
}

/// Conjugate (= inverse for unit quaternions). Quat layout `[w, x, y, z]`.
pub fn quat_conj(q: &[f64; 4]) -> [f64; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

/// Strip the rotation around the world Z axis (= yaw) from a unit
/// quaternion using swing-twist decomposition. Returns the "swing"
/// part — rotation purely around an axis in the XY plane (= pitch +
/// roll only). Useful for IMU sources whose yaw drifts (Madgwick
/// 6DOF without magnetometer) and where heading should come from
/// a different sensor (GPS course over ground).
pub fn quat_strip_yaw(q: &[f64; 4]) -> [f64; 4] {
    // Twist around Z: project (q.w, q.z) onto a unit quaternion of
    // the form (w, 0, 0, z) — that's the part of q that rotates
    // around Z. The rest is the swing.
    let (w, z) = (q[0], q[3]);
    let n = (w * w + z * z).sqrt();
    let q_twist = if n > 1e-9 {
        [w / n, 0.0, 0.0, z / n]
    } else {
        [1.0, 0.0, 0.0, 0.0]
    };
    quat_mul(q, &quat_conj(&q_twist))
}

/// Quaternion → 3×3 rotation matrix (body-frame coords → world-frame
/// coords). Madgwick's IMU output rotates world to body, so to render
/// the body in world coords we use the *transpose* of the standard
/// matrix — equivalently, we use `quat_conj(q)` and the standard
/// formula. This helper takes the conjugate-already form (caller is
/// expected to pass `quat_conj(q_madgwick)` if needed) and emits the
/// matrix that applies the rotation in the natural sense.
pub fn quat_to_matrix(q: &[f64; 4]) -> [[f32; 3]; 3] {
    let (w, x, y, z) = (q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32);
    [
        [1.0 - 2.0*(y*y + z*z), 2.0*(x*y - w*z),       2.0*(x*z + w*y)],
        [2.0*(x*y + w*z),       1.0 - 2.0*(x*x + z*z), 2.0*(y*z - w*x)],
        [2.0*(x*z - w*y),       2.0*(y*z + w*x),       1.0 - 2.0*(x*x + y*y)],
    ]
}

fn mat_mul_vec(m: &[[f32; 3]; 3], v: Vec3) -> Vec3 {
    Vec3::new(
        m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
        m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
        m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z,
    )
}

/// Rendered RGBA frame.
pub struct Frame {
    pub w: u32,
    pub h: u32,
    pub rgba: Vec<u8>,
}

/// Render the mesh at the given pitch/roll/yaw (radians) into an RGBA
/// buffer of the requested size. Background is fully transparent so
/// the animator can overlay it on the side-view panel.
///
/// `base_color` is the base surface colour (0..255 per channel); flat
/// shading multiplies it by the lambertian dot-product against the
/// fixed light direction (normalized world-frame), with an ambient
/// floor so back-facing triangles aren't pitch-black.
pub fn render(
    mesh: &Mesh,
    pitch_rad: f32,
    roll_rad: f32,
    yaw_rad: f32,
    cam: &Camera,
    w: u32,
    h: u32,
    base_color: [u8; 3],
) -> Frame {
    let rot = rotation_matrix(pitch_rad, roll_rad, yaw_rad);
    render_with_matrix(mesh, rot, cam, w, h, base_color)
}

/// Same as `render` but takes the body-to-world rotation matrix
/// directly instead of pitch/roll/yaw. Use when the rotation comes
/// from a quaternion (or any other source not naturally expressed as
/// Tait-Bryan angles).
pub fn render_with_matrix(
    mesh: &Mesh,
    rot: [[f32; 3]; 3],
    cam: &Camera,
    w: u32,
    h: u32,
    base_color: [u8; 3],
) -> Frame {
    let mut rgba = vec![0u8; (w * h * 4) as usize];
    let mut zbuf = vec![f32::INFINITY; (w * h) as usize];

    // World-frame light direction (toward light = positive). Same
    // 45°-elevation key light used elsewhere in the project.
    let light_world = Vec3::new(-0.4, 0.3, 0.85).normalized();
    let ambient = 0.35f32;

    // View matrix: camera at `cam.eye`, looking at origin, up = cam.up.
    let forward = Vec3::new(0.0, 0.0, 0.0).sub(cam.eye).normalized();
    let right = forward.cross(cam.up).normalized();
    let true_up = right.cross(forward).normalized();
    // World → camera transform: subtract eye, then rotate so forward
    // = +Z (depth into scene), right = +X, up = +Y in camera frame.
    // Points in front of the camera have c.z > 0.
    let world_to_cam = |p: Vec3| -> Vec3 {
        let q = p.sub(cam.eye);
        Vec3::new(q.dot(right), q.dot(true_up), q.dot(forward))
    };

    let aspect = w as f32 / h as f32;
    let f = 1.0 / (cam.fov_y * 0.5).tan();
    let project = |c: Vec3| -> Option<(f32, f32, f32)> {
        // c.z is forward distance in camera frame (negative because we
        // negated above). Cull points behind the camera.
        if c.z <= 0.05 { return None; }
        let nx = (c.x * f) / (c.z * aspect);
        let ny = (c.y * f) / c.z;
        // NDC [-1, 1] → screen pixels [0, w/h)
        let sx = (nx * 0.5 + 0.5) * (w as f32);
        let sy = (1.0 - (ny * 0.5 + 0.5)) * (h as f32);
        Some((sx, sy, c.z))
    };

    // Per-triangle pipeline.
    for (tri, normal) in mesh.tris.iter().zip(mesh.normals.iter()) {
        // Rotate vertices and normal into world frame.
        let n_world = mat_mul_vec(&rot, *normal);
        let v0 = mat_mul_vec(&rot, tri.v[0]);
        let v1 = mat_mul_vec(&rot, tri.v[1]);
        let v2 = mat_mul_vec(&rot, tri.v[2]);

        // Backface cull (in world frame, against camera). Since the
        // camera sits at `cam.eye`, the view direction at the triangle
        // is approximately (centroid - cam.eye); flip when needed.
        let centroid = v0.add(v1).add(v2).mul(1.0 / 3.0);
        let view_dir = centroid.sub(cam.eye);
        if n_world.dot(view_dir) > 0.0 { continue; }

        // Lambertian flat shading.
        let lam = n_world.dot(light_world).max(0.0);
        let intensity = ambient + (1.0 - ambient) * lam;
        let color = [
            (base_color[0] as f32 * intensity).clamp(0.0, 255.0) as u8,
            (base_color[1] as f32 * intensity).clamp(0.0, 255.0) as u8,
            (base_color[2] as f32 * intensity).clamp(0.0, 255.0) as u8,
        ];

        // Project into screen space.
        let p0 = match project(world_to_cam(v0)) { Some(p) => p, None => continue };
        let p1 = match project(world_to_cam(v1)) { Some(p) => p, None => continue };
        let p2 = match project(world_to_cam(v2)) { Some(p) => p, None => continue };

        rasterize_tri(p0, p1, p2, color, &mut rgba, &mut zbuf, w, h);
    }

    Frame { w, h, rgba }
}

/// Edge function — positive when the point is on the right of the
/// directed edge from `a` to `b`. Used as the standard half-plane
/// test in barycentric rasterization.
#[inline]
fn edge(a: (f32, f32), b: (f32, f32), p: (f32, f32)) -> f32 {
    (b.0 - a.0) * (p.1 - a.1) - (b.1 - a.1) * (p.0 - a.0)
}

/// Rasterize a single screen-space triangle with z-buffering, writing
/// `color` (with alpha = 255) into the RGBA framebuffer where it is
/// closer than the current z-buffer entry. p0/p1/p2 are (sx, sy, z).
fn rasterize_tri(
    p0: (f32, f32, f32),
    p1: (f32, f32, f32),
    p2: (f32, f32, f32),
    color: [u8; 3],
    rgba: &mut [u8],
    zbuf: &mut [f32],
    w: u32,
    h: u32,
) {
    // Bounding box clipped to screen.
    let min_x = p0.0.min(p1.0).min(p2.0).floor().max(0.0) as i32;
    let max_x = p0.0.max(p1.0).max(p2.0).ceil().min((w - 1) as f32) as i32;
    let min_y = p0.1.min(p1.1).min(p2.1).floor().max(0.0) as i32;
    let max_y = p0.1.max(p1.1).max(p2.1).ceil().min((h - 1) as f32) as i32;
    if min_x > max_x || min_y > max_y { return; }

    let a = (p0.0, p0.1);
    let b = (p1.0, p1.1);
    let c = (p2.0, p2.1);
    let area = edge(a, b, c);
    if area.abs() < 1e-6 { return; }
    let inv_area = 1.0 / area;
    // Fix winding so all three edge-tests are non-negative inside the
    // triangle (front-facing in our screen convention). We don't know
    // the orientation a priori from the projection, so we accept both
    // and choose based on the sign of `area`.
    let flipped = area < 0.0;

    for py in min_y..=max_y {
        for px in min_x..=max_x {
            let p = (px as f32 + 0.5, py as f32 + 0.5);
            let mut w0 = edge(b, c, p);
            let mut w1 = edge(c, a, p);
            let mut w2 = edge(a, b, p);
            if flipped { w0 = -w0; w1 = -w1; w2 = -w2; }
            if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 { continue; }

            let l0 = w0 * inv_area.abs();
            let l1 = w1 * inv_area.abs();
            let l2 = w2 * inv_area.abs();
            let z = l0 * p0.2 + l1 * p1.2 + l2 * p2.2;

            let idx = (py as u32 * w + px as u32) as usize;
            if z < zbuf[idx] {
                zbuf[idx] = z;
                let p4 = idx * 4;
                rgba[p4]     = color[0];
                rgba[p4 + 1] = color[1];
                rgba[p4 + 2] = color[2];
                rgba[p4 + 3] = 255;
            }
        }
    }
}

/// Compute per-sensor-sample yaw (radians, world-frame) from GPS
/// course-over-ground when the rider is moving, holding the last good
/// value when stationary. Returns one yaw per sample in `sensor_t_s`.
///
/// `sensor_t_s` and `gps_t_s` are seconds since base ticks (any
/// shared baseline works). `gps_course_deg` is the GPS COG (degrees,
/// CW from North). `gps_speed_kmh` is the speed used to gate
/// trustworthiness — below `min_speed_kmh` the value is held.
///
/// Note: GPS COG is degrees CW from North, but our body-frame yaw is
/// CCW around +Z. The conversion is yaw_rad = -COG_rad (so a 90° GPS
/// course = pointing East = +Y in our body frame, which is yaw of
/// -π/2 around +Z). This makes the rendered board's nose track the
/// actual ground heading.
pub fn yaw_from_gps(
    sensor_t_s: &[f64],
    gps_t_s: &[f64],
    gps_course_deg: &[f64],
    gps_speed_kmh: &[f64],
    min_speed_kmh: f64,
) -> Vec<f32> {
    let mut out = Vec::with_capacity(sensor_t_s.len());
    let mut last_yaw = 0.0f32;

    let mut gi = 0usize;
    for &t in sensor_t_s {
        // Advance GPS index until gps_t_s[gi] is the latest GPS row
        // with t_s <= t.
        while gi + 1 < gps_t_s.len() && gps_t_s[gi + 1] <= t {
            gi += 1;
        }
        if gi < gps_speed_kmh.len() && gps_speed_kmh[gi] >= min_speed_kmh
            && gi < gps_course_deg.len() && gps_course_deg[gi].is_finite()
        {
            // Convert COG (deg CW from North) → yaw (rad CCW around +Z).
            let cog_rad = (gps_course_deg[gi] as f32).to_radians();
            last_yaw = -cog_rad;
        }
        out.push(last_yaw);
    }
    out
}
