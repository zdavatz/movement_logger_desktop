//! Minimal in-process video player backed by **libmpv**, loaded at runtime
//! (dlopen, no build-time link) and rendered with mpv's **software render
//! API** — each frame lands in an RGBA byte buffer we hand straight to egui as
//! a texture. No OpenGL interop, no native child window: the video becomes a
//! normal egui image, so the app can paint overlays (cut ranges, playhead) on
//! top of it — the whole reason we do this in-process instead of a browser
//! embed or a native VLC surface.
//!
//! Runtime loading (via `libloading`) means the app builds and launches on all
//! three platforms with **no** mpv dependency; if libmpv isn't installed the
//! player is simply unavailable and the caller falls back to the old browser
//! preview. On macOS `brew install mpv` provides `libmpv.2.dylib`.

use std::ffi::CString;
use std::os::raw::{c_char, c_int, c_void};
use std::ptr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use libloading::{Library, Symbol};

// --- mpv_format ---------------------------------------------------------
const MPV_FORMAT_FLAG: c_int = 3;
const MPV_FORMAT_INT64: c_int = 4;
const MPV_FORMAT_DOUBLE: c_int = 5;

// --- mpv_render_param_type (values verified against mpv/render.h) --------
const MPV_RENDER_PARAM_INVALID: c_int = 0;
const MPV_RENDER_PARAM_API_TYPE: c_int = 1;
const MPV_RENDER_PARAM_SW_SIZE: c_int = 17;
const MPV_RENDER_PARAM_SW_FORMAT: c_int = 18;
const MPV_RENDER_PARAM_SW_STRIDE: c_int = 19;
const MPV_RENDER_PARAM_SW_POINTER: c_int = 20;

// mpv_render_context_update() return-flag bit
const MPV_RENDER_UPDATE_FRAME: u64 = 1;

// mpv_event_id
const MPV_EVENT_SHUTDOWN: c_int = 1;
const MPV_EVENT_PROPERTY_CHANGE: c_int = 22;

#[repr(C)]
struct MpvRenderParam {
    type_: c_int,
    data: *mut c_void,
}

#[repr(C)]
struct MpvEvent {
    event_id: c_int,
    error: c_int,
    reply_userdata: u64,
    data: *mut c_void,
}

#[repr(C)]
struct MpvEventProperty {
    name: *const c_char,
    format: c_int,
    data: *mut c_void,
}

type FnCreate = unsafe extern "C" fn() -> *mut c_void;
type FnInit = unsafe extern "C" fn(*mut c_void) -> c_int;
type FnSetOptStr = unsafe extern "C" fn(*mut c_void, *const c_char, *const c_char) -> c_int;
type FnSetProp = unsafe extern "C" fn(*mut c_void, *const c_char, c_int, *mut c_void) -> c_int;
type FnGetProp = unsafe extern "C" fn(*mut c_void, *const c_char, c_int, *mut c_void) -> c_int;
type FnCommand = unsafe extern "C" fn(*mut c_void, *const *const c_char) -> c_int;
type FnRenderCreate =
    unsafe extern "C" fn(*mut *mut c_void, *mut c_void, *const MpvRenderParam) -> c_int;
type FnRender = unsafe extern "C" fn(*mut c_void, *const MpvRenderParam) -> c_int;
type FnRenderSetCb =
    unsafe extern "C" fn(*mut c_void, Option<unsafe extern "C" fn(*mut c_void)>, *mut c_void);
type FnRenderUpdate = unsafe extern "C" fn(*mut c_void) -> u64;
type FnRenderFree = unsafe extern "C" fn(*mut c_void);
type FnTerminate = unsafe extern "C" fn(*mut c_void);
type FnObserve = unsafe extern "C" fn(*mut c_void, u64, *const c_char, c_int) -> c_int;
type FnWaitEvent = unsafe extern "C" fn(*mut c_void, f64) -> *mut MpvEvent;

/// The loaded libmpv and its resolved entry points. Cheap to `Arc`-clone so
/// several `Player`s share one library handle.
pub struct MpvLib {
    _lib: Library,
    create: FnCreate,
    initialize: FnInit,
    set_option_string: FnSetOptStr,
    set_property: FnSetProp,
    get_property: FnGetProp,
    command: FnCommand,
    render_create: FnRenderCreate,
    render: FnRender,
    render_set_update_cb: FnRenderSetCb,
    render_update: FnRenderUpdate,
    render_free: FnRenderFree,
    terminate: FnTerminate,
    observe_property: FnObserve,
    wait_event: FnWaitEvent,
}

// The raw function pointers are stable for the loaded library's lifetime, and
// every call is confined to the main (UI) thread; the update callback only
// touches thread-safe primitives.
unsafe impl Send for MpvLib {}
unsafe impl Sync for MpvLib {}

fn candidate_libs() -> &'static [&'static str] {
    #[cfg(target_os = "macos")]
    {
        &[
            // Versioned name via the dynamic-loader search paths.
            "libmpv.2.dylib",
            // Homebrew "opt" prefix (Apple Silicon, then Intel). This is the
            // version-stable canonical location and — crucially — resolves
            // even when the flat `<prefix>/lib/libmpv.*.dylib` symlinks are
            // broken, which is a common outcome of an interrupted
            // `brew upgrade mpv` (the exact state that made the in-window
            // preview silently fall back to the browser). Tried before the
            // flat `lib` paths for that reason.
            "/opt/homebrew/opt/mpv/lib/libmpv.2.dylib",
            "/usr/local/opt/mpv/lib/libmpv.2.dylib",
            // Flat Homebrew lib dir.
            "/opt/homebrew/lib/libmpv.2.dylib",
            "/usr/local/lib/libmpv.2.dylib",
            // Unversioned fallbacks, same order.
            "libmpv.dylib",
            "/opt/homebrew/opt/mpv/lib/libmpv.dylib",
            "/usr/local/opt/mpv/lib/libmpv.dylib",
            "/opt/homebrew/lib/libmpv.dylib",
            "/usr/local/lib/libmpv.dylib",
        ]
    }
    #[cfg(target_os = "linux")]
    {
        &["libmpv.so.2", "libmpv.so.1", "libmpv.so"]
    }
    #[cfg(target_os = "windows")]
    {
        &["libmpv-2.dll", "mpv-2.dll", "libmpv.dll", "mpv.dll"]
    }
    #[cfg(not(any(target_os = "macos", target_os = "linux", target_os = "windows")))]
    {
        &["libmpv.so.2"]
    }
}

impl MpvLib {
    /// Try each platform-standard libmpv name/path until one loads and exports
    /// every symbol we need. `Err` lists what was tried so the caller can hint.
    pub fn load() -> Result<MpvLib, String> {
        // Escape hatch: force the browser-preview fallback even when libmpv is
        // present (for reproducing the mpv-absent path during testing).
        if std::env::var_os("ML_NO_MPV").is_some() {
            return Err("disabled via ML_NO_MPV".into());
        }
        let mut last = String::new();
        for name in candidate_libs() {
            match unsafe { Self::load_from(name) } {
                Ok(l) => return Ok(l),
                Err(e) => last = format!("{}: {}", name, e),
            }
        }
        Err(format!("libmpv not found (last error — {})", last))
    }

    unsafe fn load_from(name: &str) -> Result<MpvLib, String> {
        let lib = Library::new(name).map_err(|e| e.to_string())?;
        macro_rules! sym {
            ($t:ty, $n:expr) => {{
                let s: Symbol<$t> = lib.get($n).map_err(|e| e.to_string())?;
                *s
            }};
        }
        let out = MpvLib {
            create: sym!(FnCreate, b"mpv_create\0"),
            initialize: sym!(FnInit, b"mpv_initialize\0"),
            set_option_string: sym!(FnSetOptStr, b"mpv_set_option_string\0"),
            set_property: sym!(FnSetProp, b"mpv_set_property\0"),
            get_property: sym!(FnGetProp, b"mpv_get_property\0"),
            command: sym!(FnCommand, b"mpv_command\0"),
            render_create: sym!(FnRenderCreate, b"mpv_render_context_create\0"),
            render: sym!(FnRender, b"mpv_render_context_render\0"),
            render_set_update_cb: sym!(FnRenderSetCb, b"mpv_render_context_set_update_callback\0"),
            render_update: sym!(FnRenderUpdate, b"mpv_render_context_update\0"),
            render_free: sym!(FnRenderFree, b"mpv_render_context_free\0"),
            terminate: sym!(FnTerminate, b"mpv_terminate_destroy\0"),
            observe_property: sym!(FnObserve, b"mpv_observe_property\0"),
            wait_event: sym!(FnWaitEvent, b"mpv_wait_event\0"),
            _lib: lib,
        };
        Ok(out)
    }
}

/// Passed by raw pointer to mpv's render-update callback (invoked on an
/// arbitrary mpv thread). Only touches thread-safe primitives.
struct UpdateCtx {
    redraw: Arc<AtomicBool>,
    egui: egui::Context,
}

unsafe extern "C" fn update_cb(data: *mut c_void) {
    if data.is_null() {
        return;
    }
    let u = &*(data as *const UpdateCtx);
    u.redraw.store(true, Ordering::SeqCst);
    // Wake the UI so it renders the freshly-decoded frame promptly.
    u.egui.request_repaint();
}

/// Latest property values, updated asynchronously by the event-pump thread and
/// read (non-blocking) by the UI thread. **Never** read via a synchronous
/// `mpv_get_property` on the UI thread: during video-output init mpv's core
/// thread blocks in a render rendezvous, so a blocking property call from the
/// render/UI thread deadlocks (core waits for a render, UI waits for core).
#[derive(Default, Clone, Copy)]
struct Cached {
    time_pos: f64,
    duration: f64,
    width: u32,
    height: u32,
}

/// One video surface. Loads a local file, decodes on the CPU, and renders the
/// current frame into an RGBA buffer sized to the on-screen pane.
pub struct Player {
    lib: Arc<MpvLib>,
    handle: *mut c_void,
    rctx: *mut c_void,
    update_ctx: *mut UpdateCtx,
    redraw: Arc<AtomicBool>,
    buf: Vec<u8>,
    w: i32,
    h: i32,
    paused: bool,
    /// Async-updated time-pos/duration/size (see `Cached`).
    cached: Arc<Mutex<Cached>>,
    /// Set on Drop to stop the event-pump thread.
    ev_stop: Arc<AtomicBool>,
    ev_thread: Option<std::thread::JoinHandle<()>>,
}

impl Player {
    /// Create a paused player showing the first frame of `path` (a local file).
    /// `egui` is cloned so mpv can wake the UI when a new frame is ready.
    pub fn open(lib: Arc<MpvLib>, egui: &egui::Context, path: &str) -> Result<Player, String> {
        unsafe {
            let handle = (lib.create)();
            if handle.is_null() {
                return Err("mpv_create returned null".into());
            }

            let set_opt = |k: &str, v: &str| -> c_int {
                let ck = CString::new(k).unwrap();
                let cv = CString::new(v).unwrap();
                (lib.set_option_string)(handle, ck.as_ptr(), cv.as_ptr())
            };
            // CRITICAL: force the render-API video output. Without this, mpv
            // running inside an app with a live NSApplication (eframe) picks the
            // platform VO ("gpu"/cocoa) and opens its OWN native window — the
            // frames go there instead of our software-render buffer (so the
            // in-app pane stays black, an extra window/dock-icon appears, and it
            // won't close). `vo=libmpv` makes the render context the only VO.
            set_opt("vo", "libmpv");
            // No auto window under any circumstance.
            set_opt("force-window", "no");
            // Keep libmpv lean and predictable: no terminal spam, no config
            // files, no user scripts / key bindings / OSC.
            set_opt("terminal", "no");
            set_opt("msg-level", "all=no");
            set_opt("config", "no");
            set_opt("load-scripts", "no");
            set_opt("input-default-bindings", "no");
            set_opt("osc", "no");
            // GPU-decode with copy-back so big (Enhanced 4K) sources decode
            // fast yet still land in system memory for software rendering;
            // mpv falls back to pure software decode if unavailable.
            set_opt("hwdec", "auto-copy");
            // Exact seeks so scrubbing lands on the requested frame.
            set_opt("hr-seek", "yes");
            // Hold the last frame at EOF instead of unloading (preview should
            // freeze on the final frame, not go black).
            set_opt("keep-open", "yes");
            // Start showing the first frame, paused.
            set_opt("pause", "yes");

            if (lib.initialize)(handle) != 0 {
                (lib.terminate)(handle);
                return Err("mpv_initialize failed".into());
            }

            // Software render context.
            let api = CString::new("sw").unwrap();
            let cparams = [
                MpvRenderParam { type_: MPV_RENDER_PARAM_API_TYPE, data: api.as_ptr() as *mut c_void },
                MpvRenderParam { type_: MPV_RENDER_PARAM_INVALID, data: ptr::null_mut() },
            ];
            let mut rctx: *mut c_void = ptr::null_mut();
            let rc = (lib.render_create)(&mut rctx, handle, cparams.as_ptr());
            if rc != 0 || rctx.is_null() {
                (lib.terminate)(handle);
                return Err(format!("mpv_render_context_create failed (rc={})", rc));
            }

            let redraw = Arc::new(AtomicBool::new(true));
            let update_ctx = Box::into_raw(Box::new(UpdateCtx {
                redraw: redraw.clone(),
                egui: egui.clone(),
            }));
            (lib.render_set_update_cb)(rctx, Some(update_cb), update_ctx as *mut c_void);

            // Observe the properties we display so a background thread can keep
            // them cached — we must NOT read them via a blocking
            // mpv_get_property on the UI thread (deadlocks the render rendezvous).
            (lib.observe_property)(handle, 1, b"time-pos\0".as_ptr() as *const c_char, MPV_FORMAT_DOUBLE);
            (lib.observe_property)(handle, 2, b"duration\0".as_ptr() as *const c_char, MPV_FORMAT_DOUBLE);
            (lib.observe_property)(handle, 3, b"dwidth\0".as_ptr() as *const c_char, MPV_FORMAT_INT64);
            (lib.observe_property)(handle, 4, b"dheight\0".as_ptr() as *const c_char, MPV_FORMAT_INT64);

            let cached = Arc::new(Mutex::new(Cached::default()));
            let ev_stop = Arc::new(AtomicBool::new(false));
            let ev_thread = {
                // mpv's client API is thread-safe, so this dedicated event thread
                // can pump events concurrently with the UI thread's rendering.
                // Pass the handle as a usize address (Send) and cast back inside
                // — a raw pointer can't cross the thread boundary directly.
                let h_addr = handle as usize;
                let lib2 = lib.clone();
                let cached2 = cached.clone();
                let stop2 = ev_stop.clone();
                Some(std::thread::spawn(move || {
                    let h = h_addr as *mut c_void;
                    loop {
                        if stop2.load(Ordering::SeqCst) {
                            break;
                        }
                        // 0.1s timeout → we notice `stop` promptly. wait_event
                        // dequeues client events and never blocks on the core
                        // dispatch lock, so it can't deadlock during VO init.
                        let ev = unsafe { (lib2.wait_event)(h, 0.1) };
                        if ev.is_null() {
                            continue;
                        }
                        let ev = unsafe { &*ev };
                        if ev.event_id == MPV_EVENT_SHUTDOWN {
                            break;
                        }
                        if ev.event_id == MPV_EVENT_PROPERTY_CHANGE && !ev.data.is_null() {
                            let prop = unsafe { &*(ev.data as *const MpvEventProperty) };
                            if prop.data.is_null() || prop.name.is_null() {
                                continue;
                            }
                            let name = unsafe { std::ffi::CStr::from_ptr(prop.name) };
                            if let Ok(mut c) = cached2.lock() {
                                match name.to_bytes() {
                                    b"time-pos" if prop.format == MPV_FORMAT_DOUBLE => {
                                        c.time_pos = unsafe { *(prop.data as *const f64) };
                                    }
                                    b"duration" if prop.format == MPV_FORMAT_DOUBLE => {
                                        c.duration = unsafe { *(prop.data as *const f64) };
                                    }
                                    b"dwidth" if prop.format == MPV_FORMAT_INT64 => {
                                        let v = unsafe { *(prop.data as *const i64) };
                                        if v > 0 { c.width = v as u32; }
                                    }
                                    b"dheight" if prop.format == MPV_FORMAT_INT64 => {
                                        let v = unsafe { *(prop.data as *const i64) };
                                        if v > 0 { c.height = v as u32; }
                                    }
                                    _ => {}
                                }
                            }
                        }
                    }
                }))
            };

            let mut p = Player {
                lib,
                handle,
                rctx,
                update_ctx,
                redraw,
                buf: Vec::new(),
                w: 0,
                h: 0,
                paused: true,
                cached,
                ev_stop,
                ev_thread,
            };
            p.load(path)?;
            Ok(p)
        }
    }

    fn command(&self, args: &[&str]) -> c_int {
        let cstrs: Vec<CString> = args.iter().map(|a| CString::new(*a).unwrap()).collect();
        let mut argv: Vec<*const c_char> = cstrs.iter().map(|c| c.as_ptr()).collect();
        argv.push(ptr::null());
        unsafe { (self.lib.command)(self.handle, argv.as_ptr()) }
    }

    fn load(&mut self, path: &str) -> Result<(), String> {
        let rc = self.command(&["loadfile", path]);
        if rc != 0 {
            return Err(format!("loadfile failed (rc={})", rc));
        }
        Ok(())
    }

    fn set_flag(&self, name: &str, on: bool) {
        let cn = CString::new(name).unwrap();
        let mut v: c_int = if on { 1 } else { 0 };
        unsafe {
            (self.lib.set_property)(
                self.handle,
                cn.as_ptr(),
                MPV_FORMAT_FLAG,
                &mut v as *mut c_int as *mut c_void,
            );
        }
    }

    /// The video's on-screen dimensions (post-aspect), once decoded. `None`
    /// until then. Read from the async cache — never a blocking mpv call.
    pub fn video_size(&self) -> Option<(u32, u32)> {
        let c = self.cached.lock().ok()?;
        if c.width > 0 && c.height > 0 {
            Some((c.width, c.height))
        } else {
            None
        }
    }

    pub fn is_paused(&self) -> bool {
        self.paused
    }

    pub fn set_paused(&mut self, paused: bool) {
        self.paused = paused;
        self.set_flag("pause", paused);
    }

    pub fn toggle_pause(&mut self) {
        let p = !self.paused;
        self.set_paused(p);
    }

    pub fn seek_absolute(&mut self, secs: f64) {
        let s = format!("{:.3}", secs.max(0.0));
        self.command(&["seek", &s, "absolute+exact"]);
    }

    pub fn time_pos(&self) -> f64 {
        self.cached.lock().map(|c| c.time_pos).unwrap_or(0.0)
    }

    pub fn duration(&self) -> f64 {
        self.cached.lock().map(|c| c.duration).unwrap_or(0.0)
    }

    /// Render the current frame into the internal RGBA buffer if there is a new
    /// one (or the pane size changed). Returns `Some((w, h, rgba))` when a fresh
    /// frame was produced, else `None` (nothing changed → reuse the texture).
    pub fn poll_frame(&mut self, w: i32, h: i32) -> Option<(i32, i32, &[u8])> {
        if w < 2 || h < 2 {
            return None;
        }
        let has_update = unsafe { (self.lib.render_update)(self.rctx) } & MPV_RENDER_UPDATE_FRAME != 0;
        let size_changed = w != self.w || h != self.h;
        let dirty = self.redraw.swap(false, Ordering::SeqCst);
        if !(has_update || size_changed || dirty) {
            return None;
        }
        if size_changed || self.buf.len() != (w * h * 4) as usize {
            self.w = w;
            self.h = h;
            self.buf = vec![0u8; (w * h * 4) as usize];
        }
        let size: [c_int; 2] = [self.w, self.h];
        let fmt = CString::new("rgb0").unwrap();
        let stride: usize = (self.w * 4) as usize;
        let params = [
            MpvRenderParam { type_: MPV_RENDER_PARAM_SW_SIZE, data: size.as_ptr() as *mut c_void },
            MpvRenderParam { type_: MPV_RENDER_PARAM_SW_FORMAT, data: fmt.as_ptr() as *mut c_void },
            MpvRenderParam {
                type_: MPV_RENDER_PARAM_SW_STRIDE,
                data: &stride as *const usize as *mut c_void,
            },
            MpvRenderParam {
                type_: MPV_RENDER_PARAM_SW_POINTER,
                data: self.buf.as_mut_ptr() as *mut c_void,
            },
            MpvRenderParam { type_: MPV_RENDER_PARAM_INVALID, data: ptr::null_mut() },
        ];
        let rc = unsafe { (self.lib.render)(self.rctx, params.as_ptr()) };
        if rc != 0 {
            return None;
        }
        // "rgb0" leaves the 4th byte undefined/zero → force opaque alpha.
        for px in self.buf.chunks_exact_mut(4) {
            px[3] = 255;
        }
        Some((self.w, self.h, &self.buf))
    }
}

impl Drop for Player {
    fn drop(&mut self) {
        // Stop the event-pump thread and wait for it to exit *before* tearing
        // down the mpv handle it reads from (its 0.1s wait_event timeout means
        // this returns promptly).
        self.ev_stop.store(true, Ordering::SeqCst);
        if let Some(t) = self.ev_thread.take() {
            let _ = t.join();
        }
        unsafe {
            if !self.rctx.is_null() {
                // Unhook the callback before freeing so it can't fire mid-teardown.
                (self.lib.render_set_update_cb)(self.rctx, None, ptr::null_mut());
                (self.lib.render_free)(self.rctx);
                self.rctx = ptr::null_mut();
            }
            if !self.handle.is_null() {
                (self.lib.terminate)(self.handle);
                self.handle = ptr::null_mut();
            }
            if !self.update_ctx.is_null() {
                drop(Box::from_raw(self.update_ctx));
                self.update_ctx = ptr::null_mut();
            }
        }
    }
}
