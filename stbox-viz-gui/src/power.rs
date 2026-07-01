//! Keep the machine (and our own process) awake while a BLE link is up.
//!
//! This closes the single biggest gap between the desktop and the mobile
//! apps' sync stability. The mobile clients actively fight OS suspension
//! for the whole connected FileSync session — iOS holds a renewing
//! `UIApplication` background-task assertion (`BleClient.swift`), Android
//! runs a `FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE` foreground service
//! (`BleSyncService.kt`). The desktop did **nothing**: on macOS, App Nap
//! throttled the BLE worker whenever the window was occluded and the
//! system idle-slept on its own, either of which drops the link
//! mid-transfer — exactly during the long, unattended "Keep synced"
//! passes this app is built around.
//!
//! `PowerGuard::acquire()` takes an OS assertion that (a) prevents user-
//! idle system sleep and (b) opts this process out of App Nap; the
//! assertion is released when the guard is dropped. Everything here is
//! **best-effort** — a failure to take the assertion must never break
//! sync, so `acquire()` never returns an error, it just holds less.
//!
//! Held for the whole *session* including auto-reconnect windows (see
//! `ble.rs`): released only on an explicit Disconnect / abort / final
//! give-up, never in `disconnect_inner` — otherwise the Mac could sleep
//! in the gap between a drop and the reconnect and we'd never come back.

/// RAII sleep/App-Nap assertion. Dropping it releases the OS assertion.
pub struct PowerGuard(#[allow(dead_code)] Option<imp::Assertion>);

impl PowerGuard {
    /// Take the OS "stay awake / no App Nap" assertion. Best-effort: on
    /// an unsupported platform or an OS error this is a live no-op guard.
    pub fn acquire() -> Self {
        PowerGuard(imp::acquire())
    }
}

// ---------------------------------------------------------------------------
//  macOS — IOPMAssertion (idle-sleep) + NSProcessInfo activity (App Nap)
// ---------------------------------------------------------------------------
#[cfg(target_os = "macos")]
mod imp {
    use std::os::raw::{c_char, c_void};

    type CFStringRef = *const c_void;
    type CFAllocatorRef = *const c_void;
    type IOPMAssertionID = u32;
    type IOReturn = i32;

    const IOPM_ASSERTION_LEVEL_ON: u32 = 255;
    const CF_STRING_ENCODING_UTF8: u32 = 0x0800_0100;
    const IO_RETURN_SUCCESS: IOReturn = 0;

    // NSActivityUserInitiated = 0x00FFFFFF | NSActivityIdleSystemSleepDisabled
    // (defeats App Nap *and* prevents idle system sleep in one flag).
    const NS_ACTIVITY_USER_INITIATED: u64 = 0x00FF_FFFF | (1u64 << 20);

    #[allow(non_camel_case_types)]
    type id = *mut c_void;
    type SEL = *const c_void;
    type Class = *mut c_void;

    #[link(name = "CoreFoundation", kind = "framework")]
    extern "C" {
        fn CFStringCreateWithCString(
            alloc: CFAllocatorRef,
            c_str: *const c_char,
            encoding: u32,
        ) -> CFStringRef;
        fn CFRelease(cf: *const c_void);
    }

    #[link(name = "IOKit", kind = "framework")]
    extern "C" {
        fn IOPMAssertionCreateWithName(
            assertion_type: CFStringRef,
            assertion_level: u32,
            assertion_name: CFStringRef,
            assertion_id: *mut IOPMAssertionID,
        ) -> IOReturn;
        fn IOPMAssertionRelease(assertion_id: IOPMAssertionID) -> IOReturn;
    }

    #[link(name = "objc", kind = "dylib")]
    extern "C" {
        fn objc_getClass(name: *const c_char) -> Class;
        fn sel_registerName(name: *const c_char) -> SEL;
        fn objc_msgSend();
        fn objc_autoreleasePoolPush() -> *mut c_void;
        fn objc_autoreleasePoolPop(ctx: *mut c_void);
    }

    pub struct Assertion {
        /// IOKit idle-sleep assertion id, if we got one.
        pm_id: Option<IOPMAssertionID>,
        /// Retained NSProcessInfo activity token, if we got one.
        activity: id,
    }
    // The handles are just opaque ids/pointers we own exclusively and only
    // touch on acquire/drop; safe to move across threads.
    unsafe impl Send for Assertion {}

    pub fn acquire() -> Option<Assertion> {
        let pm_id = unsafe { pm_acquire() };
        let activity = unsafe { activity_acquire() };
        if pm_id.is_none() && activity.is_null() {
            return None;
        }
        Some(Assertion { pm_id, activity })
    }

    unsafe fn pm_acquire() -> Option<IOPMAssertionID> {
        let atype = CFStringCreateWithCString(
            std::ptr::null(),
            b"PreventUserIdleSystemSleep\0".as_ptr() as *const c_char,
            CF_STRING_ENCODING_UTF8,
        );
        let aname = CFStringCreateWithCString(
            std::ptr::null(),
            b"MovementLogger BLE FileSync\0".as_ptr() as *const c_char,
            CF_STRING_ENCODING_UTF8,
        );
        if atype.is_null() || aname.is_null() {
            if !atype.is_null() { CFRelease(atype); }
            if !aname.is_null() { CFRelease(aname); }
            return None;
        }
        let mut id: IOPMAssertionID = 0;
        let rc = IOPMAssertionCreateWithName(atype, IOPM_ASSERTION_LEVEL_ON, aname, &mut id);
        CFRelease(atype);
        CFRelease(aname);
        if rc == IO_RETURN_SUCCESS { Some(id) } else { None }
    }

    // [[NSProcessInfo processInfo] beginActivityWithOptions:reason:] returns
    // an autoreleased token; retain it so it outlives this call, and run the
    // whole thing inside an autorelease pool so the transient reason string
    // (and the token's autorelease slot) don't "leak with no pool in place".
    unsafe fn activity_acquire() -> id {
        let pool = objc_autoreleasePoolPush();
        let token = (|| {
            let cls = objc_getClass(b"NSProcessInfo\0".as_ptr() as *const c_char);
            if cls.is_null() { return std::ptr::null_mut(); }
            let sel_pi = sel_registerName(b"processInfo\0".as_ptr() as *const c_char);
            let msg0: extern "C" fn(Class, SEL) -> id =
                std::mem::transmute(objc_msgSend as *const c_void);
            let pi = msg0(cls, sel_pi);
            if pi.is_null() { return std::ptr::null_mut(); }

            let ns_string = objc_getClass(b"NSString\0".as_ptr() as *const c_char);
            let sel_str = sel_registerName(b"stringWithUTF8String:\0".as_ptr() as *const c_char);
            let msg_str: extern "C" fn(Class, SEL, *const c_char) -> id =
                std::mem::transmute(objc_msgSend as *const c_void);
            let reason = msg_str(
                ns_string,
                sel_str,
                b"MovementLogger BLE FileSync in progress\0".as_ptr() as *const c_char,
            );

            let sel_begin =
                sel_registerName(b"beginActivityWithOptions:reason:\0".as_ptr() as *const c_char);
            let msg_begin: extern "C" fn(id, SEL, u64, id) -> id =
                std::mem::transmute(objc_msgSend as *const c_void);
            let token = msg_begin(pi, sel_begin, NS_ACTIVITY_USER_INITIATED, reason);
            if token.is_null() { return std::ptr::null_mut(); }

            // Retain so it survives the pool pop below.
            let sel_retain = sel_registerName(b"retain\0".as_ptr() as *const c_char);
            let msg_retain: extern "C" fn(id, SEL) -> id =
                std::mem::transmute(objc_msgSend as *const c_void);
            msg_retain(token, sel_retain)
        })();
        objc_autoreleasePoolPop(pool);
        token
    }

    impl Drop for Assertion {
        fn drop(&mut self) {
            unsafe {
                if let Some(id) = self.pm_id.take() {
                    let _ = IOPMAssertionRelease(id);
                }
                if !self.activity.is_null() {
                    let pool = objc_autoreleasePoolPush();
                    let cls = objc_getClass(b"NSProcessInfo\0".as_ptr() as *const c_char);
                    if !cls.is_null() {
                        let sel_pi =
                            sel_registerName(b"processInfo\0".as_ptr() as *const c_char);
                        let msg0: extern "C" fn(Class, SEL) -> id =
                            std::mem::transmute(objc_msgSend as *const c_void);
                        let pi = msg0(cls, sel_pi);
                        if !pi.is_null() {
                            let sel_end =
                                sel_registerName(b"endActivity:\0".as_ptr() as *const c_char);
                            let msg_end: extern "C" fn(id, SEL, id) =
                                std::mem::transmute(objc_msgSend as *const c_void);
                            msg_end(pi, sel_end, self.activity);
                        }
                    }
                    // Balance our retain.
                    let sel_release = sel_registerName(b"release\0".as_ptr() as *const c_char);
                    let msg_release: extern "C" fn(id, SEL) =
                        std::mem::transmute(objc_msgSend as *const c_void);
                    msg_release(self.activity, sel_release);
                    self.activity = std::ptr::null_mut();
                    objc_autoreleasePoolPop(pool);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
//  Windows — SetThreadExecutionState
// ---------------------------------------------------------------------------
#[cfg(windows)]
mod imp {
    type EXECUTION_STATE = u32;
    const ES_CONTINUOUS: EXECUTION_STATE = 0x8000_0000;
    const ES_SYSTEM_REQUIRED: EXECUTION_STATE = 0x0000_0001;

    #[link(name = "kernel32")]
    extern "system" {
        fn SetThreadExecutionState(es_flags: EXECUTION_STATE) -> EXECUTION_STATE;
    }

    pub struct Assertion;
    // Acquire/drop are called on the same (BLE worker) thread; the marker
    // itself carries no thread-affine state.
    unsafe impl Send for Assertion {}

    pub fn acquire() -> Option<Assertion> {
        // ES_CONTINUOUS makes the flag sticky until the next call; combined
        // with ES_SYSTEM_REQUIRED it keeps the system awake for as long as
        // the guard lives. Reset to a bare ES_CONTINUOUS on drop.
        unsafe { SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED); }
        Some(Assertion)
    }

    impl Drop for Assertion {
        fn drop(&mut self) {
            unsafe { SetThreadExecutionState(ES_CONTINUOUS); }
        }
    }
}

// ---------------------------------------------------------------------------
//  Other platforms (Linux/BSD) — no-op. On Linux the mobile-vs-desktop
//  sync gap is MTU (BlueZ auto-negotiates it), not process suspension, so
//  there's nothing worthwhile to assert here without a systemd/D-Bus dep.
// ---------------------------------------------------------------------------
#[cfg(not(any(target_os = "macos", windows)))]
mod imp {
    pub struct Assertion;
    unsafe impl Send for Assertion {}
    pub fn acquire() -> Option<Assertion> { None }
}
