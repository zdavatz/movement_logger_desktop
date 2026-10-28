//! Public surface for the stbox-viz examples and external consumers.
//! Exposes `board3d` (for the `board_debug` example) and `io` (the
//! Sens/Gps CSV loaders, used by the GUI's Session tab). Add other
//! modules here on demand.

pub mod board3d;
pub mod io;
