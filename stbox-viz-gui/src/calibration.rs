//! Box-persisted board-orientation calibration blob (v0.0.37+).
//!
//! Wire format + semantics: firmware `DESIGN.md` →
//! *Box-persisted calibration (`CAL_GET` / `CAL_SET`)*. Mirrored in the
//! iOS + Android apps so a "Zero here" / nosePlusY / heading-bias set on
//! ANY host survives on the next connect from a different one.
//!
//! Two responsibilities:
//! - `decode(blob)` — parse a 32-byte CAL_GET reply into per-field
//!   `Option`s. `None` on any field means "the box has NOT set this yet
//!   (valid_mask bit clear)" — the host should fall back to its local
//!   `AgentConfig` / defaults.
//! - `encode(fields)` — build a 32-byte payload for CAL_SET. Only fields
//!   passed as `Some(_)` have their `valid_mask` bit set; the box's merge
//!   leaves unset fields alone so a host can push a single new value
//!   without knowing the box's current others.

pub const BLOB_SIZE: usize = 32;
pub const LAYOUT_VERSION: u8 = 0x01;

pub const MASK_NOSE_PLUS_Y: u8 = 0x01;
pub const MASK_MAG_OFFSET: u8 = 0x02;
pub const MASK_ANGLE_ZERO: u8 = 0x04;
pub const MASK_HEADING_BIAS: u8 = 0x08;

/// The decoded calibration as the app models it. `None` per field means
/// "box didn't have this yet" — the local `AgentConfig` value stands.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DecodedCal {
    pub nose_plus_y: Option<bool>,
    /// Hard-iron offset in mg per axis (X, Y, Z).
    pub mag_offset_mg: Option<[f64; 3]>,
    /// [pitch, roll, yaw] in degrees.
    pub angle_zero_ref: Option<[f64; 3]>,
    /// Unix epoch ms when "Zero here" was captured; `None` if never zeroed.
    /// Distinct from `angle_zero_ref` being `None` (which means the whole
    /// zero-tare bit is clear) — this one is `None` when the bit IS set
    /// but the box has no wall-clock stamp yet.
    pub angle_zero_at_epoch_ms: Option<i64>,
    pub heading_bias_deg: Option<f64>,
}

/// Fields to include in a `CAL_SET` write. Any `Some(_)` sets its
/// valid_mask bit; the box's merge overwrites just those fields.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct EncodeInput {
    pub nose_plus_y: Option<bool>,
    pub mag_offset_mg: Option<[f64; 3]>,
    /// Pair the ref with the epoch — a `Some(ref)` implies "the zero-tare
    /// bit is being set" so the epoch (defaults to 0 if not passed)
    /// travels alongside in the same 8-byte slot.
    pub angle_zero_ref: Option<[f64; 3]>,
    pub angle_zero_at_epoch_ms: Option<i64>,
    pub heading_bias_deg: Option<f64>,
}

/// Encode a partial-update CAL_SET payload. Returns exactly `BLOB_SIZE`
/// bytes (any field passed as `None` is zero-filled AND its valid_mask
/// bit is cleared, so the box's merge leaves that field untouched).
pub fn encode(input: &EncodeInput) -> [u8; BLOB_SIZE] {
    let mut b = [0u8; BLOB_SIZE];
    b[0] = LAYOUT_VERSION;

    let mut mask = 0u8;

    if let Some(pos) = input.nose_plus_y {
        mask |= MASK_NOSE_PLUS_Y;
        b[2] = if pos { 1 } else { 0 };
    }
    if let Some(mo) = input.mag_offset_mg {
        mask |= MASK_MAG_OFFSET;
        for (i, v) in mo.iter().enumerate() {
            let vi = clamp_i16(*v);
            let off = 4 + i * 2;
            b[off..off + 2].copy_from_slice(&vi.to_le_bytes());
        }
    }
    if let Some(zr) = input.angle_zero_ref {
        mask |= MASK_ANGLE_ZERO;
        for (i, v) in zr.iter().enumerate() {
            // Tenths of a degree: ±3276.7° range, plenty.
            let vi = clamp_i16(v * 10.0);
            let off = 10 + i * 2;
            b[off..off + 2].copy_from_slice(&vi.to_le_bytes());
        }
        // Epoch travels in the same "zero-tare" bit. Missing epoch → 0
        // (the layout's own "never zeroed" sentinel), which is
        // deliberately DIFFERENT from "there IS an epoch = 0".
        let epoch = input.angle_zero_at_epoch_ms.unwrap_or(0);
        // The wire is unsigned u64; a negative epoch (nonsense but
        // defensive) is stored as its two's-complement bit pattern.
        let e = epoch as u64;
        b[16..24].copy_from_slice(&e.to_le_bytes());
    }
    if let Some(bd) = input.heading_bias_deg {
        mask |= MASK_HEADING_BIAS;
        let vi = clamp_i16(bd * 10.0);
        b[24..26].copy_from_slice(&vi.to_le_bytes());
    }

    b[1] = mask;
    b
}

/// Decode a 32-byte `CAL_GET` reply. Fields whose valid_mask bit is
/// clear come back as `None` — caller falls back to its own local
/// `AgentConfig`. Returns `Err` on the two malformed cases (short blob
/// or unknown layout version) — legacy firmware doesn't reply at all,
/// so we don't see stale layouts here.
pub fn decode(blob: &[u8]) -> Result<DecodedCal, String> {
    if blob.len() != BLOB_SIZE {
        return Err(format!(
            "CAL_GET: bad blob length {} (expected {BLOB_SIZE})",
            blob.len()
        ));
    }
    if blob[0] != LAYOUT_VERSION {
        return Err(format!(
            "CAL_GET: unknown layout version 0x{:02X} (expected 0x{:02X})",
            blob[0], LAYOUT_VERSION
        ));
    }
    let mask = blob[1];
    let mut d = DecodedCal::default();

    if mask & MASK_NOSE_PLUS_Y != 0 {
        d.nose_plus_y = Some(blob[2] != 0);
    }
    if mask & MASK_MAG_OFFSET != 0 {
        let mut a = [0f64; 3];
        for i in 0..3 {
            let off = 4 + i * 2;
            let v = i16::from_le_bytes([blob[off], blob[off + 1]]);
            a[i] = v as f64;
        }
        d.mag_offset_mg = Some(a);
    }
    if mask & MASK_ANGLE_ZERO != 0 {
        let mut a = [0f64; 3];
        for i in 0..3 {
            let off = 10 + i * 2;
            let v = i16::from_le_bytes([blob[off], blob[off + 1]]);
            a[i] = v as f64 / 10.0; // tenths → degrees
        }
        d.angle_zero_ref = Some(a);
        let e = u64::from_le_bytes([
            blob[16], blob[17], blob[18], blob[19], blob[20], blob[21], blob[22], blob[23],
        ]);
        // 0 = "the zero-tare bit is set but the box has no wall-clock
        // stamp yet" — treat as `None` so the UI can decide whether to
        // display a "zeroed just now" note or omit it.
        d.angle_zero_at_epoch_ms = if e == 0 { None } else { Some(e as i64) };
    }
    if mask & MASK_HEADING_BIAS != 0 {
        let v = i16::from_le_bytes([blob[24], blob[25]]);
        d.heading_bias_deg = Some(v as f64 / 10.0);
    }
    Ok(d)
}

fn clamp_i16(x: f64) -> i16 {
    let x = x.round();
    if x >= i16::MAX as f64 {
        i16::MAX
    } else if x <= i16::MIN as f64 {
        i16::MIN
    } else {
        x as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_all_fields() {
        let input = EncodeInput {
            nose_plus_y: Some(true),
            mag_offset_mg: Some([100.0, -50.0, 12.0]),
            angle_zero_ref: Some([-1.5, 12.3, 45.7]),
            angle_zero_at_epoch_ms: Some(1_700_000_000_000),
            heading_bias_deg: Some(90.4),
        };
        let blob = encode(&input);
        assert_eq!(blob[0], LAYOUT_VERSION);
        assert_eq!(blob[1], 0x0F); // all four bits
        let d = decode(&blob).unwrap();
        assert_eq!(d.nose_plus_y, Some(true));
        assert_eq!(d.mag_offset_mg, Some([100.0, -50.0, 12.0]));
        // Tenths quantization: 12.3 stays exact, 45.7 stays exact.
        assert_eq!(d.angle_zero_ref, Some([-1.5, 12.3, 45.7]));
        assert_eq!(d.angle_zero_at_epoch_ms, Some(1_700_000_000_000));
        assert_eq!(d.heading_bias_deg, Some(90.4));
    }

    #[test]
    fn partial_only_sets_its_bit() {
        let input = EncodeInput {
            nose_plus_y: Some(false),
            ..Default::default()
        };
        let blob = encode(&input);
        assert_eq!(blob[1], MASK_NOSE_PLUS_Y);
        let d = decode(&blob).unwrap();
        assert_eq!(d.nose_plus_y, Some(false));
        assert_eq!(d.mag_offset_mg, None);
        assert_eq!(d.angle_zero_ref, None);
        assert_eq!(d.heading_bias_deg, None);
    }

    #[test]
    fn empty_blob_is_uncalibrated() {
        let blob = [0u8; BLOB_SIZE];
        // Version 0 → error path (legacy firmware wouldn't reply at all,
        // so a version-0 blob means we misparsed).
        assert!(decode(&blob).is_err());
        // Fresh RAM blob from the firmware has version=1 + mask=0.
        let mut fresh = [0u8; BLOB_SIZE];
        fresh[0] = LAYOUT_VERSION;
        let d = decode(&fresh).unwrap();
        assert_eq!(d, DecodedCal::default());
    }
}
