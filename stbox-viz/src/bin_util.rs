//! Uniform time-bucket aggregation for display-rate reduction.
//!
//! The sensor CSV is at 100 Hz (138k samples over 23 min). Feeding that
//! directly into Plotly produces a 7 MB HTML that's slow to render. 100 ms
//! buckets (10 Hz display rate) give 10× smaller output while keeping the
//! ~1 Hz pump-stroke oscillation fully resolved.

#[allow(dead_code)] // Median used in phase-2 per-session rendering
pub enum Agg {
    Mean,
    Median,
}

/// Bin `values` (indexed 1:1 with `t_seconds`) into fixed-width buckets,
/// aggregating each bucket with `agg`. Returns (bucket_centre_s, agg_values)
/// sorted ascending by time.
///
/// Uses a `BTreeMap` keyed by bucket index so non-monotonic or duplicate
/// input times can't produce out-of-order or phantom output points — the
/// earlier "if b != last_bucket { push }" approach could emit the same
/// bucket twice and made Plotly draw ghost zig-zags past the data end
/// when a stray outlier tick landed late in the row stream.
pub fn bin_to_resolution(t_seconds: &[f64], values: &[f64], bucket_ms: u32, agg: Agg) -> (Vec<f64>, Vec<f64>) {
    use std::collections::BTreeMap;

    assert_eq!(t_seconds.len(), values.len());
    if t_seconds.is_empty() {
        return (Vec::new(), Vec::new());
    }

    let bucket_s = bucket_ms as f64 / 1000.0;
    let t0 = t_seconds[0];

    let mut buckets: BTreeMap<i64, Vec<f64>> = BTreeMap::new();
    for (&t, &v) in t_seconds.iter().zip(values.iter()) {
        if !t.is_finite() || !v.is_finite() { continue; }
        if t < t0 { continue; } // reject out-of-order stragglers
        let b = ((t - t0) / bucket_s).floor() as i64;
        buckets.entry(b).or_default().push(v);
    }

    let mut bucket_t = Vec::with_capacity(buckets.len());
    let mut out_vals = Vec::with_capacity(buckets.len());
    for (b, vs) in buckets {
        bucket_t.push(t0 + b as f64 * bucket_s);
        out_vals.push(match agg {
            Agg::Mean => vs.iter().sum::<f64>() / vs.len() as f64,
            Agg::Median => {
                let mut s = vs.clone();
                s.sort_by(|a, b| a.partial_cmp(b).unwrap());
                s[s.len() / 2]
            }
        });
    }
    (bucket_t, out_vals)
}
