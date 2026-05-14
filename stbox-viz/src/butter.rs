//! Butterworth low-pass filter + zero-phase `filtfilt`.
//!
//! scipy.signal.butter + filtfilt port for the board animation's 2 Hz
//! smoothing step. 4th-order design in the analog s-plane via pole
//! placement, bilinear-transformed to z-plane, then direct-form I IIR.
//!
//! `filtfilt` runs the filter forward then backward (reversed input)
//! to cancel the phase response — the amplitude response squares so
//! effective order = 2N, but the output timing matches the input.

use std::f64::consts::PI;

/// Design a 4th-order Butterworth low-pass filter normalised to `cutoff_hz`
/// at sampling rate `fs_hz`. Returns `(b, a)` — feedforward / feedback
/// coefficients of the same length (5 each).
pub fn butter4_lowpass(cutoff_hz: f64, fs_hz: f64) -> ([f64; 5], [f64; 5]) {
    const N: usize = 4;
    // Pre-warp cutoff for the bilinear transform
    let wd = 2.0 * PI * cutoff_hz;
    let wa = 2.0 * (wd / (2.0 * fs_hz)).tan() * fs_hz;

    // Butterworth analog prototype poles on the left half of the unit circle
    // p_k = exp(j·π·(2k + N + 1)/(2N)) for k = 0..N-1, scaled by wa.
    let mut poles_real = [0.0f64; N];
    let mut poles_imag = [0.0f64; N];
    for k in 0..N {
        let theta = PI * (2 * k + N + 1) as f64 / (2.0 * N as f64);
        poles_real[k] = wa * theta.cos();
        poles_imag[k] = wa * theta.sin();
    }

    // Bilinear transform: z = (2·fs + s) / (2·fs − s)
    // Each analog pole p → digital pole (2·fs + p) / (2·fs − p)
    let mut z_poles_r = [0.0f64; N];
    let mut z_poles_i = [0.0f64; N];
    let k2 = 2.0 * fs_hz;
    for k in 0..N {
        let (pr, pi) = (poles_real[k], poles_imag[k]);
        let num_r = k2 + pr;
        let num_i = pi;
        let den_r = k2 - pr;
        let den_i = -pi;
        // Complex divide
        let denom = den_r * den_r + den_i * den_i;
        z_poles_r[k] = (num_r * den_r + num_i * den_i) / denom;
        z_poles_i[k] = (num_i * den_r - num_r * den_i) / denom;
    }

    // All zeros at z = -1 (Nyquist) for a low-pass → (1 + z⁻¹)^N
    // Denominator = prod(1 - z_pole·z⁻¹) — expand via complex polynomial
    // multiplication.
    let mut a_poly: Vec<(f64, f64)> = vec![(1.0, 0.0)];
    for k in 0..N {
        // Multiply a_poly by (1 − p·z⁻¹) i.e. polynomial coeffs [1, -p]
        let mut next = vec![(0.0, 0.0); a_poly.len() + 1];
        for (i, &(r, im)) in a_poly.iter().enumerate() {
            // Contribution from coefficient 1 at z⁻i → next[i] += 1*coef
            next[i].0 += r;
            next[i].1 += im;
            // Contribution from coefficient -p at z⁻(i+1) → next[i+1] += -p*coef
            let pr = z_poles_r[k];
            let pi = z_poles_i[k];
            // (-p) * (r + im·j) = (−pr·r + pi·im) + (−pr·im − pi·r)·j
            next[i + 1].0 += -pr * r + pi * im;
            next[i + 1].1 += -pr * im - pi * r;
        }
        a_poly = next;
    }
    // Denominator must be purely real (pole pairs are conjugates).
    let mut a = [0.0f64; 5];
    for i in 0..5 {
        a[i] = a_poly[i].0;
    }

    // Numerator (1 + z⁻¹)^4 binomial coefficients
    let mut b = [1.0, 4.0, 6.0, 4.0, 1.0];

    // Normalise gain to unity at DC: eval H(z=1) = sum(b)/sum(a), set b *= sum(a)/sum(b)
    let sum_b: f64 = b.iter().sum();
    let sum_a: f64 = a.iter().sum();
    let gain = sum_a / sum_b;
    for coeff in &mut b {
        *coeff *= gain;
    }

    (b, a)
}

/// Direct-form I IIR filter. `y[n] = (sum b[k]·x[n-k] − sum a[k]·y[n-k]) / a[0]`.
fn lfilter(b: &[f64; 5], a: &[f64; 5], x: &[f64]) -> Vec<f64> {
    let n = x.len();
    let mut y = vec![0.0; n];
    for i in 0..n {
        let mut acc = 0.0;
        for k in 0..5 {
            if i >= k {
                acc += b[k] * x[i - k];
            }
        }
        for k in 1..5 {
            if i >= k {
                acc -= a[k] * y[i - k];
            }
        }
        y[i] = acc / a[0];
    }
    y
}

/// Zero-phase filtering: forward-then-backward pass. No padding; good
/// enough for nose-angle smoothing over 30+ s sessions where edge
/// transients (~20 ms) are invisible.
pub fn filtfilt(b: &[f64; 5], a: &[f64; 5], x: &[f64]) -> Vec<f64> {
    let forward = lfilter(b, a, x);
    let reversed: Vec<f64> = forward.into_iter().rev().collect();
    let backward = lfilter(b, a, &reversed);
    backward.into_iter().rev().collect()
}
