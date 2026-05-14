//! Short-time Fourier transform for pump-cadence analysis.
//! Emulates `scipy.signal.spectrogram` with `scaling='spectrum'` and a Hann
//! window. Parameters match the Python pipeline:
//!   nperseg = 4 · fs, noverlap = 0.9 · nperseg.

use rustfft::{FftPlanner, num_complex::Complex};

pub struct Spectrogram {
    /// Bin centre frequencies [Hz], length F.
    pub freqs: Vec<f64>,
    /// Segment centre times [s], length T.
    pub times: Vec<f64>,
    /// Power spectrum, row-major [F][T].
    pub power: Vec<Vec<f64>>,
}

/// Hann window of length N.
fn hann(n: usize) -> Vec<f64> {
    if n < 2 {
        return vec![1.0; n];
    }
    (0..n).map(|i| {
        0.5 * (1.0 - (2.0 * std::f64::consts::PI * i as f64 / (n - 1) as f64).cos())
    }).collect()
}

pub fn spectrogram(x: &[f64], fs: f64) -> Spectrogram {
    let nperseg = (fs * 4.0).round() as usize;
    let noverlap = (nperseg as f64 * 0.9) as usize;
    let step = nperseg - noverlap;
    if x.len() < nperseg || step == 0 {
        return Spectrogram {
            freqs: vec![],
            times: vec![],
            power: vec![],
        };
    }

    let win = hann(nperseg);
    let win_sum: f64 = win.iter().sum();
    let scale = 1.0 / (win_sum * win_sum);

    // Positive-frequency bins: 0 .. nperseg/2 inclusive
    let n_freqs = nperseg / 2 + 1;
    let freqs: Vec<f64> = (0..n_freqs).map(|k| k as f64 * fs / nperseg as f64).collect();

    let mut planner = FftPlanner::<f64>::new();
    let fft = planner.plan_fft_forward(nperseg);

    let mut power: Vec<Vec<f64>> = vec![Vec::new(); n_freqs];
    let mut times: Vec<f64> = Vec::new();

    let mut start = 0usize;
    while start + nperseg <= x.len() {
        let seg = &x[start..start + nperseg];
        // Detrend (remove mean) + window
        let mean: f64 = seg.iter().sum::<f64>() / seg.len() as f64;
        let mut buf: Vec<Complex<f64>> = seg.iter().zip(win.iter())
            .map(|(&v, &w)| Complex::new((v - mean) * w, 0.0))
            .collect();
        fft.process(&mut buf);
        for k in 0..n_freqs {
            let re = buf[k].re;
            let im = buf[k].im;
            power[k].push((re * re + im * im) * scale);
        }
        times.push((start + nperseg / 2) as f64 / fs);
        start += step;
    }

    Spectrogram { freqs, times, power }
}
