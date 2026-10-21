//! Small discrete-time filters used by the node.

use core::f32::consts::PI;

/// First-order low-pass filter.
#[derive(Clone, Copy, Debug)]
pub struct LowPass {
    alpha: f32,
    value: Option<f32>,
}

impl LowPass {
    /// Corner `cutoff_hz` at timestep `dt`.
    #[must_use]
    pub fn new(cutoff_hz: f32, dt: f32) -> Self {
        Self { alpha: Self::alpha(cutoff_hz, dt), value: None }
    }

    fn alpha(cutoff_hz: f32, dt: f32) -> f32 {
        let rc = 1.0 / (2.0 * PI * cutoff_hz);
        dt / (rc + dt)
    }

    /// Moves the corner to `cutoff_hz`, keeping the filter's state.
    pub fn set_cutoff(&mut self, cutoff_hz: f32, dt: f32) {
        self.alpha = Self::alpha(cutoff_hz, dt);
    }

    /// Filters one sample.
    pub fn update(&mut self, x: f32) -> f32 {
        let y = self.value.map_or(x, |y| y + self.alpha * (x - y));
        self.value = Some(y);
        y
    }

    /// The current output.
    #[must_use]
    pub fn value(&self) -> f32 {
        self.value.unwrap_or(0.0)
    }
}

/// Second-order (critically damped) low-pass: two first-order stages in series. Rolls off
/// 40 dB/decade, so damping signals keep their gain at the arm's modes (a few hertz) but
/// lose it at the rotor–gear resonance (hundreds of hertz).
#[derive(Clone, Copy, Debug)]
pub struct LowPass2 {
    first: LowPass,
    second: LowPass,
}

impl LowPass2 {
    /// Corner `cutoff_hz` per stage at timestep `dt`.
    #[must_use]
    pub fn new(cutoff_hz: f32, dt: f32) -> Self {
        Self { first: LowPass::new(cutoff_hz, dt), second: LowPass::new(cutoff_hz, dt) }
    }

    /// Moves both corners to `cutoff_hz`, keeping the filter's state.
    pub fn set_cutoff(&mut self, cutoff_hz: f32, dt: f32) {
        self.first.set_cutoff(cutoff_hz, dt);
        self.second.set_cutoff(cutoff_hz, dt);
    }

    /// Filters one sample.
    pub fn update(&mut self, x: f32) -> f32 {
        let y = self.first.update(x);
        self.second.update(y)
    }
    /// The current output.
    #[must_use]
    pub fn value(&self) -> f32 {
        self.second.value()
    }
}

/// First-order high-pass filter: passes vibration, removes slow motion and offsets.
#[derive(Clone, Copy, Debug)]
pub struct HighPass {
    alpha: f32,
    last_input: f32,
    output: f32,
    started: bool,
}

impl HighPass {
    /// Corner `cutoff_hz` at timestep `dt`.
    #[must_use]
    pub fn new(cutoff_hz: f32, dt: f32) -> Self {
        let rc = 1.0 / (2.0 * PI * cutoff_hz);
        Self { alpha: rc / (rc + dt), last_input: 0.0, output: 0.0, started: false }
    }

    /// Filters one sample.
    pub fn update(&mut self, x: f32) -> f32 {
        if !self.started {
            self.last_input = x;
            self.started = true;
        }
        self.output = self.alpha * (self.output + x - self.last_input);
        self.last_input = x;
        self.output
    }

    /// Clears the filter state (e.g. when a mode is switched on).
    pub fn reset(&mut self) {
        self.started = false;
        self.output = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::{HighPass, LowPass};

    #[test]
    fn low_pass_settles_to_dc_and_high_pass_removes_it() {
        let mut lp = LowPass::new(10.0, 0.001);
        let mut hp = HighPass::new(1.0, 0.001);
        let (mut l, mut h) = (0.0, 1.0);
        for _ in 0..5000 {
            l = lp.update(2.0);
            h = hp.update(2.0);
        }
        assert!((l - 2.0).abs() < 1e-4);
        assert!(h.abs() < 1e-4);
    }
}
