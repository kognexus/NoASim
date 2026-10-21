//! Impact detection (B2) and the comm-loss watchdog (B8).
//!
//! **Impact.** The evidence is the joint's *external torque*: the torque its gear actually
//! transmits (dual encoders) minus the torque the measured motion of everything beyond it
//! needs (the dynamic chain: mass times the specific force the IMUs measure, summed from
//! the tip inward, see `gravity.rs`). In free motion that stays within the joint's model
//! error; when the arm presses on something it grows, whether the contact is hard or soft.
//! It is low-passed and compared with the commissioned threshold or a multiple of the noise
//! floor learned while nothing happens, whichever is larger. A jolt at the housing IMU
//! (high-passed acceleration) is reported as supporting evidence only: on its own it cannot
//! tell a soft collision from the arm's own motion (NX-16 log).
//!
//! **Comm loss.** The host's cyclic frame arrives every cycle. If it stops for longer than
//! the watchdog allows, the node treats the bus as lost.

use crate::estimate::Estimate;
use crate::filters::{HighPass, LowPass};
use crate::vector::norm;
use nexus_proto::Impact;

/// High-pass corner of the IMU jolt signal, Hz.
const IMPACT_HIGHPASS_HZ: f32 = 30.0;
/// Low-pass corner of the effort signal, Hz.
const EFFORT_LOWPASS_HZ: f32 = 10.0;
/// Time constant of the learned noise floor, Hz (as a low-pass corner of the squared signal).
const NOISE_FLOOR_HZ: f32 = 0.5;
/// A detection needs the signal this many noise-floor RMS above zero.
const THRESHOLD_SIGMA: f32 = 8.0;
/// Absolute minimum thresholds, so a very quiet joint does not trip on nothing.
const MIN_JOLT: f32 = 4.0; // m/s²
/// Cycles after power-on before detection is armed (noise floor still learning).
const WARM_UP_CYCLES: u32 = 4000;

/// Learns the RMS of a signal while it is quiet.
#[derive(Clone, Copy, Debug)]
struct NoiseFloor {
    mean_square: LowPass,
}

impl NoiseFloor {
    fn new(dt: f32) -> Self {
        Self { mean_square: LowPass::new(NOISE_FLOOR_HZ, dt) }
    }

    fn rms(&self) -> f32 {
        libm::sqrtf(self.mean_square.value())
    }

    fn learn(&mut self, x: f32) {
        self.mean_square.update(x * x);
    }
}

/// Watches one joint for impacts.
#[derive(Clone, Copy, Debug)]
pub struct ImpactDetector {
    jolt: [HighPass; 3],
    effort: LowPass,
    jolt_floor: NoiseFloor,
    effort_floor: NoiseFloor,
    min_effort: f32,
    cycles: u32,
}

impl ImpactDetector {
    /// Detector at timestep `dt` for a joint that counts `min_effort` N·m of external
    /// torque as an impact at the least.
    #[must_use]
    pub fn new(dt: f32, min_effort: f32) -> Self {
        Self {
            jolt: [HighPass::new(IMPACT_HIGHPASS_HZ, dt); 3],
            effort: LowPass::new(EFFORT_LOWPASS_HZ, dt),
            jolt_floor: NoiseFloor::new(dt),
            effort_floor: NoiseFloor::new(dt),
            min_effort,
            cycles: 0,
        }
    }

    /// Feeds one cycle (the estimate and the external-torque estimate, N·m); returns what
    /// was felt if it crossed its threshold.
    pub fn update(&mut self, e: &Estimate, effort: f32) -> Option<Impact> {
        let jolt = norm([
            self.jolt[0].update(e.accel[0]),
            self.jolt[1].update(e.accel[1]),
            self.jolt[2].update(e.accel[2]),
        ]);
        let effort = libm::fabsf(self.effort.update(effort));
        self.cycles = self.cycles.saturating_add(1);
        let jolt_threshold = (THRESHOLD_SIGMA * self.jolt_floor.rms()).max(MIN_JOLT);
        let effort_threshold = (THRESHOLD_SIGMA * self.effort_floor.rms()).max(self.min_effort);
        // Learn the floors only from clearly quiet samples, so a building contact cannot
        // raise its own threshold.
        if jolt < 0.5 * jolt_threshold {
            self.jolt_floor.learn(jolt);
        }
        if effort < 0.5 * effort_threshold {
            self.effort_floor.learn(effort);
        }
        if effort <= effort_threshold || self.cycles < WARM_UP_CYCLES {
            return None;
        }
        let evidence = Impact::TORQUE | if jolt > jolt_threshold { Impact::IMU } else { 0 };
        Some(Impact { score: effort / effort_threshold, evidence })
    }
}

/// Counts cycles since the host was last heard.
#[derive(Clone, Copy, Debug, Default)]
pub struct Watchdog {
    last_heard: Option<u32>,
}

impl Watchdog {
    /// The host was heard at cycle `now`.
    pub fn feed(&mut self, now: u32) {
        self.last_heard = Some(now);
    }

    /// Whether the host has been silent for more than `limit` cycles (false before the
    /// host was ever heard: the node has not joined the bus yet).
    #[must_use]
    pub fn expired(&self, now: u32, limit: u32) -> bool {
        self.last_heard.is_some_and(|last| now.wrapping_sub(last) > limit)
    }
}

#[cfg(test)]
mod tests {
    use super::{ImpactDetector, Watchdog, WARM_UP_CYCLES};
    use crate::estimate::Estimate;

    fn quiet(k: u32) -> Estimate {
        #[allow(clippy::cast_precision_loss)]
        let wobble = 0.05 * libm::sinf(k as f32 * 0.37);
        Estimate { accel: [wobble, 0.0, 9.81 + wobble], gear_torque: 10.0 + wobble, ..Estimate::default() }
    }

    #[test]
    fn ignores_small_effort_and_catches_a_push() {
        let mut detector = ImpactDetector::new(0.00025, 6.0);
        for k in 0..WARM_UP_CYCLES + 4000 {
            // Free motion: the model explains all but a couple of N·m.
            #[allow(clippy::cast_precision_loss)]
            let residual = 2.0 * libm::sinf(k as f32 * 0.00025 * core::f32::consts::TAU);
            assert!(detector.update(&quiet(k), residual).is_none(), "false detection at cycle {k}");
        }
        // Contact: 30 N·m of unexplained effort builds up over a few milliseconds.
        let mut detected = None;
        for k in 0..200 {
            if let Some(impact) = detector.update(&quiet(k), 30.0) {
                detected = Some((k, impact));
                break;
            }
        }
        let (k, impact) = detected.expect("the push must be detected");
        assert!(k < 40, "took {k} cycles");
        assert!(impact.evidence & 2 != 0 && impact.score > 1.0);
    }

    #[test]
    fn watchdog_expires_only_after_the_host_was_heard_and_went_quiet() {
        let mut dog = Watchdog::default();
        assert!(!dog.expired(1000, 20));
        dog.feed(1000);
        assert!(!dog.expired(1020, 20));
        assert!(dog.expired(1021, 20));
    }
}
