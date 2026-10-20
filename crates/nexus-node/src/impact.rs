//! Impact detection (B2) and the comm-loss watchdog (B8).
//!
//! **Impact.** A collision is a jolt: a sudden change of acceleration at the housing and
//! of the torque through the gear. Both signals are high-passed so that commanded motion,
//! which is jerk-limited, leaves them small, and each is compared with a noise floor the
//! node learns while nothing happens. A detection is reported with how far above its
//! threshold it was.
//!
//! **Comm loss.** The host's cyclic frame arrives every cycle. If it stops for longer than
//! the watchdog allows, the node treats the bus as lost.

use crate::estimate::Estimate;
use crate::filters::{HighPass, LowPass};
use crate::vector::norm;
use nexus_proto::Impact;

/// High-pass corner of the impact signals, Hz: well above commanded motion.
const IMPACT_HIGHPASS_HZ: f32 = 30.0;
/// Time constant of the learned noise floor, Hz (as a low-pass corner of the squared signal).
const NOISE_FLOOR_HZ: f32 = 0.5;
/// A detection needs the signal this many noise-floor RMS above zero.
const THRESHOLD_SIGMA: f32 = 8.0;
/// Absolute minimum thresholds, so a very quiet joint does not trip on nothing.
const MIN_JOLT: f32 = 4.0; // m/s²
const MIN_TORQUE: f32 = 3.0; // N·m
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
    torque: HighPass,
    jolt_floor: NoiseFloor,
    torque_floor: NoiseFloor,
    cycles: u32,
}

impl ImpactDetector {
    /// Detector at timestep `dt`.
    #[must_use]
    pub fn new(dt: f32) -> Self {
        Self {
            jolt: [HighPass::new(IMPACT_HIGHPASS_HZ, dt); 3],
            torque: HighPass::new(IMPACT_HIGHPASS_HZ, dt),
            jolt_floor: NoiseFloor::new(dt),
            torque_floor: NoiseFloor::new(dt),
            cycles: 0,
        }
    }

    /// Feeds one cycle; returns what was felt if it crossed a threshold.
    pub fn update(&mut self, e: &Estimate) -> Option<Impact> {
        let jolt = norm([
            self.jolt[0].update(e.accel[0]),
            self.jolt[1].update(e.accel[1]),
            self.jolt[2].update(e.accel[2]),
        ]);
        let torque = libm::fabsf(self.torque.update(e.gear_torque));
        self.cycles = self.cycles.saturating_add(1);
        let jolt_threshold = (THRESHOLD_SIGMA * self.jolt_floor.rms()).max(MIN_JOLT);
        let torque_threshold = (THRESHOLD_SIGMA * self.torque_floor.rms()).max(MIN_TORQUE);
        let mut evidence = 0;
        if jolt > jolt_threshold {
            evidence |= Impact::IMU;
        }
        if torque > torque_threshold {
            evidence |= Impact::TORQUE;
        }
        if evidence == 0 || self.cycles < WARM_UP_CYCLES {
            self.jolt_floor.learn(jolt);
            self.torque_floor.learn(torque);
            return None;
        }
        Some(Impact { score: (jolt / jolt_threshold).max(torque / torque_threshold), evidence })
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
    fn ignores_noise_and_slow_motion_and_catches_a_jolt() {
        let mut detector = ImpactDetector::new(0.00025);
        for k in 0..WARM_UP_CYCLES + 4000 {
            // Slow commanded motion: gear torque swinging 20 N·m at 1 Hz.
            #[allow(clippy::cast_precision_loss)]
            let slow = 20.0 * libm::sinf(k as f32 * 0.00025 * core::f32::consts::TAU);
            let mut e = quiet(k);
            e.gear_torque += slow;
            assert!(detector.update(&e).is_none(), "false detection at cycle {k}");
        }
        let mut hit = quiet(0);
        hit.accel[0] += 60.0;
        hit.gear_torque += 40.0;
        let impact = detector.update(&hit).expect("the jolt must be detected");
        assert_eq!(impact.evidence, 3);
        assert!(impact.score > 1.0);
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
