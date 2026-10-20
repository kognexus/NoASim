//! Control: from estimate and reference to a motor torque.

use crate::config::NodeConfig;
use crate::estimate::Estimate;
use crate::filters::LowPass2;
use crate::reference::Reference;

/// Computes the motor torque for one cycle.
pub trait Controller {
    /// Torque at the output, N·m, before damping terms and safety limits.
    ///
    /// `feedforward` is torque the node already knows it needs (friction, gravity).
    fn torque(&mut self, estimate: &Estimate, reference: &Reference, feedforward: f32) -> f32;
    /// Forgets integral state (after a stop, or when the reference jumps).
    fn reset(&mut self);
}

/// Joint-space PID on the output encoder, plus rotor damping.
///
/// Position and damping act on the *output* (after the gear), as with any dual-encoder
/// joint. On a quasi-direct drive the reflected rotor inertia is tiny, so derivative gain
/// on the motor encoder is capped by the rotor–gear resonance (a few hundred hertz); the
/// output side is not. The damping signal is low-passed (second order) so its gain stays
/// at the arm's modes and not at that resonance. A small extra term damps the rotor
/// against the gear.
#[derive(Clone, Copy, Debug)]
pub struct JointController {
    stiffness: f32,
    damping: f32,
    integral_gain: f32,
    rotor_damping: f32,
    integral: f32,
    integral_limit: f32,
    dt: f32,
    velocity_filter: LowPass2,
}

impl JointController {
    /// Controller with the gains from `config`.
    #[must_use]
    pub fn new(config: &NodeConfig) -> Self {
        Self {
            stiffness: config.stiffness,
            damping: config.damping,
            integral_gain: config.integral_gain,
            rotor_damping: config.rotor_damping,
            integral: 0.0,
            integral_limit: config.peak_torque,
            dt: config.cycle_seconds(),
            velocity_filter: LowPass2::new(config.damping_lowpass_hz, config.cycle_seconds()),
        }
    }
}

impl Controller for JointController {
    fn torque(&mut self, e: &Estimate, r: &Reference, feedforward: f32) -> f32 {
        let error = r.position - e.output_angle;
        self.integral = (self.integral + self.integral_gain * self.stiffness * error * self.dt)
            .clamp(-self.integral_limit, self.integral_limit);
        let velocity = self.velocity_filter.update(e.output_velocity);
        self.stiffness * error
            + self.damping * (r.velocity - velocity)
            + self.rotor_damping * (e.output_velocity - e.motor_velocity)
            + self.integral
            + feedforward
    }

    fn reset(&mut self) {
        self.integral = 0.0;
    }
}
