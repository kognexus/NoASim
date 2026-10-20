//! Control: from estimate and reference to a motor torque.

use crate::config::NodeConfig;
use crate::estimate::Estimate;
use crate::reference::Reference;

/// Computes the motor torque for one cycle.
pub trait Controller {
    /// Torque at the output, N·m, before safety limits.
    fn torque(&mut self, estimate: &Estimate, reference: &Reference) -> f32;
}

/// Spring–damper on the motor side towards the reference, plus inertia feed-forward.
///
/// Feedback uses the motor-side angle (collocated with the torque), which keeps the loop
/// stable however compliant the gear and links are.
#[derive(Clone, Copy, Debug)]
pub struct PdController {
    stiffness: f32,
    damping: f32,
    inertia: f32,
}

impl PdController {
    /// Controller with the default gains from `config`.
    #[must_use]
    pub fn new(config: &NodeConfig) -> Self {
        Self { stiffness: config.stiffness, damping: config.damping, inertia: config.reflected_inertia }
    }
}

impl Controller for PdController {
    fn torque(&mut self, e: &Estimate, r: &Reference) -> f32 {
        self.stiffness * (r.position - e.motor_angle)
            + self.damping * (r.velocity - e.motor_velocity)
            + self.inertia * r.acceleration
    }
}
