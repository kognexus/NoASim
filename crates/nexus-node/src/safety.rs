//! Safety supervision: the last stage before the motor, enforcing the host's limits.

use crate::estimate::Estimate;
use nexus_proto::Limits;

/// Enforces limits on the torque the node is about to apply.
pub trait SafetySupervisor {
    /// Returns the torque that may be applied this cycle.
    fn limit(&mut self, torque: f32, estimate: &Estimate, limits: &Limits) -> f32;
}

/// Clamps torque to the host's limit. NX-18 adds speed, position and thermal limits.
#[derive(Clone, Copy, Debug, Default)]
pub struct TorqueLimiter;

impl SafetySupervisor for TorqueLimiter {
    fn limit(&mut self, torque: f32, _estimate: &Estimate, limits: &Limits) -> f32 {
        torque.clamp(-limits.max_torque, limits.max_torque)
    }
}
