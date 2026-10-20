//! Control logic of one Nexus actuator node.
//!
//! A node sees only two things: the frame of its own sensors, and the bus frames
//! addressed to it. Everything it computes (estimation, interpolation, control,
//! neighbour coordination, safety) happens here, behind small traits that are
//! composed into an [`Actuator`].
//!
//! The crate is `no_std` and allocation-free so it stays MCU-shaped; in the
//! simulation it runs as one WebAssembly instance per joint.

#![no_std]
#![forbid(unsafe_code)]

pub use nexus_proto::NodeId;

/// Readings of the node's own sensors for one bus cycle.
///
/// NX-01 placeholder: only the fields the spike needs. NX-05 replaces it with the full
/// frame (dual encoders, Hall, IMU, current, temperature).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SensorFrame {
    /// Motor-side angle referred to the joint output, in radians.
    pub motor_angle: f32,
    /// Motor-side velocity referred to the joint output, in rad/s.
    pub motor_velocity: f32,
}

/// What the node asks of its motor for one bus cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Command {
    /// Motor torque referred to the joint output, in N·m.
    pub torque: f32,
}

/// One actuator node.
///
/// NX-01 placeholder: a fixed-gain position hold, enough to measure the cost of ticking
/// one node per joint per cycle. NX-05 turns this into a composition of traits.
#[derive(Clone, Debug)]
pub struct Actuator {
    id: NodeId,
    target: f32,
    stiffness: f32,
    damping: f32,
}

impl Actuator {
    /// Creates a node that holds `target` with the given stiffness (N·m/rad) and damping
    /// (N·m·s/rad).
    #[must_use]
    pub const fn new(id: NodeId, target: f32, stiffness: f32, damping: f32) -> Self {
        Self {
            id,
            target,
            stiffness,
            damping,
        }
    }

    /// The node's bus address.
    #[must_use]
    pub const fn id(&self) -> NodeId {
        self.id
    }

    /// Runs one bus cycle.
    #[must_use]
    pub fn tick(&mut self, sensors: &SensorFrame) -> Command {
        let error = self.target - sensors.motor_angle;
        Command {
            torque: self.stiffness * error - self.damping * sensors.motor_velocity,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Actuator, NodeId, SensorFrame};

    #[test]
    fn holds_target_with_spring_and_damper() {
        let mut node = Actuator::new(NodeId(1), 1.0, 100.0, 2.0);
        let command = node.tick(&SensorFrame {
            motor_angle: 0.5,
            motor_velocity: 1.0,
        });
        assert!((command.torque - (100.0 * 0.5 - 2.0)).abs() < 1e-6);
    }
}
