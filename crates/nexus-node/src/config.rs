//! What a node knows about itself before it ever talks to the bus: the contents of its
//! EEPROM. These are nominal (datasheet) values, not the actuator's true physics.

use nexus_proto::Announce;

/// Nominal parameters and default gains of one node.
///
/// `#[repr(C)]` and made of 4-byte fields only, so the harness can write it into the
/// node's memory before start-up.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NodeConfig {
    /// Bus cycle rate, Hz.
    pub cycle_hz: f32,
    /// Gear ratio.
    pub gear_ratio: f32,
    /// Continuous output torque, N·m.
    pub rated_torque: f32,
    /// Peak output torque, N·m.
    pub peak_torque: f32,
    /// Nominal reflected rotor inertia at the output, kg·m².
    pub reflected_inertia: f32,
    /// Nominal gear-stage stiffness, N·m/rad.
    pub gear_stiffness: f32,
    /// Torque constant at the output, N·m/A.
    pub output_torque_constant: f32,
    /// Winding temperature limit, °C.
    pub max_temperature: f32,
    /// Joint axis in the joint frame.
    pub axis: [f32; 3],
    /// Vector from this joint to the next, joint frame, m.
    pub link: [f32; 3],
    /// Nominal link mass (with the next housing), kg.
    pub link_mass: f32,
    /// Holding brake fitted: 1.0 or 0.0 (kept as `f32` for the flat layout).
    pub has_brake: f32,
    /// Nominal viscous friction at the output, N·m·s/rad.
    pub viscous_friction: f32,
    /// Nominal Coulomb friction at the output, N·m.
    pub coulomb_friction: f32,
    /// Joint stiffness of the position loop on the output encoder, N·m/rad.
    pub stiffness: f32,
    /// Joint damping on the output velocity, N·m·s/rad.
    pub damping: f32,
    /// Integral action as a fraction of stiffness per second, 1/s.
    pub integral_gain: f32,
    /// Damping of the rotor against the gear (motor minus output velocity), N·m·s/rad.
    pub rotor_damping: f32,
    /// Bandwidth of the encoder velocity observer, Hz.
    pub observer_hz: f32,
    /// Gain of joint-local (own IMU) damping, N·m·s/rad.
    pub local_damping: f32,
    /// Gain of network (neighbour IMU) damping, N·m·s/rad.
    pub network_damping: f32,
    /// High-pass corner of the damping signals, Hz: only vibration is damped, not motion.
    pub damping_highpass_hz: f32,
    /// Low-pass corner of the joint damping signal, Hz (second order).
    pub damping_lowpass_hz: f32,
    /// Low-pass corner of the IMU damping signals (local and network), Hz (second order):
    /// trades damping bandwidth against sensor noise at the tool.
    pub imu_lowpass_hz: f32,
    /// Version of the node's dynamics model.
    pub model_version: f32,
}

impl NodeConfig {
    /// Length of one bus cycle, s.
    #[must_use]
    pub fn cycle_seconds(&self) -> f32 {
        1.0 / self.cycle_hz
    }

    /// The node's discovery announcement for line position `position`.
    #[must_use]
    pub fn announce(&self, position: u8) -> Announce {
        Announce {
            position,
            has_brake: self.has_brake > 0.5,
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            model_version: self.model_version as u16,
            gear_ratio: self.gear_ratio,
            rated_torque: self.rated_torque,
            peak_torque: self.peak_torque,
            axis: self.axis,
            link: self.link,
            link_mass: self.link_mass,
        }
    }
}

impl NodeConfig {
    /// A KX-6 elbow-class configuration, for tests and examples.
    #[must_use]
    pub const fn example() -> Self {
        Self {
            cycle_hz: 4000.0,
            gear_ratio: 9.0,
            rated_torque: 30.0,
            peak_torque: 90.0,
            reflected_inertia: 0.0162,
            gear_stiffness: 6.0e4,
            output_torque_constant: 2.844,
            max_temperature: 130.0,
            axis: [0.0, 1.0, 0.0],
            link: [0.0, 0.0, 0.39],
            link_mass: 1.6,
            has_brake: 0.0,
            viscous_friction: 0.3,
            coulomb_friction: 0.4,
            stiffness: 3000.0,
            damping: 100.0,
            integral_gain: 2.0,
            rotor_damping: 5.0,
            observer_hz: 150.0,
            local_damping: 40.0,
            network_damping: 40.0,
            damping_highpass_hz: 1.0,
            damping_lowpass_hz: 1000.0,
            imu_lowpass_hz: 40.0,
            model_version: 1.0,
        }
    }
}
