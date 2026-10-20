//! Vibration damping on top of the position loop (B1a, B5).
//!
//! Damping acts on the *absolute* angular velocity about the joint axis, high-pass
//! filtered so it only sees vibration, never commanded motion. What differs between
//! modes is where that velocity is measured:
//!
//! - `Local`: at the joint's own output, from its own IMU (housing rate) plus its output
//!   encoder. The joint alone cannot see its link bending.
//! - `Network` (NX-11): also at the far end of its link, from the outer neighbour's IMU.
//!
//! Damping from absolute rates is not collocated, so on its own it is not guaranteed to
//! be passive: two joints damping across a flexible link can pump energy into each other.
//! Every damping torque therefore passes an [`EnergyTank`]: it may only return to the joint
//! energy it has previously taken out (B5), so the joint port stays passive whatever the
//! damping signal is.

use crate::config::NodeConfig;
use crate::estimate::Estimate;
use crate::filters::{HighPass, LowPass2};
use crate::neighbours::{Neighbourhood, Side};
use nexus_proto::DampingMode;

/// What a damper may look at besides its own estimate.
pub struct DampingContext<'a> {
    /// The node's neighbours and their latest states.
    pub neighbours: &'a Neighbourhood,
    /// Current bus cycle.
    pub now: u32,
}

/// Adds a damping torque.
pub trait Damper {
    /// Damping torque this cycle for `mode`, N·m (before limits).
    fn torque(&mut self, mode: DampingMode, estimate: &Estimate, context: &DampingContext) -> f32;
    /// Energy left in the passivity tank, J (published in the node state).
    fn tank_energy(&self) -> f32;
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Passivity guard on a damping torque.
///
/// Power the damping torque delivers to the joint is `τ·ω` (output velocity). When it is
/// negative the torque removes energy and the tank fills (up to `capacity`); when it is
/// positive the torque may only run while the tank has energy left, and is cut otherwise.
/// So `∫ τ·ω dt ≤ capacity` at all times: the damping term never adds net energy.
#[derive(Clone, Copy, Debug)]
pub struct EnergyTank {
    energy: f32,
    capacity: f32,
    dt: f32,
}

impl EnergyTank {
    /// Empty tank holding at most `capacity` joules.
    #[must_use]
    pub const fn new(capacity: f32, dt: f32) -> Self {
        Self { energy: 0.0, capacity, dt }
    }

    /// Energy in the tank, J.
    #[must_use]
    pub const fn energy(&self) -> f32 {
        self.energy
    }

    /// Lets `torque` through if the tank allows it at joint velocity `velocity`.
    pub fn pass(&mut self, torque: f32, velocity: f32) -> f32 {
        let work = torque * velocity * self.dt;
        if work <= 0.0 {
            self.energy = (self.energy - work).min(self.capacity);
            torque
        } else if work <= self.energy {
            self.energy -= work;
            torque
        } else {
            0.0
        }
    }
}

/// Energy the damping tank of one joint may hold, J.
const TANK_CAPACITY: f32 = 0.05;

/// Joint-local damping from the node's own IMU and output encoder.
#[derive(Clone, Copy, Debug)]
pub struct LocalDamper {
    axis: [f32; 3],
    gain: f32,
    filter: HighPass,
    smoothing: LowPass2,
    tank: EnergyTank,
    active: bool,
}

impl LocalDamper {
    /// Damper for the joint described by `config`.
    #[must_use]
    pub fn new(config: &NodeConfig) -> Self {
        Self {
            axis: config.axis,
            gain: config.local_damping,
            filter: HighPass::new(config.damping_highpass_hz, config.cycle_seconds()),
            smoothing: LowPass2::new(config.imu_lowpass_hz, config.cycle_seconds()),
            tank: EnergyTank::new(TANK_CAPACITY, config.cycle_seconds()),
            active: false,
        }
    }

    /// Absolute angular velocity of the joint's output about its axis: housing rate from
    /// the IMU plus the joint's own rate from the output encoder.
    #[must_use]
    pub fn output_rate(&self, e: &Estimate) -> f32 {
        dot(e.gyro, self.axis) + e.output_velocity
    }
}

impl Damper for LocalDamper {
    fn torque(&mut self, mode: DampingMode, e: &Estimate, _context: &DampingContext) -> f32 {
        let vibration = self.smoothing.update(self.filter.update(self.output_rate(e)));
        let on = mode != DampingMode::Off;
        if on && !self.active {
            self.filter.reset();
        }
        self.active = on;
        if on {
            self.tank.pass(-self.gain * vibration, e.output_velocity)
        } else {
            0.0
        }
    }

    fn tank_energy(&self) -> f32 {
        self.tank.energy()
    }
}

#[cfg(test)]
mod tests {
    use super::EnergyTank;

    #[test]
    fn tank_never_releases_more_than_it_took() {
        let mut tank = EnergyTank::new(1.0, 0.01);
        let mut net_work = 0.0;
        for k in 0..10_000_u16 {
            let phase = f32::from(k) * 0.05;
            // A torque out of phase with the velocity (would inject energy without the tank).
            let velocity = libm::sinf(phase);
            let torque = 5.0 * libm::cosf(phase) + 2.0 * velocity;
            net_work += tank.pass(torque, velocity) * velocity * 0.01;
            assert!(net_work <= 1.0 + 1e-3, "net work {net_work}");
        }
    }
}

/// Oldest neighbour state the network term will use, in bus cycles (B5 stale-data rule).
pub const MAX_NEIGHBOUR_AGE: u32 = 8;

/// Local damping plus, in the network modes, damping of the joint's link-tip motion as
/// measured by IMUs at both ends of the link (B1a).
///
/// The outer neighbour's housing sits at the far end of this joint's link, so its gyro
/// measures the rotation rate of the link tip. Subtracting this joint's own housing rate
/// leaves the rate of the link tip relative to the housing, about the joint axis:
///
/// ```text
/// tip_rate = gyro(outer)·a − gyro(own)·a      (= joint rate + link bending rate)
/// ```
///
/// Unlike an encoder rate it needs no differentiation, so it is clean at high bandwidth,
/// and it includes the bending the joint encoder cannot see. The joint damps it
/// (`τ = −c·tip_rate`, high-passed so commanded motion is left alone). If the neighbour's
/// data is late or missing the term drops out and the joint falls back to local damping.
#[derive(Clone, Copy, Debug)]
pub struct NetworkDamper {
    local: LocalDamper,
    axis: [f32; 3],
    gain: f32,
    highpass: HighPass,
    smoothing: LowPass2,
    tank: EnergyTank,
    last_tip_rate: f32,
}

impl NetworkDamper {
    /// Damper for the joint described by `config`.
    #[must_use]
    pub fn new(config: &NodeConfig) -> Self {
        let dt = config.cycle_seconds();
        Self {
            local: LocalDamper::new(config),
            axis: config.axis,
            gain: config.network_damping,
            highpass: HighPass::new(config.damping_highpass_hz, dt),
            smoothing: LowPass2::new(config.imu_lowpass_hz, dt),
            tank: EnergyTank::new(TANK_CAPACITY, dt),
            last_tip_rate: 0.0,
        }
    }

    /// Latest link-tip rate relative to the housing, rad/s (0 without a fresh neighbour).
    #[must_use]
    pub const fn tip_rate(&self) -> f32 {
        self.last_tip_rate
    }
}

impl Damper for NetworkDamper {
    fn torque(&mut self, mode: DampingMode, e: &Estimate, context: &DampingContext) -> f32 {
        let local = if mode == DampingMode::Off { 0.0 } else { self.local.torque(mode, e, context) };
        let network_on = matches!(mode, DampingMode::Network | DampingMode::NetworkWithSource);
        let tip = context
            .neighbours
            .get(Side::Outer)
            .and_then(|n| n.fresh_state(context.now, MAX_NEIGHBOUR_AGE))
            .map(|state| dot(state.gyro, self.axis));
        let Some(tip_gyro) = tip else {
            self.last_tip_rate = 0.0;
            return local;
        };
        let tip_rate = self.smoothing.update(self.highpass.update(tip_gyro - dot(e.gyro, self.axis)));
        self.last_tip_rate = tip_rate;
        if !network_on {
            return local;
        }
        local + self.tank.pass(-self.gain * tip_rate, e.output_velocity)
    }

    fn tank_energy(&self) -> f32 {
        self.tank.energy() + self.local.tank_energy()
    }
}
