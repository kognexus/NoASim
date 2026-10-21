//! Vibration damping on top of the position loop (B1a, B5).
//!
//! Damping acts on the angular velocity about the joint axis *minus the joint's own
//! reference velocity*, high-pass filtered: what the motion does that the plan did not
//! ask for. (Damping the absolute rate, as first built, also braked commanded moves and
//! kept pulling after they ended, so a damped arm tracked tasks worse than an undamped
//! one; NX-23.)
//!
//! The reference is smoothed first, at half the rate goals arrive at (NX-25). A stream of
//! goals carries no plan above that; what the reference has there is the kinks where the
//! interpolator joins one goal's segment to the next. Subtracting the raw reference made
//! damping pull each joint onto those kinks, so under a 15 Hz policy a damped arm shook
//! more at 15 and 30 Hz than an undamped one. Under a classic controller's 1 kHz
//! setpoints the smoothing is at 500 Hz and changes nothing.
//!
//! What differs between modes is where the velocity is measured:
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
    /// Inertia beyond the joint about its axis, from the gravity chain, if known, kg·m².
    pub outboard_inertia: Option<f32>,
    /// The joint's own reference output velocity this cycle, rad/s: the commanded part of
    /// the measured rates, which damping must leave alone.
    pub reference_velocity: f32,
    /// Highest frequency the plan behind the reference carries, Hz: half the rate goals
    /// arrive at (`None` before two goals have arrived).
    pub reference_bandwidth_hz: Option<f32>,
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

/// The reference velocity smoothed down to the bandwidth of the plan behind it.
#[derive(Clone, Copy, Debug)]
pub struct ReferenceSmoother {
    filter: LowPass2,
    cutoff_hz: Option<f32>,
    dt: f32,
}

impl ReferenceSmoother {
    /// Smoother for timestep `dt`; passes the reference through until a bandwidth is known.
    #[must_use]
    pub fn new(dt: f32) -> Self {
        Self { filter: LowPass2::new(1.0, dt), cutoff_hz: None, dt }
    }

    /// Smoothed `velocity`, given the bandwidth of the plan.
    pub fn update(&mut self, velocity: f32, bandwidth_hz: Option<f32>) -> f32 {
        // Above a quarter of the cycle rate the filter would change nothing.
        let cutoff = bandwidth_hz.filter(|hz| *hz < 0.25 / self.dt);
        if cutoff != self.cutoff_hz {
            if let Some(hz) = cutoff {
                self.filter.set_cutoff(hz, self.dt);
            }
            self.cutoff_hz = cutoff;
        }
        let smoothed = self.filter.update(velocity);
        if self.cutoff_hz.is_some() {
            smoothed
        } else {
            velocity
        }
    }
}

/// Joint-local damping from the node's own IMU and output encoder.
#[derive(Clone, Copy, Debug)]
pub struct LocalDamper {
    axis: [f32; 3],
    gain: f32,
    filter: HighPass,
    smoothing: LowPass2,
    reference: ReferenceSmoother,
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
            reference: ReferenceSmoother::new(config.cycle_seconds()),
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
    fn torque(&mut self, mode: DampingMode, e: &Estimate, context: &DampingContext) -> f32 {
        let reference = self.reference.update(context.reference_velocity, context.reference_bandwidth_hz);
        let unplanned = self.output_rate(e) - reference;
        let vibration = self.smoothing.update(self.filter.update(unplanned));
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

/// Local damping, or in the network modes damping of link-tip motion measured by IMUs
/// further out along the chain (B1a), with local damping as the fallback when that data is
/// missing or stale, or the joint has no outer neighbour (B5).
///
/// The outer neighbour's housing sits at the far end of this joint's link, so its gyro
/// measures the rotation rate of the link tip; the outer neighbour also relays the rate of
/// the tip of *its* link. Relative to this joint's own housing, about the joint axis:
///
/// ```text
/// tip_rate   = gyro(outer)·a − gyro(own)·a          (joint rate + this link's bending)
/// relay_rate = outer_rate(outer)·a − gyro(own)·a    (… + the next link's bending)
/// ```
///
/// Neither needs differentiation, so both are clean at high bandwidth, and they include
/// the bending the joint encoder cannot see. The joint damps them (`τ = −c·signal`,
/// high-passed so commanded motion is left alone). The gain follows the posture: it
/// scales with the inertia beyond the joint (from the gravity chain) relative to the
/// inertia the gains were sized for, so the joints best placed to move the vibrating mass
/// take the largest share. If the neighbour's data is late or missing the term drops out
/// and the joint falls back to local damping (B5).
#[derive(Clone, Copy, Debug)]
pub struct NetworkDamper {
    local: LocalDamper,
    axis: [f32; 3],
    gain: f32,
    relay_weight: f32,
    design_inertia: f32,
    tip_filter: (HighPass, LowPass2),
    relay_filter: (HighPass, LowPass2),
    reference: ReferenceSmoother,
    tank: EnergyTank,
    last_signal: f32,
}

/// Bounds of the posture scaling of the network gain.
const POSTURE_SCALE: (f32, f32) = (0.25, 2.0);

impl NetworkDamper {
    /// Damper for the joint described by `config`.
    #[must_use]
    pub fn new(config: &NodeConfig) -> Self {
        let dt = config.cycle_seconds();
        let filters =
            || (HighPass::new(config.damping_highpass_hz, dt), LowPass2::new(config.imu_lowpass_hz, dt));
        Self {
            local: LocalDamper::new(config),
            axis: config.axis,
            gain: config.network_damping,
            relay_weight: config.relay_weight,
            design_inertia: config.design_inertia,
            tip_filter: filters(),
            relay_filter: filters(),
            reference: ReferenceSmoother::new(dt),
            tank: EnergyTank::new(TANK_CAPACITY, dt),
            last_signal: 0.0,
        }
    }

    /// Latest network damping signal, rad/s (0 without a fresh neighbour).
    #[must_use]
    pub const fn signal(&self) -> f32 {
        self.last_signal
    }

    fn posture_scale(&self, outboard_inertia: Option<f32>) -> f32 {
        match outboard_inertia {
            Some(inertia) if self.design_inertia > 0.0 => {
                (inertia / self.design_inertia).clamp(POSTURE_SCALE.0, POSTURE_SCALE.1)
            }
            _ => 1.0,
        }
    }
}

impl Damper for NetworkDamper {
    fn torque(&mut self, mode: DampingMode, e: &Estimate, context: &DampingContext) -> f32 {
        let local = if mode == DampingMode::Off { 0.0 } else { self.local.torque(mode, e, context) };
        let reference = self.reference.update(context.reference_velocity, context.reference_bandwidth_hz);
        let network_on = matches!(mode, DampingMode::Network | DampingMode::NetworkWithSource);
        let outer =
            context.neighbours.get(Side::Outer).and_then(|n| n.fresh_state(context.now, MAX_NEIGHBOUR_AGE));
        let Some(outer) = outer else {
            self.last_signal = 0.0;
            return local;
        };
        // Relative to the own housing, the tip turns at this joint's rate plus the link's
        // bending; the commanded part of that is exactly the joint's reference velocity.
        let own = dot(e.gyro, self.axis) + reference;
        let (hp, lp) = &mut self.tip_filter;
        let tip = lp.update(hp.update(dot(outer.gyro, self.axis) - own));
        let (hp, lp) = &mut self.relay_filter;
        let relay = lp.update(hp.update(dot(outer.outer_rate, self.axis) - own));
        let signal = tip + self.relay_weight * relay;
        self.last_signal = signal;
        if !network_on {
            return local;
        }
        // With fresh neighbour data the network term replaces the local one; without it,
        // the joint falls back to local damping (above).
        let gain = self.gain * self.posture_scale(context.outboard_inertia);
        self.tank.pass(-gain * signal, e.output_velocity)
    }

    fn tank_energy(&self) -> f32 {
        self.tank.energy() + self.local.tank_energy()
    }
}
