//! Gravity compensation computed joint by joint (B10).
//!
//! **Which way is up.** Each node estimates the "up" vector (specific force at rest,
//! 9.81 m/s² long) in its own housing frame from its accelerometer, fused with its gyro so
//! that motion does not fool it ([`UpEstimator`]). This works on a tilted or moving base.
//! The comparison model ([`GravitySource::FixedVertical`]) instead assumes the base is
//! level and passes "up" outward from joint to joint through the encoder angles.
//!
//! **What hangs beyond the joint.** The load is passed from the tip inward as a
//! [`ChainLoad`]: the mass of everything outboard, its centre of mass and the flange
//! position, each expressed in the sender's housing frame (which is the receiver's link
//! frame). Every node adds its own link and passes the sum inward. No node needs the whole
//! robot model.
//!
//! **Payload.** When the arm holds still, the torque a joint actually holds (current and
//! gear twist) minus the torque its model predicts, divided by the lever of a mass at the
//! flange, is a payload estimate. The best estimate (largest lever) flows outward, so the
//! tip node can include it in the chain.

use crate::config::NodeConfig;
use crate::estimate::Estimate;
use crate::filters::LowPass2;
use crate::neighbours::{Neighbourhood, Side};
use crate::vector::{add, cross, dot, norm, rotate, scale, Vec3};
use nexus_proto::Chain;

/// Standard gravity, m/s².
pub const STANDARD_GRAVITY: f32 = 9.806_65;

/// Where a node takes "up" from (`Settings::gravity`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GravitySource {
    /// No gravity feed-forward.
    Off,
    /// The node's own IMU (accelerometer fused with gyro).
    #[default]
    Imu,
    /// Encoder angles from a base assumed level (the comparison model).
    FixedVertical,
}

impl GravitySource {
    /// Maps the settings byte.
    #[must_use]
    pub const fn from_byte(byte: u8) -> Self {
        match byte {
            0 => Self::Off,
            2 => Self::FixedVertical,
            _ => Self::Imu,
        }
    }
}

/// Complementary filter for the "up" vector in the housing frame.
///
/// The gyro rotates the estimate every cycle (fast, drifts); the accelerometer pulls it
/// back with time constant `tau` (slow, but fooled by acceleration). Accelerometer
/// readings far from 1 g are trusted less.
#[derive(Clone, Copy, Debug)]
pub struct UpEstimator {
    up: Option<Vec3>,
    tau: f32,
    dt: f32,
}

impl UpEstimator {
    /// Filter with correction time constant `tau` seconds at timestep `dt`.
    #[must_use]
    pub const fn new(tau: f32, dt: f32) -> Self {
        Self { up: None, tau, dt }
    }

    /// Current estimate, m/s² (`None` before the first sample).
    #[must_use]
    pub const fn up(&self) -> Option<Vec3> {
        self.up
    }

    /// Feeds one gyro (rad/s) and accelerometer (m/s²) sample, both in the housing frame.
    pub fn update(&mut self, gyro: Vec3, accel: Vec3) -> Vec3 {
        let Some(up) = self.up else {
            self.up = Some(accel);
            return accel;
        };
        // A vector fixed in the world, seen from a frame turning at ω, turns at −ω.
        let predicted = add(up, scale(cross(up, gyro), self.dt));
        let deviation = libm::fabsf(norm(accel) - STANDARD_GRAVITY) / STANDARD_GRAVITY;
        let trust = (1.0 - 5.0 * deviation).clamp(0.0, 1.0);
        let alpha = trust * self.dt / self.tau;
        let mixed = add(scale(predicted, 1.0 - alpha), scale(accel, alpha));
        let length = norm(mixed);
        let up = if length > 0.0 { scale(mixed, STANDARD_GRAVITY / length) } else { mixed };
        self.up = Some(up);
        up
    }
}

/// The load hanging beyond a joint, in a given frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ChainLoad {
    /// Total mass, kg.
    pub mass: f32,
    /// Centre of mass, m.
    pub com: Vec3,
    /// Position of the tool flange, m.
    pub flange: Vec3,
}

/// What a node knows about its own link (nominal, from its EEPROM).
#[derive(Clone, Copy, Debug)]
pub struct LinkModel {
    /// Joint axis (unit vector).
    pub axis: Vec3,
    /// Vector from this joint to the next (or the flange), link frame.
    pub link: Vec3,
    /// Link mass, kg (centre of mass at mid-link).
    pub mass: f32,
}

impl LinkModel {
    /// The load beyond this joint, in the link (output) frame: the joint's own link plus
    /// `outer`, the load the outer neighbour reported in its housing frame (= this link's
    /// frame, translated to the end of the link). Without an outer neighbour, `outer` is the
    /// payload at the flange.
    #[must_use]
    pub fn chain_in_link_frame(&self, outer: &ChainLoad) -> ChainLoad {
        let own_com = scale(self.link, 0.5);
        let outer_com = add(self.link, outer.com);
        let mass = self.mass + outer.mass;
        let com = if mass > 0.0 {
            scale(add(scale(own_com, self.mass), scale(outer_com, outer.mass)), 1.0 / mass)
        } else {
            own_com
        };
        ChainLoad { mass, com, flange: add(self.link, outer.flange) }
    }

    /// Expresses a link-frame vector in the housing frame, at joint angle `angle`.
    #[must_use]
    pub fn to_housing(&self, v: Vec3, angle: f32) -> Vec3 {
        rotate(v, self.axis, angle)
    }

    /// Expresses a housing-frame vector in the link frame, at joint angle `angle`.
    #[must_use]
    pub fn to_link(&self, v: Vec3, angle: f32) -> Vec3 {
        rotate(v, self.axis, -angle)
    }

    /// Torque the motor must apply to hold `load` (link frame) against gravity, given
    /// "up" in the link frame.
    #[must_use]
    pub fn holding_torque(&self, load: &ChainLoad, up: Vec3) -> f32 {
        // Weight acts along −up; the joint must balance its moment about the axis.
        -dot(self.axis, cross(load.com, scale(up, -load.mass)))
    }

    /// Holding torque per kilogram placed at the flange (the payload's lever).
    #[must_use]
    pub fn payload_lever(&self, flange: Vec3, up: Vec3) -> f32 {
        dot(self.axis, cross(flange, up))
    }
}

/// Slowly averaged payload estimate from one joint's holding-torque residual.
#[derive(Clone, Copy, Debug, Default)]
pub struct PayloadEstimator {
    estimate: f32,
    lever: f32,
    samples: u32,
}

/// Holding still: joint speed below this (rad/s) counts as static.
const STATIC_SPEED: f32 = 0.01;
/// Smallest lever worth estimating from, N·m/kg.
const MIN_LEVER: f32 = 0.5;

impl PayloadEstimator {
    /// Feeds one cycle: measured holding torque, the model's torque without payload, the
    /// payload lever, and the joint speed. Only static cycles with a useful lever count.
    pub fn update(&mut self, measured: f32, model_without_payload: f32, lever: f32, speed: f32) {
        if libm::fabsf(speed) > STATIC_SPEED || libm::fabsf(lever) < MIN_LEVER {
            return;
        }
        let sample = (measured - model_without_payload) / lever;
        self.samples = self.samples.saturating_add(1);
        // Running mean over the first second, then a slow exponential average.
        #[allow(clippy::cast_precision_loss)]
        let weight = (1.0 / self.samples as f32).max(1.0 / 4000.0);
        self.estimate += weight * (sample - self.estimate);
        self.lever = libm::fabsf(lever);
    }

    /// `(mass estimate in kg, lever in N·m/kg)`; lever 0 while there is no estimate.
    #[must_use]
    pub const fn estimate(&self) -> (f32, f32) {
        if self.samples == 0 {
            (0.0, 0.0)
        } else {
            (self.estimate, self.lever)
        }
    }
}

/// Neighbour chain data older than this counts as stale, in bus cycles. Stale data is still
/// used (the load beyond a joint changes slowly, and holding with the last known load is far
/// better than holding with none); it is only reported as stale.
pub const MAX_CHAIN_AGE: u32 = 40;
/// Oldest outer-neighbour IMU data used for the dynamic chain, bus cycles.
const MAX_DYNAMIC_AGE: u32 = 8;
/// Correction time constant of the up estimator, s.
const UP_FILTER_SECONDS: f32 = 0.02;
/// Gravity changes slowly; the feed-forward is smoothed so chain updates (every few cycles)
/// and estimate noise do not reach the motor as steps, Hz.
const FEEDFORWARD_FILTER_HZ: f32 = 15.0;

/// One cycle of gravity compensation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GravityOutput {
    /// Torque to add as feed-forward, N·m (0 when the source is off).
    pub holding_torque: f32,
    /// Torque the joint should be transmitting for the motion the IMUs measure (gravity and
    /// inertia of everything beyond it), N·m. The torque the gear actually transmits minus
    /// this is the external torque: contact (B2).
    pub dynamic_torque: f32,
    /// What to tell the neighbours.
    pub chain: Chain,
}

/// The gravity stage of a node: up estimate, chain sum, holding torque and payload.
#[derive(Clone, Copy, Debug)]
pub struct GravityCompensator {
    link: LinkModel,
    up: UpEstimator,
    payload: PayloadEstimator,
    smoothing: LowPass2,
    outboard_inertia: Option<f32>,
}

impl GravityCompensator {
    /// Stage for the joint described by `config`.
    #[must_use]
    pub fn new(config: &NodeConfig) -> Self {
        Self {
            link: LinkModel { axis: config.axis, link: config.link, mass: config.link_mass },
            up: UpEstimator::new(UP_FILTER_SECONDS, config.cycle_seconds()),
            payload: PayloadEstimator::default(),
            smoothing: LowPass2::new(FEEDFORWARD_FILTER_HZ, config.cycle_seconds()),
            outboard_inertia: None,
        }
    }

    /// Inertia of everything beyond the joint about its axis, from the chain (point-mass
    /// estimate), kg·m²; `None` before the first chain sum.
    #[must_use]
    pub const fn outboard_inertia(&self) -> Option<f32> {
        self.outboard_inertia
    }

    /// The up vector this node's IMU sees, housing frame (`None` before the first cycle).
    #[must_use]
    pub const fn imu_up(&self) -> Option<Vec3> {
        self.up.up()
    }

    /// Runs one cycle.
    pub fn update(
        &mut self,
        source: GravitySource,
        e: &Estimate,
        neighbours: &Neighbourhood,
        now: u32,
    ) -> GravityOutput {
        let angle = e.output_angle;
        let imu_up = self.up.update(e.gyro, e.accel);
        // Last known chain data, fresh or not (see MAX_CHAIN_AGE).
        let inner = neighbours.get(Side::Inner).and_then(|n| n.chain.as_ref());
        let outer = neighbours.get(Side::Outer).and_then(|n| n.chain.as_ref());

        // Comparison model: "up" passed outward from a base assumed level.
        let level_up = inner.map_or([0.0, 0.0, STANDARD_GRAVITY], |c| c.up_link);
        let up_housing = match source {
            GravitySource::FixedVertical => level_up,
            _ => imu_up,
        };
        let up_link = self.link.to_link(up_housing, angle);

        // The best payload estimate so far along the chain flows outward.
        let (inner_mass, inner_lever) = inner.map_or((0.0, 0.0), |c| (c.payload_mass, c.payload_lever));
        let (own_mass, own_lever) = self.payload.estimate();
        let (payload_mass, payload_lever) = if own_lever >= libm::fabsf(inner_lever) {
            (own_mass, own_lever)
        } else {
            (inner_mass, inner_lever)
        };

        // Load beyond the joint: the outer neighbour's chain, or the payload at the flange.
        let outer_load = outer
            .map_or(ChainLoad { mass: payload_mass.max(0.0), com: [0.0; 3], flange: [0.0; 3] }, |c| {
                ChainLoad { mass: c.mass, com: c.com, flange: c.flange }
            });
        let load = self.link.chain_in_link_frame(&outer_load);
        let hold = self.link.holding_torque(&load, up_link);
        let along = dot(load.com, self.link.axis);
        let perpendicular_sq = (dot(load.com, load.com) - along * along).max(0.0);
        self.outboard_inertia = Some(load.mass * perpendicular_sq);

        // Payload: the torque the gear actually transmits, against the model without it.
        let lever = self.link.payload_lever(load.flange, up_link);
        self.payload.update(-e.gear_torque, hold - payload_mass.max(0.0) * lever, lever, e.output_velocity);

        let smoothed = self.smoothing.update(hold);

        // The same chain with measured motion: each link's mass times the specific force at
        // its centre (mean of the IMUs at its two ends), summed from the tip inward.
        let near = self.link.to_link(e.accel, angle);
        let outer_state = neighbours.get(Side::Outer).and_then(|n| n.fresh_state(now, MAX_DYNAMIC_AGE));
        let centre = outer_state.map_or(near, |state| scale(add(near, state.accel), 0.5));
        let own_force = scale(centre, self.link.mass);
        let own_moment = cross(scale(self.link.link, 0.5), own_force);
        let (outer_force, outer_moment) = match outer {
            Some(c) => (c.dynamic_force, c.dynamic_moment),
            // The tip: the payload sits at the flange and moves (about) like the link's near end.
            None => (scale(near, payload_mass.max(0.0)), [0.0; 3]),
        };
        let force = add(own_force, outer_force);
        let moment = add(add(own_moment, outer_moment), cross(self.link.link, outer_force));
        let dynamic = dot(self.link.axis, moment);

        GravityOutput {
            dynamic_torque: dynamic,
            holding_torque: if source == GravitySource::Off { 0.0 } else { smoothed },
            chain: Chain {
                up_link: self.link.to_link(level_up, angle),
                mass: load.mass,
                com: self.link.to_housing(load.com, angle),
                flange: self.link.to_housing(load.flange, angle),
                payload_mass,
                payload_lever,
                holding_torque: hold,
                dynamic_force: self.link.to_housing(force, angle),
                dynamic_moment: self.link.to_housing(moment, angle),
                link_bend: 0.0,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{ChainLoad, LinkModel, UpEstimator, STANDARD_GRAVITY};

    const UP: [f32; 3] = [0.0, 0.0, STANDARD_GRAVITY];

    #[test]
    fn horizontal_link_holds_its_own_weight_and_the_outer_load() {
        // Joint axis y, link 1 m along x, 2 kg; 3 kg hanging at the end.
        let link = LinkModel { axis: [0.0, 1.0, 0.0], link: [1.0, 0.0, 0.0], mass: 2.0 };
        let outer = ChainLoad { mass: 3.0, com: [0.0, 0.0, 0.0], flange: [0.0, 0.0, 0.0] };
        let load = link.chain_in_link_frame(&outer);
        assert!((load.mass - 5.0).abs() < 1e-6);
        // Moment of 2 kg at 0.5 m plus 3 kg at 1 m: 4 kg·m · g. Turning about +y moves +x
        // down, so the weight's moment is positive and the joint must hold a negative torque.
        let hold = link.holding_torque(&load, UP);
        assert!((hold + 4.0 * STANDARD_GRAVITY).abs() < 1e-3, "hold {hold}");
        // One more kilogram at the flange needs another −g·1 m.
        assert!((link.payload_lever(load.flange, UP) + STANDARD_GRAVITY).abs() < 1e-4);
    }

    #[test]
    fn up_estimate_follows_a_tilt_seen_by_the_accelerometer() {
        let mut filter = UpEstimator::new(0.2, 0.001);
        filter.update([0.0; 3], UP);
        let tilted = [STANDARD_GRAVITY * 0.2588, 0.0, STANDARD_GRAVITY * 0.9659]; // 15°
        let mut up = UP;
        for _ in 0..3000 {
            up = filter.update([0.0; 3], tilted);
        }
        assert!((up[0] - tilted[0]).abs() < 0.01 && (up[2] - tilted[2]).abs() < 0.01);
    }

    #[test]
    fn up_estimate_ignores_a_short_shake() {
        let mut filter = UpEstimator::new(0.5, 0.001);
        filter.update([0.0; 3], UP);
        let mut up = UP;
        for k in 0..200 {
            let shake = if k % 2 == 0 { 15.0 } else { -15.0 };
            up = filter.update([0.0; 3], [shake, 0.0, STANDARD_GRAVITY]);
        }
        assert!(up[0].abs() < 0.5, "up {up:?}");
    }
}
