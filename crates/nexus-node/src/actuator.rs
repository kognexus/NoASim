//! The node: one bus cycle from sensors and inbox to torque and outbox.

use crate::config::NodeConfig;
use crate::control::Controller;
use crate::damping::{Damper, DampingContext};
use crate::dynamics::DynamicsModel;
use crate::estimate::{Estimate, Estimator};
use crate::gravity::{GravityCompensator, GravitySource};
use crate::io::{Inbox, Outbox};
use crate::models::{ModelChoice, ModelSet};
use crate::neighbours::Neighbourhood;
use crate::reference::{InterpolationMode, Interpolator};
use crate::safety::SafetySupervisor;
use crate::sensors::SensorFrame;
use nexus_proto::{
    DampingMode, Destination, Frame, FrameKind, Header, Limits, NodeId, NodeState, Settings, StatusFlags,
};

/// The gravity chain is shared every this many cycles (1 kHz at a 4 kHz bus).
const CHAIN_PERIOD: u32 = 4;

/// The swappable stages a node is built from.
pub struct Stages<E, I, C, D, S> {
    /// Sensors → estimate.
    pub estimator: E,
    /// Goals → per-cycle reference.
    pub interpolator: I,
    /// Estimate + reference → torque.
    pub controller: C,
    /// Vibration damping on top of the controller.
    pub damper: D,
    /// Limits on the final torque.
    pub safety: S,
}

/// One actuator node, composed of swappable stages.
///
/// Each cycle: estimate → handle inbox → learn → interpolate → control + damping →
/// safety → publish.
pub struct Actuator<E, I, C, D, S> {
    config: NodeConfig,
    line_position: u8,
    id: NodeId,
    limits: Limits,
    has_limits: bool,
    started: bool,
    stages: Stages<E, I, C, D, S>,
    models: ModelSet,
    gravity: GravityCompensator,
    gravity_source: GravitySource,
    settings: Settings,
    last: Estimate,
    last_damping: f32,
    neighbours: Neighbourhood,
}

impl<E: Estimator, I: Interpolator, C: Controller, D: Damper, S: SafetySupervisor> Actuator<E, I, C, D, S> {
    /// Creates the node at `line_position` (its place in the bus line, counted from the
    /// host, as an `EtherCAT` auto-increment address).
    pub fn new(config: NodeConfig, line_position: u8, stages: Stages<E, I, C, D, S>) -> Self {
        Self {
            limits: Limits {
                max_torque: config.peak_torque,
                max_velocity: f32::MAX,
                min_position: f32::MIN,
                max_position: f32::MAX,
                max_temperature: config.max_temperature,
                network_torque_share: 0.0,
            },
            config,
            line_position,
            id: NodeId::UNASSIGNED,
            has_limits: false,
            started: false,
            models: ModelSet::new(&config),
            gravity: GravityCompensator::new(&config),
            gravity_source: GravitySource::Imu,
            stages,
            settings: Settings { gravity: 1, ..Settings::default() },
            last: Estimate::default(),
            last_damping: 0.0,
            neighbours: Neighbourhood::default(),
        }
    }

    /// The node's bus address (`NodeId::UNASSIGNED` until discovery assigns one).
    #[must_use]
    pub const fn id(&self) -> NodeId {
        self.id
    }

    /// The node's nominal parameters.
    #[must_use]
    pub const fn config(&self) -> &NodeConfig {
        &self.config
    }

    /// The latest estimate of the node's own joint.
    #[must_use]
    pub const fn estimate(&self) -> &Estimate {
        &self.last
    }

    /// The node's dynamics models.
    #[must_use]
    pub const fn models(&self) -> &ModelSet {
        &self.models
    }

    /// The behaviour switches the host set.
    #[must_use]
    pub const fn settings(&self) -> &Settings {
        &self.settings
    }

    /// The joints next to this one, as far as discovery and the bus have told it.
    #[must_use]
    pub const fn neighbours(&self) -> &Neighbourhood {
        &self.neighbours
    }

    /// Runs one bus cycle and returns the motor torque (output-referred, N·m).
    pub fn tick(&mut self, cycle: u32, sensors: &SensorFrame, inbox: &Inbox, outbox: &mut Outbox) -> f32 {
        let estimate = self.stages.estimator.update(sensors);
        if !self.started {
            // Power-on: hold where the joint is.
            self.stages.interpolator.hold(estimate.output_angle);
            self.started = true;
        }
        for frame in inbox.frames().flatten() {
            self.handle(&frame, cycle, outbox);
        }
        self.models.observe(&estimate);
        let gravity = self.gravity.update(self.gravity_source, &estimate, &self.neighbours, cycle);
        let reference = self.stages.interpolator.reference(cycle);
        let feedforward =
            self.models.friction(reference.position, reference.velocity) + gravity.holding_torque;
        let control = self.stages.controller.torque(&estimate, &reference, feedforward);
        let context = DampingContext { neighbours: &self.neighbours, now: cycle };
        let cap = self.limits.network_torque_share * self.limits.max_torque;
        let damping = self.stages.damper.torque(self.settings.damping, &estimate, &context).clamp(-cap, cap);
        let torque = self.stages.safety.limit(control + damping, &estimate, &self.limits);
        self.last = estimate;
        self.last_damping = damping;
        if self.id != NodeId::UNASSIGNED {
            outbox.push(&self.state_frame(cycle, &estimate, torque));
            if cycle % CHAIN_PERIOD == u32::from(self.line_position) % CHAIN_PERIOD {
                let header = self.header(FrameKind::Chain, Destination::Neighbours, cycle);
                outbox.push(&Frame::Chain(header, gravity.chain));
            }
        }
        torque
    }

    fn handle(&mut self, frame: &Frame, cycle: u32, outbox: &mut Outbox) {
        match frame {
            Frame::Assign(_, assign) if assign.position == self.line_position => {
                self.id = assign.id;
                let header = self.header(FrameKind::Announce, Destination::All, cycle);
                outbox.push(&Frame::Announce(header, self.config.announce(self.line_position)));
            }
            Frame::Limits(_, limits) => {
                self.limits = *limits;
                self.has_limits = true;
            }
            Frame::Goal(_, goal) => self.stages.interpolator.add_goal(goal, cycle),
            Frame::Settings(_, settings) => {
                self.settings = *settings;
                self.models.choice = ModelChoice::from_byte(settings.model);
                self.gravity_source = GravitySource::from_byte(settings.gravity);
                self.stages.interpolator.set_mode(InterpolationMode::from_byte(settings.interpolation));
            }
            Frame::Announce(header, announce) => {
                self.neighbours.on_announce(self.line_position, header.source, announce);
            }
            Frame::State(header, state) => {
                self.neighbours.on_state(header.source, state, header.cycle, cycle);
            }
            Frame::Chain(header, chain) => self.neighbours.on_chain(header.source, chain, header.cycle),
            Frame::Assign(..) => {}
        }
    }

    const fn header(&self, kind: FrameKind, destination: Destination, cycle: u32) -> Header {
        Header { kind, source: self.id, destination, cycle }
    }

    fn state_frame(&self, cycle: u32, e: &Estimate, torque: f32) -> Frame {
        let mut status = 0;
        if self.has_limits {
            status |= StatusFlags::READY;
        }
        if self.settings.damping != DampingMode::Off && self.last_damping != 0.0 {
            status |= StatusFlags::NETWORK_DAMPING;
        }
        Frame::State(
            self.header(FrameKind::State, Destination::Neighbours, cycle),
            NodeState {
                status: StatusFlags(status),
                motor_angle: e.motor_angle,
                motor_velocity: e.motor_velocity,
                output_angle: e.output_angle,
                output_velocity: e.output_velocity,
                gear_twist: e.gear_twist,
                gyro: e.gyro,
                accel: e.accel,
                current: e.current,
                torque_command: torque,
                network_torque: self.last_damping,
                temperature: e.temperature,
                thermal_headroom: ((self.config.max_temperature - e.temperature)
                    / self.config.max_temperature)
                    .clamp(0.0, 1.0),
                tank_energy: self.stages.damper.tank_energy(),
            },
        )
    }
}
