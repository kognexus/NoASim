//! The node: one bus cycle from sensors and inbox to torque and outbox.

use crate::config::NodeConfig;
use crate::control::Controller;
use crate::damping::{Damper, DampingContext, MAX_NEIGHBOUR_AGE};
use crate::dynamics::DynamicsModel;
use crate::estimate::{Estimate, Estimator};
use crate::gravity::{GravityCompensator, GravitySource};
use crate::impact::{ImpactDetector, Watchdog};
use crate::io::{Inbox, Outbox};
use crate::models::{ModelChoice, ModelSet};
use crate::neighbours::{Neighbourhood, Side};
use crate::reference::{InterpolationMode, Interpolator, Reference};
use crate::safety::{thermal_headroom, SafetySupervisor};
use crate::sensors::SensorFrame;
use crate::source::SourceTracker;
use crate::supervisor::{Directive, ImpactPolicy, Supervisor, SupervisorTiming};
use crate::vector::rotate;
use nexus_proto::{
    DampingMode, Destination, Fault, FaultCode, Frame, FrameKind, Header, Limits, NodeId, NodeState,
    Settings, StatusFlags,
};

/// Ambient temperature assumed for the thermal headroom, °C.
const AMBIENT: f32 = 25.0;

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
    impact: ImpactDetector,
    supervisor: Supervisor,
    watchdog: Watchdog,
    directive: Directive,
    last_hold: f32,
    last_effort: f32,
    limit_status: u16,
    limited_reference: Option<f32>,
    source: SourceTracker,
    last_torque: f32,
}

/// Outputs of a node besides its torque: the brake and the hardwired safe-torque-off line.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SafetyOutputs {
    /// Engage the holding brake.
    pub brake: bool,
    /// Pull the safe-torque-off line (cuts every drive on the line, independent of the bus).
    pub safe_torque_off: bool,
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
            impact: ImpactDetector::new(config.cycle_seconds(), config.impact_threshold),
            supervisor: Supervisor::new(
                SupervisorTiming::new(config.cycle_seconds()),
                config.has_brake > 0.5,
            ),
            watchdog: Watchdog::default(),
            directive: Directive::default(),
            last_hold: 0.0,
            last_effort: 0.0,
            limit_status: 0,
            limited_reference: None,
            source: SourceTracker::new(&config),
            last_torque: 0.0,
        }
    }

    /// Brake and safe-torque-off outputs of the last cycle.
    #[must_use]
    pub const fn safety_outputs(&self) -> SafetyOutputs {
        SafetyOutputs { brake: self.directive.brake, safe_torque_off: self.directive.safe_torque_off }
    }

    /// The safety supervisor, for inspection.
    #[must_use]
    pub const fn supervisor(&self) -> &Supervisor {
        &self.supervisor
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
        self.last = estimate;
        for frame in inbox.frames().flatten() {
            self.handle(&frame, cycle, outbox);
        }
        self.models.observe(&estimate);
        let gravity = self.gravity.update(self.gravity_source, &estimate, &self.neighbours, cycle);
        self.last_hold = gravity.chain.holding_torque;
        // External torque: what the gear transmits minus what the measured motion needs.
        self.last_effort = -estimate.gear_torque - gravity.dynamic_torque;

        // Impact (B2): report our own detection to everyone and to our own supervisor.
        if let Some(impact) = self.impact.update(&estimate, self.last_effort) {
            if self.settings.impact_confirm > 0
                && !self.supervisor.reacting()
                && self.id != NodeId::UNASSIGNED
            {
                outbox.push(&Frame::Impact(self.header(FrameKind::Impact, Destination::All, cycle), impact));
                self.supervisor.report_impact(
                    self.id.0,
                    cycle,
                    impact.score,
                    estimate.output_angle,
                    estimate.output_velocity,
                );
            }
        }
        let planned = self.stages.interpolator.reference(cycle);
        let host_silent =
            self.watchdog.expired(cycle, SupervisorTiming::new(self.config.cycle_seconds()).watchdog_cycles);
        let was_reacting = self.directive.reference.is_some();
        self.directive = self.supervisor.update(cycle, estimate.output_angle, &planned, host_silent);
        if was_reacting && self.directive.reference.is_none() {
            // Released (acknowledged): goals continue from where the joint is.
            self.stages.interpolator.hold(estimate.output_angle);
        }
        if let Some(fault) = self.supervisor.take_fault() {
            if self.id != NodeId::UNASSIGNED {
                outbox.push(&Frame::Fault(self.header(FrameKind::Fault, Destination::Host, cycle), fault));
            }
        }
        let reference = self.speed_limited(self.directive.reference.unwrap_or(planned));
        let feedforward =
            self.models.friction(reference.position, reference.velocity) + gravity.holding_torque;
        let control = self.stages.controller.torque(&estimate, &reference, feedforward);

        let context = DampingContext {
            neighbours: &self.neighbours,
            now: cycle,
            outboard_inertia: self.gravity.outboard_inertia(),
        };
        let cap = self.limits.network_torque_share * self.limits.max_torque;
        let damping = self.stages.damper.torque(self.settings.damping, &estimate, &context).clamp(-cap, cap);
        // Report to source (B1b): tell the others what we feel, claim what is ours, cancel it.
        let network = matches!(self.settings.damping, DampingMode::Network | DampingMode::NetworkWithSource);
        let correct = self.settings.damping == DampingMode::NetworkWithSource;
        let friction = self.models.friction(estimate.motor_angle, estimate.motor_velocity);
        let source = self.source.update(
            self.id,
            &estimate,
            self.last_torque,
            friction,
            &self.neighbours,
            cycle,
            network,
            correct,
        );
        if self.id != NodeId::UNASSIGNED {
            if let Some(report) = source.report {
                outbox.push(&Frame::Disturbance(
                    self.header(FrameKind::Disturbance, Destination::All, cycle),
                    report,
                ));
            }
            if let Some(fault) = source.fault {
                outbox.push(&Frame::Fault(self.header(FrameKind::Fault, Destination::Host, cycle), fault));
            }
        }
        let limited =
            self.stages.safety.limit(control + damping + source.compensation, &estimate, &self.limits);
        self.limit_status = limited.status;
        if let Some(kind) = limited.started {
            // Every limit the joint applies on its own is reported the cycle it starts (B6).
            if self.id != NodeId::UNASSIGNED {
                let fault = Fault { code: FaultCode::Limit, value: f32::from(kind as u8) };
                outbox.push(&Frame::Fault(self.header(FrameKind::Fault, Destination::Host, cycle), fault));
            }
        }
        let torque = if self.directive.safe_torque_off { 0.0 } else { limited.torque };
        self.last_torque = torque;
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
            Frame::Goal(_, goal) if !self.directive.ignore_goals => {
                self.stages.interpolator.add_goal(goal, cycle);
            }
            Frame::Settings(_, settings) => {
                self.settings = *settings;
                self.models.choice = ModelChoice::from_byte(settings.model);
                self.gravity_source = GravitySource::from_byte(settings.gravity);
                self.stages.interpolator.set_mode(InterpolationMode::from_byte(settings.interpolation));
                self.supervisor.configure(
                    ImpactPolicy::from_byte(settings.impact_policy),
                    settings.impact_confirm,
                    self.last_hold,
                );
            }
            Frame::Impact(header, impact) if header.source != self.id => {
                self.supervisor.report_impact(
                    header.source.0,
                    header.cycle,
                    impact.score,
                    self.last.output_angle,
                    self.last.output_velocity,
                );
            }
            Frame::Heartbeat(header) if header.source.is_host() => self.watchdog.feed(cycle),
            Frame::Disturbance(header, report) => self.source.on_report(header.source, report, cycle),
            Frame::Ack(..) => self.supervisor.acknowledge(),
            Frame::Announce(header, announce) => {
                self.neighbours.on_announce(self.line_position, header.source, announce);
            }
            Frame::State(header, state) => {
                self.neighbours.on_state(header.source, state, header.cycle, cycle);
            }
            Frame::Chain(header, chain) => self.neighbours.on_chain(header.source, chain, header.cycle),
            Frame::Assign(..)
            | Frame::Goal(..)
            | Frame::Impact(..)
            | Frame::Heartbeat(..)
            | Frame::Fault(..) => {}
        }
    }

    /// The reference, rate-limited to the host's speed limit (B6): the joint then follows a
    /// slower reference instead of falling behind a fast one.
    fn speed_limited(&mut self, reference: Reference) -> Reference {
        let dt = self.config.cycle_seconds();
        let max_step = self.limits.max_velocity * dt;
        let previous = self.limited_reference.unwrap_or(reference.position);
        let position = previous + (reference.position - previous).clamp(-max_step, max_step);
        let limited = (position - reference.position).abs() > 1e-6;
        self.limited_reference = Some(position);
        Reference {
            position,
            velocity: reference.velocity.clamp(-self.limits.max_velocity, self.limits.max_velocity),
            acceleration: if limited { 0.0 } else { reference.acceleration },
        }
    }

    /// The outer neighbour's gyro (the rate of this link's tip), in this housing frame.
    fn outer_rate(&self, cycle: u32, angle: f32) -> [f32; 3] {
        self.neighbours
            .get(Side::Outer)
            .and_then(|n| n.fresh_state(cycle, MAX_NEIGHBOUR_AGE))
            .map_or([0.0; 3], |state| rotate(state.gyro, self.config.axis, angle))
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
        status |= self.directive.status | self.limit_status;
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
                thermal_headroom: thermal_headroom(e.temperature, self.limits.max_temperature, AMBIENT),
                tank_energy: self.stages.damper.tank_energy(),
                outer_rate: self.outer_rate(cycle, e.output_angle),
            },
        )
    }
}
