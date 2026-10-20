//! The node: one bus cycle from sensors and inbox to torque and outbox.

use crate::config::NodeConfig;
use crate::control::Controller;
use crate::estimate::{Estimate, Estimator};
use crate::io::{Inbox, Outbox};
use crate::neighbours::Neighbourhood;
use crate::reference::Interpolator;
use crate::safety::SafetySupervisor;
use crate::sensors::SensorFrame;
use nexus_proto::{Destination, Frame, FrameKind, Header, Limits, NodeId, NodeState, StatusFlags};

/// One actuator node, composed of swappable stages.
///
/// Each cycle: estimate → handle inbox → interpolate → control → safety → publish.
pub struct Actuator<E, I, C, S> {
    config: NodeConfig,
    line_position: u8,
    id: NodeId,
    limits: Limits,
    has_limits: bool,
    started: bool,
    estimator: E,
    interpolator: I,
    controller: C,
    safety: S,
    last: Estimate,
    neighbours: Neighbourhood,
}

impl<E: Estimator, I: Interpolator, C: Controller, S: SafetySupervisor> Actuator<E, I, C, S> {
    /// Creates the node at `line_position` (its place in the bus line, counted from the
    /// host, as an `EtherCAT` auto-increment address).
    pub fn new(
        config: NodeConfig,
        line_position: u8,
        estimator: E,
        interpolator: I,
        controller: C,
        safety: S,
    ) -> Self {
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
            estimator,
            interpolator,
            controller,
            safety,
            last: Estimate::default(),
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

    /// The joints next to this one, as far as discovery and the bus have told it.
    #[must_use]
    pub const fn neighbours(&self) -> &Neighbourhood {
        &self.neighbours
    }

    /// Runs one bus cycle and returns the motor torque (output-referred, N·m).
    pub fn tick(&mut self, cycle: u32, sensors: &SensorFrame, inbox: &Inbox, outbox: &mut Outbox) -> f32 {
        let estimate = self.estimator.update(sensors);
        if !self.started {
            // Power-on: hold where the joint is.
            self.interpolator.hold(estimate.output_angle);
            self.started = true;
        }
        for frame in inbox.frames().flatten() {
            self.handle(&frame, cycle, outbox);
        }
        let reference = self.interpolator.reference(cycle);
        let demand = self.controller.torque(&estimate, &reference);
        let torque = self.safety.limit(demand, &estimate, &self.limits);
        self.last = estimate;
        if self.id != NodeId::UNASSIGNED {
            outbox.push(&self.state_frame(cycle, &estimate, torque));
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
            Frame::Goal(_, goal) => self.interpolator.add_goal(goal, cycle),
            Frame::Announce(header, announce) => {
                self.neighbours.on_announce(self.line_position, header.source, announce);
            }
            Frame::State(header, state) => {
                self.neighbours.on_state(header.source, state, header.cycle, cycle);
            }
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
                network_torque: 0.0,
                temperature: e.temperature,
                thermal_headroom: ((self.config.max_temperature - e.temperature)
                    / self.config.max_temperature)
                    .clamp(0.0, 1.0),
                tank_energy: 0.0,
            },
        )
    }
}
