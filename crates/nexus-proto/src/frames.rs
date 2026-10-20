//! Frame types and their fixed binary layout.

use crate::codec::{DecodeError, Reader, Writer};
use crate::NodeId;

/// Bytes in every frame header.
pub const HEADER_LEN: usize = 8;

/// Longest frame of any kind, header included.
pub const MAX_FRAME_LEN: usize = HEADER_LEN + NodeState::LEN;

/// Kind of a frame; also its first byte on the wire.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum FrameKind {
    /// Cyclic state a node publishes every bus cycle.
    State = 1,
    /// A motion goal from the host.
    Goal = 2,
    /// Operating limits from the host.
    Limits = 3,
    /// A node's identity and nominal parameters, published during discovery.
    Announce = 4,
    /// The host assigning a bus address to the node at a line position.
    Assign = 5,
    /// The host switching node behaviour (damping mode and friends) at run time.
    Settings = 6,
    /// Gravity chain data a node shares with its neighbours (B10).
    Chain = 7,
}

impl FrameKind {
    const fn from_byte(byte: u8) -> Result<Self, DecodeError> {
        Ok(match byte {
            1 => Self::State,
            2 => Self::Goal,
            3 => Self::Limits,
            4 => Self::Announce,
            5 => Self::Assign,
            6 => Self::Settings,
            7 => Self::Chain,
            other => return Err(DecodeError::UnknownKind(other)),
        })
    }

    /// Payload length in bytes (fixed per kind).
    #[must_use]
    pub const fn payload_len(self) -> usize {
        match self {
            Self::State => NodeState::LEN,
            Self::Goal => Goal::LEN,
            Self::Limits => Limits::LEN,
            Self::Announce => Announce::LEN,
            Self::Assign => Assign::LEN,
            Self::Settings => Settings::LEN,
            Self::Chain => Chain::LEN,
        }
    }
}

/// Where a frame should go. The bus routes it; nodes never address memory directly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Destination {
    /// The host (bus master).
    Host,
    /// One node.
    Node(NodeId),
    /// The sender's neighbours in the chain, and the host.
    Neighbours,
    /// Every node and the host.
    All,
}

impl Destination {
    const NEIGHBOURS: u8 = 0xFE;
    const ALL: u8 = 0xFF;

    const fn to_byte(self) -> u8 {
        match self {
            Self::Host => NodeId::HOST.0,
            Self::Node(id) => id.0,
            Self::Neighbours => Self::NEIGHBOURS,
            Self::All => Self::ALL,
        }
    }

    const fn from_byte(byte: u8) -> Self {
        match byte {
            0 => Self::Host,
            Self::NEIGHBOURS => Self::Neighbours,
            Self::ALL => Self::All,
            id => Self::Node(NodeId(id)),
        }
    }
}

/// Common header of every frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    /// What the payload is.
    pub kind: FrameKind,
    /// Sender.
    pub source: NodeId,
    /// Receiver(s).
    pub destination: Destination,
    /// Bus cycle (distributed clock) at which the sender wrote the frame.
    pub cycle: u32,
}

/// Status bits a node publishes in its [`NodeState`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StatusFlags(pub u16);

impl StatusFlags {
    /// Discovery is complete and the node follows goals.
    pub const READY: u16 = 1 << 0;
    /// The node has registered a fault.
    pub const FAULT: u16 = 1 << 1;
    /// The node is adding network damping.
    pub const NETWORK_DAMPING: u16 = 1 << 2;
    /// The node is in an emergency or impact reaction.
    pub const STOPPING: u16 = 1 << 3;
    /// The node holds position and waits for the host.
    pub const HOLDING: u16 = 1 << 4;
    /// Safe torque off is active.
    pub const TORQUE_OFF: u16 = 1 << 5;
    /// The node is derating (thermal or limit).
    pub const DERATING: u16 = 1 << 6;

    /// Returns `true` if every bit of `mask` is set.
    #[must_use]
    pub const fn has(self, mask: u16) -> bool {
        self.0 & mask == mask
    }
}

/// The state a node publishes every cycle: its measurements and what it is doing.
///
/// Angles are output-referred radians, rates rad/s, torques N·m at the output.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NodeState {
    /// Status bits.
    pub status: StatusFlags,
    /// Motor-side encoder angle.
    pub motor_angle: f32,
    /// Estimated motor-side velocity.
    pub motor_velocity: f32,
    /// Output encoder angle.
    pub output_angle: f32,
    /// Estimated output velocity.
    pub output_velocity: f32,
    /// Gear twist (output minus motor side).
    pub gear_twist: f32,
    /// Gyro reading of the housing, IMU frame.
    pub gyro: [f32; 3],
    /// Accelerometer reading of the housing, IMU frame.
    pub accel: [f32; 3],
    /// Measured current, A.
    pub current: f32,
    /// Torque the node commands this cycle.
    pub torque_command: f32,
    /// Part of the command added for the network (damping), N·m.
    pub network_torque: f32,
    /// Winding temperature, °C.
    pub temperature: f32,
    /// Fraction of thermal headroom left, 0…1.
    pub thermal_headroom: f32,
    /// Energy left in the network-damping tank, J.
    pub tank_energy: f32,
}

impl NodeState {
    /// Payload length in bytes.
    pub const LEN: usize = 4 + 17 * 4;

    fn write(&self, w: &mut Writer) {
        w.u16(self.status.0);
        w.u16(0);
        for v in
            [self.motor_angle, self.motor_velocity, self.output_angle, self.output_velocity, self.gear_twist]
        {
            w.f32(v);
        }
        w.vec3(self.gyro);
        w.vec3(self.accel);
        for v in [
            self.current,
            self.torque_command,
            self.network_torque,
            self.temperature,
            self.thermal_headroom,
            self.tank_energy,
        ] {
            w.f32(v);
        }
    }

    fn read(r: &mut Reader) -> Result<Self, DecodeError> {
        let status = StatusFlags(r.u16()?);
        r.u16()?;
        Ok(Self {
            status,
            motor_angle: r.f32()?,
            motor_velocity: r.f32()?,
            output_angle: r.f32()?,
            output_velocity: r.f32()?,
            gear_twist: r.f32()?,
            gyro: r.vec3()?,
            accel: r.vec3()?,
            current: r.f32()?,
            torque_command: r.f32()?,
            network_torque: r.f32()?,
            temperature: r.f32()?,
            thermal_headroom: r.f32()?,
            tank_energy: r.f32()?,
        })
    }
}

/// A motion goal: be at `position` with `velocity` at bus cycle `due_cycle`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Goal {
    /// Bus cycle at which the goal should be reached.
    pub due_cycle: u32,
    /// Joint (output) position, rad.
    pub position: f32,
    /// Joint (output) velocity at that time, rad/s.
    pub velocity: f32,
    /// Increments with every goal the host sends, to spot lost goals.
    pub sequence: u16,
}

impl Goal {
    /// Payload length in bytes.
    pub const LEN: usize = 16;

    fn write(&self, w: &mut Writer) {
        w.u32(self.due_cycle);
        w.f32(self.position);
        w.f32(self.velocity);
        w.u16(self.sequence);
        w.u16(0);
    }

    fn read(r: &mut Reader) -> Result<Self, DecodeError> {
        let goal = Self { due_cycle: r.u32()?, position: r.f32()?, velocity: r.f32()?, sequence: r.u16()? };
        r.u16()?;
        Ok(goal)
    }
}

/// Operating limits the host sets for one node; the node enforces them every cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Limits {
    /// Maximum output torque, N·m.
    pub max_torque: f32,
    /// Maximum output speed, rad/s.
    pub max_velocity: f32,
    /// Lowest allowed position, rad.
    pub min_position: f32,
    /// Highest allowed position, rad.
    pub max_position: f32,
    /// Winding temperature limit, °C.
    pub max_temperature: f32,
    /// Largest share of `max_torque` the network term may use, 0…1.
    pub network_torque_share: f32,
}

impl Limits {
    /// Payload length in bytes.
    pub const LEN: usize = 6 * 4;

    fn write(&self, w: &mut Writer) {
        for v in [
            self.max_torque,
            self.max_velocity,
            self.min_position,
            self.max_position,
            self.max_temperature,
            self.network_torque_share,
        ] {
            w.f32(v);
        }
    }

    fn read(r: &mut Reader) -> Result<Self, DecodeError> {
        Ok(Self {
            max_torque: r.f32()?,
            max_velocity: r.f32()?,
            min_position: r.f32()?,
            max_position: r.f32()?,
            max_temperature: r.f32()?,
            network_torque_share: r.f32()?,
        })
    }
}

/// A node's identity and nominal (datasheet) parameters, as stored in its EEPROM.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Announce {
    /// Position in the line, counted from the host (1 = next to the host).
    pub position: u8,
    /// Whether a holding brake is fitted.
    pub has_brake: bool,
    /// Version of the node's dynamics model.
    pub model_version: u16,
    /// Gear ratio.
    pub gear_ratio: f32,
    /// Continuous output torque, N·m.
    pub rated_torque: f32,
    /// Peak output torque, N·m.
    pub peak_torque: f32,
    /// Joint axis in the joint frame.
    pub axis: [f32; 3],
    /// Vector from this joint to the next, joint frame, m.
    pub link: [f32; 3],
    /// Nominal mass of the link (and the next housing), kg.
    pub link_mass: f32,
}

impl Announce {
    /// Payload length in bytes.
    pub const LEN: usize = 4 + 10 * 4;

    fn write(&self, w: &mut Writer) {
        w.u8(self.position);
        w.u8(u8::from(self.has_brake));
        w.u16(self.model_version);
        w.f32(self.gear_ratio);
        w.f32(self.rated_torque);
        w.f32(self.peak_torque);
        w.vec3(self.axis);
        w.vec3(self.link);
        w.f32(self.link_mass);
    }

    fn read(r: &mut Reader) -> Result<Self, DecodeError> {
        Ok(Self {
            position: r.u8()?,
            has_brake: r.u8()? != 0,
            model_version: r.u16()?,
            gear_ratio: r.f32()?,
            rated_torque: r.f32()?,
            peak_torque: r.f32()?,
            axis: r.vec3()?,
            link: r.vec3()?,
            link_mass: r.f32()?,
        })
    }
}

/// The host assigning bus address `id` to the node at line `position`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Assign {
    /// Line position of the node being addressed.
    pub position: u8,
    /// Address it gets.
    pub id: NodeId,
}

impl Assign {
    /// Payload length in bytes.
    pub const LEN: usize = 4;

    fn write(self, w: &mut Writer) {
        w.u8(self.position);
        w.u8(self.id.0);
        w.u16(0);
    }

    fn read(r: &mut Reader) -> Result<Self, DecodeError> {
        let assign = Self { position: r.u8()?, id: NodeId(r.u8()?) };
        r.u16()?;
        Ok(assign)
    }
}

/// How a node damps vibration (the comparison modes of PLAN §4).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum DampingMode {
    /// Joint position control only.
    #[default]
    Off = 0,
    /// Each joint damps from its own IMU and encoders.
    Local = 1,
    /// Joints also use their neighbours' data.
    Network = 2,
    /// Network damping plus report-to-source and correction at the source.
    NetworkWithSource = 3,
}

impl DampingMode {
    const fn from_byte(byte: u8) -> Self {
        match byte {
            1 => Self::Local,
            2 => Self::Network,
            3 => Self::NetworkWithSource,
            _ => Self::Off,
        }
    }
}

/// Run-time behaviour switches the host sets for every node.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Settings {
    /// How the node damps vibration.
    pub damping: DampingMode,
    /// Interpolation mode (0 = follow host setpoints directly, 1 = joint-level).
    pub interpolation: u8,
    /// Impact policy (see PLAN §2.1; 0 = retreat and hold).
    pub impact_policy: u8,
    /// Dynamics model (0 = nominal, 1 = fitted, 2 = learned).
    pub model: u8,
    /// Gravity feed-forward source (0 = off, 1 = own IMU, 2 = encoders from a level base).
    pub gravity: u8,
}

impl Settings {
    /// Payload length in bytes.
    pub const LEN: usize = 8;

    fn write(self, w: &mut Writer) {
        w.u8(self.damping as u8);
        w.u8(self.interpolation);
        w.u8(self.impact_policy);
        w.u8(self.model);
        w.u8(self.gravity);
        w.u8(0);
        w.u16(0);
    }

    fn read(r: &mut Reader) -> Result<Self, DecodeError> {
        let settings = Self {
            damping: DampingMode::from_byte(r.u8()?),
            interpolation: r.u8()?,
            impact_policy: r.u8()?,
            model: r.u8()?,
            gravity: r.u8()?,
        };
        r.u8()?;
        r.u16()?;
        Ok(settings)
    }
}

/// Gravity chain data (B10), sent to the neighbours about once per millisecond.
///
/// Vectors are in the sender's housing frame, which is its inner neighbour's link frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Chain {
    /// "Up" (specific force at rest, m/s²) in the sender's *link* frame, propagated from a
    /// base assumed level through the encoder angles. Only the comparison model uses it.
    pub up_link: [f32; 3],
    /// Mass of the sender's link and everything beyond it, kg.
    pub mass: f32,
    /// Centre of that mass, m.
    pub com: [f32; 3],
    /// Tool flange position, m.
    pub flange: [f32; 3],
    /// Best payload estimate so far along the chain, kg.
    pub payload_mass: f32,
    /// Lever of that estimate (larger is better), N·m/kg; 0 if none.
    pub payload_lever: f32,
    /// Gravity torque the sender is holding, N·m.
    pub holding_torque: f32,
}

impl Chain {
    /// Payload length in bytes.
    pub const LEN: usize = 13 * 4;

    fn write(&self, w: &mut Writer) {
        w.vec3(self.up_link);
        w.f32(self.mass);
        w.vec3(self.com);
        w.vec3(self.flange);
        w.f32(self.payload_mass);
        w.f32(self.payload_lever);
        w.f32(self.holding_torque);
    }

    fn read(r: &mut Reader) -> Result<Self, DecodeError> {
        Ok(Self {
            up_link: r.vec3()?,
            mass: r.f32()?,
            com: r.vec3()?,
            flange: r.vec3()?,
            payload_mass: r.f32()?,
            payload_lever: r.f32()?,
            holding_torque: r.f32()?,
        })
    }
}

/// A decoded frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Frame {
    /// See [`NodeState`].
    State(Header, NodeState),
    /// See [`Goal`].
    Goal(Header, Goal),
    /// See [`Limits`].
    Limits(Header, Limits),
    /// See [`Announce`].
    Announce(Header, Announce),
    /// See [`Assign`].
    Assign(Header, Assign),
    /// See [`Settings`].
    Settings(Header, Settings),
    /// See [`Chain`].
    Chain(Header, Chain),
}

impl Frame {
    /// The frame's header.
    #[must_use]
    pub const fn header(&self) -> &Header {
        match self {
            Self::State(h, _)
            | Self::Goal(h, _)
            | Self::Limits(h, _)
            | Self::Announce(h, _)
            | Self::Assign(h, _)
            | Self::Settings(h, _)
            | Self::Chain(h, _) => h,
        }
    }

    /// Encoded length in bytes.
    #[must_use]
    pub const fn len(&self) -> usize {
        HEADER_LEN + self.header().kind.payload_len()
    }

    /// Frames always carry a header, so they are never empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        false
    }

    /// Encodes the frame into the start of `buf` and returns the number of bytes written.
    ///
    /// # Panics
    /// If `buf` is shorter than [`Frame::len`].
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        let header = self.header();
        w.u8(header.kind as u8);
        w.u8(header.source.0);
        w.u8(header.destination.to_byte());
        w.u8(0);
        w.u32(header.cycle);
        match self {
            Self::State(_, p) => p.write(&mut w),
            Self::Goal(_, p) => p.write(&mut w),
            Self::Limits(_, p) => p.write(&mut w),
            Self::Announce(_, p) => p.write(&mut w),
            Self::Assign(_, p) => p.write(&mut w),
            Self::Settings(_, p) => p.write(&mut w),
            Self::Chain(_, p) => p.write(&mut w),
        }
        w.len()
    }

    /// Decodes one frame that occupies all of `buf`.
    ///
    /// # Errors
    /// If the kind is unknown or the length does not match the kind.
    pub fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        let mut r = Reader::new(buf);
        let kind = FrameKind::from_byte(r.u8()?)?;
        if buf.len() != HEADER_LEN + kind.payload_len() {
            return Err(DecodeError::WrongLength);
        }
        let source = NodeId(r.u8()?);
        let destination = Destination::from_byte(r.u8()?);
        r.u8()?;
        let header = Header { kind, source, destination, cycle: r.u32()? };
        Ok(match kind {
            FrameKind::State => Self::State(header, NodeState::read(&mut r)?),
            FrameKind::Goal => Self::Goal(header, Goal::read(&mut r)?),
            FrameKind::Limits => Self::Limits(header, Limits::read(&mut r)?),
            FrameKind::Announce => Self::Announce(header, Announce::read(&mut r)?),
            FrameKind::Assign => Self::Assign(header, Assign::read(&mut r)?),
            FrameKind::Settings => Self::Settings(header, Settings::read(&mut r)?),
            FrameKind::Chain => Self::Chain(header, Chain::read(&mut r)?),
        })
    }
}
