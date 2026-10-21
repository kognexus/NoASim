//! Behaviour of the assembled node over a few bus cycles.

use nexus_node::proto::{Assign, Destination, Frame, FrameKind, Goal, Header, Limits, NodeId, StatusFlags};
use nexus_node::{default_actuator, DefaultActuator, Inbox, NodeConfig, Outbox, SensorFrame};

const HOST: NodeId = NodeId::HOST;

fn host_frame(kind: FrameKind, destination: Destination) -> Header {
    Header { kind, source: HOST, destination, cycle: 0 }
}

/// Encodes frames into inbox records.
fn inbox(frames: &[Frame]) -> Vec<u8> {
    let mut bytes = vec![0; 1024];
    let mut out = Outbox::new(&mut bytes);
    for f in frames {
        assert!(out.push(f));
    }
    let len = out.len();
    bytes.truncate(len);
    bytes
}

fn sent(outbox: &[u8], len: usize) -> Vec<Frame> {
    Inbox::new(&outbox[..len]).frames().map(Result::unwrap).collect()
}

fn tick(
    node: &mut DefaultActuator,
    cycle: u32,
    sensors: &SensorFrame,
    frames: &[Frame],
) -> (f32, Vec<Frame>) {
    let input = inbox(frames);
    let mut buf = vec![0; 1024];
    let mut out = Outbox::new(&mut buf);
    let torque = node.tick(cycle, sensors, &Inbox::new(&input), &mut out);
    let len = out.len();
    (torque, sent(&buf, len))
}

fn at(angle: f32) -> SensorFrame {
    SensorFrame { motor_angle: angle, output_angle: angle, ..SensorFrame::default() }
}

#[test]
fn holds_its_power_on_position_and_stays_silent_until_assigned() {
    let mut node = default_actuator(NodeConfig::example(), 2);
    let (_, frames) = tick(&mut node, 0, &at(0.5), &[]);
    assert_eq!(frames, Vec::new());
    // Pushed away from 0.5, it pushes back.
    let (torque, _) = tick(&mut node, 1, &at(0.6), &[]);
    assert!(torque < 0.0);
}

#[test]
fn announces_itself_when_assigned_and_then_publishes_state() {
    let mut node = default_actuator(NodeConfig::example(), 2);
    let other =
        Frame::Assign(host_frame(FrameKind::Assign, Destination::All), Assign { position: 1, id: NodeId(1) });
    let mine =
        Frame::Assign(host_frame(FrameKind::Assign, Destination::All), Assign { position: 2, id: NodeId(2) });
    let (_, frames) = tick(&mut node, 0, &at(0.0), &[other, mine]);
    assert_eq!(node.id(), NodeId(2));
    let Some(Frame::Announce(h, announce)) = frames.first() else {
        panic!("expected announce, got {frames:?}")
    };
    assert_eq!((h.source, announce.position, announce.gear_ratio), (NodeId(2), 2, 9.0));
    let Some(Frame::State(h, state)) = frames.get(1) else { panic!("expected state") };
    assert_eq!(h.destination, Destination::Neighbours);
    assert!(!state.status.has(StatusFlags::READY));
}

#[test]
fn becomes_ready_with_limits_and_enforces_them() {
    let mut node = default_actuator(NodeConfig::example(), 1);
    let assign =
        Frame::Assign(host_frame(FrameKind::Assign, Destination::All), Assign { position: 1, id: NodeId(1) });
    let limits = Frame::Limits(
        host_frame(FrameKind::Limits, Destination::Node(NodeId(1))),
        Limits {
            max_torque: 5.0,
            max_velocity: 10.0,
            min_position: -3.0,
            max_position: 3.0,
            max_temperature: 120.0,
            network_torque_share: 0.25,
        },
    );
    tick(&mut node, 0, &at(0.0), &[assign, limits]);
    let (torque, frames) = tick(&mut node, 1, &at(1.0), &[]);
    assert!((torque + 5.0).abs() < 1e-6, "torque {torque}");
    let Some(Frame::State(_, state)) = frames.iter().find(|f| matches!(f, Frame::State(..))) else {
        panic!("expected state")
    };
    assert!(state.status.has(StatusFlags::READY));
    assert!((state.torque_command + 5.0).abs() < 1e-6);
}

#[test]
fn follows_goals() {
    let mut node = default_actuator(NodeConfig::example(), 1);
    tick(&mut node, 0, &at(0.0), &[]);
    let goal = Frame::Goal(
        host_frame(FrameKind::Goal, Destination::Node(NodeId(1))),
        Goal { due_cycle: 10, position: 0.2, velocity: 0.0, sequence: 1 },
    );
    let (torque, _) = tick(&mut node, 1, &at(0.0), &[goal]);
    assert!(torque > 0.0);
}

#[test]
fn outbox_counts_frames_that_do_not_fit() {
    let mut bytes = [0; 10];
    let mut out = Outbox::new(&mut bytes);
    let frame =
        Frame::Assign(host_frame(FrameKind::Assign, Destination::All), Assign { position: 1, id: NodeId(1) });
    assert!(!out.push(&frame));
    assert_eq!((out.len(), out.dropped()), (0, 1));
}
