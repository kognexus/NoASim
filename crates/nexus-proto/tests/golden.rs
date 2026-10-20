//! Pins the wire layout. `golden/frames.txt` holds one line per sample frame:
//! `<name> <hex>`. The TypeScript codec (`sim/src/bus/protocol.test.ts`) checks the same
//! file with the same samples, so both implementations must agree byte for byte.
//!
//! To regenerate after an intended layout change:
//! `cargo test -p nexus-proto --test golden -- --ignored --nocapture print_golden`

use nexus_proto::{
    Announce, Assign, Destination, Frame, FrameKind, Goal, Header, Limits, NodeId, NodeState, StatusFlags,
};

fn header(kind: FrameKind, source: u8, destination: Destination) -> Header {
    Header { kind, source: NodeId(source), destination, cycle: 123_456 }
}

fn samples() -> Vec<(&'static str, Frame)> {
    vec![
        (
            "state",
            Frame::State(
                header(FrameKind::State, 3, Destination::Neighbours),
                NodeState {
                    status: StatusFlags(StatusFlags::READY | StatusFlags::NETWORK_DAMPING),
                    motor_angle: 1.5,
                    motor_velocity: -0.25,
                    output_angle: 1.507_812_5,
                    output_velocity: -0.5,
                    gear_twist: 0.007_812_5,
                    gyro: [0.125, -0.0625, 2.0],
                    accel: [0.5, -9.75, 1.0],
                    current: 3.25,
                    torque_command: 12.5,
                    network_torque: -0.75,
                    temperature: 41.5,
                    thermal_headroom: 0.875,
                    tank_energy: 0.5,
                },
            ),
        ),
        (
            "goal",
            Frame::Goal(
                header(FrameKind::Goal, 0, Destination::Node(NodeId(2))),
                Goal { due_cycle: 123_656, position: -0.75, velocity: 0.125, sequence: 513 },
            ),
        ),
        (
            "limits",
            Frame::Limits(
                header(FrameKind::Limits, 0, Destination::All),
                Limits {
                    max_torque: 90.0,
                    max_velocity: 3.0,
                    min_position: -3.0,
                    max_position: 3.0,
                    max_temperature: 120.0,
                    network_torque_share: 0.25,
                },
            ),
        ),
        (
            "announce",
            Frame::Announce(
                header(FrameKind::Announce, 4, Destination::All),
                Announce {
                    position: 4,
                    has_brake: true,
                    model_version: 7,
                    gear_ratio: 9.0,
                    rated_torque: 30.0,
                    peak_torque: 90.0,
                    axis: [0.0, 1.0, 0.0],
                    link: [0.0, 0.0, 0.390_625],
                    link_mass: 1.625,
                },
            ),
        ),
        (
            "assign",
            Frame::Assign(
                header(FrameKind::Assign, 0, Destination::All),
                Assign { position: 4, id: NodeId(4) },
            ),
        ),
    ]
}

fn hex(bytes: &[u8]) -> String {
    use std::fmt::Write;
    bytes.iter().fold(String::new(), |mut out, b| {
        let _ = write!(out, "{b:02x}");
        out
    })
}

fn encode(frame: &Frame) -> Vec<u8> {
    let mut buf = vec![0; frame.len()];
    assert_eq!(frame.encode(&mut buf), frame.len());
    buf
}

#[test]
fn frames_match_golden_bytes_and_round_trip() {
    let golden = include_str!("../golden/frames.txt");
    for (name, frame) in samples() {
        let line = golden
            .lines()
            .find(|l| l.split_whitespace().next() == Some(name))
            .unwrap_or_else(|| panic!("no golden line for {name}"));
        let expected = line.split_whitespace().nth(1).unwrap();
        let bytes = encode(&frame);
        assert_eq!(hex(&bytes), expected, "{name}");
        assert_eq!(Frame::decode(&bytes), Ok(frame), "{name}");
    }
}

#[test]
fn rejects_unknown_kinds_and_wrong_lengths() {
    use nexus_proto::DecodeError;
    let bytes = encode(&samples()[1].1);
    assert_eq!(Frame::decode(&bytes[..bytes.len() - 1]), Err(DecodeError::WrongLength));
    let mut unknown = bytes.clone();
    unknown[0] = 0xEE;
    assert_eq!(Frame::decode(&unknown), Err(DecodeError::UnknownKind(0xEE)));
}

#[test]
#[ignore = "prints the golden file; run by hand after an intended layout change"]
fn print_golden() {
    for (name, frame) in samples() {
        println!("{name} {}", hex(&encode(&frame)));
    }
}
