//! Control logic of one Nexus actuator node.
//!
//! A node sees only two things: the frame of its own sensors, and the bus frames
//! addressed to it. Everything it computes (estimation, interpolation, control,
//! neighbour coordination, safety) happens here, behind small traits that are
//! composed into an `Actuator` in NX-05.
//!
//! The crate is `no_std` and allocation-free so it stays MCU-shaped; in the
//! simulation it runs as one WebAssembly instance per joint.

#![no_std]

pub use nexus_proto::NodeId;
