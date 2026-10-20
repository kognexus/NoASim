//! Control logic of one Nexus actuator node.
//!
//! A node sees only two things: the frame of its own sensors, and the bus frames
//! addressed to it. Everything it computes (estimation, interpolation, control,
//! neighbour coordination, safety) happens here, behind small traits that are
//! composed into an [`Actuator`].
//!
//! The crate is `no_std` and allocation-free so it stays MCU-shaped; in the
//! simulation it runs as one WebAssembly instance per joint.

#![no_std]
#![forbid(unsafe_code)]

mod actuator;
mod config;
mod control;
mod estimate;
mod io;
mod neighbours;
mod reference;
mod safety;
mod sensors;

pub use actuator::Actuator;
pub use config::NodeConfig;
pub use control::{Controller, PdController};
pub use estimate::{EncoderEstimator, Estimate, Estimator, TrackingObserver};
pub use io::{Frames, Inbox, Outbox};
pub use neighbours::{Neighbour, Neighbourhood, Side};
pub use nexus_proto as proto;
pub use nexus_proto::NodeId;
pub use reference::{Interpolator, Reference, StepInterpolator};
pub use safety::{SafetySupervisor, TorqueLimiter};
pub use sensors::SensorFrame;

/// The node as currently assembled from the default stages.
pub type DefaultActuator = Actuator<EncoderEstimator, StepInterpolator, PdController, TorqueLimiter>;

/// Builds the default node for `config` at `line_position`.
#[must_use]
pub fn default_actuator(config: NodeConfig, line_position: u8) -> DefaultActuator {
    Actuator::new(
        config,
        line_position,
        EncoderEstimator::new(&config),
        StepInterpolator::default(),
        PdController::new(&config),
        TorqueLimiter,
    )
}
