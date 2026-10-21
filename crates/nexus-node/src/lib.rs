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
mod damping;
mod dynamics;
mod estimate;
mod filters;
mod gravity;
mod impact;
mod io;
mod models;
mod neighbours;
mod reference;
mod safety;
mod sensors;
mod supervisor;
pub mod vector;

pub use actuator::{Actuator, SafetyOutputs, Stages};
pub use config::NodeConfig;
pub use control::{Controller, JointController};
pub use damping::{Damper, DampingContext, EnergyTank, LocalDamper, NetworkDamper, MAX_NEIGHBOUR_AGE};
pub use dynamics::{DynamicsModel, NominalModel, RlsModel};
pub use estimate::{EncoderEstimator, Estimate, Estimator, TrackingObserver};
pub use filters::{HighPass, LowPass, LowPass2};
pub use gravity::{
    ChainLoad, GravityCompensator, GravityOutput, GravitySource, LinkModel, PayloadEstimator, UpEstimator,
    MAX_CHAIN_AGE, STANDARD_GRAVITY,
};
pub use impact::{ImpactDetector, Watchdog};
pub use io::{Frames, Inbox, Outbox};
pub use models::{ModelChoice, ModelSet};
pub use neighbours::{Neighbour, Neighbourhood, Side};
pub use nexus_proto as proto;
pub use nexus_proto::NodeId;
pub use reference::{
    InterpolationMode, Interpolator, Quintic, Reference, SplineInterpolator, StepInterpolator,
    SwitchableInterpolator,
};
pub use safety::{thermal_headroom, LimitKind, LimitSupervisor, Limited, SafetySupervisor};
pub use sensors::SensorFrame;
pub use supervisor::{Directive, ImpactPolicy, Supervisor, SupervisorTiming};

/// The node as currently assembled from the default stages.
pub type DefaultActuator =
    Actuator<EncoderEstimator, SwitchableInterpolator, JointController, NetworkDamper, LimitSupervisor>;

/// Builds the default node for `config` at `line_position`.
#[must_use]
pub fn default_actuator(config: NodeConfig, line_position: u8) -> DefaultActuator {
    Actuator::new(
        config,
        line_position,
        Stages {
            estimator: EncoderEstimator::new(&config),
            interpolator: SwitchableInterpolator::new(config.cycle_seconds()),
            controller: JointController::new(&config),
            damper: NetworkDamper::new(&config),
            safety: LimitSupervisor::default(),
        },
    )
}
