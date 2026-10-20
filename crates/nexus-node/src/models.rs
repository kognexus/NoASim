//! The set of dynamics models a node carries; the host's settings pick the active one.

use crate::config::NodeConfig;
use crate::dynamics::{DynamicsModel, NominalModel, RlsModel};
use crate::estimate::Estimate;

/// Which model the controller uses (`Settings::model`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ModelChoice {
    /// Datasheet values.
    #[default]
    Nominal,
    /// Online least-squares fit of the textbook structure.
    Fitted,
}

impl ModelChoice {
    /// Maps the settings byte; unknown values fall back to the nominal model.
    #[must_use]
    pub const fn from_byte(byte: u8) -> Self {
        if byte == 1 {
            Self::Fitted
        } else {
            Self::Nominal
        }
    }
}

/// Every model the node carries. All of them observe every cycle, so switching is
/// instant.
#[derive(Clone, Copy, Debug)]
pub struct ModelSet {
    nominal: NominalModel,
    fitted: RlsModel,
    /// The model the controller uses.
    pub choice: ModelChoice,
}

impl ModelSet {
    /// Models initialised from the node's datasheet values.
    #[must_use]
    pub fn new(config: &NodeConfig) -> Self {
        Self {
            nominal: NominalModel::new(config.viscous_friction, config.coulomb_friction),
            fitted: RlsModel::new(
                config.viscous_friction,
                config.coulomb_friction,
                config.reflected_inertia,
                config.cycle_seconds(),
            ),
            choice: ModelChoice::Nominal,
        }
    }

    /// The fitted model, for inspection.
    #[must_use]
    pub const fn fitted(&self) -> &RlsModel {
        &self.fitted
    }

    fn active(&self) -> &dyn DynamicsModel {
        match self.choice {
            ModelChoice::Nominal => &self.nominal,
            ModelChoice::Fitted => &self.fitted,
        }
    }
}

impl DynamicsModel for ModelSet {
    fn observe(&mut self, estimate: &Estimate) {
        self.nominal.observe(estimate);
        self.fitted.observe(estimate);
    }

    fn friction(&self, motor_angle: f32, motor_velocity: f32) -> f32 {
        self.active().friction(motor_angle, motor_velocity)
    }
}
