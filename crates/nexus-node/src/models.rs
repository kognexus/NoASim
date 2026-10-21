//! The set of dynamics models a node carries; the host's settings pick the active one.

use crate::config::NodeConfig;
use crate::dynamics::{DynamicsModel, MotorBalance, NominalModel, RlsModel};
use crate::estimate::Estimate;
use crate::phnn::PhnnModel;

/// Which model the controller uses (`Settings::model`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ModelChoice {
    /// Datasheet values.
    #[default]
    Nominal,
    /// Online least-squares fit of the textbook structure.
    Fitted,
    /// The joint's learned port-Hamiltonian model (B11), if one is loaded.
    Learned,
}

impl ModelChoice {
    /// Maps the settings byte; unknown values fall back to the nominal model.
    #[must_use]
    pub const fn from_byte(byte: u8) -> Self {
        match byte {
            1 => Self::Fitted,
            2 => Self::Learned,
            _ => Self::Nominal,
        }
    }
}

/// Every model the node carries. All of them observe every cycle, so switching is
/// instant.
#[derive(Clone, Copy, Debug)]
pub struct ModelSet {
    nominal: NominalModel,
    fitted: RlsModel,
    learned: Option<PhnnModel>,
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
            learned: None,
            choice: ModelChoice::Nominal,
        }
    }

    /// Installs the joint's frozen learned model.
    pub fn load(&mut self, model: PhnnModel) {
        self.learned = Some(model);
    }

    /// The learned model, if one is loaded.
    #[must_use]
    pub const fn learned(&self) -> Option<&PhnnModel> {
        self.learned.as_ref()
    }

    /// The fitted model, for inspection.
    #[must_use]
    pub const fn fitted(&self) -> &RlsModel {
        &self.fitted
    }

    /// The model in use. Without a loaded learned model, `Learned` falls back to nominal.
    fn active(&self) -> &dyn DynamicsModel {
        match (self.choice, &self.learned) {
            (ModelChoice::Fitted, _) => &self.fitted,
            (ModelChoice::Learned, Some(learned)) => learned,
            _ => &self.nominal,
        }
    }
}

impl DynamicsModel for ModelSet {
    fn observe(&mut self, estimate: &Estimate) {
        self.nominal.observe(estimate);
        self.fitted.observe(estimate);
        if let Some(learned) = &mut self.learned {
            learned.observe(estimate);
        }
    }

    fn friction(&self, motor_angle: f32, motor_velocity: f32) -> f32 {
        self.active().friction(motor_angle, motor_velocity)
    }

    fn unexplained(&self, b: &MotorBalance) -> f32 {
        self.active().unexplained(b)
    }

    fn spring_torque(&self, stiffness: f32, twist: f32) -> f32 {
        self.active().spring_torque(stiffness, twist)
    }
}
