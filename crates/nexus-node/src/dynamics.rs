//! The node's model of its own actuator: what friction (and later ripple and stiffness)
//! it has to overcome. The controller uses it as feed-forward.
//!
//! Friction is measured without a torque sensor, from the motor-side balance
//!
//! ```text
//! I_m · α_m = τ_motor + k_g · twist − friction
//! ```
//!
//! where `τ_motor` comes from the current, `k_g · twist` from the dual encoders and `α_m`
//! from the motor encoder.

use crate::estimate::Estimate;
use crate::filters::LowPass;

/// Velocity scale of the smooth sign used for Coulomb friction, rad/s.
const STICTION_VELOCITY: f32 = 0.01;

fn smooth_sign(velocity: f32) -> f32 {
    libm::tanhf(velocity / STICTION_VELOCITY)
}

fn dot(a: &[f32; N], b: &[f32; N]) -> f32 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// A model of the actuator's own dynamics.
pub trait DynamicsModel {
    /// Learns from one cycle of measurements (no-op for fixed models).
    fn observe(&mut self, estimate: &Estimate);
    /// Friction torque at `motor_velocity` (same sign as the velocity), N·m.
    fn friction(&self, motor_angle: f32, motor_velocity: f32) -> f32;
}

/// Datasheet friction: viscous plus Coulomb, never updated.
#[derive(Clone, Copy, Debug)]
pub struct NominalModel {
    viscous: f32,
    coulomb: f32,
}

impl NominalModel {
    /// Model with the datasheet friction values.
    #[must_use]
    pub const fn new(viscous: f32, coulomb: f32) -> Self {
        Self { viscous, coulomb }
    }
}

impl DynamicsModel for NominalModel {
    fn observe(&mut self, _estimate: &Estimate) {}

    fn friction(&self, _motor_angle: f32, v: f32) -> f32 {
        self.viscous * v + self.coulomb * smooth_sign(v)
    }
}

/// Number of fitted parameters: viscous, Coulomb, offset.
const N: usize = 3;

/// Recursive least squares on the textbook friction model `b·v + Fc·sign(v) + c`, fitted
/// online from the motor-side torque balance. It can only represent that structure;
/// friction of another shape stays in the residual.
///
/// Written in the Kalman form (measurement noise `r`, small process noise `q` added to
/// the covariance) rather than with exponential forgetting: it stays positive definite
/// in single precision and keeps adapting slowly as the actuator changes.
#[derive(Clone, Copy, Debug)]
pub struct RlsModel {
    theta: [f32; N],
    p: [[f32; N]; N],
    inertia: f32,
    measurement: LowPass,
    regressors: [LowPass; N],
}

/// Corner of the filter applied to both sides of the regression, Hz.
const RLS_FILTER_HZ: f32 = 20.0;
/// Initial covariance (large = little trust in the initial guess).
const RLS_INITIAL_COVARIANCE: f32 = 10.0;
/// Measurement noise variance of the friction torque, N·m².
const RLS_MEASUREMENT_NOISE: f32 = 1.0;
/// Process noise added to each parameter's variance per cycle (slow drift allowed).
const RLS_PROCESS_NOISE: f32 = 1e-6;

impl RlsModel {
    /// Starts from the datasheet values with reflected rotor inertia `inertia`.
    #[must_use]
    pub fn new(viscous: f32, coulomb: f32, inertia: f32, dt: f32) -> Self {
        let mut p = [[0.0; N]; N];
        for (k, row) in p.iter_mut().enumerate() {
            row[k] = RLS_INITIAL_COVARIANCE;
        }
        Self {
            theta: [viscous, coulomb, 0.0],
            p,
            inertia,
            measurement: LowPass::new(RLS_FILTER_HZ, dt),
            regressors: [LowPass::new(RLS_FILTER_HZ, dt); N],
        }
    }

    /// Fitted `[viscous, Coulomb, offset]`.
    #[must_use]
    pub const fn parameters(&self) -> [f32; N] {
        self.theta
    }
}

impl DynamicsModel for RlsModel {
    fn observe(&mut self, e: &Estimate) {
        let measured = e.motor_torque + e.gear_torque - self.inertia * e.motor_acceleration;
        let y = self.measurement.update(measured);
        let raw = [e.motor_velocity, smooth_sign(e.motor_velocity), 1.0];
        let phi: [f32; N] = core::array::from_fn(|k| self.regressors[k].update(raw[k]));
        let p_phi: [f32; N] = core::array::from_fn(|i| dot(&self.p[i], &phi));
        let innovation_variance = RLS_MEASUREMENT_NOISE + dot(&phi, &p_phi);
        let error = y - dot(&self.theta, &phi);
        for (theta, p_phi_i) in self.theta.iter_mut().zip(p_phi) {
            *theta += p_phi_i / innovation_variance * error;
        }
        for (i, row) in self.p.iter_mut().enumerate() {
            for (j, entry) in row.iter_mut().enumerate() {
                *entry -= p_phi[i] * p_phi[j] / innovation_variance;
            }
            row[i] = row[i].max(0.0) + RLS_PROCESS_NOISE;
        }
    }

    fn friction(&self, _motor_angle: f32, v: f32) -> f32 {
        self.theta[0] * v + self.theta[1] * smooth_sign(v)
    }
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::{DynamicsModel, NominalModel, RlsModel};
    use crate::estimate::Estimate;

    #[test]
    fn rls_recovers_textbook_friction_from_the_torque_balance() {
        let dt = 0.00025;
        let (viscous, coulomb, inertia) = (0.8, 1.2, 0.02);
        let mut model = RlsModel::new(0.1, 0.1, inertia, dt);
        for k in 0..80_000_u32 {
            #[allow(clippy::cast_precision_loss)]
            let t = k as f32 * dt;
            let v = 2.0 * libm::sinf(2.0 * t) + 0.3;
            let a = 4.0 * libm::cosf(2.0 * t);
            let friction = NominalModel::new(viscous, coulomb).friction(0.0, v);
            let motor_torque = inertia * a + friction; // gear unloaded
            model.observe(&Estimate {
                motor_velocity: v,
                motor_acceleration: a,
                motor_torque,
                ..Estimate::default()
            });
        }
        let [b, fc, _] = model.parameters();
        assert!((b - viscous).abs() < 0.05, "viscous {b}");
        assert!((fc - coulomb).abs() < 0.05, "coulomb {fc}");
    }
}
