//! The joint's learned port-Hamiltonian model of itself (B11), evaluated in the node.
//!
//! Trained offline from the joint's own commissioning log (`learning/`), frozen and loaded
//! as a flat block of `f32` words ([`PhnnModel::from_words`]). The model gives the motor
//! side of the joint as
//!
//! ```text
//! ṗ = g(T, θ)·u − V_cog'(θ) + V_gear'(φ) + c·φ̇ − R(ω)·ω
//! ```
//!
//! with `R ≥ 0`, `c ≥ 0` and `g > 0` by construction, so it cannot create energy. The node
//! uses it for friction and cogging feed-forward and for the motor-side residual that
//! source attribution works on (`source.rs`). The safety path never depends on it: the
//! damping law and its energy tank stay model-free (B5).
//!
//! The three networks are small tanh MLPs (two hidden layers). The gear spring's torque
//! needs the derivative of its network with respect to its input, computed analytically.

use crate::dynamics::{DynamicsModel, MotorBalance};
use crate::estimate::Estimate;

/// Hidden width of the gear and friction networks.
pub const HIDDEN: usize = 16;
/// Hidden width of the gain's temperature network.
pub const GAIN_HIDDEN: usize = 8;
/// Most cogging orders a model can carry.
pub const MAX_COG_ORDERS: usize = 8;
/// Weight format version this node reads.
pub const FORMAT_VERSION: u32 = 1;
/// Words in a model block of this format.
pub const MODEL_WORDS: usize =
    3 + CONSTANTS + 3 * MAX_COG_ORDERS + 3 * Mlp::<1, HIDDEN>::WORDS + 2 + 4 + Mlp::<1, GAIN_HIDDEN>::WORDS;
/// Constant words after the header.
const CONSTANTS: usize = 11;
/// Largest gain change the model may claim: `g = exp(±0.2)`.
const GAIN_RANGE: f32 = 0.2;

/// Why a model block was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ModelError {
    /// Fewer words than the format needs.
    TooShort,
    /// A format version this node cannot read.
    Version,
    /// Weights for another joint.
    WrongJoint,
    /// More cogging orders than the node can carry.
    TooManyOrders,
    /// A word is not finite, or a scale is not positive.
    Invalid,
}

/// Reads words in order.
struct Words<'a> {
    words: &'a [f32],
    next: usize,
}

impl Words<'_> {
    fn take(&mut self) -> Result<f32, ModelError> {
        let value = *self.words.get(self.next).ok_or(ModelError::TooShort)?;
        self.next += 1;
        if value.is_finite() {
            Ok(value)
        } else {
            Err(ModelError::Invalid)
        }
    }

    fn array<const N: usize>(&mut self) -> Result<[f32; N], ModelError> {
        let mut out = [0.0; N];
        for x in &mut out {
            *x = self.take()?;
        }
        Ok(out)
    }
}

/// A two-hidden-layer tanh network with one output. Weights are stored as `PyTorch` does:
/// each layer's matrix row by row (one row per output), then its bias.
#[derive(Clone, Copy, Debug)]
pub struct Mlp<const IN: usize, const H: usize> {
    w1: [[f32; IN]; H],
    b1: [f32; H],
    w2: [[f32; H]; H],
    b2: [f32; H],
    w3: [f32; H],
    b3: f32,
}

impl<const IN: usize, const H: usize> Mlp<IN, H> {
    /// Words this network takes in a model block.
    pub const WORDS: usize = IN * H + H + H * H + H + H + 1;

    fn read(words: &mut Words) -> Result<Self, ModelError> {
        let mut w1 = [[0.0; IN]; H];
        for row in &mut w1 {
            *row = words.array()?;
        }
        let b1 = words.array()?;
        let mut w2 = [[0.0; H]; H];
        for row in &mut w2 {
            *row = words.array()?;
        }
        let b2 = words.array()?;
        let w3 = words.array()?;
        let b3 = words.take()?;
        Ok(Self { w1, b1, w2, b2, w3, b3 })
    }

    fn hidden(&self, x: &[f32; IN]) -> ([f32; H], [f32; H]) {
        let h1: [f32; H] = core::array::from_fn(|i| {
            libm::tanhf(self.b1[i] + self.w1[i].iter().zip(x).map(|(w, x)| w * x).sum::<f32>())
        });
        let h2: [f32; H] = core::array::from_fn(|i| {
            libm::tanhf(self.b2[i] + self.w2[i].iter().zip(&h1).map(|(w, h)| w * h).sum::<f32>())
        });
        (h1, h2)
    }

    /// The network's output at `x`.
    #[must_use]
    pub fn value(&self, x: &[f32; IN]) -> f32 {
        let (_, h2) = self.hidden(x);
        self.b3 + self.w3.iter().zip(&h2).map(|(w, h)| w * h).sum::<f32>()
    }

    /// The output's gradient with respect to the input at `x`.
    #[must_use]
    pub fn gradient(&self, x: &[f32; IN]) -> [f32; IN] {
        let (h1, h2) = self.hidden(x);
        // Back through the output layer and the second tanh, then the first.
        let d2: [f32; H] = core::array::from_fn(|i| self.w3[i] * (1.0 - h2[i] * h2[i]));
        let d1: [f32; H] = core::array::from_fn(|j| {
            (1.0 - h1[j] * h1[j]) * (0..H).map(|i| d2[i] * self.w2[i][j]).sum::<f32>()
        });
        core::array::from_fn(|k| (0..H).map(|j| d1[j] * self.w1[j][k]).sum())
    }
}

fn softplus(x: f32) -> f32 {
    if x > 20.0 {
        x
    } else {
        libm::log1pf(libm::expf(x))
    }
}

/// The learned model of one joint.
#[derive(Clone, Copy, Debug)]
pub struct PhnnModel {
    rated_torque: f32,
    stiffness: f32,
    twist_ref: f32,
    omega_ref: f32,
    stiction_speed: f32,
    temperature_ref: [f32; 2],
    viscous_scale: f32,
    coulomb_scale: f32,
    inertia: f32,
    gear_damping: f32,
    cog_count: usize,
    cog_orders: [f32; MAX_COG_ORDERS],
    cog_sin: [f32; MAX_COG_ORDERS],
    cog_cos: [f32; MAX_COG_ORDERS],
    gear: Mlp<1, HIDDEN>,
    viscous: Mlp<1, HIDDEN>,
    coulomb: Mlp<1, HIDDEN>,
    gain_orders: [f32; 2],
    gain_angle: [f32; 4],
    gain_temperature: Mlp<1, GAIN_HIDDEN>,
    /// Winding temperature last observed, °C.
    temperature: f32,
}

impl PhnnModel {
    /// Reads a model block for the joint at bus position `joint` (1-based).
    ///
    /// # Errors
    ///
    /// When the block is too short, of another version or joint, or not finite.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub fn from_words(words: &[f32], joint: u8) -> Result<Self, ModelError> {
        let mut w = Words { words, next: 0 };
        if w.take()? as u32 != FORMAT_VERSION {
            return Err(ModelError::Version);
        }
        if w.take()? as u32 != u32::from(joint) {
            return Err(ModelError::WrongJoint);
        }
        let cog_count = w.take()? as usize;
        if cog_count > MAX_COG_ORDERS {
            return Err(ModelError::TooManyOrders);
        }
        let [rated_torque, stiffness, twist_ref, omega_ref, stiction_speed, t0, t1, viscous_scale, coulomb_scale, inertia, gear_damping] =
            w.array::<CONSTANTS>()?;
        let positive = [rated_torque, stiffness, twist_ref, omega_ref, stiction_speed, t1, inertia];
        if positive.iter().any(|x| *x <= 0.0)
            || [viscous_scale, coulomb_scale, gear_damping].iter().any(|x| *x < 0.0)
        {
            return Err(ModelError::Invalid);
        }
        Ok(Self {
            rated_torque,
            stiffness,
            twist_ref,
            omega_ref,
            stiction_speed,
            temperature_ref: [t0, t1],
            viscous_scale,
            coulomb_scale,
            inertia,
            gear_damping,
            cog_count,
            cog_orders: w.array()?,
            cog_sin: w.array()?,
            cog_cos: w.array()?,
            gear: Mlp::read(&mut w)?,
            viscous: Mlp::read(&mut w)?,
            coulomb: Mlp::read(&mut w)?,
            gain_orders: w.array()?,
            gain_angle: w.array()?,
            gain_temperature: Mlp::read(&mut w)?,
            temperature: t0,
        })
    }

    /// The same model at winding temperature `temperature` (°C).
    #[must_use]
    pub const fn at_temperature(mut self, temperature: f32) -> Self {
        self.temperature = temperature;
        self
    }

    /// Rotor inertia the model learned, output-referred, kg·m².
    #[must_use]
    pub const fn inertia(&self) -> f32 {
        self.inertia
    }

    /// Cogging torque on the rotor, `−V_cog'(θ)`, N·m.
    #[must_use]
    pub fn cogging(&self, theta: f32) -> f32 {
        let slope: f32 = (0..self.cog_count)
            .map(|k| {
                let n = self.cog_orders[k];
                let (sin, cos) = libm::sincosf(n * theta);
                n * (self.cog_sin[k] * cos - self.cog_cos[k] * sin)
            })
            .sum();
        -self.rated_torque * slope
    }

    /// Gear torque on the rotor, `V_gear'(φ) + c·φ̇`, N·m.
    #[must_use]
    pub fn gear_torque(&self, twist: f32, twist_rate: f32) -> f32 {
        let correction = self.gear.gradient(&[twist / self.twist_ref])[0];
        self.stiffness * twist + self.rated_torque * correction + self.gear_damping * twist_rate
    }

    /// Friction torque opposing the motor, `R(ω)·ω`, N·m; `R ≥ 0`.
    #[must_use]
    pub fn friction_torque(&self, omega: f32) -> f32 {
        let x = [omega / self.omega_ref];
        let viscous = softplus(self.viscous.value(&x)) * self.viscous_scale;
        let coulomb = softplus(self.coulomb.value(&x)) * self.coulomb_scale;
        (viscous + coulomb / libm::sqrtf(omega * omega + self.stiction_speed * self.stiction_speed)) * omega
    }

    /// Torque produced per N·m commanded at angle `theta` and the last temperature.
    #[must_use]
    pub fn gain(&self, theta: f32) -> f32 {
        let t = (self.temperature - self.temperature_ref[0]) / self.temperature_ref[1];
        let (s1, c1) = libm::sincosf(self.gain_orders[0] * theta);
        let (s2, c2) = libm::sincosf(self.gain_orders[1] * theta);
        let angle = self.gain_angle[0] * s1
            + self.gain_angle[1] * s2
            + self.gain_angle[2] * c1
            + self.gain_angle[3] * c2;
        libm::expf(GAIN_RANGE * libm::tanhf(self.gain_temperature.value(&[t]) + angle))
    }
}

impl DynamicsModel for PhnnModel {
    fn observe(&mut self, estimate: &Estimate) {
        self.temperature = estimate.temperature;
    }

    /// The command that cancels friction and cogging: `(R·ω − cogging) / g`.
    fn friction(&self, motor_angle: f32, motor_velocity: f32) -> f32 {
        (self.friction_torque(motor_velocity) - self.cogging(motor_angle)) / self.gain(motor_angle)
    }

    fn unexplained(&self, b: &MotorBalance) -> f32 {
        let produced =
            self.gain(b.angle) * b.applied + self.cogging(b.angle) + self.gear_torque(b.twist, b.twist_rate)
                - self.friction_torque(b.velocity);
        self.inertia * b.acceleration - produced
    }

    /// The learned gear, without its damping.
    fn spring_torque(&self, _stiffness: f32, twist: f32) -> f32 {
        self.gear_torque(twist, 0.0)
    }
}

#[cfg(test)]
#[allow(clippy::cast_precision_loss)]
mod tests {
    use super::{Mlp, ModelError, PhnnModel, Words, FORMAT_VERSION, MODEL_WORDS};
    use crate::dynamics::DynamicsModel;

    /// A model block with every network at zero except the given first-layer weights.
    fn block(joint: u8) -> [f32; MODEL_WORDS] {
        let mut w = [0.0; MODEL_WORDS];
        w[0] = FORMAT_VERSION as f32;
        w[1] = f32::from(joint);
        w[2] = 1.0; // one cogging order
                    // rated, k, twist_ref, omega_ref, stiction, T0, T1, viscous, coulomb, inertia, c
        w[3..14].copy_from_slice(&[12.0, 6000.0, 0.002, 2.0, 0.01, 25.0, 50.0, 0.05, 0.3, 0.005, 0.6]);
        w[14] = 36.0; // cogging order
        w[22] = 0.01; // its sine weight
        w
    }

    #[test]
    fn mlp_gradient_matches_finite_differences() {
        let words: [f32; Mlp::<1, 16>::WORDS] = core::array::from_fn(|k| libm::sinf(k as f32 * 0.7) * 0.5);
        let mlp = Mlp::<1, 16>::read(&mut Words { words: &words, next: 0 }).unwrap();
        for x in [-1.3_f32, 0.0, 0.4, 2.1] {
            let h = 1e-3;
            let numeric = (mlp.value(&[x + h]) - mlp.value(&[x - h])) / (2.0 * h);
            assert!((mlp.gradient(&[x])[0] - numeric).abs() < 1e-3, "{x}");
        }
    }

    #[test]
    fn reads_a_block_and_evaluates_the_physics() {
        let model = PhnnModel::from_words(&block(4), 4).unwrap();
        // Zero networks: friction is softplus(0)·scale, spring is linear, gain is 1.
        let ln2 = core::f32::consts::LN_2;
        let expected = (0.05 * ln2 + 0.3 * ln2 / libm::sqrtf(1.0 + 1e-4)) * 1.0;
        assert!((model.friction_torque(1.0) - expected).abs() < 1e-5);
        assert!((model.gear_torque(1e-3, 0.0) - 6.0).abs() < 1e-4);
        // A joint using this model reads its gear torque from the learned gear, not the
        // datasheet stiffness it is given.
        assert!((model.spring_torque(1.0e9, 1e-3) - 6.0).abs() < 1e-4);
        assert!((model.gain(0.3) - 1.0).abs() < 1e-6);
        // V_cog = 12·0.01·sin(36θ): torque −12·0.01·36·cos(36θ).
        assert!((model.cogging(0.0) + 12.0 * 0.01 * 36.0).abs() < 1e-4);
    }

    #[test]
    fn friction_never_feeds_energy_in() {
        let mut words = block(2);
        for (k, w) in words.iter_mut().enumerate().skip(40) {
            *w = libm::sinf(k as f32 * 1.3);
        }
        let model = PhnnModel::from_words(&words, 2).unwrap();
        for k in 0..200 {
            let omega = (k as f32 - 100.0) * 0.05;
            assert!(model.friction_torque(omega) * omega >= 0.0);
        }
    }

    #[test]
    fn refuses_blocks_it_cannot_trust() {
        assert_eq!(PhnnModel::from_words(&block(4)[..100], 4).unwrap_err(), ModelError::TooShort);
        assert_eq!(PhnnModel::from_words(&block(4), 5).unwrap_err(), ModelError::WrongJoint);
        let mut bad = block(4);
        bad[0] = 2.0;
        assert_eq!(PhnnModel::from_words(&bad, 4).unwrap_err(), ModelError::Version);
        let mut nan = block(4);
        nan[500] = f32::NAN;
        assert_eq!(PhnnModel::from_words(&nan, 4).unwrap_err(), ModelError::Invalid);
        let mut negative = block(4);
        negative[12] = -1.0; // inertia
        assert_eq!(PhnnModel::from_words(&negative, 4).unwrap_err(), ModelError::Invalid);
    }
}
