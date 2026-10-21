//! The coordinated model (NX-31): the joint's share of its neighbourhood's dynamics,
//! learned from neighbourhood commissioning (`learning/src/nexus_learning/coordinated.py`).
//!
//! Two things, both used as feed-forward from what the joints already publish:
//!
//! - **Its row of the neighbourhood's inertia**, `m_ij(q_N)` over the joints up to two
//!   places away, linear in Fourier features of their angles. With the references the
//!   neighbours publish, the joint adds the torque their accelerations and its own need,
//!   `I_rotor·q̈_i + Σ_j m_ij · q̈_j`. Gravity stays with the gravity chain.
//! - **Its link's compliance**, `b = c · τ`, learned from the next joint's gyro (which
//!   sits at the link's far end). The joint predicts its link's bend from the torque it
//!   is about to deliver and publishes it in its chain frame. It takes `split` of its own
//!   link's bend out of its angle and the rest of its inner neighbour's (when their axes
//!   are parallel): exact for a bend concentrated at mid-length.
//!
//! Every joint publishes its reference acceleration as the smoothed derivative of its
//! reference velocity ([`ReferenceAcceleration`]): the interpolators' own accelerations
//! spike (a setpoint arriving a cycle after the last, a quintic segment ending at zero
//! acceleration), and summed over a neighbourhood those spikes cost more than the
//! coupling gained. A neighbour's acceleration counts only while its frame is at most
//! [`MAX_AGE`] cycles old; the angle of its latest frame is used however old it is.

use crate::coordinated::NearStates;
use crate::filters::LowPass2;
use crate::phnn::ModelError;
use crate::reference::Reference;

/// Weight format version this node reads.
pub const COORDINATED_MODEL_VERSION: u32 = 1;
/// Most joints a model reads, its own included (two either side).
pub const MAX_NEIGHBOURS: usize = 5;
/// Most features over [`MAX_NEIGHBOURS`] angles.
pub const MAX_FEATURES: usize = feature_count(MAX_NEIGHBOURS);
/// Header words: version, joint, neighbour count, compliance, split.
const HEADER: usize = 5;
/// Most words in a model block.
pub const COORDINATED_MODEL_WORDS: usize = HEADER + MAX_NEIGHBOURS + MAX_NEIGHBOURS * MAX_FEATURES;
/// Cycles a neighbour's reference acceleration is used for.
pub const MAX_AGE: u32 = 8;
/// Corner of the reference acceleration's low-pass, as a share of the goal rate. The
/// accelerations of 15 Hz goals ripple at the goal rate; fed forward unfiltered they
/// raised the joints' vibration fivefold.
const ACCELERATION_SHARE: f32 = 1.0 / 6.0;
/// The low-pass corner without a known goal rate, and its ceiling, Hz.
const ACCELERATION_HZ_MAX: f32 = 10.0;
/// Its floor, Hz.
const ACCELERATION_HZ_MIN: f32 = 2.0;

/// Number of features over `joints` angles: a constant, cos and sin of each angle, then
/// cos and sin of every pair's sum and difference.
#[must_use]
pub const fn feature_count(joints: usize) -> usize {
    1 + 2 * joints + 4 * (joints * joints.saturating_sub(1) / 2)
}

/// Fills `out` with the features of `angles` (as the trainer's `features`); returns how
/// many it wrote.
pub fn features(angles: &[f32], out: &mut [f32; MAX_FEATURES]) -> usize {
    let k = angles.len().min(MAX_NEIGHBOURS);
    let mut n = 0;
    let mut push = |v: f32, n: &mut usize| {
        out[*n] = v;
        *n += 1;
    };
    push(1.0, &mut n);
    for &q in &angles[..k] {
        push(libm::cosf(q), &mut n);
        push(libm::sinf(q), &mut n);
    }
    for a in 0..k {
        for b in a + 1..k {
            let (s, d) = (angles[a] + angles[b], angles[a] - angles[b]);
            push(libm::cosf(s), &mut n);
            push(libm::sinf(s), &mut n);
            push(libm::cosf(d), &mut n);
            push(libm::sinf(d), &mut n);
        }
    }
    n
}

/// The joint's reference acceleration as it publishes it: the derivative of its reference
/// velocity through a second-order low-pass whose corner follows the goal rate.
#[derive(Clone, Copy, Debug)]
pub struct ReferenceAcceleration {
    dt: f32,
    corner_hz: f32,
    last_velocity: Option<f32>,
    filter: LowPass2,
}

impl ReferenceAcceleration {
    /// For a bus cycle of `dt`.
    #[must_use]
    pub fn new(dt: f32) -> Self {
        Self {
            dt,
            corner_hz: ACCELERATION_HZ_MAX,
            last_velocity: None,
            filter: LowPass2::new(ACCELERATION_HZ_MAX, dt),
        }
    }

    /// The acceleration this cycle for reference velocity `velocity` and the goal rate.
    pub fn update(&mut self, velocity: f32, goal_hz: Option<f32>) -> f32 {
        let corner = goal_hz.map_or(ACCELERATION_HZ_MAX, |hz| {
            (hz * ACCELERATION_SHARE).clamp(ACCELERATION_HZ_MIN, ACCELERATION_HZ_MAX)
        });
        if (corner - self.corner_hz).abs() > 0.1 {
            self.corner_hz = corner;
            self.filter.set_cutoff(corner, self.dt);
        }
        let raw = self.last_velocity.map_or(0.0, |last| (velocity - last) / self.dt);
        self.last_velocity = Some(velocity);
        self.filter.update(raw)
    }
}

/// A loaded coordinated model.
#[derive(Clone, Copy, Debug)]
pub struct CoordinatedModel {
    count: usize,
    /// Line positions of the joints read, ascending, own included.
    positions: [u8; MAX_NEIGHBOURS],
    weights: [f32; MAX_NEIGHBOURS * MAX_FEATURES],
    compliance: f32,
    split: f32,
}

impl CoordinatedModel {
    /// Reads a model block for the joint at line position `joint`.
    ///
    /// # Errors
    ///
    /// When the block is too short, of another version or joint, reads more joints than a
    /// node holds or not its own, or is not finite.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub fn from_words(words: &[f32], joint: u8) -> Result<Self, ModelError> {
        let at = |i: usize| -> Result<f32, ModelError> {
            let v = *words.get(i).ok_or(ModelError::TooShort)?;
            if v.is_finite() {
                Ok(v)
            } else {
                Err(ModelError::Invalid)
            }
        };
        if at(0)? as u32 != COORDINATED_MODEL_VERSION {
            return Err(ModelError::Version);
        }
        if at(1)? as u32 != u32::from(joint) {
            return Err(ModelError::WrongJoint);
        }
        let count = at(2)? as usize;
        if count == 0 || count > MAX_NEIGHBOURS {
            return Err(ModelError::Invalid);
        }
        let (compliance, split) = (at(3)?, at(4)?);
        if !(0.0..=1.0).contains(&split) {
            return Err(ModelError::Invalid);
        }
        let mut positions = [0; MAX_NEIGHBOURS];
        for (k, p) in positions.iter_mut().enumerate().take(count) {
            *p = at(HEADER + k)? as u8;
        }
        if !positions[..count].contains(&joint) || positions[..count].windows(2).any(|w| w[0] >= w[1]) {
            return Err(ModelError::Invalid);
        }
        let features = feature_count(count);
        let mut weights = [0.0; MAX_NEIGHBOURS * MAX_FEATURES];
        for (i, w) in weights.iter_mut().enumerate().take(count * features) {
            *w = at(HEADER + count + i)?;
        }
        Ok(Self { count, positions, weights, compliance, split })
    }

    /// Line positions of the joints the model reads, own included.
    #[must_use]
    pub fn positions(&self) -> &[u8] {
        &self.positions[..self.count]
    }

    /// `Σ_j m_ij(q) · q̈_j` at the neighbourhood's `angles` and `accelerations` (in the
    /// order of [`Self::positions`]).
    #[must_use]
    pub fn inertial_torque(&self, angles: &[f32], accelerations: &[f32]) -> f32 {
        let mut phi = [0.0; MAX_FEATURES];
        let f = features(&angles[..self.count.min(angles.len())], &mut phi);
        let mut torque = 0.0;
        for (j, &acc) in accelerations.iter().enumerate().take(self.count) {
            let row = &self.weights[j * f..(j + 1) * f];
            torque += acc * row.iter().zip(&phi[..f]).map(|(w, x)| w * x).sum::<f32>();
        }
        torque
    }

    /// The bend of the joint's link for `torque` delivered to it, rad.
    #[must_use]
    pub fn bend(&self, torque: f32) -> f32 {
        self.compliance * torque
    }

    /// Share of its own link's bend the joint takes out.
    #[must_use]
    pub const fn split(&self) -> f32 {
        self.split
    }
}

/// What the coordinated model adds this cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CoordinatedFeedforward {
    /// Inertial torque to add, N·m.
    pub torque: f32,
    /// Predicted bend of the joint's own link, rad (published in the chain frame).
    pub bend: f32,
    /// Shift of the joint's reference angle that takes the bends out, rad.
    pub shift: f32,
}

/// The coordinated model at work.
#[derive(Clone, Copy, Debug)]
pub struct CoordinatedFeedforwardStage {
    model: Option<CoordinatedModel>,
    rotor_inertia: f32,
}

impl CoordinatedFeedforwardStage {
    /// Stage for a joint whose reflected rotor inertia is `rotor_inertia`.
    #[must_use]
    pub const fn new(rotor_inertia: f32) -> Self {
        Self { model: None, rotor_inertia }
    }

    /// Takes `model`.
    pub fn load(&mut self, model: CoordinatedModel) {
        self.model = Some(model);
    }

    /// Whether a model is loaded.
    #[must_use]
    pub const fn loaded(&self) -> bool {
        self.model.is_some()
    }

    /// One cycle: the own `reference` and its published `acceleration`, the neighbours'
    /// frames in `near`, the joint's line position `own`, the torque the gravity chain
    /// holds `hold`, and the inner neighbour's published bend `inner_bend` (already scaled
    /// by how parallel its axis is to ours).
    #[allow(clippy::too_many_arguments)]
    #[must_use]
    pub fn update(
        &self,
        reference: &Reference,
        acceleration: f32,
        near: &NearStates,
        own: u8,
        now: u32,
        hold: f32,
        inner_bend: Option<f32>,
    ) -> CoordinatedFeedforward {
        let Some(model) = self.model else { return CoordinatedFeedforward::default() };
        let mut angles = [0.0; MAX_NEIGHBOURS];
        let mut accelerations = [0.0; MAX_NEIGHBOURS];
        for (k, &position) in model.positions().iter().enumerate() {
            (angles[k], accelerations[k]) = if position == own {
                (reference.position, acceleration)
            } else {
                match near.latest(position) {
                    // A late neighbour's acceleration is taken as zero.
                    Some((state, cycle)) => {
                        let fresh = now.wrapping_sub(cycle) <= MAX_AGE;
                        (state.reference[0], if fresh { state.reference[2] } else { 0.0 })
                    }
                    // Never heard from: no feed-forward until it has spoken.
                    None => return CoordinatedFeedforward::default(),
                }
            };
        }
        let torque = self.rotor_inertia * acceleration
            + model.inertial_torque(&angles[..model.count], &accelerations[..model.count]);
        let bend = model.bend(hold + torque);
        let shift = -model.split() * bend - (1.0 - model.split()) * inner_bend.unwrap_or(0.0);
        CoordinatedFeedforward { torque, bend, shift }
    }
}

#[cfg(test)]
mod tests {
    use super::{feature_count, features, CoordinatedModel, MAX_FEATURES};

    #[allow(clippy::cast_precision_loss)] // a handful of joints
    fn block(joint: u8, positions: &[u8], weights: &[f32], compliance: f32) -> [f32; 64] {
        let mut words = [0.0; 64];
        words[..5].copy_from_slice(&[1.0, f32::from(joint), positions.len() as f32, compliance, 0.5]);
        for (k, p) in positions.iter().enumerate() {
            words[5 + k] = f32::from(*p);
        }
        words[5 + positions.len()..5 + positions.len() + weights.len()].copy_from_slice(weights);
        words
    }

    #[test]
    fn features_match_the_trainers_order() {
        let mut out = [0.0; MAX_FEATURES];
        let n = features(&[0.3, -0.2], &mut out);
        assert_eq!(n, feature_count(2));
        let expect = [
            1.0,
            0.3_f32.cos(),
            0.3_f32.sin(),
            (-0.2_f32).cos(),
            (-0.2_f32).sin(),
            0.1_f32.cos(),
            0.1_f32.sin(),
            0.5_f32.cos(),
            0.5_f32.sin(),
        ];
        for (a, b) in out[..n].iter().zip(expect) {
            assert!((a - b).abs() < 1e-6, "{a} {b}");
        }
    }

    #[test]
    fn a_constant_row_gives_the_torque_of_its_inertias() {
        // Own joint at position 2, inner neighbour at 1; m_21 = 0.5, m_22 = 2 (constant).
        let f = feature_count(2);
        let mut weights = [0.0; 18];
        weights[0] = 0.5;
        weights[f] = 2.0;
        let words = block(2, &[1, 2], &weights[..2 * f], -1e-4);
        let model = CoordinatedModel::from_words(&words, 2).expect("valid block");
        let torque = model.inertial_torque(&[0.4, 1.1], &[3.0, -1.0]);
        assert!((torque - (0.5 * 3.0 - 2.0)).abs() < 1e-5, "{torque}");
        assert!((model.bend(20.0) + 2e-3).abs() < 1e-7);
        assert!(CoordinatedModel::from_words(&words, 3).is_err(), "another joint's model");
    }
}
