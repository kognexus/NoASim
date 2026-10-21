//! Coordinated damping (NX-30): a Kalman filter on the state frames of the joints up to
//! two places away, and this joint's row of a gain designed on the same model.
//!
//! The model, the filter and the gain are designed offline for the arm holding at one
//! pose (`sim/src/analysis/estimator.ts`) and loaded as a block of words, like the learned
//! models. The model is the arm with its joints' position loops, the dry friction as
//! equivalent damping, and every joint's damping command through a second-order low-pass
//! acting a cycle late. Its state `z` is the arm's, then those loops' torques, then the
//! command filters.
//!
//! Every cycle while the arm holds, with `y` the measurements of the joints near this one
//! (motor angle, twist, gyro; what arrives in their latest frames, a cycle old) and `u`
//! every joint's command:
//!
//! ```text
//! ẑ ← A ẑ + B u(two cycles ago) + L (y − C ẑ)      filter, at the measurements' time
//! z_now = A ẑ + B u(one cycle ago)                  carried forward
//! u = −K z_now                                      this joint's row is its command
//! ```
//!
//! The joints publish their commands in their state frames, so the commands of the
//! joints near this one are read; the others' are predicted from this joint's estimate.
//! The command is limited to the host's share of the torque limit and published; it then
//! goes through the same low-pass as in the model and the energy tank.
//!
//! The model holds for the arm at rest, so the damper acts only while the joint holds
//! still: after `SETTLE_CYCLES` of holding it averages the rest values of its
//! measurements for the model's calibration time, then acts until the joint moves again.
//! Without a model, while moving or calibrating, or when a near joint's data is late, the
//! joint uses network damping.

use crate::damping::EnergyTank;
use nexus_proto::{NodeId, NodeState};

/// Most states the model may have.
pub const MAX_STATES: usize = 72;
/// Most joints.
pub const MAX_JOINTS: usize = 8;
/// Most measurement rows one joint uses (five joints, five channels each).
pub const MAX_ROWS: usize = 25;
/// Words in the header of a model block.
const HEADER: usize = 8;
/// Most words in a model block.
pub const COORDINATED_WORDS: usize = HEADER
    + 2 * MAX_ROWS
    + MAX_STATES * MAX_STATES
    + MAX_STATES * MAX_JOINTS
    + MAX_ROWS * MAX_STATES
    + MAX_STATES * MAX_ROWS
    + MAX_JOINTS * MAX_STATES;
/// Format version this node reads.
pub const COORDINATED_FORMAT: u32 = 1;
/// Cycles of holding still before calibration starts.
const SETTLE_CYCLES: u32 = 1000;
/// Oldest state frame of a near joint the filter uses, cycles. With older data the joint
/// uses network damping: on a bus losing a fifth of its frames with 4 cycles of jitter, a
/// filter fed late data (up to 16 cycles) helped at two postures and oscillated at a
/// third, because its commands and measurements no longer belong to the same cycles.
const MAX_AGE: u32 = 4;
/// Below this reference speed the joint counts as holding, rad/s.
const HOLDING_SPEED: f32 = 1e-6;
/// Energy the damper's tank may hold, J (as the network term's).
const TANK_CAPACITY: f32 = 0.05;

/// Why a model block was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoordinatedError {
    /// Fewer words than the header says.
    TooShort,
    /// Another format version.
    Version,
    /// A model for another joint.
    WrongJoint,
    /// More states, joints or rows than the node can hold.
    TooLarge,
}

/// The latest state frames of the joints, by line position, with each one's previous
/// command (the filter needs the commands of two cycles ago).
#[derive(Clone, Copy, Debug)]
pub struct NearStates {
    slots: [Slot; MAX_JOINTS + 1],
}

#[derive(Clone, Copy, Debug, Default)]
struct Slot {
    id: Option<NodeId>,
    state: Option<NodeState>,
    cycle: u32,
    previous_command: f32,
}

impl Default for NearStates {
    fn default() -> Self {
        Self { slots: [Slot::default(); MAX_JOINTS + 1] }
    }
}

impl NearStates {
    /// Records which node sits at `position`.
    pub fn on_announce(&mut self, position: u8, id: NodeId) {
        if let Some(slot) = self.slots.get_mut(usize::from(position)) {
            slot.id = Some(id);
        }
    }

    /// Records a state frame from `source`, sent at `cycle`.
    pub fn on_state(&mut self, source: NodeId, state: &NodeState, cycle: u32) {
        for slot in &mut self.slots {
            if slot.id == Some(source)
                && (slot.state.is_none() || cycle.wrapping_sub(slot.cycle) < u32::MAX / 2)
            {
                slot.previous_command = slot.state.map_or(0.0, |s| s.network_torque);
                slot.state = Some(*state);
                slot.cycle = cycle;
            }
        }
    }

    /// The latest frame of the joint at `position` and the cycle it was sent, however old.
    pub(crate) fn latest(&self, position: u8) -> Option<(&NodeState, u32)> {
        let slot = self.slots.get(usize::from(position))?;
        Some((slot.state.as_ref()?, slot.cycle))
    }

    /// The latest frame of the joint at `position`, if it is at most `max_age` old.
    fn fresh(&self, position: usize, now: u32, max_age: u32) -> Option<(&NodeState, f32)> {
        let slot = self.slots.get(position)?;
        let state = slot.state.as_ref()?;
        (now.wrapping_sub(slot.cycle) <= max_age).then_some((state, slot.previous_command))
    }
}

/// One measurement of a joint's frame.
fn channel(state: &NodeState, channel: usize) -> f32 {
    match channel {
        0 => state.motor_angle,
        1 => state.gear_twist,
        c => state.gyro.get(c - 2).copied().unwrap_or(0.0),
    }
}

/// The sizes of a loaded model; its matrices are indexed in place in the damper's words.
#[derive(Clone, Copy, Debug)]
struct Dims {
    n: usize,
    joints: usize,
    rows: usize,
    alpha: f32,
    calibration: u32,
}

impl Dims {
    const fn a_at(&self) -> usize {
        HEADER + 2 * self.rows
    }
    const fn b_at(&self) -> usize {
        self.a_at() + self.n * self.n
    }
    const fn c_at(&self) -> usize {
        self.b_at() + self.n * self.joints
    }
    const fn l_at(&self) -> usize {
        self.c_at() + self.rows * self.n
    }
    const fn k_at(&self) -> usize {
        self.l_at() + self.n * self.rows
    }
}

/// The model's words with its sizes.
struct Model<'a> {
    words: &'a [f32],
    dims: Dims,
}

impl Model<'_> {
    fn get(&self, at: usize) -> f32 {
        self.words.get(at).copied().unwrap_or(0.0)
    }
    /// Line position (1-based) and channel of measurement row `r`.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // small integers stored as f32
    fn row(&self, r: usize) -> (usize, usize) {
        (self.get(HEADER + 2 * r) as usize, self.get(HEADER + 2 * r + 1) as usize)
    }
    /// `out = M·x` for the `rows × cols` matrix stored row by row at `at`.
    fn times(&self, at: usize, rows: usize, cols: usize, x: &[f32], out: &mut [f32]) {
        for (i, o) in out.iter_mut().enumerate().take(rows) {
            let row = at + i * cols;
            *o = x.iter().take(cols).enumerate().map(|(k, v)| self.get(row + k) * v).sum();
        }
    }
}

/// The coordinated damper of one joint.
#[derive(Clone, Debug)]
pub struct Coordinated {
    words: [f32; COORDINATED_WORDS],
    dims: Option<Dims>,
    position: usize,
    holding: u32,
    rest: [f32; MAX_ROWS],
    samples: u32,
    estimate: [f32; MAX_STATES],
    /// This joint's own commands: two cycles ago, one cycle ago.
    own: [f32; 2],
    first: f32,
    second: f32,
    tank: EnergyTank,
    command: f32,
}

impl Coordinated {
    /// The damper of the joint at line `position`, with no model yet.
    #[must_use]
    #[allow(clippy::large_stack_arrays)] // built once, when the node starts
    pub fn new(position: u8, cycle_seconds: f32) -> Self {
        Self {
            words: [0.0; COORDINATED_WORDS],
            dims: None,
            position: usize::from(position),
            holding: 0,
            rest: [0.0; MAX_ROWS],
            samples: 0,
            estimate: [0.0; MAX_STATES],
            own: [0.0; 2],
            first: 0.0,
            second: 0.0,
            tank: EnergyTank::new(TANK_CAPACITY, cycle_seconds),
            command: 0.0,
        }
    }

    /// Copies `words` into the model block being loaded, from word `offset` on. The
    /// model in use, if any, is dropped until the block is finished.
    pub fn load_chunk(&mut self, offset: usize, words: &[f32]) {
        self.dims = None;
        for (slot, w) in self.words.iter_mut().skip(offset).zip(words) {
            *slot = *w;
        }
    }

    /// Checks and takes the loaded block, `length` words, as the model.
    ///
    /// # Errors
    ///
    /// When the block is not a model for this joint that fits the node.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // small integers stored as f32
    pub fn finish_load(&mut self, length: usize) -> Result<(), CoordinatedError> {
        let w = &self.words;
        if length < HEADER {
            return Err(CoordinatedError::TooShort);
        }
        if w[0] as u32 != COORDINATED_FORMAT {
            return Err(CoordinatedError::Version);
        }
        if w[1] as usize != self.position {
            return Err(CoordinatedError::WrongJoint);
        }
        let dims = Dims {
            n: w[2] as usize,
            joints: w[3] as usize,
            rows: w[4] as usize,
            alpha: w[6],
            calibration: w[7] as u32,
        };
        if dims.n > MAX_STATES || dims.joints > MAX_JOINTS || dims.rows > MAX_ROWS {
            return Err(CoordinatedError::TooLarge);
        }
        if length < dims.k_at() + dims.joints * dims.n {
            return Err(CoordinatedError::TooShort);
        }
        self.dims = Some(dims);
        self.reset();
        Ok(())
    }

    /// Whether a model is loaded.
    #[must_use]
    pub const fn loaded(&self) -> bool {
        self.dims.is_some()
    }

    /// The command published in this joint's state frame (0 when not acting).
    #[must_use]
    pub const fn command(&self) -> f32 {
        self.command
    }

    fn reset(&mut self) {
        self.holding = 0;
        self.samples = 0;
        self.estimate = [0.0; MAX_STATES];
        self.own = [0.0; 2];
        self.first = 0.0;
        self.second = 0.0;
        self.command = 0.0;
    }

    /// The damping torque this cycle, or `None` when network damping should act instead.
    /// `output_velocity` is the joint's own (for the tank), `reference_velocity` tells
    /// whether it holds, `cap` is the most damping torque the host allows.
    pub fn torque(
        &mut self,
        near: &NearStates,
        now: u32,
        reference_velocity: f32,
        output_velocity: f32,
        cap: f32,
    ) -> Option<f32> {
        let model = Model { words: &self.words, dims: self.dims? };
        if libm::fabsf(reference_velocity) > HOLDING_SPEED {
            self.reset();
            return None;
        }
        self.holding = self.holding.saturating_add(1);
        if self.holding < SETTLE_CYCLES {
            return None;
        }
        // The near joints' latest measurements; without them, no coordinated damping.
        let mut y = [0.0_f32; MAX_ROWS];
        for (r, value) in y.iter_mut().enumerate().take(model.dims.rows) {
            let (position, ch) = model.row(r);
            let (state, _) = near.fresh(position, now, MAX_AGE)?;
            *value = channel(state, ch);
        }
        if self.samples < model.dims.calibration {
            // Where the arm holds, and each gyro's bias: averaged, before acting.
            self.samples += 1;
            #[allow(clippy::cast_precision_loss)] // a few hundred samples
            let weight = 1.0 / self.samples as f32;
            for (rest, value) in self.rest.iter_mut().zip(y).take(model.dims.rows) {
                *rest += weight * (value - *rest);
            }
            return None;
        }
        for (value, rest) in y.iter_mut().zip(self.rest).take(model.dims.rows) {
            *value -= rest;
        }
        let (n, m, rows) = (model.dims.n, model.dims.joints, model.dims.rows);
        // Every joint's commands two cycles and one cycle ago: read for the near ones,
        // predicted from the estimate for the others.
        let mut guess = [0.0_f32; MAX_JOINTS];
        model.times(model.dims.k_at(), m, n, &self.estimate, &mut guess);
        let mut then = [0.0_f32; MAX_JOINTS];
        let mut since = [0.0_f32; MAX_JOINTS];
        for j in 0..m {
            let position = j + 1;
            if position == self.position {
                then[j] = self.own[0];
                since[j] = self.own[1];
            } else if let Some((state, previous)) = near.fresh(position, now, MAX_AGE) {
                then[j] = previous;
                since[j] = state.network_torque;
            } else {
                then[j] = -guess[j];
                since[j] = -guess[j];
            }
        }
        // Filter at the measurements' time.
        let mut predicted = [0.0_f32; MAX_ROWS];
        model.times(model.dims.c_at(), rows, n, &self.estimate, &mut predicted);
        let mut innovation = [0.0_f32; MAX_ROWS];
        for r in 0..rows {
            innovation[r] = y[r] - predicted[r];
        }
        let mut next = [0.0_f32; MAX_STATES];
        let mut part = [0.0_f32; MAX_STATES];
        model.times(model.dims.a_at(), n, n, &self.estimate, &mut next);
        model.times(model.dims.b_at(), n, m, &then, &mut part);
        for (v, p) in next.iter_mut().zip(part).take(n) {
            *v += p;
        }
        model.times(model.dims.l_at(), n, rows, &innovation, &mut part);
        for (v, p) in next.iter_mut().zip(part).take(n) {
            *v += p;
        }
        self.estimate = next;
        // Carried forward a cycle, then this joint's row of the gain.
        let mut now_state = [0.0_f32; MAX_STATES];
        model.times(model.dims.a_at(), n, n, &self.estimate, &mut now_state);
        model.times(model.dims.b_at(), n, m, &since, &mut part);
        for (v, p) in now_state.iter_mut().zip(part).take(n) {
            *v += p;
        }
        let mut commands = [0.0_f32; MAX_JOINTS];
        model.times(model.dims.k_at(), m, n, &now_state, &mut commands);
        // Limited before the low-pass, so what the model is told acted did (anti-windup:
        // clamped after it, a strong tap left the filter assuming torque that never came).
        let command = (-commands.get(self.position - 1).copied().unwrap_or(0.0)).clamp(-cap, cap);
        self.own = [self.own[1], command];
        self.command = command;
        // The command filter, as in the model; its output acts next cycle.
        self.second += model.dims.alpha * (self.first - self.second);
        self.first += model.dims.alpha * (command - self.first);
        Some(self.tank.pass(self.second, output_velocity))
    }
}

#[cfg(test)]
mod tests {
    use super::{Coordinated, CoordinatedError, NearStates, COORDINATED_FORMAT, HEADER};
    use nexus_proto::{NodeId, NodeState};

    /// A one-joint model with one state (the motor angle) and one measurement.
    #[allow(clippy::cast_precision_loss)] // small integers
    fn block(position: f32) -> [f32; 64] {
        let mut w = [0.0; 64];
        // header: format, joint, n, joints, rows, delay, alpha, calibration
        w[..HEADER].copy_from_slice(&[COORDINATED_FORMAT as f32, position, 1.0, 1.0, 1.0, 2.0, 1.0, 10.0]);
        let rows = HEADER;
        w[rows] = position; // row 0: this joint, motor angle
        w[rows + 1] = 0.0;
        let a = rows + 2;
        w[a] = 1.0; // A
        w[a + 1] = 0.0; // B
        w[a + 2] = 1.0; // C
        w[a + 3] = 1.0; // L: trust the measurement fully
        w[a + 4] = 2.0; // K
        w
    }

    #[test]
    fn refuses_a_model_for_another_joint_or_format() {
        let mut d = Coordinated::new(1, 0.000_25);
        let mut w = block(2.0);
        d.load_chunk(0, &w);
        assert_eq!(d.finish_load(w.len()), Err(CoordinatedError::WrongJoint));
        w = block(1.0);
        w[0] = 9.0;
        d.load_chunk(0, &w);
        assert_eq!(d.finish_load(w.len()), Err(CoordinatedError::Version));
        d.load_chunk(0, &block(1.0));
        assert!(d.finish_load(64).is_ok());
    }

    #[test]
    fn waits_for_holding_and_calibration_then_damps_against_the_measured_offset() {
        let mut d = Coordinated::new(1, 0.000_25);
        d.load_chunk(0, &block(1.0));
        d.finish_load(64).unwrap();
        let mut near = NearStates::default();
        near.on_announce(1, NodeId(1));
        let mut cycle = 0;
        let send = |near: &mut NearStates, angle: f32, cycle: u32| {
            near.on_state(NodeId(1), &NodeState { motor_angle: angle, ..NodeState::default() }, cycle);
        };
        // Moving: nothing; holding but settling or calibrating: nothing.
        send(&mut near, 0.5, cycle);
        assert_eq!(d.torque(&near, cycle, 0.1, 0.0, 10.0), None);
        let mut acted = None;
        for _ in 0..1100 {
            cycle += 1;
            send(&mut near, 0.5, cycle);
            acted = d.torque(&near, cycle, 0.0, -1.0, 10.0);
        }
        assert!(acted.is_some(), "acts after settling and calibration");
        // A step of the angle from its rest: the command opposes it (u = −K·ẑ, K = 2).
        for _ in 0..50 {
            cycle += 1;
            send(&mut near, 0.6, cycle);
            d.torque(&near, cycle, 0.0, -1.0, 10.0);
        }
        assert!((d.command() + 0.2).abs() < 1e-3, "command {}", d.command());
        // Stale data: network damping instead.
        assert_eq!(d.torque(&near, cycle + 10, 0.0, -1.0, 10.0), None);
    }
}
