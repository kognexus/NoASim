//! Interpolation: from sparse goals to a reference for every bus cycle.

use nexus_proto::Goal;

/// Where the joint should be this cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Reference {
    /// Output position, rad.
    pub position: f32,
    /// Output velocity, rad/s.
    pub velocity: f32,
    /// Output acceleration, rad/s².
    pub acceleration: f32,
}

/// Turns goals from the host into a reference for every cycle.
pub trait Interpolator {
    /// Holds `position` until the next goal (start-up, stops).
    fn hold(&mut self, position: f32);
    /// Accepts a goal received at bus cycle `now`.
    fn add_goal(&mut self, goal: &Goal, now: u32);
    /// The reference for bus cycle `now`.
    fn reference(&mut self, now: u32) -> Reference;
    /// Switches interpolation mode, if the interpolator has more than one.
    fn set_mode(&mut self, _mode: InterpolationMode) {}
}

/// Follows each goal as a setpoint the moment it arrives: position and velocity jump,
/// then hold until the next one. This is how joints follow a host that interpolates
/// centrally and streams setpoints.
#[derive(Clone, Copy, Debug, Default)]
pub struct StepInterpolator {
    target: f32,
    velocity: f32,
}

impl Interpolator for StepInterpolator {
    fn hold(&mut self, position: f32) {
        self.target = position;
        self.velocity = 0.0;
    }

    fn add_goal(&mut self, goal: &Goal, _now: u32) {
        self.target = goal.position;
        self.velocity = goal.velocity;
    }

    fn reference(&mut self, _now: u32) -> Reference {
        Reference { position: self.target, velocity: self.velocity, acceleration: 0.0 }
    }
}

/// A quintic polynomial segment from one (position, velocity, acceleration) state to
/// another over a fixed duration: continuous up to acceleration, so jerk stays bounded.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Quintic {
    /// Start, in bus cycles.
    start: u32,
    /// Duration, s.
    duration: f32,
    /// Coefficients of `p(τ) = Σ c_k τ^k`, τ in seconds since `start`.
    c: [f32; 6],
}

impl Quintic {
    /// Segment from `from` at cycle `start` to `to` after `duration` seconds.
    #[must_use]
    pub fn new(start: u32, from: Reference, to: Reference, duration: f32) -> Self {
        let t = duration.max(1e-4);
        let (p0, v0, a0) = (from.position, from.velocity, from.acceleration);
        let (p1, v1, a1) = (to.position, to.velocity, to.acceleration);
        let (t2, t3, t4, t5) = (t * t, t * t * t, t * t * t * t, t * t * t * t * t);
        let c3 = (20.0 * (p1 - p0) - (8.0 * v1 + 12.0 * v0) * t - (3.0 * a0 - a1) * t2) / (2.0 * t3);
        let c4 = (30.0 * (p0 - p1) + (14.0 * v1 + 16.0 * v0) * t + (3.0 * a0 - 2.0 * a1) * t2) / (2.0 * t4);
        let c5 = (12.0 * (p1 - p0) - (6.0 * v1 + 6.0 * v0) * t - (a0 - a1) * t2) / (2.0 * t5);
        Self { start, duration: t, c: [p0, v0, a0 / 2.0, c3, c4, c5] }
    }

    /// Cycle at which the segment ends.
    #[must_use]
    pub fn end_cycle(&self, cycle_seconds: f32) -> u32 {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let cycles = libm::roundf(self.duration / cycle_seconds) as u32;
        self.start.wrapping_add(cycles)
    }

    /// State at bus cycle `now` (held at the end state after the segment ends).
    #[must_use]
    pub fn at(&self, now: u32, cycle_seconds: f32) -> Reference {
        #[allow(clippy::cast_precision_loss)]
        let elapsed = now.wrapping_sub(self.start) as f32 * cycle_seconds;
        let tau = elapsed.clamp(0.0, self.duration);
        let c = self.c;
        let position = c[0] + tau * (c[1] + tau * (c[2] + tau * (c[3] + tau * (c[4] + tau * c[5]))));
        let velocity = c[1] + tau * (2.0 * c[2] + tau * (3.0 * c[3] + tau * (4.0 * c[4] + tau * 5.0 * c[5])));
        let acceleration = 2.0 * c[2] + tau * (6.0 * c[3] + tau * (12.0 * c[4] + tau * 20.0 * c[5]));
        if elapsed > self.duration {
            // Past the end: continue at the end velocity (callers replan before that matters).
            let extra = elapsed - self.duration;
            return Reference { position: position + velocity * extra, velocity, acceleration: 0.0 };
        }
        Reference { position, velocity, acceleration }
    }
}

/// Shortest segment the interpolator will plan, s (goals due sooner are pushed back).
const MIN_SEGMENT_SECONDS: f32 = 0.01;
/// Time to coast to a stop when goals stop arriving, s.
const COAST_SECONDS: f32 = 0.15;

/// How often goals arrive: an average of the gaps between them, in bus cycles.
#[derive(Clone, Copy, Debug, Default)]
pub struct GoalRate {
    last: Option<u32>,
    gap: Option<f32>,
}

impl GoalRate {
    /// A gap longer than this is a pause between tasks, not the goal rate, cycles.
    const PAUSE_CYCLES: u32 = 2_000;
    /// Weight of the newest gap in the average.
    const WEIGHT: f32 = 0.2;

    /// A goal arrived at `cycle`.
    pub fn arrived(&mut self, cycle: u32) {
        if let Some(gap) =
            self.last.map(|last| cycle.wrapping_sub(last)).filter(|g| (1..Self::PAUSE_CYCLES).contains(g))
        {
            #[allow(clippy::cast_precision_loss)] // gaps are below 2000 cycles
            let gap = gap as f32;
            self.gap = Some(self.gap.map_or(gap, |avg| avg + Self::WEIGHT * (gap - avg)));
        }
        self.last = Some(cycle);
    }

    /// Goals per second, once two have arrived close enough together.
    #[must_use]
    pub fn hz(&self, cycle_seconds: f32) -> Option<f32> {
        self.gap.map(|gap| 1.0 / (gap * cycle_seconds))
    }
}

/// Joint-level interpolation (B3): every goal becomes a quintic from the current reference
/// to the goal's position and velocity at its due cycle. All joints share the bus clock,
/// so goals stamped with the same cycle are reached together. When the next goal does not
/// arrive in time, the joint coasts to a smooth stop.
#[derive(Clone, Copy, Debug)]
pub struct SplineInterpolator {
    cycle_seconds: f32,
    segment: Quintic,
    /// Whether the current segment ends at a goal (then a coast follows) or is the coast.
    coasting: bool,
    last: Reference,
}

impl SplineInterpolator {
    /// Interpolator for a bus with cycle length `cycle_seconds`.
    #[must_use]
    pub fn new(cycle_seconds: f32) -> Self {
        Self { cycle_seconds, segment: Quintic::default(), coasting: true, last: Reference::default() }
    }
}

impl Interpolator for SplineInterpolator {
    fn hold(&mut self, position: f32) {
        let still = Reference { position, velocity: 0.0, acceleration: 0.0 };
        self.segment = Quintic::new(0, still, still, MIN_SEGMENT_SECONDS);
        self.coasting = true;
        self.last = still;
    }

    fn add_goal(&mut self, goal: &Goal, now: u32) {
        #[allow(clippy::cast_precision_loss)]
        let until_due = goal.due_cycle.wrapping_sub(now).cast_signed() as f32 * self.cycle_seconds;
        let duration = until_due.max(MIN_SEGMENT_SECONDS);
        let to = Reference { position: goal.position, velocity: goal.velocity, acceleration: 0.0 };
        self.segment = Quintic::new(now, self.last, to, duration);
        self.coasting = false;
    }

    fn reference(&mut self, now: u32) -> Reference {
        if !self.coasting && now >= self.segment.end_cycle(self.cycle_seconds) {
            // The goal was reached and no newer one has come: coast to a stop.
            let state = self.segment.at(now, self.cycle_seconds);
            let stop = Reference {
                position: state.position + 0.5 * state.velocity * COAST_SECONDS,
                velocity: 0.0,
                acceleration: 0.0,
            };
            self.segment = Quintic::new(now, state, stop, COAST_SECONDS);
            self.coasting = true;
        }
        self.last = self.segment.at(now, self.cycle_seconds);
        self.last
    }
}

/// Which interpolation the host has selected (`Settings::interpolation`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum InterpolationMode {
    /// Follow streamed setpoints as they arrive (central interpolation on the host).
    #[default]
    Setpoints,
    /// Interpolate sparse goals in the joint.
    JointLevel,
}

impl InterpolationMode {
    /// Maps the settings byte.
    #[must_use]
    pub const fn from_byte(byte: u8) -> Self {
        if byte == 1 {
            Self::JointLevel
        } else {
            Self::Setpoints
        }
    }
}

/// Both interpolators, switched by the host's settings. Switching starts the new one from
/// the current reference, so the joint does not jump.
#[derive(Clone, Copy, Debug)]
pub struct SwitchableInterpolator {
    setpoints: StepInterpolator,
    spline: SplineInterpolator,
    mode: InterpolationMode,
    last: Reference,
}

impl SwitchableInterpolator {
    /// Starts in setpoint mode.
    #[must_use]
    pub fn new(cycle_seconds: f32) -> Self {
        Self {
            setpoints: StepInterpolator::default(),
            spline: SplineInterpolator::new(cycle_seconds),
            mode: InterpolationMode::Setpoints,
            last: Reference::default(),
        }
    }
}

impl Interpolator for SwitchableInterpolator {
    /// Selects `mode`, carrying the current reference over.
    fn set_mode(&mut self, mode: InterpolationMode) {
        if mode == self.mode {
            return;
        }
        self.mode = mode;
        self.setpoints.hold(self.last.position);
        self.spline.hold(self.last.position);
    }

    fn hold(&mut self, position: f32) {
        self.setpoints.hold(position);
        self.spline.hold(position);
        self.last = Reference { position, velocity: 0.0, acceleration: 0.0 };
    }

    fn add_goal(&mut self, goal: &Goal, now: u32) {
        match self.mode {
            InterpolationMode::Setpoints => self.setpoints.add_goal(goal, now),
            InterpolationMode::JointLevel => self.spline.add_goal(goal, now),
        }
    }

    fn reference(&mut self, now: u32) -> Reference {
        self.last = match self.mode {
            InterpolationMode::Setpoints => self.setpoints.reference(now),
            InterpolationMode::JointLevel => self.spline.reference(now),
        };
        self.last
    }
}

#[cfg(test)]
mod tests {
    use super::{GoalRate, Interpolator, Quintic, Reference, SplineInterpolator};
    use nexus_proto::Goal;

    const DT: f32 = 0.00025;

    #[test]
    fn goal_rate_averages_gaps_and_ignores_pauses() {
        let mut rate = GoalRate::default();
        assert!(rate.hz(0.000_25).is_none());
        for k in 0..20 {
            rate.arrived(k * 267); // 15 Hz at 4 kHz
        }
        rate.arrived(20 * 267 + 40_000); // a pause between tasks
        let hz = rate.hz(0.000_25).unwrap();
        assert!((hz - 4000.0 / 267.0).abs() < 0.01, "{hz}");
    }

    #[test]
    fn quintic_hits_both_end_states() {
        let from = Reference { position: 0.1, velocity: 0.5, acceleration: -2.0 };
        let to = Reference { position: 0.4, velocity: -0.2, acceleration: 0.0 };
        let q = Quintic::new(100, from, to, 0.05);
        let start = q.at(100, DT);
        let end = q.at(100 + 200, DT);
        assert!((start.position - 0.1).abs() < 1e-5 && (start.velocity - 0.5).abs() < 1e-3);
        assert!((start.acceleration + 2.0).abs() < 1e-2);
        assert!((end.position - 0.4).abs() < 1e-4 && (end.velocity + 0.2).abs() < 1e-2);
    }

    #[test]
    fn joints_reach_a_shared_due_cycle_together_whenever_the_goal_arrives() {
        let goal = Goal { due_cycle: 1000, position: 0.3, velocity: 0.0, sequence: 1 };
        let mut early = SplineInterpolator::new(DT);
        let mut late = SplineInterpolator::new(DT);
        early.hold(0.0);
        late.hold(0.0);
        for now in 0..1000 {
            if now == 700 {
                early.add_goal(&goal, now);
            }
            if now == 940 {
                late.add_goal(&goal, now); // the bus delivered this joint's goal 60 ms later
            }
            early.reference(now);
            late.reference(now);
        }
        assert!((early.reference(1000).position - 0.3).abs() < 1e-4);
        assert!((late.reference(1000).position - 0.3).abs() < 1e-4);
    }

    #[test]
    fn reaches_goals_on_their_due_cycle_and_coasts_when_they_stop() {
        let mut interpolator = SplineInterpolator::new(DT);
        interpolator.hold(0.0);
        let mut now = 0_u32;
        for k in 1..=5_u16 {
            // A goal every 268 cycles (15 Hz), arriving 120 cycles before it is due.
            let due = u32::from(k) * 268;
            while now + 120 < due {
                interpolator.reference(now);
                now += 1;
            }
            let position = f32::from(k) * 0.01;
            interpolator.add_goal(&Goal { due_cycle: due, position, velocity: 0.15, sequence: k }, now);
            while now < due {
                interpolator.reference(now);
                now += 1;
            }
            let at_due = interpolator.reference(now);
            assert!((at_due.position - position).abs() < 1e-4, "goal {k}: {at_due:?}");
        }
        // No more goals: the reference slows to a stop instead of running away.
        for _ in 0..4000 {
            interpolator.reference(now);
            now += 1;
        }
        let rest = interpolator.reference(now);
        assert!(rest.velocity.abs() < 1e-4 && (rest.position - 0.05).abs() < 0.02, "{rest:?}");
    }
}
