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
}

/// Jumps to each goal's position as soon as it arrives. The NX-05 placeholder; NX-10
/// replaces it with jerk-limited, clock-synchronised interpolation.
#[derive(Clone, Copy, Debug, Default)]
pub struct StepInterpolator {
    target: f32,
}

impl Interpolator for StepInterpolator {
    fn hold(&mut self, position: f32) {
        self.target = position;
    }

    fn add_goal(&mut self, goal: &Goal, _now: u32) {
        self.target = goal.position;
    }

    fn reference(&mut self, _now: u32) -> Reference {
        Reference { position: self.target, velocity: 0.0, acceleration: 0.0 }
    }
}
