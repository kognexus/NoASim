//! Limits (B6, B7): the last stage before the motor. Every cycle the joint enforces the
//! host's torque, speed, position and temperature limits with its own data, and reports
//! each limit it applies, so the host is never surprised by its joints.
//!
//! Thermal headroom is published continuously; the joint warns before it derates and
//! never cuts torque silently.

use crate::estimate::Estimate;
use nexus_proto::{Limits, StatusFlags};

/// Which limit acted (the value of a `Limit` fault report).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum LimitKind {
    /// Output speed beyond the host's limit.
    Speed = 1,
    /// Position beyond the host's range.
    Position = 2,
    /// Torque derated for temperature.
    Thermal = 3,
}

/// What the limiter did this cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Limited {
    /// Torque that may be applied, N·m.
    pub torque: f32,
    /// Status bits to publish (`DERATING`, `THERMAL_WARNING`).
    pub status: u16,
    /// A limit that started acting this cycle, to report once.
    pub started: Option<LimitKind>,
}

/// Enforces limits on the torque the node is about to apply.
pub trait SafetySupervisor {
    /// Returns the torque that may be applied this cycle and what was limited.
    fn limit(&mut self, torque: f32, estimate: &Estimate, limits: &Limits) -> Limited;
}

/// Thermal headroom (share of the band from ambient to the limit) below which the joint
/// warns, and below which it derates.
const WARN_HEADROOM: f32 = 0.2;
const DERATE_HEADROOM: f32 = 0.1;
/// At zero headroom the torque limit is this share of the normal limit.
const DERATED_SHARE: f32 = 0.5;
/// Stiffness of the virtual wall at a position limit, per N·m of torque limit, 1/rad.
const WALL_STIFFNESS_PER_NM: f32 = 50.0;
/// Damping that holds the joint at its speed limit, per N·m of torque limit, s/rad. Kept
/// small: the reference is already rate-limited, and a stiff brake on a light wrist joint
/// would itself oscillate at the bus rate.
const SPEED_DAMPING_PER_NM: f32 = 0.05;

/// Thermal headroom: share of the band from `ambient` to the limit still left, 0…1.
#[must_use]
pub fn thermal_headroom(temperature: f32, limit: f32, ambient: f32) -> f32 {
    ((limit - temperature) / (limit - ambient).max(1.0)).clamp(0.0, 1.0)
}

/// A limit that keeps switching on and off is reported again only after this many cycles.
const REPORT_COOLDOWN_CYCLES: u32 = 1000;

/// Torque, speed, position and thermal limiter.
#[derive(Clone, Copy, Debug)]
pub struct LimitSupervisor {
    ambient: f32,
    active: [bool; 3],
    /// Cycles since each limit was last reported.
    since_report: [u32; 3],
}

impl LimitSupervisor {
    /// Limiter assuming `ambient` °C around the motor.
    #[must_use]
    pub const fn new(ambient: f32) -> Self {
        Self { ambient, active: [false; 3], since_report: [u32::MAX; 3] }
    }

    fn track(&mut self, kind: LimitKind, on: bool, started: &mut Option<LimitKind>) {
        let k = kind as usize - 1;
        self.since_report[k] = self.since_report[k].saturating_add(1);
        if on && !self.active[k] && started.is_none() && self.since_report[k] > REPORT_COOLDOWN_CYCLES {
            *started = Some(kind);
            self.since_report[k] = 0;
        }
        self.active[k] = on;
    }
}

impl Default for LimitSupervisor {
    fn default() -> Self {
        Self::new(25.0)
    }
}

impl SafetySupervisor for LimitSupervisor {
    fn limit(&mut self, torque: f32, e: &Estimate, limits: &Limits) -> Limited {
        let mut started = None;
        let mut status = 0;

        // Thermal: warn first, then derate linearly towards half the limit.
        let headroom = thermal_headroom(e.temperature, limits.max_temperature, self.ambient);
        if headroom < WARN_HEADROOM {
            status |= StatusFlags::THERMAL_WARNING;
        }
        let derating = headroom < DERATE_HEADROOM;
        let share =
            if derating { DERATED_SHARE + (1.0 - DERATED_SHARE) * headroom / DERATE_HEADROOM } else { 1.0 };
        if derating {
            status |= StatusFlags::DERATING;
        }
        self.track(LimitKind::Thermal, derating, &mut started);
        let max = limits.max_torque * share;
        let mut torque = torque;

        // Position: a virtual wall pushes back beyond the range.
        let wall = WALL_STIFFNESS_PER_NM * limits.max_torque;
        let beyond = if e.output_angle > limits.max_position {
            e.output_angle - limits.max_position
        } else if e.output_angle < limits.min_position {
            e.output_angle - limits.min_position
        } else {
            0.0
        };
        if beyond != 0.0 {
            torque = torque.min(-wall * beyond).max(-max).min(max);
            torque = if beyond > 0.0 { torque.min(-wall * beyond) } else { torque.max(-wall * beyond) };
            status |= StatusFlags::DERATING;
        }
        self.track(LimitKind::Position, beyond != 0.0, &mut started);

        // Speed: beyond the limit, torque may only slow the joint down.
        let v = e.output_velocity;
        let over = libm::fabsf(v) - limits.max_velocity;
        if over > 0.0 {
            let brake = -libm::copysignf(SPEED_DAMPING_PER_NM * limits.max_torque * over, v);
            torque = if v > 0.0 { torque.min(brake) } else { torque.max(brake) };
            status |= StatusFlags::DERATING;
        }
        self.track(LimitKind::Speed, over > 0.0, &mut started);

        Limited { torque: torque.clamp(-max, max), status, started }
    }
}

#[cfg(test)]
mod tests {
    use super::{thermal_headroom, LimitKind, LimitSupervisor, SafetySupervisor};
    use crate::estimate::Estimate;
    use nexus_proto::{Limits, StatusFlags};

    fn limits() -> Limits {
        Limits {
            max_torque: 20.0,
            max_velocity: 2.0,
            min_position: -1.0,
            max_position: 1.0,
            max_temperature: 125.0,
            network_torque_share: 0.25,
        }
    }

    fn at(position: f32, velocity: f32, temperature: f32) -> Estimate {
        Estimate { output_angle: position, output_velocity: velocity, temperature, ..Estimate::default() }
    }

    #[test]
    fn clamps_torque_and_reports_nothing_in_range() {
        let mut l = LimitSupervisor::default();
        let out = l.limit(50.0, &at(0.0, 0.0, 40.0), &limits());
        assert!((out.torque - 20.0).abs() < 1e-6);
        assert_eq!((out.status, out.started), (0, None));
    }

    #[test]
    fn warns_before_it_derates_and_reports_the_derating_once() {
        let mut l = LimitSupervisor::default();
        // 82 % of the band used: warning only.
        let warn = l.limit(20.0, &at(0.0, 0.0, 107.0), &limits());
        assert!(warn.status & StatusFlags::THERMAL_WARNING != 0 && warn.status & StatusFlags::DERATING == 0);
        assert!((warn.torque - 20.0).abs() < 1e-6);
        // 95 % used: derated to 75 % and reported once.
        let derate = l.limit(20.0, &at(0.0, 0.0, 120.0), &limits());
        assert!(derate.status & StatusFlags::DERATING != 0);
        assert!((derate.torque - 15.0).abs() < 1e-3, "{}", derate.torque);
        assert_eq!(derate.started, Some(LimitKind::Thermal));
        assert_eq!(l.limit(20.0, &at(0.0, 0.0, 120.0), &limits()).started, None);
        assert!((thermal_headroom(125.0, 125.0, 25.0)).abs() < 1e-6);
    }

    #[test]
    fn pushes_back_beyond_the_position_range_and_brakes_beyond_the_speed_limit() {
        let mut l = LimitSupervisor::default();
        let wall = l.limit(10.0, &at(1.05, 0.0, 40.0), &limits());
        assert!(wall.torque < 0.0 && wall.started == Some(LimitKind::Position));
        let mut l = LimitSupervisor::default();
        let fast = l.limit(10.0, &at(0.0, 2.5, 40.0), &limits());
        assert!(fast.torque < 0.0 && fast.started == Some(LimitKind::Speed));
    }
}
