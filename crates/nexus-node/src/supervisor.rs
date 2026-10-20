//! The safety supervisor: what a joint does when the arm is hit or the bus goes quiet
//! (B2, B8, PLAN §2.1).
//!
//! Every joint runs the same state machine on the same inputs (its own detection, the
//! impact frames of the others, the host's settings and acknowledgements), so the whole
//! arm reacts as one without a central coordinator. Reactions are referenced to the
//! detection's bus cycle, which every joint shares, so they stay in step.
//!
//! Impact policies, armed by the host:
//! - **P1 retreat and hold** (no brakes): go compliant briefly, move back to the last
//!   correct pose recorded before the impact, hold it under power, report, wait for ACK.
//! - **P2 E-stop** (brakes fitted): controlled stop (safe stop 1), brake, safe torque off.
//! - **P3 float**: gravity compensation only, report, wait for ACK.

use crate::reference::{Quintic, Reference};
use nexus_proto::{Fault, FaultCode, StatusFlags};

/// What the host armed (`Settings::impact_policy`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ImpactPolicy {
    /// P1.
    #[default]
    RetreatAndHold,
    /// P2.
    EStop,
    /// P3.
    Float,
    /// Comparison only (H3b): hold stiffly where the impact was detected, as a
    /// conventional stop would.
    StopInPlace,
}

impl ImpactPolicy {
    /// Maps the settings byte.
    #[must_use]
    pub const fn from_byte(byte: u8) -> Self {
        match byte {
            1 => Self::EStop,
            2 => Self::Float,
            3 => Self::StopInPlace,
            _ => Self::RetreatAndHold,
        }
    }
}

/// Timing and limits of the reactions.
#[derive(Clone, Copy, Debug)]
pub struct SupervisorTiming {
    /// Bus cycle length, s.
    pub cycle_seconds: f32,
    /// P1: compliant phase after detection, cycles.
    pub compliant_cycles: u32,
    /// P1: the last correct pose is taken this long before detection, cycles.
    pub snapshot_margin_cycles: u32,
    /// P1: duration of the move back, s (the same on every joint, so all arrive together).
    pub retreat_seconds: f32,
    /// P1: fastest allowed retreat speed, rad/s; a slower-than-this retreat is refused.
    pub retreat_speed_limit: f32,
    /// P2: controlled stop duration, s.
    pub stop_seconds: f32,
    /// P2: brake engagement time before torque off, cycles.
    pub brake_cycles: u32,
    /// Impact reports from other joints count if within this many cycles of each other.
    pub confirm_window_cycles: u32,
    /// Comm-loss watchdog, cycles of host silence.
    pub watchdog_cycles: u32,
    /// Comm loss: hold this long before stopping (only with a brake), cycles.
    pub comm_hold_cycles: u32,
}

impl SupervisorTiming {
    /// Defaults for a bus with cycle `cycle_seconds`.
    #[must_use]
    pub fn new(cycle_seconds: f32) -> Self {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let cycles = |s: f32| libm::roundf(s / cycle_seconds) as u32;
        Self {
            cycle_seconds,
            compliant_cycles: cycles(0.02),
            snapshot_margin_cycles: cycles(0.04),
            retreat_seconds: 0.5,
            retreat_speed_limit: 1.0,
            stop_seconds: 0.2,
            brake_cycles: cycles(0.05),
            confirm_window_cycles: cycles(0.01),
            watchdog_cycles: cycles(0.005),
            comm_hold_cycles: cycles(0.5),
        }
    }
}

/// Recent poses known to be correct, for P1.
#[derive(Clone, Copy, Debug)]
struct SnapshotRing {
    entries: [(u32, f32); SNAPSHOTS],
    next: usize,
    len: usize,
}

const SNAPSHOTS: usize = 128;
/// A pose is recorded every this many cycles (128 × 16 cycles ≈ 0.5 s of history at 4 kHz).
const SNAPSHOT_EVERY: u32 = 16;
/// A pose counts as correct when the joint tracks its reference this closely, rad.
const TRACKING_BAND: f32 = 0.02;

impl SnapshotRing {
    const fn new() -> Self {
        Self { entries: [(0, 0.0); SNAPSHOTS], next: 0, len: 0 }
    }

    fn record(&mut self, cycle: u32, position: f32) {
        self.entries[self.next] = (cycle, position);
        self.next = (self.next + 1) % SNAPSHOTS;
        self.len = (self.len + 1).min(SNAPSHOTS);
    }

    /// The newest pose recorded at or before `cycle`.
    fn at_or_before(&self, cycle: u32) -> Option<f32> {
        (0..self.len)
            .map(|k| self.entries[(self.next + SNAPSHOTS - 1 - k) % SNAPSHOTS])
            .find(|(c, _)| cycle.wrapping_sub(*c) < u32::MAX / 2)
            .map(|(_, p)| p)
    }
}

/// Where the supervisor is.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    Normal,
    /// P1: compliant until `until`, then retreat.
    Compliant {
        until: u32,
        target: Option<f32>,
    },
    /// P1: moving back along `path`.
    Retreat {
        path: Quintic,
    },
    /// P1 end state, and the refuge when a retreat is refused: hold `position`.
    Hold {
        position: f32,
    },
    /// P2: decelerating to `stop` along `path`.
    Stopping {
        path: Quintic,
        until: u32,
    },
    /// P2: brake engaging.
    Braking {
        until: u32,
        position: f32,
    },
    /// Safe torque off.
    TorqueOff,
    /// P3: gravity compensation only.
    Float,
    /// The host is silent: hold `position`.
    CommLost {
        position: f32,
        since: u32,
    },
}

/// What the supervisor wants from the rest of the node this cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Directive {
    /// Reference to use instead of the interpolator's (`None` = normal operation).
    pub reference: Option<Reference>,
    /// Engage the holding brake.
    pub brake: bool,
    /// Request safe torque off on the hardwired line.
    pub safe_torque_off: bool,
    /// Ignore goals from the host.
    pub ignore_goals: bool,
    /// Status bits to publish.
    pub status: u16,
}

/// The supervisor of one joint.
#[derive(Clone, Copy, Debug)]
pub struct Supervisor {
    timing: SupervisorTiming,
    has_brake: bool,
    policy: ImpactPolicy,
    confirm: u8,
    ring: SnapshotRing,
    phase: Phase,
    reports: [(u8, u32); 8],
    report_count: usize,
    pending_fault: Option<Fault>,
}

impl Supervisor {
    /// Supervisor for a joint with or without a holding brake. Impact reaction starts
    /// disarmed until the host's settings arm it.
    #[must_use]
    pub const fn new(timing: SupervisorTiming, has_brake: bool) -> Self {
        Self {
            timing,
            has_brake,
            policy: ImpactPolicy::RetreatAndHold,
            confirm: 0,
            ring: SnapshotRing::new(),
            phase: Phase::Normal,
            reports: [(0, 0); 8],
            report_count: 0,
            pending_fault: None,
        }
    }

    /// The armed policy.
    #[must_use]
    pub const fn policy(&self) -> ImpactPolicy {
        self.policy
    }

    /// True while the joint is reacting to an impact, holding, floating or stopped.
    #[must_use]
    pub fn reacting(&self) -> bool {
        self.phase != Phase::Normal
    }

    /// Applies the host's settings. P2 on a joint without a brake that carries load is
    /// refused (a brakeless backdrivable joint cannot be safely de-energised), and the
    /// joint keeps P1.
    pub fn configure(&mut self, policy: ImpactPolicy, confirm: u8, holding_torque: f32) {
        self.confirm = confirm;
        let carries_load = libm::fabsf(holding_torque) > 0.5;
        if policy == ImpactPolicy::EStop && !self.has_brake && carries_load {
            self.pending_fault = Some(Fault { code: FaultCode::ConfigRejected, value: 1.0 });
            self.policy = ImpactPolicy::RetreatAndHold;
        } else {
            self.policy = policy;
        }
    }

    /// A fault to report to the host, once.
    pub fn take_fault(&mut self) -> Option<Fault> {
        self.pending_fault.take()
    }

    /// Records an impact report from node `source` detected at `cycle` (including this
    /// node's own). Returns true when enough joints agree and the reaction starts.
    pub fn report_impact(
        &mut self,
        source: u8,
        cycle: u32,
        score: f32,
        position: f32,
        velocity: f32,
    ) -> bool {
        if self.confirm == 0 || self.reacting() {
            return false;
        }
        let window = self.timing.confirm_window_cycles;
        let recent = |c: u32| cycle.wrapping_sub(c) <= window || c.wrapping_sub(cycle) <= window;
        let mut kept = 0;
        for k in 0..self.report_count {
            if recent(self.reports[k].1) && self.reports[k].0 != source {
                self.reports[kept] = self.reports[k];
                kept += 1;
            }
        }
        if kept < self.reports.len() {
            self.reports[kept] = (source, cycle);
            kept += 1;
        }
        self.report_count = kept;
        if kept < usize::from(self.confirm) {
            return false;
        }
        // Everyone references the earliest report of the confirming set.
        let detected =
            self.reports[..kept].iter().map(|r| r.1).min_by_key(|c| c.wrapping_sub(cycle)).unwrap_or(cycle);
        self.start_reaction(detected, score, position, velocity);
        true
    }

    fn start_reaction(&mut self, detected: u32, score: f32, position: f32, velocity: f32) {
        self.pending_fault = Some(Fault { code: FaultCode::Impact, value: score });
        let now_ref = Reference { position, velocity, acceleration: 0.0 };
        self.phase = match self.policy {
            ImpactPolicy::RetreatAndHold => {
                let snapshot =
                    self.ring.at_or_before(detected.wrapping_sub(self.timing.snapshot_margin_cycles));
                Phase::Compliant {
                    until: detected.wrapping_add(self.timing.compliant_cycles),
                    target: snapshot,
                }
            }
            ImpactPolicy::EStop => {
                let stop = Reference {
                    position: position + 0.5 * velocity * self.timing.stop_seconds,
                    velocity: 0.0,
                    acceleration: 0.0,
                };
                let path = Quintic::new(detected, now_ref, stop, self.timing.stop_seconds);
                Phase::Stopping { path, until: path.end_cycle(self.timing.cycle_seconds) }
            }
            ImpactPolicy::Float => Phase::Float,
            ImpactPolicy::StopInPlace => Phase::Hold { position },
        };
    }

    /// The host acknowledged: release a holding or floating joint.
    pub fn acknowledge(&mut self) {
        if matches!(self.phase, Phase::Hold { .. } | Phase::Float | Phase::CommLost { .. }) {
            self.phase = Phase::Normal;
            self.report_count = 0;
        }
    }

    /// Runs one cycle with the joint at `position` (output angle), `velocity`, tracking
    /// `reference`, and the host silent for `host_silent` (true when the watchdog expired).
    pub fn update(&mut self, now: u32, position: f32, reference: &Reference, host_silent: bool) -> Directive {
        let still = |p: f32| Reference { position: p, velocity: 0.0, acceleration: 0.0 };
        if self.phase == Phase::Normal {
            if libm::fabsf(reference.position - position) < TRACKING_BAND
                && now.is_multiple_of(SNAPSHOT_EVERY)
            {
                self.ring.record(now, position);
            }
            if host_silent {
                self.pending_fault = Some(Fault { code: FaultCode::CommLoss, value: 0.0 });
                self.phase = Phase::CommLost { position, since: now };
            } else {
                return Directive::default();
            }
        }
        let reacting = StatusFlags::STOPPING | StatusFlags::FAULT;
        match self.phase {
            Phase::Normal => Directive::default(),
            Phase::Compliant { until, target } => {
                if now.wrapping_sub(until) < u32::MAX / 2 {
                    self.phase = self.plan_retreat(now, position, target);
                }
                // Compliant: the reference follows the joint, leaving only damping and gravity.
                Directive {
                    reference: Some(still(position)),
                    ignore_goals: true,
                    status: reacting,
                    ..Directive::default()
                }
            }
            Phase::Retreat { path } => {
                let r = path.at(now, self.timing.cycle_seconds);
                if now.wrapping_sub(path.end_cycle(self.timing.cycle_seconds)) < u32::MAX / 2 {
                    self.phase = Phase::Hold { position: r.position };
                }
                Directive { reference: Some(r), ignore_goals: true, status: reacting, ..Directive::default() }
            }
            Phase::Hold { position: hold } => Directive {
                reference: Some(still(hold)),
                ignore_goals: true,
                status: StatusFlags::HOLDING | StatusFlags::FAULT,
                ..Directive::default()
            },
            Phase::Stopping { path, until } => {
                let r = path.at(now, self.timing.cycle_seconds);
                if now.wrapping_sub(until) < u32::MAX / 2 {
                    self.phase = Phase::Braking {
                        until: now.wrapping_add(self.timing.brake_cycles),
                        position: r.position,
                    };
                }
                Directive { reference: Some(r), ignore_goals: true, status: reacting, ..Directive::default() }
            }
            Phase::Braking { until, position: hold } => {
                if now.wrapping_sub(until) < u32::MAX / 2 {
                    self.phase = Phase::TorqueOff;
                }
                Directive {
                    reference: Some(still(hold)),
                    brake: true,
                    ignore_goals: true,
                    status: reacting,
                    ..Directive::default()
                }
            }
            Phase::TorqueOff => Directive {
                reference: Some(still(position)),
                brake: self.has_brake,
                safe_torque_off: true,
                ignore_goals: true,
                status: StatusFlags::TORQUE_OFF | StatusFlags::FAULT,
            },
            Phase::Float => Directive {
                reference: Some(still(position)),
                ignore_goals: true,
                status: StatusFlags::HOLDING | StatusFlags::FAULT,
                ..Directive::default()
            },
            Phase::CommLost { position: hold, since } => {
                if !host_silent {
                    // The host is back; the fault stays until it acknowledges.
                } else if self.has_brake && now.wrapping_sub(since) > self.timing.comm_hold_cycles {
                    let path = Quintic::new(now, still(position), still(position), self.timing.stop_seconds);
                    self.phase = Phase::Stopping { path, until: path.end_cycle(self.timing.cycle_seconds) };
                }
                // Without a brake there is no safe unpowered state: keep holding (§2.1).
                Directive {
                    reference: Some(still(hold)),
                    ignore_goals: true,
                    status: StatusFlags::HOLDING | StatusFlags::FAULT,
                    ..Directive::default()
                }
            }
        }
    }

    fn plan_retreat(&self, now: u32, position: f32, target: Option<f32>) -> Phase {
        let Some(target) = target else {
            return Phase::Hold { position };
        };
        let seconds = self.timing.retreat_seconds;
        // A quintic's peak speed is 1.875× its mean speed.
        if 1.875 * libm::fabsf(target - position) / seconds > self.timing.retreat_speed_limit {
            return Phase::Hold { position };
        }
        let from = Reference { position, velocity: 0.0, acceleration: 0.0 };
        let to = Reference { position: target, velocity: 0.0, acceleration: 0.0 };
        Phase::Retreat { path: Quintic::new(now, from, to, seconds) }
    }
}

#[cfg(test)]
mod tests {
    use super::{ImpactPolicy, Supervisor, SupervisorTiming};
    use crate::reference::Reference;
    use nexus_proto::FaultCode;

    const DT: f32 = 0.00025;

    fn tracking(p: f32) -> Reference {
        Reference { position: p, velocity: 0.0, acceleration: 0.0 }
    }

    #[test]
    fn retreats_to_the_pose_from_before_the_impact_and_holds_until_acknowledged() {
        let mut s = Supervisor::new(SupervisorTiming::new(DT), false);
        s.configure(ImpactPolicy::RetreatAndHold, 1, 10.0);
        // Moving at 0.4 rad/s for a second, then hit at cycle 4000 at 0.4 rad.
        for now in 0..4000_u32 {
            #[allow(clippy::cast_precision_loss)]
            let p = 0.4 * now as f32 * DT;
            assert_eq!(s.update(now, p, &tracking(p), false).reference, None);
        }
        assert!(s.report_impact(3, 4000, 2.0, 0.4, 0.4));
        assert_eq!(s.take_fault().map(|f| f.code), Some(FaultCode::Impact));
        let mut position = 0.4;
        for now in 4000..8000 {
            let d = s.update(now, position, &tracking(position), false);
            assert!(d.ignore_goals && !d.safe_torque_off);
            position = d.reference.map_or(position, |r| r.position);
        }
        // Back to where it was 40 ms (and up to one snapshot period) before the hit.
        assert!((position - 0.384).abs() < 0.003, "held at {position}");
        s.acknowledge();
        assert_eq!(s.update(8000, position, &tracking(position), false).reference, None);
    }

    #[test]
    fn needs_the_configured_number_of_joints_to_agree() {
        let mut s = Supervisor::new(SupervisorTiming::new(DT), false);
        s.configure(ImpactPolicy::RetreatAndHold, 2, 10.0);
        assert!(!s.report_impact(2, 100, 1.5, 0.0, 0.0));
        assert!(!s.report_impact(2, 101, 1.5, 0.0, 0.0)); // same joint again
        assert!(s.report_impact(4, 103, 1.5, 0.0, 0.0));
    }

    #[test]
    fn refuses_e_stop_without_a_brake_on_a_loaded_joint() {
        let mut s = Supervisor::new(SupervisorTiming::new(DT), false);
        s.configure(ImpactPolicy::EStop, 1, 20.0);
        assert_eq!(s.policy(), ImpactPolicy::RetreatAndHold);
        assert_eq!(s.take_fault().map(|f| f.code), Some(FaultCode::ConfigRejected));
        let mut braked = Supervisor::new(SupervisorTiming::new(DT), true);
        braked.configure(ImpactPolicy::EStop, 1, 20.0);
        assert_eq!(braked.policy(), ImpactPolicy::EStop);
    }

    #[test]
    fn e_stop_stops_then_brakes_then_cuts_torque() {
        let mut s = Supervisor::new(SupervisorTiming::new(DT), true);
        s.configure(ImpactPolicy::EStop, 1, 20.0);
        s.report_impact(1, 0, 2.0, 0.0, 0.5);
        let (mut braked_at, mut off_at) = (None, None);
        for now in 0..2000 {
            let d = s.update(now, 0.0, &tracking(0.0), false);
            if d.brake && braked_at.is_none() {
                braked_at = Some(now);
            }
            if d.safe_torque_off && off_at.is_none() {
                off_at = Some(now);
            }
        }
        let (braked, off) = (braked_at.unwrap(), off_at.unwrap());
        assert!(braked >= 800 && off > braked, "brake at {braked}, STO at {off}");
    }

    #[test]
    fn comm_loss_holds_without_a_brake() {
        let mut s = Supervisor::new(SupervisorTiming::new(DT), false);
        s.update(0, 0.2, &tracking(0.2), false);
        let d = s.update(1, 0.2, &tracking(0.2), true);
        assert_eq!(s.take_fault().map(|f| f.code), Some(FaultCode::CommLoss));
        assert!(d.ignore_goals);
        for now in 2..10_000 {
            let d = s.update(now, 0.2, &tracking(0.2), true);
            assert!(!d.safe_torque_off, "a brakeless joint must keep holding");
        }
    }
}
