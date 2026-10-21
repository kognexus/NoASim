//! Report to source, correction at the source, and fault (B1b).
//!
//! **Attribution.** A disturbance an actuator causes (gear mesh, cogging, ripple, a worn
//! bearing) repeats at fixed *orders* of its own rotation: so many cycles per output turn.
//! Each node announces these orders (its fingerprint). A node tests every candidate by
//! demodulating a signal at the candidate's orders, locked to the candidate's own published
//! motor angle; a disturbance from joint *k* is coherent with joint *k*'s angle and with no
//! other joint's, so the candidate with the strongest coherent amplitude is the source.
//!
//! The signal differs by candidate. For itself a node uses its *motor-side residual*: the
//! torque its motor really produced (`I·α − gear torque + friction`) minus what it
//! commanded. Vibration of the arm appears on both sides of that balance and cancels, so the
//! residual holds only what arises inside this actuator. For a neighbour it uses the
//! vibration it feels (high-passed gear torque).
//!
//! Felt vibration alone names the wrong joint far too often: every KX-6 joint has the same
//! fingerprint, and the arm's lightly damped modes ring coherently at the order frequency
//! of any joint moving at a similar speed (NX-12 log). So the two kinds of evidence are
//! combined. A neighbour that feels a vibration locked to joint *k* reports it to *k*
//! ("felt" report, broadcast). The joint whose own residual shows that order *claims* the
//! disturbance; in the network modes it also needs a fresh felt report at that order: the
//! vibration must reach the arm, not just the joint's own balance. The felt report need
//! not name the claimant, since it is the residual that tells who the source is. A joint
//! alone claims on its own evidence.
//!
//! **Correction.** A joint that claims a disturbance (in `NetworkWithSource` mode) takes the
//! part of its residual locked to the order and feeds its opposite forward. This cancels
//! torque ripple and cogging; it cannot cancel a disturbance that is not locked to the angle
//! (a worn bearing), nor a ripple beyond the drive's compensation budget (a damaged gear).
//! The residual still shows the disturbance after it is cancelled (the compensation is part
//! of the command); whether it is fixed is for the neighbours to say.
//!
//! **Fault.** If neighbours still feel it after the deadline, the joint reports a fault.

use crate::config::NodeConfig;
use crate::estimate::Estimate;
use crate::filters::{HighPass, LowPass, LowPass2};
use crate::neighbours::{Neighbourhood, Side};
use nexus_proto::{Disturbance, Fault, FaultCode, NodeId};

/// Corner of the high-pass on the vibration signal, Hz: above commanded motion.
const SIGNAL_HIGHPASS_HZ: f32 = 3.0;
/// Corner of the low-pass that band-limits the motor-side residual, Hz. The raw second
/// difference of the encoder is noisy; every term of the balance passes the same filter,
/// so none lags another.
const RESIDUAL_LOWPASS_HZ: f32 = 150.0;
/// A candidate order only counts while its frequency (order × speed) lies in this band, Hz:
/// below it the demodulator would just pass slow motion through; above it the signal is
/// filtered away.
const ORDER_BAND_HZ: (f32, f32) = (4.0, 100.0);
/// In-band cycles an order needs before its amplitude is trusted (1 s). A real source stays
/// coherent as the speed changes; a passing coincidence with one of the arm's own
/// vibration modes does not. Out of band the demodulator is frozen and this count leaks
/// away, so a speed reversal pauses attribution instead of restarting it.
const SETTLE_CYCLES: u32 = 4000;
/// Share of its signal a candidate's angle-locked amplitude must explain (against the
/// signal's RMS, 1 for a pure tone) before it is reported.
const MIN_COHERENCE: f32 = 0.5;
/// The best candidate must beat the runner-up by this factor.
const MIN_MARGIN: f32 = 1.5;
/// Corner of the attribution demodulators' second-order low-pass, Hz. Every KX-6 joint has
/// the same fingerprint, so only the angle tells candidates apart: a wrong candidate beats
/// against the true one at the difference of their order frequencies (a couple of hertz),
/// which this must reject.
const DEMOD_LOWPASS_HZ: f32 = 0.5;
/// The correction may use at most this share of the joint's rated torque, as drives cap
/// anti-cogging feed-forward. A ripple beyond it (a damaged gear) cannot be cancelled.
const MAX_COMPENSATION_SHARE: f32 = 0.2;
/// Corner of the canceller's demodulator, Hz: faster, since it only has to follow the
/// joint's own ripple.
const CANCEL_LOWPASS_HZ: f32 = 2.0;
/// Felt reports and claims are each made at most this often, cycles; claims half a period
/// after felt reports.
const REPORT_EVERY: u32 = 400;
/// The source must have corrected the disturbance within this many cycles of the first
/// report naming it (2 s at 4 kHz): long enough for the reporters' demodulators to see
/// the vibration gone.
const DEADLINE_CYCLES: u32 = 8000;
/// A claim stays current for this many cycles.
const REPORT_FRESH_CYCLES: u32 = 800;
/// A felt report corroborates claims for this many cycles (2 s): the vibration is felt as
/// soon as it starts, the joint's own evidence takes longer to settle.
const FELT_MEMORY_CYCLES: u32 = 8000;
/// Oldest neighbour state a candidate's angle may come from, cycles.
const MAX_ANGLE_AGE: u32 = 8;

/// Locked-in amplitude of a signal at a phase.
#[derive(Clone, Copy, Debug)]
struct Demodulator {
    i: LowPass2,
    q: LowPass2,
    /// In-band cycles, leaking away while out of band.
    in_band: u32,
}

impl Demodulator {
    fn new(cutoff_hz: f32, dt: f32) -> Self {
        Self { i: LowPass2::new(cutoff_hz, dt), q: LowPass2::new(cutoff_hz, dt), in_band: 0 }
    }

    fn settled(&self) -> bool {
        self.in_band >= SETTLE_CYCLES
    }

    fn update(&mut self, signal: f32, phase: f32) -> (f32, f32) {
        let (s, c) = (libm::sinf(phase), libm::cosf(phase));
        (self.i.update(signal * c), self.q.update(signal * s))
    }

    fn amplitude(&self) -> f32 {
        2.0 * libm::sqrtf(self.i.value() * self.i.value() + self.q.value() * self.q.value())
    }
}

/// Which joints a node may name.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Candidate {
    Own,
    Neighbour(Side),
}

const CANDIDATES: [Candidate; 3] =
    [Candidate::Own, Candidate::Neighbour(Side::Inner), Candidate::Neighbour(Side::Outer)];

/// Past values the motor-side balance needs.
#[derive(Clone, Copy, Debug)]
struct History {
    /// Motor angle at k−1 and k−2, rad.
    angle: [f32; 2],
    /// Gear torque at k−1, N·m.
    gear_torque: f32,
    /// Commands sent at k−1, k−2 and k−3, N·m.
    command: [f32; 3],
}

impl History {
    /// History of a joint that has been at rest in the state `e` with `command` applied.
    fn at(e: &Estimate, command: f32) -> Self {
        Self { angle: [e.motor_angle; 2], gear_torque: e.gear_torque, command: [command; 3] }
    }
}

/// The most coherent candidate seen this cycle and the coherence of the runner-up.
#[derive(Clone, Copy, Debug, Default)]
struct Ranking {
    best: Option<(f32, Disturbance)>,
    runner_up: f32,
}

impl Ranking {
    fn offer(&mut self, coherence: f32, d: Disturbance) {
        match self.best {
            Some((b, _)) if coherence <= b => self.runner_up = self.runner_up.max(coherence),
            Some((b, _)) => {
                self.runner_up = self.runner_up.max(b);
                self.best = Some((coherence, d));
            }
            None => self.best = Some((coherence, d)),
        }
    }

    /// The best candidate if it is coherent enough and clearly ahead of the runner-up.
    fn confident(&self) -> Option<Disturbance> {
        self.best.filter(|(c, _)| *c > MIN_COHERENCE && *c > MIN_MARGIN * self.runner_up).map(|(_, d)| d)
    }
}

/// What the source stage asks of the node this cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SourceOutput {
    /// Torque to add to cancel this joint's own disturbance, N·m.
    pub compensation: f32,
    /// A report to broadcast, if due: a felt report naming a neighbour, or a claim naming
    /// this joint.
    pub report: Option<Disturbance>,
    /// A fault to send to the host, once.
    pub fault: Option<Fault>,
}

/// The report-to-source stage of one node.
#[derive(Clone, Copy, Debug)]
pub struct SourceTracker {
    fingerprint: [f32; 4],
    gear_stiffness: f32,
    inertia: f32,
    threshold: f32,
    max_compensation: f32,
    dt: f32,
    vibration: HighPass,
    vibration_power: LowPass,
    residual_band: LowPass2,
    residual: HighPass,
    residual_power: LowPass,
    demods: [[Demodulator; 4]; 3],
    /// The motor-side balance one cycle back: motor angles at k−1 and k−2, gear torque at
    /// k−1, and the commands that acted over the two cycles around it.
    history: Option<History>,
    /// Cycle of the latest felt report at each of this joint's orders.
    felt: [Option<u32>; 4],
    // Correction at the source.
    correcting: Option<usize>,
    ripple: Demodulator,
    // Deadline.
    /// Index of the order last claimed.
    claimed: Option<usize>,
    first_claimed: Option<u32>,
    last_claimed: u32,
    faulted: bool,
}

impl SourceTracker {
    /// Stage for the joint described by `config`.
    #[must_use]
    pub fn new(config: &NodeConfig) -> Self {
        let dt = config.cycle_seconds();
        Self {
            fingerprint: config.fingerprint,
            gear_stiffness: config.gear_stiffness,
            inertia: config.reflected_inertia,
            threshold: config.report_threshold,
            max_compensation: MAX_COMPENSATION_SHARE * config.rated_torque,
            dt,
            vibration: HighPass::new(SIGNAL_HIGHPASS_HZ, dt),
            vibration_power: LowPass::new(DEMOD_LOWPASS_HZ, dt),
            residual_band: LowPass2::new(RESIDUAL_LOWPASS_HZ, dt),
            residual: HighPass::new(SIGNAL_HIGHPASS_HZ, dt),
            residual_power: LowPass::new(DEMOD_LOWPASS_HZ, dt),
            demods: [[Demodulator::new(DEMOD_LOWPASS_HZ, dt); 4]; 3],
            history: None,
            felt: [None; 4],
            correcting: None,
            ripple: Demodulator::new(CANCEL_LOWPASS_HZ, dt),
            claimed: None,
            first_claimed: None,
            last_claimed: 0,
            faulted: false,
        }
    }

    /// A report from `reporter` arrived. A felt report at one of this joint's orders is
    /// kept as corroboration.
    pub fn on_report(&mut self, reporter: NodeId, report: &Disturbance, now: u32) {
        // A claim is the reporter naming itself; anything else is felt.
        if report.source == reporter {
            return;
        }
        if let Some(k) = self.order_index(report.order) {
            self.felt[k] = Some(now);
        }
    }

    fn order_index(&self, order: f32) -> Option<usize> {
        self.fingerprint.iter().position(|o| libm::fabsf(o - order) < 1e-3)
    }

    /// Whether a neighbour felt `order` recently.
    fn felt_recently(&self, order: f32, now: u32) -> bool {
        self.order_index(order)
            .and_then(|k| self.felt[k])
            .is_some_and(|at| now.wrapping_sub(at) < FELT_MEMORY_CYCLES)
    }

    /// Runs one cycle. `command` is the torque the node commanded last cycle; `network`
    /// says whether neighbours are tested and claims need corroboration; `correct` whether
    /// a claimed disturbance is cancelled.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        own_id: NodeId,
        e: &Estimate,
        command: f32,
        friction: f32,
        neighbours: &Neighbourhood,
        now: u32,
        network: bool,
        correct: bool,
    ) -> SourceOutput {
        let mut out = SourceOutput::default();
        // Motor-side balance at the previous cycle, where the encoder's second difference is
        // centred. With one cycle of actuation delay, the command sent at k−1 acts over
        // [k, k+1], so the cycles around k−1 felt the commands sent at k−2 and k−3.
        let past = self.history.get_or_insert(History::at(e, command));
        let acceleration = (e.motor_angle - 2.0 * past.angle[0] + past.angle[1]) / (self.dt * self.dt);
        let applied = f32::midpoint(past.command[1], past.command[2]);
        let unexplained = self.inertia * acceleration - past.gear_torque + friction - applied;
        *past = History {
            angle: [e.motor_angle, past.angle[0]],
            gear_torque: e.gear_torque,
            command: [command, past.command[0], past.command[1]],
        };
        let unexplained = self.residual_band.update(unexplained);
        let residual = self.residual.update(unexplained);
        let residual_rms = libm::sqrtf(self.residual_power.update(residual * residual));
        let vibration = self.vibration.update(self.gear_stiffness * e.gear_twist);
        let vibration_rms = libm::sqrtf(self.vibration_power.update(vibration * vibration));

        // Attribution: coherent amplitude at every candidate's orders, locked to its angle.
        // Best (coherence, amplitude, source, order) for this joint and for a neighbour, each
        // with its runner-up; coherence is the amplitude against the RMS of its signal.
        let mut own = Ranking::default();
        let mut felt = Ranking::default();
        for (c, candidate) in CANDIDATES.iter().enumerate() {
            let (id, angle, speed, orders, signal, rms) = match candidate {
                Candidate::Own => {
                    (own_id, e.motor_angle, e.motor_velocity, self.fingerprint, residual, residual_rms)
                }
                Candidate::Neighbour(side) if network => {
                    let Some(n) = neighbours.get(*side) else { continue };
                    let Some(state) = n.fresh_state(now, MAX_ANGLE_AGE) else { continue };
                    let fingerprint = n.announce.fingerprint;
                    (n.id, state.motor_angle, state.motor_velocity, fingerprint, vibration, vibration_rms)
                }
                Candidate::Neighbour(_) => continue,
            };
            for (k, order) in orders.iter().enumerate() {
                let demod = &mut self.demods[c][k];
                let frequency = order * libm::fabsf(speed) / core::f32::consts::TAU;
                if *order <= 0.0 || frequency < ORDER_BAND_HZ.0 || frequency > ORDER_BAND_HZ.1 {
                    demod.in_band = demod.in_band.saturating_sub(1);
                    continue;
                }
                demod.in_band = (demod.in_band + 1).min(SETTLE_CYCLES);
                demod.update(signal, order * angle);
                let amplitude = demod.amplitude();
                if !demod.settled() || amplitude <= self.threshold {
                    continue;
                }
                let coherence = amplitude / (core::f32::consts::SQRT_2 * rms).max(f32::EPSILON);
                let ranking = if *candidate == Candidate::Own { &mut own } else { &mut felt };
                ranking.offer(coherence, Disturbance { source: id, order: *order, amplitude });
            }
        }
        let phase = now % REPORT_EVERY;
        if phase == 0 {
            out.report = felt.confident();
        } else if phase == REPORT_EVERY / 2 {
            let claim = own.confident().filter(|d| !network || self.felt_recently(d.order, now));
            if let Some(claim) = claim {
                self.claimed = self.order_index(claim.order);
                self.first_claimed.get_or_insert(now);
                self.last_claimed = now;
                if correct && self.correcting.is_none() {
                    self.correcting = self.order_index(claim.order);
                }
            }
            out.report = claim;
        }

        // Correction: cancel the angle-locked part of the torque the motor really produced.
        if let Some(k) = self.correcting {
            let order = self.fingerprint[k];
            // The residual is a cycle old and the band-limit filter delays it further, by
            // 2·atan(ω/ωc)/ω at the order frequency ω, and shrinks it by 1/(1 + (ω/ωc)²).
            // The compensation acts from the next cycle on, 1.5 cycles from now on average.
            let corner = core::f32::consts::TAU * RESIDUAL_LOWPASS_HZ;
            let omega = order * libm::fabsf(e.motor_velocity);
            let lag = if omega > 1e-3 { 2.0 * libm::atanf(omega / corner) / omega } else { 2.0 / corner };
            let measured_at = e.motor_angle - e.motor_velocity * (self.dt + lag);
            let (in_phase, quadrature) = self.ripple.update(unexplained, order * measured_at);
            let gain = 1.0 + (omega / corner) * (omega / corner);
            let phase = order * (e.motor_angle + 1.5 * self.dt * e.motor_velocity);
            let locked = in_phase * libm::cosf(phase) + quadrature * libm::sinf(phase);
            let cap = self.max_compensation;
            out.compensation = (-2.0 * gain * locked).clamp(-cap, cap);
        }

        // Deadline: still claiming, and neighbours still feeling it after the deadline, means
        // the joint cannot fix it. A joint alone cannot tell, so it never faults.
        if let Some(first) = self.first_claimed.filter(|_| network) {
            let order = self.claimed.unwrap_or(0);
            let felt_after_deadline =
                self.felt[order].and_then(|at| at.checked_sub(first)).is_some_and(|d| d > DEADLINE_CYCLES);
            if now.wrapping_sub(self.last_claimed) >= REPORT_FRESH_CYCLES {
                self.first_claimed = None;
            } else if felt_after_deadline && !self.faulted {
                self.faulted = true;
                let amplitude = own.best.map_or(0.0, |b| b.1.amplitude);
                out.fault = Some(Fault { code: FaultCode::Source, value: amplitude });
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::{Demodulator, SourceTracker};
    use crate::config::NodeConfig;
    use crate::estimate::Estimate;
    use crate::neighbours::Neighbourhood;
    use nexus_proto::NodeId;

    #[test]
    fn demodulator_locks_onto_a_signal_at_its_phase() {
        let mut d = Demodulator::new(2.0, 0.00025);
        for k in 0..40_000_u32 {
            #[allow(clippy::cast_precision_loss)]
            let phase = k as f32 * 0.01;
            d.update(0.8 * libm::sinf(phase + 0.3), phase);
        }
        assert!((d.amplitude() - 0.8).abs() < 0.02, "{}", d.amplitude());
    }

    #[test]
    fn names_itself_when_its_own_angle_explains_its_residual() {
        let config = NodeConfig::example();
        let mut tracker = SourceTracker::new(&config);
        let mut reported = None;
        for k in 0..24_000_u32 {
            #[allow(clippy::cast_precision_loss)]
            let angle = k as f32 * 0.00025 * 1.5; // 1.5 rad/s: order 36 at 8.6 Hz
            let torque = 2.0 * libm::sinf(config.fingerprint[1] * angle);
            let e = Estimate {
                motor_angle: angle,
                motor_velocity: 1.5,
                // The motor makes `torque` it was not asked for; with the rotor at constant
                // speed, the gear passes all of it on.
                gear_twist: -torque / config.gear_stiffness,
                gear_torque: -torque,
                ..Estimate::default()
            };
            let out = tracker.update(NodeId(3), &e, 0.0, 0.0, &Neighbourhood::default(), k, false, false);
            reported = out.report.or(reported);
        }
        let report = reported.expect("a report");
        assert_eq!(report.source, NodeId(3));
        assert!((report.order - config.fingerprint[1]).abs() < 1e-3);
    }
}
