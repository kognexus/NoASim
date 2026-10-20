# ADR-0004: Why network damping cannot destabilise the arm, and what that guarantee does not cover

*Status: accepted, 21 Oct 2026 (NX-11).*

## Context

Network damping (B1a) feeds a joint's torque with rates measured at *other* joints'
IMUs, delivered over a bus with delay, jitter and loss. Such feedback is not collocated:
the sensor is not where the torque acts. On its own, nothing guarantees that it removes
energy. The SPRIND claims rule says we may not claim stability without an argument. This
is the argument, together with the evidence the H6 sweep gives.

## The argument

1. **Each joint without the network term is passive at its port.** The joint controller
   is a spring–damper on the joint's own output encoder (collocated), plus gravity and
   friction feed-forward, which are functions of the pose and so store no energy. The arm,
   a mechanical system with springs and dampers, is passive. So the closed loop without
   network terms is stable. NX-07/08 bound the gains so that the sampled, delayed version
   keeps this property in practice: the joints are quiet at rest (ADR-0003).
2. **The network term is wrapped in an energy tank** (`EnergyTank`, `damping.rs`). With
   `τ_n` the network torque and `ω` the joint's own output velocity, the tank lets
   `τ_n` through only while `∫ τ_n·ω dt ≤ E_max`, with `E_max = 0.05 J` per joint. The
   tank measures power at the joint's own port with the joint's own sensors, so **no delay,
   jitter or lost frame can change this bound.** Late or missing neighbour data changes
   *what* the network term asks for, never *how much energy* it may add.
3. **A passive system plus elements of bounded net energy stays bounded.** The total
   energy the six tanks can ever release is 0.3 J. That is enough to move the tool a few
   millimetres, never enough to make the arm run away.
4. **Further guards:** the network torque is capped at the host's
   `network_torque_share` of the joint's limit; neighbour data older than 8 cycles
   (2 ms) drops the term entirely, and the joint falls back to local behaviour (B5).

## What the guarantee does not cover

- **Bounded is not quiet.** A tank bounds the *net* energy over time. It does not stop a
  term from taking energy in one half-cycle and returning it in the next, so a
  high-frequency oscillation of small amplitude stays possible. During NX-11 an untuned
  network term made the joints ring at 120 Hz: the clamped mode of a flexible link
  between two damped joints (ADR-0003), excited through the network loop's delay. The
  tank kept it bounded. The fix was bandwidth: a second-order 15 Hz low-pass on the
  network signal, which still passes the arm's 4–7 Hz modes with moderate phase lag. The
  fair tuner rejects any gain set whose housings are not quiet at rest.
- **The passivity of step 1 is an engineering property, not a proof.** It rests on the
  sampled controller being close enough to its continuous ideal, which the quietness and
  jitter checks test but do not prove.

## Consequences

- H6 is measured as: (a) the damping work at every joint never exceeds `E_max`, and
  (b) the arm settles and stays quiet with bus latency from 1 to 20 cycles and up to 20 %
  lost frames.
- Gains are tuned only inside the stable, quiet region. The tuner's budget makes that
  explicit.
