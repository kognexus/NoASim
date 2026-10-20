# ADR-0003: Joint control on a quasi-direct drive, and what limits it

*Status: accepted, 21 Oct 2026 (NX-07).*

## Context

Every comparison in the plan starts from "network off": each joint controls itself with
its own encoders. That baseline has to be as good as a competent integrator would make it,
or the network's gains mean nothing. Building it on the KX-6 plant (9:1 QDD, 1-cycle
actuation delay, real sensor noise) showed four limits that shape the whole design.

## Findings

1. **Derivative gain on the motor encoder is capped by the rotor inertia.** The reflected
   rotor inertia of a 9:1 QDD is tiny (0.005–0.03 kg·m²) next to the arm it drives (up to
   6.6 kg·m²). A motor-side velocity loop crosses over at `k_v / I_rotor` and runs into
   the rotor–gear resonance (170–300 Hz) and the observer's lag. A motor-side cascade
   could not damp the arm.
2. **Position and damping therefore close on the output encoder** (as on any dual-encoder
   joint). A small rotor-against-gear term damps the rotor.
3. **Joint damping is capped by a coupled mode between adjacent joints.** Each joint
   alone is stable at any damping (checked on a two-mass model). Two adjacent
   parallel-axis joints, both strongly damped, clamp the flexible link between them, and
   that clamped mode (≈ 135 Hz) is driven unstable by the velocity observer's phase lag.
   Low-passing the damping signal moved the instability to ≈ 25–30 Hz instead of removing
   it. What works is a faster observer (400 Hz) on the 19-bit output encoder. That trades
   lag for noise: at the same damping, tool jitter at rest grows from 0.01 mm (150 Hz)
   to 0.08 mm (400 Hz) and 0.14 mm (600 Hz).
4. **Light joints need a stiffness floor.** Sizing stiffness from one natural frequency
   (`K = ω²·I_load`) gives the wrist almost none (41 N·m/rad on joint 6 with a 3 kg
   payload). Stiffness is now `max(ω²·I_load, 50 N·m/rad per rated N·m)`.

## Decision

- Baseline ("network off"): output-encoder PD with stiffness from a 4 Hz natural frequency
  on the stretched-arm load inertia (with the stiffness floor), damping ratio 0.7, 400 Hz
  observer, no integral until gravity feed-forward exists (NX-08), 1-cycle actuation
  delay. It settles a 200 N × 10 ms tool tap in about 0.7 s with 0.08 mm jitter at rest.
- **Fairness rule for H1:** every mode is tuned for its fastest settling under the same
  rest-jitter budget at the tool. Settling is measured to 5 % of the peak, or to 3× the
  pre-tap jitter if that is larger.
- Every damping term built on absolute rates goes through an energy tank (B5). Such
  terms are not collocated and can pump energy between neighbours.
- Gains are sized in the harness from the datasheet (`sim/src/harness/gains.ts`). The
  node only receives the numbers, as it would from its EEPROM.

## Consequences

- **On the stiff KX-6, links hardly bend under the joint servo** (0.02 mrad, against
  10–30 mrad of joint motion after a tap). The tap response is the joints' own servo
  modes, which joint damping already handles. Network damping of link bending has little
  to act on there. Early sweeps of joint-local damping were faster than the baseline at
  equal gains, but only by exceeding the jitter budget.
- **On a lightweight variant with flexible links** (first locked mode 4.5 Hz), joint
  control alone kept ringing at 10–14 mm, and a first network law (damping of the
  link-tip rate measured by the IMUs at both ends of the link) settled it in about
  0.6 s. This is the regime the SPRIND claim is about. NX-11 adds that arm (KX-6L) and
  measures H1 on both arms, reporting the stiff arm's result as it is.
- The light wrist gear (6 000 N·m/rad, very little damping) rings at the rotor–gear
  resonance at rest (gear twist swinging by a few mrad). It is invisible at the tool, but
  NX-11 should damp it.
