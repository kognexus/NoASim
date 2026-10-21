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

## The same argument in energy terms (NX-28)

*Added 21 Oct 2026, after reviewing a colleague's Lyapunov derivation of nearest-neighbour
damping.*

Take the arm holding a fixed reference, with ω the joints' output rates. Its energy, the
kinetic energy, gravity, the gear and link springs and the position loops' springs,

```text
H = ½ q̇ᵀ M(q) q̇ + V_gravity(q) + ½ Σ k_gear φ² + ½ Σ k_link b² + ½ Σ K_i (θ_i − θ_ref,i)²
```

changes as `Ḣ = −ωᵀ D ω − (gear and link damping) + ωᵀ τ_n`, where D is the diagonal of
the joints' own damping gains (collocated: each acts on its own output rate) and `τ_n` is
whatever else the joints add. Then:

- **Collocated damping** (joint-local) adds to D: `Ḣ` only falls.
- **A damper between neighbours**, `τ_i = −Σ_j c_ij (ω_i − ω_j)`, adds `−ωᵀ L ω`, with
  `L = Σ_edges c_ij (e_i − e_j)(e_i − e_j)ᵀ` a weighted graph Laplacian, which is positive
  semi-definite. It too can only take energy out, and cutting a link (an edge weight to
  zero) keeps L positive semi-definite. A spring between neighbours' angles is not a
  damper: it belongs in H, and adds no dissipation.
- **The network term is neither.** It uses the neighbour's gyro (the link tip's rate),
  so its power `−c_i ω_i s_i` has no fixed sign. That is why it passes the energy tank:
  `∫ τ_n,i ω_i dt ≤ E_max` at every joint, measured at the joint's own port, so
  `H(t) ≤ H(0) + N·E_max` whatever the signal.

What the clean form needs and the bus breaks: a neighbour coupling is passive only while
both joints act on the same velocities at the same time. With delay, jitter or loss each
sees the other late, the power is no longer a square, and the guarantee is gone; the
tank is what still holds. Fading a late neighbour out (NX-28) scales `τ_n` by a weight in
[0, 1] and leaves the tank bound as it is.

What "energy goes to zero" further needs (LaSalle): every mode must be reached by some
dissipation. NX-28 measured this. On the light arm, link bending at 55–69 Hz is reached
by no joint; it decays only through the structure's own damping. And with a moving
reference the arm is not autonomous; the argument is about deviations from the plan, an
engineering argument, not a proof.

NX-28 also measured whether a neighbour damper is worth adding: at most a few percent on
a tap, and the arm rings when it is strong (RESULTS.md, "Headroom"). Being passive is not
the same as damping well.

## A frame-aligned Laplacian law, tested (NX-32)

A colleague's derivation (`lyapunov_qdd.pdf`) argues that damping each joint on the
relative rate between its IMU and each neighbour's, the neighbour's gyro resolved in the
joint's frame through the encoders and high-passed, `τ_i = −Σ_j D_ij a_iᵀ HPF(ω_i − R_ij ω_j)`,
gives `V̇ = −q̇ᵀD q̇ − Δωᵀ L Δω ≤ 0` unconditionally. The step that does not hold is
`q̇ᵀ τ = −Δωᵀ L Δω`: a joint's torque does work only on its own coordinate, while the
relative rate across its link is `q̇_i + ḃ_i` (outer side) or the inner joint's rate and
bend (inner side), so the power has cross terms of no fixed sign. Resolving the gyros in
the joint's frame is right (`outer_rate` does it) but does not change this.

Run as written outside the joints, with true gyros and encoder angles, no tank, a tap at
home (`experiments/src/frame-aligned.ts`; gain as a multiple of the network-damping gain;
with the joints' IMU low-pass, which the note lacks, so that no gear mode is involved):

| Law | Both arms, 0.25–0.5× | 1× (our tuned gain) | 8× |
|---|---|---|---|
| As written (both neighbours) | puts 20–121 mJ in per tap; settles no better than the joints alone | unstable, with or without delay | unstable |
| Every term flipped (both neighbours) | puts 0–26 mJ in; settles worse than the joints alone | puts 40–76 mJ in | unstable |
| Flipped, outer neighbour only | takes 40–100 mJ out; settles faster | takes 100–150 mJ out, never puts any in | puts energy in (light arm; stiff arm with 4 cycles of delay); light arm unstable with 4 cycles |

As written, the outer term pushes each joint along its own speed (`+D(q̇_i + ḃ_i)`), and
it destabilises the arm from our tuned gain on, with no delay at all. With the sign of a
damper, the inner-neighbour term still pumps energy in. The outer term alone, which is
the shape of our network damping, takes energy out at every gain up to 4×, but not
unconditionally: at 8× and with delay it puts energy in. Without the IMU low-pass even the
outer term destabilises the light arm at 1× (rotor–gear modes). The energy tank stays.

## Consequences

- H6 is measured as: (a) the damping work at every joint never exceeds `E_max`, and
  (b) the arm settles and stays quiet with bus latency from 1 to 20 cycles and up to 20 %
  lost frames.
- Gains are tuned only inside the stable, quiet region. The tuner's budget makes that
  explicit.
