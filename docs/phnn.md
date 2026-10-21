# A learned model per joint (PHNN)

The claim: a Nexus unit does not need to know the actuator it sits on. It learns a model
of it from what it measures, and the model is physically sane by construction.

## What each joint learns

The state is the motor angle θ, the rotor momentum p and the gear twist φ (output minus
motor angle). The model is port-Hamiltonian:

```text
H(θ, p, φ) = p²/2I + V_cog(θ) + V_gear(φ)

θ̇ = ∂H/∂p
ṗ = −∂H/∂θ + ∂H/∂φ − R(ω)·ω + c·φ̇ + g(T, θ)·u
φ̇ = ω_out − ∂H/∂p
```

| Part | Learned as | Why this form |
|---|---|---|
| `V_cog` | Linear layer over Fourier features at the joint's own announced orders, from the gear mesh up | Cogging is periodic in the motor angle at orders the unit knows from its own geometry |
| `V_gear` | Datasheet spring ½kφ² plus a two-layer tanh MLP | Stiffening and play are corrections to a known spring |
| `R(ω)` | Two MLPs through softplus (viscous-like, Coulomb-like) | Friction can only take energy: R ≥ 0 |
| `c` | Softplus of one parameter | Gear damping can only take energy |
| `g(T, θ)` | exp(0.2·tanh(·)) of an MLP of temperature plus electrical-angle Fourier terms | Torque constant drift and ripple; always positive, within ±20 % |
| `I` | Datasheet inertia × [0.5, 2] | A geometric quantity the unit knows roughly |

Whatever the weights, the model's power balance is `Ḣ = ω·g·u + port − R·ω² − c·φ̇²`. It
cannot create energy; that is checked numerically on every held-out run (H10c).

## What it learns from

Commissioning (NX-13): on a host request a joint follows a seeded multisine (0.2–6 Hz)
inside its own position and speed limits while the others hold, and publishes its state
every cycle. Training (`learning/`, NX-14) fits the model to the motor-side balance
`I·α = ṗ` of that log after a 200 Hz smoothing; the encoder's second difference is too
noisy to compare sample by sample. The run's last fifth picks the epoch kept. The held-out
run is another excitation, 0.25 rad away.

The model never sees the actuator family or any ground truth; the dataset's `truth/` folder
is read only to score the Oracle baseline.

## What the joint uses it for

- **Feed-forward:** the command that cancels friction and cogging, `(R·ω − cogging)/g`,
  evaluated at the measured motor angle.
- **The self-test of report to source:** what the model leaves unexplained of the motor
  balance. On actuators nobody described, the learned residual cut false claims from 6
  in 30 trials to 1 (NX-15).
- **Not** for the damping law or its energy tank, which use measured power only. A wrong
  model can degrade performance but cannot inject energy through the damping term (H10d).

## How well it works

See [RESULTS.md](RESULTS.md) (H10a–H10d). In short:
- On the five open actuator families the model beats a least-squares fit of the textbook
  structure everywhere. It closes most of the gap to the truth for gears with backlash or
  stiffening, and less for hidden-state effects (hysteresis, thermal drift) and for
  friction that varies with position.
- On the sealed family, opened once for the final run, it closes about 60 % of the gap
  against a 70 % target, and two of six joints end worse than the datasheet.

## What would make it better

- Commission over a wider range of angle, so once-per-turn effects become learnable.
- Give the model an internal friction state (LuGre-like), which a port-Hamiltonian model
  can carry as an extra dissipative state.
- Learn the payload's centre of mass and the accelerometer offsets at commissioning (H9).

PHNNs are an existing technique (Greydanus et al. 2019; Desai et al. 2021). What is
particular here is one learned model per joint, trained from that joint's own
commissioning, run inside the joint, and used for attribution across the arm.
