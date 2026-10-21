# Changelog

All notable changes, one entry per ticket. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Fixed

- Duplicate-time policy steps at the end of a task made the arm lurch (NX-09/NX-10);
  the figure of eight now ramps out. H2 re-measured in the NX-10 ticket.

### Added

- **NX-19** `npm run prove`: one module per group of hypotheses, each measured from fixed
  seeds and compared with its target in code, writing `docs/RESULTS.md` with SVG plots.
  Met: H3a–H3d, H4, H5, H6, H10b–H10d. Partly met: H2, H7, H9, H10a. Not met: H1 (2.0–2.5×
  vs off, about even with joint-local damping). The learned models are re-scored, not
  retrained (`nexus-train --evaluate`).
- **NX-20** Interactive demo (`npm run dev -w web`): two arms side by side, drawn from
  their MuJoCo models with three.js, the same disturbances at the same cycle. Damping
  mode and model per arm, switched live. Tasks; tap, shake and collide; a click on a joint
  injects fixable or unfixable torque ripple. Impact policy, reaction switch, ACK,
  E-STOP and reset. Bus latency, jitter and loss applied live. Arm body, payload, link
  stiffness, brakes, and committed units with learned models. A tool-distance chart and
  a bus view with an event log.
- **NX-15** The learned model in the node: no_std inference with analytic gradients, loaded
  as a versioned word block (checked against the trainer's probe values). It is used
  for friction and cogging feed-forward and for the source self-test's residual, never
  in the safety path. H10b: joints stray 0.11 mrad from a datasheet-actuator arm with it,
  against 0.17–0.22 for the datasheet and up to 2.4 for the online fit. H10d: broken
  models stay bounded; the damping term never injects energy. H10a on the sealed family:
  61 % of the gap closed, target 70 %, not met; better than RLS. The self-test now uses a
  lock-in reference beside each order (it never claimed on the lightweight arm) and the
  corrected command alignment. H7 100/100 in both modes; on hidden actuators the
  learned residual cuts false claims from 6 in 30 trials to 1.
- **NX-14** `learning/` (uv, PyTorch): a port-Hamiltonian model of each joint, trained on
  its own commissioning log. It has a cogging and a gear potential, friction and gear
  damping that can only dissipate, and a torque-constant gain. Exported as versioned
  JSON with probe values (`models/`). On held-out runs at another pose it beats the
  fitted textbook model in every open family (gap to the Oracle closed: backlash 100 %,
  stiffening gear 95 %, ripple 57 %, textbook 44 %, position-dependent friction 33 %).
  The power balance holds to 10⁻¹⁴ (H10c).
- **NX-13** Commissioning: on a `Commission` request a joint excites itself with a seeded
  multisine (0.2–6 Hz) inside its position and speed limits while the others hold.
  `experiments/src/commission.ts` writes per-joint train and held-out logs from what each
  joint publishes, the node's own parameters, and a separate ground truth for the Oracle
  and evaluation only.
- **NX-12** Report to source. Each joint announces its fingerprint (excitation orders)
  and tests itself on its motor-side residual (produced minus commanded torque), which arm
  vibration cannot reach. Neighbours broadcast the vibration they feel at each order. A
  joint claims a disturbance when its residual shows the order and, in the network modes,
  a neighbour feels it. In `NetworkWithSource` mode the claimant cancels it by
  lag-compensated, angle-locked feed-forward (capped at 20 % of rated torque); if it is
  still felt 2 s later, the claimant reports a source fault. H5 met; H7 in the ticket.
  Felt vibration alone named the wrong joint most of the time (identical fingerprints,
  coherent arm modes), so it corroborates instead of naming.
- **NX-18** Limits enforced in every joint: torque, position wall, speed (reference rate
  limit plus a light brake) and thermal derating with a warning first; every limit
  reported to the host the cycle it starts acting; thermal headroom published.
- **NX-16, NX-17** Impact detection from external torque (gear-twist torque minus the
  IMU-measured dynamic chain) with commissioned thresholds, host heartbeat and comm-loss
  hold, and the impact policies: retreat and hold, E-stop with brake and safe torque off,
  float, plus a configuration check against brakes. All joints react within one cycle of
  detection; retreat returns to the pre-impact pose within 0.1 mm and lets go of the
  obstacle; no false alarms in normal tasks.
- **NX-11** Network damping on IMU-measured link-tip rates with a relayed second hop,
  posture-aware gains, 15 Hz bandwidth limit, energy tank and stale-data fallback; the
  lightweight KX-6L arm; a fair per-mode tuner and an H6 robustness sweep; ADR-0004
  (stability argument). Tap settling, network vs off / local: 2.75× / 1.28× on KX-6L,
  2.87× / 1.12× on KX-6 (H1 targets not met, see ticket). Discovery retries and periodic
  settings make the bus robust to frame loss.
- **NX-10** Joint-level interpolation: quintic segments to cycle-stamped goals, shared
  bus clock, smooth coast on missing goals, run-time switch from central setpoint
  following. Against central interpolation: jerk 3.0× / 5.0× / 1.4× lower and path error
  2–4× lower on figure of eight / pick and place / fast reach.
- **NX-09** Host side of motion: VLA policy emulator (rate, latency, jitter, drops),
  scripted tasks, damped-least-squares IK on the nominal kinematics, the central
  interpolation baseline and joint-level goals, a host work counter, and a motion
  scenario measuring tool jerk and path error.
- **NX-08** Gravity compensation computed joint by joint: IMU "up" estimate, tip-inward
  load chain over a new `Chain` frame, payload estimate from gear-twist torque, a
  level-base comparison model, and base tilt and motion injectors. Sideways tilt of 15°:
  tool held within 0.4 mm against 4.9 mm for the level-base model. Baseline joint damping
  moved to ζ 0.3 after the IMUs revealed rotor–gear ringing (ADR-0003 update).
- **NX-07** Node control stack: output-encoder joint controller with rotor damping,
  richer estimator (twist torque, current torque, acceleration), nominal and RLS friction
  models, local and network dampers behind energy tanks, run-time `Settings` frame. Harness
  gets actuation delay, physics substeps, gain sizing from nominal kinematics, the tap
  scenario and settling metrics. ADR-0003 records why joint control on a QDD is limited,
  and which acceptance criterion is still open.
- **NX-06** EtherCAT-like bus (latency, jitter, loss, cable cut, byte budget), host
  discovery with watchdog, node neighbour table, and `ArmSystem`, the composition root
  that runs plant, actuators, sensors, nodes, bus and host one cycle at a time.
- **NX-05** Bus protocol (`nexus-proto`: State, Goal, Limits, Announce, Assign frames,
  golden-byte layout shared with a TypeScript codec), node core (`nexus-node`: `Actuator`
  composed of estimator, interpolator, controller and safety traits), self-describing
  WASM I/O block with inbox and outbox, `nodeConfigFor` EEPROM builder.
- **NX-04** Hidden actuator families: composable physics effects (Stribeck and
  position-dependent friction, cogging, current ripple, torque-constant drift, cubic
  stiffening, Dahl hysteresis, backlash), five open families and one sealed family for the
  out-of-family test (hash-pinned), `ActuatorPhysics` interface, architecture test that
  keeps node code away from ground truth.
- **NX-03** Sensor models of the Nexus unit (dual absolute encoders, Hall, 6-axis IMU,
  current, winding temperature), seeded random streams, motor electrical and thermal
  data in `ActuatorSpec`, ideal drive and two-node thermal model.
- **NX-02** `ArmSpec` → MJCF generator for any serial arm (QDD motor + gear-spring hinge per
  joint, lumped link bending, IMU on the actuator housing), the KX-6 default arm, `Plant`
  wrapper with named access to ground truth, impulse modal test. KX-6 first modes: 9.7 Hz
  (ζ 1.6 %) vertical, 12.5 Hz (ζ 2.7 %) lateral. Benchmark now runs on KX-6 (6.0× real time
  for two arms with nodes).
- **NX-01** Spike: MuJoCo WASM steps the arm in Node and in the browser at about 5× real
  time for two arms with 12 node instances. `nexus-wasm` runs one node per WebAssembly
  instance over a C ABI; `WasmNode` loader with isolation tests; `npm run bench:step`;
  spike page in `web/`; ADR-0002. Toolchain moved to Rust 1.99.
- **NX-00** Repository scaffold: Cargo and npm workspaces, `nexus-proto` and `nexus-node`
  crate skeletons, `sim` harness skeleton, formatting, linting and test tooling behind
  `npm run check`, ticket files and generated board, ADR-0001.
