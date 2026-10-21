# Changelog

All notable changes, one entry per ticket. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Fixed

- Duplicate-time policy steps at the end of a task made the arm lurch (NX-09/NX-10);
  the figure of eight now ramps out. H2 re-measured in the NX-10 ticket.

### Added

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
