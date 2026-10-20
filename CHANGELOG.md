# Changelog

All notable changes, one entry per ticket. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added

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
