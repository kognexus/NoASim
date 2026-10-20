# ADR-0001: Architecture of the simulation

*Status: accepted, 21 Oct 2026.*

## Context

The repository has to show, with reproducible numbers, that a network of actuators can
damp vibration, trace faults, react to impacts and interpolate motion at the joint level.
A viewer has to be able to trust that the joints really do this work themselves, and not
a central program pretending to be them.

## Decision

1. **Joint code is firmware-shaped Rust.** `nexus-node` is `no_std` and allocation-free.
   Each joint runs as a separate WebAssembly instance with its own memory. A node's only
   inputs are its own sensor frame and the bus frames addressed to it.
2. **One harness for proof and demo.** The plant (MuJoCo WASM), sensor models, bus and host
   live in a TypeScript package that runs both in Node (for `npm run prove`) and in the
   browser (for the demo), so the two cannot drift apart.
3. **The bus is an explicit boundary.** All node-to-node and node-to-host traffic passes
   through a simulated EtherCAT-like bus with a 4 kHz cycle, a distributed clock, and
   configurable latency, jitter and frame loss. The bus counts bytes per cycle against a
   realistic budget.
4. **Interfaces over implementations.** Every node stage (estimator, interpolator,
   controller, neighbour coordinator, source corrector, safety supervisor, dynamics model)
   is a trait with swappable implementations, so each comparison in the plan is a
   different implementation behind the same interface.
5. **Learning stays offline.** PHNN training runs in Python on logs the harness writes.
   The node only runs frozen inference in Rust. Nothing learned sits in the safety path.

## Consequences

- The node code can later be compiled for the actuator MCU, but this repository does not
  prove MCU timing.
- Two languages (Rust, TypeScript) plus Python for training. Each is confined to one role.
- NX-01 must confirm that MuJoCo WASM runs headless in Node at the needed speed. If not,
  the fallback is Python `mujoco` driving the same Rust node (recorded in ADR-0002).
