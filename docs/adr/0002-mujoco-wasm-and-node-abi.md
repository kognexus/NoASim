# ADR-0002: MuJoCo WASM for plant and harness, plain C ABI for nodes

*Status: accepted, 21 Oct 2026 (NX-01).*

## Context

ADR-0001 rests on two assumptions that NX-01 had to check:

1. The official MuJoCo WebAssembly bindings run headless in Node (for `npm run prove`) and in
   the browser (for the demo), fast enough to step two arms at the 4 kHz bus rate.
2. Each joint can run in its own WebAssembly instance, cheaply enough to tick 12 of them per
   cycle.

## Measurements

`npm run bench:step` (Node 26) and the spike page in `web/` (Chromium, built-in browser),
Apple M1 Pro, 5 s of simulated time per row. The spike arm has the plant's
structure: per joint a QDD motor hinge and a compliant gear hinge, and two bending
hinges in each long link (16 DOF per arm). Each node tick is a fixed-gain hold.

| Arms | Node ticks | Node: steps/s | Node: × real time | Browser: steps/s | Browser: × real time |
|---|---|---|---|---|---|
| 1 | no | 39,500–46,300 | 9.9–11.6× | 47,400 | 11.9× |
| 1 | yes (6) | 49,600–52,000 | 12.4–13.0× | 51,200 | 12.8× |
| 2 | no | 23,300–24,100 | 5.8–6.0× | 23,900 | 6.0× |
| 2 | yes (12) | 19,500–19,900 | 4.9–5.0× | 20,100 | 5.0× |

The first row includes JIT warm-up, which is why it is slower than the second. Calling a
node costs about 1 µs per joint per cycle.

## Decision

1. **MuJoCo WASM (`@mujoco/mujoco` 3.16) is the plant in both environments.** The Python
   fallback from ADR-0001 is not needed. Two arms with all node ticks run at about 5× real
   time, which leaves room for the sensor models, the bus and rendering in the demo.
2. **Nodes export a plain C ABI, not `wasm-bindgen`.** `wasm-bindgen` glue binds one
   instance per JavaScript module, which would put all joints in one memory. `nexus-wasm`
   instead exports `nexus_init`, `nexus_tick` and a pointer to a `#[repr(C)]` I/O block.
   The harness instantiates the module once per joint, and a test checks that the
   memories are distinct. The module is 553 bytes. `wasm-pack` is not used.
3. **`unsafe` is confined to `nexus-wasm`**, which owns its instance's single global node.
   `nexus-proto` and `nexus-node` are `#![forbid(unsafe_code)]`.
4. **Rust 1.99** (current stable) instead of 1.88, so the WASM tooling builds.

## Consequences

- If later tickets make the per-cycle work much heavier, the demo can drop to one arm or
  slow down simulated time. The UI shows the real-time factor (PLAN §6).
- The I/O block layout is duplicated in Rust (`Io`) and TypeScript (`IO_LAYOUT`). The
  harness checks the block size at instantiation. NX-05 replaces the block with proper
  bus frames and a shared layout description.

## Update (NX-02)

The spike benchmark read joint addresses as `qposadr[0]`, but the bindings return
scalars, so its nodes saw zero sensors. The timing was still valid, because every node
tick and plant step ran. From NX-02 on, the benchmark runs the real KX-6 plant through
`Plant`, with each node holding its joint against gravity:

| Arms | Node ticks | Node: steps/s | Node: × real time |
|---|---|---|---|
| 1 | no | 46,000 | 11.5× |
| 1 | yes (6) | 48,800 | 12.2× |
| 2 | no | 25,000 | 6.2× |
| 2 | yes (12) | 24,100 | 6.0× |

The decision stands.
