# KognexusNexus

**A simulation of robot joints that work as a team.**

Each actuator of a lightweight 6-DOF arm carries its own processor, motion sensor and
safety path. The joints share what they sense with their neighbours every 250 µs. With
that shared data they damp vibration that no single joint could reach, trace a disturbance
back to the joint that caused it, stop the whole arm when any one of them feels an impact,
and turn the sparse goals of a slow AI policy into smooth motion. The robot's main computer
only sends goals and limits.

This repository makes those claims testable:

- **`npm run prove`** runs a fixed set of scenarios headless and checks each hypothesis
  as an automated test, then writes the measured numbers to `docs/RESULTS.md`.
- **The web demo** shows two identical arms side by side, one with the network off and one
  with it on. You inject vibration, ripple, a tap or an impact at any joint and watch both.

The joint code is real, not scripted. Each joint is a separate instance of a `no_std` Rust
crate shaped like actuator firmware. It sees only its own sensors and the bus frames sent
to it, and tests check that nothing else reaches it.

> **Status:** early development. See the [ticket board](docs/tickets/BOARD.md) for what
> works today.

## Repository layout

| Path | What it holds |
|---|---|
| `crates/nexus-proto` | Bus frames shared by nodes and host (Rust, `no_std`) |
| `crates/nexus-node` | Control logic of one actuator node (Rust, `no_std`) |
| `crates/nexus-wasm` | Runs one node per WebAssembly instance (C ABI) |
| `sim/` | Simulation harness: MuJoCo plant, sensor models, bus, host (TypeScript) |
| `web/` | Browser demo (Vite); a step-cost page until NX-20 |
| `docs/PLAN.md` | Behaviours, hypotheses, architecture and tickets |
| `docs/adr/` | Architecture decision records |
| `docs/tickets/` | One file per ticket, plus the generated board |

More folders (`experiments/`, `learning/`) arrive with their tickets.

## Getting started

Requirements: Rust 1.99 (installed automatically by `rustup` from `rust-toolchain.toml`)
and Node.js 22 or newer.

```bash
npm install
npm run check
```

To see the simulation speed on your machine, run `npm run bench:step` (headless) or
`npm run dev -w web` and open the printed address.

`npm run check` formats-checks, lints and tests both the Rust and the TypeScript code, and
verifies that the ticket board is up to date.

## How the work is tracked

Work follows the tickets in [docs/PLAN.md](docs/PLAN.md). Each ticket has its own file with
acceptance criteria and a log, lands as one commit with its ID in the subject, and is noted
in the [CHANGELOG](CHANGELOG.md).
