# KognexusNexus

**A simulation of robot joints that work as a team.**

Each actuator of a 6-DOF arm carries its own processor, encoders, motion sensor and safety
path. Every 250 µs the joints share what they sense with their neighbours over an
EtherCAT-like bus. With that shared data they:
- damp vibration that a single joint can't reach;
- trace a disturbance back to the joint that causes it, which fixes it or reports that it
  can't;
- stop the whole arm when any one of them feels an impact;
- compute gravity from their own IMUs;
- turn the sparse goals of a slow AI policy into smooth motion.

Each joint also learns a model of its own actuator, without being told what the actuator
is. The robot's main computer only sends goals, limits and settings.

![One tap, three ways to damp it](docs/results/h1-tap.svg)

The joint code is real, not scripted. Each joint is a separate WebAssembly instance of a
`no_std` Rust crate shaped like actuator firmware. It sees only its own sensors and the bus
frames sent to it, and a test checks that nothing else can reach it.

## Try it

Requirements:
- Rust (`rustup` installs 1.99 from `rust-toolchain.toml`);
- Node.js 22 or newer;
- for the learned models, [uv](https://docs.astral.sh/uv/).

```bash
npm install
npm run build:wasm
npm run dev -w web
```

Open the printed address. Two identical arms stand side by side; set one to network
damping and the other to joint control only, then:
- **Tap the tool** to see how each arm damps it.
- **Click a joint** and give it a torque ripple. With "network + report to source" the
  joint names itself and cancels a fixable ripple; with an unfixable one, it reports a
  fault.
- **Collide** to see the impact policy act on every joint at once.

The bus view shows what travels between the joints.

```bash
npm run prove   # measures every hypothesis, writes docs/RESULTS.md (about 20 min)
npm run check   # format, lint and test Rust, TypeScript and Python
```

## What it shows

Every number is in [docs/RESULTS.md](docs/RESULTS.md), measured by `npm run prove` against
the targets set in the plan before any code was written. In short:

| | Result |
|---|---|
| Impact reaction (H3, H4) | Every joint reacts within 1 cycle of detection. Retreat-and-hold, E-stop with brakes and the brake configuration check all work. No false stops in 15 fast tasks. |
| Report to source (H5, H7) | A joint finds its own torque ripple, cancels it within 0.7 s or reports a fault 2 s after naming itself. The source is named 100/100 times. |
| Learned model per joint (H10) | Passive by construction. Beats a fitted textbook model on every actuator family. On actuators nobody described, it cuts false fault claims from 6/30 to 1/30. |
| Network damping (H1, H6) | Stable and energy-bounded with 20 cycles of latency and 20 % lost frames. 2–2.5× faster settling than joint control alone, **but not faster than joint-local damping**: the 10×/3× targets are missed. |
| Joint-level interpolation (H2) | Path error 2–5× lower in every task, but jerk only 1–3× lower (target 5×). |
| Gravity from each IMU (H9) | Survives a 15° tilted or accelerating base far better than a level-base model; three joints and the payload estimate miss the 1 % / 5 % targets. |
| Sealed actuator family (H10a) | 61 % of the gap to the truth closed, target 70 %: missed. |

What this does and does not prove is in [docs/limitations.md](docs/limitations.md).

## Repository layout

| Path | What it holds |
|---|---|
| `crates/nexus-proto` | Bus frames shared by nodes and host (Rust, `no_std`), pinned by golden bytes |
| `crates/nexus-node` | Everything one actuator node computes (Rust, `no_std`) |
| `crates/nexus-wasm` | One node per WebAssembly instance (C ABI) |
| `sim/` | Harness: MuJoCo plant, actuator and sensor models, bus, host, scenarios (TypeScript) |
| `web/` | The interactive demo (Vite, three.js) |
| `experiments/` | `npm run prove`, the tuner and the experiments behind each hypothesis |
| `learning/` | Training of each joint's port-Hamiltonian model (Python, uv, PyTorch) |
| `models/` | Frozen, versioned per-joint model weights |
| `docs/` | Plan, ADRs, tickets, results, [architecture](docs/architecture.md), [the learned model](docs/phnn.md), [limitations](docs/limitations.md) |

## How the work is tracked

The plan ([docs/PLAN.md](docs/PLAN.md)) was reviewed before any code was written. Each
ticket has its own file in `docs/tickets/` with acceptance criteria, tests and a log of
what was found, including what did not work. Each ticket landed as one commit with its ID
in the subject, and is noted in the [CHANGELOG](CHANGELOG.md). Decisions are recorded as
ADRs in `docs/adr/`.

## Licence

Not chosen yet. Until a licence is added, all rights are reserved.
