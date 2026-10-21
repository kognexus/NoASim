# learning/

Offline training of each joint's port-Hamiltonian model of itself (B11, NX-14). Python
never runs in the control loop: it reads the joints' commissioning logs and writes
frozen, versioned weights that the Rust node evaluates (`crates/nexus-node/src/phnn.rs`).

```bash
# 1. Commissioning logs (NX-13), from the repository root
node experiments/src/commission.ts 3
# 2. Train, evaluate (H10a, H10c) and export
cd learning
uv run nexus-train ../experiments/results/commissioning/kx6-s300 \
  ../experiments/results/commissioning/kx6-s301 ../experiments/results/commissioning/kx6-s302
```

Weights go to `models/<unit>/joint<n>.json`. The report goes to
`experiments/results/phnn.json`.

| Module | What it does |
|---|---|
| `data.py` | Reads a joint's log and aligns the motor-side balance (α centred on a cycle, the commands that acted around it) |
| `phnn.py` | The model: `H = p²/2I + V_cog(θ) + V_gear(φ)`, friction `R(θ, ω) ≥ 0`, gear damping `c ≥ 0`, gain `g(T, θ) > 0` |
| `baselines.py` | Nominal (datasheet), Fitted (the node's RLS structure, least squares) and Oracle (ground truth, evaluation only) |
| `train.py` | Training loop, held-out scoring, energy-balance check, CLI |
| `export.py` | JSON weights plus probe values the node is checked against (`sim/src/node/phnnModel.test.ts`) |

The model sees only what the joint publishes and what it knows about itself (its EEPROM
values and announced orders). The `truth/` folder of a dataset is read only to score the
Oracle.

Tests: `uv run pytest` (passivity for any weights, recovery of an actuator nobody
described, export shape). Lint: `uv run ruff check . && uv run ruff format --check .`.
