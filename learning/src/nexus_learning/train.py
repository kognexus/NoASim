"""Trains each joint's PHNN on its commissioning run and reports H10a and H10c.

    uv run nexus-train ../experiments/results/commissioning/kx6-s300 [more datasets]
        [--out ../models] [--report ../experiments/results/phnn.json] [--epochs N]
        [--evaluate]

With ``--evaluate`` the committed weights are scored instead of trained (npm run prove).

For every joint of every dataset: train on the training run, then score Nominal, Fitted,
PHNN and Oracle on the held-out run (RMS of the smoothed torque-balance residual), check
the learned model's energy balance on the held-out run, and export the frozen weights.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import time
from pathlib import Path

import numpy as np
import torch

from .baselines import FittedModel, nominal_residual, oracle_residual, score
from .data import NARROW_HZ, Run, load_run, read_meta, read_truth_info, smoothing_kernel
from .export import export_weights, load_weights
from .phnn import JointPHNN

#: Version of the exported weight format and training recipe.
MODEL_VERSION = 1


def tensors(run: Run) -> dict[str, torch.Tensor]:
    """The run's signals as float64 tensors."""
    names = ("theta", "omega", "phi", "phi_rate", "temperature", "u", "alpha")
    return {n: torch.from_numpy(np.ascontiguousarray(getattr(run, n))) for n in names}


#: Share of the training run kept aside to choose when to stop.
VALIDATION_SHARE = 0.2
#: Epochs between validation checks.
VALIDATE_EVERY = 25


def train(run: Run, epochs: int = 1500, seed: int = 0) -> tuple[JointPHNN, list[float]]:
    """Fits a PHNN to ``run``: smoothed torque-balance residual, least squares.

    The last fifth of the run is kept aside; the weights that explain it best are kept.
    """
    torch.manual_seed(seed)
    model = JointPHNN(run.meta)
    data = tensors(run)
    split = int(len(run) * (1.0 - VALIDATION_SHARE))
    fit = {k: v[:split] for k, v in data.items()}
    check = {k: v[split:] for k, v in data.items()}
    kernel = torch.from_numpy(smoothing_kernel(run.meta.cycle_hz))[None, None, :]
    scale = run.meta.rated_torque

    def loss_on(part: dict[str, torch.Tensor]) -> torch.Tensor:
        residual = model.residual(part) / scale
        smoothed = torch.nn.functional.conv1d(residual[None, None, :], kernel)[0, 0]
        return torch.mean(smoothed**2)

    optimiser = torch.optim.Adam(model.parameters(), lr=1e-2)
    schedule = torch.optim.lr_scheduler.CosineAnnealingLR(optimiser, epochs, eta_min=1e-4)
    losses = []
    best = (math.inf, copy.deepcopy(model.state_dict()))
    for epoch in range(epochs):
        optimiser.zero_grad()
        loss = loss_on(fit)
        loss.backward()
        optimiser.step()
        schedule.step()
        losses.append(loss.item())
        if (epoch + 1) % VALIDATE_EVERY == 0:
            validation = loss_on(check).item()
            if validation < best[0]:
                best = (validation, copy.deepcopy(model.state_dict()))
    model.load_state_dict(best[1])
    return model, losses


def energy_check(model: JointPHNN, run: Run) -> dict[str, float]:
    """H10c on ``run``: the model's own power balance and the sign of its losses.

    Along the model's vector field, ``Ḣ`` must equal motor power plus output-port power
    minus dissipation, with the dissipation never negative.
    """
    d = tensors(run)
    f = model.forces(d["theta"], d["omega"], d["phi"], d["phi_rate"], d["temperature"], d["u"])
    theta = d["theta"].clone().requires_grad_(True)
    phi = d["phi"].clone().requires_grad_(True)
    grad_theta, grad_phi = torch.autograd.grad(model.potential(theta, phi).sum(), (theta, phi))
    omega, omega_out = d["omega"], d["omega"] + d["phi_rate"]
    # Ḣ = ω·ṗ + V_cog'(θ)·ω + V_gear'(φ)·φ̇, with ṗ from the model.
    h_dot = omega * f["total"] + grad_theta * omega + grad_phi * d["phi_rate"]
    c = model.gear_damping
    supplied = omega * f["drive"] + (grad_phi + c * d["phi_rate"]) * omega_out
    imbalance = h_dot - (supplied - f["dissipation"])
    return {
        "minDissipation": f["dissipation"].min().item(),
        "maxImbalance": imbalance.abs().max().item(),
        "meanSupplied": supplied.abs().mean().item(),
    }


def evaluate(
    dataset: Path, joint: int, epochs: int, frozen: Path | None = None
) -> tuple[dict, JointPHNN]:
    """Scores every model on joint ``joint``'s held-out run of ``dataset``.

    The PHNN is trained on the training run, or read from ``frozen`` (exported weights).
    """
    train_run = load_run(dataset, joint, "train")
    heldout = load_run(dataset, joint, "heldout", with_truth=True)
    info = read_truth_info(dataset)
    started = time.time()
    if frozen is None:
        model, losses = train(train_run, epochs)
    else:
        model, losses = load_weights(frozen, train_run.meta), [math.nan]
    phnn = model.residual(tensors(heldout)).detach().numpy()
    hz = heldout.meta.cycle_hz
    residuals = {
        "nominal": nominal_residual(heldout),
        "fitted": FittedModel(train_run).residual(heldout),
        "phnn": phnn,
        "oracle": oracle_residual(heldout, info["gearDamping"][joint - 1]),
    }
    scores = {name: score(r, hz) for name, r in residuals.items()}
    narrow = {name: score(r, hz, NARROW_HZ) for name, r in residuals.items()}
    gap = scores["nominal"] - scores["oracle"]
    result = {
        "dataset": dataset.name,
        "joint": joint,
        "family": info["families"][joint - 1],
        "scores": scores,
        "scoresNarrow": narrow,
        "gapClosed": (scores["nominal"] - scores["phnn"]) / gap if gap > 0 else math.nan,
        "energy": energy_check(model, heldout),
        "trainLoss": losses[-1],
        "seconds": round(time.time() - started, 1),
    }
    return result, model


def family_summary(results: list[dict]) -> dict[str, dict]:
    """Per family: RMS of each model's score over its joints, and the gap the PHNN closes."""
    summary: dict[str, dict] = {}
    for family in sorted({r["family"] for r in results}):
        rows = [r for r in results if r["family"] == family]
        rms = {
            m: math.sqrt(sum(r["scores"][m] ** 2 for r in rows) / len(rows))
            for m in ("nominal", "fitted", "phnn", "oracle")
        }
        gap = rms["nominal"] - rms["oracle"]
        summary[family] = {
            "joints": len(rows),
            **rms,
            "phnnGapClosed": (rms["nominal"] - rms["phnn"]) / gap if gap > 0 else math.nan,
            "fittedGapClosed": (rms["nominal"] - rms["fitted"]) / gap if gap > 0 else math.nan,
        }
    return summary


def finite(value: object) -> object:
    """``value`` with NaN and infinities replaced by None, so the report is valid JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: finite(v) for k, v in value.items()}
    if isinstance(value, list):
        return [finite(v) for v in value]
    return value


def main() -> None:
    """Command-line entry point."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("datasets", nargs="+", type=Path)
    parser.add_argument("--out", type=Path, default=Path("../models"))
    parser.add_argument("--report", type=Path, default=Path("../experiments/results/phnn.json"))
    parser.add_argument("--epochs", type=int, default=1500)
    parser.add_argument("--joints", type=str, default="", help="comma-separated, default all")
    parser.add_argument(
        "--evaluate",
        action="store_true",
        help="score the weights already in --out instead of training (deterministic)",
    )
    args = parser.parse_args()
    results = []
    for dataset in args.datasets:
        joints = [int(j) for j in args.joints.split(",") if j] or [
            m.joint for m in read_meta(dataset)
        ]
        for joint in joints:
            path = args.out / dataset.name / f"joint{joint}.json"
            result, model = evaluate(dataset, joint, args.epochs, path if args.evaluate else None)
            if not args.evaluate:
                export_weights(model, path, MODEL_VERSION, dataset.name)
            results.append(result)
            s = result["scores"]
            print(
                f"{dataset.name} j{joint} {result['family']:<22} nominal {s['nominal']:.3f}"
                f"  fitted {s['fitted']:.3f}  phnn {s['phnn']:.3f}  oracle {s['oracle']:.3f}"
                f"  closed {result['gapClosed']:.0%}  ({result['seconds']} s)",
                flush=True,
            )
    summary = family_summary(results)
    for family, s in summary.items():
        print(
            f"{family:<24} nominal {s['nominal']:.3f}  fitted {s['fitted']:.3f}"
            f"  phnn {s['phnn']:.3f}  oracle {s['oracle']:.3f}"
            f"  PHNN closes {s['phnnGapClosed']:.0%}, fitted {s['fittedGapClosed']:.0%}"
        )
    args.report.parent.mkdir(parents=True, exist_ok=True)
    report = {"version": MODEL_VERSION, "results": results, "families": summary}
    args.report.write_text(json.dumps(finite(report), indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
