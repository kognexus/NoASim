"""Frozen, versioned weights for the node (NX-15 reads them)."""

from __future__ import annotations

import json
import math
from pathlib import Path

import torch
from torch import nn

from .data import JointMeta
from .phnn import INERTIA_RANGE, OMEGA_REF, STICTION_SPEED, TEMPERATURE_REF, JointPHNN

#: Probe inputs written with every model, so the node's evaluation can be checked against
#: PyTorch's: (θ, ω, φ / twist_ref, φ̇, T, u / rated, α).
PROBES = (
    (0.10, 0.50, 0.20, 0.01, 30.0, 0.30, 20.0),
    (-0.70, -1.20, -0.60, -0.05, 45.0, -0.20, -60.0),
    (1.30, 0.02, 0.90, 0.00, 60.0, 0.05, 5.0),
    (2.40, -0.004, -1.10, 0.08, 25.0, -0.45, 0.0),
)


def probe(model: JointPHNN) -> list[dict[str, float]]:
    """The model's residual ``I·α − ṗ`` at each probe, float64."""
    m = model.meta
    rows = []
    for theta, omega, twist, twist_rate, temperature, u, alpha in PROBES:
        inputs = {
            "theta": theta,
            "omega": omega,
            "phi": twist * model.twist_ref,
            "phi_rate": twist_rate,
            "temperature": temperature,
            "u": u * m.rated_torque,
            "alpha": alpha,
        }
        tensors = {k: torch.tensor([v], dtype=torch.float64) for k, v in inputs.items()}
        rows.append({**inputs, "unexplained": float(model.residual(tensors)[0].detach())})
    return rows


def _layers(net: nn.Sequential) -> list[dict[str, list]]:
    """Weights and biases of each linear layer (tanh between them)."""
    return [
        {"weight": layer.weight.detach().tolist(), "bias": layer.bias.detach().tolist()}
        for layer in net
        if isinstance(layer, nn.Linear)
    ]


def _load_layers(net: nn.Sequential, layers: list[dict]) -> None:
    linear = [layer for layer in net if isinstance(layer, nn.Linear)]
    for layer, saved in zip(linear, layers, strict=True):
        layer.weight.copy_(torch.tensor(saved["weight"], dtype=torch.float64))
        layer.bias.copy_(torch.tensor(saved["bias"], dtype=torch.float64))


def load_weights(path: Path, meta: JointMeta) -> JointPHNN:
    """Rebuilds a model exported by :func:`export_weights` (for the joint ``meta``)."""
    document = json.loads(path.read_text())
    model = JointPHNN(meta)
    with torch.no_grad():
        spread = math.log(document["inertia"] / meta.reflected_inertia) / math.log(INERTIA_RANGE)
        model.raw_inertia.copy_(torch.tensor(spread).atanh())
        damping = document["gearDamping"] / (1e-3 * meta.gear_stiffness)
        model.raw_gear_damping.copy_(torch.tensor(damping).expm1().clamp_min(1e-30).log())
        model.cog.weight.copy_(torch.tensor([document["cogging"]["weight"]], dtype=torch.float64))
        _load_layers(model.gear, document["gear"])
        _load_layers(model.viscous, document["friction"]["viscous"])
        _load_layers(model.coulomb, document["friction"]["coulomb"])
        model.gain_angle.weight.copy_(
            torch.tensor([document["gain"]["angle"]], dtype=torch.float64)
        )
        _load_layers(model.gain_temperature, document["gain"]["temperature"])
    return model


def export_weights(model: JointPHNN, path: Path, version: int, unit: str) -> None:
    """Writes ``model`` as JSON: everything the node needs to evaluate it."""
    m = model.meta
    with torch.no_grad():
        document = {
            "format": "nexus-phnn",
            "version": version,
            "unit": unit,
            "joint": m.joint,
            "constants": {
                "ratedTorque": m.rated_torque,
                "gearStiffness": m.gear_stiffness,
                "twistRef": model.twist_ref,
                "omegaRef": OMEGA_REF,
                "stictionSpeed": STICTION_SPEED,
                "temperatureRef": list(TEMPERATURE_REF),
                "viscousScale": model.viscous_scale,
                "coulombScale": model.coulomb_scale,
            },
            "inertia": float(model.inertia),
            "gearDamping": float(model.gear_damping),
            "cogging": {
                "orders": model.cog_features.orders.tolist(),
                "weight": model.cog.weight[0].tolist(),
            },
            "gear": _layers(model.gear),
            "friction": {
                "viscous": _layers(model.viscous),
                "coulomb": _layers(model.coulomb),
            },
            "gain": {
                "orders": model.gain_features.orders.tolist(),
                "angle": model.gain_angle.weight[0].tolist(),
                "temperature": _layers(model.gain_temperature),
            },
        }
    document["probes"] = probe(model)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=1) + "\n")
