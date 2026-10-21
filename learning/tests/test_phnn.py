"""Tests of the per-joint port-Hamiltonian model (NX-14)."""

from __future__ import annotations

import json
import math

import numpy as np
import torch

from nexus_learning.baselines import nominal_residual, score
from nexus_learning.data import JointMeta, Run
from nexus_learning.export import export_weights
from nexus_learning.phnn import JointPHNN
from nexus_learning.train import energy_check, tensors, train

META = JointMeta(
    joint=4,
    gear_ratio=9.0,
    pole_pairs=14,
    rated_torque=12.0,
    reflected_inertia=0.005,
    gear_stiffness=6000.0,
    viscous_friction=0.05,
    coulomb_friction=0.3,
    fingerprint=(9.0, 36.0, 756.0, 1.0),
    cycle_hz=4000.0,
)


def synthetic_run(seconds: float = 4.0, seed: int = 1) -> Run:
    """A joint whose friction and cogging differ from its datasheet, simulated exactly."""
    rng = np.random.default_rng(seed)
    n = int(seconds * META.cycle_hz)
    t = np.arange(n) / META.cycle_hz
    theta = 0.3 * np.sin(2 * np.pi * 0.4 * t) + 0.05 * np.sin(2 * np.pi * 3.1 * t + 1.0)
    omega = np.gradient(theta, t)
    alpha = np.gradient(omega, t)
    phi = 1e-3 * np.sin(2 * np.pi * 1.7 * t) + 2e-4 * rng.standard_normal(n).cumsum() / 100
    phi_rate = np.gradient(phi, t)
    friction = 0.2 * omega + 0.6 * np.tanh(omega / 0.01)  # 4× viscous, 2× Coulomb
    cogging = 0.15 * np.sin(36.0 * theta + 0.4)
    gear = META.gear_stiffness * phi
    u = META.reflected_inertia * alpha + friction - cogging - gear
    return Run(
        meta=META,
        theta=theta,
        omega=omega,
        phi=phi,
        phi_rate=phi_rate,
        temperature=np.full(n, 30.0),
        u=u,
        alpha=alpha,
    )


def test_model_cannot_create_energy_whatever_its_weights() -> None:
    torch.manual_seed(3)
    model = JointPHNN(META)
    with torch.no_grad():
        for p in model.parameters():
            p.add_(torch.randn_like(p))
    check = energy_check(model, synthetic_run(1.0))
    assert check["minDissipation"] >= 0.0
    assert check["maxImbalance"] < 1e-9 * max(1.0, check["meanSupplied"])


def test_learns_friction_and_cogging_nobody_described() -> None:
    run = synthetic_run()
    model, losses = train(run, epochs=400)
    assert losses[-1] < losses[0]
    learned = model.residual(tensors(run)).detach().numpy()
    before = score(nominal_residual(run), META.cycle_hz)
    after = score(learned, META.cycle_hz)
    assert after < 0.1 * before, f"nominal {before:.3f}, learned {after:.3f}"


def test_exports_every_layer(tmp_path) -> None:
    path = tmp_path / "joint4.json"
    export_weights(JointPHNN(META), path, version=1, unit="test")
    document = json.loads(path.read_text())
    assert document["format"] == "nexus-phnn" and document["version"] == 1
    assert [len(layer["weight"]) for layer in document["gear"]] == [16, 16, 1]
    assert len(document["cogging"]["weight"]) == 2 * len(document["cogging"]["orders"])
    assert math.isclose(document["inertia"], META.reflected_inertia, rel_tol=1e-6)
