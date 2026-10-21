"""Tests of the coordinated models (NX-31)."""

from __future__ import annotations

import numpy as np

from nexus_learning.coordinated import (
    ArmRun,
    JointInfo,
    export,
    feature_count,
    feature_mask,
    features,
    fit,
)


def test_features_count_and_order():
    q = np.array([[0.3, -0.2]])
    phi = features(q)[0]
    assert len(phi) == feature_count(2) == 9
    expect = [1.0, np.cos(0.3), np.sin(0.3), np.cos(-0.2), np.sin(-0.2)]
    expect += [np.cos(0.1), np.sin(0.1), np.cos(0.5), np.sin(0.5)]
    assert np.allclose(phi, expect)


def test_mask_keeps_only_features_of_allowed_joints():
    mask = feature_mask([1, 2], {2})
    # constant, cos/sin of joint 2, no pair (it involves joint 1)
    assert mask.tolist() == [True, False, False, True, True, False, False, False, False]


def _arm(seed: int, samples: int = 4000) -> tuple[ArmRun, np.ndarray]:
    """Three joints whose torques follow a known, angle-dependent inertia row."""
    rng = np.random.default_rng(seed)
    t = np.arange(samples) * 1e-3
    q = np.stack(
        [0.3 * np.sin(2 * np.pi * f * t + p) for f, p in [(0.7, 0), (1.3, 1), (2.9, 2)]], 1
    )
    acc = rng.normal(0, 3, (samples, 3))
    # Joint 0's row: m_00 = 2 + 0.5 cos q1 (depends on the joint beyond it), m_01 = 0.4
    # (constant), m_02 = 0; gravity 5 cos q0.
    m00 = 2 + 0.5 * np.cos(q[:, 1])
    torque = np.zeros((samples, 3))
    torque[:, 0] = m00 * acc[:, 0] + 0.4 * acc[:, 1] + 5 * np.cos(q[:, 0])
    run = ArmRun(
        q=q,
        acc=acc,
        torque=torque,
        turned=np.zeros((samples, 3, 3)),
        axes=np.array([[0, 1, 0]] * 3, dtype=float),
        dt=1e-3,
    )
    return run, m00


def test_fit_recovers_a_known_row():
    run, m00 = _arm(1)
    model = fit([run], joint=0, reach=2)
    row = model.row(run.q[:, model.neighbours])
    assert np.allclose(row[:, 0], m00, atol=1e-3)
    assert np.allclose(row[:, 1], 0.4, atol=1e-3)
    assert np.allclose(row[:, 2], 0.0, atol=1e-3)
    held, _ = _arm(2)
    predicted = model.torque(held.q[:, model.neighbours], held.acc[:, model.neighbours])
    assert np.sqrt(np.mean((predicted - held.torque[:, 0]) ** 2)) < 1e-2


def test_export_carries_the_row_and_probes_it():
    run, _ = _arm(3)
    model = fit([run], joint=1, reach=1)
    info = [JointInfo(j + 1, (0.0, 1.0, 0.0), 6e4, 0.01) for j in range(3)]
    doc = export(model, info, reach=1)
    assert doc["neighbours"] == [1, 2, 3]
    assert len(doc["inertia"]) == 3 and len(doc["inertia"][0]) == doc["features"]
    probe = doc["probes"][0]
    q = np.array([probe["angles"]])
    acc = np.array([probe["accelerations"]])
    assert np.isclose(model.inertial_torque(q, acc)[0], probe["inertialTorque"])
