"""Coordinated models (NX-31): each joint's share of its neighbourhood's dynamics.

A joint's learned model of its actuator (``phnn.py``) explains the motor side of its gear.
What the gear carries, the arm beyond it, depends on how the neighbours move. From
neighbourhood commissioning (every joint on its own excitation at once), joint ``i``
learns two things from its own published state and that of the joints up to ``reach``
places away, which is what the bus brings it:

* **Its row of the neighbourhood's inertia.** The torque its gear delivers to its link,

      τ_i = Σ_j m_ij(q_N) · q̈_j + g_i(q_N),

  with ``m_ij`` and ``g_i`` linear in Fourier features of the neighbourhood's angles
  ``q_N`` (a serial arm's mass matrix is a sum of such terms). ``m_ij`` may depend only on
  the joints beyond the inner of ``i`` and ``j``, as in any serial arm's mass matrix: fitted
  freely, it took up dependence on the base joint's angle that is not there, and away from
  the commissioning postures it was wrong. At run time the joint feeds
  forward ``Σ_j m_ij · q̈_ref,j`` from the references its neighbours publish; gravity stays
  with the gravity chain, which also knows the payload. ``m_ii`` must stay positive over
  the data, which is checked.

* **Its link's compliance.** The link bends under the torque it carries,
  ``b_i = c_i · τ_i``. No joint can measure its link's tilt (an accelerometer bias of
  0.02 g is 20 mrad), but the bend *rate* is measurable: the next joint's gyro, which sits
  at the link's far end, minus the joint's own gyro, on the joint's axis, minus its own
  speed. Integrated, with the gyros' bias taken out as a constant, it is the bend up to a
  constant; ``c_i`` is fitted on that against the torque, below the link's modes.

The gear torque is the datasheet stiffness times the measured twist, delivered to the
link as ``−k·φ``.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .data import read_csv, smooth, smoothing_kernel

#: Every signal is smoothed with the same zero-phase window before fitting, Hz: enough for
#: the commissioning's band (0.2–6 Hz) and the arm's first modes.
SMOOTH_HZ = 20.0
#: Fitting keeps one sample in this many after smoothing.
DECIMATE = 4
#: The bend and torque are compared below this corner, Hz (under the light arm's first
#: bending mode, so the bend is quasi-static).
BEND_HZ = 3.0
#: Ridge weight on the inertia fit, relative to the mean diagonal of the normal matrix.
RIDGE = 1e-7
#: Share of a link's bend its own joint takes out; the next joint takes the rest. Exact
#: for a bend concentrated at mid-length, as the simulation models a flexible link.
BEND_SPLIT = 0.5
#: A compliance fit explaining less of the bend than this is taken as a rigid link. Rigid
#: links score under 0.1; the light arm's flexible ones 0.4–0.6, the rest being gyro noise
#: in the integrated bend (their compliance is within 3 % of the true one).
MIN_BEND_FIT = 0.2
#: Probe angles and accelerations written with each model, so the node's evaluation can be
#: checked against this one.
PROBE_SEEDS = (11, 12, 13)

FORMAT = "nexus-coordinated"
VERSION = 1


def feature_count(joints: int) -> int:
    """Number of features over ``joints`` angles."""
    return 1 + 2 * joints + 4 * (joints * (joints - 1) // 2)


def features(q: np.ndarray) -> np.ndarray:
    """Fourier features of the angles ``q`` (samples × joints).

    A constant, then cos and sin of every angle, then cos and sin of every pair's sum and
    difference.
    """
    q = np.atleast_2d(q)
    cols = [np.ones(len(q))]
    k = q.shape[1]
    for a in range(k):
        cols += [np.cos(q[:, a]), np.sin(q[:, a])]
    for a in range(k):
        for b in range(a + 1, k):
            s, d = q[:, a] + q[:, b], q[:, a] - q[:, b]
            cols += [np.cos(s), np.sin(s), np.cos(d), np.sin(d)]
    return np.stack(cols, axis=1)


@dataclass(frozen=True)
class JointInfo:
    """What a node knows about itself (its EEPROM): joint number, axis, gear stiffness."""

    joint: int
    axis: tuple[float, float, float]
    gear_stiffness: float
    reflected_inertia: float


@dataclass
class ArmRun:
    """One neighbourhood run, smoothed and decimated: one column per joint."""

    q: np.ndarray
    """Output angles, rad."""
    acc: np.ndarray
    """Output accelerations (from the smoothed observer speeds), rad/s²."""
    torque: np.ndarray
    """Torque each gear delivers to its link, ``−k·φ``, N·m."""
    turned: np.ndarray
    """Integrated gyro rates, samples × joints × 3 (each in its housing frame), rad (up to
    a constant)."""
    axes: np.ndarray
    """Each joint's axis, joints × 3."""
    dt: float
    """Sample time after decimation, s."""
    truth: dict[str, np.ndarray] = field(default_factory=dict)
    """Ground truth on the same samples (evaluation only)."""


def read_joints(dataset: Path) -> list[JointInfo]:
    """The per-joint parameters a dataset's nodes know about themselves."""
    meta = json.loads((dataset / "meta.json").read_text())
    return [
        JointInfo(
            joint=j["joint"],
            axis=tuple(j["axis"]),
            gear_stiffness=j["gearStiffness"],
            reflected_inertia=j["reflectedInertia"],
        )
        for j in meta["joints"]
    ]


def load_arm_run(dataset: Path, name: str, with_truth: bool = False) -> ArmRun:
    """Loads ``<name>.csv`` of a neighbourhood dataset (e.g. ``home-train``)."""
    meta = json.loads((dataset / "meta.json").read_text())
    joints = read_joints(dataset)
    dt = 1.0 / meta["cycleHz"]
    log = read_csv(dataset / f"{name}.csv")
    if not np.all(np.diff(log["cycle"]) == 1):
        raise ValueError(f"{dataset.name} {name}: log has gaps")
    kernel = smoothing_kernel(meta["cycleHz"], SMOOTH_HZ)

    def col(j: JointInfo, name: str) -> np.ndarray:
        return log[f"j{j.joint}_{name}"]

    q, acc, torque, turned = [], [], [], []
    for j in joints:
        speed = smooth(col(j, "output_velocity"), kernel)
        q.append(smooth(col(j, "output_angle"), kernel)[1:-1])
        acc.append((speed[2:] - speed[:-2]) / (2.0 * dt))
        torque.append(smooth(-j.gear_stiffness * col(j, "gear_twist"), kernel)[1:-1])
        turned.append([smooth(np.cumsum(col(j, f"gyro_{x}")) * dt, kernel)[1:-1] for x in "xyz"])
    run = ArmRun(
        q=np.stack(q, axis=1)[::DECIMATE],
        acc=np.stack(acc, axis=1)[::DECIMATE],
        torque=np.stack(torque, axis=1)[::DECIMATE],
        turned=np.transpose(np.array(turned), (2, 0, 1))[::DECIMATE],
        axes=np.array([j.axis for j in joints], dtype=float),
        dt=dt * DECIMATE,
    )
    if with_truth:
        raw = read_csv(dataset / "truth" / f"{name}.csv")
        if not np.array_equal(raw["cycle"], log["cycle"]):
            raise ValueError("truth and log cycles differ")
        for column in ("output_acceleration", "link_bend"):
            series = [smooth(raw[f"j{j.joint}_{column}"], kernel)[1:-1][::DECIMATE] for j in joints]
            run.truth[column] = np.stack(series, axis=1)
    return run


def feature_mask(cols: list[int], allowed: set[int]) -> np.ndarray:
    """Which features of the angles of ``cols`` involve only the joints in ``allowed``."""
    k = len(cols)
    ok = [c in allowed for c in cols]
    mask = [True]
    for a in range(k):
        mask += [ok[a], ok[a]]
    for a in range(k):
        for b in range(a + 1, k):
            mask += [ok[a] and ok[b]] * 4
    return np.array(mask)


def neighbourhood(joint: int, joints: int, reach: int) -> list[int]:
    """0-based columns of the joints up to ``reach`` places from ``joint`` (0-based)."""
    return [j for j in range(joint - reach, joint + reach + 1) if 0 <= j < joints]


@dataclass
class CoordinatedModel:
    """Joint ``joint``'s coordinated model (0-based index, as the columns)."""

    joint: int
    neighbours: list[int]
    """0-based columns of the joints the model reads, own included, ascending."""
    inertia: np.ndarray
    """Weights of ``m_ij``, one row per neighbour, over the features."""
    gravity: np.ndarray
    """Weights of ``g_i`` over the features (evaluation only; the node uses its chain)."""
    compliance: float
    """Bend of the joint's link per N·m it delivers, rad/(N·m); 0 for a rigid link."""
    bend_fit: float
    """Share of the bend's variance the compliance explains on the training data."""

    def row(self, q: np.ndarray) -> np.ndarray:
        """``m_ij`` at the neighbourhood's angles ``q`` (samples × neighbours)."""
        return features(q) @ self.inertia.T

    def inertial_torque(self, q: np.ndarray, acc: np.ndarray) -> np.ndarray:
        """``Σ_j m_ij · q̈_j`` (samples)."""
        return np.sum(self.row(q) * acc, axis=1)

    def torque(self, q: np.ndarray, acc: np.ndarray) -> np.ndarray:
        """Inertial plus gravity torque the joint's gear should deliver."""
        return self.inertial_torque(q, acc) + features(q) @ self.gravity


def fit(runs: list[ArmRun], joint: int, reach: int) -> CoordinatedModel:
    """Fits joint ``joint`` (0-based) on ``runs``, reading the joints up to ``reach`` away.

    ``reach = 0`` is the own-only model: the joint's own angle and acceleration only.
    """
    cols = neighbourhood(joint, runs[0].q.shape[1], reach)
    f = feature_count(len(cols))
    # m_ij depends on the joints beyond the inner of i and j; gravity on every joint.
    masks = [feature_mask(cols, {k for k in cols if k > min(joint, c)}) for c in cols]
    keep = np.concatenate([*masks, np.ones(f, dtype=bool)])
    blocks, targets = [], []
    for run in runs:
        phi = features(run.q[:, cols])
        inertial = [phi * run.acc[:, [c]] for c in cols]
        blocks.append(np.concatenate([*inertial, phi], axis=1)[:, keep])
        targets.append(run.torque[:, joint])
    a = np.concatenate(blocks)
    y = np.concatenate(targets)
    normal = a.T @ a
    ridge = RIDGE * np.trace(normal) / len(normal)
    w = np.zeros(len(keep))
    w[keep] = np.linalg.solve(normal + ridge * np.eye(len(normal)), a.T @ y)
    inertia = w[: f * len(cols)].reshape(len(cols), f)
    gravity = w[f * len(cols) :]
    compliance, bend_fit = fit_compliance(runs, joint)
    return CoordinatedModel(joint, cols, inertia, gravity, compliance, bend_fit)


def _bend_and_torque(run: ArmRun, joint: int) -> tuple[np.ndarray, np.ndarray] | None:
    """The link's measured bend (up to a constant) and its torque, below ``BEND_HZ``."""
    if joint + 1 >= run.q.shape[1]:
        return None  # no joint beyond: nobody measures this link's far end
    # The next joint's housing is this link's far end, and its frame is this link's: both
    # gyros on this joint's axis.
    axis = run.axes[joint]
    bend = (run.turned[:, joint + 1] - run.turned[:, joint]) @ axis - run.q[:, joint]
    kernel = smoothing_kernel(1.0 / run.dt, BEND_HZ)
    b, t = smooth(bend, kernel), smooth(run.torque[:, joint], kernel)
    # The gyros' bias integrates to a ramp: take out a straight line from both.
    x = np.arange(len(b))
    b = b - np.polyval(np.polyfit(x, b, 1), x)
    t = t - np.polyval(np.polyfit(x, t, 1), x)
    return b, t


def fit_compliance(runs: list[ArmRun], joint: int) -> tuple[float, float]:
    """Least-squares ``c`` in ``bend = c · torque`` and the share of variance explained."""
    pairs = [p for p in (_bend_and_torque(r, joint) for r in runs) if p is not None]
    if not pairs:
        return 0.0, 0.0
    b = np.concatenate([p[0] for p in pairs])
    t = np.concatenate([p[1] for p in pairs])
    if t @ t <= 0.0 or b @ b <= 0.0:
        return 0.0, 0.0
    c = float(t @ b / (t @ t))
    explained = 1.0 - float(np.sum((b - c * t) ** 2) / np.sum(b**2))
    return (c, explained) if explained >= MIN_BEND_FIT else (0.0, explained)


def scores(model: CoordinatedModel, run: ArmRun) -> dict[str, float]:
    """Held-out scores of ``model`` on ``run``.

    The share of the joint's torque the model leaves unexplained, the smallest ``m_ii``
    met, and (with truth) the compliance's bend against the true bend.
    """
    cols = model.neighbours
    q, acc = run.q[:, cols], run.acc[:, cols]
    y = run.torque[:, model.joint]
    residual = y - model.torque(q, acc)
    # The inertial part alone, against what the torque varies by beyond gravity.
    dynamic = y - features(q) @ model.gravity
    own = cols.index(model.joint)
    out = {
        "unexplained": float(np.sqrt(np.mean(residual**2) / np.mean((y - y.mean()) ** 2))),
        "dynamicUnexplained": float(
            np.sqrt(np.mean(residual**2) / max(np.mean(dynamic**2), 1e-12))
        ),
        "minOwnInertia": float(model.row(q)[:, own].min()),
    }
    if "link_bend" in run.truth and model.compliance != 0.0:
        bend = run.truth["link_bend"][:, model.joint]
        predicted = model.compliance * y
        out["bendError"] = float(np.sqrt(np.mean((bend - predicted) ** 2)))
        out["bendRms"] = float(np.sqrt(np.mean(bend**2)))
    return out


def export(model: CoordinatedModel, info: list[JointInfo], reach: int) -> dict:
    """The frozen model for the node, with probes of its feed-forward."""
    rng = np.random.default_rng(PROBE_SEEDS[0] + model.joint)
    probes = []
    for _ in PROBE_SEEDS:
        q = rng.uniform(-1.5, 1.5, (1, len(model.neighbours)))
        acc = rng.uniform(-5.0, 5.0, (1, len(model.neighbours)))
        probes.append(
            {
                "angles": q[0].tolist(),
                "accelerations": acc[0].tolist(),
                "inertialTorque": float(model.inertial_torque(q, acc)[0]),
            }
        )
    return {
        "format": FORMAT,
        "version": VERSION,
        "joint": model.joint + 1,
        "reach": reach,
        "neighbours": [c + 1 for c in model.neighbours],
        "features": feature_count(len(model.neighbours)),
        "inertia": model.inertia.tolist(),
        "compliance": model.compliance,
        "bendSplit": BEND_SPLIT,
        "axis": list(info[model.joint].axis),
        "probes": probes,
    }


#: Postures of the neighbourhood datasets (``experiments/src/commission-neighbourhood.ts``).
POSTURES = ("home", "reach", "mid")
#: Reaches exported: 0 (the joint alone, the baseline) and 2 (two joints either side).
REACHES = (0, 2)


def main() -> None:
    """Trains and exports every joint's coordinated model for each dataset, both reaches."""
    import argparse

    parser = argparse.ArgumentParser(description="Trains the coordinated models (NX-31).")
    parser.add_argument("datasets", nargs="+", type=Path)
    parser.add_argument("--out", type=Path, default=Path("../models/coordinated-models"))
    parser.add_argument(
        "--report", type=Path, default=Path("../experiments/results/coordinated-models.json")
    )
    args = parser.parse_args()
    report: dict[str, dict] = {}
    for dataset in args.datasets:
        arm = json.loads((dataset / "meta.json").read_text())["arm"]
        info = read_joints(dataset)
        train = [load_arm_run(dataset, f"{p}-train") for p in POSTURES]
        held = [load_arm_run(dataset, f"{p}-heldout", with_truth=True) for p in POSTURES]
        for reach in REACHES:
            out = args.out / f"{arm}-reach{reach}"
            out.mkdir(parents=True, exist_ok=True)
            rows = []
            for joint in range(len(info)):
                model = fit(train, joint, reach)
                if reach == 0:
                    # Alone, a joint cannot see its link bend: that takes the next joint's gyro.
                    model.compliance, model.bend_fit = 0.0, 0.0
                held_scores = [scores(model, run) for run in held]
                row = {
                    "joint": joint + 1,
                    "compliance": model.compliance,
                    "bendFit": model.bend_fit,
                    **{
                        key: float(np.mean([s[key] for s in held_scores if key in s]))
                        for key in held_scores[0]
                    },
                    "minOwnInertia": min(s["minOwnInertia"] for s in held_scores),
                }
                rows.append(row)
                doc = export(model, info, reach)
                (out / f"joint{joint + 1}.json").write_text(json.dumps(doc) + "\n")
                print(
                    f"{arm} reach {reach} J{joint + 1}: unexplained {row['unexplained']:.3f}, "
                    f"compliance {model.compliance:.3e} (fit {model.bend_fit:.2f})"
                )
            report[f"{arm}-reach{reach}"] = {"joints": rows}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
