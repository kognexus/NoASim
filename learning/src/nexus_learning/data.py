"""Commissioning logs (NX-13) as aligned arrays.

A joint's log holds one row per bus cycle of what it published. The motor-side balance a
model has to explain is, with every torque output-referred,

    I·α = g·u + τ_gear(φ) + c·φ̇ − τ_friction(θ, ω) + τ_cogging(θ)

where α is the motor's angular acceleration. It is taken from the second difference of the
encoder angle, centred on cycle k. The command logged at cycle k acts over the next
cycle, so the two cycles around k felt the commands logged at k−2 and k−1; ``u`` is
their mean.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

#: Corner of the smoothing every residual passes before it is compared, Hz. The second
#: difference of a 14-bit encoder is far too noisy to compare sample by sample, but the
#: band must hold what a model is meant to explain: torque ripple at the sixth electrical
#: harmonic runs at 50–150 Hz at commissioning speeds. (At 50 Hz it was filtered out of
#: the loss, and the model could not learn it; NX-14 log.)
COMPARE_HZ = 200.0
#: A narrower band, reported next to the main one.
NARROW_HZ = 50.0


@dataclass(frozen=True)
class JointMeta:
    """What a node knows about itself: its EEPROM values (never the hidden truth)."""

    joint: int
    gear_ratio: float
    pole_pairs: int
    rated_torque: float
    reflected_inertia: float
    gear_stiffness: float
    viscous_friction: float
    coulomb_friction: float
    fingerprint: tuple[float, ...]
    cycle_hz: float

    @property
    def dt(self) -> float:
        """Bus cycle, s."""
        return 1.0 / self.cycle_hz


@dataclass(frozen=True)
class Run:
    """One commissioning run of one joint, aligned on the cycles where α is defined."""

    meta: JointMeta
    theta: np.ndarray
    """Motor angle (output-referred), rad."""
    omega: np.ndarray
    """Motor velocity from the node's observer, rad/s."""
    phi: np.ndarray
    """Gear twist (output minus motor), rad."""
    phi_rate: np.ndarray
    """Gear twist rate (central difference), rad/s."""
    temperature: np.ndarray
    """Winding temperature, °C."""
    u: np.ndarray
    """Commanded torque that acted around each cycle, N·m."""
    alpha: np.ndarray
    """Motor acceleration (second difference), rad/s²."""
    truth: dict[str, np.ndarray] | None = None
    """Ground-truth actuator torques around each cycle (only for the Oracle and evaluation)."""

    def __len__(self) -> int:
        """Number of aligned cycles."""
        return len(self.theta)


def read_csv(path: Path) -> dict[str, np.ndarray]:
    """Reads a log or truth CSV into named columns."""
    with path.open() as f:
        header = f.readline().strip().split(",")
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return {name: data[:, i] for i, name in enumerate(header)}


def read_meta(dataset: Path) -> list[JointMeta]:
    """The per-joint parameters a dataset's nodes know about themselves."""
    meta = json.loads((dataset / "meta.json").read_text())
    return [
        JointMeta(
            joint=j["joint"],
            gear_ratio=j["gearRatio"],
            pole_pairs=j["polePairs"],
            rated_torque=j["ratedTorque"],
            reflected_inertia=j["reflectedInertia"],
            gear_stiffness=j["gearStiffness"],
            viscous_friction=j["viscousFriction"],
            coulomb_friction=j["coulombFriction"],
            fingerprint=tuple(j["fingerprint"]),
            cycle_hz=meta["cycleHz"],
        )
        for j in meta["joints"]
    ]


def read_truth_info(dataset: Path) -> dict:
    """Hidden families and the plant's gear damping (evaluation only)."""
    return json.loads((dataset / "truth" / "families.json").read_text())


def load_run(dataset: Path, joint: int, part: str, with_truth: bool = False) -> Run:
    """Loads joint ``joint`` (1-based) of ``dataset``, ``part`` = "train" or "heldout"."""
    meta = read_meta(dataset)[joint - 1]
    log = read_csv(dataset / f"joint{joint}-{part}.csv")
    cycles = log["cycle"]
    if not np.all(np.diff(cycles) == 1):
        raise ValueError(f"{dataset.name} joint {joint} {part}: log has gaps")
    dt = meta.dt
    theta, phi, cmd = log["motor_angle"], log["gear_twist"], log["torque_command"]
    k = slice(2, len(theta) - 1)  # cycles with k−2 and k+1 available
    alpha = (theta[3:] - 2.0 * theta[2:-1] + theta[1:-2]) / dt**2
    u = 0.5 * (cmd[:-3] + cmd[1:-2])
    phi_rate = (phi[3:] - phi[1:-2]) / (2.0 * dt)
    truth = None
    if with_truth:
        raw = read_csv(dataset / "truth" / f"joint{joint}-{part}.csv")
        if not np.array_equal(raw["cycle"], cycles):
            raise ValueError("truth and log cycles differ")
        # Physics row j acts over [j, j+1]; the two cycles around k are rows k−1 and k.
        truth = {
            name: 0.5 * (raw[name][1:-2] + raw[name][2:-1])
            for name in ("drive_torque", "friction_torque", "gear_extra_torque")
        }
    return Run(
        meta=meta,
        theta=theta[k],
        omega=log["motor_velocity"][k],
        phi=phi[k],
        phi_rate=phi_rate,
        temperature=log["temperature"][k],
        u=u,
        alpha=alpha,
        truth=truth,
    )


def smoothing_kernel(cycle_hz: float, corner_hz: float = COMPARE_HZ) -> np.ndarray:
    """Hann window whose main lobe passes up to about ``corner_hz``; unit gain."""
    taps = int(2 * cycle_hz / corner_hz) | 1
    window = np.hanning(taps)
    return window / window.sum()


def smooth(x: np.ndarray, kernel: np.ndarray) -> np.ndarray:
    """Zero-phase smoothing; the edges where the window runs off the data are dropped."""
    return np.convolve(x, kernel, mode="valid")
