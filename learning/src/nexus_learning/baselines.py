"""The models the PHNN competes with (H10a), as residuals ``I·α − ṗ`` per sample.

* **Nominal**: the datasheet. Viscous plus Coulomb friction, the linear gear spring, the
  nominal inertia; no cogging, no gear damping, no torque-constant drift.
* **Fitted**: the node's RLS structure (viscous, Coulomb, offset), fitted by least squares
  on the training run. It can only represent that structure.
* **Oracle**: the hidden truth. The actuator's real drive, friction and gear torques and
  the plant's gear damping, seen through the same sensors. It is an upper bound only and
  never available to a joint.
"""

from __future__ import annotations

import numpy as np

from .data import COMPARE_HZ, Run, smooth, smoothing_kernel

#: Speed scale of the node's smooth sign, rad/s (``dynamics.rs``).
STICTION_SPEED = 0.01


def _known(run: Run) -> np.ndarray:
    """``I·α − u − k·φ``: what the friction-and-offset terms must explain."""
    m = run.meta
    return m.reflected_inertia * run.alpha - run.u - m.gear_stiffness * run.phi


def _regressors(run: Run) -> np.ndarray:
    """Columns of the textbook structure: viscous, Coulomb, offset (as friction terms)."""
    return np.stack(
        [run.omega, np.tanh(run.omega / STICTION_SPEED), np.ones_like(run.omega)], axis=1
    )


def nominal_residual(run: Run) -> np.ndarray:
    """Datasheet model."""
    m = run.meta
    friction = m.viscous_friction * run.omega + m.coulomb_friction * np.tanh(
        run.omega / STICTION_SPEED
    )
    return _known(run) + friction


class FittedModel:
    """Least-squares fit of the RLS structure on smoothed signals (as the node filters)."""

    def __init__(self, train: Run) -> None:
        kernel = smoothing_kernel(train.meta.cycle_hz)
        x = np.stack([smooth(c, kernel) for c in _regressors(train).T], axis=1)
        y = -smooth(_known(train), kernel)
        self.parameters, *_ = np.linalg.lstsq(x, y, rcond=None)

    def residual(self, run: Run) -> np.ndarray:
        """Residual of the fitted model on ``run``."""
        return _known(run) + _regressors(run) @ self.parameters


def oracle_residual(run: Run, gear_damping: float) -> np.ndarray:
    """The true actuator physics on the measured signals."""
    if run.truth is None:
        raise ValueError("the Oracle needs the run's ground truth")
    m, t = run.meta, run.truth
    p_dot = (
        t["drive_torque"]
        + t["friction_torque"]
        + m.gear_stiffness * run.phi
        + gear_damping * run.phi_rate
        - t["gear_extra_torque"]
    )
    return m.reflected_inertia * run.alpha - p_dot


def score(residual: np.ndarray, cycle_hz: float, corner_hz: float = COMPARE_HZ) -> float:
    """RMS of the smoothed residual, N·m: the comparison metric for every model."""
    r = smooth(residual, smoothing_kernel(cycle_hz, corner_hz))
    return float(np.sqrt(np.mean(r**2)))
