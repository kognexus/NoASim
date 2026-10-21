"""A joint's port-Hamiltonian model of itself (B11).

State ``x = (θ, p, φ)``: the motor angle, the rotor momentum ``p = I·ω`` and the gear twist
``φ`` (output minus motor angle), all output-referred. The energy is

    H(x) = p²/(2I) + V_cog(θ) + V_gear(φ)

and the dynamics are port-Hamiltonian:

    θ̇ = ∂H/∂p
    ṗ = −∂H/∂θ + ∂H/∂φ − R(θ, ω)·ω + c·φ̇ + g(T, θ)·u
    φ̇ = ω_out − ∂H/∂p                          (ω_out: the output side, a port)

* ``V_cog`` is a cogging potential over Fourier features at the joint's announced orders
  from the gear mesh up (and their second harmonics): what the unit knows about its own
  geometry, not its physics. Lower orders (once per motor or output turn) are left out: a
  commissioning run covers less than one of their periods, so they could fit any smooth
  function of position, which is memory, not a model.
* ``V_gear`` is the nominal spring ½kφ² plus a learned correction (stiffening, play).
* ``R(ω) ≥ 0`` (a viscous-like and a Coulomb-like part, both through softplus) and the
  gear damping ``c ≥ 0`` are the losses. Friction depends on speed only: friction that
  varies once per turn cannot be told apart from anything else that varies with position
  over the third of a period one commissioning run covers. The skew interconnection is
  the canonical one.
* ``g(T, θ) = exp(…) > 0`` is the torque constant's drift with temperature and its ripple
  with angle, within ±20 %.
* The rotor inertia is learned within ×0.5…2 of the datasheet value: it is a geometric
  quantity the unit knows roughly, and leaving it free lets it absorb other effects.

Whatever the weights, the model's power balance is ``Ḣ = ω·g·u + port − R·ω² − c·φ̇²``,
so the model cannot create energy (H10c).
"""

from __future__ import annotations

import math

import torch
from torch import nn

from .data import JointMeta

#: Speed scale of the friction inputs, rad/s.
OMEGA_REF = 2.0
#: Speed below which Coulomb friction fades out, rad/s (keeps R finite at rest).
STICTION_SPEED = 0.01
#: Temperature offset and scale of the gain input, °C.
TEMPERATURE_REF = (25.0, 50.0)
#: The learned inertia stays within this factor of the datasheet value.
INERTIA_RANGE = 2.0


def cogging_orders(meta: JointMeta) -> list[float]:
    """Announced orders from the mesh up, once per electrical turn, and their doubles.

    The mesh is taken as four per motor turn. The electrical order is where errors in the
    phase-current sensing act (offset once, gain mismatch twice per electrical turn;
    NX-26). The unit knows its pole pairs.
    """
    mesh = 4.0 * meta.gear_ratio
    electrical = float(meta.pole_pairs * meta.gear_ratio)
    base = {o for o in meta.fingerprint if o >= mesh} | {electrical}
    return sorted(base | {2.0 * o for o in base})


def mlp(inputs: int, hidden: int, outputs: int = 1) -> nn.Sequential:
    """Two tanh hidden layers."""
    return nn.Sequential(
        nn.Linear(inputs, hidden),
        nn.Tanh(),
        nn.Linear(hidden, hidden),
        nn.Tanh(),
        nn.Linear(hidden, outputs),
    )


class FourierFeatures(nn.Module):
    """``[sin(nθ), cos(nθ)]`` for a fixed set of orders ``n``."""

    def __init__(self, orders: list[float]) -> None:
        super().__init__()
        self.register_buffer("orders", torch.tensor(orders, dtype=torch.float64))

    def forward(self, theta: torch.Tensor) -> torch.Tensor:
        """Features of shape (N, 2·len(orders))."""
        angle = theta[:, None] * self.orders[None, :]
        return torch.cat([torch.sin(angle), torch.cos(angle)], dim=1)


class JointPHNN(nn.Module):
    """The port-Hamiltonian model of one joint, scaled by what the joint knows of itself."""

    def __init__(
        self, meta: JointMeta, hidden: int = 16, orders: list[float] | None = None
    ) -> None:
        super().__init__()
        self.meta = meta
        k = meta.gear_stiffness
        self.twist_ref = meta.rated_torque / k
        orders = cogging_orders(meta) if orders is None else orders
        self.cog_features = FourierFeatures(orders)
        self.cog = nn.Linear(2 * len(orders), 1, bias=False)
        nn.init.zeros_(self.cog.weight)
        self.gear = mlp(1, hidden)
        self.viscous = mlp(1, hidden)
        self.coulomb = mlp(1, hidden)
        electrical = 6.0 * meta.pole_pairs * meta.gear_ratio
        self.gain_features = FourierFeatures([electrical, 2.0 * electrical])
        self.gain_angle = nn.Linear(4, 1, bias=False)
        nn.init.zeros_(self.gain_angle.weight)
        self.gain_temperature = mlp(1, 8)
        self.raw_inertia = nn.Parameter(torch.tensor(0.0))
        self.raw_gear_damping = nn.Parameter(torch.tensor(-4.0))
        # Friction outputs are scaled by the datasheet values and start at them
        # (softplus(bias) = 1); every other output starts at zero.
        self.viscous_scale = meta.viscous_friction
        self.coulomb_scale = meta.coulomb_friction
        for net in (self.viscous, self.coulomb):
            last = net[-1]
            assert isinstance(last, nn.Linear)
            nn.init.zeros_(last.weight)
            nn.init.constant_(last.bias, math.log(math.expm1(1.0)))
        for net in (self.gear, self.gain_temperature):
            last = net[-1]
            assert isinstance(last, nn.Linear)
            nn.init.zeros_(last.weight)
            nn.init.zeros_(last.bias)
        self.double()

    # -- parts ---------------------------------------------------------------------------

    @property
    def inertia(self) -> torch.Tensor:
        """Rotor inertia, output-referred, kg·m²."""
        spread = math.log(INERTIA_RANGE) * torch.tanh(self.raw_inertia)
        return self.meta.reflected_inertia * torch.exp(spread)

    @property
    def gear_damping(self) -> torch.Tensor:
        """Gear damping c ≥ 0, N·m·s/rad (scaled by the gear stiffness)."""
        return nn.functional.softplus(self.raw_gear_damping) * 1e-3 * self.meta.gear_stiffness

    def potential(self, theta: torch.Tensor, phi: torch.Tensor) -> torch.Tensor:
        """``V_cog(θ) + V_gear(φ)``, J, per sample."""
        rated = self.meta.rated_torque
        cogging = rated * self.cog(self.cog_features(theta))[:, 0]
        x = phi / self.twist_ref
        spring = 0.5 * self.meta.gear_stiffness * phi**2
        correction = rated * self.twist_ref * self.gear(x[:, None])[:, 0]
        return cogging + spring + correction

    def damping(self, omega: torch.Tensor) -> torch.Tensor:
        """``R(ω) ≥ 0``, N·m·s/rad, per sample."""
        inputs = (omega / OMEGA_REF)[:, None]
        viscous = nn.functional.softplus(self.viscous(inputs)[:, 0]) * self.viscous_scale
        coulomb = nn.functional.softplus(self.coulomb(inputs)[:, 0]) * self.coulomb_scale
        return viscous + coulomb / torch.sqrt(omega**2 + STICTION_SPEED**2)

    def gain(self, theta: torch.Tensor, temperature: torch.Tensor) -> torch.Tensor:
        """``g(T, θ) > 0``: torque produced per N·m commanded."""
        t = (temperature - TEMPERATURE_REF[0]) / TEMPERATURE_REF[1]
        log_gain = (
            self.gain_temperature(t[:, None])[:, 0]
            + self.gain_angle(self.gain_features(theta))[:, 0]
        )
        return torch.exp(0.2 * torch.tanh(log_gain))

    # -- dynamics ------------------------------------------------------------------------

    def forces(
        self,
        theta: torch.Tensor,
        omega: torch.Tensor,
        phi: torch.Tensor,
        phi_rate: torch.Tensor,
        temperature: torch.Tensor,
        u: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        """Every term of ``ṗ``, per sample (N·m), and the dissipated power (W)."""
        theta = theta.detach().requires_grad_(True)
        phi = phi.detach().requires_grad_(True)
        grad_theta, grad_phi = torch.autograd.grad(
            self.potential(theta, phi).sum(), (theta, phi), create_graph=True
        )
        r = self.damping(omega)
        c = self.gear_damping
        terms = {
            "drive": self.gain(theta, temperature) * u,
            "cogging": -grad_theta,
            "gear": grad_phi + c * phi_rate,
            "friction": -r * omega,
        }
        terms["total"] = terms["drive"] + terms["cogging"] + terms["gear"] + terms["friction"]
        terms["dissipation"] = r * omega**2 + c * phi_rate**2
        return terms

    def residual(self, run_tensors: dict[str, torch.Tensor]) -> torch.Tensor:
        """``I·α − ṗ`` per sample: zero for a perfect model, N·m."""
        f = self.forces(
            run_tensors["theta"],
            run_tensors["omega"],
            run_tensors["phi"],
            run_tensors["phi_rate"],
            run_tensors["temperature"],
            run_tensors["u"],
        )
        return self.inertia * run_tensors["alpha"] - f["total"]
