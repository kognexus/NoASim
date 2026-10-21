/**
 * Damping between neighbouring joints (NX-28), from a colleague's derivation: each pair
 * of neighbours damps the difference of their output velocities,
 *
 * ```text
 * τ_i = −Σ_{j neighbour of i} c_ij · (ω_i − ω_j),   c_ij = κ·√(c_i c_j)
 * ```
 *
 * with c_i each joint's network-damping gain. With both joints acting on the same,
 * current data, the pair can only take energy out: the power is −c_ij (ω_i − ω_j)².
 * Here it is an experiment, run outside the joints on true velocities with no delay, on
 * top of the joints' own network damping, held to their budget (a share of peak torque,
 * an energy tank per joint).
 */
import type { ArmSystem } from "../harness/armSystem.ts";
import type { NodeConfig } from "../node/nodeConfig.ts";

export function attachNeighbourCoupling(
  system: ArmSystem,
  configs: readonly NodeConfig[],
  kappa: number,
  torqueShare: number,
  tankJ: number,
): void {
  const tank = configs.map(() => 0);
  const dt = system.plant.timestep;
  system.outsideTorque = (s) => {
    const rates = configs.map((_, i) => s.plant.outputVelocity(i));
    return configs.map((c, i) => {
      let u = 0;
      for (const j of [i - 1, i + 1]) {
        const other = configs[j];
        if (!other) continue;
        u -=
          kappa *
          Math.sqrt(c.networkDamping * other.networkDamping) *
          ((rates[i] ?? 0) - (rates[j] ?? 0));
      }
      const cap = torqueShare * c.peakTorque;
      u = Math.max(-cap, Math.min(cap, u));
      const work = u * (rates[i] ?? 0) * dt;
      const energy = tank[i] ?? 0;
      if (work <= 0) tank[i] = Math.min(tankJ, energy - work);
      else if (work <= energy) tank[i] = energy - work;
      else u = 0;
      return u;
    });
  };
}
