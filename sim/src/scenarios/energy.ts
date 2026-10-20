/**
 * H6: energy the damping terms put into the arm. Each node publishes its damping torque;
 * multiplied by the joint's true output velocity and summed over time, that is the work
 * the term did. Passivity (B5) means it never rises above the tank's capacity.
 */
import type { ArmSystem } from "../harness/armSystem.ts";
import { BUS_CYCLE_HZ } from "../index.ts";

export interface EnergyRecord {
  /** Largest cumulative work by any joint's damping term at any moment, J. */
  readonly maxWork: number;
  /** Cumulative work per joint at the end, J (negative = energy removed). */
  readonly finalWork: number[];
}

/** Runs `seconds` and integrates the work of the published damping torques. */
export function dampingWork(system: ArmSystem, seconds: number): EnergyRecord {
  const dt = 1 / BUS_CYCLE_HZ;
  const torque = system.joints.map(() => 0);
  const work = system.joints.map(() => 0);
  let maxWork = 0;
  for (let k = 0; k < Math.round(seconds * BUS_CYCLE_HZ); k++) {
    system.step();
    for (const f of system.host.lastReceived)
      if ("state" in f) torque[f.header.source - 1] = f.state.networkTorque;
    system.joints.forEach((j, i) => {
      work[i] = (work[i] ?? 0) + (torque[i] ?? 0) * system.plant.outputVelocity(j.index) * dt;
      maxWork = Math.max(maxWork, work[i] ?? 0);
    });
  }
  return { maxWork, finalWork: work };
}
