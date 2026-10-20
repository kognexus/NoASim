/**
 * Impulse test of an arm's structure: the first mode in a given direction, with the
 * motors locked so no controller is involved.
 */
import type { ArmSpec, Vec3 } from "../plant/armSpec.ts";
import type { Mujoco } from "../plant/mujoco.ts";
import { Plant } from "../plant/plant.ts";
import { estimateMode, type ModeEstimate } from "./modal.ts";

export interface ImpulseTestOptions {
  /** Unit direction of the tap at the tool, world frame. */
  readonly direction: Vec3;
  /** Force of the tap, N, held for {@link pulseSeconds}. */
  readonly force?: number;
  readonly pulseSeconds?: number;
  /** Length of the recorded decay, s. */
  readonly recordSeconds?: number;
}

/**
 * Taps the tool of `spec` (motors locked, gravity off) and estimates the dominant mode of
 * the tool's motion along the tap direction.
 */
export function structuralMode(
  mujoco: Mujoco,
  spec: ArmSpec,
  options: ImpulseTestOptions,
): ModeEstimate {
  const { direction, force = 50, pulseSeconds = 0.002, recordSeconds = 3 } = options;
  const plant = Plant.create(mujoco, spec, { lockMotors: true, gravity: false });
  try {
    const dt = plant.timestep;
    const project = (p: Vec3) => p[0] * direction[0] + p[1] * direction[1] + p[2] * direction[2];
    plant.setToolForce([direction[0] * force, direction[1] * force, direction[2] * force]);
    for (let k = 0; k < Math.round(pulseSeconds / dt); k++) plant.step();
    plant.setToolForce([0, 0, 0]);
    const samples = new Float64Array(Math.round(recordSeconds / dt));
    for (let k = 0; k < samples.length; k++) {
      plant.step();
      samples[k] = project(plant.tcpPosition());
    }
    return estimateMode(samples, dt);
  } finally {
    plant.dispose();
  }
}
