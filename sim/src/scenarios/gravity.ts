/**
 * Gravity scenarios behind H9: hold the arm on a tilted or moving base and see how well
 * each joint's gravity feed-forward keeps the tool in place.
 */
import type { ArmSystem } from "../harness/armSystem.ts";
import { BUS_CYCLE_HZ } from "../index.ts";
import type { Vec3 } from "../plant/armSpec.ts";

/**
 * Gravity vector for a base tilted sideways by `degrees` (about the world x axis, the
 * direction the KX-6 reaches in), as on a mobile base on a slope. A tilt within the arm's
 * own plane would only scale gravity, which a payload estimate can absorb; a sideways tilt
 * loads the base joint in a way no payload explains.
 */
export function tiltedGravity(degrees: number, g = 9.81): Vec3 {
  const a = (degrees * Math.PI) / 180;
  return [0, g * Math.sin(a), -g * Math.cos(a)];
}

export interface HoldResult {
  /** Tool position at the end, m. */
  readonly tool: Vec3;
  /** Each joint's published gravity feed-forward minus the true holding torque, N·m. */
  readonly torqueError: number[];
}

/** Holds the arm for `seconds` and reports the tool position and feed-forward errors. */
export function hold(system: ArmSystem, seconds: number): HoldResult {
  const feedforward = new Map<number, number>();
  for (let k = 0; k < Math.round(seconds * BUS_CYCLE_HZ); k++) {
    system.step();
    for (const f of system.host.lastReceived)
      if ("chain" in f) feedforward.set(f.header.source, f.chain.holdingTorque);
  }
  return {
    tool: system.plant.tcpPosition(),
    torqueError: system.joints.map(
      (j) => (feedforward.get(j.index + 1) ?? 0) - system.plant.holdingTorque(j.index),
    ),
  };
}

/**
 * Accelerates the base sinusoidally sideways (world y; amplitude `acceleration` m/s²,
 * period `period` s) for `seconds`, emulated by adding the opposite acceleration to
 * gravity, and returns the RMS tool deviation from where it started, m.
 */
export function movingBase(
  system: ArmSystem,
  acceleration: number,
  period: number,
  seconds: number,
): number {
  const start = system.plant.tcpPosition();
  const g = system.plant.gravity();
  let sum = 0;
  const cycles = Math.round(seconds * BUS_CYCLE_HZ);
  for (let k = 0; k < cycles; k++) {
    const a = acceleration * Math.sin((2 * Math.PI * k) / (BUS_CYCLE_HZ * period));
    system.plant.setGravity([g[0], g[1] - a, g[2]]);
    system.step();
    const p = system.plant.tcpPosition();
    sum += (p[0] - start[0]) ** 2 + (p[1] - start[1]) ** 2 + (p[2] - start[2]) ** 2;
  }
  system.plant.setGravity(g);
  return Math.sqrt(sum / cycles);
}
