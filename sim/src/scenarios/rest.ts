/**
 * Quietness at rest: how hard the actuator housings shake while the arm holds still.
 * A housing at rest reads 1 g; rotor–gear ringing shows up as much more (ADR-0003).
 */
import type { ArmSystem } from "../harness/armSystem.ts";
import { BUS_CYCLE_HZ } from "../index.ts";

/** RMS magnitude of the housing accelerometers over `seconds`, m/s², worst joint. */
export function housingShake(system: ArmSystem, seconds: number): number {
  const sums = system.joints.map(() => 0);
  const cycles = Math.round(seconds * BUS_CYCLE_HZ);
  for (let k = 0; k < cycles; k++) {
    system.step();
    system.joints.forEach(
      (j, i) =>
        (sums[i] = (sums[i] ?? 0) + Math.hypot(...system.plant.accelerometer(j.index)) ** 2),
    );
  }
  return Math.max(...sums.map((s) => Math.sqrt(s / cycles)));
}
