/**
 * Commissioned impact thresholds per arm (NX-16): twice the largest external-torque
 * estimate each joint saw in normal tasks with joint-level interpolation (3 seeds × 4
 * tasks), at least 30 % of rated torque. From `experiments/src/calibrate-impact.ts`; the
 * central-interpolation runs stayed below these thresholds as well.
 */
const THRESHOLDS: Record<string, readonly number[]> = {
  "KX-6": [53, 56, 85, 86, 3, 29],
};

/** Commissioned thresholds for the arm named `name`, if it has been commissioned. */
export function impactThresholds(name: string): readonly number[] | undefined {
  return THRESHOLDS[name];
}
