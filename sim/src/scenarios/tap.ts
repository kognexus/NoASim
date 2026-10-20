/**
 * The tap test behind H1: settle the arm, tap the tool, measure how long the tool takes to
 * come back to rest.
 */
import { rms, settlingTime } from "../analysis/metrics.ts";
import type { ArmSystem } from "../harness/armSystem.ts";
import { BUS_CYCLE_HZ } from "../index.ts";
import type { Vec3 } from "../plant/armSpec.ts";

export interface TapOptions {
  /** Unit direction of the tap, world frame. */
  readonly direction?: Vec3;
  /** Tap force, N. */
  readonly force?: number;
  /** Tap duration, s. */
  readonly pulseSeconds?: number;
  /** Time to settle before the tap, s (discovery, payload estimate). */
  readonly settleSeconds?: number;
  /** Length of the record after the tap, s. */
  readonly recordSeconds?: number;
  /** Called once the arm has settled, before the tap (e.g. to switch modes). */
  readonly beforeTap?: (system: ArmSystem) => void;
}

export interface TapResult {
  /** Largest tool deviation from rest after the tap, m. */
  readonly peak: number;
  /** RMS tool jitter in the half second before the tap (sensor noise through the loop), m. */
  readonly noise: number;
  /**
   * Time until the tool stays within 5 % of the peak, or within three times the pre-tap
   * jitter if that is larger, s.
   */
  readonly settlingTime: number;
  /** Tool deviation from rest, one sample per bus cycle. */
  readonly deviation: Float64Array;
}

const QUIET_SECONDS = 0.5;

export function tapTest(system: ArmSystem, options: TapOptions = {}): TapResult {
  const {
    direction = [0, 0, 1],
    force = 200,
    pulseSeconds = 0.01,
    settleSeconds = 5,
    recordSeconds = 4,
  } = options;
  system.run(settleSeconds);
  options.beforeTap?.(system);
  system.run(0.3);

  // Rest position and jitter, from half a second of undisturbed holding.
  const quiet: Vec3[] = [];
  for (let k = 0; k < Math.round(QUIET_SECONDS * BUS_CYCLE_HZ); k++) {
    system.step();
    quiet.push(system.plant.tcpPosition());
  }
  const mean = (axis: 0 | 1 | 2) => quiet.reduce((sum, p) => sum + p[axis], 0) / quiet.length;
  const rest: Vec3 = [mean(0), mean(1), mean(2)];
  const distance = (p: Vec3) => Math.hypot(p[0] - rest[0], p[1] - rest[1], p[2] - rest[2]);
  const noise = rms(quiet.map(distance));

  system.plant.setToolForce([direction[0] * force, direction[1] * force, direction[2] * force]);
  system.run(pulseSeconds);
  system.plant.setToolForce([0, 0, 0]);

  const deviation = new Float64Array(Math.round(recordSeconds * BUS_CYCLE_HZ));
  for (let k = 0; k < deviation.length; k++) {
    system.step();
    deviation[k] = distance(system.plant.tcpPosition());
  }
  let peak = 0;
  for (const d of deviation) peak = Math.max(peak, d);
  const dt = 1 / BUS_CYCLE_HZ;
  return { peak, noise, settlingTime: settlingTime(deviation, dt, 0.05, 3 * noise), deviation };
}
