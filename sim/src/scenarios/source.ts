/**
 * Report-to-source scenarios behind H5 and H7: inject an angle-locked torque ripple into
 * one joint, then watch which joint claims it, whether its correction removes the vibration
 * the neighbours feel, and when it faults.
 *
 * A *claim* is a disturbance report in which a joint names itself; a *felt* report is one
 * in which a joint names a neighbour (see `source.rs`).
 */
import { DampingMode, FaultCode } from "../bus/protocol.ts";
import { cogging } from "../actuators/effects.ts";
import type { ArmSystem } from "../harness/armSystem.ts";
import { BUS_CYCLE_HZ } from "../index.ts";
import type { JointSpec } from "../plant/armSpec.ts";

/** Order of the torque ripple at the sixth electrical harmonic, cycles per output turn. */
export const rippleOrder = (joint: JointSpec): number =>
  6 * joint.actuator.electrical.polePairs * joint.actuator.gearRatio;

/** Order of the gear mesh (four per motor turn), cycles per output turn. */
export const meshOrder = (joint: JointSpec): number => 4 * joint.actuator.gearRatio;

/**
 * Smooth back-and-forth strokes: joint *i* turns at `speeds[i]` rad/s, reversing every
 * `strokeSeconds` with a sine-shaped ramp of `rampSeconds`. Returns angle offsets (rad)
 * as a function of time.
 */
export function strokes(
  speeds: readonly number[],
  strokeSeconds = 1.2,
  rampSeconds = 0.3,
): (t: number) => number[] {
  const T = strokeSeconds;
  const ramp = (x: number) => Math.sin((Math.min(1, Math.max(0, x / rampSeconds)) * Math.PI) / 2);
  // Unit speed profile: +1 for a stroke, −1 for the next, with smooth ramps.
  const profile = (t: number) => {
    const phase = t % (2 * T);
    return phase < T ? ramp(phase) * ramp(T - phase) : -ramp(phase - T) * ramp(2 * T - phase);
  };
  // Integrated once, at 1 ms, for the first minute; starts mid-stroke at the middle.
  const step = 1e-3;
  const position = [0];
  for (let k = 1; k <= 60 / step; k++) {
    position.push((position[k - 1] ?? 0) + profile((k - 0.5) * step + T / 2) * step);
  }
  const at = (t: number) => position[Math.min(position.length - 1, Math.round(t / step))] ?? 0;
  return (t) => speeds.map((v) => v * at(t));
}

/** One joint turning one way at `speed` rad/s from `start` rad (offset), after a 0.5 s ramp. */
export function sweep(
  joints: number,
  joint: number,
  speed: number,
  start: number,
): (t: number) => number[] {
  const rampSeconds = 0.5;
  const offset = (t: number) =>
    start + (t < rampSeconds ? (speed * t * t) / (2 * rampSeconds) : speed * (t - rampSeconds / 2));
  return (t) => Array.from({ length: joints }, (_, i) => (i === joint ? offset(t) : 0));
}

export interface RippleOptions {
  /** Joint index (0-based) given the ripple. */
  readonly source: number;
  /** Cycles per output turn. */
  readonly order: number;
  /** Ripple amplitude, N·m. */
  readonly amplitude: number;
  /** Damping mode for the run. */
  readonly mode: DampingMode;
  /** Angle offsets from the start pose over time (see {@link strokes}, {@link sweep}). */
  readonly motion: (t: number) => number[];
  /** Seconds of motion before the ripple appears. */
  readonly leadSeconds?: number;
  /** Seconds recorded after the ripple appears. */
  readonly recordSeconds: number;
}

export interface RippleRun {
  /** Joints (1-based addresses) that claimed the ripple's order. */
  readonly claimants: number[];
  /** Seconds from the ripple appearing to the first claim by the source. */
  readonly firstClaim: number | undefined;
  /** Seconds from the ripple appearing to the source's fault, if any. */
  readonly fault: number | undefined;
  /** Window length of the effort series, s. */
  readonly windowSeconds: number;
  /**
   * Per window: the largest network torque of the source's neighbours locked to the
   * source's angle at the order, N·m ("helper effort"). NaN while the source turns too
   * slowly for the order to be measured.
   */
  readonly helperEffort: number[];
  /** Per window: the source's gear torque locked to its angle at the order, N·m. */
  readonly sourceTorque: number[];
}

/** Runs the arm along `motion`, gives joint `source` a ripple, and records the outcome. */
export function rippleRun(system: ArmSystem, options: RippleOptions): RippleRun {
  const { source, order } = options;
  const start = system.plant.spec.joints.map(
    (j, i) => system.host.jointAngles.get(i + 1) ?? j.initialAngle,
  );
  system.host.applySettings({ damping: options.mode });
  system.host.startJointMotion(
    (t) => options.motion(t).map((offset, i) => (start[i] ?? 0) + offset),
    system.cycle,
  );
  system.run(options.leadSeconds ?? 0.5);
  const reports = system.host.disturbances.length;
  const faults = system.host.faults.length;
  const injected = system.cycle;
  system.injectDisturbance(
    source,
    cogging([{ cycles: order, amplitude: options.amplitude, phase: 0.3 }]),
  );

  const stiffness = system.plant.spec.joints[source]?.actuator.gearStiffness ?? 0;
  const neighbours = [source, source + 2].filter((a) => a >= 1 && a <= system.joints.length);
  const window = Math.round(0.25 * BUS_CYCLE_HZ);
  const helperEffort: number[] = [];
  const sourceTorque: number[] = [];
  // In-phase and quadrature sums: the source's gear torque, then each neighbour's.
  let sums = Array.from({ length: 1 + neighbours.length }, () => [0, 0]);
  let counted = 0;
  const minRate = (2 * Math.PI * 4) / order; // order frequency of at least 4 Hz
  for (let c = 0; c < Math.round(options.recordSeconds * BUS_CYCLE_HZ); c++) {
    system.step();
    const states = new Map<
      number,
      { motorAngle: number; motorVelocity: number; gearTwist: number; networkTorque: number }
    >();
    for (const frame of system.host.lastReceived) {
      if ("state" in frame) states.set(frame.header.source, frame.state);
    }
    const own = states.get(source + 1);
    if (own && Math.abs(own.motorVelocity) > minRate) {
      counted++;
      const phase = order * own.motorAngle;
      const values = [
        own.gearTwist * stiffness,
        ...neighbours.map((a) => states.get(a)?.networkTorque ?? 0),
      ];
      values.forEach((x, k) => {
        const sum = sums[k];
        if (sum) {
          sum[0] = (sum[0] ?? 0) + x * Math.cos(phase);
          sum[1] = (sum[1] ?? 0) + x * Math.sin(phase);
        }
      });
    }
    if ((c + 1) % window === 0) {
      const amplitude = (k: number) => {
        const sum = sums[k] ?? [0, 0];
        return counted > window / 2
          ? (2 * Math.hypot(sum[0] ?? 0, sum[1] ?? 0)) / counted
          : Number.NaN;
      };
      sourceTorque.push(amplitude(0));
      helperEffort.push(Math.max(...neighbours.map((_, k) => amplitude(k + 1))));
      sums = sums.map(() => [0, 0]);
      counted = 0;
    }
  }

  const sameOrder = (o: number) => Math.abs(o - order) < 1e-3;
  const claims = system.host.disturbances
    .slice(reports)
    .filter((d) => d.node === d.source && sameOrder(d.order));
  const firstClaim = claims.find((d) => d.node === source + 1);
  const fault = system.host.faults
    .slice(faults)
    .find((f) => f.code === FaultCode.source && f.node === source + 1);
  const seconds = (cycle: number) => (cycle - injected) / BUS_CYCLE_HZ;
  return {
    claimants: [...new Set(claims.map((d) => d.node))].sort((a, b) => a - b),
    firstClaim: firstClaim && seconds(firstClaim.cycle),
    fault: fault && seconds(fault.cycle),
    windowSeconds: window / BUS_CYCLE_HZ,
    helperEffort,
    sourceTorque,
  };
}

export { DampingMode };
