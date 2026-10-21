/**
 * Commissioning (B11, NX-13): one joint at a time runs its own excitation while the others
 * hold, and the host records what that joint publishes. This log is all a joint's learned
 * model is trained from.
 *
 * Next to it the harness records the ground truth of the actuator physics for the same
 * cycles. That truth is never given to training; it only feeds the Oracle baseline and
 * the evaluation (H10a).
 */
import { StatusFlags } from "../bus/protocol.ts";
import type { ArmSystem } from "../harness/armSystem.ts";
import { BUS_CYCLE_HZ } from "../index.ts";

/** Columns of a joint's own log: its published state, one row per bus cycle. */
export const LOG_COLUMNS = [
  "cycle",
  "motor_angle",
  "motor_velocity",
  "output_angle",
  "output_velocity",
  "gear_twist",
  "current",
  "torque_command",
  "temperature",
] as const;

/**
 * Columns of the ground truth, per bus cycle: what the actuator physics did over the
 * cycle that follows (torques output-referred, N·m) and the true mechanical state.
 */
export const TRUTH_COLUMNS = [
  "cycle",
  "drive_torque",
  "friction_torque",
  "gear_extra_torque",
  "motor_angle",
  "motor_velocity",
  "gear_twist",
] as const;

export interface CommissioningRun {
  /** Joint index, 0-based. */
  readonly joint: number;
  readonly seed: number;
  /** Rows of {@link LOG_COLUMNS}. */
  readonly log: number[][];
  /** Rows of {@link TRUTH_COLUMNS}, for the same cycles. */
  readonly truth: number[][];
}

/** Bus cycles allowed for the request to arrive and the first state to come back. */
const SETTLE_CYCLES = 200;

/** Runs joint `joint`'s excitation for `seconds` with `seed` and records it. */
export function commissionJoint(
  system: ArmSystem,
  joint: number,
  seconds: number,
  seed: number,
): CommissioningRun {
  const address = joint + 1;
  const runtime = system.joints[joint];
  if (!runtime) throw new RangeError(`joint ${joint} out of range`);
  system.host.commission(address, seconds, seed);
  const truthByCycle = new Map<number, number[]>();
  const log: number[][] = [];
  const total = Math.round(seconds * BUS_CYCLE_HZ) + SETTLE_CYCLES;
  let started = false;
  let finished = false;
  for (let k = 0; k < total && !finished; k++) {
    const cycle = system.cycle;
    system.step();
    const state = system.plant.actuatorState(joint);
    const out = runtime.actuatorOutput;
    truthByCycle.set(cycle, [
      cycle,
      out.driveTorque,
      out.frictionTorque,
      out.gearTorque,
      state.motorAngle,
      state.motorVelocity,
      state.gearTwist,
    ]);
    for (const frame of system.host.lastReceived) {
      if (!("state" in frame) || frame.header.source !== address) continue;
      const s = frame.state;
      const running = (s.status & StatusFlags.commissioning) !== 0;
      if (!running) {
        finished = started;
        continue;
      }
      started = true;
      log.push([
        frame.header.cycle,
        s.motorAngle,
        s.motorVelocity,
        s.outputAngle,
        s.outputVelocity,
        s.gearTwist,
        s.current,
        s.torqueCommand,
        s.temperature,
      ]);
    }
  }
  const truth = log.map((row) => truthByCycle.get(row[0] ?? -1) ?? []);
  if (truth.some((row) => row.length === 0)) throw new Error("truth missing for a logged cycle");
  return { joint, seed, log, truth };
}

/**
 * Moves joint `joint` by `offset` rad over `seconds` with a smooth profile, the others
 * holding, then lets it settle for half a second. Used to commission at another pose.
 */
export function moveJoint(system: ArmSystem, joint: number, offset: number, seconds = 1.5): void {
  const start = system.joints.map(
    (j) => system.host.jointAngles.get(j.index + 1) ?? system.plant.outputAngle(j.index),
  );
  system.host.startJointMotion((t) => {
    const s = Math.min(1, t / seconds);
    const blend = s * s * s * (10 - 15 * s + 6 * s * s);
    return start.map((q, i) => (i === joint ? q + offset * blend : q));
  }, system.cycle);
  system.run(seconds + 0.1);
  system.host.stopJointMotion();
  system.run(0.5);
}

/** Formats rows as CSV with a header line. */
export function toCsv(columns: readonly string[], rows: readonly number[][]): string {
  const lines = [columns.join(",")];
  for (const row of rows)
    lines.push(row.map((x) => (Number.isInteger(x) ? String(x) : x.toPrecision(9))).join(","));
  return `${lines.join("\n")}\n`;
}
