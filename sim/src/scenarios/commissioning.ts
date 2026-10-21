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

/** The excitation's band and number of sines (`excitation.rs`), for interleaving joints. */
const EXCITATION_BAND_HZ = [0.2, 6] as const;
const EXCITATION_COMPONENTS = 8;

/** Columns of each joint in a neighbourhood log (NX-31): what it publishes every cycle. */
export const NEIGHBOURHOOD_COLUMNS = [
  "motor_angle",
  "motor_velocity",
  "output_angle",
  "output_velocity",
  "gear_twist",
  "current",
  "torque_command",
  "gyro_x",
  "gyro_y",
  "gyro_z",
] as const;

/** Columns of each joint in a neighbourhood log's ground truth (evaluation only). */
export const NEIGHBOURHOOD_TRUTH_COLUMNS = [
  "output_acceleration",
  "gear_twist",
  "link_bend",
] as const;

export interface NeighbourhoodRun {
  readonly seed: number;
  /** Column names: `cycle`, then `j<n>_<column>` for every joint. */
  readonly columns: readonly string[];
  /** One row per measured cycle, every joint's published state. */
  readonly log: number[][];
  readonly truthColumns: readonly string[];
  readonly truth: number[][];
}

/**
 * Neighbourhood commissioning (NX-31): every joint runs its own excitation at once, each
 * on its own seed, so each joint's neighbours move independently of it. The host records
 * every state frame; a joint's coordinated model is trained on its own columns and those
 * of the joints up to two places away, which is what it would receive on the bus.
 */
export function commissionNeighbourhood(
  system: ArmSystem,
  seconds: number,
  seed: number,
): NeighbourhoodRun {
  const n = system.joints.length;
  // Interleaved bands: each joint's frequencies sit between the others', so over the run
  // no joint's motion correlates with a neighbour's (with one band they all moved nearly
  // in phase, and a joint's model took its neighbours' inertia for its own).
  const spacing =
    (EXCITATION_BAND_HZ[1] / EXCITATION_BAND_HZ[0]) ** (1 / (EXCITATION_COMPONENTS - 1));
  for (let j = 0; j < n; j++) {
    system.host.commission(j + 1, seconds, seed * 16 + j, spacing ** (j / n));
  }
  const coordinates = system.joints.map((_, j) => system.plant.coordinates(j));
  const rows = new Map<number, (number[] | undefined)[]>();
  const truthByCycle = new Map<number, number[]>();
  const total = Math.round(seconds * BUS_CYCLE_HZ) + SETTLE_CYCLES;
  const running = new Array<boolean>(n).fill(false);
  let started = false;
  for (let k = 0; k < total; k++) {
    const cycle = system.cycle;
    system.step();
    const qacc = system.plant.acceleration();
    truthByCycle.set(
      cycle,
      coordinates.flatMap((c, j) => [
        (qacc[c.motor] ?? 0) + (qacc[c.gear] ?? 0),
        system.plant.gearTwist(j),
        // Bending about the hinge across the link's first axis (the joint's own axis for
        // the pitch links; see `bendingAxes`), 0 for a rigid link.
        system.plant.linkBend(j)[0] ?? 0,
      ]),
    );
    for (const frame of system.host.lastReceived) {
      if (!("state" in frame)) continue;
      const j = frame.header.source - 1;
      const s = frame.state;
      running[j] = (s.status & StatusFlags.commissioning) !== 0;
      if (!running[j]) continue;
      started = true;
      const row = rows.get(frame.header.cycle) ?? new Array<number[] | undefined>(n);
      row[j] = [
        s.motorAngle,
        s.motorVelocity,
        s.outputAngle,
        s.outputVelocity,
        s.gearTwist,
        s.current,
        s.torqueCommand,
        ...s.gyro,
      ];
      rows.set(frame.header.cycle, row);
    }
    if (started && running.every((r) => !r)) break;
  }
  const columns = [
    "cycle",
    ...Array.from({ length: n }, (_, j) =>
      NEIGHBOURHOOD_COLUMNS.map((c) => `j${j + 1}_${c}`),
    ).flat(),
  ];
  const truthColumns = [
    "cycle",
    ...Array.from({ length: n }, (_, j) =>
      NEIGHBOURHOOD_TRUTH_COLUMNS.map((c) => `j${j + 1}_${c}`),
    ).flat(),
  ];
  const log: number[][] = [];
  const truth: number[][] = [];
  for (const [cycle, row] of [...rows.entries()].sort((a, b) => a[0] - b[0])) {
    // Only cycles where every joint was in its excitation and its frame arrived.
    if (row.some((r) => r === undefined)) continue;
    const t = truthByCycle.get(cycle);
    if (!t) continue;
    log.push([cycle, ...row.flatMap((r) => r ?? [])]);
    truth.push([cycle, ...t]);
  }
  return { seed, columns, log, truthColumns, truth };
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
