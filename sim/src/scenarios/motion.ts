/**
 * The motion scenario behind H2: follow a policy-driven task and measure how smooth and
 * accurate the tool's path is.
 */
import { rms } from "../analysis/metrics.ts";
import type { ArmSystem } from "../harness/armSystem.ts";
import type { TaskOptions } from "../host/host.ts";
import { flangePose } from "../host/ik.ts";
import type { Task } from "../host/tasks.ts";
import { BUS_CYCLE_HZ } from "../index.ts";
import type { Vec3 } from "../plant/armSpec.ts";

export interface MotionResult {
  /** RMS of the tool's jerk (third derivative of position), m/s³. */
  readonly jerkRms: number;
  /** RMS distance from the task's tool path at the same time, m. */
  readonly pathErrorRms: number;
  /** Largest distance from the task's tool path, m. */
  readonly pathErrorMax: number;
  /** Policy steps that never arrived. */
  readonly droppedSteps: number;
}

/** Jerk is taken from tool velocity filtered at this corner (above any intended motion), Hz. */
const JERK_FILTER_HZ = 50;

/** Runs `task` from the arm's current pose; the arm should be settled at the task's home. */
export function followTask(
  system: ArmSystem,
  makeTask: (system: ArmSystem) => Task,
  options: TaskOptions,
): MotionResult {
  const task = makeTask(system);
  const start = system.cycle;
  const motion = system.host.startTask(task, start, options);
  const dt = 1 / BUS_CYCLE_HZ;
  const alpha = dt / (1 / (2 * Math.PI * JERK_FILTER_HZ) + dt);
  const lowpass = (prev: Vec3, next: Vec3): Vec3 => [
    prev[0] + alpha * (next[0] - prev[0]),
    prev[1] + alpha * (next[1] - prev[1]),
    prev[2] + alpha * (next[2] - prev[2]),
  ];
  let velocity: Vec3 = system.plant.tcpVelocity();
  let acceleration: Vec3 = [0, 0, 0];
  const jerk: number[] = [];
  const error: number[] = [];
  const cycles = Math.round((task.duration + 0.5) * BUS_CYCLE_HZ);
  for (let k = 0; k < cycles; k++) {
    system.step();
    const v = lowpass(velocity, system.plant.tcpVelocity());
    const a = lowpass(acceleration, [
      (v[0] - velocity[0]) / dt,
      (v[1] - velocity[1]) / dt,
      (v[2] - velocity[2]) / dt,
    ]);
    if (k > 0.2 * BUS_CYCLE_HZ) {
      jerk.push(
        Math.hypot(
          (a[0] - acceleration[0]) / dt,
          (a[1] - acceleration[1]) / dt,
          (a[2] - acceleration[2]) / dt,
        ),
      );
    }
    velocity = v;
    acceleration = a;
    const desired = task.pose(Math.min(task.duration, (system.cycle - start) * dt)).position;
    const p = system.plant.tcpPosition();
    error.push(Math.hypot(p[0] - desired[0], p[1] - desired[1], p[2] - desired[2]));
  }
  return {
    jerkRms: rms(jerk),
    pathErrorRms: rms(error),
    pathErrorMax: Math.max(...error),
    droppedSteps: motion.vla?.dropped ?? 0,
  };
}

/** The task's home pose: where the flange is at the spec's initial angles. */
export function homePose(system: ArmSystem) {
  return flangePose(
    system.plant.spec,
    system.plant.spec.joints.map((j) => j.initialAngle),
  );
}
