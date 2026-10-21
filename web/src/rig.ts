/**
 * The demo's simulation side: one {@link ArmRig} per arm, and a {@link TwinRig} that gives
 * both arms the same disturbances at the same bus cycle, so the only difference between
 * them is what the user switched (damping mode, model, …).
 */
import { cogging, type ActuatorEffect } from "../../sim/src/actuators/effects.ts";
import type { BusOptions } from "../../sim/src/bus/ethercatBus.ts";
import { DampingMode, ImpactPolicy } from "../../sim/src/bus/protocol.ts";
import { ArmSystem } from "../../sim/src/harness/armSystem.ts";
import type { ControlGains } from "../../sim/src/harness/gains.ts";
import { tunedGains } from "../../sim/src/harness/tuned.ts";
import type { TaskOptions } from "../../sim/src/host/host.ts";
import type { Task } from "../../sim/src/host/tasks.ts";
import { figureEight, pickAndPlace } from "../../sim/src/host/tasks.ts";
import { BUS_CYCLE_HZ } from "../../sim/src/index.ts";
import type { PhnnDocument } from "../../sim/src/node/phnnModel.ts";
import type { ArmSpec, Vec3 } from "../../sim/src/plant/armSpec.ts";
import { KX6, KX6_HOME, KX6L, withInitialAngles } from "../../sim/src/plant/kx6.ts";
import type { Mujoco } from "../../sim/src/plant/mujoco.ts";
import { homePose } from "../../sim/src/scenarios/motion.ts";
import { rippleOrder } from "../../sim/src/scenarios/source.ts";

export type ArmName = "kx6" | "kx6l";
export type TaskName = "hold" | "pick" | "eight";

/**
 * Which computer drives the tasks: a VLA policy computer (15 Hz goals with latency and
 * jitter; the joints interpolate) or a classic real-time controller (the program's path,
 * streamed as setpoints at 1 kHz). See `InterpolationMode` in `sim/src/host/host.ts`.
 */
export type RobotComputer = "vla" | "classic";

const TASK_OPTIONS: Record<RobotComputer, TaskOptions> = {
  vla: { mode: "joint-level" },
  classic: { mode: "classic", classic: { loopHz: 1000 } },
};

/** What both arms share; changing it rebuilds them. */
export interface SharedSettings {
  readonly arm: ArmName;
  /** Datasheet actuators, or a committed unit's hidden actuators (and its learned models). */
  readonly unit: "datasheet" | string;
  readonly brakes: boolean;
  readonly impactPolicy: number;
  /** Whether the joints react to impacts at all (a tap is an impact to them). */
  readonly reactToImpacts: boolean;
  /** Mass at the tool, kg. */
  readonly payload: number;
  /** Factor on every link's bending stiffness (arm editor). */
  readonly linkStiffness: number;
}

/** What each arm has on its own; applied live. */
export interface SideSettings {
  readonly damping: DampingMode;
  /** 0 nominal, 1 fitted, 2 learned. */
  readonly model: number;
}

export const DEFAULT_SHARED: SharedSettings = {
  arm: "kx6l",
  unit: "datasheet",
  brakes: false,
  impactPolicy: ImpactPolicy.retreatAndHold,
  reactToImpacts: false,
  payload: 0,
  linkStiffness: 1,
};

/** Seed of a unit name such as `kx6-s300`. */
const unitSeed = (unit: string) => Number(unit.split("-s").pop());

/** Both damping laws as tuned, so the mode can be switched live by the settings byte. */
function demoGains(arm: ArmName): ControlGains {
  const local = tunedGains(arm, "local");
  return { ...tunedGains(arm, "network"), localDampingRatio: local.localDampingRatio };
}

/** The arm the shared settings describe: body, brakes, payload and link stiffness. */
function armSpec(settings: SharedSettings): ArmSpec {
  const base = withInitialAngles(settings.arm === "kx6" ? KX6 : KX6L, KX6_HOME);
  const k = settings.linkStiffness;
  return {
    ...base,
    payload: { ...base.payload, mass: settings.payload },
    joints: base.joints.map((j) => ({
      ...j,
      actuator: { ...j.actuator, brake: j.actuator.brake || settings.brakes },
      link: {
        ...j.link,
        ...(j.link.flex
          ? {
              flex: {
                stiffness: j.link.flex.stiffness * k,
                damping: j.link.flex.damping * Math.sqrt(k),
              },
            }
          : {}),
      },
    })),
  };
}

/** A force applied to the tool until a given cycle. */
interface TimedForce {
  readonly until: number;
  readonly force: (cycle: number) => Vec3;
}

/** One arm of the demo. */
export class ArmRig {
  readonly system: ArmSystem;
  readonly spec: ArmSpec;
  #forces: TimedForce[] = [];
  #task: { readonly name: TaskName; task: Task | undefined; start: number } = {
    name: "hold",
    task: undefined,
    start: 0,
  };
  #rest: Vec3;
  #computer: RobotComputer = "vla";

  private constructor(system: ArmSystem, spec: ArmSpec) {
    this.system = system;
    this.spec = spec;
    this.#rest = system.plant.tcpPosition();
  }

  static async create(
    mujoco: Mujoco,
    nodeModule: WebAssembly.Module,
    shared: SharedSettings,
    side: SideSettings,
    models: readonly PhnnDocument[] | undefined,
    bus: BusOptions,
  ): Promise<ArmRig> {
    const spec = armSpec(shared);
    const hidden = shared.unit !== "datasheet";
    const system = await ArmSystem.create(mujoco, nodeModule, {
      spec,
      seed: hidden ? unitSeed(shared.unit) : 1,
      gains: demoGains(shared.arm),
      bus,
      ...(hidden ? { actuators: {}, ...(models ? { models } : {}) } : {}),
    });
    system.run(2);
    system.host.applySettings({
      impactPolicy: shared.impactPolicy,
      impactConfirm: shared.reactToImpacts ? 1 : 0,
    });
    const rig = new ArmRig(system, spec);
    rig.apply(side);
    system.run(0.5);
    rig.#rest = system.plant.tcpPosition();
    return rig;
  }

  /** Switches damping mode and model, from the next cycle. */
  apply(side: SideSettings): void {
    this.system.host.applySettings({ damping: side.damping, model: side.model });
  }

  get cycle(): number {
    return this.system.cycle;
  }

  /** Where the tool should be now: at rest, or on the running task's path. */
  target(): Vec3 {
    const { task, start } = this.#task;
    if (!task) return this.#rest;
    const t = Math.min(task.duration, (this.cycle - start) / BUS_CYCLE_HZ);
    return task.pose(t).position;
  }

  /** Distance of the tool from {@link target}, m. */
  deviation(): number {
    const p = this.system.plant.tcpPosition();
    const q = this.target();
    return Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
  }

  /** Runs `cycles` bus cycles, applying timed forces and restarting the task when it ends. */
  run(cycles: number): void {
    for (let k = 0; k < cycles; k++) {
      const cycle = this.cycle;
      this.#forces = this.#forces.filter((f) => f.until > cycle);
      const force: [number, number, number] = [0, 0, 0];
      for (const f of this.#forces) {
        const [x, y, z] = f.force(cycle);
        force[0] += x;
        force[1] += y;
        force[2] += z;
      }
      this.system.plant.setToolForce(force);
      const { task, start, name } = this.#task;
      if (task && cycle - start > (task.duration + 0.6) * BUS_CYCLE_HZ) this.startTask(name);
      this.system.step();
    }
  }

  /** Starts task `name`, driven by `computer` (default: the one of the last task). */
  startTask(name: TaskName, computer: RobotComputer = this.#computer): void {
    this.#computer = computer;
    if (name === "hold") {
      this.#task = { name, task: undefined, start: this.cycle };
      return;
    }
    const home = homePose(this.system);
    const task = name === "pick" ? pickAndPlace(home) : figureEight(home, 0.12, 4, 1);
    this.system.host.startTask(task, this.cycle, TASK_OPTIONS[computer]);
    this.#task = { name, task, start: this.cycle };
  }

  /** A short blow at the tool: `force` N for 10 ms. */
  tap(direction: Vec3, force = 200): void {
    const v: Vec3 = [direction[0] * force, direction[1] * force, direction[2] * force];
    this.#forces.push({ until: this.cycle + 0.01 * BUS_CYCLE_HZ, force: () => v });
  }

  /** A sustained push at the tool, as a collision would be: `force` N for 0.4 s. */
  push(direction: Vec3, force = 150): void {
    const v: Vec3 = [direction[0] * force, direction[1] * force, direction[2] * force];
    this.#forces.push({ until: this.cycle + 0.4 * BUS_CYCLE_HZ, force: () => v });
  }

  /** Shakes the tool sideways at `hz` for `seconds` (a machine nearby, say). */
  shake(hz: number, newtons = 15, seconds = 3): void {
    const start = this.cycle;
    this.#forces.push({
      until: start + seconds * BUS_CYCLE_HZ,
      force: (cycle) => [
        0,
        newtons * Math.sin((2 * Math.PI * hz * (cycle - start)) / BUS_CYCLE_HZ),
        0,
      ],
    });
  }

  /** Gives joint `joint` an actuator fault from now on. */
  inject(joint: number, effect: ActuatorEffect): void {
    this.system.injectDisturbance(joint, effect);
  }

  clearFaults(): void {
    this.system.clearDisturbances();
  }

  /** Pulls the safe-torque-off line (E-STOP). */
  emergencyStop(): void {
    this.system.safeTorqueOff = true;
  }

  acknowledge(): void {
    this.system.host.acknowledge();
  }

  dispose(): void {
    this.system.dispose();
  }
}

/**
 * Torque ripple the demo can inject into a joint, at the sixth electrical harmonic (where
 * it is felt even at slow speeds): 10 % of rated torque, which the joint can cancel, or
 * 50 %, beyond its compensation budget (the joint reports that it cannot).
 */
export function faultEffect(
  spec: ArmSpec,
  joint: number,
  kind: "fixable" | "unfixable",
): ActuatorEffect {
  const j = spec.joints[joint];
  if (!j) throw new RangeError(`joint ${joint} out of range`);
  const share = kind === "fixable" ? 0.1 : 0.5;
  return cogging([
    { cycles: rippleOrder(j), amplitude: share * j.actuator.ratedTorque, phase: 0.3 },
  ]);
}

/** Two arms that get every disturbance at the same cycle. */
export class TwinRig {
  readonly arms: readonly [ArmRig, ArmRig];

  constructor(left: ArmRig, right: ArmRig) {
    this.arms = [left, right];
  }

  /** Runs both arms `cycles` bus cycles. */
  run(cycles: number): void {
    for (const arm of this.arms) arm.run(cycles);
  }

  /** Does `action` to both arms. */
  both(action: (arm: ArmRig) => void): void {
    for (const arm of this.arms) action(arm);
  }

  dispose(): void {
    for (const arm of this.arms) arm.dispose();
  }
}

export { BUS_CYCLE_HZ, DampingMode, ImpactPolicy };
