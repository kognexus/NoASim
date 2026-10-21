/**
 * How the host turns a task into joint goals.
 *
 * Two strategies for a policy computer (the comparison of H2):
 *
 * - {@link CentralInterpolation} (the usual way): the host solves IK for every policy
 *   step and interpolates in joint space on its own loop, which runs at a few hundred
 *   hertz and is late now and then. It streams a setpoint to every joint each host tick,
 *   and the joints follow each setpoint as it comes.
 * - {@link JointLevelGoals} (Nexus): the host solves IK for every policy step and sends
 *   the joints one goal per step, stamped with the bus cycle it is due. The joints
 *   interpolate it themselves at the bus rate (NX-10).
 *
 * And one for a classic robot controller (NX-22):
 *
 * - {@link ClassicController}: a deterministic real-time computer running a robot program.
 *   It samples the programmed path itself on a fixed loop (1 kHz by default, up to the
 *   4 kHz bus rate), never late, solves IK every tick and streams a setpoint to every
 *   joint, as EtherCAT drives in cyclic position mode are fed. No policy is involved.
 */
import { Address, type Frame, FrameKind } from "../bus/protocol.ts";
import { BUS_CYCLE_HZ } from "../index.ts";
import type { ArmSpec } from "../plant/armSpec.ts";
import type { Random } from "../util/random.ts";
import { solveIk } from "./ik.ts";
import type { Task } from "./tasks.ts";
import type { VlaEmulator, VlaStep } from "./vla.ts";

/** Work the host did, for H8. */
export interface HostLoad {
  ikSolves: number;
  interpolationSteps: number;
  goalFramesSent: number;
}

export interface MotionStrategy {
  readonly name: string;
  /** Called every bus cycle; returns goal frames to send. */
  tick(cycle: number, step: VlaStep | undefined): Frame[];
}

function goalFrame(
  node: number,
  cycle: number,
  dueCycle: number,
  position: number,
  velocity: number,
  sequence: number,
): Frame {
  return {
    header: { kind: FrameKind.Goal, source: Address.host, destination: node, cycle },
    goal: { dueCycle, position, velocity, sequence: sequence & 0xffff },
  };
}

/** Policy steps closer together than this are treated as duplicates, s. */
const MIN_STEP_SECONDS = 1e-3;

/** Shared IK bookkeeping: joint goals for successive policy steps. */
class GoalTrack {
  readonly #spec: ArmSpec;
  readonly #load: HostLoad;
  previous: { time: number; angles: number[] };
  latest: { time: number; angles: number[] };

  constructor(spec: ArmSpec, start: number[], startTime: number, load: HostLoad) {
    this.#spec = spec;
    this.#load = load;
    this.previous = { time: startTime, angles: start };
    this.latest = { time: startTime, angles: start };
  }

  /** Adds a policy step; returns false (and ignores it) if it is not later than the last one. */
  add(step: VlaStep): boolean {
    if (step.targetTime <= this.latest.time + MIN_STEP_SECONDS) return false;
    const angles = solveIk(this.#spec, step.target, this.latest.angles);
    this.#load.ikSolves++;
    this.previous = this.latest;
    this.latest = { time: step.targetTime, angles };
    return true;
  }
}

export interface CentralOptions {
  /** Host control loop rate, Hz. */
  readonly loopHz?: number;
  /** Probability that a host tick runs late (by one to four loop periods). */
  readonly lateRate?: number;
}

export class CentralInterpolation implements MotionStrategy {
  readonly name = "central";
  readonly #track: GoalTrack;
  readonly #load: HostLoad;
  readonly #rng: Random;
  readonly #loopCycles: number;
  readonly #lateRate: number;
  #nextTick = 0;
  #sequence = 0;

  constructor(
    spec: ArmSpec,
    start: number[],
    startTime: number,
    load: HostLoad,
    rng: Random,
    options: CentralOptions = {},
  ) {
    this.#track = new GoalTrack(spec, start, startTime, load);
    this.#load = load;
    this.#rng = rng;
    this.#loopCycles = Math.round(BUS_CYCLE_HZ / (options.loopHz ?? 250));
    this.#lateRate = options.lateRate ?? 0.1;
  }

  tick(cycle: number, step: VlaStep | undefined): Frame[] {
    if (step) this.#track.add(step);
    if (cycle < this.#nextTick) return [];
    const late = this.#rng.uniform() < this.#lateRate ? 1 + Math.floor(this.#rng.uniform() * 4) : 0;
    this.#nextTick = cycle + this.#loopCycles * (1 + late);
    const time = cycle / BUS_CYCLE_HZ;
    const { previous, latest } = this.#track;
    const span = Math.max(1e-6, latest.time - previous.time);
    const s = Math.min(1, Math.max(0, (time - previous.time) / span));
    this.#sequence++;
    return latest.angles.map((q, i) => {
      const q0 = previous.angles[i] ?? q;
      this.#load.interpolationSteps++;
      this.#load.goalFramesSent++;
      const velocity = s < 1 ? (q - q0) / span : 0;
      return goalFrame(i + 1, cycle, cycle, q0 + s * (q - q0), velocity, this.#sequence);
    });
  }
}

export class JointLevelGoals implements MotionStrategy {
  readonly name = "joint-level";
  readonly #track: GoalTrack;
  readonly #load: HostLoad;
  #sequence = 0;

  constructor(spec: ArmSpec, start: number[], startTime: number, load: HostLoad) {
    this.#track = new GoalTrack(spec, start, startTime, load);
    this.#load = load;
  }

  tick(cycle: number, step: VlaStep | undefined): Frame[] {
    if (!step || !this.#track.add(step)) return [];
    this.#sequence++;
    const { previous, latest } = this.#track;
    const span = Math.max(1e-6, latest.time - previous.time);
    const due = Math.round(latest.time * BUS_CYCLE_HZ);
    return latest.angles.map((q, i) => {
      this.#load.goalFramesSent++;
      return goalFrame(
        i + 1,
        cycle,
        due,
        q,
        (q - (previous.angles[i] ?? q)) / span,
        this.#sequence,
      );
    });
  }
}

/** Drives one task: polls the policy and lets the strategy turn steps into goals. */
export interface ClassicOptions {
  /** Controller loop rate, Hz; a divisor of the bus rate (4 kHz). */
  readonly loopHz?: number;
}

/** Fastest joint speed of the lead-in to a program's start, rad/s. */
const LEAD_IN_SPEED = 0.5;
/** Shortest lead-in, s. */
const MIN_LEAD_IN = 0.3;

export class ClassicController implements MotionStrategy {
  readonly name = "classic";
  readonly #spec: ArmSpec;
  readonly #task: Task;
  readonly #startTime: number;
  readonly #load: HostLoad;
  readonly #loopCycles: number;
  readonly #from: number[];
  readonly #programStart: number[];
  /** Duration of the move from where the joints are to the program's start, s. */
  readonly leadIn: number;
  #angles: number[];
  #nextTick = 0;
  #sequence = 0;

  constructor(
    spec: ArmSpec,
    task: Task,
    start: number[],
    startTime: number,
    load: HostLoad,
    options: ClassicOptions = {},
  ) {
    this.#spec = spec;
    this.#task = task;
    this.#startTime = startTime;
    this.#load = load;
    this.#angles = start;
    this.#loopCycles = Math.max(1, Math.round(BUS_CYCLE_HZ / (options.loopHz ?? 1000)));
    // Like any robot controller, first move to where the program starts (a quintic joint
    // move, peak speed 1.875·distance/duration): the arm may be anywhere, say when this
    // controller takes over from a policy computer mid-motion.
    this.#from = start;
    this.#programStart = solveIk(spec, task.pose(0), start);
    load.ikSolves++;
    const distance = Math.max(...this.#programStart.map((q, i) => Math.abs(q - (start[i] ?? q))));
    this.leadIn = distance < 1e-3 ? 0 : Math.max(MIN_LEAD_IN, (1.875 * distance) / LEAD_IN_SPEED);
  }

  tick(cycle: number): Frame[] {
    if (cycle < this.#nextTick) return [];
    this.#nextTick = cycle + this.#loopCycles;
    const elapsed = cycle / BUS_CYCLE_HZ - this.#startTime;
    let angles: number[];
    if (elapsed < this.leadIn) {
      const s = elapsed / this.leadIn;
      const blend = s * s * s * (10 - 15 * s + 6 * s * s);
      angles = this.#programStart.map(
        (q, i) => (this.#from[i] ?? q) + (q - (this.#from[i] ?? q)) * blend,
      );
    } else {
      const t = Math.min(this.#task.duration, elapsed - this.leadIn);
      angles = solveIk(this.#spec, this.#task.pose(Math.max(0, t)), this.#angles);
      this.#load.ikSolves++;
    }
    const period = this.#loopCycles / BUS_CYCLE_HZ;
    const previous = this.#angles;
    this.#angles = angles;
    this.#sequence++;
    return angles.map((q, i) => {
      this.#load.interpolationSteps++;
      this.#load.goalFramesSent++;
      return goalFrame(i + 1, cycle, cycle, q, (q - (previous[i] ?? q)) / period, this.#sequence);
    });
  }
}

/** Runs a task: polls the policy (if the task comes from one) and lets the strategy send goals. */
export class MotionController {
  /** The policy emulator, for a policy computer; none for a classic controller. */
  readonly vla: VlaEmulator | undefined;
  readonly strategy: MotionStrategy;

  constructor(vla: VlaEmulator | undefined, strategy: MotionStrategy) {
    this.vla = vla;
    this.strategy = strategy;
  }

  tick(cycle: number): Frame[] {
    return this.strategy.tick(cycle, this.vla?.poll(cycle / BUS_CYCLE_HZ));
  }
}
