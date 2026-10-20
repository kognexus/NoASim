/**
 * How the host turns policy steps into joint goals: the two strategies compared in H2.
 *
 * - {@link CentralInterpolation} (the usual way): the host solves IK for every policy
 *   step and interpolates in joint space on its own loop, which runs at a few hundred
 *   hertz and is late now and then. It streams a setpoint to every joint each host tick,
 *   and the joints follow each setpoint as it comes.
 * - {@link JointLevelGoals} (Nexus): the host solves IK for every policy step and sends
 *   the joints one goal per step, stamped with the bus cycle it is due. The joints
 *   interpolate it themselves at the bus rate (NX-10).
 */
import { Address, type Frame, FrameKind } from "../bus/protocol.ts";
import { BUS_CYCLE_HZ } from "../index.ts";
import type { ArmSpec } from "../plant/armSpec.ts";
import type { Random } from "../util/random.ts";
import { solveIk } from "./ik.ts";
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

  add(step: VlaStep): void {
    const angles = solveIk(this.#spec, step.target, this.latest.angles);
    this.#load.ikSolves++;
    this.previous = this.latest;
    this.latest = { time: step.targetTime, angles };
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
    if (!step) return [];
    this.#track.add(step);
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
export class MotionController {
  readonly vla: VlaEmulator;
  readonly strategy: MotionStrategy;

  constructor(vla: VlaEmulator, strategy: MotionStrategy) {
    this.vla = vla;
    this.strategy = strategy;
  }

  tick(cycle: number): Frame[] {
    return this.strategy.tick(cycle, this.vla.poll(cycle / BUS_CYCLE_HZ));
  }
}
