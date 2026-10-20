/**
 * Emulates a vision-language-action policy: it emits a new tool target every 1/rate
 * seconds, a little late and sometimes not at all. Each target is where the tool should
 * be one policy step later.
 */
import type { Random } from "../util/random.ts";
import type { Pose } from "./ik.ts";
import type { Task } from "./tasks.ts";

export interface VlaOptions {
  /** Policy rate, Hz (10–50 for current VLAs). */
  readonly rateHz?: number;
  /** Inference time, s: a step planned at t arrives at t + latency. */
  readonly latencySeconds?: number;
  /** Extra random lateness, uniform in 0…jitter, s. */
  readonly jitterSeconds?: number;
  /** Probability that a step's output never arrives. */
  readonly dropRate?: number;
}

/** One policy output. */
export interface VlaStep {
  /** Step index (increments even when a step is dropped). */
  readonly index: number;
  /** Time the target refers to: the tool should be there at `targetTime`, s. */
  readonly targetTime: number;
  readonly target: Pose;
}

export class VlaEmulator {
  readonly task: Task;
  readonly period: number;
  readonly #latency: number;
  readonly #jitter: number;
  readonly #dropRate: number;
  readonly #rng: Random;
  readonly #start: number;
  #next = 0;
  #arrival: number;
  #dropped = 0;

  constructor(task: Task, startTime: number, rng: Random, options: VlaOptions = {}) {
    this.task = task;
    this.period = 1 / (options.rateHz ?? 15);
    this.#latency = options.latencySeconds ?? 0.03;
    this.#jitter = options.jitterSeconds ?? 0.015;
    this.#dropRate = options.dropRate ?? 0.05;
    this.#rng = rng;
    this.#start = startTime;
    this.#arrival = this.#scheduleArrival(0);
  }

  /** Steps that were dropped so far. */
  get dropped(): number {
    return this.#dropped;
  }

  /** True once the task's last step has been emitted. */
  get finished(): boolean {
    return this.#next * this.period > this.task.duration;
  }

  /** The step that arrives at or before `time`, if any (at most one per call). */
  poll(time: number): VlaStep | undefined {
    while (!this.finished && time >= this.#arrival) {
      const index = this.#next++;
      const drop = index > 0 && this.#rng.uniform() < this.#dropRate;
      this.#arrival = this.#scheduleArrival(this.#next);
      if (drop) {
        this.#dropped++;
        continue;
      }
      const taskTime = Math.min(this.task.duration, (index + 1) * this.period);
      return { index, targetTime: this.#start + taskTime, target: this.task.pose(taskTime) };
    }
    return undefined;
  }

  #scheduleArrival(index: number): number {
    return this.#start + index * this.period + this.#latency + this.#rng.uniform() * this.#jitter;
  }
}
