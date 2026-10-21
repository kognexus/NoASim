/**
 * The robot's main computer, as far as the bus is concerned: it discovers the nodes, sets
 * their limits and (from NX-09) sends goals. It never computes damping, impact detection
 * or interpolation.
 */
import { BUS_CYCLE_HZ } from "../index.ts";
import {
  Address,
  type Announce,
  decodeFrame,
  decodeRecords,
  encodeFrame,
  encodeRecords,
  type Frame,
  FrameKind,
  type Limits,
  type Settings,
} from "../bus/protocol.ts";
import type { ArmSpec } from "../plant/armSpec.ts";
import type { Random } from "../util/random.ts";
import { Discovery, type DiscoveryEvents } from "./discovery.ts";
import {
  CentralInterpolation,
  type CentralOptions,
  ClassicController,
  type ClassicOptions,
  type HostLoad,
  JointLevelGoals,
  MotionController,
} from "./motion.ts";
import type { Task } from "./tasks.ts";
import { VlaEmulator, type VlaOptions } from "./vla.ts";

/** The host re-sends its settings this often, in bus cycles (100 ms). */
const SETTINGS_REPEAT_CYCLES = 400;

/**
 * Which computer drives the task, and who interpolates.
 *
 * - `"joint-level"`: a policy computer (a VLA on a Thor-class machine, 10–50 Hz, with
 *   latency, jitter and lost steps) sends one goal per policy step; the joints interpolate.
 * - `"central"`: the same policy computer interpolates itself on a few-hundred-hertz loop
 *   and streams setpoints (the baseline of H2).
 * - `"classic"`: a classic real-time robot controller runs the task as a program, samples
 *   its path on a fixed 1–4 kHz loop and streams a setpoint every tick (NX-22).
 *
 * The joints follow setpoints in `"central"` and `"classic"` and interpolate goals in
 * `"joint-level"`; starting a task switches them (`Settings.interpolation`).
 */
export type InterpolationMode = "central" | "joint-level" | "classic";

export interface TaskOptions {
  readonly mode: InterpolationMode;
  readonly vla?: VlaOptions;
  readonly central?: CentralOptions;
  readonly classic?: ClassicOptions;
}

export interface HostOptions {
  /** Changes to the default limits sent to every node. */
  readonly limits?: Partial<Limits>;
  /** Share of each node's peak torque the network term may use. */
  readonly networkTorqueShare?: number;
  /** Cycles without state before the host declares a node lost. */
  readonly watchdogCycles?: number;
  readonly events?: DiscoveryEvents;
}

/** Default limits: the node's peak torque and a conservative speed and position range. */
export function defaultLimits(announce: Announce, networkTorqueShare: number): Limits {
  return {
    maxTorque: announce.peakTorque,
    maxVelocity: 3,
    minPosition: -2 * Math.PI,
    maxPosition: 2 * Math.PI,
    maxTemperature: 120,
    networkTorqueShare,
  };
}

export class Host {
  readonly discovery: Discovery;
  /** Work done on the host, for H8. */
  readonly load: HostLoad = { ikSolves: 0, interpolationSteps: 0, goalFramesSent: 0 };
  /** Latest output angle reported by each node, by address. */
  readonly jointAngles = new Map<number, number>();
  /**
   * Each joint's commanded reference angle, as it publishes it (NX-31). Tasks start from
   * these: a joint taking its link's sag out stands a little off its commanded angle.
   */
  readonly jointReferences = new Map<number, number>();
  /** Every fault the nodes reported, in order. */
  readonly faults: { cycle: number; node: number; code: number; value: number }[] = [];
  /** Every disturbance report the nodes broadcast, in order. */
  readonly disturbances: {
    cycle: number;
    node: number;
    source: number;
    order: number;
    amplitude: number;
  }[] = [];
  /** Every impact the nodes reported, in order. */
  readonly impacts: { cycle: number; node: number; score: number; evidence: number }[] = [];
  /** Whether the host sends its cyclic frame (switch off to emulate a dead host). */
  heartbeat = true;
  readonly #spec: ArmSpec;
  readonly #rng: Random;
  #motion: MotionController | undefined;
  #jointMotion: ((cycle: number) => Frame[]) | undefined;
  /** Every frame the host received, decoded, for the current cycle (monitoring). */
  lastReceived: Frame[] = [];
  /** Frames queued by the application (settings, goals), sent on the next tick. */
  readonly #queue: Frame[] = [];
  #settings: Settings = {
    damping: 0,
    interpolation: 0,
    impactPolicy: 0,
    model: 0,
    gravity: 1,
    impactConfirm: 0,
    coordinatedModel: 0,
  };

  constructor(spec: ArmSpec, rng: Random, options: HostOptions = {}) {
    this.#spec = spec;
    this.#rng = rng;
    const share = options.networkTorqueShare ?? 0.25;
    this.discovery = new Discovery(
      {
        limitsFor: (announce) => ({ ...defaultLimits(announce, share), ...options.limits }),
        watchdogCycles: options.watchdogCycles ?? 40,
      },
      options.events,
    );
  }

  /** Starts the bus: discovery of `reachable` nodes. Returns mailbox records to send. */
  start(reachable: number, cycle: number): Uint8Array {
    return encodeRecords(this.discovery.start(reachable, cycle).map(encodeFrame));
  }

  /** The behaviour switches last sent to the nodes. */
  get settings(): Settings {
    return this.#settings;
  }

  /** Switches node behaviour (damping mode, model, …) for every node, from the next cycle. */
  applySettings(changes: Partial<Settings>): void {
    this.#settings = { ...this.#settings, ...changes };
    this.#queue.push({
      header: {
        kind: FrameKind.Settings,
        source: Address.host,
        destination: Address.all,
        cycle: 0,
      },
      settings: this.#settings,
    });
  }

  /** The running task, if any. */
  get motion(): MotionController | undefined {
    return this.#motion;
  }

  /** Starts following `task` from the joints' current angles. */
  startTask(task: Task, cycle: number, options: TaskOptions): MotionController {
    const start = this.#spec.joints.map(
      (joint, i) =>
        this.jointReferences.get(i + 1) ?? this.jointAngles.get(i + 1) ?? joint.initialAngle,
    );
    const time = cycle / BUS_CYCLE_HZ;
    this.applySettings({ interpolation: options.mode === "joint-level" ? 1 : 0 });
    if (options.mode === "classic") {
      const strategy = new ClassicController(
        this.#spec,
        task,
        start,
        time,
        this.load,
        options.classic,
      );
      this.#motion = new MotionController(undefined, strategy);
      return this.#motion;
    }
    const vla = new VlaEmulator(
      task,
      time,
      this.#rng.derive(`vla/${task.name}/${cycle}`),
      options.vla,
    );
    const strategy =
      options.mode === "central"
        ? new CentralInterpolation(
            this.#spec,
            start,
            time,
            this.load,
            this.#rng.derive(`host-loop/${cycle}`),
            options.central,
          )
        : new JointLevelGoals(this.#spec, start, time, this.load);
    this.#motion = new MotionController(vla, strategy);
    return this.#motion;
  }

  /** Acknowledges every fault: holding joints accept goals again. */
  acknowledge(): void {
    this.#queue.push({
      header: { kind: FrameKind.Ack, source: Address.host, destination: Address.all, cycle: 0 },
      ack: { code: 0 },
    });
  }

  /**
   * Moves the joints along `angles(t)` (rad, t in seconds from now), as one goal per joint
   * every `periodSeconds`, for joint-level interpolation. For tests and demos that need
   * every joint turning.
   */
  startJointMotion(
    angles: (t: number) => readonly number[],
    cycle: number,
    periodSeconds = 0.02,
  ): void {
    const start = cycle;
    const every = Math.round(periodSeconds * BUS_CYCLE_HZ);
    this.applySettings({ interpolation: 1 });
    this.#jointMotion = (now: number) => {
      if ((now - start) % every !== 0) return [];
      const due = now + every;
      const t = (due - start) / BUS_CYCLE_HZ;
      const q = angles(t);
      const q1 = angles(t + 1e-3);
      return q.map((angle, i) => ({
        header: { kind: FrameKind.Goal, source: Address.host, destination: i + 1, cycle: now },
        goal: {
          dueCycle: due,
          position: angle,
          velocity: ((q1[i] ?? angle) - angle) / 1e-3,
          sequence: 0,
        },
      }));
    };
  }

  /** Stops a joint motion started with {@link startJointMotion}. */
  stopJointMotion(): void {
    this.#jointMotion = undefined;
  }

  /**
   * Asks node `address` to run its commissioning excitation for `seconds` (B11). `seed`
   * varies the excitation, so separate runs give independent data.
   */
  commission(address: number, seconds: number, seed: number, frequencyScale = 1): void {
    this.#queue.push({
      header: { kind: FrameKind.Commission, source: Address.host, destination: address, cycle: 0 },
      commission: { durationCycles: Math.round(seconds * BUS_CYCLE_HZ), seed, frequencyScale },
    });
  }

  /** One host cycle: reads delivered records, returns records to send. */
  tick(cycle: number, inbox: Uint8Array): Uint8Array {
    this.lastReceived = decodeRecords(inbox).map(decodeFrame);
    for (const f of this.lastReceived) {
      if ("state" in f) {
        this.jointAngles.set(f.header.source, f.state.outputAngle);
        this.jointReferences.set(f.header.source, f.state.reference[0]);
      } else if ("fault" in f)
        this.faults.push({ cycle: f.header.cycle, node: f.header.source, ...f.fault });
      else if ("impact" in f)
        this.impacts.push({ cycle: f.header.cycle, node: f.header.source, ...f.impact });
      else if ("disturbance" in f)
        this.disturbances.push({ cycle: f.header.cycle, node: f.header.source, ...f.disturbance });
    }
    const out = this.discovery.handle(this.lastReceived, cycle);
    if (this.heartbeat) {
      out.push({
        header: {
          kind: FrameKind.Heartbeat,
          source: Address.host,
          destination: Address.all,
          cycle,
        },
        heartbeat: true,
      });
    }
    // Settings are re-sent periodically, as a mailbox would retry, so a lost frame does not
    // leave a joint in the wrong mode.
    if (cycle > 0 && cycle % SETTINGS_REPEAT_CYCLES === 0 && this.discovery.phase === "ready") {
      out.push({
        header: { kind: FrameKind.Settings, source: Address.host, destination: Address.all, cycle },
        settings: this.#settings,
      });
    }
    if (this.#motion) out.push(...this.#motion.tick(cycle));
    if (this.#jointMotion) out.push(...this.#jointMotion(cycle));
    for (const frame of this.#queue.splice(0))
      out.push({ ...frame, header: { ...frame.header, cycle } } as Frame);
    return encodeRecords(out.map(encodeFrame));
  }
}
