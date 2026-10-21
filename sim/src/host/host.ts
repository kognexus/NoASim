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
  type HostLoad,
  JointLevelGoals,
  MotionController,
} from "./motion.ts";
import type { Task } from "./tasks.ts";
import { VlaEmulator, type VlaOptions } from "./vla.ts";

/** The host re-sends its settings this often, in bus cycles (100 ms). */
const SETTINGS_REPEAT_CYCLES = 400;

/** How a task's policy steps reach the joints (H2). */
export type InterpolationMode = "central" | "joint-level";

export interface TaskOptions {
  readonly mode: InterpolationMode;
  readonly vla?: VlaOptions;
  readonly central?: CentralOptions;
}

export interface HostOptions {
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
  /** Every fault the nodes reported, in order. */
  readonly faults: { cycle: number; node: number; code: number; value: number }[] = [];
  /** Every impact the nodes reported, in order. */
  readonly impacts: { cycle: number; node: number; score: number; evidence: number }[] = [];
  /** Whether the host sends its cyclic frame (switch off to emulate a dead host). */
  heartbeat = true;
  readonly #spec: ArmSpec;
  readonly #rng: Random;
  #motion: MotionController | undefined;
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
  };

  constructor(spec: ArmSpec, rng: Random, options: HostOptions = {}) {
    this.#spec = spec;
    this.#rng = rng;
    const share = options.networkTorqueShare ?? 0.25;
    this.discovery = new Discovery(
      {
        limitsFor: (announce) => defaultLimits(announce, share),
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

  /** The running task's policy emulator, if any. */
  get motion(): MotionController | undefined {
    return this.#motion;
  }

  /** Starts following `task` from the joints' current angles. */
  startTask(task: Task, cycle: number, options: TaskOptions): MotionController {
    const start = this.#spec.joints.map(
      (joint, i) => this.jointAngles.get(i + 1) ?? joint.initialAngle,
    );
    const time = cycle / BUS_CYCLE_HZ;
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
    this.applySettings({ interpolation: options.mode === "central" ? 0 : 1 });
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

  /** One host cycle: reads delivered records, returns records to send. */
  tick(cycle: number, inbox: Uint8Array): Uint8Array {
    this.lastReceived = decodeRecords(inbox).map(decodeFrame);
    for (const f of this.lastReceived) {
      if ("state" in f) this.jointAngles.set(f.header.source, f.state.outputAngle);
      else if ("fault" in f)
        this.faults.push({ cycle: f.header.cycle, node: f.header.source, ...f.fault });
      else if ("impact" in f)
        this.impacts.push({ cycle: f.header.cycle, node: f.header.source, ...f.impact });
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
    for (const frame of this.#queue.splice(0))
      out.push({ ...frame, header: { ...frame.header, cycle } } as Frame);
    return encodeRecords(out.map(encodeFrame));
  }
}
