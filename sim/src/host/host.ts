/**
 * The robot's main computer, as far as the bus is concerned: it discovers the nodes, sets
 * their limits and (from NX-09) sends goals. It never computes damping, impact detection
 * or interpolation.
 */
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
import { Discovery, type DiscoveryEvents } from "./discovery.ts";

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
  /** Every frame the host received, decoded, for the current cycle (monitoring). */
  lastReceived: Frame[] = [];
  /** Frames queued by the application (settings, goals), sent on the next tick. */
  readonly #queue: Frame[] = [];
  #settings: Settings = { damping: 0, interpolation: 0, impactPolicy: 0, model: 0, gravity: 1 };

  constructor(options: HostOptions = {}) {
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

  /** One host cycle: reads delivered records, returns records to send. */
  tick(cycle: number, inbox: Uint8Array): Uint8Array {
    this.lastReceived = decodeRecords(inbox).map(decodeFrame);
    const out = this.discovery.handle(this.lastReceived, cycle);
    for (const frame of this.#queue.splice(0))
      out.push({ ...frame, header: { ...frame.header, cycle } } as Frame);
    return encodeRecords(out.map(encodeFrame));
  }
}
