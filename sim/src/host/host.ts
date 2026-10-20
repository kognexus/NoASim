/**
 * The robot's main computer, as far as the bus is concerned: it discovers the nodes, sets
 * their limits and (from NX-09) sends goals. It never computes damping, impact detection
 * or interpolation.
 */
import {
  type Announce,
  decodeFrame,
  decodeRecords,
  encodeFrame,
  encodeRecords,
  type Frame,
  type Limits,
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

  /** One host cycle: reads delivered records, returns records to send. */
  tick(cycle: number, inbox: Uint8Array): Uint8Array {
    this.lastReceived = decodeRecords(inbox).map(decodeFrame);
    const out = this.discovery.handle(this.lastReceived, cycle);
    return encodeRecords(out.map(encodeFrame));
  }
}
