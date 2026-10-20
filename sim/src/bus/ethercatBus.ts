/**
 * An EtherCAT-like line bus between the host (port 0) and the nodes (ports 1…N, by line
 * position).
 *
 * The bus only relays: it reads each frame's destination byte and delivers the frame,
 * without understanding the payload. It models what matters for the network's claims:
 * - a fixed cycle with a delivery delay of at least one cycle, plus optional jitter;
 * - lost frames (random, seeded) and a cut cable (everything beyond the cut is lost);
 * - the byte budget of one cycle, so the traffic the design assumes is realistic.
 *
 * Neighbour frames go to the sender's line neighbours and to the host, which monitors
 * everything, as an EtherCAT master sees all process data.
 */
import type { Random } from "../util/random.ts";
import { Address, decodeRecords, encodeRecords, FrameKind } from "./protocol.ts";

export const HOST_PORT = 0;

export interface BusOptions {
  /** Delivery delay in cycles (≥ 1: a frame written this cycle is read next cycle). */
  readonly latencyCycles?: number;
  /** Extra random delay, uniform in 0…jitterCycles. */
  readonly jitterCycles?: number;
  /** Probability that a frame is lost. */
  readonly dropRate?: number;
  /** Bytes the wire carries per cycle (100 Mbit/s at 4 kHz ≈ 3125). */
  readonly bytesPerCycle?: number;
}

/** Traffic counters, for reports and tests. */
export interface BusStats {
  framesSent: number;
  framesDelivered: number;
  framesDropped: number;
  /** Most bytes put on the wire in one cycle. */
  peakBytesPerCycle: number;
  /** Cycles in which the byte budget was exceeded. */
  overBudgetCycles: number;
}

interface Pending {
  readonly due: number;
  readonly to: number;
  readonly bytes: Uint8Array;
}

/** Overhead per frame on the wire (EtherCAT datagram header and working counter). */
const DATAGRAM_OVERHEAD = 12;

export class EthercatBus {
  readonly nodeCount: number;
  readonly #rng: Random;
  readonly #latency: number;
  readonly #jitter: number;
  readonly #dropRate: number;
  readonly #budget: number;
  readonly #pending: Pending[] = [];
  /** Last scheduled delivery per (from, to) link, so jitter never reorders a link. */
  readonly #lastDue = new Map<number, number>();
  /** Node address → line position, learned from the host's Assign frames. */
  readonly #positionOf = new Map<number, number>();
  #cutAt = Infinity;
  #bytesThisCycle = 0;
  #cycle = -1;
  readonly stats: BusStats = {
    framesSent: 0,
    framesDelivered: 0,
    framesDropped: 0,
    peakBytesPerCycle: 0,
    overBudgetCycles: 0,
  };

  constructor(nodeCount: number, rng: Random, options: BusOptions = {}) {
    this.nodeCount = nodeCount;
    this.#rng = rng;
    this.#latency = Math.max(1, options.latencyCycles ?? 1);
    this.#jitter = Math.max(0, options.jitterCycles ?? 0);
    this.#dropRate = options.dropRate ?? 0;
    this.#budget = options.bytesPerCycle ?? 3125;
  }

  /** Number of nodes the host can reach, as an EtherCAT broadcast read would count. */
  reachableNodes(): number {
    return Math.min(this.nodeCount, this.#cutAt - 1);
  }

  /** Cuts the cable just before line position `position`: it and everything beyond go dark. */
  cutBefore(position: number): void {
    this.#cutAt = Math.min(this.#cutAt, position);
  }

  /** Restores the cable. */
  reconnect(): void {
    this.#cutAt = Infinity;
  }

  /** Queues the mailbox `records` that `port` wrote during `cycle`. */
  send(port: number, records: Uint8Array, cycle: number): void {
    this.#startCycle(cycle);
    if (!this.#reachable(port)) return;
    for (const frame of decodeRecords(records)) {
      this.stats.framesSent++;
      this.#bytesThisCycle += frame.length + DATAGRAM_OVERHEAD;
      this.#learn(frame);
      for (const to of this.#route(port, frame)) this.#enqueue(port, to, frame, cycle);
    }
    this.stats.peakBytesPerCycle = Math.max(this.stats.peakBytesPerCycle, this.#bytesThisCycle);
  }

  /** Mailbox records due at `port` in `cycle` (removed from the bus). */
  receive(port: number, cycle: number): Uint8Array {
    this.#startCycle(cycle);
    const due: Uint8Array[] = [];
    let kept = 0;
    for (const p of this.#pending) {
      if (p.to !== port || p.due > cycle) {
        this.#pending[kept++] = p;
      } else if (this.#reachable(port)) {
        due.push(p.bytes);
        this.stats.framesDelivered++;
      } else {
        this.stats.framesDropped++;
      }
    }
    this.#pending.length = kept;
    return encodeRecords(due);
  }

  #startCycle(cycle: number): void {
    if (cycle === this.#cycle) return;
    if (this.#bytesThisCycle > this.#budget) this.stats.overBudgetCycles++;
    this.#cycle = cycle;
    this.#bytesThisCycle = 0;
  }

  #reachable(port: number): boolean {
    return port === HOST_PORT || port < this.#cutAt;
  }

  /** The host's Assign frames tell the bus which address sits at which position. */
  #learn(frame: Uint8Array): void {
    // Assign payload: position at byte 8, address at byte 9 (after the 8-byte header).
    if (frame[0] === FrameKind.Assign) this.#positionOf.set(frame[9] ?? 0, frame[8] ?? 0);
  }

  #route(from: number, frame: Uint8Array): number[] {
    const destination = frame[2] ?? Address.host;
    const ports: number[] = [];
    if (destination === Address.host) return [HOST_PORT];
    if (destination === Address.all) {
      for (let p = 0; p <= this.nodeCount; p++) if (p !== from) ports.push(p);
      return ports;
    }
    if (destination === Address.neighbours) {
      for (const p of [from - 1, from + 1]) if (p >= 1 && p <= this.nodeCount) ports.push(p);
      if (from !== HOST_PORT) ports.push(HOST_PORT);
      return ports;
    }
    const position = this.#positionOf.get(destination);
    return position === undefined ? [] : [position];
  }

  #enqueue(from: number, to: number, frame: Uint8Array, cycle: number): void {
    if (this.#dropRate > 0 && this.#rng.uniform() < this.#dropRate) {
      this.stats.framesDropped++;
      return;
    }
    const jitter = this.#jitter > 0 ? Math.floor(this.#rng.uniform() * (this.#jitter + 1)) : 0;
    const link = from * 1024 + to;
    const due = Math.max(cycle + this.#latency + jitter, this.#lastDue.get(link) ?? 0);
    this.#lastDue.set(link, due);
    this.#pending.push({ due, to, bytes: frame });
  }
}
