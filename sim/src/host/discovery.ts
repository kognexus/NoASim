/**
 * Host-side discovery (B0): find the nodes on the line, give them addresses, learn what
 * they are, set their limits, and notice when one goes quiet.
 *
 * 1. Count the reachable nodes (an EtherCAT broadcast read).
 * 2. Assign address p to line position p.
 * 3. Collect every node's announcement (nominal parameters, brake, model version).
 * 4. Send each node its limits.
 * 5. Done when every node publishes READY. Afterwards a watchdog reports nodes whose
 *    state stops arriving.
 */
import {
  Address,
  type Announce,
  encodeFrame,
  type Frame,
  FrameKind,
  type Limits,
  StatusFlags,
} from "../bus/protocol.ts";

export type DiscoveryPhase = "idle" | "announcing" | "configuring" | "ready";

export interface DiscoveredNode {
  readonly id: number;
  readonly announce: Announce;
  /** Cycle of the latest state frame received from it. */
  lastStateCycle: number;
  ready: boolean;
  lost: boolean;
}

export interface DiscoveryEvents {
  /** Called once when every node is READY. */
  onReady?(cycle: number, nodes: readonly DiscoveredNode[]): void;
  /** Called when a node's state has not arrived for longer than the watchdog allows. */
  onLost?(cycle: number, node: DiscoveredNode): void;
}

export interface DiscoveryOptions {
  /** Limits sent to each node, by address. */
  readonly limitsFor: (announce: Announce) => Limits;
  /** Cycles without state before a node counts as lost. */
  readonly watchdogCycles?: number;
}

/** Unanswered discovery frames are repeated after this many cycles. */
const RETRY_CYCLES = 20;

export class Discovery {
  readonly #options: DiscoveryOptions;
  readonly #events: DiscoveryEvents;
  readonly #nodes = new Map<number, DiscoveredNode>();
  #phase: DiscoveryPhase = "idle";
  #expected = 0;
  #readyCycle: number | undefined;
  #lastSend = 0;

  constructor(options: DiscoveryOptions, events: DiscoveryEvents = {}) {
    this.#options = options;
    this.#events = events;
  }

  get phase(): DiscoveryPhase {
    return this.#phase;
  }

  /** Cycle at which every node became READY. */
  get readyCycle(): number | undefined {
    return this.#readyCycle;
  }

  /** Discovered nodes by line position (= address). */
  get nodes(): readonly DiscoveredNode[] {
    return [...this.#nodes.values()].sort((a, b) => a.id - b.id);
  }

  /** Starts discovery of `reachable` nodes; returns the Assign frames to send. */
  start(reachable: number, cycle: number): Frame[] {
    this.#phase = "announcing";
    this.#expected = reachable;
    this.#nodes.clear();
    this.#lastSend = cycle;
    return this.#assignments(cycle);
  }

  #assignments(cycle: number): Frame[] {
    return Array.from({ length: this.#expected }, (_, k) => k + 1)
      .filter((position) => !this.#nodes.has(position))
      .map((position) => ({
        header: { kind: FrameKind.Assign, source: Address.host, destination: Address.all, cycle },
        assign: { position, id: position },
      }));
  }

  #limits(cycle: number, nodes: readonly DiscoveredNode[]): Frame[] {
    return nodes.map((node) => ({
      header: { kind: FrameKind.Limits, source: Address.host, destination: node.id, cycle },
      limits: this.#options.limitsFor(node.announce),
    }));
  }

  /** Handles frames the host received in `cycle`; returns frames to send. */
  handle(frames: readonly Frame[], cycle: number): Frame[] {
    const out: Frame[] = [];
    for (const frame of frames) {
      if ("announce" in frame && this.#phase === "announcing") {
        this.#nodes.set(frame.header.source, {
          id: frame.header.source,
          announce: frame.announce,
          lastStateCycle: cycle,
          ready: false,
          lost: false,
        });
      } else if ("state" in frame) {
        const node = this.#nodes.get(frame.header.source);
        if (!node) continue;
        node.lastStateCycle = Math.max(node.lastStateCycle, frame.header.cycle);
        node.ready = (frame.state.status & StatusFlags.ready) !== 0;
      }
    }
    if (this.#phase === "announcing" && this.#nodes.size === this.#expected) {
      this.#phase = "configuring";
      this.#lastSend = cycle;
      out.push(...this.#limits(cycle, this.nodes));
    }
    // Frames can be lost: repeat what has not been answered yet.
    if (cycle - this.#lastSend >= RETRY_CYCLES) {
      this.#lastSend = cycle;
      if (this.#phase === "announcing") out.push(...this.#assignments(cycle));
      if (this.#phase === "configuring")
        out.push(
          ...this.#limits(
            cycle,
            this.nodes.filter((n) => !n.ready),
          ),
        );
    }
    if (this.#phase === "configuring" && this.nodes.every((n) => n.ready)) {
      this.#phase = "ready";
      this.#readyCycle = cycle;
      this.#events.onReady?.(cycle, this.nodes);
    }
    if (this.#phase === "ready") this.#watch(cycle);
    return out;
  }

  #watch(cycle: number): void {
    const limit = this.#options.watchdogCycles ?? 40;
    for (const node of this.#nodes.values()) {
      if (!node.lost && cycle - node.lastStateCycle > limit) {
        node.lost = true;
        this.#events.onLost?.(cycle, node);
      }
    }
  }
}

/** Encodes frames for sending. */
export const encodeAll = (frames: readonly Frame[]): Uint8Array[] => frames.map(encodeFrame);
