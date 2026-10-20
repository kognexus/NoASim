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

export class Discovery {
  readonly #options: DiscoveryOptions;
  readonly #events: DiscoveryEvents;
  readonly #nodes = new Map<number, DiscoveredNode>();
  #phase: DiscoveryPhase = "idle";
  #expected = 0;
  #readyCycle: number | undefined;

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
    return Array.from({ length: reachable }, (_, k) => ({
      header: { kind: FrameKind.Assign, source: Address.host, destination: Address.all, cycle },
      assign: { position: k + 1, id: k + 1 },
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
      for (const node of this.nodes) {
        out.push({
          header: { kind: FrameKind.Limits, source: Address.host, destination: node.id, cycle },
          limits: this.#options.limitsFor(node.announce),
        });
      }
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
