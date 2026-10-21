/**
 * What travels on one arm's bus, as the host sees it: each node's reported status, the
 * frames per second by kind, and an event log (impacts, faults, disturbance claims).
 * Drawn as a line of nodes with pulses for the traffic of the last frame.
 */
import { FaultCode, FrameKind } from "../../sim/src/bus/protocol.ts";
import type { Host } from "../../sim/src/host/host.ts";
import { BUS_CYCLE_HZ } from "../../sim/src/index.ts";
import { jointState, STATUS_VAR } from "./status.ts";

const FAULT_NAMES: Record<number, string> = {
  [FaultCode.impact]: "impact",
  [FaultCode.configRejected]: "configuration rejected",
  [FaultCode.commLoss]: "comm loss",
  [FaultCode.source]: "cannot fix its disturbance",
  [FaultCode.limit]: "limit acting",
};

const KIND_NAMES: Partial<Record<number, string>> = {
  [FrameKind.State]: "state",
  [FrameKind.Chain]: "gravity chain",
  [FrameKind.Disturbance]: "disturbance",
  [FrameKind.Impact]: "impact",
  [FrameKind.Fault]: "fault",
  [FrameKind.Goal]: "goal",
};

export interface BusEvent {
  readonly time: number;
  readonly text: string;
  readonly severity: "info" | "warn" | "fault";
}

/** Keeps the host's view of one bus up to date, frame by frame. */
export class BusMonitor {
  /** Last status each node reported, by address. */
  readonly status = new Map<number, number>();
  readonly events: BusEvent[] = [];
  /** Frames per second by kind, over the last second. */
  readonly rate = new Map<number, number>();
  /** Frames of each kind from each node during the last update, for the pulses. */
  readonly pulses = new Map<number, Set<number>>();
  #counts = new Map<number, number>();
  #windowStart = 0;
  #seen = { faults: 0, impacts: 0, disturbances: 0 };

  readonly #host: Host;

  constructor(host: Host) {
    this.#host = host;
  }

  /** Call after every bus cycle. */
  observe(cycle: number): void {
    for (const frame of this.#host.lastReceived) {
      const kind = frame.header.kind;
      this.#counts.set(kind, (this.#counts.get(kind) ?? 0) + 1);
      if (kind !== FrameKind.State) {
        let set = this.pulses.get(frame.header.source);
        if (!set) this.pulses.set(frame.header.source, (set = new Set()));
        set.add(kind);
      }
      if ("state" in frame) this.status.set(frame.header.source, frame.state.status);
    }
    if (cycle - this.#windowStart >= BUS_CYCLE_HZ) {
      this.rate.clear();
      for (const [kind, n] of this.#counts)
        this.rate.set(kind, (n * BUS_CYCLE_HZ) / (cycle - this.#windowStart));
      this.#counts = new Map();
      this.#windowStart = cycle;
    }
    this.#collectEvents();
  }

  /** Forgets the pulses once drawn. */
  takePulses(): Map<number, Set<number>> {
    const pulses = new Map(this.pulses);
    this.pulses.clear();
    return pulses;
  }

  #collectEvents(): void {
    const time = (cycle: number) => cycle / BUS_CYCLE_HZ;
    const h = this.#host;
    for (const f of h.faults.slice(this.#seen.faults)) {
      const what = FAULT_NAMES[f.code] ?? `fault ${f.code}`;
      this.#log(
        time(f.cycle),
        `J${f.node}: ${what}`,
        f.code === FaultCode.limit ? "warn" : "fault",
      );
    }
    for (const i of h.impacts.slice(this.#seen.impacts)) {
      this.#log(
        time(i.cycle),
        `J${i.node}: impact felt (×${i.score.toFixed(1)} threshold)`,
        "fault",
      );
    }
    for (const d of h.disturbances.slice(this.#seen.disturbances)) {
      // Claims only; felt reports are many and go to the rate counter.
      if (d.node === d.source) {
        this.#log(
          time(d.cycle),
          `J${d.node}: claims a ${d.amplitude.toFixed(2)} N·m ripple at order ${d.order}`,
          "warn",
        );
      }
    }
    this.#seen = {
      faults: h.faults.length,
      impacts: h.impacts.length,
      disturbances: h.disturbances.length,
    };
  }

  #log(time: number, text: string, severity: BusEvent["severity"]): void {
    const last = this.events[this.events.length - 1];
    if (last && last.text === text && time - last.time < 1) return; // repeated report
    this.events.push({ time, text, severity });
    if (this.events.length > 60) this.events.shift();
  }
}

/** Horizontal position of bus node `i` (0 is the host) on a canvas `width` px wide. */
const nodeX = (i: number, width: number, nodes: number) => 28 + (i * (width - 56)) / nodes;

/** The joint (0-based) whose node is at `clientX` on `canvas`, or −1 if none is near. */
export function busJointAt(canvas: HTMLCanvasElement, clientX: number, nodes: number): number {
  const x = clientX - canvas.getBoundingClientRect().left;
  for (let i = 1; i <= nodes; i++) {
    if (Math.abs(x - nodeX(i, canvas.clientWidth, nodes)) <= 16) return i - 1;
  }
  return -1;
}

/**
 * Draws the host and the line of nodes, with pulses for non-cyclic frames and a ring on
 * the selected joint (0-based, −1 for none).
 */
export function drawBus(
  canvas: HTMLCanvasElement,
  monitor: BusMonitor,
  nodes: number,
  selected = -1,
): void {
  const ratio = Math.min(2, window.devicePixelRatio);
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * ratio)) canvas.width = Math.round(w * ratio);
  if (canvas.height !== Math.round(h * ratio)) canvas.height = Math.round(h * ratio);
  const g = canvas.getContext("2d");
  if (!g) return;
  const css = getComputedStyle(canvas);
  const colour = (name: string) => css.getPropertyValue(name);
  g.setTransform(ratio, 0, 0, ratio, 0, 0);
  g.clearRect(0, 0, w, h);
  const y = h / 2;
  const x = (i: number) => nodeX(i, w, nodes);
  g.strokeStyle = colour("--grid");
  g.lineWidth = 2;
  g.beginPath();
  g.moveTo(x(0), y);
  g.lineTo(x(nodes), y);
  g.stroke();
  const pulses = monitor.takePulses();
  g.font = "11px system-ui, sans-serif";
  g.textAlign = "center";
  for (let i = 0; i <= nodes; i++) {
    const kinds = pulses.get(i);
    if (kinds?.size) {
      const urgent = kinds.has(FrameKind.Impact) || kinds.has(FrameKind.Fault);
      g.fillStyle = colour(
        urgent
          ? "--status-fault"
          : kinds.has(FrameKind.Disturbance)
            ? "--status-holding"
            : "--pulse",
      );
      g.globalAlpha = 0.35;
      g.beginPath();
      g.arc(x(i), y, 19, 0, 2 * Math.PI);
      g.fill();
      g.globalAlpha = 1;
    }
    g.fillStyle = colour(i === 0 ? "--host" : STATUS_VAR[jointState(monitor.status.get(i))]);
    g.beginPath();
    g.arc(x(i), y, 12, 0, 2 * Math.PI);
    g.fill();
    if (i === selected + 1 && i > 0) {
      g.strokeStyle = colour("--text");
      g.lineWidth = 2;
      g.beginPath();
      g.arc(x(i), y, 16, 0, 2 * Math.PI);
      g.stroke();
    }
    g.fillStyle = colour("--muted");
    g.fillText(i === 0 ? "host" : `J${i}`, x(i), y + 28);
  }
}

/** Frames per second by kind, for the table under the bus line. */
export function rateRows(monitor: BusMonitor): [string, number][] {
  return [...monitor.rate]
    .filter(([kind]) => KIND_NAMES[kind])
    .map(([kind, rate]) => [KIND_NAMES[kind] ?? String(kind), rate] as [string, number])
    .sort((a, b) => b[1] - a[1]);
}
