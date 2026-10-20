import { describe, expect, it } from "vitest";
import { Random } from "../util/random.ts";
import { EthercatBus, HOST_PORT } from "./ethercatBus.ts";
import {
  Address,
  decodeFrame,
  decodeRecords,
  encodeFrame,
  encodeRecords,
  FrameKind,
  type Frame,
} from "./protocol.ts";

const goal = (source: number, destination: number, sequence = 0): Frame => ({
  header: { kind: FrameKind.Goal, source, destination, cycle: 0 },
  goal: { dueCycle: 0, position: 0, velocity: 0, sequence },
});
const records = (...frames: Frame[]) => encodeRecords(frames.map(encodeFrame));
const received = (bus: EthercatBus, port: number, cycle: number) =>
  decodeRecords(bus.receive(port, cycle)).map(decodeFrame);

function assignAll(bus: EthercatBus, n: number): void {
  const frames: Frame[] = Array.from({ length: n }, (_, k) => ({
    header: { kind: FrameKind.Assign, source: 0, destination: Address.all, cycle: 0 },
    assign: { position: k + 1, id: 10 + k },
  }));
  bus.send(HOST_PORT, records(...frames), 0);
  for (let p = 0; p <= n; p++) bus.receive(p, 1);
}

describe("EthercatBus", () => {
  it("delivers no earlier than the next cycle", () => {
    const bus = new EthercatBus(3, new Random(1), { latencyCycles: 2 });
    bus.send(2, records(goal(2, Address.host)), 10);
    expect(received(bus, HOST_PORT, 11)).toHaveLength(0);
    expect(received(bus, HOST_PORT, 12)).toHaveLength(1);
  });

  it("sends neighbour frames to the two adjacent nodes and the host only", () => {
    const bus = new EthercatBus(4, new Random(1));
    bus.send(2, records(goal(2, Address.neighbours)), 0);
    const counts = [0, 1, 2, 3, 4].map((p) => received(bus, p, 1).length);
    expect(counts).toEqual([1, 1, 0, 1, 0]);
  });

  it("routes unicast frames by the addresses the host assigned", () => {
    const bus = new EthercatBus(3, new Random(1));
    assignAll(bus, 3);
    bus.send(HOST_PORT, records(goal(0, 11)), 5);
    expect([1, 2, 3].map((p) => received(bus, p, 6).length)).toEqual([0, 1, 0]);
  });

  it("drops frames at the configured rate and never reorders a link under jitter", () => {
    const bus = new EthercatBus(2, new Random(3), { dropRate: 0.2, jitterCycles: 3 });
    const sequences: number[] = [];
    for (let c = 0; c < 5000; c++) {
      bus.send(1, records(goal(1, Address.host, c % 60000)), c);
      for (const f of received(bus, HOST_PORT, c)) if ("goal" in f) sequences.push(f.goal.sequence);
    }
    expect(bus.stats.framesDropped / bus.stats.framesSent).toBeCloseTo(0.2, 1);
    expect(sequences).toEqual([...sequences].sort((a, b) => a - b));
  });

  it("loses everything beyond a cut cable", () => {
    const bus = new EthercatBus(6, new Random(1));
    bus.cutBefore(4);
    expect(bus.reachableNodes()).toBe(3);
    bus.send(5, records(goal(5, Address.host)), 0);
    bus.send(HOST_PORT, records(goal(0, Address.all)), 0);
    expect(received(bus, HOST_PORT, 1)).toHaveLength(0);
    expect(received(bus, 3, 1)).toHaveLength(1);
    expect(received(bus, 4, 1)).toHaveLength(0);
  });
});
