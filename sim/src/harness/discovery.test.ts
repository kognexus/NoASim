import { afterEach, beforeAll, describe, expect, it } from "vitest";
import type { DiscoveredNode } from "../host/discovery.ts";
import { KX6 } from "../plant/kx6.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import { ArmSystem } from "./armSystem.ts";

describe("discovery over the bus (NX-06)", () => {
  let mujoco: Mujoco;
  let nodeModule: WebAssembly.Module;
  let system: ArmSystem | undefined;
  beforeAll(async () => {
    [mujoco, nodeModule] = await Promise.all([loadMujoco(), loadNodeModule()]);
  });
  afterEach(() => {
    system?.dispose();
    system = undefined;
  });

  it.each([1, 3])(
    "finds and readies all six nodes within a bounded time (latency %i)",
    async (latency) => {
      system = await ArmSystem.create(mujoco, nodeModule, { bus: { latencyCycles: latency } });
      for (let k = 0; k < 200 && system.host.discovery.phase !== "ready"; k++) system.step();
      const discovery = system.host.discovery;
      expect(discovery.phase).toBe("ready");
      // Assign → announce → limits → READY state: four bus trips plus one cycle per node tick.
      expect(discovery.readyCycle).toBeLessThanOrEqual(4 * latency + 4);
      expect(discovery.nodes.map((n) => [n.id, n.announce.position, n.announce.gearRatio])).toEqual(
        KX6.joints.map((j, k) => [k + 1, k + 1, j.actuator.gearRatio]),
      );
    },
  );

  it("stays within the bus byte budget", async () => {
    system = await ArmSystem.create(mujoco, nodeModule);
    system.run(0.25);
    expect(system.bus.stats.overBudgetCycles).toBe(0);
    expect(system.bus.stats.peakBytesPerCycle).toBeLessThan(3125);
  });

  it("reports the nodes beyond an unplugged cable as lost", async () => {
    const lost: DiscoveredNode[] = [];
    system = await ArmSystem.create(mujoco, nodeModule, {
      host: { watchdogCycles: 20, events: { onLost: (_, node) => lost.push(node) } },
    });
    system.run(0.02);
    expect(system.host.discovery.phase).toBe("ready");
    system.bus.cutBefore(4);
    const cutCycle = system.cycle;
    while (lost.length < 3 && system.cycle < cutCycle + 100) system.step();
    expect(lost.map((n) => n.id)).toEqual([4, 5, 6]);
    expect(system.cycle - cutCycle).toBeLessThanOrEqual(20 + 3);
  });

  it("holds the arm while it starts up", async () => {
    system = await ArmSystem.create(mujoco, nodeModule);
    const start = system.plant.tcpPosition();
    system.run(0.5);
    const end = system.plant.tcpPosition();
    // The payload estimate is still converging after half a second (see gravity.test.ts).
    expect(Math.hypot(end[0] - start[0], end[1] - start[1], end[2] - start[2])).toBeLessThan(0.05);
  });
});
