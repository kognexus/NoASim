import { afterEach, beforeAll, describe, expect, it } from "vitest";
import { decodeFrame, decodeRecords, StatusFlags } from "../bus/protocol.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import { tapTest } from "../scenarios/tap.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import { ArmSystem } from "./armSystem.ts";

describe("joint control on the arm (NX-07)", () => {
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

  it("estimates gear twist from the dual encoders to within a few encoder counts", async () => {
    system = await ArmSystem.create(mujoco, nodeModule, { seed: 4 });
    system.run(1);
    // Truth as each node sampled it, keyed by bus cycle (frames arrive a cycle later).
    const truth = new Map<number, number[]>();
    for (let k = 0; k < 4; k++) {
      truth.set(
        system.cycle,
        [0, 1, 2, 3, 4, 5].map((i) => system!.plant.gearTwist(i)),
      );
      system.step();
    }
    const states = system.host.lastReceived.flatMap((f) => ("state" in f ? [f] : []));
    expect(states).toHaveLength(6);
    for (const frame of states) {
      const sampled = truth.get(frame.header.cycle)?.[frame.header.source - 1];
      expect(sampled).toBeDefined();
      // Motor-side encoder: 14 bits on the shaft, 9× finer at the output → 4.3e-5 rad.
      expect(Math.abs(frame.state.gearTwist - (sampled ?? 0))).toBeLessThan(1.5e-4);
    }
  });

  it("settles a tap with joint control alone (the network-off baseline)", async () => {
    system = await ArmSystem.create(mujoco, nodeModule, { seed: 4 });
    const result = tapTest(system);
    expect(result.noise).toBeLessThan(1e-4);
    expect(result.settlingTime).toBeLessThan(1.5);
  });

  it("switches damping modes at run time and reports it", async () => {
    system = await ArmSystem.create(mujoco, nodeModule, { seed: 4 });
    system.run(0.5);
    system.host.applySettings({ damping: 2 });
    system.plant.setToolForce([0, 0, 200]);
    system.run(0.01);
    system.plant.setToolForce([0, 0, 0]);
    let damping = false;
    for (let k = 0; k < 400 && !damping; k++) {
      system.step();
      damping = system.host.lastReceived.some(
        (f) => "state" in f && (f.state.status & StatusFlags.networkDamping) !== 0,
      );
    }
    expect(damping).toBe(true);
    expect(decodeRecords(new Uint8Array()).map(decodeFrame)).toEqual([]);
  });
});
