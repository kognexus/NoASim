import { beforeAll, describe, expect, it } from "vitest";
import { ArmSystem } from "../harness/armSystem.ts";
import { nodeConfigsFor } from "../harness/gains.ts";
import { tunedGains } from "../harness/tuned.ts";
import { BUS_CYCLE_HZ } from "../index.ts";
import { KX6_HOME, KX6L, withInitialAngles } from "../plant/kx6.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import { modeReach } from "./reach.ts";

describe("which modes a joint can reach (NX-28)", () => {
  let mujoco: Mujoco;
  let nodeModule: WebAssembly.Module;
  beforeAll(async () => {
    [mujoco, nodeModule] = await Promise.all([loadMujoco(), loadNodeModule()]);
  });

  it("on the light arm, joints reach the arm's low modes but not its link bending", async () => {
    const spec = withInitialAngles(KX6L, KX6_HOME);
    const gains = tunedGains("kx6l", "network");
    const system = await ArmSystem.create(mujoco, nodeModule, { spec, gains, seed: 31 });
    try {
      system.run(3);
      const modes = modeReach(system, nodeConfigsFor(spec, BUS_CYCLE_HZ, gains));
      const best = (m: (typeof modes)[number]) => Math.max(...m.local.map(Math.abs));
      // Every mode below 10 Hz: some joint adds at least 5 % damping on its own.
      const low = modes.filter((m) => m.hz < 10);
      expect(low.length).toBeGreaterThanOrEqual(3);
      for (const m of low) expect(best(m)).toBeGreaterThan(0.05);
      // A bending mode above 50 Hz that no joint reaches.
      const bending = modes.filter((m) => m.hz > 50 && m.shares.bend > 0.9);
      expect(bending.length).toBeGreaterThan(0);
      for (const m of bending) expect(best(m)).toBeLessThan(0.01);
    } finally {
      system.dispose();
    }
  });
});
