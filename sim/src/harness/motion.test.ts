import { afterEach, beforeAll, describe, expect, it } from "vitest";
import { figureEight } from "../host/tasks.ts";
import { KX6, KX6_HOME, withInitialAngles } from "../plant/kx6.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import { followTask, homePose } from "../scenarios/motion.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import { ArmSystem } from "./armSystem.ts";

describe("policy-driven motion with central interpolation (NX-09)", () => {
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

  it("follows a figure of eight and counts the host's work", async () => {
    system = await ArmSystem.create(mujoco, nodeModule, {
      spec: withInitialAngles(KX6, KX6_HOME),
      seed: 7,
    });
    system.run(4);
    const result = followTask(system, (s) => figureEight(homePose(s), 0.15, 4, 1), {
      mode: "central",
      central: { loopHz: 250, lateRate: 0.1 },
    });
    expect(result.pathErrorRms).toBeLessThan(0.015);
    const load = system.host.load;
    // One IK solve per policy step that arrived; six setpoints per host-loop tick.
    expect(load.ikSolves).toBeGreaterThan(50);
    expect(load.interpolationSteps).toBe(load.goalFramesSent);
    expect(load.interpolationSteps % 6).toBe(0);
    expect(load.interpolationSteps / 6).toBeGreaterThan(4.5 * 250 * 0.7);
  });
});
