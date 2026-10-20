import { afterEach, beforeAll, describe, expect, it } from "vitest";
import { pickAndPlace, figureEight } from "../host/tasks.ts";
import { KX6, KX6_HOME, withInitialAngles } from "../plant/kx6.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import { followTask, homePose, type MotionResult } from "../scenarios/motion.ts";
import type { InterpolationMode } from "../host/host.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import { ArmSystem } from "./armSystem.ts";

describe("joint-level interpolation against central interpolation (NX-10, H2)", () => {
  let mujoco: Mujoco;
  let nodeModule: WebAssembly.Module;
  const systems: ArmSystem[] = [];
  beforeAll(async () => {
    [mujoco, nodeModule] = await Promise.all([loadMujoco(), loadNodeModule()]);
  });
  afterEach(() => {
    systems.splice(0).forEach((s) => s.dispose());
  });

  async function run(mode: InterpolationMode, task: "figure" | "pick"): Promise<MotionResult> {
    const system = await ArmSystem.create(mujoco, nodeModule, {
      spec: withInitialAngles(KX6, KX6_HOME),
      seed: 7,
    });
    systems.push(system);
    system.run(4);
    const make =
      task === "figure"
        ? (s: ArmSystem) => figureEight(homePose(s))
        : (s: ArmSystem) => pickAndPlace(homePose(s));
    return followTask(system, make, { mode });
  }

  it.each(["figure", "pick"] as const)("%s: smoother and at least as accurate", async (task) => {
    const central = await run("central", task);
    const joint = await run("joint-level", task);
    expect(joint.jerkRms).toBeLessThan(central.jerkRms / 2.5);
    expect(joint.pathErrorRms).toBeLessThan(central.pathErrorRms);
  });

  it("sends the joints one goal per policy step instead of a stream of setpoints", async () => {
    const system = await ArmSystem.create(mujoco, nodeModule, {
      spec: withInitialAngles(KX6, KX6_HOME),
      seed: 7,
    });
    systems.push(system);
    system.run(4);
    followTask(system, (s) => figureEight(homePose(s), 0.15, 4, 1), { mode: "joint-level" });
    const load = system.host.load;
    expect(load.interpolationSteps).toBe(0);
    expect(load.goalFramesSent).toBe(6 * load.ikSolves);
  });
});
