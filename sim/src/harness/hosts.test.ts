import { afterEach, beforeAll, describe, expect, it } from "vitest";
import type { TaskOptions } from "../host/host.ts";
import { figureEight, pickAndPlace } from "../host/tasks.ts";
import { KX6, KX6_HOME, withInitialAngles } from "../plant/kx6.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import { followTask, homePose } from "../scenarios/motion.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import { ArmSystem } from "./armSystem.ts";

const VLA_COMPUTER: TaskOptions = { mode: "joint-level" };
const CLASSIC_1KHZ: TaskOptions = { mode: "classic", classic: { loopHz: 1000 } };
const CLASSIC_4KHZ: TaskOptions = { mode: "classic", classic: { loopHz: 4000 } };

describe("a policy computer or a classic controller drives the same joints (NX-22)", () => {
  let mujoco: Mujoco;
  let nodeModule: WebAssembly.Module;
  const systems: ArmSystem[] = [];
  beforeAll(async () => {
    [mujoco, nodeModule] = await Promise.all([loadMujoco(), loadNodeModule()]);
  });
  afterEach(() => {
    systems.splice(0).forEach((s) => s.dispose());
  });

  async function arm(): Promise<ArmSystem> {
    const system = await ArmSystem.create(mujoco, nodeModule, {
      spec: withInitialAngles(KX6, KX6_HOME),
      seed: 7,
    });
    systems.push(system);
    system.run(4);
    return system;
  }

  it.each([
    ["1 kHz", CLASSIC_1KHZ, 1000],
    ["4 kHz", CLASSIC_4KHZ, 4000],
  ] as const)(
    "a classic controller at %s streams setpoints the joints follow",
    async (_, options, hz) => {
      const system = await arm();
      const result = followTask(system, (s) => pickAndPlace(homePose(s)), options);
      expect(result.pathErrorRms).toBeLessThan(0.002);
      expect(system.host.faults).toHaveLength(0);
      // Six setpoints per controller tick, every tick, no policy involved.
      const load = system.host.load;
      expect(load.goalFramesSent % 6).toBe(0);
      expect(load.goalFramesSent / 6).toBeGreaterThan(0.95 * hz * 4.2);
    },
  );

  it("switches between the classic controller and the policy computer, task by task", async () => {
    const system = await arm();
    const sequence = [
      ["classic pick", CLASSIC_1KHZ, pickAndPlace],
      ["policy figure of eight", VLA_COMPUTER, figureEight],
      ["classic figure of eight", CLASSIC_4KHZ, figureEight],
      ["policy pick", VLA_COMPUTER, pickAndPlace],
    ] as const;
    for (const [name, options, task] of sequence) {
      const result = followTask(system, (s) => task(homePose(s)), options);
      // As accurate as each computer is on its own (a jump at the switch would show here).
      const limit = options.mode === "classic" ? 0.005 : 0.015;
      expect(result.pathErrorMax, name).toBeLessThan(limit);
      system.run(0.5);
    }
  });

  it("lets the classic controller take over mid-motion, smoothly", async () => {
    const system = await arm();
    system.host.startTask(figureEight(homePose(system)), system.cycle, VLA_COMPUTER);
    const speed = () => Math.hypot(...system.plant.tcpVelocity());
    /** Largest change of tool speed over 10 ms during `seconds`, m/s². */
    const roughest = (seconds: number) => {
      let previous = speed();
      let worst = 0;
      for (let k = 0; k < seconds * 100; k++) {
        system.run(0.01);
        const now = speed();
        worst = Math.max(worst, Math.abs(now - previous) / 0.01);
        previous = now;
      }
      return worst;
    };
    const whilePolicy = roughest(2);
    const takeover = system.host.startTask(
      figureEight(homePose(system)),
      system.cycle,
      CLASSIC_1KHZ,
    );
    const afterTakeover = roughest(3);
    expect(takeover.strategy.name).toBe("classic");
    expect(afterTakeover).toBeLessThan(whilePolicy);
    expect(system.host.faults).toHaveLength(0);
  });
});
