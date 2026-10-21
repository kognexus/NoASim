import { beforeAll, describe, expect, it } from "vitest";
import { DampingMode } from "../bus/protocol.ts";
import { ArmSystem } from "../harness/armSystem.ts";
import { nodeConfigsFor } from "../harness/gains.ts";
import { tunedGains } from "../harness/tuned.ts";
import { BUS_CYCLE_HZ } from "../index.ts";
import { KX6_HOME, KX6L, withInitialAngles } from "../plant/kx6.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import { tapTest } from "../scenarios/tap.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import { figureEight } from "../host/tasks.ts";
import { followTask, homePose } from "../scenarios/motion.ts";
import { attachLqg, coordinatedBlocks, designLqg, type LoopModel, loopModel } from "./estimator.ts";

describe("estimator-based coordinated damping (NX-30)", () => {
  let mujoco: Mujoco;
  let nodeModule: WebAssembly.Module;
  const spec = withInitialAngles(KX6L, KX6_HOME);
  const gains = tunedGains("kx6l", "off");
  const configs = nodeConfigsFor(spec, BUS_CYCLE_HZ, gains);
  beforeAll(async () => {
    [mujoco, nodeModule] = await Promise.all([loadMujoco(), loadNodeModule()]);
  });

  async function held(seed: number, mode: "off" | "network" = "off"): Promise<ArmSystem> {
    const system = await ArmSystem.create(mujoco, nodeModule, {
      spec,
      gains: tunedGains("kx6l", mode),
      seed,
    });
    system.host.applySettings({ damping: DampingMode[mode] });
    system.run(5);
    return system;
  }

  /** Mean tool distance over a second after a vertical tap at the tool, m. */
  const mean = (d: Float64Array) =>
    d.slice(0, BUS_CYCLE_HZ).reduce((a, b) => a + b, 0) / BUS_CYCLE_HZ;

  it("models the held arm's tap response to within 10 % of the simulation", async () => {
    const system = await held(32);
    try {
      const model: LoopModel = loopModel(system, configs);
      const n = model.a.rows;
      const nq = model.nx / 2;
      // The model's response to 200 N for 10 ms, upwards.
      let z = new Float64Array(n);
      let sum = 0;
      for (let k = 0; k < BUS_CYCLE_HZ; k++) {
        const next = new Float64Array(n);
        for (let i = 0; i < n; i++) {
          let s = k < 40 ? 200 * model.force.get(i, 2) : 0;
          for (let j = 0; j < n; j++) s += model.a.get(i, j) * (z[j] ?? 0);
          next[i] = s;
        }
        z = next;
        let d2 = 0;
        for (let r = 0; r < 3; r++) {
          let p = 0;
          for (let j = 0; j < nq; j++) p += model.tool.get(r, j) * (z[j] ?? 0);
          d2 += p * p;
        }
        sum += Math.sqrt(d2);
      }
      const r = tapTest(system, { settleSeconds: 0, recordSeconds: 1 });
      expect(sum / BUS_CYCLE_HZ).toBeGreaterThan(0.9 * mean(r.deviation));
      expect(sum / BUS_CYCLE_HZ).toBeLessThan(1.1 * mean(r.deviation));
    } finally {
      system.dispose();
    }
  });

  it("on bus data, each joint seeing two either side, holds a tap far closer than network damping", async () => {
    const design = await held(31);
    const model = loopModel(design, configs);
    const lqg = designLqg(
      model,
      configs,
      { torqueWeight: 3e-5, speedWeight: 1e-4, forceNoise: 20 },
      2,
    );
    design.dispose();
    const tap = async (coordinated: boolean) => {
      const system = await held(33, coordinated ? "off" : "network");
      try {
        const r = tapTest(system, {
          settleSeconds: 0,
          recordSeconds: 2,
          beforeTap: (s) => {
            if (coordinated) attachLqg(s, configs, model, lqg, 0.25, 0.05);
          },
        });
        return { mean: mean(r.deviation), tail: r.deviation.slice(-BUS_CYCLE_HZ / 2) };
      } finally {
        system.dispose();
      }
    };
    const [network, coordinated] = [await tap(false), await tap(true)];
    expect(coordinated.mean).toBeLessThan(0.7 * network.mean);
    // And it settles: no oscillation left at the end.
    expect(Math.max(...coordinated.tail)).toBeLessThan(0.1e-3);
  });

  it("in the joints: a tap held closer than network damping, a task tracked as with it", async () => {
    const design = await held(31);
    const model = loopModel(design, configs);
    const lqg = designLqg(
      model,
      configs,
      { torqueWeight: 3e-5, speedWeight: 1e-4, forceNoise: 20 },
      2,
    );
    design.dispose();
    const blocks = coordinatedBlocks(model, lqg, configs, 2, 1 / BUS_CYCLE_HZ);
    const arm = async (coordinated: boolean) =>
      ArmSystem.create(mujoco, nodeModule, {
        spec,
        gains: tunedGains("kx6l", "network"),
        seed: 33,
        ...(coordinated ? { coordinated: blocks } : {}),
      });
    const mode = (coordinated: boolean) =>
      coordinated ? DampingMode.coordinated : DampingMode.network;
    const tap = async (coordinated: boolean) => {
      const system = await arm(coordinated);
      try {
        const r = tapTest(system, {
          recordSeconds: 2,
          beforeTap: (s) => s.host.applySettings({ damping: mode(coordinated) }),
        });
        return mean(r.deviation);
      } finally {
        system.dispose();
      }
    };
    expect(await tap(true)).toBeLessThan(0.7 * (await tap(false)));
    // Moving, the joints use network damping: the same tracking, within 10 µm (the
    // coordinated damper acted while the arm held before the task, so it starts a hair
    // apart).
    const track = async (coordinated: boolean) => {
      const system = await arm(coordinated);
      try {
        system.run(3);
        system.host.applySettings({ damping: mode(coordinated) });
        system.run(1);
        return followTask(system, (s) => figureEight(homePose(s)), { mode: "joint-level" })
          .pathErrorRms;
      } finally {
        system.dispose();
      }
    };
    expect(Math.abs((await track(true)) - (await track(false)))).toBeLessThan(1e-5);
  });
});
