import { beforeAll, describe, expect, it } from "vitest";
import { ArmSystem } from "../harness/armSystem.ts";
import { nodeConfigsFor } from "../harness/gains.ts";
import { tunedGains } from "../harness/tuned.ts";
import { BUS_CYCLE_HZ } from "../index.ts";
import { KX6_HOME, KX6L, withInitialAngles } from "../plant/kx6.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import { coordinateOwners } from "./ideal.ts";
import { lyapunov, designStructured } from "./structured.ts";
import { Matrix } from "../util/matrix.ts";

describe("a damping gain designed for a structure (NX-29)", () => {
  let mujoco: Mujoco;
  let nodeModule: WebAssembly.Module;
  beforeAll(async () => {
    [mujoco, nodeModule] = await Promise.all([loadMujoco(), loadNodeModule()]);
  });

  it("solves the discrete Lyapunov equation and refuses an unstable matrix", () => {
    const a = Matrix.from([
      [0.9, 0.1],
      [0, 0.8],
    ]);
    const p = lyapunov(a, Matrix.identity(2));
    const back = a.transpose().times(p).times(a).plus(Matrix.identity(2));
    for (let i = 0; i < 2; i++)
      for (let j = 0; j < 2; j++) expect(back.get(i, j)).toBeCloseTo(p.get(i, j), 9);
    expect(Number.isFinite(lyapunov(Matrix.from([[1.01]]), Matrix.identity(1)).get(0, 0))).toBe(
      false,
    );
  });

  it("with two joints either side, gets within 5 % of the full gain's cost, inside the structure", async () => {
    const spec = withInitialAngles(KX6L, KX6_HOME);
    const gains = tunedGains("kx6l", "off");
    const system = await ArmSystem.create(mujoco, nodeModule, { spec, gains, seed: 31 });
    try {
      system.run(3);
      const configs = nodeConfigsFor(spec, BUS_CYCLE_HZ, gains);
      const result = designStructured(system, configs, {
        torqueWeight: 1e-6,
        speedWeight: 1e-4,
        reach: 2,
      });
      expect(result.cost.structured).toBeLessThan(1.05 * result.cost.full);
      expect(result.cost.structured).toBeLessThan(0.2 * result.cost.none);
      const owner = coordinateOwners(system, configs.length);
      const nq = owner.length;
      for (let i = 0; i < result.gain.rows; i++)
        for (let k = 0; k < result.gain.cols; k++) {
          const j = owner[k % nq] ?? -1;
          if (j < 0 || Math.abs(j - i) > 2) expect(result.gain.get(i, k)).toBe(0);
        }
    } finally {
      system.dispose();
    }
  });
});
