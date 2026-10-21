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
import { IdealController } from "./ideal.ts";

describe("the ideal reference controller (NX-27)", () => {
  let mujoco: Mujoco;
  let nodeModule: WebAssembly.Module;
  beforeAll(async () => {
    [mujoco, nodeModule] = await Promise.all([loadMujoco(), loadNodeModule()]);
  });

  const spec = withInitialAngles(KX6L, KX6_HOME);
  const gains = tunedGains("kx6l", "off");

  /** Mean tool distance over the second after a tap, mm, and the controller's peak share. */
  async function tap(ideal: boolean): Promise<{ meanMm: number; peakShare: number }> {
    const system = await ArmSystem.create(mujoco, nodeModule, { spec, gains, seed: 31 });
    let controller: IdealController | undefined;
    try {
      const r = tapTest(system, {
        recordSeconds: 1,
        beforeTap: (s) => {
          s.host.applySettings({ damping: DampingMode.off });
          if (!ideal) return;
          controller = IdealController.design(s, nodeConfigsFor(spec, BUS_CYCLE_HZ, gains), {
            torqueWeight: 1e-6,
            speedWeight: 1e-4,
            torqueShare: 0.25,
            tankJ: 0.05,
          });
          controller.attach(s);
        },
      });
      const mean = r.deviation.reduce((a, b) => a + b, 0) / r.deviation.length;
      return { meanMm: 1e3 * mean, peakShare: controller?.peakShare ?? 0 };
    } finally {
      system.dispose();
    }
  }

  it("holds the arm far closer to rest after a tap, within the joints' torque budget", async () => {
    const [none, ideal] = [await tap(false), await tap(true)];
    expect(ideal.meanMm).toBeLessThan(0.3 * none.meanMm);
    expect(ideal.peakShare).toBeGreaterThan(0);
    expect(ideal.peakShare).toBeLessThanOrEqual(1);
  });

  it("is deterministic", async () => {
    const [a, b] = [await tap(true), await tap(true)];
    expect(a).toEqual(b);
  });
});
