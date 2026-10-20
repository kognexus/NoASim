import { afterEach, beforeAll, describe, expect, it } from "vitest";
import { DampingMode } from "../bus/protocol.ts";
import { KX6L } from "../plant/kx6.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import { dampingWork } from "../scenarios/energy.ts";
import { housingShake } from "../scenarios/rest.ts";
import { tapTest } from "../scenarios/tap.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import { ArmSystem, type ArmSystemOptions } from "./armSystem.ts";
import { tunedGains, type TunedMode } from "./tuned.ts";

/** Seeds not used by the tuner. */
const SEED = 31;
const QUIET = 9.81 * 1.05;

describe("network damping on the lightweight arm (NX-11)", () => {
  let mujoco: Mujoco;
  let nodeModule: WebAssembly.Module;
  const systems: ArmSystem[] = [];
  beforeAll(async () => {
    [mujoco, nodeModule] = await Promise.all([loadMujoco(), loadNodeModule()]);
  });
  afterEach(() => {
    systems.splice(0).forEach((s) => s.dispose());
  });

  async function arm(mode: TunedMode, extra: Partial<ArmSystemOptions> = {}): Promise<ArmSystem> {
    const system = await ArmSystem.create(mujoco, nodeModule, {
      spec: KX6L,
      gains: tunedGains("kx6l", mode),
      seed: SEED,
      ...extra,
    });
    systems.push(system);
    return system;
  }

  const tapIn = async (mode: TunedMode) => {
    const system = await arm(mode);
    const result = tapTest(system, {
      beforeTap: (s) => s.host.applySettings({ damping: DampingMode[mode] }),
    });
    return { ...result, shake: housingShake(system, 1) };
  };

  it("settles a tap faster than joint control alone and than joint-local damping, quietly (H1)", async () => {
    const [off, local, network] = [
      await tapIn("off"),
      await tapIn("local"),
      await tapIn("network"),
    ];
    expect(network.settlingTime).toBeLessThan(off.settlingTime / 2);
    expect(network.settlingTime).toBeLessThan(local.settlingTime);
    expect(network.noise).toBeLessThan(1e-4);
    expect(network.shake).toBeLessThan(QUIET);
  });

  it("never puts more energy into the arm than its tanks hold, even with a slow lossy bus (H6)", async () => {
    const system = await arm("network", { bus: { latencyCycles: 10, dropRate: 0.2 } });
    const tap = tapTest(system, {
      beforeTap: (s) => s.host.applySettings({ damping: DampingMode.network }),
    });
    expect(Number.isFinite(tap.settlingTime)).toBe(true);
    system.plant.setToolForce([0, 0, 200]);
    const during = dampingWork(system, 0.01);
    system.plant.setToolForce([0, 0, 0]);
    const after = dampingWork(system, 2);
    const tank = 0.05;
    expect(during.maxWork).toBeLessThan(tank * 1.1);
    expect(after.maxWork + Math.max(...during.finalWork)).toBeLessThan(tank * 1.1);
    expect(housingShake(system, 1)).toBeLessThan(QUIET);
  });

  it("falls back to joint-local behaviour when a neighbour goes silent", async () => {
    const system = await arm("network");
    system.run(5);
    system.host.applySettings({ damping: DampingMode.network });
    system.run(0.5);
    system.bus.cutBefore(4); // joint 3 loses its outer neighbour; joints 4–6 lose the bus
    const tap = tapTest(system, { settleSeconds: 0.5 });
    expect(Number.isFinite(tap.settlingTime)).toBe(true);
  });

  it("relays the rate of the link tip one joint further in", async () => {
    const system = await arm("network");
    system.run(1);
    system.plant.setToolForce([0, 200, 0]);
    system.run(0.02);
    system.plant.setToolForce([0, 0, 0]);
    system.step();
    const states = new Map(
      system.host.lastReceived.flatMap((f) =>
        "state" in f ? [[f.header.source, f.state] as const] : [],
      ),
    );
    // Joint 2 relays joint 3's gyro (the tip of link 2); it must be moving after the tap.
    const relayed = states.get(2)?.outerRate ?? [0, 0, 0];
    expect(Math.hypot(...relayed)).toBeGreaterThan(1e-3);
  });
});
