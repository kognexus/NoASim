import { afterEach, beforeAll, describe, expect, it } from "vitest";
import { KX6, KX6_HOME, withInitialAngles } from "../plant/kx6.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import { commissionJoint, LOG_COLUMNS } from "../scenarios/commissioning.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import { ArmSystem } from "./armSystem.ts";

const SPEC = withInitialAngles(KX6, KX6_HOME);
const column = (name: (typeof LOG_COLUMNS)[number]) => LOG_COLUMNS.indexOf(name);

describe("commissioning run and dataset (NX-13)", () => {
  let mujoco: Mujoco;
  let nodeModule: WebAssembly.Module;
  const systems: ArmSystem[] = [];
  beforeAll(async () => {
    [mujoco, nodeModule] = await Promise.all([loadMujoco(), loadNodeModule()]);
  });
  afterEach(() => {
    systems.splice(0).forEach((s) => s.dispose());
  });

  async function arm(seed: number): Promise<ArmSystem> {
    const system = await ArmSystem.create(mujoco, nodeModule, { spec: SPEC, seed, actuators: {} });
    systems.push(system);
    system.run(2);
    return system;
  }

  it("moves only the commissioned joint, inside its limits, and returns to its start", async () => {
    const system = await arm(41);
    const before = SPEC.joints.map((_, i) => system.plant.outputAngle(i));
    let maxOther = 0;
    const run = commissionJoint(system, 2, 3, 7);
    for (let i = 0; i < SPEC.joints.length; i++) {
      if (i !== 2)
        maxOther = Math.max(maxOther, Math.abs(system.plant.outputAngle(i) - (before[i] ?? 0)));
    }
    expect(run.log).toHaveLength(3 * 4000);
    const q = run.log.map((r) => r[column("output_angle")] ?? 0);
    const v = run.log.map((r) => Math.abs(r[column("output_velocity")] ?? 0));
    expect(Math.max(...q) - Math.min(...q)).toBeGreaterThan(0.1);
    expect(Math.max(...q.map((x) => Math.abs(x - (before[2] ?? 0))))).toBeLessThan(0.4);
    expect(Math.max(...v)).toBeLessThan(1.6); // half the 3 rad/s limit, plus tracking error
    expect(maxOther).toBeLessThan(0.01);
    system.run(1);
    // Back at the start, up to the static error the hidden friction leaves under a PD with
    // nominal feed-forward (what a learned model is for).
    expect(Math.abs(system.plant.outputAngle(2) - (before[2] ?? 0))).toBeLessThan(0.03);
    expect(system.host.faults).toHaveLength(0);
  }, 60_000);

  it("is deterministic per seed, and logs the truth for every logged cycle", async () => {
    const a = commissionJoint(await arm(43), 4, 1, 9);
    const b = commissionJoint(await arm(43), 4, 1, 9);
    expect(a.log).toEqual(b.log);
    expect(a.truth.map((r) => r[0])).toEqual(a.log.map((r) => r[0]));
    const c = commissionJoint(await arm(43), 4, 1, 10);
    expect(c.log).not.toEqual(a.log);
  }, 60_000);
});
