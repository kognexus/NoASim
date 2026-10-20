import { afterEach, beforeAll, describe, expect, it } from "vitest";
import { KX6 } from "../plant/kx6.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import { hold, movingBase, tiltedGravity } from "../scenarios/gravity.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import { ArmSystem } from "./armSystem.ts";

const IMU = 1;
const LEVEL_BASE = 2;

describe("gravity compensation from each joint's IMU (NX-08, H9)", () => {
  let mujoco: Mujoco;
  let nodeModule: WebAssembly.Module;
  const systems: ArmSystem[] = [];
  beforeAll(async () => {
    [mujoco, nodeModule] = await Promise.all([loadMujoco(), loadNodeModule()]);
  });
  afterEach(() => {
    systems.splice(0).forEach((s) => s.dispose());
  });

  async function arm(gravity: number): Promise<ArmSystem> {
    const system = await ArmSystem.create(mujoco, nodeModule, { seed: 5 });
    system.host.applySettings({ gravity });
    systems.push(system);
    return system;
  }

  it("holds the shoulder and elbow within 1 % of rated torque, and finds the payload", async () => {
    const system = await arm(IMU);
    const result = hold(system, 5);
    for (const i of [1, 2]) {
      expect(Math.abs(result.torqueError[i]!)).toBeLessThan(
        0.01 * KX6.joints[i]!.actuator.ratedTorque,
      );
    }
    const chain = system.host.lastReceived.flatMap((f) =>
      "chain" in f && f.header.source === 6 ? [f.chain] : [],
    );
    // 3 kg on the flange; the nodes assume it sits on the flange, it really sits 5 cm beyond.
    expect(chain[0]?.payloadMass).toBeGreaterThan(2.7);
    expect(chain[0]?.payloadMass).toBeLessThan(3.3);
  });

  it("keeps the tool in place on a base tilted 15° sideways, unlike a level-base model", async () => {
    const drift = async (gravity: number) => {
      const system = await arm(gravity);
      const level = hold(system, 3);
      system.plant.setGravity(tiltedGravity(15));
      const tilted = hold(system, 3);
      return Math.hypot(...tilted.tool.map((v, k) => v - level.tool[k]!));
    };
    const [imu, levelBase] = [await drift(IMU), await drift(LEVEL_BASE)];
    expect(imu).toBeLessThan(0.001);
    expect(levelBase).toBeGreaterThan(5 * imu);
  });

  it("holds better than a level-base model while the base accelerates", async () => {
    const rms = async (gravity: number) => {
      const system = await arm(gravity);
      system.run(3);
      return movingBase(system, 2, 1, 4);
    };
    const [imu, levelBase] = [await rms(IMU), await rms(LEVEL_BASE)];
    expect(imu).toBeLessThan(levelBase / 2);
  });

  it("holds the arm still from power-on", async () => {
    const system = await arm(IMU);
    const start = system.plant.tcpPosition();
    system.run(5);
    const end = system.plant.tcpPosition();
    expect(Math.hypot(end[0] - start[0], end[1] - start[1], end[2] - start[2])).toBeLessThan(0.01);
  });
});
