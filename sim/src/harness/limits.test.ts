import { afterEach, beforeAll, describe, expect, it } from "vitest";
import { FaultCode, LimitKind, StatusFlags } from "../bus/protocol.ts";
import { fastReach } from "../host/tasks.ts";
import type { ArmSpec } from "../plant/armSpec.ts";
import { KX6, KX6_HOME, withInitialAngles } from "../plant/kx6.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import { homePose } from "../scenarios/motion.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import { ArmSystem, type ArmSystemOptions } from "./armSystem.ts";

/** KX-6 whose shoulder heats a thousand times faster than the real one. */
const HOT_SHOULDER: ArmSpec = {
  ...KX6,
  joints: KX6.joints.map((joint, i) => {
    if (i !== 1) return joint;
    const thermal = joint.actuator.electrical.thermal;
    return {
      ...joint,
      actuator: {
        ...joint.actuator,
        electrical: {
          ...joint.actuator.electrical,
          thermal: {
            ...thermal,
            windingCapacity: thermal.windingCapacity / 1000,
            housingCapacity: thermal.housingCapacity / 1000,
          },
        },
      },
    };
  }),
};

describe("limits and announcements (NX-18)", () => {
  let mujoco: Mujoco;
  let nodeModule: WebAssembly.Module;
  const systems: ArmSystem[] = [];
  beforeAll(async () => {
    [mujoco, nodeModule] = await Promise.all([loadMujoco(), loadNodeModule()]);
  });
  afterEach(() => {
    systems.splice(0).forEach((s) => s.dispose());
  });

  async function arm(options: Partial<ArmSystemOptions>): Promise<ArmSystem> {
    const system = await ArmSystem.create(mujoco, nodeModule, { seed: 71, ...options });
    systems.push(system);
    return system;
  }

  it("warns before it derates a hot joint, and announces the derating (B7)", async () => {
    // The shoulder holds the stretched arm at about rated torque, and its limit is low.
    const system = await arm({ spec: HOT_SHOULDER, host: { limits: { maxTemperature: 70 } } });
    let warnedAt: number | undefined;
    let deratedAt: number | undefined;
    for (let k = 0; k < 30 * 4000 && deratedAt === undefined; k++) {
      system.step();
      const state = system.host.lastReceived.find((f) => "state" in f && f.header.source === 2);
      if (state && "state" in state) {
        if (warnedAt === undefined && (state.state.status & StatusFlags.thermalWarning) !== 0)
          warnedAt = system.cycle;
        if ((state.state.status & StatusFlags.derating) !== 0) deratedAt = system.cycle;
      }
    }
    expect(warnedAt).toBeDefined();
    expect(deratedAt).toBeDefined();
    expect(deratedAt! - warnedAt!).toBeGreaterThan(400); // at least 0.1 s of warning first
    system.run(0.01);
    expect(system.host.faults).toContainEqual(
      expect.objectContaining({ node: 2, code: FaultCode.limit, value: LimitKind.thermal }),
    );
  });

  it("holds the host's speed limit during a fast move, and reports it (B6)", async () => {
    const system = await arm({
      spec: withInitialAngles(KX6, KX6_HOME),
      host: { limits: { maxVelocity: 0.3 } },
    });
    system.run(5);
    const task = fastReach(homePose(system));
    system.host.startTask(task, system.cycle, { mode: "joint-level" });
    // Joint speed averaged over about 20 ms (vibration may peak briefly above it).
    const speed = [0, 0, 0, 0, 0, 0];
    const alpha = 1 / (1 + 4000 / (2 * Math.PI * 8));
    let fastest = 0;
    for (let k = 0; k < (task.duration + 0.5) * 4000; k++) {
      system.step();
      for (let i = 0; i < 6; i++) {
        speed[i] = speed[i]! + alpha * (Math.abs(system.plant.outputVelocity(i)) - speed[i]!);
        fastest = Math.max(fastest, speed[i]!);
      }
    }
    // The reference never exceeds the limit; the servo overshoots it by about a third.
    expect(fastest).toBeLessThan(0.3 * 1.5);
    expect(
      system.host.faults.some((f) => f.code === FaultCode.limit && f.value === LimitKind.speed),
    ).toBe(true);
  });
});
