import { afterEach, beforeAll, describe, expect, it } from "vitest";
import { FaultCode } from "../../sim/src/bus/protocol.ts";
import { loadMujoco, type Mujoco } from "../../sim/src/plant/mujoco.ts";
import { loadNodeModule } from "../../sim/src/testing/fixtures.ts";
import { ArmRig, BUS_CYCLE_HZ, DampingMode, DEFAULT_SHARED, TwinRig } from "./rig.ts";
import { VibrationMeter } from "../../sim/src/analysis/vibration.ts";

describe("the demo's twin arms (NX-20)", () => {
  let mujoco: Mujoco;
  let nodeModule: WebAssembly.Module;
  let twin: TwinRig | undefined;
  beforeAll(async () => {
    [mujoco, nodeModule] = await Promise.all([loadMujoco(), loadNodeModule()]);
  });
  afterEach(() => {
    twin?.dispose();
    twin = undefined;
  });

  async function twinArms(): Promise<TwinRig> {
    const bus = { latencyCycles: 1 };
    const [left, right] = await Promise.all(
      [DampingMode.network, DampingMode.off].map((damping) =>
        ArmRig.create(mujoco, nodeModule, DEFAULT_SHARED, { damping, model: 0 }, undefined, bus),
      ),
    );
    twin = new TwinRig(left!, right!);
    return twin;
  }

  it("under 15 Hz policy goals, network damping shakes no joint more than none (NX-25)", async () => {
    const vibration = async (damping: DampingMode): Promise<number[]> => {
      const arm = await ArmRig.create(
        mujoco,
        nodeModule,
        DEFAULT_SHARED,
        { damping, model: 0 },
        undefined,
        { latencyCycles: 1 },
      );
      const meter = new VibrationMeter(arm.system.host, arm.spec.joints.length);
      arm.startTask("eight", "vla");
      const sums = new Array<number>(arm.spec.joints.length).fill(0);
      for (let k = 0; k < 5 * BUS_CYCLE_HZ; k++) {
        arm.run(1);
        meter.observe();
        if (k > BUS_CYCLE_HZ && k % 40 === 0) {
          meter.levels().forEach((v, j) => (sums[j]! += v * v));
        }
      }
      arm.dispose();
      return sums.map(Math.sqrt);
    };
    const [none, network] = [
      await vibration(DampingMode.off),
      await vibration(DampingMode.network),
    ];
    // Joints 2 to 6 (the base joint only turns); each 9–13 % below none when measured.
    for (let j = 1; j < none.length; j++) expect(network[j]!).toBeLessThan(0.97 * none[j]!);
  });

  it("builds the out-of-family test set from the sealed family and names it plainly", async () => {
    const arm = await ArmRig.create(
      mujoco,
      nodeModule,
      { ...DEFAULT_SHARED, arm: "kx6", unit: "kx6-sealed-s900" },
      { damping: DampingMode.off, model: 0 },
      undefined,
      { latencyCycles: 1 },
    );
    expect(new Set(arm.actuatorDescriptions())).toEqual(
      new Set(["sticky friction with memory, eccentric gear"]),
    );
    arm.dispose();
  });

  it("taps both arms at the same cycle; the networked arm moves less", async () => {
    const t = await twinArms();
    expect(t.arms[0].cycle).toBe(t.arms[1].cycle);
    // Distance from where each tool was just before the tap: how much the tap moved it.
    const before = t.arms.map((arm) => arm.system.plant.tcpPosition());
    t.both((arm) => arm.tap([0, 0.6, -0.8]));
    const peak = [0, 0];
    const sum = [0, 0];
    for (let k = 0; k < 0.5 * BUS_CYCLE_HZ; k += 40) {
      t.run(40);
      t.arms.forEach((arm, i) => {
        const p = arm.system.plant.tcpPosition();
        const q = before[i] ?? p;
        const d = Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
        peak[i] = Math.max(peak[i]!, d);
        sum[i]! += d;
      });
    }
    expect(peak[1]).toBeGreaterThan(5e-3); // the same tap reached both arms
    // The first swing is set by inertia; the network shows in how fast it dies out.
    expect(sum[0]!).toBeLessThan(0.6 * sum[1]!); // mean distance over the first half second
    // Nothing reacted as to an impact: reaction is off by default.
    for (const arm of t.arms) expect(arm.system.host.impacts).toHaveLength(0);
  });

  it("torque ripple in one joint is claimed by that joint on both arms", async () => {
    const t = await twinArms();
    t.both((arm) => arm.apply({ damping: DampingMode.networkWithSource, model: 0 }));
    t.both((arm) => arm.startTask("eight"));
    t.both((arm) => arm.setRipple(2, "fixable"));
    t.run(4 * BUS_CYCLE_HZ);
    for (const arm of t.arms) {
      const claimants = new Set(
        arm.system.host.disturbances.filter((d) => d.node === d.source).map((d) => d.node),
      );
      expect([...claimants]).toEqual([3]);
    }
  });

  it("a collision with reaction on makes both arms react; E-STOP cuts torque", async () => {
    const bus = { latencyCycles: 1 };
    const shared = { ...DEFAULT_SHARED, reactToImpacts: true };
    const [left, right] = await Promise.all(
      [DampingMode.network, DampingMode.off].map((damping) =>
        ArmRig.create(mujoco, nodeModule, shared, { damping, model: 0 }, undefined, bus),
      ),
    );
    const t = (twin = new TwinRig(left!, right!));
    t.both((arm) => arm.push([0, 1, 0]));
    t.run(0.6 * BUS_CYCLE_HZ);
    for (const arm of t.arms) {
      expect(arm.system.host.impacts.length).toBeGreaterThan(0);
      expect(arm.system.host.faults.some((f) => f.code === FaultCode.impact)).toBe(true);
    }
    t.both((arm) => arm.emergencyStop());
    t.run(40);
    for (const arm of t.arms) {
      expect(arm.system.safeTorqueOff).toBe(true);
      expect(arm.system.joints.every((j) => j.actuatorOutput.driveTorque === 0)).toBe(true);
    }
  });
});
