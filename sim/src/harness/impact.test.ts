import { afterEach, beforeAll, describe, expect, it } from "vitest";
import { FaultCode, ImpactPolicy, StatusFlags } from "../bus/protocol.ts";
import { fastReach, figureEight, pickAndPlace } from "../host/tasks.ts";
import { flangePose } from "../host/ik.ts";
import { KX6, KX6_HOME, withInitialAngles } from "../plant/kx6.ts";
import type { ArmSpec } from "../plant/armSpec.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import { impactRun, sideObstacle, sideReach } from "../scenarios/impact.ts";
import { homePose } from "../scenarios/motion.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import { ArmSystem, type ArmSystemOptions } from "./armSystem.ts";

const SPEC = withInitialAngles(KX6, KX6_HOME);
const HOME = flangePose(SPEC, KX6_HOME).position;
const withBrakes = (spec: ArmSpec): ArmSpec => ({
  ...spec,
  joints: spec.joints.map((j) => ({ ...j, actuator: { ...j.actuator, brake: true } })),
});

describe("impact detection and reaction (NX-16, NX-17)", () => {
  let mujoco: Mujoco;
  let nodeModule: WebAssembly.Module;
  const systems: ArmSystem[] = [];
  beforeAll(async () => {
    [mujoco, nodeModule] = await Promise.all([loadMujoco(), loadNodeModule()]);
  });
  afterEach(() => {
    systems.splice(0).forEach((s) => s.dispose());
  });

  async function armed(
    policy: number,
    options: Partial<ArmSystemOptions> = {},
  ): Promise<ArmSystem> {
    const system = await ArmSystem.create(mujoco, nodeModule, {
      spec: SPEC,
      seed: 61,
      obstacle: sideObstacle(HOME),
      ...options,
    });
    systems.push(system);
    system.run(5);
    system.host.applySettings({ impactConfirm: 1, impactPolicy: policy });
    system.run(0.2);
    return system;
  }

  it("every joint reacts within two bus cycles of the detection (H3a)", async () => {
    const system = await armed(ImpactPolicy.retreatAndHold);
    const result = impactRun(system, sideReach(system));
    expect(result.detectionCycle).toBeDefined();
    expect(result.reactionCycles.every((c) => c !== undefined && c <= 2)).toBe(true);
  });

  it.each([
    ["figure eight", (s: ArmSystem) => figureEight(homePose(s))],
    ["pick and place", (s: ArmSystem) => pickAndPlace(homePose(s))],
    ["fast reach", (s: ArmSystem) => fastReach(homePose(s))],
  ] as const)("raises no false alarm during a %s (H4)", async (_, task) => {
    const system = await ArmSystem.create(mujoco, nodeModule, { spec: SPEC, seed: 62 });
    systems.push(system);
    system.run(5);
    system.host.applySettings({ impactConfirm: 1 });
    const t = task(system);
    system.host.startTask(t, system.cycle, { mode: "joint-level" });
    system.run(t.duration + 0.5);
    expect(system.host.impacts).toEqual([]);
  });

  it("P1 retreats to the pre-impact pose, together, and holds until acknowledged (H3b)", async () => {
    const system = await armed(ImpactPolicy.retreatAndHold);
    const result = impactRun(system, sideReach(system));
    const snapshot = result.snapshotTool!;
    expect(Math.hypot(...result.finalTool.map((v, k) => v - snapshot[k]!))).toBeLessThan(0.001);
    const holding = result.holdingCycles.filter((c): c is number => c !== undefined);
    expect(holding).toHaveLength(6);
    expect(Math.max(...holding) - Math.min(...holding)).toBeLessThanOrEqual(1);
    expect(system.host.faults.filter((f) => f.code === FaultCode.impact)).toHaveLength(6);
    // Goals are ignored while holding…
    const held = system.plant.tcpPosition();
    system.host.startTask(sideReach(system), system.cycle, { mode: "joint-level" });
    system.run(1);
    expect(Math.hypot(...system.plant.tcpPosition().map((v, k) => v - held[k]!))).toBeLessThan(
      0.002,
    );
    // …until the host acknowledges.
    system.host.acknowledge();
    system.run(0.05);
    const states = system.host.lastReceived.flatMap((f) => ("state" in f ? [f.state] : []));
    expect(states.every((s) => (s.status & StatusFlags.holding) === 0)).toBe(true);
  });

  it("P1 presses on the obstacle less than stopping in place does (H3b)", async () => {
    const retreat = await armed(ImpactPolicy.retreatAndHold);
    const p1 = impactRun(retreat, sideReach(retreat));
    const stop = await armed(ImpactPolicy.stopInPlace);
    const stiff = impactRun(stop, sideReach(stop));
    expect(p1.meanForceAfterDetection).toBeLessThan(stiff.meanForceAfterDetection);
    // Stopping in place keeps pressing; retreating lets go.
    expect(stiff.finalForce).toBeGreaterThan(20);
    expect(p1.finalForce).toBeLessThan(1);
  });

  it("P2 stops, brakes, then cuts torque, and the brakes hold the arm (H3c)", async () => {
    const system = await armed(ImpactPolicy.eStop, { spec: withBrakes(SPEC) });
    const result = impactRun(system, sideReach(system), { seconds: 2 });
    expect(result.detectionCycle).toBeDefined();
    expect(system.safeTorqueOff).toBe(true);
    const z = system.plant.tcpPosition()[2];
    system.run(1);
    expect(Math.abs(system.plant.tcpPosition()[2] - z)).toBeLessThan(0.002);
  });

  it("refuses P2 on a brakeless arm, joint by joint (H3d)", async () => {
    const system = await armed(ImpactPolicy.eStop);
    system.run(0.2);
    const rejected = new Set(
      system.host.faults.filter((f) => f.code === FaultCode.configRejected).map((f) => f.node),
    );
    // Every joint that carries gravity load in this pose refuses. Joints 1 and 5 turn about
    // vertical axes, and joint 6's payload hangs along its axis with the tool pointing down.
    expect([...rejected].sort()).toEqual([2, 3, 4]);
  });

  it("holds the arm without brakes when the host falls silent (B8)", async () => {
    const system = await armed(ImpactPolicy.retreatAndHold);
    const before = system.plant.tcpPosition();
    system.host.heartbeat = false;
    system.run(2);
    expect(system.host.faults.some((f) => f.code === FaultCode.commLoss)).toBe(true);
    expect(system.safeTorqueOff).toBe(false);
    expect(Math.hypot(...system.plant.tcpPosition().map((v, k) => v - before[k]!))).toBeLessThan(
      0.005,
    );
  });
});
