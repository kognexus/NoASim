/**
 * H3a–H3d and H4: impact reaction. The KX-6 reaches sideways into a spring-mounted block
 * with impact reaction armed (any joint triggers); and runs normal tasks with reaction
 * armed, to count false stops.
 */
import { FaultCode, ImpactPolicy, StatusFlags } from "../../../sim/src/bus/protocol.ts";
import type { ArmSystem, ArmSystemOptions } from "../../../sim/src/harness/armSystem.ts";
import { flangePose } from "../../../sim/src/host/ik.ts";
import { fastReach, figureEight, pickAndPlace, type Task } from "../../../sim/src/host/tasks.ts";
import { BUS_CYCLE_HZ } from "../../../sim/src/index.ts";
import type { ArmSpec, Vec3 } from "../../../sim/src/plant/armSpec.ts";
import { KX6, KX6_HOME, withInitialAngles } from "../../../sim/src/plant/kx6.ts";
import { impactRun, sideObstacle, sideReach } from "../../../sim/src/scenarios/impact.ts";
import { homePose } from "../../../sim/src/scenarios/motion.ts";
import { withArm } from "../runner.ts";
import type { Finding, ProveContext } from "./findings.ts";

const SPEC = withInitialAngles(KX6, KX6_HOME);
const HOME = flangePose(SPEC, KX6_HOME).position;
const withBrakes = (spec: ArmSpec): ArmSpec => ({
  ...spec,
  joints: spec.joints.map((j) => ({ ...j, actuator: { ...j.actuator, brake: true } })),
});
const distance = (a: Vec3, b: Vec3) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

/** An arm in front of the block, settled, with reaction armed under `policy`. */
function armed<T>(
  ctx: ProveContext,
  policy: number,
  seed: number,
  run: (system: ArmSystem) => T,
  options: Partial<ArmSystemOptions> = {},
): Promise<T> {
  return withArm(
    ctx.lab,
    { spec: SPEC, seed, obstacle: sideObstacle(HOME), ...options },
    (system) => {
      system.run(5);
      system.host.applySettings({ impactConfirm: 1, impactPolicy: policy });
      system.run(0.2);
      return run(system);
    },
  );
}

export async function h3h4(ctx: ProveContext): Promise<Finding[]> {
  const seeds = ctx.quick ? [61] : [61, 63, 65];
  /** The seed of the single-run measurements (H3b–H3d). */
  const first = 61;
  const findings: Finding[] = [];

  // H3a: reaction latency of every joint after detection.
  let worstReaction = 0;
  let allReacted = true;
  for (const seed of seeds) {
    const r = await armed(ctx, ImpactPolicy.retreatAndHold, seed, (s) =>
      impactRun(s, sideReach(s)),
    );
    allReacted &&= r.detectionCycle !== undefined && r.reactionCycles.every((c) => c !== undefined);
    worstReaction = Math.max(worstReaction, ...r.reactionCycles.map((c) => c ?? Infinity));
  }
  findings.push({
    id: "H3a",
    title: "Every joint reacts to an impact fast",
    target: "All joints react within 2 bus cycles of detection",
    measured: `slowest joint ${worstReaction} cycle${worstReaction === 1 ? "" : "s"} after detection (${seeds.length} collisions)`,
    verdict: allReacted && worstReaction <= 2 ? "met" : "not met",
    detail:
      "The detecting joint stops at once and broadcasts the impact; every other joint reacts when the frame arrives (one cycle of bus latency).",
  });

  // H3b: P1 retreat and hold.
  const p1 = await armed(ctx, ImpactPolicy.retreatAndHold, first, (system) => {
    const r = impactRun(system, sideReach(system));
    const holding = r.holdingCycles.filter((c): c is number => c !== undefined);
    const held = system.plant.tcpPosition();
    system.host.startTask(sideReach(system), system.cycle, { mode: "joint-level" });
    system.run(1);
    const movedBeforeAck = distance(system.plant.tcpPosition(), held);
    system.host.acknowledge();
    system.run(0.05);
    const released = system.host.lastReceived.every(
      (f) => !("state" in f) || (f.state.status & StatusFlags.holding) === 0,
    );
    return {
      poseError: r.snapshotTool ? distance(r.finalTool, r.snapshotTool) : Infinity,
      spread: holding.length === 6 ? Math.max(...holding) - Math.min(...holding) : Infinity,
      meanForce: r.meanForceAfterDetection,
      finalForce: r.finalForce,
      faults: system.host.faults.filter((f) => f.code === FaultCode.impact).length,
      movedBeforeAck,
      released,
    };
  });
  const stop = await armed(ctx, ImpactPolicy.stopInPlace, first, (s) => impactRun(s, sideReach(s)));
  const p1Met =
    p1.poseError < 0.001 &&
    p1.spread <= 1 &&
    p1.meanForce < stop.meanForceAfterDetection &&
    p1.movedBeforeAck < 0.002 &&
    p1.released;
  findings.push({
    id: "H3b",
    title: "P1 retreat and hold works without brakes",
    target:
      "Pose within 1 mm, arrival spread ≤ 1 cycle, less contact force than stopping in place, no motion before ACK",
    measured: `pose ${(p1.poseError * 1e3).toFixed(2)} mm, spread ${p1.spread} cycle, force ${p1.meanForce.toFixed(1)} N vs ${stop.meanForceAfterDetection.toFixed(1)} N, ${(p1.movedBeforeAck * 1e3).toFixed(2)} mm before ACK`,
    verdict: p1Met ? "met" : "not met",
    detail: [
      "| Measure | Retreat and hold | Stop in place |",
      "|---|---|---|",
      `| Mean force on the block over 0.5 s after detection | ${p1.meanForce.toFixed(1)} N | ${stop.meanForceAfterDetection.toFixed(1)} N |`,
      `| Force at the end | ${p1.finalForce.toFixed(1)} N | ${stop.finalForce.toFixed(1)} N |`,
      "",
      `Tool back at the pre-impact snapshot within ${(p1.poseError * 1e3).toFixed(2)} mm; all six joints holding within ${p1.spread} cycle of each other; ${p1.faults} impact faults reported to the host; goals ignored until the ACK (${(p1.movedBeforeAck * 1e3).toFixed(2)} mm of motion), joints released after it: ${p1.released ? "yes" : "no"}.`,
    ].join("\n"),
  });

  // H3c: P2 with brakes. Stepped by hand to time the brake and torque-off.
  const p2 = await armed(
    ctx,
    ImpactPolicy.eStop,
    first,
    (system) => {
      system.host.startTask(sideReach(system), system.cycle, { mode: "joint-level" });
      let detected: number | undefined;
      let braked: number | undefined;
      let torqueOff: number | undefined;
      for (let k = 0; k < 2 * BUS_CYCLE_HZ; k++) {
        system.step();
        if (detected === undefined && system.host.impacts.length > 0) detected = system.cycle;
        if (braked === undefined && system.joints.some((j) => j.brake)) braked = system.cycle;
        if (torqueOff === undefined && system.safeTorqueOff) torqueOff = system.cycle;
      }
      const z = system.plant.tcpPosition()[2];
      system.run(1);
      return { detected, braked, torqueOff, sag: Math.abs(system.plant.tcpPosition()[2] - z) };
    },
    { spec: withBrakes(SPEC) },
  );
  const after = (c: number | undefined) =>
    c === undefined || p2.detected === undefined
      ? "never"
      : `+${((c - p2.detected) / BUS_CYCLE_HZ).toFixed(3)} s`;
  const ordered =
    p2.detected !== undefined &&
    p2.braked !== undefined &&
    p2.torqueOff !== undefined &&
    p2.detected <= p2.braked &&
    p2.braked < p2.torqueOff;
  findings.push({
    id: "H3c",
    title: "P2 E-stop works with brakes",
    target: "Ramp, brake, then safe torque off, in that order; no sag beyond the brake model",
    measured: `brake ${after(p2.braked)}, torque off ${after(p2.torqueOff)} after detection; sag ${(p2.sag * 1e3).toFixed(2)} mm in the second after`,
    verdict: ordered && p2.sag < 0.002 ? "met" : "not met",
    detail:
      "Each joint ramps its speed down (SS1-style), closes its brake, then pulls the hardwired safe-torque-off line.",
  });

  // H3d: configuration check.
  const rejected = await armed(ctx, ImpactPolicy.eStop, first, (system) => {
    system.run(0.2);
    return [
      ...new Set(
        system.host.faults.filter((f) => f.code === FaultCode.configRejected).map((f) => f.node),
      ),
    ].sort();
  });
  findings.push({
    id: "H3d",
    title: "Configuration check",
    target:
      "P2 armed on a joint without a brake that carries gravity load is rejected at start-up, with a reason",
    measured: `rejected by joints ${rejected.join(", ") || "none"}`,
    verdict: rejected.join() === "2,3,4" ? "met" : "not met",
    detail:
      "Joints 2, 3 and 4 carry gravity load in the home pose and have no brake; joints 1 and 5 turn about vertical axes and joint 6's payload hangs along its axis, so they accept.",
  });

  // H4: no false stops in normal fast motion.
  const tasks: Record<string, (s: ArmSystem) => Task> = {
    "figure of eight": (s) => figureEight(homePose(s)),
    "pick and place": (s) => pickAndPlace(homePose(s)),
    "fast reach": (s) => fastReach(homePose(s)),
  };
  const h4Seeds = ctx.quick ? [62] : [62, 64, 66, 68, 70];
  let falseStops = 0;
  let runs = 0;
  for (const seed of h4Seeds) {
    for (const make of Object.values(tasks)) {
      falseStops += await withArm(ctx.lab, { spec: SPEC, seed }, (system) => {
        system.run(5);
        system.host.applySettings({ impactConfirm: 1 });
        const t = make(system);
        system.host.startTask(t, system.cycle, { mode: "joint-level" });
        system.run(t.duration + 0.5);
        return system.host.impacts.length > 0 ? 1 : 0;
      });
      runs++;
    }
  }
  ctx.log(`H3/H4 done: reaction ${worstReaction} cycles, false stops ${falseStops}/${runs}`);
  findings.push({
    id: "H4",
    title: "No false stops",
    target: "0 E-stops during normal fast motion, N seeds",
    measured: `${falseStops} false stops in ${runs} runs (${h4Seeds.length} seeds × 3 tasks)`,
    verdict: falseStops === 0 ? "met" : "not met",
    detail: `Detection compares each joint's external torque (gear torque minus what the IMU-measured motion of the links beyond needs) with thresholds commissioned at twice the largest seen in normal tasks (NX-16). Reaction armed on any joint; ${BUS_CYCLE_HZ / 1000} kHz bus.`,
  });
  return findings;
}
