import { afterEach, beforeAll, describe, expect, it } from "vitest";
import { KX6, KX6_HOME, withInitialAngles } from "../plant/kx6.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import {
  DampingMode,
  meshOrder,
  rippleOrder,
  rippleRun,
  strokes,
  sweep,
} from "../scenarios/source.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import { ArmSystem } from "./armSystem.ts";
import { tunedGains } from "./tuned.ts";

const SPEC = withInitialAngles(KX6, KX6_HOME);
const SOURCE = 2;
const JOINT = SPEC.joints[SOURCE]!;
/** The source must have fixed the vibration within this long of its first claim (`source.rs`). */
const DEADLINE = 2;

describe("report to source, correction and fault (NX-12)", () => {
  let mujoco: Mujoco;
  let nodeModule: WebAssembly.Module;
  const systems: ArmSystem[] = [];
  beforeAll(async () => {
    [mujoco, nodeModule] = await Promise.all([loadMujoco(), loadNodeModule()]);
  });
  afterEach(() => {
    systems.splice(0).forEach((s) => s.dispose());
  });

  async function settled(seed = 201): Promise<ArmSystem> {
    const system = await ArmSystem.create(mujoco, nodeModule, {
      spec: SPEC,
      seed,
      gains: tunedGains("kx6", "network"),
    });
    systems.push(system);
    system.run(3);
    return system;
  }

  /** A slow sweep of joint 3 with a ripple at its sixth electrical harmonic. */
  const sweepRun = (system: ArmSystem, share: number, mode: DampingMode) =>
    rippleRun(system, {
      source: SOURCE,
      order: rippleOrder(JOINT),
      amplitude: share * JOINT.actuator.ratedTorque,
      mode,
      motion: sweep(SPEC.joints.length, SOURCE, 0.3, -0.9),
      recordSeconds: 4.5,
    });

  it("names the source, and only the source, while every joint moves", async () => {
    const system = await settled(203);
    const run = rippleRun(system, {
      source: 4,
      order: meshOrder(SPEC.joints[4]!),
      amplitude: 0.1 * SPEC.joints[4]!.actuator.ratedTorque,
      mode: DampingMode.network,
      motion: strokes([1.2, 1.5, 1.3, 1.6, 1.4, 1.1]),
      recordSeconds: 4,
    });
    expect(run.claimants).toEqual([5]);
  }, 120_000);

  it("cancels a compensable ripple before the deadline, without a fault", async () => {
    const run = sweepRun(await settled(), 0.1, DampingMode.networkWithSource);
    expect(run.firstClaim).toBeDefined();
    expect(run.fault).toBeUndefined();
    const claim = Math.floor(run.firstClaim! / run.windowSeconds);
    const before = run.helperEffort[claim - 1]!;
    const atDeadline = run.helperEffort.slice(claim + 1, claim + DEADLINE / run.windowSeconds);
    expect(Math.min(...atDeadline.filter(Number.isFinite))).toBeLessThan(0.1 * before);
  }, 120_000);

  it("faults when the source may not correct", async () => {
    const run = sweepRun(await settled(), 0.1, DampingMode.network);
    expect(run.fault).toBeDefined();
    expect(run.fault! - run.firstClaim!).toBeLessThan(DEADLINE + 0.5);
  }, 120_000);

  it("faults when the ripple is beyond the compensation budget", async () => {
    const run = sweepRun(await settled(), 0.5, DampingMode.networkWithSource);
    expect(run.fault).toBeDefined();
    expect(run.fault! - run.firstClaim!).toBeLessThan(DEADLINE + 0.5);
  }, 120_000);
});
