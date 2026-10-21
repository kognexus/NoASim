/**
 * H10b and H10d: the learned models in closed loop.
 *
 * H10b: every unit (an arm with its own hidden actuators, `kx6-s300…`) runs a figure of
 * eight and a pick and place, then takes a tap, once with each model the joints can
 * use: Nominal (datasheet), Fitted (online RLS of the textbook structure) and Learned
 * (the unit's committed PHNNs). The same arm with datasheet actuators, where the nominal
 * model is exact, is the reference ("Oracle"). The headline metric is how far each arm's
 * joints stray from the reference arm's during slow sweeps, where friction matters most.
 *
 * H10d: the learned models deliberately broken (friction ×5, cogging ×−10, noise on every
 * weight). The arm must stay stable, and the network term within its energy tank.
 *
 *   node experiments/src/learned-models.ts [units=kx6-s300,kx6-s301,kx6-s302]
 *
 * Writes experiments/results/learned-models.json. `npm run prove` calls
 * {@link measureLearnedModels}.
 */
import { mkdir, writeFile } from "node:fs/promises";
import { DampingMode } from "../../sim/src/bus/protocol.ts";
import { BUS_CYCLE_HZ } from "../../sim/src/index.ts";
import type { ArmSystem } from "../../sim/src/harness/armSystem.ts";
import { tunedGains } from "../../sim/src/harness/tuned.ts";
import { figureEight, pickAndPlace } from "../../sim/src/host/tasks.ts";
import type { Layer, PhnnDocument } from "../../sim/src/node/phnnModel.ts";
import { KX6, KX6_HOME, withInitialAngles } from "../../sim/src/plant/kx6.ts";
import { dampingWork } from "../../sim/src/scenarios/energy.ts";
import { followTask, homePose } from "../../sim/src/scenarios/motion.ts";
import { tapTest } from "../../sim/src/scenarios/tap.ts";
import { Random } from "../../sim/src/util/random.ts";
import { type Lab, openLab, unitModels, withArm } from "./runner.ts";

/** The units of hidden actuators with committed learned models. */
export const UNITS = ["kx6-s300", "kx6-s301", "kx6-s302"];
const spec = withInitialAngles(KX6, KX6_HOME);
const gains = tunedGains("kx6", "network");

/** Settings byte of each model (`ModelChoice`). */
const MODEL_BYTE = { nominal: 0, fitted: 1, learned: 2 } as const;
type ModelName = keyof typeof MODEL_BYTE;

export interface Outcome {
  /**
   * How far the joints stray, during slow sweeps, from where the same arm with datasheet
   * actuators goes: what the unknown actuators do that the model does not cancel. RMS over
   * joints and the last 7 s, mrad.
   */
  readonly sweepDeviationMrad: number;
  readonly figureEightMm: number;
  readonly pickAndPlaceMm: number;
  readonly tapSettleS: number;
  readonly tapPeakMm: number;
  /** Largest tool distance from rest in the last half second of the tap record, mm. */
  readonly tapTailMm: number;
  /** Most energy the network term put into the arm after the tap (≤ 0: it only took), J. */
  readonly maxDampingWorkJ: number;
}

/** Every joint sweeps ±0.2 rad at its own slow frequency: friction matters most here. */
const SWEEP_HZ = [0.15, 0.2, 0.17, 0.23, 0.19, 0.21];
const SWEEP_SECONDS = 10;
/** The first seconds of the sweep are not scored (the online fit is still learning). */
const SWEEP_SKIP_SECONDS = 3;

/** Runs the slow sweeps and returns the joint angles of every cycle; then returns home. */
function sweeps(system: ArmSystem): number[][] {
  const start = system.joints.map((j) => system.plant.outputAngle(j.index));
  const angles = (t: number) =>
    start.map((q, i) => q + 0.2 * Math.sin(2 * Math.PI * (SWEEP_HZ[i] ?? 0.2) * t));
  system.host.startJointMotion(angles, system.cycle);
  const trajectory: number[][] = [];
  for (let k = 0; k < SWEEP_SECONDS * BUS_CYCLE_HZ; k++) {
    system.step();
    trajectory.push(system.joints.map((j) => system.plant.outputAngle(j.index)));
  }
  const end = angles(SWEEP_SECONDS);
  const back = (t: number) => {
    const s = Math.min(1, t / 2);
    const blend = s * s * (3 - 2 * s);
    return end.map((q, i) => q + ((start[i] ?? q) - q) * blend);
  };
  system.host.startJointMotion(back, system.cycle);
  system.run(2.5);
  system.host.stopJointMotion();
  return trajectory;
}

/** RMS difference of two sweep trajectories after the unscored start, rad. */
function deviation(a: number[][], b: number[][]): number {
  let sum = 0;
  let count = 0;
  for (let k = SWEEP_SKIP_SECONDS * BUS_CYCLE_HZ; k < Math.min(a.length, b.length); k++) {
    const qb = b[k] ?? [];
    (a[k] ?? []).forEach((q, i) => {
      sum += (q - (qb[i] ?? q)) ** 2;
      count++;
    });
  }
  return Math.sqrt(sum / Math.max(1, count));
}

type Measured = Omit<Outcome, "sweepDeviationMrad">;

/** Slow sweeps, two tasks and a tap, with the joints using `model`. */
function trial(system: ArmSystem, model: ModelName): { outcome: Measured; sweep: number[][] } {
  system.run(3);
  system.host.applySettings({ damping: DampingMode.network, model: MODEL_BYTE[model] });
  const options = { mode: "joint-level" } as const;
  const sweep = sweeps(system);
  const eight = followTask(system, (s) => figureEight(homePose(s)), options);
  system.run(1);
  const pick = followTask(system, (s) => pickAndPlace(homePose(s)), options);
  system.run(1);
  const tap = tapTest(system, { settleSeconds: 0, recordSeconds: 2 });
  const work = dampingWork(system, 1);
  return {
    sweep,
    outcome: {
      figureEightMm: eight.pathErrorRms * 1e3,
      pickAndPlaceMm: pick.pathErrorRms * 1e3,
      tapSettleS: tap.settlingTime,
      tapPeakMm: tap.peak * 1e3,
      tapTailMm: 1e3 * Math.max(...tap.deviation.slice(-Math.round(0.5 * BUS_CYCLE_HZ))),
      maxDampingWorkJ: work.maxWork,
    },
  };
}

export const fmt = (o: Outcome) =>
  `sweep ${o.sweepDeviationMrad.toFixed(3)} mrad  eight ${o.figureEightMm.toFixed(2)} mm` +
  `  pick ${o.pickAndPlaceMm.toFixed(2)} mm  tap ${o.tapSettleS.toFixed(3)} s` +
  ` / ${o.tapPeakMm.toFixed(1)} mm, tail ${o.tapTailMm.toFixed(3)} mm  work ${o.maxDampingWorkJ.toFixed(4)} J`;

function mapLayers(layers: readonly Layer[], f: (w: number) => number): Layer[] {
  return layers.map((l) => ({ weight: l.weight.map((row) => row.map(f)), bias: l.bias.map(f) }));
}

const CORRUPTIONS: Record<string, (doc: PhnnDocument, rng: Random) => PhnnDocument> = {
  "friction x5": (doc) => ({
    ...doc,
    constants: {
      ...doc.constants,
      viscousScale: 5 * doc.constants.viscousScale,
      coulombScale: 5 * doc.constants.coulombScale,
    },
  }),
  "cogging x-10": (doc) => ({
    ...doc,
    cogging: { ...doc.cogging, weight: doc.cogging.weight.map((w) => -10 * w) },
  }),
  "noise on every weight": (doc, rng) => {
    const noisy = (w: number) => w + rng.normal() * (0.5 + Math.abs(w));
    return {
      ...doc,
      gear: mapLayers(doc.gear, noisy),
      friction: {
        ...doc.friction,
        viscous: mapLayers(doc.friction.viscous, noisy),
        coulomb: mapLayers(doc.friction.coulomb, noisy),
      },
      gain: { ...doc.gain, temperature: mapLayers(doc.gain.temperature, noisy) },
    };
  },
};

export interface LearnedModels {
  readonly h10b: Record<string, Record<string, Outcome>>;
  readonly h10d: Record<string, Outcome>;
}

/**
 * H10b on every unit of `units`, then H10d (broken models) on the first. Each unit's
 * reference is the same arm with datasheet actuators.
 */
export async function measureLearnedModels(
  lab: Lab,
  units: readonly string[] = UNITS,
  log: (line: string) => void = console.log,
): Promise<LearnedModels> {
  const h10b: Record<string, Record<string, Outcome>> = {};
  const references: Record<string, number[][]> = {};
  for (const unit of units) {
    const seed = Number(unit.split("-s").pop());
    const models = await unitModels(unit, spec.joints.length);
    const oracle = await withArm(lab, { spec, gains, seed }, (s) => trial(s, "nominal"));
    references[unit] = oracle.sweep;
    const reference = { ...oracle.outcome, sweepDeviationMrad: 0 };
    const rows: Record<string, Outcome> = { oracle: reference };
    log(`H10b ${unit} ${"oracle".padEnd(8)} ${fmt(reference)}  (datasheet actuators)`);
    for (const model of Object.keys(MODEL_BYTE) as ModelName[]) {
      const run = await withArm(lab, { spec, gains, seed, actuators: {}, models }, (s) =>
        trial(s, model),
      );
      rows[model] = {
        ...run.outcome,
        sweepDeviationMrad: 1e3 * deviation(run.sweep, oracle.sweep),
      };
      log(`H10b ${unit} ${model.padEnd(8)} ${fmt(rows[model])}`);
    }
    h10b[unit] = rows;
  }

  const h10d: Record<string, Outcome> = {};
  {
    const unit = units[0] ?? "kx6-s300";
    const seed = Number(unit.split("-s").pop());
    const models = await unitModels(unit, spec.joints.length);
    for (const [name, corrupt] of Object.entries(CORRUPTIONS)) {
      const rng = new Random(77);
      const broken = models.map((doc) => corrupt(doc, rng));
      const run = await withArm(lab, { spec, gains, seed, actuators: {}, models: broken }, (s) =>
        trial(s, "learned"),
      );
      const reference = references[unit] ?? run.sweep;
      const outcome = { ...run.outcome, sweepDeviationMrad: 1e3 * deviation(run.sweep, reference) };
      h10d[name] = outcome;
      log(`H10d ${unit} ${name.padEnd(22)} ${fmt(outcome)}`);
    }
  }
  return { h10b, h10d };
}

if (import.meta.main) {
  const units = (process.argv[2] ?? UNITS.join(",")).split(",");
  const { h10b, h10d } = await measureLearnedModels(await openLab(), units);
  await mkdir(new URL("../results/", import.meta.url), { recursive: true });
  await writeFile(
    new URL("../results/learned-models.json", import.meta.url),
    `${JSON.stringify({ h10b, h10d }, null, 2)}\n`,
  );
}
