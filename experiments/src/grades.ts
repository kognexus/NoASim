/**
 * NX-26: the same arm on budget and premium actuators (`sim/src/actuators/grades.ts`),
 * next to the datasheet ("ideal") arm. For each grade:
 *
 * - **Damping:** a tap at rest, and a figure of eight from the VLA computer, with joint
 *   control only and with network damping, with datasheet models and (for grades that
 *   have them) learned models: settling, tracking, and the vibration the joints feel.
 * - **Models:** slow sweeps, two tasks and a tap with network damping, once with the
 *   datasheet model and once with each joint's learned model (`models/<unit>/`). Sweep
 *   deviation is measured against the ideal arm with the same seed.
 * - **Report to source:** blinded trials as in H7 (a random joint gets a gear-mesh ripple
 *   of 5–15 % of rated while all joints stroke), with the datasheet and the learned
 *   residual; and a figure of eight with nothing injected, counting every claim and
 *   fault the grade's own imperfections cause.
 *
 *   node experiments/src/grades.ts [trials=10]
 *
 * Writes experiments/results/grades.json. `npm run prove` calls {@link measureGrades}.
 */
import { mkdir, writeFile } from "node:fs/promises";
import { VibrationMeter } from "../../sim/src/analysis/vibration.ts";
import { DampingMode, FaultCode } from "../../sim/src/bus/protocol.ts";
import type { ArmSystem, ArmSystemOptions } from "../../sim/src/harness/armSystem.ts";
import { tunedGains } from "../../sim/src/harness/tuned.ts";
import { figureEight } from "../../sim/src/host/tasks.ts";
import { BUS_CYCLE_HZ } from "../../sim/src/index.ts";
import type { PhnnDocument } from "../../sim/src/node/phnnModel.ts";
import { KX6, KX6_HOME, withInitialAngles } from "../../sim/src/plant/kx6.ts";
import { homePose } from "../../sim/src/scenarios/motion.ts";
import { meshOrder, rippleRun, strokes } from "../../sim/src/scenarios/source.ts";
import { tapTest } from "../../sim/src/scenarios/tap.ts";
import { Random } from "../../sim/src/util/random.ts";
import { deviation, MODEL_BYTE, type Measured, trial } from "./learned-models.ts";
import { type Lab, openLab, unitModels, withArm } from "./runner.ts";

const spec = withInitialAngles(KX6, KX6_HOME);
const gains = tunedGains("kx6", "network");

export interface Grade {
  readonly name: string;
  /** Unit with committed learned models, or none for the ideal arm. */
  readonly unit?: string;
  /** Arm seed: sensor noise and the actuator draw. */
  readonly seed: number;
  readonly family?: string;
}

export const GRADE_RUNS: readonly Grade[] = [
  { name: "ideal", seed: 500 },
  { name: "premium", unit: "kx6-premium-s600", seed: 600, family: "grade-premium" },
  { name: "budget", unit: "kx6-budget-s500", seed: 500, family: "grade-budget" },
];

export interface DampingOutcome {
  readonly tapSettleS: number;
  readonly tapPeakMm: number;
  readonly eightMm: number;
  /** Mean over joints 2–6 of the vibration each joint feels during the task, °/s RMS. */
  readonly vibrationDegS: number;
}

export interface Score {
  right: number;
  wrong: number;
  none: number;
}

export interface GradeOutcome {
  readonly damping: Record<"off" | "network", DampingOutcome>;
  /** The same with each joint's learned model, for grades that have one. */
  readonly dampingLearned?: Record<"off" | "network", DampingOutcome>;
  /** Network damping with each model; sweep deviation against the ideal arm, mrad. */
  readonly models: Partial<Record<"nominal" | "learned", Measured & { sweepMrad: number }>>;
  readonly attribution: Partial<Record<"nominal" | "learned", Score>>;
  /** With nothing injected: joints that claimed anything, and source faults. */
  readonly unprompted: Partial<Record<"nominal" | "learned", { claims: number[]; faults: number }>>;
}

function options(grade: Grade, models?: PhnnDocument[]): ArmSystemOptions {
  return {
    spec,
    gains,
    seed: grade.seed,
    ...(grade.family ? { actuators: { families: [grade.family] } } : {}),
    ...(models ? { models } : {}),
  };
}

/** A figure of eight from the VLA computer: tracking and the vibration the joints feel. */
function eight(system: ArmSystem): { mm: number; vibration: number } {
  const meter = new VibrationMeter(system.host, system.joints.length);
  const task = figureEight(homePose(system));
  const start = system.cycle;
  system.host.startTask(task, start, { mode: "joint-level" });
  const sums = new Array<number>(system.joints.length).fill(0);
  let error = 0;
  let n = 0;
  for (let k = 0; k < Math.round(task.duration * BUS_CYCLE_HZ); k++) {
    system.step();
    meter.observe();
    if (k > BUS_CYCLE_HZ && k % 40 === 0) {
      meter.levels().forEach((v, j) => (sums[j] = (sums[j] ?? 0) + v * v));
      const want = task.pose((system.cycle - start) / BUS_CYCLE_HZ).position;
      const p = system.plant.tcpPosition();
      error += (p[0] - want[0]) ** 2 + (p[1] - want[1]) ** 2 + (p[2] - want[2]) ** 2;
      n++;
    }
  }
  const rms = sums.map((s) => Math.sqrt(s / Math.max(1, n)));
  const outer = rms.slice(1);
  return {
    mm: 1e3 * Math.sqrt(error / Math.max(1, n)),
    vibration: outer.reduce((a, b) => a + b, 0) / Math.max(1, outer.length),
  };
}

async function damping(
  lab: Lab,
  grade: Grade,
  mode: DampingMode,
  models?: PhnnDocument[],
): Promise<DampingOutcome> {
  const model = models ? MODEL_BYTE.learned : MODEL_BYTE.nominal;
  const tap = await withArm(lab, options(grade, models), (system) => {
    system.host.applySettings({ model });
    return tapTest(system, {
      recordSeconds: 3,
      beforeTap: (s) => s.host.applySettings({ damping: mode }),
    });
  });
  const task = await withArm(lab, options(grade, models), (system) => {
    system.host.applySettings({ model });
    system.run(3);
    system.host.applySettings({ damping: mode });
    system.run(0.5);
    return eight(system);
  });
  return {
    tapSettleS: tap.settlingTime,
    tapPeakMm: tap.peak * 1e3,
    eightMm: task.mm,
    vibrationDegS: task.vibration,
  };
}

/** Runs every measurement on every grade; `trials` blinded attribution trials per model. */
export async function measureGrades(
  lab: Lab,
  trials = 10,
  log: (line: string) => void = console.log,
): Promise<Record<string, GradeOutcome>> {
  const out: Record<string, GradeOutcome> = {};
  for (const grade of GRADE_RUNS) {
    // The same arm and seed (sensor noise and bias) with datasheet actuators.
    const reference = await withArm(lab, options({ name: "ideal", seed: grade.seed }), (s) =>
      trial(s, "nominal"),
    );
    const models = grade.unit ? await unitModels(grade.unit, spec.joints.length) : undefined;
    const choices = models ? (["nominal", "learned"] as const) : (["nominal"] as const);
    const result: GradeOutcome = {
      damping: {
        off: await damping(lab, grade, DampingMode.off),
        network: await damping(lab, grade, DampingMode.network),
      },
      ...(models
        ? {
            dampingLearned: {
              off: await damping(lab, grade, DampingMode.off, models),
              network: await damping(lab, grade, DampingMode.network, models),
            },
          }
        : {}),
      models: {},
      attribution: {},
      unprompted: {},
    };
    for (const [mode, d] of [
      ...Object.entries(result.damping),
      ...Object.entries(result.dampingLearned ?? {}).map(([m, o]) => [`${m}, learned`, o] as const),
    ]) {
      log(
        `grades ${grade.name} ${mode}: tap ${d.tapSettleS.toFixed(3)} s / ${d.tapPeakMm.toFixed(1)} mm,` +
          ` eight ${d.eightMm.toFixed(2)} mm, vibration ${d.vibrationDegS.toFixed(2)} °/s`,
      );
    }

    for (const model of choices) {
      const run = await withArm(lab, options(grade, models), (s) => trial(s, model));
      result.models[model] = {
        ...run.outcome,
        sweepMrad: 1e3 * deviation(run.sweep, reference.sweep),
      };
      const m = result.models[model];
      log(
        `grades ${grade.name} ${model}: sweep ${m.sweepMrad.toFixed(3)} mrad, eight ${m.figureEightMm.toFixed(2)} mm,` +
          ` pick ${m.pickAndPlaceMm.toFixed(2)} mm, tap ${m.tapSettleS.toFixed(3)} s`,
      );

      result.unprompted[model] = await withArm(lab, options(grade, models), (system) => {
        system.host.applySettings({ model: MODEL_BYTE[model] });
        system.run(3);
        system.host.applySettings({ damping: DampingMode.networkWithSource });
        system.run(0.5);
        eight(system);
        return {
          claims: [
            ...new Set(
              system.host.disturbances.filter((d) => d.node === d.source).map((d) => d.node),
            ),
          ].sort((a, b) => a - b),
          faults: system.host.faults.filter((f) => f.code === FaultCode.source).length,
        };
      });
      const u = result.unprompted[model];
      log(`grades ${grade.name} ${model}: unprompted claims [${u.claims}], faults ${u.faults}`);

      const score: Score = { right: 0, wrong: 0, none: 0 };
      const rng = new Random(321);
      for (let t = 0; t < trials; t++) {
        const source = Math.floor(rng.uniform() * spec.joints.length);
        const joint = spec.joints[source];
        if (!joint) throw new Error("bad source");
        const amplitude = rng.between(0.05, 0.15) * joint.actuator.ratedTorque;
        const speeds = spec.joints.map(() => rng.between(1.0, 1.6));
        const run = await withArm(lab, options(grade, models), (system) => {
          system.host.applySettings({ model: MODEL_BYTE[model] });
          system.run(3);
          return rippleRun(system, {
            source,
            order: meshOrder(joint),
            amplitude,
            mode: DampingMode.network,
            motion: strokes(speeds),
            recordSeconds: 4,
          });
        });
        if (run.claimants.some((c) => c !== source + 1)) score.wrong++;
        else if (run.claimants.includes(source + 1)) score.right++;
        else score.none++;
      }
      result.attribution[model] = score;
      log(`grades ${grade.name} ${model}: attribution ${JSON.stringify(score)}`);
    }
    out[grade.name] = result;
  }
  return out;
}

if (import.meta.main) {
  const trials = Number(process.argv[2] ?? 10);
  const result = await measureGrades(await openLab(), trials);
  await mkdir(new URL("../results/", import.meta.url), { recursive: true });
  await writeFile(
    new URL("../results/grades.json", import.meta.url),
    `${JSON.stringify(result, null, 2)}\n`,
  );
}
