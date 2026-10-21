/**
 * H5 and H7: report to source.
 *
 * H5: joint 3 of the KX-6 sweeps slowly while its motor gets a torque ripple at the sixth
 * electrical harmonic. A ripple within the drive's compensation budget must be cancelled
 * (the neighbours' helper effort at that order under 10% of what it was) before the
 * deadline, without a fault; one beyond it must reach the host as a source fault.
 *
 * H7: blinded attribution. Each trial gives a random joint a gear-mesh ripple of random
 * size while every joint strokes back and forth at its own speed; the trial is right if the
 * source, and only the source, claims it. Network mode against joints alone (local mode).
 *
 * H7 on hidden actuators (NX-15): the same trials on the committed units, whose actuators
 * nobody described, with each joint's self-test using the nominal or its learned model.
 *
 *   node experiments/src/report-to-source.ts [trials=100] [hiddenTrials=30]
 *
 * Writes experiments/results/report-to-source.json. `npm run prove` calls
 * {@link measureReportToSource}.
 */
import { mkdir, writeFile } from "node:fs/promises";
import { tunedGains } from "../../sim/src/harness/tuned.ts";
import { KX6, KX6_HOME, withInitialAngles } from "../../sim/src/plant/kx6.ts";
import {
  DampingMode,
  meshOrder,
  rippleOrder,
  rippleRun,
  type RippleRun,
  strokes,
  sweep,
} from "../../sim/src/scenarios/source.ts";
import { Random } from "../../sim/src/util/random.ts";
import { type Lab, openLab, unitModels, withArm } from "./runner.ts";

/** Seconds from the first claim within which the correction must have worked. */
export const DEADLINE_SECONDS = 2;
/** The units of hidden actuators with committed learned models. */
const UNITS = ["kx6-s300", "kx6-s301", "kx6-s302"];
const SOURCE = 2;
const MODEL_BYTE = { nominal: 0, learned: 2 } as const;

const spec = withInitialAngles(KX6, KX6_HOME);
const gains = tunedGains("kx6", "network");

export interface Score {
  right: number;
  wrong: number;
  none: number;
}

export interface H5Case {
  readonly name: string;
  readonly share: number;
  readonly run: RippleRun;
  /** Helper effort before the claim, N·m, and when it fell under 10 % (s after the claim). */
  readonly decay: { before: number; decayedAfter: number | undefined };
}

export interface ReportToSource {
  readonly deadlineSeconds: number;
  readonly h5: H5Case[];
  readonly h7: { trials: number; score: Record<"network" | "local", Score>; runs: unknown[] };
  readonly h7Hidden: {
    trials: number;
    score: Record<keyof typeof MODEL_BYTE, Score>;
    runs: unknown[];
  };
}

/** First time (s after the claim) the helper effort falls under 10% of its level before. */
function effortDecay(run: RippleRun): H5Case["decay"] {
  const claimWindow = Math.floor((run.firstClaim ?? Number.NaN) / run.windowSeconds);
  const before = run.helperEffort
    .slice(Math.max(0, claimWindow - 2), claimWindow)
    .filter(Number.isFinite);
  const level = before.reduce((a, b) => a + b, 0) / Math.max(1, before.length);
  const index = run.helperEffort.findIndex((e, k) => k > claimWindow && e < 0.1 * level);
  return {
    before: level,
    decayedAfter: index < 0 ? undefined : (index + 1) * run.windowSeconds - (run.firstClaim ?? 0),
  };
}

/** Right if the source, and only the source, claims it. */
function tally(s: Score, run: RippleRun, source: number): void {
  if (run.claimants.some((c) => c !== source + 1)) s.wrong++;
  else if (run.claimants.includes(source + 1)) s.right++;
  else s.none++;
}

/** Runs H5, then `trials` blinded H7 trials, then `hiddenTrials` on hidden actuators. */
export async function measureReportToSource(
  lab: Lab,
  trials: number,
  hiddenTrials: number,
  log: (line: string) => void = console.log,
): Promise<ReportToSource> {
  const joint = spec.joints[SOURCE];
  if (!joint) throw new Error("no joint 3");

  // ---- H5 -------------------------------------------------------------------------------
  const cases = [
    { name: "compensable, corrected", share: 0.1, mode: DampingMode.networkWithSource },
    { name: "compensable, not corrected", share: 0.1, mode: DampingMode.network },
    { name: "beyond budget, corrected", share: 0.5, mode: DampingMode.networkWithSource },
  ] as const;
  const h5: H5Case[] = [];
  for (const c of cases) {
    const run = await withArm(lab, { spec, gains, seed: 201 }, (system) => {
      system.run(3);
      return rippleRun(system, {
        source: SOURCE,
        order: rippleOrder(joint),
        amplitude: c.share * joint.actuator.ratedTorque,
        mode: c.mode,
        motion: sweep(spec.joints.length, SOURCE, 0.3, -0.9),
        recordSeconds: 5,
      });
    });
    const decay = effortDecay(run);
    h5.push({ name: c.name, share: c.share, run, decay });
    const fmt = (x: number | undefined) => (x === undefined ? "—" : x.toFixed(2));
    log(
      `H5 ${c.name}: claim ${fmt(run.firstClaim)} s, fault ${fmt(run.fault)} s,` +
        ` helper effort under 10 % after ${fmt(decay.decayedAfter)} s`,
    );
  }

  // ---- H7 -------------------------------------------------------------------------------
  const score: Record<"network" | "local", Score> = {
    network: { right: 0, wrong: 0, none: 0 },
    local: { right: 0, wrong: 0, none: 0 },
  };
  const runs = [];
  const rng = new Random(123);
  for (let t = 0; t < trials; t++) {
    const source = Math.floor(rng.uniform() * spec.joints.length);
    const actuator = spec.joints[source]?.actuator;
    if (!actuator) throw new Error("bad source");
    const amplitude = rng.between(0.05, 0.15) * actuator.ratedTorque;
    const speeds = spec.joints.map(() => rng.between(1.0, 1.6));
    const outcome: Record<string, number[]> = {};
    for (const mode of ["network", "local"] as const) {
      const run = await withArm(lab, { spec, gains, seed: 200 + t }, (system) => {
        system.run(3);
        return rippleRun(system, {
          source,
          order: meshOrder(spec.joints[source] ?? joint),
          amplitude,
          mode: mode === "network" ? DampingMode.network : DampingMode.local,
          motion: strokes(speeds),
          recordSeconds: 4,
        });
      });
      tally(score[mode], run, source);
      outcome[mode] = run.claimants;
    }
    runs.push({ source: source + 1, amplitude, speeds, claimants: outcome });
    if ((t + 1) % 10 === 0) log(`H7 after ${t + 1} trials: ${JSON.stringify(score)}`);
  }

  // ---- H7 on hidden actuators: the self-test with the nominal and the learned model ------
  const hiddenScore: Record<keyof typeof MODEL_BYTE, Score> = {
    nominal: { right: 0, wrong: 0, none: 0 },
    learned: { right: 0, wrong: 0, none: 0 },
  };
  const hidden = [];
  for (let t = 0; t < hiddenTrials; t++) {
    const unit = UNITS[t % UNITS.length] ?? "kx6-s300";
    const seed = Number(unit.split("-s").pop());
    const models = await unitModels(unit, spec.joints.length);
    const source = Math.floor(rng.uniform() * spec.joints.length);
    const amplitude = rng.between(0.05, 0.15) * (spec.joints[source]?.actuator.ratedTorque ?? 0);
    const speeds = spec.joints.map(() => rng.between(1.0, 1.6));
    const outcome: Record<string, number[]> = {};
    for (const model of ["nominal", "learned"] as const) {
      const run = await withArm(lab, { spec, gains, seed, actuators: {}, models }, (system) => {
        system.run(3);
        system.host.applySettings({ model: MODEL_BYTE[model] });
        return rippleRun(system, {
          source,
          order: meshOrder(spec.joints[source] ?? joint),
          amplitude,
          mode: DampingMode.network,
          motion: strokes(speeds),
          recordSeconds: 4,
        });
      });
      tally(hiddenScore[model], run, source);
      outcome[model] = run.claimants;
    }
    hidden.push({ unit, source: source + 1, amplitude, speeds, claimants: outcome });
    if ((t + 1) % 10 === 0) log(`H7 hidden, after ${t + 1} trials: ${JSON.stringify(hiddenScore)}`);
  }
  return {
    deadlineSeconds: DEADLINE_SECONDS,
    h5,
    h7: { trials, score, runs },
    h7Hidden: { trials: hiddenTrials, score: hiddenScore, runs: hidden },
  };
}

if (import.meta.main) {
  const trials = Number(process.argv[2] ?? 100);
  const hiddenTrials = Number(process.argv[3] ?? 30);
  const result = await measureReportToSource(await openLab(), trials, hiddenTrials);
  const { network, local } = result.h7.score;
  console.log(
    `H7 network ${network.right}/${trials} right, ${network.wrong} wrong;` +
      ` joints alone ${local.right}/${trials} right, ${local.wrong} wrong`,
  );
  await mkdir(new URL("../results/", import.meta.url), { recursive: true });
  await writeFile(
    new URL("../results/report-to-source.json", import.meta.url),
    `${JSON.stringify(result, null, 2)}\n`,
  );
}
