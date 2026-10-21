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
 *   node experiments/src/report-to-source.ts [trials]
 *
 * Writes experiments/results/report-to-source.json.
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
import { openLab, withArm } from "./runner.ts";

/** Seconds from the first claim within which the correction must have worked. */
const DEADLINE_SECONDS = 2;

const trials = Number(process.argv[2] ?? 100);
const spec = withInitialAngles(KX6, KX6_HOME);
const gains = tunedGains("kx6", "network");
const lab = await openLab();

// ---- H5 -------------------------------------------------------------------------------
const SOURCE = 2;
const joint = spec.joints[SOURCE];
if (!joint) throw new Error("no joint 3");
const cases = [
  { name: "compensable, corrected", share: 0.1, mode: DampingMode.networkWithSource },
  { name: "compensable, not corrected", share: 0.1, mode: DampingMode.network },
  { name: "beyond budget, corrected", share: 0.5, mode: DampingMode.networkWithSource },
] as const;

/** First time (s after the claim) the helper effort falls under 10% of its level before. */
function effortDecay(run: RippleRun): { before: number; decayedAfter: number | undefined } {
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

console.log(
  "H5  case                          claim s  fault s  helper before N·m  under 10% after s",
);
const h5 = [];
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
  h5.push({ ...c, run, decay });
  const fmt = (x: number | undefined) => (x === undefined ? "—" : x.toFixed(2));
  console.log(
    `    ${c.name.padEnd(28)}  ${fmt(run.firstClaim).padStart(7)}  ${fmt(run.fault).padStart(7)}` +
      `  ${decay.before.toFixed(3).padStart(17)}  ${fmt(decay.decayedAfter).padStart(17)}`,
  );
}

// ---- H7 -------------------------------------------------------------------------------
interface Score {
  right: number;
  wrong: number;
  none: number;
}
const score: Record<"network" | "local", Score> = {
  network: { right: 0, wrong: 0, none: 0 },
  local: { right: 0, wrong: 0, none: 0 },
};
const h7 = [];
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
    const s = score[mode];
    if (run.claimants.some((c) => c !== source + 1)) s.wrong++;
    else if (run.claimants.includes(source + 1)) s.right++;
    else s.none++;
    outcome[mode] = run.claimants;
  }
  h7.push({ source: source + 1, amplitude, speeds, claimants: outcome });
  if ((t + 1) % 10 === 0) console.log(`H7  after ${t + 1} trials: ${JSON.stringify(score)}`);
}
console.log(
  `H7  network ${score.network.right}/${trials} right, ${score.network.wrong} wrong;` +
    ` joints alone ${score.local.right}/${trials} right, ${score.local.wrong} wrong`,
);

await mkdir(new URL("../results/", import.meta.url), { recursive: true });
await writeFile(
  new URL("../results/report-to-source.json", import.meta.url),
  `${JSON.stringify({ deadlineSeconds: DEADLINE_SECONDS, h5, h7: { trials, score, runs: h7 } }, null, 2)}\n`,
);
