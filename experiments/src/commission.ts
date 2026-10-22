/**
 * NX-13: commissioning datasets for the per-joint learned models (B11).
 *
 * For each arm seed, every joint of a KX-6 with hidden actuators runs its own excitation
 * twice: once for training, once (with another seed, 0.25 rad away) held out for
 * evaluation. Each run
 * is written as the joint's own log, plus a separate ground-truth file the training never
 * reads.
 *
 *   node experiments/src/commission.ts [armSeeds=3] [--sealed | --grade budget|premium]
 *
 * Writes experiments/results/commissioning/<arm>-s<seed>/:
 *   meta.json                what each node knows about itself (its EEPROM) and the format
 *   joint<n>-{train,heldout}.csv          the joint's log (LOG_COLUMNS)
 *   truth/joint<n>-{train,heldout}.csv    ground truth (TRUTH_COLUMNS)
 *   truth/families.json      which hidden family each joint was drawn from, and the
 *                            plant's gear damping
 *
 * `--sealed` draws only the sealed out-of-family actuator. It is for the final H10 run and
 * nothing else (see issue NX-04). `--grade` draws every joint from one actuator
 * grade (NX-26): units `kx6-budget-s500…` and `kx6-premium-s600…`.
 */
import { mkdir, writeFile } from "node:fs/promises";
import type { HiddenActuator } from "../../sim/src/actuators/hiddenActuator.ts";
import { DampingMode } from "../../sim/src/bus/protocol.ts";
import { BUS_CYCLE_HZ } from "../../sim/src/index.ts";
import { nodeConfigsFor } from "../../sim/src/harness/gains.ts";
import { tunedGains } from "../../sim/src/harness/tuned.ts";
import { KX6, KX6_HOME, withInitialAngles } from "../../sim/src/plant/kx6.ts";
import {
  commissionJoint,
  LOG_COLUMNS,
  moveJoint,
  toCsv,
  TRUTH_COLUMNS,
} from "../../sim/src/scenarios/commissioning.ts";
import { openLab, withArm } from "./runner.ts";

const TRAIN_SECONDS = 10;
const HELDOUT_SECONDS = 5;
/** The held-out run starts this far from the training run, rad. */
const HELDOUT_SHIFT = 0.25;
const SEALED_FAMILY = "sealed-lugre-eccentric";

const sealed = process.argv.includes("--sealed");
const gradeArg = process.argv.indexOf("--grade");
const grade = gradeArg > 0 ? process.argv[gradeArg + 1] : undefined;
if (grade !== undefined && grade !== "budget" && grade !== "premium") {
  throw new Error(`unknown grade ${grade}`);
}
const GRADE_SEEDS = { budget: 500, premium: 600 } as const;
const armSeeds = Number(process.argv.find((a, i) => i > 1 && /^\d+$/.test(a)) ?? 3);
const spec = withInitialAngles(KX6, KX6_HOME);
const gains = tunedGains("kx6", "network");
const configs = nodeConfigsFor(spec, BUS_CYCLE_HZ, gains);
const lab = await openLab();

for (let s = 0; s < armSeeds; s++) {
  const seed = (grade ? GRADE_SEEDS[grade] : sealed ? 900 : 300) + s;
  const name = `kx6-${grade ? `${grade}-` : sealed ? "sealed-" : ""}s${seed}`;
  const dir = new URL(`../results/commissioning/${name}/`, import.meta.url);
  await mkdir(new URL("truth/", dir), { recursive: true });
  const actuators = grade
    ? { families: [`grade-${grade}`] }
    : sealed
      ? { families: [SEALED_FAMILY], includeSealed: true }
      : {};
  const families = await withArm(lab, { spec, gains, seed, actuators }, async (system) => {
    system.run(3);
    system.host.applySettings({ damping: DampingMode.network });
    system.run(0.5);
    for (let j = 0; j < spec.joints.length; j++) {
      for (const [part, seconds, runSeed] of [
        ["train", TRAIN_SECONDS, 2 * j + 1],
        ["heldout", HELDOUT_SECONDS, 2 * j + 2],
      ] as const) {
        // The held-out run is at another pose, so it tests what the model generalises,
        // not what it memorised about one stretch of angle.
        if (part === "heldout") moveJoint(system, j, HELDOUT_SHIFT);
        const run = commissionJoint(system, j, seconds, seed * 100 + runSeed);
        await writeFile(new URL(`joint${j + 1}-${part}.csv`, dir), toCsv(LOG_COLUMNS, run.log));
        await writeFile(
          new URL(`truth/joint${j + 1}-${part}.csv`, dir),
          toCsv(TRUTH_COLUMNS, run.truth),
        );
        if (part === "heldout") moveJoint(system, j, -HELDOUT_SHIFT);
        system.run(0.5);
      }
    }
    return system.joints.map((j) => (j.physics as HiddenActuator).family);
  });
  await writeFile(
    new URL("truth/families.json", dir),
    `${JSON.stringify(
      {
        families,
        sealed,
        // The plant's gear damping, which no node is told; only the Oracle uses it.
        gearDamping: spec.joints.map((j) => j.actuator.gearDamping),
      },
      null,
      2,
    )}\n`,
  );
  await writeFile(
    new URL("meta.json", dir),
    `${JSON.stringify(
      {
        arm: "kx6",
        seed,
        cycleHz: BUS_CYCLE_HZ,
        actuationDelayCycles: 1,
        columns: LOG_COLUMNS,
        note:
          "torque_command at cycle k acts on the plant over cycle k+1. " +
          "Angles are output-referred, rad; torques output-referred, N·m.",
        joints: configs.map((c, i) => ({
          joint: i + 1,
          gearRatio: c.gearRatio,
          polePairs: spec.joints[i]?.actuator.electrical.polePairs,
          ratedTorque: c.ratedTorque,
          reflectedInertia: c.reflectedInertia,
          gearStiffness: c.gearStiffness,
          viscousFriction: c.viscousFriction,
          coulombFriction: c.coulombFriction,
          fingerprint: c.fingerprint,
        })),
      },
      null,
      2,
    )}\n`,
  );
  console.log(`${name}: ${families.join(", ")}`);
}
