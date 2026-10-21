/**
 * NX-31: neighbourhood commissioning datasets for the coordinated models.
 *
 * For each arm (datasheet actuators), at ten postures spread over the joints' working
 * range (three were too few: between them the learned rows went wrong), every joint runs
 * its own excitation at once on its own seed: once for training, once on other seeds held
 * out. The host records every joint's state frames; a joint's coordinated model is
 * trained on its own columns and those of the joints up to two places away.
 *
 *   node experiments/src/commission-neighbourhood.ts
 *
 * Writes experiments/results/commissioning/<arm>-neighbourhood/:
 *   meta.json                       what each node knows about itself, and the format
 *   <posture>-{train,heldout}.csv   every joint's published state, one row per cycle
 *   truth/<posture>-{train,heldout}.csv   true output accelerations and gear twists
 */
import { mkdir, writeFile } from "node:fs/promises";
import { DampingMode } from "../../sim/src/bus/protocol.ts";
import { nodeConfigsFor } from "../../sim/src/harness/gains.ts";
import { tunedGains } from "../../sim/src/harness/tuned.ts";
import { BUS_CYCLE_HZ } from "../../sim/src/index.ts";
import { KX6, KX6_HOME, KX6L, withInitialAngles } from "../../sim/src/plant/kx6.ts";
import { commissionNeighbourhood, toCsv } from "../../sim/src/scenarios/commissioning.ts";
import { POSTURES } from "./estimation.ts";
import { openLab, withArm } from "./runner.ts";

const TRAIN_SECONDS = 20;
const HELDOUT_SECONDS = 10;
/** Where the neighbourhood is commissioned: NX-30's three postures and seven more, each
 * with the tool at least 0.3 m above the floor. */
const COMMISSIONING_POSTURES: Record<string, readonly number[]> = {
  ...POSTURES,
  low: [0, 0.8, 0.3, 1.8, 0, 0],
  stretched: [0, 0.6, 0.2, 1.4, 0, 0],
  folded: [0, 0.35, 0.8, 1.9, 0, 0],
  up: [0, 0.2, 1.0, 1.3, 0, 0],
  tilted: [0, 0.7, 0.6, 1.6, 0, 0],
  // The excitation moves each joint only about ±0.1 rad (its speed budget), so the base and
  // wrist joints must be turned here: tasks turn them by ±0.4 rad.
  "turned-left": [0.3, 0.55, 0.7, 1.6, 0.35, 0.3],
  "turned-right": [-0.3, 0.45, 0.9, 1.3, -0.35, -0.3],
};
const ARMS = { kx6l: KX6L, kx6: KX6 } as const;

const lab = await openLab();
for (const [arm, base] of Object.entries(ARMS)) {
  const gains = tunedGains(arm, "network");
  const dir = new URL(`../results/commissioning/${arm}-neighbourhood/`, import.meta.url);
  await mkdir(new URL("truth/", dir), { recursive: true });
  let seed = 7000;
  for (const [posture, angles] of Object.entries(COMMISSIONING_POSTURES)) {
    const spec = withInitialAngles(base, angles);
    await withArm(lab, { spec, gains, seed: 41 }, async (system) => {
      system.run(3);
      system.host.applySettings({ damping: DampingMode.network });
      system.run(0.5);
      for (const [part, seconds] of [
        ["train", TRAIN_SECONDS],
        ["heldout", HELDOUT_SECONDS],
      ] as const) {
        const run = commissionNeighbourhood(system, seconds, seed++);
        await writeFile(new URL(`${posture}-${part}.csv`, dir), toCsv(run.columns, run.log));
        await writeFile(
          new URL(`truth/${posture}-${part}.csv`, dir),
          toCsv(run.truthColumns, run.truth),
        );
        console.log(`${arm} ${posture} ${part}: ${run.log.length} cycles`);
        system.run(1);
      }
    });
  }
  const configs = nodeConfigsFor(withInitialAngles(base, KX6_HOME), BUS_CYCLE_HZ, gains);
  await writeFile(
    new URL("meta.json", dir),
    `${JSON.stringify(
      {
        arm,
        cycleHz: BUS_CYCLE_HZ,
        actuationDelayCycles: 1,
        postures: Object.keys(COMMISSIONING_POSTURES),
        note:
          "torque_command at cycle k acts on the plant over cycle k+1. " +
          "Angles are output-referred, rad; torques output-referred, N·m.",
        joints: configs.map((c, i) => ({
          joint: i + 1,
          axis: c.axis,
          ratedTorque: c.ratedTorque,
          reflectedInertia: c.reflectedInertia,
          gearStiffness: c.gearStiffness,
          viscousFriction: c.viscousFriction,
          coulombFriction: c.coulombFriction,
        })),
      },
      null,
      2,
    )}\n`,
  );
}
