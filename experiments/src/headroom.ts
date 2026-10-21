/**
 * NX-27: how much better could any damping do? The joints' three modes against the ideal
 * reference controller (`sim/src/analysis/ideal.ts`), which sees the plant's whole true
 * state with no delay: an upper bound for a learned damper.
 *
 * Taps at rest, as in H1: both arms, two postures, two directions, unseen seeds. Every
 * mode is scored by where the tool is, in absolute terms: its mean distance from rest over
 * the first second, and the time until it stays within 0.5 mm. (H1's settling time is
 * relative to each run's own peak, which would penalise a controller that lowers the
 * peak.) The ideal controller's weighting is chosen per arm on one tuning tap, like the
 * joints' gains; it is then also run without the energy tank and with one bus cycle of
 * delay, to show where its advantage comes from. On the KX-6 it is also run on budget
 * actuators (designed on the datasheet model, as a joint would be).
 *
 * NX-28 adds two questions about coordination: how far the ideal controller gets when
 * joint i may use only the joints within one or two places of it, and what a damper
 * between neighbouring joints (`sim/src/analysis/coupling.ts`) adds on top of network
 * damping.
 *
 *   node experiments/src/headroom.ts [--quick]
 *
 * Writes experiments/results/headroom.json. `npm run prove` calls {@link measureHeadroom}.
 */
import { mkdir, writeFile } from "node:fs/promises";
import { attachNeighbourCoupling } from "../../sim/src/analysis/coupling.ts";
import { IdealController } from "../../sim/src/analysis/ideal.ts";
import { DampingMode } from "../../sim/src/bus/protocol.ts";
import { nodeConfigsFor } from "../../sim/src/harness/gains.ts";
import { tunedGains, type TunedMode } from "../../sim/src/harness/tuned.ts";
import { BUS_CYCLE_HZ } from "../../sim/src/index.ts";
import type { ArmSpec, Vec3 } from "../../sim/src/plant/armSpec.ts";
import { KX6, KX6_HOME, KX6L, withInitialAngles } from "../../sim/src/plant/kx6.ts";
import { tapTest } from "../../sim/src/scenarios/tap.ts";
import { type Lab, openLab, withArm } from "./runner.ts";

/** Tool distance the tap must stay within to count as settled, m. */
export const BAND = 0.5e-3;
const TUNING_SEED = 31;
const POSTURES: Record<string, readonly number[]> = {
  home: KX6_HOME,
  reach: [0, 0.527, 0.65, 1.2, 0, 0],
};
const DIRECTIONS: Record<string, Vec3> = { vertical: [0, 0, 1], sideways: [0, 1, 0] };
/** The weightings tried for the ideal controller. */
const TORQUE_WEIGHTS = [1e-4, 1e-5, 1e-6, 1e-7];
const SPEED_WEIGHTS = [1e-4, 3e-4, 1e-3];
/** As the joints' damping: a quarter of peak torque, and their 50 mJ tank. */
const SHARE = 0.25;
const TANK_J = 0.05;

export interface TapScore {
  /** Mean tool distance from rest over the first second after the tap, mm. */
  readonly meanMm: number;
  /** Time until the tool stays within {@link BAND}, s. */
  readonly settleS: number;
  readonly peakMm: number;
}

/** Neighbour coupling strengths tried, relative to the joints' network gains. */
export const COUPLINGS = [0.1, 0.3, 1] as const;
type Coupling = `network + neighbour coupling ${(typeof COUPLINGS)[number]}`;
export type Variant =
  | TunedMode
  | "ideal"
  | "ideal, no tank"
  | "ideal, 1 cycle late"
  | "ideal, neighbours only"
  | "ideal, two joints either side"
  | Coupling;
interface Case {
  arm: string;
  spec: ArmSpec;
  key: string;
  actuators?: string;
}

function score(deviation: Float64Array, peak: number): TapScore {
  const second = Math.min(deviation.length, BUS_CYCLE_HZ);
  let sum = 0;
  for (let k = 0; k < second; k++) sum += deviation[k] ?? 0;
  let last = 0;
  deviation.forEach((d, k) => {
    if (d > BAND) last = k;
  });
  return { meanMm: (1e3 * sum) / second, settleS: last / BUS_CYCLE_HZ, peakMm: 1e3 * peak };
}

async function tap(
  lab: Lab,
  c: Case,
  variant: Variant,
  posture: readonly number[],
  direction: Vec3,
  seed: number,
  design: { torqueWeight: number; speedWeight: number },
): Promise<TapScore> {
  const spec = withInitialAngles(c.spec, posture);
  const ideal = variant.startsWith("ideal");
  const coupling = variant.startsWith("network + neighbour coupling")
    ? Number(variant.split(" ").pop())
    : undefined;
  const tuned: TunedMode = ideal
    ? "off"
    : coupling !== undefined
      ? "network"
      : (variant as TunedMode);
  const gains = tunedGains(c.key, tuned);
  const options = {
    spec,
    gains,
    seed,
    ...(c.actuators ? { actuators: { families: [c.actuators] } } : {}),
  };
  return withArm(lab, options, (system) => {
    const r = tapTest(system, {
      direction,
      recordSeconds: 3,
      beforeTap: (s) => {
        s.host.applySettings({ damping: DampingMode[tuned] });
        const configs = nodeConfigsFor(spec, BUS_CYCLE_HZ, gains);
        if (coupling !== undefined) attachNeighbourCoupling(s, configs, coupling, SHARE, TANK_J);
        if (!ideal) return;
        const reach =
          variant === "ideal, neighbours only"
            ? 1
            : variant === "ideal, two joints either side"
              ? 2
              : undefined;
        const controller = IdealController.design(s, configs, {
          ...design,
          torqueShare: SHARE,
          ...(variant === "ideal, no tank" ? {} : { tankJ: TANK_J }),
          ...(variant === "ideal, 1 cycle late" ? { delayCycles: 1 } : {}),
          ...(reach !== undefined ? { reach } : {}),
        });
        controller.attach(s);
      },
    });
    return score(r.deviation, r.peak);
  });
}

export interface Headroom {
  readonly design: Record<string, { torqueWeight: number; speedWeight: number }>;
  /** Mean over postures, directions and seeds, per case and variant. */
  readonly results: Record<string, Partial<Record<Variant, TapScore>>>;
}

export async function measureHeadroom(
  lab: Lab,
  quick = false,
  log: (line: string) => void = console.log,
): Promise<Headroom> {
  const seeds = quick ? [32] : [32, 33];
  const cases: Case[] = [
    { arm: "KX-6L", spec: KX6L, key: "kx6l" },
    { arm: "KX-6", spec: KX6, key: "kx6" },
    { arm: "KX-6, budget actuators", spec: KX6, key: "kx6", actuators: "grade-budget" },
  ];
  const design: Headroom["design"] = {};
  const results: Headroom["results"] = {};
  for (const c of cases) {
    // Weighting: the best mean distance on one tuning tap (home, vertical).
    if (!design[c.key]) {
      let best = { torqueWeight: 1e-6, speedWeight: 1e-4, meanMm: Infinity };
      for (const torqueWeight of TORQUE_WEIGHTS) {
        for (const speedWeight of SPEED_WEIGHTS) {
          const s = await tap(lab, c, "ideal", KX6_HOME, [0, 0, 1], TUNING_SEED, {
            torqueWeight,
            speedWeight,
          });
          if (s.meanMm < best.meanMm) best = { torqueWeight, speedWeight, meanMm: s.meanMm };
        }
      }
      design[c.key] = { torqueWeight: best.torqueWeight, speedWeight: best.speedWeight };
      log(`headroom ${c.arm}: ideal weighting ${JSON.stringify(design[c.key])}`);
    }
    const weighting = design[c.key] ?? { torqueWeight: 1e-6, speedWeight: 1e-4 };
    const row: Partial<Record<Variant, TapScore>> = {};
    const variants: Variant[] = c.actuators
      ? ["off", "local", "network", "ideal"]
      : [
          "off",
          "local",
          "network",
          ...COUPLINGS.map((k): Coupling => `network + neighbour coupling ${k}`),
          "ideal",
          "ideal, no tank",
          "ideal, 1 cycle late",
          "ideal, neighbours only",
          "ideal, two joints either side",
        ];
    for (const variant of variants) {
      const scores: TapScore[] = [];
      for (const posture of Object.values(POSTURES))
        for (const direction of Object.values(DIRECTIONS))
          for (const seed of seeds)
            scores.push(await tap(lab, c, variant, posture, direction, seed, weighting));
      const mean = (f: (s: TapScore) => number) =>
        scores.reduce((sum, s) => sum + f(s), 0) / scores.length;
      row[variant] = {
        meanMm: mean((s) => s.meanMm),
        settleS: mean((s) => s.settleS),
        peakMm: mean((s) => s.peakMm),
      };
      const r = row[variant];
      log(
        `headroom ${c.arm} ${variant.padEnd(32)} mean ${r.meanMm.toFixed(3)} mm, within 0.5 mm after ${r.settleS.toFixed(3)} s, peak ${r.peakMm.toFixed(2)} mm`,
      );
    }
    results[c.arm] = row;
  }
  return { design, results };
}

if (import.meta.main) {
  const result = await measureHeadroom(await openLab(), process.argv.includes("--quick"));
  await mkdir(new URL("../results/", import.meta.url), { recursive: true });
  await writeFile(
    new URL("../results/headroom.json", import.meta.url),
    `${JSON.stringify(result, null, 2)}\n`,
  );
}
