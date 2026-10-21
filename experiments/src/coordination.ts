/**
 * NX-29: coordinated damping from joints up to two places away. Taps as in NX-27 (two
 * directions, unseen seeds), at the home pose the gains are designed at and at the
 * stretched-out pose they are not:
 *
 * - network damping, as the joints have it;
 * - a gain designed for the structure (`designStructured`) on the plant's true state,
 *   for one and for two joints either side, and for two joints with 3 cycles of delay
 *   (designed for that delay);
 * - the measured version (`designCoordinated`), which runs only on the joints' state
 *   frames as they arrive over the bus.
 *
 *   node experiments/src/coordination.ts [--quick]
 *
 * Writes experiments/results/coordination.json. `npm run prove` calls
 * {@link measureCoordination}.
 */
import { mkdir, writeFile } from "node:fs/promises";
import { attachCoordinated, designCoordinated } from "../../sim/src/analysis/coordinated.ts";
import { IdealController } from "../../sim/src/analysis/ideal.ts";
import { designStructured } from "../../sim/src/analysis/structured.ts";
import { DampingMode } from "../../sim/src/bus/protocol.ts";
import type { ArmSystem } from "../../sim/src/harness/armSystem.ts";
import { nodeConfigsFor } from "../../sim/src/harness/gains.ts";
import { tunedGains } from "../../sim/src/harness/tuned.ts";
import { BUS_CYCLE_HZ } from "../../sim/src/index.ts";
import type { Vec3 } from "../../sim/src/plant/armSpec.ts";
import { KX6, KX6_HOME, KX6L, withInitialAngles } from "../../sim/src/plant/kx6.ts";
import { tapTest } from "../../sim/src/scenarios/tap.ts";
import type { Matrix } from "../../sim/src/util/matrix.ts";
import { BAND, type TapScore } from "./headroom.ts";
import { type Lab, openLab, withArm } from "./runner.ts";

const POSTURES: Record<string, readonly number[]> = {
  home: KX6_HOME,
  reach: [0, 0.527, 0.65, 1.2, 0, 0],
};
const DIRECTIONS: Vec3[] = [
  [0, 0, 1],
  [0, 1, 0],
];
const SHARE = 0.25;
const TANK_J = 0.05;
const DESIGN_SEED = 31;
/** The ideal controller's weighting per arm (NX-27's tuning tap). */
const WEIGHTING: Record<string, { torqueWeight: number; speedWeight: number }> = {
  kx6l: { torqueWeight: 1e-6, speedWeight: 1e-4 },
  kx6: { torqueWeight: 1e-7, speedWeight: 1e-4 },
};

export const VARIANTS = [
  "network",
  "structured, neighbours, true state",
  "structured, two either side, true state",
  "structured, two either side, true state 3 cycles late",
  "measured, two either side, from the bus",
] as const;
export type Variant = (typeof VARIANTS)[number];

export interface Coordination {
  /** Per arm, variant and posture: taps averaged over directions and seeds. */
  readonly results: Record<string, Record<Variant, Record<string, TapScore>>>;
  /** Linear-model cost of each design at home, relative to no gain. */
  readonly cost: Record<string, Record<string, number>>;
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

export async function measureCoordination(
  lab: Lab,
  quick = false,
  log: (line: string) => void = console.log,
): Promise<Coordination> {
  const seeds = quick ? [32] : [32, 33];
  const results: Coordination["results"] = {};
  const cost: Coordination["cost"] = {};
  for (const [arm, base, key] of [
    ["KX-6L", KX6L, "kx6l"],
    ["KX-6", KX6, "kx6"],
  ] as const) {
    const weighting = WEIGHTING[key] ?? { torqueWeight: 1e-6, speedWeight: 1e-4 };
    const gains = tunedGains(key, "off");
    const home = withInitialAngles(base, KX6_HOME);
    const configs = nodeConfigsFor(home, BUS_CYCLE_HZ, gains);
    // Designs, at home, on an arm that has settled with its damping off.
    const designs = await withArm(lab, { spec: home, gains, seed: DESIGN_SEED }, (system) => {
      system.run(5.3);
      const one = designStructured(system, configs, { ...weighting, reach: 1 });
      const two = designStructured(system, configs, { ...weighting, reach: 2 });
      const late = designStructured(system, configs, { ...weighting, reach: 2, delayCycles: 3 });
      const measured = designCoordinated([system], configs, {
        ...weighting,
        reach: 2,
        delayCycles: 3,
        iterations: 150,
      });
      return { one, two, late, measured };
    });
    cost[arm] = {
      full: designs.two.cost.full / designs.two.cost.none,
      neighbours: designs.one.cost.structured / designs.one.cost.none,
      "two either side": designs.two.cost.structured / designs.two.cost.none,
      "two either side, 3 cycles late": designs.late.cost.structured / designs.late.cost.none,
      "measured from the bus": designs.measured.cost[0]?.measured ?? Infinity,
    };
    log(`coordination ${arm}: design costs ${JSON.stringify(cost[arm])}`);
    const attach = (variant: Variant, system: ArmSystem) => {
      const state = (gain: Matrix, delayCycles = 0) =>
        IdealController.withGain(gain, configs, {
          torqueShare: SHARE,
          tankJ: TANK_J,
          delayCycles,
        }).attach(system);
      if (variant === "structured, neighbours, true state") state(designs.one.gain);
      if (variant === "structured, two either side, true state") state(designs.two.gain);
      if (variant === "structured, two either side, true state 3 cycles late")
        state(designs.late.gain, 3);
      if (variant === "measured, two either side, from the bus")
        attachCoordinated(system, configs, designs.measured.gain, SHARE, TANK_J);
    };
    const byVariant = {} as Record<Variant, Record<string, TapScore>>;
    for (const variant of VARIANTS) {
      byVariant[variant] = {};
      for (const [posture, angles] of Object.entries(POSTURES)) {
        const spec = withInitialAngles(base, angles);
        const scores: TapScore[] = [];
        for (const direction of DIRECTIONS) {
          for (const seed of seeds) {
            const tuned = variant === "network" ? tunedGains(key, "network") : gains;
            const options = { spec, gains: tuned, seed };
            scores.push(
              await withArm(lab, options, (system) => {
                const r = tapTest(system, {
                  direction,
                  recordSeconds: 3,
                  beforeTap: (s) => {
                    const damping = variant === "network" ? DampingMode.network : DampingMode.off;
                    s.host.applySettings({ damping });
                    attach(variant, s);
                  },
                });
                return score(r.deviation, r.peak);
              }),
            );
          }
        }
        const mean = (f: (s: TapScore) => number) =>
          scores.reduce((sum, s) => sum + f(s), 0) / scores.length;
        const r = {
          meanMm: mean((s) => s.meanMm),
          settleS: mean((s) => s.settleS),
          peakMm: mean((s) => s.peakMm),
        };
        byVariant[variant][posture] = r;
        log(
          `coordination ${arm} ${posture.padEnd(5)} ${variant.padEnd(52)} mean ${r.meanMm.toFixed(3)} mm, within 0.5 mm after ${r.settleS.toFixed(3)} s`,
        );
      }
    }
    results[arm] = byVariant;
  }
  return { results, cost };
}

if (import.meta.main) {
  const result = await measureCoordination(await openLab(), process.argv.includes("--quick"));
  await mkdir(new URL("../results/", import.meta.url), { recursive: true });
  await writeFile(
    new URL("../results/coordination.json", import.meta.url),
    `${JSON.stringify(result, null, 2)}\n`,
  );
}
