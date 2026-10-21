/**
 * NX-30: estimator-based coordinated damping (`sim/src/analysis/estimator.ts`) against
 * network damping. Taps as in NX-27 (two directions, unseen seeds) at three postures, on
 * a clean bus and on a lossy one (20 % of frames lost, 4 cycles of jitter). The model and
 * gains are built once, at the home pose.
 *
 *   node experiments/src/estimation.ts [--quick]
 *
 * Writes experiments/results/estimation.json. `npm run prove` calls
 * {@link measureEstimation}.
 */
import { mkdir, writeFile } from "node:fs/promises";
import { attachLqg, designLqg, loopModel } from "../../sim/src/analysis/estimator.ts";
import { DampingMode } from "../../sim/src/bus/protocol.ts";
import { nodeConfigsFor } from "../../sim/src/harness/gains.ts";
import { tunedGains } from "../../sim/src/harness/tuned.ts";
import { BUS_CYCLE_HZ } from "../../sim/src/index.ts";
import type { Vec3 } from "../../sim/src/plant/armSpec.ts";
import { KX6, KX6_HOME, KX6L, withInitialAngles } from "../../sim/src/plant/kx6.ts";
import { tapTest } from "../../sim/src/scenarios/tap.ts";
import { BAND, type TapScore } from "./headroom.ts";
import { type Lab, openLab, withArm } from "./runner.ts";

export const POSTURES: Record<string, readonly number[]> = {
  home: KX6_HOME,
  reach: [0, 0.527, 0.65, 1.2, 0, 0],
  mid: [0, 0.3, 0.4, 0.6, 0, 0],
};
const DIRECTIONS: Vec3[] = [
  [0, 0, 1],
  [0, 1, 0],
];
const BUSES = {
  clean: { latencyCycles: 1 },
  lossy: { latencyCycles: 2, jitterCycles: 4, dropRate: 0.2 },
} as const;
const SHARE = 0.25;
const TANK_J = 0.05;
/** Torque weight per arm: 30× the ideal controller's (a gain the measurements can carry). */
const WEIGHTING: Record<string, { torqueWeight: number; speedWeight: number }> = {
  kx6l: { torqueWeight: 3e-5, speedWeight: 1e-4 },
  kx6: { torqueWeight: 3e-6, speedWeight: 1e-4 },
};
/** The filter's assumed disturbance at the tool, N per cycle. */
const FORCE_NOISE = 20;

export const VARIANTS = [
  "network",
  "estimator, every frame",
  "estimator, two either side",
] as const;
export type Variant = (typeof VARIANTS)[number];

export type Estimation = Record<
  string,
  Record<keyof typeof BUSES, Record<Variant, Record<string, TapScore>>>
>;

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

export async function measureEstimation(
  lab: Lab,
  quick = false,
  log: (line: string) => void = console.log,
): Promise<Estimation> {
  const seeds = quick ? [32] : [32, 33];
  const out: Estimation = {};
  for (const [arm, base, key] of [
    ["KX-6L", KX6L, "kx6l"],
    ["KX-6", KX6, "kx6"],
  ] as const) {
    const gains = tunedGains(key, "off");
    const home = withInitialAngles(base, KX6_HOME);
    const configs = nodeConfigsFor(home, BUS_CYCLE_HZ, gains);
    const weighting = WEIGHTING[key] ?? { torqueWeight: 3e-5, speedWeight: 1e-4 };
    const design = { ...weighting, forceNoise: FORCE_NOISE };
    const { model, full, local } = await withArm(lab, { spec: home, gains, seed: 31 }, (s) => {
      s.host.applySettings({ damping: DampingMode.off });
      s.run(5.3);
      const m = loopModel(s, configs);
      return {
        model: m,
        full: designLqg(m, configs, design),
        local: designLqg(m, configs, design, 2),
      };
    });
    const byBus = {} as Estimation[string];
    for (const [busName, bus] of Object.entries(BUSES) as [
      keyof typeof BUSES,
      (typeof BUSES)[keyof typeof BUSES],
    ][]) {
      const byVariant = {} as Record<Variant, Record<string, TapScore>>;
      for (const variant of VARIANTS) {
        byVariant[variant] = {};
        for (const [posture, angles] of Object.entries(POSTURES)) {
          const spec = withInitialAngles(base, angles);
          const scores: TapScore[] = [];
          for (const direction of DIRECTIONS) {
            for (const seed of seeds) {
              const tuned = variant === "network" ? tunedGains(key, "network") : gains;
              scores.push(
                await withArm(lab, { spec, gains: tuned, seed, bus }, (system) => {
                  const r = tapTest(system, {
                    direction,
                    recordSeconds: 3,
                    beforeTap: (s) => {
                      const network = variant === "network";
                      s.host.applySettings({
                        damping: network ? DampingMode.network : DampingMode.off,
                      });
                      if (!network) {
                        const lqg = variant === "estimator, every frame" ? full : local;
                        attachLqg(s, configs, model, lqg, SHARE, TANK_J);
                      }
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
            `estimation ${arm} ${busName} ${posture.padEnd(5)} ${variant.padEnd(28)} mean ${r.meanMm.toFixed(3)} mm, within 0.5 mm after ${r.settleS.toFixed(3)} s`,
          );
        }
      }
      byBus[busName] = byVariant;
    }
    out[arm] = byBus;
  }
  return out;
}

if (import.meta.main) {
  const result = await measureEstimation(await openLab(), process.argv.includes("--quick"));
  await mkdir(new URL("../results/", import.meta.url), { recursive: true });
  await writeFile(
    new URL("../results/estimation.json", import.meta.url),
    `${JSON.stringify(result, null, 2)}\n`,
  );
}
