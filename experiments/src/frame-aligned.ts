/**
 * NX-32: a colleague's frame-aligned damping law (`sim/src/analysis/frameAligned.ts`) run
 * as written, without an energy tank, outside the joints, on a vertical tap at home.
 * Swept over the edge gain (a multiple of the network-damping gains) and bus delay, both
 * arms; with no delay also on the linearised arm (spectral radius of the closed loop).
 *
 *   node experiments/src/frame-aligned.ts
 *
 * Writes experiments/results/frame-aligned.json.
 */
import { mkdir, writeFile } from "node:fs/promises";
import {
  attachFrameAligned,
  type FrameAlignedOptions,
  frameAlignedGain,
} from "../../sim/src/analysis/frameAligned.ts";
import { heldPlant } from "../../sim/src/analysis/ideal.ts";
import { DampingMode } from "../../sim/src/bus/protocol.ts";
import { nodeConfigsFor } from "../../sim/src/harness/gains.ts";
import { tunedGains } from "../../sim/src/harness/tuned.ts";
import { BUS_CYCLE_HZ } from "../../sim/src/index.ts";
import { KX6, KX6_HOME, KX6L, withInitialAngles } from "../../sim/src/plant/kx6.ts";
import { tapTest } from "../../sim/src/scenarios/tap.ts";
import type { Matrix } from "../../sim/src/util/matrix.ts";
import { BAND } from "./headroom.ts";
import { openLab, withArm } from "./runner.ts";

const GAINS = [0.25, 0.5, 1, 2, 4, 8];
const DELAYS = [0, 2, 4];
const VARIANTS: Record<string, Pick<FrameAlignedOptions, "sign" | "edges" | "lowpass">> = {
  "as written": { sign: 1, edges: "both" },
  flipped: { sign: -1, edges: "both" },
  "flipped, outer edge only": { sign: -1, edges: "outer" },
  "as written, with the IMU low-pass": { sign: 1, edges: "both", lowpass: true },
  "flipped, with the IMU low-pass": { sign: -1, edges: "both", lowpass: true },
  "flipped, outer edge only, with the IMU low-pass": { sign: -1, edges: "outer", lowpass: true },
};
/** A tool still this far from rest after the tap is taken as unstable, m. */
const DIVERGED = 5e-3;

interface Row {
  arm: string;
  variant: string;
  gain: number;
  delay: number;
  meanMm: number;
  settleS: number;
  /** Largest energy any joint's term put into the arm at any moment, J. */
  peakWorkJ: number;
  /** Net energy all terms together took out (negative) or put in, J. */
  netWorkJ: number;
  stable: boolean;
}

/** Growth of the closed loop per second from the spectral radius (power iteration). */
function growthPerSecond(a: Matrix): number {
  const n = a.rows;
  let x = Float64Array.from({ length: n }, (_, i) => Math.sin(1 + 7 * i));
  let log = 0;
  const steps = 40_000;
  for (let k = 0; k < steps; k++) {
    const y = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      let s = 0;
      for (let j = 0; j < n; j++) s += a.get(i, j) * (x[j] ?? 0);
      y[i] = s;
    }
    const norm = Math.hypot(...y);
    if (k >= steps / 2) log += Math.log(norm);
    x = y.map((v) => v / norm);
  }
  return (log / (steps / 2)) * BUS_CYCLE_HZ;
}

const lab = await openLab();
const rows: Row[] = [];
const linear: { arm: string; variant: string; gain: number; growth: number }[] = [];
for (const [arm, base, key] of [
  ["KX-6L", KX6L, "kx6l"],
  ["KX-6", KX6, "kx6"],
] as const) {
  const spec = withInitialAngles(base, KX6_HOME);
  const gains = tunedGains(key, "off");
  const network = tunedGains(key, "network");
  const configs = nodeConfigsFor(spec, BUS_CYCLE_HZ, network);
  const score = (deviation: Float64Array) => {
    let sum = 0;
    for (let k = 0; k < BUS_CYCLE_HZ; k++) sum += deviation[k] ?? 0;
    let last = 0;
    deviation.forEach((d, k) => {
      if (d > BAND) last = k;
    });
    const tail = deviation.slice(-BUS_CYCLE_HZ / 2);
    return {
      meanMm: (1e3 * sum) / BUS_CYCLE_HZ,
      settleS: last / BUS_CYCLE_HZ,
      stable: tail.every((d) => Number.isFinite(d) && d < DIVERGED),
    };
  };
  // References: the joints' own control only, and their network damping (with its tank).
  for (const [variant, g, mode] of [
    ["joint control only", gains, DampingMode.off],
    ["network damping, with its tank", network, DampingMode.network],
  ] as const) {
    const r = await withArm(lab, { spec, gains: g, seed: 32 }, (s) =>
      score(
        tapTest(s, { recordSeconds: 3, beforeTap: (x) => x.host.applySettings({ damping: mode }) })
          .deviation,
      ),
    );
    rows.push({ arm, variant, gain: 0, delay: 0, peakWorkJ: 0, netWorkJ: 0, ...r });
    console.log(`${arm} ${variant}: ${r.meanMm.toFixed(3)} mm, ${r.settleS.toFixed(3)} s`);
  }
  // The linearised arm with the law and no delay.
  const held = await withArm(lab, { spec, gains, seed: 32 }, (s) => {
    s.run(5);
    const plant = heldPlant(s, nodeConfigsFor(spec, BUS_CYCLE_HZ, gains));
    const laws = Object.fromEntries(
      Object.entries(VARIANTS)
        .filter(([, v]) => !v.lowpass)
        .flatMap(([variant, v]) =>
          GAINS.map((gain) => [`${variant}/${gain}`, frameAlignedGain(s, configs, { gain, ...v })]),
        ),
    );
    return { ...plant, laws };
  });
  linear.push({ arm, variant: "joint control only", gain: 0, growth: growthPerSecond(held.held) });
  for (const [variant, options] of Object.entries(VARIANTS)) {
    for (const gain of GAINS) {
      // The linear check has no filters: run once per law, without the low-pass.
      const k = options.lowpass ? undefined : held.laws[`${variant}/${gain}`];
      const growth = k ? growthPerSecond(held.held.plus(held.b.times(k))) : Number.NaN;
      if (k) linear.push({ arm, variant, gain, growth });
      for (const delay of DELAYS) {
        const r = await withArm(lab, { spec, gains, seed: 32 }, (s) => {
          let probe = undefined as ReturnType<typeof attachFrameAligned> | undefined;
          const tap = tapTest(s, {
            recordSeconds: 3,
            beforeTap: (x) => {
              x.host.applySettings({ damping: DampingMode.off });
              probe = attachFrameAligned(x, configs, { gain, delayCycles: delay, ...options });
            },
          });
          const work = probe ? Array.from(probe.work) : [];
          return {
            ...score(tap.deviation),
            peakWorkJ: probe?.peakWork ?? 0,
            netWorkJ: work.reduce((a, b) => a + b, 0),
          };
        });
        rows.push({ arm, variant, gain, delay, ...r });
        console.log(
          `${arm} ${variant.padEnd(26)} gain ${String(gain).padStart(4)} delay ${delay}: ` +
            `${r.stable ? `${r.meanMm.toFixed(3)} mm, ${r.settleS.toFixed(3)} s` : "UNSTABLE"}; ` +
            `peak work in ${(1e3 * r.peakWorkJ).toFixed(1)} mJ, net ${(1e3 * r.netWorkJ).toFixed(1)} mJ` +
            (delay === 0 && Number.isFinite(growth)
              ? `; linear growth ${growth.toFixed(2)}/s`
              : ""),
        );
      }
    }
  }
}
await mkdir(new URL("../results/", import.meta.url), { recursive: true });
await writeFile(
  new URL("../results/frame-aligned.json", import.meta.url),
  `${JSON.stringify({ rows, linear }, null, 2)}\n`,
);
