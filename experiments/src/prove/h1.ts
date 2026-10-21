/**
 * H1: networked damping settles vibration faster. Tap settling time on both arms, in each
 * damping mode with its own tuned gains, at two postures and in two tap directions, over
 * seeds the tuner never saw. The worst posture counts.
 */
import { DampingMode } from "../../../sim/src/bus/protocol.ts";
import { tunedGains, type TunedMode } from "../../../sim/src/harness/tuned.ts";
import { BUS_CYCLE_HZ } from "../../../sim/src/index.ts";
import type { Vec3 } from "../../../sim/src/plant/armSpec.ts";
import { KX6, KX6_HOME, KX6L, withInitialAngles } from "../../../sim/src/plant/kx6.ts";
import { tapTest } from "../../../sim/src/scenarios/tap.ts";
import { withArm } from "../runner.ts";
import type { Finding, ProveContext } from "./findings.ts";
import { lineChart } from "./svg.ts";

const MODES: [TunedMode, DampingMode][] = [
  ["off", DampingMode.off],
  ["local", DampingMode.local],
  ["network", DampingMode.network],
];
const POSTURES: Record<string, readonly number[]> = {
  home: KX6_HOME,
  // Forearm stretched out: longer lever, lower first mode.
  reach: [0, 0.527, 0.65, 1.2, 0, 0],
};
const DIRECTIONS: Record<string, Vec3> = { vertical: [0, 0, 1], sideways: [0, 1, 0] };

interface Tap {
  readonly settle: number;
  readonly deviation: Float64Array;
}

export async function h1(ctx: ProveContext): Promise<Finding> {
  const seeds = ctx.quick ? [31] : [31, 32, 33];
  const rows: string[] = [];
  const ratios: { arm: string; vsOff: number; vsLocal: number }[] = [];
  let plotted: Record<string, Float64Array> = {};
  for (const [arm, base] of [
    ["KX-6L", KX6L],
    ["KX-6", KX6],
  ] as const) {
    const key = arm === "KX-6L" ? "kx6l" : "kx6";
    const worst: Record<string, number> = {};
    for (const [posture, angles] of Object.entries(POSTURES)) {
      const spec = withInitialAngles(base, angles);
      const means: Record<string, number> = {};
      for (const [name, mode] of MODES) {
        const taps: Tap[] = [];
        for (const seed of seeds) {
          for (const direction of Object.values(DIRECTIONS)) {
            taps.push(
              await withArm(lab(ctx), { spec, gains: tunedGains(key, name), seed }, (system) => {
                const r = tapTest(system, {
                  direction,
                  recordSeconds: 3,
                  beforeTap: (s) => s.host.applySettings({ damping: mode }),
                });
                return { settle: r.settlingTime, deviation: r.deviation };
              }),
            );
          }
        }
        means[name] = taps.reduce((sum, t) => sum + t.settle, 0) / taps.length;
        worst[name] = Math.max(worst[name] ?? 0, means[name]);
        if (arm === "KX-6L" && posture === "home") {
          const first = taps[0];
          if (first) plotted = { ...plotted, [name]: first.deviation };
        }
      }
      rows.push(
        `| ${arm} | ${posture} | ${fmt(means.off)} | ${fmt(means.local)} | ${fmt(means.network)} |`,
      );
      ctx.log(
        `H1 ${arm} ${posture}: off ${fmt(means.off)} local ${fmt(means.local)} network ${fmt(means.network)}`,
      );
    }
    ratios.push({
      arm,
      vsOff: (worst.off ?? 0) / (worst.network ?? 1),
      vsLocal: (worst.local ?? 0) / (worst.network ?? 1),
    });
  }
  const step = 20;
  const series = Object.entries(plotted).map(([label, d]) => ({
    label:
      label === "off"
        ? "joint control only"
        : label === "local"
          ? "joint-local damping"
          : "network damping",
    x: Array.from({ length: Math.floor(d.length / step) }, (_, k) => (k * step) / BUS_CYCLE_HZ),
    y: Array.from({ length: Math.floor(d.length / step) }, (_, k) => (d[k * step] ?? 0) * 1e3),
  }));
  const plot = await ctx.plot(
    "h1-tap.svg",
    lineChart(
      "KX-6L, one vertical tap: tool distance from rest",
      series,
      "time after the tap, s",
      "mm",
    ),
  );
  const best = ratios.reduce((a, b) => (a.vsOff < b.vsOff ? a : b));
  const met = ratios.every((r) => r.vsOff >= 10 && r.vsLocal >= 3);
  const someGain = ratios.every((r) => r.vsOff > 1 && r.vsLocal > 1);
  return {
    id: "H1",
    title: "Networked damping settles vibration faster",
    target: "≥ 10× faster than network off, ≥ 3× than joint-local damping (worst posture)",
    measured: ratios
      .map((r) => `${r.arm}: ${r.vsOff.toFixed(2)}× vs off, ${r.vsLocal.toFixed(2)}× vs local`)
      .join("; "),
    verdict: met ? "met" : someGain ? "partly met" : "not met",
    detail: [
      `Mean settling time to the 5 % band, s (${seeds.length} seed${seeds.length > 1 ? "s" : ""} × 2 tap directions; each mode with its own tuned gains):`,
      "",
      "| Arm | Posture | Joint control only | Joint-local | Network |",
      "|---|---|---|---|---|",
      ...rows,
      "",
      someGain
        ? `Network damping settles faster than both alternatives on both arms (smallest gain: ${best.arm}, ${best.vsOff.toFixed(2)}× vs off). ${met ? "" : "The targets assumed an arm whose structural modes are almost undamped; a tuned joint servo already damps them well, so there is less left to gain (NX-11)."}`
        : [
            `Network damping settles ${range(ratios.map((r) => r.vsOff))}× faster than joint control alone, but against joint-local damping it is ${range(ratios.map((r) => r.vsLocal))}× at the worst posture: about even, and slower in some postures and seeds the tuner never saw (the tuner found it 1.1–1.3× faster at its own seeds, NX-11).`,
            "The targets assumed an arm whose structural modes are almost undamped. A tuned joint servo already damps them well, and joint-local damping on the joint's own encoders gets most of what is left. What the network buys is that speed at about a sixth of joint-local damping's rest jitter (a gyro needs no differentiation) and a damping term whose energy is bounded by measured power (H6).",
          ].join(" "),
    ].join("\n"),
    plots: [plot],
  };
}

const range = (xs: number[]) => `${Math.min(...xs).toFixed(2)}–${Math.max(...xs).toFixed(2)}`;
const fmt = (x: number | undefined) => (x === undefined ? "–" : x.toFixed(3));
const lab = (ctx: ProveContext) => ctx.lab;
