/**
 * H10b and H10d: the learned models in closed loop (`experiments/src/learned-models.ts`).
 */
import { measureLearnedModels, UNITS } from "../learned-models.ts";
import type { Finding, ProveContext } from "./findings.ts";
import { barChart } from "./svg.ts";

export async function h10bd(ctx: ProveContext): Promise<Finding[]> {
  const units = ctx.quick ? UNITS.slice(0, 1) : UNITS;
  const { h10b, h10d } = await measureLearnedModels(ctx.lab, units, ctx.log);
  const models = ["nominal", "fitted", "learned"] as const;
  const mean = (model: (typeof models)[number]) =>
    units.reduce((sum, u) => sum + (h10b[u]?.[model]?.sweepDeviationMrad ?? 0), 0) / units.length;
  const [nominal, fitted, learned] = models.map(mean) as [number, number, number];
  const rows = units.flatMap((u) =>
    ["oracle", ...models].map((m) => {
      const o = h10b[u]?.[m];
      if (!o) return "";
      return `| ${u} | ${m === "oracle" ? "reference (datasheet actuators)" : m} | ${o.sweepDeviationMrad.toFixed(3)} | ${o.figureEightMm.toFixed(2)} | ${o.pickAndPlaceMm.toFixed(2)} | ${o.tapSettleS.toFixed(3)} |`;
    }),
  );
  const plot = await ctx.plot(
    "h10b-sweeps.svg",
    barChart(
      "Joints' deviation from the datasheet-actuator arm during slow sweeps",
      units.map((u) => ({
        label: u,
        values: models.map((m) => h10b[u]?.[m]?.sweepDeviationMrad ?? 0),
      })),
      ["datasheet model", "online RLS fit", "learned model"],
      "mrad RMS",
    ),
  );
  const broken = Object.entries(h10d);
  const bounded = broken.every(
    ([, o]) => Number.isFinite(o.figureEightMm) && o.figureEightMm < 100 && o.tapTailMm < 10,
  );
  const tankHeld = broken.every(([, o]) => o.maxDampingWorkJ <= 0.05);
  const settleText = (s: number) => (Number.isFinite(s) ? `${s.toFixed(3)} s` : "not within 2 s");
  return [
    {
      id: "H10b",
      title: "The learned model improves the arm",
      target: "PHNN measurably better than RLS; reported against Oracle",
      measured: `slow-sweep deviation from the datasheet-actuator arm: learned ${learned.toFixed(3)}, datasheet ${nominal.toFixed(3)}, RLS ${fitted.toFixed(3)} mrad (mean of ${units.length} unit${units.length > 1 ? "s" : ""})`,
      verdict:
        learned < fitted && learned < nominal ? "met" : learned < fitted ? "partly met" : "not met",
      detail: [
        "Each unit is a KX-6 with hidden actuators and its committed models. The headline is how far the joints stray, during slow sweeps, from the same arm with datasheet actuators (the reference); task path errors and tap settling are shown for context.",
        "",
        "| Unit | Model | Sweep deviation, mrad | Figure of eight, mm | Pick and place, mm | Tap settling, s |",
        "|---|---|---|---|---|---|",
        ...rows.filter(Boolean),
        "",
        "The H1 tap and H2 tasks barely change with any model: the stiff output-encoder servo already rejects most of what the actuators do differently. Where friction matters (slow motion), the learned feed-forward brings the arm closest to the reference; the online RLS fit chases what its structure cannot represent and can end worse than the datasheet.",
      ].join("\n"),
      plots: [plot],
    },
    {
      id: "H10d",
      title: "A wrong model can't destabilise the arm",
      target:
        "Closed loop with a deliberately corrupted or mismatched model: never unstable; the energy tank caps the network term",
      measured: `bounded in every case: ${bounded ? "yes" : "no"}; network term within its tank: ${tankHeld ? "yes" : "no"}`,
      verdict: bounded && tankHeld ? "met" : "not met",
      detail: [
        `Unit ${units[0]}, every joint's learned model broken the same way:`,
        "",
        "| Corruption | Sweep deviation, mrad | Figure of eight, mm | Tap settling | Rest motion after the tap, mm |",
        "|---|---|---|---|---|",
        ...broken.map(
          ([name, o]) =>
            `| ${name} | ${o.sweepDeviationMrad.toFixed(3)} | ${o.figureEightMm.toFixed(2)} | ${settleText(o.tapSettleS)} | ${o.tapTailMm.toFixed(3)} |`,
        ),
        "",
        "Never unstable, but not free: the feed-forward is in the loop through the measured motor angle, so cogging ten times too large and of the wrong sign acts like ten times the real cogging, and the tool chatters at rest. A wrong model degrades performance; the servo and the energy tank keep the arm bounded.",
      ].join("\n"),
    },
  ];
}
