/**
 * Actuator grades (NX-26): the same arm on premium and budget actuators, next to the
 * datasheet arm (`experiments/src/grades.ts`). Reported, not a hypothesis of the plan.
 */
import { type GradeOutcome, measureGrades, type Score } from "../grades.ts";
import type { Finding, ProveContext } from "./findings.ts";

const f2 = (x: number | undefined) => (x === undefined ? "—" : x.toFixed(2));
const f3 = (x: number | undefined) => (x === undefined ? "—" : x.toFixed(3));
const score = (s: Score | undefined) => (s ? `${s.right} / ${s.wrong} / ${s.none}` : "—");

export async function grades(ctx: ProveContext): Promise<Finding> {
  const trials = ctx.quick ? 4 : 10;
  const r = await measureGrades(ctx.lab, trials, ctx.log);
  const rows = Object.entries(r) as [string, GradeOutcome][];
  const budget = r.budget;
  const premium = r.premium;
  const vibrationCut = (g: GradeOutcome | undefined) =>
    g ? 1 - g.damping.network.vibrationDegS / g.damping.off.vibrationDegS : Number.NaN;
  const sweepGain = (g: GradeOutcome | undefined) =>
    g?.models.nominal && g.models.learned
      ? g.models.nominal.sweepMrad / g.models.learned.sweepMrad
      : Number.NaN;
  return {
    id: "Grades",
    title: "Budget and premium actuators",
    target:
      "Reported (NX-26): how each grade behaves, and what the network and the learned models change",
    measured:
      `network damping cuts task vibration ${(100 * vibrationCut(premium)).toFixed(0)} % (premium), ` +
      `${(100 * vibrationCut(budget)).toFixed(0)} % (budget); learned models cut sweep deviation ` +
      `${f2(sweepGain(premium))}× (premium), ${f2(sweepGain(budget))}× (budget)`,
    verdict: "reported",
    detail: [
      "KX-6. Ideal: datasheet actuators. Premium and budget: every joint drawn from the grade (`sim/src/actuators/grades.ts`); budget adds current-sensing offset (ripple once per electrical turn) and gain mismatch (twice per turn), strong cogging, stickier friction and gear play. Each grade's learned models were trained on its own commissioning run, like the open units.",
      "",
      "**Damping**, with each joint's datasheet model and, for the graded units, its learned model. Tap: settling time to the 5 % band and peak; task: a figure of eight from the VLA computer, tracking RMS and the vibration joints 2–6 feel (mean, °/s RMS above 3 Hz):",
      "",
      "| Grade | Tap, joint control only | Tap, network | Task, joint control only | Task, network |",
      "|---|---|---|---|---|",
      ...rows.flatMap(([name, g]) =>
        (
          [
            ["datasheet", g.damping],
            ["learned", g.dampingLearned],
          ] as const
        ).flatMap(([label, d]) =>
          d
            ? [
                `| ${name}, ${label} models | ${f3(d.off.tapSettleS)} s, ${f2(d.off.tapPeakMm)} mm | ${f3(d.network.tapSettleS)} s, ${f2(d.network.tapPeakMm)} mm | ${f2(d.off.eightMm)} mm, ${f2(d.off.vibrationDegS)} °/s | ${f2(d.network.eightMm)} mm, ${f2(d.network.vibrationDegS)} °/s |`,
              ]
            : [],
        ),
      ),
      "",
      "**Models** (network damping). Sweep: how far the joints stray from the ideal arm during slow sweeps (mrad RMS); tasks: tracking RMS; tap: settling time:",
      "",
      "| Grade | Model | Sweep | Figure of eight | Pick and place | Tap |",
      "|---|---|---|---|---|---|",
      ...rows.flatMap(([name, g]) =>
        (["nominal", "learned"] as const)
          .filter((m) => g.models[m])
          .map((m) => {
            const o = g.models[m];
            return `| ${name} | ${m === "nominal" ? "datasheet" : "learned"} | ${f3(o?.sweepMrad)} | ${f2(o?.figureEightMm)} mm | ${f2(o?.pickAndPlaceMm)} mm | ${f3(o?.tapSettleS)} s |`;
          }),
      ),
      "",
      `**Report to source.** ${trials} blinded trials per grade and model (a random joint gets a gear-mesh ripple of 5–15 % of rated; right / wrong / missed), and a figure of eight with nothing injected (joints that claimed a disturbance of their own, source faults):`,
      "",
      "| Grade | Residual | Trials right / wrong / missed | Nothing injected: claims | Faults |",
      "|---|---|---|---|---|",
      ...rows.flatMap(([name, g]) =>
        (["nominal", "learned"] as const)
          .filter((m) => g.attribution[m])
          .map(
            (m) =>
              `| ${name} | ${m === "nominal" ? "datasheet" : "learned"} | ${score(g.attribution[m])} | ${g.unprompted[m]?.claims.length ? g.unprompted[m]?.claims.map((j) => `J${j}`).join(", ") : "none"} | ${g.unprompted[m]?.faults ?? "—"} |`,
          ),
      ),
    ].join("\n"),
  };
}
