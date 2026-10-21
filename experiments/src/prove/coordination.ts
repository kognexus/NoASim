/**
 * Coordinated damping from joints up to two places away (NX-29,
 * `experiments/src/coordination.ts`). Reported.
 */
import { measureCoordination, VARIANTS } from "../coordination.ts";
import type { TapScore } from "../headroom.ts";
import type { Finding, ProveContext } from "./findings.ts";

const cell = (s: TapScore | undefined) =>
  s
    ? `${s.meanMm.toFixed(3)} mm, ${s.settleS >= 2.9 ? "not within 3 s" : `${s.settleS.toFixed(3)} s`}`
    : "—";

export async function coordination(ctx: ProveContext): Promise<Finding> {
  const r = await measureCoordination(ctx.lab, ctx.quick, ctx.log);
  const arms = Object.keys(r.results);
  const columns = arms.flatMap((arm) => ["home", "reach"].map((p) => [arm, p] as const));
  return {
    id: "Coordinated",
    title: "Coordinated damping from joints two places away",
    target:
      "Reported (NX-29): a gain designed for each joint seeing the joints up to two places away, on the true state and on what the bus carries",
    measured:
      arms
        .map((arm) => {
          const t = r.results[arm]?.["structured, two either side, true state"]?.home;
          const n = r.results[arm]?.network.home;
          return t && n
            ? `${arm}: ${(n.meanMm / t.meanMm).toFixed(1)}× closer than network on the true state`
            : arm;
        })
        .join("; ") + "; from the bus alone, not yet robust",
    verdict: "reported",
    detail: [
      "Taps as in NX-27 (two directions, unseen seeds), each cell the mean tool distance over the first second and the time until the tool stays within 0.5 mm. Gains designed at the home pose; the stretched-out pose is not designed for. All inside the joints' torque cap and energy tank.",
      "",
      `| Variant | ${columns.map(([a, p]) => `${a}, ${p}`).join(" | ")} |`,
      `|---|${columns.map(() => "---").join("|")}|`,
      ...VARIANTS.map(
        (v) => `| ${v} | ${columns.map(([a, p]) => cell(r.results[a]?.[v]?.[p])).join(" | ")} |`,
      ),
      "",
      `Linear-model cost at home, relative to no gain: ${arms
        .map(
          (a) =>
            `${a}: ${Object.entries(r.cost[a] ?? {})
              .map(([k, v]) => `${k} ${v.toFixed(3)}`)
              .join(", ")}`,
        )
        .join("; ")}.`,
      "",
      "Designed for the structure (not cut from the full gain, as in NX-28), a gain that sees only the joints one or two places away matches the ideal controller on the true state, and holds up with three cycles of delay at the pose it was designed at. Run on the joints' state frames alone it is not yet robust: better than network damping at the home pose of the light arm, worse or oscillating elsewhere. The design treats the measurements as clean, delayed states; the joints' observers, the integration of the gyros and the position loops' actuation delay are not in its model. Not built into the node.",
    ].join("\n"),
  };
}
