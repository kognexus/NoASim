/**
 * Estimator-based coordinated damping (NX-30, `experiments/src/estimation.ts`). Reported.
 */
import { measureEstimation, VARIANTS } from "../estimation.ts";
import type { TapScore } from "../headroom.ts";
import type { Finding, ProveContext } from "./findings.ts";

const cell = (s: TapScore | undefined) =>
  s
    ? `${s.meanMm.toFixed(3)} mm, ${s.settleS >= 2.9 ? "not within 3 s" : `${s.settleS.toFixed(3)} s`}`
    : "—";

export async function estimation(ctx: ProveContext): Promise<Finding> {
  const r = await measureEstimation(ctx.lab, ctx.quick, ctx.log);
  const rows: string[] = [];
  const ratios: Record<string, string[]> = {};
  for (const [arm, byBus] of Object.entries(r)) {
    for (const [bus, byVariant] of Object.entries(byBus)) {
      for (const posture of Object.keys(byVariant.network)) {
        rows.push(
          `| ${arm} | ${bus} | ${posture} | ${VARIANTS.map((v) => cell(byVariant[v][posture])).join(" | ")} |`,
        );
      }
      const each = Object.keys(byVariant.network).map(
        (p) =>
          (byVariant.network[p]?.meanMm ?? 0) /
          (byVariant["coordinated, in the joints"][p]?.meanMm ?? Infinity),
      );
      const [low, high] = [Math.min(...each).toFixed(1), Math.max(...each).toFixed(1)];
      (ratios[bus] ??= []).push(`${arm} ${low === high ? low : `${low}–${high}`}×`);
    }
  }
  return {
    id: "Estimator",
    title: "Estimator-based coordinated damping",
    target:
      "Reported (NX-30): a Kalman filter on the joints' frames and a designed gain, in the joints, against network damping",
    measured: `in the joints, after a tap, mean distance against network damping: ${Object.entries(
      ratios,
    )
      .map(([bus, each]) => `${bus} bus ${each.join(", ")}`)
      .join("; ")}`,
    verdict: "reported",
    detail: [
      "Taps as in NX-27 (two directions, unseen seeds) at three postures; the model and gains built once, at the home pose. Each cell: mean tool distance over the first second, time to stay within 0.5 mm. Lossy bus: 20 % of frames lost, 4 cycles of jitter. In coordinated mode each joint filters the state frames of the joints up to two places away while it holds still, and uses network damping while it moves or when that data is late.",
      "",
      `| Arm | Bus | Posture | ${VARIANTS.join(" | ")} |`,
      `|---|---|---|${VARIANTS.map(() => "---").join("|")}|`,
      ...rows,
      "",
      "The model is the arm linearised where it holds, with dry friction as damping of the fast part of the motor speed only, the joints' position loops and the damper's command (40 Hz low-pass) each a cycle late. On the lossy bus the joints are no better than network damping: a joint's filter needs the latest frame of every joint within its reach, no more than 4 cycles old, with the commands of its own cycle; with one frame in five lost and 4 cycles of jitter it often has not, and falls back to network damping. The same estimator outside the joints, on the frames as the host receives them, keeps its advantage there; what the joints lack is a way to carry on through missing frames.",
    ].join("\n"),
  };
}
