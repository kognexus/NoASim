/**
 * Coordinated models, learned from the neighbourhood (NX-31,
 * `experiments/src/coordinated-models.ts`). Reported.
 */
import { measureCoordinatedModels, VARIANTS } from "../coordinated-models.ts";
import type { Finding, ProveContext } from "./findings.ts";

const pct = (now: number, before: number) => `${Math.round((100 * (before - now)) / before)} %`;

export async function neighbourhood(ctx: ProveContext): Promise<Finding> {
  const r = await measureCoordinatedModels(ctx.lab, ctx.quick, ctx.log);
  const tracking: string[] = [];
  const holding: string[] = [];
  const summary: string[] = [];
  let taskNames: string[] = [];
  for (const [arm, byVariant] of Object.entries(r)) {
    const today = byVariant["today's joints"];
    const coordinated = byVariant["coordinated model"];
    taskNames = Object.keys(today.tracking);
    for (const variant of VARIANTS) {
      const s = byVariant[variant];
      tracking.push(
        `| ${arm} | ${variant} | ${taskNames.map((t) => `${s.tracking[t]?.mm.toFixed(3)} mm`).join(" | ")} | ${(
          taskNames.reduce((a, t) => a + (s.tracking[t]?.vibration ?? 0), 0) / taskNames.length
        ).toFixed(2)} °/s |`,
      );
      holding.push(
        `| ${arm} | ${variant} | ${Object.values(s.hold)
          .map((h) => `${h.toFixed(3)} mm`)
          .join(
            " | ",
          )} | ${s.tap.meanMm.toFixed(3)} mm, ${s.tap.settleS.toFixed(3)} s | ${s.shake.mm.toFixed(3)} mm, ${s.shake.vibration.toFixed(2)} °/s | ${s.push.peakMm.toFixed(1)} mm, ${s.push.meanMm.toFixed(2)} mm |`,
      );
    }
    const gain = taskNames.map((t) =>
      pct(coordinated.tracking[t]?.mm ?? 0, today.tracking[t]?.mm ?? 1),
    );
    const hold = (s: typeof today) => Object.values(s.hold).reduce((a, b) => a + b, 0) / 3;
    summary.push(
      `${arm}: tracking ${gain.join("/")} lower, hold offset ${hold(today).toFixed(2)} → ${hold(coordinated).toFixed(2)} mm, tap ${pct(coordinated.tap.meanMm, today.tap.meanMm)} lower`,
    );
  }
  return {
    id: "Neighbourhood",
    title: "Coordinated models, learned from the neighbourhood",
    target:
      "Reported (NX-31): each joint's learned share of its neighbourhood's dynamics (its inertia row and its link's sag), in the joints, against today's joints",
    measured: `coordinated model against today's joints, ${summary.join("; ")}`,
    verdict: "reported",
    detail: [
      "Each joint learns, from neighbourhood commissioning (all joints excited at once on interleaved frequencies, at three postures), its row of the inertia of the joints up to two places away and its link's compliance; the compliance from the next joint's gyro, which sits at the link's far end. At run time it feeds forward the inertial torque of its own and its neighbours' published references, and shifts its angle by half its own link's predicted sag and half its inner neighbour's. The own-only model is the same fit with the joint's own angle and acceleration only. Network damping throughout, except in the last variant (NX-30's coordinated damping). Datasheet actuators, the arm's 3 kg payload, as commissioned.",
      "",
      `**Tracking** (tool path error RMS; vibration: mean over joints 2–6 and tasks):`,
      "",
      `| Arm | Variant | ${taskNames.join(" | ")} | Vibration |`,
      `|---|---|${taskNames.map(() => "---").join("|")}|---|`,
      ...tracking,
      "",
      "**Holding and disturbances** (hold: tool offset from the commanded pose at home, reach and between; tap: mean distance over the first second, time to stay within 0.5 mm; shake: 15 N sideways near the first mode for 3 s, mean distance over 4 s and joint vibration; push: 150 N sideways for 0.4 s, peak and mean distance over 2 s):",
      "",
      "| Arm | Variant | Hold, home | Hold, reach | Hold, between | Tap | Shake | Push |",
      "|---|---|---|---|---|---|---|---|",
      ...holding,
    ].join("\n"),
  };
}
