/**
 * Headroom for learned damping (NX-27): the joints' damping against an ideal controller
 * that sees the plant's whole state (`experiments/src/headroom.ts`). Reported.
 */
import { COUPLINGS, measureHeadroom, type TapScore } from "../headroom.ts";
import type { Finding, ProveContext } from "./findings.ts";

const cell = (s: TapScore | undefined) =>
  s ? `${s.meanMm.toFixed(3)} mm, ${s.settleS.toFixed(3)} s, ${s.peakMm.toFixed(1)} mm` : "—";

export async function headroom(ctx: ProveContext): Promise<Finding> {
  const r = await measureHeadroom(ctx.lab, ctx.quick, ctx.log);
  const ratio = (arm: string) => {
    const row = r.results[arm];
    return row?.network && row.ideal ? row.network.meanMm / row.ideal.meanMm : Number.NaN;
  };
  const variants = [
    ["off", "Joint control only"],
    ["local", "Joint-local damping"],
    ["network", "Network damping"],
    ["ideal", "Ideal (whole state, no delay)"],
    ["ideal, no tank", "Ideal without the energy tank"],
    ["ideal, 1 cycle late", "Ideal, one bus cycle late"],
    ["ideal, two joints either side", "Ideal, each joint seeing two joints either side"],
    ["ideal, neighbours only", "Ideal, each joint seeing its neighbours only"],
    ...COUPLINGS.map(
      (k) =>
        [`network + neighbour coupling ${k}`, `Network + neighbour coupling, κ = ${k}`] as const,
    ),
  ] as const;
  const arms = Object.keys(r.results);
  return {
    id: "Headroom",
    title: "Headroom for learned damping",
    target: "Reported (NX-27): how much better any damping could do after a tap",
    measured: arms
      .filter((a) => !a.includes("budget"))
      .map((a) => `${a}: ideal ${ratio(a).toFixed(1)}× lower mean distance than network`)
      .join("; "),
    verdict: "reported",
    detail: [
      "Taps at rest as in H1 (two postures, two directions, unseen seeds). Per cell: mean tool distance from rest over the first second, time until the tool stays within 0.5 mm, peak. The ideal controller is an LQR on the linearised plant with the joints' own position loops, seeing every motor, gear and link-bending coordinate with no delay or noise, held to the joints' damping budget (a quarter of peak torque, a 50 mJ energy tank); its weighting was chosen per arm on one tuning tap. No joint can build it: it is an upper bound.",
      "",
      `| Variant | ${arms.join(" | ")} |`,
      `|---|${arms.map(() => "---").join("|")}|`,
      ...variants.map(
        ([key, label]) =>
          `| ${label} | ${arms.map((a) => cell(r.results[a]?.[key])).join(" | ")} |`,
      ),
      "",
      "The tank and a bus cycle of delay cost the ideal controller almost nothing: its advantage comes from acting on the whole arm's state at once, every joint coordinated, not from speed or from giving up passivity. The neighbour rows are NX-28: cutting the ideal gain to each joint's neighbours keeps part of the gain but leaves a slow tail, two joints either side gets most of it (a gain designed for the structure instead of cut from the full one does far better, even for neighbours only: see Coordinated, NX-29), and a damper between neighbouring joints (from a colleague's derivation; passive with fresh data) adds at most a few percent on top of network damping, and makes the arm ring when strong. On budget actuators, designed on the datasheet plant, it lowers the mean but is slow to stay within 0.5 mm: gear play and stiction are what a linear design ignores.",
    ].join("\n"),
  };
}
