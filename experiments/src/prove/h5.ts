/**
 * H5 and H7: report to source (`experiments/src/report-to-source.ts`).
 */
import { DEADLINE_SECONDS, measureReportToSource } from "../report-to-source.ts";
import type { Finding, ProveContext } from "./findings.ts";
import { barChart } from "./svg.ts";

const fmt = (x: number | undefined) => (x === undefined ? "—" : `${x.toFixed(2)} s`);

export async function h5h7(ctx: ProveContext): Promise<Finding[]> {
  const trials = ctx.quick ? 20 : 100;
  const hiddenTrials = ctx.quick ? 6 : 30;
  const r = await measureReportToSource(ctx.lab, trials, hiddenTrials, ctx.log);

  const [fixed, unfixed, beyond] = r.h5;
  if (!fixed || !unfixed || !beyond) throw new Error("H5 cases missing");
  const claim = (c: typeof fixed) => c.run.firstClaim ?? Infinity;
  const fixedInTime =
    fixed.decay.decayedAfter !== undefined &&
    fixed.decay.decayedAfter < DEADLINE_SECONDS &&
    fixed.run.fault === undefined;
  const faultInTime = (c: typeof fixed) =>
    c.run.fault !== undefined && c.run.fault - claim(c) <= DEADLINE_SECONDS + 0.2;
  const h5Met = fixedInTime && faultInTime(beyond);

  const { network, local } = r.h7.score;
  const { nominal, learned } = r.h7Hidden.score;
  const plot = await ctx.plot(
    "h7-attribution.svg",
    barChart(
      "Blinded attribution trials",
      [
        {
          label: `datasheet actuators, network (${trials})`,
          values: [network.right, network.wrong, network.none],
        },
        {
          label: `datasheet actuators, alone (${trials})`,
          values: [local.right, local.wrong, local.none],
        },
        {
          label: `hidden, datasheet residual (${hiddenTrials})`,
          values: [nominal.right, nominal.wrong, nominal.none],
        },
        {
          label: `hidden, learned residual (${hiddenTrials})`,
          values: [learned.right, learned.wrong, learned.none],
        },
      ],
      ["source named, alone", "another joint also claims", "nobody claims"],
      "trials",
    ),
  );
  const share = (s: { right: number }, n: number) => s.right / Math.max(1, n);
  const h7Met =
    share(network, trials) >= 0.97 && network.wrong + network.none < local.wrong + local.none;
  const h7Count = share(network, trials) >= 0.97;
  return [
    {
      id: "H5",
      title: "Report to source works",
      target: `Compensable ripple: helper effort under 10 % before the deadline (${DEADLINE_SECONDS} s). Non-compensable: fault at the host within the deadline`,
      measured: `compensable: under 10 % ${fmt(fixed.decay.decayedAfter)} after the claim, no fault; beyond budget: fault ${fmt((beyond.run.fault ?? NaN) - claim(beyond))} after the claim`,
      verdict: h5Met ? "met" : "not met",
      detail: [
        "KX-6, joint 3 sweeping at 0.3 rad/s, torque ripple at the sixth electrical harmonic (order 864, about 41 Hz).",
        "",
        "| Case | First claim | Fault at the host | Helper effort before | Under 10 % |",
        "|---|---|---|---|---|",
        ...r.h5.map(
          (c) =>
            `| ${c.name} (${c.share * 100} % of rated) | ${fmt(c.run.firstClaim)} | ${fmt(c.run.fault)} | ${c.decay.before.toFixed(3)} N·m | ${c.decay.decayedAfter === undefined ? "—" : `${fmt(c.decay.decayedAfter)} after the claim`} |`,
        ),
        "",
        "A joint claims a disturbance when its own motor-side residual shows the order clearly above a lock-in reference and a neighbour feels the order; it then cancels it by angle-locked feed-forward, capped at 20 % of rated torque. If a neighbour still feels it 2 s after the claim, the joint reports a fault.",
      ].join("\n"),
    },
    {
      id: "H7",
      title: "Attribution",
      target:
        "Correct source joint over 100 blinded fault trials: ≥ 97/100, and fewer errors than joints alone",
      measured: `network ${network.right}/${trials}, joints alone ${local.right}/${trials}; on hidden actuators ${nominal.right}/${hiddenTrials} with the datasheet residual, ${learned.right}/${hiddenTrials} with the learned one`,
      verdict: h7Met ? "met" : h7Count ? "partly met" : "not met",
      detail: [
        `Each trial gives a random joint a gear-mesh ripple of 5–15 % of rated torque while every joint strokes at its own speed (1.0–1.6 rad/s). Right means the source, and only the source, claims it.`,
        "",
        "| Condition | Right | Wrong | Missed |",
        "|---|---|---|---|",
        `| Datasheet actuators, network | ${network.right} | ${network.wrong} | ${network.none} |`,
        `| Datasheet actuators, joints alone | ${local.right} | ${local.wrong} | ${local.none} |`,
        `| Hidden actuators, datasheet residual (network) | ${nominal.right} | ${nominal.wrong} | ${nominal.none} |`,
        `| Hidden actuators, learned residual (network) | ${learned.right} | ${learned.wrong} | ${learned.none} |`,
        "",
        'Attribution rests on each joint\'s self-test of its own motor-side residual, which arm vibration cannot reach; joints alone run the same self-test, so on datasheet actuators the network cannot do better ("fewer errors than joints alone" is not met when neither makes any). What the network adds is the corroboration that the disturbance reaches the arm, the correction and the fault verdict. On actuators nobody described, the learned residual removes most false claims. Naming the source from felt vibration alone, the first design, named the wrong joint most of the time (NX-12).',
      ].join("\n"),
      plots: [plot],
    },
  ];
}
