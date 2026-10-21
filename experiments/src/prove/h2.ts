/**
 * H2: joint-level interpolation is smoother than central interpolation at equal or better
 * path error. H8: how much work moves off the host. Same runs: three scripted policy tasks
 * on the KX-6 (15 Hz policy with latency, jitter and drops; central host loop at 250 Hz
 * with late ticks), each in both modes.
 */
import type { InterpolationMode } from "../../../sim/src/host/host.ts";
import { fastReach, figureEight, pickAndPlace, type Task } from "../../../sim/src/host/tasks.ts";
import type { ArmSystem } from "../../../sim/src/harness/armSystem.ts";
import { KX6, KX6_HOME, withInitialAngles } from "../../../sim/src/plant/kx6.ts";
import { followTask, homePose, type MotionResult } from "../../../sim/src/scenarios/motion.ts";
import { withArm } from "../runner.ts";
import type { Finding, ProveContext } from "./findings.ts";
import { barChart } from "./svg.ts";

const TASKS: Record<string, (s: ArmSystem) => Task> = {
  "figure of eight": (s) => figureEight(homePose(s)),
  "pick and place": (s) => pickAndPlace(homePose(s)),
  "fast reach": (s) => fastReach(homePose(s)),
};

interface Run extends MotionResult {
  readonly ikSolves: number;
  readonly setpoints: number;
  readonly seconds: number;
}

async function run(
  ctx: ProveContext,
  mode: InterpolationMode,
  task: string,
  seed: number,
): Promise<Run> {
  const make = TASKS[task];
  if (!make) throw new Error(task);
  return withArm(ctx.lab, { spec: withInitialAngles(KX6, KX6_HOME), seed }, (system) => {
    system.run(4);
    const t = make(system);
    const result = followTask(system, () => t, { mode });
    const load = system.host.load;
    return {
      ...result,
      ikSolves: load.ikSolves,
      setpoints: load.interpolationSteps + load.goalFramesSent,
      seconds: t.duration,
    };
  });
}

export async function h2h8(ctx: ProveContext): Promise<Finding[]> {
  const seeds = ctx.quick ? [7] : [7, 8, 9];
  const rows: string[] = [];
  const loadRows: string[] = [];
  const groups = [];
  let jerkMet = true;
  let pathMet = true;
  for (const task of Object.keys(TASKS)) {
    const sum = {
      central: { jerk: 0, path: 0, ik: 0, sent: 0, s: 0 },
      joint: { jerk: 0, path: 0, ik: 0, sent: 0, s: 0 },
    };
    for (const seed of seeds) {
      for (const [key, mode] of [
        ["central", "central"],
        ["joint", "joint-level"],
      ] as const) {
        const r = await run(ctx, mode, task, seed);
        const acc = sum[key];
        acc.jerk += r.jerkRms / seeds.length;
        acc.path += r.pathErrorRms / seeds.length;
        acc.ik += r.ikSolves / seeds.length;
        acc.sent += r.setpoints / seeds.length;
        acc.s += r.seconds / seeds.length;
      }
    }
    const { central: c, joint: j } = sum;
    const ratio = c.jerk / j.jerk;
    jerkMet &&= ratio >= 5;
    pathMet &&= j.path <= c.path;
    rows.push(
      `| ${task} | ${c.jerk.toFixed(0)} → ${j.jerk.toFixed(0)} m/s³ | ${ratio.toFixed(1)}× | ${(c.path * 1e3).toFixed(1)} → ${(j.path * 1e3).toFixed(1)} mm |`,
    );
    loadRows.push(
      `| ${task} | ${(c.ik / c.s).toFixed(0)} / ${(j.ik / j.s).toFixed(0)} | ${(c.sent / c.s).toFixed(0)} / ${(j.sent / j.s).toFixed(0)} |`,
    );
    groups.push({ label: task, values: [c.jerk, j.jerk] });
    ctx.log(
      `H2 ${task}: jerk ${ratio.toFixed(1)}×, path ${(c.path * 1e3).toFixed(1)} → ${(j.path * 1e3).toFixed(1)} mm`,
    );
  }
  const plot = await ctx.plot(
    "h2-jerk.svg",
    barChart(
      "Tool jerk RMS by task",
      groups,
      ["central interpolation", "joint-level interpolation"],
      "m/s³",
    ),
  );
  return [
    {
      id: "H2",
      title: "Joint-level interpolation is smoother",
      target: "Tool jerk ≥ 5× lower at equal or better path error",
      measured: `path error better in every task: ${pathMet ? "yes" : "no"}; jerk ≥ 5× lower in every task: ${jerkMet ? "yes" : "no"}`,
      verdict: jerkMet && pathMet ? "met" : pathMet ? "partly met" : "not met",
      detail: [
        `Mean over ${seeds.length} seed${seeds.length > 1 ? "s" : ""}; KX-6, 15 Hz policy with 30 ms latency, 15 ms jitter, 5 % lost steps; central host loop 250 Hz with 10 % late ticks.`,
        "",
        "| Task | Jerk RMS, central → joint-level | Ratio | Path error RMS |",
        "|---|---|---|---|",
        ...rows,
        "",
        "Joints rate-limit their reference to the host's speed limit, which also smooths the central setpoint stream; both modes run the same node code. Where the jerk target is missed, the arm's own 4 Hz servo bandwidth, not interpolation, limits how smooth the tool can be (NX-10).",
      ].join("\n"),
      plots: [plot],
    },
    {
      id: "H8",
      title: "Work moves off the host",
      target: "Reported (no target)",
      measured: "host computations per second, central / joint-level (below)",
      verdict: "reported",
      detail: [
        "Per second of task, central / joint-level:",
        "",
        "| Task | IK solves | Setpoints or goals sent |",
        "|---|---|---|",
        ...loadRows,
        "",
        "Both modes solve the same IK per policy step; joint-level interpolation sends one goal per joint and step instead of a 250 Hz setpoint stream, and the joints interpolate.",
      ].join("\n"),
    },
  ];
}
