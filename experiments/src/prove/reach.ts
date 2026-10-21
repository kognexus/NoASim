/**
 * Modes no joint can reach (NX-28, `sim/src/analysis/reach.ts`). Reported.
 */
import { modeReach } from "../../../sim/src/analysis/reach.ts";
import { nodeConfigsFor } from "../../../sim/src/harness/gains.ts";
import { tunedGains } from "../../../sim/src/harness/tuned.ts";
import { BUS_CYCLE_HZ } from "../../../sim/src/index.ts";
import { KX6, KX6_HOME, KX6L, withInitialAngles } from "../../../sim/src/plant/kx6.ts";
import { withArm } from "../runner.ts";
import type { Finding, ProveContext } from "./findings.ts";

const POSTURES: Record<string, readonly number[]> = {
  home: KX6_HOME,
  reach: [0, 0.527, 0.65, 1.2, 0, 0],
};
/** A joint that adds less than this damping ratio to a mode does not reach it. */
const REACHED = 0.01;

const pct = (x: number) => `${(100 * x).toFixed(1)} %`;
const strongest = (values: readonly number[]) => {
  let best = 0;
  values.forEach((v, i) => {
    if (Math.abs(v) > Math.abs(values[best] ?? 0)) best = i;
  });
  return { joint: best + 1, value: values[best] ?? 0 };
};

export async function reach(ctx: ProveContext): Promise<Finding> {
  const rows: string[] = [];
  const unreached: string[] = [];
  for (const [arm, base, key] of [
    ["KX-6L", KX6L, "kx6l"],
    ["KX-6", KX6, "kx6"],
  ] as const) {
    for (const [posture, angles] of Object.entries(POSTURES)) {
      const spec = withInitialAngles(base, angles);
      const gains = tunedGains(key, "network");
      const modes = await withArm(ctx.lab, { spec, gains, seed: 31 }, (system) => {
        system.run(3);
        return modeReach(system, nodeConfigsFor(spec, BUS_CYCLE_HZ, gains));
      });
      for (const m of modes) {
        const local = strongest(m.local);
        const network = strongest(m.network);
        const reached = Math.abs(local.value) >= REACHED || Math.abs(network.value) >= REACHED;
        if (!reached) unreached.push(`${arm} ${posture} ${m.hz.toFixed(0)} Hz`);
        rows.push(
          `| ${arm} | ${posture} | ${m.hz.toFixed(1)} | ${pct(m.zeta)} | ${pct(m.shares.bend)} | J${local.joint} ${pct(local.value)} | J${network.joint} ${pct(network.value)} | ${reached ? "" : "no joint"} |`,
        );
      }
      ctx.log(`reach ${arm} ${posture}: ${modes.length} modes below 80 Hz`);
    }
  }
  return {
    id: "Reach",
    title: "Modes no joint can reach",
    target: "Reported (NX-28): for each mode below 80 Hz, how much damping any one joint can add",
    measured: unreached.length
      ? `reached by no joint: ${unreached.join(", ")}`
      : "every mode below 80 Hz is reached by some joint",
    verdict: "reported",
    detail: [
      "The arm linearised where it holds (MuJoCo's accelerations, the joints' position loops closed), its modes from the full state matrix. For each mode, the damping ratio one joint adds on its own at its network-damping gain, to first order: with its own output rate (collocated), and with its network signal (outer gyro minus its own, on its axis). Filters and bus delay are left out: this is what the sensors can see at all. A negative value: that joint alone would feed the mode, not damp it; the network signal's 15 Hz low-pass and the energy tank are what keep that in check (ADR-0004).",
      "",
      "| Arm | Posture | Mode, Hz | Damping held | Link bending | Largest one-joint effect, collocated | Largest one-joint effect, network | |",
      "|---|---|---|---|---|---|---|---|",
      ...rows,
      "",
      "Below 15 Hz the modes are the joints' own position loops, already 24–56 % damped, and the network signal reaches them about as well as each joint's own rate: one reason network and joint-local damping settle a tap about equally (H1). On the light arm, link bending at 55–69 Hz is reached by no joint (it bends across every joint axis); the structure's own damping (13–16 %) is all it has.",
    ].join("\n"),
  };
}
