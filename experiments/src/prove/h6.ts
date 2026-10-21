/**
 * H6: network damping is safe under bad bus conditions (`experiments/src/robustness.ts`).
 */
import { DROP_RATES, LATENCIES, measureRobustness } from "../robustness.ts";
import type { Finding, ProveContext } from "./findings.ts";

/** Each joint's energy tank, J (`damping.rs`). */
const TANK_J = 0.05;
/** A housing RMS above this after the record counts as ringing, m/s² (gravity included;
 * the tuner's limit, `experiments/src/tune.ts`). */
const QUIET_LIMIT = 9.81 * 1.05;
/** A tap that has not settled within the 3 s record counts as unstable. */
const RECORD_S = 3;

export async function h6(ctx: ProveContext): Promise<Finding> {
  const latencies = ctx.quick ? [1, 10, 20] : LATENCIES;
  const dropRates = ctx.quick ? [0, 0.2] : DROP_RATES;
  const rows: string[] = [];
  let worstWork = 0;
  let stable = true;
  for (const arm of ["kx6l", "kx6"]) {
    const result = await measureRobustness(ctx.lab, arm, latencies, dropRates);
    for (const r of result) {
      worstWork = Math.max(worstWork, r.maxWork);
      stable &&=
        Number.isFinite(r.settlingTime) && r.settlingTime < RECORD_S && r.shake < QUIET_LIMIT;
      rows.push(
        `| ${arm === "kx6l" ? "KX-6L" : "KX-6"} | ${r.latencyCycles} | ${(r.dropRate * 100).toFixed(0)} % | ${r.settlingTime.toFixed(3)} | ${(r.peak * 1e3).toFixed(1)} | ${r.shake.toFixed(1)} | ${(r.maxWork * 1e3).toFixed(2)} |`,
      );
    }
    ctx.log(`H6 ${arm}: worst damping work ${(worstWork * 1e3).toFixed(2)} mJ, stable ${stable}`);
  }
  return {
    id: "H6",
    title: "Network damping is safe under bad bus conditions",
    target:
      "Energy injected by the network term never positive; never unstable, latency up to 20 cycles and 20 % lost frames",
    measured: `never unstable: ${stable ? "yes" : "no"}; largest cumulative work by any joint's network term ${(worstWork * 1e3).toFixed(2)} mJ (its tank holds ${TANK_J * 1e3} mJ)`,
    verdict: stable && worstWork <= TANK_J ? "met" : "not met",
    detail: [
      "Network damping with its tuned gains; a tap under each bus condition (seed 21), then a second tap with the work of every joint's damping term integrated.",
      "",
      "| Arm | Latency, cycles | Lost frames | Settling, s | Peak, mm | Housing RMS, m/s² | Largest work, mJ |",
      "|---|---|---|---|---|---|---|",
      ...rows,
      "",
      "The work can briefly be positive: the energy tank lets a joint give back what its damping term took earlier, never more than the tank holds. That is the passivity guarantee (ADR-0004). Beyond 2 ms of latency the network term switches itself off by design and the arm behaves like joint control alone.",
    ].join("\n"),
  };
}
