/**
 * H6 sweep: network damping with bad bus conditions. For each latency and frame-loss
 * rate, tap the arm and check that it settles, stays quiet, and that no joint's damping
 * term ever put more energy into the arm than its tank holds.
 *
 *   node experiments/src/robustness.ts kx6l
 *
 * `npm run prove` calls {@link measureRobustness}.
 */
import { DampingMode } from "../../sim/src/bus/protocol.ts";
import { tunedGains } from "../../sim/src/harness/tuned.ts";
import { dampingWork } from "../../sim/src/scenarios/energy.ts";
import { housingShake } from "../../sim/src/scenarios/rest.ts";
import { tapTest } from "../../sim/src/scenarios/tap.ts";
import { ARMS, type Lab, openLab, withArm } from "./runner.ts";

export const LATENCIES = [1, 2, 5, 10, 20];
export const DROP_RATES = [0, 0.1, 0.2];

export interface RobustnessRow {
  readonly latencyCycles: number;
  readonly dropRate: number;
  readonly settlingTime: number;
  /** Tool deviation peak after the tap, m. */
  readonly peak: number;
  /** Worst housing acceleration RMS after the tap record, m/s². */
  readonly shake: number;
  /** Largest cumulative work any joint's damping term did on the arm, J. */
  readonly maxWork: number;
}

/** Taps `arm` (a name in {@link ARMS}) under every bus condition of the sweep. */
export async function measureRobustness(
  lab: Lab,
  arm: string,
  latencies: readonly number[] = LATENCIES,
  dropRates: readonly number[] = DROP_RATES,
): Promise<RobustnessRow[]> {
  const spec = ARMS[arm];
  if (!spec) throw new Error(`unknown arm ${arm}`);
  const rows: RobustnessRow[] = [];
  for (const latencyCycles of latencies) {
    for (const dropRate of dropRates) {
      rows.push(
        await withArm(
          lab,
          { spec, gains: tunedGains(arm, "network"), seed: 21, bus: { latencyCycles, dropRate } },
          (system) => {
            const tap = tapTest(system, {
              recordSeconds: 3,
              beforeTap: (s) => s.host.applySettings({ damping: DampingMode.network }),
            });
            // A second tap, with the damping work integrated from the moment it lands.
            system.plant.setToolForce([0, 0, 200]);
            const during = dampingWork(system, 0.01);
            system.plant.setToolForce([0, 0, 0]);
            const after = dampingWork(system, 2);
            return {
              latencyCycles,
              dropRate,
              settlingTime: tap.settlingTime,
              peak: tap.peak,
              shake: housingShake(system, 1),
              maxWork: Math.max(during.maxWork, after.maxWork + Math.max(...during.finalWork)),
            };
          },
        ),
      );
    }
  }
  return rows;
}

if (import.meta.main) {
  const arm = process.argv[2] ?? "kx6l";
  const rows = await measureRobustness(await openLab(), arm);
  console.log("latency  drop   settle s  peak mm  shake m/s²  max damping work J");
  for (const r of rows) {
    console.log(
      `${String(r.latencyCycles).padStart(7)}  ${r.dropRate.toFixed(2)}  ${r.settlingTime.toFixed(3).padStart(8)}` +
        `  ${(r.peak * 1e3).toFixed(1).padStart(7)}  ${r.shake.toFixed(1).padStart(10)}  ${r.maxWork.toFixed(4)}`,
    );
  }
}
