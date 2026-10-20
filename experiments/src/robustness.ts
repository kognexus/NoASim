/**
 * H6 sweep: network damping with bad bus conditions. For each latency and frame-loss
 * rate, tap the arm and check that it settles, stays quiet, and that no joint's damping
 * term ever put more energy into the arm than its tank holds.
 *
 *   node experiments/src/robustness.ts kx6l
 */
import { DampingMode } from "../../sim/src/bus/protocol.ts";
import { tunedGains } from "../../sim/src/harness/tuned.ts";
import { dampingWork } from "../../sim/src/scenarios/energy.ts";
import { housingShake } from "../../sim/src/scenarios/rest.ts";
import { tapTest } from "../../sim/src/scenarios/tap.ts";
import { ARMS, openLab, withArm } from "./runner.ts";

const LATENCIES = [1, 2, 5, 10, 20];
const DROP_RATES = [0, 0.1, 0.2];

const arm = process.argv[2] ?? "kx6l";
const spec = ARMS[arm];
if (!spec) throw new Error(`unknown arm ${arm}`);
const lab = await openLab();
console.log("latency  drop   settle s  peak mm  shake m/s²  max damping work J");
for (const latencyCycles of LATENCIES) {
  for (const dropRate of DROP_RATES) {
    const row = await withArm(
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
        const energy = {
          maxWork: Math.max(during.maxWork, after.maxWork + Math.max(...during.finalWork)),
        };
        return { tap, shake: housingShake(system, 1), energy };
      },
    );
    console.log(
      `${String(latencyCycles).padStart(7)}  ${dropRate.toFixed(2)}  ${row.tap.settlingTime.toFixed(3).padStart(8)}` +
        `  ${(row.tap.peak * 1e3).toFixed(1).padStart(7)}  ${row.shake.toFixed(1).padStart(10)}  ${row.energy.maxWork.toFixed(4)}`,
    );
  }
}
