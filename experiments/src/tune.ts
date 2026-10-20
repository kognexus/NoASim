/**
 * Fair per-mode tuning for H1 (ADR-0003): every mode gets the gains that give it the
 * fastest settling, subject to the same budget at rest — tool jitter below
 * `JITTER_BUDGET` and housings quiet (no rotor–gear ringing) — over several seeds and
 * both tap directions.
 *
 *   node experiments/src/tune.ts kx6l
 *
 * Writes experiments/results/tuning-<arm>.json and prints a table.
 */
import { mkdir, writeFile } from "node:fs/promises";
import { type ControlGains, DEFAULT_GAINS, type Mode, openLab, tap } from "./runner.ts";

/** Largest rest jitter at the tool any mode may have, m. */
export const JITTER_BUDGET = 0.1e-3;
/** Housings count as quiet up to this RMS acceleration (1 g plus 5 %), m/s². */
export const QUIET_LIMIT = 9.81 * 1.05;
const SEEDS = [11, 12];
const DIRECTIONS = ["z", "y"] as const;

type Candidate = Partial<ControlGains>;

/** The search space per mode. The joint controller is the same in every mode. */
const CANDIDATES: Record<Mode, Candidate[]> = {
  off: [{}],
  local: [0.1, 0.2, 0.4, 0.7].flatMap((localDampingRatio) =>
    [10, 15, 25].map((imuLowpassHz) => ({
      localDampingRatio,
      imuLowpassHz,
      networkDampingRatio: 0,
    })),
  ),
  network: [0.3, 0.5, 0.7, 1.0, 1.4].flatMap((networkDampingRatio) =>
    [10, 15].flatMap((imuLowpassHz) =>
      [0, 0.5].map((relayWeight) => ({
        networkDampingRatio,
        imuLowpassHz,
        relayWeight,
        localDampingRatio: 0,
      })),
    ),
  ),
};

interface Scored {
  readonly candidate: Candidate;
  readonly settling: number;
  readonly worstJitter: number;
  readonly worstShake: number;
  readonly peak: number;
  readonly admissible: boolean;
}

async function main(): Promise<void> {
  const arm = process.argv[2] ?? "kx6l";
  const modes = (process.argv[3]?.split(",") ?? ["off", "local", "network"]) as Mode[];
  const lab = await openLab();
  const results: Record<string, Scored> = {};
  for (const mode of modes) {
    let best: Scored | undefined;
    for (const candidate of CANDIDATES[mode]) {
      const gains = { ...DEFAULT_GAINS, ...candidate };
      const runs = [];
      for (const seed of SEEDS)
        for (const direction of DIRECTIONS)
          runs.push(await tap(lab, arm, mode, gains, seed, direction));
      const settling = runs.reduce((s, r) => s + r.settlingTime, 0) / runs.length;
      const worstJitter = Math.max(...runs.map((r) => r.jitter));
      const worstShake = Math.max(...runs.map((r) => r.shake));
      const peak = runs.reduce((s, r) => s + r.peak, 0) / runs.length;
      const admissible =
        worstJitter <= JITTER_BUDGET && worstShake <= QUIET_LIMIT && Number.isFinite(settling);
      const scored = { candidate, settling, worstJitter, worstShake, peak, admissible };
      console.log(
        `${arm} ${mode.padEnd(7)} ${JSON.stringify(candidate).padEnd(90)} settle ${settling.toFixed(3)} s` +
          ` peak ${(peak * 1e3).toFixed(1)} mm jitter ${(worstJitter * 1e3).toFixed(3)} mm shake ${worstShake.toFixed(1)}` +
          (admissible ? "" : "  (outside budget)"),
      );
      if (admissible && (!best || settling < best.settling)) best = scored;
    }
    if (best) results[mode] = best;
  }
  await mkdir(new URL("../results/", import.meta.url), { recursive: true });
  await writeFile(
    new URL(`../results/tuning-${arm}.json`, import.meta.url),
    JSON.stringify(
      { arm, jitterBudget: JITTER_BUDGET, quietLimit: QUIET_LIMIT, seeds: SEEDS, results },
      null,
      2,
    ) + "\n",
  );
  console.log("\nBest admissible per mode:");
  for (const [mode, r] of Object.entries(results)) {
    console.log(
      `  ${mode.padEnd(8)} settle ${r.settling.toFixed(3)} s  ${JSON.stringify(r.candidate)}`,
    );
  }
}

await main();
