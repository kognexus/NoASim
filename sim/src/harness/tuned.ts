/**
 * Gains per arm and damping mode, chosen by the fair tuner (`experiments/src/tune.ts`):
 * fastest tap settling with rest jitter ≤ 0.1 mm at the tool and quiet housings, over
 * two seeds and two tap directions. The joint controller itself is the same in every
 * mode; only the damping terms differ. Re-run the tuner and update these values together
 * with docs/tickets/NX-11.md.
 */
import { type ControlGains, DEFAULT_GAINS } from "./gains.ts";

export type TunedMode = "off" | "local" | "network";

const TUNED: Record<string, Record<TunedMode, Partial<ControlGains>>> = {
  "KX-6": {
    off: { localDampingRatio: 0, networkDampingRatio: 0 },
    local: { localDampingRatio: 0.4, imuLowpassHz: 10, networkDampingRatio: 0 },
    network: { networkDampingRatio: 0.5, imuLowpassHz: 15, relayWeight: 0, localDampingRatio: 0 },
  },
  "KX-6L": {
    off: { localDampingRatio: 0, networkDampingRatio: 0 },
    local: { localDampingRatio: 0.4, imuLowpassHz: 10, networkDampingRatio: 0 },
    network: { networkDampingRatio: 0.5, imuLowpassHz: 15, relayWeight: 0, localDampingRatio: 0 },
  },
};

const ALIASES: Record<string, string> = { kx6: "KX-6", kx6l: "KX-6L" };

/** Tuned gains for `arm` (spec name or short alias) in `mode`. */
export function tunedGains(arm: string, mode: TunedMode): ControlGains {
  const table = TUNED[ALIASES[arm] ?? arm];
  if (!table) throw new Error(`no tuned gains for arm ${arm}`);
  return { ...DEFAULT_GAINS, ...table[mode] };
}
