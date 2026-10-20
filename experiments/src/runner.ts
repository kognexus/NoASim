/**
 * Shared set-up for experiments: loads MuJoCo and the node module once, builds arms by
 * name, and runs the tap test in a given damping mode.
 */
import { readFile } from "node:fs/promises";
import { DampingMode } from "../../sim/src/bus/protocol.ts";
import { ArmSystem, type ArmSystemOptions } from "../../sim/src/harness/armSystem.ts";
import { type ControlGains, DEFAULT_GAINS } from "../../sim/src/harness/gains.ts";
import type { ArmSpec } from "../../sim/src/plant/armSpec.ts";
import { KX6, KX6L } from "../../sim/src/plant/kx6.ts";
import { loadMujoco, type Mujoco } from "../../sim/src/plant/mujoco.ts";
import { housingShake } from "../../sim/src/scenarios/rest.ts";
import { tapTest } from "../../sim/src/scenarios/tap.ts";

export const ARMS: Record<string, ArmSpec> = { kx6: KX6, kx6l: KX6L };
export type Mode = "off" | "local" | "network";
export const MODE_SETTING: Record<Mode, DampingMode> = {
  off: DampingMode.off,
  local: DampingMode.local,
  network: DampingMode.network,
};

const WASM = new URL(
  "../../target/wasm32-unknown-unknown/release/nexus_wasm.wasm",
  import.meta.url,
);

export interface Lab {
  readonly mujoco: Mujoco;
  readonly nodeModule: WebAssembly.Module;
}

export async function openLab(): Promise<Lab> {
  const [mujoco, nodeModule] = await Promise.all([
    loadMujoco(),
    readFile(WASM).then((bytes) => WebAssembly.compile(bytes)),
  ]);
  return { mujoco, nodeModule };
}

export async function withArm<T>(
  lab: Lab,
  options: ArmSystemOptions,
  run: (system: ArmSystem) => T,
): Promise<T> {
  const system = await ArmSystem.create(lab.mujoco, lab.nodeModule, options);
  try {
    return run(system);
  } finally {
    system.dispose();
  }
}

export interface TapOutcome {
  readonly settlingTime: number;
  readonly peak: number;
  /** Rest jitter at the tool before the tap, m. */
  readonly jitter: number;
  /** Worst housing acceleration RMS after the tap record, m/s². */
  readonly shake: number;
}

/** One tap test of `arm` in `mode` with `gains`; `direction` is z (vertical) or y. */
export function tap(
  lab: Lab,
  arm: string,
  mode: Mode,
  gains: ControlGains,
  seed: number,
  direction: "z" | "y" = "z",
): Promise<TapOutcome> {
  const spec = ARMS[arm];
  if (!spec) throw new Error(`unknown arm ${arm}`);
  return withArm(lab, { spec, gains, seed }, (system) => {
    const result = tapTest(system, {
      direction: direction === "z" ? [0, 0, 1] : [0, 1, 0],
      recordSeconds: 4,
      beforeTap: (s) => s.host.applySettings({ damping: MODE_SETTING[mode] }),
    });
    return {
      settlingTime: result.settlingTime,
      peak: result.peak,
      jitter: result.noise,
      shake: housingShake(system, 1),
    };
  });
}

export { DEFAULT_GAINS, type ControlGains };
