/**
 * Loads the MuJoCo WebAssembly module once per process (Node) or page (browser).
 */
import loadMujocoModule, { type MainModule } from "@mujoco/mujoco";

export type Mujoco = MainModule;

let loading: Promise<Mujoco> | undefined;

/** Returns the shared MuJoCo module, loading it on first use. */
export function loadMujoco(): Promise<Mujoco> {
  loading ??= loadMujocoModule();
  return loading;
}
