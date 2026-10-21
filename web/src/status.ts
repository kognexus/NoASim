/**
 * What a joint is doing, from the status bits it publishes on the bus. The 3D view and the
 * bus view colour joints by it.
 */
import { StatusFlags } from "../../sim/src/bus/protocol.ts";

export type JointState =
  "idle" | "ready" | "network" | "commissioning" | "holding" | "fault" | "torqueOff";

/** The most important thing the status says, safety first. */
export function jointState(status: number | undefined): JointState {
  if (status === undefined) return "idle";
  const has = (bit: number) => (status & bit) !== 0;
  if (has(StatusFlags.torqueOff)) return "torqueOff";
  if (has(StatusFlags.fault)) return "fault";
  if (has(StatusFlags.holding) || has(StatusFlags.stopping)) return "holding";
  if (has(StatusFlags.commissioning)) return "commissioning";
  if (has(StatusFlags.networkDamping)) return "network";
  if (has(StatusFlags.ready)) return "ready";
  return "idle";
}
