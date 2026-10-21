/**
 * What a joint is doing, from the status bits it publishes on the bus, and how the demo
 * shows it. The 3D discs, the bus view and the legend all take their colours from the
 * same CSS custom properties, which avoid the arms' own colours (blue, orange).
 */
import { StatusFlags } from "../../sim/src/bus/protocol.ts";

export type JointState =
  "idle" | "ready" | "damping" | "commissioning" | "holding" | "fault" | "torqueOff";

/** The most important thing the status says, safety first. */
export function jointState(status: number | undefined): JointState {
  if (status === undefined) return "idle";
  const has = (bit: number) => (status & bit) !== 0;
  if (has(StatusFlags.torqueOff)) return "torqueOff";
  if (has(StatusFlags.fault)) return "fault";
  if (has(StatusFlags.holding) || has(StatusFlags.stopping)) return "holding";
  if (has(StatusFlags.commissioning)) return "commissioning";
  // The bit is named for network damping but is set whenever a damping term acts.
  if (has(StatusFlags.networkDamping)) return "damping";
  if (has(StatusFlags.ready)) return "ready";
  return "idle";
}

/** CSS custom property holding each state's colour. */
export const STATUS_VAR: Record<JointState, string> = {
  idle: "--status-idle",
  ready: "--status-ready",
  damping: "--status-damping",
  commissioning: "--status-commissioning",
  holding: "--status-holding",
  fault: "--status-fault",
  torqueOff: "--status-off",
};

/** Legend text, in the order shown. */
export const STATUS_LABEL: [JointState, string][] = [
  ["ready", "ready (position control)"],
  ["damping", "damping active"],
  ["holding", "holding after an impact"],
  ["fault", "fault"],
  ["torqueOff", "torque off"],
  ["commissioning", "commissioning"],
];
