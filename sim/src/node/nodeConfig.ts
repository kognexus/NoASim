/**
 * What each node is told at power-on: its EEPROM contents (nominal datasheet values and
 * default gains). Built from the `ArmSpec`, never from the hidden actuator physics.
 *
 * Field order must match `NodeConfig` in `crates/nexus-node/src/config.rs`; the node
 * echoes its config in its discovery announcement, which a test checks.
 */
import type { JointSpec } from "../plant/armSpec.ts";
import { outputTorqueConstant, reflectedInertia } from "../plant/armSpec.ts";

export interface NodeConfig {
  cycleHz: number;
  gearRatio: number;
  ratedTorque: number;
  peakTorque: number;
  reflectedInertia: number;
  gearStiffness: number;
  outputTorqueConstant: number;
  maxTemperature: number;
  axis: readonly [number, number, number];
  link: readonly [number, number, number];
  linkMass: number;
  hasBrake: boolean;
  stiffness: number;
  damping: number;
  observerHz: number;
  modelVersion: number;
}

/** Flattens a config into the node's `f32` layout. */
export function packNodeConfig(c: NodeConfig): number[] {
  return [
    c.cycleHz,
    c.gearRatio,
    c.ratedTorque,
    c.peakTorque,
    c.reflectedInertia,
    c.gearStiffness,
    c.outputTorqueConstant,
    c.maxTemperature,
    ...c.axis,
    ...c.link,
    c.linkMass,
    c.hasBrake ? 1 : 0,
    c.stiffness,
    c.damping,
    c.observerHz,
    c.modelVersion,
  ];
}

/** Default position-loop gains scale with the joint's size (rated torque). */
const STIFFNESS_PER_RATED_NM = 100;
const DAMPING_PER_RATED_NM = 2;

/** The EEPROM contents of the node driving `joint`. */
export function nodeConfigFor(joint: JointSpec, cycleHz: number): NodeConfig {
  const a = joint.actuator;
  return {
    cycleHz,
    gearRatio: a.gearRatio,
    ratedTorque: a.ratedTorque,
    peakTorque: a.peakTorque,
    reflectedInertia: reflectedInertia(a),
    gearStiffness: a.gearStiffness,
    outputTorqueConstant: outputTorqueConstant(a),
    maxTemperature: a.electrical.thermal.maxWindingTemperature,
    axis: joint.axis,
    link: joint.link.vector,
    linkMass: joint.link.mass,
    hasBrake: a.brake,
    stiffness: STIFFNESS_PER_RATED_NM * a.ratedTorque,
    damping: DAMPING_PER_RATED_NM * a.ratedTorque,
    observerHz: 150,
    modelVersion: 1,
  };
}
