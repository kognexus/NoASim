/**
 * Sizing of each node's EEPROM contents from the datasheet, as an integrator would do it.
 */
import { type NodeConfig } from "../node/nodeConfig.ts";
import type { ArmSpec, JointSpec } from "../plant/armSpec.ts";
import { outputTorqueConstant, reflectedInertia } from "../plant/armSpec.ts";
import { jointLoadInertia } from "../plant/kinematics.ts";

/**
 * Control gains for every joint, given as dynamics rather than raw numbers: each joint's
 * stiffness and damping follow from its load inertia (stretched-out arm, from the
 * datasheet), as an integrator would size them.
 */
export interface ControlGains {
  /** Natural frequency of each joint's position loop on its load, Hz. */
  readonly naturalHz: number;
  /**
   * Minimum joint stiffness per rated N·m, 1/rad. Light joints (the wrist) would get almost
   * no stiffness from `naturalHz` alone; this keeps their deflection at rated torque
   * below 1/value rad.
   */
  readonly minStiffnessPerNm: number;
  /** Damping ratio of the position loop. */
  readonly dampingRatio: number;
  /** Integral action, 1/s. */
  readonly integralGain: number;
  /** Rotor-against-gear damping per rated N·m, s/rad. */
  readonly rotorDampingPerNm: number;
  /** Encoder velocity observer bandwidth, Hz. */
  readonly observerHz: number;
  /** Damping ratio added by joint-local damping (scales its gain like `dampingRatio`). */
  readonly localDampingRatio: number;
  /** Damping ratio added by network damping. */
  readonly networkDampingRatio: number;
  /** High-pass corner of the damping signals, Hz. */
  readonly dampingHighpassHz: number;
  /** Low-pass corner of the joint damping signal, Hz (second order; ≥ 1 kHz = off). */
  readonly dampingLowpassHz: number;
  /** Low-pass corner of the IMU damping signals, Hz (second order). */
  readonly imuLowpassHz: number;
}

/** Defaults, tuned in NX-07 and NX-08 (see ADR-0003). */
export const DEFAULT_GAINS: ControlGains = {
  naturalHz: 4,
  minStiffnessPerNm: 50,
  dampingRatio: 0.3,
  integralGain: 0,
  rotorDampingPerNm: 0.1,
  observerHz: 150,
  localDampingRatio: 0.3,
  networkDampingRatio: 1,
  dampingHighpassHz: 1,
  dampingLowpassHz: 1000,
  imuLowpassHz: 1000,
};

/** The EEPROM contents of the node driving `joint`, whose load inertia is `loadInertia`. */
export function nodeConfigFor(
  joint: JointSpec,
  cycleHz: number,
  loadInertia: number,
  gains: ControlGains = DEFAULT_GAINS,
): NodeConfig {
  const a = joint.actuator;
  const stiffness = Math.max(
    (2 * Math.PI * gains.naturalHz) ** 2 * loadInertia,
    gains.minStiffnessPerNm * a.ratedTorque,
  );
  // Damping ratios refer to the resulting natural frequency √(K/I).
  const dampingPerRatio = 2 * Math.sqrt(stiffness * loadInertia);
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
    viscousFriction: a.viscousFriction,
    coulombFriction: a.coulombFriction,
    stiffness,
    damping: gains.dampingRatio * dampingPerRatio,
    integralGain: gains.integralGain,
    rotorDamping: gains.rotorDampingPerNm * a.ratedTorque,
    observerHz: gains.observerHz,
    localDamping: gains.localDampingRatio * dampingPerRatio,
    networkDamping: gains.networkDampingRatio * dampingPerRatio,
    dampingHighpassHz: gains.dampingHighpassHz,
    dampingLowpassHz: gains.dampingLowpassHz,
    imuLowpassHz: gains.imuLowpassHz,
    modelVersion: 1,
  };
}

/** EEPROM contents for every joint of `spec`. */
export function nodeConfigsFor(
  spec: ArmSpec,
  cycleHz: number,
  gains: ControlGains = DEFAULT_GAINS,
): NodeConfig[] {
  const inertia = jointLoadInertia(spec);
  return spec.joints.map((joint, i) => nodeConfigFor(joint, cycleHz, inertia[i] ?? 0, gains));
}
