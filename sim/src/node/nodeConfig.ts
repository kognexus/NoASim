/**
 * What each node is told at power-on: its EEPROM contents (nominal datasheet values and
 * control gains). `sim/src/harness/gains.ts` fills it from the `ArmSpec`, never from the
 * hidden actuator physics.
 *
 * Field order must match `NodeConfig` in `crates/nexus-node/src/config.rs`; the node
 * echoes its config in its discovery announcement, which a test checks.
 */

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
  viscousFriction: number;
  coulombFriction: number;
  stiffness: number;
  damping: number;
  integralGain: number;
  rotorDamping: number;
  observerHz: number;
  localDamping: number;
  networkDamping: number;
  dampingHighpassHz: number;
  dampingLowpassHz: number;
  imuLowpassHz: number;
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
    c.viscousFriction,
    c.coulombFriction,
    c.stiffness,
    c.damping,
    c.integralGain,
    c.rotorDamping,
    c.observerHz,
    c.localDamping,
    c.networkDamping,
    c.dampingHighpassHz,
    c.dampingLowpassHz,
    c.imuLowpassHz,
    c.modelVersion,
  ];
}
