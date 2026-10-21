/**
 * Actuator grades (NX-26): the same datasheet built to two levels of quality. A grade is
 * drawn like a hidden family, but every joint gets every imperfection, at the grade's
 * level: friction, cogging, torque ripple, phase-current sensing errors, gear play and
 * torque-constant drift.
 *
 * - **Premium**: ground gears without play, skewed magnets (little cogging), calibrated
 *   current sensing.
 * - **Budget**, typical of low-cost quasi-direct-drive modules: strong cogging and
 *   ripple, stickier friction, gear play, and current sensing with offset and gain
 *   mismatch between phases.
 *
 * Levels are relative to the datasheet (rated torque, friction, stiffness); they are our
 * estimates for the two ends of the market, not measurements of any product.
 */
import type { ActuatorSpec } from "../plant/armSpec.ts";
import type { Random } from "../util/random.ts";
import {
  backlash,
  cogging,
  currentRipple,
  currentSensorGain,
  currentSensorOffset,
  stribeck,
  torqueConstantDrift,
  viscousCoulomb,
  type ActuatorEffect,
} from "./effects.ts";
import type { ActuatorFamily } from "./families.ts";

type Range = readonly [number, number];

interface GradeLevels {
  /** Coulomb friction relative to the datasheet. */
  readonly coulomb: Range;
  /** Breakaway relative to Coulomb (1: no Stribeck hump). */
  readonly breakaway: Range;
  /** Cogging at the sixth electrical order, relative to rated torque. */
  readonly cogging: Range;
  /** Current ripple at the sixth electrical order, relative to the torque produced. */
  readonly ripple6: Range;
  /** Current ripple at the twelfth electrical order, relative to the torque produced. */
  readonly ripple12: Range;
  /** Current-sensor offset as torque, relative to rated torque. */
  readonly sensorOffset: Range;
  /** Mean current-sensor gain error (±). */
  readonly sensorGain: number;
  /** Gain mismatch between phases. */
  readonly sensorMismatch: Range;
  /** Total gear play, rad (0: none). */
  readonly backlash: Range;
  /** Torque-constant loss per kelvin. */
  readonly ktDrift: Range;
}

const PREMIUM: GradeLevels = {
  coulomb: [0.5, 1],
  breakaway: [1, 1],
  cogging: [0.001, 0.003],
  ripple6: [0.002, 0.005],
  ripple12: [0, 0],
  sensorOffset: [0.001, 0.003],
  sensorGain: 0.002,
  sensorMismatch: [0.001, 0.003],
  backlash: [0, 0],
  ktDrift: [0.001, 0.0012],
};

const BUDGET: GradeLevels = {
  coulomb: [1, 2.5],
  breakaway: [1.3, 2],
  cogging: [0.02, 0.05],
  ripple6: [0.02, 0.04],
  ripple12: [0.005, 0.015],
  sensorOffset: [0.01, 0.03],
  sensorGain: 0.03,
  sensorMismatch: [0.01, 0.04],
  backlash: [0.002, 0.006],
  ktDrift: [0.001, 0.0014],
};

const phase = (rng: Random): number => rng.between(0, 2 * Math.PI);

function drawGrade(levels: GradeLevels, a: ActuatorSpec, rng: Random): ActuatorEffect[] {
  const electrical = a.electrical.polePairs * a.gearRatio;
  const coulomb = a.coulombFriction * rng.between(...levels.coulomb);
  const effects = [
    levels.breakaway[1] > 1
      ? stribeck(a.viscousFriction, coulomb, coulomb * rng.between(...levels.breakaway), 0.05)
      : viscousCoulomb(a.viscousFriction, coulomb),
    cogging([
      {
        cycles: 6 * electrical,
        amplitude: a.ratedTorque * rng.between(...levels.cogging),
        phase: phase(rng),
      },
    ]),
    currentRipple(electrical, [
      { cycles: 6, amplitude: rng.between(...levels.ripple6), phase: phase(rng) },
      { cycles: 12, amplitude: rng.between(...levels.ripple12), phase: phase(rng) },
    ]),
    currentSensorOffset(
      electrical,
      a.ratedTorque * rng.between(...levels.sensorOffset),
      phase(rng),
    ),
    currentSensorGain(
      electrical,
      rng.between(-levels.sensorGain, levels.sensorGain),
      rng.between(...levels.sensorMismatch),
      phase(rng),
    ),
    torqueConstantDrift(rng.between(...levels.ktDrift)),
  ];
  const play = rng.between(...levels.backlash);
  return play > 0 ? [...effects, backlash(a.gearStiffness, play)] : effects;
}

/** The two grades, drawn only when asked for by id (`DrawOptions.families`). */
export const GRADES: readonly ActuatorFamily[] = [
  {
    id: "grade-premium",
    title: "Premium: little cogging, no gear play, calibrated current sensing",
    sealed: false,
    onRequest: true,
    draw: (a, rng) => drawGrade(PREMIUM, a, rng),
  },
  {
    id: "grade-budget",
    title: "Budget: strong cogging and ripple, gear play, current-sensing offset and mismatch",
    sealed: false,
    onRequest: true,
    draw: (a, rng) => drawGrade(BUDGET, a, rng),
  },
];
