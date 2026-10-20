/**
 * Hidden actuator families: structurally different actuators the node is not told about
 * (PLAN §2.2). Every joint gets a random family and randomly drawn parameters around its
 * datasheet values. The draw is ground truth for the harness and the scorer only.
 */
import type { ActuatorSpec, ArmSpec } from "../plant/armSpec.ts";
import type { Random } from "../util/random.ts";
import {
  backlash,
  cogging,
  cubicStiffening,
  currentRipple,
  dahlHysteresis,
  type Harmonic,
  positionDependentFriction,
  stribeck,
  torqueConstantDrift,
  viscousCoulomb,
  type ActuatorEffect,
} from "./effects.ts";
import { HiddenActuator } from "./hiddenActuator.ts";
import { SEALED_FAMILY } from "./sealed.ts";

export interface ActuatorFamily {
  readonly id: string;
  readonly title: string;
  /** Sealed families are excluded unless explicitly requested (out-of-family test). */
  readonly sealed: boolean;
  draw(actuator: ActuatorSpec, rng: Random): ActuatorEffect[];
}

const randomPhase = (rng: Random): number => rng.between(0, 2 * Math.PI);
/** Electrical radians per output radian. */
const electricalPerOutput = (a: ActuatorSpec): number => a.electrical.polePairs * a.gearRatio;
/** Gear twist at rated torque, rad. */
const ratedTwist = (a: ActuatorSpec): number => a.ratedTorque / a.gearStiffness;

function frictionLevels(a: ActuatorSpec, rng: Random): { viscous: number; coulomb: number } {
  return {
    viscous: a.viscousFriction * rng.between(0.5, 2),
    coulomb: a.coulombFriction * rng.between(0.5, 2),
  };
}

function coggingTerms(a: ActuatorSpec, rng: Random, relative: [number, number]): Harmonic[] {
  // Dominant cogging order: 6 per electrical period.
  return [
    {
      cycles: 6 * electricalPerOutput(a),
      amplitude: a.ratedTorque * rng.between(...relative),
      phase: randomPhase(rng),
    },
  ];
}

export const OPEN_FAMILIES: readonly ActuatorFamily[] = [
  {
    id: "textbook",
    title: "Textbook: viscous + Coulomb friction, mild cogging and 6th-order ripple",
    sealed: false,
    draw(a, rng) {
      const { viscous, coulomb } = frictionLevels(a, rng);
      return [
        viscousCoulomb(viscous, coulomb),
        cogging(coggingTerms(a, rng, [0.003, 0.01])),
        currentRipple(electricalPerOutput(a), [
          { cycles: 6, amplitude: rng.between(0.005, 0.015), phase: randomPhase(rng) },
        ]),
      ];
    },
  },
  {
    id: "stribeck-position",
    title: "Stribeck friction plus friction that varies with position",
    sealed: false,
    draw(a, rng) {
      const { viscous, coulomb } = frictionLevels(a, rng);
      return [
        stribeck(viscous, coulomb, coulomb * rng.between(1.3, 2), rng.between(0.02, 0.1)),
        positionDependentFriction(coulomb * 0.3, [
          { cycles: 1, amplitude: rng.between(0.3, 0.8), phase: randomPhase(rng) },
          { cycles: a.gearRatio, amplitude: rng.between(0.2, 0.6), phase: randomPhase(rng) },
        ]),
      ];
    },
  },
  {
    id: "ripple-thermal",
    title: "Strong cogging, 6th and 12th-order ripple, torque constant falling with heat",
    sealed: false,
    draw(a, rng) {
      const { viscous, coulomb } = frictionLevels(a, rng);
      return [
        viscousCoulomb(viscous, coulomb),
        cogging(coggingTerms(a, rng, [0.01, 0.03])),
        currentRipple(electricalPerOutput(a), [
          { cycles: 6, amplitude: rng.between(0.01, 0.04), phase: randomPhase(rng) },
          { cycles: 12, amplitude: rng.between(0.003, 0.015), phase: randomPhase(rng) },
        ]),
        torqueConstantDrift(rng.between(0.0008, 0.0014)),
      ];
    },
  },
  {
    id: "hysteretic-stiffening",
    title: "Gear that stiffens with load and shows hysteresis",
    sealed: false,
    draw(a, rng) {
      const { viscous, coulomb } = frictionLevels(a, rng);
      return [
        viscousCoulomb(viscous, coulomb),
        cubicStiffening((rng.between(0.2, 0.6) * a.gearStiffness) / ratedTwist(a) ** 2),
        dahlHysteresis(
          a.gearStiffness * rng.between(0.1, 0.3),
          a.ratedTorque * rng.between(0.01, 0.04),
        ),
      ];
    },
  },
  {
    id: "backlash",
    title: "Gear backlash with mild stiffening",
    sealed: false,
    draw(a, rng) {
      const { viscous, coulomb } = frictionLevels(a, rng);
      return [
        viscousCoulomb(viscous, coulomb),
        backlash(a.gearStiffness, rng.between(0.0015, 0.005)),
        cubicStiffening((rng.between(0.05, 0.2) * a.gearStiffness) / ratedTwist(a) ** 2),
      ];
    },
  },
];

export const ALL_FAMILIES: readonly ActuatorFamily[] = [...OPEN_FAMILIES, SEALED_FAMILY];

export interface DrawOptions {
  /** Restrict the draw to these family ids. Defaults to all open families. */
  readonly families?: readonly string[];
  /** Allow the sealed family. Only the final H10 run sets this. */
  readonly includeSealed?: boolean;
  /** Ambient temperature, °C. */
  readonly ambient?: number;
}

/** Draws a hidden actuator for every joint of `arm`. Reproducible from `rng`'s seed. */
export function drawHiddenActuators(
  arm: ArmSpec,
  rng: Random,
  options: DrawOptions = {},
): HiddenActuator[] {
  const allowed = ALL_FAMILIES.filter(
    (f) =>
      (options.includeSealed === true || !f.sealed) && (options.families?.includes(f.id) ?? true),
  );
  if (allowed.length === 0) throw new Error("no actuator family matches the draw options");
  return arm.joints.map((joint) => {
    const jointRng = rng.derive(`actuator/${joint.name}`);
    const family = allowed[Math.floor(jointRng.uniform() * allowed.length)];
    if (!family) throw new Error("family index out of range");
    return new HiddenActuator(
      joint.actuator,
      family.id,
      family.draw(joint.actuator, jointRng),
      options.ambient,
    );
  });
}
