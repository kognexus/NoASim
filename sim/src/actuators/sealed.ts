/**
 * SEALED actuator family for the out-of-family PHNN test (PLAN §2.2, H10a).
 *
 * Do not use this family for training, tuning, choosing model sizes or debugging any
 * learned or fitted model. It is opened only by the final H10 run (NX-19). Its SHA-256
 * is pinned in `sealed.test.ts` and recorded in issue NX-04, so any later change
 * to this file is visible.
 *
 * Structurally different from every open family:
 * - LuGre friction: pre-sliding "bristle" dynamics with its own state, plus a different
 *   Coulomb level in each direction;
 * - gear stiffness that varies with output position (eccentric gear);
 * - torque ripple that grows with the square of the load.
 */
import type { ActuatorSpec } from "../plant/armSpec.ts";
import type { Random } from "../util/random.ts";
import type { ActuatorEffect } from "./effects.ts";

/** LuGre friction with direction-dependent Coulomb level. */
function lugreAsymmetric(
  bristleStiffness: number,
  bristleDamping: number,
  viscous: number,
  coulombPositive: number,
  coulombNegative: number,
  breakawayRatio: number,
  stribeckVelocity: number,
): ActuatorEffect {
  let z = 0;
  return {
    name: "LuGre friction (asymmetric)",
    friction: ({ motorVelocity: v }, dt) => {
      const coulomb = v >= 0 ? coulombPositive : coulombNegative;
      const g = coulomb * (1 + (breakawayRatio - 1) * Math.exp(-((v / stribeckVelocity) ** 2)));
      // dz/dt = v − σ0·|v|·z/g, integrated exactly over one step.
      const zSteady = (g * Math.sign(v)) / bristleStiffness;
      const zNext =
        v === 0
          ? z
          : zSteady + (z - zSteady) * Math.exp((-bristleStiffness * Math.abs(v) * dt) / g);
      const zRate = (zNext - z) / dt;
      z = zNext;
      return -(bristleStiffness * z + bristleDamping * zRate + viscous * v);
    },
  };
}

/** Gear stiffness `k·(1 + e·sin(θ_out + φ))`: deviation from the linear spring. */
function eccentricStiffness(
  stiffness: number,
  eccentricity: number,
  phase: number,
): ActuatorEffect {
  return {
    name: "eccentric gear stiffness",
    gearTorque: ({ motorAngle, gearTwist }) =>
      -stiffness * eccentricity * Math.sin(motorAngle + gearTwist + phase) * gearTwist,
  };
}

/** Ripple at the 3rd electrical harmonic whose amplitude grows with load squared. */
function quadraticRipple(
  electricalPerOutputRad: number,
  perNm: number,
  phase: number,
): ActuatorEffect {
  return {
    name: "load-squared ripple",
    driveTorque: ({ motorAngle }, produced) =>
      perNm *
      produced *
      Math.abs(produced) *
      Math.sin(3 * electricalPerOutputRad * motorAngle + phase),
  };
}

export const SEALED_FAMILY = {
  id: "sealed-lugre-eccentric",
  title: "Sealed: LuGre, eccentric gear, load-squared ripple",
  sealed: true,
  draw(actuator: ActuatorSpec, rng: Random): ActuatorEffect[] {
    const coulomb = actuator.coulombFriction * rng.between(0.7, 1.8);
    const stiffness = actuator.gearStiffness;
    return [
      lugreAsymmetric(
        rng.between(2e3, 6e3) * coulomb,
        rng.between(0.5, 2) * Math.sqrt(coulomb),
        actuator.viscousFriction * rng.between(0.5, 2),
        coulomb,
        coulomb * rng.between(0.6, 0.9),
        rng.between(1.2, 1.8),
        rng.between(0.02, 0.08),
      ),
      eccentricStiffness(stiffness, rng.between(0.05, 0.2), rng.between(0, 2 * Math.PI)),
      quadraticRipple(
        actuator.electrical.polePairs * actuator.gearRatio,
        rng.between(0.02, 0.06) / actuator.ratedTorque,
        rng.between(0, 2 * Math.PI),
      ),
    ];
  },
} as const;
