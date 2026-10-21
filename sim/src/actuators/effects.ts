/**
 * Building blocks of actuator behaviour the node is not told about.
 *
 * Each effect adds one physical phenomenon. A hidden actuator family (see `families.ts`)
 * is a set of effects with randomly drawn parameters. All torques are output-referred,
 * N·m; angles are output-referred, rad.
 */
import type { ActuatorState } from "../plant/drive.ts";

/** One physical phenomenon of an actuator. Hooks an effect does not use are omitted. */
export interface ActuatorEffect {
  /** Short label for logs and reports. */
  readonly name: string;
  /** Friction torque on the motor hinge (it opposes motion). */
  friction?(state: ActuatorState, dt: number): number;
  /** Torque added to the motor's electromagnetic torque (cogging, ripple). */
  driveTorque?(state: ActuatorState, producedTorque: number): number;
  /** Gear torque beyond the model's linear spring. */
  gearTorque?(state: ActuatorState, dt: number): number;
  /** Factor on the torque constant (magnets weaken when hot). */
  torqueConstantScale?(windingTemperature: number): number;
}

/** Velocity scale of the smooth sign used for Coulomb friction, rad/s. */
const STICTION_VELOCITY = 0.01;
const smoothSign = (velocity: number): number => Math.tanh(velocity / STICTION_VELOCITY);

/** A harmonic term `amplitude · sin(cycles · angle + phase)`. */
export interface Harmonic {
  readonly cycles: number;
  readonly amplitude: number;
  readonly phase: number;
}

const harmonicSum = (terms: readonly Harmonic[], angle: number): number =>
  terms.reduce((sum, t) => sum + t.amplitude * Math.sin(t.cycles * angle + t.phase), 0);

/** Viscous plus Coulomb friction: the textbook model. */
export function viscousCoulomb(viscous: number, coulomb: number): ActuatorEffect {
  return {
    name: "viscous+Coulomb friction",
    friction: ({ motorVelocity: v }) => -(viscous * v + coulomb * smoothSign(v)),
  };
}

/** Stribeck friction: breakaway above Coulomb level, falling off with speed. */
export function stribeck(
  viscous: number,
  coulomb: number,
  breakaway: number,
  stribeckVelocity: number,
): ActuatorEffect {
  return {
    name: "Stribeck friction",
    friction: ({ motorVelocity: v }) =>
      -(
        viscous * v +
        (coulomb + (breakaway - coulomb) * Math.exp(-((v / stribeckVelocity) ** 2))) * smoothSign(v)
      ),
  };
}

/**
 * Coulomb friction that varies with position (bearing preload, seal contact, gear
 * runout): `coulomb · (1 + Σ harmonics(motorAngle))`.
 */
export function positionDependentFriction(
  coulomb: number,
  terms: readonly Harmonic[],
): ActuatorEffect {
  return {
    name: "position-dependent friction",
    friction: ({ motorAngle, motorVelocity }) =>
      -coulomb * (1 + harmonicSum(terms, motorAngle)) * smoothSign(motorVelocity),
  };
}

/** Cogging: torque that depends on rotor position only. */
export function cogging(terms: readonly Harmonic[]): ActuatorEffect {
  return { name: "cogging", driveTorque: ({ motorAngle }) => harmonicSum(terms, motorAngle) };
}

/** Current-dependent torque ripple: `produced · Σ harmonics(electrical angle)`. */
export function currentRipple(
  electricalPerOutputRad: number,
  terms: readonly Harmonic[],
): ActuatorEffect {
  return {
    name: "current ripple",
    driveTorque: ({ motorAngle }, produced) =>
      produced * harmonicSum(terms, motorAngle * electricalPerOutputRad),
  };
}

/** Torque constant falling with magnet temperature (NdFeB ≈ −0.12 %/K). */
export function torqueConstantDrift(perKelvin: number, reference = 20): ActuatorEffect {
  return {
    name: "torque-constant drift",
    torqueConstantScale: (temperature) => 1 - perKelvin * (temperature - reference),
  };
}

/** Gear that stiffens with load: extra torque `−k₃ · twist³`. */
export function cubicStiffening(k3: number): ActuatorEffect {
  return { name: "cubic gear stiffening", gearTorque: ({ gearTwist }) => -k3 * gearTwist ** 3 };
}

/**
 * Dahl hysteresis in the gear: a friction-like torque that builds up with twist and
 * saturates at `limit`, so the twist–torque curve encloses a loop.
 */
export function dahlHysteresis(stiffness: number, limit: number): ActuatorEffect {
  let z = 0;
  return {
    name: "gear hysteresis (Dahl)",
    gearTorque: ({ gearTwistRate: rate }, dt) => {
      const direction = Math.sign(rate);
      // Exact update of dz/dt = σ·rate·(1 − z·sign(rate)/limit) over one step.
      const target = direction * limit;
      if (direction !== 0)
        z = target + (z - target) * Math.exp((-stiffness * Math.abs(rate) * dt) / limit);
      return -z;
    },
  };
}

/**
 * Backlash of total width `width` in a gear of stiffness `stiffness`: inside the play the
 * gear transmits (almost) nothing. Implemented as the deviation from the model's linear
 * spring, smoothed over the play so the plant stays well-posed.
 */
export function backlash(stiffness: number, width: number): ActuatorEffect {
  const half = width / 2;
  return {
    name: "backlash",
    gearTorque: ({ gearTwist }) => stiffness * half * Math.tanh(gearTwist / half),
  };
}

/**
 * Offsets in the phase-current sensors. The drive regulates the current it measures, so
 * an offset fixed in the stator frame turns into a q-axis current error that rotates
 * with the rotor: a torque ripple once per electrical turn, the same at any load,
 * `amplitude · sin(θe + phase)` with `amplitude` = |offset| · Kt. The joint measures the
 * current it regulates and does not see the error in its current reading.
 */
export function currentSensorOffset(
  electricalPerOutputRad: number,
  amplitude: number,
  phase: number,
): ActuatorEffect {
  return {
    name: "current-sensor offset",
    driveTorque: ({ motorAngle }) =>
      amplitude * Math.sin(motorAngle * electricalPerOutputRad + phase),
  };
}

/**
 * Gain errors of the phase-current sensors. With the two phases' gains off by `mismatch`
 * from each other and by `meanError` on average, the measured q-axis current reads high
 * by `meanError + (mismatch/2)·cos(2θe + phase)` of the true one; the drive regulates the
 * measured current, so the motor produces that much less: a torque constant error, and a
 * ripple twice per electrical turn in proportion to the load.
 */
export function currentSensorGain(
  electricalPerOutputRad: number,
  meanError: number,
  mismatch: number,
  phase: number,
): ActuatorEffect {
  return {
    name: "current-sensor gain error",
    driveTorque: ({ motorAngle }, produced) =>
      -produced *
      (meanError + (mismatch / 2) * Math.cos(2 * motorAngle * electricalPerOutputRad + phase)),
  };
}
