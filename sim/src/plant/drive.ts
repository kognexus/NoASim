/**
 * Actuator physics: turns the torque a node commands into what the actuator really does.
 *
 * The node asks for an output torque. The physics returns the torque the motor produces,
 * the friction and gear torques the plant should apply, the current that flows and the
 * heat it makes. {@link IdealDrive} is the datasheet actuator. NX-04 adds actuators the
 * node does not know (`sim/src/actuators/`) behind the same interface.
 */
import type { ActuatorSpec } from "./armSpec.ts";
import { outputTorqueConstant } from "./armSpec.ts";
import { ThermalModel } from "./thermal.ts";

/** Copper resistance rises about 0.393 %/K. */
const COPPER_TEMPERATURE_COEFFICIENT = 0.00393;

/** The mechanical state an actuator's physics depends on (all output-referred). */
export interface ActuatorState {
  readonly motorAngle: number;
  readonly motorVelocity: number;
  readonly gearTwist: number;
  readonly gearTwistRate: number;
}

/** What the actuator does in one cycle. Torques are output-referred, N·m. */
export interface ActuatorOutput {
  /** Electromagnetic torque of the motor, applied on the motor hinge. */
  readonly driveTorque: number;
  /** Friction on the motor hinge (zero when the model carries friction itself). */
  readonly frictionTorque: number;
  /** Gear torque beyond the model's linear spring, applied on the gear hinge. */
  readonly gearTorque: number;
  /** q-axis current, A. */
  readonly current: number;
}

/** The physics between a node's torque command and the plant. */
export interface ActuatorPhysics {
  /** Advances one cycle of `commandTorque` with the actuator in `state`. */
  update(commandTorque: number, state: ActuatorState, dt: number): ActuatorOutput;
  /** Winding temperature, °C. */
  readonly windingTemperature: number;
  /**
   * Whether the MuJoCo model should carry the datasheet friction (`true`), or the physics
   * supplies all friction itself (`false`).
   */
  readonly frictionInModel: boolean;
}

/**
 * The common electrical core: torque command → current → heat, with a torque-constant
 * scale for actuators whose magnets weaken when hot.
 */
export class Winding {
  readonly #actuator: ActuatorSpec;
  readonly #thermal: ThermalModel;
  readonly #ktOut: number;

  constructor(actuator: ActuatorSpec, ambient = 25) {
    this.#actuator = actuator;
    this.#thermal = new ThermalModel(actuator.electrical.thermal, ambient);
    this.#ktOut = outputTorqueConstant(actuator);
  }

  get temperature(): number {
    return this.#thermal.winding;
  }

  /**
   * The drive controls current to `command / Kt_nominal` (clamped to peak). The motor
   * produces `ktScale` times the commanded torque.
   */
  drive(command: number, ktScale: number, dt: number): { torque: number; current: number } {
    const peak = this.#actuator.peakTorque;
    const clamped = Math.min(peak, Math.max(-peak, command));
    const current = clamped / this.#ktOut;
    const resistance =
      this.#actuator.electrical.phaseResistance *
      (1 + COPPER_TEMPERATURE_COEFFICIENT * (this.#thermal.winding - 20));
    this.#thermal.step(1.5 * resistance * current * current, dt);
    return { torque: clamped * ktScale, current };
  }
}

/** The datasheet actuator: produces exactly the commanded torque, friction as in the model. */
export class IdealDrive implements ActuatorPhysics {
  readonly frictionInModel = true;
  readonly #winding: Winding;

  constructor(actuator: ActuatorSpec, ambient = 25) {
    this.#winding = new Winding(actuator, ambient);
  }

  get windingTemperature(): number {
    return this.#winding.temperature;
  }

  update(commandTorque: number, _state: ActuatorState, dt: number): ActuatorOutput {
    const { torque, current } = this.#winding.drive(commandTorque, 1, dt);
    return { driveTorque: torque, frictionTorque: 0, gearTorque: 0, current };
  }
}
