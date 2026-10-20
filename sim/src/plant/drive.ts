/**
 * Motor drive physics: turns the torque a node commands into the torque the motor
 * produces, the current that flows and the heat it makes.
 *
 * NX-03 provides the ideal drive (perfect current loop, constant torque constant).
 * NX-04 adds actuators whose behaviour the node does not know (ripple, friction shapes,
 * torque-constant drift) behind the same interface.
 */
import type { ActuatorSpec } from "./armSpec.ts";
import { outputTorqueConstant } from "./armSpec.ts";
import { ThermalModel } from "./thermal.ts";

/** Copper resistance rises about 0.393 %/K. */
const COPPER_TEMPERATURE_COEFFICIENT = 0.00393;

/** State of a drive after one cycle. */
export interface DriveOutput {
  /** Torque the motor produces at the output, N·m. */
  readonly torque: number;
  /** q-axis current, A. */
  readonly current: number;
}

/** The physics between a node's torque command and the plant. */
export interface Drive {
  /**
   * Applies one cycle of `commandTorque` (output-referred, N·m) with the motor at
   * `motorAngle` (output-referred, rad) and returns what the motor actually does.
   */
  update(commandTorque: number, motorAngle: number, dt: number): DriveOutput;
  /** Winding temperature, °C. */
  readonly windingTemperature: number;
}

/** A drive with an ideal current loop: the motor produces exactly the commanded torque. */
export class IdealDrive implements Drive {
  readonly #actuator: ActuatorSpec;
  readonly #thermal: ThermalModel;
  readonly #ktOut: number;

  constructor(actuator: ActuatorSpec, ambient = 25) {
    this.#actuator = actuator;
    this.#thermal = new ThermalModel(actuator.electrical.thermal, ambient);
    this.#ktOut = outputTorqueConstant(actuator);
  }

  get windingTemperature(): number {
    return this.#thermal.winding;
  }

  update(commandTorque: number, _motorAngle: number, dt: number): DriveOutput {
    const peak = this.#actuator.peakTorque;
    const torque = Math.min(peak, Math.max(-peak, commandTorque));
    const current = torque / this.#ktOut;
    const resistance =
      this.#actuator.electrical.phaseResistance *
      (1 + COPPER_TEMPERATURE_COEFFICIENT * (this.#thermal.winding - 20));
    this.#thermal.step(1.5 * resistance * current * current, dt);
    return { torque, current };
  }
}
