/**
 * An actuator whose physics is a set of {@link ActuatorEffect}s the node is not told
 * about. It supplies all friction itself, so the MuJoCo model is built without the
 * datasheet friction (`externalFriction`).
 */
import type { ActuatorSpec } from "../plant/armSpec.ts";
import {
  type ActuatorOutput,
  type ActuatorPhysics,
  type ActuatorState,
  Winding,
} from "../plant/drive.ts";
import type { ActuatorEffect } from "./effects.ts";

export class HiddenActuator implements ActuatorPhysics {
  readonly frictionInModel = false;
  /** Family the actuator was drawn from (ground truth, never given to a node). */
  readonly family: string;
  readonly effects: readonly ActuatorEffect[];
  readonly #winding: Winding;

  constructor(
    actuator: ActuatorSpec,
    family: string,
    effects: readonly ActuatorEffect[],
    ambient = 25,
  ) {
    this.family = family;
    this.effects = effects;
    this.#winding = new Winding(actuator, ambient);
  }

  get windingTemperature(): number {
    return this.#winding.temperature;
  }

  update(commandTorque: number, state: ActuatorState, dt: number): ActuatorOutput {
    let ktScale = 1;
    for (const effect of this.effects)
      ktScale *= effect.torqueConstantScale?.(this.#winding.temperature) ?? 1;
    const { torque, current } = this.#winding.drive(commandTorque, ktScale, dt);
    let driveTorque = torque;
    let frictionTorque = 0;
    let gearTorque = 0;
    for (const effect of this.effects) {
      driveTorque += effect.driveTorque?.(state, torque) ?? 0;
      frictionTorque += effect.friction?.(state, dt) ?? 0;
      gearTorque += effect.gearTorque?.(state, dt) ?? 0;
    }
    return { driveTorque, frictionTorque, gearTorque, current };
  }
}
