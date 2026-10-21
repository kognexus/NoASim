/**
 * An actuator with a disturbance added on top of its own physics: how the demo and the
 * report-to-source tests inject ripple or a worn bearing into one joint.
 */
import type { ActuatorOutput, ActuatorPhysics, ActuatorState } from "../plant/drive.ts";
import type { ActuatorEffect } from "./effects.ts";

export class DisturbedActuator implements ActuatorPhysics {
  readonly inner: ActuatorPhysics;
  readonly effect: ActuatorEffect;

  constructor(inner: ActuatorPhysics, effect: ActuatorEffect) {
    this.inner = inner;
    this.effect = effect;
  }

  get windingTemperature(): number {
    return this.inner.windingTemperature;
  }

  get frictionInModel(): boolean {
    return this.inner.frictionInModel;
  }

  update(commandTorque: number, state: ActuatorState, dt: number): ActuatorOutput {
    const out = this.inner.update(commandTorque, state, dt);
    return {
      ...out,
      driveTorque: out.driveTorque + (this.effect.driveTorque?.(state, out.driveTorque) ?? 0),
      frictionTorque: out.frictionTorque + (this.effect.friction?.(state, dt) ?? 0),
      gearTorque: out.gearTorque + (this.effect.gearTorque?.(state, dt) ?? 0),
    };
  }
}
