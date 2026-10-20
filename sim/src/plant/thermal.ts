/**
 * Lumped thermal model of a motor: winding and housing as two heat capacities in series
 * between the copper losses and the ambient air.
 *
 *   P_cu → [C_w] ─R_wh─ [C_h] ─R_ha─ ambient
 */
import type { ThermalSpec } from "./armSpec.ts";

export class ThermalModel {
  readonly #spec: ThermalSpec;
  readonly ambient: number;
  #winding: number;
  #housing: number;

  constructor(spec: ThermalSpec, ambient = 25) {
    this.#spec = spec;
    this.ambient = ambient;
    this.#winding = ambient;
    this.#housing = ambient;
  }

  /** Winding temperature, °C. */
  get winding(): number {
    return this.#winding;
  }

  /** Housing temperature, °C. */
  get housing(): number {
    return this.#housing;
  }

  /** Advances by `dt` seconds with `power` watts of copper loss in the winding. */
  step(power: number, dt: number): void {
    const s = this.#spec;
    const toHousing = (this.#winding - this.#housing) / s.windingToHousing;
    const toAmbient = (this.#housing - this.ambient) / s.housingToAmbient;
    this.#winding += ((power - toHousing) / s.windingCapacity) * dt;
    this.#housing += ((toHousing - toAmbient) / s.housingCapacity) * dt;
  }
}
