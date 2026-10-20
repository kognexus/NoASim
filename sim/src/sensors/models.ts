/**
 * Models of the individual sensors on a Nexus unit. Each turns a true quantity into what
 * the sensor would report: quantised, noisy, offset, filtered and range-limited.
 */
import type { Random } from "../util/random.ts";
import type { CurrentSensorSpec, EncoderSpec, ImuAxisSpec, TemperatureSensorSpec } from "./spec.ts";

function quantize(value: number, step: number): number {
  return Math.round(value / step) * step;
}

function clamp(value: number, limit: number): number {
  return Math.min(limit, Math.max(-limit, value));
}

/**
 * Absolute encoder on a shaft geared `ratio`:1 to the quantity it reports.
 *
 * The motor-side encoder has `ratio` = gear ratio, so its resolution at the output is
 * `ratio` times finer. Multi-turn tracking is assumed to happen in the encoder, so the
 * reading is continuous.
 */
export class AbsoluteEncoder {
  readonly #step: number;
  readonly #ratio: number;
  readonly #noise: number;
  readonly #rng: Random;

  constructor(spec: EncoderSpec, ratio: number, rng: Random) {
    this.#step = (2 * Math.PI) / 2 ** spec.bits;
    this.#ratio = ratio;
    this.#noise = spec.noiseCounts * this.#step;
    this.#rng = rng;
  }

  /** Resolution at the reported quantity, rad. */
  get resolution(): number {
    return this.#step / this.#ratio;
  }

  /** Measures `angle` (rad, at the reported side). */
  measure(angle: number): number {
    const shaft = angle * this.#ratio + this.#noise * this.#rng.normal();
    return quantize(shaft, this.#step) / this.#ratio;
  }
}

/** Three Hall sensors 120° apart electrically: reports the commutation sector 0…5. */
export class HallSensors {
  readonly #electricalPerOutput: number;
  readonly #offset: number;

  /**
   * @param polePairs motor pole pairs
   * @param gearRatio motor turns per output turn
   * @param offset electrical angle of the Hall pattern relative to the rotor, rad
   */
  constructor(polePairs: number, gearRatio: number, offset = 0) {
    this.#electricalPerOutput = polePairs * gearRatio;
    this.#offset = offset;
  }

  /** Sector for an output-referred motor angle. */
  sector(motorAngle: number): number {
    const electrical = motorAngle * this.#electricalPerOutput + this.#offset;
    const wrapped = ((electrical % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
    return Math.min(5, Math.floor(wrapped / (Math.PI / 3)));
  }
}

/**
 * One IMU axis: constant bias, bias random walk, white noise, first-order low-pass
 * (the sensor's digital filter), clipping and quantisation.
 */
export class ImuAxis {
  readonly #spec: ImuAxisSpec;
  readonly #rng: Random;
  readonly #lsb: number;
  readonly #dt: number;
  readonly #alpha: number;
  readonly #whiteStd: number;
  readonly #walkStd: number;
  #bias: number;
  #filtered: number | undefined;

  constructor(spec: ImuAxisSpec, sampleRateHz: number, rng: Random) {
    this.#spec = spec;
    this.#rng = rng;
    this.#lsb = (2 * spec.range) / 2 ** spec.bits;
    this.#dt = 1 / sampleRateHz;
    this.#alpha = 1 - Math.exp(-2 * Math.PI * spec.bandwidthHz * this.#dt);
    // White noise of density n sampled at fs has standard deviation n·√(fs/2).
    this.#whiteStd = spec.noiseDensity * Math.sqrt(sampleRateHz / 2);
    this.#walkStd = spec.biasRandomWalk * Math.sqrt(this.#dt);
    this.#bias = spec.biasStd * rng.normal();
  }

  /** The unit's current bias (for tests and ground-truth logging only). */
  get bias(): number {
    return this.#bias;
  }

  measure(truth: number): number {
    this.#bias += this.#walkStd * this.#rng.normal();
    const raw = truth + this.#bias + this.#whiteStd * this.#rng.normal();
    this.#filtered =
      this.#filtered === undefined ? raw : this.#filtered + this.#alpha * (raw - this.#filtered);
    return quantize(clamp(this.#filtered, this.#spec.range), this.#lsb);
  }
}

/** Phase-current measurement of the q-axis current. */
export class CurrentSensor {
  readonly #spec: CurrentSensorSpec;
  readonly #rng: Random;
  readonly #offset: number;
  readonly #lsb: number;

  constructor(spec: CurrentSensorSpec, rng: Random) {
    this.#spec = spec;
    this.#rng = rng;
    this.#offset = spec.offsetStd * rng.normal();
    this.#lsb = (2 * spec.range) / 2 ** spec.bits;
  }

  measure(current: number): number {
    const raw = current + this.#offset + this.#spec.noiseStd * this.#rng.normal();
    return quantize(clamp(raw, this.#spec.range), this.#lsb);
  }
}

/** Winding temperature sensor (NTC): lags the winding, noisy, coarse. */
export class TemperatureSensor {
  readonly #spec: TemperatureSensorSpec;
  readonly #rng: Random;
  #lagged: number | undefined;

  constructor(spec: TemperatureSensorSpec, rng: Random) {
    this.#spec = spec;
    this.#rng = rng;
  }

  measure(winding: number, dt: number): number {
    const alpha = 1 - Math.exp(-dt / this.#spec.timeConstant);
    this.#lagged =
      this.#lagged === undefined ? winding : this.#lagged + alpha * (winding - this.#lagged);
    return quantize(this.#lagged + this.#spec.noiseStd * this.#rng.normal(), this.#spec.resolution);
  }
}
