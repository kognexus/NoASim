/**
 * Sensor parameters of the Nexus unit (the same board on every joint).
 *
 * Values are typical of the parts named in the SPRIND notes: a 14-bit magnetic encoder on
 * the motor shaft, a 19-bit absolute ring encoder on the output, a BMI330-class 6-axis
 * IMU, phase-current sensing on a 12-bit ADC and an NTC in the winding.
 */

const DEG = Math.PI / 180;
const G = 9.80665;

export interface EncoderSpec {
  /** Counts per revolution of the shaft it sits on, as bits. */
  readonly bits: number;
  /** Measurement noise, standard deviation in counts. */
  readonly noiseCounts: number;
}

/** One IMU axis. Units are rad/s for the gyro and m/s² for the accelerometer. */
export interface ImuAxisSpec {
  /** White-noise density, unit/√Hz. */
  readonly noiseDensity: number;
  /** −3 dB bandwidth of the sensor's internal low-pass filter, Hz. */
  readonly bandwidthHz: number;
  /** Spread of the constant bias between units, standard deviation. */
  readonly biasStd: number;
  /** Bias random walk, unit/√s. */
  readonly biasRandomWalk: number;
  /** Full-scale range (±), unit. */
  readonly range: number;
  /** Bits of the output word. */
  readonly bits: number;
}

export interface CurrentSensorSpec {
  readonly noiseStd: number;
  readonly offsetStd: number;
  /** Full-scale range (±), A. */
  readonly range: number;
  readonly bits: number;
}

export interface TemperatureSensorSpec {
  readonly noiseStd: number;
  /** Lag of the sensor behind the winding, s. */
  readonly timeConstant: number;
  /** Resolution, °C. */
  readonly resolution: number;
}

export interface NexusSensorSpec {
  readonly motorEncoder: EncoderSpec;
  readonly outputEncoder: EncoderSpec;
  readonly gyro: ImuAxisSpec;
  readonly accelerometer: ImuAxisSpec;
  readonly current: CurrentSensorSpec;
  readonly temperature: TemperatureSensorSpec;
}

export const NEXUS_SENSORS: NexusSensorSpec = {
  motorEncoder: { bits: 14, noiseCounts: 0.7 },
  outputEncoder: { bits: 19, noiseCounts: 1.0 },
  gyro: {
    noiseDensity: 0.007 * DEG,
    bandwidthHz: 750,
    biasStd: 0.5 * DEG,
    biasRandomWalk: 0.002 * DEG,
    range: 2000 * DEG,
    bits: 16,
  },
  accelerometer: {
    noiseDensity: 180e-6 * G,
    bandwidthHz: 750,
    biasStd: 0.02 * G,
    biasRandomWalk: 2e-5 * G,
    range: 16 * G,
    bits: 16,
  },
  current: { noiseStd: 0.05, offsetStd: 0.05, range: 60, bits: 12 },
  temperature: { noiseStd: 0.1, timeConstant: 2, resolution: 0.1 },
};
