/**
 * Everything one Nexus unit senses about its own joint, sampled once per bus cycle.
 *
 * {@link JointSensors} depends only on {@link JointTruth}, not on the plant, so the same
 * sensor models work for any source of ground truth.
 */
import type { ActuatorSpec, Vec3 } from "../plant/armSpec.ts";
import type { Random } from "../util/random.ts";
import {
  AbsoluteEncoder,
  CurrentSensor,
  HallSensors,
  ImuAxis,
  TemperatureSensor,
} from "./models.ts";
import type { NexusSensorSpec } from "./spec.ts";

/** The true physical quantities at one joint for one cycle. */
export interface JointTruth {
  /** Motor-side angle, output-referred, rad. */
  readonly motorAngle: number;
  /** Output angle relative to the housing, rad. */
  readonly outputAngle: number;
  /** Angular velocity of the housing, IMU frame, rad/s. */
  readonly gyro: Vec3;
  /** Specific force at the housing, IMU frame, m/s². */
  readonly accel: Vec3;
  /** q-axis current, A. */
  readonly current: number;
  /** Winding temperature, °C. */
  readonly windingTemperature: number;
}

/** What a node receives from its own sensors each cycle. */
export interface SensorFrame {
  /** Motor-side encoder, output-referred, rad. */
  motorAngle: number;
  /** Output encoder, rad. */
  outputAngle: number;
  /** Hall commutation sector, 0…5. */
  hallSector: number;
  gyro: [number, number, number];
  accel: [number, number, number];
  /** Measured q-axis current, A. */
  current: number;
  /** Winding temperature, °C. */
  temperature: number;
}

export function emptySensorFrame(): SensorFrame {
  return {
    motorAngle: 0,
    outputAngle: 0,
    hallSector: 0,
    gyro: [0, 0, 0],
    accel: [0, 0, 0],
    current: 0,
    temperature: 0,
  };
}

type Triple<T> = readonly [T, T, T];

function measure3(axes: Triple<ImuAxis>, truth: Vec3, out: [number, number, number]): void {
  out[0] = axes[0].measure(truth[0]);
  out[1] = axes[1].measure(truth[1]);
  out[2] = axes[2].measure(truth[2]);
}

export class JointSensors {
  readonly #dt: number;
  readonly #motorEncoder: AbsoluteEncoder;
  readonly #outputEncoder: AbsoluteEncoder;
  readonly #hall: HallSensors;
  readonly #gyro: Triple<ImuAxis>;
  readonly #accel: Triple<ImuAxis>;
  readonly #current: CurrentSensor;
  readonly #temperature: TemperatureSensor;

  /**
   * @param rng stream for this joint; each sensor derives its own sub-stream from it
   */
  constructor(actuator: ActuatorSpec, spec: NexusSensorSpec, sampleRateHz: number, rng: Random) {
    this.#dt = 1 / sampleRateHz;
    this.#motorEncoder = new AbsoluteEncoder(
      spec.motorEncoder,
      actuator.gearRatio,
      rng.derive("motor-encoder"),
    );
    this.#outputEncoder = new AbsoluteEncoder(spec.outputEncoder, 1, rng.derive("output-encoder"));
    this.#hall = new HallSensors(actuator.electrical.polePairs, actuator.gearRatio);
    const axes = (name: string, axis: NexusSensorSpec["gyro"]): Triple<ImuAxis> => [
      new ImuAxis(axis, sampleRateHz, rng.derive(`${name}-x`)),
      new ImuAxis(axis, sampleRateHz, rng.derive(`${name}-y`)),
      new ImuAxis(axis, sampleRateHz, rng.derive(`${name}-z`)),
    ];
    this.#gyro = axes("gyro", spec.gyro);
    this.#accel = axes("accel", spec.accelerometer);
    this.#current = new CurrentSensor(spec.current, rng.derive("current"));
    this.#temperature = new TemperatureSensor(spec.temperature, rng.derive("temperature"));
  }

  /** Samples every sensor for one cycle and writes the readings into `out`. */
  sample(truth: JointTruth, out: SensorFrame): void {
    out.motorAngle = this.#motorEncoder.measure(truth.motorAngle);
    out.outputAngle = this.#outputEncoder.measure(truth.outputAngle);
    out.hallSector = this.#hall.sector(truth.motorAngle);
    measure3(this.#gyro, truth.gyro, out.gyro);
    measure3(this.#accel, truth.accel, out.accel);
    out.current = this.#current.measure(truth.current);
    out.temperature = this.#temperature.measure(truth.windingTemperature, this.#dt);
  }
}
