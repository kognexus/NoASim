import { describe, expect, it } from "vitest";
import { KX6 } from "../plant/kx6.ts";
import { Random } from "../util/random.ts";
import { emptySensorFrame, JointSensors, type JointTruth } from "./jointSensors.ts";
import {
  AbsoluteEncoder,
  CurrentSensor,
  HallSensors,
  ImuAxis,
  TemperatureSensor,
} from "./models.ts";
import { NEXUS_SENSORS } from "./spec.ts";

const FS = 4000;

function stats(values: number[]): { mean: number; std: number } {
  const mean = values.reduce((s, v) => s + v, 0) / values.length;
  const std = Math.sqrt(values.reduce((s, v) => s + (v - mean) ** 2, 0) / values.length);
  return { mean, std };
}

describe("AbsoluteEncoder", () => {
  it("resolves the output to 19 bits and the motor side 9× finer than 14 bits", () => {
    const rng = new Random(1);
    expect(new AbsoluteEncoder(NEXUS_SENSORS.outputEncoder, 1, rng).resolution).toBeCloseTo(
      (2 * Math.PI) / 2 ** 19,
      15,
    );
    expect(new AbsoluteEncoder(NEXUS_SENSORS.motorEncoder, 9, rng).resolution).toBeCloseTo(
      (2 * Math.PI) / 2 ** 14 / 9,
      15,
    );
  });

  it("is unbiased with an error of about one count", () => {
    const encoder = new AbsoluteEncoder(NEXUS_SENSORS.outputEncoder, 1, new Random(2));
    const errors = Array.from({ length: 20_000 }, (_, k) => {
      const angle = 0.001 * k;
      return encoder.measure(angle) - angle;
    });
    const { mean, std } = stats(errors);
    expect(Math.abs(mean)).toBeLessThan(0.05 * encoder.resolution);
    // Noise of 1 count plus quantisation (1/√12 count).
    expect(std / encoder.resolution).toBeCloseTo(Math.sqrt(1 + 1 / 12), 1);
  });
});

describe("HallSensors", () => {
  it("steps through all six sectors once per electrical revolution", () => {
    const hall = new HallSensors(16, 9);
    const electricalRevolution = (2 * Math.PI) / (16 * 9);
    const seen = Array.from({ length: 60 }, (_, k) =>
      hall.sector((k + 0.5) * (electricalRevolution / 60)),
    );
    expect(new Set(seen)).toEqual(new Set([0, 1, 2, 3, 4, 5]));
    expect(seen.filter((s) => s === 3)).toHaveLength(10);
    expect(hall.sector(-0.001)).toBeGreaterThanOrEqual(0);
  });
});

describe("ImuAxis", () => {
  it.each([
    ["gyro", NEXUS_SENSORS.gyro],
    ["accelerometer", NEXUS_SENSORS.accelerometer],
  ] as const)("%s noise matches its density over the filter bandwidth", (_, spec) => {
    const axis = new ImuAxis({ ...spec, biasRandomWalk: 0 }, FS, new Random(3));
    const samples = Array.from({ length: 40_000 }, () => axis.measure(0));
    const { mean, std } = stats(samples.slice(100));
    // A first-order filter passes noise over an equivalent bandwidth of π/2 · f_c.
    const expected = spec.noiseDensity * Math.sqrt((Math.PI / 2) * spec.bandwidthHz);
    expect(std / expected).toBeGreaterThan(0.85);
    expect(std / expected).toBeLessThan(1.15);
    expect(Math.abs(mean - axis.bias)).toBeLessThan(0.2 * expected);
  });

  it("gives every unit a different bias and clips at full range", () => {
    const a = new ImuAxis(NEXUS_SENSORS.gyro, FS, new Random(4));
    const b = new ImuAxis(NEXUS_SENSORS.gyro, FS, new Random(5));
    expect(a.bias).not.toBeCloseTo(b.bias, 6);
    let reading = 0;
    for (let k = 0; k < 100; k++) reading = a.measure(1e3);
    expect(reading).toBeCloseTo(NEXUS_SENSORS.gyro.range, 2);
  });
});

describe("CurrentSensor and TemperatureSensor", () => {
  it("measures current with a fixed offset and white noise", () => {
    const sensor = new CurrentSensor(NEXUS_SENSORS.current, new Random(6));
    const { std } = stats(Array.from({ length: 20_000 }, () => sensor.measure(5)));
    expect(std).toBeCloseTo(NEXUS_SENSORS.current.noiseStd, 2);
  });

  it("lags a temperature step by its time constant", () => {
    const sensor = new TemperatureSensor(
      { ...NEXUS_SENSORS.temperature, noiseStd: 0 },
      new Random(7),
    );
    sensor.measure(25, 0.01);
    let reading = 0;
    for (let t = 0; t < NEXUS_SENSORS.temperature.timeConstant; t += 0.01)
      reading = sensor.measure(125, 0.01);
    expect(reading).toBeCloseTo(25 + 100 * (1 - Math.exp(-1)), 0);
  });
});

describe("JointSensors", () => {
  const truth: JointTruth = {
    motorAngle: 0.3,
    outputAngle: 0.31,
    gyro: [0.1, 0, 0],
    accel: [0, 0, 9.81],
    current: 2,
    windingTemperature: 40,
  };

  it("is reproducible from its seed", () => {
    const read = (seed: number) => {
      const sensors = new JointSensors(
        KX6.joints[0]!.actuator,
        NEXUS_SENSORS,
        FS,
        new Random(seed),
      );
      const frame = emptySensorFrame();
      sensors.sample(truth, frame);
      return frame;
    };
    expect(read(9)).toEqual(read(9));
    expect(read(9)).not.toEqual(read(10));
  });

  it("reports values close to the truth", () => {
    const sensors = new JointSensors(KX6.joints[0]!.actuator, NEXUS_SENSORS, FS, new Random(11));
    const frame = emptySensorFrame();
    sensors.sample(truth, frame);
    expect(frame.motorAngle).toBeCloseTo(0.3, 3);
    expect(frame.outputAngle).toBeCloseTo(0.31, 4);
    expect(frame.accel[2]).toBeCloseTo(9.81, 0);
    expect(frame.current).toBeCloseTo(2, 0);
    expect(frame.temperature).toBeCloseTo(40, 0);
  });
});
