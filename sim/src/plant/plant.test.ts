import { afterEach, beforeAll, describe, expect, it } from "vitest";
import { structuralMode } from "../analysis/structuralMode.ts";
import { KX6, KX6L } from "./kx6.ts";
import { loadMujoco, type Mujoco } from "./mujoco.ts";
import { Plant } from "./plant.ts";

describe("Plant (KX-6)", () => {
  let mujoco: Mujoco;
  let plant: Plant | undefined;
  beforeAll(async () => {
    mujoco = await loadMujoco();
  });
  afterEach(() => {
    plant?.dispose();
    plant = undefined;
  });

  it("starts stretched out horizontally", () => {
    plant = Plant.create(mujoco, KX6);
    const [x, y, z] = plant.tcpPosition();
    expect(x).toBeCloseTo(0.42 + 0.39 + 0.11 + 0.1 + 0.09, 6);
    expect(y).toBeCloseTo(0, 6);
    expect(z).toBeCloseTo(0.05 + 0.16, 6);
    expect(plant.motorAngle(1)).toBeCloseTo(Math.PI / 2, 12);
  });

  it("turns a joint with motor torque and twists its gear stage", () => {
    plant = Plant.create(mujoco, KX6, { gravity: false });
    plant.setMotorTorque(0, 5);
    for (let k = 0; k < 400; k++) plant.step();
    expect(plant.motorVelocity(0)).toBeGreaterThan(0);
    // The gear stage lags the motor while it accelerates the arm.
    expect(plant.gearTwist(0)).toBeLessThan(0);
    expect(plant.outputAngle(0)).toBeCloseTo(plant.motorAngle(0) + plant.gearTwist(0), 12);
  });

  it("measures joint 2's housing motion with joint 2's IMU, which joint 1 carries", () => {
    plant = Plant.create(mujoco, KX6, { gravity: false });
    plant.setMotorTorque(0, 5);
    for (let k = 0; k < 400; k++) plant.step();
    expect(Math.hypot(...plant.gyro(0))).toBe(0); // joint 1's housing is the fixed base
    expect(Math.abs(plant.gyro(1)[2])).toBeGreaterThan(0.01); // joint 2's housing turns with link 1
  });

  it("sags a few millimetres under gravity with the motors locked", () => {
    plant = Plant.create(mujoco, KX6, { lockMotors: true });
    const z0 = plant.tcpPosition()[2];
    for (let k = 0; k < 4 * 4000; k++) plant.step();
    const sag = z0 - plant.tcpPosition()[2];
    expect(sag).toBeGreaterThan(0.001);
    expect(sag).toBeLessThan(0.02);
  });
});

describe("KX-6 structural modes (NX-02 acceptance)", () => {
  it.each([
    ["vertical", [0, 0, 1]],
    ["lateral", [0, 1, 0]],
  ] as const)("first %s mode lies at 8–15 Hz with 1–3 %% damping", async (_, direction) => {
    const mode = structuralMode(await loadMujoco(), KX6, { direction });
    expect(mode.frequencyHz).toBeGreaterThanOrEqual(8);
    expect(mode.frequencyHz).toBeLessThanOrEqual(15);
    expect(mode.dampingRatio).toBeGreaterThanOrEqual(0.01);
    expect(mode.dampingRatio).toBeLessThanOrEqual(0.03);
  });
});

describe("KX-6L structural modes", () => {
  it("has a low, lightly damped first mode (lightweight arm)", async () => {
    const mode = structuralMode(await loadMujoco(), KX6L, { direction: [0, 0, 1] });
    expect(mode.frequencyHz).toBeGreaterThan(4);
    expect(mode.frequencyHz).toBeLessThan(8);
    expect(mode.dampingRatio).toBeLessThan(0.02);
  });
});
