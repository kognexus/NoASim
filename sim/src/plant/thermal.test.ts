import { describe, expect, it } from "vitest";
import { KX6 } from "./kx6.ts";
import { ThermalModel } from "./thermal.ts";
import { IdealDrive } from "./drive.ts";

describe("ThermalModel", () => {
  it("settles at ambient + P·(R_wh + R_ha)", () => {
    const spec = KX6.joints[0]!.actuator.electrical.thermal;
    const model = new ThermalModel(spec, 25);
    for (let t = 0; t < 20_000; t += 0.5) model.step(20, 0.5);
    expect(model.winding).toBeCloseTo(25 + 20 * (spec.windingToHousing + spec.housingToAmbient), 1);
    expect(model.housing).toBeCloseTo(25 + 20 * spec.housingToAmbient, 1);
  });
});

describe("IdealDrive", () => {
  it("delivers the commanded torque, clamped to peak, with current τ / (Kt·N)", () => {
    const actuator = KX6.joints[0]!.actuator;
    const drive = new IdealDrive(actuator);
    const out = drive.update(10, 0, 0.00025);
    expect(out.torque).toBe(10);
    expect(out.current).toBeCloseTo(10 / (0.316 * 9), 9);
    expect(drive.update(1e6, 0, 0.00025).torque).toBe(actuator.peakTorque);
  });

  it.each(KX6.joints.map((joint) => [joint.name, joint.actuator] as const))(
    "keeps %s below its winding limit at rated torque, continuously",
    (_, actuator) => {
      const drive = new IdealDrive(actuator);
      for (let t = 0; t < 4 * 3600; t += 0.05) drive.update(actuator.ratedTorque, 0, 0.05);
      expect(drive.windingTemperature).toBeGreaterThan(80);
      expect(drive.windingTemperature).toBeLessThan(
        actuator.electrical.thermal.maxWindingTemperature,
      );
    },
  );
});
