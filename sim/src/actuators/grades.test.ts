import { describe, expect, it } from "vitest";
import type { ActuatorState } from "../plant/drive.ts";
import { KX6 } from "../plant/kx6.ts";
import { Random } from "../util/random.ts";
import { currentSensorGain, currentSensorOffset } from "./effects.ts";
import { ALL_FAMILIES, drawHiddenActuators } from "./families.ts";
import { GRADES } from "./grades.ts";

const LARGE = KX6.joints[0]!.actuator;
const ELECTRICAL = LARGE.electrical.polePairs * LARGE.gearRatio;
const at = (electricalAngle: number): ActuatorState => ({
  motorAngle: electricalAngle / ELECTRICAL,
  motorVelocity: 0,
  gearTwist: 0,
  gearTwistRate: 0,
});

/** Amplitude of the `order`-th harmonic of `f` over one electrical turn. */
function harmonic(f: (electricalAngle: number) => number, order: number): number {
  const n = 720;
  let re = 0;
  let im = 0;
  for (let k = 0; k < n; k++) {
    const x = (2 * Math.PI * k) / n;
    re += f(x) * Math.cos(order * x);
    im += f(x) * Math.sin(order * x);
  }
  return (2 * Math.hypot(re, im)) / n;
}

describe("phase-current sensing errors (NX-26)", () => {
  it("an offset ripples the torque once per electrical turn, the same at any load", () => {
    const offset = currentSensorOffset(ELECTRICAL, 0.6, 1.1);
    for (const load of [0, 20]) {
      const torque = (x: number) => offset.driveTorque!(at(x), load);
      expect(harmonic(torque, 1)).toBeCloseTo(0.6, 6);
      expect(harmonic(torque, 2)).toBeLessThan(1e-9);
    }
  });

  it("gain mismatch ripples it twice per electrical turn, in proportion to the load", () => {
    const gain = currentSensorGain(ELECTRICAL, 0.01, 0.04, 0.4);
    for (const load of [10, 20]) {
      const torque = (x: number) => gain.driveTorque!(at(x), load);
      expect(harmonic(torque, 2)).toBeCloseTo(load * 0.02, 6);
      expect(harmonic(torque, 1)).toBeLessThan(1e-9);
      // The mean gain error: the motor makes 1 % less than the drive thinks.
      const mean = Array.from({ length: 360 }, (_, k) => torque((2 * Math.PI * k) / 360));
      expect(mean.reduce((a, b) => a + b, 0) / 360).toBeCloseTo(-0.01 * load, 6);
    }
  });
});

describe("actuator grades (NX-26)", () => {
  it("are drawn only when asked for, so earlier draws do not change", () => {
    const before = drawHiddenActuators(KX6, new Random(300)).map((a) => a.family);
    expect(before.every((f) => ALL_FAMILIES.some((family) => family.id === f))).toBe(true);
    const budget = drawHiddenActuators(KX6, new Random(300), { families: ["grade-budget"] });
    expect(budget.map((a) => a.family)).toEqual(KX6.joints.map(() => "grade-budget"));
  });

  it("draw deterministically per seed", () => {
    const draw = () =>
      drawHiddenActuators(KX6, new Random(5), { families: ["grade-budget"] }).map(
        (a) => a.update(3, at(0.7), 0.00025).driveTorque,
      );
    expect(draw()).toEqual(draw());
  });

  it("a budget actuator ripples far more than a premium one", () => {
    // RMS of the torque ripple at rated load over an electrical turn, averaged over draws.
    const ripple = (id: string) => {
      const grade = GRADES.find((g) => g.id === id)!;
      let sum = 0;
      for (let seed = 1; seed <= 10; seed++) {
        const effects = grade.draw(LARGE, new Random(seed));
        const torque = Array.from({ length: 720 }, (_, k) =>
          effects.reduce(
            (t, e) => t + (e.driveTorque?.(at((2 * Math.PI * k) / 720), LARGE.ratedTorque) ?? 0),
            0,
          ),
        );
        const mean = torque.reduce((a, b) => a + b, 0) / torque.length;
        sum += Math.sqrt(torque.reduce((a, b) => a + (b - mean) ** 2, 0) / torque.length);
      }
      return sum / 10;
    };
    expect(ripple("grade-budget")).toBeGreaterThan(5 * ripple("grade-premium"));
  });
});
