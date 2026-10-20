import { describe, expect, it } from "vitest";
import { forwardKinematics, jointLoadInertia } from "./kinematics.ts";
import { KX6 } from "./kx6.ts";

describe("nominal kinematics", () => {
  it("puts the KX-6 flange where the plant does", () => {
    const pose = forwardKinematics(
      KX6,
      KX6.joints.map((j) => j.initialAngle),
    );
    expect(pose.flange[0]).toBeCloseTo(1.11, 6);
    expect(pose.flange[2]).toBeCloseTo(0.21, 6);
  });

  it("gives the shoulder the largest load and grows with reach", () => {
    const stretched = jointLoadInertia(KX6);
    expect(Math.max(...stretched)).toBe(stretched[1]);
    expect(stretched[1]).toBeGreaterThan(4);
    const folded = jointLoadInertia(KX6, [0, Math.PI / 2, -Math.PI / 2, 0, 0, 0]);
    expect(folded[1]).toBeLessThan(stretched[1]!);
  });
});
