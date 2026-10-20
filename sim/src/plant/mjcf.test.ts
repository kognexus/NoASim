import { beforeAll, describe, expect, it } from "vitest";
import type { ArmSpec } from "./armSpec.ts";
import { validateArmSpec } from "./armSpec.ts";
import { KX6 } from "./kx6.ts";
import { bendingAxes, buildMjcf } from "./mjcf.ts";
import { loadMujoco, type Mujoco } from "./mujoco.ts";

/** A 3-joint planar arm, to show the generator is not tied to KX-6. */
const THREE_LINK: ArmSpec = {
  name: "planar-3",
  joints: [0.3, 0.25, 0.2].map((length, i) => ({
    name: `p${i + 1}`,
    axis: [0, 1, 0] as const,
    initialAngle: 0,
    actuator: KX6.joints[2]!.actuator,
    link: { vector: [length, 0, 0] as const, mass: 1, radius: 0.03 },
  })),
  payload: { mass: 0, offset: [0, 0, 0] },
};

describe("validateArmSpec", () => {
  it("accepts KX-6", () => {
    expect(validateArmSpec(KX6)).toEqual([]);
  });

  it("reports bad values", () => {
    const [first, second] = KX6.joints;
    const bad: ArmSpec = {
      ...KX6,
      joints: [
        { ...first!, axis: [0, 0, 2], actuator: { ...first!.actuator, gearRatio: 0 } },
        { ...second!, name: first!.name },
      ],
    };
    const problems = validateArmSpec(bad);
    expect(problems).toContain("joint j1: axis must be a unit vector");
    expect(problems).toContain("joint j1: gearRatio must be positive");
    expect(problems).toContain("joint j1: duplicate name");
  });

  it("makes buildMjcf refuse an invalid spec", () => {
    expect(() => buildMjcf({ ...KX6, joints: [] })).toThrow(/no joints/);
  });
});

describe("bendingAxes", () => {
  it("returns two unit axes perpendicular to the link and to each other", () => {
    for (const link of [
      [0, 0, 0.4],
      [0.3, 0, 0],
      [0.1, 0.2, 0.3],
    ] as const) {
      const [a, b] = bendingAxes(link);
      const dot = (u: readonly number[], v: readonly number[]) =>
        u.reduce((s, x, k) => s + x * v[k]!, 0);
      expect(dot(a, link)).toBeCloseTo(0, 12);
      expect(dot(b, link)).toBeCloseTo(0, 12);
      expect(dot(a, b)).toBeCloseTo(0, 12);
      expect(Math.hypot(...a)).toBeCloseTo(1, 12);
    }
  });
});

describe("buildMjcf", () => {
  let mujoco: Mujoco;
  beforeAll(async () => {
    mujoco = await loadMujoco();
  });

  function load(spec: ArmSpec, lockMotors = false) {
    const model = mujoco.MjModel.from_xml_string(buildMjcf(spec, { lockMotors }));
    const sizes = { nq: model.nq, nu: model.nu, timestep: model.opt.timestep };
    model.delete();
    return sizes;
  }

  it("gives KX-6 a motor and a gear hinge per joint plus two bending hinges per flexible link", () => {
    expect(load(KX6)).toEqual({ nq: 6 + 6 + 2 * 2, nu: 6, timestep: 0.00025 });
  });

  it("drops motor hinges and actuators when motors are locked", () => {
    const sizes = load(KX6, true);
    expect(sizes.nq).toBe(6 + 2 * 2);
    expect(sizes.nu).toBe(0);
  });

  it("builds arms other than KX-6", () => {
    expect(load(THREE_LINK)).toMatchObject({ nq: 6, nu: 3 });
  });

  it("puts each joint's IMU on its housing, not on its own link", () => {
    const xml = buildMjcf(KX6);
    const rotor = xml.indexOf('<body name="j2_rotor"');
    const imu = xml.indexOf('<site name="j2_imu"');
    expect(imu).toBeGreaterThan(0);
    expect(imu).toBeLessThan(rotor);
  });
});
