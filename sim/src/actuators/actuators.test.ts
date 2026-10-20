import { beforeAll, describe, expect, it } from "vitest";
import type { ActuatorState } from "../plant/drive.ts";
import { KX6 } from "../plant/kx6.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import { Plant } from "../plant/plant.ts";
import { Random } from "../util/random.ts";
import {
  backlash,
  cogging,
  currentRipple,
  dahlHysteresis,
  torqueConstantDrift,
} from "./effects.ts";
import { ALL_FAMILIES, drawHiddenActuators, OPEN_FAMILIES } from "./families.ts";
import { HiddenActuator } from "./hiddenActuator.ts";

const LARGE = KX6.joints[0]!.actuator;
const state = (overrides: Partial<ActuatorState> = {}): ActuatorState => ({
  motorAngle: 0.2,
  motorVelocity: 0,
  gearTwist: 0,
  gearTwistRate: 0,
  ...overrides,
});

describe("open actuator families", () => {
  it.each(OPEN_FAMILIES.map((f) => [f.id, f] as const))(
    "%s: friction opposes motion",
    (_, family) => {
      const actuator = new HiddenActuator(LARGE, family.id, family.draw(LARGE, new Random(1)));
      for (const v of [0.5, -0.5]) {
        const out = actuator.update(0, state({ motorVelocity: v }), 0.00025);
        expect(Math.sign(out.frictionTorque)).toBe(-Math.sign(v));
      }
    },
  );
});

describe("effects", () => {
  it("cogging depends on position only; ripple scales with load", () => {
    const cog = cogging([{ cycles: 864, amplitude: 0.5, phase: 0 }]);
    expect(cog.driveTorque!(state(), 0)).toBe(cog.driveTorque!(state(), 20));
    const ripple = currentRipple(144, [{ cycles: 6, amplitude: 0.02, phase: 0.3 }]);
    expect(ripple.driveTorque!(state(), 20)).toBeCloseTo(2 * ripple.driveTorque!(state(), 10), 12);
  });

  it("weakens torque when the winding is hot", () => {
    const drift = torqueConstantDrift(0.0012);
    expect(drift.torqueConstantScale!(120)).toBeCloseTo(0.88, 12);
  });

  it("Dahl hysteresis removes energy over a twist cycle", () => {
    const effect = dahlHysteresis(1e4, 1);
    const dt = 1e-4;
    let work = 0;
    for (let k = 0; k < 2e4; k++) {
      const t = k * dt;
      const twist = 1e-3 * Math.sin(2 * Math.PI * t);
      const rate = 2e-3 * Math.PI * Math.cos(2 * Math.PI * t);
      work += effect.gearTorque!(state({ gearTwist: twist, gearTwistRate: rate }), dt) * rate * dt;
    }
    expect(work).toBeLessThan(0);
  });

  it("backlash cancels the spring inside the play and offsets it outside", () => {
    const k = 6e4;
    const width = 0.004;
    const effect = backlash(k, width);
    const net = (twist: number) => -k * twist + effect.gearTorque!(state({ gearTwist: twist }), 0);
    expect(Math.abs(net(0.0002))).toBeLessThan(0.02 * k * 0.0002);
    expect(net(0.02)).toBeCloseTo(-k * (0.02 - width / 2), 0);
  });
});

describe("drawHiddenActuators", () => {
  it("is reproducible and gives joints different families", () => {
    const ids = (seed: number) => drawHiddenActuators(KX6, new Random(seed)).map((a) => a.family);
    expect(ids(3)).toEqual(ids(3));
    expect(new Set(ids(3)).size).toBeGreaterThan(1);
  });

  it("never draws the sealed family unless asked", () => {
    const sealed = ALL_FAMILIES.filter((f) => f.sealed).map((f) => f.id);
    expect(sealed).toHaveLength(1);
    for (let seed = 0; seed < 50; seed++) {
      for (const actuator of drawHiddenActuators(KX6, new Random(seed))) {
        expect(sealed).not.toContain(actuator.family);
      }
    }
    const forced = drawHiddenActuators(KX6, new Random(0), {
      families: sealed,
      includeSealed: true,
    });
    expect(forced.every((a) => a.family === sealed[0])).toBe(true);
  });
});

describe("hidden actuators in the plant", () => {
  let mujoco: Mujoco;
  beforeAll(async () => {
    mujoco = await loadMujoco();
  });

  // Stability smoke test only: every family, including the sealed one, must run.
  it.each(ALL_FAMILIES.map((f) => [f.id] as const))("%s drives the arm without diverging", (id) => {
    const plant = Plant.create(mujoco, KX6, { gravity: false, externalFriction: true });
    const actuators = drawHiddenActuators(KX6, new Random(5), {
      families: [id],
      includeSealed: true,
    });
    try {
      for (let k = 0; k < 2000; k++) {
        actuators.forEach((actuator, i) => {
          const out = actuator.update(i === 0 ? 8 : 0, plant.actuatorState(i), plant.timestep);
          plant.setMotorTorque(i, out.driveTorque);
          plant.setActuatorForces(i, out.frictionTorque, out.gearTorque);
        });
        plant.step();
      }
      expect(plant.motorVelocity(0)).toBeGreaterThan(0);
      expect(plant.tcpPosition().every(Number.isFinite)).toBe(true);
    } finally {
      plant.dispose();
    }
  });
});
