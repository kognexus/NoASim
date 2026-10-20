import { describe, expect, it } from "vitest";
import { KX6, KX6_HOME } from "../plant/kx6.ts";
import { Random } from "../util/random.ts";
import { flangePose, poseError, solveIk } from "./ik.ts";
import { fastReach, figureEight, pickAndPlace } from "./tasks.ts";
import { VlaEmulator } from "./vla.ts";

const HOME = flangePose(KX6, KX6_HOME);

describe("inverse kinematics", () => {
  it("reaches targets within 10 cm of the home pose to within 0.1 mm", () => {
    const rng = new Random(1);
    for (let k = 0; k < 20; k++) {
      const target = {
        position: [0, 1, 2].map((a) => HOME.position[a as 0 | 1 | 2] + rng.between(-0.1, 0.1)) as [
          number,
          number,
          number,
        ],
        rotation: HOME.rotation,
      };
      const q = solveIk(KX6, target, KX6_HOME, { iterations: 10, lambda: 0.005 });
      const e = poseError(flangePose(KX6, q), target);
      expect(Math.hypot(e[0]!, e[1]!, e[2]!)).toBeLessThan(1e-4);
    }
  });
});

describe("tasks", () => {
  it.each([figureEight(HOME), pickAndPlace(HOME), fastReach(HOME)])(
    "$name starts at home and moves smoothly",
    (task) => {
      const at = (t: number) => task.pose(t).position;
      expect(Math.hypot(...at(0).map((v, k) => v - HOME.position[k]!))).toBeLessThan(1e-9);
      for (let t = 0; t < task.duration; t += 0.001) {
        const step = Math.hypot(...at(t + 0.001).map((v, k) => v - at(t)[k]!));
        expect(step).toBeLessThan(0.002); // under 2 m/s
      }
    },
  );
});

describe("VLA emulator", () => {
  it("emits late, jittery steps at its rate and drops some", () => {
    const task = figureEight(HOME, 0.15, 4, 5);
    const vla = new VlaEmulator(task, 1, new Random(2), {
      rateHz: 15,
      latencySeconds: 0.03,
      jitterSeconds: 0.015,
      dropRate: 0.1,
    });
    const steps = [];
    for (let t = 1; t < 1 + task.duration + 1; t += 0.00025) {
      const step = vla.poll(t);
      if (step) steps.push({ step, t });
    }
    const planned = Math.floor(task.duration * 15) + 1;
    expect(steps.length + vla.dropped).toBe(planned);
    expect(vla.dropped / planned).toBeGreaterThan(0.05);
    expect(vla.dropped / planned).toBeLessThan(0.16);
    for (const { step, t } of steps) {
      const plannedAt = 1 + step.index / 15;
      expect(t - plannedAt).toBeGreaterThanOrEqual(0.03 - 1e-9);
      expect(t - plannedAt).toBeLessThanOrEqual(0.045 + 0.0003);
    }
  });
});
