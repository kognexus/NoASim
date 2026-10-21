/**
 * Commissioning of the impact thresholds (NX-16): run normal tasks, record each joint's
 * external-torque estimate exactly as the node computes it (gear-twist torque minus the
 * dynamic chain), and set each threshold to twice the envelope seen with joint-level
 * interpolation (the arm's normal mode), but at least 30 % of rated torque. The central
 * runs are reported to check those thresholds hold there too.
 *
 *   node experiments/src/calibrate-impact.ts
 *
 * Prints the thresholds to paste into sim/src/harness/impactThresholds.ts.
 */
import { ArmSystem } from "../../sim/src/harness/armSystem.ts";
import type { InterpolationMode } from "../../sim/src/host/host.ts";
import { fastReach, figureEight, pickAndPlace } from "../../sim/src/host/tasks.ts";
import { KX6, KX6_HOME, withInitialAngles } from "../../sim/src/plant/kx6.ts";
import { sideReach } from "../../sim/src/scenarios/impact.ts";
import { homePose } from "../../sim/src/scenarios/motion.ts";
import { openLab } from "./runner.ts";

const SEEDS = [51, 52, 53];
const MODES: InterpolationMode[] = ["joint-level", "central"];
const FILTER_HZ = 10;
const MARGIN = 2;
const MIN_SHARE = 0.3;

const spec = withInitialAngles(KX6, KX6_HOME);
const tasks = {
  "figure-eight": (s: ArmSystem) => figureEight(homePose(s)),
  "pick-and-place": (s: ArmSystem) => pickAndPlace(homePose(s)),
  "fast-reach": (s: ArmSystem) => fastReach(homePose(s)),
  "side-reach": (s: ArmSystem) => sideReach(s),
};

const lab = await openLab();
const envelope = spec.joints.map(() => 0);
const centralEnvelope = spec.joints.map(() => 0);
const alpha = 1 / (1 + 4000 / (2 * Math.PI * FILTER_HZ));
for (const seed of SEEDS) {
  for (const mode of MODES) {
    for (const [name, make] of Object.entries(tasks)) {
      const system = await ArmSystem.create(lab.mujoco, lab.nodeModule, { spec, seed });
      system.run(5);
      const task = make(system);
      system.host.startTask(task, system.cycle, { mode });
      const dynamic = new Map<number, number>();
      const twist = new Map<number, number>();
      const filtered = spec.joints.map(() => 0);
      const local = spec.joints.map(() => 0);
      for (let k = 0; k < (task.duration + 0.5) * 4000; k++) {
        system.step();
        for (const f of system.host.lastReceived) {
          const axis = spec.joints[f.header.source - 1]?.axis ?? [0, 0, 0];
          if ("chain" in f) {
            const m = f.chain.dynamicMoment;
            dynamic.set(f.header.source, axis[0] * m[0] + axis[1] * m[1] + axis[2] * m[2]);
          }
          if ("state" in f) twist.set(f.header.source, f.state.gearTwist);
        }
        spec.joints.forEach((joint, i) => {
          const residual =
            -joint.actuator.gearStiffness * (twist.get(i + 1) ?? 0) - (dynamic.get(i + 1) ?? 0);
          filtered[i] = (filtered[i] ?? 0) + alpha * (residual - (filtered[i] ?? 0));
          if (k > 400) local[i] = Math.max(local[i] ?? 0, Math.abs(filtered[i] ?? 0));
        });
      }
      const target = mode === "joint-level" ? envelope : centralEnvelope;
      local.forEach((v, i) => (target[i] = Math.max(target[i] ?? 0, v)));
      console.log(
        `seed ${seed} ${mode.padEnd(11)} ${name.padEnd(15)} ${local.map((v) => v.toFixed(1).padStart(6)).join("")}`,
      );
      system.dispose();
    }
  }
}
const thresholds = spec.joints.map((joint, i) =>
  Math.max(MARGIN * (envelope[i] ?? 0), MIN_SHARE * joint.actuator.ratedTorque),
);
console.log("\nenvelope (joint-level)", envelope.map((v) => v.toFixed(1)).join(", "));
console.log("envelope (central)    ", centralEnvelope.map((v) => v.toFixed(1)).join(", "));
console.log("thresholds ", thresholds.map((v) => v.toFixed(0)).join(", "));
