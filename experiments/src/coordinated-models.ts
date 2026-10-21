/**
 * NX-31: the coordinated models in the joints, against today's joints, on the three things
 * the owner asked about: settling after a tap, tracking under both robot computers, and
 * full-body stabilisation (the tool's offset while holding at three postures, a sideways
 * shake near the first mode, a push). Both arms, datasheet actuators, network damping on.
 *
 *   node experiments/src/coordinated-models.ts [--quick]
 *
 * Variants:
 * - today's joints (no coordinated model);
 * - own model only (reach 0): each joint's own inertia, learned from the same data;
 * - coordinated model (reach 2): its row of the neighbourhood's inertia and its link's
 *   compliance (sag), from the joints up to two places away;
 * - the same without the sag compensation, to tell the two parts apart;
 * - the coordinated model with NX-30's coordinated damping instead of network damping:
 *   everything the network does here.
 *
 * Writes experiments/results/coordinated-models.json. `npm run prove` calls
 * {@link measureCoordinatedModels}.
 */
import { mkdir, readFile, writeFile } from "node:fs/promises";
import { VibrationMeter } from "../../sim/src/analysis/vibration.ts";
import { coordinatedBlocks, designLqg, loopModel } from "../../sim/src/analysis/estimator.ts";
import { DampingMode } from "../../sim/src/bus/protocol.ts";
import type { ArmSystem, ArmSystemOptions } from "../../sim/src/harness/armSystem.ts";
import { nodeConfigsFor } from "../../sim/src/harness/gains.ts";
import { tunedGains } from "../../sim/src/harness/tuned.ts";
import { figureEight, pickAndPlace, type Task } from "../../sim/src/host/tasks.ts";
import type { TaskOptions } from "../../sim/src/host/host.ts";
import { BUS_CYCLE_HZ } from "../../sim/src/index.ts";
import type { CoordinatedModelDocument } from "../../sim/src/node/coordinatedModel.ts";
import type { Pose } from "../../sim/src/host/ik.ts";
import type { Vec3 } from "../../sim/src/plant/armSpec.ts";
import { KX6, KX6_HOME, KX6L, withInitialAngles } from "../../sim/src/plant/kx6.ts";
import { homePose } from "../../sim/src/scenarios/motion.ts";
import { tapTest } from "../../sim/src/scenarios/tap.ts";
import { POSTURES } from "./estimation.ts";
import { BAND } from "./headroom.ts";
import { type Lab, openLab, withArm } from "./runner.ts";

export const VARIANTS = [
  "today's joints",
  "own model only",
  "coordinated model",
  "coordinated model, no sag compensation",
  "coordinated model and coordinated damping",
] as const;
export type Variant = (typeof VARIANTS)[number];

const ARMS = [
  ["KX-6L", KX6L, "kx6l"],
  ["KX-6", KX6, "kx6"],
] as const;
const COMPUTERS: Record<string, TaskOptions> = {
  VLA: { mode: "joint-level" },
  classic: { mode: "classic", classic: { loopHz: 1000 } },
};
const TASKS: Record<string, (home: Pose) => Task> = {
  "figure of eight": (home) => figureEight(home),
  "pick and place": (home) => pickAndPlace(home),
};
/** Sideways shake at the tool near each arm's first mode (as the demo's), Hz and N. */
const SHAKE = { kx6l: 6, kx6: 9 } as const;
const SHAKE_N = 15;
const PUSH_N = 150;
const PUSH_SECONDS = 0.4;
/** As in `experiments/src/estimation.ts`. */
const DAMPING_DESIGN = {
  kx6l: { torqueWeight: 3e-5, speedWeight: 1e-4, forceNoise: 20 },
  kx6: { torqueWeight: 3e-6, speedWeight: 1e-4, forceNoise: 20 },
} as const;

export interface TrackScore {
  /** Tool path error RMS, mm. */
  readonly mm: number;
  /** Mean over joints 2–6 of the vibration each feels, °/s RMS above 3 Hz. */
  readonly vibration: number;
}
export interface ArmScores {
  readonly tracking: Record<string, TrackScore>;
  /** Tool offset from the commanded pose while holding, per posture, mm. */
  readonly hold: Record<string, number>;
  /** After a tap: mean distance over the first second (mm), time to stay within 0.5 mm. */
  readonly tap: { readonly meanMm: number; readonly settleS: number };
  /** Shake: mean tool distance from rest over 4 s (mm), joints' vibration (°/s). */
  readonly shake: TrackScore;
  /** Push: peak and mean tool distance from rest over 2 s, mm. */
  readonly push: { readonly peakMm: number; readonly meanMm: number };
}
export type CoordinatedModelsResult = Record<string, Record<Variant, ArmScores>>;

async function models(dir: string): Promise<CoordinatedModelDocument[]> {
  return Promise.all(
    [1, 2, 3, 4, 5, 6].map(async (j) => {
      const url = new URL(`../../models/coordinated-models/${dir}/joint${j}.json`, import.meta.url);
      return JSON.parse(await readFile(url, "utf8")) as CoordinatedModelDocument;
    }),
  );
}

const distance = (p: Vec3, q: Vec3) => Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
const outerMean = (levels: number[]) => {
  const outer = levels.slice(1);
  return outer.reduce((a, b) => a + b, 0) / Math.max(1, outer.length);
};

/** Runs `task` from rest; path error RMS and the vibration the joints feel. */
function track(system: ArmSystem, task: Task, options: TaskOptions): TrackScore {
  const meter = new VibrationMeter(system.host, system.joints.length);
  const start = system.cycle;
  system.host.startTask(task, start, options);
  const sums = new Array<number>(system.joints.length).fill(0);
  let error = 0;
  let n = 0;
  for (let k = 0; k < Math.round(task.duration * BUS_CYCLE_HZ); k++) {
    system.step();
    meter.observe();
    if (k > BUS_CYCLE_HZ && k % 40 === 0) {
      meter.levels().forEach((v, j) => (sums[j] = (sums[j] ?? 0) + v * v));
      const want = task.pose((system.cycle - start) / BUS_CYCLE_HZ).position;
      error += distance(system.plant.tcpPosition(), want) ** 2;
      n++;
    }
  }
  return {
    mm: 1e3 * Math.sqrt(error / Math.max(1, n)),
    vibration: outerMean(sums.map((s) => Math.sqrt(s / Math.max(1, n)))),
  };
}

/** A force at the tool, `force(k)` for `cycles` cycles; distances from rest over `cycles`. */
function disturb(
  system: ArmSystem,
  cycles: number,
  force: (k: number) => Vec3,
): { meanMm: number; peakMm: number; vibration: number } {
  const meter = new VibrationMeter(system.host, system.joints.length);
  const rest = system.plant.tcpPosition();
  const sums = new Array<number>(system.joints.length).fill(0);
  let sum = 0;
  let peak = 0;
  let n = 0;
  for (let k = 0; k < cycles; k++) {
    system.plant.setToolForce(force(k));
    system.step();
    meter.observe();
    const d = distance(system.plant.tcpPosition(), rest);
    sum += d;
    peak = Math.max(peak, d);
    if (k % 40 === 0) {
      meter.levels().forEach((v, j) => (sums[j] = (sums[j] ?? 0) + v * v));
      n++;
    }
  }
  system.plant.setToolForce([0, 0, 0]);
  return {
    meanMm: (1e3 * sum) / cycles,
    peakMm: 1e3 * peak,
    vibration: outerMean(sums.map((s) => Math.sqrt(s / Math.max(1, n)))),
  };
}

export async function measureCoordinatedModels(
  lab: Lab,
  quick = false,
  log: (line: string) => void = console.log,
): Promise<CoordinatedModelsResult> {
  const seeds = quick ? [32] : [32, 33];
  const out: CoordinatedModelsResult = {};
  for (const [arm, base, key] of ARMS) {
    const home = withInitialAngles(base, KX6_HOME);
    const gains = tunedGains(key, "network");
    const [own, reach2] = await Promise.all([models(`${key}-reach0`), models(`${key}-reach2`)]);
    const noSag = reach2.map((d) => ({ ...d, compliance: 0 }));
    // NX-30's coordinated damping, designed at home as in its experiment.
    const offGains = tunedGains(key, "off");
    const configs = nodeConfigsFor(home, BUS_CYCLE_HZ, offGains);
    const blocks = await withArm(lab, { spec: home, gains: offGains, seed: 31 }, (s) => {
      s.host.applySettings({ damping: DampingMode.off });
      s.run(5.3);
      const m = loopModel(s, configs);
      return coordinatedBlocks(
        m,
        designLqg(m, configs, DAMPING_DESIGN[key], 2),
        configs,
        2,
        1 / BUS_CYCLE_HZ,
      );
    });
    const setups: Record<Variant, { docs?: CoordinatedModelDocument[]; damping: DampingMode }> = {
      "today's joints": { damping: DampingMode.network },
      "own model only": { docs: own, damping: DampingMode.network },
      "coordinated model": { docs: reach2, damping: DampingMode.network },
      "coordinated model, no sag compensation": { docs: noSag, damping: DampingMode.network },
      "coordinated model and coordinated damping": {
        docs: reach2,
        damping: DampingMode.coordinated,
      },
    };
    const byVariant = {} as Record<Variant, ArmScores>;
    for (const variant of VARIANTS) {
      const setup = setups[variant];
      const options = (seed: number, angles = KX6_HOME): ArmSystemOptions => ({
        spec: withInitialAngles(base, angles),
        gains,
        seed,
        ...(setup.docs ? { coordinatedModels: setup.docs } : {}),
        ...(setup.damping === DampingMode.coordinated ? { coordinated: blocks } : {}),
      });
      const settle = (s: ArmSystem) => {
        s.run(3);
        s.host.applySettings({ damping: setup.damping, coordinatedModel: setup.docs ? 1 : 0 });
        s.run(2);
      };
      const mean = (xs: number[]) => xs.reduce((a, b) => a + b, 0) / xs.length;
      const tracking: Record<string, TrackScore> = {};
      for (const [computer, taskOptions] of Object.entries(COMPUTERS)) {
        for (const [name, make] of Object.entries(TASKS)) {
          const runs: TrackScore[] = [];
          for (const seed of seeds) {
            runs.push(
              await withArm(lab, options(seed), (s) => {
                settle(s);
                return track(s, make(homePose(s)), taskOptions);
              }),
            );
          }
          tracking[`${computer}, ${name}`] = {
            mm: mean(runs.map((r) => r.mm)),
            vibration: mean(runs.map((r) => r.vibration)),
          };
        }
      }
      const hold: Record<string, number> = {};
      for (const [posture, angles] of Object.entries(POSTURES)) {
        hold[posture] = await withArm(lab, options(seeds[0] ?? 32, angles), (s) => {
          settle(s);
          return 1e3 * distance(s.plant.tcpPosition(), homePose(s).position);
        });
      }
      const taps: { meanMm: number; settleS: number }[] = [];
      for (const direction of [
        [0, 0, 1],
        [0, 1, 0],
      ] as Vec3[]) {
        for (const seed of seeds) {
          taps.push(
            await withArm(lab, options(seed), (s) => {
              const r = tapTest(s, { direction, recordSeconds: 2, beforeTap: settle });
              let sum = 0;
              for (let k = 0; k < BUS_CYCLE_HZ; k++) sum += r.deviation[k] ?? 0;
              let last = 0;
              r.deviation.forEach((d, k) => {
                if (d > BAND) last = k;
              });
              return { meanMm: (1e3 * sum) / BUS_CYCLE_HZ, settleS: last / BUS_CYCLE_HZ };
            }),
          );
        }
      }
      const shakes = [];
      const pushes = [];
      for (const seed of seeds) {
        shakes.push(
          await withArm(lab, options(seed), (s) => {
            settle(s);
            const w = (2 * Math.PI * SHAKE[key]) / BUS_CYCLE_HZ;
            return disturb(s, 4 * BUS_CYCLE_HZ, (k) =>
              k < 3 * BUS_CYCLE_HZ ? [0, SHAKE_N * Math.sin(w * k), 0] : [0, 0, 0],
            );
          }),
        );
        pushes.push(
          await withArm(lab, options(seed), (s) => {
            settle(s);
            return disturb(s, 2 * BUS_CYCLE_HZ, (k) =>
              k < PUSH_SECONDS * BUS_CYCLE_HZ ? [0, PUSH_N, 0] : [0, 0, 0],
            );
          }),
        );
      }
      const scores: ArmScores = {
        tracking,
        hold,
        tap: { meanMm: mean(taps.map((t) => t.meanMm)), settleS: mean(taps.map((t) => t.settleS)) },
        shake: {
          mm: mean(shakes.map((s) => s.meanMm)),
          vibration: mean(shakes.map((s) => s.vibration)),
        },
        push: {
          peakMm: mean(pushes.map((p) => p.peakMm)),
          meanMm: mean(pushes.map((p) => p.meanMm)),
        },
      };
      byVariant[variant] = scores;
      log(
        `coordinated models ${arm} ${variant.padEnd(42)} ` +
          `${Object.entries(tracking)
            .map(([k, v]) => `${k} ${v.mm.toFixed(3)} mm`)
            .join(", ")}; hold ${Object.values(hold)
            .map((h) => h.toFixed(3))
            .join(
              "/",
            )} mm; tap ${scores.tap.meanMm.toFixed(3)} mm, ${scores.tap.settleS.toFixed(3)} s; ` +
          `shake ${scores.shake.mm.toFixed(3)} mm; push peak ${scores.push.peakMm.toFixed(1)} mm`,
      );
    }
    out[arm] = byVariant;
  }
  return out;
}

if (import.meta.main) {
  const result = await measureCoordinatedModels(await openLab(), process.argv.includes("--quick"));
  await mkdir(new URL("../results/", import.meta.url), { recursive: true });
  await writeFile(
    new URL("../results/coordinated-models.json", import.meta.url),
    `${JSON.stringify(result, null, 2)}\n`,
  );
}
