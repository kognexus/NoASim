/**
 * Impact scenarios behind H3 and H4: drive the arm into a spring-mounted block and watch
 * how each joint reacts, or run normal tasks and count false alarms.
 */
import { StatusFlags } from "../bus/protocol.ts";
import type { ArmSystem } from "../harness/armSystem.ts";
import type { InterpolationMode } from "../host/host.ts";
import type { Task } from "../host/tasks.ts";
import { BUS_CYCLE_HZ } from "../index.ts";
import type { Vec3 } from "../plant/armSpec.ts";
import type { ObstacleSpec } from "../plant/mjcf.ts";
import { homePose } from "./motion.ts";

/** Snapshot spacing and margin of the node's P1 ring buffer (see `supervisor.rs`). */
const SNAPSHOT_EVERY = 16;
const SNAPSHOT_MARGIN = 160;

/** A block 25 cm to the side of the KX-6 home pose, at tool height. */
export function sideObstacle(home: Vec3): ObstacleSpec {
  return {
    position: [home[0], home[1] + 0.2, home[2] - 0.02],
    halfSize: [0.05, 0.03, 0.1],
    axis: [0, 1, 0],
    stiffness: 2e4,
  };
}

/** A 1 s straight move of the tool 30 cm sideways (into {@link sideObstacle}). */
export function sideReach(system: ArmSystem): Task {
  const home = homePose(system);
  return {
    name: "side-reach",
    duration: 1.0,
    pose(t) {
      const s = 0.5 - 0.5 * Math.cos(Math.PI * Math.min(1, t / 1.0));
      return {
        position: [home.position[0], home.position[1] + 0.3 * s, home.position[2]],
        rotation: home.rotation,
      };
    },
  };
}

export interface ImpactResult {
  /** Bus cycle of the first impact report, or undefined if none. */
  readonly detectionCycle: number | undefined;
  /** Per joint: cycles from detection to the first state frame showing the reaction. */
  readonly reactionCycles: (number | undefined)[];
  /** Largest force on the block after detection, N. */
  readonly peakForceAfterDetection: number;
  /** Mean force on the block over the half second after detection, N. */
  readonly meanForceAfterDetection: number;
  /** Bus cycle the arm first touched the block (force above 1 N). */
  readonly contactCycle: number | undefined;
  /** Force on the block at the end of the run, N. */
  readonly finalForce: number;
  /** Tool position at the snapshot the joints retreat to, m. */
  readonly snapshotTool: Vec3 | undefined;
  /** Tool position at the end of the run, m. */
  readonly finalTool: Vec3;
  /** Per joint: cycle of the first state frame showing HOLDING. */
  readonly holdingCycles: (number | undefined)[];
}

const REACTING = StatusFlags.stopping | StatusFlags.holding | StatusFlags.torqueOff;

/** Runs `task` with impact reaction armed and records the reaction. */
export function impactRun(
  system: ArmSystem,
  task: Task,
  options: { mode?: InterpolationMode; seconds?: number } = {},
): ImpactResult {
  const tool: Vec3[] = [];
  const start = system.cycle;
  system.host.startTask(task, start, { mode: options.mode ?? "joint-level" });
  const reaction: (number | undefined)[] = system.joints.map(() => undefined);
  const holding: (number | undefined)[] = system.joints.map(() => undefined);
  let detection: number | undefined;
  let contact: number | undefined;
  let peak = 0;
  let forceSum = 0;
  let forceSamples = 0;
  for (let k = 0; k < Math.round((options.seconds ?? 3) * BUS_CYCLE_HZ); k++) {
    tool.push(system.plant.tcpPosition());
    system.step();
    if (detection === undefined && system.host.impacts.length > 0)
      detection = system.host.impacts[0]?.cycle;
    const force = system.plant.obstacleForce();
    if (contact === undefined && force > 1) contact = system.cycle;
    if (detection !== undefined && system.cycle > detection) {
      peak = Math.max(peak, force);
      if (system.cycle <= detection + 0.5 * BUS_CYCLE_HZ) {
        forceSum += force;
        forceSamples++;
      }
    }
    for (const f of system.host.lastReceived) {
      if (!("state" in f)) continue;
      const i = f.header.source - 1;
      if (
        reaction[i] === undefined &&
        (f.state.status & REACTING) !== 0 &&
        detection !== undefined
      ) {
        reaction[i] = f.header.cycle - detection;
      }
      if (holding[i] === undefined && (f.state.status & StatusFlags.holding) !== 0)
        holding[i] = f.header.cycle;
    }
  }
  const snapshotCycle =
    detection === undefined
      ? undefined
      : Math.floor((detection - SNAPSHOT_MARGIN) / SNAPSHOT_EVERY) * SNAPSHOT_EVERY;
  return {
    detectionCycle: detection,
    reactionCycles: reaction,
    peakForceAfterDetection: peak,
    meanForceAfterDetection: forceSamples > 0 ? forceSum / forceSamples : 0,
    contactCycle: contact,
    finalForce: system.plant.obstacleForce(),
    snapshotTool: snapshotCycle === undefined ? undefined : tool[snapshotCycle - start],
    finalTool: system.plant.tcpPosition(),
    holdingCycles: holding,
  };
}
