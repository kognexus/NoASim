/**
 * Scripted tasks the policy emulator follows: where the tool should be at time `t`.
 */
import type { Vec3 } from "../plant/armSpec.ts";
import type { Pose } from "./ik.ts";

/** A tool trajectory. */
export interface Task {
  readonly name: string;
  readonly duration: number;
  /** Desired tool pose at time `t` (s) after the task starts. */
  pose(t: number): Pose;
}

const offset = (p: Vec3, d: Vec3): Vec3 => [p[0] + d[0], p[1] + d[1], p[2] + d[2]];
const lerp = (a: Vec3, b: Vec3, s: number): Vec3 => [
  a[0] + s * (b[0] - a[0]),
  a[1] + s * (b[1] - a[1]),
  a[2] + s * (b[2] - a[2]),
];
/** Smooth 0 → 1 over [0, 1] (cosine). */
const blend = (s: number): number => 0.5 - 0.5 * Math.cos(Math.PI * Math.min(1, Math.max(0, s)));

/** Moves through `points` (offsets from home), `move` s per leg then `pause` s still. */
function waypointTask(
  name: string,
  home: Pose,
  points: readonly Vec3[],
  move: number,
  pause: number,
): Task {
  const waypoints = points.map((d) => offset(home.position, d));
  const leg = move + pause;
  const legs = waypoints.length - 1;
  return {
    name,
    duration: leg * legs,
    pose(t) {
      const i = Math.min(legs - 1, Math.max(0, Math.floor(t / leg)));
      const a = waypoints[i] ?? home.position;
      const b = waypoints[i + 1] ?? a;
      return { position: lerp(a, b, blend((t - i * leg) / move)), rotation: home.rotation };
    },
  };
}

/** A figure of eight in the y–z plane around `home`, `size` metres wide. */
export function figureEight(home: Pose, size = 0.15, period = 4, cycles = 2): Task {
  return {
    name: "figure-eight",
    duration: period * cycles,
    pose(t) {
      const w = (2 * Math.PI * t) / period;
      // Ramp the amplitude in over the first half period so the start is smooth.
      const ramp = blend(t / (period / 2));
      return {
        position: offset(home.position, [
          0,
          ramp * size * Math.sin(w),
          ramp * (size / 2) * Math.sin(2 * w),
        ]),
        rotation: home.rotation,
      };
    },
  };
}

/** Pick and place: down, up, across, down, back home, with pauses. */
export function pickAndPlace(home: Pose, moveSeconds = 1.2, pauseSeconds = 0.4): Task {
  return waypointTask(
    "pick-and-place",
    home,
    [
      [0, 0, 0],
      [0.1, 0.2, -0.1],
      [0.1, 0.2, 0],
      [-0.1, -0.2, 0],
      [-0.1, -0.2, -0.1],
      [0, 0, 0],
    ],
    moveSeconds,
    pauseSeconds,
  );
}

/** Fast reach: targets 0.25 m apart, each reached in `moveSeconds`. */
export function fastReach(home: Pose, moveSeconds = 0.5): Task {
  return waypointTask(
    "fast-reach",
    home,
    [
      [0, 0, 0],
      [0, 0.25, 0],
      [0, -0.25, 0],
      [0, 0, 0],
    ],
    moveSeconds,
    0.5,
  );
}
