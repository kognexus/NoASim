/**
 * Damped-least-squares inverse kinematics on the nominal (datasheet) kinematics, as the
 * robot's main computer would run it after every policy step.
 */
import type { ArmSpec, Vec3 } from "../plant/armSpec.ts";
import { forwardKinematics, type Mat3 } from "../plant/kinematics.ts";

/** A tool pose: flange position and orientation, world frame. */
export interface Pose {
  readonly position: Vec3;
  readonly rotation: Mat3;
}

export function flangePose(spec: ArmSpec, angles: readonly number[]): Pose {
  const fk = forwardKinematics(spec, angles);
  return { position: fk.flange, rotation: fk.flangeRotation };
}

const column = (m: Mat3, c: 0 | 1 | 2): Vec3 => [m[c], m[3 + c] ?? 0, m[6 + c] ?? 0];
const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];

/** 6-vector error (position, then rotation as a small-angle vector) from `pose` to `target`. */
export function poseError(pose: Pose, target: Pose): number[] {
  const r: [number, number, number] = [0, 0, 0];
  for (const c of [0, 1, 2] as const) {
    const v = cross(column(pose.rotation, c), column(target.rotation, c));
    r[0] += 0.5 * v[0];
    r[1] += 0.5 * v[1];
    r[2] += 0.5 * v[2];
  }
  const p = pose.position;
  const t = target.position;
  return [t[0] - p[0], t[1] - p[1], t[2] - p[2], ...r];
}

/** Dense row-major matrix helper. */
class Matrix {
  readonly rows: number;
  readonly cols: number;
  readonly data: Float64Array;
  constructor(rows: number, cols: number) {
    this.rows = rows;
    this.cols = cols;
    this.data = new Float64Array(rows * cols);
  }
  get(r: number, c: number): number {
    return this.data[r * this.cols + c] ?? 0;
  }
  set(r: number, c: number, v: number): void {
    this.data[r * this.cols + c] = v;
  }
}

/** Solves `(JᵀJ + λ²I) x = Jᵀe` by Gaussian elimination with partial pivoting. */
function dampedSolve(J: Matrix, e: readonly number[], lambda: number): number[] {
  const n = J.cols;
  const A = new Matrix(n, n);
  const b = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      let s = i === j ? lambda * lambda : 0;
      for (let k = 0; k < J.rows; k++) s += J.get(k, i) * J.get(k, j);
      A.set(i, j, s);
    }
    let s = 0;
    for (let k = 0; k < J.rows; k++) s += J.get(k, i) * (e[k] ?? 0);
    b[i] = s;
  }
  for (let p = 0; p < n; p++) {
    let best = p;
    for (let r = p + 1; r < n; r++) if (Math.abs(A.get(r, p)) > Math.abs(A.get(best, p))) best = r;
    for (let c = 0; c < n; c++) {
      const tmp = A.get(p, c);
      A.set(p, c, A.get(best, c));
      A.set(best, c, tmp);
    }
    [b[p], b[best]] = [b[best] ?? 0, b[p] ?? 0];
    for (let r = p + 1; r < n; r++) {
      const f = A.get(r, p) / A.get(p, p);
      for (let c = p; c < n; c++) A.set(r, c, A.get(r, c) - f * A.get(p, c));
      b[r] = (b[r] ?? 0) - f * (b[p] ?? 0);
    }
  }
  const x = new Array<number>(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = b[r] ?? 0;
    for (let c = r + 1; c < n; c++) s -= A.get(r, c) * (x[c] ?? 0);
    x[r] = s / A.get(r, r);
  }
  return x;
}

export interface IkOptions {
  readonly iterations?: number;
  /** Damping, keeps steps small near singularities. */
  readonly lambda?: number;
  /** Weight of orientation error relative to position (m per rad). */
  readonly orientationWeight?: number;
}

/** Joint angles that reach `target`, starting from `seed`. Counts as one IK solve. */
export function solveIk(
  spec: ArmSpec,
  target: Pose,
  seed: readonly number[],
  options: IkOptions = {},
): number[] {
  const { iterations = 4, lambda = 0.02, orientationWeight = 0.3 } = options;
  const weigh = (v: number, k: number) => (k < 3 ? v : orientationWeight * v);
  const q = [...seed];
  const h = 1e-6;
  for (let it = 0; it < iterations; it++) {
    const pose = flangePose(spec, q);
    const e = poseError(pose, target).map(weigh);
    const J = new Matrix(6, q.length);
    for (let j = 0; j < q.length; j++) {
      const qh = q.map((v, k) => (k === j ? v + h : v));
      poseError(pose, flangePose(spec, qh))
        .map(weigh)
        .forEach((d, k) => J.set(k, j, d / h));
    }
    dampedSolve(J, e, lambda).forEach((step, j) => (q[j] = (q[j] ?? 0) + step));
  }
  return q;
}
