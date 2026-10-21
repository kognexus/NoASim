/**
 * A damping gain designed for a structure (NX-29): joint i may use only the states of
 * the joints up to `reach` places away. Cutting the ideal LQR gain to that structure
 * (NX-28) throws away what the cut entries did; here the gain is optimised with the
 * structure in place.
 *
 * The cost is the LQR cost of the ideal controller (`designCost`), averaged over the
 * arm's response to a tap at the tool in two directions:
 *
 * ```text
 * J(K) = trace(P Σ₀),  P = A_Kᵀ P A_K + Q + Kᵀ R K,  A_K = A − B K
 * ∇J   = 2 ((R + BᵀPB) K − BᵀPA) Σ,  Σ = A_K Σ A_Kᵀ + Σ₀
 * ```
 *
 * (A the held arm linearised for one bus cycle, B the joints' torques, Σ₀ the taps'
 * initial states). It starts from the full LQR gain cut to the structure (NX-28), if that
 * stabilises the arm, else from no gain (the joints' own position loops hold the arm).
 * Plain gradient steps are badly conditioned here, so each step is the Gauss–Newton
 * (policy-iteration) step `(R + BᵀPB)⁻¹((R + BᵀPB)K − BᵀPA)` masked to the structure,
 * with a line search that keeps every gain stabilising and every step improving.
 */
import type { ArmSystem } from "../harness/armSystem.ts";
import type { NodeConfig } from "../node/nodeConfig.ts";
import type { Vec3 } from "../plant/armSpec.ts";
import { lqrGain, Matrix } from "../util/matrix.ts";
import { coordinateOwners, designCost, heldPlant } from "./ideal.ts";

const mask = (m: Matrix, allowed: (i: number, k: number) => boolean) => {
  for (let i = 0; i < m.rows; i++)
    for (let k = 0; k < m.cols; k++) if (!allowed(i, k)) m.set(i, k, 0);
  return m;
};

export interface StructuredDesign {
  readonly torqueWeight: number;
  readonly speedWeight: number;
  /** Joint i uses the states of joints i − reach … i + reach. */
  readonly reach: number;
  /** Gradient steps. */
  readonly iterations?: number;
  /**
   * Cycles between what the gain sees and when its torque acts, beyond the one the
   * model already has (bus latency, estimator lag). For a fixed gain a delay on the
   * measurements is the same loop as one on the torque, so the model carries a buffer of
   * past torques.
   */
  readonly delayCycles?: number;
}

export interface StructuredResult {
  /** Rows: joints; columns: the plant's state (zero outside the structure). */
  readonly gain: Matrix;
  /** Cost with no gain, with the optimised gain, and with the full LQR gain. */
  readonly cost: { none: number; structured: number; full: number };
  /** The cost of any gain on the same model. */
  readonly costOf: (gain: Matrix) => number;
  /** The cost's gradient at any stabilising gain, ∂J/∂K. */
  readonly gradientOf: (gain: Matrix) => Matrix;
  /**
   * Steady-state cost per cycle of torque noise of covariance `noise` (joints × joints)
   * at gain K, and its gradient in K through the closed loop (∂/∂K at fixed noise).
   */
  readonly noiseCost: (
    gain: Matrix,
    noise: Matrix,
  ) => { cost: number; gradient: Matrix; p: Matrix };
  /** Torque input of the model (padded state), for noise entering with the torque. */
  readonly input: Matrix;
}

/** Taps the cost averages over: unit impulses at the tool, world frame. */
const TAPS: Vec3[] = [
  [0, 0, 1],
  [0, 1, 0],
];

/**
 * Solution of `P = AᵀPA + M` for a stable A, by doubling; `Infinity` in every entry if A
 * is not stable.
 */
export function lyapunov(a: Matrix, m: Matrix): Matrix {
  let p = m;
  let ak = a;
  for (let it = 0; it < 60; it++) {
    p = p.plus(ak.transpose().times(p).times(ak));
    ak = ak.times(ak);
    const size = ak.norm1();
    if (!Number.isFinite(size) || size > 1e12) break;
    if (size < 1e-14) return p;
  }
  return new Matrix(m.rows, m.cols, new Float64Array(m.rows * m.cols).fill(Infinity));
}

/** trace(a·b); infinite when a is (an unstable gain's cost). */
const trace = (a: Matrix, b: Matrix) => {
  if (!Number.isFinite(a.get(0, 0))) return Infinity;
  let t = 0;
  for (let i = 0; i < a.rows; i++) for (let k = 0; k < a.cols; k++) t += a.get(i, k) * b.get(k, i);
  return t;
};

/** The states right after each tap: velocity changes from an impulse at the tool. */
function tapStates(system: ArmSystem): Matrix {
  const plant = system.plant;
  const x0 = plant.state();
  const n = x0.length;
  const nq = n / 2;
  const sigma = new Matrix(n, n);
  plant.setState(x0);
  const rest = plant.acceleration();
  for (const direction of TAPS) {
    plant.setToolForce(direction);
    plant.setState(x0);
    const pushed = plant.acceleration();
    plant.setToolForce([0, 0, 0]);
    const x = new Float64Array(n);
    for (let k = 0; k < nq; k++) x[nq + k] = (pushed[k] ?? 0) - (rest[k] ?? 0);
    for (let i = 0; i < n; i++)
      for (let j = 0; j < n; j++) sigma.set(i, j, sigma.get(i, j) + (x[i] ?? 0) * (x[j] ?? 0));
  }
  plant.setState(x0);
  // A little of every state, so Σ is not singular.
  const scale = (1e-6 * sigma.data.reduce((s, v) => s + Math.abs(v), 0)) / n;
  for (let i = 0; i < n; i++) sigma.set(i, i, sigma.get(i, i) + scale);
  return sigma;
}

export function designStructured(
  system: ArmSystem,
  configs: readonly NodeConfig[],
  design: StructuredDesign,
  log?: (line: string) => void,
): StructuredResult {
  const plant = heldPlant(system, configs);
  const { q: qx, r } = designCost(plant.tool, configs, design);
  const taps = tapStates(system);
  const nx = plant.held.rows;
  const nq = nx / 2;
  const m = configs.length;
  const d = design.delayCycles ?? 0;
  // The model with d cycles of torque delay: state [x, u_{k−1} … u_{k−d}].
  const n = nx + m * d;
  const a = new Matrix(n, n);
  const b = new Matrix(n, m);
  const q = new Matrix(n, n);
  const sigma0 = new Matrix(n, n);
  for (let i = 0; i < nx; i++) {
    for (let k = 0; k < nx; k++) {
      a.set(i, k, plant.held.get(i, k));
      q.set(i, k, qx.get(i, k));
      sigma0.set(i, k, taps.get(i, k));
    }
    for (let u = 0; u < m; u++) {
      if (d === 0) b.set(i, u, plant.b.get(i, u));
      else a.set(i, nx + m * (d - 1) + u, plant.b.get(i, u));
    }
  }
  for (let u = 0; u < m && d > 0; u++) {
    b.set(nx + u, u, 1);
    for (let slot = 1; slot < d; slot++) a.set(nx + m * slot + u, nx + m * (slot - 1) + u, 1);
  }
  for (let k = nx; k < n; k++) sigma0.set(k, k, sigma0.get(nx - 1, nx - 1));
  const owner = coordinateOwners(system, m);
  const allowed = (i: number, k: number) => {
    if (k >= nx) return false;
    const j = owner[k % nq] ?? -1;
    return j >= 0 && Math.abs(j - i) <= design.reach;
  };
  /** A gain on the plant's state, padded with zeros for the torque buffer. */
  const pad = (k: Matrix) => {
    if (k.cols === n) return k;
    const out = new Matrix(m, n);
    for (let i = 0; i < m; i++) for (let c = 0; c < k.cols; c++) out.set(i, c, k.get(i, c));
    return out;
  };
  const trim = (k: Matrix) => {
    const out = new Matrix(m, nx);
    for (let i = 0; i < m; i++) for (let c = 0; c < nx; c++) out.set(i, c, k.get(i, c));
    return out;
  };
  const cost = (k: Matrix) => {
    const ak = a.plus(b.times(k), -1);
    const p = lyapunov(ak, q.plus(k.transpose().times(r).times(k)));
    return { j: trace(p, sigma0), p, ak };
  };

  const none = cost(new Matrix(m, n)).j;
  const lqr = lqrGain(a, b, q, r);
  const full = cost(lqr).j;
  let gain = mask(lqr, allowed);
  let current = cost(gain);
  if (!Number.isFinite(current.j)) {
    gain = new Matrix(m, n);
    current = cost(gain);
  }
  log?.(`structured reach ${design.reach}: start at ${(current.j / none).toFixed(4)} of no gain`);
  let step = 1;
  for (let it = 0; it < (design.iterations ?? 100); it++) {
    const btp = b.transpose().times(current.p);
    const curvature = r.plus(btp.times(b));
    const direction = mask(
      curvature.inverse().times(curvature.times(gain).plus(btp.times(a), -1)),
      allowed,
    );
    // Line search: a stabilising gain with a lower cost.
    let accepted = false;
    for (let tries = 0; tries < 30; tries++) {
      const trial = cost(gain.plus(direction, -step));
      if (Number.isFinite(trial.j) && trial.j < current.j * (1 - 1e-6)) {
        gain = gain.plus(direction, -step);
        current = trial;
        step = Math.min(1, step * 2);
        accepted = true;
        break;
      }
      step *= 0.5;
    }
    if (!accepted) break;
    if (it % 10 === 0)
      log?.(
        `structured reach ${design.reach}: step ${it}, cost ${(current.j / none).toFixed(4)} of no gain`,
      );
  }
  const gradientOf = (k: Matrix) => {
    const kp = pad(k);
    const { p, ak } = cost(kp);
    const sigma = lyapunov(ak.transpose(), sigma0);
    const btp = b.transpose().times(p);
    const g = r.plus(btp.times(b)).times(kp).plus(btp.times(a), -1).times(sigma);
    g.data.forEach((v, i) => (g.data[i] = 2 * v));
    return trim(g);
  };
  const noiseCost = (k: Matrix, noise: Matrix) => {
    const kp = pad(k);
    const { p, ak } = cost(kp);
    // Torque noise w enters as B·w each cycle and costs wᵀRw itself.
    const w = b.times(noise).times(b.transpose());
    let j = trace(p, w);
    for (let i = 0; i < m; i++) for (let l = 0; l < m; l++) j += r.get(i, l) * noise.get(l, i);
    const sigma = lyapunov(ak.transpose(), w);
    const btp = b.transpose().times(p);
    const g = r.plus(btp.times(b)).times(kp).plus(btp.times(a), -1).times(sigma);
    g.data.forEach((v, i) => (g.data[i] = 2 * v));
    return { cost: j, gradient: trim(g), p };
  };
  return {
    gain: trim(gain),
    cost: { none, structured: current.j, full },
    costOf: (k: Matrix) => cost(pad(k)).j,
    gradientOf,
    noiseCost,
    input: b,
  };
}
