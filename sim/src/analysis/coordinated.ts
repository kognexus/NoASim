/**
 * Coordinated damping from what the joints have (NX-29): each joint combines the bus
 * states of the joints up to `reach` places away, no plant state.
 *
 * Every joint's state frame carries its motor angle and speed, its gear twist and rate,
 * and its housing gyro. No joint measures link bending. But the gyro integrated over time
 * is the housing's orientation, which near the pose the gain was designed at is a linear
 * function of every angle up the chain, bending included; so the joints' integrated gyros
 * and angles together carry the bending. Per joint j, ten measurements:
 *
 * ```text
 * y_j = [motor angle, twist, motor speed, twist rate, gyro (3), integrated gyro (3)]
 * ```
 *
 * all as deviations from where the arm held when the damper was switched on (the gyro's
 * bias at that moment is taken out before integrating, and the integral leaks at
 * `LEAK_HZ` so it cannot drift away).
 *
 * The gain `τ = −F·y` (joint i only uses y_j with |j − i| ≤ reach) is designed on the
 * linearised arm: the structured state gain (`designStructured`) is fitted row by row
 * onto the measurements, then refined through `K = F·C` with the measured sensor noise
 * in the cost.
 *
 * Status (NX-29): an experiment, not in the node. Designed at the home pose it beats
 * network damping there by a third; at other postures, or designed for two at once, it
 * oscillates. The model it is designed on treats the measurements as clean, delayed
 * states; the joints' observers, the gyros' integration and the position loops'
 * actuation delay are not in it. An estimator-based design that models them is the next
 * step.
 */
import type { ArmSystem } from "../harness/armSystem.ts";
import type { NodeConfig } from "../node/nodeConfig.ts";
import { Matrix } from "../util/matrix.ts";
import { designStructured, type StructuredDesign } from "./structured.ts";

/** Measurements per joint. */
export const PER_JOINT = 10;
/**
 * The measurements the gain may use: angles, twists, gyros and integrated gyros. The
 * speeds from the joints' observers are left out. The twist rate (output minus motor
 * speed) is off by 100–200 % on the bus; the motor speed lags 7–8 cycles, and a gain
 * using it settled into a 40–90 Hz oscillation. The gyros carry the same motion with
 * little lag.
 */
export const USED: ReadonlySet<number> = new Set([0, 1, 4, 5, 6, 7, 8, 9]);
/** Corner of the leak of the integrated gyro, Hz. */
const LEAK_HZ = 0.5;
/** Cycles averaged for the rest angles and gyro biases before the damper acts (0.2 s). */
export const CALIBRATION = 800;

/**
 * Noise of each measurement on the bus at rest (standard deviation, per channel of a
 * joint), measured on the light arm: encoder quantisation and noise on the angles, the
 * observers' speed estimates, the gyros; the integrated gyro's from its leak.
 */
export const MEASUREMENT_NOISE = [
  3.3e-5, 3.5e-5, 9.1e-3, 9.6e-3, 4e-3, 4e-3, 4e-3, 2.5e-5, 2.5e-5, 2.5e-5,
];

/** Cycles of sensor noise counted against one tap's cost (one second at 4 kHz). */
const NOISE_WEIGHT = 4000;

/** `y = C·x` near the current pose (C: joints·PER_JOINT rows, the plant's state columns). */
export function measurementModel(system: ArmSystem, joints: number): Matrix {
  const plant = system.plant;
  const x0 = plant.state();
  const n = x0.length;
  const nq = n / 2;
  const c = new Matrix(joints * PER_JOINT, n);
  for (let j = 0; j < joints; j++) {
    const { motor, gear } = plant.coordinates(j);
    const row = j * PER_JOINT;
    c.set(row, motor, 1);
    c.set(row + 1, gear, 1);
    c.set(row + 2, nq + motor, 1);
    c.set(row + 3, nq + gear, 1);
  }
  // Gyros are linear in the speeds; their integrals in the angles, with the same map.
  for (let k = 0; k < nq; k++) {
    const gyros = (speed: number) => {
      const x = Float64Array.from(x0);
      x[nq + k] = (x[nq + k] ?? 0) + speed;
      plant.setState(x);
      return Array.from({ length: joints }, (_, j) => plant.gyro(j));
    };
    const plus = gyros(1);
    const minus = gyros(-1);
    for (let j = 0; j < joints; j++) {
      for (let axis = 0; axis < 3; axis++) {
        const g = ((plus[j]?.[axis] ?? 0) - (minus[j]?.[axis] ?? 0)) / 2;
        c.set(j * PER_JOINT + 4 + axis, nq + k, g);
        c.set(j * PER_JOINT + 7 + axis, k, g);
      }
    }
  }
  plant.setState(x0);
  return c;
}

/** Least squares `f` with `f·C_rows ≈ k`, rows restricted to `columns` of y. */
function fitRow(k: Float64Array, c: Matrix, columns: number[]): Float64Array {
  const m = columns.length;
  // Normal equations (C_s C_sᵀ + ε) fᵀ = C_s kᵀ.
  const g = new Matrix(m, m);
  const rhs = new Matrix(m, 1);
  columns.forEach((a, ia) => {
    let s = 0;
    for (let x = 0; x < c.cols; x++) s += c.get(a, x) * (k[x] ?? 0);
    rhs.set(ia, 0, s);
    columns.forEach((b, ib) => {
      let t = 0;
      for (let x = 0; x < c.cols; x++) t += c.get(a, x) * c.get(b, x);
      g.set(ia, ib, t);
    });
  });
  let trace = 0;
  for (let i = 0; i < m; i++) trace += g.get(i, i);
  for (let i = 0; i < m; i++) g.set(i, i, g.get(i, i) + 1e-9 * (trace / m));
  const f = g.inverse().times(rhs);
  const row = new Float64Array(c.rows);
  columns.forEach((a, ia) => (row[a] = f.get(ia, 0)));
  return row;
}

export interface CoordinatedResult {
  /** Rows: joints; columns: joints·PER_JOINT measurements (zero outside the reach). */
  readonly gain: Matrix;
  /**
   * Per posture designed for, relative to no gain: the structured state gain's cost, and
   * the measured gain's (taps plus sensor noise).
   */
  readonly cost: { state: number; measured: number }[];
}

/**
 * Designs one F for the arm holding at each of `systems` (the same arm in different
 * postures, damping off): the sum of their costs, each relative to its cost with no gain.
 * One gain for several postures cannot be tuned to one posture's model and fail at
 * another, and the joints need no posture-dependent gains.
 */
export function designCoordinated(
  systems: readonly ArmSystem[],
  configs: readonly NodeConfig[],
  design: StructuredDesign,
  log?: (line: string) => void,
): CoordinatedResult {
  const joints = configs.length;
  const models = systems.map((system) => ({
    structured: designStructured(system, configs, design, log),
    c: measurementModel(system, joints),
  }));
  const first = models[0];
  if (!first) throw new Error("no posture to design for");
  const rows = joints * PER_JOINT;
  const columns = (i: number) =>
    Array.from({ length: rows }, (_, a) => a).filter(
      (a) => Math.abs(Math.floor(a / PER_JOINT) - i) <= design.reach && USED.has(a % PER_JOINT),
    );
  // Start: the first posture's structured state gain, fitted row by row.
  let f = new Matrix(joints, rows);
  for (let i = 0; i < joints; i++) {
    const k = new Float64Array(first.c.cols);
    for (let x = 0; x < first.c.cols; x++) k[x] = first.structured.gain.get(i, x);
    const row = fitRow(k, first.c, columns(i));
    for (let a = 0; a < rows; a++) f.set(i, a, row[a] ?? 0);
  }
  // Measurement noise N (diagonal), as torque noise F·N·Fᵀ.
  const noise = new Matrix(rows, rows);
  for (let a = 0; a < rows; a++) noise.set(a, a, (MEASUREMENT_NOISE[a % PER_JOINT] ?? 0) ** 2);
  const perPosture = (gain: Matrix) =>
    models.map(({ structured, c }) => {
      const tap = structured.costOf(gain.times(c));
      if (!Number.isFinite(tap)) return Infinity;
      const torqueNoise = gain.times(noise).times(gain.transpose());
      const total = tap + NOISE_WEIGHT * structured.noiseCost(gain.times(c), torqueNoise).cost;
      return total / structured.cost.none;
    });
  const total = (gain: Matrix) => perPosture(gain).reduce((a, b) => a + b, 0);
  let best = total(f);
  // A fit that does not stabilise every posture is scaled down until it does.
  for (let shrink = 0; shrink < 20 && !Number.isFinite(best); shrink++) {
    f = new Matrix(
      f.rows,
      f.cols,
      f.data.map((v) => 0.5 * v),
    );
    best = total(f);
  }
  log?.(
    `coordinated reach ${design.reach}: fitted, cost ${best.toFixed(4)} over ${models.length} postures`,
  );
  // Refine by gradient steps in F: through the loop, ∂J/∂F = ∂J/∂K · Cᵀ; and directly,
  // the noise's own cost, 2·(R + BᵀPB)·F·N; summed over postures.
  let step = 0;
  for (let it = 0; it < (design.iterations ?? 100) && Number.isFinite(best); it++) {
    let g = new Matrix(joints, rows);
    for (const { structured, c } of models) {
      const k = f.times(c);
      const n = structured.noiseCost(k, f.times(noise).times(f.transpose()));
      const loop = structured.gradientOf(k).plus(n.gradient, NOISE_WEIGHT).times(c.transpose());
      const curvature = structured.input.transpose().times(n.p).times(structured.input);
      configs.forEach((cfg, x) =>
        curvature.set(x, x, curvature.get(x, x) + design.torqueWeight / cfg.ratedTorque ** 2),
      );
      const posture = loop.plus(curvature.times(f).times(noise), 2 * NOISE_WEIGHT);
      g = g.plus(posture, 1 / structured.cost.none);
    }
    for (let i = 0; i < g.rows; i++) {
      const allowed = new Set(columns(i));
      for (let a = 0; a < g.cols; a++) if (!allowed.has(a)) g.set(i, a, 0);
    }
    const norm2 = g.data.reduce((sum, v) => sum + v * v, 0);
    if (norm2 === 0) break;
    if (step === 0) step = (0.01 * best) / norm2;
    let accepted = false;
    for (let tries = 0; tries < 30; tries++) {
      const candidate = f.plus(g, -step);
      const j = total(candidate);
      if (Number.isFinite(j) && j <= best - 0.1 * step * norm2) {
        f = candidate;
        best = j;
        step *= 1.5;
        accepted = true;
        break;
      }
      step *= 0.5;
    }
    if (!accepted) break;
  }
  log?.(`coordinated reach ${design.reach}: refined, cost ${best.toFixed(4)}`);
  const measured = perPosture(f);
  return {
    gain: f,
    cost: models.map(({ structured }, k) => ({
      state: structured.cost.structured / structured.cost.none,
      measured: measured[k] ?? Infinity,
    })),
  };
}

/** One joint's measurements from its state frame, as the others receive it. */
interface Frame {
  motorAngle: number;
  motorVelocity: number;
  outputVelocity: number;
  gearTwist: number;
  gyro: readonly number[];
}

/**
 * Runs `gain` on what arrives over the bus: every joint's latest state frame as the host
 * receives it (so a cycle or more late), with the joints' own sensor noise and bias.
 * Torque per joint is capped at `torqueShare` of peak and passes the joints' energy tank.
 */
export function attachCoordinated(
  system: ArmSystem,
  configs: readonly NodeConfig[],
  gain: Matrix,
  torqueShare: number,
  tankJ: number,
): void {
  const joints = configs.length;
  const latest = new Map<number, Frame>();
  const read = () => {
    for (const f of system.host.lastReceived)
      if ("state" in f) latest.set(f.header.source, f.state);
  };
  // Where the arm holds, and each gyro's bias, averaged over the first CALIBRATION cycles
  // with no torque: a single sample's noise, integrated, would hold the arm off by itself.
  const rest = Array.from({ length: joints }, () => ({
    motorAngle: 0,
    gearTwist: 0,
    gyro: [0, 0, 0],
  }));
  let samples = 0;
  const angle = Array.from({ length: joints }, () => [0, 0, 0]);
  const tank = configs.map(() => 0);
  const dt = system.plant.timestep;
  const leak = 2 * Math.PI * LEAK_HZ * dt;
  system.outsideTorque = (s) => {
    read();
    if (samples < CALIBRATION) {
      samples++;
      for (let j = 0; j < joints; j++) {
        const f = latest.get(j + 1);
        const r = rest[j];
        if (!f || !r) continue;
        r.motorAngle += (f.motorAngle - r.motorAngle) / samples;
        r.gearTwist += (f.gearTwist - r.gearTwist) / samples;
        r.gyro = r.gyro.map((g, axis) => g + ((f.gyro[axis] ?? 0) - g) / samples);
      }
      return configs.map(() => 0);
    }
    const y = new Float64Array(joints * PER_JOINT);
    for (let j = 0; j < joints; j++) {
      const f = latest.get(j + 1);
      const r = rest[j];
      if (!f || !r) continue;
      const row = j * PER_JOINT;
      y[row] = f.motorAngle - r.motorAngle;
      y[row + 1] = f.gearTwist - r.gearTwist;
      y[row + 2] = f.motorVelocity;
      y[row + 3] = f.outputVelocity - f.motorVelocity;
      for (let axis = 0; axis < 3; axis++) {
        const rate = (f.gyro[axis] ?? 0) - (r.gyro[axis] ?? 0);
        const integral = angle[j];
        if (integral) integral[axis] = (integral[axis] ?? 0) * (1 - leak) + rate * dt;
        y[row + 4 + axis] = rate;
        y[row + 7 + axis] = integral?.[axis] ?? 0;
      }
    }
    return configs.map((c, i) => {
      let u = 0;
      for (let a = 0; a < y.length; a++) u -= gain.get(i, a) * (y[a] ?? 0);
      const cap = torqueShare * c.peakTorque;
      u = Math.max(-cap, Math.min(cap, u));
      const work = u * s.plant.outputVelocity(i) * dt;
      const energy = tank[i] ?? 0;
      if (work <= 0) tank[i] = Math.min(tankJ, energy - work);
      else if (work <= energy) tank[i] = energy - work;
      else u = 0;
      return u;
    });
  };
}
