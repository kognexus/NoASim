/**
 * Estimator-based coordinated damping (NX-30): a Kalman filter on the joints' raw bus data
 * and a gain designed on the same model.
 *
 * **The model**, checked against the simulation before anything is designed on it:
 *
 * - the arm linearised where it holds, with the hinges' dry friction taken out of the
 *   linearisation (at rest it sticks, and MuJoCo's constraint then looks like an enormous
 *   damper) and put back as damping of the fast part of the motor speeds,
 *   `c_i = FRICTION_DAMPING · coulomb_i` on `v − LP(v)` (fitted once on a tap);
 * - the joints' position loops acting one cycle late, as `ArmSystem` applies them;
 * - the damper's command through a second-order low-pass at `COMMAND_LOWPASS_HZ`, whose
 *   output acts one cycle late as it would inside a joint. Without it the gain found
 *   the rotor–gear modes at a few hundred hertz, where the model is least accurate, and
 *   drove them (at a posture between home and reach, J2 at 24 N·m RMS and 300 Hz).
 *
 * State `z = [x, u_loops, LP(v_motor), f₁, f₂]` (f: the command filter's two stages).
 * Checked on the light arm, a vertical tap, mean tool distance over a second: 1.860 mm
 * against the simulation's 1.746 at home, 4.669 against 4.327 at reach, 6.147 against
 * 6.230 between them.
 *
 * **The measurements** are only what is raw in the joints' state frames: each joint's
 * motor encoder angle, its twist (output encoder minus motor), and its gyro (three axes),
 * arriving `MEASUREMENT_DELAY` cycles late. The joints' speed estimates are not used.
 *
 * **The filter** is the steady-state Kalman predictor for that model, run at the
 * measurements' time; the estimate is then carried forward over the delay with the
 * damper's own known commands. **The gain** is the LQR of NX-27's cost on the same model.
 */
import type { ArmSystem } from "../harness/armSystem.ts";
import type { NodeConfig } from "../node/nodeConfig.ts";
import { lqrGain, Matrix } from "../util/matrix.ts";
import { designCost, linearise } from "./ideal.ts";

/**
 * Dry friction as damping of the fast part of the motor speed (above `FRICTION_CORNER_HZ`),
 * per N·m of Coulomb friction, N·m·s/rad. The fast rotor–gear modes need it to be stable;
 * the slow modes, where the joints slip during a tap, must not get it. (Damping the whole
 * motor speed, `10·coulomb`, matched a tap at home but made the model unstable at another
 * posture.)
 */
export const FRICTION_DAMPING = 100;
/** Corner below which friction does not damp the motor speed in the model, Hz. */
export const FRICTION_CORNER_HZ = 20;
/** Corner of the low-pass on the damper's command, Hz: above the arm's modes, below the gears'. */
export const COMMAND_LOWPASS_HZ = 40;
/** Cycles between a joint's sensor sample and its use from the bus. */
export const MEASUREMENT_DELAY = 2;
/** Measurements per joint: motor angle, twist, gyro (3). */
export const MEASURED = 5;
/** Measurement noise, standard deviation: encoder angles (rad), gyro (rad/s). */
const ANGLE_NOISE = 3.5e-5;
const GYRO_NOISE = 4e-3;

export interface LoopModel {
  /** One bus cycle of `z = [x, u_loops, LP(v_motor), f₁, f₂]`. */
  readonly a: Matrix;
  /** The damper's command (it reaches the plant one cycle later). */
  readonly b: Matrix;
  /** A force at the tool, per N for one cycle. */
  readonly force: Matrix;
  /** Measurements `y = C·z`: per joint motor angle, twist, gyro (3). */
  readonly c: Matrix;
  /** Tool position against the plant's coordinates. */
  readonly tool: Matrix;
  /** Size of the plant's state x. */
  readonly nx: number;
}

/** The model of the arm holding where it is now, damping off. */
export function loopModel(system: ArmSystem, configs: readonly NodeConfig[]): LoopModel {
  const plant = system.plant;
  const lin = plant.withoutFriction(() => linearise(system));
  const nx = lin.a.rows;
  const nq = nx / 2;
  const m = configs.length;
  const n = nx + 4 * m;
  const slow = nx + m; // low-pass of each motor speed, for the friction term
  const f1 = nx + 2 * m;
  const f2 = nx + 3 * m;
  const alpha = 1 - Math.exp(-2 * Math.PI * COMMAND_LOWPASS_HZ * plant.timestep);
  const beta = 1 - Math.exp(-2 * Math.PI * FRICTION_CORNER_HZ * plant.timestep);
  // The joints' torque from the state, as their position loops compute it, plus friction
  // on the fast part of the motor speed: −c·(v − LP(v)).
  const loops = new Matrix(m, nx);
  configs.forEach((c, i) => {
    const { motor, gear } = plant.coordinates(i);
    loops.set(i, motor, -c.stiffness);
    loops.set(i, gear, -c.stiffness);
    loops.set(i, nq + motor, -c.damping - FRICTION_DAMPING * c.coulombFriction);
    loops.set(i, nq + gear, -c.damping + c.rotorDamping);
  });
  const a = new Matrix(n, n);
  for (let i = 0; i < nx; i++) {
    for (let k = 0; k < nx; k++) a.set(i, k, lin.a.get(i, k));
    for (let u = 0; u < m; u++) {
      a.set(i, nx + u, lin.b.get(i, u));
      a.set(i, f2 + u, lin.b.get(i, u));
    }
  }
  for (let u = 0; u < m; u++) {
    for (let k = 0; k < nx; k++) a.set(nx + u, k, loops.get(u, k));
    const motor = nq + plant.coordinates(u).motor;
    a.set(nx + u, slow + u, FRICTION_DAMPING * (configs[u]?.coulombFriction ?? 0));
    // LP(v) ← LP(v) + β (v − LP(v))
    a.set(slow + u, slow + u, 1 - beta);
    a.set(slow + u, motor, beta);
  }
  const b = new Matrix(n, m);
  // The command filter: f₁ ← f₁ + α(command − f₁), f₂ ← f₂ + α(f₁ − f₂); f₂ acts.
  for (let u = 0; u < m; u++) {
    a.set(f1 + u, f1 + u, 1 - alpha);
    a.set(f2 + u, f2 + u, 1 - alpha);
    a.set(f2 + u, f1 + u, alpha);
    b.set(f1 + u, u, alpha);
  }
  // A tool force: its effect on the accelerations over one cycle.
  const x0 = plant.state();
  plant.setState(x0);
  const rest = plant.acceleration();
  const force = new Matrix(n, 3);
  for (let axis = 0; axis < 3; axis++) {
    const f: [number, number, number] = [0, 0, 0];
    f[axis] = 1;
    plant.setToolForce(f);
    plant.setState(x0);
    const pushed = plant.acceleration();
    for (let k = 0; k < nq; k++)
      force.set(nq + k, axis, ((pushed[k] ?? 0) - (rest[k] ?? 0)) * plant.timestep);
  }
  plant.setToolForce([0, 0, 0]);
  // Measurements: angles directly; gyros, linear in the speeds.
  const c = new Matrix(m * MEASURED, n);
  configs.forEach((_, j) => {
    const { motor, gear } = plant.coordinates(j);
    c.set(j * MEASURED, motor, 1);
    c.set(j * MEASURED + 1, gear, 1);
  });
  for (let k = 0; k < nq; k++) {
    const gyros = (speed: number) => {
      const x = Float64Array.from(x0);
      x[nq + k] = (x[nq + k] ?? 0) + speed;
      plant.setState(x);
      return configs.map((_, j) => plant.gyro(j));
    };
    const plus = gyros(1);
    const minus = gyros(-1);
    configs.forEach((_, j) => {
      for (let axis = 0; axis < 3; axis++)
        c.set(
          j * MEASURED + 2 + axis,
          nq + k,
          ((plus[j]?.[axis] ?? 0) - (minus[j]?.[axis] ?? 0)) / 2,
        );
    });
  }
  plant.setState(x0);
  return { a, b, force, c, tool: lin.tool, nx };
}

export interface EstimatorDesign {
  readonly torqueWeight: number;
  readonly speedWeight: number;
  /** Assumed disturbance at the tool, N per cycle (standard deviation): the filter's trust in the model. */
  readonly forceNoise: number;
}

export interface Lqg {
  /** Damper command `u = −K·ẑ`. */
  readonly gain: Matrix;
  /** Kalman predictor gain: `ẑ⁺ = A ẑ + B u + L (y − C ẑ)`, using every joint's frame. */
  readonly filter: Matrix;
  /**
   * One filter per joint, each using only the frames of the joints up to `reach` places
   * away (columns of the others zero); absent when every joint uses every frame.
   */
  readonly neighbourhoods?: Matrix[];
}

export function designLqg(
  model: LoopModel,
  configs: readonly NodeConfig[],
  design: EstimatorDesign,
  reach?: number,
): Lqg {
  const n = model.a.rows;
  const nx = model.nx;
  const { q: qx, r } = designCost(model.tool, configs, design);
  const q = new Matrix(n, n);
  for (let i = 0; i < nx; i++) for (let k = 0; k < nx; k++) q.set(i, k, qx.get(i, k));
  for (let i = nx; i < n; i++) q.set(i, i, 1e-10);
  const gain = lqrGain(model.a, model.b, q, r);
  // The filter: the dual problem. Process noise: a force at the tool, plus a little of everything.
  const w = model.force.times(model.force.transpose());
  w.data.forEach((v, k) => (w.data[k] = v * design.forceNoise ** 2));
  let size = 0;
  for (let i = 0; i < n; i++) size += w.get(i, i);
  for (let i = 0; i < n; i++) w.set(i, i, w.get(i, i) + (1e-6 * size) / n);
  const p = model.c.rows;
  const v = new Matrix(p, p);
  for (let i = 0; i < p; i++) v.set(i, i, (i % MEASURED < 2 ? ANGLE_NOISE : GYRO_NOISE) ** 2);
  const kalman = (rows: number[]) => {
    const c = new Matrix(rows.length, n);
    const noise = new Matrix(rows.length, rows.length);
    rows.forEach((row, i) => {
      for (let k = 0; k < n; k++) c.set(i, k, model.c.get(row, k));
      noise.set(i, i, v.get(row, row));
    });
    const l = lqrGain(model.a.transpose(), c.transpose(), w, noise).transpose();
    const full = new Matrix(n, p);
    rows.forEach((row, i) => {
      for (let k = 0; k < n; k++) full.set(k, row, l.get(k, i));
    });
    return full;
  };
  const all = Array.from({ length: p }, (_, row) => row);
  const filter = kalman(all);
  if (reach === undefined) return { gain, filter };
  const neighbourhoods = configs.map((_, i) =>
    kalman(all.filter((row) => Math.abs(Math.floor(row / MEASURED) - i) <= reach)),
  );
  return { gain, filter, neighbourhoods };
}

/**
 * Runs the filter and gain on what arrives over the bus: every joint's latest state
 * frame as the host receives it. Rest angles and gyro biases are averaged first, with no
 * torque. The command acts one cycle later, capped at `torqueShare` of peak and passed
 * through the joints' energy tank.
 */
export function attachLqg(
  system: ArmSystem,
  configs: readonly NodeConfig[],
  model: LoopModel,
  lqg: Lqg,
  torqueShare: number,
  tankJ: number,
  calibrationCycles = 800,
): void {
  const m = configs.length;
  const n = model.a.rows;
  interface Frame {
    motorAngle: number;
    gearTwist: number;
    gyro: readonly number[];
  }
  const latest = new Map<number, Frame>();
  const rest = configs.map(() => ({ motorAngle: 0, gearTwist: 0, gyro: [0, 0, 0] }));
  let samples = 0;
  // ẑ at the measurements' time: one shared, or one per joint.
  let estimates = Array.from({ length: lqg.neighbourhoods ? m : 1 }, () => new Float64Array(n));
  const commands: Float64Array[] = []; // the last MEASUREMENT_DELAY commands
  // The command filter, as in the model: its second stage acts on the plant.
  let first = new Float64Array(m);
  let second = new Float64Array(m);
  const alpha = 1 - Math.exp(-2 * Math.PI * COMMAND_LOWPASS_HZ * system.plant.timestep);
  const tank = configs.map(() => 0);
  const dt = system.plant.timestep;
  const times = (mat: Matrix, x: Float64Array) => {
    const out = new Float64Array(mat.rows);
    for (let i = 0; i < mat.rows; i++) {
      let s = 0;
      for (let k = 0; k < mat.cols; k++) s += mat.get(i, k) * (x[k] ?? 0);
      out[i] = s;
    }
    return out;
  };
  const add = (x: Float64Array, y: Float64Array) => x.map((v, i) => v + (y[i] ?? 0));
  system.outsideTorque = (s) => {
    for (const f of s.host.lastReceived) if ("state" in f) latest.set(f.header.source, f.state);
    if (samples < calibrationCycles) {
      samples++;
      configs.forEach((_, j) => {
        const f = latest.get(j + 1);
        const r = rest[j];
        if (!f || !r) return;
        r.motorAngle += (f.motorAngle - r.motorAngle) / samples;
        r.gearTwist += (f.gearTwist - r.gearTwist) / samples;
        r.gyro = r.gyro.map((g, axis) => g + ((f.gyro[axis] ?? 0) - g) / samples);
      });
      return configs.map(() => 0);
    }
    const y = new Float64Array(m * MEASURED);
    configs.forEach((_, j) => {
      const f = latest.get(j + 1);
      const r = rest[j];
      if (!f || !r) return;
      y[j * MEASURED] = f.motorAngle - r.motorAngle;
      y[j * MEASURED + 1] = f.gearTwist - r.gearTwist;
      for (let axis = 0; axis < 3; axis++)
        y[j * MEASURED + 2 + axis] = (f.gyro[axis] ?? 0) - (r.gyro[axis] ?? 0);
    });
    // Filter at the measurements' time with the commands that acted then: every joint
    // puts its command in its state frame, so they arrive with the measurements. (A joint
    // guessing the others' commands from its own estimate here oscillated: the joints'
    // estimates drift apart and their guesses about each other compound it.)
    const then = commands[0] ?? new Float64Array(m);
    const step = (z: Float64Array, u: Float64Array) => add(times(model.a, z), times(model.b, u));
    const filters = lqg.neighbourhoods ?? [lqg.filter];
    estimates = estimates.map((estimate, f) => {
      const innovation = y.map((v, i) => v - (times(model.c, estimate)[i] ?? 0));
      return add(step(estimate, then), times(filters[f] ?? lqg.filter, innovation));
    });
    // Carry each estimate forward over the delay. A joint knows its own commands since;
    // the others' it predicts from its own estimate.
    const command = new Float64Array(m);
    estimates.forEach((estimate, f) => {
      let now = estimate;
      for (let d = 1; d < commands.length; d++) {
        const sent = commands[d] ?? new Float64Array(m);
        const guess = lqg.neighbourhoods
          ? times(lqg.gain, now).map((v, j) => (j === f ? (sent[j] ?? 0) : -v))
          : sent;
        now = step(now, guess);
      }
      // Limited before the low-pass (anti-windup), as in the joints.
      const u = times(lqg.gain, now).map((v, j) => {
        const cap = torqueShare * (configs[j]?.peakTorque ?? 0);
        if (lqg.neighbourhoods && j !== f) return 0; // only this joint's own command is used
        return Math.max(-cap, Math.min(cap, -v));
      });
      if (lqg.neighbourhoods) command[f] = u[f] ?? 0;
      else command.set(u);
    });
    commands.push(command);
    if (commands.length > MEASUREMENT_DELAY) commands.shift();
    // The damper's torque acts one cycle late, as inside a joint.
    const torque = second;
    second = second.map((v, i) => v + alpha * ((first[i] ?? 0) - v));
    first = first.map((v, i) => v + alpha * ((command[i] ?? 0) - v));
    return configs.map((c, i) => {
      const cap = torqueShare * c.peakTorque;
      let u = Math.max(-cap, Math.min(cap, torque[i] ?? 0));
      const work = u * s.plant.outputVelocity(i) * dt;
      const energy = tank[i] ?? 0;
      if (work <= 0) tank[i] = Math.min(tankJ, energy - work);
      else if (work <= energy) tank[i] = energy - work;
      else u = 0;
      return u;
    });
  };
}

/** Format of a coordinated-damping block (`coordinated.rs`). */
export const BLOCK_FORMAT = 1;
/** Cycles a joint averages its rest values over before acting. */
export const CALIBRATION_CYCLES = 800;

/**
 * Packs, for each joint, what its node needs (`crates/nexus-node/src/coordinated.rs`):
 * header [format, line position, states, joints, rows, delay, filter α, calibration
 * cycles], then per measurement row (line position, channel), then A, B, the rows of C,
 * the joint's own filter for those rows, and the whole gain K, each row by row.
 */
export function coordinatedBlocks(
  model: LoopModel,
  lqg: Lqg,
  configs: readonly NodeConfig[],
  reach: number,
  cycleSeconds: number,
): Float32Array[] {
  const n = model.a.rows;
  const m = configs.length;
  const alpha = 1 - Math.exp(-2 * Math.PI * COMMAND_LOWPASS_HZ * cycleSeconds);
  return configs.map((_, i) => {
    const filter = lqg.neighbourhoods?.[i];
    if (!filter) throw new Error("design the filters with a reach (designLqg(…, reach))");
    const rows = Array.from({ length: m * MEASURED }, (_, r) => r).filter(
      (r) => Math.abs(Math.floor(r / MEASURED) - i) <= reach,
    );
    const words: number[] = [
      BLOCK_FORMAT,
      i + 1,
      n,
      m,
      rows.length,
      MEASUREMENT_DELAY,
      alpha,
      CALIBRATION_CYCLES,
    ];
    for (const r of rows) words.push(Math.floor(r / MEASURED) + 1, r % MEASURED);
    for (let a = 0; a < n; a++) for (let k = 0; k < n; k++) words.push(model.a.get(a, k));
    for (let a = 0; a < n; a++) for (let u = 0; u < m; u++) words.push(model.b.get(a, u));
    for (const r of rows) for (let k = 0; k < n; k++) words.push(model.c.get(r, k));
    for (let a = 0; a < n; a++) for (const r of rows) words.push(filter.get(a, r));
    for (let u = 0; u < m; u++) for (let k = 0; k < n; k++) words.push(lqg.gain.get(u, k));
    return Float32Array.from(words);
  });
}

/**
 * A coordinated-damping design as committed for an arm (`models/coordinated-damping/<arm>.json`):
 * the shared matrices once, each joint's own filter, row by row with 7 significant digits.
 */
export interface CoordinatedDocument {
  readonly format: 1;
  readonly arm: string;
  /** The payload at the tool the design was made with, kg. */
  readonly payload: number;
  readonly reach: number;
  readonly cycleSeconds: number;
  readonly states: number;
  readonly joints: number;
  readonly a: number[];
  readonly b: number[];
  readonly c: number[];
  readonly gain: number[];
  /** Each joint's filter, states × measurement rows (all rows; zero outside its reach). */
  readonly filters: number[][];
}

const digits = (m: Matrix) => Array.from(m.data, (v) => Number(v.toPrecision(7)));

export function coordinatedDocument(
  arm: string,
  payload: number,
  model: LoopModel,
  lqg: Lqg,
  reach: number,
  cycleSeconds: number,
): CoordinatedDocument {
  if (!lqg.neighbourhoods) throw new Error("design the filters with a reach");
  return {
    format: 1,
    arm,
    payload,
    reach,
    cycleSeconds,
    states: model.a.rows,
    joints: model.b.cols,
    a: digits(model.a),
    b: digits(model.b),
    c: digits(model.c),
    gain: digits(lqg.gain),
    filters: lqg.neighbourhoods.map(digits),
  };
}

/** The per-joint blocks for the nodes, from a committed design. */
export function blocksFromDocument(
  doc: CoordinatedDocument,
  configs: readonly NodeConfig[],
): Float32Array[] {
  const n = doc.states;
  const m = doc.joints;
  const p = m * MEASURED;
  const model: LoopModel = {
    a: new Matrix(n, n, Float64Array.from(doc.a)),
    b: new Matrix(n, m, Float64Array.from(doc.b)),
    force: new Matrix(n, 3),
    c: new Matrix(p, n, Float64Array.from(doc.c)),
    tool: new Matrix(3, 0),
    nx: n - 4 * m,
  };
  const lqg: Lqg = {
    gain: new Matrix(m, n, Float64Array.from(doc.gain)),
    filter: new Matrix(n, p),
    neighbourhoods: doc.filters.map((f) => new Matrix(n, p, Float64Array.from(f))),
  };
  return coordinatedBlocks(model, lqg, configs, doc.reach, doc.cycleSeconds);
}
