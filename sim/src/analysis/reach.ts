/**
 * Which of the arm's vibration modes can a joint reach (NX-28)? A mode that no joint's
 * torque drives, or that no joint's sensors see, cannot be damped by any controller in
 * the joints: not the network term, not a learned one.
 *
 * The arm is linearised where it holds, from MuJoCo's continuous accelerations (one
 * integrator step would mix the implicit damping in), with the joints' position loops
 * closed (`control.rs`). The position loop pushes on the motor and measures the output,
 * so the held arm is not a conservative system; its modes are the eigenvalues of the
 * full state matrix, found by inverse iteration with shifts swept over frequency (up to
 * `maxHz`) and damping.
 *
 * For each mode λ (right eigenvector v, left w) and each joint on its own, a damper
 * `τ = −c·y` on a signal `y = cᵀx`, through the joint's input b, moves the eigenvalue by
 * `Δλ = −c (wᵀb)(cᵀv) / (wᵀv)` to first order. The added damping ratio `−Re Δλ / |λ|` is
 * reported for two signals at the joint's network-damping gain:
 *
 * - local: the joint's own output rate (motor + twist, collocated);
 * - network: outer neighbour's gyro minus its own, on its axis (`damping.rs`).
 *
 * Filters and bus delay are left out: this is what the sensors can see at all. A negative
 * ratio means that joint, alone, would pump the mode instead of damping it.
 */
import type { ArmSystem } from "../harness/armSystem.ts";
import type { NodeConfig } from "../node/nodeConfig.ts";
import { Matrix } from "../util/matrix.ts";

export interface ModeReach {
  /** Natural frequency, Hz. */
  readonly hz: number;
  /** Damping ratio the mode has with the joints holding and no damping term. */
  readonly zeta: number;
  /** Share of the mode's motion in each kind of coordinate (sums to 1). */
  readonly shares: { motor: number; gear: number; bend: number };
  /** Damping ratio each joint would add on its own with collocated damping. */
  readonly local: number[];
  /** Damping ratio each joint would add on its own with its network term. */
  readonly network: number[];
}

/** A complex vector as real and imaginary parts. */
interface Complex {
  re: Float64Array;
  im: Float64Array;
}

const STEP = 1e-6;

/** Continuous state matrix [[0, I], [∂q̈/∂q, ∂q̈/∂q̇]] with the position loops closed. */
function stateMatrix(system: ArmSystem, configs: readonly NodeConfig[]): Matrix {
  const plant = system.plant;
  const x0 = plant.state();
  const n = x0.length;
  const nq = n / 2;
  const u0 = system.joints.map((j) => j.actuatorOutput.driveTorque);
  // τ_i = u0 − K·(motor + twist) − D·(their rates) + rotor damping·twist rate.
  const loops = (dx: Float64Array) =>
    configs.map((c, i) => {
      const { motor, gear } = plant.coordinates(i);
      const angle = (dx[motor] ?? 0) + (dx[gear] ?? 0);
      const rate = (dx[nq + motor] ?? 0) + (dx[nq + gear] ?? 0);
      return (
        (u0[i] ?? 0) -
        c.stiffness * angle -
        c.damping * rate +
        c.rotorDamping * (dx[nq + gear] ?? 0)
      );
    });
  const at = (dx: Float64Array) => {
    loops(dx).forEach((t, i) => plant.setMotorTorque(i, t));
    plant.setState(x0.map((x, k) => x + (dx[k] ?? 0)));
    return plant.acceleration();
  };
  const a = new Matrix(n, n);
  for (let i = 0; i < nq; i++) a.set(i, nq + i, 1);
  for (let k = 0; k < n; k++) {
    const dx = new Float64Array(n);
    dx[k] = STEP;
    const ap = at(dx);
    dx[k] = -STEP;
    const am = at(dx);
    for (let i = 0; i < nq; i++) a.set(nq + i, k, ((ap[i] ?? 0) - (am[i] ?? 0)) / (2 * STEP));
  }
  at(new Float64Array(n));
  return a;
}

/** Input of joint `i`'s motor torque: ∂ẋ/∂τ_i. */
function inputOf(system: ArmSystem, i: number): Float64Array {
  const plant = system.plant;
  const x0 = plant.state();
  const nq = x0.length / 2;
  const u0 = system.joints[i]?.actuatorOutput.driveTorque ?? 0;
  const accelerationWith = (torque: number) => {
    plant.setMotorTorque(i, torque);
    plant.setState(x0);
    return plant.acceleration();
  };
  const ap = accelerationWith(u0 + 1e-3);
  const am = accelerationWith(u0 - 1e-3);
  accelerationWith(u0);
  const b = new Float64Array(2 * nq);
  for (let k = 0; k < nq; k++) b[nq + k] = ((ap[k] ?? 0) - (am[k] ?? 0)) / 2e-3;
  return b;
}

/** Each joint's network signal as a row on the state: gyros are linear in the speeds. */
function networkRows(system: ArmSystem, configs: readonly NodeConfig[]): Float64Array[] {
  const plant = system.plant;
  const x0 = plant.state();
  const nq = x0.length / 2;
  const rows = configs.map(() => new Float64Array(2 * nq));
  const gyrosWith = (k: number, speed: number) => {
    const x = Float64Array.from(x0);
    x[nq + k] = (x[nq + k] ?? 0) + speed;
    plant.setState(x);
    return configs.map((_, i) => plant.gyro(i));
  };
  for (let k = 0; k < nq; k++) {
    const plus = gyrosWith(k, 1);
    const minus = gyrosWith(k, -1);
    configs.forEach((c, i) => {
      const along = (g: readonly number[] | undefined) =>
        g ? g.reduce((s, v, d) => s + v * (c.axis[d] ?? 0), 0) : 0;
      if (i + 1 >= configs.length) return;
      const signal = (g: readonly (readonly number[])[]) => along(g[i + 1]) - along(g[i]);
      const row = rows[i];
      if (row) row[nq + k] = (signal(plus) - signal(minus)) / 2;
    });
  }
  plant.setState(x0);
  return rows;
}

/** Real 2n form of `(A − σI)`, for complex σ, inverted. */
function shiftedInverse(a: Matrix, re: number, im: number): Matrix {
  const n = a.rows;
  const m = new Matrix(2 * n, 2 * n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      const v = a.get(i, j) - (i === j ? re : 0);
      m.set(i, j, v);
      m.set(n + i, n + j, v);
    }
    m.set(i, n + i, im);
    m.set(n + i, i, -im);
  }
  return m.inverse();
}

/** Inverse iteration from shift σ: the eigenpair nearest to it. */
function nearest(a: Matrix, re: number, im: number): { lambda: [number, number]; v: Complex } {
  const n = a.rows;
  const inv = shiftedInverse(a, re, im);
  let x = new Float64Array(2 * n).map((_, k) => 1 + 0.1 * Math.sin(k));
  for (let it = 0; it < 40; it++) {
    const y = new Float64Array(2 * n);
    for (let i = 0; i < 2 * n; i++) {
      let s = 0;
      for (let j = 0; j < 2 * n; j++) s += inv.get(i, j) * (x[j] ?? 0);
      y[i] = s;
    }
    const norm = Math.hypot(...y);
    x = y.map((v) => v / norm);
  }
  const v = { re: x.slice(0, n), im: x.slice(n) };
  // Rayleigh quotient vᴴAv / vᴴv (v has unit norm).
  let nr = 0;
  let ni = 0;
  for (let i = 0; i < n; i++) {
    let ar = 0;
    let ai = 0;
    for (let j = 0; j < n; j++) {
      ar += a.get(i, j) * (v.re[j] ?? 0);
      ai += a.get(i, j) * (v.im[j] ?? 0);
    }
    nr += (v.re[i] ?? 0) * ar + (v.im[i] ?? 0) * ai;
    ni += (v.re[i] ?? 0) * ai - (v.im[i] ?? 0) * ar;
  }
  return { lambda: [nr, ni], v };
}

/** `uᵀv` for complex u, v (no conjugation). */
function dot(u: Complex, v: Complex): [number, number] {
  let r = 0;
  let i = 0;
  u.re.forEach((ur, k) => {
    const ui = u.im[k] ?? 0;
    const vr = v.re[k] ?? 0;
    const vi = v.im[k] ?? 0;
    r += ur * vr - ui * vi;
    i += ur * vi + ui * vr;
  });
  return [r, i];
}

/** |uᴴv| for unit-norm complex u, v. */
function overlap(u: Complex, v: Complex): number {
  let r = 0;
  let i = 0;
  u.re.forEach((ur, k) => {
    const ui = u.im[k] ?? 0;
    const vr = v.re[k] ?? 0;
    const vi = v.im[k] ?? 0;
    r += ur * vr + ui * vi;
    i += ur * vi - ui * vr;
  });
  return Math.hypot(r, i);
}

const real = (x: Float64Array): Complex => ({ re: x, im: new Float64Array(x.length) });
const times = (a: [number, number], b: [number, number]): [number, number] => [
  a[0] * b[0] - a[1] * b[1],
  a[0] * b[1] + a[1] * b[0],
];
const over = (a: [number, number], b: [number, number]): [number, number] => {
  const d = b[0] * b[0] + b[1] * b[1];
  return [(a[0] * b[0] + a[1] * b[1]) / d, (a[1] * b[0] - a[0] * b[1]) / d];
};

/** Modes of `system`, holding where it is, below `maxHz`, and how each joint reaches them. */
export function modeReach(
  system: ArmSystem,
  configs: readonly NodeConfig[],
  maxHz = 80,
): ModeReach[] {
  const plant = system.plant;
  const a = stateMatrix(system, configs);
  const at = a.transpose();
  const n = a.rows;
  const nq = n / 2;
  const inputs = configs.map((_, i) => real(inputOf(system, i)));
  const network = networkRows(system, configs).map(real);
  const local = configs.map((_, i) => {
    const { motor, gear } = plant.coordinates(i);
    const row = new Float64Array(n);
    row[nq + motor] = 1;
    row[nq + gear] = 1;
    return real(row);
  });
  const coordinates = configs.map((_, i) => plant.coordinates(i));
  const kind = (k: number): "motor" | "gear" | "bend" =>
    coordinates.some((c) => c.motor === k)
      ? "motor"
      : coordinates.some((c) => c.gear === k)
        ? "gear"
        : "bend";

  const found: { lambda: [number, number]; v: Complex }[] = [];
  // Shifts over frequency and damping: the held arm's modes are heavily damped.
  const shifts = [0.05, 0.3, 0.6].flatMap((damping) =>
    Array.from({ length: 40 }, (_, k) => [damping, 0.5 * 1.15 ** k] as const),
  );
  for (const [damping, hz] of shifts) {
    if (hz > maxHz) continue;
    const w = 2 * Math.PI * hz;
    const pair = nearest(a, -damping * w, w * Math.sqrt(1 - damping * damping));
    const im = pair.lambda[1];
    if (im <= 0 || im / (2 * Math.PI) > maxHz) continue;
    // The same mode, reached from another shift: close in frequency, same shape.
    const seen = found.some(
      (p) => Math.abs(p.lambda[1] - im) < 0.05 * im && overlap(p.v, pair.v) > 0.9,
    );
    if (!seen) found.push(pair);
  }
  return found
    .sort((p, q) => p.lambda[1] - q.lambda[1])
    .map(({ lambda, v }) => {
      const magnitude = Math.hypot(...lambda);
      // The left eigenvector: the right one of Aᵀ for the same eigenvalue.
      const left = nearest(at, lambda[0], lambda[1] * (1 + 1e-9)).v;
      const norm = dot(left, v);
      const added = (rows: Complex[]) =>
        rows.map((row, i) => {
          const b = inputs[i];
          if (!b) return 0;
          const shift = over(times(dot(left, b), dot(row, v)), norm);
          // Δλ = −c·shift, so the added damping ratio −Re Δλ / |λ| is c·Re(shift) / |λ|.
          return ((configs[i]?.networkDamping ?? 0) * shift[0]) / magnitude;
        });
      const shares = { motor: 0, gear: 0, bend: 0 };
      for (let k = 0; k < nq; k++) shares[kind(k)] += (v.re[k] ?? 0) ** 2 + (v.im[k] ?? 0) ** 2;
      const total = shares.motor + shares.gear + shares.bend || 1;
      return {
        hz: lambda[1] / (2 * Math.PI),
        zeta: -lambda[0] / magnitude,
        shares: {
          motor: shares.motor / total,
          gear: shares.gear / total,
          bend: shares.bend / total,
        },
        local: added(local),
        network: added(network),
      };
    });
}
