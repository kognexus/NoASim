/**
 * A colleague's frame-aligned network damping law (NX-32, `lyapunov_qdd.pdf`), run as
 * written: each joint damps the relative rate between its own IMU and each neighbour's,
 * the neighbour's gyro resolved in the joint's frame through the encoder angles,
 * high-passed, on both sides, with no energy tank:
 *
 * ```text
 * Δω_ij = ω_i − R_ij(q)·ω_j,     τ_i = −Σ_{j∈{i−1,i+1}} D_ij · a_iᵀ HPF(Δω_ij)
 * ```
 *
 * `D_ij = gain·√(c_i c_j)` with `c` each joint's network-damping gain; `a_i` the joint's
 * axis (a joint can only apply torque about it). The joints' own collocated damping (the
 * derivative terms of their position loops) plays the note's `d_i θ̇_m`.
 *
 * As written, the outer edge's term opposes `a_iᵀ(ω_i − ω_{i+1}) = −(q̇_i + ḃ_i)`, which
 * pushes the joint along its own speed; `sign: −1` flips every term so that the outer
 * edge damps the joint's speed instead. Both are run. Experiments only: the torques are
 * added outside the joints, on true gyros and encoder angles.
 */
import type { ArmSystem } from "../harness/armSystem.ts";
import type { NodeConfig } from "../node/nodeConfig.ts";
import type { Vec3 } from "../plant/armSpec.ts";
import { Matrix } from "../util/matrix.ts";

export interface FrameAlignedOptions {
  /** Edge gain as a multiple of the network-damping gains. */
  readonly gain: number;
  /** Bus cycles between the gyros and the torque. */
  readonly delayCycles: number;
  /** +1: the law as written; −1: every term flipped. */
  readonly sign: 1 | -1;
  /** Which neighbours each joint damps against. */
  readonly edges: "both" | "outer";
  /** Also low-pass the signal (second order) at the joints' IMU corner, as `damping.rs`
   * does; the law as written has none. */
  readonly lowpass?: boolean;
}

export interface FrameAlignedProbe {
  /** Work each joint's term has done on the arm so far (∫ τ·θ̇_motor dt), J. */
  readonly work: Float64Array;
  /** Largest work any joint's term has put in at any moment, J. */
  peakWork: number;
}

/** `v` turned by `angle` about the unit `axis` (Rodrigues). */
export function rotate(v: Vec3, axis: readonly number[], angle: number): Vec3 {
  const [x, y, z] = [axis[0] ?? 0, axis[1] ?? 0, axis[2] ?? 0];
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const d = (x * v[0] + y * v[1] + z * v[2]) * (1 - c);
  return [
    v[0] * c + (y * v[2] - z * v[1]) * s + x * d,
    v[1] * c + (z * v[0] - x * v[2]) * s + y * d,
    v[2] * c + (x * v[1] - y * v[0]) * s + z * d,
  ];
}

const dot = (a: readonly number[], b: readonly number[]) =>
  (a[0] ?? 0) * (b[0] ?? 0) + (a[1] ?? 0) * (b[1] ?? 0) + (a[2] ?? 0) * (b[2] ?? 0);

/**
 * How joint `i`'s frame relates to its neighbours': the sign of the turn that resolves a
 * neighbour's gyro in joint `i`'s housing frame, found once by spinning the base, which
 * turns every housing beyond it rigidly (so a correct `Δω` is zero).
 */
function turnSigns(
  system: ArmSystem,
  configs: readonly NodeConfig[],
): { outer: number; inner: number } {
  const plant = system.plant;
  const x0 = plant.state();
  const nq = x0.length / 2;
  const x = Float64Array.from(x0);
  x.fill(0, nq);
  x[nq + plant.coordinates(0).motor] = 1;
  plant.setState(x);
  const gyros = configs.map((_, i) => plant.gyro(i));
  const angles = configs.map((_, i) => plant.outputAngle(i));
  plant.setState(x0);
  const residual = (s: number, edge: "outer" | "inner") => {
    let sum = 0;
    for (let i = 1; i < configs.length; i++) {
      const j = edge === "outer" ? i + 1 : i - 1;
      if (j < 1 || j >= configs.length) continue;
      // The turn between the frames is the inner one of the two joints' angle.
      const k = Math.min(i, j);
      const turned = rotate(
        gyros[j] ?? [0, 0, 0],
        configs[k]?.axis ?? [0, 0, 1],
        s * (angles[k] ?? 0),
      );
      const g = gyros[i] ?? [0, 0, 0];
      sum += (g[0] - turned[0]) ** 2 + (g[1] - turned[1]) ** 2 + (g[2] - turned[2]) ** 2;
    }
    return sum;
  };
  const pick = (edge: "outer" | "inner") => (residual(1, edge) <= residual(-1, edge) ? 1 : -1);
  return { outer: pick("outer"), inner: pick("inner") };
}

/** The law's `a_iᵀ Δω_ij` for every joint and edge, from the plant's gyros and angles. */
function relativeRates(
  system: ArmSystem,
  configs: readonly NodeConfig[],
  signs: { outer: number; inner: number },
): [number, number][] {
  const plant = system.plant;
  const gyros = configs.map((_, i) => plant.gyro(i));
  const angles = configs.map((_, i) => plant.outputAngle(i));
  return configs.map((c, i) => {
    const own = gyros[i] ?? [0, 0, 0];
    const edge = (j: number, s: number): number => {
      const other = gyros[j];
      if (!other) return 0;
      const k = Math.min(i, j);
      const turned = rotate(other, configs[k]?.axis ?? [0, 0, 1], s * (angles[k] ?? 0));
      return dot(c.axis, [own[0] - turned[0], own[1] - turned[1], own[2] - turned[2]]);
    };
    return [edge(i - 1, signs.inner), edge(i + 1, signs.outer)];
  });
}

/** Adds the law to `system` from now on; the probe accumulates each joint's work. */
export function attachFrameAligned(
  system: ArmSystem,
  configs: readonly NodeConfig[],
  options: FrameAlignedOptions,
): FrameAlignedProbe {
  const dt = system.plant.timestep;
  const signs = turnSigns(system, configs);
  const n = configs.length;
  const gains = configs.map((c, i) =>
    [i - 1, i + 1].map((j) => {
      const other = configs[j];
      if (!other || (options.edges === "outer" && j < i)) return 0;
      return options.gain * Math.sqrt(c.networkDamping * other.networkDamping);
    }),
  );
  // First-order high-pass per joint and edge, at the network damper's corner.
  const alpha = configs.map((c) => 1 / (1 + 2 * Math.PI * c.dampingHighpassHz * dt));
  const beta = configs.map((c) => 1 - Math.exp(-2 * Math.PI * c.imuLowpassHz * dt));
  const stages = configs.map(() => [
    [0, 0],
    [0, 0],
  ]);
  const lastIn = configs.map(() => [0, 0]);
  const lastOut = configs.map(() => [0, 0]);
  const queue: number[][] = [];
  const probe: FrameAlignedProbe = { work: new Float64Array(n), peakWork: 0 };
  let torque = new Array<number>(n).fill(0);
  let first = true;
  system.outsideTorque = (s) => {
    // The work of last cycle's torque, at the motor where it acts.
    torque.forEach((t, i) => {
      probe.work[i] = (probe.work[i] ?? 0) + t * s.plant.motorVelocity(i) * dt;
      probe.peakWork = Math.max(probe.peakWork, probe.work[i] ?? 0);
    });
    const rates = relativeRates(s, configs, signs);
    const filtered = rates.map((pair, i) =>
      pair.map((x, e) => {
        const a = alpha[i] ?? 1;
        const y = first ? 0 : a * ((lastOut[i]?.[e] ?? 0) + x - (lastIn[i]?.[e] ?? 0));
        (lastIn[i] as number[])[e] = x;
        (lastOut[i] as number[])[e] = y;
        if (!options.lowpass) return y;
        const st = stages[i] as number[][];
        const b = beta[i] ?? 1;
        (st[0] as number[])[e] = (st[0]?.[e] ?? 0) + b * (y - (st[0]?.[e] ?? 0));
        (st[1] as number[])[e] = (st[1]?.[e] ?? 0) + b * ((st[0]?.[e] ?? 0) - (st[1]?.[e] ?? 0));
        return st[1]?.[e] ?? 0;
      }),
    );
    first = false;
    const now = filtered.map((pair, i) =>
      pair.reduce((sum, y, e) => sum - options.sign * (gains[i]?.[e] ?? 0) * y, 0),
    );
    queue.push(now);
    torque =
      queue.length > options.delayCycles ? (queue.shift() ?? now) : new Array<number>(n).fill(0);
    return torque;
  };
  return probe;
}

/**
 * The law with no delay and no high-pass as a gain on the held arm's state, rows per
 * joint: `τ = −sign·Σ D_ij a_iᵀ Δω_ij`, gyros being linear in the speeds.
 */
export function frameAlignedGain(
  system: ArmSystem,
  configs: readonly NodeConfig[],
  options: Pick<FrameAlignedOptions, "gain" | "sign" | "edges">,
): Matrix {
  const plant = system.plant;
  const x0 = plant.state();
  const nq = x0.length / 2;
  const signs = turnSigns(system, configs);
  const gains = configs.map((c, i) =>
    [i - 1, i + 1].map((j) => {
      const other = configs[j];
      if (!other || (options.edges === "outer" && j < i)) return 0;
      return options.gain * Math.sqrt(c.networkDamping * other.networkDamping);
    }),
  );
  const k = new Matrix(configs.length, 2 * nq);
  const at = (coordinate: number, speed: number) => {
    const x = Float64Array.from(x0);
    x[nq + coordinate] = (x[nq + coordinate] ?? 0) + speed;
    plant.setState(x);
    return relativeRates(system, configs, signs);
  };
  for (let q = 0; q < nq; q++) {
    const plus = at(q, 1);
    const minus = at(q, -1);
    configs.forEach((_, i) => {
      let row = 0;
      for (const e of [0, 1]) {
        const slope = ((plus[i]?.[e] ?? 0) - (minus[i]?.[e] ?? 0)) / 2;
        row -= options.sign * (gains[i]?.[e] ?? 0) * slope;
      }
      k.set(i, nq + q, row);
    });
  }
  plant.setState(x0);
  return k;
}
