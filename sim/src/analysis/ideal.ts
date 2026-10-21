/**
 * An ideal reference controller (NX-27): how well *any* damping could do, as an upper
 * bound for what a learned damper might reach. No joint can build it.
 *
 * It sees the plant's whole true state (every motor, gear and link-bending coordinate)
 * with no bus delay, no noise and no estimation, and adds a torque to every joint's
 * command through {@link ArmSystem.outsideTorque}. The gain is a discrete-time LQR on
 * the plant linearised where the arm holds (finite differences on the MuJoCo model, one
 * bus cycle per step), with the joints' own position loops folded in, weighting the tool's
 * distance and speed against torque. Like the joints' damping it is held to a share of
 * each joint's peak torque and, optionally, to the same energy tank (B5).
 *
 * It regulates to the state where it was attached, so it applies to disturbances at rest
 * (taps), not to tasks.
 */
import type { ArmSystem } from "../harness/armSystem.ts";
import { BUS_CYCLE_HZ } from "../index.ts";
import type { NodeConfig } from "../node/nodeConfig.ts";
import { lqrGain, Matrix } from "../util/matrix.ts";

export interface IdealDesign {
  /** Weight of torque (relative to rated torque) against tool distance. */
  readonly torqueWeight: number;
  /** Weight of tool speed against tool distance, s². */
  readonly speedWeight: number;
  /** Share of each joint's peak torque it may use (the joints' damping: 0.25). */
  readonly torqueShare: number;
  /** Pass every torque through an energy tank of this capacity, J (the joints': 0.05). */
  readonly tankJ?: number;
  /** Act on the state this many bus cycles old (a bus between the plant and it). */
  readonly delayCycles?: number;
}

/** Linearisation step for angles (rad) and torques (N·m). */
const STATE_STEP = 1e-6;
const TORQUE_STEP = 1e-3;

/** `x' = Ax + Bu` around the plant's current state, one bus cycle per step. */
function linearise(system: ArmSystem): { a: Matrix; b: Matrix; tool: Matrix } {
  const plant = system.plant;
  if (Math.abs(plant.timestep * BUS_CYCLE_HZ - 1) > 1e-9) {
    throw new Error("the ideal controller assumes one physics step per bus cycle");
  }
  const x0 = plant.state();
  const n = x0.length;
  const nq = n / 2;
  const u0 = system.joints.map((j) => j.actuatorOutput.driveTorque);
  const after = (x: Float64Array, u: readonly number[]) => {
    plant.setState(x);
    u.forEach((t, i) => plant.setMotorTorque(i, t));
    plant.step();
    return plant.state();
  };
  const a = new Matrix(n, n);
  for (let k = 0; k < n; k++) {
    const plus = Float64Array.from(x0);
    const minus = Float64Array.from(x0);
    plus[k] = (plus[k] ?? 0) + STATE_STEP;
    minus[k] = (minus[k] ?? 0) - STATE_STEP;
    const xp = after(plus, u0);
    const xm = after(minus, u0);
    for (let i = 0; i < n; i++) a.set(i, k, ((xp[i] ?? 0) - (xm[i] ?? 0)) / (2 * STATE_STEP));
  }
  const b = new Matrix(n, u0.length);
  u0.forEach((_, k) => {
    const nudge = (s: number) => u0.map((t, i) => (i === k ? t + s * TORQUE_STEP : t));
    const xp = after(x0, nudge(1));
    const xm = after(x0, nudge(-1));
    for (let i = 0; i < n; i++) b.set(i, k, ((xp[i] ?? 0) - (xm[i] ?? 0)) / (2 * TORQUE_STEP));
  });
  // Tool position against every coordinate.
  const tool = new Matrix(3, nq);
  for (let k = 0; k < nq; k++) {
    const at = (s: number) => {
      const x = Float64Array.from(x0);
      x[k] = (x[k] ?? 0) + s * STATE_STEP;
      plant.setState(x);
      return plant.tcpPosition();
    };
    const p = at(1);
    const m = at(-1);
    for (let i = 0; i < 3; i++) tool.set(i, k, ((p[i] ?? 0) - (m[i] ?? 0)) / (2 * STATE_STEP));
  }
  plant.setState(x0);
  u0.forEach((t, i) => plant.setMotorTorque(i, t));
  return { a, b, tool };
}

export class IdealController {
  readonly #gain: Matrix;
  readonly #caps: number[];
  readonly #tankJ: number | undefined;
  readonly #delay: number;
  /** Largest torque used, as a share of the cap. */
  peakShare = 0;

  private constructor(gain: Matrix, caps: number[], design: IdealDesign) {
    this.#gain = gain;
    this.#caps = caps;
    this.#tankJ = design.tankJ;
    this.#delay = design.delayCycles ?? 0;
  }

  /**
   * Designs the controller for `system` as it is now: holding still, with every joint's
   * damping off. `configs` are the joints' own gains, which the design takes as given.
   */
  static design(system: ArmSystem, configs: readonly NodeConfig[], design: IdealDesign) {
    const { a, b, tool } = linearise(system);
    const n = a.rows;
    const nq = n / 2;
    // The joints' position loops: τ = −K·(motor + twist) − D·(their rates) + rotor damping.
    const joints = new Matrix(b.cols, n);
    configs.forEach((c, i) => {
      const { motor, gear } = system.plant.coordinates(i);
      joints.set(i, motor, -c.stiffness);
      joints.set(i, gear, -c.stiffness);
      joints.set(i, nq + motor, -c.damping);
      joints.set(i, nq + gear, -c.damping + c.rotorDamping);
    });
    const held = a.plus(b.times(joints));
    // Cost: tool distance² + speedWeight·tool speed² (+ a trace of every state).
    const c = new Matrix(6, n);
    for (let i = 0; i < 3; i++) {
      for (let k = 0; k < nq; k++) {
        c.set(i, k, tool.get(i, k));
        c.set(3 + i, nq + k, Math.sqrt(design.speedWeight) * tool.get(i, k));
      }
    }
    const q = c.transpose().times(c).plus(Matrix.identity(n), 1e-10);
    const r = new Matrix(b.cols, b.cols);
    configs.forEach((cfg, i) => r.set(i, i, design.torqueWeight / cfg.ratedTorque ** 2));
    const caps = configs.map((cfg) => design.torqueShare * cfg.peakTorque);
    return new IdealController(lqrGain(held, b, q, r), caps, design);
  }

  /** Acts on `system` from now on, holding it to its state now. */
  attach(system: ArmSystem): void {
    const rest = system.plant.state();
    const tank = this.#caps.map(() => 0);
    const dt = system.plant.timestep;
    const history: Float64Array[] = [];
    system.outsideTorque = (s) => {
      history.push(s.plant.state());
      const x = history.length > this.#delay ? (history.shift() ?? rest) : rest;
      return this.#caps.map((cap, i) => {
        let u = 0;
        for (let k = 0; k < x.length; k++)
          u -= this.#gain.get(i, k) * ((x[k] ?? 0) - (rest[k] ?? 0));
        u = Math.max(-cap, Math.min(cap, u));
        if (this.#tankJ !== undefined) {
          // The joints' energy tank (damping.rs), on the true output velocity.
          const work = u * s.plant.outputVelocity(i) * dt;
          const energy = tank[i] ?? 0;
          if (work <= 0) tank[i] = Math.min(this.#tankJ, energy - work);
          else if (work <= energy) tank[i] = energy - work;
          else u = 0;
        }
        this.peakShare = Math.max(this.peakShare, Math.abs(u) / cap);
        return u;
      });
    };
  }
}
