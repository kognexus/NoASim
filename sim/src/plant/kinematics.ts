/**
 * Nominal forward kinematics and load inertia of an {@link ArmSpec}, as an integrator would
 * compute them from the datasheet when sizing joint gains.
 */
import type { ArmSpec, Vec3 } from "./armSpec.ts";
import { reflectedInertia } from "./armSpec.ts";

type Mat3 = [number, number, number, number, number, number, number, number, number];

const IDENTITY: Mat3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];

/** Rotation by `angle` about unit `axis` (Rodrigues). */
function rotation(axis: Vec3, angle: number): Mat3 {
  const [x, y, z] = axis;
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const t = 1 - c;
  return [
    t * x * x + c,
    t * x * y - s * z,
    t * x * z + s * y,
    t * x * y + s * z,
    t * y * y + c,
    t * y * z - s * x,
    t * x * z - s * y,
    t * y * z + s * x,
    t * z * z + c,
  ];
}

function multiply(a: Mat3, b: Mat3): Mat3 {
  const [a0, a1, a2, a3, a4, a5, a6, a7, a8] = a;
  const [b0, b1, b2, b3, b4, b5, b6, b7, b8] = b;
  return [
    a0 * b0 + a1 * b3 + a2 * b6,
    a0 * b1 + a1 * b4 + a2 * b7,
    a0 * b2 + a1 * b5 + a2 * b8,
    a3 * b0 + a4 * b3 + a5 * b6,
    a3 * b1 + a4 * b4 + a5 * b7,
    a3 * b2 + a4 * b5 + a5 * b8,
    a6 * b0 + a7 * b3 + a8 * b6,
    a6 * b1 + a7 * b4 + a8 * b7,
    a6 * b2 + a7 * b5 + a8 * b8,
  ];
}

function apply(m: Mat3, v: Vec3): Vec3 {
  return [
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
  ];
}

const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a: Vec3, s: number): Vec3 => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

/** World position and axis of each joint, and the flange position, at `angles`. */
export interface Pose {
  readonly jointPositions: readonly Vec3[];
  readonly jointAxes: readonly Vec3[];
  /** Midpoint of each link, world frame. */
  readonly linkCentres: readonly Vec3[];
  readonly flange: Vec3;
  /** Orientation of the flange frame. */
  readonly flangeRotation: Mat3;
}

/** Height of joint 1 above the base, as built by `buildMjcf`. */
const BASE_HEIGHT = 0.05;

export function forwardKinematics(spec: ArmSpec, angles: readonly number[]): Pose {
  let position: Vec3 = [0, 0, BASE_HEIGHT];
  let frame: Mat3 = IDENTITY;
  const jointPositions: Vec3[] = [];
  const jointAxes: Vec3[] = [];
  const linkCentres: Vec3[] = [];
  spec.joints.forEach((joint, i) => {
    jointPositions.push(position);
    jointAxes.push(apply(frame, joint.axis));
    frame = multiply(frame, rotation(joint.axis, angles[i] ?? 0));
    const link = apply(frame, joint.link.vector);
    linkCentres.push(add(position, scale(link, 0.5)));
    position = add(position, link);
  });
  return { jointPositions, jointAxes, linkCentres, flange: position, flangeRotation: frame };
}

/**
 * Inertia each joint drives about its own axis at `angles`: point masses at the link
 * centres and the payload, plus the joint's reflected rotor inertia.
 */
export function jointLoadInertia(
  spec: ArmSpec,
  angles: readonly number[] = spec.joints.map((j) => j.initialAngle),
): number[] {
  const pose = forwardKinematics(spec, angles);
  const masses: { position: Vec3; mass: number }[] = spec.joints.map((joint, k) => ({
    position: pose.linkCentres[k] ?? [0, 0, 0],
    mass: joint.link.mass,
  }));
  masses.push({
    position: add(pose.flange, apply(pose.flangeRotation, spec.payload.offset)),
    mass: spec.payload.mass,
  });
  return spec.joints.map((joint, i) => {
    const origin = pose.jointPositions[i] ?? [0, 0, 0];
    const axis = pose.jointAxes[i] ?? joint.axis;
    let inertia = reflectedInertia(joint.actuator);
    masses.forEach(({ position, mass }, k) => {
      if (k < i) return; // inboard of joint i
      const r = sub(position, origin);
      const along = dot(r, axis);
      inertia += mass * (dot(r, r) - along * along);
    });
    return inertia;
  });
}
