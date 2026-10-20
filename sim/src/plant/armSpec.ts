/**
 * Description of an arm: its joints, links and actuators.
 *
 * An `ArmSpec` is plain data (JSON-serialisable), so the demo can edit it and the
 * experiments can sweep it. {@link buildMjcf} turns it into a MuJoCo model.
 *
 * Conventions: SI units, angles in radians. At the zero pose every link frame is
 * aligned with the world frame; `link` is the vector from a joint to the next joint (or
 * to the tool flange for the last joint), expressed in that frame.
 */

export type Vec3 = readonly [number, number, number];

/** A quasi-direct-drive actuator, described at the joint output. */
export interface ActuatorSpec {
  /** Gear ratio, motor turns per output turn. */
  readonly gearRatio: number;
  /** Rotor inertia on the motor shaft, kg·m². Reflected inertia is this × ratio². */
  readonly rotorInertia: number;
  /** Torsional stiffness of the gear stage at the output, N·m/rad. */
  readonly gearStiffness: number;
  /** Damping of the gear stage at the output, N·m·s/rad. */
  readonly gearDamping: number;
  /** Viscous friction at the output, N·m·s/rad. */
  readonly viscousFriction: number;
  /** Coulomb friction at the output, N·m. */
  readonly coulombFriction: number;
  /** Continuous torque at the output, N·m. */
  readonly ratedTorque: number;
  /** Peak torque at the output, N·m. Also the actuator's control range. */
  readonly peakTorque: number;
  /** Whether a holding brake is fitted. */
  readonly brake: boolean;
}

/** Lumped bending flexibility of a link: one bending joint (two axes) at mid-length. */
export interface FlexSpec {
  /** Bending stiffness about each axis perpendicular to the link, N·m/rad. */
  readonly stiffness: number;
  /** Bending damping about each axis, N·m·s/rad. */
  readonly damping: number;
}

export interface LinkSpec {
  /** Vector from this joint to the next joint (or the flange), m. */
  readonly vector: Vec3;
  /** Link mass including the next actuator's housing, kg. */
  readonly mass: number;
  /** Capsule radius used for collision and rendering, m. */
  readonly radius: number;
  /** Bending flexibility; omitted for links treated as rigid. */
  readonly flex?: FlexSpec;
}

export interface JointSpec {
  /** Short name, used as a prefix for every MuJoCo element of this joint. */
  readonly name: string;
  /** Rotation axis in the joint's frame (unit vector). */
  readonly axis: Vec3;
  /** Joint angle at which the arm starts, rad. */
  readonly initialAngle: number;
  readonly actuator: ActuatorSpec;
  readonly link: LinkSpec;
}

export interface PayloadSpec {
  /** Mass at the tool, kg. */
  readonly mass: number;
  /** Centre of mass relative to the flange, m. */
  readonly offset: Vec3;
}

export interface ArmSpec {
  readonly name: string;
  /** Joints from the base outward. */
  readonly joints: readonly JointSpec[];
  readonly payload: PayloadSpec;
}

/** Reflected rotor inertia at the output, kg·m². */
export function reflectedInertia(actuator: ActuatorSpec): number {
  return actuator.rotorInertia * actuator.gearRatio ** 2;
}

/**
 * Checks an arm spec for values that would make an invalid or meaningless model.
 *
 * @returns the list of problems; empty when the spec is valid.
 */
export function validateArmSpec(spec: ArmSpec): string[] {
  const problems: string[] = [];
  if (spec.joints.length === 0) problems.push("arm has no joints");
  const names = new Set<string>();
  for (const joint of spec.joints) {
    const where = `joint ${joint.name}`;
    if (!/^[a-z][a-z0-9]*$/.test(joint.name))
      problems.push(`${where}: name must be lower-case alphanumeric`);
    if (names.has(joint.name)) problems.push(`${where}: duplicate name`);
    names.add(joint.name);
    if (Math.abs(Math.hypot(...joint.axis) - 1) > 1e-9)
      problems.push(`${where}: axis must be a unit vector`);
    const a = joint.actuator;
    const positive: [string, number][] = [
      ["gearRatio", a.gearRatio],
      ["rotorInertia", a.rotorInertia],
      ["gearStiffness", a.gearStiffness],
      ["ratedTorque", a.ratedTorque],
      ["peakTorque", a.peakTorque],
      ["link.mass", joint.link.mass],
      ["link.radius", joint.link.radius],
    ];
    for (const [field, value] of positive) {
      if (!(value > 0)) problems.push(`${where}: ${field} must be positive`);
    }
    const nonNegative: [string, number][] = [
      ["gearDamping", a.gearDamping],
      ["viscousFriction", a.viscousFriction],
      ["coulombFriction", a.coulombFriction],
    ];
    for (const [field, value] of nonNegative) {
      if (!(value >= 0)) problems.push(`${where}: ${field} must not be negative`);
    }
    if (a.peakTorque < a.ratedTorque) problems.push(`${where}: peakTorque below ratedTorque`);
    if (Math.hypot(...joint.link.vector) <= 0)
      problems.push(`${where}: link vector must be non-zero`);
    const flex = joint.link.flex;
    if (flex && !(flex.stiffness > 0 && flex.damping >= 0)) {
      problems.push(`${where}: flex needs positive stiffness and non-negative damping`);
    }
  }
  if (!(spec.payload.mass >= 0)) problems.push("payload mass must not be negative");
  return problems;
}
