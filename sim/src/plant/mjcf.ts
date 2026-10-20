/**
 * Turns an {@link ArmSpec} into MJCF (MuJoCo's XML model format).
 *
 * Each joint becomes two hinges in series on the same axis:
 *
 *   housing ─[motor hinge]─ rotor ─[gear hinge]─ link
 *
 * The motor hinge carries the rotor inertia reflected through the gear (`armature`) and
 * the motor torque; the gear hinge is a spring-damper for the gear stage's compliance.
 * The difference between the motor-side and the output encoder is therefore the gear
 * twist. A flexible link is split at mid-length by two bending hinges perpendicular to
 * the link.
 *
 * The actuator board (and its IMU) sits on the actuator housing, which is fixed to the
 * distal end of the previous link. Joint i's IMU therefore moves with link i-1.
 */
import type { ArmSpec, JointSpec, Vec3 } from "./armSpec.ts";
import { reflectedInertia, validateArmSpec } from "./armSpec.ts";
import { ARM_NAMES, jointElementNames } from "./names.ts";
import { el, serialize, vec, type XmlElement } from "./xml.ts";

export interface MjcfOptions {
  /** Physics timestep, s. Defaults to one 4 kHz bus cycle. */
  readonly timestep?: number;
  /** Turn gravity off (modal analysis). */
  readonly gravity?: boolean;
  /**
   * Omit the motor hinges, so the rotor is rigidly fixed to the housing. Used to measure
   * the structure's modes independent of any controller. No actuators are generated.
   */
  readonly lockMotors?: boolean;
  /**
   * Leave friction off the motor hinges because the actuator physics applies it (hidden
   * actuator families, NX-04). Defaults to `false`: the datasheet friction is in the model.
   */
  readonly externalFriction?: boolean;
  /** A spring-mounted block for impact tests; its spring doubles as a force gauge. */
  readonly obstacle?: ObstacleSpec;
}

export interface ObstacleSpec {
  /** Centre of the block at rest, world frame, m. */
  readonly position: Vec3;
  /** Half-sizes of the block, m. */
  readonly halfSize: Vec3;
  /** Direction the block can be pushed along (unit vector). */
  readonly axis: Vec3;
  /** Spring stiffness of the mount, N/m. */
  readonly stiffness: number;
}

/** Names of the obstacle's elements. */
export const OBSTACLE = { body: "obstacle", joint: "obstacle_slide" } as const;

/** Height of joint 1 above the floor, m. */
const BASE_HEIGHT = 0.05;
/** Rotor body mass and inertia; the dynamics come from `armature`, this only keeps MuJoCo happy. */
const ROTOR_BODY = { mass: 0.05, inertia: 1e-5 } as const;
/** Collision filtering: arm geoms touch the environment but not each other. */
const ARM_CONTACT = { contype: 2, conaffinity: 1 } as const;
const ENV_CONTACT = { contype: 1, conaffinity: 2 } as const;

const scale = (v: Vec3, s: number): Vec3 => [v[0] * s, v[1] * s, v[2] * s];
const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const unit = (v: Vec3): Vec3 => scale(v, 1 / Math.hypot(...v));

/** Two unit axes perpendicular to the link, for the bending hinges. */
export function bendingAxes(link: Vec3): [Vec3, Vec3] {
  const along = unit(link);
  const helper: Vec3 = Math.abs(along[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const first = unit(cross(along, helper));
  return [first, cross(along, first)];
}

function capsule(from: Vec3, to: Vec3, radius: number, mass: number): XmlElement {
  return el("geom", {
    type: "capsule",
    fromto: vec([...from, ...to]),
    size: radius,
    mass,
    ...ARM_CONTACT,
  });
}

function tool(spec: ArmSpec, position: Vec3): XmlElement[] {
  const elements = [el("site", { name: ARM_NAMES.tcpSite, pos: vec(position), size: 0.01 })];
  if (spec.payload.mass > 0) {
    elements.push(
      el("body", { name: ARM_NAMES.payloadBody, pos: vec(position) }, [
        el("geom", {
          type: "sphere",
          pos: vec(spec.payload.offset),
          size: 0.04,
          mass: spec.payload.mass,
          ...ARM_CONTACT,
        }),
      ]),
    );
  }
  return elements;
}

/** The housing site and rotor body of joint `index`, placed at `position` in the parent body. */
function jointChain(
  spec: ArmSpec,
  index: number,
  position: Vec3,
  options: MjcfOptions,
): XmlElement[] {
  const joint = spec.joints[index];
  if (!joint) return tool(spec, position);
  const names = jointElementNames(joint.name);
  const actuator = joint.actuator;
  const linkBody = el("body", { name: names.linkBody }, [
    el("joint", {
      name: names.gearJoint,
      type: "hinge",
      axis: vec(joint.axis),
      stiffness: actuator.gearStiffness,
      damping: actuator.gearDamping,
    }),
    ...linkContents(spec, index, joint, options),
  ]);
  const rotorChildren = options.lockMotors
    ? [linkBody]
    : [
        el("joint", {
          name: names.motorJoint,
          type: "hinge",
          axis: vec(joint.axis),
          armature: reflectedInertia(actuator),
          damping: options.externalFriction ? undefined : actuator.viscousFriction,
          frictionloss: options.externalFriction ? undefined : actuator.coulombFriction,
        }),
        el("inertial", {
          pos: "0 0 0",
          mass: ROTOR_BODY.mass,
          diaginertia: vec([ROTOR_BODY.inertia, ROTOR_BODY.inertia, ROTOR_BODY.inertia]),
        }),
        linkBody,
      ];
  return [
    el("site", { name: names.imuSite, pos: vec(position), size: 0.012 }),
    el(
      "body",
      {
        name: `${joint.name}_rotor`,
        pos: vec(position),
        // With the motor hinge removed, the initial angle becomes a fixed rotation.
        axisangle: options.lockMotors ? vec([...joint.axis, joint.initialAngle]) : undefined,
      },
      rotorChildren,
    ),
  ];
}

function linkContents(
  spec: ArmSpec,
  index: number,
  joint: JointSpec,
  options: MjcfOptions,
): XmlElement[] {
  const { vector, radius, mass, flex } = joint.link;
  const origin: Vec3 = [0, 0, 0];
  if (!flex) {
    return [capsule(origin, vector, radius, mass), ...jointChain(spec, index + 1, vector, options)];
  }
  const half = scale(vector, 0.5);
  const names = jointElementNames(joint.name);
  const axes = bendingAxes(vector);
  return [
    capsule(origin, half, radius, mass / 2),
    el("body", { name: `${joint.name}_flex`, pos: vec(half) }, [
      ...axes.map((axis, k) =>
        el("joint", {
          name: names.bendJoints[k],
          type: "hinge",
          axis: vec(axis),
          stiffness: flex.stiffness,
          damping: flex.damping,
        }),
      ),
      capsule(origin, half, radius, mass / 2),
      ...jointChain(spec, index + 1, half, options),
    ]),
  ];
}

/**
 * Builds the MJCF model of one arm on a floor.
 *
 * @throws if the spec fails {@link validateArmSpec}.
 */
export function buildMjcf(spec: ArmSpec, options: MjcfOptions = {}): string {
  const problems = validateArmSpec(spec);
  if (problems.length > 0)
    throw new Error(`invalid arm spec "${spec.name}":\n- ${problems.join("\n- ")}`);

  const timestep = options.timestep ?? 1 / 4000;
  const actuators = options.lockMotors
    ? []
    : spec.joints.map((joint) => {
        const names = jointElementNames(joint.name);
        const peak = joint.actuator.peakTorque;
        return el("motor", {
          name: names.actuator,
          joint: names.motorJoint,
          gear: 1,
          ctrlrange: vec([-peak, peak]),
        });
      });
  const sensors = [
    ...spec.joints.flatMap((joint) => {
      const names = jointElementNames(joint.name);
      return [
        el("gyro", { name: names.gyroSensor, site: names.imuSite }),
        el("accelerometer", { name: names.accelSensor, site: names.imuSite }),
      ];
    }),
    el("framepos", { name: ARM_NAMES.tcpPosSensor, objtype: "site", objname: ARM_NAMES.tcpSite }),
    el("framelinvel", {
      name: ARM_NAMES.tcpVelSensor,
      objtype: "site",
      objname: ARM_NAMES.tcpSite,
    }),
  ];

  const model = el("mujoco", { model: spec.name }, [
    el("compiler", { angle: "radian", autolimits: "true" }),
    el("option", {
      timestep,
      integrator: "implicitfast",
      gravity: options.gravity === false ? "0 0 0" : undefined,
    }),
    el("worldbody", {}, [
      el("light", { pos: "0 -1 3", dir: "0 0.3 -1", directional: "true" }),
      el("geom", { name: "floor", type: "plane", size: "3 3 0.05", ...ENV_CONTACT }),
      ...(options.obstacle
        ? [
            el("body", { name: OBSTACLE.body, pos: vec(options.obstacle.position) }, [
              el("joint", {
                name: OBSTACLE.joint,
                type: "slide",
                axis: vec(options.obstacle.axis),
                stiffness: options.obstacle.stiffness,
                damping: 0.05 * Math.sqrt(options.obstacle.stiffness),
              }),
              el("geom", {
                type: "box",
                size: vec(options.obstacle.halfSize),
                mass: 2,
                ...ENV_CONTACT,
              }),
            ]),
          ]
        : []),
      el("body", { name: ARM_NAMES.base, pos: "0 0 0" }, [
        el("geom", {
          type: "cylinder",
          size: vec([0.07, BASE_HEIGHT / 2]),
          pos: vec([0, 0, BASE_HEIGHT / 2]),
        }),
        ...jointChain(spec, 0, [0, 0, BASE_HEIGHT], options),
      ]),
    ]),
    el("actuator", {}, actuators),
    el("sensor", {}, sensors),
  ]);
  return `${serialize(model)}\n`;
}
