/**
 * KX-6: the default lightweight 6-DOF arm with quasi-direct-drive joints.
 *
 * Proportions follow a UR5e-class arm (shoulder 0.16 m, upper arm 0.42 m, forearm
 * 0.39 m, three short wrist links), lighter and with 9:1 QDD actuators. The upper arm
 * and forearm bend; their stiffness and damping are tuned so the first structural mode
 * of the stretched arm sits in the 8–15 Hz band with 1–3 % damping, as expected for
 * lightweight arms (PLAN §4, NX-02).
 */
import type { ActuatorSpec, ArmSpec, ThermalSpec } from "./armSpec.ts";

/**
 * Thermal network of a ~85 mm actuator: about 1.9 K/W winding to ambient and a time
 * constant of several minutes, so rated torque settles near 110 °C in 25 °C air.
 */
const THERMAL_LARGE: ThermalSpec = {
  windingToHousing: 0.7,
  housingToAmbient: 1.2,
  windingCapacity: 180,
  housingCapacity: 650,
  maxWindingTemperature: 130,
};

/** Shoulder and elbow class: 9:1, high-pole SPM motor. */
const LARGE: ActuatorSpec = {
  gearRatio: 9,
  rotorInertia: 2.0e-4,
  gearStiffness: 6.0e4,
  gearDamping: 40,
  viscousFriction: 0.3,
  coulombFriction: 0.4,
  ratedTorque: 30,
  peakTorque: 90,
  brake: false,
  // 30-slot / 32-pole SPM from the motor study: 3.3 N·m at 10.5 A rated.
  electrical: {
    torqueConstant: 0.316,
    polePairs: 16,
    phaseResistance: 0.2,
    thermal: THERMAL_LARGE,
  },
};

/** Shoulder pitch carries the whole arm, so it gets the larger motor. */
const SHOULDER: ActuatorSpec = {
  ...LARGE,
  rotorInertia: 3.5e-4,
  ratedTorque: 60,
  peakTorque: 160,
  electrical: {
    torqueConstant: 0.63,
    polePairs: 16,
    phaseResistance: 0.24,
    thermal: {
      ...THERMAL_LARGE,
      windingCapacity: 300,
      housingCapacity: 1000,
      housingToAmbient: 0.9,
    },
  },
};

/** Wrist class: 9:1, smaller motor. */
const WRIST: ActuatorSpec = {
  gearRatio: 9,
  rotorInertia: 0.6e-4,
  gearStiffness: 6.0e3,
  gearDamping: 0.6,
  viscousFriction: 0.08,
  coulombFriction: 0.12,
  ratedTorque: 10,
  peakTorque: 30,
  brake: false,
  electrical: {
    torqueConstant: 0.15,
    polePairs: 14,
    phaseResistance: 0.3,
    thermal: {
      windingToHousing: 0.8,
      housingToAmbient: 1.8,
      windingCapacity: 60,
      housingCapacity: 220,
      maxWindingTemperature: 130,
    },
  },
};

export const KX6: ArmSpec = {
  name: "KX-6",
  joints: [
    {
      name: "j1",
      axis: [0, 0, 1],
      initialAngle: 0,
      actuator: LARGE,
      link: { vector: [0, 0, 0.16], mass: 2.0, radius: 0.05 },
    },
    {
      name: "j2",
      axis: [0, 1, 0],
      initialAngle: Math.PI / 2,
      actuator: SHOULDER,
      link: {
        vector: [0, 0, 0.42],
        mass: 2.4,
        radius: 0.045,
        flex: { stiffness: 1.2e5, damping: 80 },
      },
    },
    {
      name: "j3",
      axis: [0, 1, 0],
      initialAngle: 0,
      actuator: LARGE,
      link: {
        vector: [0, 0, 0.39],
        mass: 1.6,
        radius: 0.04,
        flex: { stiffness: 6.0e4, damping: 40 },
      },
    },
    {
      name: "j4",
      axis: [0, 1, 0],
      initialAngle: 0,
      actuator: WRIST,
      link: { vector: [0, 0, 0.11], mass: 0.8, radius: 0.035 },
    },
    {
      name: "j5",
      axis: [0, 0, 1],
      initialAngle: 0,
      actuator: WRIST,
      link: { vector: [0, 0, 0.1], mass: 0.7, radius: 0.035 },
    },
    {
      name: "j6",
      axis: [0, 1, 0],
      initialAngle: 0,
      actuator: WRIST,
      link: { vector: [0, 0, 0.09], mass: 0.5, radius: 0.03 },
    },
  ],
  payload: { mass: 3.0, offset: [0, 0, 0.05] },
};

/**
 * Working pose for motion tasks: tool pointing down at (0.60, 0, 0.25) m. The stretched
 * start pose of {@link KX6} is the worst case for vibration, but a singularity for
 * reaching.
 */
export const KX6_HOME: readonly number[] = [0, 0.527, 1.102, 1.512, 0, 0];

/** `spec` starting at `angles` instead of its own initial pose. */
export function withInitialAngles(spec: ArmSpec, angles: readonly number[]): ArmSpec {
  return {
    ...spec,
    joints: spec.joints.map((joint, i) => ({
      ...joint,
      initialAngle: angles[i] ?? joint.initialAngle,
    })),
  };
}

/**
 * KX-6L: the same joints on lightweight, flexible links (thin carbon tubes): about 60 %
 * of the link mass and a tenth of the bending stiffness. The first structural mode with
 * the motors locked drops to about 6 Hz with about 1 % damping. This is the lightweight-arm
 * case the network is meant for (ADR-0003); KX-6 is the stiff reference.
 */
export const KX6L: ArmSpec = {
  ...KX6,
  name: "KX-6L",
  joints: KX6.joints.map((joint) => ({
    ...joint,
    link: {
      ...joint.link,
      mass: joint.link.mass * 0.6,
      ...(joint.link.flex
        ? {
            flex: {
              stiffness: joint.link.flex.stiffness * 0.1,
              damping: joint.link.flex.damping * 0.1,
            },
          }
        : {}),
    },
  })),
};
