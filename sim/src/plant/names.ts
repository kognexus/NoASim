/**
 * Names of the MuJoCo elements generated for an arm.
 *
 * The MJCF builder and every reader of the model take names from here, so the two
 * cannot disagree.
 */

/** Element names belonging to one joint. */
export interface JointElementNames {
  /** Motor-side hinge (rotor reflected through the gear). */
  readonly motorJoint: string;
  /** Gear-stage hinge in series with the motor hinge (compliance of the gear). */
  readonly gearJoint: string;
  /** Torque actuator on the motor hinge. */
  readonly actuator: string;
  /** Site of the actuator board, on the actuator housing (parent side of the joint). */
  readonly imuSite: string;
  readonly gyroSensor: string;
  readonly accelSensor: string;
  /** Link bending hinges, present only on flexible links. */
  readonly bendJoints: readonly [string, string];
  readonly linkBody: string;
}

export function jointElementNames(joint: string): JointElementNames {
  return {
    motorJoint: `${joint}_motor`,
    gearJoint: `${joint}_gear`,
    actuator: joint,
    imuSite: `${joint}_imu`,
    gyroSensor: `${joint}_gyro`,
    accelSensor: `${joint}_accel`,
    bendJoints: [`${joint}_bend_a`, `${joint}_bend_b`],
    linkBody: `${joint}_link`,
  };
}

/** Arm-level element names. */
export const ARM_NAMES = {
  base: "base",
  tcpSite: "tcp",
  tcpPosSensor: "tcp_pos",
  tcpVelSensor: "tcp_vel",
  payloadBody: "payload",
} as const;
