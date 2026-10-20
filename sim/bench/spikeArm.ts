/**
 * Rough KX-6 arm for the NX-01 step-cost benchmark.
 *
 * Same size and joint count as the real plant (QDD motor + compliant gear stage per
 * joint, two bending joints in each long link), but hand-tuned and without a generator.
 * NX-02 replaces it with the ArmSpec → MJCF generator.
 */

/** Joint definition: axis in the parent frame, offset to the next joint, link mass. */
interface SpikeJoint {
  axis: string;
  length: number;
  mass: number;
  flex: boolean;
}

const JOINTS: readonly SpikeJoint[] = [
  { axis: "0 0 1", length: 0.16, mass: 2.0, flex: false },
  { axis: "0 1 0", length: 0.42, mass: 2.2, flex: true },
  { axis: "0 1 0", length: 0.39, mass: 1.6, flex: true },
  { axis: "0 1 0", length: 0.11, mass: 0.7, flex: false },
  { axis: "0 0 1", length: 0.1, mass: 0.6, flex: false },
  { axis: "0 1 0", length: 0.09, mass: 0.5, flex: false },
];

/** Reflected rotor inertia at the output for a 9:1 QDD, kg·m². */
const ARMATURE = 0.008;
/** Gear-stage stiffness and damping at the output. */
const GEAR = { stiffness: 2.0e4, damping: 4.0 };
/** Link bending stiffness and damping per axis. */
const FLEX = { stiffness: 3.0e3, damping: 0.6 };

function linkXml(prefix: string, i: number, depth: number): string {
  const joint = JOINTS[i];
  if (!joint) return `<site name="${prefix}tcp" pos="0 0 0"/>`;
  const name = `${prefix}j${i + 1}`;
  const pad = "  ".repeat(depth);
  const half = joint.length / 2;
  const flexBodies = joint.flex
    ? `
${pad}    <body name="${name}_flex" pos="0 0 ${half}">
${pad}      <joint name="${name}_bend_x" axis="1 0 0" stiffness="${FLEX.stiffness}" damping="${FLEX.damping}"/>
${pad}      <joint name="${name}_bend_y" axis="0 1 0" stiffness="${FLEX.stiffness}" damping="${FLEX.damping}"/>
${pad}      <geom type="capsule" fromto="0 0 0 0 0 ${half}" size="0.035" mass="${joint.mass / 2}"/>
${pad}      <body pos="0 0 ${half}">
${linkXml(prefix, i + 1, depth + 4)}
${pad}      </body>
${pad}    </body>`
    : `
${pad}    <body pos="0 0 ${joint.length}">
${linkXml(prefix, i + 1, depth + 3)}
${pad}    </body>`;
  return `${pad}<body name="${name}_rotor">
${pad}  <joint name="${name}_motor" axis="${joint.axis}" armature="${ARMATURE}" damping="0.02"/>
${pad}  <inertial pos="0 0 0" mass="0.2" diaginertia="1e-4 1e-4 1e-4"/>
${pad}  <body name="${name}_link">
${pad}    <joint name="${name}_gear" axis="${joint.axis}" stiffness="${GEAR.stiffness}" damping="${GEAR.damping}"/>
${pad}    <site name="${name}_imu" pos="0 0 0.02"/>
${pad}    <geom type="capsule" fromto="0 0 0 0 0 ${joint.flex ? half : joint.length}" size="0.04" mass="${joint.flex ? joint.mass / 2 : joint.mass}"/>${flexBodies}
${pad}  </body>
${pad}</body>`;
}

function sensorsXml(prefix: string): string {
  return JOINTS.map((_, i) => {
    const name = `${prefix}j${i + 1}`;
    return `    <gyro site="${name}_imu"/>
    <accelerometer site="${name}_imu"/>`;
  }).join("\n");
}

/** Returns MJCF for `arms` identical arms placed side by side, 0.25 ms timestep. */
export function spikeArmXml(arms: number): string {
  const prefixes = Array.from({ length: arms }, (_, a) => `a${a}_`);
  const bodies = prefixes
    .map(
      (p, a) => `    <body name="${p}base" pos="${a * 1.2} 0 0">
${linkXml(p, 0, 3)}
    </body>`,
    )
    .join("\n");
  const actuators = prefixes
    .flatMap((p) => JOINTS.map((_, i) => `    <motor joint="${p}j${i + 1}_motor" gear="1"/>`))
    .join("\n");
  return `<mujoco model="kx6-spike">
  <option timestep="0.00025" integrator="implicitfast"/>
  <worldbody>
    <geom type="plane" size="3 3 0.1"/>
${bodies}
  </worldbody>
  <actuator>
${actuators}
  </actuator>
  <sensor>
${prefixes.map(sensorsXml).join("\n")}
  </sensor>
</mujoco>
`;
}

/** Number of actuated joints per spike arm. */
export const SPIKE_JOINTS = JOINTS.length;
