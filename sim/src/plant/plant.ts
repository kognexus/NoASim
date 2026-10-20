/**
 * The simulated arm: a MuJoCo model built from an {@link ArmSpec}, with named access to
 * the physical quantities the sensor models and metrics need.
 *
 * `Plant` exposes ground truth. Nodes never see it directly: the sensor models (NX-03)
 * read it and add the imperfections of real sensors.
 */
import type { MjData, MjModel } from "@mujoco/mujoco";
import type { ArmSpec, Vec3 } from "./armSpec.ts";
import { buildMjcf, type MjcfOptions } from "./mjcf.ts";
import type { Mujoco } from "./mujoco.ts";
import { ARM_NAMES, jointElementNames } from "./names.ts";

/** MuJoCo addresses of one joint's elements; -1 where an element is absent. */
interface JointAddresses {
  readonly motorQpos: number;
  readonly motorDof: number;
  readonly gearQpos: number;
  readonly gearDof: number;
  readonly bendQpos: readonly number[];
  readonly actuator: number;
  readonly gyro: number;
  readonly accel: number;
  readonly linkBody: number;
}

/** Named-access fields are numbers for scalars in some binding versions and arrays in others. */
function scalar(value: unknown): number {
  const n = typeof value === "number" ? value : Number((value as ArrayLike<number>)[0]);
  if (!Number.isInteger(n))
    throw new Error(`expected an integer MuJoCo address, got ${String(value)}`);
  return n;
}

export class Plant {
  readonly spec: ArmSpec;
  readonly #mujoco: Mujoco;
  readonly #model: MjModel;
  readonly #data: MjData;
  readonly #joints: readonly JointAddresses[];
  readonly #tcpPos: number;
  readonly #tcpVel: number;
  readonly #toolBody: number;
  #disposed = false;

  private constructor(mujoco: Mujoco, spec: ArmSpec, options: MjcfOptions) {
    this.#mujoco = mujoco;
    this.spec = spec;
    this.#model = mujoco.MjModel.from_xml_string(buildMjcf(spec, options));
    this.#data = new mujoco.MjData(this.#model);
    this.#joints = spec.joints.map((joint) => this.#addresses(joint.name, options));
    this.#tcpPos = this.#sensorAdr(ARM_NAMES.tcpPosSensor);
    this.#tcpVel = this.#sensorAdr(ARM_NAMES.tcpVelSensor);
    const last = spec.joints.at(-1);
    this.#toolBody =
      spec.payload.mass > 0
        ? this.#id(mujoco.mjtObj.mjOBJ_BODY.value, ARM_NAMES.payloadBody)
        : this.#id(mujoco.mjtObj.mjOBJ_BODY.value, jointElementNames(last?.name ?? "").linkBody);
    this.reset();
  }

  /** Builds the plant for `spec`. Call {@link dispose} when done to free WebAssembly memory. */
  static create(mujoco: Mujoco, spec: ArmSpec, options: MjcfOptions = {}): Plant {
    return new Plant(mujoco, spec, options);
  }

  /** Physics timestep, s. */
  get timestep(): number {
    return this.#model.opt.timestep;
  }

  /** Simulated time since the last reset, s. */
  get time(): number {
    return this.#data.time;
  }

  get jointCount(): number {
    return this.#joints.length;
  }

  /** Puts every joint at its initial angle, at rest, and clears all inputs. */
  reset(): void {
    this.#mujoco.mj_resetData(this.#model, this.#data);
    const qpos = this.#data.qpos as Float64Array;
    this.spec.joints.forEach((joint, i) => {
      // With locked motors the initial angle is built into the model (see buildMjcf).
      const adr = this.#joint(i).motorQpos;
      if (adr >= 0) qpos[adr] = joint.initialAngle;
    });
    this.#mujoco.mj_forward(this.#model, this.#data);
  }

  /** Advances the physics by one timestep. */
  step(): void {
    this.#mujoco.mj_step(this.#model, this.#data);
  }

  /** Sets the motor torque of joint `i` (output-referred, N·m). MuJoCo clamps it to ±peak. */
  setMotorTorque(i: number, torque: number): void {
    const adr = this.#joint(i).actuator;
    if (adr < 0) throw new Error("motors are locked in this plant");
    (this.#data.ctrl as Float64Array)[adr] = torque;
  }

  /** Motor-side angle of joint `i` referred to the output, rad. */
  motorAngle(i: number): number {
    return this.#qpos(this.#joint(i).motorQpos);
  }

  /** Motor-side velocity of joint `i` referred to the output, rad/s. */
  motorVelocity(i: number): number {
    return this.#qvel(this.#joint(i).motorDof);
  }

  /** Twist of the gear stage of joint `i` (output minus motor side), rad. */
  gearTwist(i: number): number {
    return this.#qpos(this.#joint(i).gearQpos);
  }

  /** Angle of joint `i`'s output relative to its housing, rad. */
  outputAngle(i: number): number {
    return this.motorAngle(i) + this.gearTwist(i);
  }

  /** Velocity of joint `i`'s output relative to its housing, rad/s. */
  outputVelocity(i: number): number {
    return this.motorVelocity(i) + this.#qvel(this.#joint(i).gearDof);
  }

  /** Bending angles of link `i` (two axes), rad; empty for a rigid link. */
  linkBend(i: number): number[] {
    return this.#joint(i).bendQpos.map((adr) => this.#qpos(adr));
  }

  /** Angular velocity of joint `i`'s housing in the IMU frame, rad/s (ideal gyro). */
  gyro(i: number): Vec3 {
    return this.#sensor3(this.#joint(i).gyro);
  }

  /** Specific force at joint `i`'s housing in the IMU frame, m/s² (ideal accelerometer). */
  accelerometer(i: number): Vec3 {
    return this.#sensor3(this.#joint(i).accel);
  }

  /** Position of the tool centre point in the world frame, m. */
  tcpPosition(): Vec3 {
    return this.#sensor3(this.#tcpPos);
  }

  /** Velocity of the tool centre point in the world frame, m/s. */
  tcpVelocity(): Vec3 {
    return this.#sensor3(this.#tcpVel);
  }

  /** Applies an external force (world frame, N) at the tool body's centre of mass until changed. */
  setToolForce(force: Vec3): void {
    const xfrc = this.#data.xfrc_applied as Float64Array;
    xfrc.set(force, this.#toolBody * 6);
  }

  /** Frees the MuJoCo model and data. The plant must not be used afterwards. */
  dispose(): void {
    if (this.#disposed) return;
    this.#disposed = true;
    this.#data.delete();
    this.#model.delete();
  }

  #joint(i: number): JointAddresses {
    const adr = this.#joints[i];
    if (!adr) throw new RangeError(`joint index ${i} out of range`);
    return adr;
  }

  #qpos(adr: number): number {
    return adr < 0 ? 0 : ((this.#data.qpos as Float64Array)[adr] ?? 0);
  }

  #qvel(adr: number): number {
    return adr < 0 ? 0 : ((this.#data.qvel as Float64Array)[adr] ?? 0);
  }

  #sensor3(adr: number): Vec3 {
    const data = this.#data.sensordata as Float64Array;
    return [data[adr] ?? 0, data[adr + 1] ?? 0, data[adr + 2] ?? 0];
  }

  #id(type: number, name: string): number {
    const id = this.#mujoco.mj_name2id(this.#model, type, name);
    if (id < 0) throw new Error(`MuJoCo element "${name}" not found`);
    return id;
  }

  #sensorAdr(name: string): number {
    const sensor = this.#model.sensor(name);
    const adr = scalar(sensor.adr);
    sensor.delete();
    return adr;
  }

  #jointAdr(name: string): { qpos: number; dof: number } {
    const joint = this.#model.jnt(name);
    const result = { qpos: scalar(joint.qposadr), dof: scalar(joint.dofadr) };
    joint.delete();
    return result;
  }

  #addresses(jointName: string, options: MjcfOptions): JointAddresses {
    const names = jointElementNames(jointName);
    const { mjOBJ_ACTUATOR, mjOBJ_BODY, mjOBJ_JOINT } = this.#mujoco.mjtObj;
    const motor = options.lockMotors ? { qpos: -1, dof: -1 } : this.#jointAdr(names.motorJoint);
    const gear = this.#jointAdr(names.gearJoint);
    const bendQpos = names.bendJoints
      .filter((name) => this.#mujoco.mj_name2id(this.#model, mjOBJ_JOINT.value, name) >= 0)
      .map((name) => this.#jointAdr(name).qpos);
    return {
      motorQpos: motor.qpos,
      motorDof: motor.dof,
      gearQpos: gear.qpos,
      gearDof: gear.dof,
      bendQpos,
      actuator: options.lockMotors ? -1 : this.#id(mjOBJ_ACTUATOR.value, names.actuator),
      gyro: this.#sensorAdr(names.gyroSensor),
      accel: this.#sensorAdr(names.accelSensor),
      linkBody: this.#id(mjOBJ_BODY.value, names.linkBody),
    };
  }
}
