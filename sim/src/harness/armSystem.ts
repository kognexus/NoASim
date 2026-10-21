/**
 * Composition root: one arm with its plant, actuator physics, sensors, nodes, bus and
 * host, advanced one 4 kHz bus cycle at a time.
 *
 * Each cycle:
 * 1. every joint's sensors sample the plant's ground truth;
 * 2. the bus delivers what is due to each node and to the host;
 * 3. each node ticks (in its own WebAssembly instance) and sends frames;
 * 4. the host ticks and sends frames;
 * 5. each actuator's physics turns its node's torque into plant forces;
 * 6. the plant steps.
 *
 * The node only ever receives its own sensor frame and its inbox.
 */
import { drawHiddenActuators, type DrawOptions } from "../actuators/families.ts";
import { type BusOptions, EthercatBus, HOST_PORT } from "../bus/ethercatBus.ts";
import { Host, type HostOptions } from "../host/host.ts";
import { BUS_CYCLE_HZ } from "../index.ts";
import type { NodeConfig } from "../node/nodeConfig.ts";
import { WasmNode } from "../node/wasmNode.ts";
import type { ArmSpec } from "../plant/armSpec.ts";
import { type ActuatorPhysics, IdealDrive } from "../plant/drive.ts";
import { KX6 } from "../plant/kx6.ts";
import type { ObstacleSpec } from "../plant/mjcf.ts";
import type { Mujoco } from "../plant/mujoco.ts";
import { Plant } from "../plant/plant.ts";
import { emptySensorFrame, JointSensors, type SensorFrame } from "../sensors/jointSensors.ts";
import { type ControlGains, DEFAULT_GAINS, nodeConfigsFor } from "./gains.ts";
import { impactThresholds } from "./impactThresholds.ts";
import { NEXUS_SENSORS, type NexusSensorSpec } from "../sensors/spec.ts";
import { Random } from "../util/random.ts";

export interface ArmSystemOptions {
  readonly spec?: ArmSpec;
  /** Seed for every random stream (sensor noise, bus, actuator draw). */
  readonly seed?: number;
  readonly bus?: BusOptions;
  /** `"ideal"` for datasheet actuators, or options for drawing hidden actuators. */
  readonly actuators?: "ideal" | DrawOptions;
  readonly sensors?: NexusSensorSpec;
  readonly host?: HostOptions;
  readonly gravity?: boolean;
  /** Node control gains (EEPROM). */
  readonly gains?: ControlGains;
  /**
   * Cycles between a node computing a torque and the motor applying it (EtherCAT
   * distributed clocks apply outputs at the next sync). Default 1.
   */
  readonly actuationDelayCycles?: number;
  /** Physics steps per bus cycle (finer integration of stiff gear springs). Default 1. */
  readonly physicsSubsteps?: number;
  /** A spring-mounted obstacle for impact tests. */
  readonly obstacle?: ObstacleSpec;
  /** Adjusts the generated EEPROM contents per joint (experiments, tuning). */
  readonly configure?: (configs: NodeConfig[]) => NodeConfig[];
}

/** Everything belonging to one joint. */
export interface JointRuntime {
  readonly index: number;
  readonly physics: ActuatorPhysics;
  readonly sensors: JointSensors;
  readonly node: WasmNode;
  readonly frame: SensorFrame;
  /** Current that flowed in the last cycle (what the current sensor measures next). */
  current: number;
  /** Torque the node commanded in the last cycle. */
  commandTorque: number;
  /** Commands waiting to be applied (actuation delay). */
  readonly pending: number[];
  /** The node asked for its brake. */
  brake: boolean;
}

export class ArmSystem {
  readonly plant: Plant;
  readonly bus: EthercatBus;
  readonly host: Host;
  readonly joints: readonly JointRuntime[];
  readonly #substeps: number;
  #cycle = 0;
  /**
   * The hardwired safe-torque-off line: once any node pulls it, every drive on the line is
   * de-energised, whatever the bus does. It stays pulled until {@link resetSafeTorqueOff}.
   */
  safeTorqueOff = false;

  private constructor(
    plant: Plant,
    bus: EthercatBus,
    host: Host,
    joints: JointRuntime[],
    substeps: number,
  ) {
    this.plant = plant;
    this.bus = bus;
    this.host = host;
    this.joints = joints;
    this.#substeps = substeps;
  }

  /** Builds and powers up the arm. Call {@link dispose} when done. */
  static async create(
    mujoco: Mujoco,
    nodeModule: WebAssembly.Module,
    options: ArmSystemOptions = {},
  ): Promise<ArmSystem> {
    const spec = options.spec ?? KX6;
    const rng = new Random(options.seed ?? 1);
    const hidden = options.actuators !== undefined && options.actuators !== "ideal";
    const physics: ActuatorPhysics[] = hidden
      ? drawHiddenActuators(spec, rng.derive("actuators"), options.actuators as DrawOptions)
      : spec.joints.map((j) => new IdealDrive(j.actuator));
    const substeps = Math.max(1, Math.round(options.physicsSubsteps ?? 1));
    const plant = Plant.create(mujoco, spec, {
      timestep: 1 / (BUS_CYCLE_HZ * substeps),
      externalFriction: hidden,
      ...(options.obstacle ? { obstacle: options.obstacle } : {}),
      ...(options.gravity === undefined ? {} : { gravity: options.gravity }),
    });
    const thresholds = impactThresholds(spec.name);
    const generated = nodeConfigsFor(spec, BUS_CYCLE_HZ, options.gains ?? DEFAULT_GAINS).map(
      (config, i) => ({
        ...config,
        impactThreshold: thresholds?.[i] ?? config.impactThreshold,
      }),
    );
    const configs = options.configure ? options.configure(generated) : generated;
    const joints = await Promise.all(
      spec.joints.map(async (joint, index): Promise<JointRuntime> => {
        const node = await WasmNode.instantiate(nodeModule);
        const config = configs[index];
        if (!config) throw new Error(`no node config for joint ${joint.name}`);
        node.init(config, index + 1);
        const sensors = new JointSensors(
          joint.actuator,
          options.sensors ?? NEXUS_SENSORS,
          BUS_CYCLE_HZ,
          rng.derive(`sensors/${joint.name}`),
        );
        const p = physics[index];
        if (!p) throw new Error(`no actuator physics for joint ${joint.name}`);
        const delay = Math.max(0, options.actuationDelayCycles ?? 1);
        return {
          index,
          physics: p,
          sensors,
          node,
          frame: emptySensorFrame(),
          current: 0,
          commandTorque: 0,
          pending: new Array<number>(delay).fill(0),
          brake: false,
        };
      }),
    );
    const bus = new EthercatBus(spec.joints.length, rng.derive("bus"), options.bus);
    const host = new Host(spec, rng.derive("host"), options.host);
    bus.send(HOST_PORT, host.start(bus.reachableNodes(), 0), 0);
    return new ArmSystem(plant, bus, host, joints, substeps);
  }

  /** Bus cycles run so far. */
  get cycle(): number {
    return this.#cycle;
  }

  /** Simulated time, s. */
  get time(): number {
    return this.#cycle / BUS_CYCLE_HZ;
  }

  /** Advances one bus cycle. */
  step(): void {
    const cycle = this.#cycle;
    const plant = this.plant;
    const dt = plant.timestep * this.#substeps;
    for (const j of this.joints) {
      j.sensors.sample(
        {
          motorAngle: plant.motorAngle(j.index),
          outputAngle: plant.outputAngle(j.index),
          gyro: plant.gyro(j.index),
          accel: plant.accelerometer(j.index),
          current: j.current,
          windingTemperature: j.physics.windingTemperature,
        },
        j.frame,
      );
      const position = j.index + 1;
      const out = j.node.tick(cycle, j.frame, this.bus.receive(position, cycle));
      this.bus.send(position, out.outbox, cycle);
      j.pending.push(out.torque);
      j.commandTorque = j.pending.shift() ?? out.torque;
      j.brake = out.brake;
      if (out.safeTorqueOff) this.safeTorqueOff = true;
    }
    this.bus.send(HOST_PORT, this.host.tick(cycle, this.bus.receive(HOST_PORT, cycle)), cycle);
    for (const j of this.joints) {
      const actuator = plant.spec.joints[j.index]?.actuator;
      const command = this.safeTorqueOff ? 0 : j.commandTorque;
      const out = j.physics.update(command, plant.actuatorState(j.index), dt);
      j.current = out.current;
      plant.setMotorTorque(j.index, this.safeTorqueOff ? 0 : out.driveTorque);
      plant.setActuatorForces(j.index, out.frictionTorque, out.gearTorque);
      if (actuator?.brake) plant.setBrake(j.index, j.brake);
    }
    for (let k = 0; k < this.#substeps; k++) plant.step();
    this.#cycle++;
  }

  /** Releases the safe-torque-off line (a deliberate reset after a stop). */
  resetSafeTorqueOff(): void {
    this.safeTorqueOff = false;
  }

  /** Advances `seconds` of simulated time. */
  run(seconds: number): void {
    const cycles = Math.round(seconds * BUS_CYCLE_HZ);
    for (let k = 0; k < cycles; k++) this.step();
  }

  dispose(): void {
    this.plant.dispose();
  }
}
