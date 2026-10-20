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
import { nodeConfigFor } from "../node/nodeConfig.ts";
import { WasmNode } from "../node/wasmNode.ts";
import type { ArmSpec } from "../plant/armSpec.ts";
import { type ActuatorPhysics, IdealDrive } from "../plant/drive.ts";
import { KX6 } from "../plant/kx6.ts";
import type { Mujoco } from "../plant/mujoco.ts";
import { Plant } from "../plant/plant.ts";
import { emptySensorFrame, JointSensors, type SensorFrame } from "../sensors/jointSensors.ts";
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
}

export class ArmSystem {
  readonly plant: Plant;
  readonly bus: EthercatBus;
  readonly host: Host;
  readonly joints: readonly JointRuntime[];
  #cycle = 0;

  private constructor(plant: Plant, bus: EthercatBus, host: Host, joints: JointRuntime[]) {
    this.plant = plant;
    this.bus = bus;
    this.host = host;
    this.joints = joints;
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
    const plant = Plant.create(mujoco, spec, {
      externalFriction: hidden,
      ...(options.gravity === undefined ? {} : { gravity: options.gravity }),
    });
    const joints = await Promise.all(
      spec.joints.map(async (joint, index): Promise<JointRuntime> => {
        const node = await WasmNode.instantiate(nodeModule);
        node.init(nodeConfigFor(joint, BUS_CYCLE_HZ), index + 1);
        const sensors = new JointSensors(
          joint.actuator,
          options.sensors ?? NEXUS_SENSORS,
          BUS_CYCLE_HZ,
          rng.derive(`sensors/${joint.name}`),
        );
        const p = physics[index];
        if (!p) throw new Error(`no actuator physics for joint ${joint.name}`);
        return {
          index,
          physics: p,
          sensors,
          node,
          frame: emptySensorFrame(),
          current: 0,
          commandTorque: 0,
        };
      }),
    );
    const bus = new EthercatBus(spec.joints.length, rng.derive("bus"), options.bus);
    const host = new Host(options.host);
    bus.send(HOST_PORT, host.start(bus.reachableNodes(), 0), 0);
    return new ArmSystem(plant, bus, host, joints);
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
    const dt = plant.timestep;
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
      const { torque, outbox } = j.node.tick(cycle, j.frame, this.bus.receive(position, cycle));
      this.bus.send(position, outbox, cycle);
      j.commandTorque = torque;
    }
    this.bus.send(HOST_PORT, this.host.tick(cycle, this.bus.receive(HOST_PORT, cycle)), cycle);
    for (const j of this.joints) {
      const out = j.physics.update(j.commandTorque, plant.actuatorState(j.index), dt);
      j.current = out.current;
      plant.setMotorTorque(j.index, out.driveTorque);
      plant.setActuatorForces(j.index, out.frictionTorque, out.gearTorque);
    }
    plant.step();
    this.#cycle++;
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
