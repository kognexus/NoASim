/**
 * Step-cost benchmark core: cost of one 4 kHz bus cycle for the KX-6 plant. With nodes,
 * each cycle includes the sensor models, one node tick per joint and the drive physics.
 * Environment-agnostic, so the same loop runs in Node (`stepCost.ts`) and in the browser
 * (`web/`).
 */
import { BUS_CYCLE_HZ } from "../src/index.ts";
import { nodeConfigsFor } from "../src/harness/gains.ts";
import { WasmNode } from "../src/node/wasmNode.ts";
import { IdealDrive } from "../src/plant/drive.ts";
import { KX6 } from "../src/plant/kx6.ts";
import type { Mujoco } from "../src/plant/mujoco.ts";
import { Plant } from "../src/plant/plant.ts";
import { emptySensorFrame, JointSensors } from "../src/sensors/jointSensors.ts";
import { NEXUS_SENSORS } from "../src/sensors/spec.ts";
import { Random } from "../src/util/random.ts";

const NO_FRAMES = new Uint8Array();

/** Runs `simSeconds` of simulated time and returns simulated steps per wall-clock second. */
export async function measureStepRate(
  mujoco: Mujoco,
  nodeModule: WebAssembly.Module,
  arms: number,
  withNodes: boolean,
  simSeconds = 5,
): Promise<number> {
  const plants = Array.from({ length: arms }, () => Plant.create(mujoco, KX6));
  const configs = nodeConfigsFor(KX6, BUS_CYCLE_HZ);
  const joints = await Promise.all(
    plants.flatMap((plant, a) =>
      KX6.joints.map(async (joint, i) => {
        const node = await WasmNode.instantiate(nodeModule);
        const config = configs[i];
        if (!config) throw new Error(`no config for joint ${joint.name}`);
        node.init(config, i + 1);
        const rng = new Random(a).derive(joint.name);
        const sensors = new JointSensors(joint.actuator, NEXUS_SENSORS, BUS_CYCLE_HZ, rng);
        return {
          node,
          plant,
          i,
          sensors,
          drive: new IdealDrive(joint.actuator),
          frame: emptySensorFrame(),
        };
      }),
    ),
  );

  const steps = simSeconds * BUS_CYCLE_HZ;
  const dt = 1 / BUS_CYCLE_HZ;
  let current = 0;
  const start = performance.now();
  for (let s = 0; s < steps; s++) {
    if (withNodes) {
      for (const j of joints) {
        j.sensors.sample(
          {
            motorAngle: j.plant.motorAngle(j.i),
            outputAngle: j.plant.outputAngle(j.i),
            gyro: j.plant.gyro(j.i),
            accel: j.plant.accelerometer(j.i),
            current,
            windingTemperature: j.drive.windingTemperature,
          },
          j.frame,
        );
        const { torque } = j.node.tick(s, j.frame, NO_FRAMES);
        const out = j.drive.update(torque, j.plant.actuatorState(j.i), dt);
        current = out.current;
        j.plant.setMotorTorque(j.i, out.driveTorque);
      }
    }
    for (const plant of plants) plant.step();
  }
  const seconds = (performance.now() - start) / 1000;
  const finite = plants.every((plant) => plant.tcpPosition().every(Number.isFinite));
  plants.forEach((plant) => plant.dispose());
  if (!finite) throw new Error(`simulation diverged (${arms} arms, nodes=${withNodes})`);
  return steps / seconds;
}
