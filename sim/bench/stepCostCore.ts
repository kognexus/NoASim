/**
 * Step-cost benchmark core: cost of one 4 kHz bus cycle (plant step plus one node tick per
 * joint) for the KX-6 plant. Environment-agnostic, so the same loop runs in Node
 * (`stepCost.ts`) and in the browser (`web/`).
 */
import { WasmNode } from "../src/node/wasmNode.ts";
import { BUS_CYCLE_HZ } from "../src/index.ts";
import { KX6 } from "../src/plant/kx6.ts";
import type { Mujoco } from "../src/plant/mujoco.ts";
import { Plant } from "../src/plant/plant.ts";

/** Runs `simSeconds` of simulated time and returns simulated steps per wall-clock second. */
export async function measureStepRate(
  mujoco: Mujoco,
  nodeModule: WebAssembly.Module,
  arms: number,
  withNodes: boolean,
  simSeconds = 5,
): Promise<number> {
  const plants = Array.from({ length: arms }, () => Plant.create(mujoco, KX6));
  const nodes = await Promise.all(
    plants.map((plant) =>
      Promise.all(
        KX6.joints.map(async (joint, i) => {
          const node = await WasmNode.instantiate(nodeModule);
          node.init(i + 1, joint.initialAngle, 3000, 60);
          return { node, plant, i };
        }),
      ),
    ),
  ).then((perArm) => perArm.flat());

  const steps = simSeconds * BUS_CYCLE_HZ;
  const start = performance.now();
  for (let s = 0; s < steps; s++) {
    if (withNodes) {
      for (const { node, plant, i } of nodes) {
        plant.setMotorTorque(
          i,
          node.tick({ motorAngle: plant.motorAngle(i), motorVelocity: plant.motorVelocity(i) }),
        );
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
