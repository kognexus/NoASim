/**
 * NX-01 benchmark core: cost of one 4 kHz bus cycle (plant step + one node tick per
 * joint). Environment-agnostic, so the same loop runs in Node (`stepCost.ts`) and in the
 * browser (`web/`).
 */
import type { MainModule } from "@mujoco/mujoco";
import { WasmNode } from "../src/node/wasmNode.ts";
import { BUS_CYCLE_HZ } from "../src/index.ts";
import { SPIKE_JOINTS, spikeArmXml } from "./spikeArm.ts";

/** Runs `simSeconds` of simulated time and returns simulated steps per wall-clock second. */
export async function measureStepRate(
  mujoco: MainModule,
  nodeModule: WebAssembly.Module,
  arms: number,
  withNodes: boolean,
  simSeconds = 5,
): Promise<number> {
  const model = mujoco.MjModel.from_xml_string(spikeArmXml(arms));
  const data = new mujoco.MjData(model);
  const qpos: Float64Array = data.qpos;
  const qvel: Float64Array = data.qvel;
  const ctrl: Float64Array = data.ctrl;

  const joints: { node: WasmNode; qposAdr: number; dofAdr: number }[] = [];
  for (let a = 0; a < arms; a++) {
    for (let j = 0; j < SPIKE_JOINTS; j++) {
      const joint = model.jnt(`a${a}_j${j + 1}_motor`);
      const node = await WasmNode.instantiate(nodeModule);
      node.init(j + 1, j === 1 ? 1.2 : 0.3, 400, 8);
      joints.push({ node, qposAdr: Number(joint.qposadr[0]), dofAdr: Number(joint.dofadr[0]) });
      joint.delete();
    }
  }

  const steps = simSeconds * BUS_CYCLE_HZ;
  const start = performance.now();
  for (let s = 0; s < steps; s++) {
    if (withNodes) {
      joints.forEach((j, i) => {
        ctrl[i] = j.node.tick({
          motorAngle: qpos[j.qposAdr] ?? 0,
          motorVelocity: qvel[j.dofAdr] ?? 0,
        });
      });
    }
    mujoco.mj_step(model, data);
  }
  const seconds = (performance.now() - start) / 1000;
  const finite = Array.from(qpos).every(Number.isFinite);
  data.delete();
  model.delete();
  if (!finite) throw new Error(`simulation diverged (${arms} arms, nodes=${withNodes})`);
  return steps / seconds;
}
