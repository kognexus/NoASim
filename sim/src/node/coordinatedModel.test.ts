import { readFile } from "node:fs/promises";
import { beforeAll, describe, expect, it } from "vitest";
import { DampingMode } from "../bus/protocol.ts";
import { ArmSystem } from "../harness/armSystem.ts";
import { nodeConfigsFor } from "../harness/gains.ts";
import { tunedGains } from "../harness/tuned.ts";
import { KX6_HOME, KX6L, withInitialAngles } from "../plant/kx6.ts";
import { loadMujoco, type Mujoco } from "../plant/mujoco.ts";
import { homePose } from "../scenarios/motion.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import {
  type CoordinatedModelDocument,
  coordinatedInertialTorque,
  packCoordinatedModel,
} from "./coordinatedModel.ts";
import { WasmNode } from "./wasmNode.ts";

const MODELS = new URL("../../../models/coordinated-models/", import.meta.url);

async function committed(dir: string): Promise<CoordinatedModelDocument[]> {
  return Promise.all(
    [1, 2, 3, 4, 5, 6].map(
      async (j) =>
        JSON.parse(
          await readFile(new URL(`${dir}/joint${j}.json`, MODELS), "utf8"),
        ) as CoordinatedModelDocument,
    ),
  );
}

describe("coordinated models in the node (NX-31)", () => {
  let mujoco: Mujoco;
  let module: WebAssembly.Module;
  let light: CoordinatedModelDocument[];
  beforeAll(async () => {
    [mujoco, module, light] = await Promise.all([
      loadMujoco(),
      loadNodeModule(),
      committed("kx6l-reach2"),
    ]);
  });

  it("evaluates every committed model as the trainer does", async () => {
    for (const dir of ["kx6l-reach0", "kx6l-reach2", "kx6-reach0", "kx6-reach2"]) {
      for (const doc of await committed(dir)) {
        expect(doc.neighbours).toContain(doc.joint);
        for (const p of doc.probes) {
          const torque = coordinatedInertialTorque(doc, p.angles, p.accelerations);
          expect(torque).toBeCloseTo(p.inertialTorque, 3);
        }
      }
    }
  });

  it("refuses a model for another joint", async () => {
    const doc = light[1];
    if (!doc) throw new Error("no model for joint 2");
    const node = await WasmNode.instantiate(module);
    const configs = nodeConfigsFor(KX6L, 4000);
    node.init(configs[2] ?? configs[0]!, 3);
    expect(() => node.loadCoordinatedModel(packCoordinatedModel(doc))).toThrow(/wrong joint/);
  });

  it("takes most of the light arm's sag out of a hold, learned from the neighbours' gyros", async () => {
    const offset = async (on: boolean) => {
      const system = await ArmSystem.create(mujoco, module, {
        spec: withInitialAngles(KX6L, KX6_HOME),
        gains: tunedGains("kx6l", "network"),
        seed: 32,
        coordinatedModels: light,
      });
      try {
        system.run(3);
        system.host.applySettings({ damping: DampingMode.network, coordinatedModel: on ? 1 : 0 });
        system.run(2);
        const p = system.plant.tcpPosition();
        const q = homePose(system).position;
        return Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
      } finally {
        system.dispose();
      }
    };
    const [without, withModel] = [await offset(false), await offset(true)];
    expect(without).toBeGreaterThan(1e-3);
    expect(withModel).toBeLessThan(0.4 * without);
  });
});
