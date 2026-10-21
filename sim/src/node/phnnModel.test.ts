import { readdir, readFile } from "node:fs/promises";
import { beforeAll, describe, expect, it } from "vitest";
import { nodeConfigsFor } from "../harness/gains.ts";
import { KX6 } from "../plant/kx6.ts";
import { loadNodeModule } from "../testing/fixtures.ts";
import { packPhnnModel, type PhnnDocument } from "./phnnModel.ts";
import { WasmNode } from "./wasmNode.ts";

const MODELS = new URL("../../../models/", import.meta.url);
const CONFIGS = nodeConfigsFor(KX6, 4000);

/** Every committed model, by unit. */
async function committedModels(): Promise<PhnnDocument[]> {
  const units = await readdir(MODELS, { withFileTypes: true });
  const documents: PhnnDocument[] = [];
  for (const unit of units.filter((u) => u.isDirectory())) {
    for (const file of await readdir(new URL(`${unit.name}/`, MODELS))) {
      // Learned joint models only (`models/coordinated-damping/` holds NX-30's designs).
      if (/^joint\d+\.json$/.test(file)) {
        const text = await readFile(new URL(`${unit.name}/${file}`, MODELS), "utf8");
        documents.push(JSON.parse(text) as PhnnDocument);
      }
    }
  }
  return documents;
}

describe("learned models in the node (NX-15)", () => {
  let module: WebAssembly.Module;
  let models: PhnnDocument[];
  beforeAll(async () => {
    [module, models] = await Promise.all([loadNodeModule(), committedModels()]);
  });

  async function nodeAt(joint: number): Promise<WasmNode> {
    const node = await WasmNode.instantiate(module);
    node.init(CONFIGS[joint - 1]!, joint);
    return node;
  }

  it("evaluates every committed model as the trainer does", async () => {
    expect(models.length).toBeGreaterThan(0);
    for (const doc of models) {
      const node = await nodeAt(doc.joint);
      node.loadModel(packPhnnModel(doc));
      expect(doc.probes?.length).toBeGreaterThan(0);
      for (const p of doc.probes ?? []) {
        const residual = node.modelResidual({
          angle: p.theta,
          velocity: p.omega,
          acceleration: p.alpha,
          twist: p.phi,
          twistRate: p.phi_rate,
          applied: p.u,
          temperature: p.temperature,
        });
        // The node runs in f32; the trainer in f64.
        const tolerance = 1e-4 * doc.constants.ratedTorque + 1e-4 * Math.abs(p.unexplained);
        expect(Math.abs(residual - p.unexplained), `${doc.unit} joint ${doc.joint}`).toBeLessThan(
          tolerance,
        );
      }
    }
  });

  it("refuses a model for another joint, and has none until one is loaded", async () => {
    const doc = models[0]!;
    const other = doc.joint === 1 ? 2 : 1;
    const node = await nodeAt(other);
    expect(
      Number.isNaN(
        node.modelResidual({
          angle: 0,
          velocity: 0,
          acceleration: 0,
          twist: 0,
          twistRate: 0,
          applied: 0,
          temperature: 25,
        }),
      ),
    ).toBe(true);
    expect(() => node.loadModel(packPhnnModel(doc))).toThrow(/wrong joint/);
  });
});
