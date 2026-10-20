import { readFile } from "node:fs/promises";
import { beforeAll, describe, expect, it } from "vitest";
import { WasmNode } from "./wasmNode.ts";

const WASM_PATH = new URL(
  "../../../target/wasm32-unknown-unknown/release/nexus_wasm.wasm",
  import.meta.url,
);

describe("WasmNode", () => {
  let module: WebAssembly.Module;

  beforeAll(async () => {
    module = await WebAssembly.compile(await readFile(WASM_PATH));
  });

  it("computes the command inside the node", async () => {
    const node = await WasmNode.instantiate(module);
    node.init(1, 1.0, 100.0, 2.0);
    expect(node.tick({ motorAngle: 0.5, motorVelocity: 1.0 })).toBeCloseTo(48.0, 5);
  });

  it("gives every joint its own memory", async () => {
    const a = await WasmNode.instantiate(module);
    const b = await WasmNode.instantiate(module);
    expect(a.memory.buffer).not.toBe(b.memory.buffer);

    a.init(1, 1.0, 100.0, 0.0);
    b.init(2, -1.0, 100.0, 0.0);
    const torqueA = a.tick({ motorAngle: 0.0, motorVelocity: 0.0 });
    const torqueB = b.tick({ motorAngle: 0.0, motorVelocity: 0.0 });
    expect(torqueA).toBeCloseTo(100.0, 5);
    expect(torqueB).toBeCloseTo(-100.0, 5);
  });
});
