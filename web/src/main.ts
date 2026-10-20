/**
 * NX-01 spike page: runs the step-cost benchmark in the browser. NX-20 replaces this
 * with the interactive demo.
 */
import loadMujoco from "@mujoco/mujoco";
import nodeWasmUrl from "../../target/wasm32-unknown-unknown/release/nexus_wasm.wasm?url";
import { BUS_CYCLE_HZ } from "../../sim/src/index.ts";
import { measureStepRate } from "../../sim/bench/stepCostCore.ts";

const out = document.querySelector<HTMLPreElement>("#out");
if (!out) throw new Error("missing #out");

const mujoco = await loadMujoco();
const nodeModule = await WebAssembly.compileStreaming(fetch(nodeWasmUrl));

const lines = ["arms | nodes | steps/s | real-time factor"];
for (const arms of [1, 2]) {
  for (const withNodes of [false, true]) {
    const rate = await measureStepRate(mujoco, nodeModule, arms, withNodes);
    lines.push(
      `${arms}    | ${withNodes ? "yes" : "no "}   | ${rate.toFixed(0).padStart(7)} | ${(rate / BUS_CYCLE_HZ).toFixed(2)}×`,
    );
    out.textContent = lines.join("\n") + "\n…";
  }
}
out.textContent = lines.join("\n");
out.dataset["done"] = "true";
