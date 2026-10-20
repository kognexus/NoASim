/**
 * NX-01 benchmark, Node entry point. Run with `npm run bench:step`.
 *
 * Prints simulated steps per wall-clock second and the real-time factor for one and two
 * arms, with and without the node ticks.
 */
import { readFile } from "node:fs/promises";
import loadMujoco from "@mujoco/mujoco";
import { BUS_CYCLE_HZ } from "../src/index.ts";
import { measureStepRate } from "./stepCostCore.ts";

const WASM_PATH = new URL(
  "../../target/wasm32-unknown-unknown/release/nexus_wasm.wasm",
  import.meta.url,
);

const mujoco = await loadMujoco();
const nodeModule = await WebAssembly.compile(await readFile(WASM_PATH));

console.log(`Simulating 5 s at ${BUS_CYCLE_HZ} Hz per configuration.\n`);
console.log("arms | nodes | steps/s | real-time factor");
for (const arms of [1, 2]) {
  for (const withNodes of [false, true]) {
    const rate = await measureStepRate(mujoco, nodeModule, arms, withNodes);
    const factor = rate / BUS_CYCLE_HZ;
    console.log(
      `${arms}    | ${withNodes ? "yes" : "no "}   | ${rate.toFixed(0).padStart(7)} | ${factor.toFixed(2)}×`,
    );
  }
}
