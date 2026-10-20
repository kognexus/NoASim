/** Shared test fixtures (Node only). */
import { readFile } from "node:fs/promises";

const WASM_PATH = new URL(
  "../../../target/wasm32-unknown-unknown/release/nexus_wasm.wasm",
  import.meta.url,
);

let compiled: Promise<WebAssembly.Module> | undefined;

/** The compiled `nexus-wasm` module (built by `npm run build:wasm`). */
export function loadNodeModule(): Promise<WebAssembly.Module> {
  compiled ??= readFile(WASM_PATH).then((bytes) => WebAssembly.compile(bytes));
  return compiled;
}
