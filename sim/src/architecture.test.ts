/**
 * Dependency rules that keep the joint code honest: a node can only learn about its
 * actuator and the arm through its own sensors and the bus, never through the harness.
 */
import { readdir, readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";

const ROOT = new URL("../../", import.meta.url);

async function sources(dir: URL): Promise<{ path: string; text: string }[]> {
  const entries = await readdir(dir, { recursive: true, withFileTypes: true });
  return Promise.all(
    entries
      .filter((e) => e.isFile() && /\.(ts|rs)$/.test(e.name))
      .map(async (e) => {
        const path = `${e.parentPath}/${e.name}`;
        return { path, text: await readFile(path, "utf8") };
      }),
  );
}

function dependencies(cargoToml: string): string[] {
  const section = cargoToml.split("[dependencies]")[1]?.split("\n[")[0] ?? "";
  return [...section.matchAll(/^([\w-]+)\s*=/gm)].map((m) => m[1] ?? "");
}

describe("architecture", () => {
  it("node-side TypeScript does not import the plant, actuators or ground truth", async () => {
    for (const { path, text } of await sources(new URL("sim/src/node/", ROOT))) {
      expect(text, path).not.toMatch(/from\s+["'][^"']*\/(plant|actuators|analysis)\//);
    }
  });

  it("node crates depend only on each other", async () => {
    const node = await readFile(new URL("crates/nexus-node/Cargo.toml", ROOT), "utf8");
    const wasm = await readFile(new URL("crates/nexus-wasm/Cargo.toml", ROOT), "utf8");
    const proto = await readFile(new URL("crates/nexus-proto/Cargo.toml", ROOT), "utf8");
    expect(dependencies(proto)).toEqual([]);
    expect(dependencies(node)).toEqual(["nexus-proto"]);
    expect(dependencies(wasm)).toEqual(["nexus-node"]);
  });
});
