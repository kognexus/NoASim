import { createHash } from "node:crypto";
import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";

/**
 * The sealed family must not change after NX-04 (PLAN §2.2). If this test fails, the file
 * was edited: say so openly in the ticket log, because the out-of-family result then
 * no longer counts as sealed.
 */
const SEALED_SHA256 = "23cc7c06ce1fa16e66a11589d2d320e17d658134f46098367a760349a12c9cd3";

describe("sealed actuator family", () => {
  it("is unchanged since it was sealed", async () => {
    const source = await readFile(new URL("./sealed.ts", import.meta.url));
    expect(createHash("sha256").update(source).digest("hex")).toBe(SEALED_SHA256);
  });
});
