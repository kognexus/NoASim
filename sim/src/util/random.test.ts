import { describe, expect, it } from "vitest";
import { Random } from "./random.ts";

describe("Random", () => {
  it("repeats for the same seed and differs between derived streams", () => {
    const a = new Random(42);
    const b = new Random(42);
    const seqA = Array.from({ length: 5 }, () => a.uniform());
    expect(Array.from({ length: 5 }, () => b.uniform())).toEqual(seqA);
    expect(new Random(42).derive("j1/gyro").uniform()).not.toBe(
      new Random(42).derive("j2/gyro").uniform(),
    );
  });

  it("produces standard normal samples", () => {
    const rng = new Random(7);
    const n = 200_000;
    let sum = 0;
    let sumSq = 0;
    for (let i = 0; i < n; i++) {
      const x = rng.normal();
      sum += x;
      sumSq += x * x;
    }
    expect(sum / n).toBeCloseTo(0, 2);
    expect(Math.sqrt(sumSq / n)).toBeCloseTo(1, 2);
  });
});
