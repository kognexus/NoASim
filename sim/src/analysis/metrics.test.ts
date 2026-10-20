import { describe, expect, it } from "vitest";
import { rms, settlingTime } from "./metrics.ts";

describe("settlingTime", () => {
  it("finds when a decay enters the 5 % band for good", () => {
    const dt = 0.001;
    const decay = Array.from({ length: 5000 }, (_, k) => Math.exp(-k * dt));
    expect(settlingTime(decay, dt)).toBeCloseTo(-Math.log(0.05), 2);
  });

  it("reports a signal that never settles as Infinity", () => {
    expect(settlingTime([1, 0.5, 1, 0.5, 1], 0.1)).toBe(Infinity);
  });

  it("computes RMS", () => {
    expect(rms([3, -3, 3, -3])).toBe(3);
  });
});
