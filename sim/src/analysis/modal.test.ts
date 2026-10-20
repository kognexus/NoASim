import { describe, expect, it } from "vitest";
import { estimateMode } from "./modal.ts";

describe("estimateMode", () => {
  it("recovers frequency and damping of a synthetic decay", () => {
    const dt = 1e-4;
    const zeta = 0.02;
    const fn = 10;
    const wn = 2 * Math.PI * fn;
    const wd = wn * Math.sqrt(1 - zeta ** 2);
    const signal = Array.from(
      { length: 20000 },
      (_, k) => 0.3 + Math.exp(-zeta * wn * k * dt) * Math.cos(wd * k * dt),
    );
    const mode = estimateMode(signal, dt);
    expect(mode.frequencyHz).toBeCloseTo(wd / (2 * Math.PI), 1);
    expect(mode.dampingRatio).toBeCloseTo(zeta, 3);
  });

  it("rejects a signal without oscillation", () => {
    expect(() => estimateMode([1, 0.5, 0.25, 0.1, 0], 0.01)).toThrow();
  });
});
