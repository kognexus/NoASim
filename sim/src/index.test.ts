import { describe, expect, it } from "vitest";
import { BUS_CYCLE_HZ, BUS_CYCLE_S } from "./index";

describe("bus timing", () => {
  it("uses a 250 µs cycle", () => {
    expect(BUS_CYCLE_HZ).toBe(4000);
    expect(BUS_CYCLE_S).toBeCloseTo(250e-6, 12);
  });
});
