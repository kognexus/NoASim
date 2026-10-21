import { describe, expect, it } from "vitest";
import { type Frame, FrameKind } from "../../sim/src/bus/protocol.ts";
import type { Host } from "../../sim/src/host/host.ts";
import { BUS_CYCLE_HZ } from "../../sim/src/index.ts";
import { VibrationMeter } from "./vibration.ts";

/** A host whose last received frames are one state frame per joint with `gyro`. */
function hostWith(gyro: (joint: number) => [number, number, number]): {
  host: Host;
  next: () => void;
} {
  const fake = { lastReceived: [] as Frame[] };
  const next = () => {
    fake.lastReceived = [1, 2].map(
      (source) =>
        ({
          header: { kind: FrameKind.State, source, destination: 0xfe, cycle: 0 },
          state: { gyro: gyro(source) },
        }) as unknown as Frame,
    );
  };
  return { host: fake as unknown as Host, next };
}

describe("vibration felt by each joint (demo)", () => {
  it("ignores steady rotation and reads a 10 Hz shake as its RMS", () => {
    let k = 0;
    const shake = 0.2; // rad/s amplitude on joint 2
    const { host, next } = hostWith((joint) => [
      0.5, // joint 1: turning steadily
      joint === 2 ? shake * Math.sin((2 * Math.PI * 10 * k) / BUS_CYCLE_HZ) : 0,
      0,
    ]);
    const meter = new VibrationMeter(host, 2);
    for (; k < BUS_CYCLE_HZ; k++) {
      next();
      meter.observe();
    }
    const [steady, shaking] = meter.levels();
    const expected = ((shake / Math.SQRT2) * 180) / Math.PI;
    expect(steady!).toBeLessThan(0.01 * expected);
    expect(shaking!).toBeGreaterThan(0.9 * expected);
    expect(shaking!).toBeLessThan(1.1 * expected);
  });
});
