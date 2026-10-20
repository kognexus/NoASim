import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import {
  decodeFrame,
  decodeRecords,
  encodeFrame,
  encodeRecords,
  FrameKind,
  StatusFlags,
  type Frame,
} from "./protocol.ts";

const GOLDEN = new URL("../../../crates/nexus-proto/golden/frames.txt", import.meta.url);
const header = <K extends FrameKind>(kind: K, source: number, destination: number) => ({
  kind,
  source,
  destination,
  cycle: 123_456,
});

/** The same samples as `crates/nexus-proto/tests/golden.rs`. */
const SAMPLES: Record<string, Frame> = {
  state: {
    header: header(FrameKind.State, 3, 0xfe),
    state: {
      status: StatusFlags.ready | StatusFlags.networkDamping,
      motorAngle: 1.5,
      motorVelocity: -0.25,
      outputAngle: 1.5078125,
      outputVelocity: -0.5,
      gearTwist: 0.0078125,
      gyro: [0.125, -0.0625, 2.0],
      accel: [0.5, -9.75, 1.0],
      current: 3.25,
      torqueCommand: 12.5,
      networkTorque: -0.75,
      temperature: 41.5,
      thermalHeadroom: 0.875,
      tankEnergy: 0.5,
    },
  },
  goal: {
    header: header(FrameKind.Goal, 0, 2),
    goal: { dueCycle: 123_656, position: -0.75, velocity: 0.125, sequence: 513 },
  },
  limits: {
    header: header(FrameKind.Limits, 0, 0xff),
    limits: {
      maxTorque: 90,
      maxVelocity: 3,
      minPosition: -3,
      maxPosition: 3,
      maxTemperature: 120,
      networkTorqueShare: 0.25,
    },
  },
  announce: {
    header: header(FrameKind.Announce, 4, 0xff),
    announce: {
      position: 4,
      hasBrake: true,
      modelVersion: 7,
      gearRatio: 9,
      ratedTorque: 30,
      peakTorque: 90,
      axis: [0, 1, 0],
      link: [0, 0, 0.390625],
      linkMass: 1.625,
    },
  },
  assign: { header: header(FrameKind.Assign, 0, 0xff), assign: { position: 4, id: 4 } },
  settings: {
    header: header(FrameKind.Settings, 0, 0xff),
    settings: { damping: 2, interpolation: 1, impactPolicy: 2, model: 1, gravity: 1 },
  },
  chain: {
    header: header(FrameKind.Chain, 2, 0xfe),
    chain: {
      upLink: [0.5, -0.25, 9.75],
      mass: 7.5,
      com: [0.125, 0, 0.375],
      flange: [0, 0.25, 0.875],
      payloadMass: 2.875,
      payloadLever: -4.5,
      holdingTorque: -31.25,
    },
  },
};

const hex = (bytes: Uint8Array) =>
  Array.from(bytes, (b) => b.toString(16).padStart(2, "0")).join("");

describe("bus protocol (TypeScript codec)", () => {
  it("matches the Rust codec byte for byte and round-trips", async () => {
    const lines = (await readFile(GOLDEN, "utf8")).trim().split("\n");
    const golden = Object.fromEntries(lines.map((l) => l.split(" ") as [string, string]));
    expect(Object.keys(golden).sort()).toEqual(Object.keys(SAMPLES).sort());
    for (const [name, frame] of Object.entries(SAMPLES)) {
      const bytes = encodeFrame(frame);
      expect(hex(bytes), name).toBe(golden[name]);
      expect(decodeFrame(bytes), name).toEqual(frame);
    }
  });

  it("packs and unpacks mailbox records", () => {
    const frames = [encodeFrame(SAMPLES.goal!), encodeFrame(SAMPLES.assign!)];
    const records = decodeRecords(encodeRecords(frames));
    expect(records.map((r) => hex(r))).toEqual(frames.map((f) => hex(f)));
  });

  it("rejects unknown kinds and wrong lengths", () => {
    const bytes = encodeFrame(SAMPLES.goal!);
    expect(() => decodeFrame(bytes.subarray(0, bytes.length - 1))).toThrow(/wrong length/);
    bytes[0] = 0xee;
    expect(() => decodeFrame(bytes)).toThrow(/unknown/);
  });
});
