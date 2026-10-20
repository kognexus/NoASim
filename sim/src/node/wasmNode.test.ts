import { readFile } from "node:fs/promises";
import { beforeAll, describe, expect, it } from "vitest";
import {
  Address,
  decodeFrame,
  decodeRecords,
  encodeFrame,
  encodeRecords,
  FrameKind,
} from "../bus/protocol.ts";
import { KX6 } from "../plant/kx6.ts";
import { emptySensorFrame, type SensorFrame } from "../sensors/jointSensors.ts";
import { nodeConfigsFor } from "../harness/gains.ts";
import { WasmNode } from "./wasmNode.ts";

const WASM_PATH = new URL(
  "../../../target/wasm32-unknown-unknown/release/nexus_wasm.wasm",
  import.meta.url,
);
const CONFIG = nodeConfigsFor(KX6, 4000)[2]!;

const assign = (position: number, id: number) =>
  encodeRecords([
    encodeFrame({
      header: { kind: FrameKind.Assign, source: Address.host, destination: Address.all, cycle: 0 },
      assign: { position, id },
    }),
  ]);

const at = (angle: number): SensorFrame => ({
  ...emptySensorFrame(),
  motorAngle: angle,
  outputAngle: angle,
});

describe("WasmNode", () => {
  let module: WebAssembly.Module;

  beforeAll(async () => {
    module = await WebAssembly.compile(await readFile(WASM_PATH));
  });

  it("gives every joint its own memory", async () => {
    const a = await WasmNode.instantiate(module);
    const b = await WasmNode.instantiate(module);
    expect(a.memory.buffer).not.toBe(b.memory.buffer);
    a.init(CONFIG, 1);
    b.init(CONFIG, 2);
    a.tick(0, at(0.0), new Uint8Array());
    b.tick(0, at(1.0), new Uint8Array());
    // Each holds its own power-on position: pushed to 0.5, they push in opposite directions.
    expect(a.tick(1, at(0.5), new Uint8Array()).torque).toBeLessThan(0);
    expect(b.tick(1, at(0.5), new Uint8Array()).torque).toBeGreaterThan(0);
  });

  it("echoes its config when assigned, so the config layout matches", async () => {
    const node = await WasmNode.instantiate(module);
    node.init(CONFIG, 3);
    const { outbox } = node.tick(7, at(0), assign(3, 3));
    const [announce, state] = decodeRecords(outbox).map(decodeFrame);
    expect(announce).toMatchObject({
      header: { kind: FrameKind.Announce, source: 3, destination: Address.all, cycle: 7 },
      announce: {
        position: 3,
        hasBrake: false,
        gearRatio: 9,
        ratedTorque: 30,
        peakTorque: 90,
      },
    });
    if (announce && "announce" in announce) {
      expect(announce.announce.linkMass).toBeCloseTo(1.6, 6);
      expect(announce.announce.link[2]).toBeCloseTo(0.39, 6);
      expect(announce.announce.axis).toEqual([0, 1, 0]);
    }
    expect(state?.header.kind).toBe(FrameKind.State);
  });

  it("publishes its own sensor readings, so the sensor layout matches", async () => {
    const node = await WasmNode.instantiate(module);
    node.init(CONFIG, 1);
    const sensors: SensorFrame = {
      motorAngle: 0.25,
      outputAngle: 0.375,
      hallSector: 4,
      gyro: [0.5, -0.25, 0.125],
      accel: [1.5, -2.5, 9.75],
      current: 3.5,
      temperature: 42.5,
    };
    node.tick(0, sensors, assign(1, 1));
    const { outbox } = node.tick(1, sensors, new Uint8Array());
    const frame = decodeFrame(decodeRecords(outbox)[0]!);
    expect("state" in frame).toBe(true);
    if (!("state" in frame)) return;
    expect(frame.state).toMatchObject({
      motorAngle: 0.25,
      outputAngle: 0.375,
      gearTwist: 0.125,
      gyro: [0.5, -0.25, 0.125],
      accel: [1.5, -2.5, 9.75],
      current: 3.5,
      temperature: 42.5,
    });
  });
});
