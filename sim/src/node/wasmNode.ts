/**
 * Runs one Nexus node (the `nexus-wasm` crate) in its own WebAssembly instance.
 *
 * Every call to {@link WasmNode.instantiate} creates a fresh instance with its own linear
 * memory, so joints cannot share state. The harness talks to a node only through its I/O
 * block: it writes the node's own sensor frame, the bus cycle and the frames the bus
 * delivered, calls {@link WasmNode.tick}, and reads back the torque and the frames the
 * node sent. The block's layout comes from the module itself (`nexus_layout`).
 */
import type { SensorFrame } from "../sensors/jointSensors.ts";
import { type NodeConfig, packNodeConfig } from "./nodeConfig.ts";

/** Exports of the `nexus-wasm` module (see `crates/nexus-wasm/src/lib.rs`). */
interface NexusExports {
  readonly memory: WebAssembly.Memory;
  nexus_io_ptr(): number;
  nexus_layout(field: number): number;
  nexus_init(linePosition: number): void;
  nexus_tick(): void;
}

/** Byte offsets and sizes of the I/O block, as reported by the module. */
interface IoLayout {
  config: number;
  sensors: number;
  cycle: number;
  inboxLen: number;
  outboxLen: number;
  torque: number;
  inbox: number;
  outbox: number;
  inboxCapacity: number;
  outboxCapacity: number;
  configSize: number;
  sensorsSize: number;
}

/** Number of `f32` fields in the node's `SensorFrame`. */
const SENSOR_FIELDS = 11;

function readLayout(exports: NexusExports): IoLayout {
  const base = exports.nexus_io_ptr();
  const at = (field: number) => exports.nexus_layout(field);
  return {
    config: base + at(0),
    sensors: base + at(1),
    cycle: base + at(2),
    inboxLen: base + at(3),
    outboxLen: base + at(4),
    torque: base + at(5),
    inbox: base + at(6),
    outbox: base + at(7),
    inboxCapacity: at(8),
    outboxCapacity: at(9),
    configSize: at(11),
    sensorsSize: at(12),
  };
}

export class WasmNode {
  readonly #exports: NexusExports;
  readonly #layout: IoLayout;
  readonly #sensors = new Float32Array(SENSOR_FIELDS);

  private constructor(exports: NexusExports) {
    this.#exports = exports;
    this.#layout = readLayout(exports);
    if (this.#layout.sensorsSize !== SENSOR_FIELDS * 4) {
      throw new Error(
        `node sensor frame is ${this.#layout.sensorsSize} bytes; harness writes ${SENSOR_FIELDS * 4}`,
      );
    }
  }

  /** Creates a new, isolated node instance from a compiled `nexus-wasm` module. */
  static async instantiate(module: WebAssembly.Module): Promise<WasmNode> {
    const instance = await WebAssembly.instantiate(module, {});
    return new WasmNode(instance.exports as unknown as NexusExports);
  }

  /** The instance's linear memory; exposed only so tests can check isolation. */
  get memory(): WebAssembly.Memory {
    return this.#exports.memory;
  }

  /** Capacity of the node's inbox, bytes. */
  get inboxCapacity(): number {
    return this.#layout.inboxCapacity;
  }

  /** Writes the node's EEPROM contents and powers it on at bus line position `linePosition`. */
  init(config: NodeConfig, linePosition: number): void {
    const packed = packNodeConfig(config);
    if (packed.length * 4 !== this.#layout.configSize) {
      throw new Error(
        `node config is ${this.#layout.configSize} bytes; harness packs ${packed.length * 4}`,
      );
    }
    new Float32Array(this.#exports.memory.buffer, this.#layout.config, packed.length).set(packed);
    this.#exports.nexus_init(linePosition);
  }

  /**
   * Runs one bus cycle.
   *
   * @param inbox mailbox records delivered by the bus this cycle
   * @returns the motor torque (output-referred, N·m) and the mailbox records the node sent
   *   (a copy, safe to keep)
   */
  tick(
    cycle: number,
    sensors: SensorFrame,
    inbox: Uint8Array,
  ): { torque: number; outbox: Uint8Array } {
    const { buffer } = this.#exports.memory;
    const layout = this.#layout;
    if (inbox.length > layout.inboxCapacity)
      throw new Error(`inbox overflow: ${inbox.length} bytes`);
    const s = this.#sensors;
    s[0] = sensors.motorAngle;
    s[1] = sensors.outputAngle;
    s[2] = sensors.hallSector;
    s.set(sensors.gyro, 3);
    s.set(sensors.accel, 6);
    s[9] = sensors.current;
    s[10] = sensors.temperature;
    new Float32Array(buffer, layout.sensors, SENSOR_FIELDS).set(s);
    const view = new DataView(buffer);
    view.setUint32(layout.cycle, cycle, true);
    view.setUint32(layout.inboxLen, inbox.length, true);
    new Uint8Array(buffer, layout.inbox, inbox.length).set(inbox);

    this.#exports.nexus_tick();

    const after = new DataView(this.#exports.memory.buffer);
    const outboxLen = after.getUint32(layout.outboxLen, true);
    return {
      torque: after.getFloat32(layout.torque, true),
      outbox: new Uint8Array(this.#exports.memory.buffer, layout.outbox, outboxLen).slice(),
    };
  }
}
