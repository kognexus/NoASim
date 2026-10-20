/**
 * Runs one Nexus node (the `nexus-wasm` crate) in its own WebAssembly instance.
 *
 * Every call to {@link WasmNode.instantiate} creates a fresh instance with its own linear
 * memory, so joints cannot share state. The harness talks to a node only through the
 * node's I/O block: it writes the node's own sensor frame, calls {@link WasmNode.tick},
 * and reads the command back.
 */

/** Exports of the `nexus-wasm` module (see `crates/nexus-wasm/src/lib.rs`). */
interface NexusExports {
  readonly memory: WebAssembly.Memory;
  nexus_io_ptr(): number;
  nexus_io_len(): number;
  nexus_init(id: number, target: number, stiffness: number, damping: number): void;
  nexus_tick(): void;
}

/** Offsets (in `f32` slots) of the fields in the node's `Io` block. */
const IO_LAYOUT = {
  motorAngle: 0,
  motorVelocity: 1,
  torque: 2,
  /** Total number of `f32` slots; checked against the module at instantiation. */
  length: 3,
} as const;

/** Sensor readings the harness hands to a node for one cycle (NX-01 subset). */
export interface NodeSensors {
  motorAngle: number;
  motorVelocity: number;
}

export class WasmNode {
  readonly #exports: NexusExports;
  readonly #ioOffset: number;
  #io: Float32Array;

  private constructor(exports: NexusExports) {
    this.#exports = exports;
    this.#ioOffset = exports.nexus_io_ptr();
    const bytes = exports.nexus_io_len();
    if (bytes !== IO_LAYOUT.length * Float32Array.BYTES_PER_ELEMENT) {
      throw new Error(
        `nexus-wasm I/O block is ${bytes} bytes; harness expects ${IO_LAYOUT.length * 4}`,
      );
    }
    this.#io = this.#view();
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

  /** Starts the node with its bus address and a position-hold target (NX-01 placeholder). */
  init(id: number, target: number, stiffness: number, damping: number): void {
    this.#exports.nexus_init(id, target, stiffness, damping);
  }

  /** Runs one bus cycle with the given sensor frame and returns the motor torque in N·m. */
  tick(sensors: NodeSensors): number {
    const io = this.#ioView();
    io[IO_LAYOUT.motorAngle] = sensors.motorAngle;
    io[IO_LAYOUT.motorVelocity] = sensors.motorVelocity;
    this.#exports.nexus_tick();
    return io[IO_LAYOUT.torque] ?? 0;
  }

  /** Returns the I/O view, re-creating it if the instance's memory was grown. */
  #ioView(): Float32Array {
    if (this.#io.buffer !== this.#exports.memory.buffer) this.#io = this.#view();
    return this.#io;
  }

  #view(): Float32Array {
    return new Float32Array(this.#exports.memory.buffer, this.#ioOffset, IO_LAYOUT.length);
  }
}
