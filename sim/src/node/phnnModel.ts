/**
 * A joint's frozen learned model (B11) as `learning/` exports it, and its packing into the
 * flat word block the node reads (`PhnnModel::from_words` in `phnn.rs`). The order here
 * must match the Rust reader exactly; `phnnModel.test.ts` checks it against the node.
 */

/** One linear layer as PyTorch stores it: `weight[out][in]`, `bias[out]`. */
export interface Layer {
  readonly weight: readonly (readonly number[])[];
  readonly bias: readonly number[];
}

/** The exported JSON (`models/<unit>/joint<n>.json`). */
export interface PhnnDocument {
  readonly format: "nexus-phnn";
  readonly version: number;
  readonly unit: string;
  readonly joint: number;
  readonly constants: {
    readonly ratedTorque: number;
    readonly gearStiffness: number;
    readonly twistRef: number;
    readonly omegaRef: number;
    readonly stictionSpeed: number;
    readonly temperatureRef: readonly [number, number];
    readonly viscousScale: number;
    readonly coulombScale: number;
  };
  readonly inertia: number;
  readonly gearDamping: number;
  readonly cogging: { readonly orders: readonly number[]; readonly weight: readonly number[] };
  readonly gear: readonly Layer[];
  readonly friction: {
    readonly viscous: readonly Layer[];
    readonly coulomb: readonly Layer[];
  };
  readonly gain: {
    readonly orders: readonly number[];
    readonly angle: readonly number[];
    readonly temperature: readonly Layer[];
  };
  /** Inputs and the trainer's float64 residual, for checking the node's evaluation. */
  readonly probes?: readonly {
    readonly theta: number;
    readonly omega: number;
    readonly phi: number;
    readonly phi_rate: number;
    readonly temperature: number;
    readonly u: number;
    readonly alpha: number;
    readonly unexplained: number;
  }[];
}

/** Sizes fixed by the node (`phnn.rs`). */
const MAX_COG_ORDERS = 8;
const HIDDEN = 16;
const GAIN_HIDDEN = 8;

function padded(values: readonly number[], length: number): number[] {
  if (values.length > length) throw new Error(`${values.length} values; the node holds ${length}`);
  return [...values, ...new Array<number>(length - values.length).fill(0)];
}

/** A two-hidden-layer network: each layer's rows, then its bias. */
function mlp(layers: readonly Layer[], inputs: number, hidden: number): number[] {
  const shape = layers.map((l) => [l.weight.length, l.weight[0]?.length ?? 0]);
  const expected = [
    [hidden, inputs],
    [hidden, hidden],
    [1, hidden],
  ];
  if (JSON.stringify(shape) !== JSON.stringify(expected)) {
    throw new Error(
      `network shape ${JSON.stringify(shape)}, node expects ${JSON.stringify(expected)}`,
    );
  }
  return layers.flatMap((l) => [...l.weight.flat(), ...l.bias]);
}

/** Packs `doc` into the words the node's `nexus_load_model` reads. */
export function packPhnnModel(doc: PhnnDocument): Float32Array {
  if (doc.format !== "nexus-phnn") throw new Error("not a nexus-phnn model");
  const c = doc.constants;
  const orders = doc.cogging.orders;
  const n = orders.length;
  const words = [
    doc.version,
    doc.joint,
    n,
    c.ratedTorque,
    c.gearStiffness,
    c.twistRef,
    c.omegaRef,
    c.stictionSpeed,
    c.temperatureRef[0],
    c.temperatureRef[1],
    c.viscousScale,
    c.coulombScale,
    doc.inertia,
    doc.gearDamping,
    ...padded(orders, MAX_COG_ORDERS),
    ...padded(doc.cogging.weight.slice(0, n), MAX_COG_ORDERS),
    ...padded(doc.cogging.weight.slice(n), MAX_COG_ORDERS),
    ...mlp(doc.gear, 1, HIDDEN),
    ...mlp(doc.friction.viscous, 1, HIDDEN),
    ...mlp(doc.friction.coulomb, 1, HIDDEN),
    ...padded(doc.gain.orders, 2),
    ...padded(doc.gain.angle, 4),
    ...mlp(doc.gain.temperature, 1, GAIN_HIDDEN),
  ];
  return Float32Array.from(words);
}
