/**
 * A joint's coordinated model (NX-31) as `learning/` exports it
 * (`nexus_learning/coordinated.py`), its packing into the word block the node reads
 * (`CoordinatedModel::from_words` in `coordinated_model.rs`), and the same evaluation in
 * TypeScript, which `coordinatedModel.test.ts` checks against the trainer's probes and the
 * node.
 */

/** The exported JSON (`models/coordinated-models/<arm>-reach<r>/joint<n>.json`). */
export interface CoordinatedModelDocument {
  readonly format: "nexus-coordinated";
  readonly version: number;
  /** Line position of the joint, 1-based. */
  readonly joint: number;
  readonly reach: number;
  /** Line positions of the joints read, ascending, own included. */
  readonly neighbours: readonly number[];
  readonly features: number;
  /** One row of feature weights per neighbour: `m_ij`. */
  readonly inertia: readonly (readonly number[])[];
  /** Bend of the joint's link per N·m, rad/(N·m). */
  readonly compliance: number;
  /** Share of its own link's bend the joint takes out. */
  readonly bendSplit: number;
  readonly axis: readonly number[];
  readonly probes: readonly {
    readonly angles: readonly number[];
    readonly accelerations: readonly number[];
    readonly inertialTorque: number;
  }[];
}

/** Features of `angles`, in the trainer's order. */
export function coordinatedFeatures(angles: readonly number[]): number[] {
  const out = [1];
  for (const q of angles) out.push(Math.cos(q), Math.sin(q));
  for (let a = 0; a < angles.length; a++) {
    for (let b = a + 1; b < angles.length; b++) {
      const s = (angles[a] ?? 0) + (angles[b] ?? 0);
      const d = (angles[a] ?? 0) - (angles[b] ?? 0);
      out.push(Math.cos(s), Math.sin(s), Math.cos(d), Math.sin(d));
    }
  }
  return out;
}

/** `Σ_j m_ij(q) · q̈_j`. */
export function coordinatedInertialTorque(
  doc: CoordinatedModelDocument,
  angles: readonly number[],
  accelerations: readonly number[],
): number {
  const phi = coordinatedFeatures(angles);
  return doc.inertia.reduce(
    (sum, row, j) =>
      sum + (accelerations[j] ?? 0) * row.reduce((s, w, f) => s + w * (phi[f] ?? 0), 0),
    0,
  );
}

/** The block the node reads: version, joint, count, compliance, split, positions, weights. */
export function packCoordinatedModel(doc: CoordinatedModelDocument): Float32Array {
  if (doc.format !== "nexus-coordinated") throw new Error("not a nexus-coordinated model");
  return Float32Array.from([
    doc.version,
    doc.joint,
    doc.neighbours.length,
    doc.compliance,
    doc.bendSplit,
    ...doc.neighbours,
    ...doc.inertia.flat(),
  ]);
}
