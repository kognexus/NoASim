/**
 * Small dense matrices for the ideal reference controller (NX-27): enough to design a
 * discrete-time LQR for a plant of a few dozen states. Not for anything in a node.
 */

/** Row-major dense matrix. */
export class Matrix {
  readonly rows: number;
  readonly cols: number;
  readonly data: Float64Array;

  constructor(rows: number, cols: number, data = new Float64Array(rows * cols)) {
    this.rows = rows;
    this.cols = cols;
    this.data = data;
  }

  static from(rows: readonly (readonly number[])[]): Matrix {
    const m = new Matrix(rows.length, rows[0]?.length ?? 0);
    rows.forEach((row, i) => row.forEach((x, j) => m.set(i, j, x)));
    return m;
  }

  static identity(n: number): Matrix {
    const m = new Matrix(n, n);
    for (let i = 0; i < n; i++) m.set(i, i, 1);
    return m;
  }

  get(i: number, j: number): number {
    return this.data[i * this.cols + j] ?? 0;
  }

  set(i: number, j: number, x: number): void {
    this.data[i * this.cols + j] = x;
  }

  times(b: Matrix): Matrix {
    const out = new Matrix(this.rows, b.cols);
    for (let i = 0; i < this.rows; i++) {
      for (let k = 0; k < this.cols; k++) {
        const x = this.get(i, k);
        if (x === 0) continue;
        for (let j = 0; j < b.cols; j++) out.data[i * b.cols + j] = out.get(i, j) + x * b.get(k, j);
      }
    }
    return out;
  }

  /** `this + scale·b`. */
  plus(b: Matrix, scale = 1): Matrix {
    return new Matrix(
      this.rows,
      this.cols,
      this.data.map((x, k) => x + scale * (b.data[k] ?? 0)),
    );
  }

  transpose(): Matrix {
    const out = new Matrix(this.cols, this.rows);
    for (let i = 0; i < this.rows; i++)
      for (let j = 0; j < this.cols; j++) out.set(j, i, this.get(i, j));
    return out;
  }

  /** Inverse by Gauss–Jordan elimination with partial pivoting. */
  inverse(): Matrix {
    const n = this.rows;
    const m = new Matrix(n, n, Float64Array.from(this.data));
    const inv = Matrix.identity(n);
    const swap = (a: Matrix, r: number, s: number) => {
      for (let j = 0; j < n; j++) {
        const t = a.get(r, j);
        a.set(r, j, a.get(s, j));
        a.set(s, j, t);
      }
    };
    for (let c = 0; c < n; c++) {
      let pivot = c;
      for (let r = c + 1; r < n; r++)
        if (Math.abs(m.get(r, c)) > Math.abs(m.get(pivot, c))) pivot = r;
      swap(m, c, pivot);
      swap(inv, c, pivot);
      const d = m.get(c, c);
      if (d === 0) throw new Error("singular matrix");
      for (let j = 0; j < n; j++) {
        m.set(c, j, m.get(c, j) / d);
        inv.set(c, j, inv.get(c, j) / d);
      }
      for (let r = 0; r < n; r++) {
        const f = m.get(r, c);
        if (r === c || f === 0) continue;
        for (let j = 0; j < n; j++) {
          m.set(r, j, m.get(r, j) - f * m.get(c, j));
          inv.set(r, j, inv.get(r, j) - f * inv.get(c, j));
        }
      }
    }
    return inv;
  }

  /** Sum of absolute entries. */
  norm1(): number {
    return this.data.reduce((s, x) => s + Math.abs(x), 0);
  }
}

/**
 * Solution P of the discrete algebraic Riccati equation
 * `P = Q + AᵀPA − AᵀPB (R + BᵀPB)⁻¹ BᵀPA`, by the structure-preserving doubling
 * algorithm (quadratic convergence, no eigenvectors needed).
 */
export function solveDare(a: Matrix, b: Matrix, q: Matrix, r: Matrix): Matrix {
  const eye = Matrix.identity(a.rows);
  let ak = a;
  let g = b.times(r.inverse()).times(b.transpose());
  let h = q;
  for (let it = 0; it < 100; it++) {
    const w = eye.plus(g.times(h)).inverse();
    const aw = ak.times(w);
    const next = h.plus(ak.transpose().times(h).times(w).times(ak));
    g = g.plus(aw.times(g).times(ak.transpose()));
    ak = aw.times(ak);
    const change = next.plus(h, -1).norm1();
    h = next;
    if (change <= 1e-12 * h.norm1()) return h;
  }
  throw new Error("Riccati iteration did not converge");
}

/** LQR gain K for `x' = Ax + Bu` minimising `Σ xᵀQx + uᵀRu`, with `u = −Kx`. */
export function lqrGain(a: Matrix, b: Matrix, q: Matrix, r: Matrix): Matrix {
  const p = solveDare(a, b, q, r);
  const btp = b.transpose().times(p);
  return r.plus(btp.times(b)).inverse().times(btp.times(a));
}
