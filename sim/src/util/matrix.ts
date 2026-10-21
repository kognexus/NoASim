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

/** Lower-triangular L with `a = L·Lᵀ`, for a symmetric positive-definite `a`. */
export function cholesky(a: Matrix): Matrix {
  const n = a.rows;
  const l = new Matrix(n, n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j <= i; j++) {
      let sum = a.get(i, j);
      for (let k = 0; k < j; k++) sum -= l.get(i, k) * l.get(j, k);
      if (i === j) {
        if (sum <= 0) throw new Error("matrix is not positive definite");
        l.set(i, i, Math.sqrt(sum));
      } else {
        l.set(i, j, sum / l.get(j, j));
      }
    }
  }
  return l;
}

/**
 * Eigenvalues (ascending) and eigenvectors (columns) of a symmetric matrix, by cyclic
 * Jacobi rotations.
 */
export function symmetricEigen(a: Matrix): { values: number[]; vectors: Matrix } {
  const n = a.rows;
  const m = new Matrix(n, n, Float64Array.from(a.data));
  const v = Matrix.identity(n);
  for (let sweep = 0; sweep < 100; sweep++) {
    let off = 0;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) off += m.get(p, q) ** 2;
    if (off <= 1e-24 * Math.max(1e-300, m.norm1() ** 2)) break;
    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) {
        const apq = m.get(p, q);
        if (apq === 0) continue;
        const theta = (m.get(q, q) - m.get(p, p)) / (2 * apq);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        for (let k = 0; k < n; k++) {
          const mkp = m.get(k, p);
          const mkq = m.get(k, q);
          m.set(k, p, c * mkp - s * mkq);
          m.set(k, q, s * mkp + c * mkq);
        }
        for (let k = 0; k < n; k++) {
          const mpk = m.get(p, k);
          const mqk = m.get(q, k);
          m.set(p, k, c * mpk - s * mqk);
          m.set(q, k, s * mpk + c * mqk);
        }
        for (let k = 0; k < n; k++) {
          const vkp = v.get(k, p);
          const vkq = v.get(k, q);
          v.set(k, p, c * vkp - s * vkq);
          v.set(k, q, s * vkp + c * vkq);
        }
      }
    }
  }
  const order = Array.from({ length: n }, (_, i) => i).sort((i, j) => m.get(i, i) - m.get(j, j));
  const vectors = new Matrix(n, n);
  order.forEach((col, k) => {
    for (let i = 0; i < n; i++) vectors.set(i, k, v.get(i, col));
  });
  return { values: order.map((i) => m.get(i, i)), vectors };
}
