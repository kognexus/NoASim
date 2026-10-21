import { describe, expect, it } from "vitest";
import { lqrGain, Matrix, solveDare } from "./matrix.ts";

describe("small matrices (NX-27)", () => {
  it("inverts", () => {
    const a = Matrix.from([
      [4, 1, 0],
      [1, 3, 1],
      [0, 1, 2],
    ]);
    const product = a.times(a.inverse());
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++) expect(product.get(i, j)).toBeCloseTo(i === j ? 1 : 0, 12);
  });

  it("solves the scalar Riccati equation", () => {
    // P = q + a²P − a²P²b²/(r + b²P), with a = 1.1, b = 1, q = r = 1.
    const one = Matrix.from([[1]]);
    const p = solveDare(Matrix.from([[1.1]]), one, one, one).get(0, 0);
    expect(p - (1 + 1.21 * p - (1.21 * p * p) / (1 + p))).toBeCloseTo(0, 10);
  });

  it("stabilises a double integrator", () => {
    const dt = 0.01;
    const a = Matrix.from([
      [1, dt],
      [0, 1],
    ]);
    const b = Matrix.from([[0], [dt]]);
    const k = lqrGain(a, b, Matrix.identity(2), Matrix.from([[0.01]]));
    let x = [1, 0];
    for (let step = 0; step < 2000; step++) {
      const u = -(k.get(0, 0) * x[0]! + k.get(0, 1) * x[1]!);
      x = [x[0]! + dt * x[1]!, x[1]! + dt * u];
    }
    expect(Math.hypot(x[0]!, x[1]!)).toBeLessThan(1e-6);
  });
});
