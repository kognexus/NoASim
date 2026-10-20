/**
 * Deterministic random numbers, so every experiment is reproducible from its seed.
 *
 * xoshiro128** seeded through splitmix32. Independent streams for each sensor are made
 * with {@link Random.derive}, so adding a sensor does not shift the numbers of another.
 */

function splitmix32(seed: number): () => number {
  let state = seed >>> 0;
  return () => {
    state = (state + 0x9e3779b9) >>> 0;
    let z = state;
    z = Math.imul(z ^ (z >>> 16), 0x85ebca6b) >>> 0;
    z = Math.imul(z ^ (z >>> 13), 0xc2b2ae35) >>> 0;
    return (z ^ (z >>> 16)) >>> 0;
  };
}

/** FNV-1a hash of a label, used to derive stream seeds. */
function hash(label: string): number {
  let h = 0x811c9dc5;
  for (let i = 0; i < label.length; i++) h = Math.imul(h ^ label.charCodeAt(i), 0x01000193) >>> 0;
  return h;
}

export class Random {
  readonly seed: number;
  #s0: number;
  #s1: number;
  #s2: number;
  #s3: number;
  #spareNormal: number | undefined;

  constructor(seed: number) {
    this.seed = seed >>> 0;
    const next = splitmix32(this.seed);
    this.#s0 = next();
    this.#s1 = next();
    this.#s2 = next();
    this.#s3 = next();
  }

  /** An independent stream identified by `label` (e.g. "j3/gyro"). */
  derive(label: string): Random {
    return new Random((Math.imul(this.seed, 0x9e3779b1) ^ hash(label)) >>> 0);
  }

  /** Uniform in [0, 1). */
  uniform(): number {
    const result = Math.imul(rotl(Math.imul(this.#s1, 5), 7), 9) >>> 0;
    const t = (this.#s1 << 9) >>> 0;
    this.#s2 = (this.#s2 ^ this.#s0) >>> 0;
    this.#s3 = (this.#s3 ^ this.#s1) >>> 0;
    this.#s1 = (this.#s1 ^ this.#s2) >>> 0;
    this.#s0 = (this.#s0 ^ this.#s3) >>> 0;
    this.#s2 = (this.#s2 ^ t) >>> 0;
    this.#s3 = rotl(this.#s3, 11);
    return result / 4294967296;
  }

  /** Uniform in [low, high). */
  between(low: number, high: number): number {
    return low + (high - low) * this.uniform();
  }

  /** Standard normal (Box–Muller, both values used). */
  normal(): number {
    if (this.#spareNormal !== undefined) {
      const spare = this.#spareNormal;
      this.#spareNormal = undefined;
      return spare;
    }
    let u = 0;
    while (u === 0) u = this.uniform();
    const v = this.uniform();
    const r = Math.sqrt(-2 * Math.log(u));
    this.#spareNormal = r * Math.sin(2 * Math.PI * v);
    return r * Math.cos(2 * Math.PI * v);
  }
}

function rotl(x: number, k: number): number {
  return ((x << k) | (x >>> (32 - k))) >>> 0;
}
