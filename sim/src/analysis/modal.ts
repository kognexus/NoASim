/**
 * Estimates the dominant mode of a free decay: frequency and damping ratio.
 */

export interface ModeEstimate {
  /** Damped natural frequency, Hz. */
  readonly frequencyHz: number;
  /** Damping ratio ζ (dimensionless). */
  readonly dampingRatio: number;
}

/**
 * Estimates frequency and damping from a free-decay signal.
 *
 * Frequency comes from the mean spacing of successive positive peaks, damping from the
 * logarithmic decrement over those peaks: δ = ln(x₀/xₙ)/n, ζ = δ/√(4π² + δ²). The signal
 * is taken relative to its final value, so a static offset does not matter.
 *
 * @param signal samples of the decay, starting after the excitation ends
 * @param dt sample interval, s
 * @param maxPeaks number of peaks to use (early peaks dominate the first mode)
 */
export function estimateMode(signal: ArrayLike<number>, dt: number, maxPeaks = 8): ModeEstimate {
  const n = signal.length;
  const tail = Math.max(1, Math.floor(n / 10));
  let rest = 0;
  for (let k = n - tail; k < n; k++) rest += signal[k] ?? 0;
  rest /= tail;

  const peaks: { index: number; value: number }[] = [];
  for (let k = 1; k < n - 1 && peaks.length < maxPeaks; k++) {
    const prev = (signal[k - 1] ?? 0) - rest;
    const here = (signal[k] ?? 0) - rest;
    const next = (signal[k + 1] ?? 0) - rest;
    if (here > 0 && here >= prev && here > next) peaks.push({ index: k, value: here });
  }
  const first = peaks[0];
  const last = peaks.at(-1);
  if (!first || !last || peaks.length < 3)
    throw new Error("not enough oscillation peaks to estimate a mode");

  const cycles = peaks.length - 1;
  const period = ((last.index - first.index) * dt) / cycles;
  const decrement = Math.log(first.value / last.value) / cycles;
  return {
    frequencyHz: 1 / period,
    dampingRatio: decrement / Math.sqrt(4 * Math.PI ** 2 + decrement ** 2),
  };
}
