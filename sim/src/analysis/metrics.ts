/**
 * Metrics for the hypotheses in PLAN §3, computed from recorded signals.
 */

/**
 * Time after which `deviation` stays within `band` × its peak (or within `floor`, if that
 * is larger), s.
 *
 * @param deviation non-negative distance from rest, sampled every `dt` seconds
 * @param floor smallest band, e.g. a multiple of the noise measured before the event
 * @returns the settling time, or `Infinity` if it never settles within the record
 */
export function settlingTime(
  deviation: ArrayLike<number>,
  dt: number,
  band = 0.05,
  floor = 0,
): number {
  let peak = 0;
  for (const d of Array.from(deviation)) peak = Math.max(peak, d);
  if (peak === 0) return 0;
  const threshold = Math.max(band * peak, floor);
  let last = -1;
  for (let k = 0; k < deviation.length; k++) if ((deviation[k] ?? 0) > threshold) last = k;
  if (last === deviation.length - 1) return Infinity;
  return (last + 1) * dt;
}

/** Root mean square of a signal. */
export function rms(values: ArrayLike<number>): number {
  let sum = 0;
  for (const v of Array.from(values)) sum += v ** 2;
  return values.length === 0 ? 0 : Math.sqrt(sum / values.length);
}
