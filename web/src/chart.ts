/**
 * A scrolling strip chart: one line per series over the last few seconds of simulated
 * time, drawn on a canvas. Colours come from the page's CSS custom properties so the
 * chart follows the light and dark themes.
 */

export interface Series {
  readonly label: string;
  /** CSS custom property holding the line colour, e.g. `--left`. */
  readonly colourVar: string;
}

export class StripChart {
  readonly #canvas: HTMLCanvasElement;
  #series: readonly Series[];
  readonly #samples: number[][];
  readonly #capacity: number;
  readonly #unit: string;

  /**
   * @param capacity samples kept (the chart's width in samples)
   * @param unit label after the axis value, e.g. "mm"
   */
  constructor(
    canvas: HTMLCanvasElement,
    series: readonly Series[],
    capacity: number,
    unit: string,
  ) {
    this.#canvas = canvas;
    this.#series = series;
    this.#samples = series.map(() => []);
    this.#capacity = capacity;
    this.#unit = unit;
  }

  /** Renames the series (the legend), keeping their samples. */
  setLabels(labels: readonly string[]): void {
    this.#series = this.#series.map((s, i) => ({ ...s, label: labels[i] ?? s.label }));
  }

  /** Adds one sample per series. */
  push(values: readonly number[]): void {
    values.forEach((v, i) => {
      const s = this.#samples[i];
      if (!s) return;
      s.push(v);
      if (s.length > this.#capacity) s.shift();
    });
  }

  clear(): void {
    for (const s of this.#samples) s.length = 0;
  }

  /** Largest value shown, for charts that share a scale. */
  peak(): number {
    return Math.max(0, ...this.#samples.flat().map(Math.abs));
  }

  /** Draws, with `top` as the scale's maximum (default: this chart's own peak). */
  draw(top = this.peak()): void {
    const canvas = this.#canvas;
    const ratio = Math.min(2, window.devicePixelRatio);
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (canvas.width !== Math.round(w * ratio)) canvas.width = Math.round(w * ratio);
    if (canvas.height !== Math.round(h * ratio)) canvas.height = Math.round(h * ratio);
    const g = canvas.getContext("2d");
    if (!g) return;
    const css = getComputedStyle(canvas);
    g.setTransform(ratio, 0, 0, ratio, 0, 0);
    g.clearRect(0, 0, w, h);
    const scale = niceCeiling(Math.max(1e-9, top));
    // Gridline at the top value and the zero line.
    g.strokeStyle = css.getPropertyValue("--grid");
    g.lineWidth = 1;
    g.beginPath();
    g.moveTo(0, 8.5);
    g.lineTo(w, 8.5);
    g.moveTo(0, h - 0.5);
    g.lineTo(w, h - 0.5);
    g.stroke();
    g.fillStyle = css.getPropertyValue("--muted");
    g.font = "11px system-ui, sans-serif";
    g.fillText(`${formatValue(scale)} ${this.#unit}`, 4, 20);
    this.#series.forEach((series, i) => {
      const s = this.#samples[i] ?? [];
      g.strokeStyle = css.getPropertyValue(series.colourVar);
      g.lineWidth = 2;
      g.beginPath();
      s.forEach((v, k) => {
        const x = (k / Math.max(1, this.#capacity - 1)) * w;
        const y = h - 1 - (Math.abs(v) / scale) * (h - 10);
        if (k === 0) g.moveTo(x, y);
        else g.lineTo(x, y);
      });
      g.stroke();
    });
    // Legend: one row, right-aligned at the top.
    g.textAlign = "right";
    let x = w - 4;
    for (const series of [...this.#series].reverse()) {
      g.fillStyle = css.getPropertyValue("--text");
      g.fillText(series.label, x, 20);
      x -= g.measureText(series.label).width + 6;
      g.fillStyle = css.getPropertyValue(series.colourVar);
      g.fillRect(x - 10, 11, 10, 10);
      x -= 20;
    }
    g.textAlign = "left";
  }
}

/** The next value of the 1–2–5 sequence at or above `x`. */
function niceCeiling(x: number): number {
  const exponent = Math.floor(Math.log10(x));
  const base = 10 ** exponent;
  for (const step of [1, 2, 5, 10]) if (step * base >= x) return step * base;
  return 10 * base;
}

function formatValue(x: number): string {
  return x >= 10 ? x.toFixed(0) : x >= 1 ? x.toFixed(1) : x.toPrecision(1);
}
