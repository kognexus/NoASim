/**
 * Small, dependency-free SVG charts for docs/RESULTS.md: line charts of time series and
 * grouped bar charts. Fixed light colours, since the files are viewed as images.
 */

const PALETTE = ["#2f6fd6", "#d9822b", "#3f8f6b", "#8a5cd6", "#d64545", "#5d6673"];
const INK = "#1c2129";
const MUTED = "#5d6673";
const GRID = "#e4e7eb";
const FONT = "font-family='system-ui, -apple-system, Segoe UI, sans-serif'";

const escape = (s: string) => s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");

/** The next value of the 1–2–5 sequence at or above `x`. */
function niceCeiling(x: number): number {
  if (!(x > 0)) return 1;
  const base = 10 ** Math.floor(Math.log10(x));
  for (const step of [1, 2, 5, 10]) if (step * base >= x) return step * base;
  return 10 * base;
}

const fmt = (x: number) =>
  Math.abs(x) >= 10 ? x.toFixed(0) : Math.abs(x) >= 1 ? x.toFixed(1) : x.toPrecision(2);

function frame(width: number, height: number, title: string, body: string): string {
  return [
    `<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 ${width} ${height}' width='${width}' height='${height}' ${FONT}>`,
    `<rect width='${width}' height='${height}' fill='#ffffff'/>`,
    `<text x='16' y='24' font-size='15' font-weight='600' fill='${INK}'>${escape(title)}</text>`,
    body,
    "</svg>",
    "",
  ].join("\n");
}

function legend(labels: readonly string[], x: number, y: number): string {
  return labels
    .map((label, i) => {
      const lx = x + i * 150;
      return (
        `<rect x='${lx}' y='${y - 9}' width='12' height='12' rx='2' fill='${PALETTE[i % PALETTE.length]}'/>` +
        `<text x='${lx + 18}' y='${y + 1}' font-size='12' fill='${INK}'>${escape(label)}</text>`
      );
    })
    .join("\n");
}

export interface LineSeries {
  readonly label: string;
  readonly x: readonly number[];
  readonly y: readonly number[];
}

/** Line chart; `yUnit` and `xUnit` label the axes. */
export function lineChart(
  title: string,
  series: readonly LineSeries[],
  xUnit: string,
  yUnit: string,
): string {
  const width = 720;
  const height = 320;
  const left = 64;
  const right = 16;
  const top = 56;
  const bottom = 44;
  const xs = series.flatMap((s) => s.x);
  const xMax = Math.max(...xs);
  const xMin = Math.min(...xs);
  const yMax = niceCeiling(Math.max(...series.flatMap((s) => s.y)));
  const px = (x: number) => left + ((x - xMin) / (xMax - xMin || 1)) * (width - left - right);
  const py = (y: number) => height - bottom - (y / yMax) * (height - top - bottom);
  const grid = [0, 0.25, 0.5, 0.75, 1]
    .map((f) => {
      const y = py(f * yMax);
      return (
        `<line x1='${left}' x2='${width - right}' y1='${y}' y2='${y}' stroke='${GRID}'/>` +
        `<text x='${left - 8}' y='${y + 4}' font-size='11' text-anchor='end' fill='${MUTED}'>${fmt(f * yMax)}</text>`
      );
    })
    .join("\n");
  const ticks = [0, 0.25, 0.5, 0.75, 1]
    .map((f) => {
      const x = xMin + f * (xMax - xMin);
      return `<text x='${px(x)}' y='${height - bottom + 18}' font-size='11' text-anchor='middle' fill='${MUTED}'>${fmt(x)}</text>`;
    })
    .join("\n");
  const lines = series
    .map((s, i) => {
      const points = s.x
        .map((x, k) => `${px(x).toFixed(1)},${py(s.y[k] ?? 0).toFixed(1)}`)
        .join(" ");
      return `<polyline points='${points}' fill='none' stroke='${PALETTE[i % PALETTE.length]}' stroke-width='2' stroke-linejoin='round'/>`;
    })
    .join("\n");
  const axes =
    `<text x='${(left + width - right) / 2}' y='${height - 8}' font-size='12' text-anchor='middle' fill='${MUTED}'>${escape(xUnit)}</text>` +
    `<text x='14' y='${top - 10}' font-size='12' fill='${MUTED}'>${escape(yUnit)}</text>`;
  return frame(
    width,
    height,
    title,
    [
      grid,
      ticks,
      lines,
      axes,
      legend(
        series.map((s) => s.label),
        left,
        44,
      ),
    ].join("\n"),
  );
}

export interface BarGroup {
  readonly label: string;
  /** One value per series, in the order of `seriesLabels`. */
  readonly values: readonly number[];
}

/** Grouped bar chart: one group per category, one bar per series. */
export function barChart(
  title: string,
  groups: readonly BarGroup[],
  seriesLabels: readonly string[],
  unit: string,
): string {
  const width = 720;
  const height = 320;
  const left = 64;
  const right = 16;
  const top = 56;
  const bottom = 52;
  const yMax = niceCeiling(Math.max(...groups.flatMap((g) => g.values)));
  const py = (y: number) => height - bottom - (Math.max(0, y) / yMax) * (height - top - bottom);
  const groupWidth = (width - left - right) / groups.length;
  const barWidth = Math.min(36, (groupWidth * 0.8) / seriesLabels.length);
  const grid = [0, 0.25, 0.5, 0.75, 1]
    .map((f) => {
      const y = py(f * yMax);
      return (
        `<line x1='${left}' x2='${width - right}' y1='${y}' y2='${y}' stroke='${GRID}'/>` +
        `<text x='${left - 8}' y='${y + 4}' font-size='11' text-anchor='end' fill='${MUTED}'>${fmt(f * yMax)}</text>`
      );
    })
    .join("\n");
  const bars = groups
    .map((g, gi) => {
      const start = left + gi * groupWidth + (groupWidth - barWidth * seriesLabels.length) / 2;
      const rects = g.values
        .map((v, si) => {
          const x = start + si * barWidth;
          const y = py(v);
          return `<rect x='${x.toFixed(1)}' y='${y.toFixed(1)}' width='${(barWidth - 3).toFixed(1)}' height='${(height - bottom - y).toFixed(1)}' rx='2' fill='${PALETTE[si % PALETTE.length]}'/>`;
        })
        .join("");
      const label = `<text x='${(left + gi * groupWidth + groupWidth / 2).toFixed(1)}' y='${height - bottom + 18}' font-size='11' text-anchor='middle' fill='${INK}'>${escape(g.label)}</text>`;
      return rects + label;
    })
    .join("\n");
  const axis = `<text x='14' y='${top - 10}' font-size='12' fill='${MUTED}'>${escape(unit)}</text>`;
  return frame(width, height, title, [grid, bars, axis, legend(seriesLabels, left, 44)].join("\n"));
}
