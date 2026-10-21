/**
 * `npm run prove`: measures every hypothesis of the plan (H1–H10), the actuator grades
 * (NX-26), the headroom for learned damping (NX-27), the modes joints can reach (NX-28)
 * and coordinated damping (NX-29), from the code in this repository, and writes
 * docs/RESULTS.md with SVG plots in docs/results/. Deterministic: every run uses fixed
 * seeds, and the learned models are the committed ones.
 *
 *   node experiments/src/prove.ts [--quick] [--only H1,H7]
 *
 * `--quick` uses fewer seeds and trials (the report says so); `--only` runs a subset and
 * writes its report next to the full one (docs/RESULTS-partial.md).
 */
import { mkdir, writeFile } from "node:fs/promises";
import { type Finding, type ProveContext, resultsMarkdown } from "./prove/findings.ts";
import { h1 } from "./prove/h1.ts";
import { h10ac } from "./prove/h10.ts";
import { h10bd } from "./prove/h10b.ts";
import { h2h8 } from "./prove/h2.ts";
import { h3h4 } from "./prove/h3.ts";
import { h5h7 } from "./prove/h5.ts";
import { h6 } from "./prove/h6.ts";
import { grades } from "./prove/grades.ts";
import { h9 } from "./prove/h9.ts";
import { headroom } from "./prove/headroom.ts";
import { coordination } from "./prove/coordination.ts";
import { reach } from "./prove/reach.ts";
import { openLab } from "./runner.ts";

const quick = process.argv.includes("--quick");
const onlyArg = process.argv[process.argv.indexOf("--only") + 1];
const only = process.argv.includes("--only") && onlyArg ? new Set(onlyArg.split(",")) : undefined;

const DOCS = new URL("../../docs/", import.meta.url);
const PLOTS = new URL("results/", DOCS);
await mkdir(PLOTS, { recursive: true });

const started = Date.now();
const ctx: ProveContext = {
  lab: await openLab(),
  quick,
  log: (line) =>
    console.log(`[${((Date.now() - started) / 1000).toFixed(0).padStart(5)} s] ${line}`),
  plot: async (name, svg) => {
    await writeFile(new URL(name, PLOTS), svg);
    return `results/${name}`;
  },
};

/** Each step measures one or more hypotheses; listed in the plan's order. */
const STEPS: [string[], () => Promise<Finding | Finding[]>][] = [
  [["H1"], () => h1(ctx)],
  [["H2", "H8"], () => h2h8(ctx)],
  [["H3a", "H3b", "H3c", "H3d", "H4"], () => h3h4(ctx)],
  [["H5", "H7"], () => h5h7(ctx)],
  [["H6"], () => h6(ctx)],
  [["H9"], () => h9(ctx)],
  [["H10a", "H10c"], () => h10ac(ctx)],
  [["H10b", "H10d"], () => h10bd(ctx)],
  [["Grades"], () => grades(ctx)],
  [["Headroom"], () => headroom(ctx)],
  [["Reach"], () => reach(ctx)],
  [["Coordinated"], () => coordination(ctx)],
];

const ORDER = [
  "H1",
  "H2",
  "H3a",
  "H3b",
  "H3c",
  "H3d",
  "H4",
  "H5",
  "H6",
  "H7",
  "H8",
  "H9",
  "H10a",
  "H10b",
  "H10c",
  "H10d",
  "Grades",
  "Headroom",
  "Reach",
  "Coordinated",
];
const findings: Finding[] = [];
for (const [ids, step] of STEPS) {
  if (only && !ids.some((id) => only.has(id))) continue;
  ctx.log(`measuring ${ids.join(", ")}`);
  findings.push(...[await step()].flat());
}
findings.sort((a, b) => ORDER.indexOf(a.id) - ORDER.indexOf(b.id));

const seconds = (Date.now() - started) / 1000;
const report = resultsMarkdown(findings, { quick, seconds });
const name = only ? "RESULTS-partial.md" : "RESULTS.md";
await writeFile(new URL(name, DOCS), report);
await writeFile(
  new URL("../results/prove.json", import.meta.url),
  `${JSON.stringify({ quick, seconds, findings }, null, 2)}\n`,
);
console.log(`\nwrote docs/${name}`);
for (const f of findings) console.log(`${f.id.padEnd(5)} ${f.verdict.padEnd(11)} ${f.measured}`);
