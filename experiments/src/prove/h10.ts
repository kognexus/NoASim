/**
 * H10a and H10c: the committed learned models, scored on the commissioning datasets
 * (regenerated from their seeds if missing, so the run is deterministic). Training is not
 * repeated here: `learning/` trains; the proof scores what is committed.
 */
import { execFile } from "node:child_process";
import { access, readFile } from "node:fs/promises";
import { promisify } from "node:util";
import { fileURLToPath } from "node:url";
import type { Finding, ProveContext } from "./findings.ts";
import { barChart } from "./svg.ts";

const run = promisify(execFile);
const ROOT = new URL("../../../", import.meta.url);
const DATASETS = new URL("experiments/results/commissioning/", ROOT);
const OPEN_UNITS = ["kx6-s300", "kx6-s301", "kx6-s302"];
const SEALED_UNIT = "kx6-sealed-s900";

interface FamilySummary {
  readonly joints: number;
  readonly nominal: number;
  readonly fitted: number;
  readonly phnn: number;
  readonly oracle: number;
  readonly phnnGapClosed: number;
  readonly fittedGapClosed: number;
}
interface Report {
  readonly results: { energy: { minDissipation: number; maxImbalance: number } }[];
  readonly families: Record<string, FamilySummary>;
}

const exists = (url: URL) =>
  access(url).then(
    () => true,
    () => false,
  );

/** Regenerates commissioning datasets that are missing. */
async function ensureDatasets(ctx: ProveContext): Promise<void> {
  const open = await Promise.all(
    OPEN_UNITS.map((u) => exists(new URL(`${u}/meta.json`, DATASETS))),
  );
  const script = fileURLToPath(new URL("experiments/src/commission.ts", ROOT));
  if (open.some((ok) => !ok)) {
    ctx.log("H10: regenerating the open commissioning datasets");
    await run("node", [script, String(OPEN_UNITS.length)], {
      cwd: fileURLToPath(ROOT),
      maxBuffer: 1 << 24,
    });
  }
  if (!(await exists(new URL(`${SEALED_UNIT}/meta.json`, DATASETS)))) {
    ctx.log("H10: regenerating the sealed commissioning dataset");
    await run("node", [script, "1", "--sealed"], { cwd: fileURLToPath(ROOT), maxBuffer: 1 << 24 });
  }
}

/** Scores the committed weights of `units` with `learning/` and returns its report. */
async function score(units: string[], report: string): Promise<Report> {
  const learning = fileURLToPath(new URL("learning/", ROOT));
  const reportPath = fileURLToPath(new URL(`experiments/results/${report}`, ROOT));
  await run(
    "uv",
    [
      "run",
      "nexus-train",
      ...units.map((u) => fileURLToPath(new URL(u, DATASETS))),
      "--evaluate",
      "--report",
      reportPath,
    ],
    { cwd: learning, maxBuffer: 1 << 24 },
  );
  return JSON.parse(await readFile(reportPath, "utf8")) as Report;
}

export async function h10ac(ctx: ProveContext): Promise<Finding[]> {
  await ensureDatasets(ctx);
  const open = await score(OPEN_UNITS, "prove-phnn-open.json");
  const sealed = await score([SEALED_UNIT], "prove-phnn-sealed.json");
  const s = Object.values(sealed.families)[0];
  if (!s) throw new Error("no sealed result");
  const rows = Object.entries(open.families).map(
    ([family, f]) =>
      `| ${family} | ${f.joints} | ${f.nominal.toFixed(3)} | ${f.fitted.toFixed(3)} | ${f.phnn.toFixed(3)} | ${f.oracle.toFixed(3)} | ${(f.phnnGapClosed * 100).toFixed(0)} % | ${(f.fittedGapClosed * 100).toFixed(0)} % |`,
  );
  rows.push(
    `| **sealed (out of family)** | ${s.joints} | ${s.nominal.toFixed(3)} | ${s.fitted.toFixed(3)} | ${s.phnn.toFixed(3)} | ${s.oracle.toFixed(3)} | **${(s.phnnGapClosed * 100).toFixed(0)} %** | ${(s.fittedGapClosed * 100).toFixed(0)} % |`,
  );
  const plot = await ctx.plot(
    "h10a-gap.svg",
    barChart(
      "Share of the datasheet-to-truth gap each model closes (held out, another pose)",
      [
        ...Object.entries(open.families).map(([family, f]) => ({
          label: family.replace("-", " "),
          values: [Math.max(0, f.fittedGapClosed * 100), Math.max(0, f.phnnGapClosed * 100)],
        })),
        {
          label: "SEALED",
          values: [Math.max(0, s.fittedGapClosed * 100), Math.max(0, s.phnnGapClosed * 100)],
        },
      ],
      ["fitted textbook model (RLS structure)", "learned port-Hamiltonian model"],
      "% of gap",
    ),
  );
  const all = [...open.results, ...sealed.results];
  const minDissipation = Math.min(...all.map((r) => r.energy.minDissipation));
  const maxImbalance = Math.max(...all.map((r) => r.energy.maxImbalance));
  ctx.log(
    `H10a: sealed ${(s.phnnGapClosed * 100).toFixed(0)} % closed (fitted ${(s.fittedGapClosed * 100).toFixed(0)} %)`,
  );
  return [
    {
      id: "H10a",
      title: "A PHNN learns an actuator nobody described",
      target:
        "On the sealed out-of-family actuator: closes ≥ 70 % of the Nominal-to-Oracle gap and beats RLS",
      measured: `sealed: ${(s.phnnGapClosed * 100).toFixed(0)} % of the gap closed; RLS structure ${(s.fittedGapClosed * 100).toFixed(0)} %`,
      verdict:
        s.phnnGapClosed >= 0.7 && s.phnn < s.fitted
          ? "met"
          : s.phnn < s.fitted
            ? "partly met"
            : "not met",
      detail: [
        "Held-out commissioning run at another pose; RMS of the motor-side balance residual after 200 Hz smoothing, N·m (per family, the RMS over its joints). Nominal: datasheet. Fitted: least squares of the node's RLS structure. Oracle: the hidden truth through the same sensors.",
        "",
        "| Family | Joints | Nominal | Fitted | PHNN | Oracle | PHNN closes | Fitted closes |",
        "|---|---|---|---|---|---|---|---|",
        ...rows,
        "",
        "The sealed family (LuGre friction with an internal state, an eccentric gear) was written in NX-04, never used to design or tune anything, and opened once for the final run (NX-15). Two of its six joints end worse than the datasheet: the model has no internal friction state, and one commissioning run covers only part of the gear's eccentricity.",
      ].join("\n"),
      plots: [plot],
    },
    {
      id: "H10c",
      title: "The learned model is passive",
      target:
        "Energy balance dH/dt ≤ input power on held-out trajectories; holds by construction, checked numerically",
      measured: `dissipation never negative (smallest ${minDissipation.toExponential(1)} W); balance closes to ${maxImbalance.toExponential(1)} W on ${all.length} joints`,
      verdict: minDissipation >= 0 && maxImbalance < 1e-9 ? "met" : "not met",
      detail:
        "Along the model's own vector field, Ḣ = ω·g·u + port power − R·ω² − c·φ̇², with R ≥ 0, c ≥ 0 and g > 0 by construction.",
    },
  ];
}
