import { describe, expect, it } from "vitest";
import { type Finding, resultsMarkdown } from "./findings.ts";
import { barChart, lineChart } from "./svg.ts";

describe("proof report (NX-19)", () => {
  it("lists every finding with its verdict and keeps missed targets visible", () => {
    const findings: Finding[] = [
      { id: "H1", title: "A", target: "≥ 10×", measured: "2.8×", verdict: "partly met" },
      {
        id: "H4",
        title: "B",
        target: "0",
        measured: "0 of 15",
        verdict: "met",
        plots: ["results/x.svg"],
      },
      { id: "H10a", title: "C", target: "≥ 70 %", measured: "61 %", verdict: "not met" },
    ];
    const md = resultsMarkdown(findings, { quick: false, seconds: 60 });
    expect(md).toContain("| H1 | A | ≥ 10× | 2.8× | ◐ partly met |");
    expect(md).toContain("| H10a | C | ≥ 70 % | 61 % | ✘ not met |");
    expect(md).toContain("![H4](results/x.svg)");
    expect(md).not.toContain("Quick run");
    expect(resultsMarkdown(findings, { quick: true, seconds: 60 })).toContain("Quick run");
  });

  it("writes self-contained SVG with escaped labels", () => {
    const line = lineChart("a < b", [{ label: "x & y", x: [0, 1, 2], y: [0, 2, 1] }], "s", "mm");
    expect(line.startsWith("<svg")).toBe(true);
    expect(line).toContain("a &lt; b");
    expect(line).toContain("x &amp; y");
    expect(line).toContain("<polyline");
    const bars = barChart("t", [{ label: "g", values: [1, 3] }], ["one", "two"], "n");
    expect(bars.match(/<rect /g)?.length).toBe(1 + 2 + 2); // background, two bars, two swatches
  });
});
