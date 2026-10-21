/**
 * The interactive demo (NX-20): two identical arms side by side. Each arm can use its own
 * damping mode and model; every disturbance hits both at the same bus cycle.
 */
import loadMujoco from "@mujoco/mujoco";
import nodeWasmUrl from "../../target/wasm32-unknown-unknown/release/nexus_wasm.wasm?url";
import type { CoordinatedDocument } from "../../sim/src/analysis/estimator.ts";
import type { BusOptions } from "../../sim/src/bus/ethercatBus.ts";
import type { PhnnDocument } from "../../sim/src/node/phnnModel.ts";
import type { Vec3 } from "../../sim/src/plant/armSpec.ts";
import { BusMonitor, busJointAt, drawBus, rateRows } from "./busView.ts";
import { StripChart } from "./chart.ts";
import {
  ArmRig,
  BUS_CYCLE_HZ,
  DampingMode,
  DEFAULT_SHARED,
  type SharedSettings,
  type SideSettings,
  type RippleKind,
  type RobotComputer,
  type TaskName,
  TwinRig,
} from "./rig.ts";
import { STATUS_LABEL, STATUS_VAR } from "./status.ts";
import { VibrationMeter } from "../../sim/src/analysis/vibration.ts";
import { ArmView } from "./view.ts";

const $ = <T extends HTMLElement>(selector: string, root: ParentNode = document): T => {
  const el = root.querySelector<T>(selector);
  if (!el) throw new Error(`missing ${selector}`);
  return el;
};

const MODES: [string, DampingMode][] = [
  ["Joint control only", DampingMode.off],
  ["Joint-local damping", DampingMode.local],
  ["Network damping", DampingMode.network],
  ["Network + report to source", DampingMode.networkWithSource],
  ["Coordinated (while holding)", DampingMode.coordinated],
];
const MODELS: [string, number][] = [
  ["Datasheet model", 0],
  ["Fitted (RLS)", 1],
  ["Learned (PHNN)", 2],
];

/** The committed coordinated-damping designs, by arm (NX-30). */
const COORDINATED: Record<string, CoordinatedDocument> = Object.fromEntries(
  Object.entries(
    import.meta.glob<CoordinatedDocument>("../../models/coordinated-damping/*.json", {
      import: "default",
      eager: true,
    }),
  ).map(([path, doc]) => [path.split("/").pop()?.replace(".json", "") ?? "", doc]),
);
/** Committed per-joint models, by unit (`models/<unit>/joint<n>.json`), loaded on demand. */
const MODEL_FILES = import.meta.glob<PhnnDocument>("../../models/*/joint*.json", {
  import: "default",
});
const UNITS = [
  ...new Set(Object.keys(MODEL_FILES).map((path) => path.split("/").at(-2) ?? "")),
].sort();

async function unitModels(unit: string): Promise<PhnnDocument[] | undefined> {
  const files = Object.entries(MODEL_FILES)
    .filter(([path]) => path.split("/").at(-2) === unit)
    .sort(([a], [b]) => a.localeCompare(b, undefined, { numeric: true }));
  if (files.length === 0) return undefined;
  return Promise.all(files.map(([, load]) => load()));
}

const [mujoco, nodeModule] = await Promise.all([
  loadMujoco(),
  WebAssembly.compileStreaming(fetch(nodeWasmUrl)),
]);

// ---- state ------------------------------------------------------------------------------
let shared: SharedSettings = { ...DEFAULT_SHARED, unit: "datasheet" };
const sides: SideSettings[] = [
  { damping: DampingMode.network, model: 0 },
  { damping: DampingMode.off, model: 0 },
];
let bus: BusOptions = { latencyCycles: 1, jitterCycles: 0, dropRate: 0 };
let task: TaskName = "hold";
let computer: RobotComputer = "vla";
let selected = -1;
let twin: TwinRig | undefined;
let monitors: BusMonitor[] = [];
let meters: VibrationMeter[] = [];

const armEls = [...document.querySelectorAll<HTMLElement>(".arm")];
const views: ArmView[] = [];
/** Chart samples kept: 5 s at one sample per 10 ms. */
const CHART_SAMPLES = 500;
const JOINT_COUNT = 6;
/** One line per arm, in the arm's colour. */
const ARM_SERIES = [
  { label: "left arm", colourVar: "--left" },
  { label: "right arm", colourVar: "--right" },
];
/** Both arms' tool distance from target, on one chart. */
const distanceChart = new StripChart($("#distance"), ARM_SERIES, CHART_SAMPLES, "mm");
/**
 * Both arms' vibration at one joint, on one chart; one chart per joint keeps each joint's
 * history, and the dropdown picks which is drawn.
 */
const vibrationCharts = Array.from(
  { length: JOINT_COUNT },
  () => new StripChart($("#vibration"), ARM_SERIES, CHART_SAMPLES, "°/s"),
);
let vibrationJoint = Number($<HTMLSelectElement>("#vibration-joint").value);

// ---- building ---------------------------------------------------------------------------
async function build(): Promise<void> {
  const busy = $("#busy");
  busy.hidden = false;
  await new Promise((resolve) => requestAnimationFrame(resolve));
  // Nothing may step the old arms while the new ones load: the frame loop skips while
  // `twin` is unset, and stepping a disposed arm would throw.
  const old = twin;
  twin = undefined;
  old?.dispose();
  const models = shared.unit === "datasheet" ? undefined : await unitModels(shared.unit);
  const [left, right] = await Promise.all(
    sides.map((side) =>
      ArmRig.create(mujoco, nodeModule, shared, side, models, bus, COORDINATED[shared.arm]),
    ),
  );
  if (!left || !right) throw new Error("arms not built");
  twin = new TwinRig(left, right);
  monitors = twin.arms.map((arm) => new BusMonitor(arm.system.host));
  meters = twin.arms.map((arm) => new VibrationMeter(arm.system.host, arm.spec.joints.length));
  twin.both((arm) => arm.startTask(task, computer));
  twin.arms.forEach((arm, i) => {
    const canvas = $<HTMLCanvasElement>(".view", armEls[i]);
    let view = views[i];
    if (!view) {
      // The right view shares the left one's camera: both always show the same angle.
      view = new ArmView(canvas, arm.system.plant, views[0]);
      view.onSelect = selectJoint;
      views[i] = view;
    } else {
      view.attach(arm.system.plant);
    }
  });
  for (const chart of [distanceChart, ...vibrationCharts]) chart.clear();
  updateJointPanel();
  showActuatorDetail();
  busy.hidden = true;
}

function selectJoint(joint: number): void {
  selected = joint;
  if (joint >= 0) showVibrationOf(joint);
  for (const view of views) view.select(joint);
  $("#joint-label").textContent = joint >= 0 ? `J${joint + 1}` : "click a joint";
  updateJointPanel();
}

/** Ripple buttons and the ripple line: what the selected joint and the arm have now. */
function updateJointPanel(): void {
  const ripples = twin?.arms[0].ripples ?? new Map<number, RippleKind>();
  const current = selected >= 0 ? ripples.get(selected) : undefined;
  for (const [id, kind, name] of [
    ["#ripple", "fixable", "fixable ripple"],
    ["#damaged", "unfixable", "unfixable ripple"],
  ] as const) {
    const button = $<HTMLButtonElement>(id);
    button.disabled = selected < 0;
    button.setAttribute("aria-pressed", String(current === kind));
    button.textContent = current === kind ? `Remove ${name}` : `Add ${name}`;
  }
  $("#ripple-state").textContent =
    ripples.size === 0
      ? "No ripple applied."
      : `Ripple on: ${[...ripples]
          .sort(([a], [b]) => a - b)
          .map(([j, kind]) => `J${j + 1} (${kind})`)
          .join(", ")}`;
}

/** Adds `kind` of ripple to the selected joint, or removes it if it is already there. */
function toggleRipple(kind: RippleKind): void {
  if (!twin || selected < 0) return;
  const next = twin.arms[0].ripples.get(selected) === kind ? undefined : kind;
  twin.both((arm) => arm.setRipple(selected, next));
  updateJointPanel();
}

/** What each joint's actuator is like, under the actuator selector. */
function showActuatorDetail(): void {
  const descriptions = twin?.arms[0].actuatorDescriptions() ?? [];
  $("#actuator-detail").textContent =
    shared.unit === "datasheet"
      ? "Every joint's actuator is exactly as specified."
      : new Set(descriptions).size === 1
        ? `Every joint: ${descriptions[0]}.`
        : descriptions.map((d, j) => `J${j + 1}: ${d}`).join("; ");
}

// ---- controls ---------------------------------------------------------------------------
function fillSelect(select: HTMLSelectElement, options: [string, number][], value: number): void {
  select.replaceChildren(
    ...options.map(([label, v]) => new Option(label, String(v), false, v === value)),
  );
}

/** The comparison charts name each arm by its damping mode. */
function labelChart(): void {
  const labels = sides.map((side, i) => {
    const mode = MODES.find(([, m]) => m === side.damping)?.[0] ?? "";
    return `${i === 0 ? "left" : "right"}: ${mode}`;
  });
  for (const chart of [distanceChart, ...vibrationCharts]) chart.setLabels(labels);
}

function showVibrationOf(joint: number): void {
  vibrationJoint = joint;
  $<HTMLSelectElement>("#vibration-joint").value = String(joint);
}

armEls.forEach((el, i) => {
  const side = sides[i];
  if (!side) return;
  const mode = $<HTMLSelectElement>(".mode", el);
  const model = $<HTMLSelectElement>(".model", el);
  fillSelect(mode, MODES, side.damping);
  fillSelect(model, MODELS, side.model);
  const update = () => {
    const next = { damping: Number(mode.value) as DampingMode, model: Number(model.value) };
    sides[i] = next;
    twin?.arms[i]?.apply(next);
    labelChart();
  };
  mode.addEventListener("change", update);
  model.addEventListener("change", update);
});

/** Units with a plain name of their own; the others are numbered imperfect sets. */
const NAMED_UNITS: [string, string][] = [
  ["premium", "Premium actuators"],
  ["budget", "Budget actuators"],
  ["sealed", "Out-of-family test set"],
];
const unitName = (unit: string) => NAMED_UNITS.find(([key]) => unit.includes(key))?.[1];
const numbered = UNITS.filter((u) => !unitName(u));
for (const unit of [...numbered, ...UNITS.filter((u) => unitName(u))]) {
  const name = unitName(unit) ?? `Imperfect set ${numbered.indexOf(unit) + 1}`;
  $<HTMLSelectElement>("#unit").add(new Option(`${name}, with learned models (stiff links)`, unit));
}

labelChart();

$("#computer").addEventListener("change", (e) => {
  // The switch takes effect with the next task; a running one restarts on the new computer.
  computer = (e.target as HTMLSelectElement).value as RobotComputer;
  twin?.both((arm) => arm.startTask(task, computer));
});

$("#task").addEventListener("click", (e) => {
  const button = (e.target as HTMLElement).closest<HTMLButtonElement>("button[data-task]");
  if (!button) return;
  task = button.dataset["task"] as TaskName;
  for (const b of $("#task").querySelectorAll("button"))
    b.setAttribute("aria-pressed", String(b === button));
  twin?.both((arm) => arm.startTask(task, computer));
});

const sideways: Vec3 = [0, 1, 0];
$("#tap").addEventListener("click", () => twin?.both((arm) => arm.tap([0, 0.6, -0.8])));
$("#shake").addEventListener("click", () =>
  twin?.both((arm) => arm.shake(shared.arm === "kx6l" ? 6 : 9)),
);
function setReaction(on: boolean): void {
  shared = { ...shared, reactToImpacts: on };
  $<HTMLInputElement>("#react").checked = on;
  twin?.both((arm) => arm.system.host.applySettings({ impactConfirm: on ? 1 : 0 }));
}
$("#react").addEventListener("change", (e) => setReaction((e.target as HTMLInputElement).checked));
$("#push").addEventListener("click", () => {
  // A collision demo needs the joints to react.
  setReaction(true);
  twin?.both((arm) => arm.push(sideways));
});
$("#ripple").addEventListener("click", () => toggleRipple("fixable"));
$("#damaged").addEventListener("click", () => toggleRipple("unfixable"));
$("#clear").addEventListener("click", () => {
  twin?.both((arm) => arm.clearRipples());
  updateJointPanel();
});
$("#ack").addEventListener("click", () => twin?.both((arm) => arm.acknowledge()));
$("#estop").addEventListener("click", () => twin?.both((arm) => arm.emergencyStop()));
$("#reset").addEventListener("click", () => void build());

$("#policy").addEventListener("change", (e) => {
  shared = { ...shared, impactPolicy: Number((e.target as HTMLSelectElement).value) };
  twin?.both((arm) => arm.system.host.applySettings({ impactPolicy: shared.impactPolicy }));
});
$("#arm").addEventListener("change", (e) => {
  shared = { ...shared, arm: (e.target as HTMLSelectElement).value as SharedSettings["arm"] };
  void build();
});
$("#unit").addEventListener("change", (e) => {
  const unit = (e.target as HTMLSelectElement).value;
  // Learned models exist for the KX-6 units only.
  shared = { ...shared, unit, ...(unit === "datasheet" ? {} : { arm: "kx6" as const }) };
  $<HTMLSelectElement>("#arm").value = shared.arm;
  void build();
});
$("#brakes").addEventListener("change", (e) => {
  shared = { ...shared, brakes: (e.target as HTMLInputElement).checked };
  void build();
});
/** Arm-editor sliders: show the value while dragging, rebuild when released. */
function editorSlider(id: string, show: (v: number) => string, set: (v: number) => void): void {
  const input = $<HTMLInputElement>(`#${id}`);
  input.addEventListener("input", () => ($(`#${id}-out`).textContent = show(Number(input.value))));
  input.addEventListener("change", () => {
    set(Number(input.value));
    void build();
  });
}
editorSlider(
  "payload",
  (v) => `${v} kg`,
  (v) => (shared = { ...shared, payload: v }),
);
editorSlider(
  "stiffness",
  (v) => `${v} %`,
  (v) => (shared = { ...shared, linkStiffness: v / 100 }),
);

$("#vibration-joint").addEventListener("change", (e) => {
  showVibrationOf(Number((e.target as HTMLSelectElement).value));
});

function busSlider(id: string, apply: (value: number) => string): void {
  const input = $<HTMLInputElement>(`#${id}`);
  input.addEventListener("input", () => {
    $(`#${id}-out`).textContent = apply(Number(input.value));
    twin?.both((arm) => arm.system.bus.configure(bus));
  });
}
busSlider("latency", (v) => {
  bus = { ...bus, latencyCycles: v };
  return String(v);
});
busSlider("jitter", (v) => {
  bus = { ...bus, jitterCycles: v };
  return String(v);
});
busSlider("drop", (v) => {
  bus = { ...bus, dropRate: v / 100 };
  return `${v} %`;
});

window.addEventListener("resize", () => views.forEach((v) => v.resize()));

// A joint can also be selected by clicking its node in either arm's bus view.
for (const el of armEls) {
  const canvas = $<HTMLCanvasElement>(".bus", el);
  canvas.addEventListener("click", (e) => {
    const joint = busJointAt(canvas, e.clientX, JOINT_COUNT);
    if (joint >= 0) selectJoint(joint);
  });
  canvas.addEventListener("mousemove", (e) => {
    canvas.style.cursor = busJointAt(canvas, e.clientX, JOINT_COUNT) >= 0 ? "pointer" : "";
  });
}

// What the joint disc colours mean (the same colours as in the bus views).
$("#status-legend").replaceChildren(
  Object.assign(document.createElement("span"), { textContent: "Joint colours:" }),
  ...STATUS_LABEL.map(([state, text]) => {
    const item = document.createElement("span");
    const dot = document.createElement("i");
    dot.style.background = `var(${STATUS_VAR[state]})`;
    item.append(dot, text);
    return item;
  }),
);

// Help tooltips open toward the side of the screen with room.
for (const help of document.querySelectorAll<HTMLElement>(".help")) {
  const place = () =>
    help.classList.toggle("tip-left", help.getBoundingClientRect().left > window.innerWidth / 2);
  help.addEventListener("mouseenter", place);
  help.addEventListener("focus", place);
}

// ---- loop -------------------------------------------------------------------------------
/** Real time spent stepping per frame at most, ms; simulated time slows if it is not enough. */
const STEP_BUDGET_MS = 14;
/** Bus cycles between chart samples (10 ms). */
const SAMPLE_CYCLES = 40;
let last = performance.now();
let rateWindow = { real: 0, sim: 0, shown: performance.now() };

function frame(now: number): void {
  // Scheduled first, so one failing frame cannot stop the loop.
  requestAnimationFrame(frame);
  const elapsed = Math.min(0.1, (now - last) / 1000);
  last = now;
  if (twin) {
    const wanted = Math.round(elapsed * BUS_CYCLE_HZ);
    const started = performance.now();
    let done = 0;
    while (done < wanted && performance.now() - started < STEP_BUDGET_MS) {
      for (let k = 0; k < SAMPLE_CYCLES; k++) {
        twin.run(1);
        twin.arms.forEach((arm, i) => {
          monitors[i]?.observe(arm.cycle);
          meters[i]?.observe();
        });
      }
      done += SAMPLE_CYCLES;
      distanceChart.push(twin.arms.map((arm) => arm.deviation() * 1e3));
      const levels = meters.map((m) => m.levels());
      vibrationCharts.forEach((chart, j) => chart.push(levels.map((l) => l[j] ?? 0)));
    }
    rateWindow = {
      ...rateWindow,
      real: rateWindow.real + elapsed,
      sim: rateWindow.sim + done / BUS_CYCLE_HZ,
    };
    twin.arms.forEach((arm, i) => {
      views[i]?.render(monitors[i]?.status ?? new Map());
      $(".dev", armEls[i]).textContent = `${(arm.deviation() * 1e3).toFixed(2)} mm`;
    });
    distanceChart.draw();
    vibrationCharts[vibrationJoint]?.draw();
    monitors.forEach((monitor, i) => {
      const el = armEls[i];
      if (!el) return;
      const rippled = new Set(twin?.arms[i]?.ripples.keys() ?? []);
      drawBus($(".bus", el), monitor, JOINT_COUNT, selected, rippled);
      if (now - rateWindow.shown > 500) showBusText(monitor, el);
    });
    if (now - rateWindow.shown > 500) {
      $("#sim-time").textContent = `${(twin.arms[0].cycle / BUS_CYCLE_HZ).toFixed(1)} s simulated`;
      $("#rtf").textContent =
        `${(rateWindow.sim / Math.max(1e-6, rateWindow.real)).toFixed(2)}× real time`;
      rateWindow = { real: 0, sim: 0, shown: now };
    }
  }
}

/** The frame rates and event log under arm `el`. */
function showBusText(monitor: BusMonitor, el: HTMLElement): void {
  $(".rates", el).replaceChildren(
    ...rateRows(monitor).map(([name, rate]) => {
      const row = document.createElement("tr");
      row.innerHTML = `<td>${name}</td><td>${rate.toFixed(0)}/s</td>`;
      return row;
    }),
  );
  $(".events", el).replaceChildren(
    ...[...monitor.events].reverse().map((event) => {
      const li = document.createElement("li");
      li.dataset["severity"] = event.severity;
      li.textContent = `${event.time.toFixed(2)} s  ${event.text}`;
      return li;
    }),
  );
}

await build();
requestAnimationFrame(frame);
