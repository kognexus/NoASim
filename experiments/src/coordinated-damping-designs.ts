/**
 * NX-30: commits the coordinated-damping design of each arm for the demo
 * (`models/coordinated-damping/<arm>.json`): the demo's arm as it starts, with datasheet actuators,
 * no payload (the arm's spec carries 3 kg) and its links as specified, holding at the home
 * pose.
 *
 *   node experiments/src/coordinated-damping-designs.ts
 */
import { mkdir, writeFile } from "node:fs/promises";
import { coordinatedDocument, designLqg, loopModel } from "../../sim/src/analysis/estimator.ts";
import { DampingMode } from "../../sim/src/bus/protocol.ts";
import { nodeConfigsFor } from "../../sim/src/harness/gains.ts";
import { tunedGains } from "../../sim/src/harness/tuned.ts";
import { BUS_CYCLE_HZ } from "../../sim/src/index.ts";
import { KX6, KX6_HOME, KX6L, withInitialAngles } from "../../sim/src/plant/kx6.ts";
import { openLab, withArm } from "./runner.ts";

/** As in `experiments/src/estimation.ts`. */
const DESIGNS = {
  kx6l: { spec: KX6L, torqueWeight: 3e-5 },
  kx6: { spec: KX6, torqueWeight: 3e-6 },
} as const;

const lab = await openLab();
const dir = new URL("../../models/coordinated-damping/", import.meta.url);
await mkdir(dir, { recursive: true });
for (const [arm, { spec, torqueWeight }] of Object.entries(DESIGNS)) {
  const at = withInitialAngles(spec, KX6_HOME);
  const home = { ...at, payload: { ...at.payload, mass: 0 } };
  const gains = tunedGains(arm, "off");
  const configs = nodeConfigsFor(home, BUS_CYCLE_HZ, gains);
  const doc = await withArm(lab, { spec: home, gains, seed: 31 }, (s) => {
    s.host.applySettings({ damping: DampingMode.off });
    s.run(5.3);
    const model = loopModel(s, configs);
    const lqg = designLqg(model, configs, { torqueWeight, speedWeight: 1e-4, forceNoise: 20 }, 2);
    return coordinatedDocument(arm, home.payload.mass, model, lqg, 2, 1 / BUS_CYCLE_HZ);
  });
  await writeFile(new URL(`${arm}.json`, dir), `${JSON.stringify(doc)}\n`);
  console.log(`models/coordinated-damping/${arm}.json: ${doc.states} states`);
}
