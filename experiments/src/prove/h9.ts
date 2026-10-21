/**
 * H9: each joint computes its gravity torque from its own IMU. The KX-6 with a 3 kg
 * payload holds still; then the base tilts 15° sideways; then it accelerates sideways.
 * The comparison is the same nodes with a host-style model that assumes a level base
 * (gravity from the encoder angles).
 */
import { KX6 } from "../../../sim/src/plant/kx6.ts";
import { hold, movingBase, tiltedGravity } from "../../../sim/src/scenarios/gravity.ts";
import { withArm } from "../runner.ts";
import type { Finding, ProveContext } from "./findings.ts";

const IMU = 1;
const LEVEL_BASE = 2;

export async function h9(ctx: ProveContext): Promise<Finding> {
  const seed = 5;
  const atRest = await withArm(ctx.lab, { seed }, (system) => {
    system.host.applySettings({ gravity: IMU });
    const result = hold(system, 5);
    const payload = system.host.lastReceived.flatMap((f) =>
      "chain" in f && f.header.source === 6 ? [f.chain.payloadMass] : [],
    )[0];
    return { errors: result.torqueError, payload };
  });
  const drift = (gravity: number) =>
    withArm(ctx.lab, { seed }, (system) => {
      system.host.applySettings({ gravity });
      const level = hold(system, 3);
      system.plant.setGravity(tiltedGravity(15));
      const tilted = hold(system, 3);
      return Math.hypot(...tilted.tool.map((v, k) => v - (level.tool[k] ?? 0)));
    });
  const shaking = (gravity: number) =>
    withArm(ctx.lab, { seed }, (system) => {
      system.host.applySettings({ gravity });
      system.run(3);
      return movingBase(system, 2, 1, 4);
    });
  const [tiltImu, tiltLevel, moveImu, moveLevel] = [
    await drift(IMU),
    await drift(LEVEL_BASE),
    await shaking(IMU),
    await shaking(LEVEL_BASE),
  ];
  const shares = atRest.errors.map(
    (e, i) => Math.abs(e) / (KX6.joints[i]?.actuator.ratedTorque ?? 1),
  );
  const within = shares.filter((s) => s <= 0.01).length;
  const payloadError = ((atRest.payload ?? 0) - KX6.payload.mass) / KX6.payload.mass;
  const tiltBetter = tiltLevel > 2 * tiltImu;
  const moveBetter = moveLevel > 2 * moveImu;
  const payloadOk = Math.abs(payloadError) <= 0.05;
  ctx.log(
    `H9: ${within}/6 joints within 1 %, payload ${(payloadError * 100).toFixed(0)} %, tilt ${(tiltImu * 1e3).toFixed(2)} vs ${(tiltLevel * 1e3).toFixed(2)} mm`,
  );
  return {
    id: "H9",
    title: "Each joint computes gravity correctly from its own IMU",
    target:
      "Torque error ≤ 1 % of rated; drift with tilt clearly lower than a fixed-vertical model; payload within ±5 %",
    measured: `${within}/6 joints within 1 %; tilt drift ${(tiltImu * 1e3).toFixed(2)} vs ${(tiltLevel * 1e3).toFixed(2)} mm; payload ${(payloadError * 100).toFixed(0)} %`,
    verdict:
      within === 6 && tiltBetter && moveBetter && payloadOk
        ? "met"
        : tiltBetter && moveBetter
          ? "partly met"
          : "not met",
    detail: [
      "| Measure | Target | Result |",
      "|---|---|---|",
      ...shares.map(
        (s, i) =>
          `| Gravity feed-forward error, joint ${i + 1} | ≤ 1 % of rated | ${(s * 100).toFixed(1)} % (${Math.abs(atRest.errors[i] ?? 0).toFixed(2)} N·m) |`,
      ),
      `| Payload mass (3 kg) | ±5 % | ${(atRest.payload ?? 0).toFixed(2)} kg (${(payloadError * 100).toFixed(0)} %) |`,
      `| Tool drift, base tilted 15° sideways | clearly lower than the level-base model | ${(tiltImu * 1e3).toFixed(2)} mm vs ${(tiltLevel * 1e3).toFixed(2)} mm |`,
      `| Tool RMS, base accelerating 2 m/s² at 1 Hz | clearly lower than the level-base model | ${(moveImu * 1e3).toFixed(2)} mm vs ${(moveLevel * 1e3).toFixed(2)} mm |`,
      "",
      within === 6
        ? ""
        : "Why some joints miss 1 %: the base joint reads a 20 mg accelerometer bias as a 1.1° tilt, and the wrist joints assume the payload sits on the flange while it sits 5 cm beyond, which also makes the payload estimate heavy (NX-08). Estimating the payload's centre of mass and calibrating accelerometer offsets at commissioning would remove most of both.",
    ].join("\n"),
  };
}
