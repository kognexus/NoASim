/**
 * How much each joint vibrates, from what it reports on the bus: the angular rate its
 * housing IMU measures, high-passed so commanded motion drops out, as an RMS over a
 * short window. This is the shaking each actuator itself feels.
 */
import type { Host } from "../host/host.ts";
import { BUS_CYCLE_HZ } from "../index.ts";

/** Below this, rotation counts as commanded motion, not vibration, Hz. */
const HIGHPASS_HZ = 3;
/** Averaging time of the RMS, s. */
const WINDOW_S = 0.05;

/** First-order high-pass of a three-axis signal. */
class HighPass3 {
  readonly #a: number;
  #x: number[] | undefined;
  #y = [0, 0, 0];

  constructor(hz: number, dt: number) {
    const rc = 1 / (2 * Math.PI * hz);
    this.#a = rc / (rc + dt);
  }

  update(x: readonly number[]): number[] {
    const last = this.#x ?? [...x];
    this.#y = this.#y.map((y, i) => this.#a * (y + (x[i] ?? 0) - (last[i] ?? 0)));
    this.#x = [...x];
    return this.#y;
  }
}

export class VibrationMeter {
  readonly #host: Host;
  readonly #filters: HighPass3[];
  readonly #meanSquare: number[];
  readonly #alpha = 1 / (WINDOW_S * BUS_CYCLE_HZ);

  constructor(host: Host, joints: number) {
    this.#host = host;
    this.#filters = Array.from(
      { length: joints },
      () => new HighPass3(HIGHPASS_HZ, 1 / BUS_CYCLE_HZ),
    );
    this.#meanSquare = new Array<number>(joints).fill(0);
  }

  /** Call after every bus cycle. */
  observe(): void {
    for (const frame of this.#host.lastReceived) {
      if (!("state" in frame)) continue;
      const i = frame.header.source - 1;
      const filter = this.#filters[i];
      if (!filter) continue;
      const [x = 0, y = 0, z = 0] = filter.update(frame.state.gyro);
      const ms = this.#meanSquare[i] ?? 0;
      this.#meanSquare[i] = ms + this.#alpha * (x * x + y * y + z * z - ms);
    }
  }

  /** Each joint's vibration, deg/s RMS. */
  levels(): number[] {
    return this.#meanSquare.map((ms) => (Math.sqrt(ms) * 180) / Math.PI);
  }
}
