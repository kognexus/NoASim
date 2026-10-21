/**
 * TypeScript codec for the Nexus bus frames.
 *
 * Mirrors `crates/nexus-proto`. The Rust crate is the source of truth; both codecs are
 * checked against `crates/nexus-proto/golden/frames.txt`, so they cannot drift apart.
 */

export const HEADER_LEN = 8;

export const FrameKind = {
  State: 1,
  Goal: 2,
  Limits: 3,
  Announce: 4,
  Assign: 5,
  Settings: 6,
  Chain: 7,
  Impact: 8,
  Heartbeat: 9,
  Ack: 10,
  Fault: 11,
} as const;
export type FrameKind = (typeof FrameKind)[keyof typeof FrameKind];

/** Destination byte values with special meaning. */
export const Address = {
  host: 0,
  neighbours: 0xfe,
  all: 0xff,
  unassigned: 0xff,
} as const;

export const StatusFlags = {
  ready: 1 << 0,
  fault: 1 << 1,
  networkDamping: 1 << 2,
  stopping: 1 << 3,
  holding: 1 << 4,
  torqueOff: 1 << 5,
  derating: 1 << 6,
} as const;

export interface Header {
  kind: FrameKind;
  source: number;
  /** 0 = host, 1…253 = node, 0xFE = neighbours, 0xFF = all. */
  destination: number;
  cycle: number;
}

export type Vec3f = [number, number, number];

export interface NodeState {
  status: number;
  motorAngle: number;
  motorVelocity: number;
  outputAngle: number;
  outputVelocity: number;
  gearTwist: number;
  gyro: Vec3f;
  accel: Vec3f;
  current: number;
  torqueCommand: number;
  networkTorque: number;
  temperature: number;
  thermalHeadroom: number;
  tankEnergy: number;
  /** Angular velocity of the far end of the sender's link, sender's housing frame. */
  outerRate: Vec3f;
}

export interface Goal {
  dueCycle: number;
  position: number;
  velocity: number;
  sequence: number;
}

export interface Limits {
  maxTorque: number;
  maxVelocity: number;
  minPosition: number;
  maxPosition: number;
  maxTemperature: number;
  networkTorqueShare: number;
}

export interface Announce {
  position: number;
  hasBrake: boolean;
  modelVersion: number;
  gearRatio: number;
  ratedTorque: number;
  peakTorque: number;
  axis: Vec3f;
  link: Vec3f;
  linkMass: number;
}

export interface Assign {
  position: number;
  id: number;
}

/** How nodes damp vibration (PLAN §4 comparison modes). */
export const DampingMode = {
  off: 0,
  local: 1,
  network: 2,
  networkWithSource: 3,
} as const;
export type DampingMode = (typeof DampingMode)[keyof typeof DampingMode];

export interface Settings {
  damping: DampingMode;
  /** 0 = follow host setpoints directly, 1 = joint-level interpolation. */
  interpolation: number;
  /** PLAN §2.1; 0 = retreat and hold. */
  impactPolicy: number;
  /** 0 = nominal, 1 = fitted, 2 = learned. */
  model: number;
  /** Gravity feed-forward source: 0 = off, 1 = own IMU, 2 = encoders from a level base. */
  gravity: number;
  /** Joints that must report an impact before the arm reacts (0 = disarmed, 1 = any). */
  impactConfirm: number;
}

/** Impact policies (PLAN §2.1). */
export const ImpactPolicy = {
  retreatAndHold: 0,
  eStop: 1,
  float: 2,
  /** Comparison only: hold stiffly where the impact was detected. */
  stopInPlace: 3,
} as const;

export interface Impact {
  /** Detection score relative to the threshold. */
  score: number;
  /** Bit 0 = IMU jolt, bit 1 = gear torque. */
  evidence: number;
}

export const FaultCode = {
  unknown: 0,
  impact: 1,
  configRejected: 2,
  commLoss: 3,
  source: 4,
  limit: 5,
} as const;

export interface Fault {
  code: number;
  value: number;
}

/** Gravity chain data (B10); vectors in the sender's housing frame. */
export interface Chain {
  upLink: Vec3f;
  mass: number;
  com: Vec3f;
  flange: Vec3f;
  payloadMass: number;
  payloadLever: number;
  holdingTorque: number;
  /** Σ m·f over the sender's link and beyond, N. */
  dynamicForce: Vec3f;
  /** Moment of those m·f about the sender's joint, N·m. */
  dynamicMoment: Vec3f;
}

export type Frame =
  | { header: Header & { kind: typeof FrameKind.State }; state: NodeState }
  | { header: Header & { kind: typeof FrameKind.Goal }; goal: Goal }
  | { header: Header & { kind: typeof FrameKind.Limits }; limits: Limits }
  | { header: Header & { kind: typeof FrameKind.Announce }; announce: Announce }
  | { header: Header & { kind: typeof FrameKind.Assign }; assign: Assign }
  | { header: Header & { kind: typeof FrameKind.Settings }; settings: Settings }
  | { header: Header & { kind: typeof FrameKind.Chain }; chain: Chain }
  | { header: Header & { kind: typeof FrameKind.Impact }; impact: Impact }
  | { header: Header & { kind: typeof FrameKind.Heartbeat }; heartbeat: true }
  | { header: Header & { kind: typeof FrameKind.Ack }; ack: { code: number } }
  | { header: Header & { kind: typeof FrameKind.Fault }; fault: Fault };

const PAYLOAD_LEN: Record<FrameKind, number> = {
  [FrameKind.State]: 4 + 20 * 4,
  [FrameKind.Goal]: 16,
  [FrameKind.Limits]: 24,
  [FrameKind.Announce]: 4 + 10 * 4,
  [FrameKind.Assign]: 4,
  [FrameKind.Settings]: 8,
  [FrameKind.Chain]: 19 * 4,
  [FrameKind.Impact]: 8,
  [FrameKind.Heartbeat]: 0,
  [FrameKind.Ack]: 4,
  [FrameKind.Fault]: 8,
};

/** Encoded length of a frame of `kind`, header included. */
export function frameLength(kind: FrameKind): number {
  return HEADER_LEN + PAYLOAD_LEN[kind];
}

class Writer {
  readonly view: DataView;
  pos = 0;
  constructor(bytes: Uint8Array) {
    this.view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  }
  u8(v: number): void {
    this.view.setUint8(this.pos, v);
    this.pos += 1;
  }
  u16(v: number): void {
    this.view.setUint16(this.pos, v, true);
    this.pos += 2;
  }
  u32(v: number): void {
    this.view.setUint32(this.pos, v, true);
    this.pos += 4;
  }
  f32(...values: number[]): void {
    for (const v of values) {
      this.view.setFloat32(this.pos, v, true);
      this.pos += 4;
    }
  }
}

class Reader {
  readonly view: DataView;
  pos = 0;
  constructor(bytes: Uint8Array) {
    this.view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  }
  u8(): number {
    return this.view.getUint8(this.pos++);
  }
  u16(): number {
    const v = this.view.getUint16(this.pos, true);
    this.pos += 2;
    return v;
  }
  u32(): number {
    const v = this.view.getUint32(this.pos, true);
    this.pos += 4;
    return v;
  }
  f32(): number {
    const v = this.view.getFloat32(this.pos, true);
    this.pos += 4;
    return v;
  }
  vec3(): Vec3f {
    return [this.f32(), this.f32(), this.f32()];
  }
}

/** Encodes `frame` into a new byte array. */
export function encodeFrame(frame: Frame): Uint8Array {
  const { header } = frame;
  const bytes = new Uint8Array(frameLength(header.kind));
  const w = new Writer(bytes);
  w.u8(header.kind);
  w.u8(header.source);
  w.u8(header.destination);
  w.u8(0);
  w.u32(header.cycle);
  if ("state" in frame) {
    const s = frame.state;
    w.u16(s.status);
    w.u16(0);
    w.f32(s.motorAngle, s.motorVelocity, s.outputAngle, s.outputVelocity, s.gearTwist);
    w.f32(...s.gyro, ...s.accel);
    w.f32(
      s.current,
      s.torqueCommand,
      s.networkTorque,
      s.temperature,
      s.thermalHeadroom,
      s.tankEnergy,
    );
    w.f32(...s.outerRate);
  } else if ("goal" in frame) {
    const g = frame.goal;
    w.u32(g.dueCycle);
    w.f32(g.position, g.velocity);
    w.u16(g.sequence);
    w.u16(0);
  } else if ("limits" in frame) {
    const l = frame.limits;
    w.f32(
      l.maxTorque,
      l.maxVelocity,
      l.minPosition,
      l.maxPosition,
      l.maxTemperature,
      l.networkTorqueShare,
    );
  } else if ("announce" in frame) {
    const a = frame.announce;
    w.u8(a.position);
    w.u8(a.hasBrake ? 1 : 0);
    w.u16(a.modelVersion);
    w.f32(a.gearRatio, a.ratedTorque, a.peakTorque, ...a.axis, ...a.link, a.linkMass);
  } else if ("settings" in frame) {
    const st = frame.settings;
    w.u8(st.damping);
    w.u8(st.interpolation);
    w.u8(st.impactPolicy);
    w.u8(st.model);
    w.u8(st.gravity);
    w.u8(st.impactConfirm);
    w.u16(0);
  } else if ("impact" in frame) {
    w.f32(frame.impact.score);
    w.u8(frame.impact.evidence);
    w.u8(0);
    w.u16(0);
  } else if ("heartbeat" in frame) {
    // No payload.
  } else if ("ack" in frame) {
    w.u8(frame.ack.code);
    w.u8(0);
    w.u16(0);
  } else if ("fault" in frame) {
    w.u8(frame.fault.code);
    w.u8(0);
    w.u16(0);
    w.f32(frame.fault.value);
  } else if ("chain" in frame) {
    const c = frame.chain;
    w.f32(
      ...c.upLink,
      c.mass,
      ...c.com,
      ...c.flange,
      c.payloadMass,
      c.payloadLever,
      c.holdingTorque,
    );
    w.f32(...c.dynamicForce, ...c.dynamicMoment);
  } else {
    w.u8(frame.assign.position);
    w.u8(frame.assign.id);
    w.u16(0);
  }
  return bytes;
}

/** Decodes one frame occupying all of `bytes`; throws on unknown kinds or wrong lengths. */
export function decodeFrame(bytes: Uint8Array): Frame {
  const r = new Reader(bytes);
  const kind = r.u8();
  if (!(kind in PAYLOAD_LEN)) throw new Error(`unknown frame kind ${kind}`);
  if (bytes.length !== frameLength(kind as FrameKind))
    throw new Error(`wrong length for frame kind ${kind}`);
  const source = r.u8();
  const destination = r.u8();
  r.u8();
  const cycle = r.u32();
  switch (kind) {
    case FrameKind.State: {
      const status = r.u16();
      r.u16();
      const state: NodeState = {
        status,
        motorAngle: r.f32(),
        motorVelocity: r.f32(),
        outputAngle: r.f32(),
        outputVelocity: r.f32(),
        gearTwist: r.f32(),
        gyro: r.vec3(),
        accel: r.vec3(),
        current: r.f32(),
        torqueCommand: r.f32(),
        networkTorque: r.f32(),
        temperature: r.f32(),
        thermalHeadroom: r.f32(),
        tankEnergy: r.f32(),
        outerRate: r.vec3(),
      };
      return { header: { kind, source, destination, cycle }, state };
    }
    case FrameKind.Goal: {
      const goal = { dueCycle: r.u32(), position: r.f32(), velocity: r.f32(), sequence: r.u16() };
      return { header: { kind, source, destination, cycle }, goal };
    }
    case FrameKind.Limits: {
      const limits: Limits = {
        maxTorque: r.f32(),
        maxVelocity: r.f32(),
        minPosition: r.f32(),
        maxPosition: r.f32(),
        maxTemperature: r.f32(),
        networkTorqueShare: r.f32(),
      };
      return { header: { kind, source, destination, cycle }, limits };
    }
    case FrameKind.Announce: {
      const announce: Announce = {
        position: r.u8(),
        hasBrake: r.u8() !== 0,
        modelVersion: r.u16(),
        gearRatio: r.f32(),
        ratedTorque: r.f32(),
        peakTorque: r.f32(),
        axis: r.vec3(),
        link: r.vec3(),
        linkMass: r.f32(),
      };
      return { header: { kind, source, destination, cycle }, announce };
    }
    case FrameKind.Settings: {
      const damping = r.u8();
      const settings: Settings = {
        damping: (damping <= 3 ? damping : 0) as DampingMode,
        interpolation: r.u8(),
        impactPolicy: r.u8(),
        model: r.u8(),
        gravity: r.u8(),
        impactConfirm: r.u8(),
      };
      return { header: { kind, source, destination, cycle }, settings };
    }
    case FrameKind.Chain: {
      const chain: Chain = {
        upLink: r.vec3(),
        mass: r.f32(),
        com: r.vec3(),
        flange: r.vec3(),
        payloadMass: r.f32(),
        payloadLever: r.f32(),
        holdingTorque: r.f32(),
        dynamicForce: r.vec3(),
        dynamicMoment: r.vec3(),
      };
      return { header: { kind, source, destination, cycle }, chain };
    }
    case FrameKind.Impact: {
      const impact = { score: r.f32(), evidence: r.u8() };
      return { header: { kind, source, destination, cycle }, impact };
    }
    case FrameKind.Heartbeat:
      return { header: { kind, source, destination, cycle }, heartbeat: true };
    case FrameKind.Ack:
      return { header: { kind, source, destination, cycle }, ack: { code: r.u8() } };
    case FrameKind.Fault: {
      const code = r.u8();
      r.u8();
      r.u16();
      return { header: { kind, source, destination, cycle }, fault: { code, value: r.f32() } };
    }
    default: {
      const assign = { position: r.u8(), id: r.u8() };
      return { header: { kind: FrameKind.Assign, source, destination, cycle }, assign };
    }
  }
}

/** Packs frames into mailbox records: `[length: u16 LE][frame]` each. */
export function encodeRecords(frames: readonly Uint8Array[]): Uint8Array {
  const total = frames.reduce((n, f) => n + 2 + f.length, 0);
  const out = new Uint8Array(total);
  const view = new DataView(out.buffer);
  let pos = 0;
  for (const f of frames) {
    view.setUint16(pos, f.length, true);
    out.set(f, pos + 2);
    pos += 2 + f.length;
  }
  return out;
}

/** Splits mailbox records back into frame byte arrays (views, not copies). */
export function decodeRecords(bytes: Uint8Array): Uint8Array[] {
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const frames: Uint8Array[] = [];
  for (let pos = 0; pos + 2 <= bytes.length;) {
    const len = view.getUint16(pos, true);
    frames.push(bytes.subarray(pos + 2, pos + 2 + len));
    pos += 2 + len;
  }
  return frames;
}
