import {ByteBuffer} from 'flatbuffers';

import {Frame} from './generated/talos/telemetry/frame';
import {Pose as WirePose} from './generated/talos/telemetry/pose';

export type Pose = {
  x: number; y: number; z: number; roll: number; pitch: number; yaw: number
};
export type Snapshot = {
  timestamp_ns: string; sequence_id: string; chassis: Pose;
  ghosts: {name: string; pose: Pose}[];
  channels: Record<string, number>;
  units: Record<string, string>;
  targets: {id: number; pose: Pose}[];
  elevator_m: number;
  arm_rad: number;
  camera: Pose
};
export type Mode = 'PAUSED'|'LIVE_STREAMING'|'REPLAY_PLAYING';
export const MAX_FRAME_BYTES = 65507;

// JS generated accessors do not verify untrusted offsets. Verify this schema
// before constructing views; all offsets below are relative to the datagram.
function verify(bytes: Uint8Array) {
  const v = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const check = (p: number, n: number) => {
    if (!Number.isInteger(p) || p < 0 || n < 0 || p + n > bytes.length)
      throw Error('Truncated FlatBuffer');
  };
  const u32 = (p: number) => {
    check(p, 4);
    return v.getUint32(p, true);
  };
  const ref = (p: number) => {
    const n = u32(p);
    if (n < 4) throw Error('Invalid offset');
    const q = p + n;
    check(q, 4);
    return q;
  };
  if (bytes.length < 16 || bytes.length > MAX_FRAME_BYTES ||
      u32(0) !== bytes.length - 4)
    throw Error('Invalid size prefix');
  if (String.fromCharCode(...bytes.subarray(8, 12)) !== 'TLMS')
    throw Error('Invalid identifier');
  const table = (p: number, widths: number[]) => {
    check(p, 4);
    const vt = p - v.getInt32(p, true);
    check(vt, 4);
    const n = v.getUint16(vt, true), size = v.getUint16(vt + 2, true);
    if (n < 4 || n % 2 || size < 4) throw Error('Invalid table');
    check(vt, n);
    check(p, size);
    return widths.map((w, i) => {
      if (4 + i * 2 >= n) return 0;
      const o = v.getUint16(vt + 4 + i * 2, true);
      if (!o) return 0;
      if (o < 4 || o + w > size) throw Error('Invalid field');
      check(p + o, w);
      return p + o;
    });
  };
  const string = (p: number) => {
    if (!p) throw Error('Missing name');
    const q = ref(p), n = u32(q);
    if (n > 256) throw Error('Name too long');
    check(q + 4, n + 1);
    if (v.getUint8(q + 4 + n) !== 0) throw Error('Invalid string');
  };
  const vector = (p: number, widths: number[], kind: string) => {
    if (!p) return;
    const q = ref(p), n = u32(q);
    if (n > 256) throw Error('Too many fields');
    check(q + 4, n * 4);
    for (let i = 0; i < n; i++) {
      const f = table(ref(q + 4 + i * 4), widths);
      if (kind !== 'target') string(f[0]);
      if (kind === 'channel' && f[2]) string(f[2]);
    }
  };
  const f = table(ref(4), [8, 8, 48, 4, 4, 4, 8, 8, 48]);
  if (!f[2]) throw Error('Missing chassis');
  vector(f[3], [4, 48], 'ghost');
  vector(f[4], [4, 8, 4], 'channel');
  vector(f[5], [4, 48], 'target');
}
const finite = (n: number) => {
  if (!Number.isFinite(n)) throw Error('Non-finite telemetry');
  return n;
};
const pose = (p: WirePose|null): Pose => {
  if (!p) throw Error('Missing pose');
  return {
    x: finite(p.x()),
    y: finite(p.y()),
    z: finite(p.z()),
    roll: finite(p.roll()),
    pitch: finite(p.pitch()),
    yaw: finite(p.yaw())
  };
};
export function decode(bytes: Uint8Array): Snapshot {
  verify(bytes);
  const f = Frame.getSizePrefixedRootAsFrame(new ByteBuffer(bytes));
  const channels: Record < string, number >= Object.create(null),
      units: Record < string, string >= Object.create(null);
  for (let i = 0; i < f.channelsLength(); i++) {
    const c = f.channels(i)!;
    const name = c.name()!;
    if (Object.hasOwn(channels, name)) throw Error('Duplicate channel');
    channels[name] = finite(c.value());
    units[name] = c.unit() ?? '';
  }
  const chassis = pose(f.chassis());
  return {
    timestamp_ns: f.timestampNs().toString(),
    sequence_id: f.sequenceId().toString(),
    chassis,
    ghosts: Array.from(
        {length: f.ghostsLength()},
        (_, i) => {
          const g = f.ghosts(i)!;
          return {name: g.name()!, pose: pose(g.pose())};
        }),
    channels,
    units,
    targets: Array.from(
        {length: f.targetsLength()},
        (_, i) => {
          const t = f.targets(i)!;
          return {id: t.id(), pose: pose(t.pose())};
        }),
    elevator_m: finite(f.elevatorM()),
    arm_rad: finite(f.armRad()),
    camera: f.camera() ? pose(f.camera()) : chassis
  };
}

export class JitterBuffer {
  private pending = new Map < bigint, {
    frame: Snapshot;
    arrival: number
  }
  >();
  private last: bigint|null = null;
  stats = {dropped: 0, late: 0, invalid: 0};
  constructor(public delayMs = 10) {
    if (delayMs < 0 || delayMs > 1000) throw Error('Invalid jitter delay');
  }
  push(frame: Snapshot, now: number) {
    const seq = BigInt(frame.sequence_id);
    if (this.last !== null && seq <= this.last) {
      this.stats.late++;
      return;
    }
    if (this.pending.has(seq)) return;
    if (this.pending.size >= 4096) {
      this.stats.invalid++;
      return;
    }
    this.pending.set(seq, {frame, arrival: now});
  }
  flush(now: number): Snapshot[] {
    const out: Snapshot[] = [];
    // Release an entire sequence prefix only after each candidate has aged.
    // A mature later packet expires the wait for missing earlier packets.
    const entries = [...this.pending].sort(([a], [b]) => a < b ? -1 : 1);
    let end = -1;
    for (let i = 0; i < entries.length; i++)
      if (now - entries[i][1].arrival >= this.delayMs) end = i;
    for (let i = 0; i <= end; i++) {
      const [seq, {frame}] = entries[i];
      this.pending.delete(seq);
      if (this.last !== null)
        this.stats.dropped += Number(seq - this.last - 1n);
      this.last = seq;
      out.push(frame);
    }
    return out;
  }
}

export class Timeline {
  frames: Snapshot[] = [];
  constructor(public capacity = 30000) {
    if (!Number.isInteger(capacity) || capacity < 1)
      throw Error('Invalid capacity');
  }
  append(frame: Snapshot): boolean {
    const last = this.frames.at(-1);
    if (last &&
        (BigInt(frame.timestamp_ns) <= BigInt(last.timestamp_ns) ||
         BigInt(frame.sequence_id) <= BigInt(last.sequence_id)))
      return false;
    this.frames.push(frame);
    if (this.frames.length > this.capacity)
      this.frames.splice(0, this.frames.length - this.capacity);
    return true;
  }
  at(t: bigint): Snapshot|null {
    let lo = 0, hi = this.frames.length;
    while (lo < hi) {
      const mid = (lo + hi) >>> 1;
      if (BigInt(this.frames[mid].timestamp_ns) <= t)
        lo = mid + 1;
      else
        hi = mid;
    }
    return this.frames[lo - 1] ?? null;
  }
  exact(t: bigint): Snapshot|null {
    const f = this.at(t);
    return f && BigInt(f.timestamp_ns) === t ? f : null;
  }
  bounds() {
    return this.frames.length ? {
      start: this.frames[0].timestamp_ns,
      end: this.frames.at(-1)!.timestamp_ns
    } :
                                null;
  }
}

export class Coordinator {
  mode: Mode = 'LIVE_STREAMING';
  cursor = 0n;
  constructor(public timeline = new Timeline()) {}
  snapshot() {
    return this.timeline.at(this.cursor);
  }
  seek(t: bigint) {
    const b = this.timeline.bounds();
    if (!b) return;
    this.mode = 'PAUSED';
    this.cursor = t < BigInt(b.start) ? BigInt(b.start) :
        t > BigInt(b.end)             ? BigInt(b.end) :
                                        t;
  }
  step(dt: bigint) {
    this.seek(this.cursor + dt);
  }
  tick(dt: bigint) {
    const b = this.timeline.bounds();
    if (!b) return;
    if (this.mode === 'LIVE_STREAMING')
      this.cursor = BigInt(b.end);
    else if (this.mode === 'REPLAY_PLAYING') {
      this.cursor += dt;
      if (this.cursor >= BigInt(b.end)) {
        this.cursor = BigInt(b.end);
        this.mode = 'PAUSED';
      }
    }
  }
}
