import {ByteBuffer} from 'flatbuffers';

import {Frame} from './generated/talos/telemetry/frame';
import {Pose as WirePose} from './generated/talos/telemetry/pose';

export type Pose = {
  x: number; y: number; z: number; roll: number; pitch: number; yaw: number
};
export type Snapshot = {
  stream_id: string;
  sequence_id: string;
  timestamp_ns: string;
  dictionary_id: number;
  chassis: Pose;
  channels: Record<string, number>;
  units: Record<string, string>;
  labels: Record<string, string[]>
};
export type Mode = 'PAUSED'|'LIVE_STREAMING'|'REPLAY_PLAYING';
export const MAX_FRAME_BYTES = 16384;

// The display string for a channel value: the enum label when the channel
// declared labels and the value is a valid index, else the number itself.
export function channelDisplay(s: Snapshot, name: string): string {
  const value = s.channels[name];
  if (value === undefined) return '';
  const labels = s.labels[name];
  if (labels && labels.length && Number.isInteger(value) && value >= 0 &&
      value < labels.length)
    return labels[value];
  return String(value);
}

// JS generated accessors do not verify untrusted offsets. Verify this schema
// before constructing views; all offsets below are relative to the frame.
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
  if (String.fromCharCode(...bytes.subarray(8, 12)) !== 'TLM2')
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
  const stringVector = (p: number) => {
    if (!p) return;
    const q = ref(p), n = u32(q);
    if (n > 64) throw Error('Too many labels');
    check(q + 4, n * 4);
    for (let i = 0; i < n; i++) string(q + 4 + i * 4);
  };
  const f = table(ref(4), [8, 8, 8, 4, 48, 4, 4]);
  if (!f[4]) throw Error('Missing chassis');
  if (f[5]) {
    const q = ref(f[5]), n = u32(q);
    if (n > 4096) throw Error('Too many values');
    check(q + 4, n * 8);
  }
  if (f[6]) {
    const q = ref(f[6]), n = u32(q);
    if (n > 1024) throw Error('Too many channels');
    check(q + 4, n * 4);
    for (let i = 0; i < n; i++) {
      const entry = table(ref(q + 4 + i * 4), [4, 4, 4]);
      string(entry[0]);
      if (entry[1]) string(entry[1]);
      stringVector(entry[2]);
    }
  }
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

type Dictionary = {name: string; unit: string; labels: string[]}[];

// Stateful v2 decoder. The wire carries a channel dictionary once (keyed by
// dictionary_id) and bare value vectors after that, so a value frame whose
// dictionary has not arrived yet cannot be named: it is held, not dropped and
// not fatal, and resolves when the dictionary frame lands. Dictionaries are
// per-stream; a new stream_id invalidates every dictionary and held frame
// from the old one, since ids are only unique within a stream.
export class TelemetryDecoder {
  private dictionaries = new Map<number, Dictionary>();
  private held: Uint8Array[] = [];
  private stream: string|null = null;
  heldCount = 0;

  reset() {
    this.dictionaries.clear();
    this.held = [];
    this.stream = null;
    this.heldCount = 0;
  }

  // Snapshots for frames held earlier that this dictionary unblocked, each
  // with the raw bytes it arrived as (a log export needs them back).
  private drainHeld(): {frame: Snapshot; raw: Uint8Array}[] {
    if (!this.held.length) return [];
    const pending = this.held;
    this.held = [];
    const out: {frame: Snapshot; raw: Uint8Array}[] = [];
    for (const bytes of pending) {
      const snapshot = this.decode(bytes);
      if (snapshot) out.push({frame: snapshot, raw: bytes});
    }
    this.heldCount = this.held.length;
    return out;
  }

  // Every snapshot this byte frame produces: itself (once its dictionary is
  // known), plus any held frames the dictionary it carried unblocks.
  decodeAll(bytes: Uint8Array): Snapshot[] {
    return this.decodeFrames(bytes).map(entry => entry.frame);
  }

  decodeFrames(bytes: Uint8Array): {frame: Snapshot; raw: Uint8Array}[] {
    const snapshot = this.decode(bytes);
    if (!snapshot) return [];
    const out = [{frame: snapshot, raw: bytes}, ...this.drainHeld()];
    this.heldCount = this.held.length;
    return out;
  }

  decode(bytes: Uint8Array): Snapshot|null {
    verify(bytes);
    const f = Frame.getSizePrefixedRootAsFrame(new ByteBuffer(bytes));
    const streamId = f.streamId().toString();
    if (this.stream !== null && streamId !== this.stream) {
      this.dictionaries.clear();
      this.held = [];
    }
    this.stream = streamId;
    const dictionaryId = f.dictionaryId();
    let dictionary: Dictionary|undefined = this.dictionaries.get(dictionaryId);
    if (f.dictionaryLength() > 0) {
      const parsed: Dictionary = [];
      const seen = new Set<string>();
      for (let i = 0; i < f.dictionaryLength(); i++) {
        const info = f.dictionary(i)!;
        const name = info.name()!;
        if (seen.has(name)) throw Error('Duplicate channel');
        seen.add(name);
        const labels: string[] = [];
        for (let j = 0; j < info.labelsLength(); j++)
          labels.push(info.labels(j) as string);
        parsed.push({name, unit: info.unit() ?? '', labels});
      }
      dictionary = parsed;
      this.dictionaries.set(dictionaryId, parsed);
    }
    if (!dictionary) {
      if (this.held.length < 4096) this.held.push(bytes.slice());
      this.heldCount = this.held.length;
      return null;
    }
    if (f.valuesLength() > dictionary.length)
      throw Error('More values than dictionary channels');
    const channels: Record<string, number> = Object.create(null),
          units: Record<string, string> = Object.create(null),
          labels: Record<string, string[]> = Object.create(null);
    for (let i = 0; i < dictionary.length; i++) {
      const entry = dictionary[i];
      units[entry.name] = entry.unit;
      if (entry.labels.length) labels[entry.name] = entry.labels;
      if (i >= f.valuesLength()) continue;
      const value = f.values(i)!;
      // A NaN sample is a channel with no reading this tick, not a poisoned
      // frame: omit it so plots and agents see absence, not a fake zero.
      if (Number.isNaN(value)) continue;
      channels[entry.name] = finite(value);
    }
    return {
      stream_id: streamId,
      sequence_id: f.sequenceId().toString(),
      timestamp_ns: f.timestampNs().toString(),
      dictionary_id: dictionaryId,
      chassis: pose(f.chassis()),
      channels,
      units,
      labels
    };
  }
}

// Shared decoder for callers that decode an ordered stream (tests, log
// import). Stateful on purpose: bare value frames need the dictionary that
// preceded them.
const sharedDecoder = new TelemetryDecoder();
export function decode(bytes: Uint8Array): Snapshot|null {
  return sharedDecoder.decode(bytes);
}
export function resetSharedDecoder() {
  sharedDecoder.reset();
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
  lastArrivalMs = 0;
  constructor(public capacity = 30000) {
    if (!Number.isInteger(capacity) || capacity < 1)
      throw Error('Invalid capacity');
  }
  clear() {
    this.frames = [];
    this.lastArrivalMs = 0;
  }
  latest(): Snapshot|null {
    return this.frames.at(-1) ?? null;
  }
  append(frame: Snapshot, nowMs = Date.now()): boolean {
    const last = this.frames.at(-1);
    if (last && last.stream_id !== frame.stream_id) return false;
    if (last &&
        (BigInt(frame.timestamp_ns) <= BigInt(last.timestamp_ns) ||
         BigInt(frame.sequence_id) <= BigInt(last.sequence_id)))
      return false;
    this.frames.push(frame);
    this.lastArrivalMs = nowMs;
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
