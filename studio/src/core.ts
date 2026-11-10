import {ByteBuffer} from 'flatbuffers';

import {type ChannelSpec, doubleFromRaw, encodeDictionaryPages, encodeValuePages, rawFromDouble} from './encode';
import {ChannelKind} from './generated/talos/telemetry/channel-kind';
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
  // Every present channel as a number (an integer past 2^53 rounded, for
  // plotting; a NaN or infinite reading as itself, which plots as a gap);
  // `exact` holds every integer channel's exact decimal value. An absent
  // channel (its topic was not published) has no entry.
  channels: Record<string, number>;
  exact?: Record<string, string>;
  units: Record<string, string>;
  labels: Record<string, string[]>;
  kinds?: Readonly<Record<string, ChannelKind>>
};

export type PlanarPose = Pick<Pose, 'x'|'y'|'yaw'>;

// Mechanism-only frames have a required zero chassis and pose/valid = 0.
// Older recordings without this flag always carried a real pose.
export function poseValid(s: Snapshot|null|undefined): boolean {
  if (!s) return false;
  const v: number|undefined = s.channels['pose/valid'];
  return v === undefined || v === 1;
}

// Telemetry removes the topic's leading slash and joins nested struct fields
// with slashes (talOS/telemetry/node.h, ObserveDescribedTopic/FlattenStruct).
// Missing or stale samples are absent from Snapshot.channels, not zero poses.
export function simTruthPose(s: Snapshot|null|undefined): PlanarPose|null {
  if (!s) return null;
  const x = s.channels['sim/state/pose/x_m'],
        y = s.channels['sim/state/pose/y_m'],
        yaw = s.channels['sim/state/pose/yaw_rad'];
  if (![x, y, yaw].every(Number.isFinite)) return null;
  return {x, y, yaw};
}

export function poseError(estimate: PlanarPose, truth: PlanarPose):
    {position_m: number; heading_deg: number} {
  const yaw = estimate.yaw - truth.yaw;
  return {
    position_m: Math.hypot(estimate.x - truth.x, estimate.y - truth.y),
    heading_deg: Math.atan2(Math.sin(yaw), Math.cos(yaw)) * 180 / Math.PI
  };
}
export type Mode = 'PAUSED'|'LIVE_STREAMING'|'REPLAY_PLAYING';
export const MAX_FRAME_BYTES = 16384;
// The dictionary limits of talOS/telemetry/wire.h (docs/DATA_FLOW.md): the
// producer never emits past them and Studio decodes up to exactly them, so no
// dictionary the producer emits blanks the stream.
export const MAX_DICTIONARY_STRING_BYTES = 256;
export const MAX_ENUM_LABELS = 1024;

// The display string for a channel value: the enum label when the channel
// declared labels and the value is a valid index, else the exact value.
export function channelDisplay(s: Snapshot, name: string): string {
  const value = s.channels[name];
  if (value === undefined) return '';
  const labels = s.labels[name];
  if (labels && labels.length && Number.isInteger(value) && value >= 0 &&
      value < labels.length)
    return labels[value];
  return s.exact?.[name] ?? String(value);
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
  if (String.fromCharCode(...bytes.subarray(8, 12)) !== TELEMETRY_FORMAT)
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
  // The producer writes every dictionary string once, so their text fits in
  // the frame; offsets shared to multiply it past that (a decode bomb) are
  // refused. Value and entry counts need no cap: each one takes frame bytes.
  let text = bytes.length;
  const string = (p: number) => {
    if (!p) throw Error('Missing name');
    const q = ref(p), n = u32(q);
    if (n > MAX_DICTIONARY_STRING_BYTES) throw Error('Name too long');
    check(q + 4, n + 1);
    if (v.getUint8(q + 4 + n) !== 0) throw Error('Invalid string');
    text -= n + 1;
    if (text < 0) throw Error('Dictionary text exceeds its frame');
  };
  const stringVector = (p: number) => {
    if (!p) return;
    const q = ref(p), n = u32(q);
    if (n > MAX_ENUM_LABELS) throw Error('Too many labels');
    check(q + 4, n * 4);
    for (let i = 0; i < n; i++) string(q + 4 + i * 4);
  };
  const vector = (p: number, width: number) => {
    if (!p) return;
    const q = ref(p);
    check(q + 4, u32(q) * width);
  };
  // stream, sequence, timestamp, dictionary id, chassis, page, page count,
  // channel count, keyframe, channels, values, absent, first channel,
  // dictionary.
  const f = table(ref(4), [8, 8, 8, 4, 48, 2, 2, 4, 1, 4, 4, 4, 4, 4]);
  vector(f[9], 4);
  vector(f[10], 8);
  vector(f[11], 4);
  if (f[13]) {
    const q = ref(f[13]), n = u32(q);
    check(q + 4, n * 4);
    for (let i = 0; i < n; i++) {
      const entry = table(ref(q + 4 + i * 4), [4, 4, 4, 1]);
      string(entry[0]);
      if (entry[1]) string(entry[1]);
      stringVector(entry[2]);
    }
  } else if (!f[4]) {
    throw Error('Missing chassis');
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

// A decoded dictionary: channel names and kinds in channel order, and units,
// labels and kinds frozen and shared by every snapshot decoded against it, so
// a long timeline holds one copy per dictionary rather than one per sample.
// `derivedUnits` adds the units of Studio's derived/ channels, for the
// snapshots that carry them.
export type Dictionary = {
  id: number;
  names: string[];
  kindList: ChannelKind[];
  units: Readonly<Record<string, string>>;
  labels: Readonly<Record<string, string[]>>;
  kinds: Readonly<Record<string, ChannelKind>>;
  derivedUnits?: Readonly<Record<string, string>>;
};

// Splits a buffer of one or more size-prefixed pages (a WebSocket message
// holds one; the demo, tests and log files hold several back to back).
// The telemetry format this Studio reads, live and in exports. An export is
// readable only by a Studio of the same format version; a match recorded
// before a format change stays readable through the nodes' event logs
// (log_dump), which are the complete record.
export const TELEMETRY_FORMAT = 'TLM3';

// Refuses an export of another telemetry format by name, before decoding
// fails on it with a less useful error.
export function checkExportFormat(bytes: Uint8Array) {
  if (bytes.length < 12) return;
  const found = String.fromCharCode(...bytes.subarray(8, 12));
  if (found !== TELEMETRY_FORMAT && /^TLM\d$/.test(found))
    throw Error(`telemetry export format ${found} is no longer supported; ` +
                `this Studio reads ${TELEMETRY_FORMAT}`);
}

export function splitPages(bytes: Uint8Array): Uint8Array[] {
  if (!bytes.length) throw Error('Empty message');
  const pages: Uint8Array[] = [];
  for (let p = 0; p < bytes.length;) {
    if (p + 4 > bytes.length) throw Error('Truncated page');
    const size = new DataView(bytes.buffer, bytes.byteOffset + p, 4)
                     .getUint32(0, true) + 4;
    if (size < 16 || p + size > bytes.length) throw Error('Truncated page');
    pages.push(bytes.subarray(p, p + size));
    p += size;
  }
  return pages;
}

// Stateful v3 decoder (talOS/telemetry/README.md). Dictionary pages complete a
// dictionary; value pages apply to the current channel values as they
// arrive, a keyframe's first page starting from every channel absent, and an
// update's last page yields its snapshot. The bridge sends the dictionary and
// a keyframe before any change-only update, so a value page with no
// dictionary or no keyframe before it is refused, and the caller counts it.
// Dictionaries are per stream: a new stream_id discards the old one.
export class TelemetryDecoder {
  private stream: string|null = null;
  private dictionary: Dictionary|null = null;
  private pending: {stream: string; id: number; pageCount: number;
                    count: number; pages: Frame[]}|null = null;
  private raw = new BigUint64Array(0);
  private present = new Uint8Array(0);
  private hasValues = false;

  reset() {
    this.stream = null;
    this.dictionary = null;
    this.pending = null;
    this.hasValues = false;
  }

  // The dictionary snapshots currently decode against.
  currentDictionary(): Dictionary|null {
    return this.dictionary;
  }

  // Every snapshot the pages in `bytes` complete, in order.
  decodeAll(bytes: Uint8Array): Snapshot[] {
    const out: Snapshot[] = [];
    for (const page of splitPages(bytes)) {
      const snapshot = this.decodePage(page);
      if (snapshot) out.push(snapshot);
    }
    return out;
  }

  // The last snapshot the pages in `bytes` complete, or null.
  decode(bytes: Uint8Array): Snapshot|null {
    return this.decodeAll(bytes).at(-1) ?? null;
  }

  private decodePage(bytes: Uint8Array): Snapshot|null {
    verify(bytes);
    const f = Frame.getSizePrefixedRootAsFrame(new ByteBuffer(bytes));
    const streamId = f.streamId().toString();
    const page = f.page(), pageCount = f.pageCount();
    if (pageCount === 0 || page >= pageCount) throw Error('Invalid page');
    if (f.dictionaryLength() > 0 || f.bb!.__offset(f.bb_pos, 30) !== 0) {
      this.dictionaryPage(f, streamId, page, pageCount);
      return null;
    }
    const dictionary = this.dictionary;
    if (!dictionary || streamId !== this.stream ||
        f.dictionaryId() !== dictionary.id ||
        f.channelCount() !== dictionary.names.length)
      throw Error('Value page before its dictionary');
    const keyframe = f.keyframe();
    if (!keyframe && !this.hasValues) throw Error('Update before a keyframe');
    // Validate the whole page before it changes anything.
    const chassis = pose(f.chassis());
    const count = f.channelsLength();
    if (f.valuesLength() !== count) throw Error('Channels and values differ');
    const raws: bigint[] = [];
    for (let i = 0; i < count; i++) {
      const channel = f.channels(i)!, raw = f.values(i)!;
      if (channel >= dictionary.names.length) throw Error('Unknown channel');
      // Any double is a reading, NaN and the infinities included.
      if (dictionary.kindList[channel] === ChannelKind.Bool && raw > 1n)
        throw Error('Invalid bool');
      raws.push(raw);
    }
    for (let i = 0; i < f.absentLength(); i++)
      if (f.absent(i)! >= dictionary.names.length) throw Error('Unknown channel');

    if (keyframe && page === 0) {
      this.present.fill(0);
      this.hasValues = true;
    }
    for (let i = 0; i < count; i++) {
      this.raw[f.channels(i)!] = raws[i];
      this.present[f.channels(i)!] = 1;
    }
    for (let i = 0; i < f.absentLength(); i++) this.present[f.absent(i)!] = 0;
    if (page + 1 !== pageCount) return null;
    return this.snapshot(f, streamId, chassis, dictionary);
  }

  private dictionaryPage(f: Frame, streamId: string, page: number,
                         pageCount: number) {
    if (f.channelsLength() || f.valuesLength())
      throw Error('Dictionary page with values');
    const pending = this.pending;
    if (page === 0) {
      this.pending = {stream: streamId, id: f.dictionaryId(), pageCount,
                      count: f.channelCount(), pages: [f]};
    } else if (pending && pending.stream === streamId &&
               pending.id === f.dictionaryId() &&
               pending.pageCount === pageCount &&
               pending.pages.length === page) {
      pending.pages.push(f);
    } else {
      this.pending = null;
      throw Error('Dictionary page out of order');
    }
    const complete = this.pending!;
    if (complete.pages.length < complete.pageCount) return;
    this.pending = null;
    const names: string[] = [], kindList: ChannelKind[] = [];
    const units: Record<string, string> = Object.create(null),
          labels: Record<string, string[]> = Object.create(null),
          kinds: Record<string, ChannelKind> = Object.create(null);
    const seen = new Set<string>();
    for (const part of complete.pages) {
      if (part.firstChannel() !== names.length)
        throw Error('Dictionary pages do not join');
      for (let i = 0; i < part.dictionaryLength(); i++) {
        const info = part.dictionary(i)!;
        const name = info.name()!;
        if (seen.has(name)) throw Error('Duplicate channel');
        // Studio owns this namespace; never silently overwrite a wire channel.
        if (name.startsWith('derived/'))
          throw Error('Reserved derived/ channel namespace');
        const kind = info.kind();
        if (!(kind in ChannelKind)) throw Error('Unknown channel kind');
        seen.add(name);
        names.push(name);
        kindList.push(kind);
        kinds[name] = kind;
        units[name] = info.unit() ?? '';
        if (info.labelsLength()) {
          const channelLabels: string[] = [];
          for (let j = 0; j < info.labelsLength(); j++)
            channelLabels.push(info.labels(j) as string);
          Object.freeze(channelLabels);
          labels[name] = channelLabels;
        }
      }
    }
    if (names.length !== complete.count) throw Error('Dictionary is incomplete');
    const same = this.dictionary && this.stream === streamId &&
        this.dictionary.id === complete.id;
    this.stream = streamId;
    this.dictionary = {
      id: complete.id, names, kindList, units: Object.freeze(units),
      labels: Object.freeze(labels), kinds: Object.freeze(kinds)
    };
    if (!same) {
      this.raw = new BigUint64Array(names.length);
      this.present = new Uint8Array(names.length);
      this.hasValues = false;
    }
  }

  private snapshot(f: Frame, streamId: string, chassis: Pose,
                   dictionary: Dictionary): Snapshot {
    const channels: Record<string, number> = Object.create(null);
    const exact: Record<string, string> = Object.create(null);
    for (let i = 0; i < dictionary.names.length; i++) {
      // An absent channel has no reading this tick: omitted, so plots and
      // agents see absence, not a fake zero.
      if (!this.present[i]) continue;
      const name = dictionary.names[i], raw = this.raw[i];
      switch (dictionary.kindList[i]) {
        case ChannelKind.Double:
          channels[name] = doubleFromRaw(raw);
          break;
        case ChannelKind.Int: {
          const value = BigInt.asIntN(64, raw);
          channels[name] = Number(value);
          exact[name] = value.toString();
          break;
        }
        case ChannelKind.UInt:
          channels[name] = Number(raw);
          exact[name] = raw.toString();
          break;
        default:
          channels[name] = Number(raw);
      }
    }
    const snapshot: Snapshot = {
      stream_id: streamId,
      sequence_id: f.sequenceId().toString(),
      timestamp_ns: f.timestampNs().toString(),
      dictionary_id: dictionary.id,
      chassis,
      channels,
      exact,
      units: dictionary.units,
      labels: dictionary.labels,
      kinds: dictionary.kinds
    };
    // Derive once, before history/UI/RPC consume the snapshot. Missing inputs
    // leave no reading (or unit) rather than inventing a zero-error sample.
    const truth = simTruthPose(snapshot);
    if (truth && poseValid(snapshot)) {
      const error = poseError(snapshot.chassis, truth);
      channels['derived/pose_error_m'] = error.position_m;
      channels['derived/heading_error_deg'] = error.heading_deg;
      snapshot.units = dictionary.derivedUnits ??= Object.freeze(Object.assign(
          Object.create(null), dictionary.units,
          {'derived/pose_error_m': 'm', 'derived/heading_error_deg': 'deg'}));
    }
    return snapshot;
  }
}


// The dictionary a snapshot was decoded against. The wire contract caches a
// dictionary by stream_id and dictionary_id (talOS/telemetry/README.md), so
// the pair names it.
export function dictionaryKey(frame: Pick<Snapshot, 'stream_id'|'dictionary_id'>):
    string {
  return `${frame.stream_id}:${frame.dictionary_id}`;
}

// A timeline as a TLM3 log: each snapshot as a keyframe, exact integers
// included, preceded by its dictionary's pages wherever the dictionary
// differs from the previous snapshot's (a timeline that spans a producer
// restart holds one per stream). `dictionaries` is keyed by `dictionaryKey`.
// Derived channels are Studio's own and are derived again on import.
export function exportPages(frames: Snapshot[],
                            dictionaries: ReadonlyMap<string, Dictionary>):
    Uint8Array[] {
  const pages: Uint8Array[] = [];
  let current: Dictionary|null = null, currentKey = '';
  for (const f of frames) {
    const header = (dictionary: Dictionary) => ({
      streamId: BigInt(f.stream_id),
      sequenceId: BigInt(f.sequence_id),
      timestampNs: BigInt(f.timestamp_ns),
      dictionaryId: dictionary.id,
      channelCount: dictionary.names.length
    });
    const key = dictionaryKey(f);
    if (current === null || key !== currentKey) {
      const next = dictionaries.get(key);
      if (!next) throw Error(`No dictionary for stream ${f.stream_id}`);
      current = next;
      currentKey = key;
      const channels: ChannelSpec[] = current.names.map((name, i) => ({
        name,
        unit: next.units[name],
        labels: next.labels[name],
        kind: next.kindList[i]
      }));
      pages.push(...encodeDictionaryPages(header(current), channels));
    }
    const dictionary = current;
    const samples: [number, bigint][] = [];
    dictionary.names.forEach((name, i) => {
      const value = f.channels[name];
      if (value === undefined) return;
      const exact = f.exact?.[name];
      switch (dictionary.kindList[i]) {
        case ChannelKind.Double:
          samples.push([i, rawFromDouble(value)]);
          break;
        case ChannelKind.Int:
          samples.push([i, BigInt.asUintN(64, BigInt(exact ?? value))]);
          break;
        default:
          samples.push([i, BigInt(exact ?? value)]);
      }
    });
    pages.push(...encodeValuePages(header(dictionary), f.chassis, true, samples));
  }
  return pages;
}

export class JitterBuffer {
  private pending = new Map < bigint, {
    frame: Snapshot;
    arrival: number
  }
  >();
  private last: bigint|null = null;
  stats = {skipped: 0, late: 0, invalid: 0};
  constructor(public delayMs = 10) {
    if (delayMs < 0 || delayMs > 1000) throw Error('Invalid jitter delay');
  }
  push(frame: Snapshot, now: number) {
    const seq = BigInt(frame.sequence_id);
    // The bridge's heartbeat repeats the tick already held: not late.
    if (seq === this.last) return;
    if (this.last !== null && seq < this.last) {
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
  // The frames waiting to be released.
  held(): Snapshot[] {
    return [...this.pending.values()].map(entry => entry.frame);
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
        this.stats.skipped += Number(seq - this.last - 1n);
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
    // A new stream (a restarted producer) continues the timeline only if its
    // clock does; its sequence restarts, so sequence order holds per stream.
    if (last &&
        (BigInt(frame.timestamp_ns) <= BigInt(last.timestamp_ns) ||
         (frame.stream_id === last.stream_id &&
          BigInt(frame.sequence_id) <= BigInt(last.sequence_id))))
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
    if (this.timeline.frames.length > 0) {
      const first = BigInt(this.timeline.frames[0].timestamp_ns);
      const last = BigInt(this.timeline.latest()!.timestamp_ns);
      if (this.cursor < first) this.cursor = first;
      if (this.mode === 'REPLAY_PLAYING' && this.cursor > last)
        this.cursor = last;
    }
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
    const first = BigInt(b.start);
    const last = BigInt(b.end);
    if (this.cursor < first) this.cursor = first;
    if (this.mode === 'REPLAY_PLAYING' && this.cursor > last)
      this.cursor = last;
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
