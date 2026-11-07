// The main thread's columnar render cache. The worker keeps decoding, the
// jitter buffer, the timeline and the cursor; this store is fed the frames it
// has already ordered, so panels can plot the whole retained history without
// the worker re-posting it.
//
// Memory: one Float64Array per channel plus one for times and six for the
// chassis pose, each `STORE_CAPACITY` wide -- `(channels + 7) × capacity × 8 B`
// for the numeric columns, plus a handful of per-frame strings (timestamp,
// stream, sequence) kept only for exact lookup and snapshot reconstruction.
// At the default capacity of 30,000 that is 240 KB per channel.

import type {Snapshot} from '../core';
import type {SystemGraph} from '../system';
import type {SessionManifest} from './manifest';
import {driverStationMode} from './robot';
import type {
  ChannelInfo,
  DriverStationMode,
  ModeBand,
  RobotEvent,
  Series,
  SeriesWindow
} from './types';
import {EventEngine} from './events';

// The worker's Timeline capacity (core.ts Timeline defaults to 30000); the
// cache never holds more frames than the timeline it mirrors.
export const STORE_CAPACITY = 30000;

const defaultScheduler = (cb: () => void): void => {
  if (typeof requestAnimationFrame === 'function') requestAnimationFrame(() => cb());
  else setTimeout(cb, 0);
};

export class TelemetryStore {
  // Numeric ring columns. `head` indexes the oldest frame; a frame's logical
  // index i in [0, count) lives at physical index (head + i) % capacity.
  private readonly capacity: number;
  private readonly times: Float64Array;
  private readonly timestamps: (string|null)[];
  private readonly streamIds: (string|null)[];
  private readonly sequenceIds: (string|null)[];
  private readonly dictionaryIds: Int32Array;
  private readonly chassisX: Float64Array;
  private readonly chassisY: Float64Array;
  private readonly chassisZ: Float64Array;
  private readonly chassisRoll: Float64Array;
  private readonly chassisPitch: Float64Array;
  private readonly chassisYaw: Float64Array;
  private readonly columns = new Map<string, Float64Array>();
  private readonly infos = new Map<string, ChannelInfo>();
  private readonly dictionaries = new Map<number, {
    infos: Map<string, ChannelInfo>; frames: number;
  }>();
  private readonly channelReferences = new Map<string, number>();
  private dictionaryCache: ChannelInfo[]|null = null;
  private readonly memo: (Snapshot|null)[];

  private head = 0;
  private count = 0;
  private origin: bigint|null = null;

  private readonly eventEngine: EventEngine;
  private bands: ModeBand[] = [];
  private bandMode: DriverStationMode|null = null;

  private cursor: string|null = null;

  version = 0;

  private readonly listeners = new Set<() => void>();
  private notifyScheduled = false;
  private readonly scheduler: (cb: () => void) => void;

  constructor(scheduler: (cb: () => void) => void = defaultScheduler,
              capacity: number = STORE_CAPACITY) {
    if (!Number.isInteger(capacity) || capacity < 1)
      throw Error('Invalid capacity');
    this.capacity = capacity;
    this.times = new Float64Array(capacity);
    this.timestamps = new Array<string|null>(capacity).fill(null);
    this.streamIds = new Array<string|null>(capacity).fill(null);
    this.sequenceIds = new Array<string|null>(capacity).fill(null);
    this.dictionaryIds = new Int32Array(capacity).fill(-1);
    this.chassisX = new Float64Array(capacity);
    this.chassisY = new Float64Array(capacity);
    this.chassisZ = new Float64Array(capacity);
    this.chassisRoll = new Float64Array(capacity);
    this.chassisPitch = new Float64Array(capacity);
    this.chassisYaw = new Float64Array(capacity);
    this.memo = new Array<Snapshot|null>(capacity).fill(null);
    this.scheduler = scheduler;
    this.eventEngine = new EventEngine();
  }

  subscribe(listener: () => void): () => void {
    this.listeners.add(listener);
    return () => {
      this.listeners.delete(listener);
    };
  }

  private scheduleNotify() {
    if (this.notifyScheduled || this.listeners.size === 0) return;
    this.notifyScheduled = true;
    this.scheduler(() => {
      this.notifyScheduled = false;
      for (const listener of this.listeners) listener();
    });
  }

  ingest(frames: Snapshot[], reset: boolean) {
    if (reset) this.clear();
    for (const frame of frames) this.append(frame);
    this.version++;
    this.scheduleNotify();
  }

  setCursor(ns: string|null) {
    if (this.cursor === ns) return;
    this.cursor = ns;
    this.scheduleNotify();
  }

  pushSystem(graph: SystemGraph|null) {
    if (graph) this.eventEngine.pushSystem(graph);
    else this.eventEngine.resetSystem();
    this.scheduleNotify();
  }

  pushManifest(manifest: SessionManifest) {
    this.eventEngine.pushManifest(manifest);
    this.scheduleNotify();
  }

  // --- frame access ---------------------------------------------------------

  snapshotAt(ns: string|null): Snapshot|null {
    if (ns === null || this.count === 0) return null;
    const index = this.indexOf(BigInt(ns));
    if (index < 0) return null;
    const slot = this.phys(index);
    const cached = this.memo[slot];
    if (cached) return cached;
    const snapshot = this.reconstruct(slot);
    this.memo[slot] = snapshot;
    return snapshot;
  }

  snapshotAtCursor(): Snapshot|null {
    return this.snapshotAt(this.cursor);
  }

  channelValue(name: string): number|undefined {
    return this.snapshotAtCursor()?.channels[name];
  }

  channelInfo(name: string): ChannelInfo|undefined {
    return this.infos.get(name);
  }

  dictionary(): ChannelInfo[] {
    if (this.dictionaryCache === null)
      this.dictionaryCache = [...this.infos.values()].sort(
          (a, b) => a.name < b.name ? -1 : a.name > b.name ? 1 : 0);
    return this.dictionaryCache;
  }

  series(channels: string[], window: SeriesWindow,
         cursor_ns: string|null = this.cursor): Series {
    const empty: Series = {
      t0_ns: this.origin?.toString() ?? '0',
      times: new Float64Array(0),
      values: {}
    };
    if (this.count === 0 || cursor_ns === null) return empty;
    const end = this.indexOf(BigInt(cursor_ns));
    if (end < 0) return empty;

    let start = 0;
    if (window !== 'all') {
      const threshold = this.times[this.phys(end)] - window.seconds;
      let lo = 0, hi = end + 1;
      while (lo < hi) {
        const mid = (lo + hi) >>> 1;
        if (this.times[this.phys(mid)] < threshold) lo = mid + 1;
        else hi = mid;
      }
      start = Math.min(lo, end);
    }

    const values: Record<string, Float64Array> = {};
    for (const name of channels) {
      const column = this.columns.get(name);
      values[name] = column ?
          this.sliceColumn(column, start, end) :
          new Float64Array(end - start + 1).fill(NaN);
    }
    return {
      t0_ns: this.origin!.toString(),
      times: this.sliceColumn(this.times, start, end),
      values
    };
  }

  events(): RobotEvent[] {
    return this.eventEngine.events;
  }

  setEvents(events: RobotEvent[]) {
    this.eventEngine.setEvents(events);
    this.scheduleNotify();
  }

  modeBands(): ModeBand[] {
    return this.bands;
  }

  // --- internals ------------------------------------------------------------

  private phys(index: number): number {
    return (this.head + index) % this.capacity;
  }

  // at_or_before: the newest frame whose timestamp is <= t, matching Timeline.at.
  private indexOf(t: bigint): number {
    let lo = 0, hi = this.count;
    while (lo < hi) {
      const mid = (lo + hi) >>> 1;
      if (BigInt(this.timestamps[this.phys(mid)]!) <= t) lo = mid + 1;
      else hi = mid;
    }
    return lo - 1;
  }

  private sliceColumn(column: Float64Array, start: number, end: number): Float64Array {
    const ps = this.phys(start), pe = this.phys(end);
    if (ps <= pe) return column.subarray(ps, pe + 1);
    const out = new Float64Array(end - start + 1);
    let k = 0;
    for (let i = ps; i < this.capacity; i++) out[k++] = column[i];
    for (let i = 0; i <= pe; i++) out[k++] = column[i];
    return out;
  }

  private reconstruct(slot: number): Snapshot {
    const channels: Record<string, number> = Object.create(null);
    for (const [name, column] of this.columns) {
      const value = column[slot];
      if (!Number.isNaN(value)) channels[name] = value;
    }
    const units: Record<string, string> = Object.create(null);
    const labels: Record<string, string[]> = Object.create(null);
    for (const [name, info] of this.dictionaries.get(this.dictionaryIds[slot])!.infos) {
      units[name] = info.unit;
      if (info.labels.length) labels[name] = info.labels;
    }
    return {
      stream_id: this.streamIds[slot]!,
      sequence_id: this.sequenceIds[slot]!,
      timestamp_ns: this.timestamps[slot]!,
      dictionary_id: this.dictionaryIds[slot],
      chassis: {
        x: this.chassisX[slot], y: this.chassisY[slot], z: this.chassisZ[slot],
        roll: this.chassisRoll[slot], pitch: this.chassisPitch[slot],
        yaw: this.chassisYaw[slot]
      },
      channels,
      units,
      labels
    };
  }

  private append(frame: Snapshot) {
    const ts = BigInt(frame.timestamp_ns);
    // The worker already rejects out-of-order frames; guard here too so the
    // cache can be fed an unsanitised list (a static studio) without breaking.
    if (this.count > 0 &&
        ts <= BigInt(this.timestamps[this.phys(this.count - 1)]!))
      return;
    if (this.origin === null) this.origin = ts;

    const slot = (this.head + this.count) % this.capacity;
    const evictedDictionary = this.count === this.capacity ? this.dictionaryIds[slot] : null;
    let dictionary = this.dictionaries.get(frame.dictionary_id);
    if (!dictionary) {
      dictionary = {infos: new Map(), frames: 0};
      this.dictionaries.set(frame.dictionary_id, dictionary);
    }
    dictionary.frames++;
    // Units declare channels even when their current reading is absent.
    for (const name of new Set([...Object.keys(frame.units), ...Object.keys(frame.channels)])) {
      if (dictionary.infos.has(name)) continue;
      const info = {name, unit: frame.units[name] ?? '', labels: frame.labels[name] ?? []};
      dictionary.infos.set(name, info);
      this.channelReferences.set(name, (this.channelReferences.get(name) ?? 0) + 1);
      this.infos.set(name, info);
      this.dictionaryCache = null;
    }
    if (evictedDictionary !== null) this.releaseDictionary(evictedDictionary);
    this.timestamps[slot] = frame.timestamp_ns;
    this.times[slot] = Number(ts - this.origin) / 1e9;
    this.streamIds[slot] = frame.stream_id;
    this.sequenceIds[slot] = frame.sequence_id;
    this.dictionaryIds[slot] = frame.dictionary_id;
    this.chassisX[slot] = frame.chassis.x;
    this.chassisY[slot] = frame.chassis.y;
    this.chassisZ[slot] = frame.chassis.z;
    this.chassisRoll[slot] = frame.chassis.roll;
    this.chassisPitch[slot] = frame.chassis.pitch;
    this.chassisYaw[slot] = frame.chassis.yaw;
    this.memo[slot] = null;

    // A reused ring slot must not keep the overwritten frame's value for a
    // channel this frame does not carry (absent, stale, or NaN-omitted).
    for (const column of this.columns.values()) column[slot] = NaN;
    for (const [name, value] of Object.entries(frame.channels)) {
      let column = this.columns.get(name);
      if (!column) {
        column = new Float64Array(this.capacity).fill(NaN);
        this.columns.set(name, column);
      }
      column[slot] = value;
    }

    if (this.count < this.capacity) {
      this.count++;
    } else {
      this.head = (this.head + 1) % this.capacity;
      this.trimHistory();
    }

    this.eventEngine.pushFrame(frame);
    this.updateBands(frame);
  }

  private releaseDictionary(id: number) {
    const dictionary = this.dictionaries.get(id)!;
    if (--dictionary.frames > 0) return;
    this.dictionaries.delete(id);
    for (const name of dictionary.infos.keys()) {
      const references = this.channelReferences.get(name)! - 1;
      if (references > 0) {
        this.channelReferences.set(name, references);
      } else {
        this.channelReferences.delete(name);
        this.columns.delete(name);
        this.infos.delete(name);
        this.dictionaryCache = null;
      }
    }
  }

  private clear() {
    this.head = 0;
    this.count = 0;
    this.origin = null;
    this.columns.clear();
    this.infos.clear();
    this.dictionaries.clear();
    this.channelReferences.clear();
    this.dictionaryCache = null;
    this.memo.fill(null);
    this.bands = [];
    this.bandMode = null;
    this.eventEngine.clear();
  }

  private trimHistory() {
    const oldest = this.count > 0 ? this.timestamps[this.head] : null;
    this.eventEngine.trim(oldest);
    if (oldest === null) {
      this.bands = [];
      return;
    }
    const bound = BigInt(oldest);
    let i = 0;
    while (i < this.bands.length && BigInt(this.bands[i].end_ns) < bound) i++;
    if (i > 0) this.bands = this.bands.slice(i);
    if (this.bands.length && BigInt(this.bands[0].start_ns) < bound)
      this.bands = [{...this.bands[0], start_ns: oldest}, ...this.bands.slice(1)];
  }

  private updateBands(frame: Snapshot) {
    const mode = driverStationMode(frame);
    const ts = frame.timestamp_ns;
    if (mode === this.bandMode) {
      if (mode !== null && this.bands.length) {
        const last = this.bands[this.bands.length - 1];
        this.bands = [...this.bands.slice(0, -1), {...last, end_ns: ts}];
      }
    } else {
      if (mode !== null)
        this.bands = [...this.bands, {start_ns: ts, end_ns: ts, mode}];
      this.bandMode = mode;
    }
  }
}
