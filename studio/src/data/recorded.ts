// A recorded session read from the review server (review.ts), as a DataSource.
//
// The playhead is an event time inside the session's bounds. Value panels read
// the snapshot at the playhead: each channel's newest value at or before it,
// with its own sample time, and its age is measured against the snapshot's
// own t_ns (playback.clock_ns), as live measures it against its frame's time:
// a snapshot that lags a playing playhead does not make its readings stale.
// Plots read decimated series: min, max and last per bucket, the plot's pixel
// width rounded up to one of POINT_SIZES buckets.
//
// Requests: a getter that misses starts a fetch and returns what it has (the
// last result for that consumer, else nothing); the arrival bumps `version`
// and notifies. Identical requests share one fetch, completed ones are kept in
// a small LRU, and a consumer's newer request aborts its superseded one. A
// failed request, HTTP or payload, sets `error` and pauses playback; nothing is
// thrown into a panel, and the failed request is not issued again until
// `retry()` (a different one, after the cursor or the plot moved, is). A plot's
// series is split into requests within the server's limits (bounds.json) and
// merged.

import type {Pose, Snapshot} from '../core';
import type {AlertLevel} from '../generated/talos/telemetry/alert-level';
import {discoverEstimate, type EstimateChannels} from './discovery';
import {
  decodeSeries,
  errorMessage,
  getBytes,
  getJson,
  isAbort,
  parseBounds,
  parseChannels,
  parseSnapshot,
  reviewUrl,
  type FetchLike,
  type ReviewBounds,
  type ReviewSnapshot,
  type SeriesBuckets,
  type SeriesColumns
} from './review';
import type {ChannelSample, DataSource} from './source';
import type {
  ChannelInfo,
  Mode,
  ModeBand,
  PlaybackState,
  RobotEvent,
  Series,
  SeriesWindow
} from './types';

// Milliseconds, and a periodic callback (an animation frame in the browser).
// Tests inject both.
export type ReviewClock = {now(): number; every(tick: () => void): () => void};

export const browserClock: ReviewClock = {
  now: () => performance.now(),
  every(tick) {
    if (typeof requestAnimationFrame === 'function') {
      let id = 0;
      const loop = () => {
        tick();
        id = requestAnimationFrame(loop);
      };
      id = requestAnimationFrame(loop);
      return () => cancelAnimationFrame(id);
    }
    const id = setInterval(tick, 16);
    return () => clearInterval(id);
  }
};

export const PLAY_RATES = [1, 2, 4] as const;
export type PlayRate = typeof PLAY_RATES[number];

export const SERIES_CACHE_ENTRIES = 32;
// Consumers whose last result is kept, least recently used dropped first,
// beyond those the latest render showed, which are never dropped: so the
// kept results are at most max(SERIES_SLOTS, the requests on screen) plus
// SERIES_CACHE_ENTRIES, each within the server's max_series_values.
export const SERIES_SLOTS = 64;
// Buckets for a consumer that does not say how wide it is.
export const DEFAULT_POINTS = 400;
// The bucket counts a series is requested at: a width rounds up to the next
// one (the last for anything wider), so resizing a plot changes its request
// only where it crosses one.
export const POINT_SIZES = [256, 512, 1024, 2048, 4096] as const;
export const quantizePoints = (width: number) =>
    POINT_SIZES.find(size => size >= width) ?? POINT_SIZES[POINT_SIZES.length - 1];
// A window's fetched range ends on a grid of 1/16 of the window, so a playing
// cursor refetches once per 1/16 window rather than every frame.
const GRID_DIVISIONS = 16n;
const ALL_GRID_DIVISIONS = 64n;
const MIN_GRID_NS = 1000000n;

const NO_EVENTS: RobotEvent[] = [];
const NO_BANDS: ModeBand[] = [];
const ZERO_POSE: Pose = {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0};

type Request = {controller: AbortController; promise: Promise<SeriesBuckets|null>;
                slots: Set<string>};

const seriesKey = (channels: string[], start: string, end: string, points: number) =>
    `${channels.join(',')}|${start}|${end}|${points}`;

const clamp = (t: bigint, lo: bigint, hi: bigint) => t < lo ? lo : t > hi ? hi : t;
const ceilDiv = (a: bigint, b: bigint) => (a + b - 1n) / b;

export type RecordedOptions = {fetch?: FetchLike; clock?: ReviewClock};

const defaultFetch: FetchLike = (url, init) => fetch(url, init);

export class RecordedSource implements DataSource {
  readonly kind = 'recorded';
  version = 0;
  playback: PlaybackState;
  rate: PlayRate = 1;
  error = '';

  private readonly listeners = new Set<() => void>();
  private readonly infos: Map<string, ChannelInfo>;
  private readonly estimate: EstimateChannels|null;
  private readonly fetchFn: FetchLike;
  private readonly clock: ReviewClock;
  private readonly start: bigint;
  private readonly end: bigint;
  private playhead: bigint;
  private stopTicker: (() => void)|null = null;
  private lastTickMs = 0;
  private closed = false;

  private snapshot: Snapshot|null = null;
  private samples = new Map<string, ChannelSample>();
  private chassisReading: {pose: Pose; sample_ns: string}|null = null;
  private snapshotWanted: string;
  private snapshotRequest: {t: string; controller: AbortController}|null = null;

  private readonly cache = new Map<string, SeriesBuckets>();
  private readonly inFlight = new Map<string, Request>();
  private readonly failed = new Set<string>();
  // Per consumer (channels, window, bucket count): its pending key, its last
  // result (at most SERIES_SLOTS).
  private readonly slotPending = new Map<string, string>();
  private readonly slotLast = new Map<string, {key: string; buckets: SeriesBuckets}>();
  // The consumers one render pass (series() calls at one `version`) asked
  // for, and those of the latest pass that asked for any: what is on screen.
  private pass = {version: -1, slots: new Set<string>()};

  static async open(server: string, id: string,
                    options: RecordedOptions = {}): Promise<RecordedSource> {
    const fetchFn = options.fetch ?? defaultFetch;
    const [channels, bounds] = await Promise.all([
      getJson(fetchFn, reviewUrl(server, id, 'channels.json')).then(parseChannels),
      getJson(fetchFn, reviewUrl(server, id, 'bounds.json')).then(parseBounds)
    ]);
    return new RecordedSource(server, id, channels, bounds, fetchFn,
                              options.clock ?? browserClock);
  }

  constructor(readonly server: string, readonly id: string,
              private readonly channels: ChannelInfo[], readonly bounds: ReviewBounds,
              fetchFn: FetchLike, clock: ReviewClock) {
    this.infos = new Map(channels.map(c => [c.name, c]));
    this.estimate = discoverEstimate(channels);
    this.fetchFn = fetchFn;
    this.clock = clock;
    this.start = BigInt(bounds.start_ns);
    this.end = BigInt(bounds.end_ns);
    this.playhead = this.start;
    this.snapshotWanted = bounds.start_ns;
    this.playback = this.makePlayback('PAUSED');
    this.requestSnapshot(bounds.start_ns, true);
  }

  subscribe = (listener: () => void): () => void => {
    this.listeners.add(listener);
    return () => {
      this.listeners.delete(listener);
    };
  };

  private changed() {
    if (this.closed) return;
    for (const listener of this.listeners) listener();
  }

  private makePlayback(mode: Mode): PlaybackState {
    const cursor = this.playhead.toString();
    return {
      mode,
      cursor_ns: cursor,
      // Readings are as old as their snapshot's time, not the playhead's.
      clock_ns: this.snapshot?.timestamp_ns ?? null,
      bounds: {start: this.bounds.start_ns, end: this.bounds.end_ns},
      stats: {skipped: 0, late: 0, invalid: 0},
      status: this.error ? `Review error: ${this.error}` : `Recorded session ${this.id}`,
      connection: this.error ? 'error' : 'replay'
    };
  }

  private setError(message: string) {
    this.error = message;
    this.pause();
    this.playback = this.makePlayback('PAUSED');
    this.changed();
  }

  // Clears the error and lets failed requests run again.
  retry() {
    this.error = '';
    this.failed.clear();
    this.snapshot = null;
    this.playback = this.makePlayback(this.playback.mode);
    this.version++;
    this.requestSnapshot(this.playhead.toString(), true);
    this.changed();
  }

  close() {
    this.pause();
    this.snapshotRequest?.controller.abort();
    this.snapshotRequest = null;
    for (const request of this.inFlight.values()) request.controller.abort();
    this.inFlight.clear();
    this.closed = true;
    this.listeners.clear();
  }

  // --- transport -----------------------------------------------------------

  private moveTo(t: bigint, interrupt: boolean) {
    this.playhead = clamp(t, this.start, this.end);
    this.playback = this.makePlayback(this.stopTicker ? 'REPLAY_PLAYING' : 'PAUSED');
    this.requestSnapshot(this.playhead.toString(), interrupt);
    this.changed();
  }

  seek(t_ns: string) {
    this.moveTo(BigInt(t_ns), true);
  }

  step(dt_ns: string) {
    this.pause();
    this.moveTo(this.playhead + BigInt(dt_ns), true);
  }

  setMode(mode: Mode) {
    if (mode === 'REPLAY_PLAYING') this.play();
    else if (mode === 'PAUSED') this.pause();
  }

  setRate(rate: PlayRate) {
    this.rate = rate;
    this.playback = {...this.playback};
    this.changed();
  }

  play() {
    if (this.stopTicker || this.closed) return;
    if (this.playhead >= this.end) this.playhead = this.start;
    this.lastTickMs = this.clock.now();
    this.stopTicker = this.clock.every(() => this.tick());
    this.moveTo(this.playhead, true);
  }

  pause() {
    if (!this.stopTicker) return;
    this.stopTicker();
    this.stopTicker = null;
    this.playback = this.makePlayback('PAUSED');
    this.changed();
  }

  private tick() {
    const now = this.clock.now();
    const elapsedMs = Math.max(0, now - this.lastTickMs);
    this.lastTickMs = now;
    const next = this.playhead + BigInt(Math.round(elapsedMs * 1e6 * this.rate));
    if (next >= this.end) {
      this.pause();
      this.moveTo(this.end, false);
      return;
    }
    this.moveTo(next, false);
  }

  // --- snapshot ------------------------------------------------------------

  // One snapshot request at a time. A seek aborts the one in flight; while
  // playing, the one in flight completes and the newest playhead follows it.
  private requestSnapshot(t: string, interrupt: boolean) {
    this.snapshotWanted = t;
    if (this.error || this.closed) return;
    const current = this.snapshotRequest;
    if (current) {
      if (current.t === t || !interrupt) return;
      current.controller.abort();
      this.snapshotRequest = null;
    }
    if (this.snapshot?.timestamp_ns === t) return;
    const controller = new AbortController();
    const request = {t, controller};
    this.snapshotRequest = request;
    const url = reviewUrl(this.server, this.id, 'snapshot') +
        '?' + new URLSearchParams({t_ns: t});
    getJson(this.fetchFn, url, controller.signal)
        .then(parseSnapshot)
        .then(snapshot => {
          if (this.snapshotRequest !== request) return;
          this.snapshotRequest = null;
          this.applySnapshot(snapshot);
          if (this.snapshotWanted !== t) this.requestSnapshot(this.snapshotWanted, false);
        }, e => {
          if (this.snapshotRequest !== request) return;
          this.snapshotRequest = null;
          if (!isAbort(e)) this.setError(errorMessage(e));
        });
  }

  private applySnapshot(review: ReviewSnapshot) {
    const channels: Record<string, number> = {};
    const exact: Record<string, string> = {};
    const units: Record<string, string> = {};
    const labels: Record<string, string[]> = {};
    const alerts: Record<string, AlertLevel> = {};
    const samples = new Map<string, ChannelSample>();
    for (const [name, sample] of review.channels) {
      channels[name] = sample.value;
      if (sample.exact !== undefined) exact[name] = sample.exact;
      const info = this.infos.get(name);
      units[name] = info?.unit ?? '';
      if (info?.labels.length) labels[name] = info.labels;
      if (info?.alert) alerts[name] = info.alert;
      samples.set(name, {value: sample.value, timestamp_ns: sample.sample_ns});
    }
    this.snapshot = {
      stream_id: `review:${this.id}`,
      sequence_id: '0',
      timestamp_ns: review.t_ns,
      dictionary_id: 0,
      // The frame-level pose is a live telemetry product; panels read the
      // recorded estimate through chassis().
      chassis: ZERO_POSE,
      channels,
      exact,
      units,
      labels,
      alerts
    };
    this.samples = samples;
    this.chassisReading = this.readChassis();
    this.playback = this.makePlayback(this.playback.mode);
    this.version++;
    this.changed();
  }

  // The estimator's pose (geometry.PoseEstimate: pose, valid), with its own
  // sample time. Invalid or non-finite is no pose, as telemetry's chassis.
  private readChassis(): {pose: Pose; sample_ns: string}|null {
    const est = this.estimate;
    if (!est) return null;
    const x = this.samples.get(est.x), y = this.samples.get(est.y),
          yaw = this.samples.get(est.yaw);
    if (!x || !y || !yaw) return null;
    if (![x.value, y.value, yaw.value].every(Number.isFinite)) return null;
    const valid = this.samples.get(est.x.replace(/pose\/x_m$/, 'valid'));
    if (valid && valid.value !== 1) return null;
    const oldest = [x, y, yaw].map(s => BigInt(s.timestamp_ns))
                       .reduce((a, b) => a < b ? a : b);
    return {pose: {...ZERO_POSE, x: x.value, y: y.value, yaw: yaw.value},
            sample_ns: oldest.toString()};
  }

  // --- series --------------------------------------------------------------

  // Fetch one decimated range. Resolves null when superseded (aborted) or
  // failed; a failure sets `error`. `slot` names the consumer: its earlier
  // request for a different range is aborted unless another consumer shares
  // it.
  fetchSeries(channels: string[], start_ns: string, end_ns: string, points: number,
              slot?: string): Promise<SeriesBuckets|null> {
    const key = seriesKey(channels, start_ns, end_ns, points);
    const owner = slot ?? key;
    const previous = this.slotPending.get(owner);
    if (previous !== undefined && previous !== key) this.release(previous, owner);
    const cached = this.cache.get(key);
    if (cached) {
      this.cache.delete(key);
      this.cache.set(key, cached);
      this.keepLast(owner, key, cached);
      return Promise.resolve(cached);
    }
    let request = this.inFlight.get(key);
    if (!request) {
      const controller = new AbortController();
      const url = reviewUrl(this.server, this.id, 'series') + '?' + new URLSearchParams({
        c: channels.join(','), start_ns, end_ns, points: String(points)
      });
      const created: Request = {controller, slots: new Set(), promise: Promise.resolve(null)};
      created.promise = getBytes(this.fetchFn, url, controller.signal)
          .then(buffer => decodeSeries(buffer, channels.length))
          .then(buckets => {
            this.settle(key, created);
            this.cache.set(key, buckets);
            while (this.cache.size > SERIES_CACHE_ENTRIES)
              this.cache.delete(this.cache.keys().next().value!);
            for (const s of created.slots) this.keepLast(s, key, buckets);
            this.version++;
            this.changed();
            return buckets;
          }, e => {
            this.settle(key, created);
            if (isAbort(e)) return null;
            this.failed.add(key);
            this.setError(errorMessage(e));
            return null;
          });
      request = created;
      this.inFlight.set(key, request);
    }
    request.slots.add(owner);
    this.slotPending.set(owner, key);
    return request.promise;
  }

  private keepLast(slot: string, key: string, buckets: SeriesBuckets) {
    this.slotLast.delete(slot);
    this.slotLast.set(slot, {key, buckets});
    // Least recently kept first, never one the latest render showed: a
    // completion must not evict what the unchanged view will ask for again.
    for (const old of this.slotLast.keys()) {
      if (this.slotLast.size <= SERIES_SLOTS) break;
      if (!this.pass.slots.has(old)) this.slotLast.delete(old);
    }
  }

  // How many consumers' last results are kept: at most SERIES_SLOTS.
  get keptSlots() {
    return this.slotLast.size;
  }

  private settle(key: string, request: Request) {
    if (this.inFlight.get(key) === request) this.inFlight.delete(key);
    for (const s of request.slots)
      if (this.slotPending.get(s) === key) this.slotPending.delete(s);
  }

  private release(key: string, slot: string) {
    this.slotPending.delete(slot);
    const request = this.inFlight.get(key);
    if (!request) return;
    request.slots.delete(slot);
    if (request.slots.size) return;
    this.inFlight.delete(key);
    request.controller.abort();
  }

  // The range a window ending at `cursor` fetches, on a grid so that a moving
  // cursor reuses it, and the part of it the window shows.
  private plan(window: SeriesWindow, cursor: bigint, points: number) {
    const c = clamp(cursor, this.start, this.end);
    const span = this.end - this.start;
    if (window === 'all') {
      const grid = span / ALL_GRID_DIVISIONS > MIN_GRID_NS ? span / ALL_GRID_DIVISIONS : MIN_GRID_NS;
      const end = clamp(this.start + (ceilDiv(c - this.start, grid) || 1n) * grid,
                        this.start, this.end);
      return {start: this.start, end, points, from: this.start, to: c};
    }
    const width = BigInt(Math.max(1, Math.round(window.seconds * 1e9)));
    const grid = width / GRID_DIVISIONS > MIN_GRID_NS ? width / GRID_DIVISIONS : MIN_GRID_NS;
    const end = clamp(this.start + (ceilDiv(c - this.start, grid) || 1n) * grid,
                      this.start, this.end);
    const start = clamp(end - width - grid, this.start, this.end);
    return {start, end, points: points + Math.ceil(points / Number(GRID_DIVISIONS)),
            from: c - width, to: c};
  }

  series(channels: string[], window: SeriesWindow, cursor_ns: string|null,
         points = DEFAULT_POINTS): Series {
    if (!channels.length || cursor_ns === null || this.end === this.start)
      return emptySeries(this.bounds.start_ns, channels);
    const size = quantizePoints(points);
    const plan = this.plan(window, BigInt(cursor_ns), size);
    const start = plan.start.toString(), end = plan.end.toString();
    const windowName = window === 'all' ? 'all' : window.seconds;
    // Requests within the server's limits (bounds.json): the channels split
    // into groups that each fit at full resolution, merged back below.
    const perRequest = Math.max(1, Math.min(
        this.bounds.max_series_channels,
        Math.floor(this.bounds.max_series_values / plan.points)));
    const parts: (SeriesBuckets|undefined)[] = [];
    for (let at = 0; at < channels.length; at += perRequest) {
      const group = channels.slice(at, at + perRequest);
      const key = seriesKey(group, start, end, plan.points);
      const slot = `${group.join(',')}|${windowName}|${size}`;
      if (this.pass.version !== this.version) {
        this.pass = {version: this.version, slots: new Set()};
      }
      this.pass.slots.add(slot);
      const last = this.slotLast.get(slot);
      const have = this.cache.get(key) ?? (last?.key === key ? last.buckets : undefined);
      // A failed request is not issued again until retry(); another one
      // (the cursor or the plot moved) is. One already answered and kept for
      // this consumer is not asked again.
      if (!have && !this.failed.has(key) && !this.closed)
        void this.fetchSeries(group, start, end, plan.points, slot);
      parts.push(have ?? last?.buckets);
    }
    const buckets = mergeParts(parts, channels.length, perRequest);
    return buckets ?
        bucketSeries(channels, buckets, this.bounds.start_ns, plan.from, plan.to) :
        emptySeries(this.bounds.start_ns, channels);
  }

  // --- DataSource getters --------------------------------------------------

  dictionary() {
    return this.channels;
  }
  channelInfo(name: string) {
    return this.infos.get(name);
  }
  snapshotAtCursor() {
    return this.snapshot;
  }
  channelValue(name: string) {
    return this.snapshot?.channels[name];
  }
  channelSample(name: string) {
    return this.samples.get(name) ?? null;
  }
  chassis() {
    return this.chassisReading;
  }
  // Timeline lanes (modes, alerts) are not served yet.
  events() {
    return NO_EVENTS;
  }
  modeBands() {
    return NO_BANDS;
  }
  // The review server's alert intervals are not served yet; a snapshot still
  // carries the alerts active at the playhead.
  alertLatch() {
    return null;
  }
}

// One request's columns per group of `perRequest` channels, in order, as one
// SeriesBuckets of `count` channels; a group with no result yet is empty
// columns. Undefined when no group has one.
function mergeParts(parts: (SeriesBuckets|undefined)[], count: number,
                    perRequest: number): SeriesBuckets|undefined {
  const present = parts.find(p => p !== undefined);
  if (!present) return undefined;
  if (parts.length === 1) return present;
  const buckets = present.buckets;
  const empty = (): SeriesColumns => ({
    t_ns: new Float64Array(buckets).fill(NaN), min: new Float64Array(buckets).fill(NaN),
    max: new Float64Array(buckets).fill(NaN), last: new Float64Array(buckets).fill(NaN),
    count: new Uint32Array(buckets)
  });
  const channels: SeriesColumns[] = [];
  parts.forEach((part, i) => {
    const size = Math.min(perRequest, count - i * perRequest);
    for (let c = 0; c < size; c++)
      channels.push(part && part.buckets === buckets ? part.channels[c] : empty());
  });
  return {buckets, channels};
}

function emptySeries(t0_ns: string, channels: string[]): Series {
  const values: Record<string, Float64Array> = {};
  for (const name of channels) values[name] = new Float64Array(0);
  return {t0_ns, times: new Float64Array(0), values};
}

// Buckets as a Series. Each channel's point for a bucket sits at its own
// t_ns, its last sample's publication time: channels of one request share
// bucket edges, not times. The rows are every distinct point time in
// [from, to], ascending; a channel is NaN (count 0) in a row that is not one
// of its points, which the plot draws through, and has no row for an empty
// bucket.
export function bucketSeries(channels: string[], buckets: SeriesBuckets, t0_ns: string,
                             from: bigint, to: bigint): Series {
  const lo = Number(from), hi = Number(to);
  const times = new Set<number>();
  channels.forEach((_, c) => {
    const column = buckets.channels[c];
    for (let i = 0; i < buckets.buckets; i++) {
      const t = column.t_ns[i];
      if (column.count[i] > 0 && Number.isFinite(t) && t >= lo && t <= hi) times.add(t);
    }
  });
  const rowTimes = [...times].sort((a, b) => a - b);
  const rowOf = new Map(rowTimes.map((t, row) => [t, row]));
  const t0 = Number(t0_ns);
  const values: Record<string, Float64Array> = {};
  const min: Record<string, Float64Array> = {};
  const max: Record<string, Float64Array> = {};
  const count: Record<string, Uint32Array> = {};
  channels.forEach((name, c) => {
    const column = buckets.channels[c];
    values[name] = new Float64Array(rowTimes.length).fill(NaN);
    min[name] = new Float64Array(rowTimes.length).fill(NaN);
    max[name] = new Float64Array(rowTimes.length).fill(NaN);
    count[name] = new Uint32Array(rowTimes.length);
    for (let i = 0; i < buckets.buckets; i++) {
      const row = column.count[i] > 0 ? rowOf.get(column.t_ns[i]) : undefined;
      if (row === undefined) continue;
      values[name][row] = column.last[i];
      min[name][row] = column.min[i];
      max[name][row] = column.max[i];
      count[name][row] = column.count[i];
    }
  });
  return {t0_ns, times: Float64Array.from(rowTimes, t => (t - t0) / 1e9),
          values, min, max, count};
}
