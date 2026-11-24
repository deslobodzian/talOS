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
import {AlertLevel} from '../generated/talos/telemetry/alert-level';
import type {DeclaredGraph} from '../system';
import {
  alertText,
  compareAlerts,
  type AlertLatch,
  type LatchedAlert
} from './alerts';
import {discoverEstimate} from './discovery';
import type {SessionManifest} from './manifest';
import {
  decodeSeries,
  errorMessage,
  fetchDeclared,
  fetchManifest,
  fetchTimeline,
  getBytes,
  getJson,
  isAbort,
  parseBounds,
  parseChannels,
  parseSnapshot,
  reviewUrl,
  type FetchLike,
  type ReviewAlertInterval,
  type ReviewBounds,
  type ReviewGap,
  type ReviewRunInterval,
  type ReviewSnapshot,
  type ReviewTimeline,
  type SeriesBuckets,
  type SeriesColumns,
  type TimelineAlertLevel,
  type TimelineMode
} from './review';
import type {ChannelSample, DataSource} from './source';
import type {
  ChannelInfo,
  DriverStationMode,
  EventCategory,
  Mode,
  ModeBand,
  PlaybackState,
  RobotEvent,
  Series,
  SeriesWindow,
  Severity
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

const NO_BANDS: ModeBand[] = [];
const ZERO_POSE: Pose = {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0};

export const EMPTY_TIMELINE: ReviewTimeline = {
  hardware_not_decoded: false,
  modes: [],
  modes_dropped: 0,
  alerts: [],
  alerts_dropped: 0,
  runs: [],
  runs_dropped: 0
};

type Request = {controller: AbortController; promise: Promise<SeriesBuckets|null>;
                slots: Set<string>};

const seriesKey = (channels: string[], start: string, end: string, points: number) =>
    `${channels.join(',')}|${start}|${end}|${points}`;

const clamp = (t: bigint, lo: bigint, hi: bigint) => t < lo ? lo : t > hi ? hi : t;
const ceilDiv = (a: bigint, b: bigint) => (a + b - 1n) / b;

export type RecordedOptions = {fetch?: FetchLike; clock?: ReviewClock};

const defaultFetch: FetchLike = (url, init) => fetch(url, init);

type PoseChannels = {x: string; y: string; yaw: string; valid: string};

// Resolves chassis pose channels, preferring `[studio] pose_topic` from the
// session's config (bounds.pose_topic) over estimator discovery so recorded
// sessions match live telemetry's SetupPoseExtraction.
function resolvePoseChannels(channels: ChannelInfo[],
                             poseTopic?: string): PoseChannels|null {
  const est = discoverEstimate(channels, poseTopic);
  return est ? {x: est.x, y: est.y, yaw: est.yaw, valid: est.valid} : null;
}

export class RecordedSource implements DataSource {
  readonly kind = 'recorded';
  version = 0;
  playback: PlaybackState;
  rate: PlayRate = 1;
  error = '';

  private readonly listeners = new Set<() => void>();
  private readonly infos: Map<string, ChannelInfo>;
  private readonly poseChannels: PoseChannels|null;
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
  private readonly recordedEvents: RobotEvent[];
  private readonly recordedModeBands: ModeBand[];
  private cachedLatch: {cursor: bigint; latch: AlertLatch}|null = null;

  static async open(server: string, id: string,
                    options: RecordedOptions = {}): Promise<RecordedSource> {
    const fetchFn = options.fetch ?? defaultFetch;
    const [channels, bounds, timeline, declared, manifestResult] = await Promise.all([
      getJson(fetchFn, reviewUrl(server, id, 'channels.json')).then(parseChannels),
      getJson(fetchFn, reviewUrl(server, id, 'bounds.json')).then(parseBounds),
      fetchTimeline(server, id, fetchFn),
      fetchDeclared(server, id, fetchFn),
      fetchManifest(server, id, fetchFn)
    ]);
    return new RecordedSource(server, id, channels, bounds, fetchFn,
                              options.clock ?? browserClock, timeline, declared,
                              manifestResult.manifest, manifestResult.error);
  }

  constructor(readonly server: string, readonly id: string,
              private readonly channels: ChannelInfo[], readonly bounds: ReviewBounds,
              fetchFn: FetchLike, clock: ReviewClock,
              readonly timeline: ReviewTimeline = EMPTY_TIMELINE,
              readonly declared: DeclaredGraph|null = null,
              readonly manifest: SessionManifest|null = null,
              readonly manifestError: string = '') {
    this.infos = new Map(channels.map(c => [c.name, c]));
    this.poseChannels = resolvePoseChannels(channels, bounds.pose_topic);
    this.fetchFn = fetchFn;
    this.clock = clock;
    this.start = BigInt(bounds.start_ns);
    this.end = BigInt(bounds.end_ns);
    this.playhead = this.start;
    this.snapshotWanted = bounds.start_ns;
    this.recordedEvents = timelineEvents(bounds, timeline);
    this.recordedModeBands = timelineModeBands(timeline);
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
      // A session whose hardware packets the server refused says why, so
      // its empty hardware panels are explained (REQ-OPS-51).
      // Log gaps are counted after either (REQ-OPS-41), nothing when none.
      status: (this.error ? `Review error: ${this.error}` :
          this.bounds.hardware_not_decoded ?
          `Recorded session ${this.id} · ${this.bounds.hardware_not_decoded}` :
          `Recorded session ${this.id}`) + gapCount(this.bounds.gaps.length),
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
    const elapsedMs = now - this.lastTickMs;
    this.lastTickMs = now;
    if (!Number.isFinite(elapsedMs) || elapsedMs <= 0) return;
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
    const channels: Record<string, number> = Object.create(null);
    const exact: Record<string, string> = Object.create(null);
    const units: Record<string, string> = Object.create(null);
    const labels: Record<string, string[]> = Object.create(null);
    const alerts: Record<string, AlertLevel> = Object.create(null);
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

  // The configured `[studio] pose_topic` (else discovered estimator) pose,
  // with its own sample time. Invalid or non-finite is no pose, as live.
  private readChassis(): {pose: Pose; sample_ns: string}|null {
    const pose = this.poseChannels;
    if (!pose) return null;
    const x = this.samples.get(pose.x), y = this.samples.get(pose.y),
          yaw = this.samples.get(pose.yaw);
    if (!x || !y || !yaw) return null;
    if (![x.value, y.value, yaw.value].every(Number.isFinite)) return null;
    const valid = this.samples.get(pose.valid);
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
            if (this.failed.size > 256)
              this.failed.delete(this.failed.values().next().value!);
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
    const rawNs = Number.isFinite(window.seconds) && window.seconds > 0 ?
        Math.round(window.seconds * 1e9) : 1;
    const width = Number.isFinite(rawNs) ? BigInt(Math.max(1, rawNs)) : (span > 0n ? span : 1n);
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
    const requested = Number.isFinite(points) && points > 0 ? points : DEFAULT_POINTS;
    const size = quantizePoints(requested);
    const plan = this.plan(window, BigInt(cursor_ns), size);
    const cappedPoints = Math.max(1, Math.min(plan.points, this.bounds.max_series_values));
    const start = plan.start.toString(), end = plan.end.toString();
    const windowName = window === 'all' ? 'all' : window.seconds;
    // Requests within the server's limits (bounds.json): the channels split
    // into groups that each fit at full resolution, merged back below.
    const perRequest = Math.max(1, Math.min(
        this.bounds.max_series_channels,
        Math.floor(this.bounds.max_series_values / cappedPoints)));
    const parts: (SeriesBuckets|undefined)[] = [];
    for (let at = 0; at < channels.length; at += perRequest) {
      const group = channels.slice(at, at + perRequest);
      const key = seriesKey(group, start, end, cappedPoints);
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
        void this.fetchSeries(group, start, end, cappedPoints, slot);
      parts.push(have ?? last?.buckets);
    }
    const buckets = mergeParts(parts, channels.length, perRequest);
    return buckets ?
        bucketSeries(channels, buckets, this.bounds.start_ns, plan.from, plan.to,
                     this.bounds.gaps, this.timeline.runs) :
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
  events() {
    return this.recordedEvents;
  }
  modeBands() {
    return this.recordedModeBands;
  }
  // Active and latched alerts up to the current playhead from timeline.json
  // (REQ-OPS-37), cached per playhead position for useSyncExternalStore.
  alertLatch(): AlertLatch {
    if (this.cachedLatch && this.cachedLatch.cursor === this.playhead)
      return this.cachedLatch.latch;
    const latch = timelineAlertLatch(this.bounds, this.timeline, this.playhead);
    this.cachedLatch = {cursor: this.playhead, latch};
    return latch;
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
  const values: Record<string, Float64Array> = Object.create(null);
  for (const name of channels) values[name] = new Float64Array(0);
  return {t0_ns, times: new Float64Array(0), values};
}

type BreakSpan = {
  process: string;
  start: number;
  end: number;
  kind: 'gap' | 'run_gap' | 'run_restart' | 'run_exit';
};

function buildBreakSpans(gaps: readonly ReviewGap[],
                         runs: readonly ReviewRunInterval[],
                         lo: number, hi: number): BreakSpan[] {
  const spans: BreakSpan[] = [];
  for (const g of gaps) {
    const start = Number(BigInt(g.start_ns));
    const end = Number(BigInt(g.end_ns));
    if (end >= lo && start <= hi) spans.push({process: g.process, start, end, kind: 'gap'});
  }
  const byNode = new Map<string, ReviewRunInterval[]>();
  for (const run of runs) {
    let list = byNode.get(run.node);
    if (!list) {
      list = [];
      byNode.set(run.node, list);
    }
    list.push(run);
  }
  for (const [node, list] of byNode) {
    const sorted = list.length > 1 ?
        [...list].sort((a, b) => {
          const dt = BigInt(a.start_ns) - BigInt(b.start_ns);
          return dt < 0n ? -1 : dt > 0n ? 1 : 0;
        }) :
        list;
    for (let i = 1; i < sorted.length; i++) {
      const prev = sorted[i - 1], curr = sorted[i];
      const restart = Number(BigInt(curr.start_ns));
      if (prev.end_ns !== null) {
        const prevEnd = Number(BigInt(prev.end_ns));
        if (prevEnd <= restart) {
          if (restart >= lo && prevEnd <= hi)
            spans.push({process: node, start: prevEnd, end: restart, kind: 'run_gap'});
          continue;
        }
      }
      if (restart >= lo && restart <= hi)
        spans.push({process: node, start: restart, end: restart, kind: 'run_restart'});
    }
    const last = sorted[sorted.length - 1];
    if (last.end_ns !== null) {
      const exitAt = Number(BigInt(last.end_ns));
      if (exitAt >= lo && exitAt <= hi)
        spans.push({process: node, start: exitAt, end: exitAt, kind: 'run_exit'});
    }
  }
  return spans;
}

function channelAffectedByBreak(channel: string, span: BreakSpan): boolean {
  if (!span.process) return true;
  const parts = channel.split('/');
  if (parts.length < 2 || !parts[0]) return true;
  const owner = parts[0];
  if (owner === 'hw') {
    if (parts[1] === 'request' && parts.length >= 3 && parts[2]) {
      return parts[2] === span.process;
    }
    if (span.process === 'hardware_node' || span.process === 'bridge') return true;
    // A session without a bridge log may read /hw/state/driver_station from a
    // reader process; straddling the gap interval still avoids false breaks.
    return span.kind === 'gap';
  }
  // Target-lane channels (`<subsystem>/target/<source>/*`) are written by
  // `<source>`, not `<subsystem>`.
  const writer = (parts[1] === 'target' && parts.length >= 3 && parts[2]) ?
      parts[2] : owner;
  return writer === span.process;
}

// Finds the break timestamp strictly between two consecutive points of `pts`
// separated by `span`, or null when no consecutive pair straddles `span`.
function findBreakBetween(pts: readonly number[], span: BreakSpan): number|null {
  if (pts.length < 2) return null;
  // Binary search for the largest index k with pts[k] <= span.start
  // (or pts[k] < span.start for run_restart).
  let lo = 0, hi = pts.length - 1, k = -1;
  while (lo <= hi) {
    const mid = (lo + hi) >>> 1;
    const ok = span.kind === 'run_restart' ? pts[mid] < span.start : pts[mid] <= span.start;
    if (ok) {
      k = mid;
      lo = mid + 1;
    } else {
      hi = mid - 1;
    }
  }
  if (k < 0 || k + 1 >= pts.length) return null;
  const a = pts[k], b = pts[k + 1];
  const matches =
      span.kind === 'run_exit' ? (a <= span.end && span.end < b) :
      span.kind === 'run_restart' ? (a < span.start && span.start <= b) :
      (a <= span.start && span.end <= b && a < b);
  if (!matches) return null;
  let tBreak = (span.start + span.end) / 2;
  if (!(tBreak > a && tBreak < b)) tBreak = (a + b) / 2;
  return (tBreak > a && tBreak < b) ? tBreak : null;
}

// Buckets as a Series. Each channel's point for a bucket sits at its own
// t_ns, its last sample's publication time: channels of one request share
// bucket edges, not times. The rows are every distinct point time in
// [from, to], ascending; a channel is NaN (count 0) in a row that is not one
// of its points, which the plot draws through (plotColumn -> undefined), and
// has no row for an empty bucket. Explicit gap-break rows (NaN with count 1,
// which plotColumn maps to null to break the uPlot line) are inserted ONLY at
// real `bounds.gaps` and `timeline.runs` restart/exit boundaries — never on a
// fixed time threshold.
export function bucketSeries(channels: string[], buckets: SeriesBuckets, t0_ns: string,
                             from: bigint, to: bigint,
                             gaps: readonly ReviewGap[] = [],
                             runs: readonly ReviewRunInterval[] = []): Series {
  const lo = Number(from), hi = Number(to);
  const times = new Set<number>();
  const channelPoints: number[][] = channels.map((_, c) => {
    const column = buckets.channels[c];
    const pts = new Set<number>();
    for (let i = 0; i < buckets.buckets; i++) {
      const t = column.t_ns[i];
      if (column.count[i] > 0 && Number.isFinite(t) && t >= lo && t <= hi) {
        times.add(t);
        pts.add(t);
      }
    }
    return [...pts].sort((a, b) => a - b);
  });
  const breakSpans = (gaps.length || runs.length) ? buildBreakSpans(gaps, runs, lo, hi) : [];
  const channelBreaks: number[][] = channels.map(() => []);
  if (breakSpans.length) {
    for (let c = 0; c < channels.length; c++) {
      const pts = channelPoints[c];
      if (pts.length < 2) continue;
      for (const span of breakSpans) {
        if (!channelAffectedByBreak(channels[c], span)) continue;
        const tBreak = findBreakBetween(pts, span);
        if (tBreak === null) continue;
        times.add(tBreak);
        channelBreaks[c].push(tBreak);
      }
    }
  }
  const rowTimes = [...times].sort((a, b) => a - b);
  const rowOf = new Map(rowTimes.map((t, row) => [t, row]));
  const t0 = Number(t0_ns);
  const values: Record<string, Float64Array> = Object.create(null);
  const min: Record<string, Float64Array> = Object.create(null);
  const max: Record<string, Float64Array> = Object.create(null);
  const count: Record<string, Uint32Array> = Object.create(null);
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
    for (const tBreak of channelBreaks[c]) {
      const row = rowOf.get(tBreak);
      if (row !== undefined && count[name][row] === 0) count[name][row] = 1;
    }
  });
  return {t0_ns, times: Float64Array.from(rowTimes, t => (t - t0) / 1e9),
          values, min, max, count};
}

function gapCount(n: number): string {
  return n === 0 ? '' : n === 1 ? ' · 1 log gap' : ` · ${n} log gaps`;
}

// The narrowest gap band, as a fraction of the scrub bar, so a gap of a few
// dropped records is still visible over a long session.
export const GAP_MIN_WIDTH = 0.004;

// A gap's band on the scrub bar: left and width as fractions of the session,
// and the tooltip naming what was lost (REQ-OPS-41).
export type GapBand = {left: number; width: number; title: string};

export function gapBands(bounds: ReviewBounds): GapBand[] {
  const start = BigInt(bounds.start_ns);
  const span = Number(BigInt(bounds.end_ns) - start);
  const at = (t: string) => span <= 0 ? 0 :
      Math.min(1, Math.max(0, Number(BigInt(t) - start) / span));
  return bounds.gaps.map(g => {
    const width = Math.max(GAP_MIN_WIDTH, at(g.end_ns) - at(g.start_ns));
    const left = Math.min(at(g.start_ns), 1 - width);
    const seconds = (Number(BigInt(g.end_ns) - BigInt(g.start_ns)) / 1e9).toFixed(3);
    return {
      left, width,
      title: `log gap in ${g.process}: ${seconds} s, lost ${g.records} records, ` +
          `${g.dispatches} dispatches, ${g.sends} sends` +
          (g.closed_at_flush ? '; closed at the stop' : '')
    };
  });
}

const TIMELINE_TO_DS_MODE: Record<TimelineMode, DriverStationMode> = {
  disabled: 'disabled',
  teleop: 'teleop',
  auto: 'autonomous',
  test: 'test',
  estop: 'estop'
};

const TIMELINE_TO_ALERT_LEVEL: Record<TimelineAlertLevel, AlertLevel> = {
  error: AlertLevel.Error,
  warning: AlertLevel.Warning,
  info: AlertLevel.Info
};

const ALERT_TO_SEVERITY: Record<TimelineAlertLevel, Severity> = {
  error: 'error',
  warning: 'warn',
  info: 'info'
};

const CLEAN_RUN_REASONS: ReadonlySet<string> = new Set(['stop', 'exit', 'running']);

export const isUncleanRunExit = (reason: string, end_ns: string|null): boolean =>
    end_ns !== null && !CLEAN_RUN_REASONS.has(reason);

function alertCategory(channel: string): EventCategory {
  if (channel.startsWith('hw/status/')) {
    const rest = channel.slice('hw/status/'.length);
    if (rest.startsWith('battery') || rest.startsWith('brownout') || rest.startsWith('can_'))
      return 'power';
    return 'hardware';
  }
  if (channel.startsWith('hw/')) return 'hardware';
  return 'system';
}

function modeEventTitle(mode: DriverStationMode): string {
  if (mode === 'estop') return 'E-stop';
  if (mode === 'disabled') return 'Disabled';
  return `Enabled · ${mode}`;
}

export function timelineModeBands(timeline: ReviewTimeline): ModeBand[] {
  if (timeline.hardware_not_decoded) return NO_BANDS;
  return timeline.modes.map(m => ({
    start_ns: m.start_ns,
    end_ns: m.end_ns,
    mode: TIMELINE_TO_DS_MODE[m.mode]
  }));
}

const alertActiveAtEnd = (a: ReviewAlertInterval, sessionEnd: bigint): boolean =>
    a.active_at_end !== undefined ? a.active_at_end : BigInt(a.end_ns) >= sessionEnd;

export function timelineEvents(bounds: ReviewBounds, timeline: ReviewTimeline): RobotEvent[] {
  const out: RobotEvent[] = [];
  let seq = 0;
  const add = (tag: string, t_ns: string, severity: Severity,
               category: EventCategory, title: string, detail?: string,
               channel?: string) => {
    const event: RobotEvent = {
      id: `recorded-${tag}:${t_ns}:${++seq}`,
      t_ns,
      severity,
      category,
      title
    };
    if (detail !== undefined) event.detail = detail;
    if (channel !== undefined) event.channel = channel;
    out.push(event);
  };

  // Mode transitions (or one explicit notice when hardware was not decoded,
  // never misreporting undecoded hardware as disabled; REQ-OPS-51).
  if (timeline.hardware_not_decoded) {
    add('hw-not-decoded', bounds.start_ns, 'warn', 'hardware',
        'Hardware packets not decoded',
        bounds.hardware_not_decoded ?? 'Driver Station modes unavailable');
  } else {
    for (let i = 0; i < timeline.modes.length; i++) {
      const m = timeline.modes[i];
      const dsMode = TIMELINE_TO_DS_MODE[m.mode];
      const prev = i > 0 ? timeline.modes[i - 1] : null;
      if (!prev || prev.mode !== m.mode) {
        const severity: Severity = dsMode === 'estop' ? 'error' : 'info';
        const detail = prev?.mode === 'estop' ? 'from estop' :
            m.fms ? 'FMS attached' : undefined;
        add('mode', m.start_ns, severity, 'driver_station',
            modeEventTitle(dsMode), detail);
      }
      if (prev && prev.fms !== m.fms) {
        add('fms', m.start_ns, 'info', 'driver_station',
            m.fms ? 'FMS attached' : 'FMS detached');
      }
    }
  }

  // Alert activations and clears from timeline.alerts (REQ-OPS-37).
  const sessionEnd = BigInt(bounds.end_ns);
  for (const a of timeline.alerts) {
    const levelEnum = TIMELINE_TO_ALERT_LEVEL[a.level];
    const owner = a.channel.slice(0, Math.max(0, a.channel.indexOf('/')));
    const text = alertText(a.channel, levelEnum);
    const label = owner ? `${owner}: ${text}` : text;
    const category = alertCategory(a.channel);
    add('alert-on', a.start_ns, ALERT_TO_SEVERITY[a.level], category, label,
        `alert raised (${a.level})`, a.channel);
    if (!alertActiveAtEnd(a, sessionEnd)) {
      const seconds = (Number(BigInt(a.end_ns) - BigInt(a.start_ns)) / 1e9).toFixed(3);
      add('alert-off', a.end_ns, 'info', category, `${label} cleared`,
          `active ${seconds} s`, a.channel);
    }
  }

  // Node restarts (non-initial runs) and unclean run exits from manifest runs[].
  const priorRun = new Map<string, {count: number; reason: string}>();
  for (const run of timeline.runs) {
    const seen = priorRun.get(run.node);
    const runNumber = (seen?.count ?? 0) + 1;
    if (seen) {
      add('run-restart', run.start_ns, 'warn', 'system',
          `${run.node} restarted (run ${runNumber})`,
          seen.reason ? `after ${seen.reason.replace(/_/g, ' ')}` : undefined);
    }
    if (isUncleanRunExit(run.reason, run.end_ns)) {
      add('run-exit', run.end_ns!, 'error', 'system',
          `${run.node} ${run.reason.replace(/_/g, ' ')}`,
          `run ${runNumber}`);
    }
    priorRun.set(run.node, {count: runNumber, reason: run.reason});
  }

  // Log gaps from bounds.gaps (REQ-OPS-41).
  for (const g of bounds.gaps) {
    const seconds = (Number(BigInt(g.end_ns) - BigInt(g.start_ns)) / 1e9).toFixed(3);
    add('gap', g.start_ns, 'warn', 'stream', `Log gap in ${g.process}`,
        `${seconds} s, lost ${g.records} records (${g.dispatches} dispatches, ${g.sends} sends)` +
            (g.closed_at_flush ? '; closed at stop' : ''));
  }

  // Sort chronologically, keeping emission order for equal timestamps.
  out.sort((a, b) => {
    const dt = BigInt(a.t_ns) - BigInt(b.t_ns);
    return dt < 0n ? -1 : dt > 0n ? 1 : 0;
  });
  return out;
}

export function timelineAlertLatch(bounds: ReviewBounds, timeline: ReviewTimeline,
                                   playhead: bigint): AlertLatch {
  const sessionEnd = BigInt(bounds.end_ns);
  const byChannel = new Map<string, LatchedAlert>();
  for (const a of timeline.alerts) {
    const start = BigInt(a.start_ns);
    if (start > playhead) continue;
    const end = BigInt(a.end_ns);
    const activeAtPlayhead =
        (start <= playhead && playhead < end) ||
        (alertActiveAtEnd(a, sessionEnd) && start <= playhead && playhead <= end);
    const level = TIMELINE_TO_ALERT_LEVEL[a.level];
    const existing = byChannel.get(a.channel);
    if (existing) {
      existing.count++;
      if (start < BigInt(existing.first_ns)) existing.first_ns = a.start_ns;
      if (start > BigInt(existing.last_ns)) existing.last_ns = a.start_ns;
      existing.active = existing.active || activeAtPlayhead;
    } else {
      byChannel.set(a.channel, {
        channel: a.channel,
        level,
        owner: a.channel.slice(0, Math.max(0, a.channel.indexOf('/'))),
        text: alertText(a.channel, level),
        first_ns: a.start_ns,
        last_ns: a.start_ns,
        count: 1,
        active: activeAtPlayhead
      });
    }
  }
  return {
    matchStart_ns: bounds.start_ns,
    matchKey: null,
    alerts: [...byChannel.values()].sort(compareAlerts)
  };
}

export type ModeLane = {
  left: number;
  width: number;
  mode: DriverStationMode;
  fms: boolean;
  title: string;
};

export type AlertMark = {
  left: number;
  width: number;
  level: TimelineAlertLevel;
  channel: string;
  title: string;
  start_ns: string;
};

export type RestartMark = {
  left: number;
  node: string;
  kind: 'restart'|'exit';
  reason: string;
  title: string;
  t_ns: string;
};

function fraction(bounds: ReviewBounds) {
  const start = BigInt(bounds.start_ns);
  const span = Number(BigInt(bounds.end_ns) - start);
  return {
    start,
    at: (t: string) => span <= 0 ? 0 :
        Math.min(1, Math.max(0, Number(BigInt(t) - start) / span)),
    offsetS: (t: string) => (Number(BigInt(t) - start) / 1e9).toFixed(3)
  };
}

export function recordedModeLanes(bounds: ReviewBounds,
                                  timeline: ReviewTimeline): ModeLane[] {
  if (timeline.hardware_not_decoded) return [];
  const f = fraction(bounds);
  return timeline.modes.map(m => {
    const left = f.at(m.start_ns);
    const right = f.at(m.end_ns);
    const width = Math.max(GAP_MIN_WIDTH, right - left);
    const clampedLeft = Math.min(left, 1 - width);
    const mode = TIMELINE_TO_DS_MODE[m.mode];
    const fmsLabel = m.fms ? ' (FMS)' : '';
    return {
      left: clampedLeft,
      width,
      mode,
      fms: m.fms,
      title: `${mode}${fmsLabel}: ${f.offsetS(m.start_ns)} s – ${f.offsetS(m.end_ns)} s`
    };
  });
}

export function alertMarks(bounds: ReviewBounds,
                           timeline: ReviewTimeline): AlertMark[] {
  const f = fraction(bounds);
  return timeline.alerts.map(a => {
    const left = f.at(a.start_ns);
    const width = Math.max(GAP_MIN_WIDTH, f.at(a.end_ns) - left);
    const clampedLeft = Math.min(left, 1 - width);
    const owner = a.channel.slice(0, Math.max(0, a.channel.indexOf('/')));
    const text = alertText(a.channel, TIMELINE_TO_ALERT_LEVEL[a.level]);
    const label = owner ? `${owner}: ${text}` : text;
    return {
      left: clampedLeft,
      width,
      level: a.level,
      channel: a.channel,
      start_ns: a.start_ns,
      title: `${a.level} · ${label} (${f.offsetS(a.start_ns)} s – ${f.offsetS(a.end_ns)} s)`
    };
  });
}

export function restartMarks(bounds: ReviewBounds,
                             timeline: ReviewTimeline): RestartMark[] {
  const f = fraction(bounds);
  const marks: RestartMark[] = [];
  const seenNodes = new Map<string, number>();
  for (const run of timeline.runs) {
    const count = (seenNodes.get(run.node) ?? 0) + 1;
    seenNodes.set(run.node, count);
    if (count > 1) {
      marks.push({
        left: f.at(run.start_ns),
        node: run.node,
        kind: 'restart',
        reason: run.reason,
        t_ns: run.start_ns,
        title: `${run.node} restarted (run ${count}) at ${f.offsetS(run.start_ns)} s`
      });
    }
    if (isUncleanRunExit(run.reason, run.end_ns)) {
      marks.push({
        left: f.at(run.end_ns!),
        node: run.node,
        kind: 'exit',
        reason: run.reason,
        t_ns: run.end_ns!,
        title: `${run.node} ${run.reason.replace(/_/g, ' ')} at ${f.offsetS(run.end_ns!)} s`
      });
    }
  }
  return marks;
}
