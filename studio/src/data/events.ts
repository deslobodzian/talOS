// Runs the built-in detectors over incoming frames and system graphs and
// retains the events they produce. Frames run in order; each detector sees the
// previous frame of the same stream, so it reports edges and never a steady
// state. System detectors are stamped with the newest telemetry timestamp
// because a graph carries no telemetry time of its own.

import type {Snapshot} from '../core';
import type {SystemGraph} from '../system';
import type {SessionManifest} from './manifest';
import type {EventDraft, FrameDetector, RobotEvent, SystemDetector} from './types';
import {createFrameDetectors, MANIFEST_DETECTORS, type StampedDraft, SYSTEM_DETECTORS} from './detectors';

// A detector with per-engine state (the counter run trackers), which an engine
// must clear when its timeline is replaced.
type StatefulFrameDetector = FrameDetector & {reset?: () => void};

export class EventEngine {
  private readonly frameDetectors: readonly FrameDetector[];
  private readonly systemDetectors: readonly SystemDetector[];
  private prev: Snapshot|null = null;
  private prevSystem: SystemGraph|null = null;
  private prevManifest: SessionManifest|null = null;
  private lastStreamId: string|null = null;
  private lastTime: string|null = null;
  private seq = new Map<string, number>();
  private lastValues = new Map<string, number>();
  private lastLabels = new Map<string, string[]>();
  private _events: RobotEvent[] = [];

  constructor(frameDetectors: readonly FrameDetector[] = createFrameDetectors(),
              systemDetectors: readonly SystemDetector[] = SYSTEM_DETECTORS) {
    this.frameDetectors = frameDetectors;
    this.systemDetectors = systemDetectors;
  }

  get events(): RobotEvent[] {
    return this._events;
  }

  pushFrame(frame: Snapshot) {
    // A new stream is a producer restart. `lastStreamId` survives a reset, so
    // the first frame after a timeline replacement still reveals the change.
    if (this.lastStreamId !== null && this.lastStreamId !== frame.stream_id) {
      this.emit('stream', frame.timestamp_ns, {
        severity: 'info',
        category: 'stream',
        title: 'Producer restarted',
        detail: `stream ${frame.stream_id}`
      });
    }
    const prev = this.carriedPrev();
    for (const detector of this.frameDetectors) {
      for (const draft of detector.detect(prev, frame)) {
        // A deferred start draft carries its own stamp (see StampedDraft): it
        // is classified after the run began but belongs at the run's true
        // start time.
        const stamp = (draft as StampedDraft).t0_ns ?? frame.timestamp_ns;
        this.emit(detector.id, stamp, draft);
      }
    }
    for (const [name, value] of Object.entries(frame.channels))
      this.lastValues.set(name, value);
    for (const [name, labels] of Object.entries(frame.labels))
      this.lastLabels.set(name, labels);
    this.prev = frame;
    this.lastStreamId = frame.stream_id;
    this.lastTime = frame.timestamp_ns;
  }

  // The previous frame, with the last known value (and labels) of every
  // channel it does not carry overlaid on top. A change that happens while a
  // channel is absent or stale is still detected as an edge when the channel
  // returns with its new value. When nothing is missing it returns `prev`
  // unchanged, so a fully-populated stream allocates nothing per frame.
  private carriedPrev(): Snapshot|null {
    const prev = this.prev;
    if (prev === null) return null;
    let missing = false;
    for (const name of this.lastValues.keys())
      if (!(name in prev.channels)) {
        missing = true;
        break;
      }
    if (!missing)
      for (const name of this.lastLabels.keys())
        if (!(name in prev.labels)) {
          missing = true;
          break;
        }
    if (!missing) return prev;

    const channels = Object.assign(Object.create(null), prev.channels);
    for (const [name, value] of this.lastValues)
      if (!(name in prev.channels)) channels[name] = value;
    const labels = Object.assign(Object.create(null), prev.labels);
    for (const [name, entry] of this.lastLabels)
      if (!(name in prev.labels)) labels[name] = entry;
    return {...prev, channels, labels};
  }

  pushSystem(graph: SystemGraph) {
    // No telemetry yet means there is no time to stamp a system event at, and
    // "not measured yet" is not "now". The graph is still remembered as the
    // baseline, so the first graph after telemetry starts detects real edges.
    if (this.lastTime !== null) {
      for (const detector of this.systemDetectors) {
        for (const draft of detector.detect(this.prevSystem, graph))
          this.emit(detector.id, this.lastTime, draft);
      }
    }
    this.prevSystem = graph;
  }

  // A new session manifest (arrives at 1 Hz from the bridge). Stamped like the
  // system graph: with the newest telemetry time, since the manifest carries no
  // telemetry time of its own.
  pushManifest(manifest: SessionManifest) {
    if (this.lastTime !== null) {
      for (const detector of MANIFEST_DETECTORS) {
        for (const draft of detector.detect(this.prevManifest, manifest))
          this.emit(detector.id, this.lastTime, draft);
      }
    }
    this.prevManifest = manifest;
  }

  // Timeline replaced (connect, demo, new stream, import): value detectors
  // must not compare across streams, so `prev`, `prevSystem` and `prevManifest`
  // reset. Events clear with the history they were read off. `lastStreamId` is
  // kept so the stream detector can still report a producer restart.
  clear() {
    this.prev = null;
    this.prevSystem = null;
    this.prevManifest = null;
    this.lastTime = null;
    this.lastValues.clear();
    this.lastLabels.clear();
    for (const detector of this.frameDetectors)
      (detector as StatefulFrameDetector).reset?.();
    this._events = [];
    this.seq.clear();
  }

  resetSystem() {
    this.prevSystem = null;
  }

  // Replace the retained events wholesale (used by createStaticStudio to
  // inject a prepared event list without replaying the frames).
  setEvents(events: RobotEvent[]) {
    this._events = [...events];
  }

  // Drop events whose time is no longer within the retained bounds.
  trim(start_ns: string|null) {
    if (start_ns === null) {
      this._events = [];
      return;
    }
    const bound = BigInt(start_ns);
    this._events = this._events.filter(e => BigInt(e.t_ns) >= bound);
  }

  private emit(detector: string, t_ns: string, draft: EventDraft) {
    const n = (this.seq.get(detector) ?? 0) + 1;
    this.seq.set(detector, n);
    const event: RobotEvent = {
      id: `${detector}:${t_ns}:${n}`,
      t_ns,
      severity: draft.severity,
      category: draft.category,
      title: draft.title
    };
    if (draft.detail !== undefined) event.detail = draft.detail;
    if (draft.channel !== undefined) event.channel = draft.channel;

    const key = BigInt(t_ns);
    const last = this._events.at(-1);
    if (!last || BigInt(last.t_ns) <= key) {
      this._events = [...this._events, event];
      return;
    }
    // Late insert (a deferred start carries the run's true start time, which
    // predates events emitted while it was being classified). Keep ascending
    // t_ns order; equal timestamps append after existing ones.
    let lo = 0, hi = this._events.length;
    while (lo < hi) {
      const mid = (lo + hi) >>> 1;
      if (BigInt(this._events[mid].t_ns) <= key) lo = mid + 1;
      else hi = mid;
    }
    const next = this._events.slice();
    next.splice(lo, 0, event);
    this._events = next;
  }
}
