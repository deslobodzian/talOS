// The retained telemetry history of one viewer, shared by the Studio worker
// and the headless agent so both keep the same frames.
//
// A new stream_id is a restarted producer. Its sequence restarts and its
// dictionaries are its own (the wire contract caches them by stream_id and
// dictionary_id), so the jitter state starts over, after the frames still held
// for the old stream are released in order. Its timestamps come from the main
// processor's monotonic clock, which runs on across processes, so the timeline
// continues: snapshots carry channels by name, not by dictionary index. A
// stream whose clock starts behind the history (a simulated or replayed clock)
// cannot continue it and replaces it.
//
// A new launcher session is a different system, so it replaces the history.
// Each system graph names the session of the live telemetry process
// (telemetrySession); the first one seen while a stream is current tags that
// stream. A graph naming another session keeps only the current stream, when
// it began after the old session was last seen (it is the new session's
// producer), and otherwise nothing: the current stream is the old session's,
// so it is retired, and the frames the bridge keeps repeating from it until
// it re-attaches are dropped until another stream appears. With no graph
// naming a session, a restart always continues the history.

import {type Dictionary, dictionaryKey, JitterBuffer, type Snapshot, Timeline} from './core';
import {type SystemGraph, telemetrySession} from './system';

export type HistoryHooks = {
  // The wall-clock arrival (ms) to record with a frame being appended; 0 is
  // Timeline's "no known arrival".
  arrival(frame: Snapshot): number;
  // A frame appended to the timeline, in order, with the arrival recorded.
  appended?(frame: Snapshot, arrivalMs: number): void;
  // The timeline was replaced (`timeline` is the new one) by `kept`, a subset
  // of the old one, in order.
  replaced?(kept: Snapshot[]): void;
};

export class TelemetryHistory {
  jitter = new JitterBuffer(10);
  streamId: string|null = null;
  private historySession: string|null = null;
  private streamSession: string|null = null;
  private retired: string|null = null;

  // `dictionaries` holds, by dictionaryKey, the dictionary of every retained or
  // held frame, for log export.
  constructor(private readonly hooks: HistoryHooks,
              public timeline = new Timeline(),
              readonly dictionaries = new Map<string, Dictionary>()) {}

  // A decoded frame and the dictionary it was decoded against. False when it
  // is dropped as a retired stream's.
  push(frame: Snapshot, dictionary: Dictionary|null, now: number): boolean {
    if (frame.stream_id === this.retired) return false;
    this.retired = null;
    if (this.streamId !== null && frame.stream_id !== this.streamId)
      this.restart(frame);
    this.streamId = frame.stream_id;
    if (dictionary) this.remember(dictionaryKey(frame), dictionary);
    this.jitter.push(frame, now);
    return true;
  }

  // Appends the frames the jitter buffer releases at `now`.
  flush(now: number) {
    for (const frame of this.jitter.flush(now)) {
      const arrival = this.hooks.arrival(frame);
      if (!this.timeline.append(frame, arrival)) {
        this.jitter.stats.invalid++;
        continue;
      }
      this.hooks.appended?.(frame, arrival);
    }
  }

  observe(graph: SystemGraph) {
    const session = telemetrySession(graph);
    if (session === null || session === this.streamSession) return;
    if (this.streamSession === null &&
        (this.historySession === null || this.historySession === session)) {
      this.historySession = this.streamSession = session;
      return;
    }
    const keepCurrent = this.streamSession === null;
    const kept = keepCurrent ?
        this.timeline.frames.filter(f => f.stream_id === this.streamId) :
        [];
    if (!keepCurrent) {
      this.resetJitter();
      this.retired = this.streamId;
    }
    this.historySession = this.streamSession = session;
    this.replace(kept);
  }

  private restart(next: Snapshot) {
    this.flush(Infinity);
    this.resetJitter();
    this.streamSession = null;
    const last = this.timeline.latest();
    if (last && BigInt(next.timestamp_ns) <= BigInt(last.timestamp_ns))
      this.replace([]);
  }

  // Starts the jitter state over, keeping its counters: they describe the
  // link, not one stream.
  private resetJitter() {
    const stats = this.jitter.stats;
    this.jitter = new JitterBuffer(10);
    this.jitter.stats = stats;
  }

  private replace(kept: Snapshot[]) {
    const timeline = new Timeline(this.timeline.capacity);
    for (const f of kept) timeline.append(f);
    timeline.lastArrivalMs = this.timeline.lastArrivalMs;
    this.timeline = timeline;
    this.prune();
    this.hooks.replaced?.(kept);
  }

  private remember(key: string, dictionary: Dictionary) {
    if (this.dictionaries.get(key) === dictionary) return;
    const added = !this.dictionaries.has(key);
    this.dictionaries.set(key, dictionary);
    if (added) this.prune(key);
  }

  // Drops the dictionaries no retained or held frame refers to.
  private prune(current?: string) {
    const used = new Set([...this.timeline.frames, ...this.jitter.held()]
                             .map(dictionaryKey));
    if (current !== undefined) used.add(current);
    for (const key of this.dictionaries.keys())
      if (!used.has(key)) this.dictionaries.delete(key);
  }
}
