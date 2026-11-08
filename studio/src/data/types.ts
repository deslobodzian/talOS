// The data contract every Studio panel builds on. Types only: the store,
// hooks, event engine and discovery helpers in this directory implement it.
//
// Ownership: the Web Worker (../worker.ts) stays the single owner of the
// decoder, jitter buffer, timeline and playback cursor, and of the JSON-RPC
// query surface (agent/rpc.ts). The main thread keeps a columnar render cache
// (store.ts) fed incrementally by the worker, so panels can plot the whole
// retained history without the worker re-posting it.
//
// Time: every absolute timestamp is a uint64 nanosecond count carried as a
// decimal string (JSON and structured clone cannot carry 64-bit integers).
// Plot axes use float seconds relative to an explicit origin.

import type {Mode, Snapshot} from '../core';
import type {DeclaredGraph, SystemGraph} from '../system';

export type {Mode, Snapshot};

export type Bounds = {start: string; end: string};

export type StreamStats = {dropped: number; late: number; invalid: number};

export type Connection = 'idle'|'connecting'|'live'|'demo'|'replay'|'error';

// Where the cursor is and what the stream is doing. Posted by the worker.
export type PlaybackState = {
  mode: Mode;
  cursor_ns: string|null;  // null until the first frame
  clock_ns?: string|null;  // session time, advancing through a live data gap
  updates?: {dropped: number; coalesced: number};
  bounds: Bounds|null;     // retained history; null until the first frame
  stats: StreamStats;
  status: string;          // human-readable connection/import status
  connection: Connection;
};

// What the worker knows about the shape of the system, as opposed to what it
// measured. Arrives at a few hertz, separately from frames.
export type SystemState = {
  // The last good graph. Kept when it goes stale, so the last-known evidence
  // stays inspectable; read liveness through data/freshness.ts.
  graph: SystemGraph|null;
  // The graph is not current: none has arrived, the newest is older than
  // GRAPH_STALE_AFTER_MS, the socket is down, or the worker failed.
  stale: boolean;
  rates: Record<string, number>;
  error: string;
  // Null is the ordinary case ("not offered"), not an error.
  declared: DeclaredGraph|null;
  declaredError: string;
};

// One telemetry channel as the current dictionary declares it.
export type ChannelInfo = {name: string; unit: string; labels: string[]};

// Columnar history for plotting. `times` are seconds since `t0_ns`, strictly
// ascending; each value array is index-aligned with `times` and holds NaN
// where the channel had no reading (absent, stale, or not yet declared).
export type Series = {
  t0_ns: string;
  times: Float64Array;
  values: Record<string, Float64Array>;
};

// How much history a series covers, ending at the playback cursor.
export type SeriesWindow = {seconds: number}|'all';

export type Severity = 'info'|'warn'|'error';

export type EventCategory = 'driver_station'|'power'|'hardware'|'vision'|
    'estimator'|'mechanism'|'system'|'stream';

// Something that happened at one instant, derived from telemetry or the
// system graph. Events are facts read off the data; a detector never infers a
// fault the data does not show.
export type RobotEvent = {
  id: string;     // unique: `${detector}:${t_ns}:${n}`
  t_ns: string;   // telemetry time the change was first observed
  severity: Severity;
  category: EventCategory;
  title: string;  // short, e.g. "Enabled (autonomous)"
  detail?: string;
  channel?: string;  // a related channel, for "plot this"
};

export type EventDraft = Omit<RobotEvent, 'id'|'t_ns'>;

// Detectors run once per new frame, in time order. `prev` is the previous
// frame of the same stream (null at a stream start or after a reset).
export type FrameDetector = {
  id: string;
  detect(prev: Snapshot|null, next: Snapshot): EventDraft[];
};

// Detectors over system-graph updates. Their events are stamped with the
// newest telemetry timestamp at arrival (the graph carries no telemetry time).
export type SystemDetector = {
  id: string;
  detect(prev: SystemGraph|null, next: SystemGraph): EventDraft[];
};

export type DriverStationMode =
    'disconnected'|'disabled'|'autonomous'|'teleop'|'test'|'estop';

// A contiguous stretch of history in one Driver Station mode.
export type ModeBand = {start_ns: string; end_ns: string; mode: DriverStationMode};

// Actions panels and the shell may take. All are forwarded to the worker.
export type StudioActions = {
  connect(url: string): void;
  demo(): void;
  setMode(mode: Mode): void;
  live(): void;
  seek(timestamp_ns: string): void;
  step(dt_ns: string): void;
  importLog(file: File): void;
  exportLog(): void;
  rpc(method: string, params: unknown): Promise<unknown>;
};
