// The Studio object a panel reads through: the columnar store, the playback
// cursor and system graph, and the actions that drive the worker. One
// subscription covers all three, so `useSyncExternalStore` hooks can read a
// narrow slice and re-render only when that slice changes.
//
// Panels read data through `source` (source.ts): the live store, or a recorded
// session while one is open (`openRecorded`). One source is shown at a time.

import {createContext, createElement, useContext} from 'react';
import type {ReactNode} from 'react';

import type {Snapshot} from '../core';
import type {SessionManifest} from './manifest';
import type {RecordedSource} from './recorded';
import {LiveSource, type DataSource} from './source';
import {TelemetryStore} from './store';
import {WorkerClient} from './worker-client';
import type {
  Bounds,
  Mode,
  PlaybackState,
  RobotEvent,
  StudioActions,
  SystemState
} from './types';
import type {WorkerMessage, WorkerPort} from './worker-client';
import {STALE_AFTER_NS} from './freshness';

export type Studio = {
  store: TelemetryStore;
  playback: PlaybackState;
  system: SystemState;
  manifest: SessionManifest|null;
  manifestError: string;
  actions: StudioActions;
  // The source panels read: `recorded` while one is open, else live.
  readonly source: DataSource;
  readonly recorded: RecordedSource|null;
  // Shows a recorded session in place of live data, closing any previous one.
  openRecorded(source: RecordedSource): void;
  closeRecorded(): void;
  subscribe(listener: () => void): () => void;
};

const initialPlayback: PlaybackState = {
  mode: 'LIVE_STREAMING',
  cursor_ns: null,
  bounds: null,
  stats: {skipped: 0, late: 0, invalid: 0},
  status: 'Disconnected',
  connection: 'idle'
};

const initialSystem: SystemState = {
  graph: null, stale: true, rates: {}, error: '', declared: null, declaredError: ''
};

const defaultScheduler = (cb: () => void): void => {
  if (typeof requestAnimationFrame === 'function') requestAnimationFrame(() => cb());
  else setTimeout(cb, 0);
};

function boundsOf(frames: readonly Snapshot[]): Bounds|null {
  if (!frames.length) return null;
  return {start: frames[0].timestamp_ns, end: frames[frames.length - 1].timestamp_ns};
}

class StudioImpl implements Studio {
  store: TelemetryStore;
  playback: PlaybackState;
  system: SystemState;
  manifest: SessionManifest|null;
  manifestError: string;
  actions: StudioActions;
  recorded: RecordedSource|null = null;
  private readonly live: LiveSource;
  private unsubscribeRecorded: (() => void)|null = null;
  private readonly listeners = new Set<() => void>();
  private notifyScheduled = false;
  private readonly scheduler: (cb: () => void) => void;

  constructor(store: TelemetryStore, actions: StudioActions,
              scheduler: (cb: () => void) => void, playback: PlaybackState,
              system: SystemState, manifest: SessionManifest|null = null,
              manifestError = '') {
    this.store = store;
    this.actions = actions;
    this.scheduler = scheduler;
    this.playback = playback;
    this.system = system;
    this.manifest = manifest;
    this.manifestError = manifestError;
    this.live = new LiveSource(store, () => this.playback);
  }

  get source(): DataSource {
    return this.recorded ?? this.live;
  }

  openRecorded = (source: RecordedSource): void => {
    this.closeRecorded();
    this.recorded = source;
    this.unsubscribeRecorded = source.subscribe(this.notify);
    this.notify();
  };

  closeRecorded = (): void => {
    if (!this.recorded) return;
    this.unsubscribeRecorded?.();
    this.unsubscribeRecorded = null;
    this.recorded.close();
    this.recorded = null;
    this.notify();
  };

  subscribe = (listener: () => void): () => void => {
    this.listeners.add(listener);
    return () => {
      this.listeners.delete(listener);
    };
  };

  notify = (): void => {
    if (this.notifyScheduled) return;
    this.notifyScheduled = true;
    this.scheduler(() => {
      this.notifyScheduled = false;
      for (const listener of this.listeners) listener();
    });
  };
}

// A worker that has posted nothing for as long as a sample takes to go stale
// is not keeping time: it posts playback at ~30 Hz whenever it runs.
export const WORKER_SILENT_MS = Number(STALE_AFTER_NS / 1000000n);

// `factory` and `options` are the test seam: a fake port, and the silence
// watchdog's monotonic clock and periodic check.
export function createWorkerStudio(
    factory: () => WorkerPort = () =>
        new Worker(new URL('../worker.ts', import.meta.url), {type: 'module'}),
    options: {now?: () => number; watch?: (check: () => void) => void} = {}): Studio {
  const now = options.now ?? (() => performance.now());
  const watch = options.watch ?? (check => { setInterval(check, 250); });
  const store = new TelemetryStore(defaultScheduler);
  const studio = new StudioImpl(
      store, /*actions=*/{} as StudioActions, defaultScheduler, initialPlayback,
      initialSystem);
  // Live session time and graph freshness come from the worker. A failed or
  // silent one sends neither, so the last clock it posted must stop saying
  // "now": live readings become unknown and the graph last-known. Paused and
  // replay views are judged at their cursor, not wall time, so they keep it.
  const invalidateCurrent = (): boolean => {
    const before = [studio.playback, studio.system];
    if (studio.playback.mode === 'LIVE_STREAMING' && studio.playback.clock_ns !== null)
      studio.playback = {...studio.playback, clock_ns: null};
    if (!studio.system.stale) studio.system = {...studio.system, stale: true};
    return before[0] !== studio.playback || before[1] !== studio.system;
  };
  let lastUpdate = now();
  watch(() => {
    if (now() - lastUpdate >= WORKER_SILENT_MS && invalidateCurrent()) studio.notify();
  });
  const client = new WorkerClient(
      factory,
      data => receive(data),
      error => {
        studio.playback = {...studio.playback, connection: 'error', status: error.message};
        invalidateCurrent();
        studio.notify();
      });
  const post = (data: object, transfer?: Transferable[]) => client.post(data, transfer);

  studio.actions = {
    connect(url: string) {
      client.restart();
      post({type: 'connect', url});
    },
    demo() {
      client.restart();
      post({type: 'demo'});
    },
    setMode(mode: Mode) {
      post({type: 'mode', mode});
    },
    live() {
      post({type: 'mode', mode: 'LIVE_STREAMING'});
    },
    seek(timestamp_ns: string) {
      post({type: 'seek', timestamp_ns});
    },
    step(dt_ns: string) {
      post({type: 'step', dt_ns});
    },
    async importLog(file: File) {
      const buffer = await file.arrayBuffer();
      client.restart();
      post({type: 'import', buffer}, [buffer]);
    },
    exportLog() {
      post({type: 'export'});
    },
    rpc(method: string, params: unknown) {
      return client.rpc(method, params);
    }
  };

  const receive = (data: WorkerMessage) => {
    if (data.type === 'updates') {
      lastUpdate = now();
      for (const message of data.messages) receive(message);
      studio.playback = {...studio.playback, updates: {
        dropped: data.dropped, coalesced: data.coalesced
      }};
      studio.notify();
      defaultScheduler(client.acknowledger());
      return;
    }
    if (data.type === 'frames') {
      store.ingest(data.frames, data.reset);
      studio.notify();
    } else if (data.type === 'playback') {
      store.setCursor(data.state.cursor_ns);
      studio.playback = data.state;
      studio.notify();
    } else if (data.type === 'system') {
      studio.system = {
        graph: data.graph ?? null,
        stale: data.stale ?? true,
        rates: data.rates ?? {},
        error: data.error ?? '',
        declared: data.declared ?? null,
        declaredError: data.declaredError ?? ''
      };
      store.pushSystem(studio.system.graph);
      studio.notify();
    } else if (data.type === 'manifest') {
      const manifest = data.manifest ?? null;
      studio.manifest = manifest;
      studio.manifestError = data.error ?? '';
      if (manifest) store.pushManifest(manifest);
      studio.notify();
    } else if (data.type === 'export') {
      const href = URL.createObjectURL(new Blob([data.buffer]));
      const a = document.createElement('a');
      a.href = href;
      a.download = 'telemetry.tlog';
      a.click();
      setTimeout(() => URL.revokeObjectURL(href), 1000);
    }
  };

  return studio;
}

export function createStaticStudio(input: {
  frames?: Snapshot[];
  playback?: Partial<PlaybackState>;
  system?: Partial<SystemState>;
  events?: RobotEvent[];
  manifest?: SessionManifest|null;
  manifestError?: string;
} = {}): Studio {
  const scheduler = (cb: () => void) => queueMicrotask(cb);
  const store = new TelemetryStore(scheduler);
  if (input.frames?.length) store.ingest(input.frames, true);
  if (input.events) store.setEvents(input.events);

  const frames = input.frames ?? [];
  const playback: PlaybackState = {
    mode: input.playback?.mode ?? 'PAUSED',
    // An explicit null clock is "no current time known"; only an absent one
    // falls back to the cursor.
    clock_ns: input.playback?.clock_ns !== undefined ? input.playback.clock_ns :
        input.playback?.cursor_ns ?? frames.at(-1)?.timestamp_ns ?? null,
    cursor_ns: input.playback?.cursor_ns ?? frames.at(-1)?.timestamp_ns ?? null,
    bounds: input.playback?.bounds ?? boundsOf(frames),
    stats: input.playback?.stats ?? {skipped: 0, late: 0, invalid: 0},
    status: input.playback?.status ?? 'Static studio',
    connection: input.playback?.connection ?? 'replay'
  };
  const system: SystemState = {
    graph: input.system?.graph ?? null,
    stale: input.system?.stale ?? false,
    rates: input.system?.rates ?? {},
    error: input.system?.error ?? '',
    declared: input.system?.declared ?? null,
    declaredError: input.system?.declaredError ?? ''
  };

  store.setCursor(playback.cursor_ns);
  if (system.graph) store.pushSystem(system.graph);

  const actions: StudioActions = {
    connect() {},
    demo() {},
    setMode() {},
    live() {},
    seek() {},
    step() {},
    async importLog() {},
    exportLog() {},
    rpc() {
      return Promise.reject(new Error('No worker: RPC is unavailable in a static studio'));
    }
  };

  return new StudioImpl(store, actions, scheduler, playback, system,
                        input.manifest ?? null, input.manifestError ?? '');
}

const StudioContext = createContext<Studio|null>(null);

export function StudioProvider({studio, children}: {studio: Studio; children: ReactNode}) {
  return createElement(StudioContext.Provider, {value: studio}, children);
}

export function useStudio(): Studio {
  const studio = useContext(StudioContext);
  if (!studio) throw new Error('useStudio must be used within a StudioProvider');
  return studio;
}
