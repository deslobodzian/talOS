// React bindings over the Studio object. Every hook reads through
// `useSyncExternalStore` with a narrow getter, so a panel re-renders only when
// its own data changes: `useChannelValue` returns a primitive, `useDictionary`
// and friends return a stable array reference replaced only on change, and
// `useSeries` memoises on the source version plus its arguments. The third
// argument is the server snapshot, so the same hooks render under
// react-dom/server in tests. Data hooks read `studio.source` (source.ts), so
// the same panel renders a live stream or a recorded session.

import {useMemo, useSyncExternalStore} from 'react';

import type {Pose, Snapshot} from '../core';
import type {AlertLatch} from './alerts';
import type {RecordedSource} from './recorded';
import type {DataSource, SourceKind} from './source';
import {useStudio} from './studio';
import type {RobotStatus} from './robot';
import {robotStatus} from './robot';
import {readingClock, sampleStale} from './freshness';
import type {SessionManifest} from './manifest';
import type {
  ChannelInfo,
  ModeBand,
  PlaybackState,
  RobotEvent,
  Series,
  SeriesWindow,
  StudioActions,
  SystemState
} from './types';

export function useSource(): DataSource {
  const studio = useStudio();
  return useSyncExternalStore(studio.subscribe, () => studio.source, () => studio.source);
}

export function useSourceKind(): SourceKind {
  return useSource().kind;
}

export function useRecorded(): RecordedSource|null {
  const studio = useStudio();
  return useSyncExternalStore(studio.subscribe, () => studio.recorded, () => studio.recorded);
}

export function useSourceVersion(): number {
  const studio = useStudio();
  return useSyncExternalStore(
      studio.subscribe, () => studio.source.version, () => studio.source.version);
}

export function usePlayback(): PlaybackState {
  const studio = useStudio();
  return useSyncExternalStore(
      studio.subscribe, () => studio.source.playback, () => studio.source.playback);
}

export function useSystem(): SystemState {
  const studio = useStudio();
  return useSyncExternalStore(studio.subscribe, () => studio.system, () => studio.system);
}

// While a recorded session is open, transport drives its playhead, and going
// live (or connecting) closes it.
export function useActions(): StudioActions {
  const studio = useStudio();
  const recorded = useRecorded();
  return useMemo(() => recorded ? {
    ...studio.actions,
    connect(url: string) {
      studio.closeRecorded();
      studio.actions.connect(url);
    },
    demo() {
      studio.closeRecorded();
      studio.actions.demo();
    },
    setMode: mode => recorded.setMode(mode),
    live: () => studio.closeRecorded(),
    seek: t => recorded.seek(t),
    step: dt => recorded.step(dt)
  } : studio.actions, [studio, studio.actions, recorded]);
}

export function useSnapshot(): Snapshot|null {
  const studio = useStudio();
  return useSyncExternalStore(
      studio.subscribe, () => studio.source.snapshotAtCursor(),
      () => studio.source.snapshotAtCursor());
}

export function useChannelValue(name: string): number|undefined {
  const studio = useStudio();
  return useSyncExternalStore(
      studio.subscribe, () => studio.source.channelValue(name),
      () => studio.source.channelValue(name));
}

export function useChannelStale(name: string): boolean {
  const studio = useStudio();
  const read = () => {
    const source = studio.source;
    return sampleStale(
        source.channelSample(name, source.playback.cursor_ns)?.timestamp_ns ?? null,
        readingClock(source.playback));
  };
  return useSyncExternalStore(studio.subscribe, read, read);
}

export function useChannelInfo(name: string): ChannelInfo|undefined {
  const studio = useStudio();
  return useSyncExternalStore(
      studio.subscribe, () => studio.source.channelInfo(name),
      () => studio.source.channelInfo(name));
}

export function useDictionary(): ChannelInfo[] {
  const studio = useStudio();
  return useSyncExternalStore(
      studio.subscribe, () => studio.source.dictionary(), () => studio.source.dictionary());
}

export function useEvents(): RobotEvent[] {
  const studio = useStudio();
  return useSyncExternalStore(
      studio.subscribe, () => studio.source.events(), () => studio.source.events());
}

export function useModeBands(): ModeBand[] {
  const studio = useStudio();
  return useSyncExternalStore(
      studio.subscribe, () => studio.source.modeBands(), () => studio.source.modeBands());
}

// Every alert raised since the current match began (data/alerts.ts); null for
// a source that cannot know it.
export function useAlertLatch(): AlertLatch|null {
  const studio = useStudio();
  return useSyncExternalStore(
      studio.subscribe, () => studio.source.alertLatch(),
      () => studio.source.alertLatch());
}

export function useManifest(): {manifest: SessionManifest|null; error: string} {
  const studio = useStudio();
  const manifest = useSyncExternalStore(
      studio.subscribe, () => studio.manifest, () => studio.manifest);
  const error = useSyncExternalStore(
      studio.subscribe, () => studio.manifestError, () => studio.manifestError);
  return useMemo(() => ({manifest, error}), [manifest, error]);
}

export function useRobotStatus(): RobotStatus {
  const snapshot = useSnapshot();
  const system = useSystem();
  return useMemo(() => robotStatus(snapshot, system), [snapshot, system]);
}

// `points` is the consumer's width in pixels, the resolution a recorded
// source decimates to; the live store returns every retained sample.
export function useSeries(channels: string[], window: SeriesWindow,
                          points?: number): Series {
  const source = useSource();
  const playback = usePlayback();
  const version = useSourceVersion();
  const key = channels.join('\u0000') + '|' +
      (typeof window === 'string' ? window : `s:${window.seconds}`);
  return useMemo(
      () => source.series(channels, window, playback.cursor_ns, points),
      [source, version, key, playback.cursor_ns, points]);
}

// The chassis pose at the cursor, and whether it is stale by the reading
// clock: the frame-level pose live, the estimator's pose in a recording.
export function useChassis(): {pose: Pose; stale: boolean}|null {
  const source = useSource();
  const playback = usePlayback();
  const snapshot = useSnapshot();
  return useMemo(() => {
    const reading = source.chassis();
    return reading && {
      pose: reading.pose,
      stale: sampleStale(reading.sample_ns, readingClock(playback))
    };
  }, [source, snapshot, playback]);
}
