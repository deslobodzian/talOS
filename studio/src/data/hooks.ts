// React bindings over the Studio object. Every hook reads through
// `useSyncExternalStore` with a narrow getter, so a panel re-renders only when
// its own data changes: `useChannelValue` returns a primitive, `useDictionary`
// and friends return a stable array reference replaced only on change, and
// `useSeries` memoises on the store version plus its arguments. The third
// argument is the server snapshot, so the same hooks render under
// react-dom/server in tests.

import {useMemo, useSyncExternalStore} from 'react';

import type {Snapshot} from '../core';
import {useStudio} from './studio';
import type {RobotStatus} from './robot';
import {robotStatus} from './robot';
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

export function usePlayback(): PlaybackState {
  const studio = useStudio();
  return useSyncExternalStore(studio.subscribe, () => studio.playback, () => studio.playback);
}

export function useSystem(): SystemState {
  const studio = useStudio();
  return useSyncExternalStore(studio.subscribe, () => studio.system, () => studio.system);
}

export function useActions(): StudioActions {
  const studio = useStudio();
  return studio.actions;
}

export function useSnapshot(): Snapshot|null {
  const studio = useStudio();
  return useSyncExternalStore(
      studio.subscribe, () => studio.store.snapshotAtCursor(),
      () => studio.store.snapshotAtCursor());
}

export function useChannelValue(name: string): number|undefined {
  const studio = useStudio();
  return useSyncExternalStore(
      studio.subscribe, () => studio.store.channelValue(name),
      () => studio.store.channelValue(name));
}

export function useChannelInfo(name: string): ChannelInfo|undefined {
  const studio = useStudio();
  return useSyncExternalStore(
      studio.subscribe, () => studio.store.channelInfo(name),
      () => studio.store.channelInfo(name));
}

export function useDictionary(): ChannelInfo[] {
  const studio = useStudio();
  return useSyncExternalStore(
      studio.subscribe, () => studio.store.dictionary(), () => studio.store.dictionary());
}

export function useEvents(): RobotEvent[] {
  const studio = useStudio();
  return useSyncExternalStore(studio.subscribe, () => studio.store.events(), () => studio.store.events());
}

export function useModeBands(): ModeBand[] {
  const studio = useStudio();
  return useSyncExternalStore(
      studio.subscribe, () => studio.store.modeBands(), () => studio.store.modeBands());
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
  const studio = useStudio();
  const snapshot = useSnapshot();
  const system = useSystem();
  return useMemo(() => robotStatus(snapshot, system), [snapshot, system]);
}

export function useSeries(channels: string[], window: SeriesWindow): Series {
  const studio = useStudio();
  const playback = usePlayback();
  const version = useSyncExternalStore(studio.subscribe, () => studio.store.version, () => studio.store.version);
  const key = channels.join('\u0000') + '|' +
      (typeof window === 'string' ? window : `s:${window.seconds}`);
  return useMemo(
      () => studio.store.series(channels, window, playback.cursor_ns),
      [studio.store, version, key, playback.cursor_ns]);
}
