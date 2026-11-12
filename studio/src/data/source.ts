// What a panel reads through. The hooks (hooks.ts) read the Studio's current
// DataSource: `LiveSource` is the worker-fed store and playback (a live
// stream, demo or imported log), `RecordedSource` (recorded.ts) a session
// served by the review server. Every getter is synchronous and returns a
// stable reference until the source notifies, as useSyncExternalStore needs;
// a recorded source fetches what a getter misses and notifies on arrival.

import {poseValid, type Pose, type Snapshot} from '../core';
import type {TelemetryStore} from './store';
import type {
  ChannelInfo,
  ModeBand,
  PlaybackState,
  RobotEvent,
  Series,
  SeriesWindow
} from './types';

export type SourceKind = 'live'|'recorded';

export type ChannelSample = {value: number; timestamp_ns: string};

export interface DataSource {
  readonly kind: SourceKind;
  // Bumped whenever data a getter returns changes (not the cursor).
  readonly version: number;
  readonly playback: PlaybackState;
  dictionary(): ChannelInfo[];
  channelInfo(name: string): ChannelInfo|undefined;
  snapshotAtCursor(): Snapshot|null;
  channelValue(name: string): number|undefined;
  // The channel's newest reading at or before the cursor, with its own time.
  channelSample(name: string, cursor_ns?: string|null): ChannelSample|null;
  // History ending at the cursor. `points` is the plot's pixel width: a
  // recorded source decimates to it, the live store ignores it.
  series(channels: string[], window: SeriesWindow, cursor_ns: string|null,
         points?: number): Series;
  // The chassis pose at the cursor and the time it was sampled, or null when
  // there is no valid pose.
  chassis(): {pose: Pose; sample_ns: string}|null;
  events(): RobotEvent[];
  modeBands(): ModeBand[];
}

export class LiveSource implements DataSource {
  readonly kind = 'live';
  constructor(private readonly store: TelemetryStore,
              private readonly currentPlayback: () => PlaybackState) {}

  get version() {
    return this.store.version;
  }
  get playback() {
    return this.currentPlayback();
  }
  dictionary() {
    return this.store.dictionary();
  }
  channelInfo(name: string) {
    return this.store.channelInfo(name);
  }
  snapshotAtCursor() {
    return this.store.snapshotAtCursor();
  }
  channelValue(name: string) {
    return this.store.channelValue(name);
  }
  channelSample(name: string, cursor_ns?: string|null) {
    return this.store.channelSample(name, cursor_ns);
  }
  series(channels: string[], window: SeriesWindow, cursor_ns: string|null) {
    return this.store.series(channels, window, cursor_ns);
  }
  // The frame-level pose (`[studio] pose_topic`), as old as its frame.
  chassis() {
    const snapshot = this.store.snapshotAtCursor();
    if (!snapshot || !poseValid(snapshot)) return null;
    return {pose: snapshot.chassis, sample_ns: snapshot.timestamp_ns};
  }
  events() {
    return this.store.events();
  }
  modeBands() {
    return this.store.modeBands();
  }
}
