// Pure staleness derivation for the inspector: the age of a channel's newest
// reading at or before the playback cursor, measured against the reading
// clock (the live session time, or the cursor when no live clock is set). Kept
// pure so the unit tests can cover it without a worker or a browser.

import {readingClock, sampleStale} from '../../data/freshness';
import type {TelemetryStore} from '../../data/store';
import type {PlaybackState} from '../../data/types';

// `age_ns` is a decimal string so a 64-bit age stays exact; '0' means the
// reading is at the clock itself (a reading ahead of it clamps to '0'). It is
// null when no current time is known, which makes the reading stale.
export type ChannelAge = {age_ns: string|null; stale: boolean};

// Null when the channel has no reading at or before the cursor.
export function channelAge(
    store: Pick<TelemetryStore, 'channelSample'>, name: string,
    playback: Pick<PlaybackState, 'clock_ns'|'cursor_ns'>): ChannelAge|null {
  const sample = store.channelSample(name, playback.cursor_ns);
  if (!sample) return null;
  const clock = readingClock(playback);
  if (clock === null) return {age_ns: null, stale: true};
  const age = BigInt(clock) - BigInt(sample.timestamp_ns);
  return {
    age_ns: age > 0n ? age.toString() : '0',
    stale: sampleStale(sample.timestamp_ns, clock)
  };
}
