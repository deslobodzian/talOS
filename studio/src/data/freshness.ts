import type {PlaybackState} from './types';

// A sample exactly one second old is stale. Connection state never enters
// this decision; paused/replay clocks are the playback cursor, not wall time.
export const STALE_AFTER_NS = 1000000000n;
export function sampleStale(sample_ns: string|null, clock_ns: string|null) {
  return sample_ns === null || clock_ns === null ||
      BigInt(clock_ns) - BigInt(sample_ns) >= STALE_AFTER_NS;
}

// The session time every reading's age is measured against. `clock_ns: null`
// is an explicit "no current time is known" -- the newest live sample's age is
// unknown (the bridge's connect-time dictionary) -- so no reading is current.
// Absent (a static studio that never set one) falls back to the cursor.
export function readingClock(
    playback: Pick<PlaybackState, 'clock_ns'|'cursor_ns'>): string|null {
  return playback.clock_ns === undefined ? playback.cursor_ns : playback.clock_ns;
}
