// A sample exactly one second old is stale. Connection state never enters
// this decision; paused/replay clocks are the playback cursor, not wall time.
export const STALE_AFTER_NS = 1000000000n;
export function sampleStale(sample_ns: string|null, clock_ns: string|null) {
  return sample_ns === null || clock_ns === null ||
      BigInt(clock_ns) - BigInt(sample_ns) >= STALE_AFTER_NS;
}
