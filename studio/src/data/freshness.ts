import type {SystemNode} from '../system';
import type {PlaybackState, SystemState} from './types';

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

// The bridge refreshes the system graph every 250 ms. Four missed refreshes
// is not jitter: a graph this old, or one held while the socket is down, no
// longer says what is running now.
export const GRAPH_STALE_AFTER_MS = 1000;
export function graphStale(received_ms: number|null, now_ms: number,
                           connected: boolean): boolean {
  return !connected || received_ms === null ||
      now_ms - received_ms >= GRAPH_STALE_AFTER_MS;
}

// A node's liveness as far as a current graph can say. A stale graph's
// `alive` is last-known evidence, not a claim about now: 'unknown'.
export type Liveness = 'live'|'stopped'|'unknown';
export function nodeLiveness(node: Pick<SystemNode, 'alive'>,
                             system: Pick<SystemState, 'stale'>): Liveness {
  return system.stale ? 'unknown' : node.alive ? 'live' : 'stopped';
}
// How many nodes are live, or null when the graph cannot say.
export function liveNodeCount(system: Pick<SystemState, 'graph'|'stale'>): number|null {
  return system.graph && !system.stale ?
      system.graph.nodes.filter(n => n.alive).length : null;
}
