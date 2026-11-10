// Inspector: the per-channel age derivation.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import type {Snapshot} from '../src/core';
import {TelemetryStore} from '../src/data/store';
import {channelAge} from '../src/panels/inspector/staleness';

function frame(t_ns: bigint, channels: Record<string, number>): Snapshot {
  return {
    stream_id: '1', sequence_id: String(t_ns), timestamp_ns: String(t_ns),
    dictionary_id: 1, chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
    channels, units: {a: ''}, labels: {}
  };
}

test('channelAge measures against the live clock, not the cursor', () => {
  const store = new TelemetryStore(() => {});
  store.ingest([frame(0n, {a: 1}), frame(20_000_000n, {a: 2})], true);
  const last = '20000000';
  // Telemetry stopped: the cursor holds the last frame while the live clock
  // runs on. The channel's reading at the cursor is 2 s old, and stale.
  assert.deepEqual(channelAge(store, 'a', {cursor_ns: last, clock_ns: '2020000000'}),
                   {age_ns: '2000000000', stale: true});
  // A live clock at the reading is fresh.
  assert.deepEqual(channelAge(store, 'a', {cursor_ns: last, clock_ns: last}),
                   {age_ns: '0', stale: false});
  // No current time known: the age is unknown, so the reading is not current.
  assert.deepEqual(channelAge(store, 'a', {cursor_ns: last, clock_ns: null}),
                   {age_ns: null, stale: true});
  // A channel with no reading at the cursor has no age at all.
  assert.equal(channelAge(store, 'missing', {cursor_ns: last, clock_ns: last}), null);
});
