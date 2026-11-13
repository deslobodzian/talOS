// Inspector: the per-channel age derivation, and readings shown as they are.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import type {Snapshot} from '../src/core';
import {createStaticStudio, StudioProvider} from '../src/data/studio';
import {TelemetryStore} from '../src/data/store';
import {ChannelKind} from '../src/generated/talos/telemetry/channel-kind';
import {inspectorPanel} from '../src/panels/inspector';
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

// A NaN or infinite reading is a reading and an integer past 2^53 is exact:
// the store keeps both through its columns and snapshot reconstruction, a
// channel reading Infinity every tick is current, not stale, and the
// Inspector shows NaN, Infinity and the exact integer. Absence (the topic was
// not published) stays absence.
test('NaN, infinities and exact integers survive the store into the Inspector', () => {
  const kinds = {
    'a/state/nan': ChannelKind.Double, 'a/state/inf': ChannelKind.Double,
    'a/state/boot': ChannelKind.UInt, 'a/state/x': ChannelKind.Double
  };
  const tick = (t_ns: bigint, x?: number): Snapshot => ({
    stream_id: '1', sequence_id: String(t_ns), timestamp_ns: String(t_ns),
    dictionary_id: 1, chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
    channels: {
      'a/state/nan': NaN, 'a/state/inf': Infinity,
      'a/state/boot': Number(2n ** 53n + 1n), ...(x === undefined ? {} : {'a/state/x': x})
    },
    exact: {'a/state/boot': String(2n ** 53n + 1n)},
    units: {'a/state/nan': 'm', 'a/state/inf': 'm', 'a/state/boot': 'count', 'a/state/x': 'm'},
    labels: {}, kinds
  });
  const last = 1_010_000_000n;
  const studio = createStaticStudio({
    frames: [tick(1_000_000_000n, 2), tick(last)],
    playback: {mode: 'PAUSED', cursor_ns: String(last)}
  });
  const snapshot = studio.store.snapshotAt(String(last))!;
  assert.ok(Object.is(snapshot.channels['a/state/nan'], NaN));
  assert.equal(snapshot.channels['a/state/inf'], Infinity);
  assert.equal(snapshot.exact?.['a/state/boot'], '9007199254740993');
  assert.ok(!('a/state/x' in snapshot.channels));
  assert.deepEqual(channelAge(studio.store, 'a/state/inf',
                              {cursor_ns: String(last), clock_ns: String(last)}),
                   {age_ns: '0', stale: false});

  const html = renderToStaticMarkup(
      <StudioProvider studio={studio}>
        <inspectorPanel.component config={inspectorPanel.defaultConfig}
                                  setConfig={() => {}} instanceId="test"/>
      </StudioProvider>);
  assert.ok(html.includes('<b>NaN</b>'), html);
  assert.ok(html.includes('<b>Infinity</b>'));
  assert.ok(html.includes('<b>9,007,199,254,740,993</b>'));
});
