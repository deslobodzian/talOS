// Plot panel tests: the unit-axis grouping policy, the config parser (including
// the `events` field and backward compatibility with the older two-field
// shape), and an SSR smoke render for empty and populated studios.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {TelemetryDecoder} from '../src/core';
import type {Snapshot} from '../src/core';
import {bucketSeries} from '../src/data/recorded';
import {TelemetryStore} from '../src/data/store';
import {createStaticStudio, type Studio, StudioProvider} from '../src/data/studio';
import {demoPacket} from '../src/demo';
import {assignYAxes, groupByUnit, type PlotChannel} from '../src/panels/plot/axes';
import {cursorTimestampNs, plotColumn, plotData, plotPanel} from '../src/panels/plot';

const pc = (name: string, unit = '', labels: string[] = []): PlotChannel =>
    ({name, unit, labels});

function moduleFrames(): Snapshot[] {
  const channels: Record<string, number> = {};
  const units: Record<string, string> = {};
  for (const m of [0, 1, 2, 3]) {
    for (const [leaf, unit] of [['angle_rad', 'rad'], ['distance_m', 'm']]) {
      const name = `drivetrain/state/module${m}/${leaf}`;
      channels[name] = 0;
      units[name] = unit;
    }
  }
  return [{
    stream_id: 's', sequence_id: '0', timestamp_ns: '0', dictionary_id: 0,
    chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
    channels, units, labels: {}
  }];
}

function populatedStudio(): Studio {
  const decoder = new TelemetryDecoder();
  const frames = [0n, 1n, 2n].map(n => decoder.decode(demoPacket(n))!);
  return createStaticStudio({
    frames,
    playback: {
      mode: 'PAUSED',
      cursor_ns: frames[2].timestamp_ns,
      bounds: {start: frames[0].timestamp_ns, end: frames[2].timestamp_ns},
      stats: {skipped: 0, late: 0, invalid: 0},
      status: 'Demo',
      connection: 'demo'
    }
  });
}

const renderPlot = (studio: Studio): string => renderToStaticMarkup(
    <StudioProvider studio={studio}>
      <plotPanel.component config={plotPanel.defaultConfig} setConfig={() => {}}
                           instanceId="test"/>
    </StudioProvider>);

test('groupByUnit keeps first-seen order and sorts unitless last', () => {
  assert.deepEqual(
      groupByUnit(['a', 'b', 'c'], name =>
          name === 'a' ? 'm/s' : name === 'b' ? 'rad' : ''),
      [
        {unit: 'm/s', channels: ['a']},
        {unit: 'rad', channels: ['b']},
        {unit: '', channels: ['c']}
      ]);
});

test('assignYAxes puts one unit per side, enums left with their labels as ticks', () => {
  assert.deepEqual(assignYAxes([]), []);
  assert.deepEqual(assignYAxes([pc('vx', 'm/s'), pc('vy', 'm/s')]), [
    {scale: 'y', side: 'left', label: 'm/s', units: ['m/s'], labels: null,
     channels: ['vx', 'vy']}
  ]);
  assert.deepEqual(assignYAxes([pc('vx', 'm/s'), pc('yaw', 'rad')]), [
    {scale: 'y', side: 'left', label: 'm/s', units: ['m/s'], labels: null,
     channels: ['vx']},
    {scale: 'y2', side: 'right', label: 'rad', units: ['rad'], labels: null,
     channels: ['yaw']}
  ]);
  assert.deepEqual(
      assignYAxes([pc('mode', '', ['idle', 'spin', 'feed']), pc('vx', 'm/s')]), [
        {scale: 'y', side: 'left', label: '', units: [''],
         labels: ['idle', 'spin', 'feed'], channels: ['mode']},
        {scale: 'y2', side: 'right', label: 'm/s', units: ['m/s'], labels: null,
         channels: ['vx']}
      ]);
});

test('assignYAxes never applies one enum vocabulary to a different enum', () => {
  for (const labels of [['off', 'on'], ['idle', 'run', 'fault'], ['run', 'idle']]) {
    const axes = assignYAxes([
      pc('mode', '', ['idle', 'run']), pc('other', '', labels), pc('vx', 'm/s')
    ]);
    assert.ok(axes.every(axis => axis.labels === null));
    assert.deepEqual(axes.flatMap(axis => axis.channels).sort(),
                     ['mode', 'other', 'vx']);
  }
  const shared = assignYAxes([
    pc('a', '', ['idle', 'run']), pc('b', '', ['idle', 'run'])
  ]);
  assert.deepEqual(shared[0].labels, ['idle', 'run']);
  assert.deepEqual(shared[0].channels, ['a', 'b']);
});

test('assignYAxes keeps at most two axes for three numeric units', () => {
  const axes = assignYAxes([pc('vx', 'm/s'), pc('yaw', 'rad'), pc('rps', 'rps')]);
  assert.equal(axes.length, 2);
  assert.equal(axes[0].channels.join(','), 'vx');
  assert.equal(axes[1].channels.join(','), 'yaw,rps');
});

test('plot config parsing accepts the events field and stays backward compatible',
     () => {
       assert.deepEqual(
           plotPanel.parseConfig({channels: ['a', 'b'], windowSeconds: 10}),
           {channels: ['a', 'b'], windowSeconds: 10, events: true});
       assert.deepEqual(
           plotPanel.parseConfig({channels: ['a'], windowSeconds: 10, events: false}),
           {channels: ['a'], windowSeconds: 10, events: false});
       assert.deepEqual(plotPanel.parseConfig(null),
                        {channels: [], windowSeconds: null, events: true});
       assert.deepEqual(plotPanel.parseConfig('nope'),
                        {channels: [], windowSeconds: null, events: true});
       assert.deepEqual(
           plotPanel.parseConfig({channels: ['a', 2, null], windowSeconds: 'x'}),
           {channels: ['a'], windowSeconds: null, events: true});
     });


test('the plot drawer keeps module channels distinct', () => {
  const studio = createStaticStudio({frames: moduleFrames()});
  const html = renderPlot(studio);
  assert.ok(html.includes('module0/angle_rad'));
  assert.ok(html.includes('module1/angle_rad'));
  assert.ok(html.includes('module3/distance_m'));
  // A bare leaf would collapse all four modules into the same label.
  assert.ok(!html.includes('>angle_rad<'), 'module channels must not be bare leaves');
});


test('the plot window selector retains custom configured durations', () => {
  for (const windowSeconds of [5, 45, 2.5]) {
    const html = renderToStaticMarkup(
        <StudioProvider studio={populatedStudio()}>
          <plotPanel.component
              config={{channels: [], windowSeconds, events: true}}
              setConfig={() => {}} instanceId="test"/>
        </StudioProvider>);
    assert.ok(html.includes(
        `<option value="${windowSeconds}" selected="">${windowSeconds} s</option>`),
        html);
  }
});

test('plotColumn and plotData convert NaN/Infinity gaps to null and keep recorded alignment rows undefined',
     () => {
       assert.deepEqual(
           plotColumn(new Float64Array([1, NaN, Infinity, -Infinity, 0])),
           [1, null, null, null, 0]);

       const makeSnap = (stream: string, seq: number, ts: bigint,
                         channels: Record<string, number>): Snapshot => ({
         stream_id: stream,
         sequence_id: String(seq),
         timestamp_ns: String(ts),
         dictionary_id: 1,
         chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
         channels,
         units: {},
         labels: {}
       });

       // Live store with: later-declared channel `b` (NaN-backfilled at index 0),
       // a stream restart break between t=1s and t=2s, and non-finite readings.
       const store = new TelemetryStore(cb => cb());
       store.ingest([
         makeSnap('s1', 0, 1_000_000_000n, {a: 1}),
         makeSnap('s1', 1, 2_000_000_000n, {a: Infinity, b: 10}),
         makeSnap('s2', 0, 4_000_000_000n, {a: 3, b: -Infinity})
       ], true);
       store.setCursor('4000000000');
       const live = store.series(['a', 'b'], 'all');
       const [times, colA, colB] = plotData(live, ['a', 'b']);
       assert.deepEqual(times, [0, 1, 2, 3]);
       assert.ok(times.every(Number.isFinite));
       assert.deepEqual(colA, [1, null, null, 3]);
       assert.deepEqual(colB, [null, 10, null, null]);

       // Recorded multi-channel series: alignment rows (count === 0) map to
       // undefined so uPlot skips them in getMinMax without breaking lines,
       // while an actual non-finite bucket reading (count > 0) maps to null.
       const recorded = bucketSeries(
           ['a', 'b'],
           {
             buckets: 2,
             channels: [
               {
                 t_ns: new Float64Array([1e9, 3e9]),
                 min: new Float64Array([5, NaN]),
                 max: new Float64Array([5, NaN]),
                 last: new Float64Array([5, NaN]),
                 count: new Uint32Array([1, 1])
               },
               {
                 t_ns: new Float64Array([2e9, 3e9]),
                 min: new Float64Array([9, 8]),
                 max: new Float64Array([9, 8]),
                 last: new Float64Array([9, 8]),
                 count: new Uint32Array([1, 1])
               }
             ]
           },
           '0', 0n, 4_000_000_000n);
       const [recTimes, recA, recB] = plotData(recorded, ['a', 'b']);
       assert.deepEqual(recTimes, [1, 2, 3]);
       assert.deepEqual(recA, [5, undefined, null]);
       assert.deepEqual(recB, [undefined, 9, 8]);
     });

// REQ-OPS-23: plot double-click and cursor delta must not throw RangeError
// when the cursor index has no finite x-axis sample.
test('cursorTimestampNs returns null for missing or non-finite samples and exact ns otherwise', () => {
  const origin = 1_000_000_000n;
  assert.equal(cursorTimestampNs([0, 1.25], 1, origin), 2_250_000_000n);
  assert.equal(cursorTimestampNs([], 0, origin), null);
  assert.equal(cursorTimestampNs([0], 5, origin), null);
  assert.equal(cursorTimestampNs([Number.NaN, Infinity, -Infinity, 1e308], 0, origin), null);
  assert.equal(cursorTimestampNs([Number.NaN, Infinity, -Infinity, 1e308], 1, origin), null);
  assert.equal(cursorTimestampNs([Number.NaN, Infinity, -Infinity, 1e308], 2, origin), null);
  assert.equal(cursorTimestampNs([Number.NaN, Infinity, -Infinity, 1e308], 3, origin), null);
  assert.equal(cursorTimestampNs([0], null, origin), null);
  assert.equal(cursorTimestampNs([0], 0, null), null);
});
