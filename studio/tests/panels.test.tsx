import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {createStaticStudio} from '../src/data/studio';
import type {Series} from '../src/data/types';
import type {SystemGraph} from '../src/system';
import {bitHistory} from '../src/panels/hardware/logic';
import {windowDelta, phaseFromSeries} from '../src/panels/robot';
import {TopicTable} from '../src/panels/system/TopicTable';
import {TimeSeriesChart} from '../src/ui/TimeSeriesChart';
import {Sparkline} from '../src/ui/Sparkline';


const series = (values: number[]): Series => ({
  t0_ns: '0',
  times: Float64Array.from(values, (_, i) => i),
  values: {c: Float64Array.from(values)}
});

test('robot activity sums increments across counter resets and gaps', () => {
  assert.deepEqual(windowDelta(series([100, 104, NaN, 2, 5, 0, 3]), 'c', 10),
                   {current: 3, delta: 12});
  assert.deepEqual(windowDelta(series([10, 0]), 'c', 10),
                   {current: 0, delta: 0});
  assert.deepEqual(windowDelta(series([NaN, 5]), 'c', 10),
                   {current: 5, delta: 0});
  assert.deepEqual(windowDelta(series([0, 5, 1, 3]), 'c', 1),
                   {current: 3, delta: 2});
});

test('robot phase duration starts at a finite sample of the current phase', () => {
  assert.deepEqual(phaseFromSeries(series([NaN, NaN, 1, 1]), 'c', ['idle', 'run']),
                   {current: 'run', previous: null, seconds: 1});
  assert.deepEqual(phaseFromSeries(series([0, NaN, NaN, 1, 1]), 'c', ['idle', 'run']),
                   {current: 'run', previous: 'idle', seconds: 1});
  assert.deepEqual(phaseFromSeries(series([NaN, 1, NaN, 1]), 'c', ['idle', 'run']),
                   {current: 'run', previous: null, seconds: 2});
});

test('hardware retains faults active at the first finite sample after clearing', () => {
  assert.deepEqual(bitHistory([0, 1, 2], [NaN, 1, 0], 1),
                   [{bit: 0, set: false, lastSetAt: null, setCount: 1}]);
});

test('topic observers make their topic searchable', () => {
  const graph: SystemGraph = {
    version: 2, wall_ns: '0',
    registry: {available: true, node_capacity: 32, source_capacity: 64,
               liveness_timeout_ns: '5000000000'},
    bridge: {topic: '/talos/telemetry', clients: 0, frames_forwarded: '0',
             invalid: '0', ws_drops: '0'},
    nodes: [],
    topics: [{name: '/pose', message_bytes: 16, publishers: ['estimator'],
              subscribers: [], observers: ['telemetry'], published: '1',
              received: '0', dropped: '0'}]
  };
  const html = renderToStaticMarkup(
      <TopicTable graph={graph} rates={{}} filter="TELEMETRY" onPickNode={() => {}}/>);
  assert.ok(html.includes('/pose'), html);
});

test('time series renders isolated finite samples as dots without bridging gaps', () => {
  for (const values of [[5], [NaN, 5, NaN], [1, NaN, 2]]) {
    const html = renderToStaticMarkup(
        <TimeSeriesChart lines={[{name: 'v', color: '#fff', values}]}/>);
    assert.ok(!html.includes('no data'), html);
    assert.equal((html.match(/ l 0.01,0/g) ?? []).length,
                 values.filter(Number.isFinite).length, html);
  }
  const sparse = Array<number>(100).fill(NaN);
  sparse[50] = 5;
  const decimated = renderToStaticMarkup(
      <TimeSeriesChart width={10} lines={[{name: 'v', color: '#fff', values: sparse}]}/>);
  assert.ok(decimated.includes(' l 0.01,0'), decimated);
});

test('sparklines render isolated samples with finite visible segments', () => {
  for (const values of [[5], [NaN, 5, NaN], [1, NaN, 2]]) {
    const html = renderToStaticMarkup(<Sparkline values={values}/>);
    const runs = [...html.matchAll(/points="([^"]*)"/g)];
    assert.equal(runs.length, values.filter(Number.isFinite).length, html);
    for (const run of runs) {
      const [start, end] = run[1].split(' ').map(point => point.split(',').map(Number));
      assert.ok(start.every(Number.isFinite) && end.every(Number.isFinite), html);
      assert.ok(end[0] > start[0], html);
      assert.equal(end[1], start[1], html);
    }
  }
});
