// Panel behaviour tests for the views that used to live untested inside the
// old main.tsx: the plot config parser, the field pose rendering, the channel
// inspector, and the SSR placeholder for the 3D viewport.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {decode} from '../src/core';
import {createStaticStudio, type Studio, StudioProvider} from '../src/data/studio';
import {demoPacket} from '../src/demo';
import {fieldPanel} from '../src/panels/field';
import {inspectorPanel} from '../src/panels/inspector';
import {plotPanel} from '../src/panels/plot';
import {pose3dPanel} from '../src/panels/pose3d';
import type {AnyPanelDefinition} from '../src/panels/types';
import type {Series} from '../src/data/types';
import type {SystemGraph} from '../src/system';
import {bitHistory} from '../src/panels/hardware/logic';
import {windowDelta, phaseFromSeries} from '../src/panels/robot';
import {TopicTable} from '../src/panels/system/TopicTable';
import {TimeSeriesChart} from '../src/ui/TimeSeriesChart';
import {Sparkline} from '../src/ui/Sparkline';

function populatedStudio(): Studio {
  // Frame 0 carries the dictionary, so decode it first to prime the shared
  // decoder before the bare value frames.
  const frames = [0n, 1n, 2n].map(n => decode(demoPacket(n))!);
  return createStaticStudio({
    frames,
    playback: {
      mode: 'PAUSED',
      cursor_ns: frames[2].timestamp_ns,
      bounds: {start: frames[0].timestamp_ns, end: frames[2].timestamp_ns},
      stats: {dropped: 0, late: 0, invalid: 0},
      status: 'Demo',
      connection: 'demo'
    }
  });
}

const renderPanel = (def: AnyPanelDefinition, studio: Studio): string =>
    renderToStaticMarkup(
        <StudioProvider studio={studio}>
          <def.component config={def.defaultConfig} setConfig={() => {}}
                         instanceId="test"/>
        </StudioProvider>);

test('plot config parsing accepts valid input and cleans malformed input',
     () => {
       // An old workspace's config (no `events` key) gains the default.
       assert.deepEqual(
           plotPanel.parseConfig({channels: ['a', 'b'], windowSeconds: 10}),
           {channels: ['a', 'b'], windowSeconds: 10, events: true});
       assert.deepEqual(plotPanel.parseConfig(null),
                        {channels: [], windowSeconds: null, events: true});
       assert.deepEqual(plotPanel.parseConfig('nope'),
                        {channels: [], windowSeconds: null, events: true});
       // Non-string channels are dropped; a bad window is all-history.
       assert.deepEqual(
           plotPanel.parseConfig({channels: ['a', 2, null], windowSeconds: 'x'}),
           {channels: ['a'], windowSeconds: null, events: true});
       // The default config is a fixed point of its own parser.
       assert.deepEqual(plotPanel.parseConfig(plotPanel.defaultConfig),
                        plotPanel.defaultConfig);
     });

test('the field panel waits for telemetry when there is none', () => {
  const html = renderPanel(fieldPanel, createStaticStudio({}));
  assert.ok(html.includes('Awaiting telemetry'));
  assert.ok(html.includes('BLUE ORIGIN'));
});

test('the field panel shows the chassis pose once telemetry arrives', () => {
  const html = renderPanel(fieldPanel, populatedStudio());
  assert.ok(html.includes('BLUE ORIGIN'));
  assert.ok(html.includes('°'), 'the heading is not shown');
  assert.ok(html.includes('m'), 'the position is not shown');
  assert.ok(!html.includes('Awaiting telemetry'));
});

test('the inspector lists channels with their units', () => {
  const html = renderPanel(inspectorPanel, populatedStudio());
  // Demo dictionary channels, name and unit each.
  assert.ok(html.includes('velocity'));
  assert.ok(html.includes('m/s'));
  assert.ok(html.includes('sim/state/pose/x_m'));
  // The enum channel is listed and its label is rendered for the cursor value.
  assert.ok(html.includes('mode'));
  assert.ok(html.includes('disabled'));
});

test('the 3D viewport renders a placeholder under react-dom/server', () => {
  const html = renderPanel(pose3dPanel, populatedStudio());
  assert.ok(html.includes('3D viewport needs a browser'));
});

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

test('topic observers are searchable and displayed as readers', () => {
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
  assert.ok(html.includes('telemetry (observer)'), html);
  assert.ok(!html.includes('>none<'), html);
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
