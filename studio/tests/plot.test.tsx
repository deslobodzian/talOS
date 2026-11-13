// Plot panel tests: the unit-axis grouping policy, the config parser (including
// the `events` field and backward compatibility with the older two-field
// shape), and an SSR smoke render for empty and populated studios.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {TelemetryDecoder} from '../src/core';
import type {Snapshot} from '../src/core';
import {createStaticStudio, type Studio, StudioProvider} from '../src/data/studio';
import {demoPacket} from '../src/demo';
import {assignYAxes, groupByUnit, type PlotChannel} from '../src/panels/plot/axes';
import {plotPanel} from '../src/panels/plot';

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
