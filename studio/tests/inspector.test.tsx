// Inspector tests: the pure staleness derivation and an SSR smoke render for
// empty and populated studios (grouping, units, enum labels).

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {decode} from '../src/core';
import type {Snapshot} from '../src/core';
import {createStaticStudio, type Studio, StudioProvider} from '../src/data/studio';
import {demoPacket} from '../src/demo';
import {inspectorPanel} from '../src/panels/inspector';
import {channelStalenessNs} from '../src/panels/inspector/staleness';

function frame(ts: string, channels: Record<string, number>,
               units: Record<string, string>,
               labels: Record<string, string[]> = {}): Snapshot {
  return {
    stream_id: 's', sequence_id: '0', timestamp_ns: ts, dictionary_id: 0,
    chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
    channels, units, labels
  };
}

function populatedStudio(): Studio {
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

const renderInspector = (studio: Studio): string => renderToStaticMarkup(
    <StudioProvider studio={studio}>
      <inspectorPanel.component config={inspectorPanel.defaultConfig}
                               setConfig={() => {}} instanceId="test"/>
    </StudioProvider>);

test('channelStalenessNs returns the age of the newest reading', () => {
  const series = {
    t0_ns: '0',
    times: new Float64Array([0, 0.02, 0.04]),
    values: {
      a: new Float64Array([1, 2, NaN]),   // last reading at 0.02 s
      b: new Float64Array([NaN, NaN, NaN])
    }
  };
  // cursor at 60 ms: age of channel a's 20 ms reading is 40 ms.
  assert.equal(channelStalenessNs(series, 'a', '60000000'), '40000000');
  // cursor at the reading itself is fresh.
  assert.equal(channelStalenessNs(series, 'a', '20000000'), '0');
  // a cursor before the reading clamps to fresh rather than a negative age.
  assert.equal(channelStalenessNs(series, 'a', '10000000'), '0');
  // a channel that never had a value, an unknown channel, and a null cursor.
  assert.equal(channelStalenessNs(series, 'b', '60000000'), null);
  assert.equal(channelStalenessNs(series, 'missing', '60000000'), null);
  assert.equal(channelStalenessNs(series, 'a', null), null);
});

test('the inspector shows an empty state when there is no telemetry', () => {
  const html = renderInspector(createStaticStudio({}));
  assert.ok(html.includes('No channels yet.'));
});

test('the inspector renders boolean channels as on/off pills', () => {
  const studio = createStaticStudio({
    frames: [
      frame('0', {enabled: 1, armed: 0}, {enabled: '', armed: ''}),
      frame('20000000', {enabled: 0, armed: 1}, {enabled: '', armed: ''})
    ],
    playback: {mode: 'PAUSED', cursor_ns: '20000000'}
  });
  const html = renderInspector(studio);
  assert.ok(html.includes('pill on'), 'the armed (1) channel shows an on pill');
  assert.ok(html.includes('pill off'), 'the enabled (0) channel shows an off pill');
  assert.ok(!html.includes('1.0000'), 'a boolean must not render as 1.0000');
});

test('the inspector renders numerics with magnitude-aware precision', () => {
  const studio = createStaticStudio({
    frames: [frame('0', {count: 1204, speed: 3.5}, {count: '', speed: 'm/s'})],
    playback: {mode: 'PAUSED', cursor_ns: '0'}
  });
  const html = renderInspector(studio);
  assert.ok(html.includes('1,204'), 'an integer counter prints whole, with separators');
  assert.ok(html.includes('3.5'), 'a float prints only the decimals it needs');
  assert.ok(!html.includes('1204.0000'));
  assert.ok(!html.includes('3.5000'));
});

test('the inspector groups channels and keeps name, unit and enum label', () => {
  const html = renderInspector(populatedStudio());
  // Channels, units and the enum label from the demo dictionary.
  assert.ok(html.includes('velocity'));
  assert.ok(html.includes('m/s'));
  assert.ok(html.includes('pose/x_m'));
  assert.ok(html.includes('mode'));
  assert.ok(html.includes('disabled'));
  // Owner/role group headers.
  assert.ok(html.includes('(root)'));
  assert.ok(html.includes('sim/state'));
  // Staleness of a channel read at the cursor.
  assert.ok(html.includes('now'));
  // The add-to-plot affordance is present.
  assert.ok(html.includes('Add to plot'));
});
