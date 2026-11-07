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
