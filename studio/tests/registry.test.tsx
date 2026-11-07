// Registry tests: ids are unique, every default config survives its own
// parseConfig, and every panel renders under react-dom/server for both an
// empty studio and a populated one. A typecheck and a bundle prove the code
// compiles; this proves every panel actually renders.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {decode} from '../src/core';
import {createStaticStudio, type Studio, StudioProvider} from '../src/data/studio';
import {demoDeclaredGraph, demoPacket, demoSystemGraph} from '../src/demo';
import {eventRates, parseDeclaredGraph, parseSystemGraph} from '../src/system';
import {PANELS} from '../src/panels/registry';
import type {AnyPanelDefinition} from '../src/panels/types';

function renderPanel(panel: AnyPanelDefinition, studio: Studio): string {
  return renderToStaticMarkup(
      <StudioProvider studio={studio}>
        <panel.component config={panel.defaultConfig} setConfig={() => {}}
                         instanceId="test"/>
      </StudioProvider>);
}

function populatedInput() {
  // Frame 0 carries the dictionary, so decode it first to prime the shared
  // decoder before the bare value frames.
  const frames = [0n, 1n, 2n].map(n => decode(demoPacket(n))!);
  const previous = parseSystemGraph(demoSystemGraph(500n));
  const graph = parseSystemGraph(demoSystemGraph(1500n));
  const rates: Record<string, number> = {};
  for (const [key, value] of eventRates(graph, previous)) rates[key] = value;
  const system = {
    graph,
    rates,
    error: '',
    declared: parseDeclaredGraph(demoDeclaredGraph()),
    declaredError: ''
  };
  return {frames, system};
}

test('every panel has a unique id', () => {
  const ids = PANELS.map(panel => panel.id);
  assert.equal(new Set(ids).size, ids.length);
});

test('every default config passes its own parseConfig', () => {
  for (const panel of PANELS)
    assert.deepEqual(panel.parseConfig(panel.defaultConfig),
                     panel.defaultConfig, panel.id);
});

test('every panel renders with an empty studio without throwing', () => {
  const studio = createStaticStudio({});
  for (const panel of PANELS) {
    const html = renderPanel(panel, studio);
    assert.ok(html.length > 0, `${panel.id} rendered nothing`);
  }
});

test('every panel renders with a populated studio without throwing', () => {
  const studio = createStaticStudio(populatedInput());
  for (const panel of PANELS) {
    const html = renderPanel(panel, studio);
    assert.ok(html.length > 0, `${panel.id} rendered nothing`);
  }
});
