// Robot panel tests: the pure derivations (mask bits, window delta, phase
// history, enum highlight) and the rendered panel's FMS state and recent-event
// window.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {createStaticStudio, type Studio, StudioProvider} from '../src/data/studio';
import type {RobotEvent, Series} from '../src/data/types';
import {robotPanel, maskBits, windowDelta, phaseFromSeries, badgeFor} from '../src/panels/robot';

const renderPanel = (studio: Studio): string =>
    renderToStaticMarkup(
        <StudioProvider studio={studio}>
          <robotPanel.component config={robotPanel.defaultConfig}
                                setConfig={() => {}} instanceId="test"/>
        </StudioProvider>);

test('FMS distinguishes not attached from an unknown attachment state', () => {
  const text = (fms: number|undefined) => {
    const channels: Record<string, number> = {
      'hw/state/driver_station/connected': 1,
      'hw/state/driver_station/enabled': 0
    };
    if (fms !== undefined) channels['hw/state/driver_station/fms_attached'] = fms;
    const timestamp_ns = '1000000000';
    return renderPanel(createStaticStudio({
      frames: [{
        stream_id: 'a', sequence_id: '0', timestamp_ns, dictionary_id: 1,
        chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
        channels, units: {}, labels: {}
      }],
      playback: {mode: 'PAUSED', cursor_ns: timestamp_ns}
    })).replace(/<[^>]*>/g, '');
  };
  assert.match(text(0), /FMSnot attached/);
  assert.match(text(1), /FMSattached/);
  assert.match(text(undefined), /FMS—/);
});

// --- pure derivations -------------------------------------------------------

test('maskBits lists set bits, none when clear, null when unknown', () => {
  assert.equal(maskBits(0), 'none');
  assert.equal(maskBits(0b1010), '1, 3');
  assert.equal(maskBits(0b1), '0');
  assert.equal(maskBits(undefined), null);
  assert.equal(maskBits(NaN), null);
  assert.equal(maskBits(-1), null);
});

test('windowDelta measures the change over the trailing window', () => {
  const series: Series = {
    t0_ns: '0',
    times: new Float64Array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]),
    values: {c: new Float64Array([10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 25])}
  };
  const {current, delta} = windowDelta(series, 'c', 10);
  assert.equal(current, 25);
  // Ten seconds before the cursor (11 s) is 1 s, whose value was 11.
  assert.equal(delta, 14);

  // A single finite reading has no history to change against.
  const single: Series = {
    t0_ns: '0',
    times: new Float64Array([0]),
    values: {c: new Float64Array([5])}
  };
  assert.deepEqual(windowDelta(single, 'c', 10), {current: 5, delta: 0});

  // A column that is all NaN reports nothing at all.
  const empty: Series = {
    t0_ns: '0',
    times: new Float64Array([0, 1]),
    values: {c: new Float64Array([NaN, NaN])}
  };
  assert.deepEqual(windowDelta(empty, 'c', 10), {current: null, delta: null});
});

test('phaseFromSeries reads the current, previous and held duration', () => {
  const labels = ['IDLE', 'SPINUP', 'FEED', 'DRIVE', 'HOLD'];
  const series: Series = {
    t0_ns: '0',
    times: new Float64Array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]),
    // IDLE for 0-3, FEED from 4 onward.
    values: {p: new Float64Array([0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2])}
  };
  const phase = phaseFromSeries(series, 'p', labels);
  assert.equal(phase.current, 'FEED');
  assert.equal(phase.previous, 'IDLE');
  assert.equal(phase.seconds, 7);  // 11 s cursor - 4 s change

  // One phase throughout the retained history: no previous phase, held since
  // the first sample.
  const one: Series = {
    t0_ns: '0',
    times: new Float64Array([0, 1, 2]),
    values: {p: new Float64Array([2, 2, 2])}
  };
  const single = phaseFromSeries(one, 'p', labels);
  assert.equal(single.current, 'FEED');
  assert.equal(single.previous, null);
  assert.equal(single.seconds, 2);
});

test('badgeFor highlights only the non-neutral enum label', () => {
  const labels = ['DRIVE', 'GATEWAY_NOT_READY', 'TARGET_DISABLED'];
  assert.equal(badgeFor(0, labels), null);       // neutral
  assert.equal(badgeFor(1, labels), 'GATEWAY_NOT_READY');
  assert.equal(badgeFor(undefined, labels), null);
  assert.equal(badgeFor(2, []), null);           // no labels
  assert.equal(badgeFor(9, labels), null);       // out of range
});

// --- rendering --------------------------------------------------------------

test('recent events show only the last eight', () => {
  const events: RobotEvent[] = Array.from({length: 10}, (_, i) => ({
    id: `e${i}`, t_ns: String(1735689600000000000n + BigInt(i) * 1000000000n),
    severity: 'info', category: 'hardware', title: `event-${i}`
  }));
  const html = renderPanel(createStaticStudio({events}));
  for (let i = 0; i < 10; i++)
    assert.equal(html.includes(`event-${i}<`), i >= 2, `event-${i}`);
});
