// Hardware panel: pure derivations (counter rate, bitmask history, flag bands,
// min/max decimation) and the panel's handling of undeclared counters.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {createStaticStudio, StudioProvider} from '../src/data/studio';
import {hardwarePanel} from '../src/panels/hardware';
import {
  bitHistory,
  counterRate,
  flagBands,
} from '../src/panels/hardware/logic';
import {decimateMinMax} from '../src/ui/TimeSeriesChart';

// --- pure logic ------------------------------------------------------------

test('counterRate measures a counter over its samples', () => {
  // 10 units over 10 s => 1/s.
  assert.equal(counterRate([0, 2, 5, 10], [0, 2, 5, 10]), 1);
  // A flat counter is honestly 0, not absent.
  assert.equal(counterRate([0, 5], [7, 7]), 0);
  // One reading or a zero span cannot be measured.
  assert.equal(counterRate([0], [3]), null);
  assert.equal(counterRate([5, 5], [1, 2]), null);
  // A reset adds no increment: 10 -> 20, reset, 2 -> 12 is 20 units over
  // 10 s, not the first-to-last 2 units nor a negative rate.
  assert.equal(counterRate([0, 2, 5, 10], [10, 20, 2, 12]), 2);
  // NaN samples are skipped, not treated as zero.
  assert.equal(counterRate([0, 1, NaN, 5], [0, NaN, 4, 5]), 1);
});

test('bitHistory reports the last set edge and current state per bit', () => {
  // mask: 3 (bits 0,1), 2 (bit 1), 3 (bits 0,1). Bit 0 starts set, clears,
  // then rises again; bit 1 is set the whole time; bit 2 never sets.
  const bits = bitHistory([0, 1, 2], [3, 2, 3], 3);
  assert.equal(bits.length, 3);

  const b0 = bits[0];
  assert.equal(b0.bit, 0);
  assert.equal(b0.set, true);
  assert.equal(b0.setCount, 2);   // initial active state, then one 0→1 edge
  assert.equal(b0.lastSetAt, 2);

  const b1 = bits[1];
  assert.equal(b1.set, true);
  assert.equal(b1.setCount, 1);   // initial active state, never rose
  assert.equal(b1.lastSetAt, null);

  assert.equal(bits[2].set, false);
  assert.equal(bits[2].setCount, 0);
});

test('bitHistory distinguishes absent masks from measured clear bits', () => {
  for (const mask of [[], [NaN, Infinity]]) {
    const bits = bitHistory([0, 1], mask, 2);
    assert.deepEqual(bits.map(b => b.set), [null, null]);
  }
  assert.deepEqual(bitHistory([0], [0], 2).map(b => b.set), [false, false]);
});

test('flagBands groups contiguous 1 runs and skips NaN', () => {
  assert.deepEqual(flagBands([0, 1, 2, 3, 4], [0, 1, 1, 0, 1]),
                   [{start: 1, end: 3}, {start: 4, end: 4}]);
  assert.deepEqual(flagBands([0, 1, 2, 3], [1, NaN, 1, 1]),
                   [{start: 0, end: 1}, {start: 2, end: 3}]);
  assert.deepEqual(flagBands([0, 1], [0, 0]), []);
});

test('decimateMinMax collapses 12k samples to O(width) spans and keeps a spike', () => {
  const values = new Float64Array(12000);
  for (let i = 0; i < values.length; i++) values[i] = Math.sin(i * 0.01);
  values[6000] = 1000;
  const spans = decimateMinMax(undefined, values, 600, 0, 11999).flat();
  assert.ok(spans.length > 0 && spans.length <= 600, `got ${spans.length} spans`);
  assert.ok(spans.some(span => span.yMax >= 1000), 'the spike was averaged away');
});

test('decimateMinMax breaks runs at NaN gaps', () => {
  const values = new Float64Array([1, 2, 3, NaN, NaN, 4, 5, 6]);
  const runs = decimateMinMax(undefined, values, 2, 0, 7);
  assert.equal(runs.length, 2);
  assert.deepEqual(runs[0].map(s => s.yMax), [3]);
  assert.deepEqual(runs[1].map(s => s.yMax), [6]);
});

// --- rendering -------------------------------------------------------------

test('the hardware panel omits counters the bridge never declared', () => {
  // A sparse studio: only battery_v exists; no counter is listed.
  const studio = createStaticStudio({
    frames: [{
      stream_id: 'a', sequence_id: '0', timestamp_ns: '1000000000', dictionary_id: 1,
      chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
      channels: {'hw/status/battery_v': 12.0}, units: {}, labels: {}
    }],
    playback: {mode: 'PAUSED', cursor_ns: '1000000000'}
  });
  const html = renderToStaticMarkup(
      <StudioProvider studio={studio}>
        <hardwarePanel.component config={hardwarePanel.defaultConfig}
                                 setConfig={() => {}} instanceId="test"/>
      </StudioProvider>);
  assert.ok(html.includes('12.00 V'));
  assert.ok(!html.includes('States received'));
  assert.ok(!html.includes('Commands sent'));
});
