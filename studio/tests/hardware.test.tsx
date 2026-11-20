// Hardware panel: pure derivations (counter rate, bitmask history, flag bands,
// min/max decimation) and the panel's handling of undeclared counters.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {createStaticStudio, StudioProvider} from '../src/data/studio';
import {hardwarePanel} from '../src/panels/hardware';
import {canBuses, canBusReading} from '../src/data/robot';
import {
  bitHistory,
  canBusLines,
  canBusSeriesChannels,
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

// /hw/status always carries all four can_buses slots, whatever can_bus_count
// says: a series holding every slot, as the store hands it back.
function canBusSeries(slots: Array<{state: number[]; frac: number[]}>) {
  const values: Record<string, Float64Array> = {};
  for (let i = 0; i < 4; i++) {
    const slot = slots[i] ?? {state: [0, 0, 0], frac: [0, 0, 0]};
    values[`hw/status/can_buses/${i}/read_state`] = Float64Array.from(slot.state);
    values[`hw/status/can_buses/${i}/utilization_frac`] = Float64Array.from(slot.frac);
  }
  return {times: Float64Array.from([0, 1, 2]), values};
}

// REQ-OPS-20: the utilization chart draws the can_bus_count buses in use, not
// the four fixed slots /hw/status carries.
test('the CAN utilization chart draws only the buses in use', () => {
  const series = canBusSeries([{state: [1, 1, 1], frac: [0.1, 0.2, 0.3]}]);
  const lines = canBusLines(1, series);
  assert.deepEqual(lines.map(l => l.name), ['rio']);
  assert.deepEqual(Array.from(lines[0].values), [0.1, 0.2, 0.3]);
  // No configuration reported yet: no bus, and no line.
  assert.deepEqual(canBusLines(0, series), []);
  assert.deepEqual(canBusLines(undefined, series), []);
  assert.deepEqual(canBusSeriesChannels(0), []);
});

// REQ-OPS-20: a bus whose read failed, went stale or never happened has no
// utilization (its figure is the last good read's, 0 before one): the chart
// breaks its line there and the bus model carries no figure, so it never
// reads as an idle bus.
test('an unread CAN bus has no utilization, in the chart or the model', () => {
  const series = canBusSeries([
    {state: [1, 3, 1], frac: [0.2, 0.2, 0.3]},  // OK, FAILED, OK
    {state: [0, 3, 4], frac: [0, 0, 0]}         // NONE, FAILED, STALE
  ]);
  const lines = canBusLines(2, series);
  assert.equal(lines.length, 2);
  assert.deepEqual(Array.from(lines[0].values), [0.2, NaN, 0.3]);
  assert.ok(Array.from(lines[1].values).every(v => Number.isNaN(v)),
            `the unread bus is drawn at ${Array.from(lines[1].values)}`);

  const B = 'hw/status/can_buses/';
  const buses = canBuses({
    'hw/status/can_bus_count': 2,
    [B + '0/read_state']: 0, [B + '0/utilization_frac']: 0,
    [B + '1/read_state']: 3, [B + '1/utilization_frac']: 0,
    [B + '1/bus_off_count']: 4
  });
  assert.deepEqual(buses.map(b => b.frac), [null, null]);
  assert.deepEqual(buses.map(canBusReading), ['not read', 'failed']);
  assert.equal(buses[1].busOff, 4);
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

// REQ-OPS-20: a CAN bus whose read failed shows its state where an idle bus
// would show 0%, and each bus has its own row, slot 0 named "rio".
test('the hardware panel shows a failed CAN bus as failed, not as 0%', () => {
  const B = 'hw/status/can_buses/';
  const studio = createStaticStudio({
    frames: [{
      stream_id: 'a', sequence_id: '0', timestamp_ns: '1000000000', dictionary_id: 1,
      chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
      channels: {
        'hw/status/battery_v': 12.0, 'hw/status/can_bus_count': 2,
        [B + '0/read_state']: 1, [B + '0/utilization_frac']: 0.25,
        [B + '1/read_state']: 3, [B + '1/utilization_frac']: 0,
        [B + '1/bus_off_count']: 4
      },
      units: {}, labels: {}
    }],
    playback: {mode: 'PAUSED', cursor_ns: '1000000000'}
  });
  const html = renderToStaticMarkup(
      <StudioProvider studio={studio}>
        <hardwarePanel.component config={hardwarePanel.defaultConfig}
                                 setConfig={() => {}} instanceId="test"/>
      </StudioProvider>);
  assert.ok(html.includes('>rio<'), html);
  assert.ok(html.includes('CAN bus 1'));
  assert.ok(html.includes('25%'));
  assert.ok(html.includes('can-unread">failed<'), 'the failed bus is not marked');
  assert.ok(!html.includes('>0%<'), 'the failed bus reads as an idle one');
});

// REQ-OPS-23: when the declared graph carries can_buses, the hardware panel's
// table and CAN utilization chart series label slots by configured bus name
// rather than "CAN bus N".
test('the hardware panel and chart label CAN buses from the declared graph', () => {
  const series = canBusSeries([
    {state: [1, 1, 1], frac: [0.1, 0.2, 0.3]},
    {state: [1, 1, 1], frac: [0.4, 0.5, 0.6]}
  ]);
  assert.deepEqual(
      canBusLines(2, series, ['rio', 'Shooter']).map(l => l.name),
      ['rio', 'Shooter']);
  assert.deepEqual(
      canBusLines(2, series).map(l => l.name),
      ['rio', 'CAN bus 1']);

  const B = 'hw/status/can_buses/';
  const studio = createStaticStudio({
    frames: [{
      stream_id: 'a', sequence_id: '0', timestamp_ns: '1000000000', dictionary_id: 1,
      chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
      channels: {
        'hw/status/battery_v': 12.0, 'hw/status/can_bus_count': 2,
        [B + '0/read_state']: 1, [B + '0/utilization_frac']: 0.15,
        [B + '1/read_state']: 1, [B + '1/utilization_frac']: 0.42
      },
      units: {}, labels: {}
    }],
    playback: {mode: 'PAUSED', cursor_ns: '1000000000'},
    system: {
      declared: {version: 1, nodes: [], diagnostics: [], can_buses: ['rio', 'Shooter']}
    }
  });
  const html = renderToStaticMarkup(
      <StudioProvider studio={studio}>
        <hardwarePanel.component config={hardwarePanel.defaultConfig}
                                 setConfig={() => {}} instanceId="test"/>
      </StudioProvider>);
  assert.ok(html.includes('>rio<'), html);
  assert.ok(html.includes('>Shooter<'), html);
  assert.ok(!html.includes('CAN bus 1'), html);
});
