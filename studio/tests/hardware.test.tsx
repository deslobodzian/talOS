// Hardware panel: pure derivations (counter rate, bitmask history, flag bands)
// and server-side rendering of the panel against empty and populated studios.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import type {Snapshot} from '../src/core';
import {createStaticStudio, type Studio, StudioProvider} from '../src/data/studio';
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
  // A wrapped (reset) counter is not a negative rate.
  assert.equal(counterRate([0, 5], [100, 2]), null);
  // NaN samples are skipped, not treated as zero.
  assert.equal(counterRate([0, 1, NaN, 5], [0, NaN, 4, 5]), 1);
});

test('bitHistory reports the last set edge and current state per bit', () => {
  // mask: 3 (bits 0,1), 2 (bit 1), 3 (bits 0,1). Bit 0 starts set, clears,
  // then rises again; bit 1 is set the whole time.
  const bits = bitHistory([0, 1, 2], [3, 2, 3], 2);
  assert.equal(bits.length, 2);

  const b0 = bits[0];
  assert.equal(b0.bit, 0);
  assert.equal(b0.set, true);
  assert.equal(b0.setCount, 1);   // one 0→1 edge, at the last sample
  assert.equal(b0.lastSetAt, 2);

  const b1 = bits[1];
  assert.equal(b1.set, true);
  assert.equal(b1.setCount, 0);   // set at the first sample, never rose
  assert.equal(b1.lastSetAt, null);
});

test('bitHistory leaves an already-set bit without an in-view edge', () => {
  // Bit 2 is 1 from the first sample: set, no rising edge in view.
  const bits = bitHistory([0, 1], [4, 4], 4);
  assert.equal(bits[2].set, true);
  assert.equal(bits[2].setCount, 0);
  assert.equal(bits[2].lastSetAt, null);
  // Bit 3 never set.
  assert.equal(bits[3].set, false);
  assert.equal(bits[3].setCount, 0);
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

test('decimateMinMax collapses a 12k-sample series to O(width) spans', () => {
  const values = new Float64Array(12000);
  for (let i = 0; i < values.length; i++) values[i] = Math.sin(i * 0.01);
  const runs = decimateMinMax(undefined, values, 600, 0, 11999);
  const total = runs.reduce((n, run) => n + run.length, 0);
  assert.ok(total > 0, 'decimation produced spans');
  assert.ok(total <= 600, `expected ≤ 600 spans, got ${total}`);
});

test('decimateMinMax preserves a single-sample spike', () => {
  const values = new Float64Array(12000).fill(0);
  values[6000] = 1000;
  const runs = decimateMinMax(undefined, values, 600, 0, 11999);
  let sawSpike = false;
  for (const run of runs)
    for (const span of run) if (span.yMax >= 1000) sawSpike = true;
  assert.ok(sawSpike, 'the spike was averaged away');
});

test('decimateMinMax breaks runs at NaN gaps', () => {
  const values = new Float64Array([1, 2, 3, NaN, NaN, 4, 5, 6]);
  const runs = decimateMinMax(undefined, values, 2, 0, 7);
  assert.equal(runs.length, 2);
  assert.deepEqual(runs[0].map(s => s.yMax), [3]);
  assert.deepEqual(runs[1].map(s => s.yMax), [6]);
});

// --- rendering -------------------------------------------------------------

const HW = 'hw/status/';

function snap(seq: number, ts: string,
              channels: Record<string, number>): Snapshot {
  return {
    stream_id: 'a',
    sequence_id: String(seq),
    timestamp_ns: ts,
    dictionary_id: 1,
    chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
    channels,
    units: {},
    labels: {}
  };
}

// Three hw/status samples 20 ms apart, shaped like the 2026 bridge.
function populatedStudio(): Studio {
  const base = 1_000_000_000;
  const at = (n: number) => String(base + n * 20_000_000);
  const frames: Snapshot[] = [
    snap(0, at(0), {
      [`${HW}battery_v`]: 12.1,
      [`${HW}brownout`]: 0,
      [`${HW}can_utilization_frac`]: 0.5,
      [`${HW}state_age_us`]: 0,
      [`${HW}configured`]: 1,
      [`${HW}subsystem_count`]: 2,
      [`${HW}timed_out_mask`]: 0b11,
      [`${HW}motor_fault_mask`]: 0,
      [`${HW}clock_offset_us`]: 2_000_000,
      [`${HW}clock_offset_spread_us`]: 700,
      [`${HW}clock_offset_valid`]: 1,
      [`${HW}clock_resets_count`]: 0,
      [`${HW}clock_window_count`]: 8,
      [`${HW}clock_ignored_count`]: 0,
      [`${HW}health_valid`]: 1,
      [`${HW}states_received_count`]: 1000,
      [`${HW}commands_sent_count`]: 500,
      [`${HW}identity_rejected_count`]: 0,
      [`${HW}ownership_rejected_count`]: 0,
      [`${HW}driver_station_packet_count`]: 100,
      [`${HW}gateway_rejected_command_count`]: 11,
      [`${HW}gateway_neutralized_motor_count`]: 12,
      [`${HW}gateway_apply_failure_count`]: 13,
      [`${HW}gateway_device_reset_count`]: 14
    }),
    snap(1, at(1), {
      [`${HW}battery_v`]: 12.0,
      [`${HW}brownout`]: 1,
      [`${HW}can_utilization_frac`]: 0.6,
      [`${HW}state_age_us`]: 1000,
      [`${HW}configured`]: 1,
      [`${HW}subsystem_count`]: 2,
      [`${HW}timed_out_mask`]: 0b10,
      [`${HW}motor_fault_mask`]: 2,
      [`${HW}clock_offset_us`]: 2_000_000,
      [`${HW}clock_offset_spread_us`]: 700,
      [`${HW}clock_offset_valid`]: 1,
      [`${HW}clock_resets_count`]: 0,
      [`${HW}clock_window_count`]: 8,
      [`${HW}clock_ignored_count`]: 0,
      [`${HW}health_valid`]: 1,
      [`${HW}states_received_count`]: 1010,
      [`${HW}commands_sent_count`]: 505,
      [`${HW}identity_rejected_count`]: 0,
      [`${HW}ownership_rejected_count`]: 0,
      [`${HW}driver_station_packet_count`]: 100,
      [`${HW}gateway_rejected_command_count`]: 11,
      [`${HW}gateway_neutralized_motor_count`]: 12,
      [`${HW}gateway_apply_failure_count`]: 13,
      [`${HW}gateway_device_reset_count`]: 14
    }),
    snap(2, at(2), {
      [`${HW}battery_v`]: 11.9,
      [`${HW}brownout`]: 0,
      [`${HW}can_utilization_frac`]: 0.7,
      [`${HW}state_age_us`]: 3500,
      [`${HW}configured`]: 1,
      [`${HW}subsystem_count`]: 2,
      [`${HW}timed_out_mask`]: 0b11,
      [`${HW}motor_fault_mask`]: 2,
      [`${HW}clock_offset_us`]: 2_000_000,
      [`${HW}clock_offset_spread_us`]: 700,
      [`${HW}clock_offset_valid`]: 1,
      [`${HW}clock_resets_count`]: 0,
      [`${HW}clock_window_count`]: 8,
      [`${HW}clock_ignored_count`]: 0,
      [`${HW}health_valid`]: 1,
      [`${HW}states_received_count`]: 1020,
      [`${HW}commands_sent_count`]: 510,
      [`${HW}identity_rejected_count`]: 0,
      [`${HW}ownership_rejected_count`]: 0,
      [`${HW}driver_station_packet_count`]: 100,
      [`${HW}gateway_rejected_command_count`]: 11,
      [`${HW}gateway_neutralized_motor_count`]: 12,
      [`${HW}gateway_apply_failure_count`]: 13,
      [`${HW}gateway_device_reset_count`]: 14
    })
  ];
  return createStaticStudio({
    frames,
    playback: {
      mode: 'PAUSED',
      cursor_ns: frames[2].timestamp_ns,
      bounds: {start: frames[0].timestamp_ns, end: frames[2].timestamp_ns},
      stats: {dropped: 0, late: 0, invalid: 0},
      status: 'Static',
      connection: 'replay'
    }
  });
}

const renderPanel = (studio: Studio): string => renderToStaticMarkup(
    <StudioProvider studio={studio}>
      <hardwarePanel.component config={hardwarePanel.defaultConfig}
                               setConfig={() => {}} instanceId="test"/>
    </StudioProvider>);

test('the hardware panel waits for hw/status when there is none', () => {
  const html = renderPanel(createStaticStudio({}));
  assert.ok(html.includes('No /hw/status telemetry'));
});

test('the hardware panel shows power, link, clock and counters', () => {
  const html = renderPanel(populatedStudio());
  // Power: the cursor battery voltage, and CAN as a percentage.
  assert.ok(html.includes('11.90 V'));
  assert.ok(html.includes('70%'));
  assert.ok(html.includes('Battery voltage'));
  assert.ok(html.includes('CAN utilization'));
  // Link: configured, subsystem count and state age.
  assert.ok(html.includes('Configured'));
  assert.ok(html.includes('yes'));
  assert.ok(html.includes('Subsystems'));
  assert.ok(html.includes('3.50 ms'), 'state age is formatted');
  // Clock.
  assert.ok(html.includes('Offset'));
  assert.ok(html.includes('2.00 s'));
  assert.ok(html.includes('Window'));
  // Counters, total with thousands separators and a note.
  assert.ok(html.includes('States received'));
  assert.ok(html.includes('1,020'));
  assert.ok(html.includes('Gateway rejected'));
  assert.ok(html.includes('Commands the gateway refused'));
  // Motor fault mask bit list names the set bit.
  assert.ok(html.includes('motor 1'));
  // Timeouts name the actuator-owning subsystem bits.
  assert.ok(html.includes('subsystem 0'));
  assert.ok(html.includes('subsystem 1'));
});

test('the hardware panel omits counters the bridge never declared', () => {
  // A sparse studio: only battery_v exists; no counter is listed.
  const studio = createStaticStudio({
    frames: [snap(0, '1000000000', {[`${HW}battery_v`]: 12.0})],
    playback: {mode: 'PAUSED', cursor_ns: '1000000000'}
  });
  const html = renderPanel(studio);
  assert.ok(html.includes('12.00 V'));
  assert.ok(!html.includes('States received'));
  assert.ok(!html.includes('Commands sent'));
});
