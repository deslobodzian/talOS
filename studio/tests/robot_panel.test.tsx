// Robot panel tests: the pure derivations (mask bits, window delta, phase
// history, enum highlight) and the panel rendering under react-dom/server for
// an empty studio and a populated one shaped like the 2026 dictionary.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import type {Snapshot} from '../src/core';
import {createStaticStudio, type Studio, StudioProvider} from '../src/data/studio';
import type {RobotEvent, Series} from '../src/data/types';
import type {SystemGraph, SystemNode, SystemSource} from '../src/system';
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
    const sample = frame(0, 1000000000n, channels, {}, {});
    return renderPanel(createStaticStudio({
      frames: [sample], playback: {mode: 'PAUSED', cursor_ns: sample.timestamp_ns}
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

// --- fixtures ---------------------------------------------------------------

function frame(seq: number, ts: bigint, channels: Record<string, number>,
               units: Record<string, string>,
               labels: Record<string, string[]>): Snapshot {
  return {
    stream_id: 'a',
    sequence_id: String(seq),
    timestamp_ns: String(ts),
    dictionary_id: 1,
    chassis: {x: 1.2, y: 2.3, z: 0, roll: 0, pitch: 0, yaw: 0.7},
    channels,
    units,
    labels
  };
}

const UNITS: Record<string, string> = {
  'hw/state/driver_station/connected': '',
  'hw/state/driver_station/enabled': '',
  'hw/state/driver_station/autonomous': '',
  'hw/state/driver_station/teleop': '',
  'hw/state/driver_station/test': '',
  'hw/state/driver_station/estop': '',
  'hw/state/driver_station/fms_attached': '',
  'hw/state/driver_station/match_time_s': 's',
  'hw/state/driver_station/alliance': '',
  'hw/status/battery_v': 'V',
  'hw/status/brownout': '',
  'hw/status/can_utilization_frac': '',
  'hw/status/health_valid': '',
  'hw/status/state_age_us': 'us',
  'hw/status/configured': '',
  'hw/status/motor_fault_mask': '',
  'hw/status/timed_out_mask': '',
  'hw/status/gateway_rejected_command_count': '',
  'hw/status/gateway_apply_failure_count': '',
  'autonomous/state/phase': '',
  'pose_estimator/state/pose/x_m': 'm',
  'pose_estimator/state/pose/y_m': 'm',
  'pose_estimator/state/pose/yaw_rad': 'rad',
  'pose_estimator/state/valid': '',
  'pose_estimator/state/observation_accepted_count': '',
  'pose_estimator/state/observation_rejected_count': '',
  'sim/state/pose/x_m': 'm',
  'sim/state/pose/y_m': 'm',
  'sim/state/pose/yaw_rad': 'rad',
  'drivetrain/state/enabled': '',
  'shooter/state/enabled': '',
  'intake/state/enabled': '',
  'intake/state/inhibit': '',
  'front_left_camera/state/pose/x_m': 'm',
  'front_left_camera/state/x_std_m': 'm',
  'front_left_camera/state/tag_count': '',
  'front_left_camera/status/frame_period_us': 'us',
  'front_left_camera/status/visible_tag_count': '',
  'front_left_camera/status/healthy': '',
  'front_right_camera/state/pose/x_m': 'm',
  'front_right_camera/state/x_std_m': 'm',
  'front_right_camera/state/tag_count': '',
  'front_right_camera/status/frame_period_us': 'us',
  'front_right_camera/status/visible_tag_count': '',
  'front_right_camera/status/healthy': '',
  'rear_left_camera/state/pose/x_m': 'm',
  'rear_left_camera/state/x_std_m': 'm',
  'rear_left_camera/state/tag_count': '',
  'rear_left_camera/status/frame_period_us': 'us',
  'rear_left_camera/status/visible_tag_count': '',
  'rear_left_camera/status/healthy': '',
  'rear_right_camera/state/pose/x_m': 'm',
  'rear_right_camera/state/x_std_m': 'm',
  'rear_right_camera/state/tag_count': '',
  'rear_right_camera/status/frame_period_us': 'us',
  'rear_right_camera/status/visible_tag_count': '',
  'rear_right_camera/status/healthy': ''
};

const LABELS: Record<string, string[]> = {
  'hw/state/driver_station/alliance': ['unknown', 'red', 'blue'],
  'autonomous/state/phase': ['IDLE', 'SPINUP', 'FEED', 'DRIVE', 'HOLD'],
  'intake/state/inhibit': ['DRIVE', 'GATEWAY_NOT_READY', 'TARGET_DISABLED']
};

// Twelve frames, one second apart: enough history for the 10 s deltas, the 60 s
// battery sparkline, and a phase change part-way through.
function populatedFrames(): Snapshot[] {
  const base = 1735689600000000000n;
  return Array.from({length: 12}, (_, i) => {
    const channels: Record<string, number> = {
      'hw/state/driver_station/connected': 1,
      'hw/state/driver_station/enabled': 1,
      'hw/state/driver_station/autonomous': 1,
      'hw/state/driver_station/teleop': 0,
      'hw/state/driver_station/test': 0,
      'hw/state/driver_station/estop': 0,
      'hw/state/driver_station/fms_attached': 1,
      'hw/state/driver_station/match_time_s': 74 - i,
      'hw/state/driver_station/alliance': 2,
      'hw/status/battery_v': 12.3 + 0.2 * Math.sin(i / 2),
      'hw/status/brownout': 0,
      'hw/status/can_utilization_frac': 0.34,
      'hw/status/health_valid': 1,
      'hw/status/state_age_us': 900,
      'hw/status/configured': 1,
      'hw/status/motor_fault_mask': 0b101,
      'hw/status/timed_out_mask': 0,
      'hw/status/gateway_rejected_command_count': i,
      'hw/status/gateway_apply_failure_count': i * 2,
      'autonomous/state/phase': i < 4 ? 0 : 2,
      'pose_estimator/state/pose/x_m': 1.2,
      'pose_estimator/state/pose/y_m': 2.3,
      'pose_estimator/state/pose/yaw_rad': 0.7,
      'pose_estimator/state/valid': 1,
      'pose_estimator/state/observation_accepted_count': 10 + i * 5,
      'pose_estimator/state/observation_rejected_count': 1 + Math.floor(i / 3),
      'sim/state/pose/x_m': 1.0,
      'sim/state/pose/y_m': 2.0,
      'sim/state/pose/yaw_rad': 0.65,
      'drivetrain/state/enabled': 1,
      'shooter/state/enabled': 1,
      'intake/state/enabled': 1,
      'intake/state/inhibit': 1,
      'front_left_camera/state/pose/x_m': 0.1,
      'front_left_camera/state/x_std_m': 0.01,
      'front_left_camera/state/tag_count': 3,
      'front_left_camera/status/frame_period_us': 40000,
      'front_left_camera/status/visible_tag_count': 3,
      'front_left_camera/status/healthy': 1,
      'front_right_camera/state/pose/x_m': 0.2,
      'front_right_camera/state/x_std_m': 0.02,
      'front_right_camera/state/tag_count': 2,
      'front_right_camera/status/frame_period_us': 50000,
      'front_right_camera/status/visible_tag_count': 2,
      'front_right_camera/status/healthy': 1,
      'rear_left_camera/state/pose/x_m': 0.3,
      'rear_left_camera/state/x_std_m': 0.03,
      'rear_left_camera/state/tag_count': 4,
      'rear_left_camera/status/frame_period_us': 30000,
      'rear_left_camera/status/visible_tag_count': 4,
      'rear_left_camera/status/healthy': 0,
      'rear_right_camera/state/pose/x_m': 0.4,
      'rear_right_camera/state/x_std_m': 0.04,
      'rear_right_camera/state/tag_count': 0,
      'rear_right_camera/status/frame_period_us': 100000,
      'rear_right_camera/status/visible_tag_count': 0,
      'rear_right_camera/status/healthy': 1
    };
    return frame(i, base + BigInt(i) * 1000000000n, channels, UNITS, LABELS);
  });
}

function source(name: string): SystemSource {
  return {
    id: 0, kind: 'SENDER', name, message_bytes: 24, alignment: 8,
    period_ns: '0', events: '0', dropped: '0', sequence: '0',
    last_monotonic_ns: '0', last_latency_ns: '0', max_latency_ns: '0',
    external: false, optional: false, observer: false
  };
}

function node(name: string, selected: string|null): SystemNode {
  return {
    name, target: name, slot: 0, pid: '10', session_id: '1', generation: '1',
    start_wall_ns: '0', heartbeat_wall_ns: '0', dispatch_count: '0',
    declared_sources: 1, alive: true, simulation: false, replay: false,
    sources: [source('/' + name + '/state')],
    selections: [{sources: ['teleop', 'auto'], selected}]
  };
}

function graph(): SystemGraph {
  return {
    version: 2, wall_ns: '0',
    registry: {available: true, node_capacity: 32, source_capacity: 64,
               liveness_timeout_ns: '5000000000'},
    bridge: {topic: '/talos/telemetry', clients: 1, frames_forwarded: '0',
             invalid: '0', ws_drops: '0'},
    nodes: [node('drivetrain', 'teleop'), node('shooter', null), node('intake', 'auto')],
    topics: []
  };
}

function populatedStudio(): Studio {
  const frames = populatedFrames();
  const events: RobotEvent[] = Array.from({length: 10}, (_, i) => ({
    id: `e${i}`, t_ns: String(1735689600000000000n + BigInt(i) * 1000000000n),
    severity: i === 3 ? 'error' : i === 5 ? 'warn' : 'info',
    category: 'hardware', title: `event-${i}`
  }));
  return createStaticStudio({
    frames,
    system: {graph: graph(), rates: {}, error: '', declared: null, declaredError: ''},
    events
  });
}

// --- rendering --------------------------------------------------------------

test('the robot panel renders every card as an explicit no-data state', () => {
  const html = renderPanel(createStaticStudio({}));
  assert.ok(html.includes('no Driver Station data'));
  assert.ok(html.includes('no phase channel discovered'));
  assert.ok(html.includes('no power data'));
  assert.ok(html.includes('no link data'));
  assert.ok(html.includes('no estimator discovered'));
  assert.ok(html.includes('no events yet'));
  assert.ok(html.includes('no subsystems discovered'));
  assert.ok(html.includes('no cameras discovered'));
});

test('the robot panel renders a populated studio at a glance', () => {
  const html = renderPanel(populatedStudio());
  // Driver Station.
  assert.ok(html.includes('AUTONOMOUS'), 'mode badge');
  assert.ok(html.includes('blue'), 'alliance label');
  // Routine.
  assert.ok(html.includes('FEED'), 'current phase label');
  assert.ok(html.includes('IDLE'), 'previous phase label');
  // Power.
  assert.ok(html.includes('12.16 V'), 'battery voltage');
  assert.ok(html.includes('34.0 %'), 'CAN utilization');
  // Link.
  assert.ok(html.includes('0, 2'), 'motor fault mask bits');
  assert.ok(html.includes('gateway rejected command'), 'gateway counter label');
  assert.ok(html.includes('+10'), 'gateway delta');
  // Pose.
  assert.ok(html.includes('1.200 m'), 'estimator x');
  assert.ok(html.includes('40.1°'), 'heading in degrees');
  // Subsystems: every discovered tile renders its name, state and source.
  assert.ok(html.includes('drivetrain'), 'drivetrain tile');
  assert.ok(html.includes('← teleop'), 'drivetrain target source');
  assert.ok(html.includes('shooter'), 'shooter tile');
  assert.ok(html.includes('← none'), 'shooter has no target source');
  assert.ok(html.includes('intake'), 'intake tile');
  assert.ok(html.includes('inhibit: GATEWAY_NOT_READY'), 'intake highlighted enum');
  assert.ok(html.includes('← auto'), 'intake target source');
  assert.ok(html.includes('autonomous'), 'autonomous (phase) tile');
  assert.ok(html.includes('phase: FEED'), 'autonomous phase enum highlighted');
  // Cameras: every discovered camera tile renders name, health, fps and tags.
  assert.ok(html.includes('front_left_camera'), 'front_left tile');
  assert.ok(html.includes('25 fps'), 'front_left fps');
  assert.ok(html.includes('3 tags'), 'front_left tags');
  assert.ok(html.includes('front_right_camera'), 'front_right tile');
  assert.ok(html.includes('20 fps'), 'front_right fps');
  assert.ok(html.includes('2 tags'), 'front_right tags');
  assert.ok(html.includes('rear_left_camera'), 'rear_left tile');
  assert.ok(html.includes('33 fps'), 'rear_left fps');
  assert.ok(html.includes('4 tags'), 'rear_left tags');
  assert.ok(html.includes('unhealthy'), 'rear_left unhealthy state');
  assert.ok(html.includes('rear_right_camera'), 'rear_right tile');
  assert.ok(html.includes('10 fps'), 'rear_right fps');
  assert.ok(html.includes('0 tags'), 'rear_right tags');
});

test('recent events show the last eight, severity-colored, seekable', () => {
  const html = renderPanel(populatedStudio());
  assert.ok(html.includes('event-9'));
  assert.ok(html.includes('event-2'));
  assert.ok(!html.includes('event-0'), 'oldest event dropped');
  assert.ok(!html.includes('event-1'), 'second oldest dropped');
  assert.ok(html.includes('event-error'), 'error severity class');
  assert.ok(html.includes('event-warn'), 'warn severity class');
});
