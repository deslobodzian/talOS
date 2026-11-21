// Subsystems panel: pairing heuristic, config parsing, and the rendered
// latency and demand-vs-measured error of a populated (2026-shaped) studio.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import type {Snapshot} from '../src/core';
import {createStaticStudio, type Studio, StudioProvider} from '../src/data/studio';
import {subsystemsPanel} from '../src/panels/subsystems';
import {pairTargets} from '../src/panels/subsystems/pairing';
import {channelDisplayName} from '../src/ui/channel';
import type {
  SourceKind,
  SystemGraph,
  SystemNode,
  SystemSelection,
  SystemSource
} from '../src/system';

// --- pairing heuristic ------------------------------------------------------

test('pairTargets finds true demand/measured pairs', () => {
  // Shooter: target vs flywheel.
  assert.deepEqual(
      pairTargets([
        {name: 'shooter/state/target_velocity_rps', unit: 'rps'},
        {name: 'shooter/state/flywheel_velocity_rps', unit: 'rps'}
      ]),
      [{
        target: 'shooter/state/target_velocity_rps',
        measured: 'shooter/state/flywheel_velocity_rps',
        stem: 'velocity_rps',
        unit: 'rps'
      }]);
  // Intake: requested vs roller.
  assert.deepEqual(
      pairTargets([
        {name: 'intake/state/requested_velocity_rps', unit: 'rps'},
        {name: 'intake/state/roller_velocity_rps', unit: 'rps'}
      ]),
      [{
        target: 'intake/state/requested_velocity_rps',
        measured: 'intake/state/roller_velocity_rps',
        stem: 'velocity_rps',
        unit: 'rps'
      }]);
});

test('pairTargets does not pair unrelated channels', () => {
  // Wrong unit, a boolean (no unit) and a channel with no demand marker: no pair.
  assert.deepEqual(
      pairTargets([
        {name: 'shooter/state/target_velocity_rps', unit: 'rps'},
        {name: 'shooter/state/battery_v', unit: 'V'},
        {name: 'shooter/state/beam_broken', unit: ''},
        {name: 'shooter/state/flywheel_velocity_rps', unit: 'm/s'}
      ]),
      []);
  // A measured channel with the right unit but no shared stem is not a pair.
  assert.deepEqual(
      pairTargets([
        {name: 'shooter/state/target_velocity_rps', unit: 'rps'},
        {name: 'shooter/state/position_rps', unit: 'rps'}
      ]),
      []);
  // A leaf that is only a marker has no stem to match.
  assert.deepEqual(pairTargets([{name: 'x/state/target', unit: 'rps'}]), []);
});

test('channelDisplayName keeps the nested field path after owner/role', () => {
  // module0/distance_m and module1/distance_m must not both read distance_m.
  assert.equal(channelDisplayName('drivetrain/state/module0/distance_m'),
               'module0/distance_m');
});

// --- config parsing ---------------------------------------------------------

test('subsystems config parsing accepts valid input and cleans malformed input',
     () => {
       assert.deepEqual(subsystemsPanel.parseConfig({selected: ['a', 'b']}),
                        {selected: ['a', 'b']});
       assert.deepEqual(subsystemsPanel.parseConfig(null), {selected: null});
       assert.deepEqual(subsystemsPanel.parseConfig('nope'), {selected: null});
       assert.deepEqual(subsystemsPanel.parseConfig({selected: 'x'}),
                        {selected: null});
       // Non-string entries are dropped.
       assert.deepEqual(subsystemsPanel.parseConfig({selected: ['a', 2, null]}),
                        {selected: ['a']});
       assert.deepEqual(subsystemsPanel.parseConfig(subsystemsPanel.defaultConfig),
                        subsystemsPanel.defaultConfig);
     });

// --- SSR rendering ----------------------------------------------------------

const UNITS: Record<string, string> = {
  'shooter/state/enabled': '',
  'shooter/state/target_velocity_rps': 'rps',
  'shooter/state/flywheel_velocity_rps': 'rps',
  'shooter/state/at_speed': '',
  'shooter/state/beam_broken': '',
  'intake/state/enabled': '',
  'intake/state/requested_velocity_rps': 'rps',
  'intake/state/roller_velocity_rps': 'rps',
  'intake/state/beam_broken': '',
  'intake/state/inhibit': '',
  'drivetrain/state/enabled': '',
  'drivetrain/state/vx_mps': 'm/s',
  'drivetrain/state/yaw_rad': 'rad',
  'drivetrain/state/yaw_valid': ''
};

const LABELS: Record<string, string[]> = {
  'intake/state/inhibit':
      ['DRIVE', 'GATEWAY_NOT_READY', 'TARGET_DISABLED', 'TARGET_STALE',
       'FEEDBACK_INVALID', 'DEMAND_NONFINITE']
};

function frame(seq: string, ts: string,
               channels: Record<string, number>): Snapshot {
  return {
    stream_id: 'stream0',
    sequence_id: seq,
    timestamp_ns: ts,
    dictionary_id: 0,
    chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
    channels,
    units: UNITS,
    labels: LABELS
  };
}

function source(id: number, kind: SourceKind, name: string, period_ns: string,
                max_latency_ns: string): SystemSource {
  return {
    id, kind, name, message_bytes: 8, alignment: 0, period_ns, events: '0',
    dropped: '0', sequence: '0', last_monotonic_ns: '0',
    last_latency_ns: max_latency_ns, max_latency_ns, external: false,
    optional: false, observer: false
  };
}

function node(name: string, sources: SystemSource[],
              selections: SystemSelection[]): SystemNode {
  return {
    name, target: name + '/state', slot: 0, pid: '1', session_id: '0',
    generation: '0', start_wall_ns: '0', heartbeat_wall_ns: '1000000000',
    dispatch_count: '0', declared_sources: sources.length, alive: true,
    simulation: false, replay: false, sources, selections
  };
}

function populatedStudio(passiveOnly = false): Studio {
  const frames = [
    frame('1', '1000000000', {
      'shooter/state/enabled': 0, 'shooter/state/target_velocity_rps': 60,
      'shooter/state/flywheel_velocity_rps': 50, 'shooter/state/at_speed': 0,
      'shooter/state/beam_broken': 0, 'intake/state/enabled': 0,
      'intake/state/requested_velocity_rps': 32,
      'intake/state/roller_velocity_rps': 30, 'intake/state/beam_broken': 0,
      'intake/state/inhibit': 2, 'drivetrain/state/enabled': 0,
      'drivetrain/state/vx_mps': 0, 'drivetrain/state/yaw_rad': 0,
      'drivetrain/state/yaw_valid': 1
    }),
    frame('2', '1001000000', {
      'shooter/state/enabled': 0, 'shooter/state/target_velocity_rps': 60,
      'shooter/state/flywheel_velocity_rps': 55, 'shooter/state/at_speed': 0,
      'shooter/state/beam_broken': 0, 'intake/state/enabled': 0,
      'intake/state/requested_velocity_rps': 32,
      'intake/state/roller_velocity_rps': 31, 'intake/state/beam_broken': 0,
      'intake/state/inhibit': 1, 'drivetrain/state/enabled': 0,
      'drivetrain/state/vx_mps': 1.5, 'drivetrain/state/yaw_rad': 0.1,
      'drivetrain/state/yaw_valid': 1
    }),
    frame('3', '1002000000', {
      'shooter/state/enabled': 1, 'shooter/state/target_velocity_rps': 60,
      'shooter/state/flywheel_velocity_rps': 58, 'shooter/state/at_speed': 1,
      'shooter/state/beam_broken': 0, 'intake/state/enabled': 1,
      'intake/state/requested_velocity_rps': 32,
      'intake/state/roller_velocity_rps': 32, 'intake/state/beam_broken': 0,
      'intake/state/inhibit': 0, 'drivetrain/state/enabled': 1,
      'drivetrain/state/vx_mps': 3.0, 'drivetrain/state/yaw_rad': 0.2,
      'drivetrain/state/yaw_valid': 1
    })
  ];

  const graph: SystemGraph = {
    version: 2,
    wall_ns: '1002000000',
    registry: {
      available: true, node_capacity: 256, source_capacity: 512,
      liveness_timeout_ns: '1000000000'
    },
    bridge: {
      topic: 'system', clients: 0, frames_forwarded: '0', invalid: '0',
      ws_drops: '0'
    },
    nodes: [
      node('shooter',
           [source(0, 'TIMER', 'loop', '10000000', '1000000'),
            source(1, 'SENDER', 'state', '10000000', '1500000')],
           [{sources: ['auto', 'teleop'], selected: 'teleop'}]),
      node('intake', [source(0, 'TIMER', 'loop', '10000000', '1000000')],
           [{sources: ['auto', 'teleop', 'test'], selected: null}]),
      node('drivetrain',
           [source(0, 'TIMER', 'loop', '10000000', '2000000')],
           [{sources: ['teleop'], selected: 'teleop'}])
    ],
    topics: []
  };
  if (passiveOnly) {
    for (const node of graph.nodes)
      node.sources = [
        source(0, 'SENDER', 'state', '0', '0'),
        source(1, 'FETCHER', 'target', '0', '0')
      ];
  }

  return createStaticStudio({
    frames,
    playback: {
      mode: 'PAUSED', cursor_ns: frames[2].timestamp_ns,
      bounds: {start: frames[0].timestamp_ns, end: frames[2].timestamp_ns},
      stats: {skipped: 0, late: 0, invalid: 0}, status: 'Demo',
      connection: 'demo'
    },
    system: {
      graph,
      rates: {'shooter 0': 100, 'shooter 1': 200, 'intake 0': 80,
              'drivetrain 0': 120}
    }
  });
}

const renderPanel = (studio: Studio): string =>
    renderToStaticMarkup(
        <StudioProvider studio={studio}>
          <subsystemsPanel.component config={subsystemsPanel.defaultConfig}
                                     setConfig={() => {}} instanceId="test"/>
        </StudioProvider>);

test('subsystem latency stays unknown for sender and fetcher sources', () => {
  const text = renderPanel(populatedStudio(true)).replace(/<[^>]*>/g, '');
  assert.equal((text.match(/latency—/g) ?? []).length, 3);
});

test('a demand/measured pair shows measured minus demand at the cursor', () => {
  // Shooter at the cursor: flywheel 58 rps against a 60 rps target.
  assert.ok(renderPanel(populatedStudio()).includes('Δ -2.000 rps'));
});

// REQ-OPS-55: subsystem cards with status/armed + status/blocking_kind display
// the held joystick input while disarmed, for both teleop and custom operator
// nodes, and hide the notice once armed.
test('subsystem cards render operator arming blocker when disarmed and clear when armed', () => {
  const armingUnits: Record<string, string> = {
    'teleop/status/armed': '',
    'teleop/status/alert_warning_disarmed': '',
    'teleop/status/blocking_kind': '',
    'teleop/status/blocking_joystick_index': '',
    'teleop/status/blocking_input_index': '',
    'operator/status/armed': '',
    'operator/status/blocking_kind': '',
    'operator/status/blocking_joystick_index': '',
    'operator/status/blocking_input_index': '',
  };
  const armingLabels: Record<string, string[]> = {
    'teleop/status/blocking_kind': ['NONE', 'AXIS', 'BUTTON', 'POV', 'OTHER'],
    'operator/status/blocking_kind': ['NONE', 'AXIS', 'BUTTON', 'POV', 'OTHER'],
  };
  const renderWith = (channels: Record<string, number>): string => {
    const snap: Snapshot = {
      stream_id: 'stream0',
      sequence_id: '1',
      timestamp_ns: '1000000000',
      dictionary_id: 0,
      chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
      channels,
      units: armingUnits,
      labels: armingLabels,
    };
    const studio = createStaticStudio({
      frames: [snap],
      playback: {
        mode: 'PAUSED', cursor_ns: snap.timestamp_ns,
        bounds: {start: snap.timestamp_ns, end: snap.timestamp_ns},
        stats: {skipped: 0, late: 0, invalid: 0}, status: 'Demo',
        connection: 'demo',
      },
    });
    return renderPanel(studio);
  };

  // Disarmed: teleop blocked on joystick 0 axis 1 (even when disabled/auto
  // with alert_warning_disarmed === 0), custom operator on joystick 1 button 3.
  const disarmedHtml = renderWith({
    'teleop/status/armed': 0,
    'teleop/status/alert_warning_disarmed': 0,
    'teleop/status/blocking_kind': 1,
    'teleop/status/blocking_joystick_index': 0,
    'teleop/status/blocking_input_index': 1,
    'operator/status/armed': 0,
    'operator/status/blocking_kind': 2,
    'operator/status/blocking_joystick_index': 1,
    'operator/status/blocking_input_index': 3,
  });
  assert.ok(disarmedHtml.includes('Disarmed — joystick 0 axis 1 held'), disarmedHtml);
  assert.ok(disarmedHtml.includes('Disarmed — joystick 1 button 3 held'), disarmedHtml);

  // Disabled/auto with neutral inputs (alert_warning_disarmed === 0 and
  // blocking_kind === NONE): suppress the yellow "Disarmed — release inputs to
  // arm" banner.
  const disabledNeutralHtml = renderWith({
    'teleop/status/armed': 0,
    'teleop/status/alert_warning_disarmed': 0,
    'teleop/status/blocking_kind': 0,
    'teleop/status/blocking_joystick_index': -1,
    'teleop/status/blocking_input_index': -1,
    'operator/status/armed': 1,
    'operator/status/blocking_kind': 0,
    'operator/status/blocking_joystick_index': -1,
    'operator/status/blocking_input_index': -1,
  });
  assert.ok(!disabledNeutralHtml.includes('Disarmed'), disabledNeutralHtml);

  // Enabled in teleop and disarmed with NONE (alert_warning_disarmed === 1)
  // still shows the fallback banner.
  const teleopNoneHtml = renderWith({
    'teleop/status/armed': 0,
    'teleop/status/alert_warning_disarmed': 1,
    'teleop/status/blocking_kind': 0,
    'teleop/status/blocking_joystick_index': -1,
    'teleop/status/blocking_input_index': -1,
    'operator/status/armed': 1,
    'operator/status/blocking_kind': 0,
    'operator/status/blocking_joystick_index': -1,
    'operator/status/blocking_input_index': -1,
  });
  assert.ok(teleopNoneHtml.includes('Disarmed — release inputs to arm'), teleopNoneHtml);

  // Disarmed on POV and with NONE blocker (node without alert_warning_disarmed).
  const povHtml = renderWith({
    'teleop/status/armed': 0,
    'teleop/status/alert_warning_disarmed': 1,
    'teleop/status/blocking_kind': 3,
    'teleop/status/blocking_joystick_index': 0,
    'teleop/status/blocking_input_index': 1,
    'operator/status/armed': 0,
    'operator/status/blocking_kind': 0,
    'operator/status/blocking_joystick_index': -1,
    'operator/status/blocking_input_index': -1,
  });
  assert.ok(povHtml.includes('Disarmed — joystick 0 POV 1 held'), povHtml);
  assert.ok(povHtml.includes('Disarmed — release inputs to arm'), povHtml);

  // Armed: no disarmed banner shown.
  const armedHtml = renderWith({
    'teleop/status/armed': 1,
    'teleop/status/alert_warning_disarmed': 0,
    'teleop/status/blocking_kind': 0,
    'teleop/status/blocking_joystick_index': -1,
    'teleop/status/blocking_input_index': -1,
    'operator/status/armed': 1,
    'operator/status/blocking_kind': 0,
    'operator/status/blocking_joystick_index': -1,
    'operator/status/blocking_input_index': -1,
  });
  assert.ok(!armedHtml.includes('Disarmed'), armedHtml);
});
