import assert from 'node:assert/strict';
import {test} from 'node:test';

import {discoverCameras, discoverEstimators, discoverSubsystems} from '../src/data/discovery';
import type {ChannelInfo} from '../src/data/types';
import type {SystemGraph, SystemNode} from '../src/system';

const info = (name: string): ChannelInfo => ({name, unit: '', labels: []});

function node(name: string, selections: SystemNode['selections']): SystemNode {
  return {
    name, target: 't', slot: 0, pid: '1', session_id: '1', generation: '1',
    start_wall_ns: '0', heartbeat_wall_ns: '0', dispatch_count: '0',
    declared_sources: 0, alive: true, simulation: false, replay: false,
    sources: [], selections
  };
}

function graph(nodes: SystemNode[]): SystemGraph {
  return {
    version: 2, wall_ns: '0',
    registry: {available: true, node_capacity: 32, source_capacity: 64, liveness_timeout_ns: '5000000000'},
    bridge: {topic: '/talos/telemetry', clients: 0, frames_forwarded: '0', invalid: '0', ws_drops: '0'},
    nodes, topics: []
  };
}

// A realistic 2026-like dictionary, spelled with the telemetry channel facts
// from the contract: reserved owners, a mechanism, a camera and an estimator.
const dictionary: ChannelInfo[] = [
  info('hw/state/driver_station/connected'),
  info('hw/status/battery_v'),
  info('sim/state/pose/x_m'),
  info('derived/pose_error_m'),
  info('drivetrain/state/velocity'),
  info('drivetrain/state/position'),
  info('drivetrain/status/temperature'),
  info('front_left_camera/state/pose/x_m'),
  info('front_left_camera/state/x_std_m'),
  info('front_left_camera/state/tag_count'),
  info('front_left_camera/status/latency_ms'),
  info('pose_estimator/state/valid'),
  info('pose_estimator/state/observation_accepted_count'),
  info('pose_estimator/state/observation_rejected_count')
];

test('reserved owners are excluded from subsystems', () => {
  const names = discoverSubsystems(dictionary, null).map(s => s.name);
  assert.deepEqual(names, ['drivetrain']);
});

test('cameras and estimators are grouped by shape, not name', () => {
  const cameras = discoverCameras(dictionary);
  assert.deepEqual(cameras.map(c => c.name), ['front_left_camera']);
  assert.ok(cameras[0].stateChannels.includes('front_left_camera/state/pose/x_m'));
  assert.ok(cameras[0].statusChannels.includes('front_left_camera/status/latency_ms'));

  const estimators = discoverEstimators(dictionary);
  assert.deepEqual(estimators.map(e => e.name), ['pose_estimator']);
  assert.ok(estimators[0].channels.includes('pose_estimator/state/valid'));
  assert.ok(estimators[0].channels.includes('pose_estimator/state/observation_accepted_count'));
});

test('subsystem state and status channels are split by role', () => {
  const subsystems = discoverSubsystems(dictionary, null);
  assert.deepEqual(subsystems[0].stateChannels.sort(), [
    'drivetrain/state/position', 'drivetrain/state/velocity'
  ]);
  assert.deepEqual(subsystems[0].statusChannels, ['drivetrain/status/temperature']);
});

test('selection comes from the node whose name equals the owner, else null', () => {
  const g = graph([node('drivetrain', [{sources: ['teleop', 'auto'], selected: 'teleop'}])]);
  assert.deepEqual(discoverSubsystems(dictionary, g)[0].selection,
                   {sources: ['teleop', 'auto'], selected: 'teleop'});
  assert.equal(discoverSubsystems(dictionary, graph([node('other', [])]))[0].selection, null);
});
