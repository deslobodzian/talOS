import assert from 'node:assert/strict';
import {test} from 'node:test';

import type {Snapshot} from '../src/core';
import {driverStationMode, robotStatus} from '../src/data/robot';
import type {SystemGraph, SystemNode} from '../src/system';
import type {SystemState} from '../src/data/types';

function snap(channels: Record<string, number>, labels: Record<string, string[]> = {}): Snapshot {
  return {
    stream_id: 'stream-a',
    sequence_id: '1',
    timestamp_ns: '1000000000',
    dictionary_id: 1,
    chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
    channels,
    units: {},
    labels
  };
}

const DS = 'hw/state/driver_station/';
const noSystem: SystemState =
    {graph: null, stale: true, rates: {}, error: '', declared: null, declaredError: ''};

function node(name: string, alive: boolean): SystemNode {
  return {
    name, target: 't', slot: 0, pid: '1', session_id: '1', generation: '1',
    start_wall_ns: '0', heartbeat_wall_ns: '0', dispatch_count: '0',
    declared_sources: 0, alive, simulation: false, replay: false,
    sources: [], selections: []
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

test('mode precedence: estop > disconnected > disabled > mode', () => {
  assert.equal(driverStationMode(snap({
    [DS + 'connected']: 1, [DS + 'enabled']: 1, [DS + 'autonomous']: 1, [DS + 'estop']: 1
  })), 'estop');
  assert.equal(driverStationMode(snap({
    [DS + 'connected']: 0, [DS + 'enabled']: 1, [DS + 'teleop']: 1
  })), 'disconnected');
  assert.equal(driverStationMode(snap({
    [DS + 'connected']: 1, [DS + 'enabled']: 0, [DS + 'autonomous']: 1
  })), 'disabled');
  assert.equal(driverStationMode(snap({
    [DS + 'connected']: 1, [DS + 'enabled']: 1, [DS + 'teleop']: 1
  })), 'teleop');
  assert.equal(driverStationMode(snap({
    [DS + 'connected']: 1, [DS + 'enabled']: 1, [DS + 'test']: 1
  })), 'test');
  assert.equal(driverStationMode(snap({
    [DS + 'connected']: 1, [DS + 'enabled']: 1, [DS + 'autonomous']: 1
  })), 'autonomous');
});

test('robotStatus nulls when channels absent', () => {
  const status = robotStatus(snap({'hw/status/battery_v': 12.3}), noSystem);
  assert.equal(status.ds.mode, null);
  assert.equal(status.ds.enabled, null);
  assert.equal(status.ds.matchTime_s, null);
  assert.equal(status.ds.fms, null);
  assert.equal(status.ds.alliance, null);
  assert.equal(status.power.battery_v, 12.3);
  assert.equal(status.power.brownout, null);
  assert.equal(status.power.can_frac, null);
  assert.equal(status.power.healthValid, null);
  assert.equal(status.link.stateAge_us, null);
  assert.equal(status.link.configured, null);
  assert.equal(status.link.faultMask, null);
  assert.equal(status.link.timedOutMask, null);
  assert.equal(status.estimator, null);
  assert.equal(status.nodes, null);
});

test('live nodes are counted only from a fresh graph', () => {
  const nodes = (stale: boolean) => robotStatus(snap({}), {
    graph: graph([node('a', true), node('b', false)]),
    stale, rates: {}, error: '', declared: null, declaredError: ''
  }).nodes;
  assert.deepEqual(nodes(false), {alive: 1, total: 2});
  assert.deepEqual(nodes(true), {alive: null, total: 2});
});

test('alliance is read through its enum labels; out of range is null', () => {
  const alliance = (value: number) => robotStatus(
      snap({[DS + 'alliance']: value}, {[DS + 'alliance']: ['red', 'blue']}),
      noSystem).ds.alliance;
  assert.equal(alliance(1), 'blue');
  assert.equal(alliance(5), null);
});
