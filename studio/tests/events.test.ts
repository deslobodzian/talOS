import assert from 'node:assert/strict';
import {test} from 'node:test';

import type {Snapshot} from '../src/core';
import {EventEngine} from '../src/data/events';
import {FRAME_DETECTORS} from '../src/data/detectors';
import type {RobotEvent} from '../src/data/types';
import type {SessionManifest} from '../src/data/manifest';
import type {SystemGraph, SystemNode, SystemSelection} from '../src/system';

function frame(seq: number, ts: bigint, channels: Record<string, number>,
               opts: {streamId?: string; labels?: Record<string, string[]>} = {}): Snapshot {
  return {
    stream_id: opts.streamId ?? 'a',
    sequence_id: String(seq),
    timestamp_ns: String(ts),
    dictionary_id: 1,
    chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
    channels,
    units: {},
    labels: opts.labels ?? {}
  };
}

const DS = 'hw/state/driver_station/';

function sysNode(name: string, alive: boolean, overrides: Partial<SystemNode> = {}): SystemNode {
  return {
    name, target: 't', slot: 0, pid: '10', session_id: '1', generation: '1',
    start_wall_ns: '0', heartbeat_wall_ns: '0', dispatch_count: '0',
    declared_sources: 0, alive, simulation: false, replay: false,
    sources: [], selections: [], ...overrides
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

test('ds-mode reports one canonical event per transition', () => {
  const engine = new EventEngine();
  const ds = (connected: number, enabled: number, mode?: string) => {
    const channels: Record<string, number> = {
      [DS + 'connected']: connected, [DS + 'enabled']: enabled
    };
    if (mode) channels[DS + mode] = 1;
    return channels;
  };
  // First observation (disabled) is not a transition: no event.
  engine.pushFrame(frame(1, 1n, ds(1, 0)));
  // disabled -> autonomous
  engine.pushFrame(frame(2, 2n, ds(1, 1, 'autonomous')));
  // autonomous -> teleop
  engine.pushFrame(frame(3, 3n, ds(1, 1, 'teleop')));
  // teleop -> estop (error)
  engine.pushFrame(frame(4, 4n, {...ds(1, 1, 'teleop'), [DS + 'estop']: 1}));
  // estop -> disconnected (warn, detail names where it came from)
  engine.pushFrame(frame(5, 5n, ds(0, 1, 'teleop')));

  const events = engine.events.filter(e => e.id.startsWith('ds-mode:'));
  assert.equal(events.length, 4);
  assert.equal(events[0].severity, 'info');
  assert.equal(events[2].severity, 'error');
  assert.equal(events[3].severity, 'warn');
});


test('a change across a channel gap is still detected', () => {
  const engine = new EventEngine();
  engine.pushFrame(frame(1, 1n, {'hw/status/brownout': 0}));
  // brownout disappears for five frames, then returns already brownout.
  for (let i = 2; i <= 6; i++) engine.pushFrame(frame(i, BigInt(i), {}));
  engine.pushFrame(frame(7, 7n, {'hw/status/brownout': 1}));
  const brownout = engine.events.filter(e => e.id.startsWith('power-brownout:'));
  assert.equal(brownout.length, 1);
  assert.equal(brownout[0].severity, 'error');
});

test('brownout start is error and recovery is info', () => {
  const engine = new EventEngine();
  engine.pushFrame(frame(1, 1n, {'hw/status/brownout': 0}));
  engine.pushFrame(frame(2, 2n, {'hw/status/brownout': 1}));
  engine.pushFrame(frame(3, 3n, {'hw/status/brownout': 0}));
  const brownout = engine.events.filter(e => e.id.startsWith('power-brownout:'));
  assert.equal(brownout.length, 2);
  assert.equal(brownout[0].severity, 'error');
  assert.equal(brownout[1].severity, 'info');
});

test('fault mask changes raise a warning', () => {
  const engine = new EventEngine();
  engine.pushFrame(frame(1, 1n, {'hw/status/motor_fault_mask': 0}));
  engine.pushFrame(frame(2, 2n, {'hw/status/motor_fault_mask': 5}));
  const event = engine.events.find(e => e.id.startsWith('link-fault-mask:'));
  assert.ok(event);
  assert.equal(event.severity, 'warn');
});

test('identity and gateway counters report runs with their own severities', () => {
  const engine = new EventEngine();
  engine.pushFrame(frame(1, 1n, {'hw/status/identity_rejected_count': 1, 'hw/status/gateway_rx_count': 7}));
  engine.pushFrame(frame(2, 2n, {'hw/status/identity_rejected_count': 2, 'hw/status/gateway_rx_count': 7}));
  engine.pushFrame(frame(3, 3n, {'hw/status/identity_rejected_count': 2, 'hw/status/gateway_rx_count': 9}));
  const identity = engine.events.find(e => e.id.startsWith('counter-identity:'));
  assert.ok(identity);
  assert.equal(identity.severity, 'warn');
  const gateway = engine.events.find(e => e.id.startsWith('counter-gateway:'));
  assert.ok(gateway);
  assert.equal(gateway.severity, 'warn');
});

test('a steady counter run reports one start and stop with mode-dependent severity', () => {
  for (const enabled of [0, 1]) {
    const engine = new EventEngine();
    const ds = {[DS + 'connected']: 1, [DS + 'enabled']: enabled, [DS + 'teleop']: 1};
    for (let i = 0; i < 800; i++) {
      const value = i < 720 ? i : 719;
      engine.pushFrame(frame(i, BigInt(i) * 12_500_000n, {
        'hw/status/gateway_rejected_command_count': value, ...ds
      }));
    }
    const events = engine.events.filter(e => e.id.startsWith('counter-gateway:'));
    assert.deepEqual(events.map(e => e.severity), [enabled ? 'warn' : 'info', 'info']);
  }
});
test('a single increment reports a start event and, one second later, a stop', () => {
  const engine = new EventEngine();
  const ds = {[DS + 'connected']: 1, [DS + 'enabled']: 0};
  engine.pushFrame(frame(0, 0n, {'hw/status/gateway_rejected_command_count': 5, ...ds}));
  engine.pushFrame(frame(1, 1_000_000_000n, {'hw/status/gateway_rejected_command_count': 6, ...ds}));
  engine.pushFrame(frame(2, 2_000_000_000n, {'hw/status/gateway_rejected_command_count': 6, ...ds}));
  const events = engine.events.filter(e => e.id.startsWith('counter-gateway:'));
  assert.equal(events.length, 2);
});

test('a run starting on the last enabled frame of a disable transition is info', () => {
  const engine = new EventEngine();
  const enabled = {[DS + 'connected']: 1, [DS + 'enabled']: 1, [DS + 'teleop']: 1};
  engine.pushFrame(frame(0, 0n, {'hw/status/gateway_rejected_command_count': 5, ...enabled}));
  engine.pushFrame(frame(1, 12_500_000n, {'hw/status/gateway_rejected_command_count': 6, ...enabled}));
  // Disable: the in-flight commands are refused on this frame.
  engine.pushFrame(frame(2, 25_000_000n, {
    'hw/status/gateway_rejected_command_count': 7,
    [DS + 'connected']: 1, [DS + 'enabled']: 0
  }));
  const events = engine.events.filter(e => e.id.startsWith('counter-gateway:'));
  assert.equal(events.length, 1);
  assert.equal(events[0].severity, 'info');
  assert.equal(events[0].t_ns, '12500000');  // the run's true start time
});

test('a fully-enabled run of a second is one warn start', () => {
  const engine = new EventEngine();
  const ds = {[DS + 'connected']: 1, [DS + 'enabled']: 1, [DS + 'teleop']: 1};
  engine.pushFrame(frame(0, 0n, {'hw/status/gateway_rejected_command_count': 0, ...ds}));
  for (let i = 1; i <= 80; i++)
    engine.pushFrame(frame(i, BigInt(i) * 12_500_000n, {
      'hw/status/gateway_rejected_command_count': i, ...ds
    }));
  const events = engine.events.filter(e => e.id.startsWith('counter-gateway:'));
  assert.equal(events.length, 1);
  assert.equal(events[0].severity, 'warn');
  assert.equal(events[0].t_ns, '12500000');
});

test('a late-classified start keeps events ordered', () => {
  const engine = new EventEngine();
  const enabled = {[DS + 'connected']: 1, [DS + 'enabled']: 1, [DS + 'teleop']: 1};
  engine.pushFrame(frame(0, 0n, {'hw/status/gateway_rejected_command_count': 0, ...enabled}));
  engine.pushFrame(frame(1, 12_500_000n, {'hw/status/gateway_rejected_command_count': 1, ...enabled}));
  // The disable transition emits a ds-mode "Disabled" event at 25 ms and, on
  // the same frame, classifies the deferred start at its true 12.5 ms stamp.
  engine.pushFrame(frame(2, 25_000_000n, {
    'hw/status/gateway_rejected_command_count': 2,
    [DS + 'connected']: 1, [DS + 'enabled']: 0
  }));

  const times = engine.events.map(e => BigInt(e.t_ns));
  for (let i = 1; i < times.length; i++) assert.ok(times[i - 1] <= times[i]);

  const info = engine.events.find(e => e.id.startsWith('counter-gateway:'));
  assert.ok(info);
  assert.equal(info.t_ns, '12500000');
  const disabled = engine.events.find(e => e.id.startsWith('ds-mode:'));
  assert.ok(disabled);
  assert.equal(disabled.t_ns, '25000000');
  // The deferred start sorts before the disable transition that classified it.
  assert.ok(engine.events.indexOf(info) < engine.events.indexOf(disabled));
});

test('healthy and estimator-valid warn only on the false edge', () => {
  const engine = new EventEngine();
  engine.pushFrame(frame(1, 1n, {
    'intake/status/healthy': 1,
    'pose_estimator/state/valid': 1,
    'pose_estimator/state/observation_accepted_count': 10
  }));
  engine.pushFrame(frame(2, 2n, {
    'intake/status/healthy': 0,
    'pose_estimator/state/valid': 0,
    'pose_estimator/state/observation_accepted_count': 10
  }));
  engine.pushFrame(frame(3, 3n, {
    'intake/status/healthy': 1,
    'pose_estimator/state/valid': 1,
    'pose_estimator/state/observation_accepted_count': 10
  }));
  const healthy = engine.events.filter(e => e.id.startsWith('health-edge:'));
  const valid = engine.events.filter(e => e.id.startsWith('estimator-valid-edge:'));
  assert.equal(healthy.length, 1);
  assert.equal(valid.length, 1);
  assert.equal(healthy[0].severity, 'warn');
  assert.equal(valid[0].severity, 'warn');
});

test('enum transitions emit an informational event', () => {
  const engine = new EventEngine();
  engine.pushFrame(frame(1, 1n, {mode: 0}, {labels: {mode: ['disabled', 'teleop', 'auto']}}));
  engine.pushFrame(frame(2, 2n, {mode: 2}, {labels: {mode: ['disabled', 'teleop', 'auto']}}));
  const event = engine.events.find(e => e.id.startsWith('enum-change:'));
  assert.ok(event);
  assert.equal(event.severity, 'info');
});

test('a new stream reports a producer restart, and events stay ordered', () => {
  const engine = new EventEngine();
  engine.pushFrame(frame(1, 1n, {[DS + 'connected']: 1}, {streamId: 'a'}));
  engine.pushFrame(frame(2, 2n, {[DS + 'connected']: 0}, {streamId: 'a'}));
  engine.clear();
  engine.pushFrame(frame(1, 1n, {[DS + 'connected']: 1}, {streamId: 'b'}));
  const stream = engine.events.find(e => e.category === 'stream');
  assert.ok(stream);
  assert.equal(stream.t_ns, '1');

  // Ascending order.
  const times = engine.events.map(e => BigInt(e.t_ns));
  for (let i = 1; i < times.length; i++) assert.ok(times[i - 1] <= times[i]);
});

test('system detectors: liveness, restart and target selection', () => {
  const engine = new EventEngine();
  engine.pushFrame(frame(1, 1n, {[DS + 'connected']: 1}));
  const live = graph([sysNode('drivetrain', true, {
    selections: [{sources: ['teleop', 'auto'], selected: 'teleop'}]
  })]);
  engine.pushSystem(live);
  const dead = graph([sysNode('drivetrain', false, {
    selections: [{sources: ['teleop', 'auto'], selected: null}]
  })]);
  engine.pushSystem(dead);
  const restarted = graph([sysNode('drivetrain', true, {
    pid: '99', selections: [{sources: ['teleop', 'auto'], selected: 'auto'}]
  })]);
  engine.pushSystem(restarted);

  const alive = engine.events.filter(e => e.id.startsWith('node-alive:'));
  const restart = engine.events.filter(e => e.id.startsWith('node-restarted:'));
  const selection = engine.events.filter(e => e.id.startsWith('target-selection:'));
  assert.equal(alive.length, 2);
  assert.equal(alive[0].severity, 'error');   // alive -> dead
  assert.equal(alive[1].severity, 'info');    // dead -> alive
  assert.equal(restart.length, 1);
  assert.equal(restart[0].severity, 'warn');
  assert.equal(selection.length, 2);
});

test('a new launcher session is not reported as a restart', () => {
  const engine = new EventEngine();
  engine.pushFrame(frame(1, 1n, {[DS + 'connected']: 1}));
  engine.pushSystem(graph([sysNode('drivetrain', true, {session_id: '1'})]));
  engine.pushSystem(graph([sysNode('drivetrain', true, {session_id: '2', pid: '99'})]));
  const events = engine.events.filter(e => e.id.startsWith('node-restarted:'));
  assert.equal(events.length, 1);
  assert.equal(events[0].severity, 'info');
});

test('event ids are unique', () => {
  const engine = new EventEngine();
  engine.pushFrame(frame(1, 1n, {'hw/status/gateway_a_count': 1, 'hw/status/gateway_b_count': 1}));
  engine.pushFrame(frame(2, 2n, {'hw/status/gateway_a_count': 2, 'hw/status/gateway_b_count': 2}));
  const ids = engine.events.map((e: RobotEvent) => e.id);
  assert.equal(ids.length, 2);  // both gateways advanced in one frame
  assert.equal(new Set(ids).size, ids.length);
});

test('trim drops events outside the retained bounds; readers see immutable snapshots', () => {
  const engine = new EventEngine();
  engine.pushFrame(frame(1, 1000n, {'hw/status/brownout': 0}));
  engine.pushFrame(frame(2, 2000n, {'hw/status/brownout': 1}));
  engine.pushFrame(frame(3, 3000n, {'hw/status/brownout': 0}));
  const before = engine.events;
  assert.equal(before.length, 2);
  // Nothing to drop: the same array, so a subscribed view does not re-render.
  engine.trim('1500');
  assert.equal(engine.events, before);
  engine.trim('2500');
  assert.deepEqual(engine.events.map(e => e.t_ns), ['3000']);
  // An event after the trim is a new array; earlier snapshots never change.
  engine.pushFrame(frame(4, 4000n, {'hw/status/brownout': 1}));
  assert.deepEqual(engine.events.map(e => e.t_ns), ['3000', '4000']);
  assert.deepEqual(before.map(e => e.t_ns), ['2000', '3000']);
});


test('clear suppresses untimed system and manifest events but retains fresh baselines', () => {
  const engine = new EventEngine();
  const manifest = (restart_count: number): SessionManifest => ({
    session_id: '1', config_path: '', config_sha256: '', output_dir: '',
    simulation: true, start_wall_ns: '0', end_wall_ns: '0',
    nodes: [{name: 'node', target: 't', binary_path: '', log_path: '', log_paths: [],
      pid: 1, exit_code: 0, exit_signal: 0, restart_count, recorder_failed: false,
      described: true, endpoint_mismatch: false, record_count: '0'}]
  });
  engine.pushFrame(frame(0, 999n, {}));
  engine.clear();
  engine.pushSystem(graph([sysNode('node', true)]));
  engine.pushSystem(graph([sysNode('node', false)]));
  engine.pushManifest(manifest(0));
  engine.pushManifest(manifest(1));
  assert.equal(engine.events.length, 0);
  engine.pushFrame(frame(1, 1n, {}));
  engine.pushSystem(graph([sysNode('node', true)]));
  engine.pushManifest(manifest(2));
  assert.equal(engine.events.length, 2);
  assert.ok(engine.events.every(event => event.t_ns === '1'));
  assert.deepEqual(engine.events.map(event => event.severity), ['info', 'warn']);
});

test('a rejected-command run with unknown robot state ends as info', () => {
  const engine = new EventEngine();
  const channel = 'hw/status/gateway_rejected_command_count';
  engine.pushFrame(frame(0, 0n, {[channel]: 0}));
  engine.pushFrame(frame(1, 100_000_000n, {[channel]: 1}));
  engine.pushFrame(frame(2, 200_000_000n, {[channel]: 2}));
  assert.equal(engine.events.length, 0);
  engine.pushFrame(frame(3, 1_200_000_000n, {[channel]: 2}));
  assert.equal(engine.events.length, 2);
  assert.ok(engine.events.every(event => event.severity === 'info'));
  assert.equal(engine.events[0].t_ns, '100000000');
  assert.equal(engine.events[0].channel, channel);
  assert.ok(engine.events[0].detail);
});

test('unknown observations interrupt the enabled evidence needed to warn', () => {
  const engine = new EventEngine();
  const channel = 'hw/status/gateway_rejected_command_count';
  const ds = {[DS + 'enabled']: 1, [DS + 'teleop']: 1};
  const push = (seq: number, time: bigint, known = true) =>
    engine.pushFrame(frame(seq, time, {[channel]: seq, ...(known ? ds : {})}));
  push(0, 0n);
  push(1, 100_000_000n);
  push(2, 400_000_000n, false);
  push(3, 600_000_000n);
  push(4, 900_000_000n);
  assert.equal(engine.events.length, 0);
  push(5, 1_100_000_000n);
  assert.equal(engine.events.length, 1);
  assert.equal(engine.events[0].severity, 'warn');
  assert.equal(engine.events[0].t_ns, '100000000');
});

test('a short enabled rejection run does not invent a warning at its end', () => {
  const engine = new EventEngine();
  const channel = 'hw/status/gateway_rejected_command_count';
  const ds = {[DS + 'enabled']: 1, [DS + 'teleop']: 1};
  engine.pushFrame(frame(0, 0n, {[channel]: 0, ...ds}));
  engine.pushFrame(frame(1, 100_000_000n, {[channel]: 1, ...ds}));
  engine.pushFrame(frame(2, 200_000_000n, {[channel]: 1, ...ds}));
  engine.pushFrame(frame(3, 1_100_000_000n, {[channel]: 1, ...ds}));
  assert.equal(engine.events.length, 2);
  assert.ok(engine.events.every(event => event.severity === 'info'));
});

test('motor fault set bits warn; clears resolve and unchanged masks are silent', () => {
  const detector = FRAME_DETECTORS.find(d => d.id === 'link-fault-mask')!;
  const transition = (a: number, b: number) => detector.detect(
      frame(1, 1n, {'hw/status/motor_fault_mask': a}),
      frame(2, 2n, {'hw/status/motor_fault_mask': b}));
  assert.deepEqual(transition(0, 3).map(e => e.severity), ['warn']);
  assert.deepEqual(transition(3, 1).map(e => e.severity), ['info']);
  assert.deepEqual(transition(1, 0).map(e => e.severity), ['info']);
  assert.deepEqual(transition(1, 1), []);
  assert.deepEqual(transition(1, 2).map(e => e.severity), ['warn', 'info']);
});

// REQ-OPS-20: the bridge sets a timed_out_mask bit only while that subsystem's
// requests are silent on a ready link, so a newly set bit is the warning and a
// cleared bit is the recovery.
test('timeout mask set bits warn naming the subsystem bit; clears are info', () => {
  const detector = FRAME_DETECTORS.find(d => d.id === 'link-timeout-mask')!;
  const transition = (a: number, b: number) => detector.detect(
      frame(1, 1n, {'hw/status/timed_out_mask': a}),
      frame(2, 2n, {'hw/status/timed_out_mask': b}));
  const set = transition(0, 4);
  assert.deepEqual(set.map(e => e.severity), ['warn']);
  assert.match(set[0].detail ?? '', /bits: 2$/);
  assert.deepEqual(transition(4, 0).map(e => e.severity), ['info']);
  const warnings = [[0, 4], [4, 0], [0, 4]]
      .flatMap(([a, b]) => transition(a, b))
      .filter(e => e.severity === 'warn');
  assert.equal(warnings.length, 2);
});
