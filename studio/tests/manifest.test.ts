import assert from 'node:assert/strict';
import {test} from 'node:test';

import type {Snapshot} from '../src/core';
import {EventEngine} from '../src/data/events';
import {parseManifest, type SessionManifest} from '../src/data/manifest';

type ManifestNode = SessionManifest['nodes'][number];

const node = (overrides: Partial<ManifestNode> = {}): ManifestNode => ({
  name: 'drivetrain',
  target: '//2026:drivetrain',
  binary_path: '/bin/drivetrain',
  log_path: '/logs/drivetrain.tlog',
  log_paths: ['/logs/drivetrain.tlog'],
  pid: 1,
  exit_code: -1,
  exit_signal: 0,
  restart_count: 0,
  recorder_failed: false,
  described: true,
  endpoint_mismatch: false,
  record_count: '0',
  ...overrides
});

const manifest = (nodes: ManifestNode[], sessionId = '1'): SessionManifest => ({
  session_id: sessionId,
  config_path: '/robot.toml',
  config_sha256: 'abc',
  output_dir: '/tmp/session',
  simulation: false,
  start_wall_ns: '0',
  end_wall_ns: '0',
  nodes
});

function snap(ts: bigint): Snapshot {
  return {
    stream_id: 'a',
    sequence_id: '1',
    timestamp_ns: String(ts),
    dictionary_id: 1,
    chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
    channels: Object.create(null),
    units: Object.create(null),
    labels: Object.create(null)
  };
}

test('parseManifest recovers uint64 spelling exactly', () => {
  const raw = `{
    "session_id": 1735689600000000123,
    "config_path": "/robot.toml",
    "config_sha256": "abc",
    "output_dir": "/tmp/session",
    "simulation": false,
    "start_wall_ns": 1735689600000000456,
    "end_wall_ns": 1735689600000000789,
    "nodes": [
      {"name":"drivetrain","target":"//2026:drivetrain","binary_path":"/bin/d",
       "log_path":"/l","log_paths":["/l"],"pid":1,"exit_code":-1,"exit_signal":0,
       "restart_count":2,"recorder_failed":false,"described":true,"record_count":5000}
    ]
  }`;
  const m = parseManifest(raw);
  assert.equal(m.session_id, '1735689600000000123');
  assert.equal(m.start_wall_ns, '1735689600000000456');
  assert.equal(m.end_wall_ns, '1735689600000000789');
  assert.equal(m.simulation, false);
  assert.equal(m.nodes[0].restart_count, 2);
  assert.equal(m.nodes[0].record_count, '5000');
});

test('parseManifest accepts string uint64 and missing optional flags', () => {
  const raw = JSON.stringify({
    session_id: '42',
    config_path: '/robot.toml',
    config_sha256: 'abc',
    output_dir: '/tmp/session',
    simulation: true,
    start_wall_ns: '1',
    end_wall_ns: '2',
    nodes: [{
      name: 'n', target: 't', binary_path: 'b', log_path: 'l', log_paths: ['l'],
      pid: 0, exit_code: -1, restart_count: 0, record_count: 0
    }]
  });
  const m = parseManifest(raw);
  assert.equal(m.session_id, '42');
  assert.equal(m.simulation, true);
  // Absent optional booleans read as false, not an error.
  assert.equal(m.nodes[0].recorder_failed, false);
  assert.equal(m.nodes[0].described, false);
  assert.equal(m.nodes[0].endpoint_mismatch, false);
  // REQ-OPS-41: a manifest from before log_gap existed reads as no gap.
  assert.equal(m.nodes[0].log_gap, false);
  assert.equal(parseManifest(raw.replace('"record_count":0', '"record_count":0,"log_gap":true'))
                   .nodes[0].log_gap, true);
  // A manifest from before exit_signal existed reads as no signal death.
  assert.equal(m.nodes[0].exit_signal, 0);
  // REQ-OPS-11, REQ-OPS-30: legacy manifests leave lock-anomaly and
  // critical/studio_bridge fields undefined so critical is never misreported
  // as false.
  assert.equal(m.session_lock_lost, undefined);
  assert.equal(m.directory_lock_lost, undefined);
  assert.equal(m.directory_lock_retaken, undefined);
  assert.equal(m.nodes[0].critical, undefined);
  assert.equal(m.nodes[0].studio_bridge, undefined);
});

// REQ-OPS-11, REQ-OPS-30: parseManifest parses lock-loss/retake and
// per-member critical/studio_bridge booleans when present and rejects
// non-boolean values for them.
test('parseManifest reads lock flags and member critical/studio_bridge flags', () => {
  const doc = {
    session_id: '7',
    config_path: '/robot.toml',
    config_sha256: 'abc',
    output_dir: '/tmp/session',
    simulation: false,
    start_wall_ns: '1',
    end_wall_ns: '2',
    session_lock_lost: true,
    directory_lock_lost: false,
    directory_lock_retaken: true,
    nodes: [
      {
        name: 'drivetrain', target: '//2026:drivetrain', critical: true,
        studio_bridge: false, binary_path: 'b', log_path: 'l', log_paths: ['l'],
        pid: 1, exit_code: 0, restart_count: 0, record_count: 0
      },
      {
        name: 'studio_bridge', target: '//studio/bridge:studio_bridge',
        critical: false, studio_bridge: true, binary_path: 's', log_path: '',
        log_paths: [], pid: 2, exit_code: 0, restart_count: 0, record_count: 0
      }
    ]
  };
  const m = parseManifest(JSON.stringify(doc));
  assert.equal(m.session_lock_lost, true);
  assert.equal(m.directory_lock_lost, false);
  assert.equal(m.directory_lock_retaken, true);
  assert.equal(m.nodes[0].critical, true);
  assert.equal(m.nodes[0].studio_bridge, false);
  assert.equal(m.nodes[1].critical, false);
  assert.equal(m.nodes[1].studio_bridge, true);

  assert.throws(() => parseManifest(JSON.stringify({...doc, session_lock_lost: 1})));
  assert.throws(() => parseManifest(JSON.stringify({
    ...doc, nodes: [{...doc.nodes[0], critical: 'yes'}]
  })));
});

test('parseManifest reads run history and code identity; older manifests lack them', () => {
  const nodeJson = {
    name: 'n', target: 't', binary_path: 'b', binary_sha256: 'ff', log_path: 'l',
    log_paths: ['l'], pid: 8, exit_code: -1, restart_count: 1, record_count: 0,
    runs_dropped: 3,
    runs: [{pid: 7, start_wall_ns: 1735689600000000000,
            end_wall_ns: 1735689601000000000, exit_code: -1, exit_signal: 9,
            reason: 'stalled'},
           {pid: 8, start_wall_ns: 1735689602000000000, end_wall_ns: 0,
            exit_code: -1, exit_signal: 0, reason: 'running'},
           {pid: 9, start_wall_ns: 1735689603000000000,
            end_wall_ns: 1735689604000000000, exit_code: -1, exit_signal: 2,
            reason: 'stop', stopped_by_session: true}]
  };
  const document = {
    session_id: '1', config_path: '/robot.toml', config_sha256: 'abc',
    git_revision: '0123abcd', git_dirty: true, output_dir: '/tmp/session',
    simulation: false, start_wall_ns: '1', end_wall_ns: '0', nodes: [nodeJson]
  };
  const m = parseManifest(JSON.stringify(document));
  // The top-level times, not a run's, which follow them in the text.
  assert.equal(m.start_wall_ns, '1');
  assert.equal(m.end_wall_ns, '0');
  assert.equal(m.git_revision, '0123abcd');
  assert.equal(m.git_dirty, true);
  assert.equal(m.nodes[0].binary_sha256, 'ff');
  assert.equal(m.nodes[0].runs_dropped, 3);
  assert.deepEqual(m.nodes[0].runs!.map(r => [r.pid, r.exit_signal, r.reason,
                                              r.stopped_by_session]),
                   [[7, 9, 'stalled', undefined], [8, 0, 'running', undefined],
                    [9, 2, 'stop', true]]);
  const {runs: _runs, runs_dropped: _dropped, binary_sha256: _sha, ...old} = nodeJson;
  const {git_revision: _rev, git_dirty: _dirty, ...older} = document;
  const legacy = parseManifest(JSON.stringify({...older, nodes: [old]}));
  assert.equal(legacy.git_revision, undefined);
  assert.equal(legacy.nodes[0].runs, undefined);
  assert.throws(() => parseManifest(JSON.stringify(
      {...document, nodes: [{...nodeJson, runs: [{pid: 1}]}]})));
});

test('parseManifest rejects malformed documents', () => {
  assert.throws(() => parseManifest('not json'));
  assert.throws(() => parseManifest('{"session_id": "not a number"}'));
  assert.throws(() => parseManifest(JSON.stringify({
    session_id: 1, config_path: 5, config_sha256: '', output_dir: '',
    simulation: false, start_wall_ns: 0, end_wall_ns: 0, nodes: []
  })));
  // Too many nodes.
  assert.throws(() => parseManifest(JSON.stringify({
    session_id: 1, config_path: 'c', config_sha256: '', output_dir: '',
    simulation: false, start_wall_ns: 0, end_wall_ns: 0,
    nodes: Array.from({length: 300}, () => ({}))
  })));
});

test('manifest detector reports restart and failure', () => {
  const engine = new EventEngine();
  engine.pushFrame(snap(1n));  // establish a telemetry time
  engine.pushManifest(manifest([node({restart_count: 0, recorder_failed: false})]));
  engine.pushManifest(manifest([node({restart_count: 3, recorder_failed: true})]));
  const events = engine.events.filter(e => e.id.startsWith('manifest-node:'));
  assert.equal(events.length, 2);
  assert.equal(events[0].severity, 'warn');
  assert.equal(events[1].severity, 'error');
});

test('manifest detector is silent on first observation', () => {
  const engine = new EventEngine();
  engine.pushFrame(snap(1n));
  engine.pushManifest(manifest([node({restart_count: 5, recorder_failed: true})]));
  assert.equal(engine.events.filter(e => e.id.startsWith('manifest-node:')).length, 0);
});
