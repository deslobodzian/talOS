// Session panel: pure merge/sort logic (manifest-only, registry-only, both,
// declared-but-missing) and the rendered manifest error and session uptime.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {readFileSync} from 'node:fs';

import {parseManifest, type SessionManifest} from '../src/data/manifest';
import {createStaticStudio, type Studio, StudioProvider} from '../src/data/studio';
import {sessionPanel} from '../src/panels/session';
import {loopRate, mergeProcesses, summarize, worstLatency} from '../src/panels/session/logic';
import {
  type DeclaredGraph,
  type DeclaredNode,
  GraphVersionMismatch,
  SYSTEM_GRAPH_VERSION,
  type SystemGraph,
  type SystemNode,
  type SystemSource
} from '../src/system';

// --- fixtures --------------------------------------------------------------

type ManifestNode = SessionManifest['nodes'][number];

const mnode = (overrides: Partial<ManifestNode> = {}): ManifestNode => ({
  name: 'drivetrain',
  target: '//2026:drivetrain',
  binary_path: '/bin/drivetrain',
  log_path: '/logs/drivetrain.tlog',
  log_paths: ['/logs/drivetrain.tlog'],
  pid: 100,
  exit_code: -1,
  exit_signal: 0,
  restart_count: 0,
  recorder_failed: false,
  described: true,
  endpoint_mismatch: false,
  record_count: '5000',
  ...overrides
});

const manifest = (nodes: ManifestNode[]): SessionManifest => ({
  session_id: '7351220489113141',
  config_path: '/robot.toml',
  config_sha256: 'abcdef0123456789',
  output_dir: '/tmp/session',
  simulation: false,
  start_wall_ns: '1735689600000000000',
  end_wall_ns: '0',
  nodes
});

const source = (overrides: Partial<SystemSource> = {}): SystemSource => ({
  id: 0,
  kind: 'TIMER',
  name: 'loop',
  message_bytes: 0,
  alignment: 0,
  period_ns: '5000000',
  events: '0',
  dropped: '0',
  sequence: '0',
  last_monotonic_ns: '0',
  last_latency_ns: '0',
  max_latency_ns: '0',
  external: false,
  optional: false,
  observer: false,
  ...overrides
});

const snode = (overrides: Partial<SystemNode> = {}): SystemNode => ({
  name: 'drivetrain',
  target: '//2026:drivetrain',
  slot: 0,
  pid: '100',
  session_id: '1',
  generation: '1',
  start_wall_ns: '0',
  heartbeat_wall_ns: '0',
  dispatch_count: '0',
  declared_sources: 0,
  alive: true,
  simulation: false,
  replay: false,
  sources: [],
  selections: [],
  ...overrides
});

const graph = (nodes: SystemNode[], wallNs = '0'): SystemGraph => ({
  version: 2,
  wall_ns: wallNs,
  registry: {
    available: true,
    node_capacity: 32,
    source_capacity: 64,
    liveness_timeout_ns: '5000000000'
  },
  bridge: {
    topic: '/talos/telemetry',
    clients: 0,
    frames_forwarded: '0',
    invalid: '0',
    ws_drops: '0'
  },
  nodes,
  topics: []
});

const dnode = (name: string, target: string): DeclaredNode =>
    ({name, target, sources: []});

const declared = (nodes: DeclaredNode[]): DeclaredGraph =>
    ({version: 1, nodes, diagnostics: []});

// --- pure logic ------------------------------------------------------------

test('mergeProcesses reads manifest-only rows', () => {
  const m = manifest([
    mnode({name: 'a', pid: 1, exit_code: 0, record_count: '42'}),
    mnode({name: 'b', pid: 2, exit_code: -1, restart_count: 3,
           recorder_failed: true, runs_dropped: 0, runs: [
             {pid: 2, start_wall_ns: 1, end_wall_ns: 2, exit_code: -1,
              exit_signal: 9, reason: 'stalled'}]})
  ]);
  const rows = mergeProcesses(m, null, null, {}, false);
  assert.equal(rows.length, 2);

  // Problems sort first, so 'b' leads.
  assert.equal(rows[0].name, 'b');
  assert.equal(rows[1].name, 'a');

  const a = rows.find(r => r.name === 'a')!;
  assert.equal(a.pid, '1');
  assert.equal(a.alive, null);
  assert.equal(a.exitCode, 0);
  assert.equal(a.records, '42');
  assert.equal(a.hasManifest, true);
  assert.equal(a.endReason, null);  // no run history: an older manifest
  assert.deepEqual(a.problems, []);

  const b = rows.find(r => r.name === 'b')!;
  assert.equal(b.pid, '2');
  assert.equal(b.exitCode, null);   // -1 means not exited, not "exit 0"
  assert.equal(b.restarts, 3);
  assert.equal(b.recorderFailed, true);
  // The launcher's kill reason, not just its signal, reaches the row.
  assert.equal(b.endReason, 'stalled');
  assert.deepEqual(b.problems, ['restarted', 'recorder_failed']);
});

test('mergeProcesses reads registry-only rows and measures rate/latency', () => {
  const g = graph([
    snode({
      name: 'loop',
      sources: [
        source({id: 0, kind: 'TIMER', max_latency_ns: '2000000'}),
        source({id: 1, kind: 'TIMER', max_latency_ns: '500000'})
      ]
    }),
    snode({name: 'gone', alive: false})
  ]);
  const rates = {'loop 0': 200, 'loop 1': 100};
  const rows = mergeProcesses(null, g, null, rates, false);
  assert.equal(rows.length, 2);
  assert.equal(rows[0].name, 'gone');   // the problem sorts first

  const loop = rows.find(r => r.name === 'loop')!;
  assert.equal(loop.alive, true);
  assert.equal(loop.loopRate, 200);      // busiest timer
  assert.equal(loop.worstLatency, 2000000);
  assert.equal(loop.pid, '100');         // the registry pid
  assert.equal(loop.hasManifest, false);
  assert.deepEqual(loop.problems, []);

  const gone = rows.find(r => r.name === 'gone')!;
  assert.equal(gone.alive, false);
  assert.equal(gone.loopRate, null);
  assert.equal(gone.worstLatency, null);
  assert.deepEqual(gone.problems, ['dead']);

  // A stale registry graph keeps who held a slot, but claims nobody is alive
  // or dead now.
  const stale = mergeProcesses(null, g, null, rates, true);
  for (const row of stale) {
    assert.equal(row.alive, null, `${row.name} liveness from a stale graph`);
    assert.deepEqual(row.problems, []);
  }
  assert.equal(stale.find(r => r.name === 'loop')!.pid, '100');
  assert.equal(summarize(stale).running, 0);
});

test('mergeProcesses joins manifest and registry by name', () => {
  const m = manifest([
    mnode({name: 'a', pid: 1, exit_code: 3, restart_count: 1,
           record_count: '10'}),
    mnode({name: 'b', pid: 2}),
    mnode({name: 'd', pid: 4, exit_signal: 9}),
    mnode({name: 'e', pid: 5, endpoint_mismatch: true})
  ]);
  const g = graph([
    snode({name: 'a', pid: '1', alive: false}),
    snode({name: 'c', pid: '3'})   // registry-only
  ]);
  const rows = mergeProcesses(m, g, null, {}, false);
  assert.equal(rows.length, 5);

  const a = rows.find(r => r.name === 'a')!;
  assert.equal(a.alive, false);
  assert.equal(a.exitCode, 3);
  assert.equal(a.restarts, 1);
  assert.deepEqual(a.problems, ['dead', 'restarted', 'nonzero_exit']);

  const b = rows.find(r => r.name === 'b')!;
  assert.equal(b.alive, null);   // launched, never registered — not "dead"
  assert.equal(b.pid, '2');
  assert.deepEqual(b.problems, []);

  // A signal death reads as one, not as "exit -1" or as no exit at all.
  const d = rows.find(r => r.name === 'd')!;
  assert.equal(d.exitCode, null);
  assert.equal(d.exitSignal, 'signal 9 SIGKILL');
  assert.deepEqual(d.problems, ['killed_by_signal']);

  // A run whose live endpoints differed from its probe is marked.
  assert.deepEqual(rows.find(r => r.name === 'e')!.problems,
                   ['endpoint_mismatch']);

  const c = rows.find(r => r.name === 'c')!;
  assert.equal(c.hasManifest, false);
  assert.equal(c.alive, true);
  assert.equal(c.pid, '3');
});

test('mergeProcesses flags declared-but-never-registered and sorts it first',
     () => {
       const m = manifest([mnode({name: 'ok'})]);
       const g = graph([snode({name: 'ok'})]);
       const d = declared([
         dnode('ok', '//2026:ok'),
         dnode('missing', '//2026:missing')
       ]);
       const rows = mergeProcesses(m, g, d, {}, false);
       assert.equal(rows.length, 2);
       assert.equal(rows[0].name, 'missing');
       assert.deepEqual(rows[0].problems, ['never_registered']);
       assert.equal(rows[0].alive, null);
       assert.equal(rows[0].target, '//2026:missing');
       assert.equal(rows[1].name, 'ok');
       assert.deepEqual(rows[1].problems, []);
     });

test('never-registered is not claimed without a live registry', () => {
  // A declared graph with no registry to compare against cannot say "never
  // registered" — that is the same rule `declaredStatus` follows.
  const d = declared([dnode('missing', '//2026:missing')]);
  const rows = mergeProcesses(null, null, d, {}, false);
  assert.equal(rows.length, 1);
  assert.deepEqual(rows[0].problems, []);
});

test('worstLatency measures only timer and watcher dispatches', () => {
  const passive = [
    source({kind: 'SENDER', max_latency_ns: '9000000'}),
    source({kind: 'FETCHER', max_latency_ns: '0'})
  ];
  assert.equal(worstLatency(snode({sources: passive})), null);
  assert.equal(worstLatency(snode({sources: [
    ...passive, source({kind: 'TIMER', max_latency_ns: '0'})
  ]})), 0);
  assert.equal(worstLatency(snode({sources: [
    ...passive, source({kind: 'WATCHER', max_latency_ns: '123'})
  ]})), 123);
});

test('loopRate ignores non-finite source rates', () => {
  const n = snode({
    name: 'loop',
    sources: [
      source({id: 0, kind: 'TIMER'}),
      source({id: 1, kind: 'TIMER'})
    ]
  });
  assert.equal(loopRate(n, {'loop 0': NaN, 'loop 1': 200}), 200);
  assert.equal(loopRate(n, {'loop 0': Infinity, 'loop 1': NaN}), null);
});

test('summarize counts running, total and restarts', () => {
  const m = manifest([
    mnode({name: 'a', restart_count: 2}),
    mnode({name: 'b', restart_count: 1, exit_code: 1})
  ]);
  const g = graph([
    snode({name: 'a', alive: true}),
    snode({name: 'b', alive: false}),
    snode({name: 'c', alive: true})
  ]);
  const summary = summarize(mergeProcesses(m, g, null, {}, false));
  assert.equal(summary.total, 3);
  assert.equal(summary.running, 2);
  assert.equal(summary.restarts, 3);
});

// --- rendering -------------------------------------------------------------

const renderPanel = (studio: Studio): string => renderToStaticMarkup(
    <StudioProvider studio={studio}>
      <sessionPanel.component config={sessionPanel.defaultConfig}
                              setConfig={() => {}} instanceId="test"/>
    </StudioProvider>);

test('a manifest that failed to parse shows the parser\'s reason', () => {
  const html = renderPanel(createStaticStudio({
    manifest: null,
    manifestError: 'Invalid manifest: session_id must be a decimal integer'
  }));
  assert.ok(html.includes('session_id must be a decimal integer'));
});

test('uptime runs to the registry clock, then to the end; unknown without either', () => {
  const uptime = (m: SessionManifest, g: SystemGraph|null) => {
    const html = renderPanel(createStaticStudio({
      manifest: m,
      system: {graph: g, rates: {}, error: '', declared: null, declaredError: ''}
    }));
    return /Uptime([^A-Z]*)/.exec(html.replace(/<[^>]*>/g, ''))![1];
  };
  const running = manifest([mnode({name: 'a'})]);
  assert.equal(uptime(running, null), '—');
  // One hour after start by the registry clock.
  assert.equal(uptime(running, graph([snode({name: 'a'})], '1735693200000000000')),
               '1 h 0 m');
  const ended = manifest([mnode({name: 'a', exit_code: 0})]);
  ended.end_wall_ns = '1735689605000000000';   // 5 s after start
  assert.equal(uptime(ended, null), '5.0 s');
});

// REQ-OPS-06: a member ended by the launcher's own stop signal before it
// handled stops is a clean stop, not a signal death; any other signal death,
// or SIGINT for any other reason, stays flagged. The run's stopped_by_session
// decides when the manifest has it: a SIGINT from elsewhere that ended the
// run during the stop is reason stop too, and the launcher counted it. A
// manifest without the field (an older launcher) falls back to reason stop
// with SIGINT; SIGTERM (systemd's default kill mode) is never the stop's own.
test('mergeProcesses shows a launcher-stopped member as stopped', () => {
  const run = (reason: string, exit_signal: number,
               stopped_by_session?: boolean) => [{
    pid: 1, start_wall_ns: 1, end_wall_ns: 2, exit_code: -1, exit_signal,
    reason, ...(stopped_by_session === undefined ? {} : {stopped_by_session})
  }];
  const m = manifest([
    mnode({name: 'stopped', exit_signal: 2, runs: run('stop', 2, true)}),
    mnode({name: 'foreign', exit_signal: 2, runs: run('stop', 2, false)}),
    mnode({name: 'older', exit_signal: 2, runs: run('stop', 2)}),
    mnode({name: 'terminated', exit_signal: 15, runs: run('stop', 15)}),
    mnode({name: 'crashed', exit_signal: 2, runs: run('crash', 2)}),
    mnode({name: 'killed', exit_signal: 9, runs: run('stop_timeout', 9)})
  ]);
  const rows = mergeProcesses(m, null, null, {}, false);
  const row = (n: string) => rows.find(r => r.name === n)!;
  for (const name of ['stopped', 'older']) {
    assert.deepEqual(row(name).problems, [], name);
    assert.equal(row(name).exitSignal, null, name);
    assert.equal(row(name).endReason, 'stop', name);
  }
  assert.deepEqual(row('foreign').problems, ['killed_by_signal']);
  assert.equal(row('foreign').exitSignal, 'signal 2 SIGINT');
  assert.deepEqual(row('terminated').problems, ['killed_by_signal']);
  assert.equal(row('terminated').exitSignal, 'signal 15 SIGTERM');
  assert.deepEqual(row('crashed').problems, ['killed_by_signal']);
  assert.deepEqual(row('killed').problems, ['killed_by_signal']);
});

// REQ-OPS-41: a run whose event log has a marked gap is named in the problem
// list beside "recorder failed" (the launcher sets both); a run without the
// field shows exactly what it did before.
test('a run with a log gap is a problem beside recorder failed', () => {
  const rows = mergeProcesses(manifest([
    mnode({name: 'gapped', recorder_failed: true, log_gap: true}),
    mnode({name: 'old'})
  ]), null, null, {}, false);
  const row = (name: string) => rows.find(r => r.name === name)!;
  assert.deepEqual(row('gapped').problems, ['recorder_failed', 'log_gap']);
  assert.deepEqual(row('old').problems, []);
});

// REQ-OPS-43: what each node lost in delivery, as the launcher records it
// (talOS/launcher/session.cc), is shown per process, and any nonzero count is
// a problem, as `session_report --strict` fails it: a lost wake, a stalled
// arm, a wake fallback, or the bridge's skipped requests with the subsystem
// each was skipped for. A node whose registry row the launcher never saw
// carries none of the fields, which is "not recorded", never "none".
// session_manifest.json is a real simulation session's manifest
// (bench/main_processor/configuration/robot_physics_sim.toml), its paths
// shortened, with three counts set nonzero.
test('a node\'s delivery loss from the manifest is shown and is a problem', () => {
  const raw = readFileSync(new URL('./session_manifest.json', import.meta.url), 'utf8');
  const parsed = parseManifest(raw);
  const rows = mergeProcesses(parsed, null, null, {}, false);
  const row = (name: string) => rows.find(r => r.name === name)!;
  assert.deepEqual(row('telemetry').delivery, {
    wake_failures: 1, stalled_arms: 2, wake_fallbacks: 0, requests_skipped: 0,
    requests_skipped_by_subsystem: {}
  });
  assert.deepEqual(row('telemetry').problems, ['delivery_loss']);
  assert.deepEqual(row('hardware_node').delivery?.requests_skipped_by_subsystem,
                   {swerve_module: 3});
  assert.deepEqual(row('hardware_node').problems, ['delivery_loss']);
  assert.deepEqual(row('gyro').problems, []);

  const unseen = JSON.parse(raw) as {nodes: Record<string, unknown>[]};
  for (const key of ['wake_failures', 'stalled_arms', 'wake_fallbacks',
                     'requests_skipped', 'requests_skipped_by_subsystem']) {
    delete unseen.nodes[1][key];
  }
  const gyro = parseManifest(JSON.stringify(unseen)).nodes[1];
  assert.equal(gyro.name, 'gyro');
  assert.equal(gyro.delivery, undefined);

  const text = renderPanel(createStaticStudio({manifest: parsed}))
                   .replace(/<[^>]*>/g, ' ').replace(/\s+/g, ' ');
  assert.match(text, /telemetry delivery loss .*wake failures 1 · stalled arms 2/);
  assert.match(text, /hardware_node delivery loss .*skipped requests 3 \(swerve_module 3\)/);
  assert.match(text, /gyro .* none /);
});

// REQ-OPS-23: with a bridge of another graph version the page holds no
// graph, so how many processes run is unknown, not 0, and the panel names
// the mismatch by both versions.
test('a graph version mismatch leaves the running count unknown and names both versions',
     () => {
       const html = renderPanel(createStaticStudio({
         manifest: manifest([mnode({name: 'a'}), mnode({name: 'b'})]),
         system: {
           graph: null,
           stale: true,
           error: String(new GraphVersionMismatch(SYSTEM_GRAPH_VERSION + 1))
         }
       }));
       const count = html.replace(/<!-- -->/g, '');
       assert.ok(count.includes('?/2'), 'running count is not unknown');
       assert.ok(html.includes(`data-bridge-version="${SYSTEM_GRAPH_VERSION + 1}"`));
       assert.ok(html.includes(`data-page-version="${SYSTEM_GRAPH_VERSION}"`));
     });
