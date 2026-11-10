// Session panel: pure merge/sort logic (manifest-only, registry-only, both,
// declared-but-missing) and the rendered manifest error and session uptime.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import type {SessionManifest} from '../src/data/manifest';
import {createStaticStudio, type Studio, StudioProvider} from '../src/data/studio';
import {sessionPanel} from '../src/panels/session';
import {mergeProcesses, summarize, worstLatency} from '../src/panels/session/logic';
import type {
  DeclaredGraph,
  DeclaredNode,
  SystemGraph,
  SystemNode,
  SystemSource
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
           recorder_failed: true})
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
  assert.deepEqual(a.problems, []);

  const b = rows.find(r => r.name === 'b')!;
  assert.equal(b.pid, '2');
  assert.equal(b.exitCode, null);   // -1 means not exited, not "exit 0"
  assert.equal(b.restarts, 3);
  assert.equal(b.recorderFailed, true);
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
