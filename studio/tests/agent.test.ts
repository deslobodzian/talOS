import assert from 'node:assert/strict';
import {execFileSync} from 'node:child_process';
import dgram from 'node:dgram';
import {once} from 'node:events';
import fs, {existsSync, readdirSync, readFileSync, readlinkSync, statSync} from 'node:fs';
import {
  Agent as HttpAgent,
  createServer,
  request as httpRequest,
} from 'node:http';
import type {AddressInfo} from 'node:net';
import path from 'node:path';
import {test} from 'node:test';
import {fileURLToPath} from 'node:url';

import {WebSocket, WebSocketServer} from 'ws';

import {
  dispatch,
  handleRpc,
  RPC_METHODS,
  type RpcContext,
  RpcFault,
} from '../agent/rpc';
import {MAX_BATCH, MAX_WS_CLIENTS, startAgent} from '../agent/server';
import {type Snapshot, TelemetryDecoder, Timeline} from '../src/core';
import {
  type ChannelSpec,
  concatPages,
  encodeDictionaryPages,
  encodeValuePages,
  type PageHeader,
  rawFromDouble,
} from '../src/encode';
import {ChannelKind} from '../src/generated/talos/telemetry/channel-kind';
import {GRAPH_STALE_AFTER_MS} from '../src/data/freshness';
import {demoDeclaredGraph, demoPacket, demoSystemGraph} from '../src/demo';
import {
  type DeclaredGraph,
  MAX_DECLARED_BYTES,
  parseDeclaredGraph,
  parseSystemGraph,
  SYSTEM_GRAPH_VERSION,
  type SystemGraph,
} from '../src/system';

function history() {
  const timeline = new Timeline();
  const decoder = new TelemetryDecoder();
  [0, 4, 5, 0, 7].forEach((current, i) => {
    const frame = decoder.decode(demoPacket(BigInt(i)))!;
    frame.channels['intake/state/current_a'] = current;
    timeline.append(frame);
  });
  return timeline;
}

// The telemetry methods do not read the system graph, so most tests pass none.
const ctx = (timeline: Timeline, system: SystemGraph|null = null,
             previousSystem: SystemGraph|null = null,
             declared: DeclaredGraph|null = null): RpcContext =>
    ({timeline, system, previousSystem, declared});

const systemContext = (system: SystemGraph|null,
                       previousSystem: SystemGraph|null = null,
                       declared: DeclaredGraph|null = null) =>
    ctx(new Timeline(), system, previousSystem, declared);

// The same document the app consumes, so these tests fail if the graph the UI
// is fed ever stops satisfying the parser. The demo sequence advances at 50 Hz
// alongside demoPacket(), so a later sequence is a later sample of one run.
const demoGraph = (sequence: bigint = 0n) =>
    parseSystemGraph(demoSystemGraph(sequence));

test(
    'RPC exact lookup rejects in-between samples and supports explicit preceding state',
    () => {
      const timeline = history();
      assert.throws(
          () => dispatch(
              ctx(timeline), 'query_state_at', {timestamp_ns: '1020000001'}),
          (e: unknown) => e instanceof RpcFault && e.code === -32001);
      const response = dispatch(ctx(timeline), 'query_state_at', {
                         timestamp_ns: '1020000001',
                         mode: 'at_or_before',
                         topics: ['intake/state/current_a', 'chassis.x']
                       }) as any;
      assert.equal(response.timestamp_ns, '1020000000');
      assert.equal(response.topics['intake/state/current_a'], 4);
      assert.equal(typeof response.topics['chassis.x'], 'number');
      assert.throws(
          () => dispatch(
              ctx(timeline), 'query_state_at',
              {timestamp_ns: '18446744073709551616'}),
          /uint64/);
    });
// A 64-bit integer channel answers exactly: 2^53 and 2^53 + 1 are one double,
// so a rounded value would hide both the reading and the change.
test('RPC answers integer channels with their exact value', () => {
  const timeline = new Timeline();
  const decoder = new TelemetryDecoder();
  const boot = ['9007199254740992', '9007199254740992', '9007199254740993'];
  boot.forEach((id, i) => {
    const frame = decoder.decode(demoPacket(BigInt(i)))!;
    frame.channels['hw/status/clock_boot_id'] = Number(id);
    frame.exact = {...frame.exact, 'hw/status/clock_boot_id': id};
    timeline.append(frame);
  });
  const last = timeline.frames.at(-1)!;
  const response = dispatch(ctx(timeline), 'query_state_at', {
                     timestamp_ns: last.timestamp_ns,
                     topics: ['hw/status/clock_boot_id']
                   }) as any;
  assert.equal(response.topics['hw/status/clock_boot_id'], '9007199254740993');
  const first = dispatch(ctx(timeline), 'query_state_at', {
                  timestamp_ns: timeline.frames[0].timestamp_ns,
                  topics: ['hw/status/clock_boot_id']
                }) as any;
  assert.equal(first.topics['hw/status/clock_boot_id'], '9007199254740992');
  const changed = dispatch(ctx(timeline), 'scan_channel_events', {
                    topic: 'hw/status/clock_boot_id',
                    condition: {op: 'changed'}
                  }) as any;
  assert.deepEqual(changed.ranges, [
    {start_ns: last.timestamp_ns, end_ns: last.timestamp_ns, samples: 1}
  ]);
  const above = dispatch(ctx(timeline), 'scan_channel_events', {
                  topic: 'hw/status/clock_boot_id',
                  condition: {op: 'gt', value: 9007199254740992}
                }) as any;
  assert.equal(above.ranges.length, 1);
});
// NaN and the infinities are readings: an answer spells them as the bridge's
// JSON does, since JSON.stringify would turn them into null (absence). NaN
// orders with nothing, so no threshold matches it, and NaN after NaN is no
// change. An integer channel is a number while that number is exact.
test('RPC answers NaN and infinities as readings and integers as numbers when exact', () => {
  const timeline = new Timeline();
  const decoder = new TelemetryDecoder();
  const readings = [1, NaN, NaN, Infinity, -Infinity, 2];
  readings.forEach((x, i) => {
    const frame = decoder.decode(demoPacket(BigInt(i)))!;
    frame.channels['test/state/x'] = x;
    frame.channels['test/state/count'] = 5;
    frame.exact = {...frame.exact, 'test/state/count': '5'};
    frame.kinds = {...frame.kinds, 'test/state/x': ChannelKind.Double,
                   'test/state/count': ChannelKind.UInt};
    timeline.append(frame);
  });
  const at = (i: number) => timeline.frames[i].timestamp_ns;
  const value = (i: number) => (dispatch(ctx(timeline), 'query_state_at', {
    timestamp_ns: at(i), topics: ['test/state/x', 'test/state/count']
  }) as any).topics;
  assert.deepEqual(value(1), {'test/state/x': 'NaN', 'test/state/count': 5});
  assert.equal(value(3)['test/state/x'], 'Infinity');
  assert.equal(value(4)['test/state/x'], '-Infinity');
  const whole = JSON.parse(JSON.stringify(
      dispatch(ctx(timeline), 'query_state_at', {timestamp_ns: at(1)})));
  assert.equal(whole.channels['test/state/x'], 'NaN');
  assert.equal(whole.channels['test/state/count'], 5);
  assert.equal(whole.kinds['test/state/count'], 'uint');
  const latest = JSON.parse(JSON.stringify(dispatch(ctx(timeline), 'get_latest')));
  assert.equal(latest.channels['test/state/x'], 2);

  const ranges = (condition: object) =>
      (dispatch(ctx(timeline), 'scan_channel_events',
                {topic: 'test/state/x', condition}) as any)
          .ranges.map((r: any) => [r.start_ns, r.end_ns]);
  assert.deepEqual(ranges({op: 'gte', value: 0}),
                   [[at(0), at(0)], [at(3), at(3)], [at(5), at(5)]]);
  assert.deepEqual(ranges({op: 'lte', value: 0}), [[at(4), at(4)]]);
  assert.deepEqual(ranges({op: 'changed'}), [[at(1), at(1)], [at(3), at(5)]]);
  // A scan accepts the spellings an answer used: eq "NaN" finds the NaN
  // readings rather than comparing them to a string.
  assert.deepEqual(ranges({op: 'eq', value: 'NaN'}), [[at(1), at(2)]]);
  assert.deepEqual(ranges({op: 'ne', value: 'NaN'}),
                   [[at(0), at(0)], [at(3), at(5)]]);
  assert.deepEqual(ranges({op: 'eq', value: '-Infinity'}), [[at(4), at(4)]]);
  assert.deepEqual(ranges({op: 'ne', value: '-Infinity'}),
                   [[at(0), at(3)], [at(5), at(5)]]);

  // The schema tree labels channels by kind and lists no decoder internals.
  const tree = (dispatch(ctx(timeline), 'get_schema_tree') as any).topics;
  assert.equal(tree['test/state/x'], 'double');
  assert.equal(tree['test/state/count'], 'uint');
  assert.ok(!Object.keys(tree).some(
      name => name.startsWith('exact.') || name.startsWith('kinds.')));
});
test(
    'threshold scans group samples and edge scans use the preceding sample outside bounds',
    () => {
      const timeline = history();
      const result =
          dispatch(
              ctx(timeline), 'scan_channel_events',
              {
                topic: 'intake/state/current_a',
                condition: {op: 'gt', value: 3}
              }) as {ranges: {start_ns: string; end_ns: string; samples: number}[]};
      assert.deepEqual(result.ranges, [
        {start_ns: '1020000000', end_ns: '1040000000', samples: 2},
        {start_ns: '1080000000', end_ns: '1080000000', samples: 1}
      ]);
      const edge = dispatch(ctx(timeline), 'scan_channel_events', {
                     topic: 'intake/state/current_a',
                     condition: {op: 'falling'},
                     start_ns: '1060000000'
                   }) as any;
      assert.deepEqual(
          edge.ranges,
          [{start_ns: '1060000000', end_ns: '1060000000', samples: 1}]);
    });
test(
    'JSON-RPC preserves IDs, suppresses notifications, and validates requests',
    () => {
      const timeline = history();
      assert.equal(
          handleRpc(ctx(timeline), {jsonrpc: '2.0', method: 'get_schema_tree'}),
          undefined);
      assert.equal(
          (handleRpc(ctx(timeline), {
             jsonrpc: '2.0',
             id: 0,
             method: 'get_schema_tree'
           }) as any)
              .id,
          0);
      assert.equal(
          (handleRpc(
               ctx(timeline), {jsonrpc: '2.0', id: 'a', method: 'missing'}) as
           any)
              .error.code,
          -32601);
      assert.equal(
          (handleRpc(ctx(timeline), {
             jsonrpc: '2.0',
             id: 1,
             method: 'query_state_at',
             params: {timestamp_ns: 1020000000}
           }) as any)
              .error.code,
          -32602);
      assert.equal(
          (handleRpc(
               ctx(timeline), {jsonrpc: '1.0', method: 'get_schema_tree'}) as
           any)
              .error.code,
          -32600);
      assert.throws(
          () => dispatch(
              ctx(timeline), 'query_state_at',
              {timestamp_ns: '1020000000', topics: ['__proto__.polluted']}),
          /Unknown topic/);
    });

test('system methods distinguish an unknown system from an empty one', () => {
  // -32002, not an empty graph: an agent must be able to tell "nothing is
  // running" from "nobody has told me what is running yet".
  for (const method of ['get_system_graph', 'describe_topic']) {
    assert.throws(
        () => dispatch(systemContext(null), method, {topic: '/pose_estimator/state'}),
        (e: unknown) => e instanceof RpcFault && e.code === -32002);
  }
  // The declared graph is the same distinction one step earlier: the bridge may
  // have been started without --declared, which is not "the launcher declared
  // nothing".
  assert.throws(
      () => dispatch(systemContext(demoGraph()), 'get_declared_graph', {}),
      (e: unknown) => e instanceof RpcFault && e.code === -32002);
  assert.equal(
      (handleRpc(
           systemContext(null),
           {jsonrpc: '2.0', id: 7, method: 'get_system_graph'}) as any)
          .error.code,
      -32002);
});

test('the system graph reports every node, both ends of every topic, and health',
     () => {
       const system = demoGraph();
       const result =
           dispatch(systemContext(system), 'get_system_graph', {}) as any;

       assert.equal(result.nodes.length, system.nodes.length);
       assert.ok(result.nodes.length > 0, 'demo graph must contain nodes');
       assert.equal(result.topics.length, system.topics.length);
       assert.ok(result.topics.length > 0, 'demo graph must contain topics');

       // Every topic is classified, and the summary counts agree with the rows.
       const healths = new Set(result.topics.map((t: any) => t.health));
       for (const health of healths)
         assert.ok(
             ['ok', 'orphaned', 'unread', 'lossy', 'idle', 'bridged',
              'unconnected']
                 .includes(health as string),
             `unexpected health ${health}`);
       assert.equal(result.summary.nodes, system.nodes.length);
       assert.equal(
           result.summary.nodes_alive,
           system.nodes.filter(n => n.alive).length);
       assert.equal(result.summary.topics, system.topics.length);
       assert.equal(
           result.summary.orphaned_topics,
           result.topics.filter((t: any) => t.health === 'orphaned').length);
       assert.equal(
           result.summary.unread_topics,
           result.topics.filter((t: any) => t.health === 'unread').length);
       assert.equal(
           result.summary.lossy_topics,
           result.topics.filter((t: any) => t.health === 'lossy').length);

       // An end its own node declared external or optional is reported as such
       // rather than counted as a fault, and the per-state tally lets an agent
       // count faults without knowing which states are faults.
       assert.equal(
           result.topics.find((t: any) => t.name === '/hw/command').health,
           'bridged');
       assert.equal(
           result.topics.find((t: any) => t.name === '/drivetrain/target/auto')
               .health,
           'unconnected');
       assert.equal(result.summary.bridged_topics, 1);
       assert.ok(result.summary.unconnected_topics >= 1);
       assert.equal(
           Object.values(result.summary.topics_by_health)
               .reduce((total: number, n: any) => total + Number(n), 0),
           system.topics.length);
       const command = result.nodes.find((n: any) => n.name === 'hardware_node')
                           .sources.find((s: any) => s.name === '/hw/command');
       assert.equal(command.external, true);
       assert.equal(command.optional, false);

       // Registry and bridge state pass through, so a viewer can say why the
       // graph is empty when it is.
       assert.equal(result.registry.available, system.registry.available);
       assert.equal(result.bridge.topic, system.bridge.topic);
       assert.equal(result.wall_ns, system.wall_ns);
     });

test('per-source rates are absent without a previous sample and measured with one',
     () => {
       // Ten seconds into the run, and ten seconds after that.
       const previous = demoGraph(500n);
       const system = demoGraph(1000n);

       const withoutPrevious =
           dispatch(systemContext(system), 'get_system_graph', {}) as any;
       for (const node of withoutPrevious.nodes)
         for (const source of node.sources)
           assert.equal(
               source.events_per_second, null,
               'a cumulative counter is not a rate without a previous sample');

       const withPrevious =
           dispatch(
               systemContext(system, previous), 'get_system_graph', {}) as any;

       const aliveInBoth = (name: string) =>
           !!previous.nodes.find(n => n.name === name)?.alive &&
           !!system.nodes.find(n => n.name === name)?.alive;

       let checked = 0;
       for (const node of withPrevious.nodes) {
         if (!aliveInBoth(node.name)) continue;
         for (const source of node.sources) {
           assert.equal(
               typeof source.events_per_second, 'number',
               `${node.name} ${source.name} should have a measured rate`);
           // A timer's declared period is an independent statement of how
           // often it should fire, so the measured rate has to agree with it.
           if (source.kind !== 'TIMER' || source.period_ns === '0') continue;
           const expected = 1e9 / Number(source.period_ns);
           assert.ok(
               Math.abs(source.events_per_second - expected) < expected * 0.05,
               `${node.name} ${source.name}: ${
                   source.events_per_second}/s does not match its ${
                   expected}Hz period`);
           checked++;
         }
       }
       assert.ok(checked > 0, 'demo graph must contain a periodic timer');
     });

test('describe_topic names both ends of one topic and rejects unknown names',
     () => {
       const system = demoGraph();
       const connected =
           system.topics.find(t => t.publishers.length && t.subscribers.length);
       assert.ok(
           connected, 'demo graph must contain a topic with both ends wired');

       const result = dispatch(
           systemContext(system), 'describe_topic',
           {topic: connected!.name}) as any;

       assert.equal(result.name, connected!.name);
       assert.deepEqual(result.publishers, connected!.publishers);
       assert.deepEqual(result.subscribers, connected!.subscribers);
       assert.ok(['ok', 'lossy', 'idle'].includes(result.health));

       // One end per source on the topic, each naming its node and its kind.
       const expected = system.nodes.flatMap(
           node => node.sources.filter(s => s.name === connected!.name)
                       .map(s => ({node: node.name, kind: s.kind})));
       assert.deepEqual(
           result.ends.map((e: any) => ({node: e.node, kind: e.kind})),
           expected);
       assert.ok(expected.some(e => e.kind === 'SENDER'));
       assert.ok(
           expected.some(e => e.kind === 'WATCHER' || e.kind === 'FETCHER'));
       for (const end of result.ends)
         assert.equal(end.events_per_second, null);

       assert.throws(
           () => dispatch(
               systemContext(system), 'describe_topic', {topic: '/nope'}),
           (e: unknown) => e instanceof RpcFault && e.code === -32602);
       assert.throws(
           () => dispatch(systemContext(system), 'describe_topic', {}),
           (e: unknown) => e instanceof RpcFault && e.code === -32602);
     });

test('the declared graph answers what was supposed to be running', () => {
  const system = demoGraph(1000n);
  const declared = parseDeclaredGraph(demoDeclaredGraph());
  const result =
      dispatch(systemContext(system, null, declared), 'get_declared_graph', {}) as
      any;

  assert.equal(result.nodes.length, declared.nodes.length);
  assert.equal(result.summary.nodes, declared.nodes.length);
  assert.ok(result.summary.sources > result.summary.nodes);

  // The comparison, which is the reason this method exists: a name absent from
  // the registry is either a node that crashed on the way up or a node nobody
  // asked for, and only the declared graph can tell those apart.
  assert.equal(result.comparison.declared, declared.nodes.length);
  assert.equal(result.comparison.registered, system.nodes.length);
  assert.deepEqual(result.comparison.missing.map((n: any) => n.name),
                   ['shooter']);
  assert.ok(result.comparison.missing[0].target.includes('shooter'));
  assert.ok(result.comparison.missing[0].sources > 0);
  assert.deepEqual(result.comparison.undeclared, []);

  // Endpoint attributes survive the round trip, so an agent can see why a
  // half-wired topic is not a fault without asking a second question.
  const command = result.nodes.find((n: any) => n.name === 'hardware_node')
                      .sources.find((s: any) => s.name === '/hw/command');
  assert.equal(command.external, true);

  // The launcher's own findings, verbatim and counted.
  assert.equal(result.summary.errors, 0);
  assert.equal(result.summary.warnings, 1);
  assert.equal(result.diagnostics[0].subject, '/talos/telemetry');
  assert.match(result.diagnostics[0].message, /nothing subscribes/);

  // With no live graph the declared side is still an answer; the comparison is
  // null rather than a set of zeroes nobody can check.
  const alone =
      dispatch(systemContext(null, null, declared), 'get_declared_graph', {}) as
      any;
  assert.equal(alone.comparison, null);
  assert.equal(alone.nodes.length, declared.nodes.length);
});

test('get_latest returns the newest sample with its dictionary and age', () => {
  assert.throws(
      () => dispatch(ctx(new Timeline()), 'get_latest', {}),
      (e: unknown) => e instanceof RpcFault && e.code === -32002);
  const timeline = history();
  const latest = dispatch(ctx(timeline), 'get_latest', {}) as any;
  assert.equal(latest.sequence_id, '4');
  assert.equal(latest.stream_id, '42');
  assert.equal(latest.snapshot.sequence_id, '4');
  assert.equal(latest.units.velocity, 'm/s');
  assert.deepEqual(latest.labels.mode, ['disabled', 'teleop', 'auto']);
  assert.equal(typeof latest.age_ms, 'number');
  assert.ok(latest.age_ms >= 0);
});

test('get_system_graph says when the graph no longer describes now', () => {
  const system = demoGraph();
  // The judgment's clock is fixed, so no pause of the test's own can age the
  // fresh graph past the threshold.
  const nowMs = 50_000;
  const at = (systemArrivalMs: number) =>
      dispatch({...systemContext(system), systemArrivalMs, connected: true},
               'get_system_graph', {}, nowMs) as
      {stale: boolean|null; age_ms: number|null};
  // A hung bridge sends nothing; a graph past the threshold is last-known.
  const old = at(nowMs - GRAPH_STALE_AFTER_MS - 1);
  assert.equal(old.stale, true);
  assert.equal(old.age_ms, GRAPH_STALE_AFTER_MS + 1);
  assert.equal(at(nowMs - GRAPH_STALE_AFTER_MS + 1).stale, false);
});

test('every advertised method exists, and nothing else does', () => {
  // RPC_METHODS is what the Agent tab and the README list, so a name in it that
  // dispatch does not answer is a picker entry that fails on click.
  const context = systemContext(demoGraph(), null,
                                parseDeclaredGraph(demoDeclaredGraph()));
  for (const method of RPC_METHODS) {
    const response = handleRpc(
                         {...context, timeline: history()},
                         {jsonrpc: '2.0', id: 1, method, params: {
                           topic: '/pose_estimator/state',
                           timestamp_ns: '1000000000',
                           condition: {op: 'gt', value: 0}
                         }}) as any;
    assert.ok(!response.error || response.error.code !== -32601,
              `${method} is advertised and not implemented`);
  }
  assert.equal(
      (handleRpc(context, {jsonrpc: '2.0', id: 1, method: 'get_declared'}) as
       any).error.code,
      -32601);
});

// --- the declared graph over HTTP -------------------------------------------
//
// The declared graph is the one thing the agent fetches rather than being told,
// so the URL it derives and the answers it accepts are worth exercising against
// a real socket: a wrong path or an unhandled 404 would leave every declared
// view permanently empty and nothing would say why.

// `greeting` is sent as a binary message on every open, as the bridge sends
// a new client the dictionary pages and a keyframe of the latest tick.
async function fakeBridge(declared: string|null, greeting?: Uint8Array) {
  const server = createServer((request, response) => {
    if (request.url === '/declared.json' && declared !== null) {
      response.setHeader('Content-Type', 'application/json');
      response.end(declared);
      return;
    }
    // What a bridge started without --declared answers.
    response.writeHead(404).end();
  });
  const sockets = new WebSocketServer({server, path: '/telemetry'});
  if (greeting)
    sockets.on('connection', client => client.send(greeting, {binary: true}));
  server.listen(0, '127.0.0.1');
  await once(server, 'listening');
  const {port} = server.address() as AddressInfo;
  return {
    url: `ws://127.0.0.1:${port}/telemetry`,
    reconnect: (nextDeclared: string|null) => {
      declared = nextDeclared;
      for (const client of sockets.clients) client.terminate();
    },
    close: () => {
      for (const client of sockets.clients) client.terminate();
      sockets.close();
      server.close();
    }
  };
}

// Bounds a hang only (docs/TESTING.md §6.3); never the assertion.
const HANG_BOUND_MS = 10_000;

// Polls agent state that nothing announces. Turns of at least 10 ms each
// bound a hang at HANG_BOUND_MS or more.
const settle = async (until: () => boolean, what: string) => {
  for (let attempt = 0; attempt < HANG_BOUND_MS / 10; attempt++) {
    if (until()) return;
    await new Promise(resolve => setTimeout(resolve, 10));
  }
  assert.fail(what);
};

test('the agent fetches the declared graph from the bridge it follows',
     async () => {
       const bridge = await fakeBridge(demoDeclaredGraph());
       const agent = startAgent(0, bridge.url);
       try {
         await settle(() => !!agent.context().declared,
                      'the agent never fetched /declared.json');
         const result =
             dispatch(agent.context(), 'get_declared_graph', {}) as any;
         assert.equal(result.nodes.length, 7);
         assert.ok(result.nodes.some((n: any) => n.name === 'shooter'));
         // No system graph on this connection, so there is nothing to compare
         // the declared nodes against and it says so instead of guessing.
         assert.equal(result.comparison, null);
       } finally {
         agent.close();
         bridge.close();
       }
     });

// Resolves with the HTTP status of the agent's next fetch once the agent has
// handled the answer: its handling runs in microtasks after the fetch
// settles, so by this setImmediate turn it is done. The real fetch is
// restored on the first call; call the returned `restore` if none came.
function nextFetch() {
  const real = globalThis.fetch;
  const restore = () => {
    globalThis.fetch = real;
  };
  const handled = new Promise<number|null>(resolve => {
    globalThis.fetch = (input, init) => {
      restore();
      const response = real(input, init);
      response.then(r => setImmediate(resolve, r.status),
                    () => setImmediate(resolve, null));
      return response;
    };
  });
  return {handled, restore};
}

test('a bridge started without --declared leaves everything else working',
     {timeout: HANG_BOUND_MS}, async () => {
       const bridge = await fakeBridge(null);
       // The socket opening is what triggers the fetch, so waiting for a
       // parsed graph would pass vacuously; wait for the agent to have
       // handled the 404.
       const declaredFetch = nextFetch();
       const agent = startAgent(0, bridge.url);
       try {
         assert.equal(await declaredFetch.handled, 404);
         assert.equal(agent.context().declared, null);
         // Not an error, and not an empty graph: a question nobody answered.
         assert.throws(
             () => dispatch(agent.context(), 'get_declared_graph', {}),
             (e: unknown) => e instanceof RpcFault && e.code === -32002);
         // And the rest of the surface is unaffected.
         agent.ingestSystemGraph(demoSystemGraph(500n));
         const graph =
             dispatch(agent.context(), 'get_system_graph', {}) as any;
         assert.equal(graph.summary.nodes, 6);
       } finally {
         declaredFetch.restore();
         agent.close();
         bridge.close();
       }
     });

test('a reconnecting bridge 404 clears the previous declared graph', async () => {
  const bridge = await fakeBridge(demoDeclaredGraph());
  const agent = startAgent(0, bridge.url);
  try {
    await settle(() => !!agent.context().declared, 'initial graph missing');
    bridge.reconnect(null);
    await settle(() => agent.context().declared === null,
                 '404 left a stale declared graph');
    assert.throws(
        () => dispatch(agent.context(), 'get_declared_graph', {}),
        (e: unknown) => e instanceof RpcFault && e.code === -32002);
  } finally {
    agent.close();
    bridge.close();
  }
});

test('losing the bridge drops the live graph instead of reporting it alive',
     async () => {
       const bridge = await fakeBridge(demoDeclaredGraph());
       const agent = startAgent(0, bridge.url);
       try {
         // The declared fetch happens on open, so this waits for a connection.
         await settle(() => !!agent.context().declared, 'never connected');
         agent.ingestSystemGraph(demoSystemGraph(500n));
         assert.ok(agent.context().system);
         bridge.reconnect(demoDeclaredGraph());
         await settle(() => agent.context().system === null,
                      'a lost connection kept its graph');
         assert.throws(
             () => dispatch(agent.context(), 'get_system_graph', {}),
             (e: unknown) => e instanceof RpcFault && e.code === -32002);
       } finally {
         agent.close();
         bridge.close();
       }
     });

// REQ-OPS-23: a bridge sending a graph of another version is connected and
// talking, so "no system graph received yet" would be false. The agent
// answers with its own code and both versions, until a readable graph.
test('a graph of another version is answered by both versions, not as absent',
     async () => {
       const bridge = await fakeBridge(null);
       const agent = startAgent(0, bridge.url);
       try {
         const newer = {...JSON.parse(demoSystemGraph(500n)),
                        version: SYSTEM_GRAPH_VERSION + 1};
         agent.ingestSystemGraph(JSON.stringify(newer));
         const response = handleRpc(
             agent.context(),
             {jsonrpc: '2.0', id: 1, method: 'get_system_graph'}) as any;
         assert.equal(response.error.code, -32003);
         assert.deepEqual(response.error.data, {
           bridge_version: SYSTEM_GRAPH_VERSION + 1,
           reader_version: SYSTEM_GRAPH_VERSION
         });
         agent.ingestSystemGraph(demoSystemGraph(501n));
         assert.ok(dispatch(agent.context(), 'get_system_graph', {}));
       } finally {
         agent.close();
         bridge.close();
       }
     });

test('the cached frame sent on open has unknown age', async () => {
  const bridge = await fakeBridge(null, demoPacket(0n));
  const agent = startAgent(0, bridge.url);
  try {
    await settle(() => agent.timeline.frames.length === 1,
                 'the cached frame never reached the timeline');
    // It names the channels, but its values may predate this connection by
    // any amount: an age of ~0 would call a dead robot live.
    const latest = dispatch(agent.context(), 'get_latest', {}) as
        {sequence_id: string; age_ms: number|null; age_ns: string|null};
    assert.equal(latest.sequence_id, '0');
    assert.equal(latest.age_ms, null);
    assert.equal(latest.age_ns, null);
  } finally {
    agent.close();
    bridge.close();
  }
});

test('an oversized JSON-RPC batch is rejected whole', async () => {
  const bridge = await fakeBridge(null);
  const agent = startAgent(0, bridge.url);
  try {
    await settle(() => !!agent.server.address(), 'agent did not listen');
    const {port} = agent.server.address() as AddressInfo;
    const post = async (requests: number): Promise<unknown> =>
        (await fetch(`http://127.0.0.1:${port}/rpc`, {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(Array.from(
              {length: requests},
              (_, id) => ({jsonrpc: '2.0', id, method: 'get_schema_tree'})))
        })).json();
    assert.equal((await post(MAX_BATCH) as unknown[]).length, MAX_BATCH);
    const rejected = await post(MAX_BATCH + 1) as {error: {code: number}};
    assert.equal(rejected.error.code, -32600);
  } finally {
    agent.close();
    bridge.close();
  }
});

test('oversized RPC WebSocket payloads close only the offending client', async () => {
  const bridge = await fakeBridge(null);
  const agent = startAgent(0, bridge.url);
  let client: WebSocket|undefined;
  try {
    await settle(() => !!agent.server.address(), 'agent did not listen');
    const {port} = agent.server.address() as AddressInfo;
    client = new WebSocket(`ws://127.0.0.1:${port}/rpc`);
    client.on('error', () => {});
    await once(client, 'open');
    const closed = once(client, 'close');
    client.send(Buffer.alloc(65537));
    assert.equal((await closed)[0], 1009);
    const response = await fetch(`http://127.0.0.1:${port}/rpc`, {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({jsonrpc: '2.0', id: 1, method: 'get_schema_tree'})
    });
    assert.equal(response.status, 200);
    const body: unknown = await response.json();
    assert.ok(body !== null && typeof body === 'object' && 'id' in body);
    assert.equal(body.id, 1);
  } finally {
    client?.terminate();
    agent.close();
    bridge.close();
  }
});

// REQ-SAFE-35: the agent is a viewer. It may follow a bridge and answer RPC
// over TCP, and nothing else: no UDP socket (the only way it could reach the
// RoboRIO gateway), no process, and no file opened for writing.
const AGENT_DIR = fileURLToPath(new URL('../agent/', import.meta.url));
const STUDIO_ROOT = fileURLToPath(new URL('..', import.meta.url));
const FORBIDDEN_MODULES = new Set([
  'dgram', 'fs', 'fs/promises', 'child_process', 'worker_threads', 'cluster',
  'dns', 'net', 'tls', 'http2', 'vm', 'module'
]);

function walkAgentSourceFiles(dir: string): string[] {
  const files: string[] = [];
  for (const entry of readdirSync(dir, {withFileTypes: true})) {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) {
      files.push(...walkAgentSourceFiles(full));
    } else if (entry.isFile() && /\.(?:[cm]?js|tsx?)$/.test(entry.name)) {
      files.push(full);
    }
  }
  return files;
}

function stripJsComments(source: string): string {
  // Preserve string and template literals so URLs like 'http://...' stay
  // intact while line and block comments are blanked out.
  return source.replace(
      /'(?:\\.|[^'\\\n])*'|"(?:\\.|[^"\\\n])*"|`(?:\\.|[^`\\])*`|\/\/[^\n]*|\/\*[\s\S]*?\*\//g,
      match => match.startsWith('//') || match.startsWith('/*') ?
          match.replace(/[^\n]/g, ' ') :
          match);
}

function resolveLocalImport(fromFile: string, specifier: string): string {
  const base = path.resolve(path.dirname(fromFile), specifier);
  const candidates = [base];
  if (/\.(?:[cm]?js)$/.test(base)) {
    const stem = base.replace(/\.(?:[cm]?js)$/, '');
    candidates.push(`${stem}.ts`, `${stem}.tsx`, `${stem}.mts`, `${stem}.cts`);
  }
  candidates.push(
      `${base}.ts`, `${base}.tsx`, `${base}.mts`, `${base}.cts`,
      `${base}.js`, `${base}.mjs`, `${base}.cjs`,
      path.join(base, 'index.ts'), path.join(base, 'index.tsx'),
      path.join(base, 'index.js'), path.join(base, 'index.mjs'));
  for (const candidate of candidates) {
    if (existsSync(candidate) && statSync(candidate).isFile()) return candidate;
  }
  assert.fail(
      `Cannot resolve local import '${specifier}' from ${path.relative(STUDIO_ROOT, fromFile)}`);
}

function assertNoOsUdpOrRawSockets(expectedTcpPort: number): void {
  if (process.platform === 'linux') {
    const held = new Set<number>();
    for (const fd of readdirSync(`/proc/${process.pid}/fd`)) {
      let target: string;
      try {
        target = readlinkSync(`/proc/${process.pid}/fd/${fd}`);
      } catch {
        continue;
      }
      if (target.startsWith('socket:[') && target.endsWith(']')) {
        held.add(Number(target.slice('socket:['.length, -1)));
      }
    }
    const tableInodes = (tables: string[]) => {
      const inodes = new Set<number>();
      for (const table of tables) {
        const tablePath = `/proc/net/${table}`;
        if (!existsSync(tablePath)) continue;
        const lines = readFileSync(tablePath, 'utf8').split('\n').slice(1);
        for (const line of lines) {
          const cols = line.trim().split(/\s+/);
          if (cols.length > 9) inodes.add(Number(cols[9]));
        }
      }
      return inodes;
    };
    const forbidden = tableInodes(['udp', 'udp6', 'udplite', 'udplite6', 'raw', 'raw6']);
    const bad = [...held].filter(inode => forbidden.has(inode));
    assert.deepEqual(bad, [], `agent process holds UDP or raw sockets: ${bad.join(', ')}`);
    const tcp = tableInodes(['tcp', 'tcp6']);
    assert.ok([...held].some(inode => tcp.has(inode)),
              `none of ${[...held].join(', ')} is TCP: live socket inspection saw no TCP socket`);
    return;
  }
  if (process.platform === 'darwin') {
    const lsof = existsSync('/usr/sbin/lsof') ? '/usr/sbin/lsof' : 'lsof';
    let udpOut = '';
    try {
      udpOut = execFileSync(
          lsof, ['-n', '-P', '-a', '-p', String(process.pid), '-i', 'UDP'],
          {encoding: 'utf8'});
    } catch (e: unknown) {
      const err = e as {status?: number; stderr?: string; stdout?: string};
      assert.equal(err.status, 1, `lsof -i UDP failed: ${String(err.stderr ?? e)}`);
      assert.equal(String(err.stderr ?? ''), '', `lsof -i UDP stderr: ${err.stderr}`);
      udpOut = String(err.stdout ?? '');
    }
    assert.equal(udpOut.trim(), '', `agent process holds UDP sockets:\n${udpOut}`);
    const allOut = execFileSync(
        lsof, ['-n', '-P', '-a', '-p', String(process.pid), '-F', 'ftPTn'],
        {encoding: 'utf8'});
    let sawExpectedTcp = false;
    let current: Record<string, string> = {};
    const flush = () => {
      const type = current.t ?? '';
      const proto = current.P ?? '';
      const name = current.n ?? '';
      assert.notEqual(proto, 'UDP', `agent holds UDP socket: ${JSON.stringify(current)}`);
      assert.notEqual(proto, 'RAW', `agent holds RAW socket: ${JSON.stringify(current)}`);
      assert.notEqual(type, 'RAW', `agent holds RAW socket: ${JSON.stringify(current)}`);
      if (proto === 'TCP' && name.includes(`:${expectedTcpPort}`)) {
        sawExpectedTcp = true;
      }
    };
    for (const line of allOut.split('\n')) {
      if (!line) continue;
      if (line[0] === 'f') {
        if (current.f) flush();
        current = {f: line.slice(1)};
      } else {
        current[line[0]] = line.slice(1);
      }
    }
    if (current.f) flush();
    assert.ok(sawExpectedTcp,
              `lsof did not observe agent TCP listener on port ${expectedTcpPort}`);
  }
}

test('REQ-SAFE-35: no agent source or transitive local import can write or send',
     () => {
       const imported: string[] = [];
       const visited = new Set<string>();
       const queue: string[] = walkAgentSourceFiles(AGENT_DIR);

       while (queue.length > 0) {
         const file = queue.pop()!;
         if (visited.has(file)) continue;
         visited.add(file);
         const rel = path.relative(STUDIO_ROOT, file).replaceAll('\\', '/');
         const text = stripJsComments(readFileSync(file, 'utf8'));

         assert.ok(!/\bgetBuiltinModule\b/.test(text),
                   `${rel} references getBuiltinModule`);
         assert.ok(
             !/\bprocess\s*(?:\.\s*|\[\s*['"])(?:binding|_linkedBinding)\b/.test(text) &&
                 !/\b_linkedBinding\b/.test(text) &&
                 !/\.\s*binding\s*\(/.test(text),
             `${rel} references process.binding / _linkedBinding`);

         const recordSpecifier = (specifier: string) => {
           imported.push(`${rel}: ${specifier}`);
           assert.ok(!FORBIDDEN_MODULES.has(specifier.replace(/^node:/, '')),
                     `${rel} imports ${specifier}`);
           if (specifier.startsWith('.')) {
             queue.push(resolveLocalImport(file, specifier));
           }
         };

         // Static import / export ... from '...' and side-effect import '...'
         for (const m of text.matchAll(
                  /(?<![.\w$])(?:import|export)\b[\w$*,{}\s]+\bfrom\s*['"]([^'"]+)['"]|(?<![.\w$])import\s*['"]([^'"]+)['"]/g)) {
           recordSpecifier(m[1] ?? m[2]);
         }
         // Dynamic import(...) must use a plain string literal.
         for (const m of text.matchAll(/(?<![.\w$])import\s*\(/g)) {
           const rest = text.slice(m.index! + m[0].length);
           const lit = rest.match(/^\s*(?:'([^'\\\n]+)'|"([^"\\\n]+)")\s*\)/);
           assert.ok(lit, `${rel} uses non-literal dynamic import(...)`);
           recordSpecifier(lit[1] ?? lit[2]);
         }
         // require(...) must also use a plain string literal.
         for (const m of text.matchAll(/(?<![.\w$])require\s*\(/g)) {
           const rest = text.slice(m.index! + m[0].length);
           const lit = rest.match(/^\s*(?:'([^'\\\n]+)'|"([^"\\\n]+)")\s*\)/);
           assert.ok(lit, `${rel} uses non-literal require(...)`);
           recordSpecifier(lit[1] ?? lit[2]);
         }
       }

       // Confirm the scan walked both studio/agent/ and transitive ../src/ files,
       // so an empty or shallow walk cannot pass.
       const relVisited = new Set(
           [...visited].map(f => path.relative(STUDIO_ROOT, f).replaceAll('\\', '/')));
       assert.ok(imported.some(i => i.endsWith(': ws')), imported.join('\n'));
       for (const expected of [
              'agent/server.ts', 'agent/rpc.ts', 'agent/run.mjs',
              'src/core.ts', 'src/history.ts', 'src/system.ts',
              'src/data/freshness.ts', 'src/data/validate.ts', 'src/encode.ts'
            ]) {
         assert.ok(relVisited.has(expected),
                   `expected transitive scan to include ${expected}; saw ${[...relVisited].join(', ')}`);
       }
     });

test('REQ-SAFE-35: a running agent opens no datagram socket and no file for writing',
     {timeout: HANG_BOUND_MS}, async () => {
       const opened: string[] = [];
       const writable = (flags: unknown) =>
           typeof flags === 'number' ?
           (flags & (fs.constants.O_WRONLY | fs.constants.O_RDWR)) !== 0 :
           typeof flags === 'string' && /[wa+]/.test(flags);
       const real = {
         createSocket: dgram.createSocket,
         open: fs.open,
         openSync: fs.openSync,
         promisesOpen: fs.promises.open,
         createWriteStream: fs.createWriteStream
       };
       const note = (what: string, flags: unknown) => {
         if (writable(flags)) opened.push(`${what} ${String(flags)}`);
       };
       (dgram as any).createSocket = (...args: unknown[]) => {
         opened.push('dgram');
         return (real.createSocket as any)(...args);
       };
       (fs as any).open = (path: unknown, flags: unknown, ...rest: unknown[]) => {
         note('open', flags);
         return (real.open as any)(path, flags, ...rest);
       };
       (fs as any).openSync = (path: unknown, flags: unknown, ...rest: unknown[]) => {
         note('openSync', flags);
         return (real.openSync as any)(path, flags, ...rest);
       };
       (fs.promises as any).open = (path: unknown, flags: unknown, ...rest: unknown[]) => {
         note('promises.open', flags);
         return (real.promisesOpen as any)(path, flags, ...rest);
       };
       (fs as any).createWriteStream = (...args: unknown[]) => {
         opened.push('createWriteStream');
         return (real.createWriteStream as any)(...args);
       };
       const bridge = await fakeBridge(demoDeclaredGraph(), demoPacket(0n));
       const agent = startAgent(0, bridge.url);
       let wsClient: WebSocket|undefined;
       try {
         await settle(() => !!agent.context().declared &&
                            agent.timeline.latest() !== null,
                      'the agent never received the bridge\'s data');
         agent.ingestSystemGraph(demoSystemGraph(500n));
         await settle(() => !!agent.server.address(), 'agent did not listen');
         const {port} = agent.server.address() as AddressInfo;

         // Exercise /health plus every advertised RPC method over HTTP POST /rpc.
         const health = await fetch(`http://127.0.0.1:${port}/health`);
         assert.equal(health.status, 200);

         const latestFrame = agent.timeline.latest()!;
         const rpcParams: Record<(typeof RPC_METHODS)[number], Record<string, unknown>> = {
           get_latest: {},
           get_schema_tree: {},
           query_state_at: {timestamp_ns: latestFrame.timestamp_ns, mode: 'exact'},
           scan_channel_events: {
             topic: 'velocity',
             condition: {op: 'gt', value: -1e9}
           },
           get_system_graph: {},
           describe_topic: {topic: '/pose_estimator/state'},
           get_declared_graph: {}
         };
         for (let i = 0; i < RPC_METHODS.length; i++) {
           const method = RPC_METHODS[i];
           const rpc = await fetch(`http://127.0.0.1:${port}/rpc`, {
             method: 'POST', headers: {'Content-Type': 'application/json'},
             body: JSON.stringify(
                 {jsonrpc: '2.0', id: i + 1, method, params: rpcParams[method]})
           });
           assert.equal(rpc.status, 200, `${method} HTTP status`);
           const body = await rpc.json() as {result?: unknown; error?: unknown};
           assert.equal(body.error, undefined, `${method} returned error: ${JSON.stringify(body.error)}`);
           assert.notEqual(body.result, undefined, `${method} returned no result`);
         }

         // Exercise the WebSocket /rpc transport as well.
         wsClient = new WebSocket(`ws://127.0.0.1:${port}/rpc`);
         await once(wsClient, 'open');
         const wsReply = once(wsClient, 'message');
         wsClient.send(JSON.stringify([
           {jsonrpc: '2.0', id: 100, method: 'get_latest'},
           {jsonrpc: '2.0', id: 101, method: 'get_system_graph'}
         ]));
         const [rawReply] = await wsReply;
         const batch = JSON.parse(String(rawReply)) as {result?: unknown; error?: unknown}[];
         assert.equal(batch.length, 2);
         for (const entry of batch) {
           assert.equal(entry.error, undefined);
           assert.notEqual(entry.result, undefined);
         }

         // Inspect real OS open sockets while the agent is live and serving.
         assertNoOsUdpOrRawSockets(port);
       } finally {
         wsClient?.terminate();
         agent.close();
         bridge.close();
         Object.assign(dgram, {createSocket: real.createSocket});
         Object.assign(fs, {
           open: real.open,
           openSync: real.openSync,
           createWriteStream: real.createWriteStream
         });
         Object.assign(fs.promises, {open: real.promisesOpen});
       }
       assert.deepEqual(opened, []);
     });

test('chunked /declared.json past MAX_DECLARED_BYTES stops reading and records declaredError',
     {timeout: HANG_BOUND_MS}, async () => {
       let streamClosed = false;
       const server = createServer((req, res) => {
         if (req.url === '/declared.json') {
           // Chunked response with no Content-Length, kept open past the cap
           // so an unbounded response.text() read would hang indefinitely.
           res.writeHead(200, {'Content-Type': 'application/json'});
           res.on('close', () => { streamClosed = true; });
           const chunk = Buffer.alloc(65536, 0x20);
           const count = Math.floor(MAX_DECLARED_BYTES / chunk.length) + 1;
           for (let i = 0; i < count; i++) res.write(chunk);
           return;
         }
         res.writeHead(404).end();
       });
       const sockets = new WebSocketServer({server, path: '/telemetry'});
       server.listen(0, '127.0.0.1');
       await once(server, 'listening');
       const {port} = server.address() as AddressInfo;
       const agent = startAgent(0, `ws://127.0.0.1:${port}/telemetry`);
       try {
         await settle(() => streamClosed,
                      'oversized chunked /declared.json stream was not aborted');
         await settle(() => !!agent.server.address(), 'agent did not listen');
         const agentPort = (agent.server.address() as AddressInfo).port;
         const health = await (await fetch(
             `http://127.0.0.1:${agentPort}/health`)).json() as {
           declared: {received: boolean; error: string|null};
         };
         assert.equal(agent.context().declared, null);
         assert.equal(health.declared.received, false);
         assert.equal(
             health.declared.error, 'Invalid declared graph: document too large');
       } finally {
         agent.close();
         for (const client of sockets.clients) client.terminate();
         sockets.close();
         server.close();
         server.closeAllConnections();
       }
     });

test('stale declared fetch cannot overwrite a newer 404, and close aborts in-flight fetches',
     {timeout: HANG_BOUND_MS}, async () => {
       const bridge = await fakeBridge(null);
       const realFetch = globalThis.fetch;
       let callCount = 0;
       let firstSignal: AbortSignal|undefined;
       let firstStartedResolve!: () => void;
       const firstStarted = new Promise<void>(r => { firstStartedResolve = r; });
       let releaseFirst!: () => void;
       const firstGate = new Promise<void>(r => { releaseFirst = r; });
       let secondHandledResolve!: (status: number) => void;
       const secondHandled =
           new Promise<number>(r => { secondHandledResolve = r; });

       globalThis.fetch = (input, init) => {
         const url = typeof input === 'string' ? input : input.toString();
         if (!url.endsWith('/declared.json')) return realFetch(input, init);
         callCount++;
         if (callCount === 1) {
           firstSignal = init?.signal ?? undefined;
           firstStartedResolve();
           // Resolve with a stale 200 after the gate opens, even if aborted,
           // to verify the agent ignores stale connection results.
           return firstGate.then(
               () => new Response(demoDeclaredGraph(), {status: 200}));
         }
         const response = realFetch(input, init);
         response.then(r => setImmediate(secondHandledResolve, r.status),
                       () => setImmediate(secondHandledResolve, -1));
         return response;
       };

       const agent = startAgent(0, bridge.url);
       try {
         await firstStarted;
         assert.ok(firstSignal, 'fetchDeclaredGraph must pass an AbortSignal');
         bridge.reconnect(null);
         await settle(() => firstSignal?.aborted === true,
                      'closing upstream did not abort in-flight declared fetch');
         assert.equal(await secondHandled, 404);
         releaseFirst();
         await new Promise(r => setImmediate(r));
         await new Promise(r => setImmediate(r));
         assert.equal(agent.context().declared, null);
       } finally {
         releaseFirst();
         globalThis.fetch = realFetch;
         agent.close();
         bridge.close();
       }

       // Verify close() aborts an in-flight declared fetch immediately.
       const bridge2 = await fakeBridge(null);
       let closeSignal: AbortSignal|undefined;
       let closeStartedResolve!: () => void;
       const closeStarted = new Promise<void>(r => { closeStartedResolve = r; });
       globalThis.fetch = (input, init) => {
         const url = typeof input === 'string' ? input : input.toString();
         if (!url.endsWith('/declared.json')) return realFetch(input, init);
         closeSignal = init?.signal ?? undefined;
         closeStartedResolve();
         return new Promise<Response>((_, reject) => {
           init?.signal?.addEventListener(
               'abort', () => reject(new DOMException('Aborted', 'AbortError')));
         });
       };
       const agent2 = startAgent(0, bridge2.url);
       try {
         await closeStarted;
         assert.ok(closeSignal && !closeSignal.aborted);
         agent2.close();
         assert.equal(closeSignal.aborted, true);
       } finally {
         globalThis.fetch = realFetch;
         agent2.close();
         bridge2.close();
       }
     });

test('reconnecting to a restarted session resets rateWindow and decoder',
     {timeout: HANG_BOUND_MS}, async () => {
       // Build a 100-channel 2-page update so connection 1 can drop mid-update.
       const channels: ChannelSpec[] = Array.from(
           {length: 100}, (_, i) => ({name: `ch_${i}`, unit: ''}));
       const pose = {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0};
       const header1: PageHeader = {
         streamId: 99n,
         sequenceId: 1n,
         timestampNs: 1_000_000_000n,
         dictionaryId: 7,
         channelCount: channels.length
       };
       const header2: PageHeader = {
         ...header1,
         sequenceId: 2n,
         timestampNs: 2_000_000_000n
       };
       const dictPages = encodeDictionaryPages(header1, channels);
       const valPages1 = encodeValuePages(
           header1, pose, true,
           channels.map((_, i): [number, bigint] => [i, rawFromDouble(i)]));
       assert.equal(valPages1.length, 2);
       const valPages2 = encodeValuePages(
           header2, pose, true,
           channels.map(
               (_, i): [number, bigint] => [i, rawFromDouble(i === 0 ? 42 : i)]));

       const shiftWall = (raw: string, deltaNs: bigint) => {
         const doc = JSON.parse(raw);
         doc.wall_ns = (BigInt(doc.wall_ns) + deltaNs).toString();
         return JSON.stringify(doc);
       };
       const session1Graph = demoSystemGraph(1000n);
       const session2Graph1 = shiftWall(demoSystemGraph(0n), 20_500_000_000n);
       const session2Graph2 = shiftWall(demoSystemGraph(25n), 20_500_000_000n);
       const expectedWallNs2 = parseSystemGraph(session2Graph2).wall_ns;

       let connections = 0;
       const server = createServer((_, res) => res.writeHead(404).end());
       const sockets = new WebSocketServer({server, path: '/telemetry'});
       sockets.on('connection', client => {
         connections++;
         if (connections === 1) {
           client.send(session1Graph);
           // Send dictionary + page 0 of a 2-page keyframe, then drop mid-update.
           client.send(concatPages(dictPages), {binary: true});
           client.send(valPages1[0], {binary: true});
         } else {
           // Session 2 restarted with counters reset to 0.
           client.send(session2Graph1);
           client.send(session2Graph2);
           // Orphan page 1 from session 1 must NOT stitch onto page 0; then a
           // complete dictionary + 2-page keyframe must decode cleanly.
           client.send(valPages1[1], {binary: true});
           client.send(concatPages([...dictPages, ...valPages2]), {binary: true});
         }
       });
       server.listen(0, '127.0.0.1');
       await once(server, 'listening');
       const {port} = server.address() as AddressInfo;
       const agent = startAgent(0, `ws://127.0.0.1:${port}/telemetry`);
       try {
         await settle(() => agent.context().system !== null,
                      'connection 1 never received system graph');
         // Drop connection 1 to trigger socket.on('close') resets and reconnect.
         for (const client of sockets.clients) client.terminate();
         await settle(() => agent.context().system?.wall_ns === expectedWallNs2,
                      'connection 2 never received restarted system graphs');
         await settle(() => agent.timeline.frames.length >= 1,
                      'connection 2 never decoded clean telemetry');
         // Allow any buffered frame to flush so an erroneously stitched frame
         // would also be visible.
         await new Promise(r => setTimeout(r, 30));

         // RateWindow reset: valid positive rate on the 2nd graph of session 2.
         const graph =
             dispatch(agent.context(), 'get_system_graph', {}) as any;
         const drivetrain =
             graph.nodes.find((n: any) => n.name === 'drivetrain');
         const timer = drivetrain.sources.find(
             (s: any) => s.kind === 'TIMER' && s.period_ns !== '0');
         const expectedHz = 1e9 / Number(timer.period_ns);
         assert.equal(typeof timer.events_per_second, 'number');
         assert.ok(
             Math.abs(timer.events_per_second - expectedHz) < expectedHz * 0.05,
             `expected ~${expectedHz} Hz, got ${timer.events_per_second}`);

         // Decoder reset: only session 2's clean frame landed (no stitched seq 1).
         assert.equal(agent.timeline.frames.length, 1);
         assert.equal(agent.timeline.frames[0].sequence_id, '2');
         assert.equal(agent.timeline.frames[0].channels['ch_0'], 42);
       } finally {
         agent.close();
         for (const client of sockets.clients) client.terminate();
         sockets.close();
         server.close();
         server.closeAllConnections();
       }
     });

test('get_schema_tree uses a null prototype and walks shared dictionary objects once',
     () => {
       const decoder = new TelemetryDecoder();
       const base = decoder.decode(demoPacket(0n))!;
       const sharedEnumLabels =
           Object.freeze(Array.from({length: 128}, (_, i) => `state_${i}`));
       const rawLabels: Record<string, string[]> = Object.freeze(Object.assign(
           Object.create(null), base.labels,
           {shared_a: sharedEnumLabels, shared_b: sharedEnumLabels}));
       let unitsWalks = 0;
       let labelsWalks = 0;
       let alertsWalks = 0;
       const countWalks = <T extends object>(target: T, bump: () => void): T =>
           new Proxy(target, {
             ownKeys(obj) {
               bump();
               return Reflect.ownKeys(obj);
             }
           });
       const sharedUnits = countWalks(base.units, () => { unitsWalks++; });
       const sharedLabels = countWalks(rawLabels, () => { labelsWalks++; });
       const sharedAlerts =
           countWalks(base.alerts ?? Object.create(null), () => { alertsWalks++; });

       const timeline = new Timeline();
       for (let i = 0; i < 5000; i++) {
         const channels: Record<string, number> = Object.assign(
             Object.create(null), base.channels);
         if (i === 0) channels['__proto__'] = 1;
         if (i === 4999) channels['late/channel'] = 2;
         const frame: Snapshot = {
           ...base,
           sequence_id: String(i),
           timestamp_ns: String(1_000_000_000n + BigInt(i) * 1_000_000n),
           chassis: {...base.chassis},
           channels,
           units: sharedUnits,
           labels: sharedLabels,
           alerts: sharedAlerts
         };
         timeline.append(frame);
       }

       const result = dispatch(ctx(timeline), 'get_schema_tree') as {
         topics: Record<string, string>;
         frames: number;
       };
       assert.equal(result.frames, 5000);
       assert.equal(Object.getPrototypeOf(result.topics), null);
       assert.equal(unitsWalks, 1);
       assert.equal(labelsWalks, 1);
       assert.equal(alertsWalks, 1);
       assert.equal(result.topics['velocity'], 'double');
       assert.equal(result.topics['channels.velocity'], 'number');
       assert.equal(result.topics['mode'], 'uint');
       assert.equal(result.topics['units.velocity'], 'string');
       assert.equal(result.topics['labels.mode.0'], 'string');
       assert.equal(result.topics['labels.shared_a.127'], 'string');
       assert.equal(result.topics['labels.shared_b.127'], 'string');
       assert.equal(
           result.topics['alerts.hw/status/alert_error_link_lost'], 'number');
       assert.equal(result.topics['late/channel'], 'double');
       assert.equal(result.topics['__proto__'], 'double');
       assert.equal(({} as Record<string, unknown>)['polluted'], undefined);
     });

test('HTTP/WS reject empty Origin, guard multi-chunk >64 KiB POST, cap WS clients, and close keep-alive on shutdown',
     {timeout: HANG_BOUND_MS}, async () => {
       const bridge = await fakeBridge(null);
       const agent = startAgent(0, bridge.url);
       const wsClients: WebSocket[] = [];
       const keepAliveAgent = new HttpAgent({keepAlive: true});
       try {
         await settle(() => !!agent.server.address(), 'agent did not listen');
         const {port} = agent.server.address() as AddressInfo;

         // (a) Empty Origin header is rejected on HTTP and WebSocket upgrade.
         const statusFor = (method: string, path: string,
                            headers: Record<string, string>) =>
             new Promise<number>((resolve, reject) => {
               const req = httpRequest(
                   {host: '127.0.0.1', port, method, path, headers}, res => {
                     res.resume();
                     res.on('end', () => resolve(res.statusCode ?? 0));
                   });
               req.on('error', reject);
               req.end();
             });
         assert.equal(await statusFor('GET', '/health', {Origin: ''}), 403);
         assert.equal(await statusFor('POST', '/rpc', {Origin: ''}), 403);

         const emptyOriginWs = new WebSocket(
             `ws://127.0.0.1:${port}/rpc`, {headers: {Origin: ''}});
         const emptyOriginRejected = await new Promise<boolean>(resolve => {
           emptyOriginWs.on('open', () => resolve(false));
           emptyOriginWs.on('error', () => resolve(true));
           emptyOriginWs.on('close', () => resolve(true));
         });
         emptyOriginWs.terminate();
         assert.equal(emptyOriginRejected, true);

         // (b) Multi-chunk >64 KiB POST returns 413 cleanly without crashing.
         const oversizedStatus = await new Promise<number>(resolve => {
           const req = httpRequest(
               {
                 host: '127.0.0.1',
                 port,
                 method: 'POST',
                 path: '/rpc',
                 headers: {
                   'Content-Type': 'application/json',
                   'Transfer-Encoding': 'chunked'
                 }
               },
               res => {
                 res.resume();
                 res.on('end', () => resolve(res.statusCode ?? 0));
                 res.on('close', () => resolve(res.statusCode ?? 0));
               });
           // Server destroys the request stream after writing 413.
           req.on('error', () => {});
           const chunk = Buffer.alloc(20_000, 0x61);
           for (let i = 0; i < 5; i++) req.write(chunk);
           req.end();
         });
         assert.equal(oversizedStatus, 413);
         const okRpc = await fetch(`http://127.0.0.1:${port}/rpc`, {
           method: 'POST',
           headers: {'Content-Type': 'application/json'},
           body: JSON.stringify(
               {jsonrpc: '2.0', id: 1, method: 'get_schema_tree'})
         });
         assert.equal(okRpc.status, 200);

         // (c) WebSocket client cap enforces MAX_WS_CLIENTS.
         for (let i = 0; i < MAX_WS_CLIENTS; i++) {
           const wsClient = new WebSocket(`ws://127.0.0.1:${port}/rpc`);
           wsClient.on('error', () => {});
           await once(wsClient, 'open');
           wsClients.push(wsClient);
         }
         const overflowWs = new WebSocket(`ws://127.0.0.1:${port}/rpc`);
         const overflowRejected = await new Promise<boolean>(resolve => {
           overflowWs.on('open', () => resolve(false));
           overflowWs.on('error', () => resolve(true));
           overflowWs.on('close', () => resolve(true));
         });
         overflowWs.terminate();
         assert.equal(overflowRejected, true);
         for (const wsClient of wsClients) wsClient.terminate();
         wsClients.length = 0;

         // (d) Idle HTTP keep-alive closes promptly on agent.close().
         agent.server.keepAliveTimeout = 60_000;
         await new Promise<void>((resolve, reject) => {
           const req = httpRequest(
               {
                 host: '127.0.0.1',
                 port,
                 method: 'GET',
                 path: '/health',
                 agent: keepAliveAgent
               },
               res => {
                 res.resume();
                 res.on('end', () => resolve());
               });
           req.on('error', reject);
           req.end();
         });
         const serverClosed = once(agent.server, 'close');
         agent.close();
         await serverClosed;
       } finally {
         for (const wsClient of wsClients) wsClient.terminate();
         keepAliveAgent.destroy();
         agent.close();
         bridge.close();
       }
     });
