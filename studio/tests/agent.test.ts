import assert from 'node:assert/strict';
import {createServer} from 'node:http';
import type {AddressInfo} from 'node:net';
import {test} from 'node:test';

import {WebSocketServer} from 'ws';

import {
  dispatch,
  handleRpc,
  RPC_METHODS,
  type RpcContext,
  RpcFault,
  startAgent,
} from '../agent/server';
import {decode, Timeline} from '../src/core';
import {demoDeclaredGraph, demoPacket, demoSystemGraph} from '../src/demo';
import {
  type DeclaredGraph,
  parseDeclaredGraph,
  parseSystemGraph,
  type SystemGraph,
} from '../src/system';

function history() {
  const timeline = new Timeline();
  [0, 4, 5, 0, 7].forEach((current, i) => {
    const frame = decode(demoPacket(BigInt(i)));
    frame.channels['motor.current'] = current;
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
                         topics: ['motor.current', 'chassis.x']
                       }) as any;
      assert.equal(response.timestamp_ns, '1020000000');
      assert.equal(response.topics['motor.current'], 4);
      assert.equal(typeof response.topics['chassis.x'], 'number');
      assert.throws(
          () => dispatch(
              ctx(timeline), 'query_state_at',
              {timestamp_ns: '18446744073709551616'}),
          /uint64/);
    });
test(
    'threshold scans group samples and edge scans use the preceding sample outside bounds',
    () => {
      const timeline = history();
      const result =
          dispatch(
              ctx(timeline), 'scan_channel_events',
              {topic: 'motor.current', condition: {op: 'gt', value: 3}}) as any;
      assert.deepEqual(result.ranges, [
        {start_ns: '1020000000', end_ns: '1040000000', samples: 2},
        {start_ns: '1080000000', end_ns: '1080000000', samples: 1}
      ]);
      const edge = dispatch(ctx(timeline), 'scan_channel_events', {
                     topic: 'motor.current',
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
        () => dispatch(systemContext(null), method, {topic: '/odometry/state'}),
        (e: unknown) => e instanceof RpcFault && e.code === -32002);
  }
  // The declared graph is the same distinction one step earlier: the bridge may
  // have been started without --declared, which is not "the launcher declared
  // nothing".
  assert.throws(
      () => dispatch(systemContext(demoGraph()), 'get_declared_graph', {}),
      (e: unknown) => e instanceof RpcFault && e.code === -32002 &&
          /--declared/.test(e.message));
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
       // Which document the classification came from: below version 2 those two
       // states are unreachable and their topics read as faults.
       assert.equal(result.version, 2);
       assert.equal(result.summary.endpoint_attributes, true);
       assert.ok(result.summary.unconnected_topics >= 3);
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

test('every advertised method exists, and nothing else does', () => {
  // RPC_METHODS is what the Agent tab and the README list, so a name in it that
  // dispatch does not answer is a picker entry that fails on click.
  const context = systemContext(demoGraph(), null,
                                parseDeclaredGraph(demoDeclaredGraph()));
  for (const method of RPC_METHODS) {
    const response = handleRpc(
                         {...context, timeline: history()},
                         {jsonrpc: '2.0', id: 1, method, params: {
                           topic: '/odometry/state',
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

function fakeBridge(declared: string|null) {
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
  return new Promise<{url: string; close: () => void}>(resolve => {
    server.listen(0, '127.0.0.1', () => {
      const {port} = server.address() as AddressInfo;
      resolve({
        url: `ws://127.0.0.1:${port}/telemetry`,
        close: () => {
          for (const client of sockets.clients) client.terminate();
          sockets.close();
          server.close();
        }
      });
    });
  });
}

const settle = async (until: () => boolean, what: string) => {
  for (let attempt = 0; attempt < 200; attempt++) {
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
         assert.equal(result.nodes.length, 8);
         assert.ok(result.nodes.some((n: any) => n.name === 'shooter'));
         // No system graph on this connection, so there is nothing to compare
         // the declared nodes against and it says so instead of guessing.
         assert.equal(result.comparison, null);
       } finally {
         agent.close();
         bridge.close();
       }
     });

test('a bridge started without --declared leaves everything else working',
     async () => {
       const bridge = await fakeBridge(null);
       const agent = startAgent(0, bridge.url);
       try {
         // The socket opening is what triggers the fetch, so waiting for a
         // parsed graph would pass vacuously; wait for the 404 to have been
         // asked for and answered.
         await settle(() => agent.graphs.received === 0 &&
                         !!agent.context().timeline,
                     'the agent never came up');
         await new Promise(resolve => setTimeout(resolve, 150));
         assert.equal(agent.context().declared, null);
         // Not an error, and not an empty graph: a question nobody answered.
         assert.throws(
             () => dispatch(agent.context(), 'get_declared_graph', {}),
             (e: unknown) => e instanceof RpcFault && e.code === -32002);
         // And the rest of the surface is unaffected.
         agent.ingestSystemGraph(demoSystemGraph(500n));
         const graph =
             dispatch(agent.context(), 'get_system_graph', {}) as any;
         assert.equal(graph.summary.nodes, 7);
       } finally {
         agent.close();
         bridge.close();
       }
     });
