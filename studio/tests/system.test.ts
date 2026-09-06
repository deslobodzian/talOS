import assert from 'node:assert/strict';
import {test} from 'node:test';

import {demoDeclaredGraph, demoPacket, demoSystemGraph} from '../src/demo';
import {
  DECLARED_VERSION,
  type DeclaredGraph,
  declaredStatus,
  declaredUrl,
  endpointAttributesKnown,
  eventRates,
  isSubscriber,
  MAX_DECLARED_BYTES,
  MAX_SYSTEM_BYTES,
  parseDeclaredGraph,
  parseSystemGraph,
  rateKey,
  RateWindow,
  SYSTEM_GRAPH_VERSION,
  SYSTEM_HEADER_BYTES,
  SYSTEM_TAG,
  SystemAssembler,
  systemChunk,
  type SystemGraph,
  type SystemTopic,
  TOPIC_HEALTH,
  topicEnds,
  topicHealth,
  topicHealthOf
} from '../src/system';

type Json = Record<string, unknown>;

// Fixtures are built by overriding one field of a known-good document, so a
// rejection test can only fail for the reason it names.
const source = (over: Json = {}): Json => ({
  id: 0,
  kind: 'SENDER',
  name: '/odometry/state',
  message_bytes: 64,
  alignment: 8,
  period_ns: '0',
  events: '10',
  dropped: '0',
  sequence: '9',
  last_monotonic_ns: '0',
  last_latency_ns: '0',
  max_latency_ns: '0',
  ...over
});
const node = (over: Json = {}): Json => ({
  name: 'odometry',
  target: '//2026-robot/main_processor/odometry:node',
  slot: 0,
  pid: '4242',
  session_id: '18446744073709551615',
  generation: '1',
  start_wall_ns: '1700000000000000000',
  heartbeat_wall_ns: '1700000000500000000',
  dispatch_count: '9007199254740993',
  declared_sources: 1,
  alive: true,
  simulation: false,
  replay: false,
  sources: [source()],
  ...over
});
const topic = (over: Json = {}): Json => ({
  name: '/odometry/state',
  message_bytes: 64,
  publishers: ['odometry'],
  subscribers: ['telemetry'],
  published: '10',
  received: '9',
  dropped: '0',
  ...over
});
const document = (over: Json = {}): Json => ({
  kind: 'talos.system_graph',
  version: 1,
  wall_ns: '1700000000500000000',
  registry: {
    available: true,
    node_capacity: 32,
    source_capacity: 64,
    liveness_timeout_ns: '5000000000'
  },
  bridge: {
    topic: '/talos/telemetry',
    clients: 1,
    frames_forwarded: '7',
    invalid: '0',
    udp_drops: '0',
    ws_drops: '0'
  },
  nodes: [node()],
  topics: [topic()],
  ...over
});
const parse = (over: Json = {}): SystemGraph =>
    parseSystemGraph(JSON.stringify(document(over)));
const rejects = (over: Json, because: RegExp) =>
    assert.throws(() => parse(over), because);

test('a well-formed graph parses with both ends of every topic intact', () => {
  const graph = parse();
  assert.equal(graph.version, 1);
  assert.equal(graph.registry.available, true);
  assert.equal(graph.registry.node_capacity, 32);
  assert.equal(graph.bridge.topic, '/talos/telemetry');
  assert.equal(graph.bridge.clients, 1);
  assert.equal(graph.nodes.length, 1);
  assert.equal(graph.nodes[0].name, 'odometry');
  assert.equal(graph.nodes[0].sources[0].kind, 'SENDER');
  assert.deepEqual(graph.topics[0], {
    name: '/odometry/state',
    message_bytes: 64,
    publishers: ['odometry'],
    subscribers: ['telemetry'],
    published: '10',
    received: '9',
    dropped: '0'
  } satisfies SystemTopic);
});

test('uint64 counters survive as decimal strings past 2^53', () => {
  const graph = parse();
  // The whole reason these are strings: as a JSON number the same value comes
  // back off by one, silently.
  assert.equal(
      (JSON.parse('{"n":9007199254740993}') as {n: number}).n,
      9007199254740992);
  assert.equal(graph.nodes[0].dispatch_count, '9007199254740993');
  assert.equal(graph.nodes[0].session_id, '18446744073709551615');
  assert.equal(BigInt(graph.nodes[0].session_id), 2n ** 64n - 1n);
  assert.equal(graph.wall_ns, '1700000000500000000');
});

test('a document of the wrong kind is refused, not read hopefully', () => {
  rejects({kind: 'talos.telemetry'}, /unexpected document kind/);
  rejects({kind: 1}, /unexpected document kind/);
});

test('a number where a uint64 string belongs is a rejection', () => {
  rejects({wall_ns: 1700000000500000000}, /wall_ns must be a decimal integer/);
  rejects({nodes: [node({dispatch_count: 5})]}, /dispatch_count/);
  rejects({nodes: [node({sources: [source({events: 10})]})]}, /events/);
  rejects({topics: [topic({published: null})]}, /published/);
  // Twenty digits is a full uint64; twenty-one is not a number we accept.
  rejects({wall_ns: '1'.repeat(21)}, /wall_ns must be a decimal integer/);
  rejects({wall_ns: '12.5'}, /wall_ns must be a decimal integer/);
});

test('byte counts must be non-negative integers', () => {
  rejects({nodes: [node({sources: [source({message_bytes: 1.5})]})]},
          /message_bytes must be a non-negative integer/);
  rejects({nodes: [node({sources: [source({message_bytes: -1})]})]},
          /message_bytes must be a non-negative integer/);
  rejects({nodes: [node({sources: [source({message_bytes: '64'})]})]},
          /message_bytes must be a non-negative integer/);
  rejects({topics: [topic({message_bytes: 2 ** 33})]},
          /message_bytes must be a non-negative integer/);
});

test('an unknown source kind is refused rather than rendered blank', () => {
  rejects({nodes: [node({sources: [source({kind: 'PUBLISHER'})]})]},
          /kind must be one of/);
  rejects({nodes: [node({sources: [source({kind: 'sender'})]})]},
          /kind must be one of/);
  rejects({nodes: [node({sources: [source({kind: 4})]})]}, /kind must be a/);
});

test('liveness and mode flags must be actual booleans', () => {
  rejects({nodes: [node({alive: 'true'})]}, /alive must be a boolean/);
  rejects({nodes: [node({simulation: 1})]}, /simulation must be a boolean/);
  rejects({nodes: [node({replay: null})]}, /replay must be a boolean/);
});

test('collections beyond their caps are refused before being walked', () => {
  // Over-cap arrays are rejected by the bound, so these entries never have to
  // be valid; that is the point, an oversized document does no work.
  rejects({nodes: new Array(257).fill(null)}, /nodes must be an array of at most 256/);
  rejects({topics: new Array(4097).fill(null)},
          /topics must be an array of at most 4096/);
  rejects({nodes: [node({sources: new Array(513).fill(null)})]},
          /sources must be an array of at most 512/);
  rejects({nodes: {}}, /nodes must be an array/);
  rejects({topics: [topic({publishers: 'odometry'})]},
          /publishers must be an array/);
  rejects({topics: [topic({subscribers: [1]})]}, /subscribers\[0\] must be a string/);
});

test('anything that is not an object document fails closed', () => {
  for (const text of ['null', '[]', '5', '"text"', 'true'])
    assert.throws(() => parseSystemGraph(text), /must be an object/);
  // Malformed JSON still throws; it just throws from JSON.parse.
  assert.throws(() => parseSystemGraph(''));
  assert.throws(() => parseSystemGraph('{'));
  rejects({registry: null}, /registry must be an object/);
  rejects({bridge: []}, /bridge must be an object/);
});

test('an oversized document is rejected before it is parsed', () => {
  // Not valid JSON, so reaching JSON.parse would throw a SyntaxError instead:
  // the size gate has to come first for this to be the error.
  assert.throws(() => parseSystemGraph('x'.repeat(MAX_SYSTEM_BYTES + 1)),
                /document too large/);
});

// Mirrors SystemDatagrams() in studio/bridge/system.h: tag, little-endian
// document id, chunk index, chunk count, then the slice.
function datagram(text: string, documentId: number, index: number,
                  count: number): Uint8Array {
  const payload = new TextEncoder().encode(text);
  const out = new Uint8Array(SYSTEM_HEADER_BYTES + payload.length);
  for (let i = 0; i < SYSTEM_TAG.length; i++) out[i] = SYSTEM_TAG.charCodeAt(i);
  out[4] = documentId & 0xff;
  out[5] = (documentId >> 8) & 0xff;
  out[6] = index;
  out[7] = count;
  out.set(payload, SYSTEM_HEADER_BYTES);
  return out;
}

test('a tagged datagram yields its JSON and nothing else does', () => {
  const text = JSON.stringify(document());
  assert.deepEqual(systemChunk(datagram(text, 7, 0, 1)), {
    documentId: 7,
    index: 0,
    count: 1,
    payload: new TextEncoder().encode(text)
  });
  assert.equal(new SystemAssembler().push(datagram(text, 7, 0, 1)), text);
  assert.deepEqual(
      parseSystemGraph(new SystemAssembler().push(datagram(text, 7, 0, 1))!),
      parse());

  // A real telemetry frame must never be mistaken for a system graph: both
  // families share the UDP socket and only the tag separates them.
  assert.equal(systemChunk(demoPacket(0n)), null);
  assert.equal(systemChunk(new TextEncoder().encode(SYSTEM_TAG)), null);
  assert.equal(systemChunk(new TextEncoder().encode('TSY{')), null);
  assert.equal(systemChunk(new TextEncoder().encode('XSYS{}')), null);
  assert.equal(systemChunk(new Uint8Array(0)), null);

  // A header with no payload, and an index outside its own count, are both
  // malformed rather than empty.
  assert.equal(systemChunk(datagram('', 1, 0, 1)), null);
  assert.equal(systemChunk(datagram('x', 1, 3, 2)), null);
  assert.equal(systemChunk(datagram('x', 1, 0, 0)), null);

  const oversized = new Uint8Array(MAX_SYSTEM_BYTES + 1);
  for (let i = 0; i < SYSTEM_TAG.length; i++)
    oversized[i] = SYSTEM_TAG.charCodeAt(i);
  assert.equal(systemChunk(oversized), null);
});

test('a split document reassembles, in any order, and survives a lost chunk',
     () => {
       const text = JSON.stringify(document());
       const half = Math.ceil(text.length / 2);
       const parts = [text.slice(0, half), text.slice(half)];

       // In order.
       const forward = new SystemAssembler();
       assert.equal(forward.push(datagram(parts[0], 9, 0, 2)), null);
       assert.equal(forward.push(datagram(parts[1], 9, 1, 2)), text);

       // UDP does not promise order, so the last chunk to arrive completes it
       // whichever one that is.
       const reversed = new SystemAssembler();
       assert.equal(reversed.push(datagram(parts[1], 9, 1, 2)), null);
       assert.equal(reversed.push(datagram(parts[0], 9, 0, 2)), text);

       // A duplicate must not be counted twice and complete the document early.
       const duplicated = new SystemAssembler();
       assert.equal(duplicated.push(datagram(parts[0], 9, 0, 2)), null);
       assert.equal(duplicated.push(datagram(parts[0], 9, 0, 2)), null);
       assert.equal(duplicated.push(datagram(parts[1], 9, 1, 2)), text);

       // A document that lost a chunk is abandoned when the next one starts,
       // rather than waiting forever or splicing two generations together.
       const lossy = new SystemAssembler();
       assert.equal(lossy.push(datagram(parts[0], 9, 0, 2)), null);
       assert.equal(lossy.push(datagram(parts[0], 10, 0, 2)), null);
       assert.equal(lossy.push(datagram(parts[1], 10, 1, 2)), text);
     });

// topicHealthOf hands back a classifier, so a caller with a row in hand cannot
// be given undefined for a topic that is in the graph. These tests want to name
// topics, so they turn it back into a lookup.
const healthByName = (graph: SystemGraph) => {
  const health = topicHealthOf(graph);
  return new Map(graph.topics.map(topic => [topic.name, health(topic)]));
};

const healthRow = (over: Partial<SystemTopic> = {}): SystemTopic => ({
  name: '/odometry/state',
  message_bytes: 64,
  publishers: ['odometry'],
  subscribers: ['telemetry'],
  published: '10',
  received: '9',
  dropped: '0',
  ...over
});

test('topic health names the fault, worst first', () => {
  const row = healthRow;
  assert.equal(topicHealth(row()), 'ok');
  assert.equal(topicHealth(row({publishers: []})), 'orphaned');
  assert.equal(topicHealth(row({subscribers: []})), 'unread');
  assert.equal(topicHealth(row({dropped: '3'})), 'lossy');
  assert.equal(topicHealth(row({published: '0'})), 'idle');

  // A topic nobody publishes reads as orphaned whatever else is wrong with it:
  // a subscriber cannot tell that case apart from the inside, so it wins.
  assert.equal(
      topicHealth(row({publishers: [], subscribers: [], dropped: '3'})),
      'orphaned');
  assert.equal(topicHealth(row({subscribers: [], dropped: '3'})), 'unread');
});

test('a graph too old to carry endpoint attributes is read, and marked as such',
     () => {
       // The version is how a reader knows whether "not external" was stated or
       // merely unavailable. Both documents parse -- refusing the older one
       // would blank a working view -- so the difference has to be legible.
       assert.equal(endpointAttributesKnown(parse()), false);
       assert.equal(
           endpointAttributesKnown(parse({version: SYSTEM_GRAPH_VERSION})), true);
       assert.equal(endpointAttributesKnown(parse({version: 99})), true);
       // The demo states them on every end, so it must claim the version that
       // says so.
       assert.equal(
           endpointAttributesKnown(parseSystemGraph(demoSystemGraph())), true);
     });

test('a declared endpoint attribute is not a fault, and says which it is', () => {
  const row = healthRow;
  const external = {external: true, optional: false};
  const optional = {external: false, optional: true};

  // /hw/command: the consumer is the RoboRIO across UDP, so a shared-memory
  // subscriber will never exist. Reporting that as unread is what makes an
  // attention list worth ignoring.
  assert.equal(topicHealth(row({subscribers: []}), external), 'bridged');
  // The same in the other direction: a writer outside talOS.
  assert.equal(topicHealth(row({publishers: []}), external), 'bridged');
  // /drivetrain/target/auto: a reader and no writer, because autonomous is not
  // written yet, and the node holding the end says so.
  assert.equal(topicHealth(row({publishers: []}), optional), 'unconnected');
  assert.equal(topicHealth(row({subscribers: []}), optional), 'unconnected');

  // An attribute only explains a missing end. A topic with both ends is judged
  // on what it is doing, exactly as before.
  assert.equal(topicHealth(row(), external), 'ok');
  assert.equal(topicHealth(row({dropped: '3'}), external), 'lossy');
  assert.equal(topicHealth(row({published: '0'}), optional), 'idle');

  // Absent attributes keep the old answers, which is what an older bridge
  // sends and what a topic with nothing special about it means.
  assert.equal(topicHealth(row({publishers: []}), undefined), 'orphaned');
  assert.equal(
      topicHealth(row({publishers: []}), {external: false, optional: false}),
      'orphaned');
});

test('endpoint attributes are read off the ends, not the topic row', () => {
  const graph = parse({
    nodes: [
      node({
        name: 'hardware_node',
        sources: [
          source({id: 0, kind: 'SENDER', name: '/hw/command', external: true}),
          // A timer's name is not a topic, so it contributes no attribute even
          // when it collides with one.
          source({id: 1, kind: 'TIMER', name: '/hw/command', optional: true}),
          source({id: 2, kind: 'FETCHER', name: '/hw/state'})
        ]
      })
    ],
    topics: [
      topic({name: '/hw/command', subscribers: []}),
      topic({name: '/hw/state', publishers: []})
    ]
  });

  assert.equal(graph.nodes[0].sources[0].external, true);
  assert.equal(graph.nodes[0].sources[0].optional, false);
  const ends = topicEnds(graph.nodes);
  assert.deepEqual(ends.get('/hw/command'), {external: true, optional: false});
  assert.equal(ends.get('/hw/state')?.external, false);

  const health = healthByName(graph);
  assert.equal(health.get('/hw/command'), 'bridged');
  assert.equal(health.get('/hw/state'), 'orphaned');
});

test('an endpoint attribute may be absent but must not be the wrong type', () => {
  // A bridge built before endpoint attributes existed sends neither field.
  // Refusing its whole document -- and blanking a working system view -- to
  // report that a topic is ordinary would be the wrong trade.
  const older = parse({nodes: [node({sources: [source()]})]});
  assert.equal(older.nodes[0].sources[0].external, false);
  assert.equal(older.nodes[0].sources[0].optional, false);

  rejects({nodes: [node({sources: [source({external: 'true'})]})]},
          /external must be a boolean/);
  rejects({nodes: [node({sources: [source({optional: 1})]})]},
          /optional must be a boolean/);
  rejects({nodes: [node({sources: [source({external: null})]})]},
          /external must be a boolean/);
});

test('watchers and fetchers are the subscribing ends', () => {
  assert.equal(isSubscriber('WATCHER'), true);
  assert.equal(isSubscriber('FETCHER'), true);
  assert.equal(isSubscriber('SENDER'), false);
  assert.equal(isSubscriber('TIMER'), false);
});

test('the demo graph is a document the parser actually accepts', () => {
  // The demo is what the tabs render with no robot attached, so it has to
  // satisfy the same parser as the bridge's output -- otherwise it drifts into
  // a shape that only the demo can produce.
  const graph = parseSystemGraph(demoSystemGraph());
  assert.ok(graph.nodes.length > 1);
  assert.ok(graph.topics.length > 1);
  assert.equal(graph.registry.available, true);

  // Every topic in the derived list is claimed by at least one end, and every
  // named end is a node in the same document.
  const known = new Set(graph.nodes.map(n => n.name));
  for (const t of graph.topics) {
    assert.ok(t.publishers.length + t.subscribers.length > 0, t.name);
    for (const end of [...t.publishers, ...t.subscribers])
      assert.ok(known.has(end), `${t.name} names unknown node ${end}`);
  }

  // Counters advance with the demo frame counter, so a later sample yields a
  // measurable rate rather than a flat graph.
  const later = parseSystemGraph(demoSystemGraph(250n));
  assert.ok(BigInt(later.wall_ns) > BigInt(graph.wall_ns));
  assert.ok(eventRates(later, graph).size > 0);
});

test('event rates need two samples and ignore counter resets', () => {
  // `wall_ns` is deliberately the same in every graph here: a rate comes from
  // the node's own sample time, so if it ever came from the graph clock again
  // every case below would divide by zero and report nothing.
  const at = (sampled: string, events: string, id = 0): SystemGraph => parse({
    wall_ns: '5000000000',
    nodes:
        [node({heartbeat_wall_ns: sampled, sources: [source({id, events})]})]
  });
  const key = rateKey('odometry', 0);

  // No previous sample is "not measured yet", which must not read as zero.
  assert.equal(eventRates(at('2000000000', '150'), null).size, 0);

  const rates = eventRates(at('2000000000', '150'), at('1000000000', '100'));
  assert.equal(rates.get(key), 50);

  // A restarted node zeroes its counters. That is a restart, not a negative
  // rate, so the source is omitted.
  assert.equal(
      eventRates(at('2000000000', '10'), at('1000000000', '100')).size, 0);

  // A source the previous graph never saw has no baseline to subtract.
  assert.equal(
      eventRates(at('2000000000', '20', 5), at('1000000000', '0', 0)).size, 0);

  // Two graphs from the same instant give no elapsed time to divide by.
  assert.equal(
      eventRates(at('1000000000', '150'), at('1000000000', '100')).size, 0);

  // The delta is computed in BigInt, so counters past 2^53 still subtract
  // exactly instead of rounding to a rate of zero.
  const big = eventRates(
      at('2000000000', '9007199254740993'), at('1000000000', '9007199254740992'));
  assert.equal(big.get(key), 1);
});

test('a rate is measured on the node clock, not the graph clock', () => {
  const key = rateKey('odometry', 0);
  const ms = (n: number) => BigInt(n) * 1_000_000n;
  const obs = (graphWall: bigint, sampled: bigint,
               events: number): SystemGraph => parse({
    wall_ns: String(graphWall),
    nodes: [node({
      heartbeat_wall_ns: String(sampled),
      sources: [source({id: 0, events: String(events)})]
    })]
  });

  // The bridge rebuilds the graph every 250 ms; each node refreshes its own
  // slot every 250 ms on an unrelated timer. Here the bridge's next read lands
  // before the node re-sampled, so it sees the previous sample twice -- same
  // heartbeat, same counter -- and the node's following sample then covers
  // 500 ms of events at once. The source held a steady 200 Hz throughout.
  const a = obs(ms(1750), ms(1700), 1150);
  const stalled = obs(ms(2000), ms(1700), 1150);
  const b = obs(ms(2250), ms(2200), 1250);

  // Divided by the graph clock, that reads 0 across the stall and double
  // after it. Neither number happened.
  const onGraphClock = (from: SystemGraph, to: SystemGraph) =>
      Number(BigInt(to.nodes[0].sources[0].events) -
             BigInt(from.nodes[0].sources[0].events)) /
      (Number(BigInt(to.wall_ns) - BigInt(from.wall_ns)) / 1e9);
  assert.equal(onGraphClock(a, stalled), 0);
  assert.equal(onGraphClock(stalled, b), 400);

  // Divided by the node's own sample time, the stalled read has no interval to
  // measure and is omitted, and the pair spanning the catch-up is exact.
  assert.equal(eventRates(stalled, a).size, 0);
  assert.equal(eventRates(b, stalled).get(key), 200);
});

test('a rate window resolves finer than one integer count per refresh', () => {
  const key = rateKey('odometry', 0);
  const graph = (sampled: bigint, events: number): SystemGraph => parse({
    wall_ns: String(sampled),
    nodes: [node({
      heartbeat_wall_ns: String(sampled),
      sources: [source({id: 0, events: String(events)})]
    })]
  });

  // A ~201.7 Hz source sampled every 250 ms. `events` is an integer, so each
  // refresh catches 50 or 51 -- and against a single refresh that is the only
  // pair of answers available: 200 or 204, never anything between.
  const perRefresh = [50, 51, 50, 51, 50, 51, 50, 51, 50, 50, 51, 50];
  const step = 250_000_000n;

  const samples: SystemGraph[] = [];
  let events = 1000, sampled = 1_000_000_000n;
  samples.push(graph(sampled, events));
  for (const advance of perRefresh) {
    sampled += step;
    events += advance;
    samples.push(graph(sampled, events));
  }

  const pairwise = samples.slice(1).map(
      (current, i) => eventRates(current, samples[i]).get(key));
  assert.deepEqual([...new Set(pairwise)].sort((x, y) => x! - y!), [200, 204]);

  // Through a window the same samples resolve the rate between those steps.
  const window = new RateWindow(2);
  const observed: {rate: number; filled: boolean}[] = [];
  samples.forEach((sample, i) => {
    window.push(sample);
    const rate = window.rates().get(key);
    if (rate !== undefined) observed.push({rate, filled: i >= 8});
  });
  assert.ok(observed.some(o => o.filled), 'the window never filled');
  for (const {rate, filled} of observed)
    if (filled) assert.ok(rate > 200 && rate < 204, `window reported ${rate}`);
  assert.ok(observed.some(o => o.filled && o.rate !== 200 && o.rate !== 204),
            'the window resolved nothing a single refresh could not');

  // Once filled it holds a window's worth of samples, not the whole session.
  assert.ok(window.size >= 2 && window.size <= 12, `held ${window.size}`);

  window.reset();
  assert.equal(window.size, 0);
  assert.equal(window.rates().size, 0);
});

test('a rate window needs two samples and rejects a nonsense span', () => {
  assert.throws(() => new RateWindow(0), /Invalid rate window/);
  assert.throws(() => new RateWindow(2, 1), /capacity/);

  const window = new RateWindow(2);
  // One sample is "not measured yet", which must not read as zero.
  assert.equal(window.rates().size, 0);
  window.push(parse({
    wall_ns: '1000000000',
    nodes: [node({sources: [source({id: 0, events: '10'})]})]
  }));
  assert.equal(window.rates().size, 0);
});

// --- the declared graph -----------------------------------------------------

const declaredSource = (over: Json = {}): Json =>
    ({kind: 'SENDER', name: '/odometry/state', message_bytes: 64, ...over});
const declaredNode = (over: Json = {}): Json => ({
  name: 'odometry',
  target: '//2026-robot/main_processor/odometry:node',
  sources: [declaredSource()],
  ...over
});
const declaredDocument = (over: Json = {}): Json => ({
  kind: 'talos.declared_graph',
  version: 1,
  nodes: [declaredNode()],
  diagnostics: [{
    severity: 'warning',
    subject: '/talos/telemetry',
    message: 'is published but nothing subscribes to it'
  }],
  ...over
});
const parseDeclared = (over: Json = {}): DeclaredGraph =>
    parseDeclaredGraph(JSON.stringify(declaredDocument(over)));
const refuses = (over: Json, because: RegExp) =>
    assert.throws(() => parseDeclared(over), because);

test('a declared graph parses into nodes, ends and the launcher\'s lint', () => {
  const declared = parseDeclared({
    nodes: [declaredNode({
      name: 'hardware_node',
      target: '//talOS/bridge:hardware_node',
      sources: [
        declaredSource({name: '/hw/command', external: true}),
        declaredSource(
            {kind: 'FETCHER', name: '/hw/command/override', optional: true}),
        declaredSource({kind: 'TIMER', name: 'tick', message_bytes: 0})
      ]
    })]
  });
  assert.equal(declared.version, 1);
  assert.equal(declared.nodes.length, 1);
  assert.equal(declared.nodes[0].target, '//talOS/bridge:hardware_node');
  assert.deepEqual(declared.nodes[0].sources[0], {
    kind: 'SENDER',
    name: '/hw/command',
    message_bytes: 64,
    external: true,
    optional: false
  });
  assert.equal(declared.nodes[0].sources[1].optional, true);
  assert.deepEqual(declared.diagnostics, [{
    severity: 'warning',
    subject: '/talos/telemetry',
    message: 'is published but nothing subscribes to it'
  }]);

  // The declared ends classify topics the same way the live ones do, which is
  // what lets a launcher's warning and a viewer's badge agree.
  assert.deepEqual(topicEnds(declared.nodes).get('/hw/command'),
                   {external: true, optional: false});
});

test('a declared graph is refused as strictly as a live one, and says which', () => {
  refuses({nodes: [declaredNode({name: 5})]},
          /Invalid declared graph: name must be a string/);
  refuses({nodes: [declaredNode({target: null})]}, /target must be a string/);
  refuses({nodes: [declaredNode({sources: [declaredSource({kind: 'PUBLISHER'})]})]},
          /kind must be one of/);
  refuses({nodes: [declaredNode({sources: [declaredSource({message_bytes: '64'})]})]},
          /message_bytes must be a non-negative integer/);
  refuses({nodes: [declaredNode({sources: [declaredSource({external: 'yes'})]})]},
          /external must be a boolean/);
  refuses({nodes: [declaredNode({sources: {}})]}, /sources must be an array/);
  refuses({nodes: new Array(257).fill(null)},
          /nodes must be an array of at most 256/);
  refuses({nodes: undefined}, /nodes must be an array/);
  refuses({diagnostics: [{severity: 'fatal', subject: '/x', message: 'no'}]},
          /severity must be error or warning/);
  refuses({diagnostics: [{severity: 'error', subject: '/x'}]},
          /message must be a string/);
  refuses({diagnostics: new Array(1025).fill(null)},
          /diagnostics must be an array of at most 1024/);

  // A live system graph handed to this reader is a mistake, not a graph.
  assert.throws(() => parseDeclaredGraph(demoSystemGraph()),
                /Invalid declared graph: unexpected document kind/);
  for (const text of ['null', '[]', '5', '"text"', 'true'])
    assert.throws(() => parseDeclaredGraph(text),
                  /Invalid declared graph: document must be an object/);
  // Malformed JSON still fails, and now names the document it was reading.
  assert.throws(() => parseDeclaredGraph('{'), /Invalid declared graph: /);
  assert.throws(() => parseDeclaredGraph('x'.repeat(MAX_DECLARED_BYTES + 1)),
                /Invalid declared graph: document too large/);
});

test('a declared graph the launcher wrote plainly is still read', () => {
  // The launcher writes this file for `jq` and for people as much as for
  // Studio. A document with no envelope and nothing to report is a complete
  // answer, and an absent diagnostics list says the same as an empty one.
  const bare = parseDeclaredGraph(JSON.stringify({nodes: [declaredNode()]}));
  assert.equal(bare.nodes.length, 1);
  assert.deepEqual(bare.diagnostics, []);
  assert.equal(bare.version, DECLARED_VERSION);

  // ERROR is how the enumerator is spelled in C++; error is how it prints.
  const shouted = parseDeclared(
      {diagnostics: [{severity: 'ERROR', subject: '/x', message: 'two writers'}]});
  assert.equal(shouted.diagnostics[0].severity, 'error');
});

test('the declared graph is asked for at the origin that served the stream', () => {
  // The bridge serves both, so being told how to reach the stream is being told
  // how to reach this. Getting it wrong would leave every declared view empty
  // with nothing on screen to say why.
  assert.equal(declaredUrl('ws://10.0.42.2:5800/telemetry'),
               'http://10.0.42.2:5800/declared.json');
  assert.equal(declaredUrl('wss://robot.local/telemetry?tap=1'),
               'https://robot.local/declared.json');
  assert.equal(declaredUrl('ws://127.0.0.1:5173/telemetry#live'),
               'http://127.0.0.1:5173/declared.json');

  // Nowhere to ask is not an error: the desktop UDP transport has no origin,
  // and Studio has to keep working without a declared graph anyway.
  assert.equal(declaredUrl('udp://127.0.0.1:5801'), null);
  assert.equal(declaredUrl('http://127.0.0.1:5800/telemetry'), null);
  assert.equal(declaredUrl('not a url'), null);
  assert.equal(declaredUrl(''), null);
});

test('declared against actual separates a node that died from one nobody asked for',
     () => {
       const graph = parseSystemGraph(demoSystemGraph(1500n));
       const declared = parseDeclaredGraph(demoDeclaredGraph());
       const status = declaredStatus(declared, graph)!;

       assert.ok(status.declared > status.registered,
                 'demo declares no more nodes than it runs');
       assert.equal(status.registered, graph.nodes.length);
       assert.deepEqual(status.missing.map(n => n.name), ['shooter']);
       assert.ok(status.missing[0].target.includes('shooter'));
       assert.deepEqual(status.undeclared, []);
       assert.equal(status.warnings, 1);
       assert.equal(status.errors, 0);

       // A node running that nobody declared is the other asymmetry, and it is
       // a different sentence: started outside the launcher, not lost to it.
       const trimmed: DeclaredGraph = {
         ...declared,
         nodes: declared.nodes.filter(n => n.name !== 'telemetry')
       };
       const other = declaredStatus(trimmed, graph)!;
       assert.deepEqual(other.undeclared.map(n => n.name), ['telemetry']);

       // With no registry there is nothing to compare against, and answering
       // zero would be inventing a claim nobody can check.
       assert.equal(declaredStatus(declared, null), null);
     });

test('the demo reaches every health state a viewer can render', () => {
  // The demo is the only system most people will explore, so every badge the
  // topic table can show has to be reachable in it -- including the two that
  // say a half-wired topic is not a fault.
  const running = healthByName(parseSystemGraph(demoSystemGraph(1500n)));
  const starting = healthByName(parseSystemGraph(demoSystemGraph(0n)));
  const seen = new Set([...running.values(), ...starting.values()]);
  for (const state of TOPIC_HEALTH)
    assert.ok(seen.has(state), `demo never shows a ${state} topic`);

  // And each of them is the topic the demo's comment claims it is.
  assert.equal(running.get('/shooter/state'), 'orphaned');
  assert.equal(running.get('/hw/request/shooter'), 'orphaned');
  assert.equal(running.get('/shooter/target'), 'unread');
  assert.equal(running.get('/talos/telemetry'), 'unread');
  assert.equal(running.get('/hw/command'), 'bridged');
  assert.equal(running.get('/hw/command/override'), 'unconnected');
  assert.equal(running.get('/drivetrain/target/auto'), 'unconnected');
  assert.equal(running.get('/shooter/target/auto'), 'unconnected');
  assert.equal(running.get('/hw/state'), 'lossy');
  assert.equal(running.get('/hw/request/drivetrain'), 'ok');
  assert.equal(starting.get('/hw/state'), 'idle');

  // The renamed topics, and none of the old spellings, anywhere in either
  // document: a rename that missed a fixture is a graph that quietly stops
  // meeting itself.
  const documents = demoSystemGraph(1500n) + demoDeclaredGraph();
  for (const stale of ['/talos_studio', '/hw/cmd', '/hw/ds', '/hw/req/',
                       '/drivetrain/tgt', '/shooter/tgt', '"/odometry"'])
    assert.ok(!documents.includes(stale), `demo still spells ${stale}`);
});

test('the demo declared graph is a document the parser accepts', () => {
  const declared = parseDeclaredGraph(demoDeclaredGraph());
  const graph = parseSystemGraph(demoSystemGraph(1500n));

  // Every node the live graph reports is declared, with the same target and
  // the same ends: the demo derives both from one table, so a disagreement
  // here means the derivation drifted.
  for (const node of graph.nodes) {
    const twin = declared.nodes.find(n => n.name === node.name);
    assert.ok(twin, `${node.name} is running and not declared`);
    assert.equal(twin!.target, node.target);
    assert.deepEqual(twin!.sources.map(s => s.name),
                     node.sources.map(s => s.name));
    assert.deepEqual(twin!.sources.map(s => s.external),
                     node.sources.map(s => s.external));
  }

  // The launcher's own words reach the reader verbatim.
  assert.equal(declared.diagnostics[0].subject, '/talos/telemetry');
  assert.match(declared.diagnostics[0].message, /nothing subscribes/);
  // A graph with an error is one the launcher refuses to launch, so a demo
  // session that is running must not carry one.
  assert.ok(!declared.diagnostics.some(d => d.severity === 'error'));
});
