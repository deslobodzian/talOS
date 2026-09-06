// The system graph: every node, every topic, and who is on which end.
//
// The bridge sends this as JSON a few times a second, on the same transport as
// telemetry frames -- a WebSocket text frame, or a UDP datagram behind a tag.
// It arrives from the same untrusted place as everything else, so nothing here
// trusts its shape: every field is checked and every collection is bounded.
// JSON.parse cannot be fooled by an offset the way the generated FlatBuffers
// accessors can, but it will happily hand back a string where a number was
// promised, and the UI must not have to guess.
//
// uint64 values stay decimal strings, the rule the telemetry timestamps
// already follow: a JSON number is a double, and a sequence counter or a
// nanosecond timestamp loses precision past 2^53.

export const SYSTEM_TAG = 'TSYS';
export const MAX_SYSTEM_BYTES = 1 << 20;

// studio/bridge/system.h's kSystemGraphVersion. Version 2 is where a source
// began stating what its far end is: without it a document cannot say that
// /hw/command's consumer is the RoboRIO, and a viewer reading one will call
// that topic unread.
//
// An older document is read rather than refused. Reporting two designed
// dead-ends as faults is what the older bridge always did, and it is a better
// answer than an empty viewer -- but it is not the truth, so the views say
// which of the two they are looking at instead of leaving the difference
// invisible.
export const SYSTEM_GRAPH_VERSION = 2;

export const endpointAttributesKnown = (graph: SystemGraph) =>
    graph.version >= SYSTEM_GRAPH_VERSION;

// tag(4) + document id(2, little-endian) + chunk index(1) + chunk count(1).
export const SYSTEM_HEADER_BYTES = 8;

export type SourceKind = 'TIMER'|'WATCHER'|'FETCHER'|'SENDER';
const KINDS: SourceKind[] = ['TIMER', 'WATCHER', 'FETCHER', 'SENDER'];

export type SystemSource = {
  id: number; kind: SourceKind; name: string; message_bytes: number;
  alignment: number;
  period_ns: string;
  events: string;
  dropped: string;
  sequence: string;
  last_monotonic_ns: string;
  last_latency_ns: string;
  max_latency_ns: string;

  // How this end expects its far end to behave, as the owning node declared it
  // (talOS/introspection/names.h). `external` means the peer is outside talOS:
  // /hw/command is consumed by the RoboRIO over UDP, so a shared-memory
  // subscriber will never exist. `optional` means this end may legitimately
  // have no peer yet -- /drivetrain/target/auto has no publisher because
  // autonomous is not written. Neither is a fault, and reporting them as faults
  // is what teaches people to ignore a fault report.
  external: boolean;
  optional: boolean;
};

export type SystemNode = {
  name: string; target: string; slot: number; pid: string; session_id: string;
  generation: string;
  start_wall_ns: string;
  heartbeat_wall_ns: string;
  dispatch_count: string;
  declared_sources: number;
  alive: boolean;
  simulation: boolean;
  replay: boolean;
  sources: SystemSource[];
};

export type SystemTopic = {
  name: string; message_bytes: number; publishers: string[];
  subscribers: string[];
  published: string;
  received: string;
  dropped: string;
};

export type SystemGraph = {
  version: number; wall_ns: string; registry: {
    available: boolean; node_capacity: number; source_capacity: number;
    liveness_timeout_ns: string;
  };
  bridge: {
    topic: string; clients: number; frames_forwarded: string; invalid: string;
    udp_drops: string;
    ws_drops: string;
  };
  nodes: SystemNode[];
  topics: SystemTopic[];
};

// Caps chosen well above anything the registry can hold (32 nodes, 64 sources
// each) so a legitimate document never trips them, and far below anything that
// could stall the renderer.
const MAX_NODES = 256;
const MAX_SOURCES = 512;
const MAX_TOPICS = 4096;
const MAX_STRING = 512;

// A lint finding is a sentence, not a name, so it gets its own bound.
const MAX_DIAGNOSTICS = 1024;
const MAX_MESSAGE = 2048;

const fail = (what: string): never => {
  throw Error(what);
};

// Every check below raises the bare complaint. The entry points name the
// document it was about, so one set of helpers serves both the live graph and
// the declared one and neither error loses its subject -- including the
// SyntaxError from JSON.parse, which on its own does not say what failed to
// parse.
function parsing<T>(document: string, body: () => T): T {
  try {
    return body();
  } catch (e) {
    throw Error(
        `Invalid ${document}: ${e instanceof Error ? e.message : String(e)}`);
  }
}

function record(value: unknown, what: string): Record<string, unknown> {
  if (!value || typeof value !== 'object' || Array.isArray(value))
    fail(`${what} must be an object`);
  return value as Record<string, unknown>;
}

function str(source: Record<string, unknown>, key: string,
             cap = MAX_STRING): string {
  const value = source[key];
  if (typeof value !== 'string' || value.length > cap)
    fail(`${key} must be a string of at most ${cap} characters`);
  return value as string;
}

// A uint64 as the wire carries it. Kept as a string rather than converted: the
// UI formats it, and BigInt() on demand costs less than converting every field
// of every node four times a second.
function u64(source: Record<string, unknown>, key: string): string {
  const value = source[key];
  if (typeof value !== 'string' || !/^-?\d{1,20}$/.test(value))
    fail(`${key} must be a decimal integer string`);
  return value as string;
}

function num(source: Record<string, unknown>, key: string): number {
  const value = source[key];
  if (typeof value !== 'number' || !Number.isInteger(value) || value < 0 ||
      value > 4294967296)
    fail(`${key} must be a non-negative integer`);
  return value as number;
}

function bool(source: Record<string, unknown>, key: string): boolean {
  const value = source[key];
  if (typeof value !== 'boolean') fail(`${key} must be a boolean`);
  return value as boolean;
}

// An endpoint attribute. Absent is read as false rather than refused: a bridge
// built before endpoint attributes existed sends neither field, and rejecting
// its whole document over a flag whose default is the ordinary case would blank
// a working view to report that a topic is ordinary. A field that is present
// still has to be a boolean -- "true" is a mistake, not a value.
function flag(source: Record<string, unknown>, key: string): boolean {
  const value = source[key];
  if (value === undefined) return false;
  if (typeof value !== 'boolean') fail(`${key} must be a boolean`);
  return value as boolean;
}

function list(source: Record<string, unknown>, key: string,
              cap: number): unknown[] {
  const value = source[key];
  if (!Array.isArray(value) || value.length > cap)
    fail(`${key} must be an array of at most ${cap} entries`);
  return value as unknown[];
}

function names(source: Record<string, unknown>, key: string): string[] {
  return list(source, key, MAX_NODES).map((entry, i) => {
    if (typeof entry !== 'string' || entry.length > MAX_STRING)
      fail(`${key}[${i}] must be a string`);
    return entry as string;
  });
}

function parseSource(value: unknown, where: string): SystemSource {
  const source = record(value, where);
  const kind = str(source, 'kind');
  if (!KINDS.includes(kind as SourceKind))
    fail(`${where}.kind must be one of ${KINDS.join(', ')}`);
  return {
    id: num(source, 'id'),
    kind: kind as SourceKind,
    name: str(source, 'name'),
    message_bytes: num(source, 'message_bytes'),
    alignment: num(source, 'alignment'),
    period_ns: u64(source, 'period_ns'),
    events: u64(source, 'events'),
    dropped: u64(source, 'dropped'),
    sequence: u64(source, 'sequence'),
    last_monotonic_ns: u64(source, 'last_monotonic_ns'),
    last_latency_ns: u64(source, 'last_latency_ns'),
    max_latency_ns: u64(source, 'max_latency_ns'),
    external: flag(source, 'external'),
    optional: flag(source, 'optional')
  };
}

function parseNode(value: unknown, where: string): SystemNode {
  const node = record(value, where);
  return {
    name: str(node, 'name'),
    target: str(node, 'target'),
    slot: num(node, 'slot'),
    pid: u64(node, 'pid'),
    session_id: u64(node, 'session_id'),
    generation: u64(node, 'generation'),
    start_wall_ns: u64(node, 'start_wall_ns'),
    heartbeat_wall_ns: u64(node, 'heartbeat_wall_ns'),
    dispatch_count: u64(node, 'dispatch_count'),
    declared_sources: num(node, 'declared_sources'),
    alive: bool(node, 'alive'),
    simulation: bool(node, 'simulation'),
    replay: bool(node, 'replay'),
    sources:
        list(node, 'sources', MAX_SOURCES)
            .map((entry, i) => parseSource(entry, `${where}.sources[${i}]`))
  };
}

function parseTopic(value: unknown, where: string): SystemTopic {
  const topic = record(value, where);
  return {
    name: str(topic, 'name'),
    message_bytes: num(topic, 'message_bytes'),
    publishers: names(topic, 'publishers'),
    subscribers: names(topic, 'subscribers'),
    published: u64(topic, 'published'),
    received: u64(topic, 'received'),
    dropped: u64(topic, 'dropped')
  };
}

export function parseSystemGraph(text: string): SystemGraph {
  return parsing('system graph', () => {
    if (text.length > MAX_SYSTEM_BYTES) fail('document too large');
    const root = record(JSON.parse(text), 'document');
    if (root.kind !== 'talos.system_graph') fail('unexpected document kind');

    const registry = record(root.registry, 'registry');
    const bridge = record(root.bridge, 'bridge');

    return {
      version: num(root, 'version'),
      wall_ns: u64(root, 'wall_ns'),
      registry: {
        available: bool(registry, 'available'),
        node_capacity: num(registry, 'node_capacity'),
        source_capacity: num(registry, 'source_capacity'),
        liveness_timeout_ns: u64(registry, 'liveness_timeout_ns')
      },
      bridge: {
        topic: str(bridge, 'topic'),
        clients: num(bridge, 'clients'),
        frames_forwarded: u64(bridge, 'frames_forwarded'),
        invalid: u64(bridge, 'invalid'),
        udp_drops: u64(bridge, 'udp_drops'),
        ws_drops: u64(bridge, 'ws_drops')
      },
      nodes: list(root, 'nodes', MAX_NODES)
                 .map((entry, i) => parseNode(entry, `nodes[${i}]`)),
      topics: list(root, 'topics', MAX_TOPICS)
                  .map((entry, i) => parseTopic(entry, `topics[${i}]`))
    };
  });
}

// --- the declared graph ----------------------------------------------------
//
// What the launcher probed before it spawned anything. It runs every binary in
// the config with `--describe` -- which builds the node on a simulated loop,
// prints the ends its constructor registered, and exits -- assembles those into
// one graph, lints it, and writes it out. The bridge serves that file verbatim
// at `GET /declared.json`.
//
// The value is the comparison. The live registry only ever holds what did
// start, so a name's absence from it says nothing on its own: a node that
// crashed before it claimed a slot and a node that is not part of this session
// look identical from there. This is the other half of that question.
//
// Checked as strictly as the live graph, by the same helpers. It carries no
// counters -- it is a shape, not a measurement -- so nothing in it is a uint64;
// if a session id or a timestamp is ever added here it stays a decimal string,
// the rule every other number on this wire follows.

export const MAX_DECLARED_BYTES = 1 << 20;

const DECLARED_KIND = 'talos.declared_graph';

// introspect::kDescribeVersion, which is what the launcher assembles this from.
export const DECLARED_VERSION = 1;

export type DeclaredSource = {
  kind: SourceKind; name: string; message_bytes: number; external: boolean;
  optional: boolean;
};

export type DeclaredNode = {
  name: string; target: string; sources: DeclaredSource[];
};

export type DiagnosticSeverity = 'error'|'warning';

// One of the launcher's lint findings, in the launcher's own words. Kept
// verbatim rather than re-derived: the launcher is what refuses to start a
// robot whose topics do not meet, and its sentence explains a broken graph
// better than a viewer's second guess at the same conclusion.
export type DeclaredDiagnostic = {
  severity: DiagnosticSeverity; subject: string; message: string;
};

export type DeclaredGraph = {
  version: number; nodes: DeclaredNode[]; diagnostics: DeclaredDiagnostic[];
};

function parseDeclaredSource(value: unknown, where: string): DeclaredSource {
  const source = record(value, where);
  const kind = str(source, 'kind');
  if (!KINDS.includes(kind as SourceKind))
    fail(`${where}.kind must be one of ${KINDS.join(', ')}`);
  return {
    kind: kind as SourceKind,
    name: str(source, 'name'),
    message_bytes: num(source, 'message_bytes'),
    external: flag(source, 'external'),
    optional: flag(source, 'optional')
  };
}

function parseDeclaredNode(value: unknown, where: string): DeclaredNode {
  const node = record(value, where);
  return {
    name: str(node, 'name'),
    target: str(node, 'target'),
    sources: list(node, 'sources', MAX_SOURCES)
                 .map((entry, i) =>
                          parseDeclaredSource(entry, `${where}.sources[${i}]`))
  };
}

const SEVERITIES: DiagnosticSeverity[] = ['error', 'warning'];

function parseDiagnostic(value: unknown, where: string): DeclaredDiagnostic {
  const diagnostic = record(value, where);
  // naming::Diagnostic prints its severity in lower case and names the
  // enumerator in upper. Either spelling is accepted rather than making a
  // viewer depend on which one the writer reached for.
  const severity = str(diagnostic, 'severity').toLowerCase();
  if (!SEVERITIES.includes(severity as DiagnosticSeverity))
    fail(`${where}.severity must be error or warning`);
  return {
    severity: severity as DiagnosticSeverity,
    subject: str(diagnostic, 'subject'),
    message: str(diagnostic, 'message', MAX_MESSAGE)
  };
}

// Where to ask for the declared graph, given the telemetry socket's URL.
//
// The bridge serves both from one origin, so anything that was told how to
// reach the stream has already been told how to reach this. Null when the URL
// is not one this can reason about -- the desktop UDP path has no origin at all
// -- which is not an error but an absence of anywhere to ask.
export function declaredUrl(socketUrl: string): string|null {
  try {
    const target = new URL(socketUrl);
    if (target.protocol !== 'ws:' && target.protocol !== 'wss:') return null;
    // ws and http are both special schemes, so the URL parser allows this
    // swap; it keeps the host and port, which is the whole point.
    target.protocol = target.protocol === 'wss:' ? 'https:' : 'http:';
    target.pathname = '/declared.json';
    target.search = '';
    target.hash = '';
    return target.toString();
  } catch {
    return null;
  }
}

export function parseDeclaredGraph(text: string): DeclaredGraph {
  return parsing('declared graph', () => {
    if (text.length > MAX_DECLARED_BYTES) fail('document too large');
    const root = record(JSON.parse(text), 'document');
    // A document that announces itself as something else is refused, so a live
    // system graph handed to this reader cannot be shown as a declared one. A
    // document that announces nothing is judged on its contents: the launcher
    // writes this file for `jq` and for people as much as for Studio, and
    // Studio should not be the reason it has to carry a kind field.
    if (root.kind !== undefined && root.kind !== DECLARED_KIND)
      fail('unexpected document kind');

    return {
      version: root.version === undefined ? DECLARED_VERSION :
                                            num(root, 'version'),
      nodes: list(root, 'nodes', MAX_NODES)
                 .map((entry, i) => parseDeclaredNode(entry, `nodes[${i}]`)),
      // A launcher with nothing to report may leave the list out; absent and
      // empty are the same statement here, and neither is a reason to refuse
      // the graph that came with it.
      diagnostics: root.diagnostics === undefined ?
          [] :
          list(root, 'diagnostics', MAX_DIAGNOSTICS)
              .map((entry, i) => parseDiagnostic(entry, `diagnostics[${i}]`))
    };
  });
}

// --- datagram framing ------------------------------------------------------
//
// The UDP path carries both message families with no framing to tell them
// apart, so system graphs arrive behind a tag. Telemetry frames begin with a
// FlatBuffers size prefix and are left alone.
//
// They also arrive in pieces. macOS caps a datagram at 9216 bytes and a real
// robot's graph is several times that, so the bridge splits the document and
// this puts it back together. Getting that wrong is invisible rather than
// noisy: the System tab would just stay empty.

export type SystemChunk = {
  documentId: number; index: number; count: number; payload: Uint8Array;
};

export function systemChunk(bytes: Uint8Array): SystemChunk|null {
  if (bytes.length <= SYSTEM_HEADER_BYTES || bytes.length > MAX_SYSTEM_BYTES)
    return null;
  for (let i = 0; i < SYSTEM_TAG.length; i++)
    if (bytes[i] !== SYSTEM_TAG.charCodeAt(i)) return null;

  const count = bytes[7];
  const index = bytes[6];
  if (count === 0 || index >= count) return null;
  return {
    documentId: bytes[4] | (bytes[5] << 8),
    index,
    count,
    payload: bytes.subarray(SYSTEM_HEADER_BYTES)
  };
}

// Collects the chunks of one document. Only the newest document is held: a
// refresh arrives four times a second, so a graph that lost a chunk is not
// worth waiting for when a complete one is 250 ms behind it.
export class SystemAssembler {
  private documentId = -1;
  private chunks: (Uint8Array|undefined)[] = [];
  private received = 0;
  private bytes = 0;

  // Returns the assembled JSON once the last missing chunk arrives, and null
  // for a datagram that is not a system chunk or does not complete one.
  push(datagram: Uint8Array): string|null {
    const chunk = systemChunk(datagram);
    if (!chunk) return null;

    if (chunk.documentId !== this.documentId) {
      this.documentId = chunk.documentId;
      this.chunks = new Array(chunk.count);
      this.received = 0;
      this.bytes = 0;
    }
    // A count that disagrees with the rest of the document means two
    // generations collided on one id; start over from this chunk.
    if (this.chunks.length !== chunk.count) {
      this.chunks = new Array(chunk.count);
      this.received = 0;
      this.bytes = 0;
    }
    if (this.chunks[chunk.index]) return null;

    this.chunks[chunk.index] = chunk.payload;
    this.received++;
    this.bytes += chunk.payload.length;
    if (this.bytes > MAX_SYSTEM_BYTES) {
      this.reset();
      return null;
    }
    if (this.received !== chunk.count) return null;

    const whole = new Uint8Array(this.bytes);
    let at = 0;
    for (const part of this.chunks) {
      whole.set(part!, at);
      at += part!.length;
    }
    this.reset();
    return new TextDecoder().decode(whole);
  }

  private reset() {
    this.documentId = -1;
    this.chunks = [];
    this.received = 0;
    this.bytes = 0;
  }
}

// --- derived views ---------------------------------------------------------

export type TopicHealth =
    'ok'|'orphaned'|'unread'|'lossy'|'idle'|'bridged'|'unconnected';

// Every state, in the order the chips and the summaries list them: the two
// faults first, then loss, then the states that are not faults at all.
export const TOPIC_HEALTH: TopicHealth[] =
    ['ok', 'orphaned', 'unread', 'lossy', 'idle', 'bridged', 'unconnected'];

// What both ends of a topic said about their far end, gathered across the nodes
// that hold them.
export type TopicEnds = {external: boolean; optional: boolean};

const PLAIN_ENDS: TopicEnds = {external: false, optional: false};

// Anything that holds ends: a live node, or a declared one. Both answer the
// same question about a topic, so both feed the same aggregation.
type EndHolder = {
  sources: readonly
      {kind: SourceKind; name: string; external: boolean; optional: boolean}[];
};

// The endpoint attributes of every topic, by name.
//
// Derived from the ends rather than read off a topic row. A version-2 bridge
// aggregates the same two booleans onto each topic and sends those as well, and
// this deliberately ignores them: the ends are the only place the answer exists
// for a declared graph, which has no topic rows at all, and for a version-1
// document, which has no topic attributes either. One derivation that covers
// all three is worth more than a preference between two spellings of the same
// aggregation -- it is the same rule naming::CheckGraph applies, so the viewer
// and the launcher's linter agree about which half-wired topics are faults.
export function topicEnds(nodes: readonly EndHolder[]): Map<string, TopicEnds> {
  const ends = new Map<string, TopicEnds>();
  for (const node of nodes) {
    for (const source of node.sources) {
      // A timer has a name but nothing on the other end, so it is not a topic.
      if (source.kind === 'TIMER') continue;
      const current = ends.get(source.name) ?? {external: false, optional: false};
      current.external = current.external || source.external;
      current.optional = current.optional || source.optional;
      ends.set(source.name, current);
    }
  }
  return ends;
}

// What is worth flagging about a topic, in the order a person cares about it.
//
// A subscriber waiting on a topic nothing publishes cannot tell from the
// inside: it just never receives a message. That is the most useful thing this
// view can surface, so a missing end comes first -- but only when a missing end
// is news. An end declared `external` has its peer outside talOS and no
// shared-memory counterpart will ever appear; an end declared `optional` is
// allowed to be waiting for one. Calling either a fault is how an attention
// list earns the habit of being ignored, which costs more than the two states
// it would have reported.
export function topicHealth(topic: SystemTopic,
                            ends: TopicEnds = PLAIN_ENDS): TopicHealth {
  const halfWired = !topic.publishers.length || !topic.subscribers.length;
  if (halfWired && ends.external) return 'bridged';
  if (halfWired && ends.optional) return 'unconnected';
  if (!topic.publishers.length) return 'orphaned';
  if (!topic.subscribers.length) return 'unread';
  if (topic.dropped !== '0') return 'lossy';
  if (topic.published === '0') return 'idle';
  return 'ok';
}

// Health for any topic in a graph, with the attributes of its ends applied.
//
// Gathers the ends once and hands back the classifier, rather than a map: a
// caller holding a topic row asks about that row and cannot be handed undefined
// for a topic that is right there in the graph. The one-topic form above takes
// the attributes separately so a caller with a single row does not have to walk
// every node to classify it.
export function topicHealthOf(graph: SystemGraph): (topic: SystemTopic) =>
    TopicHealth {
  const ends = topicEnds(graph.nodes);
  return topic => topicHealth(topic, ends.get(topic.name));
}

// Declared against actual, per node.
//
// "Declared and never registered" is the answer the live registry cannot give
// on its own: a node that crashed before it claimed a slot, or whose binary was
// missing, is simply absent from it -- and so is a node that was never part of
// this session. A node that started and then stopped is neither of those; it
// keeps its slot with its counters frozen, which is what `alive` reports.
//
// Null when no live graph has arrived: with no registry to compare against,
// "never registered" is not a claim that can be made, and answering zero would
// be inventing one.
export type DeclaredStatus = {
  declared: number;
  registered: number;
  missing: DeclaredNode[];
  undeclared: SystemNode[];
  errors: number;
  warnings: number;
};

export function declaredStatus(declared: DeclaredGraph,
                               graph: SystemGraph|null): DeclaredStatus|null {
  if (!graph) return null;
  const live = new Set(graph.nodes.map(node => node.name));
  const named = new Set(declared.nodes.map(node => node.name));
  return {
    declared: declared.nodes.length,
    registered: declared.nodes.filter(node => live.has(node.name)).length,
    missing: declared.nodes.filter(node => !live.has(node.name)),
    undeclared: graph.nodes.filter(node => !named.has(node.name)),
    errors: declared.diagnostics.filter(d => d.severity === 'error').length,
    warnings: declared.diagnostics.filter(d => d.severity === 'warning').length
  };
}

export const isSubscriber = (kind: SourceKind) =>
    kind === 'WATCHER' || kind === 'FETCHER';

export const rateKey = (node: string, sourceId: number) => `${node} ${sourceId}`;

// Events per second per source, from two graphs and the wall clock between
// them. Counters are cumulative, so a rate needs a previous sample; without
// one this reports nothing rather than zero, because "not measured yet" and
// "not moving" are different answers.
export function eventRates(current: SystemGraph,
                           previous: SystemGraph|null): Map<string, number> {
  const rates = new Map<string, number>();
  if (!previous) return rates;
  const seconds =
      Number(BigInt(current.wall_ns) - BigInt(previous.wall_ns)) / 1e9;
  if (!(seconds > 0)) return rates;

  const before = new Map<string, bigint>();
  for (const node of previous.nodes)
    for (const source of node.sources)
      before.set(rateKey(node.name, source.id), BigInt(source.events));

  for (const node of current.nodes) {
    for (const source of node.sources) {
      const key = rateKey(node.name, source.id);
      const start = before.get(key);
      if (start === undefined) continue;
      const delta = BigInt(source.events) - start;
      // A restarted node resets its counters. A negative delta is that, not a
      // rate, so it is dropped rather than reported as a large negative number.
      if (delta >= 0n) rates.set(key, Number(delta) / seconds);
    }
  }
  return rates;
}
