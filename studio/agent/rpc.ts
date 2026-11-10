// The agent-facing query surface, with no transport and no Node dependencies.
//
// Split out of server.ts so the same code answers questions in three places:
// the headless JSON-RPC server, the Studio worker (so the Agent tab shows
// exactly what an agent would get back, rather than a second implementation
// that drifts), and the tests. Anything imported here must run in a browser.

import {type Snapshot, Timeline} from '../src/core';
import {
  type DeclaredGraph,
  declaredStatus,
  endpointAttributesKnown,
  eventRates,
  RateWindow,
  rateKey,
  type SystemGraph,
  TOPIC_HEALTH,
  topicHealthOf,
} from '../src/system';
import {graphStale} from '../src/data/freshness';

export class RpcFault extends Error {
  constructor(public code: number, message: string) {
    super(message);
  }
}

export type Params = Record<string, unknown>;

// Everything the methods can read. A context rather than a bare timeline
// because the system graph is a second, independent source of truth: what is
// running, as opposed to what it measured.
export type RpcContext = {
  timeline: Timeline;
  system: SystemGraph|null;
  previousSystem?: SystemGraph|null;

  // A window of graphs to measure rates over. Preferred over `previousSystem`
  // when present, because a single refresh apart quantises a rate into ~4 Hz
  // steps and reports 0 for a source that did not advance across it. Optional
  // so a caller holding only two graphs still gets the old answer rather than
  // no answer.
  rateWindow?: RateWindow|null;

  // What the launcher declared before it spawned anything, when the bridge was
  // started with `--declared`. A third source of truth: not what is running and
  // not what it measured, but what was supposed to be running -- which is the
  // only way to tell a node that died on the way up from one that was never
  // asked for.
  declared?: DeclaredGraph|null;

  // performance.now() when `system` arrived, and whether the connection that
  // delivered it is still open. A hung bridge sends nothing, so only the
  // graph's age says that its nodes' `alive` is no longer about now. Absent
  // arrival means the caller does not know it: age and staleness are null.
  // Absent `connected` judges by age alone.
  systemArrivalMs?: number|null;
  connected?: boolean;
};

// Rates come from the window when the caller keeps one, and from the single
// previous graph otherwise.
const ratesFor = (context: RpcContext, system: SystemGraph) =>
    context.rateWindow ? context.rateWindow.rates() :
                         eventRates(system, context.previousSystem ?? null);

const invalid = (message: string): never => {
  throw new RpcFault(-32602, message);
};

function timestamp(value: unknown): bigint {
  if (typeof value !== 'string' || !/^\d+$/.test(value))
    return invalid('timestamp_ns must be a decimal string');
  const parsed = BigInt(value);
  if (parsed > 18446744073709551615n)
    return invalid('timestamp_ns exceeds uint64');
  return parsed;
}

// A channel's value: its number, or for an integer channel whose number is
// not exact (past 2^53) its exact decimal string. A NaN or infinite reading
// is its number here; `answer` spells it for JSON. Other topics walk the
// snapshot.
function topicValue(frame: Snapshot, topic: string): unknown {
  if (Object.hasOwn(frame.channels, topic)) {
    const value = frame.channels[topic];
    const exact = frame.exact;
    return exact && Object.hasOwn(exact, topic) && !Number.isSafeInteger(value) ?
        exact[topic] :
        value;
  }
  let value: unknown = frame;
  for (const part of topic.split('.')) {
    if (['__proto__', 'prototype', 'constructor'].includes(part) || !value ||
        typeof value !== 'object' || !Object.hasOwn(value, part))
      return undefined;
    value = (value as Record<string, unknown>)[part];
  }
  return value;
}

// A value as an answer carries it: JSON has no NaN or infinities, which
// JSON.stringify would turn into null (absence), so they are the strings
// "NaN", "Infinity" and "-Infinity", as in the bridge's JSON.
const answer = (value: unknown) =>
    typeof value === 'number' && !Number.isFinite(value) ? String(value) : value;

const KIND_NAMES = ['double', 'int', 'uint', 'bool'];

// A snapshot as an answer: channels as `topicValue` reads them, spelled by
// `answer`, and each channel's kind by name. (The exact decimals are in
// `channels`, so `exact` is not repeated.)
function snapshotAnswer(frame: Snapshot): Record<string, unknown> {
  const {exact: _exact, kinds, channels, ...rest} = frame;
  const out: Record<string, unknown> = {...rest};
  const values: Record<string, unknown> = Object.create(null);
  for (const name of Object.keys(channels))
    values[name] = answer(topicValue(frame, name));
  out.channels = values;
  if (kinds) {
    const names: Record<string, string> = Object.create(null);
    for (const [name, kind] of Object.entries(kinds))
      names[name] = KIND_NAMES[kind] ?? 'unknown';
    out.kinds = names;
  }
  return out;
}

// -1, 0 or 1 as `value` is below, at or above `limit`, null when it is not a
// number or is NaN (which orders with nothing); an exact integer string
// compares exactly against an integer limit.
function compare(value: unknown, limit: number): number|null {
  if (typeof value === 'string' && /^-?\d+$/.test(value)) {
    if (Number.isInteger(limit)) {
      const a = BigInt(value), b = BigInt(limit);
      return a < b ? -1 : a > b ? 1 : 0;
    }
    value = Number(value);
  }
  if (typeof value !== 'number' || Number.isNaN(value)) return null;
  return value < limit ? -1 : value > limit ? 1 : 0;
}

// The readings an answer spells as strings (see `answer`), as an equality
// limit names them.
const SPELLED: Record<string, number> = {
  NaN: NaN,
  Infinity: Infinity,
  '-Infinity': -Infinity
};

// Equality with an exact integer string accepts the same integer as a
// number or as its decimal string; a limit spelled as `answer` spells NaN or
// an infinity names that reading, and NaN equals NaN.
function same(value: unknown, limit: unknown): boolean {
  if (typeof limit === 'string' && Object.hasOwn(SPELLED, limit)) {
    const special = SPELLED[limit];
    return typeof value === 'number' &&
        (Number.isNaN(special) ? Number.isNaN(value) : value === special);
  }
  if (typeof value === 'string' && typeof limit === 'number' &&
      Number.isInteger(limit) && /^-?\d+$/.test(value))
    return BigInt(value) === BigInt(limit);
  return value === limit;
}

// Whether two successive readings differ: NaN is the same reading as NaN.
const differs = (value: unknown, previous: unknown) =>
    value !== previous &&
    !(Number.isNaN(value as number) && Number.isNaN(previous as number) &&
      typeof value === 'number' && typeof previous === 'number');

const truthy = (value: unknown) =>
    typeof value === 'string' && /^-?\d+$/.test(value) ? BigInt(value) !== 0n :
                                                        Boolean(value);

function topicList(value: unknown): string[] {
  if (!Array.isArray(value) || value.some(t => typeof t !== 'string') ||
      value.length > 256)
    return invalid('topics must be an array of at most 256 strings');
  return value as string[];
}

// A system graph is only present once the bridge has sent one. Saying so is
// better than returning an empty graph, which an agent would read as "nothing
// is running" rather than "nobody has told me yet".
function requireSystem(context: RpcContext): SystemGraph {
  if (!context.system)
    throw new RpcFault(
        -32002,
        'No system graph received yet; is the Studio bridge connected?');
  return context.system;
}

// Same reasoning one step earlier: a declared graph exists only if the bridge
// was given one. An empty graph would read as "the launcher declared nothing",
// which is a statement about the robot rather than about this connection.
function requireDeclared(context: RpcContext): DeclaredGraph {
  if (!context.declared)
    throw new RpcFault(
        -32002,
        'No declared graph received yet; was the bridge started with --declared?');
  return context.declared;
}

export function dispatch(
    context: RpcContext, method: string, params: Params = {}): unknown {
  const {timeline} = context;

  // The newest retained sample, with its dictionary context. The zero-history
  // question ("what is it doing right now?") without a timestamp to guess:
  // units and labels travel with it because a bare number without either is
  // only half an answer, and age says how stale "latest" is.
  if (method === 'get_latest') {
    const frame = timeline.latest();
    if (!frame)
      throw new RpcFault(
          -32002, 'No telemetry received yet; is the Studio bridge connected?');
    const ageMs = timeline.lastArrivalMs ?
        Math.max(0, Date.now() - timeline.lastArrivalMs) :
        null;
    const snapshot = snapshotAnswer(frame);
    return {
      ...snapshot,
      snapshot,
      age_ms: ageMs,
      age_ns: ageMs === null ? null : String(Math.round(ageMs * 1e6))
    };
  }

  // A channel is labelled with its kind, which says what its answers hold:
  // `double` is a number or "NaN", "Infinity", "-Infinity"; `int` and `uint`
  // a number, or a decimal string beyond 2^53. The frame's `exact` and
  // `kinds` maps are how the decoder carries that, not topics.
  if (method === 'get_schema_tree') {
    const topics: Record<string, string> = {};
    function walk(value: unknown, prefix: string) {
      if (value !== null && typeof value === 'object') {
        for (const [key, child] of Object.entries(value))
          walk(child, prefix ? `${prefix}.${key}` : key);
      } else
        topics[prefix] = typeof value;
    }
    for (const frame of timeline.frames) {
      const {exact: _exact, kinds, ...rest} = frame;
      walk(rest, '');
      for (const name of Object.keys(frame.channels))
        topics[name] = kinds && Object.hasOwn(kinds, name) ?
            KIND_NAMES[kinds[name]] ?? 'unknown' :
            'double';
    }
    return {
      topics,
      timestamp_bounds: timeline.bounds(),
      frames: timeline.frames.length,
      timestamp_encoding: 'uint64 decimal string',
      snapshot_policy: 'exact'
    };
  }

  if (method === 'query_state_at') {
    const t = timestamp(params.timestamp_ns);
    if (params.mode !== undefined && params.mode !== 'exact' &&
        params.mode !== 'at_or_before')
      invalid('mode must be exact or at_or_before');
    const frame = timeline.at(t);
    if (!frame ||
        (params.mode !== 'at_or_before' && BigInt(frame.timestamp_ns) !== t))
      throw new RpcFault(-32001, 'No retained sample at requested timestamp');
    if (params.topics === undefined) return snapshotAnswer(frame);
    const values: Record<string, unknown> = {};
    for (const topic of topicList(params.topics)) {
      const value = topicValue(frame, topic);
      if (value === undefined) invalid(`Unknown topic: ${topic}`);
      Object.defineProperty(values, topic, {value: answer(value), enumerable: true});
    }
    return {
      timestamp_ns: frame.timestamp_ns,
      sequence_id: frame.sequence_id,
      topics: values
    };
  }

  if (method === 'scan_channel_events') {
    if (typeof params.topic !== 'string') invalid('topic is required');
    const topic = params.topic as string;
    const condition = params.condition as Params | undefined;
    if (!condition || typeof condition !== 'object')
      invalid('condition must be an object');
    const op = condition!.op;
    const ops =
        ['gt', 'gte', 'lt', 'lte', 'eq', 'ne', 'changed', 'rising', 'falling'];
    if (typeof op !== 'string' || !ops.includes(op))
      invalid(`condition.op must be ${ops.join(', ')}`);
    if (['gt', 'gte', 'lt', 'lte'].includes(op as string) &&
        (typeof condition!.value !== 'number' ||
         !Number.isFinite(condition!.value)))
      invalid('Numeric comparison requires finite condition.value');
    if (['eq', 'ne'].includes(op as string) &&
        !['number', 'boolean', 'string'].includes(typeof condition!.value))
      invalid('Equality requires scalar condition.value');
    const start =
        params.start_ns === undefined ? null : timestamp(params.start_ns);
    const end = params.end_ns === undefined ? null : timestamp(params.end_ns);
    if (start !== null && end !== null && start > end)
      invalid('start_ns must be <= end_ns');
    const ranges: {start_ns: string; end_ns: string; samples: number}[] = [];
    let active: typeof ranges[number]|null = null;
    let previous: unknown;
    let seen = false;
    let found = false;
    for (const frame of timeline.frames) {
      const t = BigInt(frame.timestamp_ns);
      const value = topicValue(frame, topic);
      if (value !== undefined) found = true;
      const inBounds =
          (start === null || t >= start) && (end === null || t <= end);
      let matches = false;
      const limit = condition!.value;
      if (value !== undefined && inBounds) {
        switch (op) {
          case 'gt':
            matches = (compare(value, limit as number) ?? 0) > 0;
            break;
          case 'gte':
            matches = (compare(value, limit as number) ?? -1) >= 0;
            break;
          case 'lt':
            matches = (compare(value, limit as number) ?? 0) < 0;
            break;
          case 'lte':
            matches = (compare(value, limit as number) ?? 1) <= 0;
            break;
          case 'eq':
            matches = same(value, limit);
            break;
          case 'ne':
            matches = !same(value, limit);
            break;
          case 'changed':
            matches = seen && differs(value, previous);
            break;
          case 'rising':
            matches = seen && !truthy(previous) && truthy(value);
            break;
          case 'falling':
            matches = seen && truthy(previous) && !truthy(value);
            break;
        }
      }
      if (matches) {
        if (!active) {
          active = {
            start_ns: frame.timestamp_ns,
            end_ns: frame.timestamp_ns,
            samples: 0
          };
          ranges.push(active);
        }
        active.end_ns = frame.timestamp_ns;
        active.samples++;
      } else
        active = null;
      previous = value;
      seen = value !== undefined;
    }
    if (!found && timeline.frames.length) invalid(`Unknown topic: ${topic}`);
    return {topic, condition, ranges, timestamp_bounds: timeline.bounds()};
  }

  // What is running, and who is on each end of every topic. This is the
  // question a viewer cannot answer from telemetry alone: a frame shows what
  // one node chose to publish, not the shape of the system that produced it.
  if (method === 'get_system_graph') {
    const system = requireSystem(context);
    const arrival = context.systemArrivalMs ?? null;
    const now = performance.now();
    const rates = ratesFor(context, system);
    const health = topicHealthOf(system);
    // One tally per state, so an agent does not have to know which of them are
    // faults to count the faults: `bridged` and `unconnected` are half-wired
    // topics that the nodes holding them declared as such.
    const tally: Record<string, number> = {};
    for (const state of TOPIC_HEALTH) tally[state] = 0;
    for (const topic of system.topics) tally[health(topic)]++;
    return {
      // Which document this is. Below version 2 a source could not state
      // whether its far end is outside talOS, so `bridged` and `unconnected`
      // are unreachable and the topics they describe are reported as faults --
      // the older bridge's answer, and worth being able to tell apart from a
      // graph that says so.
      version: system.version,
      wall_ns: system.wall_ns,
      // Whether this graph still says what is running now; when it is stale,
      // every node's `alive` is last-known evidence, not a claim about now.
      stale: arrival === null ? null :
                                graphStale(arrival, now, context.connected ?? true),
      age_ms: arrival === null ? null : Math.max(0, now - arrival),
      registry: system.registry,
      bridge: system.bridge,
      nodes: system.nodes.map(node => ({
                               ...node,
                               sources: node.sources.map(source => ({
                                                           ...source,
                                                           events_per_second:
                                                               rates.get(rateKey(
                                                                   node.name,
                                                                   source.id)) ??
                                                               null
                                                         }))
                             })),
      topics: system.topics.map(topic => ({...topic, health: health(topic)})),
      summary: {
        nodes: system.nodes.length,
        nodes_alive: system.nodes.filter(n => n.alive).length,
        topics: system.topics.length,
        orphaned_topics: tally.orphaned,
        unread_topics: tally.unread,
        lossy_topics: tally.lossy,
        bridged_topics: tally.bridged,
        unconnected_topics: tally.unconnected,
        topics_by_health: tally,
        endpoint_attributes: endpointAttributesKnown(system)
      }
    };
  }

  // One topic, both ends, named. Saves an agent filtering the whole graph to
  // answer "who writes this, and is anyone reading it".
  if (method === 'describe_topic') {
    const system = requireSystem(context);
    if (typeof params.topic !== 'string') invalid('topic is required');
    const name = params.topic as string;
    const topic = system.topics.find(t => t.name === name);
    if (!topic) invalid(`Unknown topic: ${name}`);

    const rates = ratesFor(context, system);
    const ends: unknown[] = [];
    for (const node of system.nodes) {
      for (const source of node.sources) {
        if (source.name !== name) continue;
        ends.push({
          node: node.name,
          alive: node.alive,
          kind: source.kind,
          source_id: source.id,
          events: source.events,
          dropped: source.dropped,
          sequence: source.sequence,
          // Why a missing peer on this end is or is not a fault, as the node
          // holding it declared.
          external: source.external,
          optional: source.optional,
          events_per_second: rates.get(rateKey(node.name, source.id)) ?? null
        });
      }
    }
    return {...topic!, health: topicHealthOf(system)(topic!), ends};
  }

  // What was supposed to be running, and what of it is. The comparison is the
  // point: the live registry holds only what did start, so a name missing from
  // it is either a node that crashed before it claimed a slot or a node that
  // was never part of this session, and from the registry those are the same
  // silence.
  if (method === 'get_declared_graph') {
    const declared = requireDeclared(context);
    const status = declaredStatus(declared, context.system);
    return {
      version: declared.version,
      nodes: declared.nodes,
      diagnostics: declared.diagnostics,
      // Null rather than zeroes when no system graph has arrived: without a
      // registry to check against, "never registered" is not a claim that can
      // be made, and answering zero would invent one.
      comparison: status ? {
        declared: status.declared,
        registered: status.registered,
        missing: status.missing.map(node => ({
                                      name: node.name,
                                      target: node.target,
                                      sources: node.sources.length
                                    })),
        undeclared: status.undeclared.map(node => node.name)
      } :
                           null,
      summary: {
        nodes: declared.nodes.length,
        sources: declared.nodes.reduce(
            (total, node) => total + node.sources.length, 0),
        errors: declared.diagnostics.filter(d => d.severity === 'error').length,
        warnings:
            declared.diagnostics.filter(d => d.severity === 'warning').length
      }
    };
  }

  throw new RpcFault(-32601, 'Method not found');
}

export function handleRpc(context: RpcContext, input: unknown): unknown|
    undefined {
  if (!input || typeof input !== 'object' || Array.isArray(input))
    return {
      jsonrpc: '2.0',
      id: null,
      error: {code: -32600, message: 'Invalid Request'}
    };
  const request = input as Params;
  const id = request.id ?? null;
  if (request.jsonrpc !== '2.0' || typeof request.method !== 'string' ||
      (request.id !== undefined && request.id !== null &&
       typeof request.id !== 'number' && typeof request.id !== 'string') ||
      (request.params !== undefined &&
       (!request.params || typeof request.params !== 'object' ||
        Array.isArray(request.params))))
    return {
      jsonrpc: '2.0',
      id: null,
      error: {code: -32600, message: 'Invalid Request'}
    };
  try {
    const result = dispatch(context, request.method, request.params as Params);
    return request.id === undefined ? undefined : {jsonrpc: '2.0', id, result};
  } catch (e) {
    return request.id === undefined ? undefined : {
      jsonrpc: '2.0',
      id,
      error: {
        code: e instanceof RpcFault ? e.code : -32603,
        message: e instanceof RpcFault ? e.message : 'Internal error'
      }
    };
  }
}

// Every method name, for the Agent tab's picker and for documentation that
// cannot fall out of step with the implementation.
export const RPC_METHODS = [
  'get_latest', 'get_schema_tree', 'query_state_at', 'scan_channel_events',
  'get_system_graph', 'describe_topic', 'get_declared_graph'
] as const;
