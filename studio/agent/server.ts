import {createServer} from 'node:http';
import {pathToFileURL} from 'node:url';
import {WebSocket, WebSocketServer} from 'ws';

import {decode, JitterBuffer, type Snapshot, Timeline} from '../src/core';

export class RpcFault extends Error {
  constructor(public code: number, message: string) {
    super(message);
  }
}
type Params = Record<string, unknown>;
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
function topicValue(frame: Snapshot, topic: string): unknown {
  if (Object.hasOwn(frame.channels, topic)) return frame.channels[topic];
  let value: unknown = frame;
  for (const part of topic.split('.')) {
    if (['__proto__', 'prototype', 'constructor'].includes(part) || !value ||
        typeof value !== 'object' || !Object.hasOwn(value, part))
      return undefined;
    value = (value as Record<string, unknown>)[part];
  }
  return value;
}
function topicList(value: unknown): string[] {
  if (!Array.isArray(value) || value.some(t => typeof t !== 'string') ||
      value.length > 256)
    return invalid('topics must be an array of at most 256 strings');
  return value as string[];
}

export function dispatch(
    timeline: Timeline, method: string, params: Params = {}): unknown {
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
      walk(frame, '');
      for (const name of Object.keys(frame.channels)) topics[name] = 'number';
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
    if (params.topics === undefined) return frame;
    const values: Record<string, unknown> = {};
    for (const topic of topicList(params.topics)) {
      const value = topicValue(frame, topic);
      if (value === undefined) invalid(`Unknown topic: ${topic}`);
      Object.defineProperty(values, topic, {value, enumerable: true});
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
            matches = typeof value === 'number' && value > (limit as number);
            break;
          case 'gte':
            matches = typeof value === 'number' && value >= (limit as number);
            break;
          case 'lt':
            matches = typeof value === 'number' && value < (limit as number);
            break;
          case 'lte':
            matches = typeof value === 'number' && value <= (limit as number);
            break;
          case 'eq':
            matches = value === limit;
            break;
          case 'ne':
            matches = value !== limit;
            break;
          case 'changed':
            matches = seen && value !== previous;
            break;
          case 'rising':
            matches = seen && !previous && Boolean(value);
            break;
          case 'falling':
            matches = seen && Boolean(previous) && !value;
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
  throw new RpcFault(-32601, 'Method not found');
}

export function handleRpc(timeline: Timeline, input: unknown): unknown|
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
    const result = dispatch(timeline, request.method, request.params as Params);
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

export function startAgent(
    port = Number(process.env.AGENT_PORT ?? 5802),
    upstream = process.env.TELEMETRY_URL ?? 'ws://127.0.0.1:5800/telemetry') {
  // Reject DNS rebinding to loopback as well as browser-originated requests.
  const allowedHost = (host: string|undefined) =>
      !!host && /^(127\.0\.0\.1|localhost)(:\d+)?$/.test(host);
  const timeline = new Timeline();
  const jitter = new JitterBuffer(10);
  const parse = (text: string) => {
    try {
      const input: unknown = JSON.parse(text);
      if (Array.isArray(input)) {
        if (!input.length)
          return {
            jsonrpc: '2.0',
            id: null,
            error: {code: -32600, message: 'Empty batch'}
          };
        const results = input.map(req => handleRpc(timeline, req))
                            .filter(value => value !== undefined);
        return results.length ? results : undefined;
      }
      return handleRpc(timeline, input);
    } catch {
      return {
        jsonrpc: '2.0',
        id: null,
        error: {code: -32700, message: 'Parse error'}
      };
    }
  };
  const server = createServer((req, res) => {
    // Loopback binding plus Origin rejection prevents browser pages reading
    // local telemetry.
    if (req.headers.origin || !allowedHost(req.headers.host)) {
      res.writeHead(403).end();
      return;
    }
    if (req.method === 'GET' && req.url === '/health') {
      res.setHeader('Content-Type', 'application/json');
      res.end(JSON.stringify({
        connected: socket?.readyState === WebSocket.OPEN,
        frames: timeline.frames.length,
        stats: jitter.stats
      }));
      return;
    }
    if (req.method !== 'POST' || req.url !== '/rpc') {
      res.writeHead(404).end();
      return;
    }
    let size = 0;
    const chunks: Buffer[] = [];
    req.on('data', (chunk: Buffer) => {
      size += chunk.length;
      if (size > 65536) {
        res.writeHead(413).end();
        req.destroy();
      } else
        chunks.push(chunk);
    });
    req.on('end', () => {
      if (res.writableEnded || req.destroyed) return;
      const response = parse(Buffer.concat(chunks).toString('utf8'));
      res.setHeader('Content-Type', 'application/json');
      res.writeHead(response === undefined ? 204 : 200)
          .end(response === undefined ? undefined : JSON.stringify(response));
    });
  });
  const ws = new WebSocketServer({noServer: true, maxPayload: 65536});
  server.on('upgrade', (req, socket, head) => {
    if (req.url !== '/rpc' || req.headers.origin ||
        !allowedHost(req.headers.host)) {
      socket.destroy();
      return;
    }
    ws.handleUpgrade(
        req, socket, head, client => ws.emit('connection', client, req));
  });
  ws.on('connection', client => client.on('message', raw => {
    const response = parse(raw.toString());
    if (response !== undefined && client.bufferedAmount < 1024 * 1024)
      client.send(JSON.stringify(response));
  }));
  let socket: WebSocket|undefined;
  let retry: NodeJS.Timeout|undefined;
  let stopped = false;
  function connect() {
    socket = new WebSocket(upstream, {maxPayload: 65507});
    socket.on('message', (raw, binary) => {
      if (!binary) return;
      try {
        const bytes = Array.isArray(raw) ? Buffer.concat(raw) :
            raw instanceof ArrayBuffer   ? new Uint8Array(raw) :
                                           raw;
        jitter.push(decode(bytes), performance.now());
      } catch {
        jitter.stats.invalid++;
      }
    });
    socket.on('error', () => {});
    socket.on('close', () => {
      if (!stopped) retry = setTimeout(connect, 1000);
    });
  }
  const tick = setInterval(() => {
    for (const frame of jitter.flush(performance.now())) timeline.append(frame);
  }, 5);
  server.listen(
      port, '127.0.0.1',
      () => console.log(
          `Agent JSON-RPC http://127.0.0.1:${port}/rpc ← ${upstream}`));
  connect();
  const close = () => {
    stopped = true;
    clearInterval(tick);
    clearTimeout(retry);
    socket?.terminate();
    for (const client of ws.clients) client.terminate();
    ws.close();
    server.close();
  };
  return {server, timeline, close};
}
if (process.argv[1] &&
    import.meta.url === pathToFileURL(process.argv[1]).href) {
  const agent = startAgent();
  process.once('SIGINT', agent.close);
  process.once('SIGTERM', agent.close);
}
