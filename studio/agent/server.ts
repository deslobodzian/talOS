import {createServer} from 'node:http';
import {pathToFileURL} from 'node:url';
import {WebSocket, WebSocketServer} from 'ws';

import {type Snapshot, TelemetryDecoder} from '../src/core';
import {isPiecesMessage} from '../src/data/pieces';
import {TelemetryHistory} from '../src/history';
import {
  type DeclaredGraph,
  declaredStatus,
  declaredUrl,
  GraphVersionMismatch,
  MAX_DECLARED_BYTES,
  MAX_SYSTEM_BYTES,
  parseDeclaredGraph,
  RateWindow,
  type SystemGraph,
  SystemGraphStream,
} from '../src/system';

import {handleRpc, type RpcContext} from './rpc';

// The most requests one JSON-RPC batch may carry. 64 KiB of text holds
// hundreds of scan_channel_events requests, each a pass over the whole
// retained timeline on the one thread that also ingests telemetry.
export const MAX_BATCH = 16;

// Maximum concurrent local WebSocket RPC clients and outbound buffer before
// inbound frames are dropped without parsing.
export const MAX_WS_CLIENTS = 16;
const MAX_WS_BUFFERED_BYTES = 1024 * 1024;
const DECLARED_FETCH_TIMEOUT_MS = 10_000;

export function startAgent(
    port = Number(process.env.AGENT_PORT ?? 5804),
    upstream = process.env.TELEMETRY_URL ?? 'ws://127.0.0.1:5800/telemetry') {
  // Reject DNS rebinding to loopback as well as browser-originated requests.
  const allowedHost = (host: string|undefined) =>
      !!host && /^(127\.0\.0\.1|localhost)(:\d+)?$/.test(host);
  const decoder = new TelemetryDecoder();
  // Stream and sequence ids whose age is unknown, as in the Studio worker: the
  // keyframe the bridge sends on every open carries the latest tick it holds
  // -- possibly long before this connection, if telemetry has stopped.
  const unknownAge = new Set<string>();
  const ageKey = (f: Snapshot) => `${f.stream_id}:${f.sequence_id}`;
  // The same retained history as the Studio worker's (history.ts): a producer
  // restart within a launcher session continues it, a new session replaces
  // it. An arrival of 0 is Timeline's "no known arrival": get_latest then
  // reports the age as null rather than as the age of the connection.
  const history = new TelemetryHistory(
      {arrival: f => unknownAge.delete(ageKey(f)) ? 0 : Date.now()});
  let firstUpdate = false;

  // `previousSystem` is the sample just before this one: what changed since
  // the last refresh. Rates need a wider baseline than that -- one refresh
  // apart quantises them into ~4 Hz steps -- so they come from the window.
  let system: SystemGraph|null = null;
  let previousSystem: SystemGraph|null = null;
  const rateWindow = new RateWindow();
  // Whole graphs and their change-only updates, applied in order.
  const graphStream = new SystemGraphStream();
  // Graphs received (whole or updated), deferral notices, and rejects.
  const graphs = {received: 0, deferred: 0, invalid: 0};
  // performance.now() when `system` arrived: a hung bridge sends nothing, and
  // only the graph's age can say so.
  let systemArrivalMs: number|null = null;
  // Set while the bridge sends a graph version this agent cannot read.
  let systemMismatch: GraphVersionMismatch|null = null;

  // What the launcher declared before it spawned anything. Fetched over HTTP
  // from the same origin as the telemetry socket rather than pushed: the
  // launcher writes it once, before the session starts, so it does not change
  // while one runs.
  let declared: DeclaredGraph|null = null;
  let declaredError = '';
  let declaredFetch: AbortController|null = null;
  let stopped = false;

  const abortDeclaredFetch = () => {
    declaredFetch?.abort();
    declaredFetch = null;
  };

  const context = (): RpcContext => ({
    timeline: history.timeline,
    system,
    previousSystem,
    rateWindow,
    declared,
    systemArrivalMs,
    systemMismatch,
    connected: socket?.readyState === WebSocket.OPEN
  });

  // A malformed document is reported and dropped, keeping the last good graph:
  // the same rule the live graph follows, for the same reason.
  const ingestDeclaredGraph = (text: string) => {
    try {
      declared = parseDeclaredGraph(text);
      declaredError = '';
    } catch (e) {
      declaredError = String(e);
    }
  };

  // A 404 is the ordinary answer -- `--declared` is optional -- and so is a
  // refused connection, which only repeats what the socket already reports.
  // Neither is recorded as an error, so `get_declared_graph` keeps saying "no
  // declared graph received yet" rather than inventing a reason.
  const fetchDeclaredGraph = async () => {
    abortDeclaredFetch();
    const target = declaredUrl(upstream);
    if (!target || stopped) return;
    const controller = new AbortController();
    declaredFetch = controller;
    const stale = () =>
        stopped || controller.signal.aborted || declaredFetch !== controller;
    const timer =
        setTimeout(() => controller.abort(), DECLARED_FETCH_TIMEOUT_MS);
    try {
      const response =
          await fetch(target, {cache: 'no-store', signal: controller.signal});
      if (stale()) {
        void response.body?.cancel().catch(() => {});
        return;
      }
      if (response.status === 404) {
        void response.body?.cancel().catch(() => {});
        declared = null;
        declaredError = '';
        return;
      }
      if (!response.ok) {
        void response.body?.cancel().catch(() => {});
        declaredError = `Declared graph unavailable: HTTP ${response.status}`;
        return;
      }
      // Advisory fast path before reading chunks; the stream loop below
      // enforces the real cap for chunked or missing Content-Length.
      const contentLength = response.headers.get('content-length');
      const declaredSize = contentLength === null ? 0 : Number(contentLength);
      if (Number.isFinite(declaredSize) && declaredSize > MAX_DECLARED_BYTES) {
        declaredError = 'Invalid declared graph: document too large';
        void response.body?.cancel().catch(() => {});
        controller.abort();
        return;
      }
      let text: string;
      if (!response.body) {
        text = await response.text();
        if (stale()) return;
        if (Buffer.byteLength(text, 'utf8') > MAX_DECLARED_BYTES) {
          declaredError = 'Invalid declared graph: document too large';
          return;
        }
      } else {
        const reader = response.body.getReader();
        const chunks: Uint8Array[] = [];
        let total = 0;
        let finished = false;
        try {
          for (;;) {
            const {done, value} = await reader.read();
            if (stale()) return;
            if (done) {
              finished = true;
              break;
            }
            total += value.byteLength;
            if (total > MAX_DECLARED_BYTES) {
              declaredError = 'Invalid declared graph: document too large';
              void reader.cancel().catch(() => {});
              finished = true;
              controller.abort();
              return;
            }
            chunks.push(value);
          }
        } finally {
          if (!finished) void reader.cancel().catch(() => {});
        }
        text = Buffer.concat(chunks, total).toString('utf8');
      }
      if (stale()) return;
      ingestDeclaredGraph(text);
    } catch {
      // Unreachable bridge or aborted fetch; the upstream socket is the thing
      // that reports connection loss.
    } finally {
      clearTimeout(timer);
      if (declaredFetch === controller) declaredFetch = null;
    }
  };

  const ingestSystemGraph = (text: string) => {
    try {
      const parsed = graphStream.ingest(text, performance.now());
      systemArrivalMs = graphStream.arrival;
      if (!parsed) {  // deferred by the budget, not lost
        graphs.deferred++;
        return;
      }
      graphs.received++;
      previousSystem = system;
      system = parsed;
      rateWindow.push(parsed);
      history.observe(parsed);
      systemMismatch = null;
    } catch (e) {
      // A malformed document is counted and dropped. Blanking the last good
      // graph would turn one bad frame into "nothing is running", which is a
      // worse answer than a slightly stale one.
      graphs.invalid++;
      // A graph of another version is not one bad frame: nothing this bridge
      // sends can be read, so no graph is kept, and get_system_graph names
      // both versions rather than saying none arrived.
      if (e instanceof GraphVersionMismatch) {
        systemMismatch = e;
        system = null;
        previousSystem = null;
        rateWindow.reset();
        systemArrivalMs = null;
      }
    }
  };

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
        if (input.length > MAX_BATCH)
          return {
            jsonrpc: '2.0',
            id: null,
            error: {
              code: -32600,
              message: `Batch exceeds ${MAX_BATCH} requests`
            }
          };
        // One context for the whole batch, so every request in it sees the
        // same system state.
        const shared = context();
        const results = input.map(req => handleRpc(shared, req))
                            .filter(value => value !== undefined);
        return results.length ? results : undefined;
      }
      return handleRpc(context(), input);
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
    if ('origin' in req.headers || !allowedHost(req.headers.host)) {
      res.writeHead(403).end();
      return;
    }
    if (req.method === 'GET' && req.url === '/health') {
      res.setHeader('Content-Type', 'application/json');
      res.end(JSON.stringify({
        connected: socket?.readyState === WebSocket.OPEN,
        frames: history.timeline.frames.length,
        stats: history.jitter.stats,
        // Whether the shape of the system is known, separately from whether
        // telemetry is arriving: a node can be publishing frames while the
        // registry is unreadable, and the reverse.
        system: {
          received: graphs.received > 0,
          graphs: graphs.received,
          deferred: graphs.deferred,
          invalid: graphs.invalid,
          nodes: system?.nodes.length ?? 0,
          nodes_alive: system?.nodes.filter(node => node.alive).length ?? 0,
          topics: system?.topics.length ?? 0,
          registry_available: system?.registry.available ?? false
        },
        // Reported separately again, and null rather than zero when the bridge
        // was started without --declared: "nobody declared anything" and "no
        // declared graph was offered" are different answers.
        declared: declared ? {
          received: true,
          nodes: declared.nodes.length,
          diagnostics: declared.diagnostics.length,
          registered: declaredStatus(declared, system)?.registered ?? null,
          missing: declaredStatus(declared, system)?.missing.length ?? null
        } :
                             {received: false, error: declaredError || null}
      }));
      return;
    }
    if (req.method !== 'POST' || req.url !== '/rpc') {
      res.writeHead(404).end();
      return;
    }
    let size = 0;
    let tooLarge = false;
    const chunks: Buffer[] = [];
    req.on('error', () => {});
    req.on('data', (chunk: Buffer) => {
      if (tooLarge) return;
      size += chunk.length;
      if (size > 65536) {
        tooLarge = true;
        chunks.length = 0;
        if (!res.writableEnded) res.writeHead(413).end();
        req.destroy();
      } else
        chunks.push(chunk);
    });
    req.on('end', () => {
      if (tooLarge || res.writableEnded || req.destroyed) return;
      const response = parse(Buffer.concat(chunks).toString('utf8'));
      res.setHeader('Content-Type', 'application/json');
      res.writeHead(response === undefined ? 204 : 200)
          .end(response === undefined ? undefined : JSON.stringify(response));
    });
  });
  const ws = new WebSocketServer({noServer: true, maxPayload: 65536});
  server.on('upgrade', (req, socket, head) => {
    if (req.url !== '/rpc' || 'origin' in req.headers ||
        !allowedHost(req.headers.host) || ws.clients.size >= MAX_WS_CLIENTS) {
      socket.destroy();
      return;
    }
    ws.handleUpgrade(
        req, socket, head, client => ws.emit('connection', client, req));
  });
  ws.on('connection', client => {
    if (ws.clients.size > MAX_WS_CLIENTS) {
      client.close(1013);
      return;
    }
    client.on('error', () => {
      try { client.close(); } catch {}
    });
    client.on('message', raw => {
      if (client.bufferedAmount >= MAX_WS_BUFFERED_BYTES) return;
      const response = parse(raw.toString());
      if (response !== undefined &&
          client.bufferedAmount < MAX_WS_BUFFERED_BYTES)
        client.send(JSON.stringify(response));
    });
  });
  let socket: WebSocket|undefined;
  let retry: NodeJS.Timeout|undefined;
  function connect() {
    // The upstream carries two message families now. A telemetry frame is at
    // most 16,384 bytes, but a system graph describing every node on every
    // topic is far larger, and a payload cap below it would drop the
    // connection instead of the message. SystemGraphStream enforces the real
    // bound.
    socket = new WebSocket(upstream, {maxPayload: MAX_SYSTEM_BYTES});
    socket.on('message', (raw, binary) => {
      // Binary is telemetry (or the game pieces, by their magic); text is the
      // system graph. The frame type carries the distinction.
      if (!binary) {
        ingestSystemGraph(raw.toString());
        return;
      }
      try {
        const bytes = Array.isArray(raw) ? Buffer.concat(raw) :
            raw instanceof ArrayBuffer   ? new Uint8Array(raw) :
                                           raw;
        // The game pieces (REQ-DEV-43) are for the 3D viewport; the agent
        // answers from telemetry and the graph.
        if (isPiecesMessage(bytes)) return;
        for (const frame of decoder.decodeAll(bytes)) {
          if (!history.push(frame, decoder.currentDictionary(),
                            performance.now()))
            continue;
          if (firstUpdate) unknownAge.add(ageKey(frame));
          firstUpdate = false;
        }
      } catch {
        history.jitter.stats.invalid++;
      }
    });
    socket.on('open', () => {
      // The bridge answers an open with its graph and then, when it has
      // them, the dictionary and a keyframe of its latest tick, whose age is
      // unknown: the first update on a socket.
      firstUpdate = true;
      unknownAge.clear();
      // Asked for on every open: the bridge may have started after this
      // process did, and the first attempt would have had nothing to answer it.
      void fetchDeclaredGraph();
    });
    socket.on('error', () => {});
    socket.on('close', () => {
      // A graph from a lost connection would keep reporting its nodes alive;
      // get_system_graph answers "no graph" until the bridge sends a new one.
      abortDeclaredFetch();
      system = null;
      previousSystem = null;
      systemArrivalMs = null;
      systemMismatch = null;
      graphStream.reset();
      rateWindow.reset();
      decoder.reset();
      if (!stopped) retry = setTimeout(connect, 1000);
    });
  }
  const tick = setInterval(() => {
    history.flush(performance.now());
  }, 5);
  server.listen(
      port, '127.0.0.1',
      () => console.log(
          `Agent JSON-RPC http://127.0.0.1:${port}/rpc ← ${upstream}`));
  connect();
  const close = () => {
    stopped = true;
    abortDeclaredFetch();
    clearInterval(tick);
    clearTimeout(retry);
    socket?.terminate();
    for (const client of ws.clients) client.terminate();
    ws.close();
    server.close();
    server.closeAllConnections();
  };
  return {
    server,
    get timeline() {
      return history.timeline;
    },
    close,
    graphs,
    // Exposed so a caller can inject a graph or read the current one without
    // standing up a bridge.
    context,
    ingestSystemGraph,
    ingestDeclaredGraph
  };
}
if (process.argv[1] &&
    import.meta.url === pathToFileURL(process.argv[1]).href) {
  const agent = startAgent();
  process.once('SIGINT', agent.close);
  process.once('SIGTERM', agent.close);
}
