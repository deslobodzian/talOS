import {createServer} from 'node:http';
import {pathToFileURL} from 'node:url';
import {WebSocket, WebSocketServer} from 'ws';

import {decode, JitterBuffer, Timeline} from '../src/core';
import {
  type DeclaredGraph,
  declaredStatus,
  declaredUrl,
  MAX_DECLARED_BYTES,
  MAX_SYSTEM_BYTES,
  parseDeclaredGraph,
  parseSystemGraph,
  RateWindow,
  type SystemGraph,
} from '../src/system';

import {handleRpc, type RpcContext} from './rpc';

// The query surface itself lives in ./rpc so the browser can run it too. These
// re-exports keep the older import path working, and keep `agent/server` the
// one name a caller has to know.
export {dispatch, handleRpc, RPC_METHODS, RpcFault} from './rpc';
export type {Params, RpcContext} from './rpc';

export function startAgent(
    port = Number(process.env.AGENT_PORT ?? 5802),
    upstream = process.env.TELEMETRY_URL ?? 'ws://127.0.0.1:5800/telemetry') {
  // Reject DNS rebinding to loopback as well as browser-originated requests.
  const allowedHost = (host: string|undefined) =>
      !!host && /^(127\.0\.0\.1|localhost)(:\d+)?$/.test(host);
  const timeline = new Timeline();
  const jitter = new JitterBuffer(10);

  // `previousSystem` is the sample just before this one: what changed since
  // the last refresh. Rates need a wider baseline than that -- one refresh
  // apart quantises them into ~4 Hz steps -- so they come from the window.
  let system: SystemGraph|null = null;
  let previousSystem: SystemGraph|null = null;
  const rateWindow = new RateWindow();
  const graphs = {received: 0, invalid: 0};

  // What the launcher declared before it spawned anything. Fetched over HTTP
  // from the same origin as the telemetry socket rather than pushed: the
  // launcher writes it once, before the session starts, so it does not change
  // while one runs.
  let declared: DeclaredGraph|null = null;
  let declaredError = '';

  const context = (): RpcContext =>
      ({timeline, system, previousSystem, rateWindow, declared});

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
    const target = declaredUrl(upstream);
    if (!target) return;
    try {
      const response = await fetch(target, {cache: 'no-store'});
      if (response.status === 404) return;
      if (!response.ok) {
        declaredError = `Declared graph unavailable: HTTP ${response.status}`;
        return;
      }
      // Advisory, and checked before the body is read rather than after: the
      // parser enforces the real bound, but there is no reason to hold a
      // gigabyte in memory first to find out it was too big.
      const declaredSize = Number(response.headers.get('content-length') ?? 0);
      if (declaredSize > MAX_DECLARED_BYTES) {
        declaredError = 'Invalid declared graph: document too large';
        return;
      }
      ingestDeclaredGraph(await response.text());
    } catch {
      // Unreachable bridge; the upstream socket is the thing that reports that.
    }
  };

  const ingestSystemGraph = (text: string) => {
    try {
      const parsed = parseSystemGraph(text);
      previousSystem = system;
      system = parsed;
      rateWindow.push(parsed);
      graphs.received++;
    } catch {
      // A malformed document is counted and dropped. Blanking the last good
      // graph would turn one bad frame into "nothing is running", which is a
      // worse answer than a slightly stale one.
      graphs.invalid++;
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
    if (req.headers.origin || !allowedHost(req.headers.host)) {
      res.writeHead(403).end();
      return;
    }
    if (req.method === 'GET' && req.url === '/health') {
      res.setHeader('Content-Type', 'application/json');
      res.end(JSON.stringify({
        connected: socket?.readyState === WebSocket.OPEN,
        frames: timeline.frames.length,
        stats: jitter.stats,
        // Whether the shape of the system is known, separately from whether
        // telemetry is arriving: a node can be publishing frames while the
        // registry is unreadable, and the reverse.
        system: {
          received: graphs.received > 0,
          graphs: graphs.received,
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
    // The upstream carries two message families now. A telemetry frame fits in
    // one datagram, but a system graph describing every node on every topic is
    // far larger, and a payload cap below it would drop the connection instead
    // of the message. parseSystemGraph enforces the real bound.
    socket = new WebSocket(upstream, {maxPayload: MAX_SYSTEM_BYTES});
    socket.on('message', (raw, binary) => {
      // Binary is telemetry; text is the system graph. The frame type carries
      // the distinction, so neither side has to sniff the bytes.
      if (!binary) {
        ingestSystemGraph(raw.toString());
        return;
      }
      try {
        const bytes = Array.isArray(raw) ? Buffer.concat(raw) :
            raw instanceof ArrayBuffer   ? new Uint8Array(raw) :
                                           raw;
        jitter.push(decode(bytes), performance.now());
      } catch {
        jitter.stats.invalid++;
      }
    });
    socket.on('open', () => {
      // Asked for on every open: the bridge may have started after this
      // process did, and the first attempt would have had nothing to answer it.
      void fetchDeclaredGraph();
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
  return {
    server,
    timeline,
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
