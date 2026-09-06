/// <reference lib="webworker" />
import {handleRpc} from '../agent/rpc';

import {Coordinator, decode, JitterBuffer, Timeline} from './core';
import {demoDeclaredGraph, demoPacket, demoSystemGraph} from './demo';
import {type DeclaredGraph, declaredUrl, MAX_DECLARED_BYTES, parseDeclaredGraph, parseSystemGraph, RateWindow, SystemAssembler, systemChunk, type SystemGraph} from './system';

const ctx = self as unknown as DedicatedWorkerGlobalScope;
let coordinator = new Coordinator(), jitter = new JitterBuffer(10);
let socket: WebSocket|null = null, demo = false, sequence = 0n,
            status = 'Disconnected';
let packets = new Map<string, Uint8Array>(),
    pendingBytes = new Map<string, Uint8Array>();
let nextDemo = 0, lastTick = performance.now(), lastView = 0;
let retry: ReturnType<typeof setTimeout>|undefined;
let generation = 0;

// The registry reports cumulative counters, not rates, so a rate needs an
// older sample to measure against. `previousSystem` is the one immediately
// before this graph, which the system view uses to show what changed;
// `rateWindow` holds enough of them to measure a rate over a window wide
// enough not to quantise it -- see RateWindow.
let system: SystemGraph|null = null, previousSystem: SystemGraph|null = null;
const rateWindow = new RateWindow();
let systemError = '';
let nextDemoSystem = 0;

// The declared graph, fetched over HTTP rather than pushed: the launcher writes
// it once before it spawns anything, so it does not change while a session
// runs, and asking for it once per connection costs less than carrying it on a
// stream that refreshes four times a second.
let declared: DeclaredGraph|null = null;
let declaredError = '';

// Reassembles graphs arriving over UDP, which the platform's maximum datagram
// forces the bridge to split. The WebSocket path needs none of this: a text
// frame carries the whole document.
const assembler = new SystemAssembler();

// Posted only when the graph changes, not on every telemetry tick: it is a few
// tens of kilobytes and structure changes at 4 Hz, not 30.
function postSystem() {
  const rates: Record<string, number> = {};
  for (const [key, value] of rateWindow.rates()) {
    rates[key] = value;
  }
  ctx.postMessage({
    type: 'system',
    graph: system,
    rates,
    error: systemError,
    declared,
    declaredError
  });
}

// GET /declared.json from the same origin as the telemetry socket.
//
// A 404 is the ordinary answer: `--declared` is optional, and a bridge started
// without it says so this way. That is not an error to report -- there is
// nothing wrong and nothing for a person to do -- so it leaves the viewer
// exactly as it was before there was a declared graph to ask for. A network
// failure is equally quiet, because the connection status already says the
// bridge is unreachable. A document that arrives and does not parse is the one
// case worth a sentence on screen.
async function fetchDeclared(url: string, gen: number) {
  const target = declaredUrl(url);
  if (!target) return;
  let text: string;
  try {
    const response = await fetch(target, {cache: 'no-store'});
    if (gen !== generation) return;
    if (response.status === 404) return;
    if (!response.ok) {
      declaredError = `Declared graph unavailable: HTTP ${response.status}`;
      postSystem();
      return;
    }
    // Advisory, and checked before the body is read rather than after: the
    // parser enforces the real bound, but there is no reason to hold a
    // gigabyte in memory first to find out it was too big.
    const declaredSize = Number(response.headers.get('content-length') ?? 0);
    if (declaredSize > MAX_DECLARED_BYTES) {
      declaredError = 'Invalid declared graph: document too large';
      postSystem();
      return;
    }
    text = await response.text();
  } catch {
    return;
  }
  if (gen !== generation) return;
  ingestDeclared(text);
  postSystem();
}

// One malformed document must not blank a working view, the same rule the live
// graph follows: keep the last good declared graph and say why this one was
// refused.
function ingestDeclared(text: string) {
  try {
    declared = parseDeclaredGraph(text);
    declaredError = '';
  } catch (e) {
    declaredError = String(e);
  }
}

function ingestSystem(text: string) {
  try {
    const graph = parseSystemGraph(text);
    previousSystem = system;
    system = graph;
    rateWindow.push(graph);
    systemError = '';
  } catch (e) {
    // One malformed document must not blank a working view. Keep the last good
    // graph and say why the new one was refused.
    systemError = String(e);
  }
  postSystem();
}
function reset() {
  coordinator = new Coordinator();
  jitter = new JitterBuffer(10);
  packets.clear();
  pendingBytes.clear();
  // Whatever is being connected to next is a different system, so the old
  // topology is not evidence about it.
  system = null;
  previousSystem = null;
  rateWindow.reset();
  systemError = '';
  declared = null;
  declaredError = '';
  postSystem();
}
function close() {
  generation++;
  clearTimeout(retry);
  if (socket) {
    socket.onclose = null;
    socket.close();
    socket = null;
  }
  demo = false;
}
function ingest(bytes: Uint8Array, now: number) {
  try {
    const frame = decode(bytes);
    jitter.push(frame, now);
    pendingBytes.set(frame.sequence_id, bytes);
    if (pendingBytes.size > 4096)
      pendingBytes.delete(pendingBytes.keys().next().value!);
  } catch {
    jitter.stats.invalid++;
  }
}
function connect(url: string, gen: number) {
  try {
    const u = new URL(url);
    if (!['ws:', 'wss:'].includes(u.protocol))
      throw Error('Use ws:// or wss://');
    socket = new WebSocket(u);
    socket.binaryType = 'arraybuffer';
    status = 'Connecting';
    socket.onopen = () => {
      status = 'WebSocket connected';
      // Asked for on every open, not once per session: the bridge may have been
      // started after Studio was, and the first attempt would have found
      // nothing to answer it.
      void fetchDeclared(url, gen);
    };
    socket.onmessage = e => {
      // Binary is telemetry, text is the system graph. The bridge picks the
      // frame type, so neither side has to sniff the payload.
      if (e.data instanceof ArrayBuffer)
        ingest(new Uint8Array(e.data), performance.now());
      else if (typeof e.data === 'string')
        ingestSystem(e.data);
    };
    socket.onclose = () => {
      status = 'Disconnected · retrying';
      if (gen === generation) retry = setTimeout(() => connect(url, gen), 1000);
    };
    socket.onerror = () => {
      status = 'WebSocket connection failed';
    };
  } catch (e) {
    status = String(e);
  }
}
ctx.onmessage = ({data}) => {
  try {
    switch (data.type) {
      case 'connect':
        close();
        reset();
        connect(data.url, generation);
        break;
      case 'demo':
        close();
        reset();
        demo = true;
        sequence = 0n;
        nextDemo = performance.now();
        nextDemoSystem = performance.now() + 250;
        // The demo's declared graph names one node the live graph does not, so
        // the declared-against-actual views are explorable with no robot and no
        // launcher attached.
        ingestDeclared(demoDeclaredGraph());
        ingestSystem(demoSystemGraph(sequence));
        status = 'Deterministic demo · 50 Hz';
        break;
      case 'packet': {
        const bytes = new Uint8Array(data.buffer);
        // Recognised before anything else: a system graph is not a frame, and
        // it is not a reason to tear down the transport already running. It
        // also spans several datagrams, so a chunk that does not complete a
        // document still must not fall through to the telemetry decoder.
        if (systemChunk(bytes)) {
          const graph = assembler.push(bytes);
          if (graph !== null) ingestSystem(graph);
          break;
        }
        if (demo || socket) {
          close();
          reset();
        }
        status = 'Desktop UDP';
        ingest(bytes, performance.now());
        break;
      }
      case 'rpc': {
        // The same query surface the headless JSON-RPC server exposes, run
        // against this worker's own timeline, so the Agent tab shows what an
        // agent would actually get back rather than a second implementation.
        let response: unknown;
        try {
          response = handleRpc(
              {timeline: coordinator.timeline, system, previousSystem, declared},
              data.request);
        } catch (e) {
          response = {
            jsonrpc: '2.0',
            id: data.request?.id ?? null,
            error: {code: -32603, message: String(e)}
          };
        }
        // Posted even when undefined -- a notification has no response, and a
        // caller waiting on this id still has to be released.
        ctx.postMessage({type: 'rpc', id: data.id, response});
        break;
      }
      case 'mode':
        if (['PAUSED', 'LIVE_STREAMING', 'REPLAY_PLAYING'].includes(data.mode))
          coordinator.mode = data.mode;
        break;
      case 'seek':
        coordinator.seek(BigInt(data.timestamp_ns));
        break;
      case 'step':
        coordinator.step(BigInt(data.dt_ns));
        break;
      case 'export': {
        const chunks =
            coordinator.timeline.frames.map(f => packets.get(f.sequence_id))
                .filter((x): x is Uint8Array => !!x);
        const out = new Uint8Array(chunks.reduce((n, c) => n + c.length, 0));
        let p = 0;
        for (const c of chunks) {
          out.set(c, p);
          p += c.length;
        }
        ctx.postMessage({type: 'export', buffer: out.buffer}, [out.buffer]);
        break;
      }
      case 'import': {
        const bytes = new Uint8Array(data.buffer);
        if (bytes.length > 256 * 1024 * 1024)
          throw Error('Log exceeds 256 MiB');
        const imported = new Timeline();
        const raw = new Map<string, Uint8Array>();
        let p = 0;
        while (p < bytes.length) {
          if (p + 4 > bytes.length) throw Error('Truncated log');
          const size = new DataView(bytes.buffer, bytes.byteOffset + p, 4)
                           .getUint32(0, true) +
              4;
          if (size < 16 || p + size > bytes.length)
            throw Error('Truncated log frame');
          const chunk = bytes.slice(p, p + size);
          const f = decode(chunk);
          if (!imported.append(f))
            throw Error('Log sequence/timestamps must strictly increase');
          raw.set(f.sequence_id, chunk);
          p += size;
        }
        // Commit only after the entire log validates. Deterministic import
        // ignores arrival time.
        close();
        reset();
        coordinator = new Coordinator(imported);
        packets = raw;
        coordinator.seek(BigInt(imported.bounds()?.start ?? '0'));
        status = 'Replay log loaded';
        break;
      }
    }
  } catch (e) {
    status = String(e);
  }
};
setInterval(() => {
  const now = performance.now();
  if (demo) {
    let n = 0;
    while (now >= nextDemo && n++ < 10) {
      ingest(demoPacket(sequence++), now);
      nextDemo += 20;
    }
    // The bridge refreshes the graph four times a second; matching that keeps
    // the demo's rate columns populated instead of permanently unmeasured.
    if (now >= nextDemoSystem) {
      ingestSystem(demoSystemGraph(sequence));
      nextDemoSystem = now + 250;
    }
  }
  for (const f of jitter.flush(now)) {
    if (!coordinator.timeline.append(f)) {
      jitter.stats.invalid++;
      continue;
    }
    const raw = pendingBytes.get(f.sequence_id);
    if (raw) packets.set(f.sequence_id, raw);
    pendingBytes.delete(f.sequence_id);
  }
  const first = coordinator.timeline.frames[0];
  if (first) {
    for (const key of packets.keys()) {
      if (BigInt(key) < BigInt(first.sequence_id))
        packets.delete(key);
      else
        break;
    }
  }
  coordinator.tick(BigInt(Math.max(0, Math.round((now - lastTick) * 1e6))));
  lastTick = now;
  if (now - lastView >= 1000 / 30) {
    lastView = now;
    const snapshot = coordinator.snapshot();
    const t = BigInt(snapshot?.timestamp_ns ?? '0');
    const frames = coordinator.timeline.frames;
    let end = frames.length;
    while (end > 0 && BigInt(frames[end - 1].timestamp_ns) > t) end--;
    ctx.postMessage({
      type: 'view',
      snapshot,
      history: frames.slice(Math.max(0, end - 600), end),
      mode: coordinator.mode,
      bounds: coordinator.timeline.bounds(),
      stats: jitter.stats,
      status
    });
  }
}, 4);
