/// <reference lib="webworker" />
import {handleRpc} from '../agent/rpc';
import {ByteBuffer} from 'flatbuffers';

import {Coordinator, JitterBuffer, TelemetryDecoder, Timeline, type Snapshot} from './core';
import {MAX_MANIFEST_BYTES, manifestUrl, parseManifest, type SessionManifest} from './data/manifest';
import {type Connection, type PlaybackState} from './data/types';
import {WorkerUpdateStream} from './data/worker-stream';
import {demoDeclaredGraph, demoPacket, demoSystemGraph} from './demo';
import {Frame} from './generated/talos/telemetry/frame';
import {type DeclaredGraph, declaredUrl, MAX_DECLARED_BYTES, parseDeclaredGraph, parseSystemGraph, RateWindow, type SystemGraph} from './system';

const ctx = self as unknown as DedicatedWorkerGlobalScope;
const updates = new WorkerUpdateStream(message => ctx.postMessage(message));
let sampleArrival = performance.now();
let coordinator = new Coordinator(), jitter = new JitterBuffer(10);
let decoder = new TelemetryDecoder();
let streamId: string|null = null;
let socket: WebSocket|null = null, demo = false, sequence = 0n,
            status = 'Disconnected';
let packets = new Map<string, Uint8Array>(),
    pendingBytes = new Map<string, Uint8Array>();
let latestDictRaw: Uint8Array|null = null;
let nextDemo = 0, lastTick = performance.now();
let retry: ReturnType<typeof setTimeout>|undefined;
let generation = 0;
let connection: Connection = 'idle';
// Frames appended to the timeline since the previous `frames` post, in order.
// `needsReset` is set whenever the timeline is replaced (connect, demo, a new
// stream_id, import) so the next post carries `reset: true`.
let pendingFrames: Snapshot[] = [];
let needsReset = false;
let lastPlayback: PlaybackState|null = null;
let lastPlaybackPost = 0;

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

// The session manifest, re-polled at 1 Hz while connected: the launcher
// atomically replaces it throughout a session, so it is a live document rather
// than a startup snapshot. Absent (404) is "not offered", not an error.
let manifest: SessionManifest|null = null;
let manifestError = '';
let nextManifestPoll = 0;
let manifestBase: string|null = null;
let lastManifestRaw: string|null = null;

// Posted only when the graph changes, not on every telemetry tick: it is a few
// tens of kilobytes and structure changes at 4 Hz, not at the 100 Hz frame rate.
function postSystem() {
  const rates: Record<string, number> = {};
  for (const [key, value] of rateWindow.rates()) {
    rates[key] = value;
  }
  updates.push({
    type: 'system',
    graph: system,
    rates,
    error: systemError,
    declared,
    declaredError
  });
}

// The session manifest, posted on change: it is re-read at 1 Hz, but re-posting
// an unchanged document would re-render the Session panel for nothing.
function postManifest() {
  updates.push({type: 'manifest', manifest, error: manifestError});
}

// Every frame appended to the timeline since the previous post, in order,
// chunked so one message never carries more than 2000 frames. The first chunk
// after a timeline replacement carries `reset: true`.
function postFrames() {
  if (!needsReset && pendingFrames.length === 0) return;
  while (needsReset || pendingFrames.length > 0) {
    const chunk = pendingFrames.splice(0, 2000);
    updates.push({type: 'frames', reset: needsReset, frames: chunk});
    needsReset = false;
    if (pendingFrames.length === 0) break;
  }
}

// The playback cursor and stream status, at most every 33 ms and immediately
// on a discrete change (mode, status, connection). `connection` is derived
// from the worker's own state rather than stored on the wire.
function postPlayback(now: number) {
  const snapshot = coordinator.snapshot();
  const state: PlaybackState = {
    mode: coordinator.mode,
    cursor_ns: snapshot?.timestamp_ns ?? null,
    clock_ns: snapshot ? (coordinator.mode === 'LIVE_STREAMING' ?
        BigInt(snapshot.timestamp_ns) + BigInt(Math.round((now - sampleArrival) * 1e6)) :
        coordinator.cursor).toString() : null,
    bounds: coordinator.timeline.bounds(),
    stats: {...jitter.stats},
    status,
    connection
  };
  const discrete = !lastPlayback || lastPlayback.mode !== state.mode ||
      lastPlayback.status !== state.status ||
      lastPlayback.connection !== state.connection;
  if (discrete || now - lastPlaybackPost >= 1000 / 30) {
    updates.push({type: 'playback', state});
    lastPlayback = state;
    lastPlaybackPost = now;
  }
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

// GET /manifest.json, re-read on every request because the launcher replaces
// it throughout a session. A 404 is "not offered", not a fault: the manifest
// is null and nothing is reported. A network failure is quiet, for the same
// reason the declared graph's is. A malformed document keeps the last good
// manifest and says why this one was refused.
async function fetchManifest(url: string, gen: number) {
  const target = manifestUrl(url);
  if (!target) return;
  let text: string;
  try {
    const response = await fetch(target, {cache: 'no-store'});
    if (gen !== generation) return;
    if (response.status === 404) {
      // "Not offered" is not a fault; it clears whatever the previous bridge
      // was serving.
      if (manifest !== null || manifestError !== '') {
        manifest = null;
        manifestError = '';
        lastManifestRaw = null;
        postManifest();
      }
      return;
    }
    if (!response.ok) {
      manifestError = `Session manifest unavailable: HTTP ${response.status}`;
      postManifest();
      return;
    }
    const size = Number(response.headers.get('content-length') ?? 0);
    if (size > MAX_MANIFEST_BYTES) {
      manifestError = 'Invalid session manifest: document too large';
      postManifest();
      return;
    }
    text = await response.text();
  } catch {
    return;
  }
  if (gen !== generation) return;
  const previousError = manifestError;
  manifestError = '';
  if (text === lastManifestRaw) {
    if (previousError) postManifest();
    return;
  }
  try {
    manifest = parseManifest(text);
    lastManifestRaw = text;
    manifestError = '';
  } catch (e) {
    manifestError = String(e);
  }
  postManifest();
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
// A new stream_id means the producer restarted: sequence and timestamp
// restart with it, so the old timeline and jitter state would reject or
// misorder every frame of the new stream. Dictionaries are per-stream too.
function resetStream(next: string) {
  streamId = next;
  coordinator = new Coordinator();
  jitter = new JitterBuffer(10);
  packets.clear();
  pendingBytes.clear();
  latestDictRaw = null;
  needsReset = true;
  pendingFrames = [];
}
function reset() {
  coordinator = new Coordinator();
  jitter = new JitterBuffer(10);
  decoder = new TelemetryDecoder();
  streamId = null;
  packets.clear();
  pendingBytes.clear();
  latestDictRaw = null;
  needsReset = true;
  pendingFrames = [];
  // Whatever is being connected to next is a different system, so the old
  // topology is not evidence about it.
  system = null;
  previousSystem = null;
  rateWindow.reset();
  systemError = '';
  declared = null;
  declaredError = '';
  manifest = null;
  manifestError = '';
  manifestBase = null;
  nextManifestPoll = 0;
  lastManifestRaw = null;
  postSystem();
  postManifest();
}
function close() {
  generation++;
  clearTimeout(retry);
  if (socket) {
    socket.onopen = null;
    socket.onmessage = null;
    socket.onerror = null;
    socket.onclose = null;
    socket.close();
    socket = null;
  }
  demo = false;
}
function ingest(bytes: Uint8Array, now: number) {
  try {
    // A value frame whose dictionary has not arrived decodes to nothing yet;
    // the decoder holds it and decodeAll returns it with the dictionary frame
    // that names its channels. Held is not invalid.
    for (const {frame, raw} of decoder.decodeFrames(bytes)) {
      if (streamId !== null && frame.stream_id !== streamId)
        resetStream(frame.stream_id);
      else
        streamId = frame.stream_id;
      if (Frame.getSizePrefixedRootAsFrame(new ByteBuffer(raw))
              .dictionaryLength() > 0)
        latestDictRaw = raw;
      jitter.push(frame, now);
      pendingBytes.set(frame.sequence_id, raw);
      if (pendingBytes.size > 4096)
        pendingBytes.delete(pendingBytes.keys().next().value!);
    }
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
    connection = 'connecting';
    manifestBase = url;
    nextManifestPoll = performance.now();
    socket.onopen = () => {
      if (gen !== generation) return;
      status = 'WebSocket connected';
      connection = 'live';
      // Asked for on every open, not once per session: the bridge may have been
      // started after Studio was, and the first attempt would have found
      // nothing to answer it.
      void fetchDeclared(url, gen);
      void fetchManifest(url, gen);
    };
    socket.onmessage = e => {
      if (gen !== generation) return;
      // Binary is telemetry, text is the system graph. The bridge picks the
      // frame type, so neither side has to sniff the payload.
      if (e.data instanceof ArrayBuffer)
        ingest(new Uint8Array(e.data), performance.now());
      else if (typeof e.data === 'string')
        ingestSystem(e.data);
    };
    socket.onclose = () => {
      if (gen !== generation) return;
      status = 'Disconnected · retrying';
      connection = 'connecting';
      if (gen === generation) retry = setTimeout(() => connect(url, gen), 1000);
    };
    socket.onerror = () => {
      if (gen !== generation) return;
      status = 'WebSocket connection failed';
      connection = 'error';
    };
  } catch (e) {
    status = String(e);
    connection = 'error';
  }
}
ctx.onmessage = ({data}) => {
  try {
    switch (data.type) {
      case 'ack':
        updates.ack();
        break;
      case 'ping':
        ctx.postMessage({type: 'pong'});
        break;
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
        connection = 'demo';
        break;
      case 'rpc': {
        // The same query surface the headless JSON-RPC server exposes, run
        // against this worker's own timeline, so the Agent tab shows what an
        // agent would actually get back rather than a second implementation.
        let response: unknown;
        try {
          response = handleRpc(
              {timeline: coordinator.timeline, system, previousSystem, declared,
               rateWindow},
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
        if (chunks.length > 0 && latestDictRaw &&
            Frame.getSizePrefixedRootAsFrame(new ByteBuffer(chunks[0]))
                    .dictionaryLength() === 0)
          chunks.unshift(latestDictRaw);
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
        const importDecoder = new TelemetryDecoder();
        let importedDictRaw: Uint8Array|null = null;
        let leadingDictionary: {frame: Snapshot; raw: Uint8Array}|null = null;
        const appendImported = (f: Snapshot, frameRaw: Uint8Array) => {
          if (!imported.append(f))
            throw Error('Log sequence/timestamps must strictly increase');
          raw.set(f.sequence_id, frameRaw);
        };
        let p = 0;
        while (p < bytes.length) {
          if (p + 4 > bytes.length) throw Error('Truncated log');
          const size = new DataView(bytes.buffer, bytes.byteOffset + p, 4)
                           .getUint32(0, true) +
              4;
          if (size < 16 || p + size > bytes.length)
            throw Error('Truncated log frame');
          const chunk = bytes.slice(p, p + size);
          const decoded = importDecoder.decodeFrames(chunk);
          if (leadingDictionary && decoded.length > 0) {
            // Export may prepend a dictionary newer than the retained start.
            // It seeds decoding only; its snapshot, if retained, comes later
            // in the chronological slice.
            if (BigInt(leadingDictionary.frame.sequence_id) <
                BigInt(decoded[0].frame.sequence_id))
              appendImported(leadingDictionary.frame, leadingDictionary.raw);
            leadingDictionary = null;
          }
          for (const entry of decoded) {
            if (p === 0 &&
                Frame.getSizePrefixedRootAsFrame(new ByteBuffer(chunk))
                        .dictionaryLength() > 0)
              leadingDictionary = entry;
            else
              appendImported(entry.frame, entry.raw);
          }
          if (Frame.getSizePrefixedRootAsFrame(new ByteBuffer(chunk))
                  .dictionaryLength() > 0)
            importedDictRaw = chunk;
          p += size;
        }
        if (leadingDictionary)
          appendImported(leadingDictionary.frame, leadingDictionary.raw);
        if (importDecoder.heldCount > 0)
          throw Error('Log ends before its channel dictionary');
        const retained = new Set(imported.frames.map(f => f.sequence_id));
        for (const sequenceId of raw.keys())
          if (!retained.has(sequenceId)) raw.delete(sequenceId);
        // Commit only after the entire log validates. Deterministic import
        // ignores arrival time.
        close();
        reset();
        coordinator = new Coordinator(imported);
        packets = raw;
        latestDictRaw = importedDictRaw;
        coordinator.seek(BigInt(imported.bounds()?.start ?? '0'));
        status = 'Replay log loaded';
        connection = 'replay';
        pendingFrames = imported.frames.slice();
        needsReset = true;
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
    pendingFrames.push(f);
    sampleArrival = now;
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
  // The session manifest is re-read once a second while connected: it is a
  // live document, and the launcher replaces it atomically as nodes restart.
  if (manifestBase !== null && connection === 'live' && now >= nextManifestPoll) {
    nextManifestPoll = now + 1000;
    void fetchManifest(manifestBase, generation);
  }
  coordinator.tick(BigInt(Math.max(0, Math.round((now - lastTick) * 1e6))));
  lastTick = now;
  postFrames();
  postPlayback(now);
  updates.flush();
}, 4);
