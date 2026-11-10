/// <reference lib="webworker" />
import {handleRpc} from '../agent/rpc';

import {concatPages} from './encode';
import {checkExportFormat, Coordinator, type Dictionary, exportPages, JitterBuffer, TelemetryDecoder, Timeline, type Snapshot} from './core';
import {graphStale} from './data/freshness';
import {MAX_MANIFEST_BYTES, manifestUrl, parseManifest, type SessionManifest} from './data/manifest';
import {type Connection, type PlaybackState} from './data/types';
import {WorkerUpdateStream} from './data/worker-stream';
import {demoDeclaredGraph, demoPacket, demoSystemGraph} from './demo';
import {type DeclaredGraph, declaredUrl, MAX_DECLARED_BYTES, parseDeclaredGraph, RateWindow, type SystemGraph, SystemGraphStream} from './system';

const ctx = self as unknown as DedicatedWorkerGlobalScope;
const updates = new WorkerUpdateStream(message => ctx.postMessage(message));
// When the newest timeline sample arrived, the anchor for live session time.
// Null when its age is unknown: then no live reading is current.
let sampleArrival: number|null = null;
// Sequence ids of snapshots whose age is unknown: the keyframe the bridge
// sends on every open carries the latest tick it holds -- possibly from long
// before this connection, if telemetry has stopped -- so its arrival time
// says nothing about its values' age.
const unknownAge = new Set<string>();
let firstUpdate = false;
let coordinator = new Coordinator(), jitter = new JitterBuffer(10);
let decoder = new TelemetryDecoder();
let streamId: string|null = null;
let socket: WebSocket|null = null, demo = false, sequence = 0n,
            status = 'Disconnected';
// The dictionary the timeline decodes against, for log export.
let dictionary: Dictionary|null = null;
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
// The graph as the bridge's whole documents and updates build it, and the
// monotonic receipt time of its last message (a graph, an update, or a notice
// that one is deferred by the radio budget). The graph carries the
// registry's wall clock, which is not this machine's, so its age is measured
// from local arrival.
const graphStream = new SystemGraphStream();
let systemArrival: number|null = null;
let postedSystemStale = true;
// A held graph is only current while the source that sent it is.
const graphConnected = () => connection === 'live' || connection === 'demo';
function systemStale(now: number) {
  return graphStale(systemArrival, now, graphConnected());
}

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

// Posted only when the graph changes or goes stale/fresh, not on every
// telemetry tick: it is a few tens of kilobytes and structure changes at 4 Hz,
// not at the 100 Hz frame rate.
function postSystem() {
  const rates: Record<string, number> = {};
  for (const [key, value] of rateWindow.rates()) {
    rates[key] = value;
  }
  postedSystemStale = systemStale(performance.now());
  updates.push({
    type: 'system',
    graph: system,
    stale: postedSystemStale,
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
        (sampleArrival === null ? null :
             (BigInt(snapshot.timestamp_ns) +
              BigInt(Math.round((now - sampleArrival) * 1e6))).toString()) :
        coordinator.cursor.toString()) : null,
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
    const graph = graphStream.ingest(text, performance.now());
    if (graph) {
      previousSystem = system;
      system = graph;
      rateWindow.push(graph);
    }
    systemError = '';
    systemArrival = graphStream.arrival;
  } catch (e) {
    // One malformed document must not blank a working view. Keep the last good
    // graph and say why the new one was refused. It is no evidence about what
    // runs now, so the last good graph keeps ageing toward stale.
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
  needsReset = true;
  pendingFrames = [];
  unknownAge.clear();
}
function reset() {
  coordinator = new Coordinator();
  jitter = new JitterBuffer(10);
  decoder = new TelemetryDecoder();
  streamId = null;
  dictionary = null;
  needsReset = true;
  pendingFrames = [];
  unknownAge.clear();
  sampleArrival = null;
  // Whatever is being connected to next is a different system, so the old
  // topology is not evidence about it.
  system = null;
  previousSystem = null;
  rateWindow.reset();
  graphStream.reset();
  systemArrival = null;
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
// One WebSocket message: a dictionary page or an update page. A snapshot
// comes out at each update's last page.
function ingest(bytes: Uint8Array, now: number) {
  try {
    for (const frame of decoder.decodeAll(bytes)) {
      if (streamId !== null && frame.stream_id !== streamId)
        resetStream(frame.stream_id);
      else
        streamId = frame.stream_id;
      dictionary = decoder.currentDictionary();
      if (firstUpdate) unknownAge.add(frame.sequence_id);
      firstUpdate = false;
      jitter.push(frame, now);
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
      // The bridge answers an open with its graph and then, when it has
      // them, the dictionary and a keyframe: the first update on a socket.
      firstUpdate = true;
      unknownAge.clear();
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
      if (e.data instanceof ArrayBuffer) {
        ingest(new Uint8Array(e.data), performance.now());
      } else if (typeof e.data === 'string')
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
               rateWindow, systemArrivalMs: systemArrival,
               connected: graphConnected()},
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
        // Each retained snapshot as a keyframe after the dictionary, so the
        // log decodes on its own however much history was trimmed.
        const out = dictionary ?
            concatPages(exportPages(coordinator.timeline.frames, dictionary)) :
            new Uint8Array(0);
        ctx.postMessage({type: 'export', buffer: out.buffer}, [out.buffer]);
        break;
      }
      case 'import': {
        const bytes = new Uint8Array(data.buffer);
        if (bytes.length > 256 * 1024 * 1024)
          throw Error('Log exceeds 256 MiB');
        checkExportFormat(bytes);
        const imported = new Timeline();
        const importDecoder = new TelemetryDecoder();
        // A value page before the dictionary is refused by the decoder.
        for (const f of importDecoder.decodeAll(bytes)) {
          if (!imported.append(f))
            throw Error('Log sequence/timestamps must strictly increase');
        }
        // Commit only after the entire log validates. Deterministic import
        // ignores arrival time.
        close();
        reset();
        coordinator = new Coordinator(imported);
        dictionary = importDecoder.currentDictionary();
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
    // An arrival of 0 is Timeline's "no known arrival", so the Agent tab's
    // get_latest reports the join keyframe's age as null too.
    const unknown = unknownAge.delete(f.sequence_id);
    if (!coordinator.timeline.append(f, unknown ? 0 : Date.now())) {
      jitter.stats.invalid++;
      continue;
    }
    pendingFrames.push(f);
    sampleArrival = unknown ? null : now;
  }
  // The session manifest is re-read once a second while connected: it is a
  // live document, and the launcher replaces it atomically as nodes restart.
  if (manifestBase !== null && connection === 'live' && now >= nextManifestPoll) {
    nextManifestPoll = now + 1000;
    void fetchManifest(manifestBase, generation);
  }
  coordinator.tick(BigInt(Math.max(0, Math.round((now - lastTick) * 1e6))));
  lastTick = now;
  // A graph goes stale with time or a dropped socket, not only with a new
  // document, so its freshness is re-judged every tick and posted on change.
  if (systemStale(now) !== postedSystemStale) postSystem();
  postFrames();
  postPlayback(now);
  updates.flush();
}, 4);
