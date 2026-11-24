// The telemetry worker driven end to end in node: a fake WebSocket stands in
// for the bridge, a fake clock for performance.now(), and the worker's 4 ms
// tick and reconnect timeout are captured and run by hand, so each test
// controls exactly what the worker has received and how long ago.
import assert from 'node:assert/strict';
import test from 'node:test';

import {demoDeclaredGraph, demoPacket, demoSystemGraph} from '../src/demo';
import {AlertLevel, ChannelKind, concatPages, dictionaryId, encodeDictionaryPages, encodeValuePages, rawFromDouble} from '../src/encode';
import type {AlertLatch} from '../src/data/alerts';
import {GRAPH_STALE_AFTER_MS, readingClock, sampleStale} from '../src/data/freshness';
import type {SessionManifest} from '../src/data/manifest';
import {TelemetryStore} from '../src/data/store';
import type {PlaybackState, Snapshot, SystemState} from '../src/data/types';
import {UPDATE_CAPACITY} from '../src/data/worker-stream';
import type {PiecesSnapshot} from '../src/data/pieces';
import {graphVersionMismatch, SYSTEM_GRAPH_VERSION} from '../src/system';
import {piecesMessage} from './pieces_message';

let now = 1000;
type Listener = ((event: {data: unknown}) => void)|null;
const sockets: FakeSocket[] = [];
class FakeSocket {
  binaryType = '';
  onopen: (() => void)|null = null;
  onmessage: Listener = null;
  onclose: (() => void)|null = null;
  onerror: (() => void)|null = null;
  constructor(readonly url: URL) {
    sockets.push(this);
  }
  close() {}
}

let playback: PlaybackState|null = null;
let system: SystemState|null = null;
let lastManifest: SessionManifest|null = null;
let frames: Snapshot[] = [];
let pieces: PiecesSnapshot|null = null;
let postedFrameChunks: {reset: boolean; count: number}[] = [];
// The main thread's render cache, fed as studio.ts feeds it.
const store = new TelemetryStore(cb => cb());
function apply(m: Record<string, unknown>) {
  if (m.type === 'playback') playback = m.state as PlaybackState;
  else if (m.type === 'system') system = m as unknown as SystemState;
  else if (m.type === 'manifest') {
    lastManifest = (m.manifest as SessionManifest|null) ?? null;
    if (m.manifest) store.pushManifest(m.manifest as SessionManifest);
  } else if (m.type === 'frames') {
    const chunk = m.frames as Snapshot[];
    postedFrameChunks.push({reset: m.reset as boolean, count: chunk.length});
    if (m.reset) frames = [];
    frames.push(...chunk);
    store.ingest(chunk, m.reset as boolean);
  } else if (m.type === 'alerts') {
    store.setAlertLatch(m.latch as AlertLatch);
  } else if (m.type === 'pieces') {
    pieces = m.pieces as PiecesSnapshot|null;
  }
}
let exported: ArrayBuffer|null = null;
let rpcResponse: unknown = null;
let autoAck = true;
let droppedUpdates = 0;
const ctx = {
  onmessage: null as Listener,
  postMessage(message: {type: string; messages?: Record<string, unknown>[];
                        dropped?: number; buffer?: ArrayBuffer; response?: unknown}) {
    if (message.type === 'export') exported = message.buffer!;
    if (message.type === 'rpc') rpcResponse = message.response;
    if (message.type !== 'updates') return;
    droppedUpdates = message.dropped ?? 0;
    for (const m of message.messages!) apply(m);
    if (autoAck) send({type: 'ack'});
  }
};
function send(data: object) {
  ctx.onmessage!({data});
}

Object.assign(globalThis, {self: ctx, WebSocket: FakeSocket});
Object.defineProperty(globalThis, 'performance',
                      {value: {now: () => now}, configurable: true, writable: true});
globalThis.fetch = async () => new Response('', {status: 404});
let tick: () => void = () => {};
const realSetInterval = globalThis.setInterval;
globalThis.setInterval = ((fn: () => void) => {
  tick = fn;
  return 0;
}) as unknown as typeof setInterval;
// Dynamic, not static: the worker reads `self`, WebSocket and the clock when
// its module evaluates, so the fakes above must be installed first.
await import('../src/worker');
globalThis.setInterval = realSetInterval;

// Advance the fake clock in 4 ms ticks, as the worker's interval would.
function advance(ms: number) {
  const end = now + ms;
  while (now < end) {
    now = Math.min(end, now + 4);
    tick();
  }
}
function connect() {
  send({type: 'connect', url: 'ws://127.0.0.1:5800/telemetry'});
  const socket = sockets.at(-1)!;
  socket.onopen!();
  return socket;
}
function binary(socket: FakeSocket, bytes: Uint8Array) {
  socket.onmessage!({data: bytes.slice().buffer});
}
// Runs `fn`, capturing the timeout it schedules (the worker's reconnect) so
// the test can fire it without waiting a real second.
function captureTimeout(fn: () => void): () => void {
  let captured: () => void = () => assert.fail('no timeout scheduled');
  const real = globalThis.setTimeout;
  globalThis.setTimeout = ((cb: () => void) => {
    captured = cb;
    return 0;
  }) as unknown as typeof setTimeout;
  try {
    fn();
  } finally {
    globalThis.setTimeout = real;
  }
  return captured;
}
// Whether the newest retained reading would render as current.
function latestCurrent() {
  const latest = frames.at(-1);
  return latest !== undefined && playback !== null &&
      !sampleStale(latest.timestamp_ns, readingClock(playback));
}

test('a connect-time dictionary names channels without presenting its values as current', () => {
  const socket = connect();
  // The bridge's connect-time dictionary and keyframe: its values are as old
  // as the telemetry that stopped producing them, which the client cannot see.
  binary(socket, demoPacket(100n));
  advance(40);
  assert.equal(frames.length, 1, 'channels are discovered from the connect-time dictionary');
  assert.ok(Object.keys(frames[0].units).length > 0);
  assert.equal(playback!.connection, 'live');
  assert.equal(latestCurrent(), false, 'cached values are not current');
  advance(3000);
  assert.equal(latestCurrent(), false);

  // A live update recovers immediately.
  binary(socket, demoPacket(101n));
  advance(40);
  assert.equal(frames.at(-1)!.sequence_id, '101');
  assert.equal(latestCurrent(), true, 'a fresh update is current');
});

test('a reconnect does not revive cached values as current', () => {
  const socket = connect();
  binary(socket, demoPacket(0n));
  binary(socket, demoPacket(1n));
  advance(40);
  assert.equal(latestCurrent(), true);
  // The socket drops; the producer ran on and stopped while it was down, so
  // the bridge's connect-time keyframe is newer than anything this viewer saw.
  const retry = captureTimeout(() => socket.onclose!());
  advance(2500);
  assert.equal(latestCurrent(), false);
  retry();
  const again = sockets.at(-1)!;
  assert.notEqual(again, socket);
  again.onopen!();
  binary(again, demoPacket(100n));
  advance(40);
  assert.equal(frames.at(-1)!.sequence_id, '100');
  assert.equal(latestCurrent(), false, 'the connect-time keyframe is not current');
  binary(again, demoPacket(101n));
  advance(40);
  assert.equal(latestCurrent(), true);
});

function text(socket: FakeSocket, data: string) {
  socket.onmessage!({data});
}

test('a graph goes stale when graph messages stop, while telemetry continues', () => {
  const socket = connect();
  text(socket, demoSystemGraph(0n));
  advance(8);
  assert.ok(system!.graph);
  assert.equal(system!.stale, false);
  // Binary telemetry keeps flowing; text graphs stop.
  let sequence = 0n;
  for (let t = 0; t < GRAPH_STALE_AFTER_MS + 100; t += 20) {
    binary(socket, demoPacket(sequence++));
    advance(20);
  }
  advance(40);
  assert.equal(latestCurrent(), true, 'telemetry is still current');
  assert.equal(system!.stale, true, 'the graph is not');
  assert.ok(system!.graph!.nodes.length > 0, 'the last-known graph is kept');
  // A malformed replacement is no evidence about what runs now.
  text(socket, '{"not": "a graph"}');
  advance(8);
  assert.ok(system!.error);
  assert.equal(system!.stale, true);
  // A good graph recovers immediately.
  text(socket, demoSystemGraph(sequence));
  advance(8);
  assert.equal(system!.stale, false);
  assert.equal(system!.error, '');
});

test('a graph goes stale the moment the socket drops', () => {
  const socket = connect();
  text(socket, demoSystemGraph(0n));
  advance(8);
  assert.equal(system!.stale, false);
  captureTimeout(() => socket.onclose!());
  advance(8);
  assert.equal(system!.stale, true);
  assert.ok(system!.graph, 'last-known graph stays inspectable');
});

// REQ-OPS-23: a bridge of another build whose graph version is not the
// page's is named, and the page shows no graph, so no node reads as live,
// stopped or stale on evidence it cannot read (system.test.ts holds the
// stream to both directions).
test('a graph of another version is named, not shown', () => {
  const socket = connect();
  text(socket, demoSystemGraph(0n));
  advance(8);
  assert.ok(system!.graph);
  const newer = {...JSON.parse(demoSystemGraph(1n)), version: SYSTEM_GRAPH_VERSION + 1};
  text(socket, JSON.stringify(newer));
  advance(8);
  assert.equal(system!.graph, null, 'no liveness from a graph the page cannot read');
  assert.equal(system!.stale, false, 'version mismatch must not mark the graph stale');
  assert.deepEqual(graphVersionMismatch(system!.error),
                   {bridge: SYSTEM_GRAPH_VERSION + 1, page: SYSTEM_GRAPH_VERSION});
  // The Agent tab answers as the headless agent does.
  send({type: 'rpc', id: 1,
        request: {jsonrpc: '2.0', id: 1, method: 'get_system_graph'}});
  const answer = rpcResponse as {error: {code: number; data: unknown}};
  assert.equal(answer.error.code, -32003);
  assert.deepEqual(answer.error.data, {bridge_version: SYSTEM_GRAPH_VERSION + 1,
                                       reader_version: SYSTEM_GRAPH_VERSION});
  text(socket, demoSystemGraph(2n));
  advance(8);
  assert.ok(system!.graph);
  assert.equal(system!.error, '');
});

// REQ-OPS-23: the mismatch is a statement about the bridge on this socket.
// Once it closes the page is disconnected, not still hearing that bridge.
test('a closed socket no longer names the last bridge\'s graph version', () => {
  const socket = connect();
  text(socket, JSON.stringify(
      {...JSON.parse(demoSystemGraph(0n)), version: SYSTEM_GRAPH_VERSION + 1}));
  advance(8);
  assert.ok(graphVersionMismatch(system!.error));
  captureTimeout(() => socket.onclose!());
  advance(8);
  assert.equal(playback!.connection, 'connecting');
  assert.equal(graphVersionMismatch(system!.error), null);
});

// An export from an older telemetry format is refused by name, not decoded
// into an unhelpful page error (studio/README.md: export compatibility).
test('an export of an older telemetry format is refused by name', () => {
  const old = new Uint8Array(32);
  new DataView(old.buffer).setUint32(0, old.length - 4, true);
  old.set([...'TLM2'].map(c => c.charCodeAt(0)), 8);
  send({type: 'import', buffer: old.buffer});
  advance(40);
  assert.match(playback!.status, /^Error: .*TLM2/);
  assert.match(playback!.status, /TLM3/);
});

// One telemetry message of a producer with the given stream id and channels
// (each a double), a dictionary first when `dictionary` is set.
function packet(stream: bigint, channels: string[], sequence: bigint,
                timestamp_ns: bigint, values: number[], dictionary = false) {
  const specs = channels.map(name => ({name, unit: 'm', kind: ChannelKind.Double}));
  const header = {streamId: stream, sequenceId: sequence, timestampNs: timestamp_ns,
                  dictionaryId: dictionaryId(specs), channelCount: channels.length};
  const pages = dictionary ? encodeDictionaryPages(header, specs) : [];
  pages.push(...encodeValuePages(
      header, {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0}, true,
      values.map((value, i): [number, bigint] => [i, rawFromDouble(value)])));
  return concatPages(pages);
}
// The demo graph with its telemetry process in launcher session `session`.
function sessionGraph(session: string) {
  const graph = JSON.parse(demoSystemGraph(0n));
  for (const node of graph.nodes) node.session_id = session;
  return JSON.stringify(graph);
}
function sessionManifest(session: string, telemetryRestarts: number) {
  return JSON.stringify({
    session_id: session, config_path: '/robot.toml', config_sha256: 'abc',
    output_dir: '/tmp/session', simulation: true, start_wall_ns: '0',
    end_wall_ns: '0',
    nodes: [{name: 'telemetry', target: '//talOS/telemetry:node', binary_path: 'b',
             log_path: 'l', log_paths: ['l'], pid: 1, exit_code: -1,
             restart_count: telemetryRestarts, record_count: 0}]
  });
}
const settle = () => new Promise(resolve => setImmediate(resolve));

test('a producer restart within a session keeps history, keyed by channel name', async () => {
  let restarts = 0;
  globalThis.fetch = async (url: string|URL|Request) =>
      String(url).endsWith('/manifest.json') ?
      new Response(sessionManifest('5', restarts)) :
      new Response('', {status: 404});
  try {
    const socket = connect();
    await settle();
    text(socket, sessionGraph('5'));
    binary(socket, packet(11n, ['x'], 1n, 1_000_000_000n, [1], true));
    binary(socket, packet(11n, ['x'], 2n, 1_020_000_000n, [2]));
    advance(40);
    // The telemetry node restarts: a new stream whose sequence starts over and
    // whose dictionary puts `x` at another channel index.
    restarts = 1;
    binary(socket, packet(12n, ['z', 'x'], 1n, 3_000_000_000n, [9, 3], true));
    advance(40);
    text(socket, sessionGraph('5'));
    advance(1000);
    await settle();
    advance(8);

    assert.deepEqual(frames.map(f => [f.stream_id, f.channels.x]),
                     [['11', 1], ['11', 2], ['12', 3]]);
    assert.equal(playback!.bounds!.start, '1000000000', 'the timeline continues');
    // The plotted series breaks between the streams.
    assert.deepEqual([...store.series(['x'], 'all', '3000000000').values.x],
                     [1, 2, NaN, 3]);
    const events = store.events();
    assert.ok(events.some(e => e.category === 'stream'), 'the boundary is marked');
    assert.ok(events.some(e => e.id.startsWith('manifest-node:')),
              'the manifest baseline from before the restart survives');

    // An export carries each stream's dictionary, so it imports whole.
    send({type: 'export'});
    send({type: 'import', buffer: exported});
    advance(8);
    assert.equal(playback!.status, 'Replay log loaded');
    assert.deepEqual(frames.map(f => [f.stream_id, f.channels.x]),
                     [['11', 1], ['11', 2], ['12', 3]]);
  } finally {
    globalThis.fetch = async () => new Response('', {status: 404});
  }
});

test('a new launcher session replaces the history', () => {
  // The new session's producer started before its graph arrived: only its
  // stream is kept.
  let socket = connect();
  text(socket, sessionGraph('5'));
  binary(socket, packet(11n, ['x'], 1n, 1_000_000_000n, [1], true));
  advance(40);
  binary(socket, packet(12n, ['x'], 1n, 2_000_000_000n, [2], true));
  advance(40);
  text(socket, sessionGraph('6'));
  advance(40);
  assert.deepEqual(frames.map(f => f.stream_id), ['12']);

  // The new session is seen while the old session's stream is still the
  // current one: nothing of the old session is kept.
  socket = connect();
  text(socket, sessionGraph('5'));
  binary(socket, packet(11n, ['x'], 1n, 1_000_000_000n, [1], true));
  advance(40);
  text(socket, sessionGraph('6'));
  advance(40);
  // Until it re-attaches, the bridge keeps repeating the retired stream's
  // last tick; it is not the new session's.
  binary(socket, packet(11n, ['x'], 1n, 1_000_000_000n, [1]));
  advance(40);
  assert.equal(frames.length, 0);
  assert.equal(store.series(['x'], 'all', '3000000000').times.length, 0);
  binary(socket, packet(12n, ['x'], 1n, 2_000_000_000n, [2], true));
  advance(40);
  assert.deepEqual(frames.map(f => f.stream_id), ['12']);
});

// REQ-OPS-23: reconnecting resets TelemetryDecoder so stale delta state from
// the prior socket never leaks onto the new connection before a fresh
// dictionary and keyframe arrive.
test('a reconnect resets the decoder so stale delta state never leaks', () => {
  const socket = connect();
  const specs = [
    {name: 'a', unit: 'm', kind: ChannelKind.Double},
    {name: 'b', unit: 'm', kind: ChannelKind.Double}
  ];
  const dictId = dictionaryId(specs);
  const chassis = {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0};
  const header = (seq: bigint, ts: bigint) => ({
    streamId: 55n, sequenceId: seq, timestampNs: ts, dictionaryId: dictId,
    channelCount: specs.length
  });
  binary(socket, concatPages([
    ...encodeDictionaryPages(header(1n, 1_000_000_000n), specs),
    ...encodeValuePages(header(1n, 1_000_000_000n), chassis, true,
                        [[0, rawFromDouble(10)], [1, rawFromDouble(20)]])
  ]));
  advance(40);
  assert.equal(frames.at(-1)!.channels.a, 10);
  const invalidBefore = playback!.stats.invalid;

  const retry = captureTimeout(() => socket.onclose!());
  retry();
  const again = sockets.at(-1)!;
  again.onopen!();

  // A delta update without a new dictionary/keyframe on the new socket must be
  // rejected rather than decoded against the previous connection's cached state.
  binary(again, concatPages(encodeValuePages(
      header(2n, 2_000_000_000n), chassis, false, [[1, rawFromDouble(99)]])));
  advance(40);
  assert.equal(frames.at(-1)!.sequence_id, '1',
               'delta before reconnect dictionary/keyframe must not be accepted');
  assert.equal(playback!.stats.invalid, invalidBefore + 1);
});

// REQ-OPS-37: when a stalled main thread withholds `ack` long enough for
// WorkerUpdateStream to drop buffered frames past UPDATE_CAPACITY, alert
// activations (and match transitions) in the dropped frames must still appear
// in the store's alert latch once the main thread resumes.
test('alert activations in frames dropped by WorkerUpdateStream stay latched', () => {
  const ALERT = 'hw/status/alert_error_link_lost';
  const DS_ENABLED = 'hw/state/driver_station/enabled';
  const DS_MATCH = 'hw/state/driver_station/match_number';
  const DS_TYPE = 'hw/state/driver_station/match_type';
  const specs = [
    {name: ALERT, unit: '', kind: ChannelKind.Bool, alert: AlertLevel.Error},
    {name: DS_ENABLED, unit: '', kind: ChannelKind.Bool},
    {name: DS_MATCH, unit: '', kind: ChannelKind.UInt},
    {name: DS_TYPE, unit: '', kind: ChannelKind.UInt}
  ];
  const dictId = dictionaryId(specs);
  const chassis = {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0};
  const head = (seq: bigint, ts: bigint) => ({
    streamId: 77n, sequenceId: seq, timestampNs: ts, dictionaryId: dictId,
    channelCount: specs.length
  });
  const socket = connect();
  // Initial dictionary + keyframe delivered and acknowledged so WorkerUpdateStream
  // is ready, then withhold ack on the next flush to simulate a stalled UI.
  binary(socket, concatPages([
    ...encodeDictionaryPages(head(1n, 1_000_000n), specs),
    ...encodeValuePages(head(1n, 1_000_000n), chassis, true,
                        [[0, 0n], [1, 0n], [2, 1n], [3, 2n]])
  ]));
  advance(20);
  assert.equal(store.alertLatch().alerts.length, 0);

  autoAck = false;
  const droppedBefore = droppedUpdates;
  try {
    // Frame 2 flushes into the in-flight batch (unacknowledged), locking
    // WorkerUpdateStream so subsequent frames queue in its bounded ring.
    binary(socket, concatPages(encodeValuePages(
        head(2n, 2_000_000n), chassis, false, [])));
    advance(20);

    // Match 2 begins on frame 3; alert activates on frame 4 and clears on frame 5.
    // All three will be evicted from WorkerUpdateStream when > UPDATE_CAPACITY
    // newer frames arrive before the UI acknowledges.
    binary(socket, concatPages(encodeValuePages(
        head(3n, 3_000_000n), chassis, false, [[2, 2n]])));
    binary(socket, concatPages(encodeValuePages(
        head(4n, 4_000_000n), chassis, false, [[0, 1n]])));
    binary(socket, concatPages(encodeValuePages(
        head(5n, 5_000_000n), chassis, false, [[0, 0n]])));
    advance(20);

    // Flood UPDATE_CAPACITY + 20 empty delta updates with strictly increasing
    // sequence_id and timestamp_ns. Pre-encode one template and patch its two
    // uint64 header fields directly so pushing 30k frames stays fast.
    const seqSentinel = 0x1122334455667788n;
    const tsSentinel = 0x2233445566778899n;
    const template = encodeValuePages(head(seqSentinel, tsSentinel), chassis, false, [])[0];
    const view = new DataView(template.buffer, template.byteOffset, template.byteLength);
    let seqOffset = -1, tsOffset = -1;
    for (let i = 0; i <= template.byteLength - 8; i++) {
      const v = view.getBigUint64(i, true);
      if (v === seqSentinel) seqOffset = i;
      if (v === tsSentinel) tsOffset = i;
    }
    assert.ok(seqOffset >= 0 && tsOffset >= 0, 'failed to locate header fields in template');
    const total = UPDATE_CAPACITY + 25;
    for (let i = 6; i <= total; i++) {
      view.setBigUint64(seqOffset, BigInt(i), true);
      view.setBigUint64(tsOffset, BigInt(i) * 1_000_000n, true);
      binary(socket, template);
      if (i % 2000 === 0) advance(12);
    }
    advance(20);
  } finally {
    autoAck = true;
  }

  // Resume UI acknowledgements and drain the worker's remaining batches.
  send({type: 'ack'});
  for (let i = 0; i < 20; i++) advance(4);

  assert.ok(droppedUpdates > droppedBefore, 'WorkerUpdateStream must have dropped frames');
  assert.ok(!frames.some(f => f.sequence_id === '4'),
            'frame 4 (the alert activation) must have been dropped before reaching store.ingest');
  const latch = store.alertLatch();
  assert.equal(latch.matchKey, '2:2', 'match transition in dropped frames must be preserved');
  assert.equal(latch.matchStart_ns, '3000000');
  assert.equal(latch.alerts.length, 1, 'alert activation in dropped frames must stay latched');
  assert.equal(latch.alerts[0].channel, ALERT);
  assert.equal(latch.alerts[0].first_ns, '4000000');
  assert.equal(latch.alerts[0].last_ns, '4000000');
  assert.equal(latch.alerts[0].count, 1);
  assert.equal(latch.alerts[0].active, false);

  // Starting a new connection resets the worker's latch cleanly.
  connect();
  advance(8);
  assert.equal(store.alertLatch().alerts.length, 0);
  assert.equal(store.alertLatch().matchKey, null);
});

// REQ-DEV-43
test('the bridge\'s game pieces reach the page beside telemetry, and a new connection clears them', () => {
  const socket = connect();
  binary(socket, demoPacket(0n));
  binary(socket, piecesMessage(true, 2, 0.075, 5n, [
    {index: 0, in_play: true, x_mm: 1000, y_mm: 2000, z_mm: 75},
    {index: 1, in_play: true, x_mm: 3000, y_mm: 4000, z_mm: 75}]));
  binary(socket, piecesMessage(false, 2, 0.075, 6n, [
    {index: 1, in_play: false, x_mm: 3001, y_mm: 4000, z_mm: 75}]));
  binary(socket, demoPacket(1n));
  advance(40);
  assert.equal(playback!.stats.invalid, 0, 'a pieces message is not a bad telemetry page');
  assert.equal(frames.at(-1)!.sequence_id, '1');
  assert.equal(pieces!.sample_time_ns, '6');
  assert.deepEqual([...pieces!.in_play], [1, 0]);
  assert.ok(Math.abs(pieces!.positions[3] - 3.001) < 1e-6);

  // Another bridge: no pieces until its keyframe.
  connect();
  advance(40);
  assert.equal(pieces, null, 'a new connection starts with no pieces');
});

// REQ-OPS-37: multi-chunk postFrames() (> 2000 frames after a reset, e.g. log
// import) sends reset: true only on chunk 1 and reset: false on subsequent
// chunks so the store retains every imported frame in order.
test('postFrames chunks > 2000 reset frames with reset:true on chunk 1 and reset:false on chunk 2', () => {
  const specs = [{name: 'x', unit: 'm', kind: ChannelKind.Double}];
  const dictId = dictionaryId(specs);
  const chassis = {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0};
  const total = 2500;
  const pages: Uint8Array[] = encodeDictionaryPages(
      {streamId: 91n, sequenceId: 1n, timestampNs: 1_000_000n,
       dictionaryId: dictId, channelCount: 1},
      specs);
  for (let i = 1; i <= total; i++) {
    pages.push(...encodeValuePages(
        {streamId: 91n, sequenceId: BigInt(i), timestampNs: BigInt(i) * 1_000_000n,
         dictionaryId: dictId, channelCount: 1},
        chassis, i === 1, [[0, rawFromDouble(i)]]));
  }
  const log = concatPages(pages);

  postedFrameChunks = [];
  send({type: 'import', buffer: log.slice().buffer});
  // Tick twice so WorkerUpdateStream flushes both chunks (each chunk <= 2000).
  advance(4);
  advance(4);

  const nonEmpty = postedFrameChunks.filter(c => c.count > 0 || c.reset);
  assert.deepEqual(nonEmpty, [
    {reset: true, count: 2000},
    {reset: false, count: 500}
  ]);
  assert.equal(frames.length, total);
  assert.equal(frames[0].sequence_id, '1');
  assert.equal(frames[total - 1].sequence_id, String(total));
  assert.equal(store.series(['x'], 'all', String(total * 1_000_000)).times.length, total);
});

// REQ-OPS-23: a pending /manifest.json or /declared.json fetch from a prior
// WebSocket open that settles after an automatic reconnect must not clear
// manifestInFlight for the new fetch or overwrite newer manifest/declared state.
test('stale manifest and declared fetches from a prior reconnect never clear manifestInFlight or overwrite state', async () => {
  type PendingFetch = {
    url: string;
    signal?: AbortSignal;
    resolve: (res: Response) => void;
    reject: (err: unknown) => void;
  };
  const manifestCalls: PendingFetch[] = [];
  const declaredCalls: PendingFetch[] = [];
  globalThis.fetch = ((url: string|URL|Request, init?: RequestInit) =>
      new Promise<Response>((resolve, reject) => {
        const entry: PendingFetch = {url: String(url), signal: init?.signal ?? undefined, resolve, reject};
        if (entry.url.endsWith('/manifest.json')) manifestCalls.push(entry);
        else if (entry.url.endsWith('/declared.json')) declaredCalls.push(entry);
        else resolve(new Response('', {status: 404}));
      })) as typeof fetch;

  const declaredWithBus = (bus: string) =>
      JSON.stringify({...JSON.parse(demoDeclaredGraph()), can_buses: ['rio', bus]});

  try {
    const socket1 = connect();
    assert.equal(manifestCalls.length, 1);
    assert.equal(declaredCalls.length, 1);

    // Automatic reconnect (same `generation`, bumped `openGeneration`).
    const retry = captureTimeout(() => socket1.onclose!());
    retry();
    const socket2 = sockets.at(-1)!;
    assert.notEqual(socket2, socket1);
    socket2.onopen!();

    assert.equal(manifestCalls[0].signal?.aborted, true);
    assert.equal(declaredCalls[0].signal?.aborted, true);
    assert.equal(manifestCalls.length, 2);
    assert.equal(declaredCalls.length, 2);

    // Fetch 1 settles AFTER fetch 2 starts (simulating both an aborted fetch
    // hitting finally and a late-resolving response that ignored AbortSignal).
    manifestCalls[0].resolve(new Response(sessionManifest('111', 0)));
    declaredCalls[0].resolve(new Response(declaredWithBus('StaleBus')));
    await settle();
    advance(8);

    // Fetch 1's response must be ignored (never overwrites state).
    assert.notEqual(lastManifest?.session_id, '111');
    assert.notEqual(system?.declared?.can_buses?.[1], 'StaleBus');

    // Advancing past the 1 Hz manifest poll interval while fetch 2 is still in
    // flight must NOT duplicate fetch 2 (fetch 1's finally did not clear
    // manifestInFlight).
    advance(1500);
    assert.equal(manifestCalls.length, 2,
                 'stale fetch 1 cleared manifestInFlight and duplicated fetch 2');

    // Now fetch 2 resolves cleanly and applies its state.
    manifestCalls[1].resolve(new Response(sessionManifest('222', 0)));
    declaredCalls[1].resolve(new Response(declaredWithBus('FreshBus')));
    await settle();
    advance(8);
    assert.equal(lastManifest?.session_id, '222');
    assert.deepEqual(system?.declared?.can_buses, ['rio', 'FreshBus']);

    // Once fetch 2 finishes, the next 1 Hz poll starts fetch 3 normally.
    advance(1100);
    assert.equal(manifestCalls.length, 3);
  } finally {
    globalThis.fetch = async () => new Response('', {status: 404});
  }
});
