// The telemetry worker driven end to end in node: a fake WebSocket stands in
// for the bridge, a fake clock for performance.now(), and the worker's 4 ms
// tick and reconnect timeout are captured and run by hand, so each test
// controls exactly what the worker has received and how long ago.
import assert from 'node:assert/strict';
import test from 'node:test';

import {demoPacket, demoSystemGraph} from '../src/demo';
import {GRAPH_STALE_AFTER_MS, readingClock, sampleStale} from '../src/data/freshness';
import type {PlaybackState, Snapshot, SystemState} from '../src/data/types';

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
let frames: Snapshot[] = [];
function apply(m: Record<string, unknown>) {
  if (m.type === 'playback') playback = m.state as PlaybackState;
  else if (m.type === 'system') system = m as unknown as SystemState;
  else if (m.type === 'frames') {
    if (m.reset) frames = [];
    frames.push(...(m.frames as Snapshot[]));
  }
}
const ctx = {
  onmessage: null as Listener,
  postMessage(message: {type: string; messages?: Record<string, unknown>[]}) {
    if (message.type !== 'updates') return;
    for (const m of message.messages!) apply(m);
    send({type: 'ack'});
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
