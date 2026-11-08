// The telemetry worker driven end to end in node: a fake WebSocket stands in
// for the bridge, a fake clock for performance.now(), and the worker's 4 ms
// tick and reconnect timeout are captured and run by hand, so each test
// controls exactly what the worker has received and how long ago.
import assert from 'node:assert/strict';
import test from 'node:test';

import {demoPacket} from '../src/demo';
import {readingClock, sampleStale} from '../src/data/freshness';
import type {PlaybackState, Snapshot} from '../src/data/types';

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
let frames: Snapshot[] = [];
function apply(m: Record<string, unknown>) {
  if (m.type === 'playback') playback = m.state as PlaybackState;
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
  // The bridge's cached dictionary frame: its values are as old as the
  // telemetry that stopped producing it, which the client cannot see.
  binary(socket, demoPacket(100n));
  advance(40);
  assert.equal(frames.length, 1, 'channels are discovered from the cached dictionary');
  assert.ok(Object.keys(frames[0].units).length > 0);
  assert.equal(playback!.connection, 'live');
  assert.equal(latestCurrent(), false, 'cached values are not current');
  advance(3000);
  assert.equal(latestCurrent(), false);

  // A live value frame recovers immediately.
  binary(socket, demoPacket(101n));
  advance(40);
  assert.equal(frames.at(-1)!.sequence_id, '101');
  assert.equal(latestCurrent(), true, 'a fresh value frame is current');
});

test('a reconnect does not revive cached values as current', () => {
  const socket = connect();
  binary(socket, demoPacket(0n));
  binary(socket, demoPacket(1n));
  advance(40);
  assert.equal(latestCurrent(), true);
  // The socket drops; the producer ran on and stopped while it was down, so
  // the bridge's cached dictionary is newer than anything this viewer saw.
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
  assert.equal(latestCurrent(), false, 'the cached dictionary is not current');
  binary(again, demoPacket(101n));
  advance(40);
  assert.equal(latestCurrent(), true);
});
