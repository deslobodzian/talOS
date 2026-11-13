// Studio data-layer behaviour under react-dom/server (no DOM): status and
// freshness rendering, and the worker client's failure, timeout, flow-control
// and replacement handling against a fake worker port.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import {renderToStaticMarkup} from 'react-dom/server';

import type {Snapshot} from '../src/core';
import {createStaticStudio, createWorkerStudio, StudioProvider, WORKER_SILENT_MS} from '../src/data/studio';
import type {PlaybackState} from '../src/data/types';
import {StatusStrip} from '../src/shell/StatusStrip';
import {Sparkline} from '../src/ui/Sparkline';
import {readingClock, sampleStale, STALE_AFTER_NS} from '../src/data/freshness';
import {WorkerClient, WorkerRpcError, type WorkerPort} from '../src/data/worker-client';
import {WorkerUpdateStream, UPDATE_CAPACITY} from '../src/data/worker-stream';
import {demoSystemGraph} from '../src/demo';
import {parseSystemGraph} from '../src/system';
import {FreshnessNotice} from '../src/ui/FreshnessNotice';

const DS = 'hw/state/driver_station/';

function snap(seq: number, ts: string, channels: Record<string, number>): Snapshot {
  return {
    stream_id: 'a',
    sequence_id: String(seq),
    timestamp_ns: ts,
    dictionary_id: 1,
    chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
    channels,
    units: {},
    labels: {}
  };
}

test('static studio rpc rejects rather than waiting for a worker it has not got', async () => {
  await assert.rejects(() => createStaticStudio({}).actions.rpc('get_latest', {}));
});

test('status strip keeps unknown enabled state free of fault or healthy tones', () => {
  for (const enabled of [null, false, true]) {
    const studio = createStaticStudio({frames: [
      snap(0, '1', enabled === null ? {} : {[DS + 'enabled']: Number(enabled)})
    ]});
    const html = renderToStaticMarkup(
      <StudioProvider studio={studio}><StatusStrip/></StudioProvider>);
    const classes = /class="([^"]*)"><small>Enabled<\/small>/.exec(html)![1].split(' ');
    assert.equal(classes.includes('unknown'), enabled === null);
    assert.equal(classes.includes('warn'), enabled === false);
    assert.equal(classes.includes('ok'), enabled === true);
  }
});

test('sparkline leaves non-finite gaps disconnected between finite runs', () => {
  const html = renderToStaticMarkup(
    <Sparkline values={[0, 1, NaN, 2, 3, Infinity, 4, 5]} width={74} height={24}/>);
  const runs = [...html.matchAll(/points="([^"]*)"/g)].map(match =>
    match[1].split(' ').map(point => Number(point.split(',')[0])));
  assert.deepEqual(runs, [[2, 12], [32, 42], [62, 72]]);
  assert.equal((renderToStaticMarkup(<Sparkline values={[NaN, Infinity]}/>)
    .match(/<polyline/g) ?? []).length, 0);
});

test('freshness uses each channel sample and the session clock at the exact boundary', () => {
  assert.equal(sampleStale('42', String(42n + STALE_AFTER_NS - 1n)), false);
  assert.equal(sampleStale('42', String(42n + STALE_AFTER_NS)), true);
  assert.equal(sampleStale(null, '42'), true);
  const studio = createStaticStudio({
    frames: [snap(1, '0', {slow: 12}), snap(2, '1000000000', {fast: 99})],
    playback: {connection: 'live', clock_ns: '1000000000'}
  });
  const html = renderToStaticMarkup(<StudioProvider studio={studio}><FreshnessNotice/></StudioProvider>);
  assert.match(html, /slow: 12/);
  assert.doesNotMatch(html, /fast: 99/);
});

test('an unknown session clock leaves no reading current, even at the cursor', () => {
  // The worker posts clock_ns: null when the newest live sample's age is
  // unknown (the bridge's connect-time dictionary). Falling back to the cursor
  // would make that sample zero seconds old.
  const studio = createStaticStudio({
    frames: [snap(1, '5000000000', {fast: 99})],
    playback: {connection: 'live', mode: 'LIVE_STREAMING', clock_ns: null}
  });
  const html = renderToStaticMarkup(<StudioProvider studio={studio}><FreshnessNotice/></StudioProvider>);
  assert.match(html, /fast: 99/);
});

function fakeWorker(): WorkerPort {
  return {onmessage: null, onerror: null, onmessageerror: null,
    postMessage() {}, terminate() {}};
}

test('worker errors reject every pending RPC and permit replacement', async () => {
  for (const kind of ['error', 'messageerror', 'terminated'] as const) {
    let port = fakeWorker();
    const failures: WorkerRpcError[] = [];
    const client = new WorkerClient(() => (port = fakeWorker()), () => {}, error => failures.push(error));
    const calls = [client.rpc('one', {}), client.rpc('two', {})];
    const rejected = calls.map(call => assert.rejects(call, WorkerRpcError));
    if (kind === 'error')
      port.onerror?.call(port as Worker, {message: 'crash', preventDefault() {}} as ErrorEvent);
    else if (kind === 'messageerror')
      port.onmessageerror?.call(port as Worker, {} as MessageEvent);
    else client.fail(new WorkerRpcError('terminated', 'worker stopped'));
    await Promise.all(rejected);
    assert.equal(failures.length, 1);
    client.restart();
    const response = client.rpc('recovered', {});
    port.onmessage?.call(port as Worker, {data: {type: 'rpc', id: 3, response: 7}} as MessageEvent);
    assert.equal(await response, 7);
    client.fail(new WorkerRpcError('terminated', 'test cleanup'));
  }
});

// A worker studio over a fake port, with the silence watchdog's clock and
// check in the test's hands. `deliver` posts one acknowledged batch.
function workerStudio() {
  let port = fakeWorker();
  let now = 0;
  let check = () => {};
  const studio = createWorkerStudio(() => (port = fakeWorker()),
                                    {now: () => now, watch: fn => { check = fn; }});
  const deliver = (...messages: object[]) => port.onmessage?.call(
      port as Worker, {data: {type: 'updates', messages, dropped: 0, coalesced: 0}} as MessageEvent);
  const playback = (state: Partial<PlaybackState>) => ({type: 'playback', state: {
    mode: 'LIVE_STREAMING', cursor_ns: '5000000000', clock_ns: '5000000001',
    bounds: {start: '5000000000', end: '5000000000'},
    stats: {skipped: 0, late: 0, invalid: 0}, status: 'WebSocket connected',
    connection: 'live', ...state
  }});
  const current = () => !sampleStale(
      studio.store.channelSample('fast', studio.playback.cursor_ns)?.timestamp_ns ?? null,
      readingClock(studio.playback));
  return {
    studio, deliver, playback, current,
    crash: () => port.onerror?.call(port as Worker, {message: 'crash', preventDefault() {}} as ErrorEvent),
    elapse: (ms: number) => {
      now += ms;
      check();
    }
  };
}
const graphMessage = {type: 'system', graph: parseSystemGraph(demoSystemGraph(0n)), stale: false,
                      rates: {}, error: '', declared: null, declaredError: ''};

test('a failed worker leaves no live reading current and the graph stale', () => {
  const w = workerStudio();
  w.deliver({type: 'frames', reset: true, frames: [snap(1, '5000000000', {fast: 99})]},
            w.playback({}), graphMessage);
  assert.equal(w.current(), true);
  assert.equal(w.studio.system.stale, false);
  w.crash();
  assert.equal(w.studio.playback.connection, 'error');
  assert.equal(w.current(), false, 'the frozen clock no longer says "now"');
  assert.equal(w.studio.system.stale, true);
  assert.equal(w.studio.store.channelValue('fast'), 99, 'the reading stays inspectable');
});

test('a silent worker leaves no live reading current until it speaks again', () => {
  const w = workerStudio();
  w.deliver({type: 'frames', reset: true, frames: [snap(1, '5000000000', {fast: 99})]},
            w.playback({}), graphMessage);
  w.elapse(WORKER_SILENT_MS - 1);
  assert.equal(w.current(), true);
  assert.equal(w.studio.system.stale, false);
  w.elapse(1);
  assert.equal(w.current(), false);
  assert.equal(w.studio.system.stale, true);
  // The worker resumes: its own clock and graph freshness apply again.
  w.deliver(w.playback({}), graphMessage);
  assert.equal(w.current(), true);
  assert.equal(w.studio.system.stale, false);
  w.crash();
});

test('paused history keeps its cursor clock when the worker fails', () => {
  const w = workerStudio();
  w.deliver({type: 'frames', reset: true, frames: [snap(1, '5000000000', {fast: 99})]},
            w.playback({mode: 'PAUSED', clock_ns: '5000000000'}));
  assert.equal(w.current(), true);
  w.elapse(WORKER_SILENT_MS);
  assert.equal(w.current(), true, 'a paused view is judged at its cursor, not wall time');
  w.crash();
  assert.equal(w.current(), true);
  assert.equal(w.studio.playback.clock_ns, '5000000000');
});

test('worker RPC timeout rejects all callers rather than hanging', async () => {
  const client = new WorkerClient(fakeWorker, () => {}, () => {}, 5);
  await assert.rejects(client.rpc('stuck', {}), error => error instanceof WorkerRpcError && error.code === 'timeout');
});

test('worker update flood has one batch in flight and bounded retained history', () => {
  const sent: object[] = [];
  const stream = new WorkerUpdateStream(message => sent.push(message));
  stream.push({type: 'playback', state: 0});
  stream.flush();
  for (let i = 0; i < UPDATE_CAPACITY + 10; i++) {
    stream.push({type: 'frames', frames: [snap(i, String(i), {value: i})]});
    stream.push({type: 'playback', state: i});
    stream.flush();
  }
  assert.equal(sent.length, 1);
  assert.equal(stream.dropped, 10);
  assert.equal(stream.coalesced, UPDATE_CAPACITY + 9);
  stream.ack();
  stream.flush();
  const batch = sent[1] as {messages: {frames?: Snapshot[]; state?: number}[]};
  assert.equal(batch.messages[0].frames?.length, 2000);
  assert.equal(batch.messages[0].frames?.[0].sequence_id, '10');
  assert.equal(batch.messages[1].state, UPDATE_CAPACITY + 9);
});

test('heartbeat detects silent worker termination and rejects outstanding RPCs', async t => {
  t.mock.timers.enable({apis: ['Date', 'setInterval'], now: 0});
  const client = new WorkerClient(fakeWorker, () => {}, () => {});
  const rejected = assert.rejects(client.rpc('pending', {}), error =>
    error instanceof WorkerRpcError && error.code === 'terminated');
  t.mock.timers.tick(10000);
  await rejected;
});

test('callbacks and scheduled credits from a failed worker cannot affect its replacement', () => {
  const ports: WorkerPort[] = [];
  const posts: object[][] = [];
  const client = new WorkerClient(() => {
    const messages: object[] = [];
    const port = {...fakeWorker(), postMessage: (message: object) => { messages.push(message); }};
    ports.push(port);
    posts.push(messages);
    return port;
  }, () => {}, () => {});
  const oldError = ports[0].onerror!;
  const oldAck = client.acknowledger();
  client.fail(new WorkerRpcError('crashed', 'first worker'));
  client.restart();
  oldError.call(ports[0] as Worker, {message: 'late error', preventDefault() {}} as ErrorEvent);
  oldAck();
  assert.equal(posts[1].length, 0);
  client.acknowledger()();
  assert.deepEqual(posts[1], [{type: 'ack'}]);
  client.fail(new WorkerRpcError('terminated', 'test cleanup'));
});
