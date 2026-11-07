// Renders the data hooks with react-dom/server (no DOM) against a static
// studio and asserts what a panel would actually read. useSyncExternalStore
// reads the initial snapshot during server render, so these prove the hooks
// wire through the Studio object without a browser.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import {renderToStaticMarkup} from 'react-dom/server';

import type {Snapshot} from '../src/core';
import {createStaticStudio, StudioProvider} from '../src/data/studio';
import {
  useActions,
  useChannelValue,
  useDictionary,
  useEvents,
  useModeBands,
  usePlayback,
  useRobotStatus,
  useSeries,
  useSnapshot,
  useSystem
} from '../src/data/hooks';
import {StatusStrip} from '../src/shell/StatusStrip';
import {Sparkline} from '../src/ui/Sparkline';
import {sampleStale, STALE_AFTER_NS} from '../src/data/freshness';
import {WorkerClient, WorkerRpcError, type WorkerPort} from '../src/data/worker-client';
import {WorkerUpdateStream, UPDATE_CAPACITY} from '../src/data/worker-stream';
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

function Probe() {
  const snapshot = useSnapshot();
  const velocity = useChannelValue('drivetrain/state/velocity');
  const absent = useChannelValue('nope/not/here');
  const status = useRobotStatus();
  const playback = usePlayback();
  const system = useSystem();
  const events = useEvents();
  const dictionary = useDictionary();
  const bands = useModeBands();
  const series = useSeries(['drivetrain/state/velocity'], 'all');
  const actions = useActions();
  return (
    <div>
      <span id="snap">{snapshot?.timestamp_ns ?? 'null'}</span>
      <span id="vel">{velocity}</span>
      <span id="absent">{absent === undefined ? 'undefined' : 'set'}</span>
      <span id="mode">{status.ds.mode ?? 'null'}</span>
      <span id="nodes">{status.nodes ? `${status.nodes.alive}/${status.nodes.total}` : 'null'}</span>
      <span id="pb">{playback.mode}</span>
      <span id="sys">{system.graph ? system.graph.nodes.length : 'null'}</span>
      <span id="events">{events.length}</span>
      <span id="dict">{dictionary.length}</span>
      <span id="bands">{bands.length}</span>
      <span id="series">{series.times.length}</span>
      <span id="actions">{typeof actions.rpc}</span>
    </div>
  );
}

test('hooks render from a static studio', () => {
  const studio = createStaticStudio({
    frames: [
      snap(0, '1000000000', {
        'drivetrain/state/velocity': 1.5,
        [DS + 'connected']: 1,
        [DS + 'enabled']: 0
      }),
      snap(1, '2000000000', {
        'drivetrain/state/velocity': 2.5,
        [DS + 'connected']: 1,
        [DS + 'enabled']: 1,
        [DS + 'teleop']: 1
      })
    ]
  });

  const html =
      renderToStaticMarkup(<StudioProvider studio={studio}><Probe/></StudioProvider>);
  assert.match(html, /id="snap">2000000000/);
  assert.match(html, /id="vel">2\.5/);
  assert.match(html, /id="absent">undefined/);
  assert.match(html, /id="mode">teleop/);
  assert.match(html, /id="nodes">null/);
  assert.match(html, /id="pb">PAUSED/);
  assert.match(html, /id="sys">null/);
  assert.match(html, /id="events">1/);
  assert.match(html, /id="dict">4/);
  assert.match(html, /id="series">2/);
  assert.match(html, /id="actions">function/);
});

test('static studio rpc rejects with a clear error', async () => {
  const studio = createStaticStudio({});
  await assert.rejects(() => studio.actions.rpc('get_latest', {}), /No worker/);
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
