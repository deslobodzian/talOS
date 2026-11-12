// Recorded sessions (REQ-OPS-41, the browser side): the review server's
// binary series decode, RecordedSource request de-duplication, caching and
// abort, the playhead under an injected clock, the live staleness rule on a
// recorded snapshot, and HTTP or payload failures as an error state. The
// review server is a fake `fetch`.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {readingClock, sampleStale} from '../src/data/freshness';
import {useChannelStale, useChassis, usePlayback} from '../src/data/hooks';
import {
  bucketSeries,
  POINT_SIZES,
  quantizePoints,
  RecordedSource,
  SERIES_SLOTS,
  type ReviewClock
} from '../src/data/recorded';
import {
  decodeSeries,
  detectReviewServer,
  ReviewError,
  type FetchLike,
  type ReviewBounds
} from '../src/data/review';
import {createStaticStudio, StudioProvider} from '../src/data/studio';
import type {ChannelInfo} from '../src/data/types';
import {Header} from '../src/shell/Header';
import {ReviewPicker} from '../src/shell/ReviewPicker';

const SERVER = 'http://review.test';
const START = 1_000_000_000n;
const END = 11_000_000_000n;  // a 10 s session
const BOUNDS: ReviewBounds = {
  start_ns: START.toString(), end_ns: END.toString(), origin_wall_ns: '0'
};

type Column = {t: number[]; min: number[]; max: number[]; last: number[]; count: number[]};

// The wire layout, written field by field: u32 channels, u32 buckets, then
// per channel f64 t_ns[], f64 min[], f64 max[], f64 last[], u32 count[].
function seriesBuffer(columns: Column[]): ArrayBuffer {
  const buckets = columns[0]?.t.length ?? 0;
  const view = new DataView(new ArrayBuffer(8 + columns.length * buckets * 36));
  view.setUint32(0, columns.length, true);
  view.setUint32(4, buckets, true);
  let at = 8;
  for (const c of columns) {
    for (const column of [c.t, c.min, c.max, c.last])
      for (const v of column) view.setFloat64((at += 8) - 8, v, true);
    for (const v of c.count) view.setUint32((at += 4) - 4, v, true);
  }
  return view.buffer;
}

type Call = {url: URL; aborted: boolean; respond(response: Response): void};

// A review server that answers nothing until the test says so, unless
// `answer` returns a response for the URL.
function fakeServer(answer: (url: URL) => Response|undefined = () => undefined) {
  const calls: Call[] = [];
  const fetchFn: FetchLike = (url, init) => new Promise((resolve, reject) => {
    const call: Call = {url: new URL(url), aborted: false, respond: resolve};
    calls.push(call);
    init?.signal?.addEventListener('abort', () => {
      call.aborted = true;
      reject(new DOMException('aborted', 'AbortError'));
    });
    const response = answer(call.url);
    if (response) resolve(response);
  });
  const of = (path: string) => calls.filter(c => c.url.pathname.endsWith(path));
  return {fetchFn, calls, of};
}

const json = (body: unknown, status = 200) =>
    new Response(JSON.stringify(body), {status});

// A snapshot whose every channel was sampled at the requested time.
const echoSnapshot = (url: URL) => url.pathname.endsWith('/snapshot') ?
    json({t_ns: url.searchParams.get('t_ns'), channels: {}}) : undefined;

function fakeClock() {
  let now = 0;
  const ticks = new Set<() => void>();
  const clock: ReviewClock = {
    now: () => now,
    every(tick) {
      ticks.add(tick);
      return () => ticks.delete(tick);
    }
  };
  return {
    clock,
    ticking: () => ticks.size,
    advance(ms: number) {
      now += ms;
      for (const tick of [...ticks]) tick();
    }
  };
}

const settle = () => new Promise(resolve => setTimeout(resolve, 0));

const info = (name: string): ChannelInfo => ({name, unit: '', labels: []});

function source(server: ReturnType<typeof fakeServer>, channels: ChannelInfo[] = [],
                clock = fakeClock().clock) {
  return new RecordedSource(SERVER, 's1', channels, BOUNDS, server.fetchFn, clock);
}

test('series decode keeps empty buckets and a channel with no data as gaps', () => {
  const buffer = seriesBuffer([
    {t: [1e9, 2e9, 3e9], min: [0, NaN, 4], max: [2, NaN, 4], last: [1, NaN, 4],
     count: [3, 0, 1]},
    {t: [1e9, 2e9, 3e9], min: [NaN, NaN, NaN], max: [NaN, NaN, NaN],
     last: [NaN, NaN, NaN], count: [0, 0, 0]}
  ]);
  const decoded = decodeSeries(buffer, 2);
  assert.equal(decoded.buckets, 3);
  assert.deepEqual([...decoded.channels[0].count], [3, 0, 1]);
  assert.deepEqual([...decoded.channels[0].max], [2, NaN, 4]);
  assert.deepEqual([...decoded.channels[1].count], [0, 0, 0]);

  const series = bucketSeries(['a', 'b'], decoded, '0', 0n, 3_000_000_000n);
  assert.deepEqual([...series.times], [1, 3]);
  assert.deepEqual([...series.values.a], [1, 4]);
  assert.deepEqual([...series.min!.a], [0, 4]);
  assert.deepEqual([...series.values.b], [NaN, NaN]);
  assert.deepEqual([...series.count!.b], [0, 0]);

  // A response for other channels, or with a byte missing, is refused.
  assert.throws(() => decodeSeries(buffer, 1), ReviewError);
  assert.throws(() => decodeSeries(buffer.slice(0, buffer.byteLength - 1), 2),
                ReviewError);
});

// The server's t_ns is each channel's own last sample's publication time
// (studio/review/review.h): two channels of one bucket are drawn where each
// was published, never one at the other's time.
test('each channel\'s point sits at its own sample time', () => {
  const decoded = decodeSeries(seriesBuffer([
    {t: [1.0e9, 2.2e9], min: [5, 7], max: [5, 7], last: [5, 7], count: [1, 1]},
    {t: [1.5e9, 2.2e9], min: [9, 8], max: [9, 8], last: [9, 8], count: [1, 1]}
  ]), 2);
  const series = bucketSeries(['a', 'b'], decoded, '0', 0n, 3_000_000_000n);
  assert.deepEqual([...series.times], [1, 1.5, 2.2]);
  assert.deepEqual([...series.values.a], [5, NaN, 7]);
  assert.deepEqual([...series.values.b], [NaN, 9, 8]);
  assert.deepEqual([...series.count!.b], [0, 1, 1]);
});

test('identical series requests share one fetch and then the cache', async () => {
  const server = fakeServer(echoSnapshot);
  const recorded = source(server);
  const a = recorded.fetchSeries(['x'], '0', '10', 100);
  const b = recorded.fetchSeries(['x'], '0', '10', 100);
  assert.equal(server.of('/series').length, 1);
  const call = server.of('/series')[0];
  assert.equal(call.url.searchParams.get('c'), 'x');
  assert.equal(call.url.searchParams.get('points'), '100');
  call.respond(new Response(seriesBuffer([
    {t: [5], min: [1], max: [2], last: [2], count: [2]}])));
  const [first, second] = await Promise.all([a, b]);
  assert.ok(first);
  assert.equal(first, second);
  assert.equal(await recorded.fetchSeries(['x'], '0', '10', 100), first);
  assert.equal(server.of('/series').length, 1);
});

test('a playing cursor reuses its window and shows nothing past the playhead', async () => {
  const buckets = (url: URL) => {
    if (!url.pathname.endsWith('/series')) return echoSnapshot(url);
    const start = Number(url.searchParams.get('start_ns'));
    const end = Number(url.searchParams.get('end_ns'));
    const t = Array.from({length: 11}, (_, i) => start + (end - start) * i / 10);
    return new Response(seriesBuffer([{t, min: t.map(() => 0), max: t.map(() => 1),
                                       last: t.map(() => 1), count: t.map(() => 1)}]));
  };
  const server = fakeServer(buckets);
  const recorded = source(server);
  // Mid-way through one 1/16-window step of the fetch grid.
  const cursor = START + 5_050_000_000n;
  recorded.series(['x'], {seconds: 2}, cursor.toString());
  await settle();
  const later = recorded.series(['x'], {seconds: 2}, (cursor + 10_000_000n).toString());
  assert.equal(server.of('/series').length, 1);
  const lastNs = START + BigInt(Math.round(later.times.at(-1)! * 1e9));
  assert.ok(later.times.length > 0);
  assert.ok(lastNs <= cursor + 10_000_000n);
});

test('a consumer\'s newer series request aborts its superseded one', async () => {
  const server = fakeServer(echoSnapshot);
  const recorded = source(server);
  const old = recorded.fetchSeries(['x'], '0', '10', 100, 'plot');
  recorded.fetchSeries(['x'], '5', '15', 100, 'plot');
  const [superseded, current] = server.of('/series');
  assert.equal(superseded.aborted, true);
  assert.equal(current.aborted, false);
  assert.equal(await old, null);
  assert.equal(recorded.error, '');
});

// Dragging a split handle re-renders a plot at every pixel width from 400 to
// 800: one request per bucket size crossed (512, then 1024), not one per
// width, and a newer request of the same plot (the cursor moving a grid
// step) aborts its superseded one. Results kept per consumer stay bounded
// however many channel sets are plotted.
test('a plot resize sweep requests per bucket size and keeps a bounded set', async () => {
  const server = fakeServer(echoSnapshot);
  const recorded = source(server);
  const cursor = START + 5_050_000_000n;
  for (let width = 400; width <= 800; width++)
    recorded.series(['x'], {seconds: 2}, cursor.toString(), width);
  const sweep = server.of('/series');
  assert.deepEqual(sweep.map(c => c.url.searchParams.get('points')),
                   [String(512 + 32), String(1024 + 64)]);
  assert.equal(quantizePoints(800), 1024);
  assert.equal(quantizePoints(100_000), POINT_SIZES[POINT_SIZES.length - 1]);

  // One grid step (2 s / 16) later, at the same width.
  recorded.series(['x'], {seconds: 2}, (cursor + 125_000_000n).toString(), 800);
  assert.equal(server.of('/series').length, 3);
  assert.equal(sweep[1].aborted, true);
  assert.equal(sweep[0].aborted, false);

  for (let i = 0; i < 3 * SERIES_SLOTS; i++)
    recorded.series([`c${i}`], {seconds: 2}, cursor.toString(), 400);
  for (const call of server.of('/series')) {
    if (call.aborted) continue;
    const channels = call.url.searchParams.get('c')!.split(',').length;
    const points = Number(call.url.searchParams.get('points'));
    const zeros = () => Array.from({length: points}, () => 0);
    call.respond(new Response(seriesBuffer(Array.from({length: channels}, () => (
        {t: zeros(), min: zeros(), max: zeros(), last: zeros(), count: zeros()})))));
  }
  await settle();
  assert.equal(recorded.error, '');
  assert.ok(recorded.keptSlots <= SERIES_SLOTS, `${recorded.keptSlots} kept`);
});

test('steps clamp to the session and play advances at the chosen rate', async () => {
  const server = fakeServer(echoSnapshot);
  const time = fakeClock();
  const recorded = source(server, [], time.clock);
  const cursor = () => BigInt(recorded.playback.cursor_ns!);

  recorded.step('20000000');
  assert.equal(cursor(), START + 20_000_000n);
  recorded.step('-40000000');
  assert.equal(cursor(), START);

  recorded.play();
  assert.equal(recorded.playback.mode, 'REPLAY_PLAYING');
  time.advance(100);
  assert.equal(cursor(), START + 100_000_000n);
  recorded.setRate(4);
  time.advance(100);
  assert.equal(cursor(), START + 500_000_000n);
  recorded.setRate(2);
  time.advance(100);
  assert.equal(cursor(), START + 700_000_000n);

  // Playing past the end stops on the last instant.
  time.advance(60_000);
  assert.equal(cursor(), END);
  assert.equal(recorded.playback.mode, 'PAUSED');
  assert.equal(time.ticking(), 0);

  // A step pauses playback.
  recorded.seek(START.toString());
  recorded.play();
  recorded.step('1000000');
  assert.equal(recorded.playback.mode, 'PAUSED');
  assert.equal(time.ticking(), 0);
  await settle();
  assert.equal(recorded.error, '');
});

test('a recorded value one second older than the playhead is stale, as live', async () => {
  const pose = 'pose_estimator/state/pose/';
  const channels = [`${pose}x_m`, `${pose}y_m`, `${pose}yaw_rad`,
                    'pose_estimator/state/valid',
                    'pose_estimator/state/observation_accepted_count',
                    'hw/status/battery_v'].map(info);
  const at = START + 5_000_000_000n;
  let poseAge = 999_999_999n, valid = 1;
  const server = fakeServer(url => {
    if (!url.pathname.endsWith('/snapshot')) return undefined;
    const sample = (value: number, age: bigint) =>
        ({value, sample_ns: (at - age).toString()});
    return json({t_ns: at.toString(), channels: {
      [`${pose}x_m`]: sample(1, poseAge), [`${pose}y_m`]: sample(2, poseAge),
      [`${pose}yaw_rad`]: sample(0, poseAge),
      'pose_estimator/state/valid': sample(valid, poseAge),
      'hw/status/battery_v': sample(12, 1_000_000_000n)
    }});
  });
  const studio = createStaticStudio();
  const readings: string[] = [];
  function Probe() {
    const chassis = useChassis();
    const battery = useChannelStale('hw/status/battery_v');
    usePlayback();
    readings.push(JSON.stringify({chassis: chassis && chassis.stale, battery}));
    return null;
  }
  const read = async () => {
    const recorded = new RecordedSource(SERVER, 's1', channels, BOUNDS, server.fetchFn,
                                        fakeClock().clock);
    recorded.seek(at.toString());
    await settle();
    studio.openRecorded(recorded);
    renderToStaticMarkup(<StudioProvider studio={studio}><Probe/></StudioProvider>);
    return JSON.parse(readings.at(-1)!);
  };

  assert.deepEqual(await read(), {chassis: false, battery: true});
  poseAge = 1_000_000_000n;
  assert.deepEqual(await read(), {chassis: true, battery: true});
  valid = 0;
  assert.deepEqual(await read(), {chassis: null, battery: true});
});

// At 4x the playhead runs ahead of the snapshot in flight: a reading's age is
// measured against its snapshot's own time (live measures it against its
// frame's), so a value 0.5 s old there stays fresh with the playhead a
// second further on, and one a second old there is stale.
test('a lagging snapshot during 4x playback ages its readings by its own time', async () => {
  const at = START + 5_000_000_000n;
  const server = fakeServer(url => {
    if (!url.pathname.endsWith('/snapshot') || url.searchParams.get('t_ns') !== at.toString())
      return undefined;  // later snapshots stay in flight
    const sample = (age: bigint) => ({value: 1, sample_ns: (at - age).toString()});
    return json({t_ns: at.toString(),
                 channels: {fresh: sample(500_000_000n), old: sample(1_000_000_000n)}});
  });
  const time = fakeClock();
  const recorded = source(server, [info('fresh'), info('old')], time.clock);
  recorded.seek(at.toString());
  await settle();
  recorded.setRate(4);
  recorded.play();
  for (let i = 0; i < 4; i++) time.advance(62.5);
  assert.equal(BigInt(recorded.playback.cursor_ns!), at + 1_000_000_000n);
  assert.equal(recorded.snapshotAtCursor()?.timestamp_ns, at.toString());
  const stale = (name: string) => sampleStale(
      recorded.channelSample(name)?.timestamp_ns ?? null, readingClock(recorded.playback));
  assert.equal(stale('fresh'), false);
  assert.equal(stale('old'), true);
  recorded.close();
});

test('an HTTP error or malformed payload is an error state, never a throw', async () => {
  // Opening: a malformed bounds document rejects with what is wrong.
  const broken = fakeServer(url => url.pathname.endsWith('/bounds.json') ?
      json({start_ns: '5', end_ns: '1', origin_wall_ns: '0'}) :
      json({channels: []}));
  await assert.rejects(RecordedSource.open(SERVER, 's1', {fetch: broken.fetchFn}),
                       ReviewError);

  // A snapshot answered with HTTP 500.
  const failing = fakeServer(url => url.pathname.endsWith('/snapshot') ?
      new Response('', {status: 500}) : undefined);
  const recorded = source(failing);
  await settle();
  assert.match(recorded.error, /HTTP 500/);
  assert.equal(recorded.playback.connection, 'error');
  assert.equal(recorded.snapshotAtCursor(), null);

  // A series of the wrong length: the getter returns no rows, sets the
  // error, and does not ask again on every render.
  const truncated = fakeServer(url => url.pathname.endsWith('/series') ?
      new Response(new ArrayBuffer(7)) : echoSnapshot(url));
  const plot = source(truncated);
  const cursor = (START + 1_000_000_000n).toString();
  assert.equal(plot.series(['x'], {seconds: 1}, cursor).times.length, 0);
  await settle();
  assert.match(plot.error, /truncated/);
  assert.equal(plot.series(['x'], {seconds: 1}, cursor).times.length, 0);
  assert.equal(truncated.of('/series').length, 1);
});

// A Studio served by studio_review reviews its own origin's sessions: found
// by that origin answering a session list, which the live bridge (404) and a
// dev server (its index page) do not. In that mode the picker has no address
// field and the header offers no live connection.
test('a Studio served by the review server reviews its own origin', async () => {
  const answering = (response: () => Response): FetchLike => () =>
      Promise.resolve(response());
  const origin = 'http://127.0.0.1:5805';
  assert.equal(await detectReviewServer(origin, answering(() => json([]))), origin);
  assert.equal(await detectReviewServer(origin, answering(() => new Response('', {status: 404}))),
               null);
  assert.equal(await detectReviewServer(
                   origin, answering(() => new Response('<!doctype html>'))), null);
  assert.equal(await detectReviewServer('file://', answering(() => json([]))), null);

  const studio = createStaticStudio();
  const render = (node: React.ReactNode) =>
      renderToStaticMarkup(<StudioProvider studio={studio}>{node}</StudioProvider>);
  const noop = () => {};
  const header = (review: boolean) => render(
      <Header url="ws://x" onUrl={noop} onConnect={noop} onDemo={noop} onImportClick={noop}
              onExport={noop} onReset={noop} onReview={noop} review={review}/>);
  assert.match(header(false), /WebSocket URL/);
  assert.doesNotMatch(header(true), /WebSocket URL|>Connect</);
  const fetchNothing: FetchLike = () => new Promise(() => {});
  assert.match(render(<ReviewPicker onClose={noop} fetchFn={fetchNothing}/>),
               /Review server address/);
  assert.doesNotMatch(
      render(<ReviewPicker onClose={noop} fetchFn={fetchNothing} server={origin}/>),
      /Review server address/);
});
