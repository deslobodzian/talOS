// Recorded sessions (REQ-OPS-41, the browser side): the review server's
// binary series decode, RecordedSource request de-duplication, caching and
// abort, the playhead under an injected clock, the live staleness rule on a
// recorded snapshot, and HTTP or payload failures as an error state. The
// review server is a fake `fetch`.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {AlertLevel} from '../src/data/alerts';
import {readingClock, sampleStale} from '../src/data/freshness';
import {useChannelStale, useChassis, usePlayback} from '../src/data/hooks';
import {
  alertMarks,
  bucketSeries,
  EMPTY_TIMELINE,
  GAP_MIN_WIDTH,
  gapBands,
  POINT_SIZES,
  quantizePoints,
  recordedModeLanes,
  RecordedSource,
  restartMarks,
  SERIES_SLOTS,
  type ReviewClock
} from '../src/data/recorded';
import {
  decodeSeries,
  detectReviewServer,
  parseBounds,
  parseTimeline,
  ReviewError,
  type FetchLike,
  type ReviewBounds,
  type ReviewGap,
  type ReviewTimeline
} from '../src/data/review';
import {createStaticStudio, StudioProvider} from '../src/data/studio';
import type {ChannelInfo} from '../src/data/types';
import {demoDeclaredGraph, demoSystemGraph} from '../src/demo';
import {eventsPanel} from '../src/panels/events';
import {healthPanel} from '../src/panels/health';
import {plotColumn} from '../src/panels/plot';
import {Header} from '../src/shell/Header';
import {RecordedTransport} from '../src/shell/RecordedTransport';
import {ReviewPicker} from '../src/shell/ReviewPicker';
import {parseDeclaredGraph, parseSystemGraph} from '../src/system';

const SERVER = 'http://review.test';
const START = 1_000_000_000n;
const END = 11_000_000_000n;  // a 10 s session
// The server's series limits, as its bounds.json reports them
// (review_golden.test.ts pins them against the golden response).
const MAX_SERIES_VALUES = 262_144;
const MAX_SERIES_CHANNELS = 256;
const BOUNDS: ReviewBounds = {
  start_ns: START.toString(), end_ns: END.toString(), origin_wall_ns: '0',
  max_series_values: MAX_SERIES_VALUES, max_series_channels: MAX_SERIES_CHANNELS, gaps: []
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
// however many channel sets are plotted one view after another.
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

  // Channel sets plotted one view after another (each answered before the
  // next): what earlier views kept is dropped past SERIES_SLOTS.
  const zeros = Array.from({length: 512 + 32}, () => 0);
  const body = seriesBuffer([{t: zeros, min: zeros, max: zeros, last: zeros, count: zeros}]);
  for (let i = 0; i < SERIES_SLOTS + 8; i++) {
    recorded.series([`c${i}`], {seconds: 2}, cursor.toString(), 400);
    server.of('/series').at(-1)!.respond(new Response(body.slice(0)));
    await Promise.resolve();
    await Promise.resolve();
    await Promise.resolve();
  }
  await settle();
  assert.equal(recorded.error, '');
  assert.ok(recorded.keptSlots <= SERIES_SLOTS, `${recorded.keptSlots} kept`);
});

// A plot of 200 channels at 4096 px asks 4352 points (4096 and the window's
// extra 1/16): one request of them all would be 870,400 values, past the
// server's 262,144 (a 400, and an empty plot). The channels are split into
// requests that each fit, and every channel gets its data.
// A static view whose plots need more requests than the result cache and
// the per-consumer slots hold (a server cap of one channel a request makes
// 70 channels 70 requests): every completion re-renders, and a render must not refetch
// what an earlier completion of this view evicted. Each request is issued
// once, then no more however often the unchanged view renders.
test('a static view needing many requests settles after one of each', async () => {
  // Every request is 1 channel of 256 + 16 points: one body serves them all.
  const t = Array.from({length: 256 + 16}, (_, i) => Number(START) + 4e9 + i * 1e6);
  const ones = t.map(() => 1);
  const body = seriesBuffer([{t, min: ones, max: ones, last: ones, count: ones}]);
  const answer = (url: URL) => url.pathname.endsWith('/series') ?
      new Response(body.slice(0)) : echoSnapshot(url);
  const server = fakeServer(answer);
  const recorded = new RecordedSource(
      SERVER, 's1', [], {...BOUNDS, max_series_values: 256 + 16},
      server.fetchFn, fakeClock().clock);
  const channels = Array.from({length: 70}, (_, i) => `c${i}`);
  const cursor = (START + 5_000_000_000n).toString();
  let rendered = recorded.series(channels, {seconds: 2}, cursor, 256);
  // Each completion re-renders; past 500 the view has not settled (the bound
  // keeps an unsettled view from spinning the test forever).
  let renders = 0;
  recorded.subscribe(() => {
    if (++renders > 500) return;
    rendered = recorded.series(channels, {seconds: 2}, cursor, 256);
  });
  for (let i = 0; i < 10; i++) await settle();
  const urls = server.of('/series').map(c => c.url.searchParams.get('c'));
  assert.equal(urls.length, 70);
  assert.equal(new Set(urls).size, 70);
  for (const name of channels)
    assert.ok([...rendered.values[name]].some(Number.isFinite), name);
});

test('a wide plot of many channels splits its series within the server cap', async () => {
  const server = fakeServer(echoSnapshot);
  const recorded = source(server);
  const channels = Array.from({length: 200}, (_, i) => `c${i}`);
  const cursor = (START + 5_000_000_000n).toString();
  recorded.series(channels, {seconds: 2}, cursor, 4096);
  const calls = server.of('/series');
  assert.ok(calls.length > 1);
  const asked: string[] = [];
  for (const call of calls) {
    const names = call.url.searchParams.get('c')!.split(',');
    const points = Number(call.url.searchParams.get('points'));
    assert.ok(names.length * points <= MAX_SERIES_VALUES, `${names.length} x ${points}`);
    assert.ok(names.length <= MAX_SERIES_CHANNELS);
    asked.push(...names);
    const t = Array.from({length: points}, (_, i) => Number(START) + 4e9 + i * 1e5);
    call.respond(new Response(seriesBuffer(names.map(name => {
      const value = Number(name.slice(1));
      return {t, min: t.map(() => value), max: t.map(() => value),
              last: t.map(() => value), count: t.map(() => 1)};
    }))));
  }
  assert.deepEqual(asked, channels);
  await settle();
  const series = recorded.series(channels, {seconds: 2}, cursor, 4096);
  assert.equal(server.of('/series').length, calls.length);
  for (const [i, name] of channels.entries()) {
    const values = [...series.values[name]].filter(Number.isFinite);
    assert.ok(values.length > 0, name);
    assert.ok(values.every(v => v === i), name);
  }
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
      url.pathname.endsWith('/timeline.json') ? json(EMPTY_TIMELINE) :
      url.pathname.endsWith('/declared.json') ? new Response('', {status: 404}) :
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
  // A different request (the cursor moved two seconds) is issued: a failure
  // does not strand the plot.
  plot.series(['x'], {seconds: 1}, (START + 3_000_000_000n).toString());
  assert.equal(truncated.of('/series').length, 2);
  await settle();
  // retry() clears the error and the footer's error status.
  plot.retry();
  assert.equal(plot.error, '');
  assert.equal(plot.playback.connection, 'replay');
  assert.doesNotMatch(plot.playback.status, /Review error/);
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

// REQ-OPS-51: a session whose hardware packets the server refused serves no
// hw/ channel, and its bounds.json says why. The page shows that line once,
// in the session status the header and the health panel read, as text: it
// is data from the session's files, so markup in it is escaped.
test('a session whose hardware packets were not decoded says why in its status', async () => {
  const reason = 'hardware packets not decoded: recorded under hardware protocol 7; ' +
      "this build's is 8 (manifest.json hardware_protocol_version) <b>x</b>";
  const document = {...BOUNDS, hardware_not_decoded: reason};
  assert.equal(parseBounds(document).hardware_not_decoded, reason);
  assert.equal(parseBounds({...BOUNDS, hardware_not_decoded: null}).hardware_not_decoded,
               undefined);
  assert.throws(() => parseBounds({...BOUNDS, hardware_not_decoded: 7}), ReviewError);

  const server = fakeServer(url => url.pathname.endsWith('/bounds.json') ?
      json(document) : url.pathname.endsWith('/channels.json') ?
      json({channels: []}) : url.pathname.endsWith('/timeline.json') ?
      json({...EMPTY_TIMELINE, hardware_not_decoded: true}) :
      url.pathname.endsWith('/declared.json') ?
      new Response('', {status: 404}) : echoSnapshot(url));
  const refused = await RecordedSource.open(SERVER, 's1', {fetch: server.fetchFn,
                                                           clock: fakeClock().clock});
  assert.equal(refused.playback.status, `Recorded session s1 · ${reason}`);
  const intact = source(server);
  assert.equal(intact.playback.status, 'Recorded session s1');

  const noop = () => {};
  const header = renderToStaticMarkup(
      <StudioProvider studio={createStaticStudio({playback: refused.playback})}>
        <Header url="ws://x" onUrl={noop} onConnect={noop} onDemo={noop} onImportClick={noop}
                onExport={noop} onReset={noop} onReview={noop} review/>
      </StudioProvider>);
  assert.match(header, /recorded under hardware protocol 7/);
  assert.doesNotMatch(header, /<b>/);
  assert.match(header, /&lt;b&gt;x&lt;\/b&gt;/);
  refused.close();
  intact.close();
});

// REQ-OPS-41: bounds.json lists every GAP record; the parser keeps the well
// formed ones, reads an absent list (an older server) as none, and drops a
// malformed entry without losing the session.
test('bounds.json gaps parse; absent is none; a malformed entry is dropped', () => {
  const wire = {process: 'bridge', start_ns: '3000000000', end_ns: '4000000000',
                records: 12, dispatches: 3, sends: 4, closed_at_flush: false};
  const {gaps: _, ...older} = BOUNDS;
  assert.deepEqual(parseBounds(older).gaps, []);
  const parsed = parseBounds({...BOUNDS, gaps: [wire, {process: 'x', start_ns: -1},
                                                {...wire, end_ns: '2000000000'}]});
  assert.deepEqual(parsed.gaps, [wire]);
  assert.equal(parsed.start_ns, BOUNDS.start_ns);
});

const gap = (over: Partial<ReviewGap> = {}): ReviewGap => ({
  process: 'bridge', start_ns: '3500000000', end_ns: '4500000000', records: 120,
  dispatches: 30, sends: 40, closed_at_flush: false, ...over
});

// REQ-OPS-41: a gap is a band over its stretch of the scrub bar, never too
// narrow to see, and one at the very end stays inside the bar.
test('gap bands sit at their fraction of the session with a minimum width', () => {
  const bounds = {...BOUNDS, gaps: [
    gap(),
    gap({process: 'shooter', start_ns: '6000000000', end_ns: '6000000001'}),
    gap({start_ns: END.toString(), end_ns: END.toString(), closed_at_flush: true})
  ]};
  const [known, thin, last] = gapBands(bounds);
  // 2.5 s into a 10 s session, 1 s long.
  assert.ok(Math.abs(known.left - 0.25) < 1e-9);
  assert.ok(Math.abs(known.width - 0.1) < 1e-9);
  assert.match(known.title, /bridge/);
  assert.match(known.title, /1\.000 s/);
  assert.match(known.title, /120 records, 30 dispatches, 40 sends/);
  assert.doesNotMatch(known.title, /stop/);
  assert.ok(Math.abs(thin.left - 0.5) < 1e-9);
  assert.equal(thin.width, GAP_MIN_WIDTH);
  assert.ok(Math.abs(last.left + last.width - 1) < 1e-9);
  assert.equal(last.width, GAP_MIN_WIDTH);
  assert.match(last.title, /closed at the stop/);
});

// REQ-OPS-41: the status line counts the session's gaps, and says nothing
// when there are none; the transport draws one band per gap.
test('the recorded transport counts and draws log gaps', () => {
  const server = fakeServer(echoSnapshot);
  const plain = source(server);
  assert.equal(plain.playback.status, 'Recorded session s1');
  const gapped = new RecordedSource(SERVER, 's1', [], {...BOUNDS, gaps: [gap(), gap()]},
                                    server.fetchFn, fakeClock().clock);
  assert.equal(gapped.playback.status, 'Recorded session s1 · 2 log gaps');
  const noop = () => {};
  const html = (s: RecordedSource) => renderToStaticMarkup(
      <StudioProvider studio={createStaticStudio({playback: s.playback})}>
        <RecordedTransport source={s} stepNs="1000000" onStepNs={noop} onClose={noop}/>
      </StudioProvider>);
  assert.equal(html(gapped).match(/class="scrub-gap"/g)?.length, 2);
  assert.doesNotMatch(html(plain), /scrub-gap/);
  plain.close();
  gapped.close();
});

const SAMPLE_TIMELINE: ReviewTimeline = {
  hardware_not_decoded: false,
  modes: [
    {start_ns: (START + 0n).toString(), end_ns: (START + 2_000_000_000n).toString(),
     mode: 'disabled', fms: false},
    {start_ns: (START + 2_000_000_000n).toString(), end_ns: (START + 5_000_000_000n).toString(),
     mode: 'auto', fms: true},
    {start_ns: (START + 5_000_000_000n).toString(), end_ns: (START + 8_000_000_000n).toString(),
     mode: 'teleop', fms: true},
    {start_ns: (START + 8_000_000_000n).toString(), end_ns: END.toString(),
     mode: 'estop', fms: false}
  ],
  modes_dropped: 0,
  alerts: [
    {channel: 'hw/status/alert_error_link_lost', level: 'error',
     start_ns: (START + 1_000_000_000n).toString(),
     end_ns: (START + 3_000_000_000n).toString()},
    {channel: 'hw/status/alert_error_link_lost', level: 'error',
     start_ns: (START + 6_000_000_000n).toString(),
     end_ns: END.toString()},
    {channel: 'shooter/status/alert_warning_temp_high', level: 'warning',
     start_ns: (START + 4_000_000_000n).toString(),
     end_ns: (START + 5_000_000_000n).toString()}
  ],
  alerts_dropped: 0,
  runs: [
    {node: 'shooter', start_ns: START.toString(),
     end_ns: (START + 6_000_000_000n).toString(), reason: 'crash'},
    {node: 'shooter', start_ns: (START + 7_000_000_000n).toString(),
     end_ns: null, reason: 'running'}
  ],
  runs_dropped: 0
};

// REQ-OPS-41, REQ-OPS-37, REQ-CONTRACT-29: RecordedSource.open fetches
// timeline.json and declared.json (treating 404 on declared.json as null),
// and parseTimeline rejects malformed payloads.
test('RecordedSource.open loads timeline and optional declared graph', async () => {
  const declaredJson = demoDeclaredGraph();
  const makeServer = (declaredStatus: number) => fakeServer(url => {
    if (url.pathname.endsWith('/channels.json')) return json({channels: []});
    if (url.pathname.endsWith('/bounds.json')) return json(BOUNDS);
    if (url.pathname.endsWith('/timeline.json')) return json(SAMPLE_TIMELINE);
    if (url.pathname.endsWith('/declared.json')) {
      return declaredStatus === 200 ?
          new Response(declaredJson, {status: 200, headers: {'content-type': 'application/json'}}) :
          new Response('no declared graph\n', {status: declaredStatus});
    }
    return echoSnapshot(url);
  });

  const withDeclared = await RecordedSource.open(SERVER, 's1', {
    fetch: makeServer(200).fetchFn, clock: fakeClock().clock
  });
  assert.ok(withDeclared.declared);
  assert.equal(withDeclared.declared.nodes.length, parseDeclaredGraph(declaredJson).nodes.length);
  assert.equal(withDeclared.timeline.modes.length, 4);
  withDeclared.close();

  const withoutDeclared = await RecordedSource.open(SERVER, 's1', {
    fetch: makeServer(404).fetchFn, clock: fakeClock().clock
  });
  assert.equal(withoutDeclared.declared, null);
  withoutDeclared.close();

  assert.throws(() => parseTimeline({...EMPTY_TIMELINE, hardware_not_decoded: 'no'}), ReviewError);
  assert.throws(() => parseTimeline({
    ...EMPTY_TIMELINE,
    modes: [{start_ns: '10', end_ns: '5', mode: 'teleop', fms: false}]
  }), ReviewError);
  assert.throws(() => parseTimeline({
    ...EMPTY_TIMELINE,
    modes: [{start_ns: '1', end_ns: '5', mode: 'bogus', fms: false}]
  }), ReviewError);
  assert.throws(() => parseTimeline({
    ...EMPTY_TIMELINE,
    alerts: [{channel: 'a', level: 'fatal', start_ns: '1', end_ns: '2'}]
  }), ReviewError);
});

// REQ-OPS-41, REQ-OPS-37, REQ-OPS-51: RecordedSource derives events, modeBands,
// and playhead-aware alertLatch from timeline.json + bounds.gaps, and never
// misreports hardware_not_decoded as disabled.
test('RecordedSource derives events, modeBands, alertLatch and respects hardware_not_decoded', () => {
  const server = fakeServer(echoSnapshot);
  const gappedBounds = {...BOUNDS, gaps: [gap({start_ns: (START + 2_500_000_000n).toString(),
                                               end_ns: (START + 2_800_000_000n).toString()})]};
  const rec = new RecordedSource(SERVER, 's1', [], gappedBounds, server.fetchFn,
                                 fakeClock().clock, SAMPLE_TIMELINE, null);

  // Mode bands map 'auto' -> 'autonomous'.
  assert.deepEqual(rec.modeBands().map(b => b.mode),
                   ['disabled', 'autonomous', 'teleop', 'estop']);

  // Events are chronologically sorted and cover modes, alerts (raise+clear,
  // no false clear on active-to-end), crash+restart, and log gaps.
  const events = rec.events();
  for (let i = 1; i < events.length; i++) {
    assert.ok(BigInt(events[i].t_ns) >= BigInt(events[i - 1].t_ns), 'not sorted');
  }
  const titles = events.map(e => e.title);
  assert.ok(titles.includes('Disabled'), titles.join(', '));
  assert.ok(titles.includes('Enabled · autonomous'), titles.join(', '));
  assert.ok(titles.includes('FMS attached'), titles.join(', '));
  assert.ok(titles.includes('Enabled · teleop'), titles.join(', '));
  assert.ok(titles.includes('E-stop'), titles.join(', '));
  assert.ok(titles.includes('hw: link lost'), titles.join(', '));
  assert.ok(titles.includes('hw: link lost cleared'), titles.join(', '));
  assert.equal(titles.filter(t => t === 'hw: link lost cleared').length, 1);
  assert.ok(titles.includes('shooter crash'), titles.join(', '));
  assert.ok(titles.includes('shooter restarted (run 2)'), titles.join(', '));
  assert.ok(titles.includes('Log gap in bridge'), titles.join(', '));

  // alertLatch tracks active vs latched across playhead positions and returns
  // a stable reference until the playhead moves.
  const atStart = rec.alertLatch();
  assert.equal(atStart.alerts.length, 0);
  assert.equal(rec.alertLatch(), atStart);

  rec.seek((START + 2_000_000_000n).toString());
  const duringFirst = rec.alertLatch();
  assert.equal(duringFirst.alerts.length, 1);
  assert.equal(duringFirst.alerts[0].channel, 'hw/status/alert_error_link_lost');
  assert.equal(duringFirst.alerts[0].level, AlertLevel.Error);
  assert.equal(duringFirst.alerts[0].active, true);
  assert.equal(duringFirst.alerts[0].count, 1);

  rec.seek((START + 4_500_000_000n).toString());
  const afterClear = rec.alertLatch();
  assert.equal(afterClear.alerts.length, 2);
  assert.equal(afterClear.alerts[0].active, false);  // link lost cleared at 3s
  assert.equal(afterClear.alerts[1].channel, 'shooter/status/alert_warning_temp_high');
  assert.equal(afterClear.alerts[1].active, true);

  rec.seek(END.toString());
  const atEnd = rec.alertLatch();
  assert.equal(atEnd.alerts[0].active, true);  // second activation lasts to END
  assert.equal(atEnd.alerts[0].count, 2);
  assert.equal(atEnd.alerts[1].active, false);
  rec.close();

  // hardware_not_decoded: modeBands is empty and events emit a single notice,
  // never pretending the robot was disabled.
  const undecoded = new RecordedSource(
      SERVER, 's1', [], {...BOUNDS, hardware_not_decoded: 'protocol mismatch'},
      server.fetchFn, fakeClock().clock,
      {...EMPTY_TIMELINE, hardware_not_decoded: true}, null);
  assert.deepEqual(undecoded.modeBands(), []);
  assert.equal(undecoded.events().length, 1);
  assert.equal(undecoded.events()[0].title, 'Hardware packets not decoded');
  assert.doesNotMatch(undecoded.events()[0].title, /Disabled/);
  undecoded.close();
});

// REQ-OPS-41, REQ-OPS-37: Events panel works on recordings and Health panel
// renders an honest recorded view without fabricating live registry liveness.
test('Events and Health panels render recorded sessions honestly', () => {
  assert.equal(eventsPanel.recorded, true);
  assert.equal(healthPanel.recorded, true);

  const server = fakeServer(echoSnapshot);
  const declared = parseDeclaredGraph(demoDeclaredGraph());
  const gappedBounds = {...BOUNDS, gaps: [gap()]};
  const rec = new RecordedSource(SERVER, 's1', [info('shooter/state/position_m')],
                                 gappedBounds, server.fetchFn, fakeClock().clock,
                                 SAMPLE_TIMELINE, declared);
  rec.seek((START + 7_000_000_000n).toString());

  // Even if studio.system holds a live graph from earlier, Health on a
  // recorded session must never show live registry counts or fabricate rows.
  const studio = createStaticStudio({
    system: {graph: parseSystemGraph(demoSystemGraph(50n)), stale: false}
  });
  studio.openRecorded(rec);

  const eventsHtml = renderToStaticMarkup(
      <StudioProvider studio={studio}>
        <eventsPanel.component config={eventsPanel.defaultConfig}
                               setConfig={() => {}} instanceId="ev"/>
      </StudioProvider>);
  assert.match(eventsHtml, /Recorded timeline/);
  assert.match(eventsHtml, /shooter crash/);
  assert.match(eventsHtml, /hw: link lost/);
  assert.match(eventsHtml, /Log gap in bridge/);

  const healthHtml = renderToStaticMarkup(
      <StudioProvider studio={studio}>
        <healthPanel.component config={healthPanel.defaultConfig}
                               setConfig={() => {}} instanceId="hl"/>
      </StudioProvider>);
  assert.match(healthHtml, /Live registry liveness is not present in recorded sessions/);
  assert.doesNotMatch(healthHtml, /Nodes live/);
  assert.doesNotMatch(healthHtml, /Frames forwarded/);
  assert.match(healthHtml, /Declared nodes/);
  assert.match(healthHtml, /shooter/);
  assert.match(healthHtml, /crash/);
  assert.match(healthHtml, /running/);  // end_ns === null handled honestly
  assert.match(healthHtml, /link lost/);
  assert.match(healthHtml, /log gap in bridge/);

  // Without declared.json, Health says so honestly.
  const noDecl = new RecordedSource(SERVER, 's1', [], BOUNDS, server.fetchFn,
                                    fakeClock().clock, EMPTY_TIMELINE, null);
  studio.openRecorded(noDecl);
  const noDeclHtml = renderToStaticMarkup(
      <StudioProvider studio={studio}>
        <healthPanel.component config={healthPanel.defaultConfig}
                               setConfig={() => {}} instanceId="hl"/>
      </StudioProvider>);
  assert.match(noDeclHtml, /No declared graph \(graph\.json\) in this session/);
  assert.match(noDeclHtml, /No faults, restarts, alerts, or log gaps recorded/);
  studio.closeRecorded();
});

// REQ-OPS-41, REQ-OPS-51: RecordedTransport renders mode bands (or an undecoded
// bar), alert marks, restart/unclean-exit marks, and gap bands.
test('RecordedTransport renders timeline lanes for modes, alerts, restarts and gaps', () => {
  const server = fakeServer(echoSnapshot);
  const rec = new RecordedSource(SERVER, 's1', [], {...BOUNDS, gaps: [gap()]},
                                 server.fetchFn, fakeClock().clock, SAMPLE_TIMELINE, null);
  assert.equal(recordedModeLanes(rec.bounds, rec.timeline).length, 4);
  assert.equal(alertMarks(rec.bounds, rec.timeline).length, 3);
  assert.equal(restartMarks(rec.bounds, rec.timeline).length, 2);  // crash + restart

  const noop = () => {};
  const html = renderToStaticMarkup(
      <StudioProvider studio={createStaticStudio({playback: rec.playback})}>
        <RecordedTransport source={rec} stepNs="1000000" onStepNs={noop} onClose={noop}/>
      </StudioProvider>);
  assert.match(html, /class="scrub-mode disabled"/);
  assert.match(html, /class="scrub-mode autonomous"/);
  assert.match(html, /class="scrub-mode teleop"/);
  assert.match(html, /class="scrub-mode estop"/);
  assert.match(html, /class="scrub-alert error"/);
  assert.match(html, /class="scrub-alert warning"/);
  assert.match(html, /class="scrub-restart exit"/);
  assert.match(html, /class="scrub-restart restart"/);
  assert.match(html, /class="scrub-gap"/);
  rec.close();

  const undecoded = new RecordedSource(
      SERVER, 's1', [], {...BOUNDS, hardware_not_decoded: 'protocol mismatch'},
      server.fetchFn, fakeClock().clock,
      {...EMPTY_TIMELINE, hardware_not_decoded: true}, null);
  const undecodedHtml = renderToStaticMarkup(
      <StudioProvider studio={createStaticStudio({playback: undecoded.playback})}>
        <RecordedTransport source={undecoded} stepNs="1000000" onStepNs={noop} onClose={noop}/>
      </StudioProvider>);
  assert.match(undecodedHtml, /class="scrub-mode undecoded"/);
  assert.doesNotMatch(undecodedHtml, /class="scrub-mode disabled"/);
  undecoded.close();
});

// REQ-OPS-37, REQ-OPS-41: an alert that genuinely cleared on exact
// bounds.end_ns (active_at_end === false) emits a cleared event and reads
// active: false at playhead === bounds.end_ns, whereas active_at_end === true
// stays active without a false cleared event.
test('alert clearing on exact bounds.end_ns is not active at end', () => {
  const server = fakeServer(echoSnapshot);
  const timeline: ReviewTimeline = parseTimeline({
    ...EMPTY_TIMELINE,
    alerts: [
      {
        channel: 'hw/status/alert_error_link_lost',
        level: 'error',
        start_ns: (START + 2_000_000_000n).toString(),
        end_ns: END.toString(),
        active_at_end: false
      },
      {
        channel: 'shooter/status/alert_warning_temp_high',
        level: 'warning',
        start_ns: (START + 3_000_000_000n).toString(),
        end_ns: END.toString(),
        active_at_end: true
      }
    ]
  });
  const rec = new RecordedSource(SERVER, 's1', [], BOUNDS, server.fetchFn,
                                 fakeClock().clock, timeline, null);
  const titles = rec.events().map(e => e.title);
  assert.ok(titles.includes('hw: link lost cleared'), titles.join(', '));
  assert.ok(!titles.includes('shooter: temp high cleared'), titles.join(', '));

  // Just before END both alerts are active; at exact END only the held one is.
  rec.seek((END - 1n).toString());
  const beforeEnd = new Map(rec.alertLatch().alerts.map(a => [a.channel, a.active]));
  assert.equal(beforeEnd.get('hw/status/alert_error_link_lost'), true);
  assert.equal(beforeEnd.get('shooter/status/alert_warning_temp_high'), true);

  rec.seek(END.toString());
  const atEnd = new Map(rec.alertLatch().alerts.map(a => [a.channel, a.active]));
  assert.equal(atEnd.get('hw/status/alert_error_link_lost'), false);
  assert.equal(atEnd.get('shooter/status/alert_warning_temp_high'), true);
  rec.close();

  assert.throws(() => parseTimeline({
    ...EMPTY_TIMELINE,
    alerts: [{channel: 'a', level: 'error', start_ns: '1', end_ns: '2', active_at_end: 'no'}]
  }), ReviewError);
});

// REQ-OPS-41, REQ-OPS-23: RecordedSource honors bounds.pose_topic ([studio]
// pose_topic) over discoverEstimate when present, including custom Pose2d field
// names and valid gating, and falls back to discoverEstimate when absent.
test('RecordedSource prefers bounds.pose_topic over discoverEstimate', async () => {
  const channels = [
    'pose_estimator/state/pose/x_m',
    'pose_estimator/state/pose/y_m',
    'pose_estimator/state/pose/yaw_rad',
    'pose_estimator/state/valid',
    'pose_estimator/state/observation_accepted_count',
    'alt_odom/state/robot_pose/x_m',
    'alt_odom/state/robot_pose/y_m',
    'alt_odom/state/robot_pose/yaw_rad',
    'alt_odom/state/valid'
  ].map(info);
  const at = START + 2_000_000_000n;
  let altValid = 1;
  const server = fakeServer(url => {
    if (!url.pathname.endsWith('/snapshot')) return undefined;
    const s = (value: number) => ({value, sample_ns: at.toString()});
    return json({t_ns: at.toString(), channels: {
      'pose_estimator/state/pose/x_m': s(1),
      'pose_estimator/state/pose/y_m': s(2),
      'pose_estimator/state/pose/yaw_rad': s(0.5),
      'pose_estimator/state/valid': s(1),
      'alt_odom/state/robot_pose/x_m': s(7),
      'alt_odom/state/robot_pose/y_m': s(8),
      'alt_odom/state/robot_pose/yaw_rad': s(1.5),
      'alt_odom/state/valid': s(altValid)
    }});
  });

  const configuredBounds = parseBounds({...BOUNDS, pose_topic: '/alt_odom/state'});
  assert.equal(configuredBounds.pose_topic, '/alt_odom/state');
  assert.equal(parseBounds({...BOUNDS, pose_topic: null}).pose_topic, undefined);
  assert.throws(() => parseBounds({...BOUNDS, pose_topic: 42}), ReviewError);

  const configured = new RecordedSource(SERVER, 's1', channels, configuredBounds,
                                        server.fetchFn, fakeClock().clock);
  configured.seek(at.toString());
  await settle();
  assert.equal(configured.chassis()?.pose.x, 7);
  assert.equal(configured.chassis()?.pose.y, 8);
  assert.equal(configured.chassis()?.pose.yaw, 1.5);
  configured.close();

  // valid === 0 on the configured pose_topic suppresses the pose.
  altValid = 0;
  const invalidPose = new RecordedSource(SERVER, 's1', channels, configuredBounds,
                                         server.fetchFn, fakeClock().clock);
  invalidPose.seek(at.toString());
  await settle();
  assert.equal(invalidPose.chassis(), null);
  invalidPose.close();

  // Without pose_topic (or when pose_topic names an absent topic), falls back
  // to discoverEstimate.
  const fallback = new RecordedSource(SERVER, 's1', channels, BOUNDS,
                                      server.fetchFn, fakeClock().clock);
  fallback.seek(at.toString());
  await settle();
  assert.equal(fallback.chassis()?.pose.x, 1);
  assert.equal(fallback.chassis()?.pose.y, 2);
  fallback.close();
});

// REQ-OPS-23, REQ-OPS-41: bucketSeries inserts explicit NaN (count = 1 ->
// plotColumn maps to null) break rows ONLY at bounds.gaps and timeline.runs
// restart/exit boundaries, never on a fixed time threshold, while normal sparse
// multi-channel buckets keep count = 0 (undefined) and draw smoothly.
test('bucketSeries inserts uPlot gap breaks only at real gaps and run restarts', async () => {
  // 1. Sparse multi-channel buckets 8 s apart with NO gaps/runs: NO break rows,
  //    alignment placeholders keep count = 0 (plotColumn -> undefined).
  const sparse = decodeSeries(seriesBuffer([
    {t: [1e9, 9e9], min: [10, 20], max: [10, 20], last: [10, 20], count: [1, 1]},
    {t: [1e9, 5e9], min: [1, 2], max: [1, 2], last: [1, 2], count: [1, 1]}
  ]), 2);
  const noBreaks = bucketSeries(['shooter/state/position_m', 'observer/status/ready'],
                                sparse, '0', 0n, 10_000_000_000n, [], []);
  assert.deepEqual([...noBreaks.times], [1, 5, 9]);
  assert.deepEqual(plotColumn(noBreaks.values['shooter/state/position_m'],
                              noBreaks.count!['shooter/state/position_m']),
                   [10, undefined, 20]);
  assert.deepEqual(plotColumn(noBreaks.values['observer/status/ready'],
                              noBreaks.count!['observer/status/ready']),
                   [1, 2, undefined]);

  // 2. Log gap in shooter [3s, 4s] + shooter crash/restart [6s, 7s]: shooter
  //    gets explicit NaN with count = 1 (plotColumn -> null) at both boundaries,
  //    while observer (no gap/restart) draws smoothly through both rows.
  const gappedBuckets = decodeSeries(seriesBuffer([
    {t: [2e9, NaN, 5e9, 8e9], min: [1, NaN, 2, 3], max: [1, NaN, 2, 3],
     last: [1, NaN, 2, 3], count: [1, 0, 1, 1]},
    {t: [2e9, 3.5e9, 5e9, 8e9], min: [9, 8, 7, 6], max: [9, 8, 7, 6],
     last: [9, 8, 7, 6], count: [1, 1, 1, 1]}
  ]), 2);
  // Note: observer has a real sample at 3.5 s (the exact midpoint of shooter's
  // [3s, 4s] gap) — that real sample must be preserved while shooter breaks!
  const gaps: ReviewGap[] = [
    gap({process: 'shooter', start_ns: '3000000000', end_ns: '4000000000'})
  ];
  const runs = [
    {node: 'observer', start_ns: '0', end_ns: '10000000000', reason: 'stop'},
    {node: 'shooter', start_ns: '0', end_ns: '6000000000', reason: 'crash'},
    {node: 'shooter', start_ns: '7000000000', end_ns: '10000000000', reason: 'stop'}
  ];
  const broken = bucketSeries(['shooter/state/position_m', 'observer/status/ready'],
                              gappedBuckets, '0', 0n, 10_000_000_000n, gaps, runs);
  assert.deepEqual([...broken.times], [2, 3.5, 5, 6.5, 8]);
  assert.deepEqual([...broken.count!['shooter/state/position_m']], [1, 1, 1, 1, 1]);
  assert.deepEqual(plotColumn(broken.values['shooter/state/position_m'],
                              broken.count!['shooter/state/position_m']),
                   [1, null, 2, null, 3]);
  assert.deepEqual(plotColumn(broken.values['observer/status/ready'],
                              broken.count!['observer/status/ready']),
                   [9, 8, 7, undefined, 6]);

  // 3. RecordedSource.series() wires bounds.gaps and timeline.runs end-to-end.
  const t = [Number(START + 2_000_000_000n), Number(START + 8_000_000_000n)];
  const body = seriesBuffer([{t, min: [1, 2], max: [1, 2], last: [1, 2], count: [1, 1]}]);
  const server = fakeServer(url => url.pathname.endsWith('/series') ?
      new Response(body.slice(0)) : echoSnapshot(url));
  const rec = new RecordedSource(
      SERVER, 's1', [info('shooter/state/position_m')], BOUNDS,
      server.fetchFn, fakeClock().clock, SAMPLE_TIMELINE, null);
  rec.series(['shooter/state/position_m'], 'all', END.toString(), 256);
  await settle();
  const s = rec.series(['shooter/state/position_m'], 'all', END.toString(), 256);
  const plotted = plotColumn(s.values['shooter/state/position_m'],
                             s.count!['shooter/state/position_m']);
  assert.deepEqual(plotted, [1, null, 2]);
  rec.close();
});
