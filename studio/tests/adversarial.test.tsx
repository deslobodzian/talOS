// Adversarial & property tests for Studio frontend (src/):
// - Binary telemetry decoder (REQ-CONTRACT-28, REQ-OPS-23)
// - History session-switch reset (REQ-OPS-23)
// - Driver dashboard freshness, sequence monotonicity, dictionary resend,
//   layout bounds/sanitization, summary bounds (REQ-OPS-36, REQ-OPS-37, REQ-OPS-39, REQ-OPS-49)
// - Data parsers: manifest, review, recorded, system (REQ-OPS-23, REQ-OPS-41, REQ-OPS-42)
// - Event detectors, alerts, freshness, formatters, robot status, store, vision
// - Workspace layout depth limit & panel error boundary coverage

import assert from 'node:assert/strict';
import {test} from 'node:test';
import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';
import {Builder} from 'flatbuffers';

import {
  Coordinator,
  dictionaryKey,
  exportPages,
  JitterBuffer,
  MAX_DICTIONARY_STRING_BYTES,
  type Snapshot,
  TelemetryDecoder,
} from '../src/core';
import {AlertLevel, ChannelKind, concatPages, encodeDictionaryPages, encodeValuePages, rawFromDouble} from '../src/encode';
import {ChannelInfo} from '../src/generated/talos/telemetry/channel-info';
import {Frame} from '../src/generated/talos/telemetry/frame';
import {TelemetryHistory} from '../src/history';
import {AlertTracker} from '../src/data/alerts';
import {createFrameDetectors} from '../src/data/detectors';
import {EventEngine} from '../src/data/events';
import {graphStale} from '../src/data/freshness';
import {parseManifest} from '../src/data/manifest';
import {RecordedSource} from '../src/data/recorded';
import {decodeSeries, parseBounds, type ReviewBounds} from '../src/data/review';
import {canBusReading, robotStatus} from '../src/data/robot';
import {TelemetryStore} from '../src/data/store';
import {createStaticStudio, StudioProvider} from '../src/data/studio';
import type {SystemState} from '../src/data/types';
import {
  CONNECTING_GRACE_MS,
  DriveModel,
  type DriveSocket,
  parseSummary,
  STALE_MS,
} from '../src/drive/feed';
import {parseLayout} from '../src/drive/layout';
import {renderWidgets} from '../src/drive/view';
import {parseWorkspace} from '../src/layout/tree';
import {robotPanel} from '../src/panels/robot';
import {observationErrors} from '../src/panels/vision/logic';
import {StatusStrip} from '../src/shell/StatusStrip';
import {RateWindow, SystemGraphStream, type SystemGraph} from '../src/system';
import {hertz, integer} from '../src/ui/formatters';

const ZERO_POSE = {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0};
const DS = 'hw/state/driver_station/';

function makeRng(seed: number) {
  let s = seed >>> 0;
  return () => {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    return s;
  };
}

function makeSnapshot(seq: number, ts: bigint, channels: Record<string, number>,
                      opts: {streamId?: string; labels?: Record<string, string[]>;
                             kinds?: Record<string, ChannelKind>;
                             alerts?: Record<string, AlertLevel>} = {}): Snapshot {
  return {
    stream_id: opts.streamId ?? '1',
    sequence_id: String(seq),
    timestamp_ns: String(ts),
    dictionary_id: 1,
    chassis: ZERO_POSE,
    channels,
    units: Object.create(null),
    labels: opts.labels ?? Object.create(null),
    ...(opts.kinds ? {kinds: opts.kinds} : {}),
    ...(opts.alerts ? {alerts: opts.alerts} : {}),
  };
}

function buildRawDictPage(opts: {
  streamId?: bigint; id: number; page: number; pageCount: number;
  channelCount: number; firstChannel: number; names: string[];
  absent?: number[];
}): Uint8Array {
  const b = new Builder(256);
  const entries = opts.names.map(name => {
    const n = b.createString(name);
    ChannelInfo.startChannelInfo(b);
    ChannelInfo.addName(b, n);
    ChannelInfo.addKind(b, ChannelKind.Double);
    return ChannelInfo.endChannelInfo(b);
  });
  const dict = Frame.createDictionaryVector(b, entries);
  const absent = opts.absent ? Frame.createAbsentVector(b, opts.absent) : 0;
  Frame.startFrame(b);
  Frame.addStreamId(b, opts.streamId ?? 1n);
  Frame.addSequenceId(b, 0n);
  Frame.addTimestampNs(b, 1n);
  Frame.addDictionaryId(b, opts.id);
  Frame.addPage(b, opts.page);
  Frame.addPageCount(b, opts.pageCount);
  Frame.addChannelCount(b, opts.channelCount);
  Frame.addFirstChannel(b, opts.firstChannel);
  Frame.addDictionary(b, dict);
  if (absent) Frame.addAbsent(b, absent);
  Frame.finishSizePrefixedFrameBuffer(b, Frame.endFrame(b));
  return b.asUint8Array().slice();
}

// --- 1. Binary telemetry decoder & property tests ---------------------------

// REQ-CONTRACT-28, REQ-OPS-23: dictionary pages reject absent vectors,
// channel counts past kMaxChannels (2048), and empty channel names.
test('TelemetryDecoder rejects dictionary absent vectors, >2048 channels, and empty names', () => {
  const d = new TelemetryDecoder();
  assert.throws(() => d.decode(buildRawDictPage({
    id: 1, page: 0, pageCount: 1, channelCount: 1, firstChannel: 0,
    names: ['a/state/x'], absent: [0]
  })), /Dictionary page with values/);

  assert.throws(() => d.decode(buildRawDictPage({
    id: 2, page: 0, pageCount: 20, channelCount: 2049, firstChannel: 0,
    names: ['a/state/x']
  })), /Too many channels/);

  assert.throws(() => d.decode(buildRawDictPage({
    id: 3, page: 0, pageCount: 1, channelCount: 1, firstChannel: 0,
    names: ['']
  })), /Empty channel name/);

  assert.throws(() => new JitterBuffer(Number.NaN), /Invalid jitter delay/);
});

// REQ-CONTRACT-28, REQ-OPS-23: seeded mutation fuzzing never throws non-Error /
// OOB RangeError or corrupts an already-initialized decoder's delta state;
// encode -> decode -> export -> decode roundtrips all kinds & edge values.
test('TelemetryDecoder fuzz mutations preserve prior state and roundtrip property holds', () => {
  const rng = makeRng(0x5eed29c);
  const channels = [
    {name: 'sys/state/d', unit: 'm/s', kind: ChannelKind.Double},
    {name: 'sys/state/i', unit: '', kind: ChannelKind.Int},
    {name: 'sys/state/u', unit: '', kind: ChannelKind.UInt, labels: ['off', 'on']},
    {name: 'sys/status/alert_error_fault', unit: '', kind: ChannelKind.Bool, alert: AlertLevel.Error},
  ];
  const header = (seq: bigint) => ({
    streamId: 9n, sequenceId: seq, timestampNs: seq * 1000n,
    dictionaryId: 77, channelCount: channels.length
  });
  const dict = encodeDictionaryPages(header(0n), channels);
  const initKf = encodeValuePages(header(1n), {x: 1.5, y: -2.25, z: 0, roll: 0, pitch: 0, yaw: 0.5}, true, [
    [0, rawFromDouble(42.5)],
    [1, BigInt.asUintN(64, -9223372036854775808n)],
    [2, 18446744073709551615n],
    [3, 1n],
  ]);

  const decoder = new TelemetryDecoder();
  for (const p of dict) decoder.decode(p);
  const baseline = decoder.decode(concatPages(initKf))!;

  // Mutate valid pages 250 times with random truncations & bit flips.
  const corpus = [...dict, ...initKf,
    ...encodeValuePages(header(2n), ZERO_POSE, false, [[0, rawFromDouble(1)]], [3])];
  for (let i = 0; i < 250; i++) {
    const src = corpus[rng() % corpus.length];
    const mode = rng() % 4;
    let mutated: Uint8Array;
    if (mode === 0) {
      mutated = src.slice(0, rng() % src.length);
    } else if (mode === 1) {
      mutated = src.slice();
      const flips = 1 + (rng() % 6);
      for (let f = 0; f < flips; f++) {
        mutated[rng() % mutated.length] ^= 1 << (rng() % 8);
      }
    } else if (mode === 2) {
      mutated = new Uint8Array(rng() % 64);
      for (let j = 0; j < mutated.length; j++) mutated[j] = rng() & 0xff;
    } else {
      mutated = src.slice();
      new DataView(mutated.buffer).setUint32(0, rng(), true);
    }
    try {
      const fresh = new TelemetryDecoder();
      fresh.decodeAll(mutated);
    } catch (e) {
      assert.ok(e instanceof Error);
      assert.notEqual(e.name, 'RangeError', `unexpected RangeError on mutation ${i}: ${e.message}`);
    }
    try {
      decoder.decodeAll(mutated);
    } catch (e) {
      assert.ok(e instanceof Error);
      assert.notEqual(e.name, 'RangeError', `unexpected RangeError on live decoder ${i}: ${e.message}`);
    }
  }
  // Decoder can still decode a clean keyframe on the same stream afterwards.
  const check = decoder.decode(concatPages(initKf))!;
  assert.equal(check.channels['sys/state/d'], baseline.channels['sys/state/d']);
  assert.equal(check.exact!['sys/state/i'], '-9223372036854775808');
  assert.equal(check.exact!['sys/state/u'], '18446744073709551615');

  // Roundtrip property over edge double / int / uint / bool values & absence.
  const edgeDoubles = [0, -0, 1, -1, Number.NaN, Infinity, -Infinity, 1e-300, 1e300];
  const edgeInts = [0n, 1n, -1n, 9007199254740993n, -9007199254740993n,
    9223372036854775807n, -9223372036854775808n];
  const edgeUints = [0n, 1n, 9007199254740993n, 18446744073709551615n];
  const rtDecoder = new TelemetryDecoder();
  for (const p of dict) rtDecoder.decode(p);
  const snapshots: Snapshot[] = [];
  for (let i = 0; i < 40; i++) {
    const samples: [number, bigint][] = [];
    if (rng() % 5 !== 0) samples.push([0, rawFromDouble(edgeDoubles[rng() % edgeDoubles.length])]);
    if (rng() % 5 !== 0) samples.push([1, BigInt.asUintN(64, edgeInts[rng() % edgeInts.length])]);
    if (rng() % 5 !== 0) samples.push([2, edgeUints[rng() % edgeUints.length]]);
    if (rng() % 5 !== 0) samples.push([3, BigInt(rng() % 2)]);
    const pages = encodeValuePages(header(BigInt(i + 1)), ZERO_POSE, true, samples);
    snapshots.push(rtDecoder.decode(concatPages(pages))!);
  }
  const exported = concatPages(exportPages(
      snapshots, new Map([[dictionaryKey(snapshots[0]), rtDecoder.currentDictionary()!]])));
  const reimported = new TelemetryDecoder().decodeAll(exported);
  assert.equal(reimported.length, snapshots.length);
  for (let i = 0; i < snapshots.length; i++) {
    const a = snapshots[i], b = reimported[i];
    assert.deepEqual(Object.keys(b.channels).sort(), Object.keys(a.channels).sort());
    for (const k of Object.keys(a.channels)) {
      assert.ok(Object.is(b.channels[k], a.channels[k]) ||
                (Number.isNaN(a.channels[k]) && Number.isNaN(b.channels[k])),
                `frame ${i} channel ${k}: ${b.channels[k]} vs ${a.channels[k]}`);
    }
    assert.deepEqual({...b.exact}, {...a.exact});
    assert.deepEqual({...b.alerts}, {...a.alerts});
  }
  assert.equal(MAX_DICTIONARY_STRING_BYTES, 256);
});

// --- 2. TelemetryHistory session switch clears lastArrivalMs ----------------

// REQ-OPS-23: replacing history with empty kept list clears timeline.lastArrivalMs.
test('TelemetryHistory clears lastArrivalMs when session switch drops all frames', () => {
  const history = new TelemetryHistory({arrival: () => 12345});
  const graphFor = (sessionId: string): SystemGraph => ({
    version: 3, wall_ns: '1000',
    registry: {available: true, node_capacity: 32, source_capacity: 64, liveness_timeout_ns: '5000000000'},
    bridge: {topic: '/talos/telemetry', clients: 1, frames_forwarded: '0', invalid: '0', ws_drops: '0'},
    nodes: [{
      name: 'telemetry', target: 't', slot: 0, pid: '1', session_id: sessionId, generation: '1',
      start_wall_ns: '0', heartbeat_wall_ns: '0', dispatch_count: '1', declared_sources: 0,
      alive: true, simulation: false, replay: false, sources: [], selections: []
    }],
    topics: [{
      name: '/talos/telemetry', message_bytes: 16384, publishers: ['telemetry'],
      subscribers: [], observers: [], published: '1', received: '0', dropped: '0'
    }]
  });
  history.push(makeSnapshot(1, 1000n, {}), null, 0);
  history.flush(20);
  assert.equal(history.timeline.lastArrivalMs, 12345);
  history.observe(graphFor('1'));
  history.observe(graphFor('2'));
  assert.equal(history.timeline.frames.length, 0);
  assert.equal(history.timeline.lastArrivalMs, 0);
});

// --- 3. Driver dashboard safety, staleness, dictionary & layout hardening ---

function makeFakeSocket(): DriveSocket & {
  closed: boolean;
  send(...frames: (Uint8Array|string)[]): void;
} {
  const sock: DriveSocket & {
    closed: boolean;
    send(...frames: (Uint8Array|string)[]): void;
  } = {
    binaryType: '',
    onopen: null,
    onmessage: null,
    onclose: null,
    onerror: null,
    closed: false,
    close() { this.closed = true; },
    send(...frames) {
      for (const f of frames) {
        if (typeof f === 'string') this.onmessage?.({data: f});
        else this.onmessage?.({data: f.slice().buffer});
      }
    }
  };
  return sock;
}

// REQ-OPS-36: an out-of-order (decreasing sequence_id) frame on the same socket
// must not roll back latest or let a subsequent heartbeat falsely refresh fresh().
test('DriveModel ignores decreasing sequence_id on the same socket', () => {
  const channels = [{name: 'hw/status/battery_v', unit: 'V', kind: ChannelKind.Double}];
  const hdr = (seq: number) => ({
    streamId: 1n, sequenceId: BigInt(seq), timestampNs: BigInt(seq) * 50_000_000n,
    dictionaryId: 1, channelCount: 1
  });
  const socks: ReturnType<typeof makeFakeSocket>[] = [];
  let now = 0;
  const model = new DriveModel('ws://test/drive', () => {
    const s = makeFakeSocket();
    socks.push(s);
    return s;
  });
  model.clock = () => now;
  model.start(0);
  socks[0].onopen!();
  socks[0].send(
      ...encodeDictionaryPages(hdr(10), channels),
      ...encodeValuePages(hdr(10), ZERO_POSE, true, [[0, rawFromDouble(12.5)]]));
  now = 50;
  socks[0].send(...encodeValuePages(hdr(11), ZERO_POSE, false, [[0, rawFromDouble(12.4)]]));
  assert.equal(model.fresh(50), true);
  assert.equal(model.latest!.sequence_id, '11');

  // Producer stops; 400 ms later (> STALE_MS) an out-of-order seq 5 arrives,
  // followed by the bridge repeating seq 11 as a heartbeat.
  now = 450;
  socks[0].send(...encodeValuePages(hdr(5), ZERO_POSE, false, [[0, rawFromDouble(9.0)]]));
  assert.equal(model.latest!.sequence_id, '11', 'older sequence must not overwrite latest');
  assert.equal(model.latest!.channels['hw/status/battery_v'], 12.4);
  now = 500;
  socks[0].send(...encodeValuePages(hdr(11), ZERO_POSE, false, []));
  assert.equal(model.fresh(500), false, 'repeated seq 11 after out-of-order seq 5 must stay stale');
});

// REQ-OPS-36: a repeated dictionary page before any value page must not count
// as a decoded value page (valuesThisSocket).
test('DriveModel dictionary resend before value page stays within grace window', () => {
  const channels = [{name: 'hw/status/battery_v', unit: 'V', kind: ChannelKind.Double}];
  const hdr = {streamId: 1n, sequenceId: 0n, timestampNs: 1n, dictionaryId: 1, channelCount: 1};
  const dict = encodeDictionaryPages(hdr, channels);
  const socks: ReturnType<typeof makeFakeSocket>[] = [];
  let now = 0;
  const model = new DriveModel('ws://test/drive', () => {
    const s = makeFakeSocket();
    socks.push(s);
    return s;
  });
  model.clock = () => now;
  model.start(0);
  socks[0].onopen!();
  // Two dictionary pages arrive (e.g. periodic dictionary resend), no value page yet.
  socks[0].send(...dict, ...dict);
  // A corrupt binary message within CONNECTING_GRACE_MS must wait out grace,
  // not immediately tear down the socket as if a value page had decoded.
  now = 100;
  socks[0].send(new Uint8Array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]));
  assert.equal(socks[0].closed, false, 'dictionary resend must not set valuesThisSocket');
  assert.equal(model.connection, 'open');
  // Once grace expires without a valid value page, tick() closes it.
  now = CONNECTING_GRACE_MS + 1;
  model.tick(now);
  assert.equal(socks[0].closed, true);
});

// REQ-OPS-39, REQ-OPS-49: layout parser sanitizes non-positive window_s /
// negative trail_s / out-of-grid cells so graphWidget never emits NaN SVG coords
// and out-of-grid cells are counted as skipped rather than creating giant grids.
test('parseLayout sanitizes non-positive window_s/trail_s and skips out-of-grid cells', () => {
  const layout = parseLayout({
    columns: 4, rows: 2, update_hz: -5,
    widgets: [
      {kind: 'graph', at: [0, 0, 2, 1], title: 'G', channels: {},
       series: ['v'], window_s: 0},
      {kind: 'field', at: [2, 0, 2, 1], title: 'F',
       channels: {x: 'x', y: 'y', yaw: 'yaw'}, trail_s: -3},
      {kind: 'number', at: [100, 100, 50, 50], title: 'OOB', channels: {channel: 'v'}},
      {kind: 'number', at: [3, 0, 2, 1], title: 'Overflow', channels: {channel: 'v'}},
    ]
  });
  assert.equal(layout.update_hz, 20);
  assert.equal(layout.skipped, 2);
  assert.equal(layout.widgets.length, 2);
  const g = layout.widgets[0];
  assert.equal(g.kind, 'graph');
  assert.ok(g.window_s > 0);
  const f = layout.widgets[1];
  assert.equal(f.kind, 'field');
  assert.ok(f.trail_s >= 0);

  // Render the graph with two samples and verify zero NaN/Infinity in SVG output.
  const channels = [{name: 'v', unit: '', kind: ChannelKind.Double}];
  const hdr = (seq: number) => ({
    streamId: 1n, sequenceId: BigInt(seq), timestampNs: BigInt(seq) * 100_000_000n,
    dictionaryId: 1, channelCount: 1
  });
  const sock = makeFakeSocket();
  const model = new DriveModel('ws://test/drive', () => sock, ['v']);
  model.clock = () => 100;
  model.start(0);
  sock.onopen!();
  sock.send(
      ...encodeDictionaryPages(hdr(1), channels),
      ...encodeValuePages(hdr(1), ZERO_POSE, true, [[0, rawFromDouble(1)]]),
      ...encodeValuePages(hdr(2), ZERO_POSE, false, [[0, rawFromDouble(2)]]));
  const html = renderWidgets(layout, model, 100);
  assert.ok(!html.includes('NaN') && !html.includes('Infinity'), html);
});

// REQ-OPS-36: parseSummary clamps non-finite omitted and caps nodes count.
test('parseSummary rejects excessive nodes and ignores non-finite omitted', () => {
  const s = parseSummary({
    registry: true, supervised: true, omitted: Infinity, nodes: []
  });
  assert.ok(s);
  assert.equal(s.omitted, 0);

  const huge = parseSummary({
    registry: true, supervised: true,
    nodes: Array.from({length: 300}, (_, i) => ({name: `n${i}`}))
  });
  assert.equal(huge, null);
});

// --- 4. Data parsers: manifest, review, recorded, system --------------------

// REQ-OPS-23, REQ-OPS-42: manifest rejects 1e25 floats and >uint64 strings.
test('parseManifest rejects huge exponential integers and >uint64 strings', () => {
  const base = {
    session_id: '1', config_path: '/c', config_sha256: 'a', output_dir: '/o',
    simulation: false, start_wall_ns: '1', end_wall_ns: '2',
    nodes: [{
      name: 'n', target: 't', binary_path: 'b', log_path: 'l', log_paths: ['l'],
      pid: 1, exit_code: 0, restart_count: 0, record_count: 0
    }]
  };
  assert.throws(() => parseManifest(JSON.stringify({...base, session_id: 1e25})), /decimal integer/);
  assert.throws(() => parseManifest(JSON.stringify({...base, start_wall_ns: '99999999999999999999'})), /decimal integer/);
  assert.throws(() => parseManifest(JSON.stringify({
    ...base, nodes: [{...base.nodes[0], record_count: 1e25}]
  })), /non-negative integer/);
  assert.throws(() => parseManifest(JSON.stringify({
    ...base, nodes: [{...base.nodes[0], runs: [], runs_dropped: 1e25}]
  })), /non-negative integer/);
  assert.throws(() => parseManifest(JSON.stringify({
    ...base, nodes: [{...base.nodes[0], runs: [{
      pid: 1, start_wall_ns: 1e25, end_wall_ns: 0, exit_code: 0, exit_signal: 0, reason: 'exit'
    }]}]
  })), /non-negative integer/);
});

// REQ-OPS-41, REQ-OPS-42: review u64 rejects >uint64 / 1e100; decodeSeries
// rejects channels=0 with nonzero buckets.
test('review parsers reject >uint64 numbers and degenerate decodeSeries headers', () => {
  const validBounds = {
    start_ns: '0', end_ns: '100', origin_wall_ns: '0',
    max_series_values: 1000, max_series_channels: 8
  };
  assert.throws(() => parseBounds({...validBounds, end_ns: 1e100}), /non-negative integer/);
  assert.throws(() => parseBounds({...validBounds, end_ns: '99999999999999999999'}), /non-negative integer/);

  const buf = new ArrayBuffer(8);
  const view = new DataView(buf);
  view.setUint32(0, 0, true);
  view.setUint32(4, 0xffffffff, true);
  assert.throws(() => decodeSeries(buf, 0), /buckets/);
});

// REQ-OPS-42: RecordedSource.series caps points to bounds.max_series_values.
test('RecordedSource caps series points to bounds.max_series_values', () => {
  const requestedUrls: string[] = [];
  const bounds: ReviewBounds = {
    start_ns: '0', end_ns: '10000000000', origin_wall_ns: '0',
    max_series_values: 200, max_series_channels: 8, gaps: []
  };
  const source = new RecordedSource(
      'http://127.0.0.1:5805', 's1',
      [{name: 'a', unit: '', labels: []}], bounds,
      async url => {
        requestedUrls.push(url);
        return new Response(new ArrayBuffer(8), {status: 200});
      },
      {now: () => 0, every: () => () => {}});
  source.series(['a'], {seconds: 5}, '5000000000', 4096);
  const seriesUrl = requestedUrls.find(u => u.includes('/series?'));
  assert.ok(seriesUrl, 'expected a series request');
  const points = Number(new URL(seriesUrl).searchParams.get('points'));
  assert.ok(points <= bounds.max_series_values, `requested ${points} > ${bounds.max_series_values}`);
  source.close();
});

// REQ-OPS-23: RateWindow rejects Infinity; SystemGraphStream rejects updates
// when the held graph carried no structure token.
test('RateWindow rejects Infinity seconds and SystemGraphStream requires structure string', () => {
  assert.throws(() => new RateWindow(Infinity), /Invalid rate window/);
  const stream = new SystemGraphStream();
  const baseGraph = {
    kind: 'talos.system_graph', version: 3, wall_ns: '100',
    registry: {available: true, node_capacity: 32, source_capacity: 64, liveness_timeout_ns: '5000000000'},
    bridge: {topic: '/talos/telemetry', clients: 0, frames_forwarded: '0', invalid: '0', ws_drops: '0'},
    nodes: [], topics: []
  };
  stream.ingest(JSON.stringify(baseGraph), 0);
  assert.throws(() => stream.ingest(JSON.stringify({
    kind: 'talos.system_graph.deferred'
  }), 100), /structure not held/);
});

// --- 5. Detectors, alerts, freshness, formatters, robot status, store, vision ---

// REQ-OPS-23, REQ-OPS-37: detectors never spam on NaN, close runs when channels
// go absent, and ignore negative masks.
test('detectors ignore NaN/Inf/negative noise and close absent counter runs', () => {
  const engine = new EventEngine();
  const labels = {mode: ['off', 'on']};
  engine.pushFrame(makeSnapshot(1, 1000n, {
    mode: Number.NaN,
    'hw/status/brownout': Number.NaN,
    'hw/status/motor_fault_mask': 0,
    'hw/status/gateway_rx_count': 1
  }, {labels}));
  engine.pushFrame(makeSnapshot(2, 2000n, {
    mode: Number.NaN,
    'hw/status/brownout': Number.NaN,
    'hw/status/motor_fault_mask': -1,
    'hw/status/gateway_rx_count': Infinity
  }, {labels}));
  assert.equal(engine.events.length, 0, `unexpected events: ${JSON.stringify(engine.events)}`);

  // 0 -> NaN on boolEdge must not emit a false "cleared" event.
  engine.pushFrame(makeSnapshot(3, 3000n, {'hw/status/brownout': 0}));
  engine.pushFrame(makeSnapshot(4, 4000n, {'hw/status/brownout': Number.NaN}));
  assert.equal(engine.events.filter(e => e.id.startsWith('power-brownout:')).length, 0);

  // An active counter run whose channel disappears must stop after STOP_AFTER_NS.
  const runEngine = new EventEngine();
  runEngine.pushFrame(makeSnapshot(1, 0n, {'hw/status/clock_resets_count': 1}));
  runEngine.pushFrame(makeSnapshot(2, 100_000_000n, {'hw/status/clock_resets_count': 2}));
  runEngine.pushFrame(makeSnapshot(3, 1_200_000_000n, {}));  // channel absent past 1 s
  const clockEvents = runEngine.events.filter(e => e.id.startsWith('counter-clock-resets:'));
  assert.equal(clockEvents.length, 2, 'absent counter run must emit stop and release state');
});

// REQ-OPS-37: NaN enabled does not wipe latched alerts; removing an alert from
// the dictionary clears its latched active flag; level change updates active().
test('AlertTracker ignores NaN enabled and clears active when dictionary drops an alert', () => {
  const A = 'hw/status/alert_error_link_lost';
  const tracker = new AlertTracker();
  tracker.push({
    timestamp_ns: '100',
    channels: {[DS + 'enabled']: 0, [A]: 1},
    alerts: {[A]: AlertLevel.Error}
  });
  assert.equal(tracker.latch().alerts.length, 1);
  assert.equal(tracker.latch().alerts[0].active, true);

  // NaN enabled must not trigger a false match start and wipe the latch.
  tracker.push({
    timestamp_ns: '200',
    channels: {[DS + 'enabled']: Number.NaN, [A]: 1},
    alerts: {[A]: AlertLevel.Error}
  });
  assert.equal(tracker.latch().matchStart_ns, null);
  assert.equal(tracker.latch().alerts.length, 1);

  // Dictionary changes and no longer declares A: latched entry must read active=false.
  tracker.push({
    timestamp_ns: '300',
    channels: {[DS + 'enabled']: 0},
    alerts: {}
  });
  assert.equal(tracker.active().length, 0);
  assert.equal(tracker.latch().alerts[0].active, false);

  // Level change on the same alert channel must update active().
  tracker.push({
    timestamp_ns: '400',
    channels: {[DS + 'enabled']: 0, [A]: 1},
    alerts: {[A]: AlertLevel.Warning}
  });
  assert.equal(tracker.active()[0].level, AlertLevel.Warning);
  tracker.push({
    timestamp_ns: '500',
    channels: {[DS + 'enabled']: 0, [A]: 1},
    alerts: {[A]: AlertLevel.Error}
  });
  assert.equal(tracker.active()[0].level, AlertLevel.Error);
});

// REQ-OPS-23: non-finite timestamps in graphStale fail closed; non-finite
// values in formatters, robotStatus, StatusStrip, RobotPanel, store, vision
// never throw or format NaN/Infinity/-1:-1.
test('non-finite values fail closed across freshness, formatters, robot status, store and vision', () => {
  assert.equal(graphStale(Number.NaN, 1000, true), true);
  assert.equal(graphStale(0, Number.NaN, true), true);

  assert.equal(integer(Number.NaN), '—');
  assert.equal(integer(Infinity), '—');
  assert.equal(integer('not-an-int'), '—');
  assert.equal(hertz(Number.NaN), '—');
  assert.equal(hertz(Infinity), '—');

  const noSystem: SystemState =
      {graph: null, stale: true, rates: {}, error: '', declared: null, declaredError: ''};
  const status = robotStatus(makeSnapshot(1, 1000n, {
    [DS + 'enabled']: Number.NaN,
    [DS + 'match_time_s']: -1,
    'hw/status/brownout': Number.NaN,
    'hw/status/configured': Number.NaN,
    'hw/status/can_bus_count': 1,
    'hw/status/can_buses/0/read_state': 1,
    'hw/status/can_buses/0/utilization_frac': Number.NaN,
  }), noSystem);
  assert.equal(status.ds.enabled, null);
  assert.equal(status.power.brownout, null);
  assert.equal(status.link.configured, null);
  assert.equal(canBusReading(status.power.canBuses[0]), '—');

  // StatusStrip and RobotPanel render '—' for negative or NaN match_time_s.
  const studio = createStaticStudio({
    frames: [makeSnapshot(1, 1000n, {
      [DS + 'connected']: 1, [DS + 'enabled']: 0, [DS + 'match_time_s']: -1
    })]
  });
  const stripHtml = renderToStaticMarkup(
      <StudioProvider studio={studio}><StatusStrip/></StudioProvider>);
  assert.ok(!stripHtml.includes('-1:-1'), stripHtml);
  const robotHtml = renderToStaticMarkup(
      <StudioProvider studio={studio}>
        <robotPanel.component config={{}} setConfig={() => {}} instanceId="r"/>
      </StudioProvider>);
  assert.ok(!robotHtml.includes('-1:-1'), robotHtml);

  // TelemetryStore.reconstruct does not throw RangeError on NaN in an Int column.
  const store = new TelemetryStore(() => {});
  store.ingest([makeSnapshot(1, 1000n, {n: Number.NaN}, {kinds: {n: ChannelKind.Int}})], true);
  const reconstructed = store.snapshotAt('1000');
  assert.ok(reconstructed);
  assert.ok(Number.isNaN(reconstructed.channels.n));

  // observationErrors without refTimes skips non-finite timestamps.
  const errs = observationErrors(
      [Number.NaN, 1], [1, 2], [1, 2], [1, 1], [0, 0], [0, 0]);
  assert.equal(errs.length, 1);
  assert.equal(errs[0].t, 1);
});

// --- 6. Workspace layout depth cap ------------------------------------------

test('parseWorkspace bounds deeply nested split trees without stack overflow', () => {
  let node: Record<string, unknown> = {type: 'panel', instanceId: 'leaf', panelId: 'health'};
  for (let i = 0; i < 500; i++) {
    node = {type: 'split', direction: 'row', children: [node], sizes: [1]};
  }
  const ws = parseWorkspace({
    version: 1, activePage: 'p', pages: [{id: 'p', title: 'Deep', root: node}]
  });
  assert.equal(ws.pages.length, 0);
});
