import assert from 'node:assert/strict';
import {test} from 'node:test';

import {Builder, ByteBuffer} from 'flatbuffers';

import {
  channelDisplay,
  Coordinator,
  JitterBuffer,
  MAX_ENUM_LABELS,
  MAX_FRAME_BYTES,
  poseError,
  simTruthPose,
  TelemetryDecoder,
  Timeline,
} from '../src/core';
import {demoPacket, demoSystemGraph} from '../src/demo';
import {ChannelKind, encodeDictionaryPages, encodeValuePages, rawFromDouble} from '../src/encode';
import {ChannelInfo} from '../src/generated/talos/telemetry/channel-info';
import {Frame} from '../src/generated/talos/telemetry/frame';
import {Pose} from '../src/generated/talos/telemetry/pose';
import {createFrameDetectors, SYSTEM_DETECTORS} from '../src/data/detectors';
import {parseSystemGraph} from '../src/system';

const frame = (i: number) => {
  const decoder = new TelemetryDecoder();
  decoder.decode(demoPacket(0n));
  return decoder.decode(demoPacket(BigInt(i)))!;
};

const DEMO_CHANNEL_COUNT = 11;

// One keyframe value page for `dictionaryId` (a number is a double sample,
// NaN and the infinities included; null is absent; a bigint is an integer
// channel's raw sample), preceded by a
// dictionary page when `options.dictionary` names its channels. Options also
// support one label vector every dictionary entry shares, per-channel kinds,
// and chassis headings for decoder-level tests.
function valueFrame(sequence: bigint, dictionaryId: number,
                    values: (number|bigint|null)[], streamId = 42n,
                    options: {dictionary?: string[]; labels?: string[];
                              kinds?: ChannelKind[]; yaw?: number;
                              missingChassis?: boolean} = {}): Uint8Array {
  const count = options.dictionary?.length ??
      (dictionaryId === 1 ? DEMO_CHANNEL_COUNT : values.length);
  const b = new Builder(256);
  const header = (frameBuilder: Builder) => {
    Frame.addStreamId(frameBuilder, streamId);
    Frame.addSequenceId(frameBuilder, sequence);
    Frame.addTimestampNs(frameBuilder, 1000000000n + sequence * 20000000n);
    Frame.addDictionaryId(frameBuilder, dictionaryId);
    Frame.addPageCount(frameBuilder, 1);
    Frame.addChannelCount(frameBuilder, count);
  };
  const pages: Uint8Array[] = [];
  if (options.dictionary) {
    const labels = options.labels &&
        ChannelInfo.createLabelsVector(
            b, options.labels.map(label => b.createString(label)));
    const dictionary = Frame.createDictionaryVector(
        b, options.dictionary.map((name, i) => {
          const text = b.createString(name);
          ChannelInfo.startChannelInfo(b);
          ChannelInfo.addName(b, text);
          if (labels !== undefined) ChannelInfo.addLabels(b, labels);
          ChannelInfo.addKind(b, options.kinds?.[i] ?? ChannelKind.Double);
          return ChannelInfo.endChannelInfo(b);
        }));
    Frame.startFrame(b);
    header(b);
    Frame.addDictionary(b, dictionary);
    Frame.finishSizePrefixedFrameBuffer(b, Frame.endFrame(b));
    pages.push(b.asUint8Array().slice());
    b.clear();
  }
  const carried = values.flatMap((value, i): [number, bigint][] =>
      typeof value === 'bigint' ? [[i, value]] :
      value === null            ? [] :
                                  [[i, rawFromDouble(value)]]);
  const channels = Frame.createChannelsVector(b, carried.map(([i]) => i));
  const raws = Frame.createValuesVector(b, carried.map(([, raw]) => raw));
  Frame.startFrame(b);
  header(b);
  Frame.addKeyframe(b, true);
  Frame.addChannels(b, channels);
  Frame.addValues(b, raws);
  if (!options.missingChassis)
    Frame.addChassis(
        b, Pose.createPose(b, 1, 2, 0, 0, 0, options.yaw ?? 0));
  Frame.finishSizePrefixedFrameBuffer(b, Frame.endFrame(b));
  pages.push(b.asUint8Array().slice());
  const out = new Uint8Array(pages.reduce((n, page) => n + page.length, 0));
  let p = 0;
  for (const page of pages) {
    out.set(page, p);
    p += page.length;
  }
  return out;
}

test(
    'size-prefixed generated FlatBuffer roundtrip preserves uint64 timestamps',
    () => {
      const decoder = new TelemetryDecoder();
      assert.ok(decoder.decode(demoPacket(0n)));
      const f = decoder.decode(valueFrame(
          9007199254740993n, 1,
          [1.75, 0, 12, 2, 0, 0, 0, 1n]))!;
      assert.equal(f.sequence_id, '9007199254740993');
      assert.equal(f.stream_id, '42');
      assert.equal(f.dictionary_id, 1);
      assert.equal(
          f.timestamp_ns,
          BigInt.asUintN(64, 1000000000n + 9007199254740993n * 20000000n)
              .toString());
      assert.equal(f.channels.velocity, 1.75);
      assert.equal(f.units.velocity, 'm/s');
      assert.deepEqual(f.labels.mode, ['disabled', 'teleop', 'auto']);
      assert.equal(channelDisplay(f, 'mode'), 'teleop');
    });
// No channel cap: an update spread over several pages is one snapshot, at
// its last page, carrying every channel; integers keep every digit.
test('a paged update decodes to one snapshot with exact integers', () => {
  const channels = Array.from({length: 200}, (_, i) => ({
    name: `test/state/channel_${i}`, unit: '',
    kind: i === 0 ? ChannelKind.UInt : i === 1 ? ChannelKind.Int :
                                         ChannelKind.Double
  }));
  const header = {streamId: 7n, sequenceId: 5n, timestampNs: 9n,
                  dictionaryId: 3, channelCount: channels.length};
  const samples: [number, bigint][] = channels.map((_, i) => [
    i, i === 0 ? 2n ** 64n - 1n :
       i === 1 ? BigInt.asUintN(64, -(2n ** 63n)) :
                 rawFromDouble(i + .5)
  ]);
  const pages = encodeValuePages(
      header, {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0}, true, samples);
  assert.ok(pages.length >= 3);
  const decoder = new TelemetryDecoder();
  for (const page of encodeDictionaryPages(header, channels))
    assert.equal(decoder.decode(page), null);
  for (const page of pages.slice(0, -1))
    assert.equal(decoder.decode(page), null);
  const snapshot = decoder.decode(pages.at(-1)!)!;
  assert.equal(Object.keys(snapshot.channels).length, 200);
  assert.equal(snapshot.channels['test/state/channel_199'], 199.5);
  assert.equal(snapshot.exact!['test/state/channel_0'], '18446744073709551615');
  assert.equal(channelDisplay(snapshot, 'test/state/channel_0'),
               '18446744073709551615');
  assert.equal(channelDisplay(snapshot, 'test/state/channel_1'),
               '-9223372036854775808');
});

// The bridge's change-only updates: what a page does not carry keeps its
// value, an absent channel loses it, and nothing applies before the keyframe
// it builds on.
test('change-only updates apply onto the keyframe', () => {
  const channels = [{name: 'a', unit: ''}, {name: 'b', unit: ''},
                    {name: 'c', unit: ''}];
  const header = (sequenceId: bigint) => ({
    streamId: 7n, sequenceId, timestampNs: sequenceId, dictionaryId: 3,
    channelCount: 3
  });
  const chassis = {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0};
  const update = encodeValuePages(header(2n), chassis, false,
                                  [[1, rawFromDouble(20)]], [2]);
  const decoder = new TelemetryDecoder();
  for (const page of encodeDictionaryPages(header(0n), channels))
    decoder.decode(page);
  assert.throws(() => decoder.decode(update[0]), /before a keyframe/);
  decoder.decode(encodeValuePages(
      header(1n), chassis, true,
      [[0, rawFromDouble(1)], [1, rawFromDouble(2)], [2, rawFromDouble(3)]])[0]);
  const snapshot = decoder.decode(update[0])!;
  assert.deepEqual({...snapshot.channels}, {a: 1, b: 20});
  assert.equal(snapshot.sequence_id, '2');
  assert.throws(() => new TelemetryDecoder().decode(update[0]),
                /before its dictionary/);
});

test('NaN and infinities are readings, absence is not, and oversize frames are refused', () => {
  const decoder = new TelemetryDecoder();
  decoder.decode(demoPacket(0n));
  for (const [reading, shown] of [[Number.NaN, 'NaN'], [Infinity, 'Infinity'],
                                  [-Infinity, '-Infinity']] as const) {
    const f = decoder.decode(
        valueFrame(1n, 1, [reading, 0, 12, 2, 0, 0, 0, 0]))!;
    assert.ok('velocity' in f.channels);
    assert.equal(channelDisplay(f, 'velocity'), shown);
  }
  const f = decoder.decode(valueFrame(2n, 1, [null, 0, 12, 2, 0, 0, 0, 0]))!;
  assert.equal('velocity' in f.channels, false);
  assert.equal(f.channels.voltage, 12);
  assert.equal(MAX_FRAME_BYTES, 16384);
  assert.throws(() => decoder.decode(new Uint8Array(16385)));
});

// Studio's own pages (demo, export) obey the producer's page cap however an
// update mixes samples and absent channels, and decode to what was encoded.
test('encoded value pages stay within the page cap with absent channels', () => {
  const header = {streamId: 5n, sequenceId: 1n, timestampNs: 2n,
                  dictionaryId: 9, channelCount: 1000};
  const chassis = {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0};
  for (const carried of [0, 1, 80, 83, 84, 200])
    for (const lost of [0, 1, 249, 250, 251, 700]) {
      const samples = Array.from({length: carried},
                                 (_, i): [number, bigint] => [i, 0xffffffffffffffffn]);
      const absent = Array.from({length: lost}, (_, i) => 999 - i);
      const pages = encodeValuePages(header, chassis, false, samples, absent);
      let seenSamples = 0, seenAbsent = 0;
      pages.forEach((page, i) => {
        assert.ok(page.length <= 1200, `${carried}+${lost}: page ${i} is ${page.length} B`);
        const f = Frame.getSizePrefixedRootAsFrame(new ByteBuffer(page));
        assert.equal(f.page(), i);
        assert.equal(f.pageCount(), pages.length);
        seenSamples += f.channelsLength();
        seenAbsent += f.absentLength();
      });
      assert.equal(seenSamples, carried);
      assert.equal(seenAbsent, lost);
    }
});

test('every dictionary the producer may emit decodes; one past its limits does not', () => {
  const decode = (values: bigint[], options: {dictionary: string[]; labels: string[]}) =>
      new TelemetryDecoder().decode(valueFrame(
          0n, 7, values, 42n,
          {...options, kinds: options.dictionary.map(() => ChannelKind.UInt)}));
  // Base-36 indices keep the producer's largest enum inside one frame.
  const labels = Array.from({length: MAX_ENUM_LABELS}, (_, i) => i.toString(36));
  const state = decode([BigInt(MAX_ENUM_LABELS - 1)], {dictionary: ['fault'], labels})!;
  assert.equal(channelDisplay(state, 'fault'), labels.at(-1));
  assert.throws(() => decode([0n], {dictionary: ['fault'], labels: [...labels, 'z']}),
                /Too many labels/);
  // The producer writes each string once: offsets shared to multiply a frame's
  // text past the frame itself are a decode bomb, not a dictionary.
  assert.throws(() => decode([], {dictionary: ['a', 'b', 'c', 'd', 'e', 'f'], labels}),
                /exceeds its frame/);
});
test('a new stream invalidates the old dictionary', () => {
  const decoder = new TelemetryDecoder();
  decoder.decode(demoPacket(0n));
  assert.throws(() => decoder.decode(valueFrame(0n, 1, [1], 77n)),
                /before its dictionary/);
});

test('explicitly empty dictionaries replace cached dictionaries', () => {
  const decoder = new TelemetryDecoder();
  decoder.decode(valueFrame(0n, 9, [1], 42n, {dictionary: ['old']}));
  assert.deepEqual(Object.keys(decoder.decode(
      valueFrame(1n, 9, [], 42n, {dictionary: []}))!.channels), []);
  assert.throws(() => decoder.decode(valueFrame(2n, 9, [1])));
});

test('an invalid page changes neither the dictionary, the stream nor the values', () => {
  const kinds = [ChannelKind.Double, ChannelKind.Bool];
  const invalid = [
    valueFrame(2n, 9, [1, 2], 77n, {dictionary: ['duplicate', 'duplicate']}),
    valueFrame(2n, 9, [1], 77n, {dictionary: ['derived/spoof']}),
    // The chassis is a pose, never a reading: it must be finite.
    valueFrame(2n, 5, [1, null], 42n, {yaw: NaN}),
    valueFrame(2n, 5, [1, null], 42n, {missingChassis: true}),
    // A bool carries 0 or 1.
    valueFrame(2n, 5, [1, 2n])
  ];
  for (const bytes of invalid) {
    const decoder = new TelemetryDecoder();
    decoder.decode(valueFrame(0n, 5, [1, 1n], 42n,
                              {dictionary: ['old', 'flag'], kinds}));
    assert.throws(() => decoder.decode(bytes));
    const next = decoder.decode(valueFrame(3n, 5, [2, null]))!;
    assert.deepEqual({...next.channels}, {old: 2});
  }
});

test('jitter sorts, deduplicates, counts gaps and rejects late packets', () => {
  const j = new JitterBuffer(10);
  j.push(frame(2), 0);
  j.push(frame(0), 1);
  j.push(frame(1), 2);
  j.push(frame(1), 3);
  assert.equal(j.flush(9).length, 0);
  assert.deepEqual(j.flush(10).map(f => f.sequence_id), ['0', '1', '2']);
  j.push(frame(4), 11);
  assert.deepEqual(j.flush(21).map(f => f.sequence_id), ['4']);
  j.push(frame(3), 22);
  assert.equal(j.stats.skipped, 1);
  assert.equal(j.stats.late, 1);
});
test('exact lookup and held state never interpolate; bounded history', () => {
  const t = new Timeline(2);
  for (let i = 0; i < 3; i++) t.append(frame(i));
  assert.equal(t.at(0n), null);
  assert.equal(t.exact(1020000001n), null);
  assert.equal(t.at(1020000001n)?.sequence_id, '1');
  assert.equal(t.append(frame(1)), false);
  assert.equal(t.frames.length, 2);
});
test('pause survives ingestion; frame step and replay clamp at end', () => {
  const c = new Coordinator();
  c.timeline.append(frame(0));
  c.tick(0n);
  c.mode = 'PAUSED';
  c.timeline.append(frame(1));
  c.tick(1n);
  assert.equal(c.snapshot()?.sequence_id, '0');
  c.step(20000000n);
  assert.equal(c.snapshot()?.sequence_id, '1');
  c.seek(1000000000n);
  c.mode = 'REPLAY_PLAYING';
  c.tick(200000000n);
  assert.equal(c.mode, 'PAUSED');
  assert.equal(c.snapshot()?.sequence_id, '1');
});

test('paused and replay cursors recover from empty or evicted history', () => {
  for (const mode of ['PAUSED', 'REPLAY_PLAYING'] as const) {
    for (const viaTick of [false, true]) {
      const c = new Coordinator(new Timeline(1));
      c.mode = mode;
      assert.equal(c.snapshot(), null);
      c.timeline.append(frame(0));
      if (viaTick) c.tick(0n);
      assert.equal(c.snapshot()?.sequence_id, '0');
      c.timeline.append(frame(1));
      if (viaTick) c.tick(0n);
      assert.equal(c.snapshot()?.sequence_id, '1');
      assert.equal(c.cursor, 1020000000n);
    }
  }
  const c = new Coordinator();
  c.timeline.append(frame(0));
  c.mode = 'REPLAY_PLAYING';
  c.cursor = 9999999999n;
  assert.equal(c.snapshot()?.sequence_id, '0');
  assert.equal(c.cursor, 1000000000n);
});

test('target changes distinguish identical source lists by target index', () => {
  const prev = parseSystemGraph(demoSystemGraph(0n));
  const next = parseSystemGraph(demoSystemGraph(1n));
  const sources = ['driver', 'auto'];
  prev.nodes[0].selections = [
    {sources, selected: 'driver'}, {sources, selected: 'auto'}
  ];
  next.nodes[0].selections = [
    {sources, selected: 'driver'}, {sources, selected: 'auto'}
  ];
  const detector = SYSTEM_DETECTORS.find(d => d.id === 'target-selection')!;
  assert.deepEqual(detector.detect(prev, next), []);
  next.nodes[0].selections[1].selected = 'driver';
  assert.equal(detector.detect(prev, next).length, 1);
});

test('counter resets refresh the baseline instead of ending a live run', () => {
  const detector = createFrameDetectors().find(d => d.id === 'counter-clock-resets')!;
  const sample = (timestamp_ns: string, value: number) => ({
    ...frame(0), timestamp_ns,
    channels: {'hw/status/clock_resets_count': value}
  });
  const a = sample('0', 99), b = sample('100000000', 100);
  const reset = sample('1200000000', 0);
  const advancing = sample('2100000000', 1);
  assert.equal(detector.detect(a, b).length, 1);
  assert.deepEqual(detector.detect(b, reset), []);
  assert.deepEqual(detector.detect(reset, advancing), []);
  assert.deepEqual(detector.detect(advancing, sample('3000000000', 1)), []);
  assert.equal(detector.detect(
      sample('3000000000', 1), sample('3100000000', 1)).length, 1);
});
test('all truncated frames and corrupt roots fail closed', () => {
  const bytes = demoPacket(2n);
  const decoder = new TelemetryDecoder();
  for (let n = 0; n < bytes.length; n++)
    assert.throws(() => decoder.decode(bytes.slice(0, n)));
  const copy = bytes.slice();
  new DataView(copy.buffer).setUint32(4, 0xfffffff0, true);
  assert.throws(() => decoder.decode(copy));
});

test('sim truth reads flattened dictionary channels, independently of estimate validity',
     () => {
       const snapshot = frame(0);
       const truth = simTruthPose({
         ...snapshot,
         channels: {
           'sim/state/pose/x_m': 0,
           'sim/state/pose/y_m': 2,
           'sim/state/pose/yaw_rad': -.5,
           'pose/valid': 0
         }
       });
       assert.deepEqual(truth, {x: 0, y: 2, yaw: -.5});
       assert.notDeepEqual(truth, snapshot.chassis);
     });

test('sim truth is absent without all three current finite pose readings', () => {
  const snapshot = frame(0);
  assert.equal(simTruthPose(null), null);
  assert.equal(simTruthPose(undefined), null);
  assert.equal(simTruthPose({...snapshot, channels: {}}), null);
  // Units describe dictionary presence, but cannot stand in for a current
  // reading: TelemetryDecoder omits absent (stale) channels from channels.
  const decoder = new TelemetryDecoder();
  decoder.decode(demoPacket(0n));
  const missing = decoder.decode(
      valueFrame(1n, 1, [1.75, 0, 12, 2, 0, 0, 0, 1]))!;
  assert.equal(missing.units['sim/state/pose/x_m'], 'm');
  assert.equal(simTruthPose(missing), null);
  for (const name of [
         'sim/state/pose/x_m', 'sim/state/pose/y_m', 'sim/state/pose/yaw_rad'
       ]) {
    for (const value of [undefined, Number.NaN, Infinity, -Infinity]) {
      const channels = {...snapshot.channels};
      if (value === undefined) delete channels[name];
      else channels[name] = value;
      assert.equal(simTruthPose({...snapshot, channels}), null, `${name}: ${value}`);
    }
  }
});

test('pose error is planar distance and signed estimate-minus-truth heading', () => {
  const error = poseError(
      {x: 4, y: 6, yaw: Math.PI / 3}, {x: 1, y: 2, yaw: 0});
  assert.equal(error.position_m, 5);
  assert.ok(Math.abs(error.heading_deg - 60) < 1e-10);
  assert.deepEqual(
      poseError({x: 0, y: 0, yaw: 0}, {x: 0, y: 0, yaw: 0}),
      {position_m: 0, heading_deg: 0});
});

test('pose heading error wraps both directions across 180 degrees and full turns',
     () => {
       const radians = Math.PI / 180;
       for (const [estimate, truth, expected] of [
              [-179, 179, 2], [179, -179, -2], [721, -1, 2],
              [-721, 1, -2], [180, 0, 180], [-180, 0, -180]
            ]) {
         const error = poseError(
             {x: 0, y: 0, yaw: estimate * radians},
             {x: 0, y: 0, yaw: truth * radians});
         assert.ok(Math.abs(error.heading_deg - expected) < 1e-10);
         assert.ok(error.heading_deg >= -180 && error.heading_deg <= 180);
       }
     });

const simPoseChannels = [
  'sim/state/pose/x_m', 'sim/state/pose/y_m', 'sim/state/pose/yaw_rad',
  'pose/valid'
];

test('decoded snapshots derive pose-error channels with units from truth and valid estimates',
     () => {
       const snapshot = new TelemetryDecoder().decode(valueFrame(
           0n, 7, [4, 6, -Math.PI / 3, 1], 42n,
           {dictionary: simPoseChannels}))!;
       assert.equal(snapshot.channels['derived/pose_error_m'], 5);
       assert.ok(
           Math.abs(snapshot.channels['derived/heading_error_deg'] - 60) < 1e-10);
       assert.equal(snapshot.units['derived/pose_error_m'], 'm');
       assert.equal(snapshot.units['derived/heading_error_deg'], 'deg');
       assert.equal(snapshot.channels['sim/state/pose/x_m'], 4);
     });

test('derived pose-error readings are absent, not zero, when truth or a valid estimate is absent',
     () => {
       for (const {dictionary, values} of [
              {dictionary: ['pose/valid'], values: [1]},
              {dictionary: simPoseChannels, values: [4, 6, 0, 0]},
              {dictionary: simPoseChannels, values: [4, 6, 0, 2]},
              {dictionary: simPoseChannels, values: [Number.NaN, 6, 0, 1]},
              {dictionary: simPoseChannels, values: [4, Number.NaN, 0, 1]},
              {dictionary: simPoseChannels, values: [4, 6, Number.NaN, 1]},
              {dictionary: simPoseChannels, values: [4, 6]}
            ]) {
         const snapshot = new TelemetryDecoder().decode(valueFrame(
             0n, 7, values, 42n, {dictionary}))!;
         for (const name of ['derived/pose_error_m', 'derived/heading_error_deg']) {
           assert.equal(name in snapshot.channels, false);
           assert.equal(name in snapshot.units, false);
         }
       }
     });

test('snapshots share their dictionary\'s frozen units and labels', () => {
  const decoder = new TelemetryDecoder();
  decoder.decode(demoPacket(0n));
  const [a, b] = [1n, 2n].map(n => decoder.decode(demoPacket(n))!);
  assert.equal(a.units, b.units);
  assert.equal(a.labels, b.labels);
  assert.ok(Object.isFrozen(a.units) && Object.isFrozen(a.labels));
  // Derived units ride their own shared copy, never the dictionary's units.
  const plain = decoder.decode(valueFrame(3n, 1, [1, 0, 12, 2, 0, 0, 0, 1]))!;
  assert.equal(a.units['derived/pose_error_m'], 'm');
  assert.equal('derived/pose_error_m' in plain.units, false);
});

test('derived heading series wraps both directions across 180 degrees', () => {
  for (const [estimate, truth, expected] of [[-179, 179, 2], [179, -179, -2]]) {
    const snapshot = new TelemetryDecoder().decode(valueFrame(
        0n, 7, [1, 2, truth * Math.PI / 180, 1], 42n,
        {dictionary: simPoseChannels, yaw: estimate * Math.PI / 180}))!;
    assert.equal(snapshot.channels['derived/pose_error_m'], 0);
    assert.ok(
        Math.abs(snapshot.channels['derived/heading_error_deg'] - expected) < 1e-10);
  }
});

test('wire channels cannot impersonate Studio-derived series', () => {
  assert.throws(
      () => new TelemetryDecoder().decode(valueFrame(
          0n, 7, [1], 42n, {dictionary: ['derived/pose_error_m']})),
      /Reserved derived\//);
});

test('derived series persist in history through seek and replay', () => {
  const decoder = new TelemetryDecoder();
  const coordinator = new Coordinator();
  for (const sequence of [0n, 1n, 2n])
    coordinator.timeline.append(decoder.decode(demoPacket(sequence))!);
  coordinator.seek(1000000000n);
  const first = coordinator.snapshot()!;
  const error = first.channels['derived/pose_error_m'];
  assert.ok(error > 0);
  coordinator.mode = 'REPLAY_PLAYING';
  coordinator.tick(40000000n);
  assert.notEqual(coordinator.snapshot()!.channels['derived/pose_error_m'], error);
  coordinator.seek(1000000000n);
  assert.equal(coordinator.snapshot()!.channels['derived/pose_error_m'], error);
  assert.equal(coordinator.timeline.exact(1000000000n), first);
});
