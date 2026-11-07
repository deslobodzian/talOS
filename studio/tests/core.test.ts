import assert from 'node:assert/strict';
import {test} from 'node:test';

import {Builder} from 'flatbuffers';

import {
  channelDisplay,
  Coordinator,
  decode,
  JitterBuffer,
  MAX_FRAME_BYTES,
  poseError,
  simTruthPose,
  TelemetryDecoder,
  Timeline,
} from '../src/core';
import {demoPacket} from '../src/demo';
import {ChannelInfo} from '../src/generated/talos/telemetry/channel-info';
import {Frame} from '../src/generated/talos/telemetry/frame';
import {Pose} from '../src/generated/talos/telemetry/pose';

// Prime the shared decoder with the dictionary frame, as a bridge does on
// connect, so bare value frames below can be named.
decode(demoPacket(0n));
const frame = (i: number) => decode(demoPacket(BigInt(i)))!;

// A bare value frame (no dictionary) for `dictionaryId`, built the way a
// producer builds every later frame. Options also support custom dictionaries
// and chassis headings for decoder-level derived-channel tests.
function valueFrame(sequence: bigint, dictionaryId: number,
                    values: number[], streamId = 42n,
                    options: {dictionary?: string[]; yaw?: number} = {}): Uint8Array {
  const b = new Builder(256);
  const dictionary = options.dictionary ?
      Frame.createDictionaryVector(b, options.dictionary.map(name => {
        const text = b.createString(name);
        ChannelInfo.startChannelInfo(b);
        ChannelInfo.addName(b, text);
        return ChannelInfo.endChannelInfo(b);
      })) :
      undefined;
  const vector = Frame.createValuesVector(b, values);
  Frame.startFrame(b);
  Frame.addStreamId(b, streamId);
  Frame.addSequenceId(b, sequence);
  Frame.addTimestampNs(b, 1000000000n + sequence * 20000000n);
  Frame.addDictionaryId(b, dictionaryId);
  Frame.addValues(b, vector);
  if (dictionary !== undefined) Frame.addDictionary(b, dictionary);
  Frame.addChassis(
      b, Pose.createPose(b, 1, 2, 0, 0, 0, options.yaw ?? 0));
  Frame.finishSizePrefixedFrameBuffer(b, Frame.endFrame(b));
  return b.asUint8Array();
}

test(
    'size-prefixed generated FlatBuffer roundtrip preserves uint64 timestamps',
    () => {
      const decoder = new TelemetryDecoder();
      assert.ok(decoder.decode(demoPacket(0n)));
      const f = decoder.decode(valueFrame(
          9007199254740993n, 1,
          [1.75, 0, 12, 2, 0, 0, 0, 1]))!;
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
test('a value frame with an unknown dictionary is held, not fatal', () => {
  const decoder = new TelemetryDecoder();
  assert.equal(decoder.decode(valueFrame(5n, 9, [3])), null);
  assert.equal(decoder.heldCount, 1);
  const resolved = decoder.decodeAll(demoPacket(0n));
  // demoPacket(0) uses dictionary 1, so the held frame for 9 stays held.
  assert.equal(resolved.length, 1);
  const other = new TelemetryDecoder();
  assert.equal(other.decode(valueFrame(5n, 1, [3, 0, 12, 2, 0, 0, 0, 0])), null);
  const unblocked = other.decodeAll(demoPacket(0n));
  assert.equal(unblocked.length, 2);
  assert.equal(unblocked[1].sequence_id, '5');
  assert.equal(unblocked[1].channels.velocity, 3);
});
test('NaN samples are omitted and oversize frames are refused', () => {
  const decoder = new TelemetryDecoder();
  decoder.decode(demoPacket(0n));
  const f = decoder.decode(
      valueFrame(1n, 1, [Number.NaN, 0, 12, 2, 0, 0, 0, 0]))!;
  assert.equal('velocity' in f.channels, false);
  assert.equal(f.channels.voltage, 12);
  assert.equal(MAX_FRAME_BYTES, 16384);
  assert.throws(() => decoder.decode(new Uint8Array(16385)));
});
test('a new stream invalidates the old dictionary', () => {
  const decoder = new TelemetryDecoder();
  decoder.decode(demoPacket(0n));
  assert.equal(decoder.decode(valueFrame(0n, 1, [1], 77n)), null);
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
  assert.equal(j.stats.dropped, 1);
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
test('all truncated frames and corrupt roots fail closed', () => {
  const bytes = demoPacket(2n);
  for (let n = 0; n < bytes.length; n++)
    assert.throws(() => decode(bytes.slice(0, n)));
  const copy = bytes.slice();
  new DataView(copy.buffer).setUint32(4, 0xfffffff0, true);
  assert.throws(() => decode(copy));
});
test(
    'identical log yields identical snapshots regardless of query order',
    () => {
      const a = new Timeline(), b = new Timeline();
      for (let i = 0; i < 100; i++) {
        a.append(frame(i));
        b.append(frame(i));
      }
      for (const i of [99, 2, 50, 0, 34, 50])
        assert.deepEqual(
            a.at(1000000000n + BigInt(i) * 20000000n),
            b.at(1000000000n + BigInt(i) * 20000000n));
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
  // reading: TelemetryDecoder omits NaN (absent/stale) values from channels.
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

test('demo dictionary and bare frames carry a small estimate offset from truth',
     () => {
       const decoder = new TelemetryDecoder();
       for (const sequence of [0n, 1n, 80n]) {
         const snapshot = decoder.decode(demoPacket(sequence))!;
         const truth = simTruthPose(snapshot);
         assert.ok(truth);
         const error = poseError(snapshot.chassis, truth);
         assert.ok(error.position_m > .1 && error.position_m < .2);
         assert.ok(error.heading_deg > 1 && error.heading_deg < 2);
         assert.equal(snapshot.units['sim/state/pose/yaw_rad'], 'rad');
         assert.equal(
             snapshot.channels['derived/pose_error_m'], error.position_m);
         assert.equal(
             snapshot.channels['derived/heading_error_deg'], error.heading_deg);
         assert.equal(snapshot.units['derived/pose_error_m'], 'm');
         assert.equal(snapshot.units['derived/heading_error_deg'], 'deg');
       }
     });
