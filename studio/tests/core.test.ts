import assert from 'node:assert/strict';
import {test} from 'node:test';

import {Builder} from 'flatbuffers';

import {
  channelDisplay,
  Coordinator,
  decode,
  JitterBuffer,
  MAX_FRAME_BYTES,
  TelemetryDecoder,
  Timeline,
} from '../src/core';
import {demoPacket} from '../src/demo';
import {Frame} from '../src/generated/talos/telemetry/frame';
import {Pose} from '../src/generated/talos/telemetry/pose';
import {
  type GraphTab,
  isGraphTabId,
  isTabId,
  newGraphTab,
  parseGraphTabs,
} from '../src/tabs';

// Prime the shared decoder with the dictionary frame, as a bridge does on
// connect, so bare value frames below can be named.
decode(demoPacket(0n));
const frame = (i: number) => decode(demoPacket(BigInt(i)))!;

// A bare value frame (no dictionary) for `dictionaryId`, built the way a
// producer that already sent its dictionary builds every later frame.
function valueFrame(sequence: bigint, dictionaryId: number,
                    values: number[], streamId = 42n): Uint8Array {
  const b = new Builder(256);
  const vector = Frame.createValuesVector(b, values);
  Frame.startFrame(b);
  Frame.addStreamId(b, streamId);
  Frame.addSequenceId(b, sequence);
  Frame.addTimestampNs(b, 1000000000n + sequence * 20000000n);
  Frame.addDictionaryId(b, dictionaryId);
  Frame.addValues(b, vector);
  Frame.addChassis(b, Pose.createPose(b, 1, 2, 0, 0, 0, 0));
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


// --- graph tabs -------------------------------------------------------------

test('a graph tab id is distinguishable from a built-in view id', () => {
  const created = newGraphTab([]);
  assert.ok(isGraphTabId(created.id));
  assert.ok(!isTabId(created.id));
  // The built-in ids must not be mistaken for graphs, or selecting one would
  // render an empty graph panel instead of the view.
  for (const id of ['overview', 'system', 'signals', 'field'])
    assert.ok(!isGraphTabId(id), id);
  // A bare prefix names no graph.
  assert.ok(!isGraphTabId('graph:'));
});

test('graph tabs are numbered from the labels in use, not the count', () => {
  let tabs: GraphTab[] = [];
  tabs = [...tabs, newGraphTab(tabs)];
  tabs = [...tabs, newGraphTab(tabs)];
  assert.deepEqual(tabs.map(t => t.label), ['Graph 1', 'Graph 2']);
  assert.notEqual(tabs[0].id, tabs[1].id);

  // Closing the first and adding another must not produce a second "Graph 2".
  const after = tabs.filter(t => t.label !== 'Graph 1');
  const next = newGraphTab(after);
  assert.equal(next.label, 'Graph 3');

  // A renamed tab does not constrain the numbering.
  assert.equal(newGraphTab([{id: 'graph:a', label: 'Shooter', channels: []}]).label,
               'Graph 1');
});

test('stored graph tabs are validated, because they come back as unknown JSON',
     () => {
       assert.deepEqual(parseGraphTabs(null), []);
       assert.deepEqual(parseGraphTabs('not an array'), []);
       assert.deepEqual(parseGraphTabs([1, null, 'x']), []);

       // A tab with no usable id could be neither selected nor closed, which
       // would strand the user on a panel with no way out.
       assert.deepEqual(parseGraphTabs([{id: 'nope', label: 'A'}]), []);
       assert.deepEqual(parseGraphTabs([{id: 'graph:a', label: ''}]), []);

       // Duplicate ids would give two tabs one identity: closing one would
       // close both, and React would warn on the repeated key.
       assert.equal(
           parseGraphTabs([
             {id: 'graph:a', label: 'A', channels: []},
             {id: 'graph:a', label: 'B', channels: []}
           ]).length,
           1);

       // Channels are filtered rather than trusted; a non-string would reach
       // uPlot as a series label and render as "[object Object]".
       assert.deepEqual(
           parseGraphTabs([{id: 'graph:a', label: 'A', channels: ['x', 2, null]}]),
           [{id: 'graph:a', label: 'A', channels: ['x']}]);

       // A missing channels list is an empty graph, not a broken one.
       assert.deepEqual(parseGraphTabs([{id: 'graph:a', label: 'A'}]),
                        [{id: 'graph:a', label: 'A', channels: []}]);
     });
