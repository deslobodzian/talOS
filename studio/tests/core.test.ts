import assert from 'node:assert/strict';
import {test} from 'node:test';

import {Coordinator, decode, JitterBuffer, Timeline} from '../src/core';
import {demoPacket} from '../src/demo';

const frame = (i: number) => decode(demoPacket(BigInt(i)));
test(
    'size-prefixed generated FlatBuffer roundtrip preserves uint64 timestamps',
    () => {
      const f = decode(demoPacket(9007199254740993n));
      assert.equal(f.sequence_id, '9007199254740993');
      assert.equal(
          f.timestamp_ns,
          BigInt.asUintN(64, 1000000000n + 9007199254740993n * 20000000n)
              .toString());
      assert.equal(f.ghosts.length, 3);
      assert.equal(f.channels.velocity, 1.75);
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
