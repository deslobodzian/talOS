import assert from 'node:assert/strict';
import {test} from 'node:test';

import {Timeline, type Snapshot} from '../src/core';
import {STORE_CAPACITY, TelemetryStore} from '../src/data/store';

const DS = 'hw/state/driver_station/';

function snap(seq: number, ts: bigint, channels: Record<string, number> = {}): Snapshot {
  return {
    stream_id: 'a',
    sequence_id: String(seq),
    timestamp_ns: String(ts),
    dictionary_id: 1,
    chassis: {x: seq, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
    channels: Object.assign(Object.create(null), channels),
    units: Object.create(null),
    labels: Object.create(null)
  };
}

test('snapshotAt matches Timeline.at over random cursors', () => {
  const frames: Snapshot[] = [];
  const timeline = new Timeline();
  for (let i = 0; i < 200; i++) {
    const frame = snap(i, 1_000_000_000n + BigInt(i) * 20_000_000n, {
      'drivetrain/state/velocity': i * 0.5,
      'hw/status/battery_v': 12 + (i % 3) * 0.1
    });
    frames.push(frame);
    timeline.append(frame);
  }
  const store = new TelemetryStore(cb => cb());
  store.ingest(frames, true);

  for (let trial = 0; trial < 500; trial++) {
    const offset = Math.floor(Math.random() * 250) - 25;  // below-first .. above-last
    const cursor = 1_000_000_000n + BigInt(offset) * 20_000_000n;
    const expected = timeline.at(cursor);
    const actual = store.snapshotAt(String(cursor));
    if (expected === null) {
      assert.equal(actual, null);
    } else {
      assert.ok(actual);
      assert.equal(actual.timestamp_ns, expected.timestamp_ns);
      assert.equal(actual.sequence_id, expected.sequence_id);
      assert.equal(actual.stream_id, expected.stream_id);
      assert.deepEqual(actual.channels, expected.channels);
      assert.deepEqual(actual.chassis, expected.chassis);
    }
  }
});

test('channels first declared later are NaN-backfilled', () => {
  const store = new TelemetryStore(cb => cb());
  store.ingest([snap(0, 1n, {a: 1}), snap(1, 2n, {a: 2, b: 5})], true);
  store.setCursor('2');
  const series = store.series(['a', 'b'], 'all');
  assert.deepEqual(Array.from(series.values.a), [1, 2]);
  assert.deepEqual(Array.from(series.values.b), [NaN, 5]);
  assert.equal(store.snapshotAt('1')!.channels.b, undefined);
});

test('reset clears the ring and the dictionary', () => {
  const store = new TelemetryStore(cb => cb());
  store.ingest([snap(0, 1n, {a: 1}), snap(1, 2n, {a: 2})], true);
  store.ingest([snap(2, 3n, {b: 3})], true);
  assert.deepEqual(store.dictionary().map(c => c.name), ['b']);
  assert.equal(store.snapshotAt('1'), null);
  assert.equal(store.snapshotAt('3')!.channels.b, 3);
});

test('trims the oldest frames past capacity', () => {
  const store = new TelemetryStore(cb => cb());
  const frames: Snapshot[] = [];
  const n = STORE_CAPACITY + 3;
  for (let i = 0; i < n; i++)
    frames.push(snap(i, 1_000_000_000n + BigInt(i) * 10n, {a: i}));
  store.ingest(frames, true);
  // Frames 0, 1 and 2 fell off the front.
  assert.equal(store.snapshotAt(String(1_000_000_000n + 2n * 10n)), null);
  assert.equal(store.snapshotAt(String(1_000_000_000n + 3n * 10n))!.channels.a, 3);
});

test('a reused ring slot does not leak the previous frame value', () => {
  const store = new TelemetryStore(cb => cb(), 4);
  store.ingest([
    snap(0, 1n, {a: 0, b: 100}),
    snap(1, 2n, {a: 1, b: 101}),
    snap(2, 3n, {a: 2, b: 102}),
    snap(3, 4n, {a: 3, b: 103}),
    snap(4, 5n, {a: 4}),      // omits b; reuses frame 0's slot (b=100)
    snap(5, 6n, {a: 5, b: 105})
  ], true);

  // Frame 4 omitted b and must read as absent, not the overwritten 100.
  assert.equal(store.snapshotAt('5')!.channels.a, 4);
  assert.equal(store.snapshotAt('5')!.channels.b, undefined);

  store.setCursor('5');
  const series = store.series(['a', 'b'], 'all');
  // Retained frames are 2, 3, 4 (frames 0 and 1 fell off the front).
  assert.deepEqual(Array.from(series.values.a), [2, 3, 4]);
  assert.deepEqual(Array.from(series.values.b), [102, 103, NaN]);
});

test('series windows: all and seconds', () => {
  const store = new TelemetryStore(cb => cb());
  const frames: Snapshot[] = [];
  for (let i = 0; i <= 10; i++)
    frames.push(snap(i, 1_000_000_000n + BigInt(i) * 1_000_000_000n, {a: i}));
  store.ingest(frames, true);
  store.setCursor(String(1_000_000_000n + 10n * 1_000_000_000n));

  const all = store.series(['a'], 'all');
  assert.equal(all.times.length, 11);
  assert.equal(all.values.a[10], 10);
  assert.equal(all.t0_ns, '1000000000');

  // 4 seconds of history ending at 10s: frames at t >= 6s.
  const window = store.series(['a'], {seconds: 4});
  assert.deepEqual(Array.from(window.values.a), [6, 7, 8, 9, 10]);
});

test('snapshotAt memoises per index', () => {
  const store = new TelemetryStore(cb => cb());
  store.ingest([snap(0, 1n, {a: 1})], true);
  assert.strictEqual(store.snapshotAt('1'), store.snapshotAt('1'));
});

test('mode bands are maintained incrementally', () => {
  const store = new TelemetryStore(cb => cb());
  store.ingest([
    snap(0, 1n, {[DS + 'connected']: 1, [DS + 'enabled']: 0}),
    snap(1, 2n, {[DS + 'connected']: 1, [DS + 'enabled']: 0}),
    snap(2, 3n, {[DS + 'connected']: 1, [DS + 'enabled']: 1, [DS + 'teleop']: 1})
  ], true);
  assert.deepEqual(store.modeBands(), [
    {start_ns: '1', end_ns: '2', mode: 'disabled'},
    {start_ns: '3', end_ns: '3', mode: 'teleop'}
  ]);
});

test('subscribe notifies once per batch via the injected scheduler', () => {
  const queue: (() => void)[] = [];
  const store = new TelemetryStore(cb => queue.push(cb));
  let notified = 0;
  store.subscribe(() => notified++);
  store.ingest([snap(0, 1n, {a: 1})], true);
  store.ingest([snap(1, 2n, {a: 2})], false);
  assert.equal(queue.length, 1);  // one scheduled callback, not two
  assert.equal(notified, 0);      // not flushed yet
  queue[0]();
  assert.equal(notified, 1);
  assert.equal(store.version, 2);
});

test('historical snapshots retain their own dictionary metadata', () => {
  const store = new TelemetryStore(cb => cb(), 3);
  const old = snap(0, 1n, {mode: 0});
  old.units = {mode: 'old-unit'};
  old.labels = {mode: ['old-label']};
  const current = {...snap(1, 2n, {mode: 0}), dictionary_id: 2,
    units: {mode: 'new-unit'}, labels: {mode: ['new-label']}};
  store.ingest([old], true);
  const cached = store.snapshotAt('1')!;
  store.ingest([current], false);
  assert.strictEqual(store.snapshotAt('1'), cached);
  assert.deepEqual({...store.snapshotAt('1')!.units}, old.units);
  assert.deepEqual({...store.snapshotAt('1')!.labels}, old.labels);
  assert.deepEqual({...store.snapshotAt('2')!.units}, current.units);
  assert.deepEqual({...store.snapshotAt('2')!.labels}, current.labels);
  // Also reconstruct an old frame that was never memoised.
  const fresh = new TelemetryStore(cb => cb());
  fresh.ingest([old, current], true);
  assert.deepEqual({...fresh.snapshotAt('1')!.units}, old.units);
  assert.deepEqual({...fresh.snapshotAt('1')!.labels}, old.labels);
});

test('dictionary eviction bounds columns and metadata to retained channels', () => {
  const store = new TelemetryStore(cb => cb(), 3);
  for (let i = 0; i < 40; i++) {
    const name = `channel-${i}`;
    store.ingest([{...snap(i, BigInt(i + 1), {[name]: i}),
      dictionary_id: i, units: {[name]: 'V'}}], false);
    assert.equal(store.dictionary().length, Math.min(i + 1, 3));
    // Storage, not just the visible dictionary, must be released.
    const storage = store as unknown as {
      columns: Map<string, Float64Array>; dictionaries: Map<number, unknown>;
    };
    assert.equal(storage.columns.size, Math.min(i + 1, 3));
    assert.equal(storage.dictionaries.size, Math.min(i + 1, 3));
    if (i >= 3) {
      assert.equal(store.channelInfo(`channel-${i - 3}`), undefined);
      store.setCursor(String(i + 1));
      assert.ok([...store.series([`channel-${i - 3}`], 'all').values[`channel-${i - 3}`]]
          .every(Number.isNaN));
    }
  }
});

test('shared channels survive until their last dictionary is evicted', () => {
  const store = new TelemetryStore(cb => cb(), 2);
  store.ingest([
    {...snap(0, 1n, {shared: 1}), dictionary_id: 1},
    {...snap(1, 2n, {shared: 2}), dictionary_id: 2},
    {...snap(2, 3n, {other: 3}), dictionary_id: 3}
  ], true);
  assert.equal(store.snapshotAt('2')!.channels.shared, 2);
  assert.ok(store.channelInfo('shared'));
  store.ingest([{...snap(3, 4n, {other: 4}), dictionary_id: 3}], false);
  assert.equal(store.channelInfo('shared'), undefined);
});
