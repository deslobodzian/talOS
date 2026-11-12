// The review API contract, browser side (REQ-OPS-41): review.ts decodes the
// responses studio_review gave for its fixture session, checked in under
// studio/review/testdata/golden, which //studio/review:review_test requires
// the server to reproduce byte for byte. A shape change on either side fails
// one of the two tests; studio/README.md ("Reviewing a session")
// says how to regenerate the files for an intended change.

import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {test} from 'node:test';

import {bucketSeries} from '../src/data/recorded';
import {
  decodeSeries,
  parseBounds,
  parseChannels,
  parseJson,
  parseSessions,
  parseSnapshot,
  wallTimeNs
} from '../src/data/review';

const golden = (name: string) =>
    readFileSync(new URL(`../review/testdata/golden/${name}`, import.meta.url));
const document = (name: string) =>
    parseJson(golden(name).toString('utf8'), name);

// The fixture's clock (studio/review/testdata/fixture.h): session time
// kStartNs = 1000 s, at wall time 2026-01-01T00:00:00Z.
const START_NS = 1_000_000_000_000n;
const START_WALL_NS = 1_767_225_600_000_000_000n;
const SHOT_ID = (k: bigint) => (1n << 60n) + k;

test('sessions, channels and bounds decode to the fixture session', () => {
  const [session, ...others] = parseSessions(document('sessions.json'));
  assert.equal(others.length, 0);
  assert.equal(session.id, 's1');
  assert.equal(session.class, 'other');
  assert.equal(session.start_wall_ns, START_WALL_NS.toString());
  assert.equal(session.end_wall_ns, (START_WALL_NS + 10_000_000_000n).toString());
  assert.deepEqual(session.files.map(f => f.path), [
    'manifest.json', 'observer.tlog', 'shooter.restart1.tlog',
    'shooter.segment0.tlog', 'shooter.segment1.tlog', 'shooter.tlog'
  ]);

  const channels = new Map(parseChannels(document('channels.json')).map(c => [c.name, c]));
  assert.equal(channels.get('shooter/state/position_m')?.unit, 'm');
  assert.ok(channels.has('shooter/state/shot_id'));
  assert.ok(channels.has('shooter/status/boot_id'));
  assert.ok(channels.has('hw/state/driver_station/enabled'));

  const bounds = parseBounds(document('bounds.json'));
  assert.equal(bounds.start_ns, START_NS.toString());
  assert.equal(bounds.end_ns, (START_NS + 10_002_000_000n).toString());
  assert.equal(wallTimeNs(bounds, bounds.start_ns), START_WALL_NS);
});

test('a snapshot keeps NaN, an integer past 2^53 exactly, and each sample time', () => {
  const snapshot = parseSnapshot(document('snapshot.json'));
  assert.equal(snapshot.t_ns, (START_NS + 10_002_000_000n).toString());
  const position = snapshot.channels.get('shooter/state/position_m')!;
  assert.ok(Number.isNaN(position.value));
  assert.equal(position.sample_ns, (START_NS + 9_980_500_000n).toString());
  const shotId = snapshot.channels.get('shooter/state/shot_id')!;
  assert.equal(shotId.exact, SHOT_ID(499n).toString());
  assert.equal(shotId.value, Number(SHOT_ID(499n)));
  // Sent once, at the first tick: its own time, not the snapshot's.
  const boot = snapshot.channels.get('shooter/status/boot_id')!;
  assert.equal(boot.value, 42);
  assert.equal(boot.sample_ns, (START_NS + 500_000n).toString());
});

test('a series holds empty buckets, a NaN last value and a channel with no data', () => {
  const buffer = golden('series.bin');
  const series = decodeSeries(
      buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.byteLength), 3);
  assert.equal(series.buckets, 4);
  const [position, shotId, boot] = series.channels;
  // [9.9 s, 10.1 s): shots 495..497, then 498 and 499 (whose position is
  // NaN), then nothing.
  assert.deepEqual([...position.count], [3, 2, 0, 0]);
  assert.deepEqual([...position.t_ns],
                   [Number(START_NS + 9_940_500_000n), Number(START_NS + 9_980_500_000n),
                    NaN, NaN]);
  assert.deepEqual([...position.min], [123.875, 124.625, NaN, NaN]);
  assert.deepEqual([...position.last], [124.375, NaN, NaN, NaN]);
  assert.deepEqual([...shotId.last], [Number(SHOT_ID(497n)), Number(SHOT_ID(499n)), NaN, NaN]);
  assert.deepEqual([...boot.count], [0, 0, 0, 0]);
  assert.deepEqual([...boot.t_ns], [NaN, NaN, NaN, NaN]);

  const rows = bucketSeries(['position', 'shot_id', 'boot'], series, START_NS.toString(),
                            START_NS, START_NS + 10_100_000_000n);
  assert.deepEqual([...rows.times], [9.9405, 9.9805]);
  assert.deepEqual([...rows.values.boot], [NaN, NaN]);
});
