// The game pieces in the 3D viewport (REQ-DEV-43): a bridge message decoded
// (keyframe, then a delta of only what changed) and the instance transforms
// the viewport draws, in the field frame the field model is drawn in, with
// pieces out of play not drawn.

// REQ-DEV-43
import assert from 'node:assert/strict';
import {test} from 'node:test';

import * as THREE from 'three';

import {MAX_PIECES, PiecesDecoder, pieceMatrices} from '../src/data/pieces';
import {FIELD_MODEL_ROTATION_X} from '../src/panels/pose3d/field_model';
import {piecesMessage} from './pieces_message';

test('a keyframe, then a delta, decode to the latest view', () => {
  const decoder = new PiecesDecoder();
  // A delta before any keyframe is no view.
  assert.equal(decoder.apply(piecesMessage(false, 3, 0.075, 1n, [])), false);
  assert.equal(decoder.snapshot(), null);
  assert.ok(decoder.apply(piecesMessage(true, 3, 0.075, 10n, [
    {index: 0, in_play: true, x_mm: 1000, y_mm: 2000, z_mm: 75},
    {index: 1, in_play: true, x_mm: -1500, y_mm: 0, z_mm: 75},
    {index: 2, in_play: false, x_mm: 500, y_mm: -2500, z_mm: 75}])));
  assert.ok(decoder.apply(piecesMessage(false, 3, 0.075, 20n, [
    {index: 1, in_play: false, x_mm: -1499, y_mm: 1, z_mm: 75}])));
  const view = decoder.snapshot()!;
  assert.equal(view.count, 3);
  assert.equal(view.sample_time_ns, '20');
  assert.ok(Math.abs(view.radius_m - 0.075) < 1e-7);
  assert.deepEqual([...view.in_play], [1, 0, 0]);
  const near = (actual: number, expected: number) =>
      assert.ok(Math.abs(actual - expected) < 1e-6, `${actual} != ${expected}`);
  [1, 2, 0.075, -1.499, 0.001, 0.075, 0.5, -2.5, 0.075].forEach(
      (expected, i) => near(view.positions[i], expected));

  // Bytes that are not a whole message leave the view as it was.
  const torn = piecesMessage(false, 3, 0.075, 30n, [
    {index: 0, in_play: true, x_mm: 0, y_mm: 0, z_mm: 0}]).slice(0, 30);
  assert.throws(() => decoder.apply(torn));
  assert.throws(() => decoder.apply(piecesMessage(false, 3, 0.075, 30n, [
    {index: 3, in_play: true, x_mm: 0, y_mm: 0, z_mm: 0}])));
  assert.equal(decoder.snapshot()!.sample_time_ns, '20');
});

test('the instances are the pieces in play, where the field model has them', () => {
  const decoder = new PiecesDecoder();
  decoder.apply(piecesMessage(true, 3, 0.075, 1n, [
    {index: 0, in_play: true, x_mm: 4626, y_mm: 4035, z_mm: 1364},
    {index: 1, in_play: false, x_mm: 500, y_mm: -2500, z_mm: 75},
    {index: 2, in_play: true, x_mm: 8270, y_mm: 1000, z_mm: 75}]));
  const out = new Float32Array(16 * MAX_PIECES).fill(7);
  assert.equal(pieceMatrices(decoder.snapshot()!, out), 2, 'piece 1 is out of play');

  // A point of the field model, in its glTF frame (+y up), placed as the
  // viewport places the model: where it lands is where a piece at that
  // field position is drawn.
  const field = new THREE.Group();
  field.rotation.x = FIELD_MODEL_ROTATION_X;
  const marker = new THREE.Object3D();
  marker.position.set(4.626, 1.364, -4.035);  // field (x, y, z) as glTF (x, z, -y)
  field.add(marker);
  field.updateMatrixWorld(true);
  const expected = new THREE.Vector3().setFromMatrixPosition(marker.matrixWorld);

  const first = new THREE.Matrix4().fromArray(out, 0);
  const position = new THREE.Vector3(), scale = new THREE.Vector3();
  const rotation = new THREE.Quaternion();
  first.decompose(position, rotation, scale);
  assert.ok(position.distanceTo(expected) < 1e-6,
            `piece at ${position.toArray()}, field model point at ${expected.toArray()}`);
  assert.ok(Math.abs(scale.x - 0.075) < 1e-7 && Math.abs(scale.y - 0.075) < 1e-7 &&
            Math.abs(scale.z - 0.075) < 1e-7);
  assert.ok(Math.abs(rotation.w - 1) < 1e-9);
  // The second instance is piece 2.
  const second = new THREE.Vector3().setFromMatrixPosition(new THREE.Matrix4().fromArray(out, 16));
  assert.ok(second.distanceTo(new THREE.Vector3(8.27, 1, 0.075)) < 1e-6);
});

test('PiecesDecoder rejects non-finite or negative radii, excessive counts, duplicates, and malformed framing without corrupting state', () => {
  const decoder = new PiecesDecoder();
  assert.ok(decoder.apply(piecesMessage(true, 2, 0.075, 10n, [
    {index: 0, in_play: true, x_mm: 1000, y_mm: 2000, z_mm: 75},
    {index: 1, in_play: false, x_mm: 0, y_mm: 0, z_mm: 0}])));

  // Non-finite (including +Infinity) and negative radii must be rejected.
  for (const badRadius of [Number.POSITIVE_INFINITY, Number.NEGATIVE_INFINITY, NaN, -0.001]) {
    assert.throws(() => decoder.apply(piecesMessage(true, 1, badRadius, 11n, [
      {index: 0, in_play: true, x_mm: 0, y_mm: 0, z_mm: 0}])));
  }

  // Excessive piece counts past MAX_PIECES and records > count.
  assert.throws(() => decoder.apply(piecesMessage(true, MAX_PIECES + 1, 0.075, 12n, [])));
  assert.throws(() => decoder.apply(piecesMessage(true, 0xffffffff, 0.075, 12n, [])));
  assert.throws(() => decoder.apply(piecesMessage(true, 1, 0.075, 13n, [
    {index: 0, in_play: true, x_mm: 0, y_mm: 0, z_mm: 0},
    {index: 0, in_play: false, x_mm: 1, y_mm: 1, z_mm: 1}])));

  // Duplicate piece index within one message (both keyframe and delta).
  assert.throws(() => decoder.apply(piecesMessage(true, 2, 0.075, 14n, [
    {index: 0, in_play: true, x_mm: 10, y_mm: 20, z_mm: 30},
    {index: 0, in_play: false, x_mm: 40, y_mm: 50, z_mm: 60}])));
  assert.throws(() => decoder.apply(piecesMessage(false, 2, 0.075, 15n, [
    {index: 1, in_play: true, x_mm: 10, y_mm: 20, z_mm: 30},
    {index: 1, in_play: true, x_mm: 40, y_mm: 50, z_mm: 60}])));

  // Wrong magic, truncated header, and trailing garbage byte.
  const badMagic = piecesMessage(true, 0, 0.075, 16n, []);
  badMagic[3] = 0x32;  // "PCS2"
  assert.throws(() => decoder.apply(badMagic));
  assert.throws(() => decoder.apply(piecesMessage(true, 0, 0.075, 16n, []).slice(0, 23)));
  const withTrailing = new Uint8Array(25);
  withTrailing.set(piecesMessage(true, 0, 0.075, 16n, []), 0);
  assert.throws(() => decoder.apply(withTrailing));

  // Prior valid view stays intact after every failed decode.
  const kept = decoder.snapshot()!;
  assert.equal(kept.sample_time_ns, '10');
  assert.equal(kept.count, 2);
  assert.deepEqual([...kept.in_play], [1, 0]);
  assert.ok(Math.abs(kept.positions[0] - 1) < 1e-6);

  // Negative zero radius normalizes to +0.
  assert.ok(decoder.apply(piecesMessage(true, 0, -0, 17n, [])));
  assert.ok(Object.is(decoder.snapshot()!.radius_m, 0));
});

test('PiecesDecoder handles unaligned non-zero byteOffset views, mismatched deltas, and full reset', () => {
  const decoder = new PiecesDecoder();
  const msg = piecesMessage(true, 2, 0.1, 100n, [
    {index: 0, in_play: true, x_mm: -2147483648, y_mm: 2147483647, z_mm: -500},
    {index: 1, in_play: true, x_mm: 1000, y_mm: 2000, z_mm: 3000}]);
  // Place the message at an unaligned byteOffset = 3 inside a larger buffer.
  const backing = new Uint8Array(msg.length + 7);
  backing.fill(0xaa);
  backing.set(msg, 3);
  assert.ok(decoder.apply(backing.subarray(3, 3 + msg.length)));
  const snap = decoder.snapshot()!;
  assert.equal(snap.sample_time_ns, '100');
  assert.ok(Number.isFinite(snap.positions[0]) && snap.positions[0] < -2_000_000);
  assert.ok(Number.isFinite(snap.positions[1]) && snap.positions[1] > 2_000_000);
  assert.ok(Math.abs(snap.positions[2] + 0.5) < 1e-6);

  // Delta with mismatched count or radius is dropped without mutating state.
  assert.equal(decoder.apply(piecesMessage(false, 3, 0.1, 101n, [])), false);
  assert.equal(decoder.apply(piecesMessage(false, 2, 0.2, 102n, [
    {index: 0, in_play: false, x_mm: 0, y_mm: 0, z_mm: 0}])), false);
  assert.equal(decoder.snapshot()!.sample_time_ns, '100');
  assert.ok(Math.abs(decoder.snapshot()!.radius_m - 0.1) < 1e-7);
  assert.deepEqual([...decoder.snapshot()!.in_play], [1, 1]);

  // Reset clears the view, drops deltas, and leaves no stale piece data.
  decoder.reset();
  assert.equal(decoder.snapshot(), null);
  assert.equal(decoder.apply(piecesMessage(false, 2, 0.1, 103n, [])), false);
  assert.ok(decoder.apply(piecesMessage(true, 2, 0.1, 104n, [
    {index: 0, in_play: true, x_mm: 500, y_mm: 0, z_mm: 0}])));
  const fresh = decoder.snapshot()!;
  assert.deepEqual([...fresh.in_play], [1, 0]);
  assert.deepEqual([...fresh.positions.slice(3, 6)], [0, 0, 0]);
});

test('pieceMatrices skips non-finite coordinates, rejects bad radii, and clamps counts to snapshot and output capacity', () => {
  const out = new Float32Array(16 * 4).fill(9);

  // Non-finite or negative radius draws nothing without touching `out`.
  for (const radius_m of [NaN, Infinity, -Infinity, -0.05]) {
    const n = pieceMatrices({
      count: 1, radius_m, sample_time_ns: '1',
      positions: new Float32Array([1, 2, 3]), in_play: new Uint8Array([1])
    }, out);
    assert.equal(n, 0);
  }
  assert.equal(out[0], 9);

  // Non-finite coordinates are skipped while valid in-play pieces are written.
  const nValid = pieceMatrices({
    count: 4, radius_m: 0.075, sample_time_ns: '2',
    positions: new Float32Array([
      NaN, 0, 0,
      1, Infinity, 0,
      2, 3, -Infinity,
      4, 5, 6
    ]),
    in_play: new Uint8Array([1, 1, 1, 1])
  }, out);
  assert.equal(nValid, 1);
  assert.equal(out[12], 4);
  assert.equal(out[13], 5);
  assert.equal(out[14], 6);
  for (let i = 0; i < 16; i++) assert.ok(Number.isFinite(out[i]));

  // Excessive count beyond typed array lengths and MAX_PIECES never overflows
  // or treats out-of-bounds undefined in_play slots as in-play.
  const nClamped = pieceMatrices({
    count: MAX_PIECES + 500, radius_m: 0.1, sample_time_ns: '3',
    positions: new Float32Array([1, 2, 3]),
    in_play: new Uint8Array([1, 1, 1])
  }, out);
  assert.equal(nClamped, 1);

  // Non-integer or negative counts write nothing.
  for (const badCount of [-1, NaN, Infinity, 1.5]) {
    assert.equal(pieceMatrices({
      count: badCount, radius_m: 0.1, sample_time_ns: '4',
      positions: new Float32Array([1, 2, 3]), in_play: new Uint8Array([1])
    }, out), 0);
  }

  // Output buffer smaller than in-play count caps written instances cleanly.
  const tiny = new Float32Array(16);
  const nCapped = pieceMatrices({
    count: 2, radius_m: 0.1, sample_time_ns: '5',
    positions: new Float32Array([1, 2, 3, 4, 5, 6]),
    in_play: new Uint8Array([1, 1])
  }, tiny);
  assert.equal(nCapped, 1);
  assert.equal(tiny[12], 1);
});
