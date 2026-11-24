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
