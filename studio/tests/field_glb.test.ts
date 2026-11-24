// The 2026 field's glTF binary in the 3D viewport (REQ-DEV-42): the GLB the
// field generator builds (talOS/simulation/mujoco/fields/rebuilt_2026),
// loaded through the panel's own loadFieldModel, lands in the viewport's
// Z-up field frame: origin at the blue wall's scoring-table corner, +x to
// red, +y away from the scoring table. Expected numbers are the field's own
// (inches, the drawings' unit), not read from the model.

// REQ-DEV-42
import assert from 'node:assert/strict';
import {existsSync, readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {test} from 'node:test';

import * as THREE from 'three';

import {loadFieldModel} from '../src/panels/pose3d/field_model';

const IN = 0.0254;
const FIELD_LENGTH = 651.22 * IN;
const FIELD_WIDTH = 317.69 * IN;
const MM = 0.001;

// The GLB is a Bazel output, named by FIELD_GLB (a runfiles path) in
// //studio:field_glb_test; `pnpm test`, which has no Bazel outputs, skips
// this file and says so.
const GLB = process.env.FIELD_GLB;

function fieldGlb(rel: string): ArrayBuffer {
  const candidates = [
    process.env.JS_BINARY__RUNFILES &&
        resolve(process.env.JS_BINARY__RUNFILES, '_main', rel),
    resolve('..', rel)
  ].filter((p): p is string => !!p && existsSync(p));
  assert.ok(candidates.length > 0, `no ${rel} in the runfiles`);
  const bytes = readFileSync(candidates[0]);
  return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
}

function near(actual: number, expected: number, what: string) {
  assert.ok(Math.abs(actual - expected) <= MM,
            `${what}: ${actual} m, expected ${expected} m`);
}

function nearBox(actual: THREE.Box3, min: number[], max: number[], what: string) {
  actual.min.toArray().forEach((v, i) => near(v, min[i], `${what} min[${i}]`));
  actual.max.toArray().forEach((v, i) => near(v, max[i], `${what} max[${i}]`));
}

test('the field GLB loads into the viewport\'s field frame',
     {skip: GLB ? false : 'FIELD_GLB unset: bazel test //studio:field_glb_test'},
     async () => {
  const scene = await loadFieldModel(fieldGlb(GLB ?? ''));
  scene.updateMatrixWorld(true);
  // The whole field: from the blue corral's back (0.05 m wall, 37.6 in
  // corral, 0.1 m backing) to the red one's, the guardrails' 0.2 m backing,
  // 0.1 m of floor-level structure under the carpet, up to the nets' tops
  // (120.36 in).
  const corral = 0.05 + 37.6 * IN + 0.1;
  nearBox(new THREE.Box3().setFromObject(scene),
          [-corral, -0.2, -0.1], [FIELD_LENGTH + corral, FIELD_WIDTH + 0.2,
                                  120.36 * IN],
          'scene');
  // Two elements no mirror or turn maps onto themselves: the red tower's
  // base plate (45.18 x 39.0 in at the red wall, centred 147.47 in from the
  // far guardrail) and the blue trench's tag panel on the scoring-table side
  // (on the hub's x, centred on the 50.34 in opening, 26.25 to 40.25 in).
  const named = (name: string) => {
    const object = scene.getObjectByName(name);
    assert.ok(object, `no node ${name}`);
    return new THREE.Box3().setFromObject(object);
  };
  const tower = named('red_tower_base');
  near(tower.min.x, FIELD_LENGTH - 45.18 * IN, 'red tower base min x');
  near(tower.max.x, FIELD_LENGTH, 'red tower base max x');
  near((tower.min.y + tower.max.y) / 2, FIELD_WIDTH - 147.47 * IN,
       'red tower base centre y');
  near(tower.max.y - tower.min.y, 39.0 * IN, 'red tower base width');
  const panel = named('blue_trench_outpost_panel');
  near((panel.min.x + panel.max.x) / 2, 182.11 * IN, 'trench panel centre x');
  near((panel.min.y + panel.max.y) / 2, 50.34 / 2 * IN, 'trench panel centre y');
  near(panel.min.z, 26.25 * IN, 'trench panel bottom');
  near(panel.max.z, 40.25 * IN, 'trench panel top');
  // Every drawn element is a mesh with normals (a viewer lights them).
  let meshes = 0;
  scene.traverse(object => {
    if (object instanceof THREE.Mesh) {
      meshes++;
      assert.ok(object.geometry.getAttribute('normal'), `${object.name} has no normals`);
    }
  });
  assert.equal(meshes, 192);
});
