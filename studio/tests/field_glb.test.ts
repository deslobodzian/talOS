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
import {createElement} from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import * as THREE from 'three';

import {createStaticStudio, StudioProvider} from '../src/data/studio';
import {pose3dPanel} from '../src/panels/pose3d';
import {
  disposeFieldModel,
  FIELD_MODEL_ROTATION_X,
  FieldModelLoader,
  isFinitePose,
  loadFieldModel,
  MAX_FIELD_MODEL_BYTES
} from '../src/panels/pose3d/field_model';

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
  disposeFieldModel(scene);
});

function makeGlb(doc: unknown, bin?: Uint8Array): ArrayBuffer {
  const rawJson = new TextEncoder().encode(JSON.stringify(doc));
  const jsonPad = (4 - (rawJson.length % 4)) % 4;
  const jsonLen = rawJson.length + jsonPad;
  const binPad = bin ? (4 - (bin.length % 4)) % 4 : 0;
  const binLen = bin ? bin.length + binPad : 0;
  const total = 12 + 8 + jsonLen + (bin ? 8 + binLen : 0);
  const out = new Uint8Array(total);
  const view = new DataView(out.buffer);
  view.setUint32(0, 0x46546c67, true);  // "glTF"
  view.setUint32(4, 2, true);
  view.setUint32(8, total, true);
  view.setUint32(12, jsonLen, true);
  view.setUint32(16, 0x4e4f534a, true);  // "JSON"
  out.set(rawJson, 20);
  out.fill(0x20, 20 + rawJson.length, 20 + jsonLen);
  if (bin) {
    const at = 20 + jsonLen;
    view.setUint32(at, binLen, true);
    view.setUint32(at + 4, 0x004e4942, true);  // "BIN\0"
    out.set(bin, at + 8);
  }
  return out.buffer;
}

function triangleBin(): Uint8Array {
  const bytes = new Uint8Array(36);
  const view = new DataView(bytes.buffer);
  const coords = [0, 0, 0, 1, 0, 0, 0, 1, 0];
  coords.forEach((v, i) => view.setFloat32(i * 4, v, true));
  return bytes;
}

function triangleGlb(multiScene = false): ArrayBuffer {
  const bin = triangleBin();
  return makeGlb({
    asset: {version: '2.0'},
    scene: 0,
    scenes: multiScene ? [{nodes: [0]}, {nodes: [1]}] : [{nodes: [0]}],
    nodes: multiScene ?
        [{name: 'tri', mesh: 0}, {name: 'unused', mesh: 1}] :
        [{name: 'tri', mesh: 0}],
    meshes: multiScene ?
        [
          {primitives: [{attributes: {POSITION: 0}}]},
          {primitives: [{attributes: {POSITION: 0}}]}
        ] :
        [{primitives: [{attributes: {POSITION: 0}}]}],
    accessors: [{
      bufferView: 0, componentType: 5126, count: 3, type: 'VEC3',
      min: [0, 0, 0], max: [1, 1, 0]
    }],
    bufferViews: [{buffer: 0, byteOffset: 0, byteLength: 36, target: 34962}],
    buffers: [{byteLength: 36}]
  }, bin);
}

function trackDispose(target: {dispose(): void}): () => number {
  let count = 0;
  const orig = target.dispose.bind(target);
  target.dispose = () => {
    count++;
    orig();
  };
  return () => count;
}

function deferred<T>() {
  let resolve!: (value: T) => void;
  let reject!: (reason?: unknown) => void;
  const promise = new Promise<T>((res, rej) => {
    resolve = res;
    reject = rej;
  });
  return {promise, resolve, reject};
}

test('loadFieldModel rejects corrupt, truncated, non-self-contained, and scene-less GLBs, and loads valid GLBs into Z-up', async () => {
  // Empty and short buffers.
  await assert.rejects(() => loadFieldModel(new ArrayBuffer(0)), /Invalid GLB size/);
  await assert.rejects(() => loadFieldModel(new ArrayBuffer(12)), /Invalid GLB size/);

  // Wrong magic and wrong version.
  const badMagic = new Uint8Array(triangleGlb());
  badMagic[0] = 0x00;
  await assert.rejects(() => loadFieldModel(badMagic.buffer), /Not a GLB binary/);

  const badVersion = new Uint8Array(triangleGlb());
  new DataView(badVersion.buffer).setUint32(4, 1, true);
  await assert.rejects(() => loadFieldModel(badVersion.buffer), /Unsupported GLB version/);

  // Mismatched declared total length and truncated chunk.
  const truncated = triangleGlb().slice(0, 28);
  await assert.rejects(() => loadFieldModel(truncated), /Truncated or mismatched GLB length/);

  const badChunk = new Uint8Array(24);
  const bcView = new DataView(badChunk.buffer);
  bcView.setUint32(0, 0x46546c67, true);
  bcView.setUint32(4, 2, true);
  bcView.setUint32(8, 24, true);
  bcView.setUint32(12, 100, true);  // chunk claims 100 bytes past 24-byte total
  bcView.setUint32(16, 0x4e4f534a, true);
  await assert.rejects(() => loadFieldModel(badChunk.buffer), /Truncated GLB chunk/);

  // Non-JSON first chunk and malformed JSON text.
  const binFirst = new Uint8Array(24);
  const bfView = new DataView(binFirst.buffer);
  bfView.setUint32(0, 0x46546c67, true);
  bfView.setUint32(4, 2, true);
  bfView.setUint32(8, 24, true);
  bfView.setUint32(12, 4, true);
  bfView.setUint32(16, 0x004e4942, true);
  await assert.rejects(() => loadFieldModel(binFirst.buffer), /JSON chunk/);

  const badJson = new Uint8Array(24);
  const bjView = new DataView(badJson.buffer);
  bjView.setUint32(0, 0x46546c67, true);
  bjView.setUint32(4, 2, true);
  bjView.setUint32(8, 24, true);
  bjView.setUint32(12, 4, true);
  bjView.setUint32(16, 0x4e4f534a, true);
  badJson.set(new TextEncoder().encode('{bad'), 20);
  await assert.rejects(() => loadFieldModel(badJson.buffer), /Malformed GLB JSON/);

  // External URIs on buffers or images are refused (must be self-contained).
  await assert.rejects(() => loadFieldModel(makeGlb({
    asset: {version: '2.0'}, scene: 0, scenes: [{}],
    buffers: [{byteLength: 4, uri: 'http://example.com/field.bin'}]
  })), /self-contained/);
  await assert.rejects(() => loadFieldModel(makeGlb({
    asset: {version: '2.0'}, scene: 0, scenes: [{}],
    images: [{uri: '../texture.png'}]
  })), /self-contained/);

  // Scene-less and out-of-bounds default scene GLBs are refused.
  await assert.rejects(() => loadFieldModel(makeGlb({
    asset: {version: '2.0'}, scenes: []
  })), /no scene/);
  await assert.rejects(() => loadFieldModel(makeGlb({
    asset: {version: '2.0'}, scene: 5, scenes: [{}]
  })), /no scene/);

  // Valid self-contained GLB loads with +y turned to +z.
  const scene = await loadFieldModel(triangleGlb());
  assert.equal(scene.rotation.x, FIELD_MODEL_ROTATION_X);
  scene.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(scene);
  nearBox(box, [0, 0, 0], [1, 0, 1], 'triangle');
  disposeFieldModel(scene);

  // Multi-scene GLB returns the default scene intact while disposing unused scenes.
  const multi = await loadFieldModel(triangleGlb(true));
  assert.ok(multi.getObjectByName('tri'));
  assert.equal(multi.getObjectByName('unused'), undefined);
  disposeFieldModel(multi);
});

test('disposeFieldModel disposes meshes, lines, points, sprites, instanced/skinned meshes, shader uniforms, and image bitmaps once each while respecting exclusions', () => {
  const root = new THREE.Group();
  const kept = new THREE.Group();

  let sharedImgClosed = 0;
  const sharedTex = new THREE.Texture();
  (sharedTex as {image: unknown}).image = {close() { sharedImgClosed++; }};
  const sharedTexDisposed = trackDispose(sharedTex);
  const sharedGeo = new THREE.BufferGeometry();
  const sharedGeoDisposed = trackDispose(sharedGeo);
  const sharedMat = new THREE.MeshStandardMaterial({map: sharedTex});
  const sharedMatDisposed = trackDispose(sharedMat);
  // Two meshes share the same geometry, material and texture.
  root.add(new THREE.Mesh(sharedGeo, sharedMat));
  root.add(new THREE.Mesh(sharedGeo, sharedMat));

  const lineGeo = new THREE.BufferGeometry();
  const lineMat = new THREE.LineBasicMaterial();
  const lineGeoDisposed = trackDispose(lineGeo);
  const lineMatDisposed = trackDispose(lineMat);
  root.add(new THREE.Line(lineGeo, lineMat));

  const pointsGeo = new THREE.BufferGeometry();
  const pointsMat = new THREE.PointsMaterial();
  const pointsGeoDisposed = trackDispose(pointsGeo);
  const pointsMatDisposed = trackDispose(pointsMat);
  root.add(new THREE.Points(pointsGeo, pointsMat));

  const spriteTex = new THREE.Texture();
  const spriteMat = new THREE.SpriteMaterial({map: spriteTex});
  const spriteTexDisposed = trackDispose(spriteTex);
  const spriteMatDisposed = trackDispose(spriteMat);
  const sprite = new THREE.Sprite(spriteMat);
  const spriteGeoDisposed = trackDispose(sprite.geometry);
  root.add(sprite);

  const instGeo = new THREE.BufferGeometry();
  const instMat = new THREE.MeshStandardMaterial();
  const inst = new THREE.InstancedMesh(instGeo, instMat, 2);
  const instDisposed = trackDispose(inst);
  const instGeoDisposed = trackDispose(instGeo);
  const instMatDisposed = trackDispose(instMat);
  root.add(inst);

  const skinGeo = new THREE.BufferGeometry();
  const skinMat = new THREE.MeshStandardMaterial();
  const skeleton = new THREE.Skeleton([new THREE.Bone()]);
  const skinned = new THREE.SkinnedMesh(skinGeo, skinMat);
  skinned.bind(skeleton);
  const skeletonDisposed = trackDispose(skeleton);
  root.add(skinned);

  const shaderTex1 = new THREE.Texture();
  const shaderTex2 = new THREE.Texture();
  const shaderMat = new THREE.ShaderMaterial({
    uniforms: {uOne: {value: shaderTex1}, uMany: {value: [shaderTex2]}}
  });
  const shaderTex1Disposed = trackDispose(shaderTex1);
  const shaderTex2Disposed = trackDispose(shaderTex2);
  const shaderMatDisposed = trackDispose(shaderMat);
  root.add(new THREE.Mesh(new THREE.BufferGeometry(), shaderMat));

  // A resource shared with `kept` must NOT be disposed when `exclude = kept`.
  const excludedGeo = new THREE.BufferGeometry();
  const excludedMat = new THREE.MeshStandardMaterial();
  const excludedGeoDisposed = trackDispose(excludedGeo);
  const excludedMatDisposed = trackDispose(excludedMat);
  root.add(new THREE.Mesh(excludedGeo, excludedMat));
  kept.add(new THREE.Mesh(excludedGeo, excludedMat));

  disposeFieldModel(root, kept);

  assert.equal(sharedGeoDisposed(), 1);
  assert.equal(sharedMatDisposed(), 1);
  assert.equal(sharedTexDisposed(), 1);
  assert.equal(sharedImgClosed, 1);
  assert.equal(lineGeoDisposed(), 1);
  assert.equal(lineMatDisposed(), 1);
  assert.equal(pointsGeoDisposed(), 1);
  assert.equal(pointsMatDisposed(), 1);
  assert.equal(spriteTexDisposed(), 1);
  assert.equal(spriteMatDisposed(), 1);
  assert.equal(spriteGeoDisposed(), 0, 'THREE.Sprite shares a module-singleton geometry');
  assert.equal(instDisposed(), 1);
  assert.equal(instGeoDisposed(), 1);
  assert.equal(instMatDisposed(), 1);
  assert.equal(skeletonDisposed(), 1);
  assert.equal(shaderTex1Disposed(), 1);
  assert.equal(shaderTex2Disposed(), 1);
  assert.equal(shaderMatDisposed(), 1);
  assert.equal(excludedGeoDisposed(), 0);
  assert.equal(excludedMatDisposed(), 0);

  disposeFieldModel(kept);
  assert.equal(excludedGeoDisposed(), 1);
  assert.equal(excludedMatDisposed(), 1);
});

test('FieldModelLoader handles model swaps, error recovery, oversized picks, async races, and unmount disposal', async () => {
  const makeTrackedScene = () => {
    const group = new THREE.Group();
    const geo = new THREE.BufferGeometry();
    const mat = new THREE.MeshStandardMaterial();
    group.add(new THREE.Mesh(geo, mat));
    return {group, disposed: trackDispose(geo)};
  };

  let current: THREE.Group|null = null;
  let lastError = '';
  let loadCount = 0;
  let errorCount = 0;
  const pending: Array<ReturnType<typeof deferred<THREE.Group>>> = [];

  const loader = new FieldModelLoader(
      {
        onLoad(scene) {
          loadCount++;
          current = scene;
          lastError = '';
        },
        onError(message) {
          errorCount++;
          lastError = message;
        }
      },
      () => {
        const d = deferred<THREE.Group>();
        pending.push(d);
        return d.promise;
      });

  const settle = () => new Promise<void>(resolve => setImmediate(resolve));
  const okSource = {size: 128, arrayBuffer: async () => new ArrayBuffer(128)};

  // 1. Load model A and retain it.
  const loadA = loader.load(okSource);
  await settle();
  const sceneA = makeTrackedScene();
  pending[0].resolve(sceneA.group);
  await loadA;
  assert.equal(current, sceneA.group);
  assert.equal(lastError, '');
  loader.retain(current);
  assert.equal(sceneA.disposed(), 0);

  // 2. Oversized pick fails immediately without reading arrayBuffer, keeping A intact.
  let readOversized = false;
  await loader.load({
    size: MAX_FIELD_MODEL_BYTES + 1,
    arrayBuffer: async () => {
      readOversized = true;
      return new ArrayBuffer(0);
    }
  });
  assert.equal(readOversized, false);
  assert.equal(lastError, 'Could not load model');
  assert.equal(current, sceneA.group);
  assert.equal(sceneA.disposed(), 0);

  // 3. Race: slow load B starts, then fast load C supersedes it and commits;
  //    when B later resolves, B is immediately disposed and never replaces C.
  const loadB = loader.load(okSource);
  await settle();
  const loadC = loader.load(okSource);
  await settle();
  const sceneC = makeTrackedScene();
  pending[2].resolve(sceneC.group);
  await loadC;
  assert.equal(current, sceneC.group);
  assert.equal(lastError, '');
  loader.retain(current);
  assert.equal(sceneA.disposed(), 1, 'previous model A disposed on swap commit');
  assert.equal(sceneC.disposed(), 0);

  const sceneB = makeTrackedScene();
  pending[1].resolve(sceneB.group);
  await loadB;
  assert.equal(sceneB.disposed(), 1, 'superseded model B disposed immediately');
  assert.equal(current, sceneC.group);

  // 4. Failed parse keeps working model C intact and reports error.
  const loadBad = loader.load(okSource);
  await settle();
  pending[3].reject(new Error('corrupt'));
  await loadBad;
  assert.equal(current, sceneC.group);
  assert.equal(sceneC.disposed(), 0);
  assert.equal(lastError, 'Could not load model');

  // 5. Uncommitted load D + in-flight load E + in-flight failing load F on destroy():
  //    C and uncommitted D are disposed on destroy(); E is disposed when it resolves
  //    post-unmount; F does not trigger onError post-unmount.
  const loadD = loader.load(okSource);
  await settle();
  const sceneD = makeTrackedScene();
  pending[4].resolve(sceneD.group);
  await loadD;
  // Intentionally do NOT call retain(sceneD) before destroy().
  const loadE = loader.load(okSource);
  await settle();
  const loadF = loader.load(okSource);
  await settle();

  const loadsBeforeDestroy = loadCount;
  const errorsBeforeDestroy = errorCount;
  loader.destroy();
  assert.equal(sceneC.disposed(), 1);
  assert.equal(sceneD.disposed(), 1, 'uncommitted model D disposed on destroy');

  const sceneE = makeTrackedScene();
  pending[5].resolve(sceneE.group);
  await loadE;
  pending[6].reject(new Error('late error'));
  await loadF;
  assert.equal(sceneE.disposed(), 1, 'post-unmount resolved model E disposed immediately');
  assert.equal(loadCount, loadsBeforeDestroy);
  assert.equal(errorCount, errorsBeforeDestroy);
});

test('isFinitePose and Pose3dPanel reject non-finite chassis poses', () => {
  const valid = {x: 1, y: 2, z: 0.1, roll: 0, pitch: 0.05, yaw: -1.2};
  assert.equal(isFinitePose(valid), true);
  for (const field of ['x', 'y', 'z', 'roll', 'pitch', 'yaw'] as const) {
    for (const bad of [NaN, Infinity, -Infinity]) {
      assert.equal(isFinitePose({...valid, [field]: bad}), false, `${field}=${bad}`);
    }
  }

  const renderPose = (chassis: typeof valid, clock_ns: string) => {
    const studio = createStaticStudio({
      frames: [{
        stream_id: '1', sequence_id: '1', timestamp_ns: '1000000000',
        dictionary_id: 1, chassis, channels: {}, units: {}, labels: {}
      }],
      playback: {cursor_ns: '1000000000', clock_ns}
    });
    return renderToStaticMarkup(createElement(StudioProvider, {
      studio,
      children: createElement(pose3dPanel.component, {
        config: pose3dPanel.defaultConfig,
        setConfig: () => {},
        instanceId: 'p3d'
      })
    }));
  };

  const okHtml = renderPose(valid, '1000000000');
  assert.doesNotMatch(okHtml, /Pose unavailable/);
  assert.doesNotMatch(okHtml, /Pose STALE/);

  const nanHtml = renderPose({...valid, x: NaN}, '1000000000');
  assert.match(nanHtml, /Pose unavailable/);
  assert.doesNotMatch(nanHtml, /Pose STALE/);

  const infStaleHtml = renderPose({...valid, yaw: Infinity}, '5000000000');
  assert.match(infStaleHtml, /Pose unavailable/);
  assert.doesNotMatch(infStaleHtml, /Pose STALE/);
});
