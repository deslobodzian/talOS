// The 3D viewport's field model: a self-contained glTF binary (Y-up, metres)
// turned into the viewport's Z-up field frame. The panel and its test load
// a GLB through this one function, so the test holds the panel's frame.

import * as THREE from 'three';
import {GLTFLoader} from 'three/examples/jsm/loaders/GLTFLoader.js';

import type {Pose} from '../../core';

// glTF's +y up turned to the field frame's +z up, about x. What else the
// viewport draws in the field frame (the game pieces, data/pieces.ts) needs
// no turn.
export const FIELD_MODEL_ROTATION_X = Math.PI / 2;

// Upper bound on a user-selected field GLB (64 MiB): far above any real
// field model while preventing multi-gigabyte allocations from corrupt or
// accidental picks.
export const MAX_FIELD_MODEL_BYTES = 64 * 1024 * 1024;

const GLB_MAGIC = 0x46546c67;       // "glTF"
const GLB_VERSION = 2;
const GLB_CHUNK_JSON = 0x4e4f534a;  // "JSON"
const GLB_HEADER_BYTES = 12;
const GLB_CHUNK_HEADER_BYTES = 8;

export function isFinitePose(pose: Pose): boolean {
  return Number.isFinite(pose.x) && Number.isFinite(pose.y) &&
      Number.isFinite(pose.z) && Number.isFinite(pose.roll) &&
      Number.isFinite(pose.pitch) && Number.isFinite(pose.yaw);
}

function validateGlb(glb: ArrayBuffer): void {
  const total = glb.byteLength;
  if (total < GLB_HEADER_BYTES + GLB_CHUNK_HEADER_BYTES ||
      total > MAX_FIELD_MODEL_BYTES)
    throw Error('Invalid GLB size');
  const view = new DataView(glb);
  if (view.getUint32(0, true) !== GLB_MAGIC)
    throw Error('Not a GLB binary');
  if (view.getUint32(4, true) !== GLB_VERSION)
    throw Error('Unsupported GLB version');
  const declaredLength = view.getUint32(8, true);
  if (declaredLength !== total)
    throw Error('Truncated or mismatched GLB length');

  let offset = GLB_HEADER_BYTES;
  let jsonBytes: Uint8Array|null = null;
  while (offset < total) {
    if (total - offset < GLB_CHUNK_HEADER_BYTES)
      throw Error('Truncated GLB chunk header');
    const chunkLength = view.getUint32(offset, true);
    const chunkType = view.getUint32(offset + 4, true);
    offset += GLB_CHUNK_HEADER_BYTES;
    if (chunkLength > total - offset)
      throw Error('Truncated GLB chunk');
    if (jsonBytes === null) {
      if (chunkType !== GLB_CHUNK_JSON || chunkLength === 0)
        throw Error('GLB must begin with a non-empty JSON chunk');
      jsonBytes = new Uint8Array(glb, offset, chunkLength);
    }
    offset += chunkLength;
  }
  if (!jsonBytes) throw Error('GLB missing JSON chunk');

  let parsed: unknown;
  try {
    const text = new TextDecoder('utf-8', {fatal: true}).decode(jsonBytes);
    parsed = JSON.parse(text);
  } catch {
    throw Error('Malformed GLB JSON');
  }
  if (!parsed || typeof parsed !== 'object' || Array.isArray(parsed))
    throw Error('Malformed GLB JSON root');
  const doc = parsed as Record<string, unknown>;
  for (const key of ['buffers', 'images'] as const) {
    const entries = doc[key];
    if (entries === undefined) continue;
    if (!Array.isArray(entries)) throw Error(`Malformed GLB ${key}`);
    for (const entry of entries) {
      if (!entry || typeof entry !== 'object' || Array.isArray(entry))
        throw Error(`Malformed GLB ${key} entry`);
      const uri = (entry as Record<string, unknown>).uri;
      if (uri !== undefined && (typeof uri !== 'string' || !uri.startsWith('data:')))
        throw Error('Field model must be self-contained');
    }
  }
}

type ResourceSets = {
  geometries: Set<THREE.BufferGeometry>;
  materials: Set<THREE.Material>;
  textures: Set<THREE.Texture>;
  instancedMeshes: Set<THREE.InstancedMesh>;
  skeletons: Set<THREE.Skeleton>;
};

function collectResources(root: THREE.Object3D): ResourceSets {
  const geometries = new Set<THREE.BufferGeometry>();
  const materials = new Set<THREE.Material>();
  const textures = new Set<THREE.Texture>();
  const instancedMeshes = new Set<THREE.InstancedMesh>();
  const skeletons = new Set<THREE.Skeleton>();

  const collectTexture = (value: unknown) => {
    if (value instanceof THREE.Texture) {
      textures.add(value);
    } else if (Array.isArray(value)) {
      for (const item of value) {
        if (item instanceof THREE.Texture) textures.add(item);
      }
    }
  };

  root.traverse(object => {
    if (object instanceof THREE.InstancedMesh) instancedMeshes.add(object);
    if (object instanceof THREE.SkinnedMesh && object.skeleton)
      skeletons.add(object.skeleton);
    if (object instanceof THREE.Mesh || object instanceof THREE.Line ||
        object instanceof THREE.Points) {
      if (object.geometry instanceof THREE.BufferGeometry)
        geometries.add(object.geometry);
    }
    if (object instanceof THREE.Mesh || object instanceof THREE.Line ||
        object instanceof THREE.Points || object instanceof THREE.Sprite) {
      const raw: THREE.Material|THREE.Material[]|null|undefined = object.material;
      const mats = Array.isArray(raw) ? raw : raw ? [raw] : [];
      for (const material of mats) {
        if (!material) continue;
        materials.add(material);
        for (const value of Object.values(material)) collectTexture(value);
        if (material instanceof THREE.ShaderMaterial && material.uniforms) {
          for (const uniform of Object.values(material.uniforms)) {
            if (uniform && typeof uniform === 'object' && 'value' in uniform)
              collectTexture(uniform.value);
          }
        }
      }
    }
  });
  return {geometries, materials, textures, instancedMeshes, skeletons};
}

// Disposes every geometry, material, texture, skeleton and instanced-mesh
// buffer reachable from `model`, each unique resource once. When `exclude`
// is provided (e.g. a kept scene from a multi-scene GLB), resources shared
// with `exclude` are left intact.
export function disposeFieldModel(model: THREE.Object3D,
                                  exclude?: THREE.Object3D): void {
  const target = collectResources(model);
  const keep = exclude ? collectResources(exclude) : null;
  const keepImages = new Set<unknown>();
  if (keep) {
    for (const tex of keep.textures) {
      if (tex.image) keepImages.add(tex.image);
    }
  }
  const closedImages = new Set<unknown>();
  for (const tex of target.textures) {
    if (keep?.textures.has(tex)) continue;
    tex.dispose();
    const img: unknown = tex.image;
    if (img && typeof img === 'object' && !keepImages.has(img) &&
        !closedImages.has(img) && 'close' in img &&
        typeof (img as {close: unknown}).close === 'function') {
      closedImages.add(img);
      (img as {close(): void}).close();
    }
  }
  for (const mat of target.materials) {
    if (!keep?.materials.has(mat)) mat.dispose();
  }
  for (const geo of target.geometries) {
    if (!keep?.geometries.has(geo)) geo.dispose();
  }
  for (const skeleton of target.skeletons) {
    if (!keep?.skeletons.has(skeleton)) skeleton.dispose();
  }
  for (const instanced of target.instancedMeshes) {
    if (!keep?.instancedMeshes.has(instanced)) instanced.dispose();
  }
}

export async function loadFieldModel(glb: ArrayBuffer): Promise<THREE.Group> {
  validateGlb(glb);
  const gltf = await new GLTFLoader().parseAsync(glb, '');
  if (!gltf.scene) {
    for (const scene of gltf.scenes ?? []) {
      if (scene) disposeFieldModel(scene);
    }
    throw Error('Field model contains no scene');
  }
  for (const scene of gltf.scenes ?? []) {
    if (scene && scene !== gltf.scene) disposeFieldModel(scene, gltf.scene);
  }
  gltf.scene.rotation.x = FIELD_MODEL_ROTATION_X;
  return gltf.scene;
}

export type FieldModelSource = {
  readonly size: number;
  arrayBuffer(): Promise<ArrayBuffer>;
};

export type FieldModelCallbacks = {
  onLoad(scene: THREE.Group): void;
  onError(message: string): void;
};

// Coordinates asynchronous field-model loads for the 3D viewport: rejects
// oversized picks before reading them into memory, cancels superseded loads
// and disposes their parsed scenes immediately, keeps the previous model on
// error, and disposes every owned model on swap or unmount (even one that
// resolved before React committed it).
export class FieldModelLoader {
  private request = 0;
  private active = true;
  private readonly owned = new Set<THREE.Group>();

  constructor(
      private readonly callbacks: FieldModelCallbacks,
      private readonly parse: (glb: ArrayBuffer) => Promise<THREE.Group> = loadFieldModel) {}

  async load(source: FieldModelSource): Promise<void> {
    if (!this.active) return;
    const id = ++this.request;
    if (!Number.isFinite(source.size) || source.size <= 0 ||
        source.size > MAX_FIELD_MODEL_BYTES) {
      this.callbacks.onError('Could not load model');
      return;
    }
    try {
      const buffer = await source.arrayBuffer();
      if (!this.active || id !== this.request) return;
      const scene = await this.parse(buffer);
      if (!this.active || id !== this.request) {
        disposeFieldModel(scene);
        return;
      }
      this.owned.add(scene);
      this.callbacks.onLoad(scene);
    } catch {
      if (this.active && id === this.request)
        this.callbacks.onError('Could not load model');
    }
  }

  retain(current: THREE.Group|null): void {
    for (const model of this.owned) {
      if (model !== current) {
        this.owned.delete(model);
        disposeFieldModel(model);
      }
    }
  }

  destroy(): void {
    this.active = false;
    this.request++;
    for (const model of this.owned) disposeFieldModel(model);
    this.owned.clear();
  }
}
