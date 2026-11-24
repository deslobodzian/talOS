// The 3D viewport's field model: a self-contained glTF binary (Y-up, metres)
// turned into the viewport's Z-up field frame. The panel and its test load
// a GLB through this one function, so the test holds the panel's frame.

import type * as THREE from 'three';
import {GLTFLoader} from 'three/examples/jsm/loaders/GLTFLoader.js';

// glTF's +y up turned to the field frame's +z up, about x. What else the
// viewport draws in the field frame (the game pieces, data/pieces.ts) needs
// no turn.
export const FIELD_MODEL_ROTATION_X = Math.PI / 2;

export async function loadFieldModel(glb: ArrayBuffer): Promise<THREE.Group> {
  const gltf = await new GLTFLoader().parseAsync(glb, '');
  gltf.scene.rotation.x = FIELD_MODEL_ROTATION_X;
  return gltf.scene;
}
