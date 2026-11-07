// The 3D spatial viewport: the chassis pose in three dimensions, with an
// optional user-loaded field model. The old Spatial view, reading its pose
// from useSnapshot(). The WebGL canvas only renders in a browser; under
// react-dom/server it renders a placeholder, so the registry test can render
// every panel without a WebGL context.

import {useEffect, useRef, useState} from 'react';
import {Canvas, useThree} from '@react-three/fiber';
import * as THREE from 'three';
import {OrbitControls} from 'three/examples/jsm/controls/OrbitControls.js';
import {GLTFLoader} from 'three/examples/jsm/loaders/GLTFLoader.js';

import {poseValid} from '../../core';
import {useSnapshot} from '../../data/hooks';
import type {PanelDefinition} from '../types';
import './pose3d.css';

export type Pose3dConfig = Record<string, never>;

const defaultConfig: Pose3dConfig = {};
const parseConfig = (_raw: unknown): Pose3dConfig => ({});

const viewportCamera = {
  position: [13, -12, 12] as [number, number, number],
  up: [0, 0, 1] as [number, number, number],
  fov: 48
};

function OrbitNavigation() {
  const {camera, gl, invalidate} = useThree();
  useEffect(() => {
    const controls = new OrbitControls(camera, gl.domElement);
    controls.target.set(8.25, 4.1, 0);
    controls.maxPolarAngle = Math.PI / 2 - .01;
    controls.minDistance = 2;
    controls.maxDistance = 40;
    const redraw = () => invalidate();
    controls.addEventListener('change', redraw);
    controls.update();
    return () => {
      controls.removeEventListener('change', redraw);
      controls.dispose();
    };
  }, [camera, gl, invalidate]);
  return null;
}

function disposeModel(model: THREE.Group) {
  model.traverse(object => {
    if (object instanceof THREE.Mesh) {
      object.geometry.dispose();
      const materials = Array.isArray(object.material) ? object.material :
                                                         [object.material];
      materials.forEach(material => {
        for (const value of Object.values(material))
          if (value instanceof THREE.Texture) value.dispose();
        material.dispose();
      });
    }
  });
}

function Pose3dPanel() {
  const s = useSnapshot();
  const valid = poseValid(s);
  const [model, setModel] = useState<THREE.Group|null>(null);
  const [error, setError] = useState('');
  const isBrowser = typeof window !== 'undefined';
  const requestRef = useRef(0);
  const mountedRef = useRef(false);

  useEffect(() => {
    mountedRef.current = true;
    return () => {
      mountedRef.current = false;
      requestRef.current++;
    };
  }, []);

  useEffect(() => () => {
    if (model) disposeModel(model);
  }, [model]);

  return <>
    <div className="spatial">
      {isBrowser ?
           <Canvas camera={viewportCamera} frameloop="demand">
             <OrbitNavigation/>
             <ambientLight intensity={1.5}/>
             <directionalLight position={[4, 2, 12]} intensity={2}/>
             <mesh position={[8.25, 4.1, -.04]}>
               <boxGeometry args={[16.5, 8.2, .08]}/>
               <meshStandardMaterial color="#182d3c"/>
             </mesh>
             <gridHelper args={[18, 18, '#486676', '#29424e']}
                         rotation={[Math.PI / 2, 0, 0]} position={[8.25, 4.1, 0]}/>
             {model && <primitive object={model}/>}
             {s && <group visible={valid}
                          position={[s.chassis.x, s.chassis.y, s.chassis.z + .2]}
                          rotation={[s.chassis.roll, s.chassis.pitch, s.chassis.yaw,
                                     'ZYX']}>
               <mesh>
                 <boxGeometry args={[.8, .7, .4]}/>
                 <meshStandardMaterial color="#48e1cf"/>
               </mesh>
             </group>}
           </Canvas> :
           <div className="spatial-ssr" role="status">
             3D viewport needs a browser.
           </div>}
    </div>
    <div className="panel-footer">
      Z UP · METERS
      {!valid && <span role="status">Pose unavailable</span>}
      {isBrowser && <label className="file">
        Load field GLB
        <input
            aria-label="Load self-contained field model in meters, glTF Y-up"
            type="file" accept=".glb" onChange={async (e) => {
              const f = e.target.files?.[0];
              if (!f) return;
              const request = ++requestRef.current;
              try {
                const gltf =
                    await new GLTFLoader().parseAsync(await f.arrayBuffer(), '');
                if (!mountedRef.current || request !== requestRef.current) {
                  disposeModel(gltf.scene);
                  return;
                }
                gltf.scene.rotation.x = Math.PI / 2;
                setModel(gltf.scene);
                setError('');
              } catch {
                if (mountedRef.current && request === requestRef.current)
                  setError('Could not load model');
              }
            }}/>
      </label>}
    </div>
    {error && <small>{error}</small>}
  </>;
}

export const pose3dPanel: PanelDefinition<Pose3dConfig> = {
  id: 'pose3d',
  title: '3D Viewport',
  category: 'Spatial',
  description: 'The chassis pose in three dimensions, with an optional field model.',
  defaultConfig,
  parseConfig,
  component: Pose3dPanel
};
