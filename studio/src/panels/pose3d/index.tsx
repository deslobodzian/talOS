// The 3D spatial viewport: the chassis pose in three dimensions, with an
// optional user-loaded field model and, under the MuJoCo gateway, the game
// pieces in play as one instanced sphere mesh (data/pieces.ts). The old Spatial view, reading its pose
// from useChassis(); a pose whose frame is no longer current is drawn
// translucent and marked STALE. The WebGL canvas only renders in a browser;
// under react-dom/server it renders a placeholder, so the registry test can
// render every panel without a WebGL context.

import {useEffect, useRef, useState} from 'react';
import {Canvas, useThree} from '@react-three/fiber';
import * as THREE from 'three';
import {OrbitControls} from 'three/examples/jsm/controls/OrbitControls.js';

import {useChassis, usePieces} from '../../data/hooks';
import {MAX_PIECES, pieceMatrices} from '../../data/pieces';
import {FIELD_HEIGHT_M, FIELD_WIDTH_M} from '../field/geometry';
import type {PanelDefinition} from '../types';
import {FieldModelLoader, isFinitePose} from './field_model';
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

// The game pieces in play, one instanced unit sphere scaled by their
// radius. The mesh and its transforms are allocated once per mount, for
// MAX_PIECES; each pieces update (a new message while live, the pieces at
// another playhead time otherwise) rewrites them in place and asks for one
// frame.
function GamePieces() {
  const pieces = usePieces();
  const {invalidate} = useThree();
  const [mesh, setMesh] = useState<THREE.InstancedMesh|null>(null);
  useEffect(() => {
    const instanced = new THREE.InstancedMesh(
        new THREE.SphereGeometry(1, 16, 12),
        new THREE.MeshStandardMaterial({color: '#f2c94c'}), MAX_PIECES);
    instanced.count = 0;
    instanced.frustumCulled = false;
    setMesh(instanced);
    return () => {
      instanced.geometry.dispose();
      (instanced.material as THREE.Material).dispose();
      instanced.dispose();
    };
  }, []);
  useEffect(() => {
    if (!mesh) return;
    mesh.count = pieces ? pieceMatrices(pieces, mesh.instanceMatrix.array as Float32Array) : 0;
    mesh.instanceMatrix.needsUpdate = true;
    invalidate();
  }, [pieces, mesh, invalidate]);
  return mesh ? <primitive object={mesh}/> : null;
}

function Pose3dPanel() {
  const reading = useChassis();
  const validReading = reading && isFinitePose(reading.pose) ? reading : null;
  const [model, setModel] = useState<THREE.Group|null>(null);
  const [error, setError] = useState('');
  const isBrowser = typeof window !== 'undefined';
  const loaderRef = useRef<FieldModelLoader|null>(null);

  useEffect(() => {
    const loader = new FieldModelLoader({
      onLoad(scene) {
        setModel(scene);
        setError('');
      },
      onError(message) {
        setError(message);
      }
    });
    loaderRef.current = loader;
    return () => {
      loaderRef.current = null;
      loader.destroy();
    };
  }, []);

  useEffect(() => {
    loaderRef.current?.retain(model);
  }, [model]);

  return <>
    <div className="spatial">
      {isBrowser ?
           <Canvas camera={viewportCamera} frameloop="demand">
             <OrbitNavigation/>
             <ambientLight intensity={1.5}/>
             <directionalLight position={[4, 2, 12]} intensity={2}/>
             <mesh position={[FIELD_WIDTH_M / 2, FIELD_HEIGHT_M / 2, -.04]}>
               <boxGeometry args={[FIELD_WIDTH_M, FIELD_HEIGHT_M, .08]}/>
               <meshStandardMaterial color="#182d3c"/>
             </mesh>
             <gridHelper args={[18, 18, '#486676', '#29424e']}
                         rotation={[Math.PI / 2, 0, 0]}
                         position={[FIELD_WIDTH_M / 2, FIELD_HEIGHT_M / 2, 0]}/>
             {model && <primitive object={model}/>}
             <GamePieces/>
             {validReading && <group
                 position={[validReading.pose.x, validReading.pose.y,
                            validReading.pose.z + .2]}
                 rotation={[validReading.pose.roll, validReading.pose.pitch,
                            validReading.pose.yaw, 'ZYX']}>
               <mesh>
                 <boxGeometry args={[.8, .7, .4]}/>
                 <meshStandardMaterial color="#48e1cf" transparent
                                       opacity={validReading.stale ? .35 : 1}/>
               </mesh>
             </group>}
           </Canvas> :
           <div className="spatial-ssr" role="status">
             3D viewport needs a browser.
           </div>}
    </div>
    <div className="panel-footer">
      Z UP · METERS
      {!validReading && <span role="status">Pose unavailable</span>}
      {validReading?.stale && <span className="text-warn" role="status">Pose STALE</span>}
      {isBrowser && <label className="file">
        Load field GLB
        <input
            aria-label="Load self-contained field model in meters, glTF Y-up"
            type="file" accept=".glb" onChange={(e) => {
              const f = e.currentTarget.files?.[0];
              e.currentTarget.value = '';
              if (f) void loaderRef.current?.load(f);
            }}/>
      </label>}
    </div>
    {error && <small role="alert">{error}</small>}
  </>;
}

export const pose3dPanel: PanelDefinition<Pose3dConfig> = {
  id: 'pose3d',
  title: '3D Viewport',
  category: 'Spatial',
  description: 'The chassis pose in three dimensions, with an optional field model and the simulated game pieces.',
  defaultConfig,
  parseConfig,
  component: Pose3dPanel,
  recorded: true
};
