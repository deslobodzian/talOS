import {Canvas, useThree} from '@react-three/fiber';
import React, {useEffect, useMemo, useRef, useState} from 'react';
import {createRoot} from 'react-dom/client';
import * as THREE from 'three';
import {OrbitControls} from 'three/examples/jsm/controls/OrbitControls.js';
import {GLTFLoader} from 'three/examples/jsm/loaders/GLTFLoader.js';
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';

import type {Pose, Snapshot} from './core';
import './style.css';
import {isTabId, TABS, type TabId} from './tabs';
import {
  AgentTab,
  Empty,
  OverviewTab,
  SystemTab,
  type SystemState,
  TimingTab,
  type View,
} from './system_view';

const worker =
    new Worker(new URL('./worker.ts', import.meta.url), {type: 'module'});

const viewportCamera = {
  position: [13, -12, 12] as [number, number, number],
  up: [0, 0, 1] as [number, number, number],
  fov: 48
};
const palette = ['#48e1cf', '#ffbd69', '#b59bff', '#ff778b'];

const initialView: View = {
  snapshot: null,
  history: [],
  mode: 'PAUSED',
  bounds: null,
  stats: {dropped: 0, late: 0, invalid: 0},
  status: 'Disconnected'
};
const initialSystem: SystemState =
    {graph: null, rates: {}, error: '', declared: null, declaredError: ''};

// --- field, spatial, plot, tree ---------------------------------------------

function Field({view}: {view: View}) {
  const s = view.snapshot;
  const prior = s ? view.history
                        .filter(
                            h => BigInt(h.timestamp_ns) <
                                BigInt(s.timestamp_ns))
                        .at(-1) :
                    null;
  const velocity = prior && s ? {
    x: (s.chassis.x - prior.chassis.x) /
        (Number(BigInt(s.timestamp_ns) - BigInt(prior.timestamp_ns)) / 1e9),
    y: (s.chassis.y - prior.chassis.y) /
        (Number(BigInt(s.timestamp_ns) - BigInt(prior.timestamp_ns)) / 1e9)
  } :
                                null;
  const point = (p: Pose) => `${p.x * 50},${410 - p.y * 50}`;
  const robot = (p: Pose, color: string, name: string) => (
      <g key={name}
         transform={`translate(${p.x * 50} ${410 - p.y * 50}) rotate(${
             -p.yaw * 180 / Math.PI})`}>
        <rect x="-20" y="-17" width="40" height="34" rx="3"
              fill={color + '22'} stroke={color} strokeWidth="2"/>
        <path d="M 0 0 H 30 M 22 -5 L 30 0 L 22 5" fill="none" stroke={color}
              strokeWidth="2"/>
      </g>);
  return <>
    <div className="legend">
      <span><i style={{background: palette[0]}}/>Fused pose</span>
      {s?.ghosts.map((g, i) => <span key={g.name}>
                                <i style={{background: palette[(i + 1) % 4]}}/>
                                {g.name}
                              </span>)}
    </div>
    <svg className="field" viewBox="-25 -25 875 475"
         aria-label="FRC field, blue alliance origin, X away from blue wall and Y left">
      <defs>
        <pattern id="grid" width="50" height="50" patternUnits="userSpaceOnUse">
          <path d="M 50 0 H 0 V 50" fill="none" stroke="#273444"
                strokeWidth=".6"/>
        </pattern>
      </defs>
      <rect width="825" height="410" rx="4" fill="#101c29" stroke="#637386"/>
      <rect width="825" height="410" fill="url(#grid)"/>
      <path d="M 0 0 H 85 V 410 H 0" fill="#387bd31a" stroke="#387bd3"/>
      <path d="M 825 0 H 740 V 410 H 825" fill="#f66a6618" stroke="#f66a66"/>
      <path d="M 412.5 0 V 410" stroke="#7c8998" strokeDasharray="5 9"/>
      <polyline
          points={view.history
                      .filter(
                          h => !s ||
                              BigInt(h.timestamp_ns) <= BigInt(s.timestamp_ns))
                      .map(h => point(h.chassis))
                      .join(' ')}
          fill="none" stroke={palette[0]} strokeOpacity=".55" strokeWidth="2"/>
      {s?.targets.map(t => <g key={t.id}>
         <line x1={s.chassis.x * 50} y1={410 - s.chassis.y * 50}
               x2={t.pose.x * 50} y2={410 - t.pose.y * 50} stroke="#b59bff"
               strokeDasharray="3 6" opacity=".6"/>
         <rect x={t.pose.x * 50 - 5} y={410 - t.pose.y * 50 - 5} width="10"
               height="10" fill="#b59bff"/>
         <text x={t.pose.x * 50 + 9} y={410 - t.pose.y * 50 + 4} fill="#b59bff"
               fontSize="10">{t.id}</text>
       </g>)}
      {s?.ghosts.map((g, i) => robot(g.pose, palette[(i + 1) % 4], g.name))}
      {s && robot(s.chassis, palette[0], 'chassis')}
      {s && velocity &&
       <line x1={s.chassis.x * 50} y1={410 - s.chassis.y * 50}
             x2={(s.chassis.x + velocity.x) * 50}
             y2={410 - (s.chassis.y + velocity.y) * 50} stroke="#ffbd69"
             strokeWidth="2"/>}
      <path d="M 5 405 H 60 M 5 405 V 350" stroke="#d8e7ef" fill="none"/>
      <text x="66" y="408" fill="#c2d1df" fontSize="11">+X</text>
      <text x="1" y="342" fill="#c2d1df" fontSize="11">+Y</text>
    </svg>
    <div className="panel-footer">
      BLUE ORIGIN · WPILIB NWU
      <span>{
          s ? `X ${s.chassis.x.toFixed(3)} m   Y ${s.chassis.y.toFixed(3)} m   θ ${
                  (s.chassis.yaw * 180 / Math.PI).toFixed(1)}°` :
              'Awaiting telemetry'}</span>
    </div>
  </>;
}

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

function Spatial({s}: {s: Snapshot|null}) {
  const [model, setModel] = useState<THREE.Group|null>(null);
  const [error, setError] = useState('');
  const camera = useRef<THREE.PerspectiveCamera>(null);
  const [helper, setHelper] = useState<THREE.CameraHelper|null>(null);
  useEffect(() => {
    if (camera.current) {
      const h = new THREE.CameraHelper(camera.current);
      setHelper(h);
      return () => h.dispose();
    }
  }, [!!s]);
  useEffect(() => {
    helper?.update();
  }, [s, helper]);
  useEffect(() => () => {
    model?.traverse(object => {
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
  }, [model]);
  return <>
    <div className="spatial">
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
        {s && <>
          <group position={[s.chassis.x, s.chassis.y, s.chassis.z + .2]}
                 rotation={[s.chassis.roll, s.chassis.pitch, s.chassis.yaw,
                            'ZYX']}>
            <mesh>
              <boxGeometry args={[.8, .7, .4]}/>
              <meshStandardMaterial color="#48e1cf"/>
            </mesh>
            <mesh position={[0, 0, .4 + s.elevator_m / 2]}>
              <boxGeometry args={[.13, .3, Math.max(.1, s.elevator_m)]}/>
              <meshStandardMaterial color="#b6c9d2"/>
            </mesh>
            <group position={[0, 0, .4 + s.elevator_m]}
                   rotation={[0, -s.arm_rad, 0]}>
              <mesh position={[.45, 0, 0]}>
                <boxGeometry args={[.9, .12, .12]}/>
                <meshStandardMaterial color="#ffbd69"/>
              </mesh>
            </group>
          </group>
          <perspectiveCamera
              ref={camera} position={[s.camera.x, s.camera.y, s.camera.z]}
              quaternion={new THREE.Quaternion()
                              .setFromEuler(new THREE.Euler(
                                  s.camera.roll, s.camera.pitch, s.camera.yaw,
                                  'ZYX'))
                              .multiply(
                                  new THREE.Quaternion().setFromRotationMatrix(
                                      new THREE.Matrix4().makeBasis(
                                          new THREE.Vector3(0, -1, 0),
                                          new THREE.Vector3(0, 0, 1),
                                          new THREE.Vector3(-1, 0, 0))))}
              fov={65} aspect={1.6} near={.1} far={3}/>
          {helper && <primitive object={helper}/>}
          {s.targets.map(
              t => <mesh key={t.id}
                         position={[t.pose.x, t.pose.y, t.pose.z]}
                         rotation={[t.pose.roll, t.pose.pitch, t.pose.yaw,
                                    'ZYX']}>
                <boxGeometry args={[.03, .3, .3]}/>
                <meshStandardMaterial color="#b59bff"/>
              </mesh>)}
        </>}
      </Canvas>
    </div>
    <div className="panel-footer">
      Z UP · METERS
      <label className="file">
        Load field GLB
        <input
            aria-label="Load self-contained field model in meters, glTF Y-up"
            type="file" accept=".glb" onChange={async (e) => {
              const f = e.target.files?.[0];
              if (!f) return;
              try {
                const gltf =
                    await new GLTFLoader().parseAsync(await f.arrayBuffer(), '');
                gltf.scene.rotation.x = Math.PI / 2;
                setModel(gltf.scene);
                setError('');
              } catch {
                setError('Could not load model');
              }
            }}/>
      </label>
    </div>
    {error && <small>{error}</small>}
  </>;
}

function Plot({view, onSeek}: {view: View; onSeek: (s: string) => void}) {
  const root = useRef<HTMLDivElement>(null), chart = useRef<uPlot|null>(null);
  const latest = useRef({view, onSeek});
  latest.current = {view, onSeek};
  const plotted = useRef<Snapshot[]>([]);
  const [chosen, setChosen] = useState<string[]|null>(null),
        [delta, setDelta] = useState<number|null>(null);
  const anchor = useRef<number|null>(null), origin = useRef<bigint|null>(null);
  const keys = Object.keys(view.snapshot?.channels ?? {}),
        selected = chosen ?? keys.slice(0, 2);
  useEffect(() => {
    if (!root.current) return;
    const container = root.current;
    const plot = new uPlot(
        {
          width: Math.max(200, container.clientWidth - 24),
          height: Math.max(160, container.clientHeight - 12),
          legend: {show: true},
          cursor: {sync: {key: 'telemetry'}},
          scales: {x: {time: false}},
          axes: [
            {stroke: '#8499ad', grid: {stroke: '#253343'}},
            {stroke: '#8499ad', grid: {stroke: '#253343'}}
          ],
          series: [
            {},
            ...selected.map(
                (key, i) => ({label: key, stroke: palette[i % 4], width: 1.5}))
          ],
          hooks: {
            setCursor: [u => {
              if (u.cursor.idx != null && anchor.current != null) {
                const t = u.data[0][u.cursor.idx];
                setDelta(t - anchor.current);
              }
            }]
          }
        },
        [[], ...selected.map(() => [])], container);
    chart.current = plot;
    const click = () => {
      if (plot.cursor.idx != null) {
        const s = plotted.current[plot.cursor.idx];
        if (s) {
          anchor.current = plot.data[0][plot.cursor.idx];
          latest.current.onSeek(s.timestamp_ns);
        }
      }
    };
    container.addEventListener('dblclick', click);
    const pan = (e: WheelEvent) => {
      if (!e.shiftKey) return;
      e.preventDefault();
      const {min, max} = plot.scales.x;
      if (min != null && max != null) {
        const offset = (max - min) * (e.deltaY || e.deltaX) / 1000;
        plot.setScale('x', {min: min + offset, max: max + offset});
      }
    };
    container.addEventListener('wheel', pan, {passive: false});
    const resize = new ResizeObserver(
        () => plot.setSize({
          width: Math.max(200, container.clientWidth - 24),
          height: Math.max(160, container.clientHeight - 12)
        }));
    resize.observe(container);
    return () => {
      resize.disconnect();
      container.removeEventListener('dblclick', click);
      container.removeEventListener('wheel', pan);
      plot.destroy();
      chart.current = null;
    };
  }, [selected.join(',')]);
  useEffect(() => {
    const plot = chart.current;
    if (!plot) return;
    plotted.current = view.history;
    if (!view.history.length) {
      plot.setData([[], ...selected.map(() => [])]);
      origin.current = null;
      anchor.current = null;
      return;
    }
    const first = BigInt(view.history[0].timestamp_ns);
    if (origin.current == null || first < origin.current) origin.current = first;
    const times = view.history.map(
        s => Number(BigInt(s.timestamp_ns) - origin.current!) / 1e9);
    plot.setData(
        [times, ...selected.map(key => view.history.map(s => s.channels[key] ?? null))],
        view.mode === 'LIVE_STREAMING' || plot.data[0].length === 0);
    if (view.snapshot && view.mode !== 'LIVE_STREAMING')
      plot.setCursor({
        left: plot.valToPos(
            Number(BigInt(view.snapshot.timestamp_ns) - origin.current) / 1e9,
            'x'),
        top: 0
      });
  }, [view.history, view.snapshot, view.mode, selected.join(',')]);
  return <>
    <div className="channel-picker">{keys.map(
        key => <label key={key}>
          <input type="checkbox" checked={selected.includes(key)}
                 onChange={() => setChosen(
                     selected.includes(key) ?
                         selected.filter(k => k !== key) :
                         [...selected, key])}/>
          {key}<em>{view.snapshot?.units[key]}</em>
        </label>)}</div>
    <div className="plot" ref={root}/>
    <div className="panel-footer">
      DRAG: ZOOM · SHIFT+WHEEL: PAN · DOUBLE CLICK: SEEK
      <span>Δt {delta == null ? '—' : delta.toFixed(4) + ' s'}</span>
    </div>
  </>;
}

function Tree({value, name = 'state'}: {value: unknown; name?: string}) {
  if (value && typeof value === 'object')
    return <details open={name === 'state' || name === 'chassis'}>
      <summary>{name}<small>{
          Array.isArray(value) ? `[${value.length}]` : '{…}'}</small></summary>
      <div className="tree-children">{
          Object.entries(value).map(
              ([k, v]) => <Tree key={k} name={k} value={v}/>)}</div>
    </details>;
  return <div className="leaf">
    <span>{name}</span>
    <b>{typeof value === 'number' ? value.toFixed(5) : String(value)}</b>
  </div>;
}

// --- shell ------------------------------------------------------------------

function savedTab(): TabId {
  try {
    const value = localStorage.getItem('talos-studio-tab');
    if (isTabId(value)) return value;
  } catch {
    // A browser with site data blocked throws on read; the default tab is a
    // fine answer and not worth failing startup over.
  }
  return 'overview';
}

function App() {
  const [view, setView] = useState<View>(initialView);
  const [system, setSystem] = useState<SystemState>(initialSystem);
  const [tab, setTab] = useState<TabId>(savedTab);
  const [url, setUrl] = useState(
      `${location.protocol === 'https:' ? 'wss' : 'ws'}://${
          location.hostname || 'localhost'}:${
          location.port && location.port !== '5173' ? location.port :
                                                      '5800'}/telemetry`);
  const [exactTimestamp, setExactTimestamp] = useState('');
  const [stepNs, setStepNs] = useState('20000000');
  const [transportError, setTransportError] = useState('');
  const udpActive = useRef(false);

  // One promise per in-flight RPC. The worker owns the timeline and the graph,
  // so the Agent tab has to ask it rather than keeping a second copy.
  const pending = useRef(new Map<number, (value: unknown) => void>());
  const nextRpcId = useRef(1);

  useEffect(() => {
    try {
      localStorage.setItem('talos-studio-tab', tab);
    } catch {
    }
  }, [tab]);

  useEffect(() => {
    worker.onmessage = e => {
      if (e.data.type === 'view') setView(e.data);
      if (e.data.type === 'system')
        setSystem({
          graph: e.data.graph,
          rates: e.data.rates ?? {},
          error: e.data.error ?? '',
          // Null when the bridge was started without --declared, which is the
          // ordinary case and not a fault: the views show nothing rather than
          // an empty panel.
          declared: e.data.declared ?? null,
          declaredError: e.data.declaredError ?? ''
        });
      if (e.data.type === 'rpc') {
        const settle = pending.current.get(e.data.id);
        pending.current.delete(e.data.id);
        settle?.(e.data.response);
      }
      if (e.data.type === 'export') {
        const href = URL.createObjectURL(new Blob([e.data.buffer]));
        const a = document.createElement('a');
        a.href = href;
        a.download = 'telemetry.tlog';
        a.click();
        setTimeout(() => URL.revokeObjectURL(href), 1000);
      }
    };
    return () => {
      worker.onmessage = null;
    };
  }, []);

  const send = (data: object) => worker.postMessage(data);
  const seek = (timestamp_ns: string) => send({type: 'seek', timestamp_ns});

  const call = (method: string, params: unknown) =>
      new Promise<unknown>(resolve => {
        const id = nextRpcId.current++;
        pending.current.set(id, resolve);
        send({
          type: 'rpc',
          id,
          request: {jsonrpc: '2.0', id, method, params}
        });
      });

  const stopDesktop = async () => {
    udpActive.current = false;
    if ('__TAURI_INTERNALS__' in window) {
      try {
        const {invoke} = await import('@tauri-apps/api/core');
        await invoke('stop_udp');
        return true;
      } catch (e) {
        setTransportError(String(e));
        return false;
      }
    }
    return true;
  };

  const start = async () => {
    setTransportError('');
    if ('__TAURI_INTERNALS__' in window) {
      try {
        const {invoke, Channel} = await import('@tauri-apps/api/core');
        const channel = new Channel<ArrayBuffer>();
        channel.onmessage = buffer => {
          try {
            if (udpActive.current)
              worker.postMessage({type: 'packet', buffer}, [buffer]);
          } finally {
            void invoke('ack_udp').catch(e => setTransportError(String(e)));
          }
        };
        udpActive.current = true;
        await invoke('start_udp', {bind: '0.0.0.0:5801', onPacket: channel});
        send({type: 'mode', mode: 'LIVE_STREAMING'});
      } catch (e) {
        setTransportError(String(e));
      }
    } else
      send({type: 'connect', url});
  };

  const startNs = BigInt(view.bounds?.start ?? 0),
        endNs = BigInt(view.bounds?.end ?? 0);
  const ratio = endNs > startNs ?
      Number(BigInt(view.snapshot?.timestamp_ns ?? startNs) - startNs) /
          Number(endNs - startNs) :
      0;

  const liveNodes = system.graph?.nodes.filter(n => n.alive).length ?? 0;
  const totalNodes = system.graph?.nodes.length ?? 0;

  return <div className="app">
    <header>
      <div className="brand">
        <div className="mark">t</div>
        <div>
          talOS <strong>STUDIO</strong>
          <small>DETERMINISTIC TELEMETRY</small>
        </div>
      </div>
      <div className="connection">
        <span className={'dot ' + (view.mode === 'LIVE_STREAMING' ? 'active' : '')}/>
        <span>{view.status}</span>
        <input aria-label="WebSocket URL" value={url}
               onChange={e => setUrl(e.target.value)}/>
        <button className="primary" onClick={start}>Connect</button>
      </div>
      <button onClick={async () => {
        if (await stopDesktop()) send({type: 'demo'});
      }}>Demo data</button>
    </header>

    {transportError && <div className="error">{transportError}</div>}

    <nav className="tabs" role="tablist" aria-label="Studio views">
      {TABS.map(entry => <button key={entry.id} role="tab"
                                 aria-selected={tab === entry.id}
                                 className={tab === entry.id ? 'selected' : ''}
                                 onClick={() => setTab(entry.id)}>
         {entry.label}
         <em>{entry.id === 'system' && totalNodes ?
                  `${liveNodes}/${totalNodes}` :
                  entry.hint}</em>
       </button>)}
      <div className="tab-spacer"/>
      <div className="tab-metrics">
        <div><small>SEQUENCE</small><b>{view.snapshot?.sequence_id ?? '—'}</b></div>
        <div><small>DROPPED / LATE</small>
          <b>{view.stats.dropped} <span>/ {view.stats.late}</span></b></div>
        <div><small>INVALID</small><b>{view.stats.invalid}</b></div>
      </div>
    </nav>

    <main className={'tab-' + tab} role="tabpanel">
      {tab === 'overview' && <OverviewTab view={view} system={system} onOpen={setTab}/>}
      {tab === 'system' && <SystemTab system={system}/>}
      {tab === 'timing' && <TimingTab system={system}/>}
      {tab === 'agent' && <AgentTab call={call}/>}
      {tab === 'field' && <section className="panel">
        <div className="panel-heading"><h2>Field odometry</h2><span>2D</span></div>
        <Field view={view}/>
      </section>}
      {tab === 'pose' && <section className="panel">
        <div className="panel-heading"><h2>Spatial viewport</h2><span>3D</span></div>
        <Spatial s={view.snapshot}/>
      </section>}
      {tab === 'signals' && <section className="panel">
        <div className="panel-heading"><h2>Signal analysis</h2><span>CHANNELS</span></div>
        <Plot view={view} onSeek={seek}/>
      </section>}
      {tab === 'state' && <section className="panel">
        <div className="panel-heading">
          <h2>State inspector</h2><span>STRUCTURED</span>
        </div>
        <div className="tree">{
            view.snapshot ?
                <Tree value={view.snapshot}/> :
                <Empty>
                  Connect to a robot, import a recording, or start demo data.
                </Empty>}</div>
      </section>}
    </main>

    <footer className="timeline">
      <div className="playback">
        <button aria-label="Step backward"
                disabled={!/^[1-9]\d{0,17}$/.test(stepNs)}
                onClick={() => send({type: 'step', dt_ns: '-' + stepNs})}>
          ▏◀
        </button>
        <button className="play" aria-label="Play or pause"
                onClick={() => send({
                  type: 'mode',
                  mode: view.mode === 'REPLAY_PLAYING' ||
                          view.mode === 'LIVE_STREAMING' ?
                      'PAUSED' :
                      'REPLAY_PLAYING'
                })}>
          {view.mode === 'PAUSED' ? '▶' : 'Ⅱ'}
        </button>
        <button aria-label="Step forward"
                disabled={!/^[1-9]\d{0,17}$/.test(stepNs)}
                onClick={() => send({type: 'step', dt_ns: stepNs})}>
          ▶▕
        </button>
        <button className={view.mode === 'LIVE_STREAMING' ? 'live selected' : 'live'}
                onClick={() => send({type: 'mode', mode: 'LIVE_STREAMING'})}>
          ● LIVE
        </button>
      </div>
      <div className="scrub">
        <div>
          <span>{view.mode.replaceAll('_', ' ')}</span>
          <code>{view.snapshot?.timestamp_ns ?? '0'} ns</code>
        </div>
        <input aria-label="Replay timestamp" type="range" min="0" max="1000000"
               value={Math.max(0, Math.min(1000000, Math.round(ratio * 1000000)))}
               disabled={!view.bounds}
               onChange={e => seek(
                   (startNs + (endNs - startNs) * BigInt(e.target.value) / 1000000n)
                       .toString())}/>
        <div>
          <span>{view.bounds?.start ?? '—'}</span>
          <span>{view.bounds?.end ?? '—'}</span>
        </div>
      </div>
      <div className="precision">
        <form onSubmit={e => {
          e.preventDefault();
          if (/^\d{1,20}$/.test(exactTimestamp))
            seek(exactTimestamp);
          else
            setTransportError('Enter a timestamp as integer nanoseconds');
        }}>
          <input aria-label="Exact replay timestamp in nanoseconds"
                 placeholder="Seek timestamp (ns)" value={exactTimestamp}
                 onChange={e => setExactTimestamp(e.target.value)}
                 inputMode="numeric"/>
          <button disabled={!view.bounds}>Seek</button>
        </form>
        <label>
          Step (ns)
          <input aria-label="Frame step duration in nanoseconds"
                 inputMode="numeric" value={stepNs}
                 onChange={e => setStepNs(e.target.value)}/>
        </label>
      </div>
      <label className="file button">
        Import log
        <input aria-label="Import telemetry log" type="file" accept=".tlog"
               onChange={async (e) => {
                 const f = e.target.files?.[0];
                 if (f && await stopDesktop()) {
                   const buffer = await f.arrayBuffer();
                   worker.postMessage({type: 'import', buffer}, [buffer]);
                 }
               }}/>
      </label>
      <button onClick={() => send({type: 'export'})}>Export log ↗</button>
    </footer>
  </div>;
}

createRoot(document.getElementById('root')!).render(<App/>);
