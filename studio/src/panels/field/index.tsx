// The field panel: an SVG top-down view of the FRC field with the chassis
// (estimate) pose, a trail of recent estimated poses, and -- when sim truth is
// present -- a ghost of the ground-truth pose and the estimate error readout.
// Layered on top: per-camera latest valid observations with their uncertainty
// ellipses, and the estimator's velocity vector. Everything optional is a
// layer with a persisted toggle; the view follows the robot or shows the whole
// field, with wheel/drag pan-zoom and a reset button. A pose whose frame is no
// longer current (telemetry stopped) stays on screen dimmed and marked STALE,
// without its velocity arrow.
//
// Data is read only through the hooks: the estimate pose/velocity come from the
// pose estimator (discovered by shape), the truth from `sim/state/pose/*`, and
// cameras from `discoverCameras`. Nothing here names a 2026 robot.

import {useEffect, useMemo, useRef, useState} from 'react';
import type {ChangeEvent, PointerEvent as RPointerEvent} from 'react';

import {simTruthPose} from '../../core';
import {discoverCameras, discoverEstimators} from '../../data/discovery';
import {chassisReading} from '../../data/freshness';
import {
  useActions,
  useChannelValue,
  useDictionary,
  usePlayback,
  useSeries,
  useSnapshot
} from '../../data/hooks';
import type {ChannelInfo} from '../../data/types';
import {number} from '../../ui/formatters';
import {FreshnessNotice} from '../../ui/FreshnessNotice';
import type {PanelDefinition, PanelProps} from '../types';
import {
  FIELD_HEIGHT_PX,
  FIELD_MARGIN_PX,
  FIELD_WIDTH_PX,
  PX_PER_M,
  clamp,
  followViewport,
  stalenessFade,
  toPx,
  toPy,
  uncertaintyEllipse,
  wholeFieldViewport,
  zoomViewport,
  type PointM,
  type Viewport
} from './geometry';
import './field.css';

export type FieldLayers = {
  trail: boolean;
  truth: boolean;
  cameras: boolean;
  velocity: boolean;
};
export type FieldConfig = {
  layers: FieldLayers;
  trailSeconds: number;
  follow: boolean;
};

const defaultConfig: FieldConfig = {
  layers: {trail: true, truth: true, cameras: true, velocity: true},
  trailSeconds: 5,
  follow: false
};

const asBool = (v: unknown, d: boolean) => (typeof v === 'boolean' ? v : d);
const asNumber = (v: unknown, d: number) =>
    typeof v === 'number' && Number.isFinite(v) ? v : d;

function parseConfig(raw: unknown): FieldConfig {
  const obj = raw && typeof raw === 'object' && !Array.isArray(raw) ?
      raw as Record<string, unknown> :
      {};
  const layersRaw = obj.layers && typeof obj.layers === 'object' &&
          !Array.isArray(obj.layers) ?
      obj.layers as Record<string, unknown> :
      {};
  return {
    layers: {
      trail: asBool(layersRaw.trail, defaultConfig.layers.trail),
      truth: asBool(layersRaw.truth, defaultConfig.layers.truth),
      cameras: asBool(layersRaw.cameras, defaultConfig.layers.cameras),
      velocity: asBool(layersRaw.velocity, defaultConfig.layers.velocity)
    },
    trailSeconds: Math.max(0.5, asNumber(obj.trailSeconds, defaultConfig.trailSeconds)),
    follow: asBool(obj.follow, defaultConfig.follow)
  };
}

const colors = {estimate: '#48e1cf', truth: '#ffbd69', velocity: '#ffbd69'};

// Distinct hues for up to eight cameras, assigned by discovery order (which is
// alphabetical, so the mapping is stable across renders).
const cameraPalette =
    ['#7aa2ff', '#a78bfa', '#4ade80', '#f472b6', '#fbbf24', '#22d3ee', '#fb7185', '#a3e635'];

const MIN_ZOOM = 0.06;  // about 1 m across, the tightest useful zoom
const MAX_ZOOM = 2.5;

type EstimateChannels = {
  x: string; y: string; yaw: string; vx: string|null; vy: string|null;
};

// The estimate pose (and, when present, velocity) channels. The estimator
// publishes the estimated pose; discover it by its accepted-observation
// counter rather than by name. Falls back to the old `/pose/x_m` suffix rule
// for recordings whose estimator predates that shape (the demo).
function discoverEstimate(dictionary: ChannelInfo[]): EstimateChannels|null {
  const names = new Set(dictionary.map(c => c.name));
  for (const est of discoverEstimators(dictionary)) {
    const x = `${est.name}/state/pose/x_m`;
    if (!names.has(x)) continue;
    const vx = `${est.name}/state/velocity/vx_mps`;
    const vy = `${est.name}/state/velocity/vy_mps`;
    return {
      x,
      y: `${est.name}/state/pose/y_m`,
      yaw: `${est.name}/state/pose/yaw_rad`,
      vx: names.has(vx) ? vx : null,
      vy: names.has(vy) ? vy : null
    };
  }
  const x = dictionary.find(c => c.name.endsWith('/pose/x_m'));
  if (!x) return null;
  const prefix = x.name.slice(0, -'pose/x_m'.length);
  return {
    x: x.name, y: prefix + 'pose/y_m', yaw: prefix + 'pose/yaw_rad',
    vx: null, vy: null
  };
}

type TrailPoint = PointM & {tNs: string};
type CameraObs = {
  name: string; color: string; x: number; y: number; yaw: number|null;
  ellipse: {rx: number; ry: number}|null; alpha: number;
};

// The current viewBox in SVG user units. It is `manualView` when the user has
// panned/zoomed, else the follow window (4 m around the robot) or whole field.
function FieldPanel({config, setConfig}: PanelProps<FieldConfig>) {
  const snapshot = useSnapshot();
  const dictionary = useDictionary();
  const playback = usePlayback();
  const actions = useActions();
  const cursorNs = playback.cursor_ns;

  const estimate = useMemo(() => discoverEstimate(dictionary), [dictionary]);
  const cameras = useMemo(() => discoverCameras(dictionary), [dictionary]);

  const trailChannels = useMemo(
      () => estimate ? [estimate.x, estimate.y, estimate.yaw] : [], [estimate]);
  const trailSeries = useSeries(trailChannels, {seconds: config.trailSeconds});

  const truthSeries = useSeries(
      ['sim/state/pose/x_m', 'sim/state/pose/y_m', 'sim/state/pose/yaw_rad'],
      {seconds: config.trailSeconds});

  const cameraChannels = useMemo(() => cameras.flatMap(cam => [
    `${cam.name}/state/pose/x_m`, `${cam.name}/state/pose/y_m`,
    `${cam.name}/state/pose/yaw_rad`, `${cam.name}/state/x_std_m`,
    `${cam.name}/state/y_std_m`, `${cam.name}/state/valid`,
    `${cam.name}/state/sample_time_ns`
  ]), [cameras]);
  const cameraWindow = Math.max(2, config.trailSeconds);
  const cameraSeries = useSeries(cameraChannels, {seconds: cameraWindow});

  const velocityVx = useChannelValue(estimate?.vx ?? '');
  const velocityVy = useChannelValue(estimate?.vy ?? '');

  // The estimate pose, and whether its frame is current by the reading clock.
  const reading = chassisReading(snapshot, playback);
  const truth = simTruthPose(snapshot);
  const chassis = reading?.pose ?? null;
  const stale = reading?.stale ?? false;
  const robotM = chassis ? {x: chassis.x, y: chassis.y} : null;
  const estimateOwner = estimate ? estimate.x.slice(0, estimate.x.indexOf('/') + 1) : '';

  const trailPoints = useMemo<TrailPoint[]>(() => {
    if (!estimate) return [];
    const xs = trailSeries.values[estimate.x];
    const ys = trailSeries.values[estimate.y];
    if (!xs || !ys) return [];
    const t0 = BigInt(trailSeries.t0_ns);
    const out: TrailPoint[] = [];
    for (let i = 0; i < xs.length; i++) {
      if (Number.isFinite(xs[i]) && Number.isFinite(ys[i])) {
        out.push({
          x: xs[i], y: ys[i],
          tNs: (t0 + BigInt(Math.round(trailSeries.times[i] * 1e9))).toString()
        });
      }
    }
    return out;
  }, [trailSeries, estimate]);

  const truthTrail = useMemo<PointM[]>(() => {
    const xs = truthSeries.values['sim/state/pose/x_m'];
    const ys = truthSeries.values['sim/state/pose/y_m'];
    if (!xs || !ys) return [];
    const out: PointM[] = [];
    for (let i = 0; i < xs.length; i++)
      if (Number.isFinite(xs[i]) && Number.isFinite(ys[i]))
        out.push({x: xs[i], y: ys[i]});
    return out;
  }, [truthSeries]);

  const cameraObs = useMemo<CameraObs[]>(() => {
    if (!cameras.length || !cursorNs) return [];
    const out: CameraObs[] = [];
    cameras.forEach((cam, i) => {
      // A per-camera series-column lookup, not a name formatter: it returns
      // the column for a camera's state leaf (kept distinct from channelLeaf).
      const column = (name: string) => cameraSeries.values[`${cam.name}/state/${name}`];
      const px = column('pose/x_m'), py = column('pose/y_m'), yaw = column('pose/yaw_rad');
      const xs = column('x_std_m'), ys = column('y_std_m');
      const valid = column('valid'), sampleNs = column('sample_time_ns');
      if (!px) return;
      // Latest valid observation; older ones are at least as stale.
      for (let j = px.length - 1; j >= 0; j--) {
        if (valid[j] !== 1) continue;
        if (!Number.isFinite(px[j]) || !Number.isFinite(py[j])) continue;
        const nowNs = Number(cursorNs);
        const t = sampleNs && Number.isFinite(sampleNs[j]) ? sampleNs[j] : nowNs;
        const alpha = stalenessFade(t, cursorNs);
        if (alpha <= 0) break;
        const ellipse = Number.isFinite(xs?.[j]) && Number.isFinite(ys?.[j]) ?
            uncertaintyEllipse(xs[j], ys[j]) : null;
        out.push({
          name: cam.name, color: cameraPalette[i % cameraPalette.length],
          x: px[j], y: py[j], yaw: Number.isFinite(yaw?.[j]) ? yaw[j] : null,
          ellipse, alpha
        });
        break;
      }
    });
    return out;
  }, [cameras, cameraSeries, cursorNs]);

  const velocity =
      estimate?.vx && estimate.vy && Number.isFinite(velocityVx) &&
              Number.isFinite(velocityVy) ?
          {x: velocityVx as number, y: velocityVy as number} :
          null;

  const positionError = useChannelValue('derived/pose_error_m');
  const headingError = useChannelValue('derived/heading_error_deg');

  // --- view transform (pan/zoom) -------------------------------------------

  const [manualView, setManualView] = useState<Viewport|null>(null);
  const svgRef = useRef<SVGSVGElement>(null);
  const dragRef = useRef<{px: number; py: number}|null>(null);
  const followRef = useRef({follow: config.follow, robot: robotM});
  followRef.current = {follow: config.follow, robot: robotM};

  const defaultView = () =>
      config.follow && robotM ? followViewport(robotM) : wholeFieldViewport();
  const viewBox = manualView ?? defaultView();

  // The wheel handler must call preventDefault to stop the page from scrolling,
  // so it attaches natively (React registers wheel as passive). It reads the
  // current follow/robot from a ref so the once-attached listener never closes
  // over a stale render.
  useEffect(() => {
    const el = svgRef.current;
    if (!el) return;
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const pt = clientToUser(el, e.clientX, e.clientY);
      const factor = Math.exp(-e.deltaY * 0.0015);
      setManualView(prev => {
        const {follow, robot} = followRef.current;
        const base = prev ??
            (follow && robot ? followViewport(robot) : wholeFieldViewport());
        const scale = base.width / wholeFieldViewport().width;
        const next = clamp(scale * factor, MIN_ZOOM, MAX_ZOOM);
        return zoomViewport(base, next / scale, pt.x, pt.y);
      });
    };
    el.addEventListener('wheel', onWheel, {passive: false});
    return () => el.removeEventListener('wheel', onWheel);
  }, []);

  const onPointerDown = (e: RPointerEvent<SVGSVGElement>) => {
    if (e.button !== 0) return;
    e.currentTarget.setPointerCapture(e.pointerId);
    const pt = clientToUser(e.currentTarget, e.clientX, e.clientY);
    dragRef.current = {px: pt.x, py: pt.y};
  };
  const onPointerMove = (e: RPointerEvent<SVGSVGElement>) => {
    const d = dragRef.current;
    if (!d) return;
    const pt = clientToUser(e.currentTarget, e.clientX, e.clientY);
    setManualView(v => {
      const base = v ?? defaultView();
      return {
        x: base.x + (d.px - pt.x), y: base.y + (d.py - pt.y),
        width: base.width, height: base.height
      };
    });
  };
  const endDrag = () => {
    dragRef.current = null;
  };

  const seek = (tNs: string) => actions.seek(tNs);
  const setLayer = (key: keyof FieldLayers) =>
      (e: ChangeEvent<HTMLInputElement>) =>
          setConfig({...config, layers: {...config.layers, [key]: e.target.checked}});
  const setFollow = (e: ChangeEvent<HTMLInputElement>) => {
    if (e.target.checked) setManualView(null);
    setConfig({...config, follow: e.target.checked});
  };
  const setTrailSeconds = (e: ChangeEvent<HTMLInputElement>) => {
    const v = Number(e.target.value);
    if (Number.isFinite(v)) setConfig({...config, trailSeconds: Math.max(0.5, v)});
  };

  const point = (p: PointM) => `${toPx(p.x)},${toPy(p.y)}`;

  const robot = (p: PointM, color: string, name: string, yaw = 0,
                 ghost = false, dim = false) => (
      <g key={name} opacity={ghost ? .55 : dim ? .35 : 1}
         strokeDasharray={ghost ? '5 4' : undefined}
         transform={`translate(${toPx(p.x)} ${toPy(p.y)}) rotate(${
             -yaw * 180 / Math.PI})`}>
        <rect x="-20" y="-17" width="40" height="34" rx="3"
              fill={ghost ? 'none' : color + '22'} stroke={color} strokeWidth="2"/>
        <path d="M 0 0 H 30 M 22 -5 L 30 0 L 22 5" fill="none" stroke={color}
              strokeWidth="2"/>
      </g>);

  const viewBoxStr = `${viewBox.x} ${viewBox.y} ${viewBox.width} ${viewBox.height}`;

  return <>
    {estimateOwner && <FreshnessNotice prefix={estimateOwner}/>}
    <div className="field-controls">
      {(['trail', 'truth', 'cameras', 'velocity'] as const).map(key => (
        <label key={key}>
          <input type="checkbox" checked={config.layers[key]}
                 onChange={setLayer(key)}/>
          {key[0].toUpperCase() + key.slice(1)}
        </label>
      ))}
      <label>
        <input type="checkbox" checked={config.follow} onChange={setFollow}/>
        Follow
      </label>
      <label className="field-trail-seconds">
        Trail
        <input type="number" min="0.5" step="0.5" value={config.trailSeconds}
               onChange={setTrailSeconds}/>
        s
      </label>
      <button type="button" onClick={() => setManualView(null)}>Reset view</button>
    </div>
    <div className="legend">
      {chassis && <span><i style={{background: colors.estimate}}/>Estimate</span>}
      {config.layers.trail && trailPoints.length > 1 && <span><i style={{
        width: 10, height: 2, borderRadius: 0, background: colors.estimate,
        opacity: .6
      }}/>Trail</span>}
      {config.layers.truth && truth && <span><i style={{
        width: 12, height: 8, borderRadius: 0,
        border: `1px dashed ${colors.truth}`, opacity: .6
      }}/>Sim truth</span>}
      {config.layers.velocity && velocity && <span><i style={{
        width: 10, height: 2, borderRadius: 0, background: colors.velocity
      }}/>Velocity</span>}
      {config.layers.cameras && cameraObs.map(obs => (
        <span key={obs.name}><i style={{background: obs.color}}/>{obs.name}</span>
      ))}
    </div>
    {snapshot && !reading &&
     <div className="empty-state" role="status">
       Awaiting pose — mechanism data only. The field view waits for a valid pose.
     </div>}
    <svg ref={svgRef} className="field" viewBox={viewBoxStr}
         onPointerDown={onPointerDown} onPointerMove={onPointerMove}
         onPointerUp={endDrag} onPointerLeave={endDrag}
         aria-label="FRC field, blue alliance origin, X away from blue wall and Y left">
      <defs>
        <pattern id="grid" width="50" height="50" patternUnits="userSpaceOnUse">
          <path d="M 50 0 H 0 V 50" fill="none" stroke="#273444"
                strokeWidth=".6"/>
        </pattern>
        <marker id="velocity-arrow" viewBox="0 0 10 10" refX="9" refY="5"
                markerWidth="6" markerHeight="6" orient="auto-start-reverse">
          <path d="M 0 0 L 10 5 L 0 10 z" fill={colors.velocity}/>
        </marker>
      </defs>
      <rect width={FIELD_WIDTH_PX} height={FIELD_HEIGHT_PX} rx="4"
            fill="#101c29" stroke="#637386"/>
      <rect width={FIELD_WIDTH_PX} height={FIELD_HEIGHT_PX} fill="url(#grid)"/>
      <path d={`M 0 0 H 85 V ${FIELD_HEIGHT_PX} H 0`} fill="#387bd31a"
            stroke="#387bd3"/>
      <path d={`M ${FIELD_WIDTH_PX} 0 H ${FIELD_WIDTH_PX - 85} V ${
          FIELD_HEIGHT_PX} H ${FIELD_WIDTH_PX}`} fill="#f66a6618"
            stroke="#f66a66"/>
      <path d={`M ${FIELD_WIDTH_PX / 2} 0 V ${FIELD_HEIGHT_PX}`}
            stroke="#7c8998" strokeDasharray="5 9"/>
      {config.layers.truth && truth && <>
        <polyline points={truthTrail.map(point).join(' ')} fill="none"
                  stroke={colors.truth} strokeOpacity=".35" strokeWidth="2"
                  strokeDasharray="5 4"/>
        {robot(truth, colors.truth, 'sim-truth', truth.yaw, true)}
      </>}
      {config.layers.trail && trailPoints.length > 1 && <>
        <polyline points={trailPoints.map(point).join(' ')} fill="none"
                  stroke={colors.estimate} strokeOpacity=".55" strokeWidth="2"/>
        {trailPoints.map(p => (
          <circle key={p.tNs} cx={toPx(p.x)} cy={toPy(p.y)} r="6" fill="transparent"
                  className="field-seek" onPointerDown={e => e.stopPropagation()}
                  onClick={() => seek(p.tNs)} role="button" tabIndex={-1}
                  aria-label={`Seek to ${p.tNs}`}/>
        ))}
      </>}
      {config.layers.cameras && cameraObs.map(obs => {
        const cx = toPx(obs.x), cy = toPy(obs.y);
        return (
          <g key={obs.name} opacity={obs.alpha}>
            {obs.ellipse && <ellipse cx={cx} cy={cy}
                     rx={obs.ellipse.rx * PX_PER_M} ry={obs.ellipse.ry * PX_PER_M}
                     fill={obs.color + '1c'} stroke={obs.color} strokeWidth="1"
                     strokeOpacity=".55"/>}
            {obs.yaw !== null && <path
                  d={`M ${cx} ${cy} L ${cx + Math.cos(obs.yaw) * 11} ${cy - Math.sin(obs.yaw) * 11}`}
                  stroke={obs.color} strokeWidth="1.5"/>}
            <circle cx={cx} cy={cy} r="4" fill={obs.color} stroke="#101c29"
                    strokeWidth="1"/>
          </g>
        );
      })}
      {chassis && robot(chassis, colors.estimate, 'chassis', chassis.yaw, false, stale)}
      {config.layers.velocity && chassis && !stale && velocity &&
       <line x1={toPx(chassis.x)} y1={toPy(chassis.y)}
             x2={toPx(chassis.x + velocity.x)}
             y2={toPy(chassis.y + velocity.y)} stroke={colors.velocity}
             strokeWidth="2" markerEnd="url(#velocity-arrow)"/>}
      <path d={`M 5 ${FIELD_HEIGHT_PX - 5} H 60 M 5 ${FIELD_HEIGHT_PX - 5} V ${
          FIELD_HEIGHT_PX - 60}`} stroke="#d8e7ef" fill="none"/>
      <text x="66" y={FIELD_HEIGHT_PX - 2} fill="#c2d1df" fontSize="11">+X</text>
      <text x="1" y={FIELD_HEIGHT_PX - 68} fill="#c2d1df" fontSize="11">+Y</text>
    </svg>
    <div className="panel-footer">
      BLUE ORIGIN · WPILIB NWU
      {positionError !== undefined && headingError !== undefined && <span>
        Position error {number(positionError)} m · Heading error {
            number(headingError, 1)}°
      </span>}
      <span className={stale ? 'text-warn' : undefined}>{
          chassis ? `X ${number(chassis.x, 3)} m   Y ${
                  number(chassis.y, 3)} m   θ ${
                  number(chassis.yaw * 180 / Math.PI, 1)}°` :
              snapshot ? 'Awaiting pose' :
                         'Awaiting telemetry'}{stale && <em> · STALE</em>}</span>
    </div>
  </>;
}

// Convert a client (screen) point into SVG user coordinates. Browser-only: it
// is only reached from wheel/pointer handlers, never during server render.
function clientToUser(svg: SVGSVGElement, clientX: number, clientY: number):
    {x: number; y: number} {
  const pt = svg.createSVGPoint();
  pt.x = clientX;
  pt.y = clientY;
  const ctm = svg.getScreenCTM();
  if (!ctm) return {x: clientX, y: clientY};
  const inverse = ctm.inverse();
  const out = pt.matrixTransform(inverse);
  return {x: out.x, y: out.y};
}

export const fieldPanel: PanelDefinition<FieldConfig> = {
  id: 'field',
  title: 'Field',
  category: 'Spatial',
  description: 'Top-down field view: estimate, truth, cameras and velocity.',
  defaultConfig,
  parseConfig,
  component: FieldPanel
};
