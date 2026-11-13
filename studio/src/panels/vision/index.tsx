// The vision panel: one row per camera (health, frame rate, latency, tag
// count, recent observation validity, dropped-frame delta, std devs and
// disagreement with the estimator), and below it an estimator summary with a
// small per-camera x/y observation-error chart against the estimator (or sim
// ground truth when present). Everything is read through the data hooks and
// discovered by shape, so the panel stays robot-agnostic.

import {useMemo, useState} from 'react';
import type {ReactNode} from 'react';

import {useDictionary, useSeries, useSnapshot} from '../../data/hooks';
import {discoverCameras, discoverEstimators} from '../../data/discovery';
import type {Series} from '../../data/types';
import {integer} from '../../ui/formatters';
import {Badge, EmptyState} from '../../ui/primitives';
import {Sparkline} from '../../ui/Sparkline';
import {TimeSeriesChart, type ChartLine} from '../../ui/TimeSeriesChart';
import type {PanelDefinition} from '../types';
import {
  acceptancePct,
  counterDelta,
  fpsFromPeriod,
  latestValidPose,
  observationErrors,
  validObservationRate,
  type Planar
} from './logic';
import './vision.css';

export type VisionConfig = Record<string, never>;

const defaultConfig: VisionConfig = {};
const parseConfig = (_raw: unknown): VisionConfig => ({});

const palette =
    ['#48e1cf', '#ffbd69', '#b59bff', '#ff778b', '#9dc2f5', '#ffd97a'];

// Telemetry flattens a topic to `<owner>/<role>/<leaf...>`.
const channel = (owner: string, suffix: string) => `${owner}/${suffix}`;

// Map one series column through a transform, preserving NaN (no reading).
function mapSeries(values: ArrayLike<number>, fn: (v: number) => number):
    Float64Array {
  const out = new Float64Array(values.length);
  for (let i = 0; i < values.length; i++) {
    const v = values[i];
    out[i] = Number.isFinite(v) ? fn(v) : NaN;
  }
  return out;
}

const col = (name: string) => (s: Series): Float64Array =>
    s.values[name] ?? new Float64Array(0);

type CameraRow = {
  name: string;
  healthy: boolean|null;
  fps: number|null;
  latencyMs: number|null;
  visibleTags: number|null;
  validRate: number|null;
  droppedDelta: number|null;
  stdX: number|null;
  stdY: number|null;
  stdYawDeg: number|null;
  disagreement: number|null;
};

// Format a measured value with its unit suffix; null is an explicit "no
// reading" (table cells wrap it in a muted dash via Cell, so a bare '—' here
// only reaches the std-dev and estimator blocks).
function fmt(value: number|null, places: number, suffix = ''): string {
  if (value === null) return '—';
  return `${value.toFixed(places)}${suffix}`;
}

// A value cell: absent readings are an explicit muted dash, never a guess.
function Cell({value, children}: {value: unknown; children: ReactNode}) {
  return value == null ?
      <span className="text-muted">—</span> :
      <>{children}</>;
}

function VisionPanel() {
  const dictionary = useDictionary();
  const snapshot = useSnapshot();
  const cameras = useMemo(() => discoverCameras(dictionary), [dictionary]);
  const estimators = useMemo(() => discoverEstimators(dictionary), [dictionary]);
  const estimator = estimators[0] ?? null;

  // Collect the windowed channels once, then read them out of a few series
  // rather than one hook per camera per window.
  const wants = useMemo(() => {
    const sample: string[] = [], valid: string[] = [], dropped: string[] = [];
    const detail: string[] = [];
    for (const c of cameras) {
      sample.push(channel(c.name, 'state/sample_time_ns'));
      valid.push(channel(c.name, 'state/valid'));
      dropped.push(channel(c.name, 'status/frames_dropped_count'));
      detail.push(
          channel(c.name, 'state/pose/x_m'), channel(c.name, 'state/pose/y_m'),
          channel(c.name, 'state/valid'),
          channel(c.name, 'status/frame_period_us'),
          channel(c.name, 'status/last_latency_us'),
          channel(c.name, 'status/visible_tag_count'));
    }
    return {sample, valid, dropped, detail};
  }, [cameras]);

  const rate5 = useSeries([...wants.sample, ...wants.valid], {seconds: 5});
  const drop10 = useSeries(wants.dropped, {seconds: 10});
  const detail30 = useSeries(wants.detail, {seconds: 30});

  // The chart reference is the estimator pose, or sim ground truth when it is
  // present (the disagreement column always compares against the estimator).
  const simPresent = Number.isFinite(snapshot?.channels['sim/state/pose/x_m']);
  const refOwner = simPresent ? 'sim' : estimator?.name ?? null;
  const refX = refOwner ? channel(refOwner, 'state/pose/x_m') : null;
  const refY = refOwner ? channel(refOwner, 'state/pose/y_m') : null;
  const ref30 = useSeries(refX && refY ? [refX, refY] : [], {seconds: 30});

  const est10 = useSeries(estimator ? [
    channel(estimator.name, 'state/observation_accepted_count'),
    channel(estimator.name, 'state/observation_rejected_count')
  ] : [], {seconds: 10});

  const ch = snapshot?.channels ?? {};

  const [open, setOpen] = useState<string|null>(null);

  const rows: CameraRow[] = useMemo(() => {
    return cameras.map(c => {
      const state = (n: string) => channel(c.name, `state/${n}`);
      const status = (n: string) => channel(c.name, `status/${n}`);

      const read = (name: string): number|null => {
        const v = ch[name];
        return v === undefined || !Number.isFinite(v) ? null : v;
      };

      const periodUs = read(status('frame_period_us'));
      const latencyUs = read(status('last_latency_us'));
      const yawStdRad = read(state('yaw_std_rad'));
      const healthyRaw = ch[status('healthy')];

      const rate = validObservationRate(
          col(state('sample_time_ns'))(rate5), col(state('valid'))(rate5));
      const dropped = counterDelta(col(status('frames_dropped_count'))(drop10));

      // Disagreement: the camera's latest valid pose, if it is still current,
      // vs the estimator pose at the cursor.
      let disagreement: number|null = null;
      const estX = estimator ? read(channel(estimator.name, 'state/pose/x_m')) : null;
      const estY = estimator ? read(channel(estimator.name, 'state/pose/y_m')) : null;
      if (estX !== null && estY !== null) {
        const cam: Planar|null = latestValidPose(
            detail30.times,
            col(state('pose/x_m'))(detail30),
            col(state('pose/y_m'))(detail30),
            col(state('valid'))(detail30));
        if (cam) disagreement = Math.hypot(cam.x - estX, cam.y - estY);
      }

      return {
        name: c.name,
        healthy: healthyRaw === undefined ? null : healthyRaw !== 0,
        fps: fpsFromPeriod(periodUs),
        latencyMs: latencyUs === null ? null : latencyUs / 1000,
        visibleTags: read(status('visible_tag_count')),
        validRate: rate,
        droppedDelta: dropped,
        stdX: read(state('x_std_m')),
        stdY: read(state('y_std_m')),
        stdYawDeg: yawStdRad === null ? null : yawStdRad * 180 / Math.PI,
        disagreement
      };
    });
  }, [cameras, ch, estimator, rate5, drop10, detail30]);

  // Estimator summary: accepted/rejected deltas over the last 10 s.
  const estAccepted = estimator ?
      counterDelta(col(channel(estimator.name, 'state/observation_accepted_count'))(est10)) :
      null;
  const estRejected = estimator ?
      counterDelta(col(channel(estimator.name, 'state/observation_rejected_count'))(est10)) :
      null;
  const estPct = acceptancePct(estAccepted, estRejected);
  const estValidRaw = estimator ? ch[channel(estimator.name, 'state/valid')] : undefined;
  const estValid = estValidRaw === undefined ? null : estValidRaw !== 0;

  // Chart lines: per camera, x/y observation error against the reference
  // (x solid, y dashed), sharing each camera's observation times.
  const chartLines = useMemo(() => {
    if (!refX || !refY) return [];
    const rX = col(refX)(ref30);
    const rY = col(refY)(ref30);
    const out: ChartLine[] = [];
    cameras.forEach((c, i) => {
      const d = detail30;
      const color = palette[i % palette.length];
      const points = observationErrors(
          d.times, col(channel(c.name, 'state/pose/x_m'))(d),
          col(channel(c.name, 'state/pose/y_m'))(d),
          col(channel(c.name, 'state/valid'))(d), rX, rY);
      const times: number[] = [], xs: number[] = [], ys: number[] = [];
      for (const p of points) {
        times.push(p.t);
        xs.push(p.x);
        ys.push(p.y);
      }
      out.push({name: c.name, color, dash: false, times, values: xs});
      out.push({name: c.name, color, dash: true, times, values: ys});
    });
    return out;
  }, [cameras, refX, refY, ref30, detail30]);

  // Detail sparklines for the open camera (fps, latency, visible tags).
  const openCamera = open ? cameras.find(c => c.name === open) : undefined;
  const openSpark = useMemo(() => {
    if (!openCamera) return null;
    const d = detail30;
    const period = col(channel(openCamera.name, 'status/frame_period_us'))(d);
    const latency = col(channel(openCamera.name, 'status/last_latency_us'))(d);
    const tags = col(channel(openCamera.name, 'status/visible_tag_count'))(d);
    return {
      fps: mapSeries(period, v => 1e6 / v),
      latencyMs: mapSeries(latency, v => v / 1000),
      tags
    };
  }, [openCamera, detail30]);

  const stdOf = (row: CameraRow): ReactNode => {
    if (row.stdX === null && row.stdY === null && row.stdYawDeg === null)
      return <span className="text-muted">—</span>;
    return <>{fmt(row.stdX, 3)} / {fmt(row.stdY, 3)} m · {fmt(row.stdYawDeg, 1, '°')}</>;
  };

  const healthOf = (row: CameraRow): ReactNode => {
    if (row.healthy === null) return <Badge>unknown</Badge>;
    return row.healthy ?
        <Badge tone="healthy">HEALTHY</Badge> :
        <Badge tone="unhealthy">UNHEALTHY</Badge>;
  };

  return <div className="vision">
    <section className="panel">
      <div className="panel-heading">
        <h2>Cameras</h2>
        <span>{cameras.length ? `${cameras.length} CAMERAS` : 'NONE'}</span>
      </div>
      {cameras.length ?
           <div className="table-scroll">
             <table className="grid vision-grid">
               <thead>
                 <tr>
                   <th>Camera</th>
                   <th>Health</th>
                   <th className="right">FPS</th>
                   <th className="right">Latency (ms)</th>
                   <th className="right">Tags</th>
                   <th className="right">Valid (5 s)</th>
                   <th className="right">Dropped (10 s)</th>
                   <th className="right">Std dev</th>
                   <th className="right">Disagreement (m)</th>
                 </tr>
               </thead>
               <tbody>
                 {rows.map(row => <VisionRow key={row.name} row={row}
                                             open={open === row.name}
                                             onToggle={() => setOpen(
                                                 open === row.name ? null :
                                                                    row.name)}
                                             health={healthOf(row)}
                                             std={stdOf(row)}
                                             spark={open === row.name ? openSpark : null}/>)}
               </tbody>
             </table>
           </div> :
           <EmptyState>
             No cameras discovered. Connect to a robot that publishes pose
             observations, or start a session with cameras in its telemetry.
           </EmptyState>}
    </section>

    <section className="panel">
      <div className="panel-heading">
        <h2>Estimator</h2>
        <span>{estimator ? estimator.name.toUpperCase() : 'NONE'}</span>
      </div>
      {estimator ?
           <div className="estimator">
             <dl className="stat-list">
               <div>
                 <dt>State</dt>
                 <dd>{estValid === null ?
                          <Badge>unknown</Badge> :
                          estValid ?
                          <Badge tone="healthy">VALID</Badge> :
                          <Badge tone="unhealthy">INVALID</Badge>}</dd>
               </div>
               <div>
                 <dt>Accepted (10 s)</dt>
                 <dd>{estAccepted === null ? '—' : integer(estAccepted)}</dd>
               </div>
               <div>
                 <dt>Rejected (10 s)</dt>
                 <dd>{estRejected === null ? '—' : integer(estRejected)}</dd>
               </div>
               <div>
                 <dt>Acceptance</dt>
                 <dd className={estPct !== null && estPct < 0.5 ? 'text-warn' : undefined}>
                   {fmt(estPct === null ? null : estPct * 100, 0, '%')}
                 </dd>
               </div>
             </dl>
             <div className="vision-chart">
               <div className="vision-chart-heading">
                 <span>observation x/y error</span>
                 <span className="text-muted">
                   {simPresent ? 'vs sim truth' : 'vs estimator pose'}
                 </span>
               </div>
               <TimeSeriesChart lines={chartLines} width={800} height={120}
                                legendNote="solid = x · dashed = y"
                                empty={<EmptyState>
                                  No accepted observations in the last 30 s.
                                </EmptyState>}/>
             </div>
           </div> :
           <EmptyState>
             No estimator discovered. Connect to a robot that publishes
             pose-estimator state to see observation agreement.
           </EmptyState>}
    </section>
  </div>;
}

// One camera row plus, when open, an inline detail row of sparklines.
function VisionRow(
    {row, open, onToggle, health, std, spark}: {
      row: CameraRow;
      open: boolean;
      onToggle: () => void;
      health: ReactNode;
      std: ReactNode;
      spark: {fps: Float64Array; latencyMs: Float64Array; tags: Float64Array}|null;
    }) {
  return <>
    <tr className={'clickable' + (open ? ' open' : '')} onClick={onToggle}>
      <td>{row.name}</td>
      <td>{health}</td>
      <td className="right"><Cell value={row.fps}>{fmt(row.fps, 1)}</Cell></td>
      <td className="right"><Cell value={row.latencyMs}>{fmt(row.latencyMs, 1)}</Cell></td>
      <td className="right"><Cell value={row.visibleTags}>{row.visibleTags}</Cell></td>
      <td className="right"><Cell value={row.validRate}>{fmt(row.validRate === null ? null : row.validRate * 100, 0, '%')}</Cell></td>
      <td className="right"><Cell value={row.droppedDelta}>{row.droppedDelta === null ? null : integer(row.droppedDelta)}</Cell></td>
      <td className="right">{std}</td>
      <td className="right"><Cell value={row.disagreement}>{fmt(row.disagreement, 3)}</Cell></td>
    </tr>
    {open && spark &&
     <tr className="detail">
       <td colSpan={9}>
         <div className="spark-row">
           <span className="spark">
             <b>fps</b>
             <Sparkline values={spark.fps} stroke="#48e1cf"/>
           </span>
           <span className="spark">
             <b>latency (ms)</b>
             <Sparkline values={spark.latencyMs} stroke="#ffbd69"/>
           </span>
           <span className="spark">
             <b>visible tags</b>
             <Sparkline values={spark.tags} stroke="#b59bff"/>
           </span>
         </div>
       </td>
     </tr>}
  </>;
}

export const visionPanel: PanelDefinition<VisionConfig> = {
  id: 'vision',
  title: 'Vision',
  category: 'Robot',
  description: 'Camera health, frame stats and estimator agreement.',
  defaultConfig,
  parseConfig,
  component: VisionPanel
};
