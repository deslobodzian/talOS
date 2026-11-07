// The frames panel: an offline viewer for recorded camera frames.
//
// Nothing here touches the telemetry worker, the bridge or the network. The
// user picks a recorded frames directory on their own disk, Studio pairs each
// <stem>.png with its <stem>.json, and everything after that is local state.
// The parsing and pairing live in frames.ts (pure, tested); this file holds
// only the parts that need a real browser: the directory pickers, an object
// URL for the selected image, and a canvas-free SVG overlay for detections.
//
// All browser access happens in effects and feature checks, so react-dom/server
// renders the empty state in tests without touching a DOM API.

import {useEffect, useMemo, useState, type KeyboardEvent, type ReactNode} from 'react';

import {
  filterFrames,
  framePoseError,
  loadFrameFiles,
  type FrameArchive,
  type FrameDetection,
  type FrameMetadata,
  type FrameReason,
  type LocalFrameFile,
} from '../../frames';
import type {PanelDefinition} from '../types';
import './frames.css';

// The File System Access directory picker is not in TypeScript's DOM lib yet,
// and the directory handle's async iteration methods are missing from the
// typings too. Both are feature-detected and cast narrowly, never assumed.
type PickerWindow = Window & {
  showDirectoryPicker?: (options?: {mode?: 'read'|'readwrite'}) =>
      Promise<FileSystemDirectoryHandle>;
};

type HandleValues = FileSystemDirectoryHandle & {
  values(): AsyncIterableIterator<FileSystemHandle>;
};

// A file plus the blob the image overlay needs. `LocalFrameFile` carries only
// what frames.ts consumes (name, path, size, text); the blob is this view's
// addition so the selected PNG can become an object URL.
type PickedFile = LocalFrameFile & {blob: Blob};

async function collectDirectory(
    handle: FileSystemDirectoryHandle, prefix = ''): Promise<PickedFile[]> {
  const out: PickedFile[] = [];
  const values = (handle as HandleValues).values();
  for await (const entry of values) {
    const path = prefix ? `${prefix}/${entry.name}` : entry.name;
    if (entry.kind === 'file') {
      const file = await (entry as FileSystemFileHandle).getFile();
      out.push({
        name: file.name,
        webkitRelativePath: path,
        size: file.size,
        text: () => file.text(),
        blob: file
      });
    } else {
      out.push(...await collectDirectory(
          entry as FileSystemDirectoryHandle, path));
    }
  }
  return out;
}

const formatNs = (ns: string) => BigInt(ns).toLocaleString('en-US');
const digits = (value: number, places = 4) => value.toFixed(places);

function Detection({detection, fontSize}: {
  detection: FrameDetection;
  fontSize: number;
}) {
  const corners = detection.corners_px;
  const points = [0, 1, 2, 3]
                     .map(k => `${corners[k * 2]},${corners[k * 2 + 1]}`)
                     .join(' ');
  const cx = (corners[0] + corners[2] + corners[4] + corners[6]) / 4;
  const cy = (corners[1] + corners[3] + corners[5] + corners[7]) / 4;
  const color = detection.valid ? '#48e1cf' : '#ff778b';
  const title = `tag ${detection.tag_id} (${
      detection.valid ? 'valid' : 'invalid'})${
      detection.reject_reason ? ' — ' + detection.reject_reason : ''}`;
  return <g>
    <polygon points={points} fill={color + '22'} stroke={color}
             strokeWidth={Math.max(1.5, fontSize / 9)}>
      <title>{title}</title>
    </polygon>
    <text x={cx} y={cy} fill={color} fontSize={fontSize} textAnchor="middle"
          dominantBaseline="middle" paintOrder="stroke" stroke="#0b121c"
          strokeWidth={fontSize / 5}>
      {detection.tag_id}
    </text>
  </g>;
}

function Observation({metadata}: {metadata: FrameMetadata}) {
  const o = metadata.observation;
  return <dl className="stat-list">
    <div><dt>Valid</dt>
      <dd className={o.valid ? '' : 'text-warn'}>{o.valid ? 'yes' : 'no'}</dd></div>
    <div><dt>X</dt><dd>{digits(o.x_m)} m</dd></div>
    <div><dt>Y</dt><dd>{digits(o.y_m)} m</dd></div>
    <div><dt>Yaw</dt>
      <dd>{digits(o.yaw_rad)} rad ({digits(o.yaw_rad * 180 / Math.PI, 1)}°)</dd></div>
    <div><dt>σx / σy / σyaw</dt>
      <dd>{digits(o.x_std_m)} / {digits(o.y_std_m)} / {digits(o.yaw_std_rad)}</dd></div>
    <div><dt>Tags used</dt><dd>{o.tag_count}</dd></div>
  </dl>;
}

function Truth({metadata}: {metadata: FrameMetadata}) {
  const truth = metadata.truth;
  const error = framePoseError(metadata);
  if (!truth)
    return <p className="frames-note">No sim truth recorded for this frame.</p>;
  return <>
    <dl className="stat-list">
      <div><dt>X</dt><dd>{digits(truth.x_m)} m</dd></div>
      <div><dt>Y</dt><dd>{digits(truth.y_m)} m</dd></div>
      <div><dt>Yaw</dt>
        <dd>{digits(truth.yaw_rad)} rad ({digits(truth.yaw_rad * 180 / Math.PI, 1)}°)</dd></div>
    </dl>
    {error ?
         <dl className="stat-list">
           <div><dt>Position error</dt>
             <dd className={error.position_m > 0.5 ? 'text-warn' : ''}>
               {digits(error.position_m)} m</dd></div>
           <div><dt>Heading error</dt>
             <dd className={Math.abs(error.heading_deg) > 5 ? 'text-warn' : ''}>
               {digits(error.heading_deg, 2)}°</dd></div>
         </dl> :
         <p className="frames-note text-warn">Observation invalid — no error computed.</p>}
  </>;
}

function Details({metadata, position, total}: {
  metadata: FrameMetadata;
  position: number;
  total: number;
}) {
  const d = metadata.detector;
  const intrinsics = metadata.intrinsics;
  const mounting = metadata.mounting;
  return <div className="frames-details-body">
    <dl className="stat-list">
      <div><dt>Frame</dt><dd>{position} of {total}</dd></div>
      <div><dt>Timestamp</dt><dd>{formatNs(metadata.capture_ns)} ns</dd></div>
      <div><dt>Reason</dt><dd>{metadata.reason}</dd></div>
      <div><dt>Camera</dt><dd>{metadata.camera}</dd></div>
      <div><dt>Resolution</dt><dd>{metadata.width} × {metadata.height}</dd></div>
      <div><dt>Detections</dt><dd>{metadata.detections.length}</dd></div>
    </dl>

    <h4>Observation</h4>
    <Observation metadata={metadata}/>

    <h4>Truth</h4>
    <Truth metadata={metadata}/>

    <h4>Detector</h4>
    <dl className="stat-list">
      <div><dt>Backend</dt><dd>{d.backend}</dd></div>
      <div><dt>Decimate</dt><dd>{digits(d.decimate)}</dd></div>
      <div><dt>Threads</dt><dd>{d.threads}</dd></div>
      <div><dt>Sigma</dt><dd>{digits(d.sigma)}</dd></div>
      <div><dt>Refine edges</dt><dd>{d.refine_edges ? 'yes' : 'no'}</dd></div>
      <div><dt>Hamming threshold</dt><dd>{d.hamming_threshold}</dd></div>
      <div><dt>Min decision margin</dt><dd>{digits(d.min_decision_margin)}</dd></div>
    </dl>

    <h4>Intrinsics</h4>
    <dl className="stat-list">
      <div><dt>fx / fy</dt>
        <dd>{digits(intrinsics.fx)} / {digits(intrinsics.fy)}</dd></div>
      <div><dt>cx / cy</dt>
        <dd>{digits(intrinsics.cx)} / {digits(intrinsics.cy)}</dd></div>
      <div><dt>k1 / k2 / k3</dt>
        <dd>{digits(intrinsics.k1)} / {digits(intrinsics.k2)} / {digits(intrinsics.k3)}</dd></div>
      <div><dt>p1 / p2</dt>
        <dd>{digits(intrinsics.p1)} / {digits(intrinsics.p2)}</dd></div>
    </dl>

    <h4>Mounting</h4>
    <dl className="stat-list">
      <div><dt>X / Y / Z</dt>
        <dd>{digits(mounting.x_m)} / {digits(mounting.y_m)} / {digits(mounting.z_m)} m</dd></div>
      <div><dt>Roll / Pitch / Yaw</dt>
        <dd>{digits(mounting.roll_rad)} / {digits(mounting.pitch_rad)} / {digits(mounting.yaw_rad)} rad</dd></div>
    </dl>
  </div>;
}

export function FramesKeyScope({onStep, children}: {
  onStep: (delta: number) => void; children?: ReactNode;
}) {
  const onKeyDown = (e: KeyboardEvent<HTMLDivElement>) => {
    const target = e.target as HTMLElement|null;
    if (target && (target.tagName === 'INPUT' || target.tagName === 'SELECT' ||
                   target.tagName === 'TEXTAREA' || target.isContentEditable))
      return;
    if (e.key !== 'ArrowLeft' && e.key !== 'ArrowRight') return;
    e.preventDefault();
    e.stopPropagation();
    onStep(e.key === 'ArrowRight' ? 1 : -1);
  };
  return <div className="frames" tabIndex={0} onKeyDown={onKeyDown}>{children}</div>;
}

function FramesPanel() {
  const [archive, setArchive] = useState<FrameArchive<PickedFile>|null>(null);
  const [camera, setCamera] = useState('');
  const [reason, setReason] = useState<FrameReason|''>('');
  const [index, setIndex] = useState(0);
  const [error, setError] = useState('');
  const [busy, setBusy] = useState(false);
  const [url, setUrl] = useState<string|null>(null);

  const hasDirectoryPicker = typeof window !== 'undefined' &&
      typeof (window as PickerWindow).showDirectoryPicker === 'function';

  const frames = archive?.frames ?? [];
  const filtered = useMemo(
      () => filterFrames(frames, camera, reason), [frames, camera, reason]);
  const cameras = useMemo(
      () => [...new Set(frames.map(f => f.metadata.camera))].sort(),
      [frames]);
  const selected = filtered[Math.min(index, filtered.length - 1)] ?? null;

  useEffect(() => {
    if (!selected) {
      setUrl(null);
      return;
    }
    const next = URL.createObjectURL(selected.image.blob);
    setUrl(next);
    return () => URL.revokeObjectURL(next);
  }, [selected]);


  const load = async (files: readonly PickedFile[]) => {
    setBusy(true);
    setError('');
    try {
      const next = await loadFrameFiles(files);
      setArchive(next);
      setCamera('');
      setReason('');
      setIndex(0);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setBusy(false);
    }
  };

  const pickDirectory = async () => {
    const picker = (window as PickerWindow).showDirectoryPicker;
    if (typeof picker !== 'function') return;
    try {
      const handle = await picker({mode: 'read'});
      await load(await collectDirectory(handle));
    } catch (err) {
      if ((err as DOMException)?.name === 'AbortError') return;
      setError(err instanceof Error ? err.message : String(err));
    }
  };

  const fromInput = (list: FileList|null) => {
    if (!list) return;
    load([...list].map(file => ({
      name: file.name,
      webkitRelativePath: file.webkitRelativePath || undefined,
      size: file.size,
      text: () => file.text(),
      blob: file
    })));
  };

  const fontSize = selected ?
      Math.max(14, Math.round(Math.min(selected.metadata.width,
                                       selected.metadata.height) /
                             40)) :
      16;

  return <FramesKeyScope onStep={delta => setIndex(i =>
      filtered.length ? Math.max(0, Math.min(filtered.length - 1, i + delta)) : i)}>
    <div className="frames-toolbar">
      {hasDirectoryPicker &&
       <button className="primary" onClick={pickDirectory} disabled={busy}>
         Choose directory…
       </button>}
      <label className="file">
        Choose directory…
        <input type="file" multiple
               aria-label="Choose a recorded frames directory"
               ref={el => { if (el) el.webkitdirectory = true; }}
               onChange={e => { fromInput(e.target.files); e.target.value = ''; }}/>
      </label>
      <label className="file">
        Choose files…
        <input type="file" multiple accept=".png,.json"
               aria-label="Choose PNG and JSON frame files"
               onChange={e => { fromInput(e.target.files); e.target.value = ''; }}/>
      </label>
      {archive &&
       <span className="frames-counts">
         {frames.length} frame{frames.length === 1 ? '' : 's'} · {archive.unpaired} unpaired
         {archive.errors.length ? ` · ${archive.errors.length} error${archive.errors.length === 1 ? '' : 's'}` : ''}
       </span>}
      {busy && <span className="frames-busy">Reading…</span>}
      {error && <span className="text-warn">{error}</span>}
    </div>

    {archive && archive.errors.length > 0 &&
     <details className="frames-errors">
       <summary>{archive.errors.length} file error{
           archive.errors.length === 1 ? '' : 's'}</summary>
       <ul>
         {archive.errors.map((entry, i) => <li key={i}>
           <code>{entry.file}</code><span>{entry.message}</span>
         </li>)}
       </ul>
     </details>}

    {!archive ?
         <section className="panel frames-empty">
           <div className="panel-heading"><h2>Frames</h2><span>OFFLINE</span></div>
           <div className="empty-state">
             Choose a recorded frames directory to browse camera frames and
             their detection context. Files stay on this machine; nothing is
             uploaded and nothing goes through the bridge.
           </div>
         </section> :
         <div className="frames-grid">
           <section className="panel frames-list">
             <div className="panel-heading">
               <h2>Frames</h2><span>{filtered.length} / {frames.length} SHOWN</span>
             </div>
             <div className="frames-filters">
               <select aria-label="Filter by camera" value={camera}
                       onChange={e => { setCamera(e.target.value); setIndex(0); }}>
                 <option value="">All cameras</option>
                 {cameras.map(name => <option key={name} value={name}>{name}</option>)}
               </select>
               <div className="chips">
                 {(['', 'sampled', 'rejected'] as const).map(value =>
                   <button key={value || 'all'}
                           className={'chip' + (reason === value ? ' selected' : '')}
                           onClick={() => { setReason(value); setIndex(0); }}>
                     {value || 'all'}
                     <em>{value ?
                              frames.filter(f => f.metadata.reason === value).length :
                              frames.length}</em>
                   </button>)}
               </div>
             </div>
             <div className="frames-list-body">
               {filtered.map((frame, i) =>
                 <button key={frame.id}
                         className={'frame-item' + (i === index ? ' selected' : '')}
                         onClick={() => setIndex(i)}>
                   <span className={'ui-badge ' + frame.metadata.reason}>
                     {frame.metadata.reason}
                   </span>
                   <code>{frame.metadata.camera}</code>
                   <small>{formatNs(frame.metadata.capture_ns)} ns</small>
                 </button>)}
               {!filtered.length && <div className="empty-state">
                 No frames match that filter.
               </div>}
             </div>
           </section>

           <section className="panel frames-image">
             <div className="panel-heading">
               <h2>{selected ? selected.metadata.camera : 'Image'}</h2>
               <span>{selected ?
                          `${selected.metadata.width} × ${selected.metadata.height}` :
                          ''}</span>
             </div>
             {selected ?
                  <>
                    <div className="frames-image-stage">
                      {url && <img src={url} alt={`Frame ${selected.id}`}/>}
                      <svg className="frames-overlay"
                           viewBox={`0 0 ${selected.metadata.width} ${selected.metadata.height}`}
                           preserveAspectRatio="xMidYMid meet">
                        {selected.metadata.detections.map((detection, i) =>
                          <Detection key={i} detection={detection}
                                     fontSize={fontSize}/>)}
                      </svg>
                    </div>
                    <div className="frames-legend">
                      {selected.metadata.detections.length ?
                           selected.metadata.detections.map((detection, i) =>
                             <span key={i} className={
                                       detection.valid ? 'valid' : 'invalid'}>
                               <i/>
                               tag {detection.tag_id}
                               {detection.reject_reason ?
                                    ` — ${detection.reject_reason}` :
                                    ` — ${detection.valid ? 'valid' : 'invalid'}`}
                             </span>) :
                           <span>No detections decoded.</span>}
                    </div>
                  </> :
                  <div className="empty-state">No frame selected.</div>}
           </section>

           <section className="panel frames-details">
             <div className="panel-heading">
               <h2>Frame details</h2><span>{selected ? `${index + 1} / ${filtered.length}` : ''}</span>
             </div>
             {selected ?
                  <Details metadata={selected.metadata} position={index + 1}
                           total={filtered.length}/> :
                  <div className="empty-state">No frame selected.</div>}
           </section>
         </div>}
  </FramesKeyScope>;
}

export const framesPanel: PanelDefinition = {
  id: 'frames',
  title: 'Frames',
  category: 'Tools',
  description: 'Browse recorded camera frames and their detection context.',
  defaultConfig: {},
  parseConfig: () => ({}),
  component: FramesPanel
};
