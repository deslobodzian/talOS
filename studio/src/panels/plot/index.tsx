// The plot panel: a uPlot chart over one or more channels, replacing both the
// old Signals tab and the user's graph tabs with one configurable panel type.
// Config is `{channels, windowSeconds, events}` (windowSeconds null = all
// history, events toggles severity-coloured event markers); `parseConfig`
// accepts the older `{channels, windowSeconds}` shape and any malformed value.
//
// Series are grouped by unit onto at most two y-axes (left/right), enum
// channels render as step lines with their labels as ticks, and the legend
// shows each series' value at the cursor. uPlot is created and torn down in
// effects; the render body only lays out the toolbar, the channel drawer and
// the plot container, so react-dom/server can render the empty panel in tests.
//
// From a recorded session the series is decimated to the plot's pixel width
// (rounded up to a bucket size, recorded.ts POINT_SIZES):
// each point is a bucket's last sample, and a bucket of more than one sample
// also draws its min..max range as a vertical bar in the series colour.

import {useEffect, useMemo, useRef, useState} from 'react';
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';

import {useActions, useDictionary, useEvents, usePlayback, useSeries} from '../../data/hooks';
import type {ChannelInfo, RobotEvent, Series} from '../../data/types';
import {channelDisplayName} from '../../ui/channel';
import {number} from '../../ui/formatters';
import {EmptyState} from '../../ui/primitives';
import type {PanelDefinition, PanelProps} from '../types';
import {assignYAxes, type PlotChannel} from './axes';
import './plot.css';

export type PlotConfig = {
  channels: string[];
  windowSeconds: number|null;
  events: boolean;
};

const defaultConfig: PlotConfig = {channels: [], windowSeconds: null, events: true};

function parseConfig(raw: unknown): PlotConfig {
  const obj = raw && typeof raw === 'object' && !Array.isArray(raw) ?
      raw as Record<string, unknown> :
      {};
  const channels = Array.isArray(obj.channels) ?
      obj.channels.filter((c): c is string => typeof c === 'string') :
      [];
  const ws = obj.windowSeconds;
  const windowSeconds =
      typeof ws === 'number' && Number.isFinite(ws) && ws > 0 ? ws : null;
  const events = obj.events !== false;
  return {channels, windowSeconds, events};
}

const palette = ['#48e1cf', '#ffbd69', '#b59bff', '#ff778b'];

const SEVERITY_COLOR: Record<string, string> = {
  info: '#48e1cf',
  warn: '#ffbd69',
  error: '#ff778b'
};

const WINDOWS: {label: string; value: number|null}[] = [
  {label: '10 s', value: 10},
  {label: '30 s', value: 30},
  {label: '2 min', value: 120},
  {label: 'All history', value: null}
];

function ownerOf(name: string): string {
  const i = name.indexOf('/');
  return i <= 0 ? '(root)' : name.slice(0, i);
}

// Thin vertical line per event, coloured by severity, drawn over the plot via
// uPlot's `draw` hook. Times are seconds since the series origin.
function drawEvents(u: uPlot, events: RobotEvent[], origin: bigint|null,
                    enabled: boolean) {
  if (!enabled || origin === null) return;
  const {min, max} = u.scales.x;
  if (min == null || max == null) return;
  const ctx = u.ctx;
  const {top, height} = u.bbox;
  for (const ev of events) {
    const sec = Number(BigInt(ev.t_ns) - origin) / 1e9;
    if (!Number.isFinite(sec) || sec < min || sec > max) continue;
    const x = u.valToPos(sec, 'x', true);
    ctx.strokeStyle = SEVERITY_COLOR[ev.severity] ?? '#8499ad';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(x, top);
    ctx.lineTo(x, top + height);
    ctx.stroke();
  }
}

// The min..max envelope of every decimated bucket that held more than one
// sample. The live store's series carries no envelope, so nothing is drawn.
function drawEnvelope(u: uPlot, series: Series, ordered: string[],
                      scaleOf: Map<string, string>) {
  if (!series.min || !series.max || !series.count) return;
  const ctx = u.ctx;
  const {left, top, width, height} = u.bbox;
  ctx.save();
  ctx.beginPath();
  ctx.rect(left, top, width, height);
  ctx.clip();
  ctx.globalAlpha = 0.35;
  ctx.lineWidth = Math.max(1, globalThis.devicePixelRatio ?? 1);
  ordered.forEach((name, k) => {
    const min = series.min![name], max = series.max![name], count = series.count![name];
    if (!min || !max || !count) return;
    const scale = scaleOf.get(name) ?? 'y';
    ctx.strokeStyle = palette[k % palette.length];
    ctx.beginPath();
    for (let j = 0; j < count.length; j++) {
      if (count[j] < 2 || !Number.isFinite(min[j]) || !Number.isFinite(max[j])) continue;
      const x = u.valToPos(series.times[j], 'x', true);
      ctx.moveTo(x, u.valToPos(min[j], scale, true));
      ctx.lineTo(x, u.valToPos(max[j], scale, true));
    }
    ctx.stroke();
  });
  ctx.restore();
}

function ChannelDrawer({channels, selected, onToggle}: {
  channels: ChannelInfo[];
  selected: string[];
  onToggle: (name: string) => void;
}) {
  const [query, setQuery] = useState('');
  const needle = query.trim().toLowerCase();

  const groups = useMemo(() => {
    const sel = new Set(selected);
    const visible = needle ?
        channels.filter(
            c => c.name.toLowerCase().includes(needle) ||
                c.unit.toLowerCase().includes(needle)) :
        channels;
    const byOwner = new Map<string, ChannelInfo[]>();
    for (const c of visible) {
      const owner = ownerOf(c.name);
      const list = byOwner.get(owner);
      if (list) list.push(c);
      else byOwner.set(owner, [c]);
    }
    return [...byOwner.entries()]
        .sort((a, b) => {
          const ra = a[0] === '(root)', rb = b[0] === '(root)';
          if (ra !== rb) return ra ? 1 : -1;
          return a[0] < b[0] ? -1 : a[0] > b[0] ? 1 : 0;
        })
        .map(([owner, list]) => ({
          owner,
          channels: list.slice().sort(
              (a, b) => Number(sel.has(b.name)) - Number(sel.has(a.name)) ||
                  a.name.localeCompare(b.name))
        }));
  }, [channels, selected, needle]);

  return <aside className="channel-drawer">
    <input aria-label="Search channels" className="search"
           placeholder="Search channels…" value={query}
           onChange={e => setQuery(e.target.value)}/>
    <div className="channel-drawer-list">
      {groups.map(g => <section className="channel-drawer-group" key={g.owner}>
        <h4>{g.owner}</h4>
        {g.channels.map(c => <label key={c.name}
            className={selected.includes(c.name) ? 'selected' : ''}
            title={c.name}>
          <input type="checkbox" checked={selected.includes(c.name)}
                 onChange={() => onToggle(c.name)}/>
          <span className="channel-name">{channelDisplayName(c.name)}</span>
          {c.unit && <em>{c.unit}</em>}
        </label>)}
      </section>)}
      {!groups.length && <span className="channel-drawer-empty">
        {channels.length ? 'No channels match.' : 'No channels yet.'}
      </span>}
    </div>
  </aside>;
}

function PlotPanel({config, setConfig}: PanelProps<PlotConfig>) {
  const dictionary = useDictionary();
  const playback = usePlayback();
  const actions = useActions();
  const events = useEvents();
  // The plot's width in CSS pixels: a recorded source's bucket count.
  const [points, setPoints] = useState<number|undefined>(undefined);
  const series = useSeries(
      config.channels,
      config.windowSeconds === null ? 'all' : {seconds: config.windowSeconds},
      points);
  const seriesRef = useRef<Series>(series);
  seriesRef.current = series;

  const [drawerOpen, setDrawerOpen] = useState(true);

  const root = useRef<HTMLDivElement>(null);
  const chart = useRef<uPlot|null>(null);
  const latest = useRef({seek: actions.seek});
  latest.current.seek = actions.seek;
  const originRef = useRef<bigint|null>(null);
  const anchor = useRef<bigint|null>(null);
  const eventsRef = useRef<RobotEvent[]>(events);
  eventsRef.current = events;
  const enabledRef = useRef(config.events);
  enabledRef.current = config.events;
  const [delta, setDelta] = useState<number|null>(null);

  const keys = config.channels;
  const keySig = keys.join('\u0000');

  const info = useMemo(() => {
    const m = new Map<string, ChannelInfo>();
    for (const c of dictionary) m.set(c.name, c);
    return m;
  }, [dictionary]);

  // A signature of everything the chart config depends on: the channel set and
  // each channel's unit and labels. When it changes the chart is rebuilt.
  const channelSig = useMemo(
      () => keys.map(k => {
        const c = info.get(k);
        return `${k}\u0001${c?.unit ?? ''}\u0001${(c?.labels ?? []).join('\u0002')}`;
      }).join('\u0000'),
      [keySig, info]);
  const prevWindowRef = useRef<number|null>(config.windowSeconds);
  const prevChannelSigRef = useRef<string>(channelSig);

  const axes = useMemo(() => {
    const plotChannels: PlotChannel[] = keys.map(k => ({
      name: k,
      unit: info.get(k)?.unit ?? '',
      labels: info.get(k)?.labels ?? []
    }));
    return assignYAxes(plotChannels);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [channelSig]);

  const ordered = useMemo(() => axes.flatMap(a => a.channels), [axes]);
  const scaleOf = useMemo(() => {
    const m = new Map<string, string>();
    for (const a of axes) for (const ch of a.channels) m.set(ch, a.scale);
    return m;
  }, [axes]);
  const orderedRef = useRef({ordered, scaleOf});
  orderedRef.current = {ordered, scaleOf};

  const toggle = (name: string) => setConfig({
    ...config,
    channels: keys.includes(name) ?
        keys.filter(k => k !== name) :
        [...keys, name]
  });

  const setWindow = (value: number|null) =>
      setConfig({...config, windowSeconds: value});

  const setEvents = (on: boolean) => setConfig({...config, events: on});

  // Build (or rebuild) the chart when the channel set, units or labels change.
  useEffect(() => {
    if (!root.current) return;
    const container = root.current;
    const leftAxis = axes.find(a => a.scale === 'y');
    const rightAxis = axes.find(a => a.scale === 'y2');
    const plot = new uPlot(
        {
          width: Math.max(200, container.clientWidth - 24),
          height: Math.max(160, container.clientHeight - 12),
          legend: {show: true, live: true},
          cursor: {sync: {key: 'telemetry'}},
          scales: {
            x: {time: false, auto: true},
            y: leftAxis?.labels ?
                {auto: false,
                 range: [0, Math.max(0, leftAxis.labels.length - 1)] as [number, number]} :
                {auto: true},
            y2: rightAxis?.labels ?
                {auto: false,
                 range: [0, Math.max(0, rightAxis.labels.length - 1)] as [number, number]} :
                {auto: true}
          },
          axes: [
            {stroke: '#8499ad', grid: {stroke: '#253343'}},
            {scale: 'y', side: 3, stroke: '#8499ad', grid: {stroke: '#253343'},
             label: leftAxis?.label ?? '',
             ...(leftAxis?.labels ? {values: enumValues(leftAxis.labels)} : {})},
            ...(rightAxis ?
                    [{scale: 'y2', side: 1, stroke: '#8499ad',
                      grid: {stroke: '#253343'},
                      label: rightAxis.label,
                      ...(rightAxis.labels ?
                              {values: enumValues(rightAxis.labels)} :
                              {})}] :
                    [])
          ],
          series: [
            {},
            ...ordered.map((name, i) => {
              const c = info.get(name);
              const isEnum = (c?.labels?.length ?? 0) > 0;
              return {
                label: channelDisplayName(name),
                scale: scaleOf.get(name) ?? 'y',
                stroke: palette[i % palette.length],
                width: 1.5,
                ...(isEnum ? {paths: uPlot.paths.stepped?.({align: 1})} : {}),
                value: (self: uPlot, raw: number) => {
                  if (raw == null) return '—';
                  if (isEnum) {
                    const labels = c!.labels;
                    const n = Math.round(raw);
                    return n >= 0 && n < labels.length ? labels[n] : String(raw);
                  }
                  return number(raw) + (c?.unit ? ` ${c.unit}` : '');
                }
              };
            })
          ],
          hooks: {
            setCursor: [u => {
              const index = u.cursor.idx;
              if (index == null || anchor.current == null) return;
              const sample = u.data[0][index];
              if (sample != null && originRef.current != null)
                setDelta(Number(BigInt(
                    originRef.current + BigInt(Math.round(sample * 1e9))) -
                    anchor.current) / 1e9);
            }],
            draw: [u => {
              drawEnvelope(u, seriesRef.current, orderedRef.current.ordered,
                           orderedRef.current.scaleOf);
              drawEvents(u, eventsRef.current, originRef.current, enabledRef.current);
            }]
          }
        },
        [[], ...ordered.map(() => [])], container);
    chart.current = plot;

    const click = () => {
      const index = plot.cursor.idx;
      if (index != null && originRef.current != null) {
        const t = originRef.current +
            BigInt(Math.round(plot.data[0][index] * 1e9));
        anchor.current = t;
        latest.current.seek(t.toString());
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

    const resize = new ResizeObserver(() => {
      plot.setSize({
        width: Math.max(200, container.clientWidth - 24),
        height: Math.max(160, container.clientHeight - 12)
      });
      setPoints(Math.max(200, container.clientWidth - 24));
    });
    resize.observe(container);
    setPoints(Math.max(200, container.clientWidth - 24));

    return () => {
      resize.disconnect();
      container.removeEventListener('dblclick', click);
      container.removeEventListener('wheel', pan);
      plot.destroy();
      chart.current = null;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [channelSig]);

  // Feed the chart when the retained series (or the cursor) moves.
  useEffect(() => {
    const plot = chart.current;
    if (!plot) return;
    const windowChanged = prevWindowRef.current !== config.windowSeconds ||
        prevChannelSigRef.current !== channelSig;
    prevWindowRef.current = config.windowSeconds;
    prevChannelSigRef.current = channelSig;
    if (ordered.length === 0) {
      plot.setData([[], ...ordered.map(() => [])]);
      originRef.current = null;
      anchor.current = null;
      return;
    }
    originRef.current = BigInt(series.t0_ns);
    const times = Array.from(series.times);
    const values = ordered.map(
        name => Array.from(series.values[name] ?? new Float64Array(0)));
    const resetScales = windowChanged || playback.mode === 'LIVE_STREAMING' ||
        times.length === 0;
    plot.setData([times, ...values], resetScales);
    if (windowChanged && config.windowSeconds !== null && times.length > 0 &&
        times[0] < times[times.length - 1])
      plot.setScale('x', {min: times[0], max: times[times.length - 1]});
    if (playback.cursor_ns && playback.mode !== 'LIVE_STREAMING')
      plot.setCursor({
        left: plot.valToPos(
            Number(BigInt(playback.cursor_ns) - BigInt(series.t0_ns)) / 1e9,
            'x'),
        top: 0
      });
  }, [series, channelSig, config.windowSeconds, playback.cursor_ns, playback.mode, ordered]);

  // Redraw when events arrive or the marker toggle flips, without rebuilding.
  useEffect(() => {
    chart.current?.redraw();
  }, [events, config.events]);

  return <div className="plot-panel">
    <div className="plot-toolbar">
      <button className={'drawer-toggle' + (drawerOpen ? ' active' : '')}
              aria-label="Toggle channel drawer" aria-expanded={drawerOpen}
              onClick={() => setDrawerOpen(o => !o)}>
        Channels
      </button>
      <label>
        Window
        <select aria-label="Plot window" value={
            config.windowSeconds === null ? 'all' : String(config.windowSeconds)}
          onChange={e => setWindow(
              e.target.value === 'all' ? null : Number(e.target.value))}>
          {WINDOWS.map(w => <option key={w.label}
              value={w.value === null ? 'all' : String(w.value)}>
            {w.label}
          </option>)}
          {config.windowSeconds !== null &&
           !WINDOWS.some(w => w.value === config.windowSeconds) &&
           <option value={String(config.windowSeconds)}>{config.windowSeconds} s</option>}
        </select>
      </label>
      <label className="events-toggle">
        <input type="checkbox" checked={config.events}
               onChange={e => setEvents(e.target.checked)}/>
        Events
      </label>
      <span className="spacer"/>
      <span>{keys.length} channel{keys.length === 1 ? '' : 's'}</span>
    </div>
    <div className="plot-body">
      {drawerOpen &&
          <ChannelDrawer channels={dictionary} selected={keys} onToggle={toggle}/>}
      {keys.length === 0 ?
          <div className="plot-empty">
            <EmptyState>Select channels from the drawer, or search.</EmptyState>
          </div> :
          <div className="plot" ref={root}/>}
    </div>
    <div className="panel-footer">
      DRAG: ZOOM · SHIFT+WHEEL: PAN · DOUBLE CLICK: SEEK
      <span>Δt {delta == null ? '—' : number(delta) + ' s'}</span>
    </div>
  </div>;
}

// Map an enum axis's integer splits to their labels.
function enumValues(labels: string[]) {
  return (_self: uPlot, splits: number[]) =>
      splits.map(s => {
        const i = Math.round(s);
        return i >= 0 && i < labels.length ? labels[i] : '';
      });
}

export const plotPanel: PanelDefinition<PlotConfig> = {
  id: 'plot',
  title: 'Plot',
  category: 'Data',
  description: 'Plot channels over time, with a cursor that follows playback.',
  defaultConfig,
  parseConfig,
  component: PlotPanel,
  recorded: true
};
