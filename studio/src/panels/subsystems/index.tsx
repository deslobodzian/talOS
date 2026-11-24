// The subsystems panel: one card per discovered subsystem, with its enabled
// state, the target lanes in policy order (selected lane highlighted), the
// node's liveness/cadence/latency, and every state/status channel as a row —
// booleans as pills, numerics with a 30 s sparkline, and each target channel
// paired with its measured counterpart in one small overlaid chart plus the
// current error.
//
// Robot-agnostic: subsystem owners, channels and pairs are all discovered by
// shape (data/discovery.ts and pairing.ts), never by a hard-coded 2026 name.

import {useMemo} from 'react';

import {discoverSubsystems, type Subsystem} from '../../data/discovery';
import {nodeLiveness} from '../../data/freshness';
import {useChannelStale, useDictionary, useRecorded, useSeries, useSnapshot, useSystem} from '../../data/hooks';
import type {ChannelInfo} from '../../data/types';
import type {SystemNode} from '../../system';
import {rateKey} from '../../system';
import {channelDisplayName, channelLeaf} from '../../ui/channel';
import {duration, hertz, number} from '../../ui/formatters';
import {EmptyState, StatusDot} from '../../ui/primitives';
import {Sparkline} from '../../ui/Sparkline';
import {TimeSeriesChart} from '../../ui/TimeSeriesChart';
import type {PanelDefinition, PanelProps} from '../types';
import {pairTargets, type Pairing} from './pairing';
import {FreshnessNotice} from '../../ui/FreshnessNotice';
import './subsystems.css';

export type SubsystemsConfig = {selected: string[] | null};

const defaultConfig: SubsystemsConfig = {selected: null};
const EMPTY_RATES: Record<string, number> = {};
const SPARKLINE_WIDTH = 150;

function parseConfig(raw: unknown): SubsystemsConfig {
  const obj = raw && typeof raw === 'object' && !Array.isArray(raw) ?
      raw as Record<string, unknown> :
      {};
  if (!Array.isArray(obj.selected)) return {selected: null};
  return {
    selected: (obj.selected as unknown[]).filter(
        (s): s is string => typeof s === 'string')
  };
}

type Kind = 'boolean' | 'enum' | 'numeric';

// Classify one channel for rendering. Enum channels declare labels; a boolean
// is a unitless, label-less channel whose value is exactly 0 or 1 (the wire's
// spelling); everything else is numeric.
function kindOf(info: ChannelInfo, value: number | undefined): Kind {
  if (info.labels.length > 0) return 'enum';
  if (!info.unit && (value === 0 || value === 1)) return 'boolean';
  return 'numeric';
}

function loopRate(node: SystemNode, rates: Record<string, number>): number | undefined {
  const timers = node.sources.filter(s => s.kind === 'TIMER');
  const candidates = timers.length ? timers : node.sources;
  const measured = candidates
                        .map(s => rates[rateKey(node.name, s.id)])
                        .filter((r): r is number => r !== undefined && Number.isFinite(r));
  return measured.length ? Math.max(...measured) : undefined;
}

function worstLatency(node: SystemNode): number | undefined {
  const latencies = node.sources
                        .filter(s => s.kind === 'TIMER' || s.kind === 'WATCHER')
                        .map(s => Number(s.max_latency_ns))
                        .filter(n => Number.isFinite(n));
  return latencies.length ? Math.max(...latencies) : undefined;
}

function ChannelRow(
    {channel, info, value, seriesValues}: {
      channel: string; info: ChannelInfo; value: number | undefined;
      seriesValues: Float64Array | undefined;
    }) {
  const stale = useChannelStale(channel);
  const kind = kindOf(info, value);
  const finiteValue = value !== undefined && Number.isFinite(value);
  return <li className="channel-row">
    <code className="channel-name">{channelDisplayName(channel)}{stale && <em className="text-warn"> · STALE</em>}</code>
    <span className={'channel-value ' + kind}>
      {kind === 'boolean' ?
           <span className={'pill ' + (value === 1 ? 'on' : 'off')}>
             {value === undefined ? '—' : value === 1 ? 'on' : 'off'}
           </span> :
       kind === 'enum' ?
           (value !== undefined && Number.isInteger(value) && value >= 0 &&
                    value < info.labels.length ?
                info.labels[value] :
                value === undefined ? '—' : number(value)) :
           <>{finiteValue ? number(value, 4) : '—'}
             {finiteValue && info.unit && <em className="unit">{info.unit}</em>}
           </>}
    </span>
    {kind === 'numeric' && seriesValues && seriesValues.length > 0 ?
         <Sparkline values={seriesValues} width={110} height={24}/> :
         <span className="spark-slot"/>}
  </li>;
}

function PairRow(
    {pair, measuredValue, targetValue, measuredSeries, targetSeries}: {
      pair: Pairing;
      measuredValue: number | undefined;
      targetValue: number | undefined;
      measuredSeries: Float64Array | undefined;
      targetSeries: Float64Array | undefined;
    }) {
  const measuredStale = useChannelStale(pair.measured);
  const targetStale = useChannelStale(pair.target);
  const rawError = measuredValue !== undefined && targetValue !== undefined &&
      Number.isFinite(measuredValue) && Number.isFinite(targetValue) ?
      measuredValue - targetValue :
      undefined;
  const error = rawError !== undefined && Number.isFinite(rawError) ? rawError : undefined;
  return <li className="pair-row">
    <div className="pair-label">
      <b>{pair.stem}</b>
      {(measuredStale || targetStale) && <em className="text-warn">STALE {measuredStale ? 'measured' : ''} {targetStale ? 'target' : ''}</em>}
      <span className="legend">
        <i className="swatch measured"/>measured
        <i className="swatch target"/>target
      </span>
    </div>
    {(measuredSeries && measuredSeries.length > 1 && targetSeries &&
      targetSeries.length > 1) ?
         <TimeSeriesChart
             lines={[
               {name: 'measured', color: '#48e1cf', dash: false,
                values: measuredSeries},
               {name: 'target', color: '#ffbd69', dash: true,
                values: targetSeries}
             ]}
             width={150} height={40} fillWidth={false} bare legend={false}/> :
         <span className="pair-empty">no history</span>}
    <span className="pair-error">
      Δ {error === undefined ? '—' : number(error, 3) + (pair.unit ? ' ' + pair.unit : '')}
    </span>
  </li>;
}

const BLOCKING_KIND_FALLBACK = ['NONE', 'AXIS', 'BUTTON', 'POV', 'OTHER'] as const;

// Human-readable arming status for any operator node publishing
// `<owner>/status/armed` and `<owner>/status/blocking_kind` (REQ-OPS-55).
function formatDisarmedStatus(
    sub: Subsystem,
    infoBy: Map<string, ChannelInfo>,
    snapshot: {channels: Record<string, number>} | null): string | null {
  const armedChannel = sub.statusChannels.find(c => channelLeaf(c) === 'armed');
  const kindChannel = sub.statusChannels.find(c => channelLeaf(c) === 'blocking_kind');
  if (!armedChannel || !kindChannel || !snapshot) return null;
  const armed = snapshot.channels[armedChannel];
  if (armed === undefined || armed !== 0) return null;

  const kindVal = snapshot.channels[kindChannel];
  const labels = infoBy.get(kindChannel)?.labels ?? [];
  let kindLabel = 'NONE';
  if (kindVal !== undefined && Number.isInteger(kindVal) && kindVal >= 0) {
    if (kindVal < labels.length) {
      kindLabel = labels[kindVal].toUpperCase();
    } else if (kindVal < BLOCKING_KIND_FALLBACK.length) {
      kindLabel = BLOCKING_KIND_FALLBACK[kindVal];
    }
  }

  const joyChannel = sub.statusChannels.find(c => channelLeaf(c) === 'blocking_joystick_index');
  const inputChannel = sub.statusChannels.find(c => channelLeaf(c) === 'blocking_input_index');
  const joy = joyChannel !== undefined ? snapshot.channels[joyChannel] : undefined;
  const input = inputChannel !== undefined ? snapshot.channels[inputChannel] : undefined;
  const hasIds = joy !== undefined && input !== undefined &&
      Number.isInteger(joy) && Number.isInteger(input) && joy >= 0 && input >= 0;

  if (kindLabel === 'AXIS') {
    return hasIds ? `Disarmed — joystick ${joy} axis ${input} held` :
                    'Disarmed — joystick axis held';
  }
  if (kindLabel === 'BUTTON') {
    return hasIds ? `Disarmed — joystick ${joy} button ${input} held` :
                    'Disarmed — joystick button held';
  }
  if (kindLabel === 'POV') {
    return hasIds ? `Disarmed — joystick ${joy} POV ${input} held` :
                    'Disarmed — joystick POV held';
  }
  if (kindLabel === 'OTHER') {
    return 'Disarmed — release held input to arm';
  }
  const alertChannel =
      sub.statusChannels.find(c => channelLeaf(c) === 'alert_warning_disarmed');
  if (alertChannel !== undefined && snapshot.channels[alertChannel] === 0) {
    return null;
  }
  return 'Disarmed — release inputs to arm';
}

function SubsystemCard(
    {sub, infoBy, snapshot, node, graphStale, rates, series}: {
      sub: Subsystem;
      infoBy: Map<string, ChannelInfo>;
      snapshot: {channels: Record<string, number>} | null;
      node: SystemNode | undefined;
      graphStale: boolean;
      rates: Record<string, number>;
      series: {values: Record<string, Float64Array>};
    }) {
  const channels = useMemo(
      () => [...sub.stateChannels, ...sub.statusChannels],
      [sub.stateChannels, sub.statusChannels]);
  const rows = channels
                   .map(channel => ({channel, info: infoBy.get(channel)}))
                   .filter((r): r is {channel: string; info: ChannelInfo} =>
                       r.info !== undefined);
  const enabledChannel = channels.find(c => channelLeaf(c) === 'enabled');
  const enabledValue = enabledChannel ?
      snapshot?.channels[enabledChannel] :
      undefined;

  const pairs = useMemo(() => {
    const subset = channels
                       .map(name => infoBy.get(name))
                       .filter((i): i is ChannelInfo => i !== undefined)
                       .map(i => ({name: i.name, unit: i.unit}));
    return pairTargets(subset);
  }, [channels, infoBy]);

  const selection = sub.selection;
  const disarmedStatus = formatDisarmedStatus(sub, infoBy, snapshot);
  const liveness = node ? nodeLiveness(node, {stale: graphStale}) : null;
  const loop = node ? loopRate(node, rates) : undefined;
  const latency = node ? worstLatency(node) : undefined;
  const lane = (name: string) =>
      <span key={name}
            className={'lane' + (selection?.selected === name ? ' selected' : '')}>
        {name}
      </span>;

  return <section className="subsystem-card">
    <FreshnessNotice prefix={sub.name + '/'}/>
    <header className="subsystem-head">
      <h3><StatusDot active={liveness === 'live'}/>{sub.name}</h3>
      <span className={
                'subsystem-enabled' +
                (enabledValue === 1 ? ' on' : enabledValue === 0 ? ' off' : '')}>
        {enabledValue === 1 ? 'enabled' : enabledValue === 0 ? 'disabled' : '—'}
      </span>
    </header>

    {disarmedStatus && <div className="subsystem-meta">
      <em className="text-warn">{disarmedStatus}</em>
    </div>}

    <div className="subsystem-meta">
      <div className="subsystem-lanes">
        {selection === null ?
             <span className="text-muted">no target node</span> :
         selection.sources.length === 0 ?
             <span className="text-muted">no lanes</span> :
             <>
               {selection.sources.map(lane)}
               {selection.selected === null && <span className="lane none">no lane selected</span>}
             </>}
      </div>
      <dl className="subsystem-health">
        <div><dt>alive</dt>
          <dd className={liveness === 'live' ? 'ok' : liveness ? 'warn' : 'muted'}>
            {liveness === 'live' ? 'yes' : liveness === 'stopped' ? 'stale' :
             liveness === 'unknown' ? 'unknown' : '—'}
          </dd></div>
        <div><dt>loop</dt><dd>{node ? hertz(loop) : '—'}</dd></div>
        <div><dt>latency</dt>
          <dd>{latency === undefined ? '—' : duration(latency)}</dd></div>
      </dl>
    </div>

    <ul className="channel-rows">
      {rows.map(({channel, info}) =>
        <ChannelRow key={channel} channel={channel} info={info}
                    value={snapshot?.channels[channel]}
                    seriesValues={series.values[channel]}/>)}
    </ul>

    {pairs.length > 0 && <div className="pairs">
      <div className="pairs-heading">demand vs measured</div>
      <ul className="pair-rows">
        {pairs.map(pair =>
          <PairRow key={pair.target + pair.measured} pair={pair}
                   measuredValue={snapshot?.channels[pair.measured]}
                   targetValue={snapshot?.channels[pair.target]}
                   measuredSeries={series.values[pair.measured]}
                   targetSeries={series.values[pair.target]}/>)}
      </ul>
    </div>}
  </section>;
}

function SubsystemsPanel({config, setConfig}: PanelProps<SubsystemsConfig>) {
  const dictionary = useDictionary();
  const system = useSystem();
  const recorded = useRecorded();
  const snapshot = useSnapshot();
  const graph = recorded ? null : system.graph;

  const subsystems = useMemo(
      () => discoverSubsystems(dictionary, graph), [dictionary, graph]);

  const infoBy = useMemo(() => {
    const map = new Map<string, ChannelInfo>();
    for (const info of dictionary) map.set(info.name, info);
    return map;
  }, [dictionary]);

  const visible = useMemo(() => {
    if (!config.selected || config.selected.length === 0) return subsystems;
    const want = new Set(config.selected);
    return subsystems.filter(s => want.has(s.name));
  }, [subsystems, config.selected]);

  const seriesChannels = useMemo(() => {
    const names = new Set<string>();
    for (const sub of visible)
      for (const channel of [...sub.stateChannels, ...sub.statusChannels]) {
        const info = infoBy.get(channel);
        if (info && info.labels.length === 0) names.add(channel);
      }
    return [...names].sort();
  }, [visible, infoBy]);

  const series = useSeries(seriesChannels, {seconds: 30}, SPARKLINE_WIDTH);

  const allSelected = !config.selected || config.selected.length === 0;

  const toggle = (name: string) => {
    const selected = config.selected;
    if (!selected || selected.length === 0) {
      setConfig({...config, selected: [name]});
    } else if (selected.includes(name)) {
      const next = selected.filter(n => n !== name);
      setConfig({...config, selected: next.length ? next : null});
    } else {
      setConfig({...config, selected: [...selected, name]});
    }
  };

  return <div className="subsystems">
    <div className="subsystems-toolbar">
      <div className="chips">
        <button className={'chip' + (allSelected ? ' selected' : '')}
                onClick={() => setConfig({...config, selected: null})}>
          all
        </button>
        {subsystems.map(sub =>
          <button key={sub.name}
                  className={'chip' +
                             (!allSelected && config.selected!.includes(sub.name) ?
                                  ' selected' :
                                  '')}
                  onClick={() => toggle(sub.name)}>
            {sub.name}
          </button>)}
      </div>
    </div>

    {visible.length === 0 ?
         <EmptyState>
           {subsystems.length ?
                'No subsystem matches the current filter.' :
                'No subsystems yet. Connect to a robot bridge, or start demo data to explore the viewer.'}
         </EmptyState> :
         <div className="subsystems-grid">
           {visible.map(sub =>
             <SubsystemCard key={sub.name} sub={sub} infoBy={infoBy}
                            snapshot={snapshot}
                            node={graph?.nodes.find(n => n.name === sub.name)}
                            graphStale={recorded ? false : system.stale}
                            rates={recorded ? EMPTY_RATES : system.rates} series={series}/>)}
         </div>}
  </div>;
}

export const subsystemsPanel: PanelDefinition<SubsystemsConfig> = {
  id: 'subsystems',
  title: 'Subsystems',
  category: 'Robot',
  description: 'One card per subsystem: state, status and demand-vs-measured.',
  defaultConfig,
  parseConfig,
  component: SubsystemsPanel,
  recorded: true
};
