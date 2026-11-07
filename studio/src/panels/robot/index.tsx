// The robot panel: the at-a-glance answer to "what is the robot doing right
// now". A dense grid of cards over Driver Station, routine phase, power, link,
// pose, the discovered subsystems and cameras, and the recent event feed. It
// reads only through the data hooks and discovers owners/channels by shape, so
// it stays robot-agnostic: no 2026 node names are hard-coded here.

import {useMemo} from 'react';
import type {ReactNode} from 'react';

import {poseError, simTruthPose} from '../../core';
import {
  useActions,
  useDictionary,
  useEvents,
  useRobotStatus,
  useSeries,
  useSnapshot,
  useSystem
} from '../../data/hooks';
import type {DriverStationMode, RobotEvent, Series} from '../../data/types';
import {
  discoverCameras,
  discoverEstimators,
  discoverSubsystems,
  type Camera,
  type Subsystem
} from '../../data/discovery';
import {channelLeaf} from '../../ui/channel';
import {duration, number} from '../../ui/formatters';
import {EmptyState, StatList} from '../../ui/primitives';
import {Sparkline} from '../../ui/Sparkline';
import type {PanelDefinition} from '../types';
import {ChannelReading, ChannelStat as Stat, FreshnessNotice} from '../../ui/FreshnessNotice';
import './robot.css';

export type RobotConfig = Record<string, never>;

const defaultConfig: RobotConfig = {};
const parseConfig = (_raw: unknown): RobotConfig => ({});

// --- pure derivations -------------------------------------------------------

// The set-bit list of a fault/status mask: a comma list of the bits that are
// set, or 'none' when the mask is clear. NaN/absent is null, not "none": a
// mask with no reading is a different statement from a mask with no faults.
export function maskBits(value: number|null|undefined): string|null {
  if (value == null || !Number.isFinite(value) || value < 0) return null;
  const bits: number[] = [];
  let v = Math.trunc(value);
  for (let i = 0; i < 53 && v !== 0; i++) {
    if (v & 1) bits.push(i);
    v = Math.floor(v / 2);
  }
  return bits.length ? bits.join(', ') : 'none';
}

// The last finite counter value and accumulated increments over the trailing
// `windowSeconds`. A decrease starts a new counter from zero. Both are null
// when the column has no reading: "not measured" is never "zero".
export function windowDelta(
    series: Series, name: string, windowSeconds: number):
    {current: number|null; delta: number|null} {
  const values = series.values[name];
  if (!values || values.length === 0) return {current: null, delta: null};
  let last = -1;
  for (let i = values.length - 1; i >= 0; i--)
    if (Number.isFinite(values[i])) { last = i; break; }
  if (last < 0) return {current: null, delta: null};

  const current = values[last];
  const bound = series.times[last] - windowSeconds;
  let prior = last;
  for (let i = last - 1; i >= 0; i--) {
    if (!Number.isFinite(values[i])) continue;
    prior = i;
    if (series.times[i] <= bound) break;
  }
  let delta = 0;
  let prev = values[prior];
  for (let i = prior + 1; i <= last; i++) {
    const v = values[i];
    if (!Number.isFinite(v)) continue;
    delta += v >= prev ? v - prev : Math.max(0, v);
    prev = v;
  }
  return {current, delta};
}

// The routine phase read off one enum channel's history: the current label,
// how long it has held (from the cursor back to the last label change), and
// the label it held before. Nulls are "not known" rather than a guessed
// steady state.
export function phaseFromSeries(
    series: Series, name: string, labels: string[]):
    {current: string|null; previous: string|null; seconds: number|null} {
  const values = series.values[name];
  if (!values || values.length === 0) return {current: null, previous: null, seconds: null};

  let end = -1;
  for (let i = values.length - 1; i >= 0; i--)
    if (Number.isFinite(values[i])) { end = i; break; }
  if (end < 0) return {current: null, previous: null, seconds: null};

  const label = (value: number): string|null => {
    if (!labels.length || !Number.isInteger(value) || value < 0 ||
        value >= labels.length)
      return null;
    return labels[value];
  };

  const currentValue = values[end];
  const current = label(currentValue);

  // Walk back through finite samples of the current phase, stopping at a change.
  let previous: string|null = null;
  let earliestFinite = end;
  for (let i = end - 1; i >= 0; i--) {
    if (!Number.isFinite(values[i])) continue;
    if (label(values[i]) !== current) {
      previous = label(values[i]);
      break;
    }
    earliestFinite = i;
  }

  // Gaps before the first finite reading of this run are not phase time.
  return {current, previous, seconds: series.times[end] - series.times[earliestFinite]};
}

const MODE_LABEL: Record<DriverStationMode, string> = {
  disconnected: 'DISCONNECTED',
  disabled: 'DISABLED',
  autonomous: 'AUTONOMOUS',
  teleop: 'TELEOP',
  test: 'TEST',
  estop: 'E-STOP'
};

const asBool = (value: number|undefined): boolean|null =>
    value === undefined ? null : value !== 0;

// `match_time_s` as m:ss, or '—' when the channel is absent.
const matchTime = (value: number|null): string => {
  if (value == null) return '—';
  const m = Math.floor(value / 60);
  const s = Math.floor(value % 60);
  return `${m}:${String(s).padStart(2, '0')}`;
};

// The enum channel label to surface on a subsystem tile: null for the neutral
// first label, for a missing reading, or for a value that names no label.
export function badgeFor(value: number|undefined, labels: string[]|undefined):
    string|null {
  if (value === undefined || !labels || labels.length < 2) return null;
  if (!Number.isInteger(value) || value < 0 || value >= labels.length)
    return null;
  return value === 0 ? null : labels[value];
}

// --- cards ------------------------------------------------------------------

function Card({title, eyebrow, children}: {
  title: string; eyebrow: string; children: ReactNode;
}) {
  return <section className="robot-card">
    <div className="robot-card-head"><h2>{title}</h2><span>{eyebrow}</span></div>
    <div className="robot-card-body">{children}</div>
  </section>;
}

function ModeBadge({mode}: {mode: DriverStationMode|null}) {
  return <em className={'mode-badge' + (mode ? ' ' + mode : ' unknown')}>
    <ChannelReading channel="hw/state/driver_station/enabled">{mode ? MODE_LABEL[mode] : '—'}</ChannelReading>
  </em>;
}

function SubsystemTile({subsystem, channels, labels}: {
  subsystem: Subsystem;
  channels: Record<string, number>;
  labels: Record<string, string[]>;
}) {
  const enabledName = subsystem.stateChannels.find(c => c.endsWith('/enabled'));
  const enabled = enabledName === undefined ? null : asBool(channels[enabledName]);
  const selected = subsystem.selection?.selected ?? 'none';
  const enumBadge = useMemo(() => {
    for (const name of subsystem.stateChannels) {
      const badge = badgeFor(channels[name], labels[name]);
      if (badge !== null)
        return {leaf: channelLeaf(name), label: badge};
    }
    return null;
  }, [subsystem, channels, labels]);

  return <div className="subsystem-tile">
    <b className="tile-name">{subsystem.name}</b>
    <span className={'tile-enabled' + (enabled === true ? ' on' :
                 enabled === false ? ' off' : '')}>
      {enabledName ? <ChannelReading channel={enabledName}>{enabled === null ? '—' : enabled ? 'enabled' : 'disabled'}</ChannelReading> : '—'}
    </span>
    <span className="tile-source">← {selected}</span>
    {enumBadge && <span className="tile-enum">{enumBadge.leaf}: {enumBadge.label}</span>}
  </div>;
}

function CameraTile({camera, channels}: {
  camera: Camera; channels: Record<string, number>;
}) {
  const healthyName = camera.statusChannels.find(c => c.endsWith('/healthy'));
  const periodName = camera.statusChannels.find(c => c.endsWith('/frame_period_us'));
  const tagsName = camera.statusChannels.find(c => c.endsWith('/visible_tag_count'));
  const healthy = healthyName === undefined ? null : asBool(channels[healthyName]);
  const period = periodName === undefined ? undefined : channels[periodName];
  const tags = tagsName === undefined ? undefined : channels[tagsName];
  const fps = period !== undefined && period > 0 ? 1e6 / period : null;

  return <div className="camera-tile">
    <b className="tile-name">{camera.name}</b>
    <span className={'tile-health' + (healthy === false ? ' bad' : healthy === true ? ' good' : '')}>
      {healthyName ? <ChannelReading channel={healthyName}>{healthy === null ? '—' : healthy ? 'healthy' : 'unhealthy'}</ChannelReading> : '—'}
    </span>
    <span className="tile-metric">{fps === null ? '—' : `${fps.toFixed(0)} fps`}</span>
    <span className="tile-metric">{tags === undefined ? '—' : `${tags} tags`}</span>
  </div>;
}

// --- panel ------------------------------------------------------------------

function RobotPanel() {
  const snapshot = useSnapshot();
  const dictionary = useDictionary();
  const system = useSystem();
  const status = useRobotStatus();
  const events = useEvents();
  const actions = useActions();

  const graph = system.graph;
  const channels = snapshot?.channels ?? {};
  const labels = snapshot?.labels ?? {};

  const subsystems = useMemo(
      () => discoverSubsystems(dictionary, graph), [dictionary, graph]);
  const cameras = useMemo(() => discoverCameras(dictionary), [dictionary]);
  const estimators = useMemo(() => discoverEstimators(dictionary), [dictionary]);
  const estimator = estimators[0] ?? null;

  // Routine: any enum channel matching <owner>/state/phase.
  const phaseChannel = useMemo(() => dictionary.find(
      c => /^[^/]+\/state\/phase$/.test(c.name) && c.labels.length > 0) ?? null,
      [dictionary]);

  // Gateway counters (any hw/status/gateway_*_count), discovered rather than
  // hard-coded so every counter the hardware reports is shown.
  const gatewayChannels = useMemo(() => dictionary
      .filter(c => /^hw\/status\/gateway_[^/]*_count$/.test(c.name))
      .map(c => c.name), [dictionary]);

  const estimatorAccepted = estimator === null ?
      null : estimator.channels.find(c => c.endsWith('/observation_accepted_count')) ?? null;
  const estimatorRejected = estimator === null ?
      null : estimator.channels.find(c => c.endsWith('/observation_rejected_count')) ?? null;
  const acceptanceChannels =
      [estimatorAccepted, estimatorRejected].filter((c): c is string => !!c);

  const batterySeries = useSeries(['hw/status/battery_v'], {seconds: 60});
  const gatewaySeries = useSeries(gatewayChannels, {seconds: 10});
  const acceptanceSeries = useSeries(acceptanceChannels, {seconds: 10});
  const phaseSeries = useSeries(
      phaseChannel ? [phaseChannel.name] : [], 'all');

  const phase = phaseChannel ?
      phaseFromSeries(phaseSeries, phaseChannel.name, phaseChannel.labels) :
      {current: null, previous: null, seconds: null};

  const gatewayDeltas = useMemo(() => gatewayChannels.map(name => ({
    name,
    label: channelLeaf(name).replace(/_count$/, '').replace(/_/g, ' '),
    ...windowDelta(gatewaySeries, name, 10)
  })), [gatewayChannels, gatewaySeries]);

  const accepted = estimatorAccepted ?
      windowDelta(acceptanceSeries, estimatorAccepted, 10) :
      {current: null, delta: null};
  const rejected = estimatorRejected ?
      windowDelta(acceptanceSeries, estimatorRejected, 10) :
      {current: null, delta: null};
  const acceptedTotal = accepted.delta !== null && rejected.delta !== null ?
      accepted.delta + rejected.delta : null;
  const acceptancePct = acceptedTotal !== null && acceptedTotal > 0 ?
      accepted.delta! / acceptedTotal * 100 : null;

  // Estimator pose (owner-agnostic): the discoverEstimators owner's pose and
  // sim ground truth, for the pose/heading error.
  const posePrefix = estimator ? `${estimator.name}/state/pose` : null;
  const poseX = posePrefix ? channels[`${posePrefix}/x_m`] : undefined;
  const poseY = posePrefix ? channels[`${posePrefix}/y_m`] : undefined;
  const poseYaw = posePrefix ? channels[`${posePrefix}/yaw_rad`] : undefined;
  const truth = simTruthPose(snapshot);
  const poseErrorValue =
      poseX !== undefined && poseY !== undefined && poseYaw !== undefined && truth ?
      poseError({x: poseX, y: poseY, yaw: poseYaw}, truth) :
      null;

  const recent = useMemo(() => events.slice(-8), [events]);

  return <div className="robot-panel">
    <FreshnessNotice/>
    <div className="robot-grid">
      <Card title="Driver Station" eyebrow="DS">
        {status.ds.mode === null ?
             <EmptyState>no Driver Station data</EmptyState> :
             <div className="ds-card">
               <ModeBadge mode={status.ds.mode}/>
               <StatList>
                 <Stat label="Enabled" channel="hw/state/driver_station/enabled" tone={status.ds.enabled ? 'ok' : 'warn'}>
                   {status.ds.enabled === null ? '—' : status.ds.enabled ? 'yes' : 'no'}
                 </Stat>
                 <Stat label="Match time" channel="hw/state/driver_station/match_time_s">{matchTime(status.ds.matchTime_s)}</Stat>
                 <Stat label="Alliance" channel="hw/state/driver_station/alliance">{status.ds.alliance ?? '—'}</Stat>
                 <Stat label="FMS" channel="hw/state/driver_station/fms_attached">{status.ds.fms === null ? '—' : status.ds.fms ? 'attached' : 'not attached'}</Stat>
               </StatList>
             </div>}
      </Card>

      <Card title="Routine" eyebrow="PHASE">
        {!phaseChannel ?
             <EmptyState>no phase channel discovered</EmptyState> :
             <div className="routine-card">
               <div className="phase-current"><ChannelReading channel={phaseChannel.name}>{phase.current ?? '—'}</ChannelReading></div>
               <StatList>
                 <Stat label="In phase" channel={phaseChannel.name}>{phase.seconds == null || !Number.isFinite(phase.seconds) ? '—' :
                     phase.seconds < 60 ? `${phase.seconds.toFixed(1)} s` :
                     phase.seconds < 3600 ? `${(phase.seconds / 60).toFixed(1)} min` :
                     `${(phase.seconds / 3600).toFixed(1)} h`}</Stat>
                 <Stat label="Previous">{phase.previous ?? '—'}</Stat>
               </StatList>
             </div>}
      </Card>

      <Card title="Power" eyebrow="HW/STATUS">
        {status.power.battery_v === null && status.power.can_frac === null ?
             <EmptyState>no power data</EmptyState> :
             <div className="power-card">
               <div className="battery-row">
                 <span className="battery-v">
                   <ChannelReading channel="hw/status/battery_v">{status.power.battery_v === null ? '—' :
                       `${number(status.power.battery_v, 2)} V`}</ChannelReading>
                 </span>
                 <Sparkline values={batterySeries.values['hw/status/battery_v'] ?? []}/>
               </div>
               <StatList>
                 <Stat label="Brownout" channel="hw/status/brownout" tone={status.power.brownout ? 'warn' : undefined}>
                   {status.power.brownout === null ? '—' : status.power.brownout ? 'yes' : 'no'}
                 </Stat>
                 <Stat label="CAN util" channel="hw/status/can_utilization_frac">{status.power.can_frac == null ? '—' :
                     `${number(status.power.can_frac * 100, 1)} %`}</Stat>
               </StatList>
             </div>}
      </Card>

      <Card title="Link" eyebrow="HW/STATUS">
        {status.link.stateAge_us === null && status.link.faultMask === null ?
             <EmptyState>no link data</EmptyState> :
             <StatList>
               <Stat label="State age" channel="hw/status/state_age_us" tone={
                   status.link.stateAge_us !== null && status.link.stateAge_us > 200000 ?
                       'warn' : undefined}>
                 {status.link.stateAge_us === null ? '—' :
                     duration(status.link.stateAge_us * 1e3)}
               </Stat>
               <Stat label="Configured" channel="hw/status/configured" tone={status.link.configured === false ? 'warn' : undefined}>
                 {status.link.configured === null ? '—' : status.link.configured ? 'yes' : 'no'}
               </Stat>
               <Stat label="Motor faults" channel="hw/status/motor_fault_mask" tone={status.link.faultMask ? 'warn' : undefined}>
                 {maskBits(status.link.faultMask) ?? '—'}
               </Stat>
               <Stat label="Timed out" channel="hw/status/timed_out_mask" tone={status.link.timedOutMask ? 'warn' : undefined}>
                 {maskBits(status.link.timedOutMask) ?? '—'}
               </Stat>
             </StatList>}
        <div className="gateway">
          {gatewayDeltas.map(g =>
            <div key={g.name} className="gateway-row">
              <span>{g.label}</span>
              <code>{g.delta === null ? '—' : g.delta > 0 ? `+${g.delta}` : g.delta}</code>
            </div>)}
          {gatewayDeltas.length === 0 && <span className="text-muted">no gateway counters</span>}
        </div>
      </Card>

      <Card title="Pose" eyebrow="ESTIMATOR">
        {estimator === null ?
             <EmptyState>no estimator discovered</EmptyState> :
             <div className="pose-card">
               <StatList>
                 <Stat label="X" channel={`${posePrefix}/x_m`}>{poseX === undefined ? '—' : number(poseX, 3) + ' m'}</Stat>
                 <Stat label="Y" channel={`${posePrefix}/y_m`}>{poseY === undefined ? '—' : number(poseY, 3) + ' m'}</Stat>
                 <Stat label="Heading" channel={`${posePrefix}/yaw_rad`}>{poseYaw === undefined || !Number.isFinite(poseYaw) ? '—' :
                     `${number(poseYaw * 180 / Math.PI, 1)}°`}</Stat>
                 <Stat label="Valid" channel={`${estimator.name}/state/valid`} tone={
                     status.estimator !== null && status.estimator.valid === false ? 'warn' : undefined}>
                   {status.estimator === null || status.estimator.valid === null ? '—' :
                       status.estimator.valid ? 'yes' : 'no'}
                 </Stat>
                 <Stat label="Accepted (10 s)" channel={estimatorAccepted ?? undefined} tone={rejected.delta !== null && rejected.delta > 0 ? 'warn' : undefined}>
                   {accepted.delta === null ? '—' : accepted.delta}
                 </Stat>
                 <Stat label="Rejected (10 s)" channel={estimatorRejected ?? undefined} tone={rejected.delta !== null && rejected.delta > 0 ? 'warn' : undefined}>
                   {rejected.delta === null ? '—' : rejected.delta}
                 </Stat>
                 <Stat label="Acceptance">{acceptancePct === null ? '—' : `${number(acceptancePct, 1)} %`}</Stat>
               </StatList>
               {truth && poseErrorValue ?
                    <div className="pose-error">
                      error {number(poseErrorValue.position_m, 3)} m ·{' '}
                      {number(poseErrorValue.heading_deg, 1)}°
                    </div> :
                    <div className="pose-error muted">no sim truth</div>}
             </div>}
      </Card>

      <Card title="Recent events" eyebrow={`${events.length} TOTAL`}>
        {recent.length ?
             <ul className="events">
               {recent.map((event: RobotEvent) =>
                 <li key={event.id} className={'event-' + event.severity}>
                   <button className="link" onClick={() => actions.seek(event.t_ns)}>
                     {event.title}
                   </button>
                   {event.detail && <span className="event-detail">{event.detail}</span>}
                 </li>)}
             </ul> :
             <EmptyState>no events yet</EmptyState>}
      </Card>
    </div>

    <section className="robot-strip">
      <div className="robot-card-head"><h2>Subsystems</h2><span>{subsystems.length} DRIVEN</span></div>
      {subsystems.length ?
           <div className="tiles">
             {subsystems.map(s => <SubsystemTile key={s.name} subsystem={s}
                 channels={channels} labels={labels}/>)}
           </div> :
           <EmptyState>no subsystems discovered</EmptyState>}
    </section>

    <section className="robot-strip">
      <div className="robot-card-head"><h2>Cameras</h2><span>{cameras.length} VISION</span></div>
      {cameras.length ?
           <div className="tiles">
             {cameras.map(c => <CameraTile key={c.name} camera={c} channels={channels}/>)}
           </div> :
           <EmptyState>no cameras discovered</EmptyState>}
    </section>
  </div>;
}

export const robotPanel: PanelDefinition<RobotConfig> = {
  id: 'robot',
  title: 'Robot',
  category: 'Robot',
  description: 'At a glance: Driver Station, routine, power, link, pose, subsystems, cameras and events.',
  defaultConfig,
  parseConfig,
  component: RobotPanel
};
