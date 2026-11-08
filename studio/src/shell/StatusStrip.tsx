// The always-visible robot status strip. Every reading comes from
// useRobotStatus(); every field whose source is not in the data shows an
// explicit "unknown" style rather than a plausible zero, because "not known
// yet" is not "nothing".

import type {ReactNode} from 'react';

import {useChannelStale, usePlayback, useRobotStatus} from '../data/hooks';
import {FreshnessNotice} from '../ui/FreshnessNotice';
import type {DriverStationMode} from '../data/types';
import {duration, number} from '../ui/formatters';

const DS_LABEL: Record<DriverStationMode, string> = {
  disconnected: 'DISCONNECTED',
  disabled: 'DISABLED',
  autonomous: 'AUTONOMOUS',
  teleop: 'TELEOP',
  test: 'TEST',
  estop: 'E-STOP'
};

function Item({label, value, tone, unknown, channel}: {
  label: string; value: ReactNode; tone?: string; unknown?: boolean; channel?: string;
}) {
  const stale = useChannelStale(channel ?? '');
  return <span className={'status-item' + (tone ? ' ' + tone : '') +
                         (unknown ? ' unknown' : '')}>
    <small>{label}</small>
    <b>{unknown ? '—' : value}{channel && stale && <em className="text-warn"> · STALE</em>}</b>
  </span>;
}

function matchTime(seconds: number|null): string {
  if (seconds == null) return '—';
  const m = Math.floor(seconds / 60);
  const s = Math.floor(seconds % 60);
  return `${m}:${String(s).padStart(2, '0')}`;
}

export function StatusStrip() {
  const status = useRobotStatus();
  const playback = usePlayback();
  const dsStale = useChannelStale('hw/state/driver_station/enabled');
  const {ds, power, link, estimator, nodes} = status;

  return <div className="status-strip" role="status" aria-label="Robot status">
    <FreshnessNotice/>
    {playback.connection === 'error' && <span className="text-warn" role="alert">{playback.status} · Reconnect to restart</span>}
    {playback.updates && <Item label="UI updates" value={`${playback.updates.dropped} dropped · ${playback.updates.coalesced} coalesced`}/>}
    <span className={'status-item mode' + (ds.mode ? ' ' + ds.mode : ' unknown')}>
      <small>DS</small>
      <b>{ds.mode ? DS_LABEL[ds.mode] : '—'}{dsStale && <em className="text-warn"> · STALE</em>}</b>
    </span>
    <Item label="Enabled" channel="hw/state/driver_station/enabled" tone={ds.enabled === null ? undefined : ds.enabled ? 'ok' : 'warn'} unknown={ds.enabled === null}
          value={ds.enabled === null ? '—' : ds.enabled ? 'yes' : 'no'}/>
    <Item label="Match" channel="hw/state/driver_station/match_time_s" unknown={ds.matchTime_s === null} value={matchTime(ds.matchTime_s)}/>
    <Item label="Alliance" channel="hw/state/driver_station/alliance" unknown={ds.alliance === null} value={ds.alliance ?? '—'}/>
    <Item label="Battery" channel="hw/status/battery_v" tone={
      power.battery_v !== null && power.battery_v < 11.5 ? 'warn' : undefined}
      unknown={power.battery_v === null}
      value={power.battery_v === null ? '—' : number(power.battery_v, 1) + ' V'}/>
    <Item label="Brownout" channel="hw/status/brownout" tone={power.brownout ? 'warn' : undefined}
          unknown={power.brownout === null}
          value={power.brownout === null ? '—' : power.brownout ? 'yes' : 'no'}/>
    <Item label="Link age" channel="hw/status/state_age_us" tone={
      link.stateAge_us !== null && link.stateAge_us > 200000 ? 'warn' : undefined}
      unknown={link.stateAge_us === null}
      value={link.stateAge_us === null ? '—' : duration(link.stateAge_us * 1e3)}/>
    <Item label="Nodes" unknown={nodes === null}
          tone={nodes !== null && nodes.alive === null ? 'warn' : undefined}
          value={nodes === null ? '—' : nodes.alive === null ?
              <>?/{nodes.total}<em className="text-warn"> · STALE</em></> :
              `${nodes.alive}/${nodes.total}`}/>
    <Item label="Estimator"
          tone={estimator !== null && !estimator.valid ? 'warn' : undefined}
          unknown={estimator === null}
          value={estimator === null ? '—' :
                 estimator.valid === null ? '—' :
                 estimator.valid ? 'valid' : 'invalid'}/>
  </div>;
}
