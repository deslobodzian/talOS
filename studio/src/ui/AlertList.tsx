// The alert lists (REQ-OPS-37): the alerts active now, by level, and the
// latch of every alert raised since the current match began. Presentational
// only: the data comes from data/alerts.ts, so the robot panel and the driver
// dashboard render the same thing. A level is told apart by its word and its
// glyph as well as its colour.

import type {ActiveAlert, AlertLatch, LatchedAlert} from '../data/alerts';
import {AlertLevel} from '../data/alerts';
import {duration, sessionTime} from './formatters';
import {EmptyState} from './primitives';
import './alerts.css';

const LEVEL: Record<number, {word: string; glyph: string; className: string}> = {
  [AlertLevel.Error]: {word: 'ERROR', glyph: '■', className: 'error'},
  [AlertLevel.Warning]: {word: 'WARN', glyph: '▲', className: 'warning'},
  [AlertLevel.Info]: {word: 'INFO', glyph: '●', className: 'info'}
};

function LevelTag({level}: {level: AlertLevel}) {
  const l = LEVEL[level];
  return <span className="alert-level"><span aria-hidden="true">{l.glyph}</span> {l.word}</span>;
}

function Name({alert}: {alert: ActiveAlert}) {
  return <span className="alert-name" title={alert.channel}>
    {alert.owner && <span className="alert-owner">{alert.owner}</span>}
    {alert.text}
  </span>;
}

// `now_ns` ages each active alert from its latest activation, when the latch
// knows it and it is not after `now_ns`.
export function ActiveAlertList({alerts, latch, now_ns}: {
  alerts: ActiveAlert[]; latch: AlertLatch|null; now_ns: string|null;
}) {
  if (!alerts.length) return <EmptyState>no active alerts</EmptyState>;
  const since = new Map(latch?.alerts.map(a => [a.channel, a.last_ns]) ?? []);
  return <ul className="alert-list">
    {alerts.map(alert => {
      const last = since.get(alert.channel);
      const age = last !== undefined && now_ns !== null && BigInt(last) <= BigInt(now_ns) ?
          duration(Number(BigInt(now_ns) - BigInt(last))) : null;
      return <li key={alert.channel} className={'alert ' + LEVEL[alert.level].className}>
        <LevelTag level={alert.level}/>
        <Name alert={alert}/>
        <span className="alert-meta">{age === null ? '' : `for ${age}`}</span>
      </li>;
    })}
  </ul>;
}

// When an alert was first raised: into the match when one has begun, else as
// a time before `now_ns`.
function raisedAt(alert: LatchedAlert, latch: AlertLatch, now_ns: string|null): string {
  if (latch.matchStart_ns !== null)
    return '+' + sessionTime(BigInt(alert.first_ns) - BigInt(latch.matchStart_ns));
  if (now_ns === null) return '';
  return sessionTime(BigInt(now_ns) - BigInt(alert.first_ns)) + ' ago';
}

export function LatchedAlertList({latch, now_ns}: {
  latch: AlertLatch|null; now_ns: string|null;
}) {
  if (latch === null)
    return <EmptyState>the alert latch needs the live stream</EmptyState>;
  if (!latch.alerts.length)
    return <EmptyState>{latch.matchStart_ns === null ? 'no alerts raised' :
                                                      'no alerts raised this match'}</EmptyState>;
  return <ul className="alert-list latched">
    {latch.alerts.map(alert =>
      <li key={alert.channel} className={'alert ' + LEVEL[alert.level].className +
                                         (alert.active ? ' active' : '')}>
        <LevelTag level={alert.level}/>
        <Name alert={alert}/>
        <span className="alert-meta">
          first {raisedAt(alert, latch, now_ns)} · ×{alert.count}{alert.active ? ' · active' : ''}
        </span>
      </li>)}
  </ul>;
}

// What the latch spans, for a heading.
export function latchSpan(latch: AlertLatch|null): string {
  if (latch === null || latch.matchStart_ns === null) return 'SINCE CONNECT';
  if (latch.matchKey === null) return 'THIS MATCH';
  return `MATCH ${latch.matchKey.slice(latch.matchKey.indexOf(':') + 1)}`;
}
