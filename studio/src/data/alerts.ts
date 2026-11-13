// Alerts (REQ-OPS-37), as a pure model with no DOM, shared by Studio and the
// driver dashboard. An alert channel is a status `bool` the dictionary marks
// with a level (Snapshot.alerts, from ChannelInfo.alert); it is active while
// it has a reading and the reading is true. Nodes report conditions and never
// latch; the viewer latches here.
//
// Two views:
//   - activeAlerts(sample): the alerts active in one sample, ordered by level
//     (error, warning, info) then channel name. A channel with no reading (a
//     stale topic) is absent: neither active nor clear.
//   - AlertTracker: fed every sample in time order, it keeps the latch, every
//     alert raised since the current match began, kept until the next match
//     begins, with its first and latest activation and activation count.
//
// When a match begins (the operator-visibility design latches "until the next
// match number"; where it is silent, this module decides):
//   - Keyed: while hw/state/driver_station/match_number is nonzero, a match is
//     keyed by (match_type, match_number). A sample whose key differs from the
//     current match's begins a match, enabled or not: the FMS assigns the next
//     match while the robot sits disabled before it. A spell with no key or no
//     Driver Station reading in between does not: the same key continues.
//   - Unkeyed (match_number zero or absent: no FMS, Driver Station practice):
//     the first enabled sample after a sample that was not enabled begins a
//     match, except the teleop enable that follows an autonomous spell, which
//     continues that match (auto, the disabled gap, then teleop is one match).
//   - A sample without a reading of hw/state/driver_station/enabled (no Driver
//     Station link) changes nothing about the match.
//   - Before the first match begins the latch holds everything since the
//     tracker started (matchStart_ns null).
//   - An alert already active when a match begins counts as raised then.
//   - A sample without a reading of an alert channel keeps its last reading
//     for edge detection: a stale spell is neither a clear nor a new raise.
//
// Guarantee. push() is synchronous and does not debounce: once it returns,
// every alert channel the sample carries as true is in active() and in
// latch(), and each false-to-true change is one activation. So an activation
// is shown exactly when some pushed sample carries it, at the push of the
// first such sample. If consecutive pushed samples are at most P apart, an
// activation lasting at least P (half-open, as the producer samples it) is
// carried by at least one of them, and is shown less than P after it began
// plus transport and render. REQ-OPS-37's 100 ms activation therefore needs
// samples no further than 100 ms apart (10 Hz or faster; telemetry reads
// status at 100 Hz and the drive feed updates at 20 Hz), and its 1 s bound
// is then the feed's latency plus under 100 ms. A sample lost before push()
// (a frame the bridge coalesced or the worker dropped) cannot be latched.

import {AlertLevel} from '../generated/talos/telemetry/alert-level';
import type {Snapshot} from '../core';

export {AlertLevel};

// What the model reads from a sample: its time, readings and alert levels.
export type AlertSample = Pick<Snapshot, 'timestamp_ns'|'channels'|'alerts'>;

export type ActiveAlert = {
  channel: string;   // e.g. hw/status/alert_error_link_lost
  level: AlertLevel; // Info, Warning or Error
  owner: string;     // the channel's first segment
  text: string;      // the humanised name, e.g. "link lost"
};

export type LatchedAlert = ActiveAlert & {
  first_ns: string;  // the first activation in this match (or at its start)
  last_ns: string;   // the latest activation's start
  count: number;     // activations in this match
  active: boolean;   // active in the newest sample pushed
};

export type AlertLatch = {
  // The sample that began the current match; null before the first one.
  matchStart_ns: string|null;
  // `${match_type}:${match_number}` of a keyed match, else null.
  matchKey: string|null;
  // Ordered by level (error first) then channel name.
  alerts: LatchedAlert[];
};

const DS = 'hw/state/driver_station/';

const LEVEL_NAME: Record<number, string> = {
  [AlertLevel.Info]: 'info',
  [AlertLevel.Warning]: 'warning',
  [AlertLevel.Error]: 'error'
};

// The level's word, for display and for stripping the field prefix.
export function alertLevelName(level: AlertLevel): string {
  return LEVEL_NAME[level] ?? 'none';
}

// The alert's display name: the field after `alert_<level>_`, with spaces for
// underscores. The level comes from the dictionary, so this strips a known
// prefix and never classifies a name.
export function alertText(channel: string, level: AlertLevel): string {
  const leaf = channel.slice(channel.lastIndexOf('/') + 1);
  const prefix = `alert_${alertLevelName(level)}_`;
  return (leaf.startsWith(prefix) ? leaf.slice(prefix.length) : leaf)
      .replace(/_/g, ' ');
}

// Error before warning before info, then by channel name.
export function compareAlerts(a: {level: AlertLevel; channel: string},
                              b: {level: AlertLevel; channel: string}): number {
  if (a.level !== b.level) return b.level - a.level;
  return a.channel < b.channel ? -1 : a.channel > b.channel ? 1 : 0;
}

const describe = (channel: string, level: AlertLevel): ActiveAlert => ({
  channel,
  level,
  owner: channel.slice(0, Math.max(0, channel.indexOf('/'))),
  text: alertText(channel, level)
});

const isAlertLevel = (level: AlertLevel) =>
    level === AlertLevel.Info || level === AlertLevel.Warning ||
    level === AlertLevel.Error;

// The alerts active in one sample, ordered by compareAlerts.
export function activeAlerts(sample: AlertSample|null): ActiveAlert[] {
  if (!sample?.alerts) return [];
  const out: ActiveAlert[] = [];
  for (const [channel, level] of Object.entries(sample.alerts)) {
    if (!isAlertLevel(level)) continue;
    const value = sample.channels[channel];
    if (value !== undefined && value !== 0 && !Number.isNaN(value))
      out.push(describe(channel, level));
  }
  return out.sort(compareAlerts);
}

// The match key of a sample, or null when it is not in an FMS-numbered match.
function matchKeyOf(sample: AlertSample): string|null {
  const number = sample.channels[DS + 'match_number'];
  if (number === undefined || !Number.isFinite(number) || number === 0)
    return null;
  return `${sample.channels[DS + 'match_type'] ?? 0}:${number}`;
}

const flag = (sample: AlertSample, name: string) =>
    (sample.channels[DS + name] ?? 0) !== 0;

export class AlertTracker {
  // Bumped whenever latch() or active() would return something different.
  version = 0;

  private matchStart: string|null = null;
  private matchKey: string|null = null;
  private wasEnabled = false;
  // The mode of the latest enabled spell of an unkeyed match.
  private spell: 'autonomous'|'other'|null = null;
  private readonly last = new Map<string, boolean>();
  private readonly entries = new Map<string, LatchedAlert>();
  private current: ActiveAlert[] = [];
  private latchView: AlertLatch|null = null;

  reset() {
    this.matchStart = null;
    this.matchKey = null;
    this.wasEnabled = false;
    this.spell = null;
    this.last.clear();
    this.entries.clear();
    this.current = [];
    this.latchView = null;
    this.version++;
  }

  // Feeds one sample, in time order. Returns whether the views changed.
  push(sample: AlertSample): boolean {
    const t = sample.timestamp_ns;
    let changed = false;

    // A sample without a Driver Station reading says nothing about the match.
    let begins = false;
    if (sample.channels[DS + 'enabled'] !== undefined) {
      const key = matchKeyOf(sample);
      const enabled = flag(sample, 'enabled');
      if (key !== null) {
        begins = key !== this.matchKey;
        this.spell = null;
      } else if (enabled && !this.wasEnabled) {
        begins = !(flag(sample, 'teleop') && this.spell === 'autonomous' &&
                   this.matchKey === null);
      }
      if (enabled && key === null)
        this.spell = flag(sample, 'autonomous') ? 'autonomous' : 'other';
      this.wasEnabled = enabled;
      if (begins) this.matchKey = key;
    }

    if (begins) {
      this.matchStart = t;
      if (this.entries.size) this.entries.clear();
      changed = true;
    }

    for (const [channel, level] of Object.entries(sample.alerts ?? {})) {
      if (!isAlertLevel(level)) continue;
      const value = sample.channels[channel];
      const present = value !== undefined && !Number.isNaN(value);
      const active = present && value !== 0;
      const before = this.last.get(channel) ?? false;
      if (present) this.last.set(channel, active);
      const entry = this.entries.get(channel);
      if (active && (!before || !entry)) {
        if (entry) {
          entry.count++;
          entry.last_ns = t;
          entry.active = true;
        } else {
          this.entries.set(channel, {...describe(channel, level), first_ns: t,
                                     last_ns: t, count: 1, active: true});
        }
        changed = true;
      } else if (entry && entry.active !== active) {
        entry.active = active;
        changed = true;
      }
    }

    const current = activeAlerts(sample);
    if (current.length !== this.current.length ||
        current.some((a, i) => a.channel !== this.current[i].channel)) {
      this.current = current;
      changed = true;
    }
    if (changed) {
      this.latchView = null;
      this.version++;
    }
    return changed;
  }

  // The alerts active in the newest sample pushed (activeAlerts of it).
  active(): ActiveAlert[] {
    return this.current;
  }

  // The latch; a stable reference until the next change.
  latch(): AlertLatch {
    return this.latchView ??= {
      matchStart_ns: this.matchStart,
      matchKey: this.matchKey,
      alerts: [...this.entries.values()].map(e => ({...e})).sort(compareAlerts)
    };
  }
}
