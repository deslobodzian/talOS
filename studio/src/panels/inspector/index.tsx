// The inspector: the old structured state tree, upgraded to a grouped,
// collapsible channel table. Rows are grouped by owner/role, each showing the
// value at the playback cursor (with the enum label when the channel declared
// labels), its unit, its age (since its newest reading, against the reading
// clock, marked STALE like every other panel), a 30 s sparkline for numeric
// channels, and an "add to plot" affordance that copies the channel name so it
// can be pasted into the Plot panel's channel drawer.

import {useMemo, useState} from 'react';

import {useDictionary, usePlayback, useSeries, useSnapshot} from '../../data/hooks';
import {useStudio} from '../../data/studio';
import type {ChannelInfo} from '../../data/types';
import {channelDisplayName} from '../../ui/channel';
import {FreshnessNotice} from '../../ui/FreshnessNotice';
import {duration, integer} from '../../ui/formatters';
import {EmptyState} from '../../ui/primitives';
import {Sparkline} from '../../ui/Sparkline';
import type {PanelDefinition} from '../types';
import {channelAge} from './staleness';
import './inspector.css';

export type InspectorConfig = Record<string, never>;

const defaultConfig: InspectorConfig = {};
const parseConfig = (_raw: unknown): InspectorConfig => ({});

type Kind = 'boolean' | 'enum' | 'numeric';

// Classify one channel for rendering. Enum channels declare labels; a boolean
// is a unitless, label-less channel whose value is exactly 0 or 1 (the wire's
// spelling); everything else is numeric. The same rule the subsystems panel
// uses.
function kindOf(info: ChannelInfo, value: number|undefined): Kind {
  if (info.labels.length > 0) return 'enum';
  if (!info.unit && (value === 0 || value === 1)) return 'boolean';
  return 'numeric';
}

// A numeric sample with precision that follows its magnitude: integers and
// counters print whole (with thousands separators), everything else prints
// just enough decimals with trailing zeros trimmed — never "1204.0000".
function formatValue(value: number): string {
  if (!Number.isFinite(value)) return '—';
  if (Number.isInteger(value)) return integer(value);
  const abs = Math.abs(value);
  const places = abs >= 100 ? 1 : abs >= 10 ? 2 : abs >= 1 ? 3 :
                 abs >= 0.01 ? 5 : 6;
  return value.toFixed(places).replace(/0+$/, '').replace(/\.$/, '');
}

// The collapsible group key: `owner/role`, `owner`, or '(root)' when the name
// has no slash.
function groupKey(name: string): string {
  const slash = name.indexOf('/');
  if (slash <= 0) return '(root)';
  const second = name.indexOf('/', slash + 1);
  return second === -1 ? name.slice(0, slash) : name.slice(0, second);
}

const EMPTY = new Float64Array(0);

function InspectorPanel() {
  const studio = useStudio();
  const snapshot = useSnapshot();
  const dictionary = useDictionary();
  const playback = usePlayback();
  const [filter, setFilter] = useState('');
  const [collapsed, setCollapsed] = useState<Set<string>>(new Set());
  const needle = filter.trim().toLowerCase();

  const allNames = useMemo(() => dictionary.map(c => c.name), [dictionary]);
  // A 30 s window for the sparklines; it also re-renders on every store change
  // so each row's age reads the current newest sample.
  const series30 = useSeries(allNames, {seconds: 30});

  const groups = useMemo(() => {
    const visible = needle ?
        dictionary.filter(
            c => c.name.toLowerCase().includes(needle) ||
                c.unit.toLowerCase().includes(needle)) :
        dictionary;
    const byKey = new Map<string, ChannelInfo[]>();
    for (const c of visible) {
      const key = groupKey(c.name);
      const list = byKey.get(key);
      if (list) list.push(c);
      else byKey.set(key, [c]);
    }
    return [...byKey.entries()]
        .sort((a, b) => {
          const ra = a[0] === '(root)', rb = b[0] === '(root)';
          if (ra !== rb) return ra ? 1 : -1;
          return a[0] < b[0] ? -1 : a[0] > b[0] ? 1 : 0;
        })
        .map(([key, channels]) => ({
          key,
          channels: channels.slice().sort((a, b) => a.name.localeCompare(b.name))
        }));
  }, [dictionary, needle]);

  const toggleGroup = (key: string) => {
    setCollapsed(prev => {
      const next = new Set(prev);
      if (next.has(key)) next.delete(key);
      else next.add(key);
      return next;
    });
  };

  const copyName = (name: string) => {
    void navigator.clipboard?.writeText(name);
  };

  const visibleCount = groups.reduce((n, g) => n + g.channels.length, 0);

  return <div className="inspector">
    <FreshnessNotice/>
    <div className="system-toolbar">
      <input aria-label="Search channels" className="search"
             placeholder="Search channels or units…" value={filter}
             onChange={e => setFilter(e.target.value)}/>
      <div className="system-counts">
        <span>{visibleCount} / {dictionary.length} channels</span>
      </div>
    </div>
    <div className="inspector-groups">
      {groups.map(g => {
        const open = !collapsed.has(g.key);
        return <section className="inspector-group" key={g.key}>
          <button className="group-header" aria-expanded={open}
                  onClick={() => toggleGroup(g.key)}>
            <span className="chevron">{open ? '▾' : '▸'}</span>
            <span className="group-name">{g.key}</span>
            <span className="group-count">{g.channels.length}</span>
          </button>
          {open && <table className="grid">
            <thead>
              <tr>
                <th>Channel</th>
                <th>Value</th>
                <th>Unit</th>
                <th>Age</th>
                <th className="right">30 s</th>
                <th className="right"></th>
              </tr>
            </thead>
            <tbody>
              {g.channels.map(c => {
                const value = snapshot?.channels[c.name];
                const kind = kindOf(c, value);
                const age = channelAge(studio.store, c.name, playback);
                const values = series30.values[c.name] ?? EMPTY;
                return <tr key={c.name}>
                  <td><code>{channelDisplayName(c.name)}</code></td>
                  <td>{kind === 'boolean' ?
                      <span className={'pill ' + (value === 1 ? 'on' : 'off')}>
                        {value === undefined ? '—' : value === 1 ? 'on' : 'off'}
                      </span> :
                      <b>{value === undefined ? '—' :
                          kind === 'enum' && Number.isInteger(value) &&
                                  value >= 0 && value < c.labels.length ?
                              c.labels[value] :
                              formatValue(value)}</b>}</td>
                  <td>{c.unit || <span className="text-muted">—</span>}</td>
                  <td>{age === null ?
                      <span className="text-muted">—</span> :
                      <span className={age.stale ? 'text-warn' : undefined}>
                        {age.age_ns === null ? 'unknown' :
                         age.age_ns === '0' ? <span className="age-fresh">now</span> :
                         duration(age.age_ns)}
                        {age.stale && <em> · STALE</em>}
                      </span>}</td>
                  <td className="right">
                    {c.labels.length === 0 &&
                        <Sparkline values={values} width={90} height={20}/>}
                  </td>
                  <td className="right">
                    <button className="add-plot" title="Add to plot"
                            aria-label={`Add ${c.name} to plot`}
                            onClick={() => copyName(c.name)}>+</button>
                  </td>
                </tr>;
              })}
            </tbody>
          </table>}
        </section>;
      })}
      {!groups.length && <EmptyState>
        {dictionary.length ? 'No channel matches that filter.' :
                             'No channels yet. Connect to a robot bridge, or start demo data.'}
      </EmptyState>}
    </div>
  </div>;
}

export const inspectorPanel: PanelDefinition<InspectorConfig> = {
  id: 'inspector',
  title: 'Inspector',
  category: 'Data',
  description: 'Every channel, its value, unit, staleness and recent shape.',
  defaultConfig,
  parseConfig,
  component: InspectorPanel
};
