// Dispatch timing: one row per source, with the declared period, the measured
// rate, and how late the loop was to service it. The old TimingTab reading its
// data through useSystem().

import {useMemo, useState} from 'react';

import {useSystem} from '../../data/hooks';
import {rateKey, type SystemSource} from '../../system';
import {duration, hertz, integer} from '../../ui/formatters';
import {EmptyState} from '../../ui/primitives';
import type {PanelDefinition} from '../types';
import './timing.css';

export type TimingConfig = Record<string, never>;

const defaultConfig: TimingConfig = {};
const parseConfig = (_raw: unknown): TimingConfig => ({});

export type TimingColumn =
    'node'|'kind'|'name'|'period'|'rate'|'events'|'latency'|'max'|'dropped';

const periodRate = (source: SystemSource) => {
  const period = Number(source.period_ns);
  return period > 0 ? 1e9 / period : undefined;
};

function TimingPanel() {
  const system = useSystem();
  const [sort, setSort] = useState<{column: TimingColumn; descending: boolean}>(
      {column: 'max', descending: true});
  const [filter, setFilter] = useState('');
  const graph = system.graph;

  const rows = useMemo(() => {
    if (!graph) return [];
    const needle = filter.trim().toLowerCase();
    const flat = graph.nodes.flatMap(
        node => node.sources
                    .filter(
                        source => !needle ||
                            source.name.toLowerCase().includes(needle) ||
                            node.name.toLowerCase().includes(needle))
                    .map(source => ({
                           node,
                           source,
                           rate: system.rates[rateKey(node.name, source.id)],
                           declared: periodRate(source)
                         })));
    const key = (row: typeof flat[number]): number|string => {
      switch (sort.column) {
        case 'node':
          return row.node.name;
        case 'kind':
          return row.source.kind;
        case 'name':
          return row.source.name;
        case 'period':
          return Number(row.source.period_ns);
        case 'rate':
          return row.rate ?? -1;
        case 'events':
          return Number(row.source.events);
        case 'latency':
          return row.source.kind === 'TIMER' || row.source.kind === 'WATCHER' ?
              Number(row.source.last_latency_ns) : -1;
        case 'max':
          return row.source.kind === 'TIMER' || row.source.kind === 'WATCHER' ?
              Number(row.source.max_latency_ns) : -1;
        case 'dropped':
          return Number(row.source.dropped);
      }
    };
    return flat.sort((a, b) => {
      const left = key(a), right = key(b);
      const order = typeof left === 'string' ?
          String(left).localeCompare(String(right)) :
          Number(left) - Number(right);
      return sort.descending ? -order : order;
    });
  }, [graph, system.rates, filter, sort]);

  if (!graph) return <div className="system"><EmptyState>
    {system.error ? system.error :
                    'No system graph yet. Connect to a robot bridge, or start demo data to explore the viewer.'}
  </EmptyState></div>;

  const header = (column: TimingColumn, label: string, right = false) =>
      <th className={(right ? 'right ' : '') + 'sortable' +
                     (sort.column === column ? ' sorted' : '')}
          onClick={() => setSort(
              current => ({
                column,
                descending: current.column === column ? !current.descending :
                                                        true
              }))}>
        {label}
        {sort.column === column && <i>{sort.descending ? '▾' : '▴'}</i>}
      </th>;

  return <section className="panel timing">
    <div className="panel-heading">
      <h2>Dispatch timing</h2>
      <span>{rows.length} SOURCES</span>
    </div>
    <div className="system-toolbar">
      <input aria-label="Filter sources" className="search"
             placeholder="Filter by node or topic…" value={filter}
             onChange={e => setFilter(e.target.value)}/>
      <div className="system-counts">
        <span>
          Latency is how late the loop was to service an event, not transport
          age: RTMS carries no publish timestamp.
        </span>
      </div>
    </div>
    <div className="table-scroll">
      <table className="grid">
        <thead>
          <tr>
            {header('node', 'Node')}
            {header('kind', 'Kind')}
            {header('name', 'Source')}
            {header('period', 'Declared', true)}
            {header('rate', 'Measured', true)}
            {header('events', 'Events', true)}
            {header('latency', 'Last late', true)}
            {header('max', 'Worst late', true)}
            {header('dropped', 'Dropped', true)}
          </tr>
        </thead>
        <tbody>
          {rows.map(({node, source, rate, declared}) =>
                        <tr key={node.name + '/' + source.kind + source.id}
                            className={node.alive ? '' : 'dead'}>
                          <td>{node.name}</td>
                          <td><em className={'kind ' + source.kind}>
                            {source.kind}
                          </em></td>
                          <td><code>{source.name}</code></td>
                          <td className="right">{hertz(declared)}</td>
                          <td className="right">{hertz(rate)}</td>
                          <td className="right">{integer(source.events)}</td>
                          <td className="right">
                            {source.kind === 'TIMER' || source.kind === 'WATCHER' ?
                                 duration(source.last_latency_ns) : '—'}
                          </td>
                          <td className="right">
                            {source.kind === 'TIMER' || source.kind === 'WATCHER' ?
                                 duration(source.max_latency_ns) : '—'}
                          </td>
                          <td className={'right' +
                                         (source.dropped === '0' ? '' : ' text-warn')}>
                            {integer(source.dropped)}
                          </td>
                        </tr>)}
        </tbody>
      </table>
      {!rows.length && <EmptyState>No source matches that filter.</EmptyState>}
    </div>
  </section>;
}

export const timingPanel: PanelDefinition<TimingConfig> = {
  id: 'timing',
  title: 'Timing',
  category: 'System',
  description: 'Dispatch timing and latency per source.',
  defaultConfig,
  parseConfig,
  component: TimingPanel
};
