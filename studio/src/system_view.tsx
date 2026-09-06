// The views built purely out of the system graph: what is running, how it is
// wired, how it is performing, and what an agent sees.
//
// Split from main.tsx because these are the pieces worth testing. They touch
// no DOM API, no canvas and no worker, so react-dom/server can render them in
// a test and assert what a person would actually read on screen -- which
// typechecking and a successful bundle do not establish. Field, Pose and
// Signals stay in the shell: SVG measurement, WebGL and uPlot all need a real
// browser.

import React, {useMemo, useState} from 'react';

import {RPC_METHODS} from '../agent/rpc';

import type {Snapshot} from './core';
import type {TabId} from './tabs';
import {
  type DeclaredGraph,
  declaredStatus,
  endpointAttributesKnown,
  isSubscriber,
  rateKey,
  type SourceKind,
  type SystemGraph,
  type SystemNode,
  type SystemSource,
  type SystemTopic,
  TOPIC_HEALTH,
  topicHealthOf,
  type TopicHealth,
} from './system';

export type View = {
  snapshot: Snapshot|null; history: Snapshot[]; mode: string;
  bounds: {start: string; end: string}|null;
  stats: {dropped: number; late: number; invalid: number};
  status: string;
};

// What the worker knows about the shape of the system, as opposed to what it
// measured. Held separately from `View` because it arrives at a few hertz
// while frames arrive at a hundred, and re-sending it with every frame would
// be a structured clone of the whole graph thirty times a second.
export type SystemState = {
  graph: SystemGraph|null;
  rates: Record<string, number>;
  error: string;

  // What the launcher declared before it spawned anything, when the bridge was
  // given it. Null is the ordinary case rather than an error: `--declared` is
  // optional, and the desktop UDP path has no HTTP endpoint to ask. Every view
  // here reads null as "not offered" and shows nothing at all, because an empty
  // panel and a count of zero would both be claims nobody made.
  declared: DeclaredGraph|null;
  declaredError: string;
};

// --- formatting -------------------------------------------------------------

export const integer = (value: string|number) => {
  const n = typeof value === 'string' ? BigInt(value) : BigInt(Math.round(value));
  return n.toLocaleString('en-US');
};

// Robot timing lives in microseconds and milliseconds; a bare nanosecond count
// is unreadable at a glance and that is the only glance a viewer gets.
export function duration(nanos: string|number): string {
  const ns = typeof nanos === 'string' ? Number(nanos) : nanos;
  if (!Number.isFinite(ns)) return '—';
  const magnitude = Math.abs(ns);
  if (magnitude === 0) return '0';
  if (magnitude < 1e3) return `${ns.toFixed(0)} ns`;
  if (magnitude < 1e6) return `${(ns / 1e3).toFixed(1)} µs`;
  if (magnitude < 1e9) return `${(ns / 1e6).toFixed(2)} ms`;
  return `${(ns / 1e9).toFixed(2)} s`;
}

export const hertz = (rate: number|undefined) =>
    rate === undefined ? '—' : rate >= 100 ? `${rate.toFixed(0)} Hz` :
    rate >= 1         ? `${rate.toFixed(1)} Hz` :
                        `${rate.toFixed(2)} Hz`;

// Node age is computed against the graph's own wall clock rather than the
// browser's: the two machines need not agree, and a viewer that says a healthy
// node is ten seconds stale because of clock skew is worse than no viewer.
export const staleness = (graph: SystemGraph, node: SystemNode) =>
    Number(BigInt(graph.wall_ns) - BigInt(node.heartbeat_wall_ns));

export const periodRate = (source: SystemSource) => {
  const period = Number(source.period_ns);
  return period > 0 ? 1e9 / period : undefined;
};

// --- system graph -----------------------------------------------------------

const KIND_LABEL: Record<SourceKind, string> = {
  SENDER: 'publishes',
  WATCHER: 'watches',
  FETCHER: 'fetches',
  TIMER: 'timer'
};

const HEALTH_NOTE: Record<TopicHealth, string> = {
  orphaned: 'subscribed but nothing publishes it',
  unread: 'published but nobody reads it',
  lossy: 'messages were lapped before a reader saw them',
  idle: 'wired up, nothing published yet',
  ok: 'publishing and being read',
  bridged: 'one end is outside talOS; no shared-memory peer will appear',
  unconnected: 'declared optional; no peer yet, and that is the design'
};

export function Empty({children}: {children: React.ReactNode}) {
  return <div className="empty">{children}</div>;
}

export function NoSystem({system}: {system: SystemState}) {
  return <Empty>
    {system.error ? system.error :
                    'No system graph yet. Connect to a robot bridge, or start demo data to explore the viewer.'}
  </Empty>;
}

export function NodeCard(
    {graph, node, rates, selected, onSelect}: {
      graph: SystemGraph; node: SystemNode; rates: Record<string, number>;
      selected: boolean;
      onSelect: () => void;
    }) {
  const publishes = node.sources.filter(s => s.kind === 'SENDER');
  const subscribes = node.sources.filter(s => isSubscriber(s.kind));
  const timers = node.sources.filter(s => s.kind === 'TIMER');

  // The node's own cadence. A loop node's timers are the honest answer; the
  // hardware node has none -- it drives its own tick -- so fall back to the
  // fastest thing it publishes rather than reporting no rate for the one
  // process whose rate matters most.
  const busiest = (timers.length ? timers : node.sources)
                      .map(s => rates[rateKey(node.name, s.id)])
                      .filter((r): r is number => r !== undefined)
                      .sort((a, b) => b - a)[0];
  return <button
      className={'node-card' + (selected ? ' selected' : '') +
                 (node.alive ? '' : ' dead')}
      onClick={onSelect} aria-pressed={selected}>
    <div className="node-card-head">
      <span className={'dot ' + (node.alive ? 'active' : '')}/>
      <b>{node.name}</b>
      {node.simulation && <em className="badge sim">SIM</em>}
      {node.replay && <em className="badge">REPLAY</em>}
      {!node.alive && <em className="badge dead">STALE</em>}
    </div>
    <code>{node.target || '—'}</code>
    <dl>
      <div><dt>pid</dt><dd>{node.pid}</dd></div>
      <div><dt>pub / sub</dt><dd>{publishes.length} / {subscribes.length}</dd></div>
      <div><dt>dispatches</dt><dd>{integer(node.dispatch_count)}</dd></div>
      <div><dt>loop</dt><dd>{hertz(busiest)}</dd></div>
      <div><dt>seen</dt><dd>{
          staleness(graph, node) <= 0 ? 'now' :
                                        duration(staleness(graph, node)) + ' ago'
      }</dd></div>
    </dl>
    {node.declared_sources > node.sources.length &&
     <small className="warn">
       {node.declared_sources - node.sources.length} more sources than the
       registry slot holds
     </small>}
  </button>;
}

// One node's wiring: what flows in, what flows out, and which node is on the
// far end of each. The question this answers is the one a topic table cannot:
// not "what topics exist" but "who is this process actually talking to".
export function Wiring(
    {graph, node, rates}:
        {graph: SystemGraph; node: SystemNode; rates: Record<string, number>}) {
  const peersOf = (topic: string, want: 'publishers'|'subscribers') =>
      graph.topics.find(t => t.name === topic)?.[want].filter(
          n => n !== node.name) ??
      [];

  const incoming = node.sources.filter(s => isSubscriber(s.kind));
  const outgoing = node.sources.filter(s => s.kind === 'SENDER');

  // A missing peer is only dangling when nobody said it would be missing. The
  // node holding this end declared whether its far end lives outside talOS or
  // is allowed not to exist yet, and drawing either as a broken wire would
  // contradict what the topic table says about the same topic.
  const edge = (source: SystemSource, direction: 'in'|'out') => {
    const peers = peersOf(source.name, direction === 'in' ? 'publishers' :
                                                            'subscribers');
    const rate = rates[rateKey(node.name, source.id)];
    const declared = source.external || source.optional;
    return <li key={source.kind + source.id}
               className={peers.length || declared ? '' : 'dangling'}>
      <div className="edge-topic">
        <code>{source.name}</code>
        <span className="edge-rate">{hertz(rate)}</span>
      </div>
      <div className="edge-meta">
        <em>{KIND_LABEL[source.kind]}</em>
        <span>{peers.length      ? peers.join(', ') :
                   source.external ? 'outside talOS — no shared-memory peer' :
                   source.optional ? 'optional — no peer yet' :
                   direction === 'in' ? 'no publisher' :
                                        'no subscriber'}</span>
        {source.dropped !== '0' &&
         <span className="warn">{integer(source.dropped)} dropped</span>}
      </div>
    </li>;
  };

  return <div className="wiring">
    <section>
      <h4>Subscribes <small>{incoming.length}</small></h4>
      {incoming.length ? <ul>{incoming.map(s => edge(s, 'in'))}</ul> :
                         <Empty>Reads nothing.</Empty>}
    </section>
    <div className="wiring-node">
      <span className={'dot ' + (node.alive ? 'active' : '')}/>
      <b>{node.name}</b>
      <small>{node.sources.filter(s => s.kind === 'TIMER').length} timers</small>
    </div>
    <section>
      <h4>Publishes <small>{outgoing.length}</small></h4>
      {outgoing.length ? <ul>{outgoing.map(s => edge(s, 'out'))}</ul> :
                         <Empty>Publishes nothing.</Empty>}
    </section>
  </div>;
}

export function TopicTable(
    {graph, rates, filter, onPickNode}: {
      graph: SystemGraph; rates: Record<string, number>; filter: string;
      onPickNode: (name: string) => void;
    }) {
  const [chosen, setChosen] = useState<TopicHealth|'all'>('all');

  // Endpoint attributes come from the ends, not from the topic row, so the
  // classifier gathers them once per graph rather than per row.
  const health = useMemo(() => topicHealthOf(graph), [graph]);
  const rows = useMemo(() => {
    const needle = filter.trim().toLowerCase();
    return graph.topics
        .map(topic => ({topic, health: health(topic)}))
        .filter(
            row => (chosen === 'all' || row.health === chosen) &&
                (!needle || row.topic.name.toLowerCase().includes(needle) ||
                 row.topic.publishers.some(
                     p => p.toLowerCase().includes(needle)) ||
                 row.topic.subscribers.some(
                     s => s.toLowerCase().includes(needle))));
  }, [graph.topics, health, filter, chosen]);

  const counts = useMemo(() => {
    const tally = new Map<TopicHealth, number>();
    for (const topic of graph.topics) {
      const state = health(topic);
      tally.set(state, (tally.get(state) ?? 0) + 1);
    }
    return tally;
  }, [graph.topics, health]);

  // Publisher-side rate: a topic moves at the rate of whoever writes it, and
  // summing both ends would double-count every healthy topic.
  const topicRate = (topic: SystemTopic) => {
    let total = 0;
    let measured = false;
    for (const node of graph.nodes) {
      for (const source of node.sources) {
        if (source.name !== topic.name || source.kind !== 'SENDER') continue;
        const rate = rates[rateKey(node.name, source.id)];
        if (rate !== undefined) {
          total += rate;
          measured = true;
        }
      }
    }
    return measured ? total : undefined;
  };

  return <>
    <div className="chips">
      {(['all', ...TOPIC_HEALTH] as const)
           .map(state => <button
                    key={state}
                    className={'chip' + (chosen === state ? ' selected' : '') +
                               (state === 'all' ? '' : ' ' + state)}
                    onClick={() => setChosen(state)}>
                 {state}
                 <em>{state === 'all' ? graph.topics.length :
                                        counts.get(state) ?? 0}</em>
               </button>)}
    </div>
    <div className="table-scroll">
      <table className="grid">
        <thead>
          <tr>
            <th>Topic</th>
            <th>Bytes</th>
            <th>Publishers</th>
            <th>Subscribers</th>
            <th className="right">Rate</th>
            <th className="right">Published</th>
            <th className="right">Received</th>
            <th className="right">Dropped</th>
          </tr>
        </thead>
        <tbody>
          {rows.map(({topic, health: state}) => <tr key={topic.name}
                                                    className={state}>
             <td>
               <code>{topic.name}</code>
               <span className={'health ' + state} title={HEALTH_NOTE[state]}>
                 {state}
               </span>
             </td>
             <td>{topic.message_bytes}</td>
             <td>{topic.publishers.length ?
                      topic.publishers.map(
                          name => <button key={name} className="link"
                                          onClick={() => onPickNode(name)}>
                            {name}
                          </button>) :
                      <span className="warn">none</span>}</td>
             <td>{topic.subscribers.length ?
                      topic.subscribers.map(
                          name => <button key={name} className="link"
                                          onClick={() => onPickNode(name)}>
                            {name}
                          </button>) :
                      <span className="warn">none</span>}</td>
             <td className="right">{hertz(topicRate(topic))}</td>
             <td className="right">{integer(topic.published)}</td>
             <td className="right">{integer(topic.received)}</td>
             <td className={'right' + (topic.dropped === '0' ? '' : ' warn')}>
               {integer(topic.dropped)}
             </td>
           </tr>)}
        </tbody>
      </table>
      {!rows.length && <Empty>No topic matches that filter.</Empty>}
    </div>
  </>;
}

// Declared against actual.
//
// The live registry holds what did start, so a name's absence from it says
// nothing on its own: a node that crashed on the way up and a node that is not
// part of this session look the same from there. This panel is the difference:
// which node was supposed to be here, and then what the launcher itself said
// about the graph -- the process that decides whether a graph can start
// explains a broken one better than a viewer re-deriving the same conclusion
// from the wreckage.
export function DeclaredPanel(
    {declared, graph}: {declared: DeclaredGraph; graph: SystemGraph|null}) {
  const status = declaredStatus(declared, graph);
  return <section className="panel declared">
    <div className="panel-heading">
      <h2>Declared graph</h2>
      <span>PROBED WITH --describe BEFORE ANYTHING SPAWNED</span>
    </div>
    <div className="system-counts declared-counts">
      <span>{declared.nodes.length} nodes declared</span>
      <span>{status ? `${status.registered} registered` :
                      'no registry yet — nothing to compare against'}</span>
      {!!status?.missing.length &&
       <span className="warn">{status.missing.length} never appeared</span>}
      {!!status?.undeclared.length &&
       <span>{status.undeclared.length} not declared</span>}
    </div>
    <div className="declared-body">
      {!!status?.missing.length && <ul className="declared-nodes">
        {status.missing.map(node => <li key={node.name} className="warn">
          <b>{node.name}</b>
          <code>{node.target || '—'}</code>
          <span>declared, never registered — it crashed at startup, or its
            binary is missing</span>
        </li>)}
      </ul>}
      {!!status?.undeclared.length && <ul className="declared-nodes">
        {status.undeclared.map(node => <li key={node.name}>
          <b>{node.name}</b>
          <code>{node.target || '—'}</code>
          <span>running but not declared — started outside the launcher</span>
        </li>)}
      </ul>}
      {declared.diagnostics.length ?
           <ul className="diagnostics">{declared.diagnostics.map(
               (diagnostic, i) => <li key={i} className={diagnostic.severity}>
                 <em>{diagnostic.severity}</em>
                 <code>{diagnostic.subject}</code>
                 <span>{diagnostic.message}</span>
               </li>)}</ul> :
           <Empty>
             The launcher's lint found nothing to report about this graph.
           </Empty>}
    </div>
  </section>;
}

export function SystemTab({system}: {system: SystemState}) {
  const [filter, setFilter] = useState('');
  const [picked, setPicked] = useState<string|null>(null);
  const graph = system.graph;

  const nodes = useMemo(() => {
    if (!graph) return [];
    const needle = filter.trim().toLowerCase();
    return graph.nodes.filter(
        node => !needle || node.name.toLowerCase().includes(needle) ||
            node.target.toLowerCase().includes(needle) ||
            node.sources.some(s => s.name.toLowerCase().includes(needle)));
  }, [graph, filter]);

  if (!graph) return <NoSystem system={system}/>;

  const selected = graph.nodes.find(n => n.name === picked) ?? nodes[0] ??
      graph.nodes[0] ?? null;

  return <div className="system">
    <div className="system-toolbar">
      <input aria-label="Filter nodes and topics" className="search"
             placeholder="Filter nodes, targets, topics…" value={filter}
             onChange={e => setFilter(e.target.value)}/>
      <div className="system-counts">
        <span>{graph.nodes.filter(n => n.alive).length}/{graph.nodes.length} nodes live</span>
        {/* Only when a declared graph arrived: "0 declared" would be a claim
            about a document nobody sent. */}
        {system.declared &&
         <span>{system.declared.nodes.length} declared</span>}
        <span>{graph.topics.length} topics</span>
        <span>slots {graph.nodes.length}/{graph.registry.node_capacity}</span>
        {!graph.registry.available &&
         <span className="warn">registry unavailable — no node has started</span>}
        {!endpointAttributesKnown(graph) &&
         <span className="warn">
           bridge sends version {graph.version} graphs, which carry no endpoint
           attributes: a topic whose far end is outside talOS reads as a fault
         </span>}
        {system.error && <span className="warn">{system.error}</span>}
        {system.declaredError &&
         <span className="warn">{system.declaredError}</span>}
      </div>
    </div>

    {system.declared &&
     <DeclaredPanel declared={system.declared} graph={graph}/>}

    <section className="panel nodes">
      <div className="panel-heading">
        <h2>Nodes</h2><span>{nodes.length} SHOWN</span>
      </div>
      <div className="node-grid">
        {nodes.map(node => <NodeCard key={node.name + node.generation}
                                     graph={graph} node={node}
                                     rates={system.rates}
                                     selected={selected?.name === node.name}
                                     onSelect={() => setPicked(node.name)}/>)}
        {!nodes.length && <Empty>No node matches that filter.</Empty>}
      </div>
    </section>

    <section className="panel wiring-panel">
      <div className="panel-heading">
        <h2>Wiring{selected && <span className="handle"> · {selected.name}</span>}</h2>
        <span>PUBLISHERS AND SUBSCRIBERS</span>
      </div>
      {selected ? <Wiring graph={graph} node={selected} rates={system.rates}/> :
                  <Empty>Select a node.</Empty>}
    </section>

    <section className="panel topics">
      <div className="panel-heading">
        <h2>Topics</h2><span>EVERY END, EVERY NODE</span>
      </div>
      <TopicTable graph={graph} rates={system.rates} filter={filter}
                  onPickNode={setPicked}/>
    </section>
  </div>;
}

// --- timing -----------------------------------------------------------------

export type TimingColumn =
    'node'|'kind'|'name'|'period'|'rate'|'events'|'latency'|'max'|'dropped';

export function TimingTab({system}: {system: SystemState}) {
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
          return Number(row.source.last_latency_ns);
        case 'max':
          return Number(row.source.max_latency_ns);
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

  if (!graph) return <NoSystem system={system}/>;

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
                            {source.kind === 'SENDER' ?
                                 '—' :
                                 duration(source.last_latency_ns)}
                          </td>
                          <td className="right">
                            {source.kind === 'SENDER' ?
                                 '—' :
                                 duration(source.max_latency_ns)}
                          </td>
                          <td className={'right' +
                                         (source.dropped === '0' ? '' : ' warn')}>
                            {integer(source.dropped)}
                          </td>
                        </tr>)}
        </tbody>
      </table>
      {!rows.length && <Empty>No source matches that filter.</Empty>}
    </div>
  </section>;
}

// --- overview ---------------------------------------------------------------

export function OverviewTab(
    {view, system, onOpen}: {
      view: View; system: SystemState;
      onOpen: (tab: TabId) => void;
    }) {
  const graph = system.graph;
  const declared = system.declared;
  const status = useMemo(
      () => declared ? declaredStatus(declared, graph) : null, [declared, graph]);

  // What is worth a person's attention, and nothing else. A list that reports
  // the design as a fault -- a topic whose consumer is the RoboRIO, an
  // autonomous target nobody publishes yet -- is a list people learn to skip,
  // and then it is not there for the fault that matters. So `bridged` and
  // `unconnected` are absent from it by construction, and loss is asked about
  // directly rather than through a classification that a declared endpoint
  // attribute could hide.
  const problems = useMemo(() => {
    const out: {severity: 'warn'|'note'; text: string; tab: TabId}[] = [];
    for (const node of status?.missing ?? [])
      out.push({
        severity: 'warn',
        text: `${node.name} was declared and never registered; it crashed ` +
            'at startup, or its binary is missing',
        tab: 'system'
      });
    // The launcher's words, not a re-derivation of them: it is the process that
    // decided whether this graph could start.
    for (const diagnostic of declared?.diagnostics ?? [])
      out.push({
        severity: diagnostic.severity === 'error' ? 'warn' : 'note',
        text: `launcher: ${diagnostic.subject} ${diagnostic.message}`,
        tab: 'system'
      });
    // What the launcher already said, by subject. A topic it has spoken about
    // is not raised again from the live graph: the linter's rules are exactly
    // "nothing publishes it" and "nothing subscribes to it", so a second item
    // for the same finding is the noise this list exists to avoid -- in worse
    // words than the launcher's. Loss is not something the launcher can know
    // about, so it is never suppressed.
    const linted = new Set((declared?.diagnostics ?? []).map(d => d.subject));

    if (graph) {
      const health = topicHealthOf(graph);
      for (const node of graph.nodes)
        if (!node.alive)
          out.push({
            severity: 'warn',
            text: `${node.name} stopped heartbeating ${
                duration(staleness(graph, node))} ago`,
            tab: 'system'
          });
      for (const topic of graph.topics) {
        const state = health(topic);
        if (state === 'orphaned' && !linted.has(topic.name))
          out.push({
            severity: 'warn',
            text: `${topic.name} is read by ${
                topic.subscribers.join(', ')} but nothing publishes it`,
            tab: 'system'
          });
        if (topic.dropped !== '0')
          out.push({
            severity: 'warn',
            text: `${topic.name} lapped ${integer(topic.dropped)} messages`,
            tab: 'timing'
          });
        if (state === 'unread' && !linted.has(topic.name))
          out.push({
            severity: 'note',
            text: `${topic.name} is published but nobody reads it`,
            tab: 'system'
          });
      }
      if (graph.bridge.ws_drops !== '0')
        out.push({
          severity: 'note',
          text: `bridge dropped ${
              integer(graph.bridge.ws_drops)} frames to slow clients`,
          tab: 'system'
        });
      // Said once, at the bottom: without endpoint attributes some of the items
      // above are the design rather than faults, and a list that cannot tell
      // which should admit it rather than be quietly wrong.
      if (!endpointAttributesKnown(graph))
        out.push({
          severity: 'note',
          text: `the bridge sends version ${
              graph.version} system graphs, which carry no endpoint ` +
              'attributes: a topic whose far end is outside talOS is listed ' +
              'here as a fault',
          tab: 'system'
        });
    }
    return out;
  }, [graph, declared, status]);

  const s = view.snapshot;
  return <div className="overview">
    <section className="panel">
      <div className="panel-heading"><h2>Stream</h2><span>TELEMETRY</span></div>
      <dl className="stat-list">
        <div><dt>Status</dt><dd>{view.status}</dd></div>
        <div><dt>Mode</dt><dd>{view.mode.replaceAll('_', ' ')}</dd></div>
        <div><dt>Sequence</dt><dd>{s?.sequence_id ?? '—'}</dd></div>
        <div><dt>Retained frames</dt><dd>{integer(view.history.length)}</dd></div>
        <div><dt>Dropped / late</dt>
          <dd className={view.stats.dropped || view.stats.late ? 'warn' : ''}>
            {view.stats.dropped} / {view.stats.late}
          </dd></div>
        <div><dt>Invalid</dt>
          <dd className={view.stats.invalid ? 'warn' : ''}>
            {view.stats.invalid}
          </dd></div>
        <div><dt>Channels</dt>
          <dd>{Object.keys(s?.channels ?? {}).length}</dd></div>
      </dl>
    </section>

    <section className="panel">
      <div className="panel-heading"><h2>System</h2><span>REGISTRY</span></div>
      {graph ? <dl className="stat-list">
        <div><dt>Nodes live</dt>
          <dd>{graph.nodes.filter(n => n.alive).length} of {graph.nodes.length}</dd>
        </div>
        {/* Declared against actual, which is the pair worth reading together.
            Omitted rather than zeroed when no declared graph arrived. */}
        {declared && <div><dt>Declared</dt>
          <dd>{declared.nodes.length} nodes, {
              status ? `${status.registered} live` : 'no registry yet'}</dd>
        </div>}
        {!!status?.missing.length && <div><dt>Never appeared</dt>
          <dd className="warn">{status.missing.map(n => n.name).join(', ')}</dd>
        </div>}
        {!!declared?.diagnostics.length && <div><dt>Launcher lint</dt>
          <dd className={status?.errors ? 'warn' : ''}>
            {status?.errors ?? 0} errors, {status?.warnings ?? 0} warnings
          </dd>
        </div>}
        <div><dt>Topics</dt><dd>{graph.topics.length}</dd></div>
        <div><dt>Session</dt>
          <dd>{graph.nodes.find(n => n.session_id !== '0')?.session_id ?? '—'}</dd>
        </div>
        <div><dt>Simulation</dt>
          <dd>{graph.nodes.some(n => n.simulation) ? 'yes' : 'no'}</dd></div>
        <div><dt>Bridge topic</dt><dd><code>{graph.bridge.topic}</code></dd></div>
        <div><dt>Bridge clients</dt><dd>{graph.bridge.clients}</dd></div>
        <div><dt>Frames forwarded</dt>
          <dd>{integer(graph.bridge.frames_forwarded)}</dd></div>
      </dl> : <NoSystem system={system}/>}
      {system.declaredError &&
       <p className="warn declared-error">{system.declaredError}</p>}
    </section>

    <section className="panel wide">
      <div className="panel-heading">
        <h2>Attention</h2>
        <span>{problems.length ? problems.length + ' ITEMS' : 'NOTHING'}</span>
      </div>
      {problems.length ?
           <ul className="attention">{problems.map(
               (problem, i) => <li key={i} className={problem.severity}>
                 <span>{problem.text}</span>
                 <button className="link" onClick={() => onOpen(problem.tab)}>
                   open
                 </button>
               </li>)}</ul> :
           <Empty>
             {graph ?
                  'Every topic that needs both ends has them, every declared node registered, and no node has gone quiet.' :
                  'Connect to see what is running.'}
           </Empty>}
    </section>
  </div>;
}

// --- agent ------------------------------------------------------------------

const METHOD_HELP: Record<string, string> = {
  get_schema_tree: 'Leaf topics observed in retained telemetry, with types.',
  query_state_at: 'A snapshot at an exact timestamp, or the one before it.',
  scan_channel_events: 'Sample ranges where a channel satisfies a condition.',
  get_system_graph: 'Every node, every topic, both ends, with rates.',
  describe_topic: 'One topic: who writes it, who reads it, how fast.',
  get_declared_graph:
      'What the launcher declared before spawning, against what registered.'
};

const METHOD_TEMPLATE: Record<string, string> = {
  get_schema_tree: '{}',
  query_state_at: '{\n  "timestamp_ns": "0",\n  "mode": "at_or_before"\n}',
  scan_channel_events:
      '{\n  "topic": "velocity",\n  "condition": {"op": "gt", "value": 1}\n}',
  get_system_graph: '{}',
  describe_topic: '{\n  "topic": "/odometry/state"\n}',
  get_declared_graph: '{}'
};

export function AgentTab({call}: {call: (method: string, params: unknown) => Promise<unknown>}) {
  const [method, setMethod] = useState<string>('get_system_graph');
  const [params, setParams] = useState<string>(
      METHOD_TEMPLATE['get_system_graph']);
  const [result, setResult] = useState('');
  const [busy, setBusy] = useState(false);

  const run = async () => {
    setBusy(true);
    try {
      const parsed = params.trim() ? JSON.parse(params) : {};
      const response = await call(method, parsed);
      setResult(JSON.stringify(response, null, 2));
    } catch (e) {
      setResult(String(e));
    } finally {
      setBusy(false);
    }
  };

  return <div className="agent">
    <section className="panel">
      <div className="panel-heading"><h2>Query</h2><span>JSON-RPC 2.0</span></div>
      <div className="agent-form">
        <label>
          Method
          <select value={method} onChange={e => {
            setMethod(e.target.value);
            setParams(METHOD_TEMPLATE[e.target.value] ?? '{}');
          }}>
            {RPC_METHODS.map(name => <option key={name} value={name}>{name}</option>)}
          </select>
        </label>
        <small>{METHOD_HELP[method]}</small>
        <label>
          Params
          <textarea aria-label="JSON-RPC params" spellCheck={false}
                    value={params} rows={7}
                    onChange={e => setParams(e.target.value)}/>
        </label>
        <button className="primary" disabled={busy} onClick={run}>
          {busy ? 'Running…' : 'Send'}
        </button>
      </div>
      <div className="panel-footer">
        SAME CODE PATH AS <code>bazel run //studio:agent</code>
        <span>http://127.0.0.1:5802/rpc</span>
      </div>
    </section>
    <section className="panel">
      <div className="panel-heading"><h2>Response</h2><span>VERBATIM</span></div>
      <pre className="agent-result">{
          result || 'Send a request to see exactly what an agent receives.'}</pre>
    </section>
  </div>;
}

