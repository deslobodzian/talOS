// The health panel: what the stream is doing, what the system looks like, and
// the short list of things actually worth a person's attention. It is the old
// OverviewTab reading its data through hooks; its "open tab" links became
// "open panel" links, still routed to the panel that explains each item.

import {useMemo} from 'react';

import {usePlayback, useSnapshot, useSystem} from '../../data/hooks';
import {liveNodeCount} from '../../data/freshness';
import {usePanelNavigation} from '../../layout/navigation';
import {
  declaredStatus,
  endpointAttributesKnown,
  topicHealthOf,
} from '../../system';
import {duration, integer, staleness} from '../../ui/formatters';
import {EmptyState, Stat, StatList} from '../../ui/primitives';
import {GraphFreshnessNotice} from '../../ui/FreshnessNotice';
import type {PanelDefinition} from '../types';
import './health.css';

export type HealthConfig = Record<string, never>;

const defaultConfig: HealthConfig = {};
const parseConfig = (_raw: unknown): HealthConfig => ({});

type Problem = {severity: 'warn'|'note'; text: string; panel: string};

function HealthPanel() {
  const playback = usePlayback();
  const snapshot = useSnapshot();
  const system = useSystem();
  const navigation = usePanelNavigation();

  const graph = system.graph;
  const graphStale = system.stale;
  const declared = system.declared;
  const status = useMemo(
      () => declared ? declaredStatus(declared, graph) : null, [declared, graph]);

  // What is worth a person's attention, and nothing else. A list that reports
  // the design as a fault -- a topic whose consumer is the RoboRIO, an
  // autonomous target nobody publishes yet -- is a list people learn to skip,
  // and then it is not there for the fault that matters. So `bridged` and
  // `unconnected` are absent by construction, and loss is asked about directly
  // rather than through a classification a declared endpoint attribute could
  // hide.
  const problems = useMemo(() => {
    const out: Problem[] = [];
    for (const node of status?.missing ?? [])
      out.push({
        severity: 'warn',
        text: `${node.name} was declared and never registered; it crashed ` +
            'at startup, or its binary is missing',
        panel: 'system'
      });
    // The launcher's words, not a re-derivation of them.
    for (const diagnostic of declared?.diagnostics ?? [])
      out.push({
        severity: diagnostic.severity === 'error' ? 'warn' : 'note',
        text: `launcher: ${diagnostic.subject} ${diagnostic.message}`,
        panel: 'system'
      });
    const linted = new Set((declared?.diagnostics ?? []).map(d => d.subject));

    if (graph) {
      const health = topicHealthOf(graph);
      // A stale graph's liveness is last-known, not current: one honest
      // "unknown" rather than a per-node claim either way.
      if (graphStale)
        out.push({
          severity: 'warn',
          text: 'the system graph is stale: which nodes are alive is unknown',
          panel: 'system'
        });
      else
        for (const node of graph.nodes)
          if (!node.alive)
            out.push({
              severity: 'warn',
              text: `${node.name} stopped heartbeating ${
                  duration(staleness(graph, node))} ago`,
              panel: 'system'
            });
      for (const topic of graph.topics) {
        const state = health(topic);
        if (state === 'orphaned' && !linted.has(topic.name))
          out.push({
            severity: 'warn',
            text: `${topic.name} is read by ${
                [...topic.subscribers, ...topic.observers].join(', ')} but nothing publishes it`,
            panel: 'system'
          });
        if (topic.dropped !== '0')
          out.push({
            severity: 'warn',
            text: `${topic.name} lost ${integer(topic.dropped)} messages`,
            panel: 'timing'
          });
        if (state === 'unread' && !linted.has(topic.name))
          out.push({
            severity: 'note',
            text: `${topic.name} is published but nobody reads it`,
            panel: 'system'
          });
      }
      if (graph.bridge.ws_drops !== '0')
        out.push({
          severity: 'note',
          text: `bridge dropped ${
              integer(graph.bridge.ws_drops)} messages to slow clients`,
          panel: 'system'
        });
      if (!endpointAttributesKnown(graph))
        out.push({
          severity: 'note',
          text: `the bridge sends version ${
              graph.version} system graphs, which carry no endpoint ` +
              'attributes: a topic whose far end is outside talOS is listed ' +
              'here as a fault',
          panel: 'system'
        });
    }
    return out;
  }, [graph, graphStale, declared, status]);

  const s = snapshot;
  const channels = Object.keys(s?.channels ?? {}).length;

  return <div className="overview">
    <section className="panel">
      <div className="panel-heading"><h2>Stream</h2><span>TELEMETRY</span></div>
      <StatList>
        <Stat label="Status">{playback.status}</Stat>
        <Stat label="Mode">{playback.mode.replaceAll('_', ' ')}</Stat>
        <Stat label="Connection">{playback.connection}</Stat>
        <Stat label="Sequence">{s?.sequence_id ?? '—'}</Stat>
        <Stat label="Skipped / late" tone={
          playback.stats.late ? 'warn' : undefined}>
          {playback.stats.skipped} / {playback.stats.late}
        </Stat>
        <Stat label="Invalid" tone={playback.stats.invalid ? 'warn' : undefined}>
          {playback.stats.invalid}
        </Stat>
        <Stat label="Channels">{channels}</Stat>
      </StatList>
    </section>

    <section className="panel">
      <div className="panel-heading"><h2>System</h2><span>REGISTRY</span></div>
      {graph && <GraphFreshnessNotice/>}
      {graph ? <StatList>
        <Stat label="Nodes live" tone={graphStale ? 'warn' : undefined}>
          {liveNodeCount(system) ?? 'unknown'} of {graph.nodes.length}
          {graphStale && ' (stale graph)'}
        </Stat>
        {declared && <Stat label="Declared">
          {declared.nodes.length} nodes, {
              status ? `${status.registered} ${graphStale ? 'registered (stale graph)' : 'live'}` :
                       'no registry yet'}
        </Stat>}
        {!!status?.missing.length && <Stat label="Never appeared" tone="warn">
          {status.missing.map(n => n.name).join(', ')}
        </Stat>}
        {!!declared?.diagnostics.length && <Stat label="Launcher lint"
              tone={status?.errors ? 'warn' : undefined}>
          {status?.errors ?? 0} errors, {status?.warnings ?? 0} warnings
        </Stat>}
        <Stat label="Topics">{graph.topics.length}</Stat>
        <Stat label="Session">
          {graph.nodes.find(n => n.session_id !== '0')?.session_id ?? '—'}
        </Stat>
        <Stat label="Simulation">
          {graph.nodes.some(n => n.simulation) ? 'yes' : 'no'}
        </Stat>
        <Stat label="Bridge topic"><code>{graph.bridge.topic}</code></Stat>
        <Stat label="Bridge clients">{graph.bridge.clients}</Stat>
        <Stat label="Frames forwarded">
          {integer(graph.bridge.frames_forwarded)}
        </Stat>
      </StatList> : <EmptyState>
        {system.error ? system.error :
                        'No system graph yet. Connect to a robot bridge, or start demo data to explore the viewer.'}
      </EmptyState>}
      {system.declaredError &&
       <p className="text-warn declared-error">{system.declaredError}</p>}
    </section>

    <section className="panel wide">
      <div className="panel-heading">
        <h2>Attention</h2>
        <span>{problems.length ? problems.length + ' ITEMS' : 'NOTHING'}</span>
      </div>
      {problems.length ?
           <ul className="attention">{problems.map((problem, i) =>
             <li key={i} className={problem.severity}>
               <span>{problem.text}</span>
               <button className="link"
                       onClick={() => navigation.openPanel(problem.panel)}>
                 open
               </button>
             </li>)}</ul> :
           <EmptyState>
             {graph ?
                  'Every topic that needs both ends has them, every declared node registered, and no node has gone quiet.' :
                  'Connect to see what is running.'}
           </EmptyState>}
    </section>
  </div>;
}

export const healthPanel: PanelDefinition<HealthConfig> = {
  id: 'health',
  title: 'Health',
  category: 'Robot',
  description: 'Stream status, system registry and the items worth attention.',
  defaultConfig,
  parseConfig,
  component: HealthPanel
};
