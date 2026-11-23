// The health panel: what the stream is doing, what the system looks like, and
// the short list of things actually worth a person's attention. It is the old
// OverviewTab reading its data through hooks; its "open tab" links became
// "open panel" links, still routed to the panel that explains each item.

import {useMemo} from 'react';

import {AlertLevel, alertLevelName} from '../../data/alerts';
import {
  useActions,
  useAlertLatch,
  usePlayback,
  useRecorded,
  useSnapshot,
  useSystem
} from '../../data/hooks';
import {liveNodeCount} from '../../data/freshness';
import {isUncleanRunExit, type RecordedSource} from '../../data/recorded';
import {wallTimeNs} from '../../data/review';
import {usePanelNavigation} from '../../layout/navigation';
import {declaredStatus, graphVersionMismatch, topicHealthOf} from '../../system';
import {duration, integer, sessionTime, staleness, wallDateTime} from '../../ui/formatters';
import {EmptyState, Stat, StatList} from '../../ui/primitives';
import {GraphFreshnessNotice} from '../../ui/FreshnessNotice';
import {GraphVersionNotice} from '../system/GraphVersionNotice';
import type {PanelDefinition} from '../types';
import './health.css';

export type HealthConfig = Record<string, never>;

const defaultConfig: HealthConfig = {};
const parseConfig = (_raw: unknown): HealthConfig => ({});

// `code` names a problem that is not about one node or topic, so its kind
// stays legible apart from its words.
type Problem = {
  severity: 'warn'|'note';
  text: string;
  panel?: string;
  seekNs?: string;
  code?: string;
};

function RecordedHealthPanel({recorded}: {recorded: RecordedSource}) {
  const playback = usePlayback();
  const latch = useAlertLatch();
  const actions = useActions();

  const {bounds, timeline, declared} = recorded;
  const start = BigInt(bounds.start_ns);
  const end = BigInt(bounds.end_ns);
  const cursor = playback.cursor_ns === null ? start : BigInt(playback.cursor_ns);
  const offsetLabel = (t_ns: string) => {
    const dt = BigInt(t_ns) - start;
    return dt < 0n ? `-${sessionTime(-dt)}` : sessionTime(dt);
  };

  const declaredErrors =
      declared?.diagnostics.filter(d => d.severity === 'error').length ?? 0;
  const declaredWarnings =
      declared?.diagnostics.filter(d => d.severity === 'warning').length ?? 0;

  const latchedAlerts = latch?.alerts ?? [];
  const activeCount = latchedAlerts.filter(a => a.active).length;

  const problems = useMemo(() => {
    const out: Problem[] = [];
    if (timeline.hardware_not_decoded) {
      out.push({
        severity: 'warn',
        text: bounds.hardware_not_decoded ??
            'Hardware packets were not decoded; Driver Station modes and hardware channels are unavailable',
        code: 'hardware-not-decoded'
      });
    }
    for (const g of bounds.gaps) {
      const seconds = (Number(BigInt(g.end_ns) - BigInt(g.start_ns)) / 1e9).toFixed(3);
      out.push({
        severity: 'warn',
        text: `log gap in ${g.process} at ${offsetLabel(g.start_ns)}: ${seconds} s, ` +
            `lost ${g.records} records (${g.dispatches} dispatches, ${g.sends} sends)` +
            (g.closed_at_flush ? '; closed at stop' : ''),
        seekNs: g.start_ns
      });
    }
    const seenNode = new Map<string, number>();
    for (const run of timeline.runs) {
      const n = (seenNode.get(run.node) ?? 0) + 1;
      seenNode.set(run.node, n);
      if (n > 1) {
        out.push({
          severity: 'warn',
          text: `${run.node} restarted (run ${n}) at ${offsetLabel(run.start_ns)}`,
          seekNs: run.start_ns
        });
      }
      if (isUncleanRunExit(run.reason, run.end_ns)) {
        out.push({
          severity: 'warn',
          text: `${run.node} ended with ${run.reason.replace(/_/g, ' ')} at ${
              offsetLabel(run.end_ns!)} (run ${n})`,
          seekNs: run.end_ns!
        });
      }
    }
    for (const alert of latchedAlerts) {
      if (alert.level === AlertLevel.Info) continue;
      out.push({
        severity: alert.level === AlertLevel.Error ? 'warn' : 'note',
        text: `alert (${alertLevelName(alert.level)}) ${alert.owner}: ${alert.text}` +
            (alert.active ? ' — active' : ' — latched') +
            (alert.count > 1 ? ` (${alert.count}x)` : ''),
        seekNs: alert.last_ns
      });
    }
    for (const diagnostic of declared?.diagnostics ?? []) {
      out.push({
        severity: diagnostic.severity === 'error' ? 'warn' : 'note',
        text: `launcher: ${diagnostic.subject} ${diagnostic.message}`
      });
    }
    if (timeline.modes_dropped || timeline.alerts_dropped || timeline.runs_dropped) {
      const parts: string[] = [];
      if (timeline.modes_dropped) parts.push(`${timeline.modes_dropped} mode transitions`);
      if (timeline.alerts_dropped) parts.push(`${timeline.alerts_dropped} alert intervals`);
      if (timeline.runs_dropped) parts.push(`${timeline.runs_dropped} node runs`);
      out.push({
        severity: 'note',
        text: `timeline capped: dropped ${parts.join(', ')}`
      });
    }
    return out;
  }, [bounds, timeline, declared, latchedAlerts]);

  return <div className="overview">
    <section className="panel">
      <div className="panel-heading"><h2>Recording</h2><span>PLAYBACK</span></div>
      <StatList>
        <Stat label="Status">{playback.status}</Stat>
        <Stat label="Mode">{playback.mode.replaceAll('_', ' ')}</Stat>
        <Stat label="Playhead">
          {sessionTime(cursor - start)} · {wallDateTime(wallTimeNs(bounds, cursor.toString()))}
        </Stat>
        <Stat label="Duration">{sessionTime(end - start)}</Stat>
        <Stat label="Channels">{recorded.dictionary().length}</Stat>
        <Stat label="Log gaps" tone={bounds.gaps.length ? 'warn' : undefined}>
          {bounds.gaps.length}
        </Stat>
        <Stat label="Hardware" tone={timeline.hardware_not_decoded ? 'warn' : undefined}>
          {timeline.hardware_not_decoded ? 'not decoded' : 'decoded'}
        </Stat>
      </StatList>
    </section>

    <section className="panel">
      <div className="panel-heading"><h2>System</h2><span>RECORDED</span></div>
      <p className="recorded-notice">
        Live registry liveness is not present in recorded sessions; node liveness
        counts and live heartbeats are unavailable.
      </p>
      {declared ? <StatList>
        <Stat label="Declared nodes">{declared.nodes.length}</Stat>
        <Stat label="Roster">{declared.nodes.map(n => n.name).join(', ') || '—'}</Stat>
        {!!declared.can_buses?.length && <Stat label="CAN buses">
          {declared.can_buses.join(', ')}
        </Stat>}
        <Stat label="Launcher lint" tone={declaredErrors ? 'warn' : undefined}>
          {declaredErrors} errors, {declaredWarnings} warnings
        </Stat>
      </StatList> : <EmptyState>
        No declared graph (graph.json) in this session.
      </EmptyState>}
    </section>

    <section className="panel">
      <div className="panel-heading">
        <h2>Alerts</h2>
        <span>{activeCount} ACTIVE · {latchedAlerts.length} LATCHED</span>
      </div>
      {latchedAlerts.length ? <ul className="recorded-list">
        {latchedAlerts.map(alert => <li key={alert.channel}
            className={'alert-' + alertLevelName(alert.level)}>
          <span className="recorded-row-main">
            <strong>{alert.owner}: {alert.text}</strong>
            <span className="text-muted">
              {alertLevelName(alert.level)} · {alert.active ? 'active' : 'cleared'}
              {alert.count > 1 ? ` · ${alert.count}x` : ''}
            </span>
          </span>
          <button className="link" onClick={() => actions.seek(alert.last_ns)}>
            {offsetLabel(alert.last_ns)}
          </button>
        </li>)}
      </ul> : <EmptyState>
        {timeline.alerts.length ?
          `No alerts raised at or before the playhead (${timeline.alerts.length} later in session).` :
          'No alerts raised in this session.'}
      </EmptyState>}
      {timeline.alerts_dropped > 0 && <p className="recorded-notice">
        {timeline.alerts_dropped} alert intervals dropped at the timeline cap.
      </p>}
    </section>

    <section className="panel">
      <div className="panel-heading">
        <h2>Node runs</h2>
        <span>{timeline.runs.length} RUNS</span>
      </div>
      {timeline.runs.length ? <div className="table-scroll">
        <table className="grid">
          <thead>
            <tr><th>Node</th><th>Start</th><th>Duration</th><th>Reason</th></tr>
          </thead>
          <tbody>
            {timeline.runs.map((run, i) => {
              const unclean = isUncleanRunExit(run.reason, run.end_ns);
              const runDuration = run.end_ns === null ? 'running' :
                  sessionTime(BigInt(run.end_ns) - BigInt(run.start_ns));
              return <tr key={`${run.node}:${run.start_ns}:${i}`}>
                <td>{run.node}</td>
                <td>
                  <button className="link" onClick={() => actions.seek(run.start_ns)}>
                    {offsetLabel(run.start_ns)}
                  </button>
                </td>
                <td>{runDuration}</td>
                <td className={unclean ? 'text-warn' : undefined}>
                  {run.reason.replace(/_/g, ' ')}
                </td>
              </tr>;
            })}
          </tbody>
        </table>
      </div> : <EmptyState>No node run history in manifest.json.</EmptyState>}
      {timeline.runs_dropped > 0 && <p className="recorded-notice">
        {timeline.runs_dropped} earlier runs dropped from manifest history.
      </p>}
    </section>

    <section className="panel wide">
      <div className="panel-heading">
        <h2>Attention</h2>
        <span>{problems.length ? problems.length + ' ITEMS' : 'NOTHING'}</span>
      </div>
      {problems.length ?
           <ul className="attention">{problems.map((problem, i) =>
             <li key={i} className={problem.severity} data-code={problem.code}>
               <span>{problem.text}</span>
               {problem.seekNs && <button className="link"
                       onClick={() => actions.seek(problem.seekNs!)}>
                 seek
               </button>}
             </li>)}</ul> :
           <EmptyState>
             No faults, restarts, alerts, or log gaps recorded.
           </EmptyState>}
    </section>
  </div>;
}

function HealthPanel() {
  const recorded = useRecorded();
  if (recorded) return <RecordedHealthPanel recorded={recorded}/>;
  return <LiveHealthPanel/>;
}

function LiveHealthPanel() {
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

    // The bridge is sending, but in a graph version this page cannot read:
    // not "nothing needs attention", and not a stale graph either.
    const versions = graph ? null : graphVersionMismatch(system.error);
    if (versions)
      out.push({
        severity: 'warn',
        text: `the bridge sends system graph version ${versions.bridge}, ` +
            `this page reads version ${versions.page}: which nodes are alive ` +
            'is unknown',
        panel: 'system',
        code: 'graph-version-mismatch'
      });

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
    }
    return out;
  }, [graph, graphStale, declared, status, system.error]);

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
      </StatList> : graphVersionMismatch(system.error) ? <GraphVersionNotice/> :
      <EmptyState>
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
             <li key={i} className={problem.severity} data-code={problem.code}>
               <span>{problem.text}</span>
               {problem.panel && <button className="link"
                       onClick={() => navigation.openPanel(problem.panel!)}>
                 open
               </button>}
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
  component: HealthPanel,
  recorded: true
};
