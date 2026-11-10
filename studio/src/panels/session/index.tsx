// The Session panel: what launched, from which config, and how every process
// has fared. Three views over two documents plus the live registry:
//
//   Session   — the launcher's manifest: session id, simulation, config path +
//               its hash, output dir, start time and uptime (end time when the
//               session ended).
//   Processes — one row per process, merging the manifest nodes, the live
//               registry and the declared graph by name, so a process that
//               crashed before claiming a slot still shows up (declared, never
//               registered) and a process started outside the launcher is not
//               lost (registry only).
//   Counts    — running / total processes and total restarts.
//
// The manifest is served by the bridge at /manifest.json only when it was
// started with --manifest; otherwise it is "not offered", which is not an
// error. Everything here is read off the data, never inferred: an absent field
// renders as "—" (no data), and a problem is shown by color and by text.

import {useMemo, useState, type ReactNode} from 'react';

import {useManifest, useSystem} from '../../data/hooks';
import type {SessionManifest} from '../../data/manifest';
import type {SystemGraph} from '../../system';
import {duration, hertz, integer} from '../../ui/formatters';
import {EmptyState, StatusDot} from '../../ui/primitives';
import {GraphFreshnessNotice} from '../../ui/FreshnessNotice';
import type {PanelDefinition} from '../types';
import {mergeProcesses, summarize, type ProcessProblem} from './logic';
import './session.css';

export type SessionConfig = Record<string, never>;

const defaultConfig: SessionConfig = {};
const parseConfig = (_raw: unknown): SessionConfig => ({});

// A wall-clock uint64 nanosecond count as a UTC date/time. Kept local: no
// other panel needs a wall-clock date, and the shared formatters stay about
// durations and relative ages.
function wallClock(ns: string): string {
  const ms = Number(BigInt(ns) / 1_000_000n);
  const date = new Date(ms);
  if (!Number.isFinite(ms) || Number.isNaN(date.getTime())) return '—';
  return date.toISOString().replace('T', ' ').replace(/\.\d{3}Z$/, ' UTC');
}

// Elapsed nanoseconds as a human uptime. The shared `duration` formatter tops
// out at seconds; a session runs for minutes or hours.
function uptime(ns: number): string {
  if (!Number.isFinite(ns) || ns < 0) return '—';
  const seconds = ns / 1e9;
  if (seconds < 60) return `${seconds.toFixed(1)} s`;
  const minutes = Math.floor(seconds / 60);
  const secs = Math.floor(seconds - minutes * 60);
  if (minutes < 60) return `${minutes} m ${secs} s`;
  const hours = Math.floor(minutes / 60);
  const mins = minutes - hours * 60;
  if (hours < 24) return `${hours} h ${mins} m`;
  const days = Math.floor(hours / 24);
  return `${days} d ${hours - days * 24} h`;
}

const PROBLEM_META: Record<ProcessProblem, {label: string; cls: string}> = {
  dead: {label: 'dead', cls: 'dead'},
  restarted: {label: 'restarted', cls: 'restarted'},
  recorder_failed: {label: 'recorder failed', cls: 'recorder'},
  nonzero_exit: {label: 'nonzero exit', cls: 'exit'},
  killed_by_signal: {label: 'killed by signal', cls: 'exit'},
  never_registered: {label: 'never registered', cls: 'missing'}
};

// A path on one line: the directory part ellipsizes in the middle so the root
// and the final segment stay visible, and the whole value is on `title` for
// hover. No mid-word wrap.
function PathText({path}: {path: string}) {
  const slash = path.lastIndexOf('/');
  const split = slash >= 0 && slash < path.length - 1;
  return <code className="path-line" title={path}>
    {split && <span className="path-head">{path.slice(0, slash + 1)}</span>}
    <span className="path-tail">{split ? path.slice(slash + 1) : path}</span>
  </code>;
}

function CopyButton(
    {path, copied, onCopy}: {
      path: string; copied: boolean; onCopy: (path: string) => void;
    }) {
  return <button className="copy" onClick={() => onCopy(path)}>
    {copied ? 'copied' : 'copy'}
  </button>;
}

// A path plus its copy affordance, used in the header and in the log column.
function PathValue(
    {path, copied, onCopy}: {
      path: string; copied: boolean; onCopy: (path: string) => void;
    }) {
  return <span className="path-value">
    <PathText path={path}/>
    <CopyButton path={path} copied={copied} onCopy={onCopy}/>
  </span>;
}

function Field({label, wide, children}: {
  label: string; wide?: boolean; children: ReactNode;
}) {
  return <div className={'session-field' + (wide ? ' wide' : '')}>
    <span className="session-field-label">{label}</span>
    <span className="session-field-value">{children}</span>
  </div>;
}

function SessionHeader(
    {manifest, graph, copied, onCopy}: {
      manifest: SessionManifest; graph: SystemGraph|null;
      copied: string|null; onCopy: (path: string) => void;
    }) {
  const start = BigInt(manifest.start_wall_ns);
  const end = BigInt(manifest.end_wall_ns);
  const ended = end > start;
  // Only the registry's clock is comparable with the manifest's clock. A
  // running replay without a graph has no known uptime.
  const upNs = ended ? Number(end - start) :
      graph ? Math.max(0, Number(BigInt(graph.wall_ns) - start)) : null;
  const shortSha = manifest.config_sha256.slice(0, 8);

  return <div className="session-fields">
    <Field label="Config" wide>
      <PathValue path={manifest.config_path}
                 copied={copied === manifest.config_path} onCopy={onCopy}/>
      <code className="sha" title={manifest.config_sha256}>{shortSha}</code>
    </Field>
    <Field label="Output" wide>
      <PathValue path={manifest.output_dir}
                 copied={copied === manifest.output_dir} onCopy={onCopy}/>
    </Field>
    <Field label="Session">{manifest.session_id}</Field>
    <Field label="Simulation">{manifest.simulation ? 'yes' : 'no'}</Field>
    <Field label="Started">{wallClock(manifest.start_wall_ns)}</Field>
    {ended && <Field label="Ended">{wallClock(manifest.end_wall_ns)}</Field>}
    <Field label="Uptime">{upNs === null ? '—' : uptime(upNs)}</Field>
  </div>;
}

function SessionPanel() {
  const {manifest, error} = useManifest();
  const system = useSystem();
  const [copied, setCopied] = useState<string|null>(null);

  const rows = useMemo(
      () => mergeProcesses(manifest, system.graph, system.declared,
                           system.rates, system.stale),
      [manifest, system.graph, system.declared, system.rates, system.stale]);
  const summary = useMemo(() => summarize(rows), [rows]);

  const copyPath = (path: string) => {
    if (typeof navigator !== 'undefined' && navigator.clipboard?.writeText) {
      navigator.clipboard.writeText(path).then(() => setCopied(path)).catch(() => {});
    }
  };

  return <div className="session">
    <section className="card wide">
      <div className="card-heading">
        <h2>Session</h2><span>MANIFEST</span>
      </div>
      {manifest ?
          <SessionHeader manifest={manifest} graph={system.graph}
                         copied={copied} onCopy={copyPath}/> :
       error ? <EmptyState>{error}</EmptyState> :
       <EmptyState>
         Session manifest not offered. This is demo or replay data, or the
         bridge was started without --manifest.
       </EmptyState>}
    </section>

    <section className="card wide">
      <div className="card-heading">
        <h2>Processes</h2><span>MANIFEST ∪ REGISTRY ∪ DECLARED</span>
      </div>
      <GraphFreshnessNotice/>
      <div className="system-counts session-counts">
        <span>{system.graph && system.stale ? '?' : summary.running}/{summary.total} processes running</span>
        <span>{summary.restarts} restart{summary.restarts === 1 ? '' : 's'}</span>
      </div>
      {rows.length === 0 ?
          <EmptyState>
            No processes. Connect to a robot bridge, or start demo data to
            explore the session view.
          </EmptyState> :
          <div className="table-scroll">
            <table className="grid session-table">
              <thead>
                <tr>
                  <th>Process</th>
                  <th>Target</th>
                  <th className="right">PID</th>
                  <th>Alive</th>
                  <th className="right">Restarts</th>
                  <th className="right">Exit</th>
                  <th>Recorder</th>
                  <th className="right">Records</th>
                  <th className="right">Loop</th>
                  <th className="right">Latency</th>
                  <th>Log</th>
                </tr>
              </thead>
              <tbody>
                {rows.map(row => <tr key={row.name}
                    className={row.problems.length ? 'problem' : ''}>
                  <td>
                    <b>{row.name}</b>
                    {row.problems.map(problem => {
                      const meta = PROBLEM_META[problem];
                      const label = problem === 'restarted' && row.restarts > 0 ?
                          `restarted ×${row.restarts}` : meta.label;
                      return <span key={problem}
                          className={'problem-tag ' + meta.cls}>{label}</span>;
                    })}
                  </td>
                  <td><code className="target">{row.target || '—'}</code></td>
                  <td className="right mono">{row.pid ?? '—'}</td>
                  <td className={row.alive === false ? 'text-warn' : ''}>
                    {row.alive === null ? '—' :
                     <span className="alive-cell">
                       <StatusDot active={row.alive}/>{
                           row.alive ? 'live' : 'dead'
                       }</span>}
                  </td>
                  <td className={'right' +
                      (row.restarts > 0 ? ' warn' : '')}>{row.restarts}</td>
                  <td className={'right' +
                      ((row.exitCode !== null && row.exitCode > 0) ||
                       row.exitSignal !== null ? ' warn' : '')}>
                    {row.exitSignal ?? (row.exitCode === null ? '—' : row.exitCode)}
                  </td>
                  <td className={row.recorderFailed ? 'text-warn' : ''}>
                    {!row.hasManifest ? '—' :
                     row.recorderFailed ? 'failed' : 'ok'}
                  </td>
                  <td className="right mono">
                    {row.records === null ? '—' : integer(row.records)}
                  </td>
                  <td className="right mono">
                    {row.loopRate === null ? '—' :
                     hertz(row.loopRate)}
                  </td>
                  <td className="right mono">
                    {row.worstLatency === null ? '—' :
                     duration(row.worstLatency)}
                  </td>
                  <td>
                    {row.logPath ? <span className="log-cell">
                      <PathValue path={row.logPath}
                                 copied={copied === row.logPath}
                                 onCopy={copyPath}/>
                    </span> : '—'}
                  </td>
                </tr>)}
              </tbody>
            </table>
          </div>}
    </section>
  </div>;
}

export const sessionPanel: PanelDefinition<SessionConfig> = {
  id: 'session',
  title: 'Session',
  category: 'System',
  description: 'What launched, from which config, and how each process fared.',
  defaultConfig,
  parseConfig,
  component: SessionPanel
};
