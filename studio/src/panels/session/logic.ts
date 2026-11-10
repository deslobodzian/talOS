// The Session panel's pure merge: one row per process, assembled from three
// documents that answer different halves of the same question.
//
//   manifest — what the launcher started, and how each process finally fared
//              (pid, exit code, restarts, recorder, record count).
//   graph    — the live registry: what holds a slot right now, alive or not.
//   declared — what the launcher probed before it spawned anything.
//
// None of them is the whole story on its own. A node that crashed before it
// claimed a slot is absent from the registry but still listed in the manifest
// and the declared graph; a node started outside the launcher appears only in
// the registry. Merging by name and saying which sources actually said
// something is the only honest way to present "how every process has fared".

import type {ManifestNode, SessionManifest} from '../../data/manifest';
import {rateKey, type DeclaredGraph, type SystemGraph, type SystemNode} from '../../system';

// A problem worth the top of the list and a visible mark, in the order a
// person cares about them.
export type ProcessProblem =
    'dead'|'restarted'|'recorder_failed'|'nonzero_exit'|'killed_by_signal'|
    'never_registered';

export type ProcessRow = {
  name: string;
  target: string;
  // The live registry's pid when it has one; otherwise the launcher's. Null
  // only when neither source saw the process.
  pid: string|null;
  // Null is "the registry has nothing to say", not "not alive": a node that
  // never claimed a slot is absent from the registry, and answering "dead"
  // for it would be inventing one. A stale registry graph says nothing
  // current either, so it is null then too.
  alive: boolean|null;
  // Whether a manifest node named this process (so the recorder/exit columns
  // can tell "no" from "the manifest did not say").
  hasManifest: boolean;
  restarts: number;
  // The manifest's exit_code when it is non-negative (the process exited),
  // else null. A null never reads as zero: "exited cleanly" is exit 0.
  exitCode: number|null;
  // The signal that ended the last run, as the launcher reports it
  // ("signal 9 SIGKILL"), else null.
  exitSignal: string|null;
  recorderFailed: boolean;
  records: string|null;
  loopRate: number|null;
  worstLatency: number|null;
  logPath: string|null;
  problems: ProcessProblem[];
};

// Signal numbers that Linux and macOS share; any other is shown by number
// only, since the viewer cannot tell which host's numbering the launcher used.
const SIGNAL_NAMES: Record<number, string> = {
  1: 'SIGHUP', 2: 'SIGINT', 3: 'SIGQUIT', 4: 'SIGILL', 5: 'SIGTRAP',
  6: 'SIGABRT', 8: 'SIGFPE', 9: 'SIGKILL', 11: 'SIGSEGV', 13: 'SIGPIPE',
  14: 'SIGALRM', 15: 'SIGTERM'
};

// The launcher's own spelling (SignalText in talOS/launcher/launcher.h).
export function signalText(signal: number): string {
  const name = SIGNAL_NAMES[signal];
  return name ? `signal ${signal} ${name}` : `signal ${signal}`;
}

export type ProcessSummary = {
  running: number;
  total: number;
  restarts: number;
};

// The node's own cadence, the same rule the System panel's node card uses: a
// loop node's timers are the honest answer, and a node without timers falls
// back to the fastest thing it publishes rather than reporting no rate for the
// one process whose rate matters most.
export function loopRate(
    node: SystemNode, rates: Record<string, number>): number|null {
  const timers = node.sources.filter(s => s.kind === 'TIMER');
  const candidates = timers.length ? timers : node.sources;
  const measured = candidates.map(s => rates[rateKey(node.name, s.id)])
                        .filter((r): r is number => r !== undefined);
  return measured.length ? Math.max(...measured) : null;
}

// The worst latency a dispatch measured, across the node's sources. A sender
// or fetcher never measures one, so it contributes nothing; reporting a
// number here would be inventing one.
export function worstLatency(node: SystemNode): number|null {
  const latencies = node.sources
                        .filter(s => s.kind === 'TIMER' || s.kind === 'WATCHER')
                        .map(s => Number(s.max_latency_ns))
                        .filter(n => Number.isFinite(n));
  return latencies.length ? Math.max(...latencies) : null;
}

// `graphStale`: the registry graph is last-known (data/freshness.ts), so it
// says who held a slot but not who is alive now.
export function mergeProcesses(
    manifest: SessionManifest|null, graph: SystemGraph|null,
    declared: DeclaredGraph|null,
    rates: Record<string, number>, graphStale: boolean): ProcessRow[] {
  interface Parts {
    manifest?: ManifestNode;
    registry?: SystemNode;
    declared?: {name: string; target: string};
  }
  const byName = new Map<string, Parts>();

  for (const n of manifest?.nodes ?? []) {
    const p = byName.get(n.name) ?? {};
    p.manifest = n;
    byName.set(n.name, p);
  }
  for (const n of graph?.nodes ?? []) {
    const p = byName.get(n.name) ?? {};
    p.registry = n;
    byName.set(n.name, p);
  }
  for (const n of declared?.nodes ?? []) {
    const p = byName.get(n.name) ?? {};
    p.declared = n;
    byName.set(n.name, p);
  }

  const rows: ProcessRow[] = [];
  for (const [name, {manifest: m, registry: r, declared: d}] of byName) {
    const problems: ProcessProblem[] = [];
    if (r && !graphStale && !r.alive) problems.push('dead');
    if (m && m.restart_count > 0) problems.push('restarted');
    if (m && m.recorder_failed) problems.push('recorder_failed');
    if (m && m.exit_code > 0) problems.push('nonzero_exit');
    if (m && m.exit_signal > 0) problems.push('killed_by_signal');
    // Declared but never registered is only a claim when there is a registry
    // to compare against; with no live graph it cannot be made (the same rule
    // `declaredStatus` follows).
    if (graph && d && !r) problems.push('never_registered');

    rows.push({
      name,
      target: m?.target ?? r?.target ?? d?.target ?? '',
      pid: r ? r.pid : (m ? String(m.pid) : null),
      alive: r && !graphStale ? r.alive : null,
      hasManifest: m !== undefined,
      restarts: m?.restart_count ?? 0,
      exitCode: m && m.exit_code >= 0 ? m.exit_code : null,
      exitSignal: m && m.exit_signal > 0 ? signalText(m.exit_signal) : null,
      recorderFailed: m?.recorder_failed ?? false,
      records: m ? m.record_count : null,
      loopRate: r ? loopRate(r, rates) : null,
      worstLatency: r ? worstLatency(r) : null,
      logPath: m ? (m.log_path || m.log_paths[0] || null) : null,
      problems
    });
  }

  // Problems first, then alphabetical, so the list reads "what needs looking
  // at" before "what is fine".
  rows.sort((a, b) => {
    const pa = a.problems.length === 0 ? 1 : 0;
    const pb = b.problems.length === 0 ? 1 : 0;
    if (pa !== pb) return pa - pb;
    return a.name.localeCompare(b.name);
  });
  return rows;
}

export function summarize(rows: readonly ProcessRow[]): ProcessSummary {
  let running = 0;
  let restarts = 0;
  for (const row of rows) {
    if (row.alive === true) running++;
    restarts += row.restarts;
  }
  return {running, total: rows.length, restarts};
}
