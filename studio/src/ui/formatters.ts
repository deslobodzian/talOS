// Shared formatting for the panel views. Moved out of the old system_view so
// every panel shows the same "5 µs ago" and "200 Hz" rather than drifting.

import type {SystemGraph, SystemNode} from '../system';

// A uint64 counter or timestamp carried as a decimal string (or an already-safe
// number) with thousands separators, so "1,204,338" not "1204338".
export const integer = (value: string|number) => {
  const n =
      typeof value === 'string' ? BigInt(value) : BigInt(Math.round(value));
  return n.toLocaleString('en-US');
};

// Robot timing lives in microseconds and milliseconds; a bare nanosecond count
// is unreadable at a glance and that is the only glance a viewer gets.
export function duration(nanos: string|number): string {
  const ns = typeof nanos === 'string' ? Number(nanos) : nanos;
  if (!Number.isFinite(ns)) return '—';
  const magnitude = Math.abs(ns);
  if (magnitude === 0) return '0 µs';
  if (magnitude < 1e3) return `${ns.toFixed(0)} ns`;
  if (magnitude < 1e6) return `${(ns / 1e3).toFixed(1)} µs`;
  if (magnitude < 1e9) return `${(ns / 1e6).toFixed(2)} ms`;
  return `${(ns / 1e9).toFixed(2)} s`;
}

// The same duration formatting for a value already in microseconds (the unit
// /hw/status and the bridge counters report), so callers do not convert to
// nanoseconds by hand: `durationUs(3500)` is `3.50 ms`.
export function durationUs(micros: string|number): string {
  const us = typeof micros === 'string' ? Number(micros) : micros;
  if (!Number.isFinite(us)) return '—';
  return duration(us * 1000);
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

// A telemetry sample: enough digits to be useful, never exponential noise.
export const number = (value: number, places = 4): string =>
    Number.isFinite(value) ? value.toFixed(places) : '—';

// A nanosecond timestamp, thousands-separated, the rule every uint64 on the
// wire follows.
export const timestamp = (ns: string|null|undefined): string =>
    ns == null ? '—' : integer(ns) + ' ns';

// A wall-clock nanosecond time as the viewer's local date and time to the
// millisecond.
export function wallDateTime(ns: string|bigint): string {
  const date = new Date(Number(BigInt(ns) / 1000000n));
  if (Number.isNaN(date.getTime())) return '—';
  const pad = (n: number, width = 2) => String(n).padStart(width, '0');
  return `${date.getFullYear()}-${pad(date.getMonth() + 1)}-${pad(date.getDate())} ` +
      `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}.` +
      pad(date.getMilliseconds(), 3);
}

// Nanoseconds since a session's start as m:ss.mmm.
export function sessionTime(ns: bigint): string {
  const sign = ns < 0n ? '-' : '';
  const ms = (ns < 0n ? -ns : ns) / 1000000n;
  const minutes = ms / 60000n, seconds = (ms / 1000n) % 60n, rest = ms % 1000n;
  return `${sign}${minutes}:${String(seconds).padStart(2, '0')}.${
      String(rest).padStart(3, '0')}`;
}
