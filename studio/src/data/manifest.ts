// The launcher's live session manifest, served by the bridge at GET
// /manifest.json (404 when the bridge was started without --manifest, which is
// the ordinary case: a session can run without one, so "not offered" is not an
// error). Parsed as strictly as system.ts; uint64 values are decimal strings,
// and the exact spelling is recovered from the raw text because the launcher
// writes them as JSON numbers, which JSON.parse would round past 2^53.

import {bool, boundedString, fail, flag, list, parsing, record} from './validate';

// One run of a node (launcher README "Session manifest"). Wall times are
// nanoseconds as a double: rounded below a microsecond, which a display of
// when a run started and ended never shows.
export type ManifestRun = {
  pid: number;
  start_wall_ns: number;
  end_wall_ns: number;  // 0 while it runs
  exit_code: number;
  exit_signal: number;
  // 'running', 'exit', 'crash', 'stalled', 'heartbeat_missing',
  // 'construction_deadline', 'endpoint_mismatch', 'stop', 'stop_timeout',
  // 'spawn_failed'.
  reason: string;
  // The launcher's own stop signal ended the run, which leaves the session's
  // exit status 0; absent in manifests written before the launcher recorded
  // it.
  stopped_by_session?: boolean;
};

// What a node's registry rows said it lost in delivery, summed over its runs
// (REQ-OPS-43; talOS/introspection README "Wake loss"). Each should stay 0;
// `session_report --strict` fails a session with any of them nonzero.
export type ManifestDelivery = {
  wake_failures: number;
  stalled_arms: number;
  wake_fallbacks: number;
  // The bridge's requests its latest-value read passed over, in total and
  // per subsystem; 0 and empty for every other node.
  requests_skipped: number;
  requests_skipped_by_subsystem: Record<string, number>;
};

export type ManifestNode = {
  name: string;
  target: string;
  // Whether the launcher supervises this member as critical (absent in legacy
  // manifests, so consumers do not mistake an absent key for non-critical).
  critical?: boolean;
  // Whether this member is the launcher-added Studio bridge (absent in legacy
  // manifests).
  studio_bridge?: boolean;
  binary_path: string;
  // Absent in manifests written before the launcher recorded them.
  binary_sha256?: string;
  runs?: ManifestRun[];
  runs_dropped?: number;
  log_path: string;
  log_paths: string[];
  pid: number;
  exit_code: number;
  // The signal that ended the last run, else 0 (exit_code is -1 then).
  exit_signal: number;
  restart_count: number;
  recorder_failed: boolean;
  // The run's event log has a marked gap: it dropped records and capture
  // resumed (REQ-OPS-41); recorder_failed is set too. Absent in manifests
  // written before the launcher recorded it, which parse as false.
  log_gap?: boolean;
  described: boolean;
  // A run's live endpoints differed from its --describe probe (the launcher
  // README's "Live endpoints").
  endpoint_mismatch: boolean;
  // Absent when the launcher never saw a run's registry row (so nothing was
  // watched, which is not "nothing lost"), and in manifests written before
  // the launcher recorded it.
  delivery?: ManifestDelivery;
  record_count: string;
};

export type SessionManifest = {
  session_id: string;
  config_path: string;
  config_sha256: string;
  // A --config=release bundle's source revision; absent in older manifests.
  git_revision?: string;
  git_dirty?: boolean;
  output_dir: string;
  simulation: boolean;
  start_wall_ns: string;
  end_wall_ns: string;
  // Lock anomalies recorded during supervision (absent in legacy manifests).
  session_lock_lost?: boolean;
  directory_lock_lost?: boolean;
  directory_lock_retaken?: boolean;
  nodes: ManifestNode[];
};

// Matches studio/review/review.h kMaxManifestBytes (4 MiB).
export const MAX_MANIFEST_BYTES = 4 << 20;

const MAX_NODES = 256;
const MAX_STRING = 512;
const MAX_PATH = 4096;
const MAX_LOG_PATHS = 64;
const MAX_RUNS = 64;
const MAX_SUBSYSTEMS = 256;

const str = (source: Record<string, unknown>, key: string, cap = MAX_STRING) =>
    boundedString(source, key, cap);

const path = (source: Record<string, unknown>, key: string) =>
    str(source, key, MAX_PATH);

function integer(source: Record<string, unknown>, key: string): number {
  const value = source[key];
  if (typeof value !== 'number' || !Number.isInteger(value) ||
      value < -2147483648 || value > 2147483647)
    fail(`${key} must be a 32-bit integer`);
  return value as number;
}

// JSON.parse validates the syntax first. Scan once to recover the exact decimal
// spelling of top-level integer tokens before JSON.parse rounds past 2^53 (last
// key wins, matching JSON.parse). Strings are indivisible and nested objects /
// arrays are skipped so field names inside string values (e.g. config_path) or
// node run histories never match.
function topLevelIntegers(raw: string): Map<string, string> {
  const out = new Map<string, string>();
  const n = raw.length;
  let i = 0;
  const skipWs = () => {
    while (i < n && (raw[i] === ' ' || raw[i] === '\t' || raw[i] === '\n' || raw[i] === '\r')) i++;
  };
  const readString = (): string => {
    const start = i++;  // opening '"'
    while (i < n) {
      const ch = raw[i++];
      if (ch === '\\') { i++; continue; }
      if (ch === '"') return raw.slice(start, i);
    }
    return raw.slice(start);
  };
  const skipValue = () => {
    skipWs();
    if (i >= n) return;
    const ch = raw[i];
    if (ch === '"') {
      readString();
    } else if (ch === '{' || ch === '[') {
      let depth = 0;
      while (i < n) {
        const c = raw[i];
        if (c === '"') { readString(); continue; }
        i++;
        if (c === '{' || c === '[') depth++;
        else if (c === '}' || c === ']') {
          depth--;
          if (depth === 0) return;
        }
      }
    } else {
      while (i < n && raw[i] !== ',' && raw[i] !== '}' && raw[i] !== ']' &&
             raw[i] !== ' ' && raw[i] !== '\t' && raw[i] !== '\n' && raw[i] !== '\r') i++;
    }
  };
  skipWs();
  if (raw[i] !== '{') return out;
  i++;
  while (i < n) {
    skipWs();
    if (raw[i] === '}') break;
    if (raw[i] === ',') { i++; continue; }
    if (raw[i] !== '"') break;
    const key = JSON.parse(readString()) as string;
    skipWs();
    if (raw[i] !== ':') break;
    i++;
    skipWs();
    const valStart = i;
    if (raw[i] === '-' || (raw[i] >= '0' && raw[i] <= '9')) {
      skipValue();
      const token = raw.slice(valStart, i);
      if (/^-?(?:0|[1-9]\d*)$/.test(token)) out.set(key, token);
      else out.delete(key);
    } else {
      out.delete(key);
      skipValue();
    }
  }
  return out;
}

const MAX_U64 = 18446744073709551615n;
const MIN_I64 = -9223372036854775808n;

// A uint64/int64 field kept as a string. Prefers the exact raw spelling, and
// falls back to a string or safe-integer number field for documents that
// already carry it as a string.
function u64Field(integers: ReadonlyMap<string, string>,
                  source: Record<string, unknown>, key: string): string {
  const exact = integers.get(key);
  if (exact !== undefined) {
    if (/^-?\d{1,20}$/.test(exact)) {
      const n = BigInt(exact);
      if (n >= MIN_I64 && n <= MAX_U64) return exact;
    }
    fail(`${key} must be a decimal integer`);
  }
  const value = source[key];
  if (typeof value === 'string' && /^-?\d{1,20}$/.test(value)) {
    const n = BigInt(value);
    if (n >= MIN_I64 && n <= MAX_U64) return value;
  }
  if (typeof value === 'number' && Number.isSafeInteger(value)) return String(value);
  fail(`${key} must be a decimal integer`);
  return '';
}

// A non-negative count that is 0 when absent.
function countNumber(source: Record<string, unknown>, key: string): number {
  const value = source[key] ?? 0;
  if (typeof value !== 'number' || !Number.isSafeInteger(value) || value < 0)
    fail(`${key} must be a non-negative integer`);
  return value as number;
}

// A nested uint64 (per-node record_count), too small to need raw-text recovery.
function countString(source: Record<string, unknown>, key: string): string {
  const value = source[key];
  if (typeof value === 'string' && /^\d{1,20}$/.test(value) && BigInt(value) <= MAX_U64)
    return value;
  if (typeof value === 'number' && Number.isSafeInteger(value) && value >= 0)
    return String(value);
  fail(`${key} must be a non-negative integer`);
  return '';
}

// Wall-clock nanoseconds as a JSON number (~1.7e18 exceeds MAX_SAFE_INTEGER, so
// bound against uint64 max rather than MAX_SAFE_INTEGER).
const MAX_WALL_NS = 18446744073709551615;
function wallNs(source: Record<string, unknown>, key: string): number {
  const value = source[key];
  if (typeof value !== 'number' || !Number.isFinite(value) ||
      !Number.isInteger(value) || value < 0 || value > MAX_WALL_NS)
    fail(`${key} must be a non-negative integer`);
  return value as number;
}

function parseRun(value: unknown, where: string): ManifestRun {
  const source = record(value, where);
  return {
    pid: integer(source, 'pid'),
    start_wall_ns: wallNs(source, 'start_wall_ns'),
    end_wall_ns: wallNs(source, 'end_wall_ns'),
    exit_code: integer(source, 'exit_code'),
    exit_signal: integer(source, 'exit_signal'),
    reason: str(source, 'reason'),
    ...('stopped_by_session' in source ?
            {stopped_by_session: bool(source, 'stopped_by_session')} : {})
  };
}

// A count the launcher always writes once it writes any of them.
function count(source: Record<string, unknown>, key: string): number {
  const value = source[key];
  if (typeof value !== 'number' || !Number.isSafeInteger(value) || value < 0)
    fail(`${key} must be a non-negative integer`);
  return value as number;
}

// The launcher writes all five fields or none (talOS/launcher/session.cc).
function parseDelivery(source: Record<string, unknown>,
                       where: string): ManifestDelivery|undefined {
  if (!('wake_failures' in source)) return undefined;
  const split = record(source['requests_skipped_by_subsystem'],
                       `${where}.requests_skipped_by_subsystem`);
  const subsystems = Object.keys(split);
  if (subsystems.length > MAX_SUBSYSTEMS)
    fail(`requests_skipped_by_subsystem must hold at most ${MAX_SUBSYSTEMS} entries`);
  // fromEntries defines each key as its own property, so no key (not even
  // "__proto__") reaches the object's prototype.
  const bySubsystem: Record<string, number> = Object.fromEntries(
      subsystems.map(subsystem => {
        if (subsystem.length > MAX_STRING)
          fail(`requests_skipped_by_subsystem key must be at most ${MAX_STRING} characters`);
        return [subsystem, count(split, subsystem)];
      }));
  return {
    wake_failures: count(source, 'wake_failures'),
    stalled_arms: count(source, 'stalled_arms'),
    wake_fallbacks: count(source, 'wake_fallbacks'),
    requests_skipped: count(source, 'requests_skipped'),
    requests_skipped_by_subsystem: bySubsystem
  };
}

function parseNode(value: unknown, where: string): ManifestNode {
  const source = record(value, where);
  const delivery = parseDelivery(source, where);
  const runs = 'runs' in source ?
      list(source, 'runs', MAX_RUNS).map(
          (entry, i) => parseRun(entry, `${where}.runs[${i}]`)) :
      undefined;
  return {
    name: str(source, 'name'),
    target: str(source, 'target'),
    ...('critical' in source ? {critical: bool(source, 'critical')} : {}),
    ...('studio_bridge' in source ?
            {studio_bridge: bool(source, 'studio_bridge')} : {}),
    binary_path: path(source, 'binary_path'),
    ...('binary_sha256' in source ?
            {binary_sha256: str(source, 'binary_sha256')} : {}),
    ...(runs ? {runs, runs_dropped: countNumber(source, 'runs_dropped')} : {}),
    log_path: path(source, 'log_path'),
    log_paths: list(source, 'log_paths', MAX_LOG_PATHS).map((entry, i) => {
      if (typeof entry !== 'string' || entry.length > MAX_PATH)
        fail(`${where}.log_paths[${i}] must be a path`);
      return entry as string;
    }),
    pid: integer(source, 'pid'),
    exit_code: integer(source, 'exit_code'),
    // Absent in manifests written before the launcher recorded it.
    exit_signal: 'exit_signal' in source ? integer(source, 'exit_signal') : 0,
    restart_count: integer(source, 'restart_count'),
    recorder_failed: flag(source, 'recorder_failed'),
    log_gap: flag(source, 'log_gap'),
    described: flag(source, 'described'),
    endpoint_mismatch: flag(source, 'endpoint_mismatch'),
    ...(delivery ? {delivery} : {}),
    record_count: countString(source, 'record_count')
  };
}

export function parseManifest(raw: string): SessionManifest {
  if (raw.length > MAX_MANIFEST_BYTES) fail('document too large');
  return parsing('manifest', () => {
    const source = record(JSON.parse(raw), 'manifest');
    const integers = topLevelIntegers(raw);
    return {
      session_id: u64Field(integers, source, 'session_id'),
      config_path: path(source, 'config_path'),
      config_sha256: path(source, 'config_sha256'),
      ...('git_revision' in source ?
              {git_revision: str(source, 'git_revision'),
               git_dirty: flag(source, 'git_dirty')} : {}),
      output_dir: path(source, 'output_dir'),
      simulation: bool(source, 'simulation'),
      start_wall_ns: u64Field(integers, source, 'start_wall_ns'),
      end_wall_ns: u64Field(integers, source, 'end_wall_ns'),
      ...('session_lock_lost' in source ?
              {session_lock_lost: bool(source, 'session_lock_lost')} : {}),
      ...('directory_lock_lost' in source ?
              {directory_lock_lost: bool(source, 'directory_lock_lost')} : {}),
      ...('directory_lock_retaken' in source ?
              {directory_lock_retaken: bool(source, 'directory_lock_retaken')} : {}),
      nodes: list(source, 'nodes', MAX_NODES).map(
          (entry, i) => parseNode(entry, `nodes[${i}]`))
    };
  });
}

// Where to ask for the manifest, given the telemetry socket's URL. The bridge
// serves it from the same origin, so the http base is the ws base with the
// scheme and path swapped; null when the URL is not a WebSocket URL.
export function manifestUrl(socketUrl: string): string|null {
  try {
    const target = new URL(socketUrl);
    if (target.protocol !== 'ws:' && target.protocol !== 'wss:') return null;
    target.protocol = target.protocol === 'wss:' ? 'https:' : 'http:';
    target.pathname = '/manifest.json';
    target.search = '';
    target.hash = '';
    return target.toString();
  } catch {
    return null;
  }
}
