// The launcher's live session manifest, served by the bridge at GET
// /manifest.json (404 when the bridge was started without --manifest, which is
// the ordinary case: a session can run without one, so "not offered" is not an
// error). Parsed as strictly as system.ts; uint64 values are decimal strings,
// and the exact spelling is recovered from the raw text because the launcher
// writes them as JSON numbers, which JSON.parse would round past 2^53.

export type ManifestNode = {
  name: string;
  target: string;
  binary_path: string;
  log_path: string;
  log_paths: string[];
  pid: number;
  exit_code: number;
  // The signal that ended the last run, else 0 (exit_code is -1 then).
  exit_signal: number;
  restart_count: number;
  recorder_failed: boolean;
  described: boolean;
  // A run's live endpoints differed from its --describe probe (the launcher
  // README's "Live endpoints").
  endpoint_mismatch: boolean;
  record_count: string;
};

export type SessionManifest = {
  session_id: string;
  config_path: string;
  config_sha256: string;
  output_dir: string;
  simulation: boolean;
  start_wall_ns: string;
  end_wall_ns: string;
  nodes: ManifestNode[];
};

export const MAX_MANIFEST_BYTES = 1 << 20;

const MAX_NODES = 256;
const MAX_STRING = 512;
const MAX_PATH = 4096;
const MAX_LOG_PATHS = 64;

const fail = (what: string): never => {
  throw Error(what);
};

function parsing<T>(body: () => T): T {
  try {
    return body();
  } catch (e) {
    throw Error(`Invalid manifest: ${e instanceof Error ? e.message : String(e)}`);
  }
}

function record(value: unknown, what: string): Record<string, unknown> {
  if (!value || typeof value !== 'object' || Array.isArray(value))
    fail(`${what} must be an object`);
  return value as Record<string, unknown>;
}

function str(source: Record<string, unknown>, key: string,
             cap = MAX_STRING): string {
  const value = source[key];
  if (typeof value !== 'string' || value.length > cap)
    fail(`${key} must be a string of at most ${cap} characters`);
  return value as string;
}

const path = (source: Record<string, unknown>, key: string) =>
    str(source, key, MAX_PATH);

function bool(source: Record<string, unknown>, key: string): boolean {
  const value = source[key];
  if (typeof value !== 'boolean') fail(`${key} must be a boolean`);
  return value as boolean;
}

// Absent is read as false: an older launcher that did not yet write the field
// still produces a readable document.
function flag(source: Record<string, unknown>, key: string): boolean {
  const value = source[key];
  if (value === undefined) return false;
  if (typeof value !== 'boolean') fail(`${key} must be a boolean`);
  return value as boolean;
}

function integer(source: Record<string, unknown>, key: string): number {
  const value = source[key];
  if (typeof value !== 'number' || !Number.isInteger(value) ||
      value < -2147483648 || value > 2147483647)
    fail(`${key} must be a 32-bit integer`);
  return value as number;
}

function list(source: Record<string, unknown>, key: string,
              cap: number): unknown[] {
  const value = source[key];
  if (!Array.isArray(value) || value.length > cap)
    fail(`${key} must be an array of at most ${cap} entries`);
  return value as unknown[];
}

// The exact decimal spelling of a top-level integer field, recovered from the
// raw text before JSON.parse rounds it (the launcher writes uint64 as raw
// numbers). Null when the field is absent or not a plain integer token.
function rawInteger(raw: string, key: string): string|null {
  const match = new RegExp(`"${key}"\\s*:\\s*(-?\\d+)`).exec(raw);
  return match ? match[1] : null;
}

// A uint64/int64 field kept as a string. Prefers the exact raw spelling, and
// falls back to a string or number field for documents that already carry it
// as a string.
function u64Field(raw: string, source: Record<string, unknown>, key: string): string {
  const exact = rawInteger(raw, key);
  if (exact !== null) return exact;
  const value = source[key];
  if (typeof value === 'string' && /^-?\d{1,20}$/.test(value)) return value;
  if (typeof value === 'number' && Number.isInteger(value)) return String(value);
  fail(`${key} must be a decimal integer`);
  return '';
}

// A nested uint64 (per-node record_count), too small to need raw-text recovery.
function countString(source: Record<string, unknown>, key: string): string {
  const value = source[key];
  if (typeof value === 'string' && /^\d{1,20}$/.test(value)) return value;
  if (typeof value === 'number' && Number.isInteger(value) && value >= 0)
    return String(value);
  fail(`${key} must be a non-negative integer`);
  return '';
}

function parseNode(value: unknown, where: string): ManifestNode {
  const source = record(value, where);
  return {
    name: str(source, 'name'),
    target: str(source, 'target'),
    binary_path: path(source, 'binary_path'),
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
    described: flag(source, 'described'),
    endpoint_mismatch: flag(source, 'endpoint_mismatch'),
    record_count: countString(source, 'record_count')
  };
}

export function parseManifest(raw: string): SessionManifest {
  if (raw.length > MAX_MANIFEST_BYTES) fail('document too large');
  return parsing(() => {
    const source = record(JSON.parse(raw), 'manifest');
    return {
      session_id: u64Field(raw, source, 'session_id'),
      config_path: path(source, 'config_path'),
      config_sha256: path(source, 'config_sha256'),
      output_dir: path(source, 'output_dir'),
      simulation: bool(source, 'simulation'),
      start_wall_ns: u64Field(raw, source, 'start_wall_ns'),
      end_wall_ns: u64Field(raw, source, 'end_wall_ns'),
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
