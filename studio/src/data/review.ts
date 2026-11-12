// The review server's API (`studio_review`, studio/review, default
// http://127.0.0.1:5805, or the page's own origin when it served Studio):
// parsers for its JSON documents and the decoder of its binary series. The
// shapes are pinned by studio/review/testdata/golden (tests/review_golden). Every parser is strict and throws a `ReviewError` naming
// what is wrong, so a malformed payload becomes an error state in the
// RecordedSource rather than a crash in a panel.
//
// 64-bit nanosecond values follow the bridge's convention: a decimal string
// (data/types.ts), with an integer JSON number also accepted.

import type {ChannelInfo} from './types';

export const DEFAULT_REVIEW_URL = 'http://127.0.0.1:5805';

export class ReviewError extends Error {}

const fail = (what: string): never => {
  throw new ReviewError(what);
};

// `fetch` as Studio calls it; tests pass a fake.
export type FetchLike = (url: string, init?: {signal?: AbortSignal}) => Promise<Response>;

export type ReviewSession = {
  id: string;
  start_wall_ns: string;
  end_wall_ns: string;
  class: string;
  files: {path: string; bytes: number}[];
};

// Event-time bounds of a session, and the wall time its clock's origin maps
// to (see wallTimeNs).
// The session's bounds, and the server's series limits (studio/review/review.h
// kMaxSeriesValues, kMaxSeriesChannels): a series request asks for at most
// max_series_values channels x points over at most max_series_channels.
export type ReviewBounds = {start_ns: string; end_ns: string; origin_wall_ns: string;
                            max_series_values: number; max_series_channels: number};

export type ReviewSample = {value: number; exact?: string; sample_ns: string};
export type ReviewSnapshot = {t_ns: string; channels: Map<string, ReviewSample>};

// One channel's decimated columns, index-aligned: `count[i]` samples fell in
// bucket i, which spans min[i]..max[i] and ends at last[i], published at
// t_ns[i] (NaN when empty). Every channel shares the bucket edges, not the
// times: each channel's t_ns is its own last sample's.
export type SeriesColumns = {
  t_ns: Float64Array;
  min: Float64Array;
  max: Float64Array;
  last: Float64Array;
  count: Uint32Array;
};
export type SeriesBuckets = {buckets: number; channels: SeriesColumns[]};

function record(value: unknown, what: string): Record<string, unknown> {
  if (!value || typeof value !== 'object' || Array.isArray(value))
    fail(`${what} must be an object`);
  return value as Record<string, unknown>;
}

function str(source: Record<string, unknown>, key: string, where: string): string {
  const value = source[key];
  if (typeof value !== 'string') fail(`${where}.${key} must be a string`);
  return value as string;
}

export function u64(value: unknown, where: string): string {
  if (typeof value === 'string' && /^\d{1,20}$/.test(value))
    return BigInt(value).toString();
  if (typeof value === 'number' && Number.isInteger(value) && value >= 0)
    return BigInt(value).toString();
  return fail(`${where} must be a non-negative integer`);
}

export function parseJson(text: string, what: string): unknown {
  try {
    return JSON.parse(text);
  } catch {
    return fail(`${what} is not JSON`);
  }
}

export function parseSessions(raw: unknown): ReviewSession[] {
  if (!Array.isArray(raw)) fail('sessions.json must be an array');
  return (raw as unknown[]).map((entry, i) => {
    const where = `sessions[${i}]`;
    const source = record(entry, where);
    const files = source.files;
    if (!Array.isArray(files)) fail(`${where}.files must be an array`);
    return {
      id: typeof source.id === 'number' ? u64(source.id, `${where}.id`) :
                                          str(source, 'id', where),
      start_wall_ns: u64(source.start_wall_ns, `${where}.start_wall_ns`),
      end_wall_ns: u64(source.end_wall_ns, `${where}.end_wall_ns`),
      class: str(source, 'class', where),
      files: (files as unknown[]).map((file, j) => {
        const f = record(file, `${where}.files[${j}]`);
        const bytes = f.bytes;
        if (typeof bytes !== 'number' || !Number.isInteger(bytes) || bytes < 0)
          fail(`${where}.files[${j}].bytes must be a non-negative integer`);
        return {path: str(f, 'path', `${where}.files[${j}]`), bytes: bytes as number};
      })
    };
  });
}

// The bridge's /channels.json shape; the review server adds "alert", which no
// panel reads yet.
export function parseChannels(raw: unknown): ChannelInfo[] {
  const source = record(raw, 'channels.json');
  const channels = source.channels;
  if (!Array.isArray(channels)) fail('channels.json channels must be an array');
  return (channels as unknown[]).map((entry, i) => {
    const where = `channels[${i}]`;
    const c = record(entry, where);
    const labels = c.labels ?? [];
    if (!Array.isArray(labels) || labels.some(l => typeof l !== 'string'))
      fail(`${where}.labels must be an array of strings`);
    return {
      name: str(c, 'name', where),
      unit: str(c, 'unit', where),
      labels: labels as string[]
    };
  });
}

function limit(value: unknown, where: string): number {
  if (typeof value !== 'number' || !Number.isSafeInteger(value) || value < 1)
    fail(`${where} must be a positive integer`);
  return value as number;
}

export function parseBounds(raw: unknown): ReviewBounds {
  const source = record(raw, 'bounds.json');
  const bounds = {
    start_ns: u64(source.start_ns, 'bounds.start_ns'),
    end_ns: u64(source.end_ns, 'bounds.end_ns'),
    origin_wall_ns: u64(source.origin_wall_ns, 'bounds.origin_wall_ns'),
    max_series_values: limit(source.max_series_values, 'bounds.max_series_values'),
    max_series_channels: limit(source.max_series_channels, 'bounds.max_series_channels')
  };
  if (BigInt(bounds.end_ns) < BigInt(bounds.start_ns))
    fail('bounds.end_ns is before bounds.start_ns');
  return bounds;
}

// A sample as the bridge writes one: a number; "NaN", "Infinity" or
// "-Infinity"; or a decimal string for an integer past 2^53. Null is a
// channel with no reading, and is left out.
function sampleValue(value: unknown, where: string): {value: number; exact?: string}|null {
  if (value === null) return null;
  if (typeof value === 'number') return {value};
  if (value === 'NaN') return {value: NaN};
  if (value === 'Infinity') return {value: Infinity};
  if (value === '-Infinity') return {value: -Infinity};
  if (typeof value === 'string' && /^-?\d{1,20}$/.test(value))
    return {value: Number(value), exact: BigInt(value).toString()};
  return fail(`${where} must be a number`);
}

export function parseSnapshot(raw: unknown): ReviewSnapshot {
  const source = record(raw, 'snapshot');
  const channels = record(source.channels, 'snapshot.channels');
  const out = new Map<string, ReviewSample>();
  for (const [name, entry] of Object.entries(channels)) {
    const where = `snapshot.channels[${JSON.stringify(name)}]`;
    const c = record(entry, where);
    const sample = sampleValue(c.value, `${where}.value`);
    if (!sample) continue;
    out.set(name, {...sample, sample_ns: u64(c.sample_ns, `${where}.sample_ns`)});
  }
  return {t_ns: u64(source.t_ns, 'snapshot.t_ns'), channels: out};
}

// GET /review/<id>/series: little-endian u32 channels, u32 buckets, then per
// channel f64 t_ns[buckets], f64 min[], f64 max[], f64 last[], u32 count[].
// `expected` is the number of channels requested; the response must carry
// exactly that many, in request order, and not one byte more or less.
export function decodeSeries(buffer: ArrayBuffer, expected: number): SeriesBuckets {
  const view = new DataView(buffer);
  if (buffer.byteLength < 8) fail('series: truncated header');
  const channels = view.getUint32(0, true), buckets = view.getUint32(4, true);
  if (channels !== expected)
    fail(`series: ${channels} channels, expected ${expected}`);
  const perChannel = buckets * (4 * 8 + 4);
  if (buffer.byteLength !== 8 + channels * perChannel)
    fail(`series: ${buffer.byteLength} bytes for ${channels} channels of ${buckets} buckets`);
  const out: SeriesColumns[] = [];
  let at = 8;
  const f64 = () => {
    const column = new Float64Array(buckets);
    for (let i = 0; i < buckets; i++, at += 8) column[i] = view.getFloat64(at, true);
    return column;
  };
  for (let c = 0; c < channels; c++) {
    const t_ns = f64(), min = f64(), max = f64(), last = f64();
    const count = new Uint32Array(buckets);
    for (let i = 0; i < buckets; i++, at += 4) count[i] = view.getUint32(at, true);
    out.push({t_ns, min, max, last, count});
  }
  return {buckets, channels: out};
}

// The wall time of an event time: the session clock's origin plus the event
// time.
export function wallTimeNs(bounds: ReviewBounds, t_ns: string): bigint {
  return BigInt(bounds.origin_wall_ns) + BigInt(t_ns);
}

const base = (url: string) => url.replace(/\/+$/, '');

export function reviewUrl(server: string, ...path: string[]): string {
  return [base(server), 'review', ...path.map(encodeURIComponent)].join('/');
}

async function get(fetchFn: FetchLike, url: string, signal?: AbortSignal): Promise<Response> {
  let response: Response;
  try {
    response = await fetchFn(url, signal ? {signal} : undefined);
  } catch (e) {
    if (isAbort(e)) throw e;
    return fail(`${url}: ${e instanceof Error ? e.message : String(e)}`);
  }
  if (!response.ok) fail(`${url}: HTTP ${response.status}`);
  return response;
}

export async function getJson(fetchFn: FetchLike, url: string,
                              signal?: AbortSignal): Promise<unknown> {
  const response = await get(fetchFn, url, signal);
  return parseJson(await response.text(), url);
}

export async function getBytes(fetchFn: FetchLike, url: string,
                               signal?: AbortSignal): Promise<ArrayBuffer> {
  return (await get(fetchFn, url, signal)).arrayBuffer();
}

export function isAbort(e: unknown): boolean {
  return !!e && typeof e === 'object' && (e as {name?: unknown}).name === 'AbortError';
}

export function errorMessage(e: unknown): string {
  return e instanceof Error ? e.message : String(e);
}

export async function listSessions(server: string, fetchFn: FetchLike): Promise<ReviewSession[]> {
  return parseSessions(await getJson(fetchFn, reviewUrl(server, 'sessions.json')));
}

// `origin` when the page was served by a review server (studio_review serves
// Studio itself), found by asking that origin for its session list; null for
// any other server (the live bridge answers 404, a dev server its index
// page), or when the origin is not http(s).
export async function detectReviewServer(origin: string,
                                         fetchFn: FetchLike): Promise<string|null> {
  if (!/^https?:\/\//.test(origin)) return null;
  try {
    await listSessions(origin, fetchFn);
    return origin;
  } catch {
    return null;
  }
}
