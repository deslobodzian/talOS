import {poseError} from './core';

// Frame archives are local, untrusted files; none of these objects enter the
// telemetry worker or the bridge. Keep the timestamp lossless (JSON's integer
// can exceed Number.MAX_SAFE_INTEGER).
export type FrameReason = 'sampled'|'rejected';
type FramePose = {x_m: number; y_m: number; yaw_rad: number};
export type FrameDetection = {
  tag_id: number;
  hamming: number;
  decision_margin: number;
  corners_px: number[];
  valid: boolean;
  reprojection_error_px: number;
  ambiguity: number;
  reject_reason: string|null;
};
export type FrameMetadata = {
  format_version: 1;
  camera: string;
  capture_ns: string;
  reason: FrameReason;
  width: number;
  height: number;
  intrinsics: Record<'fx'|'fy'|'cx'|'cy'|'k1'|'k2'|'k3'|'p1'|'p2', number>;
  mounting: Record<'x_m'|'y_m'|'z_m'|'roll_rad'|'pitch_rad'|'yaw_rad', number>;
  detector: {
    backend: string; decimate: number; threads: number; sigma: number;
    refine_edges: boolean; hamming_threshold: number; min_decision_margin: number;
  };
  detections: FrameDetection[];
  observation: FramePose & {
    valid: boolean; x_std_m: number; y_std_m: number; yaw_std_rad: number;
    tag_count: number;
  };
  truth: FramePose|null;
};

export const MAX_FRAME_JSON_BYTES = 1 << 20;
export const MAX_FRAME_FILES = 20000;
const MAX_IMAGE_BYTES = 64 << 20;
const fail = (message: string): never => { throw Error(message); };
function object(value: unknown, name: string): Record<string, unknown> {
  if (!value || typeof value !== 'object' || Array.isArray(value))
    fail(`${name} must be an object`);
  return value as Record<string, unknown>;
}
function number(source: Record<string, unknown>, key: string): number {
  const value = source[key];
  if (typeof value !== 'number' || !Number.isFinite(value))
    fail(`${key} must be a finite number`);
  return value as number;
}
function integer(source: Record<string, unknown>, key: string, max = 0x7fffffff): number {
  const value = number(source, key);
  if (!Number.isSafeInteger(value) || value < 0 || value > max)
    fail(`${key} must be an integer from 0 to ${max}`);
  return value;
}
function string(source: Record<string, unknown>, key: string): string {
  const value = source[key];
  if (typeof value !== 'string' || !value.length || value.length > 512)
    fail(`${key} must be a nonempty string of at most 512 characters`);
  return value as string;
}
function boolean(source: Record<string, unknown>, key: string): boolean {
  if (typeof source[key] !== 'boolean') fail(`${key} must be a boolean`);
  return source[key] as boolean;
}
function numbers<K extends string>(source: Record<string, unknown>, keys: readonly K[]): Record<K, number> {
  return Object.fromEntries(keys.map(key => [key, number(source, key)])) as Record<K, number>;
}
function pose(value: unknown, name: string): FramePose {
  return numbers(object(value, name), ['x_m', 'y_m', 'yaw_rad']);
}

// JSON.parse validates the syntax first. Tokenize only to recover the exact
// top-level integer spelling; strings (including escaped keys) are indivisible.
// Last key wins, just as it does in JSON.parse. No eval or lossy timestamp math.
function captureTimestamp(text: string): string {
  const tokens = text.match(/"(?:[^"\\]|\\.)*"|-?(?:0|[1-9]\d*)(?:\.\d+)?(?:[eE][+-]?\d+)?|[{}\[\]:,]/g) ?? [];
  let depth = 0;
  let value = '';
  for (let i = 0; i < tokens.length; ++i) {
    const token = tokens[i];
    if (depth === 1 && token.startsWith('"') && tokens[i + 1] === ':' &&
        JSON.parse(token) === 'capture_ns') value = tokens[i + 2] ?? '';
    if (token === '{' || token === '[') ++depth;
    if (token === '}' || token === ']') --depth;
  }
  if (!/^\d{1,20}$/.test(value) || BigInt(value) > 18446744073709551615n)
    fail('capture_ns must be an unsigned 64-bit JSON integer');
  return BigInt(value).toString();
}

export function parseFrameMetadata(text: string): FrameMetadata {
  try {
    if (text.length > MAX_FRAME_JSON_BYTES) fail('document too large');
    const root = object(JSON.parse(text), 'document');
    if (root.format_version !== 1) fail('unsupported format_version (expected 1)');
    if (typeof root.capture_ns !== 'number') fail('capture_ns must be a JSON integer');
    const capture_ns = captureTimestamp(text);
    const reason = string(root, 'reason');
    if (reason !== 'sampled' && reason !== 'rejected') fail('reason must be sampled or rejected');
    const width = integer(root, 'width', 32768), height = integer(root, 'height', 32768);
    if (!width || !height || width * height > 64 * 1024 * 1024) fail('image dimensions too large or zero');
    const detector = object(root.detector, 'detector');
    const observation = object(root.observation, 'observation');
    if (!Array.isArray(root.detections) || root.detections.length > 1024)
      fail('detections must be an array of at most 1024 entries');
    const detections = (root.detections as unknown[]).map((value, i) => {
      const entry = object(value, `detections[${i}]`);
      if (!Array.isArray(entry.corners_px) || entry.corners_px.length !== 8 ||
          !entry.corners_px.every(v => typeof v === 'number' && Number.isFinite(v)))
        fail(`detections[${i}].corners_px must contain eight finite numbers`);
      const reject_reason = entry.reject_reason === null ? null : string(entry, 'reject_reason');
      return {
        tag_id: integer(entry, 'tag_id'), hamming: integer(entry, 'hamming'),
        decision_margin: number(entry, 'decision_margin'),
        corners_px: [...entry.corners_px as number[]], valid: boolean(entry, 'valid'),
        reprojection_error_px: number(entry, 'reprojection_error_px'),
        ambiguity: number(entry, 'ambiguity'), reject_reason
      };
    });
    return {
      format_version: 1, camera: string(root, 'camera'), capture_ns,
      reason: reason as FrameReason, width, height,
      intrinsics: numbers(object(root.intrinsics, 'intrinsics'), ['fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'k3', 'p1', 'p2']),
      mounting: numbers(object(root.mounting, 'mounting'), ['x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad', 'yaw_rad']),
      detector: {
        backend: string(detector, 'backend'), decimate: number(detector, 'decimate'),
        threads: integer(detector, 'threads'), sigma: number(detector, 'sigma'),
        refine_edges: boolean(detector, 'refine_edges'),
        hamming_threshold: integer(detector, 'hamming_threshold'),
        min_decision_margin: number(detector, 'min_decision_margin')
      },
      detections,
      observation: {
        ...pose(observation, 'observation'), valid: boolean(observation, 'valid'),
        ...numbers(observation, ['x_std_m', 'y_std_m', 'yaw_std_rad']),
        tag_count: integer(observation, 'tag_count')
      },
      truth: root.truth === null ? null : pose(root.truth, 'truth')
    };
  } catch (error) {
    throw Error(`Invalid frame metadata: ${error instanceof Error ? error.message : String(error)}`);
  }
}

export type LocalFrameFile = {
  name: string; webkitRelativePath?: string; size: number; text(): Promise<string>;
};
export type RecordedFrame<T extends LocalFrameFile = File> = {
  id: string; metadata: FrameMetadata; image: T;
};
export type FrameFileError = {file: string; message: string};
export type FrameArchive<T extends LocalFrameFile = File> = {
  frames: RecordedFrame<T>[]; errors: FrameFileError[]; unpaired: number;
};

export async function loadFrameFiles<T extends LocalFrameFile>(files: readonly T[]): Promise<FrameArchive<T>> {
  const result: FrameArchive<T> = {frames: [], errors: [], unpaired: 0};
  if (files.length > MAX_FRAME_FILES) {
    result.errors.push({file: 'Selection', message: `Choose at most ${MAX_FRAME_FILES} files`});
    return result;
  }
  const pairs = new Map<string, {png?: T; json?: T; duplicate?: boolean}>();
  for (const file of files) {
    const path = file.webkitRelativePath || file.name;
    const match = /^(.*\/)?(\d{20}_(?:sampled|rejected))\.(png|json)$/.exec(path);
    if (!match) continue;
    const id = `${match[1] ?? ''}${match[2]}`;
    const pair = pairs.get(id) ?? {};
    const extension = match[3] as 'png'|'json';
    if (pair[extension]) {
      pair.duplicate = true;
      result.errors.push({file: path, message: 'Duplicate filename; choose a directory to preserve camera paths'});
    }
    pair[extension] = file;
    pairs.set(id, pair);
  }
  // Sequential metadata reads avoid thousands of simultaneous File.text calls.
  for (const [id, pair] of pairs) {
    if (pair.duplicate) continue;
    if (!pair.png || !pair.json) { ++result.unpaired; continue; }
    const path = pair.json.webkitRelativePath || pair.json.name;
    try {
      if (pair.json.size > MAX_FRAME_JSON_BYTES) fail('metadata file too large');
      if (pair.png.size > MAX_IMAGE_BYTES) fail('PNG file too large');
      const metadata = parseFrameMetadata(await pair.json.text());
      const stem = `${metadata.capture_ns.padStart(20, '0')}_${metadata.reason}`;
      if (id.split('/').at(-1) !== stem) fail('filename does not match capture_ns and reason');
      result.frames.push({id, metadata, image: pair.png});
    } catch (error) {
      result.errors.push({file: path, message: error instanceof Error ? error.message : String(error)});
    }
  }
  result.frames.sort((a, b) => {
    const left = BigInt(a.metadata.capture_ns), right = BigInt(b.metadata.capture_ns);
    return left < right ? -1 : left > right ? 1 : a.id.localeCompare(b.id);
  });
  return result;
}

export function filterFrames<T extends LocalFrameFile>(frames: readonly RecordedFrame<T>[], camera = '', reason: FrameReason|'' = ''): RecordedFrame<T>[] {
  return frames.filter(frame => (!camera || frame.metadata.camera === camera) &&
      (!reason || frame.metadata.reason === reason));
}

export function framePoseError(frame: FrameMetadata): ReturnType<typeof poseError>|null {
  if (!frame.truth || !frame.observation.valid) return null;
  const planar = (p: FramePose) => ({x: p.x_m, y: p.y_m, yaw: p.yaw_rad});
  return poseError(planar(frame.observation), planar(frame.truth));
}
