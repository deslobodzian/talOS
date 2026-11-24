// The game pieces the MuJoCo gateway simulates, for the 3D viewport
// (REQ-DEV-43). The bridge sends a programmer client binary `PCS1` messages
// (studio/bridge/README.md "Game pieces"): a keyframe of every piece, then
// only the pieces whose position, in whole millimetres, or in-play flag
// changed. The worker decodes them here (PiecesDecoder), keeps what each
// message changed (PiecesHistory) and posts the pieces as of the time the
// playhead shows: the latest while live, else those of the last message at
// or before the shown frame's time. The viewport turns them into one
// instanced sphere mesh's transforms (pieceMatrices), drawing only pieces in
// play.
//
// Positions are in the field frame the field model uses in the viewport
// (field_model.ts): x along the field from the blue wall, y away from the
// scoring table, z up, metres. The viewport's scene is that frame, so a
// piece is drawn where the gateway's MuJoCo world has it.

export const MAX_PIECES = 640;
const MAGIC = [0x50, 0x43, 0x53, 0x31];  // "PCS1"
const HEADER_BYTES = 24;
const RECORD_BYTES = 16;
const KEYFRAME = 1;
const IN_PLAY = 1;

export type PiecesSnapshot = {
  count: number;
  radius_m: number;
  sample_time_ns: string;
  // x, y, z per piece, metres, `count` pieces.
  positions: Float32Array;
  // 1 while a piece is in play, per piece.
  in_play: Uint8Array;
};

// Whether a binary bridge message is a pieces message (a telemetry page
// starts with its little-endian length, never these four bytes).
export function isPiecesMessage(bytes: Uint8Array): boolean {
  return bytes.length >= 4 && MAGIC.every((b, i) => bytes[i] === b);
}

export class PiecesDecoder {
  private readonly positions = new Float32Array(3 * MAX_PIECES);
  private readonly inPlay = new Uint8Array(MAX_PIECES);
  private readonly seen = new Uint8Array(MAX_PIECES);
  private count = 0;
  private radius = 0;
  private sampleTimeNs = 0n;
  private keyed = false;

  // Forget the view: the next message must be a keyframe (a new socket).
  reset() {
    this.positions.fill(0);
    this.inPlay.fill(0);
    this.count = 0;
    this.radius = 0;
    this.sampleTimeNs = 0n;
    this.keyed = false;
  }

  // Applies one message. False for a delta before any keyframe or against a
  // different count/radius (the bridge starts every client with a keyframe and
  // re-keyframes on count or radius changes, so it is dropped); throws for
  // bytes that are not a whole, well-formed message, leaving the view as it
  // was.
  apply(bytes: Uint8Array): boolean {
    if (!isPiecesMessage(bytes) || bytes.length < HEADER_BYTES)
      throw Error('Not a pieces message');
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const keyframe = (view.getUint8(4) & KEYFRAME) !== 0;
    const records = view.getUint16(6, true);
    const count = view.getUint32(8, true);
    const rawRadius = view.getFloat32(12, true);
    if (bytes.length !== HEADER_BYTES + records * RECORD_BYTES ||
        count > MAX_PIECES || !Number.isFinite(rawRadius) || rawRadius < 0 ||
        records > count)
      throw Error('Malformed pieces message');
    const radius = rawRadius === 0 ? 0 : rawRadius;
    this.seen.fill(0, 0, count);
    for (let r = 0; r < records; r++) {
      const i = view.getUint16(HEADER_BYTES + r * RECORD_BYTES, true);
      if (i >= count)
        throw Error('Pieces message names a piece past its count');
      if (this.seen[i] !== 0)
        throw Error('Duplicate piece index in pieces message');
      this.seen[i] = 1;
    }
    if (!keyframe &&
        (!this.keyed || count !== this.count || radius !== this.radius))
      return false;
    if (keyframe) {
      this.positions.fill(0);
      this.inPlay.fill(0);
    }
    for (let r = 0; r < records; r++) {
      const at = HEADER_BYTES + r * RECORD_BYTES;
      const i = view.getUint16(at, true);
      this.inPlay[i] = (view.getUint8(at + 2) & IN_PLAY) !== 0 ? 1 : 0;
      for (let axis = 0; axis < 3; axis++)
        this.positions[3 * i + axis] = view.getInt32(at + 4 + 4 * axis, true) / 1000;
    }
    this.count = count;
    this.radius = radius;
    this.sampleTimeNs = view.getBigUint64(16, true);
    this.keyed = true;
    return true;
  }

  // The view as of the last message, copied (it is posted to the page).
  snapshot(): PiecesSnapshot|null {
    if (!this.keyed) return null;
    return {
      count: this.count,
      radius_m: this.radius,
      sample_time_ns: this.sampleTimeNs.toString(),
      positions: this.positions.slice(0, 3 * this.count),
      in_play: this.inPlay.slice(0, this.count)
    };
  }
}

// A delta is stored for at most this many records in a row; the next is a
// keyframe. So `at` replays at most KEYFRAME_EVERY - 1 deltas (each of at
// most MAX_PIECES pieces) onto one keyframe.
export const KEYFRAME_EVERY = 64;
// The history's bound, an estimate of its bytes, not a measure of the
// heap it takes (a small delta's typed arrays cost about twice the
// estimate's overhead): past it the oldest records go first, as `trim`
// drops them. Each record is estimated at RECORD_OVERHEAD_BYTES, plus 13 B
// per piece of a keyframe (position and in-play flag) or 15 B per piece a
// delta names (with its index).
export const MAX_HISTORY_BYTES = 16 * 1024 * 1024;
const RECORD_OVERHEAD_BYTES = 256;
const KEYFRAME_PIECE_BYTES = 13;
const DELTA_PIECE_BYTES = 15;
const keyframeBytes = (count: number) =>
    RECORD_OVERHEAD_BYTES + KEYFRAME_PIECE_BYTES * count;

type PiecesRecord = {
  time_ns: bigint;
  count: number;
  radius_m: number;
  // Null for a keyframe: `positions` and `in_play` then hold every piece.
  // Otherwise the pieces this record changed, and their new values.
  indices: Uint16Array|null;
  positions: Float32Array;
  in_play: Uint8Array;
  bytes: number;
};

// The pieces of the running session over time, so the viewport can show
// them at a playhead other than the latest (REQ-DEV-43). `record` is given
// the decoder's view after each message it applied, and stores only the
// pieces that changed since the previous record, with a keyframe of every
// piece at least every KEYFRAME_EVERY records (and whenever the count or
// radius changes, or a delta would be no smaller): memory grows with what
// moved, not with every piece per message.
//
// Times are the messages' sample_time_ns, the gateway's node clock: the
// main processor's monotonic clock a telemetry frame's timestamp_ns is also
// read from (the telemetry node's Node::now()), so `at` takes the time of
// the frame the playhead shows.
export class PiecesHistory {
  private records: PiecesRecord[] = [];
  private bytes = 0;
  // The view as of the newest record, to find what the next one changed.
  private readonly positions = new Float32Array(3 * MAX_PIECES);
  private readonly inPlay = new Uint8Array(MAX_PIECES);
  private readonly changed = new Uint16Array(MAX_PIECES);
  private count = 0;
  private radius = 0;
  // Deltas stored since the newest keyframe.
  private deltas = 0;

  reset() {
    this.records = [];
    this.bytes = 0;
    this.deltas = 0;
  }

  // Records the view `state` (a decoder's, after it applied a message) as of
  // `time_ns`. A time older than the newest record's is another run of the
  // gateway (its clock went back): what was held is forgotten first.
  record(state: PiecesSnapshot, time_ns: bigint) {
    const newest = this.records.at(-1);
    if (newest && time_ns < newest.time_ns) this.reset();
    const count = state.count;
    let keyframe = this.records.length === 0 || count !== this.count ||
        state.radius_m !== this.radius || this.deltas >= KEYFRAME_EVERY - 1;
    let changed = 0;
    if (!keyframe) {
      for (let i = 0; i < count; i++) {
        if (state.in_play[i] !== this.inPlay[i] ||
            state.positions[3 * i] !== this.positions[3 * i] ||
            state.positions[3 * i + 1] !== this.positions[3 * i + 1] ||
            state.positions[3 * i + 2] !== this.positions[3 * i + 2])
          this.changed[changed++] = i;
      }
      keyframe = changed > 0 &&
          changed * DELTA_PIECE_BYTES >= count * KEYFRAME_PIECE_BYTES;
    }
    let record: PiecesRecord;
    if (keyframe) {
      record = {
        time_ns, count, radius_m: state.radius_m, indices: null,
        positions: state.positions.slice(0, 3 * count),
        in_play: state.in_play.slice(0, count),
        bytes: keyframeBytes(count)
      };
      this.deltas = 0;
    } else {
      const indices = this.changed.slice(0, changed);
      const positions = new Float32Array(3 * changed);
      const inPlay = new Uint8Array(changed);
      indices.forEach((piece, j) => {
        positions.set(state.positions.subarray(3 * piece, 3 * piece + 3), 3 * j);
        inPlay[j] = state.in_play[piece];
      });
      record = {
        time_ns, count, radius_m: state.radius_m, indices, positions,
        in_play: inPlay,
        bytes: RECORD_OVERHEAD_BYTES + DELTA_PIECE_BYTES * changed
      };
      this.deltas++;
    }
    this.positions.set(state.positions.subarray(0, 3 * count));
    this.inPlay.set(state.in_play.subarray(0, count));
    this.count = count;
    this.radius = state.radius_m;
    this.records.push(record);
    this.bytes += record.bytes;
    if (this.bytes > MAX_HISTORY_BYTES) this.dropOldest();
  }

  // The pieces as of the last record at or before `t_ns`: what a fresh
  // PiecesDecoder would hold after applying exactly the messages recorded up
  // to then. Null before the first record held.
  at(t_ns: bigint): PiecesSnapshot|null {
    const i = this.indexAt(t_ns);
    return i < 0 ? null : this.materialize(i);
  }

  // The record `at(t_ns)` answers from, as an opaque identity: the same for
  // two times with the same answer, so a caller can skip re-sending it. Null
  // where `at` is.
  recordAt(t_ns: bigint): object|null {
    return this.records[this.indexAt(t_ns)] ?? null;
  }

  // The newest record's identity: the latest view answers from it.
  newest(): object|null {
    return this.records.at(-1) ?? null;
  }

  // Forgets the records before the last one at or before `start_ns` (the
  // retained history's start) and makes that one a keyframe. `at` then
  // answers as before for every time from that record's on, so for
  // `start_ns` and later, and null before it.
  trim(start_ns: bigint) {
    const i = this.indexAt(start_ns);
    if (i > 0) this.dropBefore(i);
  }

  private indexAt(t_ns: bigint): number {
    let lo = 0, hi = this.records.length;
    while (lo < hi) {
      const mid = (lo + hi) >>> 1;
      if (this.records[mid].time_ns <= t_ns)
        lo = mid + 1;
      else
        hi = mid;
    }
    return lo - 1;
  }

  // Past MAX_HISTORY_BYTES: drops the fewest oldest records that bring what
  // is left, its first record made a keyframe, within it (always keeping
  // the newest).
  private dropOldest() {
    let k = 0, kept = this.bytes;
    const fits = () => {
      const head = this.records[k];
      return kept - head.bytes +
          (head.indices === null ? head.bytes : keyframeBytes(head.count)) <=
          MAX_HISTORY_BYTES;
    };
    while (k < this.records.length - 1 && !fits())
      kept -= this.records[k++].bytes;
    this.dropBefore(k);
  }

  // The records before index `k` go; record `k` becomes a keyframe of its
  // view, in place, so its identity (recordAt) is unchanged.
  private dropBefore(k: number) {
    if (k <= 0) return;
    const head = this.records[k];
    if (head.indices !== null) {
      const view = this.materialize(k);
      this.bytes -= head.bytes;
      head.indices = null;
      head.positions = view.positions;
      head.in_play = view.in_play;
      head.bytes = keyframeBytes(head.count);
      this.bytes += head.bytes;
    }
    for (let r = 0; r < k; r++) this.bytes -= this.records[r].bytes;
    this.records.splice(0, k);
    let since = 0;
    for (let r = this.records.length - 1; this.records[r].indices !== null; r--)
      since++;
    this.deltas = since;
  }

  // Record `i`'s view: its keyframe, then every delta after it up to `i`.
  private materialize(i: number): PiecesSnapshot {
    let k = i;
    while (this.records[k].indices !== null) k--;
    const key = this.records[k];
    const positions = key.positions.slice();
    const inPlay = key.in_play.slice();
    for (let r = k + 1; r <= i; r++) {
      const delta = this.records[r];
      delta.indices!.forEach((piece, j) => {
        positions.set(delta.positions.subarray(3 * j, 3 * j + 3), 3 * piece);
        inPlay[piece] = delta.in_play[j];
      });
    }
    const record = this.records[i];
    return {
      count: record.count,
      radius_m: record.radius_m,
      sample_time_ns: record.time_ns.toString(),
      positions,
      in_play: inPlay
    };
  }
}

// Writes the instance transform of each piece in play into `out` (16 floats
// each, column-major as THREE.Matrix4.elements: a uniform scale of the
// radius, for a unit sphere, then the piece's position), in piece order,
// and returns how many it wrote. `out` holds at least 16 * MAX_PIECES
// floats and is reused: nothing is allocated. Non-finite coordinates,
// invalid radii, and counts past the snapshot or output bounds are skipped.
export function pieceMatrices(snapshot: PiecesSnapshot, out: Float32Array): number {
  const rawRadius = snapshot.radius_m;
  if (!Number.isFinite(rawRadius) || rawRadius < 0) return 0;
  const r = rawRadius === 0 ? 0 : rawRadius;
  const rawCount = Number.isInteger(snapshot.count) && snapshot.count > 0 ?
      snapshot.count : 0;
  const count = Math.min(
      rawCount, MAX_PIECES, snapshot.in_play.length,
      Math.floor(snapshot.positions.length / 3));
  const capacity = Math.floor(out.length / 16);
  let n = 0;
  for (let i = 0; i < count && n < capacity; i++) {
    if (!snapshot.in_play[i]) continue;
    const x = snapshot.positions[3 * i];
    const y = snapshot.positions[3 * i + 1];
    const z = snapshot.positions[3 * i + 2];
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z))
      continue;
    const m = 16 * n++;
    out.fill(0, m, m + 16);
    out[m] = r;
    out[m + 5] = r;
    out[m + 10] = r;
    out[m + 12] = x;
    out[m + 13] = y;
    out[m + 14] = z;
    out[m + 15] = 1;
  }
  return n;
}
