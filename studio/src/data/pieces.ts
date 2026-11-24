// The game pieces the MuJoCo gateway simulates, for the 3D viewport
// (REQ-DEV-43). The bridge sends a programmer client binary `PCS1` messages
// (studio/bridge/README.md "Game pieces"): a keyframe of every piece, then
// only the pieces whose position, in whole millimetres, or in-play flag
// changed. The worker decodes them here (PiecesDecoder) and posts the
// latest snapshot; the viewport turns it into one instanced sphere mesh's
// transforms (pieceMatrices), drawing only pieces in play.
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
