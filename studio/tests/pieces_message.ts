// A bridge pieces message (studio/bridge/README.md "Game pieces"), written
// from the layout the README states, for the tests that read one.
export type PieceRecord = {
  index: number; in_play: boolean; x_mm: number; y_mm: number; z_mm: number;
};

export function piecesMessage(keyframe: boolean, count: number, radius_m: number,
                              time_ns: bigint, records: PieceRecord[]): Uint8Array {
  const bytes = new Uint8Array(24 + 16 * records.length);
  const view = new DataView(bytes.buffer);
  bytes.set([0x50, 0x43, 0x53, 0x31]);
  view.setUint8(4, keyframe ? 1 : 0);
  view.setUint16(6, records.length, true);
  view.setUint32(8, count, true);
  view.setFloat32(12, radius_m, true);
  view.setBigUint64(16, time_ns, true);
  records.forEach((r, i) => {
    const at = 24 + 16 * i;
    view.setUint16(at, r.index, true);
    view.setUint8(at + 2, r.in_play ? 1 : 0);
    view.setInt32(at + 4, r.x_mm, true);
    view.setInt32(at + 8, r.y_mm, true);
    view.setInt32(at + 12, r.z_mm, true);
  });
  return bytes;
}
