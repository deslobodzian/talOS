// Pure derivations for the hardware panel: counter rates, bitmask history and
// flag bands, computed over the columnar Series the store hands back. Nothing
// here reads the store or React, so it is unit-tested directly.

// A contiguous span (seconds since the series origin) where a 0/1 flag read 1.
export type Band = {start: number; end: number};

// Rate of a cumulative counter over the samples given, in units per second.
//
// Null when there are fewer than two readings, when the time span is zero, or
// when the counter wrapped (the later value is lower than the earlier one):
// "not measured" and "the bridge restarted" are different statements from a
// real rate. A flat counter yields 0, which is the honest "not moving".
export function counterRate(
    times: ArrayLike<number>, values: ArrayLike<number>): number|null {
  let first = -1;
  let last = -1;
  for (let i = 0; i < values.length; i++) {
    if (Number.isFinite(values[i])) {
      if (first < 0) first = i;
      last = i;
    }
  }
  if (first < 0 || last <= first) return null;
  const dt = times[last] - times[first];
  const dv = values[last] - values[first];
  if (!(dt > 0) || dv < 0) return null;
  return dv / dt;
}

// One bit of a bitmask channel over time.
export type BitHistory = {
  bit: number;
  set: boolean|null;   // null when there was no finite reading
  lastSetAt: number|null;  // seconds since origin of the last 0→1 edge (null
                           // if the bit never rose within the view)
  setCount: number;     // 0→1 edges in the view
};

// The history of each low bit of a bitmask channel. `bits` is how many low
// bits to report (e.g. subsystem_count, motor count), clamped to 32. NaN
// samples are skipped, so a gap neither clears nor sets a bit; a bit already
// set at the first sample is "set" with no edge in the view.
export function bitHistory(
    times: ArrayLike<number>, mask: ArrayLike<number>,
    bits = 32): BitHistory[] {
  const n = Math.min(32, Math.max(0, bits));
  const out: BitHistory[] = [];
  for (let b = 0; b < n; b++) {
    const bit = 1 << b;
    let prev = false;
    let seen = false;
    let lastSetAt: number|null = null;
    let setCount = 0;
    for (let i = 0; i < mask.length; i++) {
      const v = mask[i];
      if (!Number.isFinite(v)) continue;
      const on = (v & bit) !== 0;
      if (!seen) {
        // The first reading is a state, not an edge: a bit already set at the
        // start of the view has no observable rise inside it.
        prev = on;
        seen = true;
        continue;
      }
      if (on && !prev) {
        setCount++;
        lastSetAt = times[i];
      }
      prev = on;
    }
    out.push({bit: b, set: seen ? prev : null, lastSetAt, setCount});
  }
  return out;
}

// Contiguous spans (seconds since origin) where a 0/1 flag channel reads 1.
// NaN samples neither start nor continue a span.
export function flagBands(
    times: ArrayLike<number>, flag: ArrayLike<number>): Band[] {
  const out: Band[] = [];
  let start: number|null = null;
  for (let i = 0; i < flag.length; i++) {
    const on = Number.isFinite(flag[i]) && flag[i] !== 0;
    if (on && start === null) {
      start = times[i];
    } else if (!on && start !== null) {
      out.push({start, end: times[i]});
      start = null;
    }
  }
  if (start !== null && flag.length > 0)
    out.push({start, end: times[flag.length - 1]});
  return out;
}
