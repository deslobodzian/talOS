// Pure derivations for the vision panel: no React, no store, no worker. Each
// helper takes plain aligned arrays (or scalars) so the panel and its tests
// call them identically. NaN / absent is "no reading", never zero; a counter
// that reset contributes no increment, never a negative delta.

import {STALE_AFTER_NS} from '../../data/freshness';

export type Planar = {x: number; y: number};

// One accepted camera observation compared against a reference pose.
export type ErrorPoint = {t: number; x: number; y: number};

const finite = (n: number | null | undefined): n is number =>
    n != null && Number.isFinite(n);

// Fraction of observed timestamp transitions that carried valid=1. The first
// finite `sample_time_ns` is a baseline, not evidence of a new observation:
// it may have been held since before this window. Absent samples are skipped;
// a window with no observed transition is null, not 0% or 100%.
export function validObservationRate(
    sampleTimeNs: ArrayLike<number>, valid: ArrayLike<number>): number|null {
  const n = Math.min(sampleTimeNs.length, valid.length);
  let total = 0;
  let ok = 0;
  let last: number | undefined;
  for (let i = 0; i < n; i++) {
    const t = sampleTimeNs[i];
    const v = valid[i];
    if (!finite(t) || !finite(v)) continue;
    if (last !== undefined && t !== last) {
      total++;
      if (v === 1) ok++;
    }
    last = t;
  }
  return total === 0 ? null : ok / total;
}

// Planar distance (metres) between a camera pose and the estimator pose.
export function disagreement(camera: Planar, estimator: Planar): number {
  return Math.hypot(camera.x - estimator.x, camera.y - estimator.y);
}

// The most recent pose whose `valid` sample is 1, walking back from the cursor
// (the window's last sample). An observation as old as the stale threshold is
// not compared with the current estimate: the robot has moved since. `times`
// are seconds, aligned with the other columns.
export function latestValidPose(
    times: ArrayLike<number>, x: ArrayLike<number>, y: ArrayLike<number>,
    valid: ArrayLike<number>): Planar|null {
  const n = Math.min(times.length, x.length, y.length, valid.length);
  let now = NaN;
  for (let i = n - 1; i >= 0; i--) {
    if (finite(times[i])) {
      now = times[i];
      break;
    }
  }
  if (!finite(now)) return null;
  const maxAge = Number(STALE_AFTER_NS) / 1e9;
  for (let i = n - 1; i >= 0; i--) {
    const t = times[i];
    if (!finite(t)) continue;
    if (now - t >= maxAge) break;
    if (valid[i] === 1 && finite(x[i]) && finite(y[i]))
      return {x: x[i], y: y[i]};
  }
  return null;
}

// Increase of a monotonic counter over a window: the sum of the nonnegative
// steps between consecutive finite readings. A reset (a step down) adds
// nothing, so neither a negative drop nor a fake jump across the reset is
// reported; increments before the first reading or hidden by a reset are
// unknown and excluded. Null when there are no readings.
export function counterDelta(values: ArrayLike<number>): number|null {
  let prev: number | undefined;
  let delta = 0;
  for (let i = 0; i < values.length; i++) {
    const v = values[i];
    if (!finite(v)) continue;
    if (prev !== undefined && v > prev) delta += v - prev;
    prev = v;
  }
  return prev === undefined ? null : delta;
}

// Frame rate from a frame period in microseconds; null when absent/nonpositive.
export function fpsFromPeriod(periodUs: number|null|undefined): number|null {
  if (!finite(periodUs) || periodUs <= 0) return null;
  const fps = 1e6 / periodUs;
  return finite(fps) ? fps : null;
}

// Frame-rate series from a frame-period series (microseconds), point by point
// as fpsFromPeriod: an absent or nonpositive period is NaN, a gap in the
// sparkline, never an infinite or negative rate.
export function fpsSeries(periodUs: ArrayLike<number>): Float64Array {
  const out = new Float64Array(periodUs.length);
  for (let i = 0; i < periodUs.length; i++)
    out[i] = fpsFromPeriod(periodUs[i]) ?? NaN;
  return out;
}

// Accepted fraction over the last window, from the accepted/rejected deltas.
// Null when there is no delta to divide (no observations at all).
export function acceptancePct(
    accepted: number|null, rejected: number|null): number|null {
  if (!finite(accepted) || !finite(rejected) || accepted < 0 || rejected < 0)
    return null;
  const total = accepted + rejected;
  return total > 0 && finite(total) ? accepted / total : null;
}

// Maximum time skew (seconds) between a camera observation and its nearest
// reference sample when aligning separate decimated recorded series.
const MAX_REF_SKEW_S = 0.5;

// Per-observation x/y error of a camera against a reference pose (the
// estimator, or sim ground truth). One point per accepted observation: where
// valid=1 and both the camera and reference poses are present. When `refTimes`
// is provided (separate or sparsely-aligned camera and reference time axes,
// such as decimated recorded series), matches each observation to the nearest
// finite reference sample within MAX_REF_SKEW_S.
export function observationErrors(
    times: ArrayLike<number>,
    camX: ArrayLike<number>, camY: ArrayLike<number>, valid: ArrayLike<number>,
    refX: ArrayLike<number>, refY: ArrayLike<number>,
    refTimes?: ArrayLike<number>): ErrorPoint[] {
  const out: ErrorPoint[] = [];
  if (!refTimes) {
    const n = Math.min(
        times.length, camX.length, camY.length, valid.length, refX.length,
        refY.length);
    for (let i = 0; i < n; i++) {
      if (valid[i] !== 1) continue;
      if (!finite(times[i]) || !finite(camX[i]) || !finite(camY[i]) ||
          !finite(refX[i]) || !finite(refY[i]))
        continue;
      out.push({t: times[i], x: camX[i] - refX[i], y: camY[i] - refY[i]});
    }
    return out;
  }
  const rn = Math.min(refTimes.length, refX.length, refY.length);
  const refs: {t: number; x: number; y: number}[] = [];
  for (let j = 0; j < rn; j++) {
    const t = refTimes[j], rx = refX[j], ry = refY[j];
    if (finite(t) && finite(rx) && finite(ry)) refs.push({t, x: rx, y: ry});
  }
  if (!refs.length) return out;
  const cn = Math.min(times.length, camX.length, camY.length, valid.length);
  let r = 0;
  for (let i = 0; i < cn; i++) {
    if (valid[i] !== 1) continue;
    const t = times[i], cx = camX[i], cy = camY[i];
    if (!finite(t) || !finite(cx) || !finite(cy)) continue;
    while (r + 1 < refs.length && refs[r + 1].t <= t) r++;
    let best = refs[r];
    if (r + 1 < refs.length &&
        Math.abs(refs[r + 1].t - t) < Math.abs(best.t - t)) {
      best = refs[r + 1];
    }
    if (Math.abs(best.t - t) <= MAX_REF_SKEW_S) {
      out.push({t, x: cx - best.x, y: cy - best.y});
    }
  }
  return out;
}
