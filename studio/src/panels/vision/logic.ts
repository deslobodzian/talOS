// Pure derivations for the vision panel: no React, no store, no worker. Each
// helper takes plain aligned arrays (or scalars) so the panel and its tests
// call them identically. NaN / absent is "no reading", never zero; a counter
// that reset is "unknown", never a negative delta.

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
  let total = 0;
  let ok = 0;
  let last: number | undefined;
  for (let i = 0; i < sampleTimeNs.length; i++) {
    const t = sampleTimeNs[i];
    if (!finite(t)) continue;
    if (last !== undefined && t !== last) {
      total++;
      if (valid[i] === 1) ok++;
    }
    last = t;
  }
  return total === 0 ? null : ok / total;
}

// Planar distance (metres) between a camera pose and the estimator pose.
export function disagreement(camera: Planar, estimator: Planar): number {
  return Math.hypot(camera.x - estimator.x, camera.y - estimator.y);
}

// The most recent pose whose `valid` sample is 1, walking back from the cursor.
export function latestValidPose(
    x: ArrayLike<number>, y: ArrayLike<number>,
    valid: ArrayLike<number>): Planar|null {
  for (let i = valid.length - 1; i >= 0; i--) {
    if (valid[i] === 1 && finite(x[i]) && finite(y[i]))
      return {x: x[i] as number, y: y[i] as number};
  }
  return null;
}

// Change in a monotonic counter over a window: last finite reading minus the
// first. Null when there are no readings or the counter went backwards (a
// reset, which a viewer must not render as a negative drop).
export function counterDelta(values: ArrayLike<number>): number|null {
  let first: number | undefined;
  let last: number | undefined;
  for (let i = 0; i < values.length; i++) {
    const v = values[i];
    if (!finite(v)) continue;
    if (first === undefined) first = v;
    last = v;
  }
  if (first === undefined || last === undefined) return null;
  const delta = last - first;
  return delta < 0 ? null : delta;
}

// Frame rate from a frame period in microseconds; null when absent/nonpositive.
export function fpsFromPeriod(periodUs: number|null|undefined): number|null {
  if (!finite(periodUs) || periodUs <= 0) return null;
  return 1e6 / periodUs;
}

// Accepted fraction over the last window, from the accepted/rejected deltas.
// Null when there is no delta to divide (no observations at all).
export function acceptancePct(
    accepted: number|null, rejected: number|null): number|null {
  if (accepted === null || rejected === null) return null;
  const total = accepted + rejected;
  return total === 0 ? null : accepted / total;
}

// Per-observation x/y error of a camera against a reference pose (the
// estimator, or sim ground truth). One point per accepted observation: where
// valid=1 and both the camera and reference poses are present.
export function observationErrors(
    times: ArrayLike<number>,
    camX: ArrayLike<number>, camY: ArrayLike<number>, valid: ArrayLike<number>,
    refX: ArrayLike<number>, refY: ArrayLike<number>): ErrorPoint[] {
  const out: ErrorPoint[] = [];
  const n = Math.min(
      times.length, camX.length, camY.length, valid.length, refX.length,
      refY.length);
  for (let i = 0; i < n; i++) {
    if (valid[i] !== 1) continue;
    if (!finite(camX[i]) || !finite(camY[i]) || !finite(refX[i]) ||
        !finite(refY[i]))
      continue;
    out.push({t: times[i], x: camX[i] - refX[i], y: camY[i] - refY[i]});
  }
  return out;
}
