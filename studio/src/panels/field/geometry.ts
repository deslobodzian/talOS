// Pure field geometry and viewport math for the field panel. No React, no DOM:
// every function here maps numbers to numbers so it can be unit-tested without
// a browser. The SVG user space has the blue alliance origin at bottom-left,
// +X pointing right (away from the blue wall) and +Y pointing up; SVG y grows
// downward, so `toPy` flips.

export const FIELD_WIDTH_M = 16.54;
export const FIELD_HEIGHT_M = 8.21;
export const PX_PER_M = 50;

// Field extent in SVG user units.
export const FIELD_WIDTH_PX = FIELD_WIDTH_M * PX_PER_M;
export const FIELD_HEIGHT_PX = FIELD_HEIGHT_M * PX_PER_M;

// Padding around the field for the axis arrows and labels.
export const FIELD_MARGIN_PX = 25;

export type PointM = {x: number; y: number};

// A rectangle in SVG user space, as a viewBox wants it.
export type Viewport = {x: number; y: number; width: number; height: number};

export const toPx = (x: number) => x * PX_PER_M;
export const toPy = (y: number) => FIELD_HEIGHT_PX - y * PX_PER_M;

export const clamp = (v: number, lo: number, hi: number) =>
    Math.min(hi, Math.max(lo, v));

// One-sigma uncertainty ellipse radii from the x/y standard deviations
// (metres). Non-finite or negative stds collapse to a zero radius (a dot)
// rather than poisoning the SVG. `sigma` scales to a wider confidence band.
export function uncertaintyEllipse(xStd: number, yStd: number, sigma = 1):
    {rx: number; ry: number} {
  const s = Math.max(0, Number.isFinite(sigma) ? sigma : 1);
  const r = (std: number) => Math.max(0, Number.isFinite(std) ? std : 0);
  return {rx: r(xStd) * s, ry: r(yStd) * s};
}

// Fade factor for an observation sampled at `sampleTimeNs` relative to
// `nowNs` (both uint64 nanosecond counts, carried as numbers or decimal
// strings): 1 while fresh, 0 once the sample is `maxAgeNs` old. A sample in
// the future by clock skew reads as fresh (age clamps to 0) rather than
// over-faded.
export function stalenessFade(sampleTimeNs: number|string,
                              nowNs: number|string, maxAgeNs = 1e9): number {
  const age = Number(nowNs) - Number(sampleTimeNs);
  if (age <= 0) return 1;
  if (age >= maxAgeNs) return 0;
  return 1 - age / maxAgeNs;
}

// The whole-field viewport: the field plus margin for the axis labels.
export function wholeFieldViewport(): Viewport {
  return {
    x: -FIELD_MARGIN_PX,
    y: -FIELD_MARGIN_PX,
    width: FIELD_WIDTH_PX + 2 * FIELD_MARGIN_PX,
    height: FIELD_HEIGHT_PX + 2 * FIELD_MARGIN_PX
  };
}

// Follow mode: a square `windowMeters` viewport centred on the robot, clamped
// so the window never leaves the field.
export function followViewport(robot: PointM, windowMeters = 4): Viewport {
  const half = (windowMeters * PX_PER_M) / 2;
  const cx = clamp(toPx(robot.x), half, FIELD_WIDTH_PX - half);
  const cy = clamp(toPy(robot.y), half, FIELD_HEIGHT_PX - half);
  return {x: cx - half, y: cy - half, width: half * 2, height: half * 2};
}

// Zoom `viewport` by `factor` while keeping the SVG user-space point
// (anchorX, anchorY) fixed on screen. A non-positive factor is a no-op.
export function zoomViewport(vp: Viewport, factor: number,
                             anchorX: number, anchorY: number): Viewport {
  if (!Number.isFinite(factor) || factor <= 0) return vp;
  const w = vp.width * factor;
  const h = vp.height * factor;
  return {
    x: anchorX - (anchorX - vp.x) * (w / vp.width),
    y: anchorY - (anchorY - vp.y) * (h / vp.height),
    width: w,
    height: h
  };
}
