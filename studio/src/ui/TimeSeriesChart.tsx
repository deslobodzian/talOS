// A small pure-SVG time-series chart: one or more lines (with an optional
// legend), shaded flag bands, and an optional dashed threshold. No text is
// rendered inside the SVG, so react-dom/server renders it and the strokes stay
// crisp under a non-uniform viewBox scale.
//
// A line with far more samples than horizontal pixels is decimated into
// per-pixel min/max spans before it is drawn (see decimateMinMax), so a
// 2-minute 100 Hz channel renders a handful of points per pixel instead of
// twelve thousand, and a one-sample spike survives instead of being averaged
// away.

import {useMemo, type ReactNode} from 'react';
import './timeSeriesChart.css';

export type ChartLine = {
  name: string;                 // legend label (deduped across lines)
  color: string;
  dash?: boolean;               // dashed stroke
  times?: ArrayLike<number>;    // x positions; omit to use the sample index
  values: ArrayLike<number>;    // y; NaN = a gap (the line breaks)
};

export type ChartBand = {start: number; end: number};  // in the x domain

// One non-empty horizontal pixel bucket: the x is the bucket centre (in the
// viewBox's own 0..width space) and yMin..yMax span every sample that landed in
// the bucket.
export type Span = {x: number; yMin: number; yMax: number};

// A connected run of spans; runs are separated by NaN gaps in the source.
export type SpanRun = Span[];

// Min/max decimation of one series into `width` horizontal buckets. Samples
// are bucketed by their x position (`times[i]`, or the sample index when
// `times` is absent) mapped over [loX, hiX]; each non-empty bucket becomes one
// span holding its min and max, so a lone outlier still raises the span and a
// NaN gap breaks the run. The number of spans is at most `width`.
export function decimateMinMax(
    times: ArrayLike<number>|undefined, values: ArrayLike<number>,
    width: number, loX: number, hiX: number): SpanRun[] {
  const n = values.length;
  const domain = hiX - loX || 1;
  const xAt = (i: number) => times ? times[i] : i;
  const bucketOf = (x: number) => {
    const b = Math.floor(((x - loX) / domain) * width);
    return b < 0 ? 0 : b >= width ? width - 1 : b;
  };

  const runs: SpanRun[] = [];
  let run: Span[] = [];
  let curBucket = -1;
  let curMin = Infinity;
  let curMax = -Infinity;

  const flush = () => {
    if (curBucket >= 0) {
      run.push({x: curBucket + 0.5, yMin: curMin, yMax: curMax});
      curBucket = -1;
      curMin = Infinity;
      curMax = -Infinity;
    }
  };
  const endRun = () => {
    flush();
    if (run.length) {
      runs.push(run);
      run = [];
    }
  };

  for (let i = 0; i < n; i++) {
    const v = values[i];
    if (!Number.isFinite(v)) { endRun(); continue; }
    const x = xAt(i);
    if (!Number.isFinite(x)) { endRun(); continue; }
    const b = bucketOf(x);
    if (b !== curBucket) {
      flush();
      curBucket = b;
      curMin = v;
      curMax = v;
    } else {
      if (v < curMin) curMin = v;
      if (v > curMax) curMax = v;
    }
  }
  endRun();
  return runs;
}

type Geometry = {
  lines: {name: string; color: string; dash: boolean; d: string}[];
  legend: {name: string; color: string}[];
  xOf: (x: number) => number;
  yOf: (v: number) => number;
};

function computeGeometry(
    lines: ChartLine[], width: number, height: number,
    threshold: number|undefined): Geometry|null {
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  let finiteCount = 0;
  for (const line of lines) {
    for (let i = 0; i < line.values.length; i++) {
      const v = line.values[i];
      if (!Number.isFinite(v)) continue;
      const x = line.times ? line.times[i] : i;
      if (!Number.isFinite(x)) continue;
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (v < minY) minY = v;
      if (v > maxY) maxY = v;
      finiteCount++;
    }
  }
  if (finiteCount < 1) return null;
  if (threshold !== undefined && Number.isFinite(threshold)) {
    if (threshold < minY) minY = threshold;
    if (threshold > maxY) maxY = threshold;
  }
  if (minX === maxX) { minX -= 1; maxX += 1; }
  if (minY === maxY) { minY -= 1; maxY += 1; }
  const spanX = maxX - minX;
  const spanY = maxY - minY;
  const loX = minX - spanX * 0.02;
  const hiX = maxX + spanX * 0.02;
  const loY = minY - spanY * 0.05;
  const hiY = maxY + spanY * 0.05;
  const xOf = (x: number) => ((x - loX) / (hiX - loX)) * width;
  const yOf = (v: number) => height - ((v - loY) / (hiY - loY)) * height;

  const out: Geometry['lines'] = [];
  const legend: Geometry['legend'] = [];
  const seen = new Set<string>();
  for (const line of lines) {
    const decimate = line.values.length > 2 * width;
    const d = linePath(line.times, line.values, xOf, yOf, decimate,
                       width, loX, hiX);
    if (!d) continue;
    out.push({name: line.name, color: line.color, dash: !!line.dash, d});
    if (!seen.has(line.name)) {
      seen.add(line.name);
      legend.push({name: line.name, color: line.color});
    }
  }
  return {lines: out, legend, xOf, yOf};
}

function linePath(
    times: ArrayLike<number>|undefined, values: ArrayLike<number>,
    xOf: (x: number) => number, yOf: (v: number) => number,
    decimate: boolean, width: number, loX: number, hiX: number): string {
  if (decimate) {
    const parts: string[] = [];
    for (const run of decimateMinMax(times, values, width, loX, hiX)) {
      let d = `M${run[0].x.toFixed(1)},${yOf(run[0].yMin).toFixed(1)}` +
          `L${run[0].x.toFixed(1)},${yOf(run[0].yMax).toFixed(1)}`;
      for (let i = 1; i < run.length; i++) {
        d += `L${run[i].x.toFixed(1)},${yOf(run[i].yMin).toFixed(1)}` +
             `L${run[i].x.toFixed(1)},${yOf(run[i].yMax).toFixed(1)}`;
      }
      if (run.length === 1 && run[0].yMin === run[0].yMax) d += ' l 0.01,0';
      parts.push(d);
    }
    return parts.join('');
  }

  const parts: string[] = [];
  let cur: string[] = [];
  const endRun = () => {
    if (cur.length) {
      parts.push('M' + cur.join('L') + (cur.length === 1 ? ' l 0.01,0' : ''));
      cur = [];
    }
  };
  for (let i = 0; i < values.length; i++) {
    const v = values[i];
    if (!Number.isFinite(v)) {
      endRun();
      continue;
    }
    const x = times ? times[i] : i;
    if (!Number.isFinite(x)) {
      endRun();
      continue;
    }
    cur.push(`${xOf(x).toFixed(1)},${yOf(v).toFixed(1)}`);
  }
  endRun();
  return parts.join('');
}

export function TimeSeriesChart({lines, width = 600, height = 96, fillWidth = true, bare = false, bands = [], threshold, legend = true, legendNote, empty}: {
  lines: ChartLine[];
  width?: number;
  height?: number;
  fillWidth?: boolean;   // stretch the svg to its container; else use `width`
  bare?: boolean;        // omit the frame (border/background) for tiny overlays
  bands?: ChartBand[];
  threshold?: number;
  legend?: boolean;
  legendNote?: string;
  empty?: ReactNode;
}) {
  const geometry = useMemo(
      () => computeGeometry(lines, width, height, threshold),
      [lines, width, height, threshold]);

  return <div className="ts-chart">
    {geometry ?
        <svg viewBox={`0 0 ${width} ${height}`} preserveAspectRatio="none"
             aria-hidden="true" className={bare ? 'bare' : undefined}
             style={{height, width: fillWidth ? '100%' : width}}>
          {bands.map((b, i) => {
            const x1 = geometry.xOf(b.start);
            const x2 = geometry.xOf(b.end);
            return <rect key={i} x={x1} y={0}
                         width={Math.max(1, x2 - x1)} height={height}
                         className="ts-chart-band"/>;
          })}
          {threshold !== undefined && Number.isFinite(threshold) &&
           <line x1={0} x2={width} y1={geometry.yOf(threshold)}
                 y2={geometry.yOf(threshold)} className="ts-chart-threshold"/>}
          {geometry.lines.map((line, i) =>
            <path key={i} d={line.d} fill="none" stroke={line.color}
                  strokeWidth="1.5" strokeLinejoin="round" strokeLinecap="round"
                  strokeDasharray={line.dash ? '3 3' : undefined}
                  vectorEffect="non-scaling-stroke"/>)}
        </svg> :
        (empty ?? <div className="ts-chart-empty">no data</div>)}
    {legend && geometry && geometry.legend.length > 0 &&
        <div className="ts-chart-legend">
          {geometry.legend.map(entry =>
            <span key={entry.name}>
              <i style={{background: entry.color}}/>
              {entry.name}
            </span>)}
          {legendNote && <span className="text-muted">{legendNote}</span>}
        </div>}
  </div>;
}
