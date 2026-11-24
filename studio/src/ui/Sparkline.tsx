// A pure SVG sparkline: one polyline per finite run, no axes or DOM measurement.
// Non-finite gaps remain disconnected rather than drawn to zero or bridged.
// Used by the status strip and small panels for recent channel history.

import {useMemo} from 'react';

export function Sparkline({values, width = 120, height = 28, stroke = '#48e1cf', fill = 'none'}: {
  values: ArrayLike<number>;
  width?: number;
  height?: number;
  stroke?: string;
  fill?: string;
}) {
  const points = useMemo(() => {
    const n = values.length;
    if (n < 1) return null;
    let min = Infinity, max = -Infinity, count = 0;
    for (let i = 0; i < n; i++) {
      const v = values[i];
      if (Number.isFinite(v)) {
        if (v < min) min = v;
        if (v > max) max = v;
        count++;
      }
    }
    if (count < 1) return null;
    const span = max - min;
    const halfSpan = max * 0.5 - min * 0.5;
    const norm = (v: number): number => {
      if (!(span > 0)) return 0.5;
      if (Number.isFinite(span)) return (v - min) / span;
      if (!(halfSpan > 0) || !Number.isFinite(halfSpan)) return 0.5;
      return (v * 0.5 - min * 0.5) / halfSpan;
    };
    const pad = 2;
    const out: string[] = [];
    const runs: string[] = [];
    const endRun = () => {
      if (out.length) {
        const single = out.length === 1 ? out[0].split(',').map(Number) : null;
        runs.push(out.join(' ') +
            (single ? ` ${single[0] + .01},${single[1]}` : ''));
      }
      out.length = 0;
    };
    for (let i = 0; i < n; i++) {
      const v = values[i];
      if (!Number.isFinite(v)) {
        endRun();
        continue;
      }
      const x = (n === 1 ? .5 : i / (n - 1)) * (width - 2 * pad) + pad;
      const y = height - pad - norm(v) * (height - 2 * pad);
      out.push(`${x.toFixed(1)},${y.toFixed(1)}`);
    }
    endRun();
    return runs;
  }, [values, width, height]);

  if (!points) return <svg className="sparkline" width={width} height={height}
                           aria-hidden="true"/>;
  return <svg className="sparkline" width={width} height={height}
              aria-hidden="true">
    {points.map((run, i) =>
      <polyline key={i} points={run} fill={fill} stroke={stroke} strokeWidth="1.5"
                strokeLinejoin="round" strokeLinecap="round"/>)}
  </svg>;
}
