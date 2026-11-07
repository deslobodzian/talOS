// Pure staleness derivation for the inspector: the age of a channel's newest
// reading, as of the playback cursor. Kept pure so the unit tests can cover it
// without a worker or a browser.

import type {Series} from '../../data/types';

// The age in nanoseconds of the newest non-NaN reading for `name`, relative to
// the playback cursor. Null when the channel has never had a value or the
// cursor is unknown. Returned as a decimal string so a 64-bit age stays exact;
// '0' means the channel has a reading at the cursor itself.
export function channelStalenessNs(
    series: Series, name: string, cursorNs: string|null): string|null {
  if (cursorNs === null) return null;
  const values = series.values[name];
  if (!values) return null;

  let last = -1;
  for (let i = values.length - 1; i >= 0; i--) {
    if (Number.isFinite(values[i])) {
      last = i;
      break;
    }
  }
  if (last < 0) return null;

  const lastNs =
      BigInt(series.t0_ns) + BigInt(Math.round(series.times[last] * 1e9));
  const cursor = BigInt(cursorNs);
  if (cursor < lastNs) return '0';
  return (cursor - lastNs).toString();
}
