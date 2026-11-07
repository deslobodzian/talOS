// Target-vs-measured pairing, as a pure function over channel names and units.
//
// Within one subsystem, a channel whose leaf name carries a demand marker
// (`target`, `requested`, `setpoint`, `goal`) pairs with the measured channel
// that shares its unit and the rest of its name. The "rest of its name" is the
// leaf with every marker token removed, matched as a contiguous run against the
// measured leaf — so `target_velocity_rps` pairs with `flywheel_velocity_rps`
// (both `rps`, shared stem `velocity_rps`) and `requested_velocity_rps` pairs
// with `roller_velocity_rps`, while `target_velocity_rps` never pairs with
// `battery_v` (wrong unit) or `beam_broken` (boolean, no unit).
//
// Pure over `{name, unit}` records so the panel and its tests call the same
// function with no worker, no React, and no 2026 channel names hard-coded.

import {channelLeaf} from '../../ui/channel';

export type Pairing = {
  target: string;   // full channel name carrying the demand
  measured: string; // full channel name reporting what actually happened
  stem: string;     // the part of the leaf the two share (display)
  unit: string;
};

const MARKERS = new Set(['target', 'requested', 'setpoint', 'goal']);

const tokensOf = (leafName: string): string[] =>
    leafName.split('_').filter(token => token.length > 0);

const isTarget = (leafName: string): boolean =>
    tokensOf(leafName).some(token => MARKERS.has(token));

const stemOf = (leafName: string): string =>
    tokensOf(leafName).filter(token => !MARKERS.has(token)).join('_');

// Does `haystack` contain `needle` (a non-empty token list) as a contiguous
// run? Used to decide whether a measured leaf shares a target's stem.
function containsStem(haystack: string, needleTokens: string[]): boolean {
  if (needleTokens.length === 0) return false;
  const tokens = tokensOf(haystack);
  if (tokens.length < needleTokens.length) return false;
  outer: for (let start = 0; start + needleTokens.length <= tokens.length; start++) {
    for (let j = 0; j < needleTokens.length; j++)
      if (tokens[start + j] !== needleTokens[j]) continue outer;
    return true;
  }
  return false;
}

// Pair every demand channel with the measured channel that best shares its
// stem and unit. Returns pairs sorted by target name. A demand with no stem
// left after stripping markers (e.g. a leaf that is *only* `target`), a demand
// with no unit, and a measured channel with no unit are all left unpaired.
export function pairTargets(
    channels: readonly {name: string; unit: string}[]): Pairing[] {
  const targets = channels.filter(c => isTarget(channelLeaf(c.name)) && c.unit !== '');
  const measured =
      channels.filter(c => !isTarget(channelLeaf(c.name)) && c.unit !== '');

  const out: Pairing[] = [];
  for (const target of targets) {
    const targetLeaf = channelLeaf(target.name);
    const stem = stemOf(targetLeaf);
    const stemTokens = tokensOf(stem);
    if (stemTokens.length === 0) continue;

    let best: {name: string; extra: number}|null = null;
    for (const m of measured) {
      if (m.name === target.name || m.unit !== target.unit) continue;
      if (!containsStem(channelLeaf(m.name), stemTokens)) continue;
      // Fewer stray tokens means the stem is the more specific match; the name
      // tie-breaks so the result is deterministic.
      const extra = tokensOf(channelLeaf(m.name)).length - stemTokens.length;
      if (!best || extra < best.extra ||
          (extra === best.extra && m.name < best.name))
        best = {name: m.name, extra};
    }
    if (best)
      out.push({target: target.name, measured: best.name, stem, unit: target.unit});
  }
  return out.sort((a, b) => a.target < b.target ? -1 : a.target > b.target ? 1 : 0);
}
