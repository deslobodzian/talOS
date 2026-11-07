// Pure event filtering and time-relative formatting for the events panel.
// No React and no DOM here, so the panel and the tests can both call these
// with no worker or browser.

import type {EventCategory, RobotEvent, Severity} from '../../data/types';

export const SEVERITIES: Severity[] = ['info', 'warn', 'error'];

export const CATEGORIES: EventCategory[] = [
  'driver_station', 'power', 'hardware', 'vision',
  'estimator', 'mechanism', 'system', 'stream'
];

export type EventsFilter = {
  severities: Severity[];
  categories: EventCategory[];
  text: string;
};

// Events matching the filter, newest first. `severities` and `categories` are
// membership sets (an empty set shows nothing, it is not "all"); `text` is a
// case-insensitive substring over the title, detail and channel. The input
// order is the event engine's chronological order, so the result is the same
// timeline reversed.
export function filterEvents(
    events: readonly RobotEvent[], filter: EventsFilter): RobotEvent[] {
  const severities = new Set(filter.severities);
  const categories = new Set(filter.categories);
  const needle = filter.text.trim().toLowerCase();
  const out: RobotEvent[] = [];
  for (const event of events) {
    if (!severities.has(event.severity)) continue;
    if (!categories.has(event.category)) continue;
    if (needle) {
      const haystack =
          `${event.title}\n${event.detail ?? ''}\n${event.channel ?? ''}`
              .toLowerCase();
      if (!haystack.includes(needle)) continue;
    }
    out.push(event);
  }
  out.sort(compareNewest);
  return out;
}

// Newest first, comparing decimal uint64 nanosecond strings exactly. Equal
// timestamps keep their (chronological) input order via the stable sort.
function compareNewest(a: RobotEvent, b: RobotEvent): number {
  const ta = BigInt(a.t_ns);
  const tb = BigInt(b.t_ns);
  if (ta !== tb) return ta > tb ? -1 : 1;
  return 0;
}

// How far an event sits from the playback cursor, in seconds, with the sign a
// viewer expects: negative is "already happened", positive is "later than the
// cursor" (only possible while scrubbed back). `future` drives the dimming.
export function formatRelative(eventNs: string, cursorNs: string|null):
    {label: string; future: boolean} {
  if (cursorNs == null) return {label: '—', future: false};
  const delta = Number(BigInt(eventNs) - BigInt(cursorNs)) / 1e9;
  const future = delta > 0;
  const abs = Math.abs(delta);
  const label = abs < 0.05 ? 'now' : `${future ? '+' : '-'}${abs.toFixed(1)} s`;
  return {label, future};
}
