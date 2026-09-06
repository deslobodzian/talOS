// The views Studio offers, in the order they appear.
//
// Separate from the components so a view can link to another one -- the
// Overview's attention list sends you to the tab that explains each item --
// without the shell and the views importing each other in a circle.

export const TABS = [
  {id: 'overview', label: 'Overview', hint: 'SESSION'},
  {id: 'system', label: 'System', hint: 'PUB / SUB'},
  {id: 'field', label: 'Field', hint: '2D'},
  {id: 'pose', label: 'Pose', hint: '3D'},
  {id: 'signals', label: 'Signals', hint: 'CHANNELS'},
  {id: 'state', label: 'State', hint: 'STRUCTURED'},
  {id: 'timing', label: 'Timing', hint: 'LATENCY'},
  {id: 'agent', label: 'Agent', hint: 'JSON-RPC'},
] as const;

export type TabId = typeof TABS[number]['id'];

export const isTabId = (value: unknown): value is TabId =>
    TABS.some(tab => tab.id === value);

// A graph the user added. The built-in views above are a fixed list because
// each one is a different way of reading the same session; graphs are not --
// they are all the same view, and which signals each holds is the whole point
// of having more than one. So these carry their own identity and their own
// channel selection, and the user creates and destroys them.
export type GraphTab = {
  id: string;
  label: string;
  channels: string[];
};

export const GRAPH_PREFIX = 'graph:';

export const isGraphTabId = (value: unknown): value is string =>
    typeof value === 'string' && value.startsWith(GRAPH_PREFIX) &&
    value.length > GRAPH_PREFIX.length;

// Any tab the shell can show: a built-in view or one of the user's graphs.
export type ViewId = TabId|string;

export const newGraphTab = (existing: readonly GraphTab[]): GraphTab => {
  // Numbered from the highest label already in use rather than from the count,
  // so closing "Graph 2" and adding another does not produce a second one.
  let highest = 0;
  for (const tab of existing) {
    const match = /^Graph (\d+)$/.exec(tab.label);
    if (match) highest = Math.max(highest, Number(match[1]));
  }
  return {
    id: `${GRAPH_PREFIX}${Date.now().toString(36)}${
        Math.random().toString(36).slice(2, 6)}`,
    label: `Graph ${highest + 1}`,
    channels: []
  };
};

// Graph tabs survive a reload, so they arrive from storage as unknown JSON.
// Anything malformed is dropped rather than rendered: a tab with no id cannot
// be selected or closed, which would strand the user on a view with no way out.
export function parseGraphTabs(value: unknown): GraphTab[] {
  if (!Array.isArray(value)) return [];
  const out: GraphTab[] = [];
  const seen = new Set<string>();
  for (const entry of value) {
    if (!entry || typeof entry !== 'object') continue;
    const {id, label, channels} = entry as Record<string, unknown>;
    if (!isGraphTabId(id) || seen.has(id)) continue;
    if (typeof label !== 'string' || !label) continue;
    seen.add(id);
    out.push({
      id,
      label: label.slice(0, 40),
      channels: Array.isArray(channels) ?
          channels.filter((c): c is string => typeof c === 'string') :
          []
    });
  }
  return out;
}
