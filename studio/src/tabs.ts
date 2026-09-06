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
