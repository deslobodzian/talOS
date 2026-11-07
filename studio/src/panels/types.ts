// The panel contract. A panel is a self-contained module: it reads data only
// through the hooks in ../data (never from App props), keeps its own
// per-instance configuration, and is registered with one line in
// ./registry.ts. The layout persists `config` as JSON, so every config must
// be plain JSON data and `parseConfig` must accept anything (an old or
// hand-edited workspace) and return a valid config, falling back to
// `defaultConfig` field by field.

import type {ComponentType} from 'react';

export type PanelCategory = 'Robot'|'Spatial'|'Data'|'System'|'Tools';

export type PanelProps<C> = {
  config: C;
  setConfig: (config: C) => void;
  instanceId: string;
};

export type PanelDefinition<C = unknown> = {
  id: string;           // stable; persisted in workspaces
  title: string;
  category: PanelCategory;
  description: string;  // one line, shown in the panel picker
  defaultConfig: C;
  parseConfig: (raw: unknown) => C;
  component: ComponentType<PanelProps<C>>;
};

// Registry entries erase the config type; the layout calls parseConfig before
// handing a config to the component, so the pairing stays sound.
// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type AnyPanelDefinition = PanelDefinition<any>;
