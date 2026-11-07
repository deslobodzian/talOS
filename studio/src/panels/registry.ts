// The panel registry: one import per panel, an explicit array, and a lookup by
// id. Adding a panel is exactly this: write panels/<id>/index.tsx exporting a
// PanelDefinition, then add one import and one array entry here. Nothing else
// in Studio learns about a new panel.

import type {AnyPanelDefinition, PanelCategory} from './types';
import {agentPanel} from './agent';
import {eventsPanel} from './events';
import {fieldPanel} from './field';
import {framesPanel} from './frames';
import {hardwarePanel} from './hardware';
import {healthPanel} from './health';
import {inspectorPanel} from './inspector';
import {plotPanel} from './plot';
import {pose3dPanel} from './pose3d';
import {robotPanel} from './robot';
import {sessionPanel} from './session';
import {subsystemsPanel} from './subsystems';
import {systemPanel} from './system';
import {timingPanel} from './timing';
import {visionPanel} from './vision';

// Order within a category is the picker's order: the most useful first.
export const PANELS: AnyPanelDefinition[] = [
  robotPanel,
  subsystemsPanel,
  visionPanel,
  eventsPanel,
  healthPanel,
  fieldPanel,
  pose3dPanel,
  plotPanel,
  inspectorPanel,
  hardwarePanel,
  sessionPanel,
  systemPanel,
  timingPanel,
  agentPanel,
  framesPanel
];

export function panelById(id: string): AnyPanelDefinition|undefined {
  return PANELS.find(panel => panel.id === id);
}

// Category order for the picker and any group-of-groups listing.
export const CATEGORIES: PanelCategory[] =
    ['Robot', 'Spatial', 'Data', 'System', 'Tools'];

export function panelsByCategory(): Map<PanelCategory, AnyPanelDefinition[]> {
  const out = new Map<PanelCategory, AnyPanelDefinition[]>();
  for (const category of CATEGORIES) out.set(category, []);
  for (const panel of PANELS) out.get(panel.category)!.push(panel);
  return out;
}
