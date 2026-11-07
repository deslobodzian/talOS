// Workspace presets. Data, not code: a preset names panels and how they are
// arranged, and nothing about how a panel renders, so adding a preset is one
// entry here and no component learns that presets exist.
//
// A first launch opens every preset in DEFAULT_PAGES as its own page, so the
// page tabs read as a table of contents: what the robot is doing (Robot),
// where it thinks it is (Vision), what each mechanism is doing (Mechanisms),
// the link to the RoboRIO (Hardware), the software graph (System), and raw
// signals (Signals).

import {
  makePage,
  newInstanceId as makeInstanceId,
  panelNode,
  splitNode,
} from './tree';
import type {LayoutNode, Page, Workspace} from './types';

// A panel, or a split of preset nodes. `sizes` are relative weights; they are
// normalized by the layout, and omitted means equal.
export type PresetNode = {panelId: string; config?: unknown}|
    {direction: 'row'|'column'; children: PresetNode[]; sizes?: number[]};

export type WorkspacePreset = {
  name: string;
  description: string;
  layout: PresetNode;
};

const panel = (panelId: string, config?: unknown): PresetNode =>
    config === undefined ? {panelId} : {panelId, config};
const row = (children: PresetNode[], sizes?: number[]): PresetNode =>
    ({direction: 'row', children, sizes});
const column = (children: PresetNode[], sizes?: number[]): PresetNode =>
    ({direction: 'column', children, sizes});

export const PRESETS: WorkspacePreset[] = [
  {
    name: 'Robot',
    description: 'What the robot is doing now: status, field position, events.',
    layout: row([panel('robot'), column([panel('field'), panel('events')], [3, 2])],
                [2, 3])
  },
  {
    name: 'Vision',
    description: 'Camera health and observations against the pose estimate.',
    layout: row([panel('vision'), panel('field')], [11, 9])
  },
  {
    name: 'Mechanisms',
    description: 'Every subsystem: targets, measurements, who is driving it.',
    layout: row([panel('subsystems'), panel('plot')], [3, 2])
  },
  {
    name: 'Hardware',
    description: 'Power, the RoboRIO link and its counters, with related events.',
    layout: row([
      panel('hardware'),
      panel('events', {categories: ['power', 'hardware', 'driver_station']})
    ], [3, 2])
  },
  {
    name: 'System',
    description: 'Processes and restarts, the node graph, stream health, timing.',
    layout: row([
      panel('system'),
      column([panel('session'), panel('health'), panel('timing')], [2, 2, 1])
    ])
  },
  {
    name: 'Signals',
    description: 'Plot any channel beside a searchable channel inspector.',
    layout: row([panel('plot'), panel('inspector')], [13, 7])
  },
  {
    name: 'Spatial',
    description: 'Field and 3D viewport for watching a pose in two frames.',
    layout: row([panel('field'), panel('pose3d')])
  },
  {
    name: 'Tools',
    description: 'The JSON-RPC agent and the offline camera frame viewer.',
    layout: row([panel('agent'), panel('frames')])
  }
];

// The presets a first launch opens as pages, in tab order.
export const DEFAULT_PAGES =
    ['Robot', 'Vision', 'Mechanisms', 'Hardware', 'System', 'Signals'];

export function presetByName(name: string): WorkspacePreset|undefined {
  return PRESETS.find(p => p.name === name);
}

function build(node: PresetNode, newInstanceId: () => string): LayoutNode {
  if ('panelId' in node)
    return panelNode(node.panelId, newInstanceId(), node.config);
  const children = node.children.map(child => build(child, newInstanceId));
  // A split needs two children; collapse degenerate preset entries.
  if (children.length === 1) return children[0];
  return splitNode(node.direction, children, node.sizes ?? []);
}

export function buildPresetRoot(preset: WorkspacePreset,
                                newInstanceId: () => string = makeInstanceId):
    LayoutNode {
  return build(preset.layout, newInstanceId);
}

export function buildPresetPage(preset: WorkspacePreset,
                                newInstanceId: () => string = makeInstanceId):
    Page {
  return makePage(preset.name, buildPresetRoot(preset, newInstanceId));
}

// The workspace a first launch gets before anything is saved.
export function defaultWorkspace(): Workspace {
  const pages = DEFAULT_PAGES.map(name => buildPresetPage(presetByName(name)!));
  return {version: 1, activePage: pages[0].id, pages};
}
