// The workspace layout model. Pure data: every node is plain JSON so the whole
// workspace can round-trip through localStorage and a JSON file unchanged.
//
// A page's layout is a tree. Splits divide space; panels are leaves. A panel
// names a registered panel type (`panelId`) and keeps one `config`, which the
// owning panel's `parseConfig` turns back into a typed value before it is
// rendered. `instanceId` is the panel's identity within the workspace: two
// panels of the same type live under different instances, and the layout ops
// address a panel by its instanceId rather than its position.

export type SplitDirection = 'row'|'column';

export type SplitNode = {
  type: 'split';
  direction: SplitDirection;
  children: LayoutNode[];
  sizes: number[];  // fractions of the split, index-aligned with children
};

export type PanelNode = {
  type: 'panel';
  instanceId: string;
  panelId: string;
  config: unknown;
};

export type LayoutNode = SplitNode|PanelNode;

export type Page = {id: string; title: string; root: LayoutNode};

export type Workspace = {
  version: 1;
  activePage: string;
  pages: Page[];
};
