// Pure, immutable layout-tree operations. Nothing here touches the DOM, React
// or a panel definition: it manipulates the JSON tree, so it is unit-testable
// in plain node and its behaviour is identical for a saved workspace, an
// imported one, or one the user is building right now.
//
// Every operation returns a new tree; the input is never mutated.

import type {LayoutNode, PanelNode, Page, SplitDirection, SplitNode, Workspace} from './types';

export const WORKSPACE_VERSION = 1;

// The smallest share a panel may keep when a split is resized. As a fraction
// of the split, so it holds at any pixel width.
export const MIN_PANEL_SIZE = 0.12;

// The largest fraction, so a panel can never squeeze every sibling to zero.
const MAX_PANEL_SIZE = 1 - MIN_PANEL_SIZE;

export function isSplit(node: LayoutNode|null|undefined): node is SplitNode {
  return !!node && (node as LayoutNode).type === 'split';
}

export function isPanel(node: LayoutNode|null|undefined): node is PanelNode {
  return !!node && (node as LayoutNode).type === 'panel';
}

// --- construction -----------------------------------------------------------

export function panelNode(panelId: string, instanceId: string,
                          config: unknown): PanelNode {
  return {type: 'panel', instanceId, panelId, config};
}

export function splitNode(direction: SplitDirection, children: LayoutNode[],
                          sizes: number[] = []): SplitNode {
  return {
    type: 'split',
    direction,
    children,
    sizes: normalizeSizes(
        sizes.length === children.length ? sizes :
                                           children.map(() => 1))
  };
}

// A fresh, collision-resistant instance id. Not guaranteed globally unique
// across machines, but unique within one Studio session and stable enough to
// persist.
let counter = 0;
export function newInstanceId(): string {
  counter = (counter + 1) % 0xffff;
  return `p${Date.now().toString(36)}${counter.toString(36)}${
      Math.random().toString(36).slice(2, 6)}`;
}

// --- sizing ----------------------------------------------------------------

// Turn arbitrary sizes (pixel widths, raw fractions, whatever a drag handler
// produced) into fractions of the split that sum to 1, holding each at least
// MIN_PANEL_SIZE when that is possible.
export function normalizeSizes(sizes: number[],
                               min = MIN_PANEL_SIZE): number[] {
  const n = sizes.length;
  if (n === 0) return [];
  if (n === 1) return [1];

  if (n * min >= 1) return sizes.map(() => 1 / n);
  const raw = sizes.map(s => (Number.isFinite(s) && s > 0 ? s : 1));
  // Scale first to avoid overflowing the sum of finite weights.
  const scale = Math.max(...raw);
  const weights = raw.map(s => s / scale);
  const result = new Array<number>(n).fill(0);
  let remaining = 1;
  let active = weights.map((_, i) => i);
  while (active.length) {
    const total = active.reduce((sum, i) => sum + weights[i], 0);
    const clamped = active.filter(i => weights[i] / total * remaining < min);
    if (!clamped.length) {
      for (const i of active) result[i] = weights[i] / total * remaining;
      break;
    }
    for (const i of clamped) result[i] = min;
    remaining = 1 - result.reduce((sum, value) => sum + value, 0);
    active = active.filter(i => result[i] === 0);
  }
  return result;
}

// --- path helpers ----------------------------------------------------------

// A path is the index sequence from the root to a node. [] is the root itself.
function getAt(root: LayoutNode, path: number[]): LayoutNode|null {
  let node: LayoutNode = root;
  for (const i of path) {
    if (!isSplit(node) || i < 0 || i >= node.children.length) return null;
    node = node.children[i];
  }
  return node;
}

function setAt(root: LayoutNode, path: number[],
               next: LayoutNode): LayoutNode {
  if (path.length === 0) return next;
  if (!isSplit(root)) return root;
  const [i, ...rest] = path;
  if (i < 0 || i >= root.children.length) return root;
  const children = root.children.slice();
  children[i] = setAt(children[i], rest, next);
  return {...root, children};
}

export function findPanelPath(root: LayoutNode, instanceId: string): number[]|
    null {
  if (isPanel(root)) return root.instanceId === instanceId ? [] : null;
  for (let i = 0; i < root.children.length; i++) {
    const sub = findPanelPath(root.children[i], instanceId);
    if (sub) return [i, ...sub];
  }
  return null;
}

export function findPanel(root: LayoutNode,
                          instanceId: string): PanelNode|null {
  if (isPanel(root)) return root.instanceId === instanceId ? root : null;
  for (const child of root.children) {
    const found = findPanel(child, instanceId);
    if (found) return found;
  }
  return null;
}

// The path of the split that directly contains `instanceId`, or null when the
// panel is the root. This is the split a resize handle drags.
export function parentSplitPath(root: LayoutNode, instanceId: string): number[]|
    null {
  if (isPanel(root)) return null;
  for (let i = 0; i < root.children.length; i++) {
    const child = root.children[i];
    if (isPanel(child) && child.instanceId === instanceId) return [];
    const sub = parentSplitPath(child, instanceId);
    if (sub) return [i, ...sub];
  }
  return null;
}

// --- operations ------------------------------------------------------------

// Split a panel in two, placing the new panel to its right (row) or below it
// (column). The existing panel keeps its identity and config.
export function split(root: LayoutNode, instanceId: string,
                      direction: SplitDirection, panelId: string,
                      newInstanceId: string, config: unknown): LayoutNode {
  const path = findPanelPath(root, instanceId);
  if (!path) return root;
  const existing = getAt(root, path)!;
  const replacement =
      splitNode(direction, [existing, panelNode(panelId, newInstanceId, config)]);
  return setAt(root, path, replacement);
}

// Remove a panel. A split left with one child collapses into that child, so
// closing the last panel of a pair leaves a single panel, not an empty half.
// Closing the root panel leaves nothing: callers treat null as "empty page".
export function close(root: LayoutNode, instanceId: string): LayoutNode|null {
  if (isPanel(root)) return root.instanceId === instanceId ? null : root;
  const children: LayoutNode[] = [];
  const sizes: number[] = [];
  for (let i = 0; i < root.children.length; i++) {
    const child = close(root.children[i], instanceId);
    if (child === null) continue;  // the closed panel
    children.push(child);
    sizes.push(root.sizes[i] ?? 1);
  }
  if (children.length === 0) return null;
  if (children.length === 1) return children[0];
  // Rebuild even when the direct child count is unchanged: a descendant may
  // have collapsed (a nested split became a bare panel), which this level must
  // reflect rather than hand back the pre-collapse tree.
  return {...root, children, sizes: normalizeSizes(sizes)};
}

// Change which panel type a panel renders, resetting its config so the new
// type's parseConfig starts from its own default.
export function replacePanel(root: LayoutNode, instanceId: string,
                             panelId: string): LayoutNode {
  const path = findPanelPath(root, instanceId);
  if (!path) return root;
  const existing = getAt(root, path)! as PanelNode;
  return setAt(root, path, {...existing, panelId, config: undefined});
}

export function setConfig(root: LayoutNode, instanceId: string,
                          config: unknown): LayoutNode {
  const path = findPanelPath(root, instanceId);
  if (!path) return root;
  const existing = getAt(root, path)! as PanelNode;
  return setAt(root, path, {...existing, config});
}

// Set the sizes of the split at `splitPath`, normalized so they stay valid.
export function resize(root: LayoutNode, splitPath: number[],
                       sizes: number[]): LayoutNode {
  const split = getAt(root, splitPath);
  if (!isSplit(split)) return root;
  return setAt(
      root, splitPath, {...split, sizes: normalizeSizes(sizes)});
}

// Adjust one handle's neighbour shares by a signed delta in [-1, 1].
export function nudge(root: LayoutNode, splitPath: number[], index: number,
                      delta: number): LayoutNode {
  const split = getAt(root, splitPath);
  if (!isSplit(split) || index < 0 || index >= split.children.length - 1)
    return root;
  const sizes = split.sizes.slice();
  sizes[index] = sizes[index] + delta;
  sizes[index + 1] = sizes[index + 1] - delta;
  return resize(root, splitPath, sizes);
}

// --- traversal -------------------------------------------------------------

export function countPanels(root: LayoutNode): number {
  if (isPanel(root)) return 1;
  return root.children.reduce((n, c) => n + countPanels(c), 0);
}

export function listPanelIds(root: LayoutNode): string[] {
  if (isPanel(root)) return [root.panelId];
  return root.children.flatMap(listPanelIds);
}

// The first panel in depth-first order, used as the split target when a
// navigation action opens a panel on a page.
export function firstPanel(root: LayoutNode): PanelNode|null {
  if (isPanel(root)) return root;
  for (const child of root.children) {
    const found = firstPanel(child);
    if (found) return found;
  }
  return null;
}

export function forEachPanel(root: LayoutNode, fn: (panel: PanelNode) => void):
    void {
  if (isPanel(root)) {
    fn(root);
    return;
  }
  root.children.forEach(c => forEachPanel(c, fn));
}

// --- workspace -------------------------------------------------------------

export function emptyWorkspace(): Workspace {
  return {version: 1, activePage: '', pages: []};
}

export function makePage(title: string, root: LayoutNode, id = newId()): Page {
  return {id, title, root};
}

let pageCounter = 0;
export function newId(): string {
  pageCounter = (pageCounter + 1) % 0xffff;
  return `page${Date.now().toString(36)}${pageCounter.toString(36)}${
      Math.random().toString(36).slice(2, 6)}`;
}

export function serialize(workspace: Workspace): string {
  return JSON.stringify(workspace);
}

// --- parsing ---------------------------------------------------------------

const isString = (v: unknown): v is string => typeof v === 'string' && !!v;
const isObject = (v: unknown): v is Record<string, unknown> =>
    !!v && typeof v === 'object' && !Array.isArray(v);

function parseNode(value: unknown): LayoutNode|null {
  if (!isObject(value)) return null;
  if (value.type === 'panel') {
    if (!isString(value.instanceId) || !isString(value.panelId)) return null;
    return {
      type: 'panel',
      instanceId: value.instanceId,
      panelId: value.panelId,
      config: value.config
    };
  }
  if (value.type === 'split') {
    if (value.direction !== 'row' && value.direction !== 'column') return null;
    if (!Array.isArray(value.children) || value.children.length < 1) return null;
    const children: LayoutNode[] = [];
    for (const child of value.children) {
      const parsed = parseNode(child);
      // One malformed branch is dropped, not the whole split: a stray bad leaf
      // should not blank a page that is otherwise readable.
      if (parsed) children.push(parsed);
    }
    if (children.length < 1) return null;
    const sizes = Array.isArray(value.sizes) ?
        value.sizes.filter((s): s is number => typeof s === 'number') :
        [];
    return {
      type: 'split',
      direction: value.direction,
      children,
      sizes: normalizeSizes(
          sizes.length === children.length ? sizes :
                                             children.map(() => 1))
    };
  }
  return null;
}

// Validate a whole workspace document and return a guaranteed-usable one.
// Anything unreadable -- a non-object, a wrong version, a page without a
// valid root -- yields an empty workspace, the same "not known yet" answer a
// missing document yields. A structurally valid panel keeps its `panelId`
// even when no panel of that type is registered: the shell renders an
// "Unknown panel" placeholder so the config survives for a later upgrade.
export function parseWorkspace(raw: unknown): Workspace {
  if (!isObject(raw)) return emptyWorkspace();
  if (raw.version !== WORKSPACE_VERSION) return emptyWorkspace();
  if (!Array.isArray(raw.pages)) return emptyWorkspace();

  const pages: Page[] = [];
  const seenIds = new Set<string>();
  for (const entry of raw.pages) {
    if (!isObject(entry) || !isString(entry.id) || seenIds.has(entry.id))
      continue;
    const root = parseNode(entry.root);
    if (!root) continue;
    seenIds.add(entry.id);
    pages.push({
      id: entry.id,
      title: isString(entry.title) ? entry.title.slice(0, 80) : 'Untitled',
      root
    });
  }

  const activePage = isString(raw.activePage) && seenIds.has(raw.activePage) ?
      raw.activePage :
      (pages[0]?.id ?? '');
  return {version: 1, activePage, pages};
}
