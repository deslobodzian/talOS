// Renders a workspace page's layout tree: splits become flex rows/columns with
// resizable handles, panels become their chrome plus their component. Panels
// are each wrapped in their own error boundary, so one crashing panel cannot
// take down the rest of the page.

import {Fragment, useRef} from 'react';

import {findPanel, MIN_PANEL_SIZE} from '../layout/tree';
import type {LayoutNode, PanelNode, SplitDirection, SplitNode} from '../layout/types';
import {panelById} from '../panels/registry';
import {UnknownPanel} from '../panels/unknown';
import {PanelChrome} from '../ui/PanelChrome';
import {PanelErrorBoundary} from '../ui/PanelErrorBoundary';

export type WorkspaceCallbacks = {
  onSplit: (instanceId: string, direction: SplitDirection) => void;
  onClose: (instanceId: string) => void;
  onSwitch: (instanceId: string, panelId: string) => void;
  onConfig: (instanceId: string, config: unknown) => void;
  onResize: (splitPath: number[], sizes: number[]) => void;
  onMaximize: (instanceId: string|null) => void;
  onFocusPanel: (instanceId: string) => void;
};

function SplitHandle({path, index, direction, sizes, onResize}: {
  path: number[];
  index: number;
  direction: SplitDirection;
  sizes: number[];
  onResize: (path: number[], sizes: number[]) => void;
}) {
  const drag =
      useRef<{startPos: number; startSizes: number[]; size: number}|null>(null);

  const begin = (e: React.PointerEvent) => {
    const el = e.currentTarget as HTMLElement;
    const splitEl = el.parentElement;
    if (!splitEl) return;
    const rect = splitEl.getBoundingClientRect();
    const size = direction === 'row' ? rect.width : rect.height;
    drag.current = {
      startPos: direction === 'row' ? e.clientX : e.clientY,
      startSizes: sizes.slice(),
      size
    };
    el.setPointerCapture(e.pointerId);
  };

  const move = (e: React.PointerEvent) => {
    const d = drag.current;
    if (!d) return;
    const pos = direction === 'row' ? e.clientX : e.clientY;
    const pair = (d.startSizes[index] ?? 0) + (d.startSizes[index + 1] ?? 0);
    if (pair <= 0) return;
    const delta = (pos - d.startPos) / (d.size || 1);
    const minShare = Math.min(MIN_PANEL_SIZE, pair / 2);
    const nextA = Math.min(pair - minShare,
                          Math.max(minShare, d.startSizes[index] + delta));
    const next = d.startSizes.slice();
    next[index] = nextA;
    next[index + 1] = pair - nextA;
    onResize(path, next);
  };

  const end = () => {
    drag.current = null;
  };

  const key = (e: React.KeyboardEvent) => {
    let delta = 0;
    if (e.key === 'ArrowRight' || e.key === 'ArrowDown') delta = 0.05;
    else if (e.key === 'ArrowLeft' || e.key === 'ArrowUp') delta = -0.05;
    if (!delta) return;
    e.preventDefault();
    e.stopPropagation();
    const pair = sizes[index] + sizes[index + 1];
    if (pair <= 0) return;
    const minShare = Math.min(MIN_PANEL_SIZE, pair / 2);
    const next = sizes.slice();
    next[index] = Math.min(pair - minShare,
                           Math.max(minShare, sizes[index] + delta));
    next[index + 1] = pair - next[index];
    onResize(path, next);
  };

  return <div className="split-handle" role="separator" tabIndex={0}
              aria-orientation={direction === 'row' ? 'vertical' : 'horizontal'}
              onPointerDown={begin} onPointerMove={move} onPointerUp={end}
              onPointerCancel={end} onKeyDown={key}/>;
}

function renderPanel(node: PanelNode, callbacks: WorkspaceCallbacks,
                     maximized: string|null) {
  const def = panelById(node.panelId);
  const Component = def?.component;
  const config = def ? def.parseConfig(node.config) : undefined;

  const content = Component ?
      <Component key={`${node.instanceId}:${node.panelId}`}
                 config={config} instanceId={node.instanceId}
                 setConfig={c => callbacks.onConfig(node.instanceId, c)}/> :
      <UnknownPanel panelId={node.panelId} config={node.config}/>;

  return <PanelChrome key={node.instanceId}
      panelId={node.panelId}
      title={def?.title ?? 'Unknown panel'}
      category={def?.category ?? 'Tools'}
      onSwitch={panelId => callbacks.onSwitch(node.instanceId, panelId)}
      onSplit={direction => callbacks.onSplit(node.instanceId, direction)}
      onMaximize={() => callbacks.onMaximize(
          maximized === node.instanceId ? null : node.instanceId)}
      onClose={() => callbacks.onClose(node.instanceId)}
      onInteract={() => callbacks.onFocusPanel(node.instanceId)}
      maximized={maximized === node.instanceId}>
    <PanelErrorBoundary key={`${node.instanceId}:${node.panelId}`}>
      {content}
    </PanelErrorBoundary>
  </PanelChrome>;
}

function renderNode(node: LayoutNode, path: number[],
                    callbacks: WorkspaceCallbacks, maximized: string|null) {
  if (node.type === 'panel') return renderPanel(node, callbacks, maximized);
  return <SplitView node={node} path={path} callbacks={callbacks}
                    maximized={maximized}/>;
}

function SplitView({node, path, callbacks, maximized}: {
  node: SplitNode;
  path: number[];
  callbacks: WorkspaceCallbacks;
  maximized: string|null;
}) {
  return <div className={'split ' + node.direction}>
    {node.children.map((child, i) => {
      const childKey = child.type === 'panel' ? child.instanceId : `split-${i}`;
      return <Fragment key={childKey}>
      {i > 0 && <SplitHandle path={path} index={i - 1}
                             direction={node.direction} sizes={node.sizes}
                             onResize={callbacks.onResize}/>}
      <div className="split-child"
           style={{flexGrow: node.sizes[i] ?? 1, flexBasis: 0, minWidth: 0, minHeight: 0}}>
        {renderNode(child, [...path, i], callbacks, maximized)}
      </div>
    </Fragment>;
    })}
  </div>;
}

export function WorkspaceArea({root, maximized, callbacks}: {
  root: LayoutNode;
  maximized: string|null;
  callbacks: WorkspaceCallbacks;
}) {
  if (maximized) {
    const node = findPanel(root, maximized);
    if (node)
      return <div className="workspace maximized">
        {renderPanel(node, callbacks, maximized)}
      </div>;
  }
  return <div className="workspace">{renderNode(root, [], callbacks, maximized)}</div>;
}
