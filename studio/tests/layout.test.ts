// Pure layout-tree tests: the ops the shell runs, the workspace parser that
// guards against a hand-edited or outdated document, and the presets. Nothing
// here touches the data layer, the DOM, or React (the registry is only read
// for its ids).

import assert from 'node:assert/strict';
import {test} from 'node:test';

import {
  buildPresetRoot,
  DEFAULT_PAGES,
  defaultWorkspace,
  presetByName,
  PRESETS,
} from '../src/layout/presets';
import {panelById} from '../src/panels/registry';
import {
  close,
  countPanels,
  emptyWorkspace,
  findPanel,
  listPanelIds,
  MIN_PANEL_SIZE,
  normalizeSizes,
  nudge,
  panelNode,
  parseWorkspace,
  replacePanel,
  resize,
  serialize,
  setConfig,
  split,
  splitNode,
} from '../src/layout/tree';
import type {SplitNode} from '../src/layout/types';

const p = (panelId: string, instanceId: string) =>
    panelNode(panelId, instanceId, undefined);

test('normalizeSizes turns arbitrary values into fractions that sum to 1',
     () => {
       assert.deepEqual(normalizeSizes([1, 1]), [0.5, 0.5]);
       const [a, b] = normalizeSizes([1, 3]);
       assert.ok(Math.abs(a - 0.25) < 1e-9, `a=${a}`);
       assert.ok(Math.abs(b - 0.75) < 1e-9, `b=${b}`);
       assert.deepEqual(normalizeSizes([]), []);
       assert.deepEqual(normalizeSizes([7]), [1]);

       // Non-finite and non-positive values fall back to an equal share.
       assert.deepEqual(normalizeSizes([NaN, -1]), [0.5, 0.5]);
     });

test('normalizeSizes holds every share at or above the minimum when possible',
     () => {
       const [small, large] = normalizeSizes([0.001, 1]);
       assert.ok(small >= MIN_PANEL_SIZE - 1e-9, `small=${small}`);
       assert.ok(Math.abs(small + large - 1) < 1e-9);
     });

test('normalizeSizes pins small shares and redistributes proportionally until stable', () => {
  assert.deepEqual(normalizeSizes([0.001, 1, 1]), [0.12, 0.44, 0.44]);
  const shares = normalizeSizes([0.001, 0.125, 0.874]);
  assert.equal(shares[0], MIN_PANEL_SIZE);
  assert.equal(shares[1], MIN_PANEL_SIZE);
  assert.ok(Math.abs(shares[2] - 0.76) < 1e-12);
  const proportional = normalizeSizes([0.001, 1, 2]);
  assert.equal(proportional[0], MIN_PANEL_SIZE);
  assert.equal(proportional[2] / proportional[1], 2);
  assert.deepEqual(normalizeSizes([1, 2, 3], 0.4), [1 / 3, 1 / 3, 1 / 3]);
});

test('split adds a sibling panel and keeps the original identity', () => {
  const root = p('health', 'a');
  const next = split(root, 'a', 'row', 'field', 'b', undefined);
  assert.equal(next.type, 'split');
  assert.equal(next.direction, 'row');
  assert.equal(countPanels(next), 2);
  assert.deepEqual(listPanelIds(next), ['health', 'field']);
  // The original panel keeps its instance id.
  assert.ok(findPanel(next, 'a'));
});

test('close removes a panel and collapses a single-child split', () => {
  // Two panels side by side: closing one leaves a bare panel, not a split.
  const pair = split(p('health', 'a'), 'a', 'row', 'field', 'b', undefined);
  const single = close(pair, 'b')!;
  assert.equal(single.type, 'panel');
  assert.equal(single.instanceId, 'a');

  // A nested split collapses when its second-last child is closed.
  const inner = splitNode('column', [p('system', 's'), p('timing', 't')]);
  const root = splitNode('row', [inner, p('plot', 'c')]);
  const after = close(root, 's')!;
  assert.equal(after.type, 'split');  // outer split still has two children
  assert.equal(after.children[0].type, 'panel');
  assert.equal(after.children[0].instanceId, 't');

  // Closing the root panel leaves nothing.
  assert.equal(close(p('health', 'a'), 'a'), null);
});

test('replacePanel changes the type and resets the config', () => {
  const root = setConfig(p('health', 'a'), 'a', {channels: ['x']});
  const next = replacePanel(root, 'a', 'plot');
  const panel = findPanel(next, 'a')!;
  assert.equal(panel.panelId, 'plot');
  assert.equal(panel.config, undefined);
});

test('setConfig stores JSON config on the addressed panel only', () => {
  const root = split(p('plot', 'a'), 'a', 'row', 'plot', 'b', undefined);
  const next = setConfig(root, 'a', {channels: ['x', 'y'], windowSeconds: 5});
  assert.deepEqual(findPanel(next, 'a')!.config,
                   {channels: ['x', 'y'], windowSeconds: 5});
  assert.equal(findPanel(next, 'b')!.config, undefined);
});

test('resize normalizes the supplied sizes; a bad path is a no-op', () => {
  const root = split(p('a', 'x'), 'x', 'row', 'b', 'y', undefined) as SplitNode;
  const next = resize(root, [], [0.1, 0.9]) as SplitNode;
  const sizes = next.sizes;
  assert.ok(Math.abs(sizes[0] + sizes[1] - 1) < 1e-9);
  // A path that names no split leaves the tree alone.
  assert.equal(resize(root, [3], [0.2, 0.8]), root);
});

test('nudge shifts a share from one neighbour to the other', () => {
  const root = split(p('a', 'x'), 'x', 'row', 'b', 'y', undefined) as SplitNode;
  const next = nudge(root, [], 0, 0.1) as SplitNode;
  assert.ok(next.sizes[0] > root.sizes[0]);
  assert.ok(Math.abs(next.sizes[0] + next.sizes[1] - 1) < 1e-9);
});

test('parseWorkspace round-trips a valid workspace unchanged', () => {
  const root = splitNode('row', [p('health', 'a'), p('field', 'b')]);
  const ws = {version: 1 as const, activePage: 'pg', pages: [
    {id: 'pg', title: 'Robot', root}
  ]};
  assert.deepEqual(parseWorkspace(JSON.parse(serialize(ws))), ws);
});

test('parseWorkspace rejects unreadable documents with an empty workspace',
     () => {
       assert.deepEqual(parseWorkspace(null), emptyWorkspace());
       assert.deepEqual(parseWorkspace('nope'), emptyWorkspace());
       assert.deepEqual(parseWorkspace([1, 2]), emptyWorkspace());
       assert.deepEqual(parseWorkspace({version: 2, pages: []}),
                        emptyWorkspace());
       assert.deepEqual(parseWorkspace({version: 1}), emptyWorkspace());
       assert.deepEqual(parseWorkspace({version: 1, pages: 'x'}),
                        emptyWorkspace());
     });

test('parseWorkspace drops malformed pages and nodes, keeps the rest', () => {
  const root = splitNode('row', [p('health', 'a'), p('field', 'b')]);
  const ws = {
    version: 1,
    activePage: 'good',
    pages: [
      {id: 'good', title: 'Good', root},
      {id: 'bad', title: 'Bad', root: {type: 'panel', panelId: 'health'}},
      {id: 'badnode', title: 'Bad node', root: {type: 'split', direction: 'up'}},
      {id: 'dup', title: 'Duplicate', root: p('x', 'x')},
      {id: 'dup', title: 'Duplicate', root: p('y', 'y')}
    ]
  };
  const parsed = parseWorkspace(ws);
  assert.equal(parsed.pages.length, 2);  // good + one 'dup'
  assert.equal(parsed.pages[0].title, 'Good');
  // The bad page and the bad split root are dropped.
  assert.ok(!parsed.pages.some(page => page.id === 'bad'));
  assert.ok(!parsed.pages.some(page => page.id === 'badnode'));
});

test('parseWorkspace keeps an unknown panel id, preserving its config', () => {
  const ws = {
    version: 1,
    activePage: 'p',
    pages: [{
      id: 'p',
      title: 'Page',
      root: {
        type: 'panel',
        instanceId: 'a',
        panelId: 'a-panel-that-does-not-exist',
        config: {channels: ['x'], windowSeconds: 30}
      }
    }]
  };
  const parsed = parseWorkspace(ws);
  assert.equal(parsed.pages.length, 1);
  const panel = findPanel(parsed.pages[0].root, 'a')!;
  assert.equal(panel.panelId, 'a-panel-that-does-not-exist');
  assert.deepEqual(panel.config, {channels: ['x'], windowSeconds: 30});
});

test('parseWorkspace normalizes missing or short sizes arrays', () => {
  const ws = {
    version: 1,
    activePage: 'p',
    pages: [{
      id: 'p',
      title: 'Page',
      root: {
        type: 'split',
        direction: 'row',
        children: [p('a', 'a'), p('b', 'b'), p('c', 'c')],
        sizes: [1, 1]
      }
    }]
  };
  const parsed = parseWorkspace(ws);
  const root = parsed.pages[0].root;
  assert.equal(root.type, 'split');
  assert.equal(root.sizes.length, 3);
  assert.ok(Math.abs(root.sizes.reduce((a, b) => a + b, 0) - 1) < 1e-9);
});

test('every preset names registered panels and survives a workspace round trip',
     () => {
       const names = PRESETS.map(pr => pr.name);
       assert.equal(new Set(names).size, names.length, 'preset names are unique');
       for (const preset of PRESETS) {
         const root = buildPresetRoot(preset);
         const ids = listPanelIds(root);
         assert.ok(ids.length >= 1, `${preset.name} lays out at least one panel`);
         for (const id of ids)
           assert.ok(panelById(id), `${preset.name} names registered panel ${id}`);
         const page = {id: 'p', title: preset.name, root};
         const parsed = parseWorkspace({version: 1, activePage: 'p', pages: [page]});
         assert.deepEqual(listPanelIds(parsed.pages[0].root), ids,
                          `${preset.name} round-trips through parseWorkspace`);
       }
     });

test('the default workspace opens one page per default preset, first active',
     () => {
       for (const name of DEFAULT_PAGES)
         assert.ok(presetByName(name), `default page ${name} is a preset`);
       const workspace = defaultWorkspace();
       assert.deepEqual(workspace.pages.map(p => p.title), DEFAULT_PAGES);
       assert.equal(workspace.activePage, workspace.pages[0].id);
       for (const page of workspace.pages) assert.ok(countPanels(page.root) >= 1);
     });
