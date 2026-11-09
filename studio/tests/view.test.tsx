// System, health and timing panels against the demo graph (staleness, faults,
// declared vs live), plus the workspace and timeline input handling. Panels
// read their data through hooks, so they render inside a StudioProvider backed
// by createStaticStudio; NodeCard takes its data as props and renders directly.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {createStaticStudio, StudioProvider} from '../src/data/studio';
import type {PlaybackState, SystemState} from '../src/data/types';
import {demoDeclaredGraph, demoSystemGraph} from '../src/demo';
import {healthPanel} from '../src/panels/health';
import type {AnyPanelDefinition} from '../src/panels/types';
import {NodeCard, systemPanel, Wiring} from '../src/panels/system';
import {timingPanel} from '../src/panels/timing';
import {eventRates, parseDeclaredGraph, parseSystemGraph} from '../src/system';
import {panelNode, splitNode} from '../src/layout/tree';
import {Timeline} from '../src/shell/Timeline';
import {WorkspaceArea, type WorkspaceCallbacks} from '../src/shell/WorkspaceArea';
import {StatusStrip} from '../src/shell/StatusStrip';

const emptyPlayback: PlaybackState = {
  mode: 'PAUSED',
  cursor_ns: null,
  bounds: null,
  stats: {dropped: 0, late: 0, invalid: 0},
  status: 'Disconnected',
  connection: 'idle'
};

// Two samples apart, so rates are measured rather than absent.
function state(declared = false): SystemState {
  const previous = parseSystemGraph(demoSystemGraph(500n));
  const graph = parseSystemGraph(demoSystemGraph(1500n));
  const rates: Record<string, number> = {};
  for (const [key, value] of eventRates(graph, previous)) rates[key] = value;
  return {
    graph,
    stale: false,
    rates,
    error: '',
    declared: declared ? parseDeclaredGraph(demoDeclaredGraph()) : null,
    declaredError: ''
  };
}

const render = (element: React.ReactElement) =>
    renderToStaticMarkup(element).replace(/<[^>]*>/g, '');

// Render a panel (which reads hooks) inside a static studio with this system.
function renderPanel(def: AnyPanelDefinition, system: SystemState,
                     playback: PlaybackState = emptyPlayback): string {
  return render(
      <StudioProvider studio={createStaticStudio({system, playback})}>
        <def.component config={def.defaultConfig} setConfig={() => {}}
                       instanceId="test"/>
      </StudioProvider>);
}

test('a stale graph stays inspectable but makes no live-node claim', () => {
  const graph = parseSystemGraph(demoSystemGraph(4000n));
  const total = graph.nodes.length;
  assert.ok(graph.nodes.some(n => n.alive) && graph.nodes.some(n => !n.alive));
  const system: SystemState =
      {graph, stale: true, rates: {}, error: '', declared: null, declaredError: ''};

  const view = renderPanel(systemPanel, system);
  assert.ok(view.includes('SYSTEM GRAPH STALE'));
  assert.ok(view.includes(`?/${total} nodes live`));
  for (const node of graph.nodes) assert.ok(view.includes(node.name), node.name);
  assert.ok(view.includes('UNKNOWN · last known alive'));
  assert.ok(view.includes('UNKNOWN · last known stopped'));

  const health = renderPanel(healthPanel, system);
  assert.ok(health.includes('which nodes are alive is unknown'));
  assert.ok(!health.includes('stopped heartbeating'));
  assert.ok(health.includes(`unknown of ${total}`));

  assert.ok(renderPanel(timingPanel, system).includes('SYSTEM GRAPH STALE'));

  const strip = render(
      <StudioProvider studio={createStaticStudio({system, playback: emptyPlayback})}>
        <StatusStrip/>
      </StudioProvider>);
  assert.ok(strip.includes(`?/${total} · STALE`), strip);

  // The same graph, current, says which node stopped.
  const fresh = renderPanel(healthPanel, {...system, stale: false});
  assert.ok(fresh.includes('stopped heartbeating'));
  assert.ok(!fresh.includes('SYSTEM GRAPH STALE'));
});

test('a node that stopped heartbeating is marked stale, not merely absent',
     () => {
       const graph = parseSystemGraph(demoSystemGraph(4000n));
       const dead = graph.nodes.filter(n => !n.alive);
       assert.ok(dead.length, 'demo graph has no stale node to render');

       const html = renderPanel(systemPanel, {
         graph,
         stale: false,
         rates: {},
         error: '',
         declared: null,
         declaredError: ''
       });
       assert.ok(html.includes('STALE'));
       for (const node of dead) assert.ok(html.includes(node.name));
     });

test('a node card says who drives its subsystem, and none when nobody does',
     () => {
       const card = (sequence: bigint, name: string) => {
         const graph = parseSystemGraph(demoSystemGraph(sequence));
         return render(<NodeCard graph={graph} stale={false}
                                 node={graph.nodes.find(n => n.name === name)!}
                                 rates={{}} selected={false}
                                 onSelect={() => {}}/>);
       };
       const driving = card(50n, 'drivetrain');
       assert.match(driving, /driven byteleop/);
       const idle = card(250n, 'drivetrain');
       assert.match(idle, /driven bynone/);
       assert.ok(!card(50n, 'pose_estimator').includes('driven by'));

       const graph = parseSystemGraph(demoSystemGraph(50n));
       const wiring = render(<Wiring graph={graph} stale={false}
                                     node={graph.nodes.find(
                                         n => n.name === 'drivetrain')!}
                                     rates={{}}/>);
       assert.ok(wiring.includes('driven by teleop'));
     });


test('a malformed document is reported in place of the graph it replaced', () => {
  const html = renderPanel(systemPanel, {
    graph: null,
    stale: true,
    rates: {},
    error: 'Invalid system graph: nodes[0].name must be a string',
    declared: null,
    declaredError: ''
  });
  assert.ok(html.includes('nodes[0].name must be a string'));
});

test('timing leaves passive source latency unknown but preserves measured zero', () => {
  const system = state();
  const template = system.graph!.nodes[0].sources[0];
  system.graph!.nodes = [{
    ...system.graph!.nodes[0],
    sources: ['SENDER', 'FETCHER', 'TIMER', 'WATCHER'].map((kind, id) => ({
      ...template, kind: kind as typeof template.kind, id, name: `source-${kind}`,
      last_latency_ns: '0', max_latency_ns: '0'
    }))
  }];
  const text = renderPanel(timingPanel, system);
  for (const kind of ['SENDER', 'FETCHER', 'TIMER', 'WATCHER']) {
    const start = text.indexOf(`source-${kind}`);
    assert.ok(start >= 0);
    const end = text.indexOf('source-', start + 1);
    const row = text.slice(start, end < 0 ? undefined : end);
    if (kind === 'SENDER' || kind === 'FETCHER') {
      assert.ok(!row.includes('0 µs'));
      assert.ok(row.includes('——'));
    } else {
      assert.ok(row.includes('0 µs0 µs'));
    }
  }
});

test('with no declared graph every view is exactly what it was before', () => {
  const system = state();
  assert.equal(system.declared, null);
  for (const [name, html] of [
         ['system', renderPanel(systemPanel, system)],
         ['overview', renderPanel(healthPanel, system)],
       ] as const) {
    for (const claim of ['Declared graph', 'nodes declared', 'never appeared',
                         'Launcher lint'])
      assert.ok(!html.includes(claim), `${name} claims "${claim}"`);
  }
});

test('the overview reads declared against live, and names what never appeared',
     () => {
       const html = renderPanel(healthPanel, state(true));
       assert.ok(html.includes('Declared'), 'no declared row');
       assert.ok(html.includes('7 nodes, 6 live'), 'declared vs live not shown');
       assert.ok(html.includes('Never appeared'));
       assert.ok(html.includes('shooter'));
       assert.ok(html.includes('0 errors, 1 warnings'), 'no lint tally');

     });

test('the attention list flags real orphans and leaves the design alone', () => {
  const html = renderPanel(healthPanel, state(true));
  assert.ok(html.includes('/shooter/state'), 'a real orphan was not flagged');
  assert.ok(html.includes('lapped'), 'the lossy topic was not flagged');

  for (const quiet of ['/hw/command', '/drivetrain/target/auto',
                       '/shooter/target/auto'])
    assert.ok(!html.includes(quiet), `${quiet} was flagged as a fault`);

  // A topic the launcher already spoke about is raised once, in the launcher's
  // words; the topic name also appears elsewhere as the bridge's own topic.
  assert.ok(html.includes('launcher: /talos/telemetry'));
  assert.ok(!html.includes('/talos/telemetry is published but nobody reads it'),
            'the same finding was reported twice');

  assert.ok(html.includes('/shooter/target/teleop'));
});

const workspaceCallbacks: WorkspaceCallbacks = {
  onSplit() {}, onClose() {}, onSwitch() {}, onConfig() {}, onResize() {},
  onMaximize() {}, onFocusPanel() {}
};

type ViewElement = React.ReactElement<{children?: React.ReactNode}>;
// Inspect the returned element tree without needing a browser DOM. Components
// using hooks are evaluated only inside a server-rendered probe.
function componentOutput(element: ViewElement): ViewElement {
  return (element.type as (props: typeof element.props) => ViewElement)(
      element.props);
}

test('panel identity keys reset panel state and errors when its type changes', () => {
  const panel = (panelId: string, instanceId = 'one') => {
    const area = WorkspaceArea({
      root: panelNode(panelId, instanceId, undefined),
      maximized: null, callbacks: workspaceCallbacks
    });
    const chrome = area.props.children as ViewElement;
    const boundary = chrome.props.children as ViewElement;
    const content = boundary.props.children as ViewElement;
    return [chrome.key, boundary.key, content.key];
  };
  assert.deepEqual(panel('health'), ['one', 'one:health', 'one:health']);
  assert.deepEqual(panel('timing'), ['one', 'one:timing', 'one:timing']);
  assert.deepEqual(panel('health', 'two'), ['two', 'two:health', 'two:health']);
});

test('split siblings retain instance keys when another sibling closes', () => {
  const keys = (ids: string[]) => {
    const area = WorkspaceArea({
      root: splitNode('row', ids.map(id => panelNode('health', id, undefined))),
      maximized: null, callbacks: workspaceCallbacks
    });
    const split = componentOutput(area.props.children as ViewElement);
    return (split.props.children as ViewElement[]).map(child => child.key);
  };
  assert.deepEqual(keys(['a', 'b', 'c']), ['a', 'b', 'c']);
  assert.deepEqual(keys(['b', 'c']), ['b', 'c']);
});

function splitHandle(sizes: number[], onResize: (sizes: number[]) => void) {
  const root = splitNode('row', sizes.map((_, i) =>
      panelNode('health', `panel-${i}`, undefined)));
  root.sizes = sizes;
  const area = WorkspaceArea({
    root, maximized: null,
    callbacks: {...workspaceCallbacks, onResize: (_path, next) => onResize(next)}
  });
  const split = componentOutput(area.props.children as ViewElement);
  const fragment = (split.props.children as ViewElement[])[1];
  const handleElement = (fragment.props.children as ViewElement[])[0];
  let handle!: React.ReactElement<React.HTMLAttributes<HTMLDivElement>>;
  function Probe() {
    handle = componentOutput(handleElement) as typeof handle;
    return handle;
  }
  renderToStaticMarkup(<Probe/>);
  return handle.props;
}

test('divider keyboard resizing preserves the pair and leaves other panes alone', () => {
  for (const sizes of [[0.2, 0.3, 0.5], [0.12, 0.18, 0.7], [0.05, 0.05, 0.9]]) {
    for (const key of ['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown']) {
      let resized: number[] = [];
      let prevented = false, stopped = false;
      const handle = splitHandle(sizes, next => { resized = next; });
      handle.onKeyDown!({
        key,
        preventDefault() { prevented = true; },
        stopPropagation() { stopped = true; }
      } as unknown as React.KeyboardEvent<HTMLDivElement>);
      assert.ok(prevented && stopped);
      assert.ok(Math.abs(resized[0] + resized[1] - sizes[0] - sizes[1]) < 1e-12);
      assert.equal(resized[2], sizes[2]);
      const minimum = Math.min(0.12, (sizes[0] + sizes[1]) / 2);
      assert.ok(resized[0] >= minimum && resized[1] >= minimum - 1e-12);
    }
  }
});

test('divider pointer movement uses the whole split size rather than pair size', () => {
  let resized: number[] = [];
  const handle = splitHandle([0.2, 0.3, 0.5], next => { resized = next; });
  handle.onPointerDown!({
    clientX: 100, pointerId: 1,
    currentTarget: {
      parentElement: {getBoundingClientRect: () => ({width: 1000, height: 500})},
      setPointerCapture() {}
    }
  } as unknown as React.PointerEvent<HTMLDivElement>);
  handle.onPointerMove!({clientX: 200} as React.PointerEvent<HTMLDivElement>);
  assert.ok(Math.abs(resized[0] - 0.3) < 1e-12);
  assert.ok(Math.abs(resized[1] - 0.2) < 1e-12);
  assert.equal(resized[2], 0.5);
  handle.onPointerMove!({clientX: 2000} as React.PointerEvent<HTMLDivElement>);
  assert.deepEqual(resized, [0.38, 0.12, 0.5]);
  handle.onPointerUp!({} as React.PointerEvent<HTMLDivElement>);
  handle.onPointerMove!({clientX: 0} as React.PointerEvent<HTMLDivElement>);
  assert.deepEqual(resized, [0.38, 0.12, 0.5]);
});

test('timeline slider is focusable and its arrows seek once within bounds', () => {
  const sought: string[] = [];
  for (const [cursor, key, expected] of [
    ['500', 'ArrowLeft', '490'], ['500', 'ArrowRight', '510'],
    ['0', 'ArrowLeft', '0'], ['1000', 'ArrowRight', '1000']
  ]) {
    const studio = createStaticStudio({
      playback: {...emptyPlayback, bounds: {start: '0', end: '1000'}, cursor_ns: cursor}
    });
    studio.actions.seek = value => { sought.push(value); };
    let slider!: React.ReactElement<React.SVGProps<SVGSVGElement>>;
    function Probe() {
      const timeline = Timeline({
        stepNs: '20', onStepNs() {}, transportError: '', onTransportError() {}
      });
      const wrap = (timeline.props.children as ViewElement[])[1];
      const track = (wrap.props.children as ViewElement[])[1];
      slider = componentOutput(track) as typeof slider;
      return slider;
    }
    renderToStaticMarkup(<StudioProvider studio={studio}><Probe/></StudioProvider>);
    assert.equal(slider.props.tabIndex, 0);
    let prevented = false, stopped = false;
    slider.props.onKeyDown!({
      key, preventDefault() { prevented = true; },
      stopPropagation() { stopped = true; }
    } as unknown as React.KeyboardEvent<SVGSVGElement>);
    assert.ok(prevented && stopped);
    assert.equal(sought.at(-1), expected);
  }
  assert.equal(sought.length, 4);
});
