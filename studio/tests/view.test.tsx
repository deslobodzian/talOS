// The migrated view tests: the old system_view assertions, now exercised
// against the panels that replaced those views. Panels read their data through
// hooks, so they render inside a StudioProvider backed by createStaticStudio;
// the pure sub-components (NodeCard, Wiring, DeclaredPanel) still take their
// data as props and render directly.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {createStaticStudio, StudioProvider} from '../src/data/studio';
import type {PlaybackState, SystemState} from '../src/data/types';
import {demoDeclaredGraph, demoSystemGraph} from '../src/demo';
import {agentPanel} from '../src/panels/agent';
import {healthPanel} from '../src/panels/health';
import type {AnyPanelDefinition} from '../src/panels/types';
import {DeclaredPanel, NodeCard, systemPanel, Wiring} from '../src/panels/system';
import {timingPanel} from '../src/panels/timing';
import {RPC_METHODS} from '../agent/rpc';
import {
  eventRates,
  parseDeclaredGraph,
  parseSystemGraph,
  rateKey,
} from '../src/system';

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
    rates,
    error: '',
    declared: declared ? parseDeclaredGraph(demoDeclaredGraph()) : null,
    declaredError: ''
  };
}

const noSystem: SystemState =
    {graph: null, rates: {}, error: '', declared: null, declaredError: ''};

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

test('the system view names every node and every topic it was given', () => {
  const system = state();
  const html = renderPanel(systemPanel, system);

  for (const node of system.graph!.nodes) {
    assert.ok(html.includes(node.name), `missing node ${node.name}`);
    assert.ok(html.includes(node.target), `missing target ${node.target}`);
  }
  for (const topic of system.graph!.topics)
    assert.ok(html.includes(topic.name), `missing topic ${topic.name}`);

  const alive = system.graph!.nodes.filter(n => n.alive).length;
  assert.ok(html.includes(`${alive}/${system.graph!.nodes.length} nodes live`));
  assert.ok(html.includes(`${system.graph!.topics.length} topics`));
});

test('the system view surfaces the faults the demo deliberately carries', () => {
  const html = renderPanel(systemPanel, state());
  assert.ok(html.includes('/shooter/state'), 'the orphaned topic is missing');
});

test('a half-wired topic its own node declared is not shown as a fault', () => {
  const html = renderPanel(systemPanel, state());
  assert.ok(html.includes('bridged'), 'external endpoint status missing');
  assert.ok(html.includes('unconnected'), 'optional endpoint status missing');
  const system = state();
  const drivetrain = system.graph!.nodes.find(n => n.name === 'drivetrain')!;
  const wiring = render(<Wiring graph={system.graph!} node={drivetrain}
                                rates={system.rates}/>);
  assert.ok(wiring.includes('optional'));
});

test('a node that stopped heartbeating is marked stale, not merely absent',
     () => {
       const graph = parseSystemGraph(demoSystemGraph(4000n));
       const dead = graph.nodes.filter(n => !n.alive);
       assert.ok(dead.length, 'demo graph has no stale node to render');

       const html = renderPanel(systemPanel, {
         graph,
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
         return render(<NodeCard graph={graph}
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
       const wiring = render(<Wiring graph={graph}
                                     node={graph.nodes.find(
                                         n => n.name === 'drivetrain')!}
                                     rates={{}}/>);
       assert.ok(wiring.includes('driven by teleop'));
     });

test('every view renders without a system graph and says why', () => {
  for (const [name, html] of [
         ['system', renderPanel(systemPanel, noSystem)],
         ['timing', renderPanel(timingPanel, noSystem)],
         ['overview', renderPanel(healthPanel, noSystem)],
       ] as const) {
    assert.ok(html.length > 0, `${name} rendered nothing`);
  }
});

test('a malformed document is reported in place of the graph it replaced', () => {
  const html = renderPanel(systemPanel, {
    graph: null,
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

test('timing lists one row per source, with declared and measured rates', () => {
  const system = state();
  const html = renderPanel(timingPanel, system);

  const sources = system.graph!.nodes.reduce((n, node) => n + node.sources.length, 0);
  assert.ok(html.includes(`${sources} SOURCES`),
            `expected ${sources} sources in the header`);

  for (const kind of ['SENDER', 'WATCHER', 'FETCHER', 'TIMER'])
    assert.ok(html.includes(kind), `missing source kind ${kind}`);

  const timer = system.graph!.nodes.flatMap(
                                 node => node.sources.map(source => ({node, source})))
                    .find(({node, source}) => source.kind === 'TIMER' &&
                              node.alive &&
                              system.rates[rateKey(node.name, source.id)] > 0);
  assert.ok(timer, 'demo graph has no measured timer to check');
  const declared = 1e9 / Number(timer!.source.period_ns);
  const measured = system.rates[rateKey(timer!.node.name, timer!.source.id)];
  assert.ok(Math.abs(declared - measured) / declared < 0.05,
            `declared ${declared} Hz vs measured ${measured} Hz`);
  assert.ok(html.includes('Hz'));
});

test('the overview routes each attention item to the panel that explains it',
     () => {
       const html = renderPanel(healthPanel, state());
       assert.ok(html.includes('/shooter/state'));
     });

test('the agent tab offers every RPC method the server implements', () => {
  const html = renderPanel(agentPanel, noSystem);
  for (const method of RPC_METHODS)
    assert.ok(html.includes(method), `method ${method} not offered`);
  assert.ok(html.includes('get_declared_graph'));
});

test('the system tab reports what was declared against what registered', () => {
  const html = renderPanel(systemPanel, state(true));
  assert.ok(html.includes('7 nodes declared'), 'declared count missing');
  assert.ok(html.includes('6 registered'), 'registered count missing');
  assert.ok(html.includes('1 never appeared'), 'no missing-node count');
  assert.ok(html.includes('shooter'));
  assert.ok(html.includes('//2026-robot/main_processor/shooter:node'),
            'the missing node does not say what would have built it');
  assert.ok(html.includes('7 declared'), 'the toolbar count is missing');

  assert.ok(html.includes('/talos/telemetry'));
});

test('a running node nobody declared is a different sentence', () => {
  const system = state(true);
  const declared = {
    ...system.declared!,
    nodes: system.declared!.nodes.filter(node => node.name !== 'telemetry')
  };
  const html = render(<DeclaredPanel declared={declared} graph={system.graph}/>);
  assert.ok(html.includes('1 not declared'));
  assert.ok(html.includes('running but not declared'));
});

test('a declared graph with nothing to report says so rather than nothing', () => {
  const html = render(<DeclaredPanel
      declared={{version: 1, nodes: [], diagnostics: []}} graph={null}/>);
  assert.ok(html.includes('0 nodes declared'));
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

test('a bridge too old to state endpoint attributes says so, and still renders',
     () => {
       const older = JSON.parse(demoSystemGraph(1500n));
       older.version = 1;
       for (const node of older.nodes)
         for (const source of node.sources) {
           delete source.external;
           delete source.optional;
         }
       const graph = parseSystemGraph(JSON.stringify(older));
       const system: SystemState =
           {graph, rates: {}, error: '', declared: null, declaredError: ''};

       const html = renderPanel(systemPanel, system);
       assert.ok(html.includes('/hw/command'), 'the graph itself is missing');

       assert.ok(renderPanel(healthPanel, system).includes('/shooter/state'));
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
