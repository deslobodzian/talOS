// Renders the system-graph views and asserts what a person would read on
// screen.
//
// A successful typecheck and a successful bundle prove the code compiles and
// resolves, not that it runs: a hook in the wrong place, a BigInt() on a field
// that turned out to be a number, or an undefined access inside a map all
// survive both and fail on the first paint. These views touch no DOM API, so
// react-dom/server can render them here and catch that.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {demoDeclaredGraph, demoSystemGraph} from '../src/demo';
import {
  eventRates,
  parseDeclaredGraph,
  parseSystemGraph,
  rateKey,
} from '../src/system';
import {RPC_METHODS} from '../agent/rpc';
import {
  AgentTab,
  DeclaredPanel,
  OverviewTab,
  SystemTab,
  type SystemState,
  TimingTab,
  type View,
} from '../src/system_view';

const emptyView: View = {
  snapshot: null,
  history: [],
  mode: 'PAUSED',
  bounds: null,
  stats: {dropped: 0, late: 0, invalid: 0},
  status: 'Disconnected'
};

// Two samples apart, so rates are measured rather than absent: a view that
// only works with no history would pass a single-sample test.
//
// `declared` is passed separately because a bridge started without --declared
// is the ordinary case, and both paths have to render.
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

const render = (element: React.ReactElement) => renderToStaticMarkup(element);

test('the system view names every node and every topic it was given', () => {
  const system = state();
  const html = render(<SystemTab system={system}/>);

  for (const node of system.graph!.nodes) {
    assert.ok(html.includes(node.name), `missing node ${node.name}`);
    assert.ok(html.includes(node.target), `missing target ${node.target}`);
  }
  for (const topic of system.graph!.topics)
    assert.ok(html.includes(topic.name), `missing topic ${topic.name}`);

  // The counts a person scans first.
  const alive = system.graph!.nodes.filter(n => n.alive).length;
  assert.ok(html.includes(`${alive}/${system.graph!.nodes.length} nodes live`));
  assert.ok(html.includes(`${system.graph!.topics.length} topics`));
});

test('the system view surfaces the faults the demo deliberately carries', () => {
  const html = render(<SystemTab system={state()}/>);
  // The demo graph carries one topic of every kind on purpose. The chips name
  // every state whether or not a row has it, so these assert on the note each
  // row carries -- which only appears when a topic is actually in that state.
  assert.ok(html.includes('subscribed but nothing publishes it'),
            'no orphaned topic shown');
  assert.ok(html.includes('messages were lapped'), 'no lossy topic shown');
  assert.ok(html.includes('published but nobody reads it'),
            'no unread topic shown');
  assert.ok(html.includes('/shooter/state'), 'the orphaned topic is missing');
  assert.ok(html.includes('publishes'), 'wiring did not label an outgoing edge');
  assert.ok(html.includes('Subscribes'), 'wiring has no subscriber column');
});

test('a half-wired topic its own node declared is not shown as a fault', () => {
  const html = render(<SystemTab system={state()}/>);
  // /hw/command's consumer is the RoboRIO across UDP and /hw/command/override
  // has no publisher by design. Both are half-wired, neither is broken, and a
  // viewer that calls them broken is why nobody reads the fault list.
  assert.ok(html.includes('no shared-memory peer will appear'),
            'an external end was not labelled bridged');
  assert.ok(html.includes('declared optional; no peer yet'),
            'an optional end was not labelled unconnected');
  assert.ok(html.includes('>bridged<'), 'no bridged badge in the table');
  assert.ok(html.includes('>unconnected<'), 'no unconnected badge in the table');

  // And the wiring view agrees with the table about the same two ends, rather
  // than drawing them as broken wires.
  assert.ok(html.includes('outside talOS'),
            'wiring calls an external end a missing peer');
  assert.ok(html.includes('optional — no peer yet'),
            'wiring calls an optional end a missing peer');
});

test('a node that stopped heartbeating is marked stale, not merely absent',
     () => {
       // Late enough that the demo's stopped node has crossed the liveness
       // timeout: a viewer that silently drops a dead node is worse than one
       // that shows it as dead.
       const graph = parseSystemGraph(demoSystemGraph(4000n));
       const dead = graph.nodes.filter(n => !n.alive);
       assert.ok(dead.length, 'demo graph has no stale node to render');

       const html = render(<SystemTab system={{
         graph,
         rates: {},
         error: '',
         declared: null,
         declaredError: ''
       }}/>);
       assert.ok(html.includes('STALE'));
       for (const node of dead) assert.ok(html.includes(node.name));
     });

test('every view renders without a system graph and says why', () => {
  for (const [name, element] of [
         ['system', <SystemTab system={noSystem}/>],
         ['timing', <TimingTab system={noSystem}/>],
         ['overview', <OverviewTab view={emptyView} system={noSystem}
                                   onOpen={() => {}}/>],
       ] as const) {
    const html = render(element);
    assert.ok(html.length > 0, `${name} rendered nothing`);
    assert.ok(html.includes('No system graph yet') || html.includes('Connect'),
              `${name} did not explain the empty state`);
  }
});

test('a malformed document is reported in place of the graph it replaced', () => {
  const html = render(<SystemTab system={{
    graph: null,
    rates: {},
    error: 'Invalid system graph: nodes[0].name must be a string',
    declared: null,
    declaredError: ''
  }}/>);
  assert.ok(html.includes('nodes[0].name must be a string'));
});

test('timing lists one row per source, with declared and measured rates', () => {
  const system = state();
  const html = render(<TimingTab system={system}/>);

  const sources = system.graph!.nodes.reduce((n, node) => n + node.sources.length, 0);
  assert.ok(html.includes(`${sources} SOURCES`),
            `expected ${sources} sources in the header`);

  // Kind badges for every kind the demo declares, so the table is legible at a
  // glance rather than needing the column read.
  for (const kind of ['SENDER', 'WATCHER', 'FETCHER', 'TIMER'])
    assert.ok(html.includes(`kind ${kind}`), `no badge for ${kind}`);

  // A timer's declared rate comes from period_ns; a measured one from the
  // counters. Both must appear, and for a periodic source they should agree.
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

test('the overview routes each attention item to the tab that explains it',
     () => {
       const opened: string[] = [];
       const html = render(<OverviewTab view={emptyView} system={state()}
                                        onOpen={tab => opened.push(tab)}/>);
       // Built from the demo's deliberate faults, so the list must not be empty
       // and must offer a way to each one.
       assert.ok(html.includes('ITEMS'), 'attention list reported no items');
       assert.ok(html.includes('nothing publishes it'));
       assert.ok(html.includes('open'));
     });

test('the agent tab offers every RPC method the server implements', () => {
  const html = render(<AgentTab call={async () => ({})}/>);
  // Read from RPC_METHODS by the tab itself, so a method that exists and is
  // not offered is a bug this catches rather than a list to remember.
  for (const method of RPC_METHODS)
    assert.ok(html.includes(method), `method ${method} not offered`);
  assert.ok(html.includes('get_declared_graph'));
});

test('the system tab reports what was declared against what registered', () => {
  const html = render(<SystemTab system={state(true)}/>);
  // The demo declares a node that never claimed a registry slot. Which node,
  // what would have built it, and why it matters are all on screen: the live
  // graph alone can only be silent about it.
  assert.ok(html.includes('8 nodes declared'), 'declared count missing');
  assert.ok(html.includes('7 registered'), 'registered count missing');
  assert.ok(html.includes('1 never appeared'), 'no missing-node count');
  assert.ok(html.includes('declared, never registered'),
            'the missing node is not explained');
  assert.ok(html.includes('shooter'));
  assert.ok(html.includes('//2026-robot/main_processor/shooter:node'),
            'the missing node does not say what would have built it');
  assert.ok(html.includes('8 declared'), 'the toolbar count is missing');

  // The launcher's own sentence, not the viewer's paraphrase of it.
  assert.ok(html.includes('is published but nothing subscribes to it'),
            'the launcher lint is not shown');
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
  assert.ok(html.includes('started outside the launcher'));
});

test('a declared graph with nothing to report says so rather than nothing', () => {
  const html = render(<DeclaredPanel
      declared={{version: 1, nodes: [], diagnostics: []}} graph={null}/>);
  assert.ok(html.includes('0 nodes declared'));
  // No live graph to compare against, so it does not claim every declared node
  // went missing.
  assert.ok(html.includes('nothing to compare against'));
  assert.ok(!html.includes('never appeared'));
  // react-dom/server escapes the apostrophe in "launcher's".
  assert.ok(html.includes('lint found nothing to report'));
});

test('with no declared graph every view is exactly what it was before', () => {
  // The bridge may be run without --declared, and the desktop UDP path has no
  // HTTP endpoint to ask. Neither is a fault, so neither may produce an empty
  // panel or a count of zero that nobody claimed.
  const system = state();
  assert.equal(system.declared, null);
  for (const [name, html] of [
         ['system', render(<SystemTab system={system}/>)],
         ['overview', render(<OverviewTab view={emptyView} system={system}
                                          onOpen={() => {}}/>)],
       ] as const) {
    // The panel, the counts and the missing-node line, none of which anyone
    // sent a document for. (The word "declared" itself still appears: an
    // optional endpoint's note says a peer was declared optional, and the
    // timing table has a declared-rate column.)
    for (const claim of ['Declared graph', 'nodes declared', ' declared</span>',
                         'never appeared', 'nothing to compare against',
                         'Launcher lint'])
      assert.ok(!html.includes(claim), `${name} claims "${claim}"`);
  }
});

test('a bridge too old to state endpoint attributes says so, and still renders',
     () => {
       // What the bridge sent before version 2. Studio must show the graph --
       // an empty viewer is worse than a slightly wrong one -- but the two
       // topics it cannot classify honestly are exactly the ones this release
       // stopped calling faults, so it has to admit which document it has.
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

       const html = render(<SystemTab system={system}/>);
       assert.ok(html.includes('bridge sends version 1 graphs'),
                 'the older document is not identified');
       assert.ok(html.includes('/hw/command'), 'the graph itself is missing');
       // Which is the old answer for it, unchanged, rather than a blank view.
       assert.ok(html.includes('published but nobody reads it'));
       assert.ok(!html.includes('no shared-memory peer will appear'),
                 'a v1 document cannot know an end is external');

       const overview = render(<OverviewTab view={emptyView} system={system}
                                            onOpen={() => {}}/>);
       assert.ok(overview.includes('carry no endpoint'),
                 'the attention list does not admit what it cannot tell');
     });

test('the overview reads declared against live, and names what never appeared',
     () => {
       const html = render(<OverviewTab view={emptyView} system={state(true)}
                                        onOpen={() => {}}/>);
       assert.ok(html.includes('Declared'), 'no declared row');
       assert.ok(html.includes('8 nodes, 7 live'), 'declared vs live not shown');
       assert.ok(html.includes('Never appeared'));
       assert.ok(html.includes('shooter'));
       assert.ok(html.includes('0 errors, 1 warnings'), 'no lint tally');

       // In the attention list, with the launcher's own words, routed to a tab.
       assert.ok(html.includes('was declared and never registered'));
       assert.ok(html.includes('launcher: /talos/telemetry is published'));
     });

test('the attention list flags real orphans and leaves the design alone', () => {
  const html = render(<OverviewTab view={emptyView} system={state(true)}
                                   onOpen={() => {}}/>);
  // The orphan the demo carries reaches the list, named with its readers.
  assert.ok(html.includes('/shooter/state is read by odometry'),
            'a real orphan was not flagged');
  assert.ok(html.includes('lapped'), 'the lossy topic was not flagged');

  // The two topics that are half-wired by declaration must not be in it. That
  // is the whole point: a list that reports the design is a list people learn
  // to skip, and then it is not there for the orphan above.
  for (const quiet of ['/hw/command is', '/hw/command/override',
                       '/drivetrain/target/auto', '/shooter/target/auto'])
    assert.ok(!html.includes(quiet), `${quiet} was flagged as a fault`);

  // And a topic the launcher already spoke about is raised in the launcher's
  // words rather than twice in two voices. (The name also appears elsewhere on
  // the page as the bridge's topic, so this asserts on the two sentences.)
  assert.ok(html.includes('launcher: /talos/telemetry'));
  assert.ok(!html.includes('/talos/telemetry is published but nobody reads it'),
            'the same finding was reported twice');

  // The consequences of the node that never started are still each their own
  // item: the launcher's lint saw a graph in which the shooter was present.
  assert.ok(html.includes('/shooter/target is published but nobody reads it'));
});
