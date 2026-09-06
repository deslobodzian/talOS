// The seam between the launcher and Studio, tested against a real document.
//
// `talOS/launcher` writes the declared graph in C++ and `studio/src/system.ts`
// parses it in TypeScript, and nothing in either language can check the other.
// The parser here was in fact written before the writer existed, so its
// envelope was a guess -- an optional `kind`, an optional `version`, severities
// in either case -- and a guess about a format nobody has produced yet is the
// same shape of problem as two spellings of one topic name.
//
// `launcher_graph.json` is therefore not hand-written. It is the output of
//
//   bazel run //talOS/launcher:launcher -- --describe-only \
//     --config 2026-robot/main_processor/configuration/robot.toml
//
// against the real robot, with only `session_id` pinned so a diff is a diff in
// shape rather than in session. If the launcher's envelope changes, this test
// fails and names the field -- regenerate the fixture and fix the parser
// together, in one commit.

import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {test} from 'node:test';

import {declaredStatus, parseDeclaredGraph, parseSystemGraph} from '../src/system';
import {demoSystemGraph} from '../src/demo';

const fixture = readFileSync(new URL('./launcher_graph.json', import.meta.url), 'utf8');

// The same robot with the historical `/hw/req/drive` misspelling put back, so
// the diagnostic path is pinned against real launcher output too. A parser that
// only ever sees clean documents is untested on the one case that matters.
const faults =
    readFileSync(new URL('./launcher_graph_faults.json', import.meta.url), 'utf8');

test('the parser accepts what the launcher actually writes', () => {
  const graph = parseDeclaredGraph(fixture);

  // The roster, as `--describe` found it.
  assert.equal(graph.nodes.length, 8);
  assert.deepEqual(graph.nodes.map(n => n.name).sort(), [
    'arbiter', 'driver_station', 'drivetrain', 'hardware_node', 'odometry',
    'operator_interface', 'shooter', 'telemetry'
  ]);
  for (const node of graph.nodes) {
    assert.ok(node.target.startsWith('//'), `${node.name} has no build target`);
    assert.ok(node.sources.length > 0, `${node.name} declared no sources`);
  }

  // A clean graph carries no findings. This is the assertion that would catch
  // the launcher renaming `diagnostics` or nesting it per node: an absent key
  // parses as empty, so the count below is the only thing that distinguishes
  // "no findings" from "findings the parser could not see".
  assert.equal(graph.diagnostics.length, 0);
});

test('endpoint attributes survive the crossing', () => {
  const graph = parseDeclaredGraph(fixture);
  const ends = graph.nodes.flatMap(
      node => node.sources.map(source => ({node: node.name, ...source})));

  // The five ends that used to read as faults. Named individually rather than
  // counted, because the point of the flags is which ones carry them.
  const external = ends.filter(e => e.external).map(e => e.name).sort();
  assert.deepEqual(external, ['/hw/command', '/talos/telemetry']);

  const optional = ends.filter(e => e.optional).map(e => e.name).sort();
  assert.deepEqual(
      optional,
      ['/drivetrain/target/auto', '/hw/command/override', '/shooter/target/auto']);

  // Everything else is an ordinary end, not an unset field read as false.
  assert.equal(ends.filter(e => !e.external && !e.optional).length,
               ends.length - 5);
});

test('every declared topic name conforms to the naming protocol', () => {
  const graph = parseDeclaredGraph(fixture);
  // The grammar from talOS/NAMING.md, checked from this side too: the launcher
  // enforces it before a launch, and a viewer that renders a name the protocol
  // forbids is showing something that cannot have come from a running robot.
  const ROLES = ['state', 'target', 'command', 'request', 'status', 'event',
                 'telemetry'];
  for (const node of graph.nodes) {
    for (const source of node.sources) {
      if (source.kind === 'TIMER') continue;  // A label, not an address.
      assert.ok(source.name.length <= 31,
                `${source.name} is ${source.name.length} characters; the shm cap is 31`);
      const segments = source.name.split('/').slice(1);
      assert.ok(segments.length >= 2 && segments.length <= 4, source.name);
      for (const segment of segments)
        assert.match(segment, /^[a-z][a-z0-9_]*$/, `${source.name}: ${segment}`);
      assert.ok(ROLES.includes(segments[1]),
                `${source.name} has role '${segments[1]}'`);
    }
  }
});

test('a node that never registered is named, with its cause', () => {
  const declared = parseDeclaredGraph(fixture);
  // The live graph the demo builds omits some declared nodes, which is the one
  // question the registry cannot answer alone: "was this supposed to be here?"
  const live = parseSystemGraph(demoSystemGraph(1500n));
  const status = declaredStatus(declared, live);
  assert.ok(status, 'declaredStatus returned null for two present graphs');
  assert.equal(status!.declared, 8);
  assert.ok(status!.missing.length > 0,
            'the demo live graph declares every fixture node, so this proves nothing');
  for (const node of status!.missing)
    assert.ok(declared.nodes.some(n => n.name === node.name));
});

test('the launcher\'s findings cross the language boundary intact', () => {
  const graph = parseDeclaredGraph(faults);
  assert.equal(graph.diagnostics.length, 2,
               'the faults fixture should carry exactly the two findings the ' +
                   'misspelling produces');

  // Severity survives the C++ enumerator being spelled ERROR and printed
  // lowercase, which was a real guess in this parser.
  for (const d of graph.diagnostics) assert.equal(d.severity, 'error');

  const subjects = graph.diagnostics.map(d => d.subject).sort();
  assert.deepEqual(subjects, ['/hw/req/drive', '/hw/request/drivetrain']);

  // The sentences are the launcher's, not re-derived here: a viewer that
  // rewords them says something subtly different from the tool that refused
  // the launch.
  const messages = graph.diagnostics.map(d => d.message).join(' ');
  assert.match(messages, /is an abbreviation; use 'request'/);
  assert.match(messages, /the two ends are spelled differently/);
});

test('a malformed document is reported against the right subject', () => {
  const document = JSON.parse(fixture);
  document.nodes[0].sources[0].message_bytes = 'lots';
  assert.throws(() => parseDeclaredGraph(JSON.stringify(document)),
                /declared graph/,
                'the error must say which document was wrong');

  // Truncation, the way a half-written file arrives when the launcher is
  // writing it while the bridge is reading.
  assert.throws(() => parseDeclaredGraph(fixture.slice(0, 200) + '}'),
                /declared graph|JSON/);
});
