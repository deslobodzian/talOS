// Unit tests for the command palette: the fuzzy-ish scorer/filter and the
// command list builders. These are pure functions, so they run under plain
// node with no data layer or DOM.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import type {RobotEvent} from '../src/data/types';
import {
  eventCommands,
  filterCommands,
  relativeEventTime,
  score,
} from '../src/shell/commands';

// --- scoring ---------------------------------------------------------------

test('score ranks exact, prefix, substring and subsequence matches', () => {
  assert.equal(score('abc', 'abc'), 1000);
  assert.equal(score('ab', 'abcdef'), 800);
  // "cd" is a substring at index 2: 600 - 2.
  assert.equal(score('cd', 'abcdef'), 598);
  // "ace" is a scattered subsequence: three single steps.
  assert.equal(score('ace', 'abcdef'), 3);
  // Case-insensitive.
  assert.equal(score('ABC', 'abc'), 1000);
  // No match at all.
  assert.equal(score('xz', 'abcdef'), -1);
  // Consecutive subsequence beats a scattered one.
  assert.ok(score('ab', 'xaby') > score('ab', 'axb'));
});

test('filterCommands returns everything for an empty query, best first', () => {
  const commands = [
    {id: 'a', title: 'Add panel Field', run: () => {}},
    {id: 'b', title: 'Go to page Robot', run: () => {}},
    {id: 'c', title: 'Open preset page System', run: () => {}},
  ];
  // Empty query: unchanged order.
  assert.deepEqual(filterCommands(commands, ''), commands);
  assert.deepEqual(filterCommands(commands, '   '), commands);

  // 'page' matches two; the one with the substring earlier ranks first.
  const result = filterCommands(commands, 'page');
  assert.deepEqual(result.map(c => c.id), ['b', 'c']);

  // 'panel' matches only one.
  assert.deepEqual(filterCommands(commands, 'panel').map(c => c.id), ['a']);

  // No match yields none.
  assert.deepEqual(filterCommands(commands, 'zzz'), []);

  // An exact title outranks a longer prefix and a keyword-only match, however
  // the query is cased or padded.
  const connect = [
    {id: 'prefix', title: 'Connect robot', run() {}},
    {id: 'keyword', title: 'A command', keywords: 'Connect', run() {}},
    {id: 'exact', title: 'Connect', hint: 'Disconnected', run() {}}
  ];
  for (const query of ['connect', ' CONNECT '])
    assert.deepEqual(filterCommands(connect, query).map(c => c.id),
                     ['exact', 'prefix', 'keyword']);
});

// --- command builders ------------------------------------------------------

test('relativeEventTime distinguishes the future, present and past', () => {
  assert.equal(relativeEventTime('2000000000', '1000000000'), '+1.00 s');
  assert.equal(relativeEventTime('1000000000', '1000000000'), 'now');
  assert.equal(relativeEventTime('1000000000', '2000000000'), '1.00 s ago');
  assert.equal(relativeEventTime('0', null), '');
});

test('eventCommands sorts exact decimal timestamps numerically before limiting', () => {
  const times = ['9', '10', '9007199254740992', '9007199254740993'];
  const events: RobotEvent[] = times.map(t_ns =>
    ({id: t_ns, t_ns, severity: 'info', category: 'system', title: t_ns}));
  const sought: string[] = [];
  const commands = eventCommands(events, time => sought.push(time), null, 3);
  for (const command of commands) command.run();
  assert.deepEqual(sought, ['9007199254740993', '9007199254740992', '10']);
  assert.deepEqual(events.map(event => event.t_ns), times);
});

