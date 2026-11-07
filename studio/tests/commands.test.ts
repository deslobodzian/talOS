// Unit tests for the command palette: the fuzzy-ish scorer/filter and the
// command list builders. These are pure functions, so they run under plain
// node with no data layer or DOM.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import type {RobotEvent} from '../src/data/types';
import {
  eventCommands,
  filterCommands,
  pageCommands,
  panelCommands,
  presetCommands,
  relativeEventTime,
  score,
  transportCommands,
  workspaceCommands,
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
});

// --- command builders ------------------------------------------------------

test('pageCommands names every page and marks the active one', () => {
  const commands = pageCommands(
      [{id: 'a', title: 'Robot'}, {id: 'b', title: 'Signals'}], 'a',
      () => {});
  assert.deepEqual(commands.map(c => c.id), ['page:a', 'page:b']);
  assert.equal(commands[0].title, 'Go to page Robot');
  assert.equal(commands[0].hint, 'current');
  assert.equal(commands[1].hint, undefined);
});

test('panelCommands names every registered panel with its category', () => {
  const commands = panelCommands([
    {id: 'field', title: 'Field', category: 'Spatial', description: 'Top-down view'}
  ], () => {});
  assert.equal(commands.length, 1);
  assert.equal(commands[0].id, 'panel:field');
  assert.equal(commands[0].title, 'Add panel Field');
  assert.equal(commands[0].hint, 'Spatial');
  assert.ok(commands[0].keywords!.includes('Top-down view'));
});

test('presetCommands names every preset with its description', () => {
  const commands = presetCommands([{name: 'Robot', description: 'Health + field'}],
                                  () => {});
  assert.equal(commands[0].id, 'preset:Robot');
  assert.equal(commands[0].title, 'Open preset page Robot');
  assert.equal(commands[0].hint, 'Health + field');
});

test('relativeEventTime measures from the cursor', () => {
  assert.equal(relativeEventTime('0', null), '');
  assert.equal(relativeEventTime('0', '0'), 'now');
  assert.equal(relativeEventTime('0', '1000000'), '1.00 ms ago');
});

test('eventCommands lists newest first, capped, with a relative hint', () => {
  const event = (id: string, t_ns: string): RobotEvent =>
      ({id, t_ns, severity: 'info', category: 'system', title: id});
  const events = [event('e1', '100'), event('e2', '300'), event('e3', '200')];

  const commands = eventCommands(events, () => {}, '1000', 2);
  // Newest first: 300, 200; the 100 event is cut by the limit.
  assert.deepEqual(commands.map(c => c.id), ['event:e2', 'event:e3']);
  assert.equal(commands[0].title, 'Seek to event e2');
  assert.ok(commands[0].hint!.includes('ago'));
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

test('transportCommands toggles the play/pause label with the mode', () => {
  const callbacks = {
    onConnect: () => {}, onDemo: () => {}, onLive: () => {}, onPlayPause: () => {}
  };
  const paused = transportCommands({...callbacks, mode: 'PAUSED', status: 'x'});
  assert.equal(paused.find(c => c.id === 'play-pause')!.title, 'Play');

  const playing = transportCommands(
      {...callbacks, mode: 'REPLAY_PLAYING', status: 'x'});
  assert.equal(playing.find(c => c.id === 'play-pause')!.title, 'Pause');

  const live = transportCommands(
      {...callbacks, mode: 'LIVE_STREAMING', status: 'x'});
  assert.equal(live.find(c => c.id === 'play-pause')!.title, 'Pause');
});

test('workspaceCommands offers import and export', () => {
  const commands = workspaceCommands(() => {}, () => {});
  assert.deepEqual(commands.map(c => c.id),
                   ['workspace:import', 'workspace:export']);
});
