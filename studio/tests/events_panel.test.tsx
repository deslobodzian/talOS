// Events panel tests: the pure filtering/formatting logic and config parsing.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import type {EventCategory, RobotEvent, Severity} from '../src/data/types';
import {eventsPanel} from '../src/panels/events';
import {
  CATEGORIES,
  filterEvents,
  formatRelative
} from '../src/panels/events/filter';

const ev = (id: string, t_ns: string, severity: Severity,
            category: EventCategory, title: string, detail?: string,
            channel?: string): RobotEvent =>
    ({id, t_ns, severity, category, title, detail, channel});

// 2026-scale nanosecond timestamps: 1.7e18 plus small offsets.
const CURSOR = '1700000000000000000';
const PAST = '1699999996800000000';     // -3.2 s
const FUTURE = '1700000001000000000';   // +1.0 s

const ALL: {severities: Severity[]; categories: EventCategory[]; text: string} = {
  severities: ['info', 'warn', 'error'],
  categories: [...CATEGORIES],
  text: ''
};

test('filterEvents keeps matching events, newest first', () => {
  const events: RobotEvent[] = [
    ev('a:1:0', '1000000000', 'info', 'power', 'Power recovered'),
    ev('a:2:0', '3000000000', 'error', 'power', 'Brownout', 'bus dipped'),
    ev('a:3:0', '2000000000', 'warn', 'vision', 'Tag lost', 'rejected',
       'front_left_camera/state/tag_count')
  ];

  assert.deepEqual(filterEvents(events, ALL).map(e => e.id),
                   ['a:2:0', 'a:3:0', 'a:1:0']);

  assert.deepEqual(
      filterEvents(events, {...ALL, severities: ['error']}).map(e => e.id),
      ['a:2:0']);

  assert.deepEqual(
      filterEvents(events, {...ALL, categories: ['vision']}).map(e => e.id),
      ['a:3:0']);

  // Text matches title, detail and channel, case-insensitively.
  assert.deepEqual(filterEvents(events, {...ALL, text: 'BROWN'}).map(e => e.id),
                   ['a:2:0']);
  assert.deepEqual(filterEvents(events, {...ALL, text: 'dipped'}).map(e => e.id),
                   ['a:2:0']);
  assert.deepEqual(
      filterEvents(events, {...ALL, text: 'tag_count'}).map(e => e.id),
      ['a:3:0']);

  // An empty membership set shows nothing; it is not "all".
  assert.deepEqual(filterEvents(events, {...ALL, severities: []}), []);
  assert.deepEqual(filterEvents(events, {...ALL, categories: []}), []);
});

test('formatRelative reports the signed offset and future flag', () => {
  assert.deepEqual(formatRelative(PAST, CURSOR),
                   {label: '-3.2 s', future: false});
  assert.deepEqual(formatRelative(FUTURE, CURSOR),
                   {label: '+1.0 s', future: true});
  assert.deepEqual(formatRelative(CURSOR, CURSOR),
                   {label: 'now', future: false});
  assert.deepEqual(formatRelative(CURSOR, null),
                   {label: '—', future: false});
});

test('events config parsing accepts valid input and cleans malformed input',
     () => {
       assert.deepEqual(
           eventsPanel.parseConfig(
               {severities: ['error'], categories: ['power'], text: 'x',
                follow: false}),
           {severities: ['error'], categories: ['power'], text: 'x',
            follow: false});
       assert.deepEqual(eventsPanel.parseConfig(null), eventsPanel.defaultConfig);
       assert.deepEqual(
           eventsPanel.parseConfig(
               {severities: ['info', 'nope', 'warn'], categories: ['vision', 2],
                text: 5, follow: 'yes'}),
           {severities: ['info', 'warn'], categories: ['vision'], text: '',
            follow: true});
       // The default config is a fixed point of its own parser.
       assert.deepEqual(eventsPanel.parseConfig(eventsPanel.defaultConfig),
                        eventsPanel.defaultConfig);
     });
