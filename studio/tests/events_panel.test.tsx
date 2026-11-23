// Events panel tests: the pure filtering/formatting logic and config parsing.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {EMPTY_TIMELINE, RecordedSource} from '../src/data/recorded';
import {createStaticStudio, StudioProvider} from '../src/data/studio';
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

test('clicking a recorded event row seeks the RecordedSource to its timestamp', () => {
  const fetchFn = () => new Promise<Response>(() => {});
  const clock = {now: () => 0, every: () => () => {}};
  const recorded = new RecordedSource(
      'http://review.test', 's1', [],
      {start_ns: '1000000000', end_ns: '5000000000', origin_wall_ns: '0',
       max_series_values: 1000, max_series_channels: 10, gaps: []},
      fetchFn, clock,
      {
        ...EMPTY_TIMELINE,
        alerts: [{channel: 'shooter/status/alert_error_jam', level: 'error',
                  start_ns: '2500000000', end_ns: '3500000000'}]
      },
      null);
  const studio = createStaticStudio();
  studio.openRecorded(recorded);

  // Inspect the rendered tree to invoke the row's onClick handler directly.
  const clicks: (() => void)[] = [];
  function walk(node: unknown) {
    if (!node || typeof node !== 'object') return;
    if (Array.isArray(node)) {
      for (const item of node) walk(item);
      return;
    }
    const el = node as React.ReactElement<Record<string, unknown>>;
    if (typeof el.type === 'function') {
      walk((el.type as (p: unknown) => unknown)(el.props));
      return;
    }
    if (el.props?.className && String(el.props.className).includes('event-row') &&
        typeof el.props.onClick === 'function') {
      clicks.push(el.props.onClick as () => void);
    }
    if (el.props?.children) walk(el.props.children);
  }
  function Probe() {
    const Component = eventsPanel.component as React.FC<{
      config: typeof eventsPanel.defaultConfig;
      setConfig: (c: typeof eventsPanel.defaultConfig) => void;
      instanceId: string;
    }>;
    const out = Component({
      config: eventsPanel.defaultConfig,
      setConfig() {},
      instanceId: 'ev'
    });
    walk(out);
    return null;
  }
  renderToStaticMarkup(<StudioProvider studio={studio}><Probe/></StudioProvider>);
  // Two rows: alert cleared (3.5 s, newest first) and alert raised (2.5 s).
  assert.equal(clicks.length, 2);
  clicks[1]();
  assert.equal(recorded.playback.cursor_ns, '2500000000');
  clicks[0]();
  assert.equal(recorded.playback.cursor_ns, '3500000000');
  studio.closeRecorded();
});
