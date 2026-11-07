// Events panel tests: pure filtering/formatting logic and the panel's
// server-rendered list (empty and populated). No DOM assertions: the panel
// renders under react-dom/server, and the pure helpers have no browser
// dependency.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import type {Studio} from '../src/data/studio';
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

function renderPanel(studio: Studio): string {
  return renderToStaticMarkup(
      <StudioProvider studio={studio}>
        <eventsPanel.component config={eventsPanel.defaultConfig}
                               setConfig={() => {}} instanceId="test"/>
      </StudioProvider>);
}

test('the events panel shows an empty state when there are no events', () => {
  const html = renderPanel(createStaticStudio({}));
  assert.ok(html.includes('No events yet'));
  assert.ok(html.includes('Follow live'));
});

test('the events panel lists events newest first with counts and offsets',
     () => {
       const events: RobotEvent[] = [
         ev('a:1:0', PAST, 'info', 'power', 'Power recovered'),
         ev('a:2:0', FUTURE, 'error', 'vision', 'Tag lost', 'rejected',
            'front_left_camera/state/tag_count')
       ];
       const studio =
           createStaticStudio({events, playback: {cursor_ns: CURSOR}});
       const html = renderPanel(studio);

       assert.ok(html.includes('Power recovered'));
       assert.ok(html.includes('Tag lost'));
       // Per-severity counts in the header.
       assert.ok(html.includes('info 1'));
       assert.ok(html.includes('error 1'));
       // Relative offsets and the future-row dimming.
       assert.ok(html.includes('-3.2 s'));
       assert.ok(html.includes('+1.0 s'));
       assert.ok(html.includes('class="event-row future"'));
       // Category chip and the plot affordance (with the channel as its hint).
       assert.ok(html.includes('power'));
       assert.ok(html.includes('vision'));
       assert.ok(html.includes('>plot<'));
       assert.ok(html.includes('front_left_camera/state/tag_count'));
       // Absolute timestamp on hover (thousands-separated ns).
       assert.ok(html.includes('1,699,999,996,800,000,000 ns'));
     });

test('the events panel reports when filters exclude everything', () => {
  const events: RobotEvent[] = [ev('a:1:0', PAST, 'info', 'power', 'Recovered')];
  const studio = createStaticStudio({events, playback: {cursor_ns: CURSOR}});
  const html = renderToStaticMarkup(
      <StudioProvider studio={studio}>
        <eventsPanel.component
            config={{...eventsPanel.defaultConfig, text: 'no such text'}}
            setConfig={() => {}} instanceId="test"/>
      </StudioProvider>);
  assert.ok(html.includes('No events match'));
});
