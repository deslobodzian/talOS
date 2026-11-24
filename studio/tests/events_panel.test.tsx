// Events panel tests: the pure filtering/formatting logic and config parsing.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {EMPTY_TIMELINE, RecordedSource} from '../src/data/recorded';
import {createStaticStudio, StudioProvider} from '../src/data/studio';
import type {EventCategory, RobotEvent, Severity} from '../src/data/types';
import {
  eventsPanel,
  EventsPanelView,
  type EventsConfig,
  type EventsPanelHooks
} from '../src/panels/events';
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

type FakeScrollElement = {
  clientHeight: number;
  scrollTop: number;
};

type FakeROInstance = {
  callback: ResizeObserverCallback;
  targets: Set<Element>;
  disconnected: boolean;
};

function installFakeResizeObserver() {
  const instances: FakeROInstance[] = [];
  class FakeResizeObserver {
    private readonly record: FakeROInstance;
    constructor(callback: ResizeObserverCallback) {
      this.record = {callback, targets: new Set(), disconnected: false};
      instances.push(this.record);
    }
    observe(target: Element) {
      this.record.targets.add(target);
    }
    unobserve(target: Element) {
      this.record.targets.delete(target);
    }
    disconnect() {
      this.record.targets.clear();
      this.record.disconnected = true;
    }
  }
  const hadRO = 'ResizeObserver' in globalThis;
  const prevRO = globalThis.ResizeObserver;
  globalThis.ResizeObserver =
      FakeResizeObserver as unknown as typeof ResizeObserver;
  return {
    instances,
    trigger(target: FakeScrollElement) {
      const el = target as unknown as Element;
      for (const inst of [...instances]) {
        if (!inst.disconnected && inst.targets.has(el)) {
          inst.callback([], inst as unknown as ResizeObserver);
        }
      }
    },
    restore() {
      if (hadRO) globalThis.ResizeObserver = prevRO;
      else delete (globalThis as {ResizeObserver?: unknown}).ResizeObserver;
    }
  };
}

type RenderedRow = {
  id: string;
  top: number;
};

type ScrollElementInfo = {
  ref: React.RefObject<HTMLDivElement|null>;
  onScroll: () => void;
};

function inspectTree(node: unknown): {
  scroll: ScrollElementInfo|null;
  rows: RenderedRow[];
} {
  let scroll: ScrollElementInfo|null = null;
  const rows: RenderedRow[] = [];
  function walk(n: unknown) {
    if (!n || typeof n !== 'object') return;
    if (Array.isArray(n)) {
      for (const item of n) walk(item);
      return;
    }
    const el = n as React.ReactElement<Record<string, unknown>>;
    const cls = typeof el.props?.className === 'string' ? el.props.className : '';
    if (cls === 'events-scroll') {
      const ref = (el.props.ref ?? (el as unknown as {ref?: unknown}).ref) as
          React.RefObject<HTMLDivElement|null>;
      scroll = {ref, onScroll: el.props.onScroll as () => void};
    } else if (cls === 'event-row-slot') {
      const style = el.props.style as {top: number};
      rows.push({id: String(el.key), top: style.top});
    }
    if (el.props?.children) walk(el.props.children);
  }
  walk(node);
  return {scroll, rows};
}

function depsChanged(prev: readonly unknown[]|undefined,
                     next: readonly unknown[]|undefined): boolean {
  if (!prev || !next) return true;
  if (prev.length !== next.length) return true;
  for (let i = 0; i < prev.length; i++) {
    if (!Object.is(prev[i], next[i])) return true;
  }
  return false;
}

function mountEventsPanel(
    initial: {events: RobotEvent[]; config?: EventsConfig; clientHeight?: number},
    ro: ReturnType<typeof installFakeResizeObserver>) {
  const studio = createStaticStudio();
  let currentEvents = initial.events;
  let currentConfig: EventsConfig = initial.config ?? eventsPanel.defaultConfig;
  let containerHeight = initial.clientHeight ?? 480;

  let slot = 0;
  const states: unknown[] = [];
  const refs: {current: unknown}[] = [];
  const memos: ({value: unknown; deps: readonly unknown[]}|undefined)[] = [];
  const effects: ({
    deps: readonly unknown[]|undefined;
    cleanup: (() => void)|void;
  }|undefined)[] = [];
  let pendingEffects: {
    index: number;
    create: () => ((() => void)|void);
    deps: readonly unknown[]|undefined;
  }[] = [];
  let dirty = false;

  const hooks = {
    useState<S>(init?: S|(() => S)) {
      const idx = slot++;
      if (!(idx in states)) {
        states[idx] = typeof init === 'function' ? (init as () => S)() : init;
      }
      const setState = (next: S|((prev: S) => S)) => {
        const prev = states[idx] as S;
        const resolved = typeof next === 'function' ?
            (next as (p: S) => S)(prev) : next;
        if (!Object.is(prev, resolved)) {
          states[idx] = resolved;
          dirty = true;
        }
      };
      return [states[idx] as S, setState];
    },
    useRef<T>(init?: T) {
      const idx = slot++;
      if (!(idx in refs)) refs[idx] = {current: init};
      return refs[idx];
    },
    useMemo<T>(factory: () => T, deps: readonly unknown[]) {
      const idx = slot++;
      const entry = memos[idx];
      if (!entry || depsChanged(entry.deps, deps)) {
        const value = factory();
        memos[idx] = {value, deps};
        return value;
      }
      return entry.value as T;
    },
    useEffect(create: () => ((() => void)|void), deps?: readonly unknown[]) {
      const idx = slot++;
      pendingEffects.push({index: idx, create, deps});
    }
  } as unknown as EventsPanelHooks;

  let mountedScrollEl: FakeScrollElement|null = null;
  let lastScrollRef: React.RefObject<HTMLDivElement|null>|null = null;
  let lastOnScroll: (() => void)|null = null;
  let lastRows: RenderedRow[] = [];

  const setConfig = (next: EventsConfig) => {
    currentConfig = next;
    dirty = true;
  };

  const flush = () => {
    for (let pass = 0; pass < 20; pass++) {
      slot = 0;
      pendingEffects = [];
      dirty = false;
      const tree = EventsPanelView({
        config: currentConfig,
        setConfig,
        instanceId: 'ev',
        events: currentEvents,
        playback: studio.playback,
        actions: studio.actions,
        recorded: false,
        hooks
      });
      const info = inspectTree(tree);
      lastRows = info.rows;
      if (info.scroll) {
        if (!mountedScrollEl) {
          mountedScrollEl = {clientHeight: containerHeight, scrollTop: 0};
        }
        lastScrollRef = info.scroll.ref;
        lastOnScroll = info.scroll.onScroll;
        (info.scroll.ref as {current: unknown}).current = mountedScrollEl;
      } else {
        if (lastScrollRef) (lastScrollRef as {current: unknown}).current = null;
        mountedScrollEl = null;
        lastOnScroll = null;
      }
      const toRun = pendingEffects;
      pendingEffects = [];
      for (const eff of toRun) {
        const prev = effects[eff.index];
        if (!prev || depsChanged(prev.deps, eff.deps)) {
          if (prev?.cleanup) prev.cleanup();
          const cleanup = eff.create();
          effects[eff.index] = {deps: eff.deps, cleanup};
        }
      }
      if (!dirty) return;
    }
    assert.fail('EventsPanelView render loop did not settle');
  };

  flush();

  return {
    render(next: {events?: RobotEvent[]; config?: EventsConfig}) {
      if (next.events !== undefined) currentEvents = next.events;
      if (next.config !== undefined) currentConfig = next.config;
      flush();
    },
    scrollTo(top: number) {
      const el = mountedScrollEl;
      const onScroll = lastOnScroll;
      if (!el || !onScroll) assert.fail('events-scroll is not mounted');
      el.scrollTop = top;
      onScroll();
      if (dirty) flush();
    },
    resize(height: number) {
      containerHeight = height;
      const el = mountedScrollEl;
      if (el) {
        el.clientHeight = height;
        ro.trigger(el);
        if (dirty) flush();
      }
    },
    unmount() {
      if (lastScrollRef) (lastScrollRef as {current: unknown}).current = null;
      mountedScrollEl = null;
      lastOnScroll = null;
      for (const eff of effects) {
        if (eff?.cleanup) eff.cleanup();
      }
    },
    hasScrollList: () => mountedScrollEl !== null,
    scrollTop: () => mountedScrollEl?.scrollTop ?? null,
    rows: () => lastRows
  };
}

function makeEvents(n: number): RobotEvent[] {
  return Array.from({length: n}, (_, i) => ev(
      `e:${i}`,
      String(1_700_000_000_000_000_000n + BigInt(i) * 1_000_000n),
      'info',
      'system',
      `Event ${i}`));
}

test('EventsPanel attaches ResizeObserver and measures viewport on 0 -> N events', () => {
  const ro = installFakeResizeObserver();
  try {
    const panel = mountEventsPanel({events: [], clientHeight: 900}, ro);
    assert.equal(panel.hasScrollList(), false);
    assert.equal(panel.rows().length, 0);
    assert.equal(ro.instances.length, 0);

    // Transitioning 0 -> 100 events mounts .events-scroll, attaches
    // ResizeObserver, and measures the 900 px viewport (ceil(900/30) + 16 = 46
    // rows instead of the 480 px SSR default of 32).
    panel.render({events: makeEvents(100)});
    assert.equal(panel.hasScrollList(), true);
    assert.equal(ro.instances.length, 1);
    assert.equal(ro.instances[0].disconnected, false);
    assert.equal(ro.instances[0].targets.size, 1);
    assert.equal(panel.rows().length, 46);
    assert.deepEqual(panel.rows()[0], {id: 'e:99', top: 0});

    // Subsequent viewport resizes update the virtual window (ceil(1200/30) + 16 = 56).
    panel.resize(1200);
    assert.equal(panel.rows().length, 56);

    panel.unmount();
    assert.equal(ro.instances[0].disconnected, true);
  } finally {
    ro.restore();
  }
});

test('EventsPanel resets start after filter-clear and clamps effectiveStart when list shrinks', () => {
  const ro = installFakeResizeObserver();
  try {
    const allEvents = makeEvents(100);
    const baseConfig: EventsConfig = {...eventsPanel.defaultConfig, follow: false};
    const panel = mountEventsPanel(
        {events: allEvents, config: baseConfig, clientHeight: 300}, ro);
    // ceil(300/30) + 16 = 26 rows from index 0.
    assert.equal(panel.rows().length, 26);
    assert.deepEqual(panel.rows()[0], {id: 'e:99', top: 0});

    // Scroll down with follow: false -> start = floor(1500/30) - 8 = 42.
    panel.scrollTo(1500);
    assert.equal(panel.rows().length, 26);
    assert.deepEqual(panel.rows()[0], {id: 'e:57', top: 1260});

    // Filter to 0 matches: .events-scroll unmounts and observer disconnects.
    panel.render({config: {...baseConfig, text: 'no-match-xyz'}});
    assert.equal(panel.hasScrollList(), false);
    assert.equal(panel.rows().length, 0);
    assert.equal(ro.instances[0].disconnected, true);

    // Clear the filter: .events-scroll remounts at scrollTop = 0 without
    // firing onScroll; start resets to 0 and rows render from index 0.
    panel.render({config: baseConfig});
    assert.equal(panel.hasScrollList(), true);
    assert.equal(panel.scrollTop(), 0);
    assert.equal(panel.rows().length, 26);
    assert.deepEqual(panel.rows()[0], {id: 'e:99', top: 0});
    assert.deepEqual(panel.rows()[1], {id: 'e:98', top: 30});
    assert.equal(ro.instances.length, 2);
    assert.equal(ro.instances[1].disconnected, false);

    // Changing filter criteria while the list stays mounted also resets start
    // and scrollTop to 0.
    panel.scrollTo(1500);
    assert.deepEqual(panel.rows()[0], {id: 'e:57', top: 1260});
    panel.render({config: {...baseConfig, text: 'Event 9'}});
    assert.equal(panel.hasScrollList(), true);
    assert.equal(panel.scrollTop(), 0);
    assert.equal(panel.rows().length, 11);
    assert.deepEqual(panel.rows()[0], {id: 'e:99', top: 0});

    // If the event list shrinks directly below `start` without unmounting or
    // changing filters, effectiveStart clamps to filtered.length - 1 so rows
    // never vanish.
    panel.render({events: allEvents, config: baseConfig});
    panel.scrollTo(1500);
    assert.deepEqual(panel.rows()[0], {id: 'e:57', top: 1260});
    panel.render({events: makeEvents(5)});
    assert.equal(panel.rows().length, 1);
    assert.deepEqual(panel.rows()[0], {id: 'e:0', top: 120});

    panel.unmount();
  } finally {
    ro.restore();
  }
});
