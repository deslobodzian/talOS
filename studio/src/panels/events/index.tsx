// The events panel: a newest-first timeline of the robot events the detectors
// read off the data. Filterable by severity, category and text; pinned to the
// newest event while `follow` is on; click a row to seek playback there.
//
// The list is virtualized (fixed row height, a window of rows around the
// scroll position), so a retained history of thousands of events stays smooth.
// All browser access -- clipboard, scroll, resize -- happens in handlers and
// effects, so react-dom/server renders the list in tests.

import {useEffect, useMemo, useRef, useState} from 'react';

import {useActions, useEvents, usePlayback} from '../../data/hooks';
import type {EventCategory, RobotEvent, Severity} from '../../data/types';
import {timestamp} from '../../ui/formatters';
import {EmptyState} from '../../ui/primitives';
import type {PanelDefinition, PanelProps} from '../types';
import {CATEGORIES, SEVERITIES, filterEvents, formatRelative} from './filter';
import './events.css';

export type EventsConfig = {
  severities: Severity[];
  categories: EventCategory[];
  text: string;
  follow: boolean;
};

const defaultConfig: EventsConfig = {
  severities: [...SEVERITIES],
  categories: [...CATEGORIES],
  text: '',
  follow: true
};

function parseConfig(raw: unknown): EventsConfig {
  const obj = raw && typeof raw === 'object' && !Array.isArray(raw) ?
      raw as Record<string, unknown> :
      {};
  return {
    severities: Array.isArray(obj.severities) ?
        obj.severities.filter((s): s is Severity => SEVERITIES.includes(s as Severity)) :
        [...SEVERITIES],
    categories: Array.isArray(obj.categories) ?
        obj.categories.filter(
            (c): c is EventCategory => CATEGORIES.includes(c as EventCategory)) :
        [...CATEGORIES],
    text: typeof obj.text === 'string' ? obj.text : '',
    follow: typeof obj.follow === 'boolean' ? obj.follow : true
  };
}

// Fixed row geometry drives the virtualization: every row is exactly this tall,
// so the visible window is a slice of the sorted list without measuring DOM.
const ROW_HEIGHT = 30;
const OVERSCAN = 8;
const SSR_VIEWPORT = 480;

function EventRow({event, cursorNs, copied, onCopy, onSeek}: {
  event: RobotEvent;
  cursorNs: string|null;
  copied: boolean;
  onCopy: (channel: string) => void;
  onSeek: (t_ns: string) => void;
}) {
  const rel = formatRelative(event.t_ns, cursorNs);
  const channel = event.channel;
  return <div
      className={'event-row' + (rel.future ? ' future' : '')}
      title={timestamp(event.t_ns)}
      role="button"
      tabIndex={0}
      onClick={() => onSeek(event.t_ns)}
      onKeyDown={e => {
        if (e.key === 'Enter' || e.key === ' ') onSeek(event.t_ns);
      }}>
    <span className={'sev sev-' + event.severity} aria-label={event.severity}>
      <span className="sev-dot"/>
      {event.severity}
    </span>
    <time className="rel">{rel.label}</time>
    <span className="cat-chip">{event.category.replace('_', ' ')}</span>
    <span className="event-body">
      <span className="event-title">{event.title}</span>
      {event.detail && <span className="event-detail">{event.detail}</span>}
    </span>
    {channel && <button
        className={'plot-afford' + (copied ? ' copied' : '')}
        title={channel}
        onClick={e => {
          e.stopPropagation();
          onCopy(channel);
        }}>
      {copied ? 'copied' : 'plot'}
    </button>}
  </div>;
}

function EventsPanel({config, setConfig}: PanelProps<EventsConfig>) {
  const events = useEvents();
  const playback = usePlayback();
  const actions = useActions();

  const filtered = useMemo(
      () => filterEvents(events, config),
      [events, config.severities, config.categories, config.text]);

  const counts = useMemo(() => {
    const c: Record<Severity, number> = {info: 0, warn: 0, error: 0};
    for (const event of events) c[event.severity]++;
    return c;
  }, [events]);

  const scrollRef = useRef<HTMLDivElement>(null);
  const [start, setStart] = useState(0);
  const [viewport, setViewport] = useState(SSR_VIEWPORT);
  const [copied, setCopied] = useState<string|null>(null);

  const newestId = filtered.length ? filtered[0].id : null;

  // Keep the list pinned to the newest row while following.
  useEffect(() => {
    if (!config.follow) return;
    const el = scrollRef.current;
    if (el) el.scrollTop = 0;
    setStart(0);
  }, [config.follow, newestId]);

  // Measure the scroll viewport once mounted (fixed height per row means the
  // window size is all that is needed).
  useEffect(() => {
    const el = scrollRef.current;
    if (!el) return;
    const update = () => setViewport(el.clientHeight || SSR_VIEWPORT);
    update();
    if (typeof ResizeObserver !== 'undefined') {
      const ro = new ResizeObserver(update);
      ro.observe(el);
      return () => ro.disconnect();
    }
  }, []);

  // The "copied" hint fades back to "plot" on its own.
  useEffect(() => {
    if (copied == null) return;
    const id = setTimeout(() => setCopied(null), 1500);
    return () => clearTimeout(id);
  }, [copied]);

  const onScroll = () => {
    const el = scrollRef.current;
    if (!el) return;
    const s = Math.max(0, Math.floor(el.scrollTop / ROW_HEIGHT) - OVERSCAN);
    setStart(prev => (prev === s ? prev : s));
    // Scrolling away from the top unpins, matching "click a row turns follow
    // off": any manual move stops the list from yanking back to newest.
    if (config.follow && el.scrollTop > 4)
      setConfig({...config, follow: false});
  };

  const count = Math.ceil(viewport / ROW_HEIGHT) + OVERSCAN * 2;
  const rows = filtered.slice(start, start + count);

  const toggleSeverity = (severity: Severity) => setConfig({
    ...config,
    severities: config.severities.includes(severity) ?
        config.severities.filter(s => s !== severity) :
        [...config.severities, severity]
  });

  const toggleCategory = (category: EventCategory) => setConfig({
    ...config,
    categories: config.categories.includes(category) ?
        config.categories.filter(c => c !== category) :
        [...config.categories, category]
  });

  const copyChannel = (channel: string) => {
    if (typeof navigator !== 'undefined' && navigator.clipboard?.writeText)
      navigator.clipboard.writeText(channel).catch(() => {});
    setCopied(channel);
  };

  return <div className="events-panel">
    <div className="events-header">
      <div className="sev-counts">
        {SEVERITIES.map(severity => <span key={severity}
            className={'sev-count sev-' + severity +
                (counts[severity] === 0 ? ' zero' : '')}>
          <span className="sev-dot"/>
          {severity} {counts[severity]}
        </span>)}
      </div>
      <button className={'follow' + (config.follow ? ' selected' : '')}
              onClick={() => setConfig({...config, follow: !config.follow})}>
        Follow live: {config.follow ? 'on' : 'off'}
      </button>
    </div>

    <div className="events-filters">
      <div className="filter-group" role="group" aria-label="Severities">
        <span className="filter-label">Severity</span>
        {SEVERITIES.map(severity => <button key={severity}
            className={'chip sev-toggle sev-' + severity +
                (config.severities.includes(severity) ? ' selected' : '')}
            onClick={() => toggleSeverity(severity)}>
          {severity}
        </button>)}
      </div>
      <div className="filter-group" role="group" aria-label="Categories">
        <span className="filter-label">Category</span>
        {CATEGORIES.map(category => <button key={category}
            className={'chip' +
                (config.categories.includes(category) ? ' selected' : '')}
            onClick={() => toggleCategory(category)}>
          {category.replace('_', ' ')}
        </button>)}
      </div>
      <input className="search" aria-label="Filter events" placeholder="Filter…"
             value={config.text}
             onChange={e => setConfig({...config, text: e.target.value})}/>
    </div>

    {events.length === 0 ? <EmptyState>
      No events yet. Connect to a robot bridge, or start demo data to watch the
      event timeline.
    </EmptyState> : filtered.length === 0 ? <EmptyState>
      No events match the current filters.
    </EmptyState> : <div className="events-scroll" ref={scrollRef}
          onScroll={onScroll}>
      <div className="events-spacer" style={{height: filtered.length * ROW_HEIGHT}}>
        {rows.map((event, i) => <div key={event.id} className="event-row-slot"
            style={{top: (start + i) * ROW_HEIGHT}}>
          <EventRow event={event} cursorNs={playback.cursor_ns}
                    copied={copied === event.channel}
                    onCopy={copyChannel}
                    onSeek={t => {
                      actions.seek(t);
                      setConfig({...config, follow: false});
                    }}/>
        </div>)}
      </div>
    </div>}
  </div>;
}

export const eventsPanel: PanelDefinition<EventsConfig> = {
  id: 'events',
  title: 'Events',
  category: 'Robot',
  description: 'A seekable timeline of robot events, filterable by severity, category and text.',
  defaultConfig,
  parseConfig,
  component: EventsPanel
};
