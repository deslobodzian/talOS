// The driver dashboard as HTML text: a pure function of the layout, the feed
// model and the time, so a test renders the 2026 layout with no DOM and the
// page (main.ts) only swaps the text in. Every state that matters is told by
// a word as well as a colour (STALE, ERROR, LOW, DOWN, ...), and every
// string from the feed is escaped.

import {channelDisplay, type Snapshot} from '../core';
import {alertLevelName, type ActiveAlert, type LatchedAlert} from '../data/alerts';
import {FIELD_HEIGHT_M, FIELD_WIDTH_M} from '../panels/field/geometry';
import {type DriveModel, type DriveSummary, type Link, STALE_MS, type SummaryNode} from './feed';
import type {Cell, CheckItem, Layout, Widget} from './layout';

export function esc(text: string): string {
  return text.replace(/[&<>"']/g, c => `&#${c.charCodeAt(0)};`);
}

const ABSENT = '<span class="absent">no data</span>';

// A channel's reading in the newest tick: absent (never received, or its
// topic not published) is never shown as a number.
// `text` is the enum label when the channel has one for the value, else the
// exact value; `exact` whether an integer is past what a double holds.
type Reading = {value: number; text: string; unit: string; labelled: boolean;
                exact: boolean}|null;
function read(s: Snapshot|null, name: string|undefined): Reading {
  if (!s || !name) return null;
  const value = s.channels[name];
  if (value === undefined) return null;
  const labels = s.labels[name];
  const labelled = !!labels && Number.isInteger(value) && value >= 0 &&
      value < labels.length;
  return {value, text: channelDisplay(s, name), unit: s.units[name] ?? '',
          labelled, exact: !Number.isSafeInteger(value) && s.exact?.[name] !== undefined};
}
const on = (r: Reading) => r !== null && r.value !== 0 && !Number.isNaN(r.value);

// A number at `precision` places with its unit; a label or a non-finite or
// huge integer reading as its own text.
function fixed(r: Reading, precision: number): string {
  if (!r) return ABSENT;
  const text = !r.labelled && !r.exact && Number.isFinite(r.value) ?
      r.value.toFixed(precision) : r.text;
  return esc(text) + (r.unit ? ` <span class="unit">${esc(r.unit)}</span>` : '');
}

const leaf = (name: string) => name.slice(name.lastIndexOf('/') + 1).replace(/_/g, ' ');

const HEADING: Record<string, string> = {
  match: 'Match', power: 'Battery', routine: 'Auto routine', alerts: 'Alerts',
  subsystems: 'Subsystems', field: 'Field', checks: 'Checks', graph: 'Graph'
};

function heading(w: Widget): string {
  if (w.title) return w.title;
  if (w.kind === 'number' || w.kind === 'boolean' || w.kind === 'text')
    return w.label || leaf(w.channels.channel ?? '');
  return HEADING[w.kind] ?? w.kind;
}

// The widgets whose body is a list that scrolls when it cannot fit: their
// heading carries an empty `.out-of-view` the page fills after each paint
// (main.ts, outOfView below) with how many rows are out of view.
const LISTS = new Set(['subsystems', 'checks', 'alerts']);

function shell(w: Widget, stale: boolean, body: string, extra = ''): string {
  const {column, row, width, height} = w.at;
  return `<section class="w w-${esc(w.kind)}${stale ? ' stale' : ''}" ` +
      `style="grid-column:${column + 1} / span ${width};grid-row:${row + 1} / span ${height}">` +
      `<header><h2>${esc(heading(w))}</h2>${extra}` +
      (LISTS.has(w.kind) ? '<span class="out-of-view"></span>' : '') +
      (stale ? '<span class="tag tag-stale">STALE</span>' : '') +
      `</header><div class="body">${body}</div></section>`;
}

const tag = (word: string, kind: string) =>
    `<span class="tag tag-${kind}">${esc(word)}</span>`;

function matchWidget(s: Snapshot|null, c: Readonly<Record<string, string>>): string {
  const r = (role: string) => read(s, c[role]);
  const enabled = r('enabled');
  let mode: string, kind = 'neutral';
  if (enabled === null) {
    mode = 'NO DATA';
  } else if (on(r('estop'))) {
    mode = 'E-STOP';
    kind = 'bad';
  } else if (r('connected') !== null && !on(r('connected'))) {
    mode = 'NO DRIVER STATION';
    kind = 'bad';
  } else if (!on(enabled)) {
    mode = 'DISABLED';
  } else {
    mode = on(r('autonomous')) ? 'AUTO' : on(r('test')) ? 'TEST' :
        on(r('teleop')) ? 'TELEOP' : 'ENABLED';
    kind = 'good';
  }
  const time = r('match_time_s');
  const clock = time === null || !Number.isFinite(time.value) || time.value < 0 ?
      '—' :
      `${Math.floor(time.value / 60)}:${String(Math.floor(time.value % 60)).padStart(2, '0')}`;
  const alliance = r('alliance'), station = r('station');
  const side = alliance === null ? '' :
      esc(alliance.text.toUpperCase()) + (on(station) ? ' ' + esc(station!.text) : '');
  const number = r('match_number'), type = r('match_type');
  const match = on(number) ?
      `${type ? esc(type.text) + ' ' : ''}${esc(number!.text)}` : 'no match';
  return `<div class="big mode mode-${kind}">${esc(mode)}</div>` +
      `<div class="row"><span class="big clock">${clock}</span>` +
      `<span class="alliance alliance-${esc(alliance?.text.toLowerCase() ?? 'none')}">${side}</span></div>` +
      `<div class="row small">${match}${on(r('fms_attached')) ? ' · FMS' : ''}</div>`;
}

function powerWidget(s: Snapshot|null, w: Widget&{kind: 'power'}): string {
  const volts = read(s, w.channels.battery_v);
  const valid = read(s, w.channels.health_valid);
  if (volts === null || (valid !== null && !on(valid)))
    return `<div class="big">${volts === null ? ABSENT : '<span class="absent">no reading</span>'}</div>`;
  const low = volts.value < w.warn_below_v;
  const brownout = on(read(s, w.channels.brownout));
  return `<div class="big${low ? ' bad' : ''}">${fixed(volts, 2)}</div>` +
      `<div class="row">${low ? tag('LOW', 'bad') : ''}${brownout ? tag('BROWNOUT', 'bad') : ''}</div>`;
}

// The routine's name from the dictionary's labels for its index (REQ-OPS-38),
// its number when the index is out of range.
function routineWidget(s: Snapshot|null, c: Readonly<Record<string, string>>): string {
  const index = read(s, c.channel);
  const name = index === null ? ABSENT :
      index.labelled          ? esc(index.text) :
                                `routine #${esc(index.text)}`;
  const phase = c.phase ? read(s, c.phase) : null;
  return `<div class="big routine">${name}</div>` +
      (c.phase ? `<div class="row small">phase ${phase ? esc(phase.text) : ABSENT}</div>` : '');
}

function alertItem(a: ActiveAlert, meta: string): string {
  const level = alertLevelName(a.level);
  return `<li class="alert alert-${level}"><span class="level">${level.toUpperCase()}</span> ` +
      `<span class="name" title="${esc(a.channel)}">${a.owner ? `<b>${esc(a.owner)}</b> ` : ''}` +
      `${esc(a.text)}</span>${meta ? ` <span class="meta">${meta}</span>` : ''}</li>`;
}

function alertsWidget(model: DriveModel): string {
  const active = model.alerts.active();
  const latch = model.alerts.latch();
  const span = latch.matchStart_ns === null ? 'since connect' :
      latch.matchKey === null ? 'this match' :
                                `match ${latch.matchKey.slice(latch.matchKey.indexOf(':') + 1)}`;
  const latched = (a: LatchedAlert) =>
      alertItem(a, `×${a.count}${a.active ? ' · active' : ''}`);
  return `<h3>Active</h3>` +
      (active.length ? `<ul class="alerts">${active.map(a => alertItem(a, '')).join('')}</ul>` :
                       '<p class="none">none</p>') +
      `<h3>Raised ${esc(span)}</h3>` +
      (latch.alerts.length ? `<ul class="alerts latched">${latch.alerts.map(latched).join('')}</ul>` :
                             '<p class="none">none</p>');
}

// The rows that matter come first, so a list that overflows its cell (the
// body then scrolls) hides the least: a node down or quarantined (critical
// ones first), a named node the summary lacks, one driving from a target
// lane, a critical one, then the rest; summary order within each.
function nodeRank(n: SummaryNode|string, summary: DriveSummary): number {
  if (typeof n === 'string') return 2;
  const failed = (summary.registry && !n.alive) || (summary.supervised && n.quarantined);
  if (failed) return n.critical ? 0 : 1;
  if (summary.registry && n.drives !== null) return 3;
  return n.critical ? 4 : 5;
}

// A subsystems row's state words (the row width model sizes for the widest).
export const NODE_STATE = {failing: 'FAILING', down: 'DOWN', up: 'UP', unknown: 'unknown'} as const;

function subsystemsWidget(model: DriveModel, w: Widget&{kind: 'subsystems'}): string {
  const summary = model.summary;
  if (!summary) return `<p class="none">${ABSENT}</p>`;
  // A filter shows every node it names: one the summary does not list (cut
  // off by `omitted`, or not in the session) is unknown.
  const named: (SummaryNode|string)[] = w.nodes.length ?
      w.nodes.map(name => summary.nodes.find(n => n.name === name) ?? name) :
      summary.nodes;
  const nodes = named.map((n, i) => ({n, i, rank: nodeRank(n, summary)}))
                    .sort((a, b) => a.rank - b.rank || a.i - b.i)
                    .map(e => e.n);
  const rows = nodes.map(n => {
    if (typeof n === 'string')
      return `<li><span class="name" title="${esc(n)}">${esc(n)}</span>${tag(NODE_STATE.unknown, 'neutral')}` +
          '<span class="meta">not in the summary</span></li>';
    // One state word, the most severe: a quarantined node is not running
    // (the launcher paused its restarts), so FAILING says all DOWN would.
    const state = summary.supervised && n.quarantined ? tag(NODE_STATE.failing, 'bad') :
        !summary.registry ? tag(NODE_STATE.unknown, 'neutral') :
        n.alive           ? tag(NODE_STATE.up, 'good') :
                            tag(NODE_STATE.down, n.critical ? 'bad' : 'warn');
    const restarts = summary.supervised && n.restarts > 0 ? `restarts ${n.restarts}` : '';
    const drives = summary.registry && n.drives !== null ? `drives ${n.drives}` : '';
    // The name's tooltip repeats what a narrow row may cut off.
    // One note for both, so a narrow row loses one gap to it, not two.
    const note = [restarts, drives].filter(Boolean).join(' · ');
    const title = [n.name, note].filter(Boolean).join(' · ');
    return `<li><span class="name" title="${esc(title)}">${esc(n.name)}</span>${state}` +
        (note ? `<span class="meta">${esc(note)}</span>` : '') + '</li>';
  });
  if (summary.omitted > 0 && !w.nodes.length)
    rows.push(`<li class="more">+${summary.omitted} more</li>`);
  return `<ul class="nodes">${rows.join('')}</ul>`;
}

function fieldWidget(model: DriveModel, w: Widget&{kind: 'field'}): string {
  const s = model.latest;
  const x = read(s, w.channels.x), y = read(s, w.channels.y), yaw = read(s, w.channels.yaw);
  const valid = w.channels.valid ? read(s, w.channels.valid) : null;
  const localized = w.channels.localized ? read(s, w.channels.localized) : null;
  const notes: string[] = [];
  if (w.channels.valid && !on(valid)) notes.push(tag('POSE NOT VALID', 'bad'));
  if (w.channels.localized && !on(localized)) notes.push(tag('NOT LOCALIZED', 'warn'));
  let robot = '', trail = '';
  const have = x && y && yaw && [x.value, y.value, yaw.value].every(Number.isFinite);
  if (have) {
    const deg = -yaw!.value * 180 / Math.PI;
    robot = `<g transform="translate(${x!.value.toFixed(3)} ${(FIELD_HEIGHT_M - y!.value).toFixed(3)}) rotate(${deg.toFixed(1)})">` +
        '<rect class="robot" x="-0.45" y="-0.45" width="0.9" height="0.9"/>' +
        '<path class="heading" d="M0 0 L0.6 0"/></g>';
    const xs = model.history(w.channels.x), ys = model.history(w.channels.y);
    const newest = xs.length ? xs[xs.length - 1].t_s : 0;
    const points: string[] = [];
    for (let i = 0; i < xs.length && i < ys.length; i++) {
      if (newest - xs[i].t_s > w.trail_s) continue;
      if (!Number.isFinite(xs[i].v) || !Number.isFinite(ys[i].v)) continue;
      points.push(`${xs[i].v.toFixed(2)},${(FIELD_HEIGHT_M - ys[i].v).toFixed(2)}`);
    }
    if (points.length > 1) trail = `<polyline class="trail" points="${points.join(' ')}"/>`;
  } else {
    notes.push(tag('NO POSE', 'neutral'));
  }
  return `<svg class="field" viewBox="0 0 ${FIELD_WIDTH_M} ${FIELD_HEIGHT_M}" preserveAspectRatio="xMidYMid meet" role="img" aria-label="robot on the field">` +
      `<rect class="carpet" x="0" y="0" width="${FIELD_WIDTH_M}" height="${FIELD_HEIGHT_M}"/>` +
      `<line class="mid" x1="${FIELD_WIDTH_M / 2}" y1="0" x2="${FIELD_WIDTH_M / 2}" y2="${FIELD_HEIGHT_M}"/>` +
      `${trail}${robot}</svg>` +
      (notes.length ? `<div class="row">${notes.join('')}</div>` : '');
}

function numberWidget(s: Snapshot|null, w: Widget&{kind: 'number'}): string {
  const r = read(s, w.channels.channel);
  const low = r !== null && w.warn_below !== null && r.value < w.warn_below;
  const high = r !== null && w.warn_above !== null && r.value > w.warn_above;
  return `<div class="big${low || high ? ' bad' : ''}">${fixed(r, w.precision)}</div>` +
      (low ? tag('LOW', 'bad') : '') + (high ? tag('HIGH', 'bad') : '');
}

function booleanWidget(s: Snapshot|null, w: Widget&{kind: 'boolean'}): string {
  const r = read(s, w.channels.channel);
  if (r === null) return `<div class="big">${ABSENT}</div>`;
  const value = on(r);
  return `<div class="big flag ${value === w.good ? 'good' : 'bad'}">${value ? 'YES' : 'NO'}</div>`;
}

function textWidget(s: Snapshot|null, w: Widget&{kind: 'text'}): string {
  const r = read(s, w.channels.channel);
  return `<div class="big">${r === null ? ABSENT : esc(r.text)}</div>`;
}

const DASHES = ['', '6 3', '2 3', '8 3 2 3'];

function graphWidget(model: DriveModel, w: Widget&{kind: 'graph'}): string {
  const s = model.latest;
  const traces = w.series.map(name => model.history(name));
  const newest = Math.max(-Infinity, ...traces.map(t => t.length ? t[t.length - 1].t_s : -Infinity));
  let lo = w.min ?? Infinity, hi = w.max ?? -Infinity;
  if (w.min === null || w.max === null) {
    for (const t of traces)
      for (const p of t)
        if (Number.isFinite(p.v) && newest - p.t_s <= w.window_s) {
          if (w.min === null) lo = Math.min(lo, p.v);
          if (w.max === null) hi = Math.max(hi, p.v);
        }
  }
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) lo = 0, hi = 1;
  if (hi <= lo) hi = lo + 1;
  const H = 40, W = 100;
  const lines = traces.map((t, i) => {
    const segments: string[][] = [[]];
    for (const p of t) {
      if (newest - p.t_s > w.window_s) continue;
      if (!Number.isFinite(p.v)) {
        segments.push([]);
        continue;
      }
      const px = W - (newest - p.t_s) / w.window_s * W;
      const py = H - (Math.min(hi, Math.max(lo, p.v)) - lo) / (hi - lo) * H;
      segments[segments.length - 1].push(`${px.toFixed(2)},${py.toFixed(2)}`);
    }
    return segments.filter(seg => seg.length > 1).map(seg =>
        `<polyline class="series series-${i}" stroke-dasharray="${DASHES[i % DASHES.length]}" points="${seg.join(' ')}"/>`).join('');
  });
  const legend = w.series.map((name, i) => {
    const r = read(s, name);
    return `<li class="series-${i}"><svg class="key" viewBox="0 0 20 4"><line x1="0" y1="2" x2="20" y2="2" stroke-dasharray="${DASHES[i % DASHES.length]}"/></svg>` +
        `${esc(leaf(name))} <b>${fixed(r, 1)}</b></li>`;
  });
  return `<svg class="graph" viewBox="0 0 ${W} ${H}" preserveAspectRatio="none" role="img" aria-label="${esc(heading(w))} over ${w.window_s} s">${lines.join('')}</svg>` +
      `<div class="axis small"><span>${esc(String(+hi.toPrecision(4)))}</span><span>${esc(String(+lo.toPrecision(4)))}</span><span>last ${w.window_s} s</span></div>` +
      `<ul class="legend">${legend.join('')}</ul>`;
}

// A check passes while its channel meets every condition it sets: `equals` a
// bool (a Bool channel), a number, or a string (an enum channel's label).
export function checkPasses(s: Snapshot|null, item: CheckItem): boolean|null {
  const r = read(s, item.channel);
  if (r === null) return null;
  if (item.equals !== null) {
    if (typeof item.equals === 'boolean' && on(r) !== item.equals) return false;
    if (typeof item.equals === 'number' && r.value !== item.equals) return false;
    if (typeof item.equals === 'string' && r.text !== item.equals) return false;
  }
  if (item.at_least !== null && !(r.value >= item.at_least)) return false;
  if (item.at_most !== null && !(r.value <= item.at_most)) return false;
  return true;
}

function checksWidget(s: Snapshot|null, w: Widget&{kind: 'checks'}): string {
  const rows = w.items.map(item => {
    const pass = checkPasses(s, item);
    const verdict = pass === null ? tag('NO DATA', 'neutral') :
        pass ? tag('OK', 'good') : tag('FAIL', 'bad');
    return `<li>${verdict}<span class="name" title="${esc(item.label)}">${esc(item.label)}</span></li>`;
  });
  return `<ul class="checks">${rows.join('')}</ul>`;
}

// One widget's markup: a <section> in its cell of the grid.
export function renderWidget(w: Widget, model: DriveModel, now: number): string {
  const s = model.latest;
  const stale = !model.fresh(now);
  switch (w.kind) {
    case 'match': return shell(w, stale, matchWidget(s, w.channels));
    case 'power': return shell(w, stale, powerWidget(s, w));
    case 'routine': return shell(w, stale, routineWidget(s, w.channels));
    case 'alerts': return shell(w, stale, alertsWidget(model));
    case 'subsystems': {
      const summary = model.summary;
      const extra = summary && summary.session ?
          `<span class="meta">session ${esc(summary.session)}</span>` : '';
      return shell(w, !model.summaryFresh(now), subsystemsWidget(model, w), extra);
    }
    case 'field': return shell(w, stale, fieldWidget(model, w));
    case 'number': return shell(w, stale, numberWidget(s, w));
    case 'boolean': return shell(w, stale, booleanWidget(s, w));
    case 'text': return shell(w, stale, textWidget(s, w));
    case 'graph': return shell(w, stale, graphWidget(model, w));
    case 'checks': return shell(w, stale, checksWidget(s, w));
    case 'unsupported':
      return shell({...w, title: w.title || 'Unsupported widget'}, false,
                   `<p class="unsupported">Unsupported widget "${esc(w.name)}": ${esc(w.reason)}</p>`);
  }
}

const LINK_WORD: Record<Link, string> = {
  ok: 'OK', connecting: 'CONNECTING', bridge: 'BRIDGE LINK',
  telemetry: 'TELEMETRY', hardware: 'HARDWARE BRIDGE', roborio: 'ROBORIO LINK',
  replaced: 'REPLACED'
};

const plural = (n: number, word: string) => `${n} ${word}${n === 1 ? '' : 's'}`;

// `skipped`: the layout's widgets with no cell this page can read.
export function renderStatus(model: DriveModel, now: number, skipped = 0): string {
  const status = model.status(now);
  const c = model.counters;
  const malformed = c.malformed_binary + c.malformed_text;
  return `<div class="status link-${status.link}" role="status">` +
      `<b>${LINK_WORD[status.link]}</b> ${esc(status.text)}` +
      (status.link === 'replaced' ?
           ' <button type="button" data-action="reconnect">Reconnect</button>' : '') +
      (malformed ? ` <span class="meta">${plural(malformed, 'malformed message')} counted</span>` : '') +
      (skipped > 0 ?
           ` <span class="meta bad">${plural(skipped, 'layout widget')} not drawn: no cell this page can read</span>` :
           '') +
      '</div>';
}

// The widgets, for the grid element (gridStyle sizes it).
export function renderWidgets(layout: Layout, model: DriveModel, now: number): string {
  return layout.widgets.map(w => renderWidget(w, model, now)).join('');
}

export function gridStyle(layout: Layout): string {
  return `grid-template-columns:repeat(${layout.columns},minmax(0,1fr));` +
      `grid-template-rows:repeat(${layout.rows},minmax(0,1fr))`;
}

// What the page draws into: main.ts implements it on the DOM, a test with
// plain objects. `E` is one widget's element.
export interface Surface<E> {
  status(html: string): void;
  create(html: string): E;
  // Puts `next` at `index` of the grid, in place of `previous` if any.
  put(index: number, next: E, previous: E|undefined): void;
  // Sizes the element's list body, if any, to the whole rows its cell
  // leaves it (on the grid, so laid out).
  fit(element: E): void;
  // The scroll offset of the element's list body (0 without one, and 0 off
  // the grid, where an element has no layout box), and setting it, clamped
  // to what the body can scroll as it is now sized.
  scrollTop(element: E): number;
  setScrollTop(element: E, top: number): void;
  errored(on: boolean): void;
}

// One frame: the status line, and each widget's markup in layout order.
export type Frame = {status: string; widgets: string[]};

// Paints frames widget by widget. The page redraws every RENDER_MS, and with
// a live producer some widget (a graph, the battery, the match clock)
// changes nearly every frame: a widget whose markup did not change keeps
// its element, so a list's scroll position and a row's tooltip survive. A
// widget whose markup changed is replaced, and the new element takes the old
// one's list scroll position: read before the old element leaves the grid
// (off it, it reads 0) and set once the new body is sized to whole rows (set
// before, it is clamped to a taller body's range, so a list scrolled to its
// bottom would lose part of a row). A drawing error leaves the grid as it
// is, marks it (it shows values no longer updated) and says so on the
// status line.
export class Painter<E> {
  private status = '';
  private markup: string[] = [];
  private elements: E[] = [];

  constructor(private readonly surface: Surface<E>) {}

  paint(draw: () => Frame) {
    let next: Frame;
    try {
      next = draw();
    } catch (error) {
      // Forget the status shown, so the first good draw repaints it.
      this.status = '';
      this.surface.status('<div class="status link-error" role="status"><b>DRAWING ERROR</b> ' +
                          `The dashboard is not updating: ${esc(String(error))}</div>`);
      this.surface.errored(true);
      return;
    }
    if (next.status !== this.status) this.surface.status(this.status = next.status);
    next.widgets.forEach((html, i) => {
      if (html === this.markup[i]) return;
      this.markup[i] = html;
      const previous = this.elements[i];
      const top = previous === undefined ? 0 : this.surface.scrollTop(previous);
      const element = this.surface.create(html);
      this.surface.put(i, element, previous);
      this.elements[i] = element;
      this.surface.fit(element);
      if (top > 0) this.surface.setScrollTop(element, top);
    });
    this.surface.errored(false);
  }
}

// The page's clocks and timers; a test passes fakes.
export interface Timers {
  now(): number;   // monotonic ms (performance.now)
  wall(): number;  // wall-clock ms (Date.now)
  setTimeout(fn: () => void, ms: number): number;
  clearTimeout(handle: number): void;
}

// How late after a freshness deadline the deadline's draw is asked for:
// `fresh` holds through the deadline itself, so the draw comes 1 ms after.
export const STALE_DRAW_LAG_MS = 1;

// When the page draws. Besides the periodic render (RENDER_MS), the page's
// return to view and a user action, each draw arms a timer for the model's
// next freshness deadline (a value going stale STALE_MS after its last new
// tick, or the link going silent), so stale is drawn STALE_DRAW_LAG_MS after
// the deadline (plus the browser's timer lateness), not up to RENDER_MS
// later. A periodic draw follows every new tick within RENDER_MS, before its
// deadline, and re-arms the timer for it.
//
// performance.now() may not advance while the laptop sleeps, so a value
// last updated before the sleep would read current after it: when the wall
// clock moved more than STALE_MS further than the monotonic one between two
// draws, the model reads stale until a new tick (DriveModel.suspend).
export class DrawSchedule {
  private armed: {at: number; handle: number}|null = null;
  private last: {now: number; wall: number}|null = null;

  constructor(private readonly model: DriveModel, private readonly draw: (now: number) => void,
              private readonly timers: Timers) {}

  frame() {
    const now = this.timers.now(), wall = this.timers.wall();
    if (this.last && (wall - this.last.wall) - (now - this.last.now) > STALE_MS) this.model.suspend();
    this.last = {now, wall};
    this.draw(now);
    const at = this.model.nextChangeAt(now);
    if (this.armed?.at === at) return;
    if (this.armed) this.timers.clearTimeout(this.armed.handle);
    this.armed = null;
    if (at === null) return;
    const handle = this.timers.setTimeout(() => {
      this.armed = null;
      this.frame();
    }, at - now + STALE_DRAW_LAG_MS);
    this.armed = {at, handle};
  }
}

// How many rows of a list are out of view: `rowTops` are its rows' offsets
// from the top of the list's scroll area (rows of one line share an offset
// in a list of several columns), each `rowHeight` tall, of which the area
// shows `visibleHeight` from `scrollTop`. A row cut by an edge counts as out
// of view. The page measures the offsets after each paint (main.ts); this is
// the arithmetic.
export function outOfView(rowTops: readonly number[], rowHeight: number, visibleHeight: number,
                          scrollTop: number): {above: number; below: number} {
  let above = 0, below = 0;
  for (const top of rowTops) {
    if (top + rowHeight > scrollTop + visibleHeight + 0.5) below++;
    else if (top < scrollTop - 0.5) above++;
  }
  return {above, below};
}

// The heading's words for it: "N above", "+N below", or both.
export function outOfViewText(n: {above: number; below: number}): string {
  const words: string[] = [];
  if (n.above > 0) words.push(`${n.above} above`);
  if (n.below > 0) words.push(`+${n.below} below`);
  return words.join(' · ');
}

// The page's box sizes in px, written onto the page as custom properties
// (pageStyle) that drive.css reads with var() and states nowhere else, so
// listRowsFit below computes with the numbers the browser lays out with.
export const METRICS = {
  page_pad: 6,        // around the page
  gap: 6,             // between the top bar and the grid, and between cells
  top: 40,            // the title and status bar
  cell_border: 1,
  cell_pad_y: 4,
  cell_pad_x: 8,
  header: 22,         // a widget's heading line
  row: 22,            // one list row (subsystems, checks, alerts)
  row_font: 14,
  tag_font: 12,
  min_font: 12,       // no text is smaller
  list_gap: 8,        // between list columns
  node_column: 140,   // the narrowest subsystems column: two fit the 2026 cell at 1280 px
  check_column: 220,  // the narrowest checks column: a verdict and a label
  item_gap: 4,        // between a row's name, state word and notes
  tag_pad_x: 5,       // a state word's padding left and right
  tag_border: 1.5,    // and its border
  name_min: 64        // a row's name never narrower: 8 characters at GLYPH_PX.name; its word never shrinks
} as const;

// Glyph widths in px, a model the stylesheet cannot state: an average
// character of a row's name (14 px, semibold, lowercase and `_`) and of a
// state word (12 px, bold, upper case). Measured system-ui faces are
// narrower (QUARANTINED, 11 characters, is about 100 px with its padding),
// so the model errs towards a narrower name.
export const GLYPH_PX = {name: 8, tag: 8} as const;

export function pageStyle(): string {
  return Object.entries(METRICS)
      .map(([name, n]) => `--drive-${name.replace(/_/g, '-')}:${n}px`)
      .join(';');
}

// The columns of a list in the body of a widget at `at` of `layout`'s grid
// on a `page` of that many px: as many of at least `column` px as fit its
// width (CSS grid `repeat(auto-fill, minmax(column, 1fr))`), each `width`
// px, and the rows one column shows. Computed from METRICS and the
// stylesheet's box model, not measured.
export function listColumns(layout: {columns: number; rows: number}, at: Cell,
                            page: {width: number; height: number}, column: number):
    {columns: number; width: number; rows: number} {
  const m = METRICS;
  const gridW = page.width - 2 * m.page_pad;
  const gridH = page.height - 2 * m.page_pad - m.top - m.gap;
  const trackW = (gridW - (layout.columns - 1) * m.gap) / layout.columns;
  const trackH = (gridH - (layout.rows - 1) * m.gap) / layout.rows;
  const cellW = at.width * trackW + (at.width - 1) * m.gap;
  const cellH = at.height * trackH + (at.height - 1) * m.gap;
  const bodyW = cellW - 2 * m.cell_border - 2 * m.cell_pad_x;
  const bodyH = cellH - 2 * m.cell_border - 2 * m.cell_pad_y - m.header;
  const columns = Math.max(1, Math.floor((bodyW + m.list_gap) / (column + m.list_gap)));
  return {columns, width: (bodyW - (columns - 1) * m.list_gap) / columns,
          rows: Math.floor(bodyH / m.row)};
}

// How many list rows fit, without scrolling, in that body.
export function listRowsFit(layout: {columns: number; rows: number}, at: Cell,
                            page: {width: number; height: number}, column: number): number {
  const fit = listColumns(layout, at, page, column);
  return fit.rows * fit.columns;
}

// How many characters of a row's name show beside `word` and `notes` notes
// (a subsystems row has at most one, restarts and drives: shrunk to nothing
// first, still a gap) in a list
// column `width` px wide, by the GLYPH_PX model: what the row leaves after
// its gaps and the word's box, less one character for the ellipsis of a cut
// name.
export function nameCharsShown(width: number, word: string, notes: number): number {
  const m = METRICS;
  const tagW = word.length * GLYPH_PX.tag + 2 * m.tag_pad_x + 2 * m.tag_border;
  return Math.floor((width - (1 + notes) * m.item_gap - tagW) / GLYPH_PX.name) - 1;
}

// The page body: the top bar with its status slot, and the grid, holding
// `status` and `grid`. The page writes it empty and paints both apart
// (Painter), so a button in the status line survives grid updates.
export function renderShell(layout: Layout, status = '', grid = ''): string {
  return `<div class="page" style="${pageStyle()}">` +
      `<header class="top"><h1>${esc(layout.title)}</h1>` +
      `<div class="status-slot">${status}</div></header>` +
      `<main class="grid" style="${gridStyle(layout)}">${grid}</main>` +
      '</div>';
}

// The whole page as one frame draws it.
export function renderDashboard(layout: Layout, model: DriveModel, now: number): string {
  return renderShell(layout, renderStatus(model, now, layout.skipped),
                     renderWidgets(layout, model, now));
}
