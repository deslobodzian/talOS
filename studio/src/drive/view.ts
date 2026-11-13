// The driver dashboard as HTML text: a pure function of the layout, the feed
// model and the time, so a test renders the 2026 layout with no DOM and the
// page (main.ts) only swaps the text in. Every state that matters is told by
// a word as well as a colour (STALE, ERROR, LOW, DOWN, ...), and every
// string from the feed is escaped.

import {channelDisplay, type Snapshot} from '../core';
import {alertLevelName, type ActiveAlert, type LatchedAlert} from '../data/alerts';
import {FIELD_HEIGHT_M, FIELD_WIDTH_M} from '../panels/field/geometry';
import type {DriveModel, Link} from './feed';
import type {CheckItem, Layout, Widget} from './layout';

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

function shell(w: Widget, stale: boolean, body: string, extra = ''): string {
  const {column, row, width, height} = w.at;
  return `<section class="w w-${esc(w.kind)}${stale ? ' stale' : ''}" ` +
      `style="grid-column:${column + 1} / span ${width};grid-row:${row + 1} / span ${height}">` +
      `<header><h2>${esc(heading(w))}</h2>${extra}` +
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

function subsystemsWidget(model: DriveModel, w: Widget&{kind: 'subsystems'}): string {
  const summary = model.summary;
  if (!summary) return `<p class="none">${ABSENT}</p>`;
  const nodes = w.nodes.length ?
      summary.nodes.filter(n => w.nodes.includes(n.name)) : summary.nodes;
  const rows = nodes.map(n => {
    const alive = !summary.registry ? tag('unknown', 'neutral') :
        n.alive ? tag('UP', 'good') : tag('DOWN', n.critical ? 'bad' : 'warn');
    const restarts = summary.supervised && n.restarts > 0 ?
        `<span class="meta">restarts ${n.restarts}</span>` : '';
    const quarantined = summary.supervised && n.quarantined ? tag('QUARANTINED', 'bad') : '';
    const drives = !summary.registry ? '' : n.drives !== null ?
        `<span class="meta">drives ${esc(n.drives)}</span>` : '';
    return `<li><span class="name">${esc(n.name)}</span>${alive}${quarantined}${restarts}${drives}</li>`;
  });
  if (summary.omitted > 0) rows.push(`<li class="more">+${summary.omitted} more</li>`);
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
    return `<li>${verdict}<span class="name">${esc(item.label)}</span></li>`;
  });
  return `<ul class="checks">${rows.join('')}</ul>`;
}

function widget(w: Widget, model: DriveModel, now: number): string {
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
  telemetry: 'TELEMETRY', roborio: 'ROBORIO LINK', replaced: 'REPLACED'
};

export function renderStatus(model: DriveModel, now: number): string {
  const status = model.status(now);
  const c = model.counters;
  const ignored = c.malformed_binary + c.malformed_text;
  return `<div class="status link-${status.link}" role="status">` +
      `<b>${LINK_WORD[status.link]}</b> ${esc(status.text)}` +
      (status.link === 'replaced' ?
           ' <button type="button" data-action="reconnect">Reconnect</button>' : '') +
      (ignored ? ` <span class="meta">${ignored} malformed message${ignored === 1 ? '' : 's'} ignored</span>` : '') +
      '</div>';
}

// The widgets, for the grid element (gridStyle sizes it).
export function renderWidgets(layout: Layout, model: DriveModel, now: number): string {
  return layout.widgets.map(w => widget(w, model, now)).join('');
}

export function gridStyle(layout: Layout): string {
  return `grid-template-columns:repeat(${layout.columns},minmax(0,1fr));` +
      `grid-template-rows:repeat(${layout.rows},minmax(0,1fr))`;
}

// The whole page body; the page itself updates the status line and the grid
// apart, so a button in the status line survives grid updates.
export function renderDashboard(layout: Layout, model: DriveModel, now: number): string {
  return `<header class="top"><h1>${esc(layout.title)}</h1>` +
      `<div class="status-slot">${renderStatus(model, now)}</div></header>` +
      `<main class="grid" style="${gridStyle(layout)}">${renderWidgets(layout, model, now)}</main>`;
}
