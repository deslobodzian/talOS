// The driver dashboard's layout as the bridge serves it at /drive/layout.json
// (talOS/telemetry/README.md "Dashboard layout": DashboardLayoutJson). The C++
// parser has already validated it, so this reads the shape and fills nothing
// in; what it cannot read becomes an `unsupported` widget, which the page
// draws as a visible cell rather than a blank or a crash. No DOM.

export type Cell = {column: number; row: number; width: number; height: number};

export type CheckItem = {
  label: string;
  channel: string;
  equals: boolean|number|string|null;
  at_least: number|null;
  at_most: number|null;
};

type Common = {
  at: Cell;
  title: string;
  // Role to channel name: the widget's single channels (README table).
  channels: Readonly<Record<string, string>>;
};

export type Widget = Common&(
    {kind: 'match'}|
    {kind: 'power'; warn_below_v: number}|
    {kind: 'routine'}|
    {kind: 'alerts'}|
    {kind: 'subsystems'; nodes: string[]}|
    {kind: 'field'; trail_s: number}|
    {kind: 'number'; label: string; precision: number;
     warn_below: number|null; warn_above: number|null}|
    {kind: 'boolean'; label: string; good: boolean}|
    {kind: 'text'; label: string}|
    {kind: 'graph'; series: string[]; window_s: number;
     min: number|null; max: number|null}|
    {kind: 'checks'; items: CheckItem[]}|
    // A kind this page does not draw, or a widget it could not read.
    {kind: 'unsupported'; name: string; reason: string});

export type Layout = {
  title: string;
  columns: number;
  rows: number;
  update_hz: number;
  alerts: boolean;
  channels: string[];
  widgets: Widget[];
  // Widgets with no cell this page can read: not drawn, and counted on the
  // status line so none vanishes silently.
  skipped: number;
};

export class LayoutError extends Error {}

type Json = Record<string, unknown>;
const isObject = (v: unknown): v is Json =>
    typeof v === 'object' && v !== null && !Array.isArray(v);
const str = (v: unknown, fallback = '') => typeof v === 'string' ? v : fallback;
const num = (v: unknown, fallback: number) =>
    typeof v === 'number' && Number.isFinite(v) ? v : fallback;
const posNum = (v: unknown, fallback: number) =>
    typeof v === 'number' && Number.isFinite(v) && v > 0 ? v : fallback;
const nonNegNum = (v: unknown, fallback: number) =>
    typeof v === 'number' && Number.isFinite(v) && v >= 0 ? v : fallback;
const optionalNum = (v: unknown) =>
    typeof v === 'number' && Number.isFinite(v) ? v : null;
const strings = (v: unknown) =>
    Array.isArray(v) ? v.filter((s): s is string => typeof s === 'string') : [];

function readCell(v: unknown, columns: number, rows: number): Cell|null {
  if (!Array.isArray(v) || v.length !== 4 ||
      !v.every(n => Number.isInteger(n) && n >= 0))
    return null;
  const [column, row, width, height] = v as number[];
  if (width < 1 || height < 1 || column + width > columns || row + height > rows)
    return null;
  return {column, row, width, height};
}

function readItem(v: unknown): CheckItem|null {
  if (!isObject(v) || typeof v.channel !== 'string') return null;
  const equals = typeof v.equals === 'boolean' || typeof v.equals === 'string' ||
          (typeof v.equals === 'number' && Number.isFinite(v.equals)) ?
      v.equals as boolean|number|string :
      null;
  return {label: str(v.label, v.channel), channel: v.channel, equals,
          at_least: optionalNum(v.at_least), at_most: optionalNum(v.at_most)};
}

function readWidget(v: unknown, index: number, columns: number, rows: number): Widget|null {
  if (!isObject(v)) return null;
  const at = readCell(v.at, columns, rows);
  if (!at) return null;  // nowhere to draw even an "unsupported" cell
  const channels: Record<string, string> = Object.create(null);
  if (isObject(v.channels))
    for (const [role, name] of Object.entries(v.channels))
      if (typeof name === 'string') channels[role] = name;
  const common: Common = {at, title: str(v.title), channels};
  const unsupported = (reason: string): Widget =>
      ({...common, kind: 'unsupported', name: str(v.kind, '?'), reason});
  switch (v.kind) {
    case 'match':
    case 'alerts':
      return {...common, kind: v.kind};
    case 'routine':
      return channels.channel ? {...common, kind: 'routine'} :
                                unsupported(`widgets[${index}] has no channel`);
    case 'power':
      return {...common, kind: 'power', warn_below_v: num(v.warn_below_v, 11.5)};
    case 'subsystems':
      return {...common, kind: 'subsystems', nodes: strings(v.nodes)};
    case 'field':
      return channels.x && channels.y && channels.yaw ?
          {...common, kind: 'field', trail_s: nonNegNum(v.trail_s, 5)} :
          unsupported(`widgets[${index}] has no pose`);
    case 'number':
      return {...common, kind: 'number', label: str(v.label),
              precision: Math.min(6, Math.max(0, Math.trunc(num(v.precision, 2)))),
              warn_below: optionalNum(v.warn_below),
              warn_above: optionalNum(v.warn_above)};
    case 'boolean':
      return {...common, kind: 'boolean', label: str(v.label),
              good: v.good !== false};
    case 'text':
      return {...common, kind: 'text', label: str(v.label)};
    case 'graph':
      return {...common, kind: 'graph', series: strings(v.series),
              window_s: posNum(v.window_s, 10), min: optionalNum(v.min),
              max: optionalNum(v.max)};
    case 'checks': {
      const items: CheckItem[] = [];
      for (const item of Array.isArray(v.items) ? v.items : []) {
        const read = readItem(item);
        if (read) items.push(read);
      }
      return {...common, kind: 'checks', items};
    }
    default:
      return unsupported(`this page does not draw a "${str(v.kind, '?')}" widget`);
  }
}

// Reads /drive/layout.json. Throws LayoutError only when there is no grid to
// draw; a widget it cannot read is `unsupported`, and one with no cell is
// left out and counted in `skipped`, which the status line shows.
export function parseLayout(json: unknown): Layout {
  if (!isObject(json)) throw new LayoutError('layout is not a JSON object');
  const columns = num(json.columns, 0), rows = num(json.rows, 0);
  if (!Number.isInteger(columns) || !Number.isInteger(rows) || columns < 1 ||
      rows < 1 || columns > 24 || rows > 24)
    throw new LayoutError('layout has no valid grid (columns, rows)');
  const widgets: Widget[] = [];
  let skipped = 0;
  (Array.isArray(json.widgets) ? json.widgets : []).forEach((w, i) => {
    const widget = readWidget(w, i, columns, rows);
    if (widget) widgets.push(widget);
    else skipped++;
  });
  return {
    title: str(json.title, 'Drive'),
    columns,
    rows,
    update_hz: posNum(json.update_hz, 20),
    alerts: json.alerts === true,
    channels: strings(json.channels),
    widgets,
    skipped
  };
}

// The channels whose history a graph or a field trail draws.
export function trackedChannels(layout: Layout): string[] {
  const names: string[] = [];
  for (const w of layout.widgets) {
    if (w.kind === 'graph') names.push(...w.series);
    if (w.kind === 'field') names.push(w.channels.x, w.channels.y);
  }
  return names;
}

function deepEqual(a: unknown, b: unknown): boolean {
  if (Object.is(a, b)) return true;
  if (typeof a !== 'object' || typeof b !== 'object' || a === null || b === null)
    return false;
  if (Array.isArray(a) || Array.isArray(b)) {
    if (!Array.isArray(a) || !Array.isArray(b) || a.length !== b.length) return false;
    for (let i = 0; i < a.length; i++)
      if (!deepEqual(a[i], b[i])) return false;
    return true;
  }
  const ao = a as Record<string, unknown>, bo = b as Record<string, unknown>;
  const ak = Object.keys(ao), bk = Object.keys(bo);
  if (ak.length !== bk.length) return false;
  for (const k of ak) {
    if (!Object.prototype.hasOwnProperty.call(bo, k) || !deepEqual(ao[k], bo[k]))
      return false;
  }
  return true;
}

// Structural equality of two parsed layouts, so a reconnect whose layout did
// not change keeps the existing grid elements and list scroll positions.
export function layoutsEqual(a: Layout, b: Layout): boolean {
  return deepEqual(a, b);
}
