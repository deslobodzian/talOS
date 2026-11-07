// Command palette commands. Each source is a small pure function that turns its
// inputs into a list of `{id, title, hint?, run}` entries, so a new command is
// one more function here -- the palette and the shell never learn about a
// specific command. `run` is a closure supplied by the shell; the builders
// themselves are deterministic and unit-tested in tests/commands.test.ts.

import type {Mode, RobotEvent} from '../data/types';
import {duration} from '../ui/formatters';

export type Command = {
  id: string;
  title: string;
  hint?: string;
  keywords?: string;
  run: () => void;
};

// --- matching --------------------------------------------------------------

// A fuzzy-ish substring score. Higher is better; -1 means no match. The
// ranking prefers, in order: exact, prefix, an early consecutive substring,
// then a subsequence with consecutive runs weighted higher than scattered
// matches. Case-insensitive.
export function score(query: string, text: string): number {
  const q = query.trim().toLowerCase();
  const t = text.toLowerCase();
  if (!q) return 0;
  if (t === q) return 1000;
  if (t.startsWith(q)) return 800;
  const index = t.indexOf(q);
  if (index >= 0) return 600 - Math.min(index, 400);

  // Subsequence fallback: every character in order, scattered or consecutive.
  let out = 0;
  let from = 0;
  let last = -1;
  for (const ch of q) {
    const found = t.indexOf(ch, from);
    if (found < 0) return -1;
    out += last >= 0 && found === last + 1 ? 3 : 1;
    last = found;
    from = found + 1;
  }
  return out;
}

// Filter and rank commands against a query. An empty query returns them in
// their given order; otherwise only matches, best first, ties alphabetical.
export function filterCommands(commands: readonly Command[], query: string):
    Command[] {
  const q = query.trim();
  if (!q) return [...commands];
  return commands
      .map(command => {
        const titleRank = score(q, command.title);
        const rank = titleRank >= 800 ? titleRank + 1000 :
            score(q, `${command.title} ${command.keywords ?? ''} ${command.hint ?? ''}`);
        return {command, rank};
      })
      .filter(entry => entry.rank >= 0)
      .sort((a, b) => b.rank - a.rank ||
                     a.command.title.localeCompare(b.command.title))
      .map(entry => entry.command);
}

// --- command sources -------------------------------------------------------

type PageRef = {id: string; title: string};
export function pageCommands(pages: readonly PageRef[], activePageId: string,
                             run: (id: string) => void): Command[] {
  return pages.map(page => ({
    id: `page:${page.id}`,
    title: `Go to page ${page.title}`,
    hint: page.id === activePageId ? 'current' : undefined,
    keywords: page.title,
    run: () => run(page.id)
  }));
}

type PanelRef = {id: string; title: string; category: string; description: string};
export function panelCommands(panels: readonly PanelRef[],
                              run: (panelId: string) => void): Command[] {
  return panels.map(panel => ({
    id: `panel:${panel.id}`,
    title: `Add panel ${panel.title}`,
    hint: panel.category,
    keywords: panel.description,
    run: () => run(panel.id)
  }));
}

type PresetRef = {name: string; description: string};
export function presetCommands(presets: readonly PresetRef[],
                               run: (name: string) => void): Command[] {
  return presets.map(preset => ({
    id: `preset:${preset.name}`,
    title: `Open preset page ${preset.name}`,
    hint: preset.description,
    keywords: preset.description,
    run: () => run(preset.name)
  }));
}

// How far back an event is from the playback cursor, for the event command
// hint. Empty when there is no cursor to measure against.
export function relativeEventTime(eventNs: string,
                                  cursorNs: string|null|undefined): string {
  if (cursorNs == null) return '';
  const delta = Number(BigInt(cursorNs) - BigInt(eventNs));
  if (delta < 0) return '+' + duration(-delta);
  if (delta === 0) return 'now';
  return duration(delta) + ' ago';
}

// Newest first, capped so a long session's history stays a usable menu.
export function eventCommands(events: readonly RobotEvent[],
                              run: (t_ns: string) => void,
                              cursorNs?: string|null,
                              limit = 50): Command[] {
  return [...events]
      .sort((a, b) => (BigInt(a.t_ns) < BigInt(b.t_ns) ? 1 :
                      BigInt(a.t_ns) > BigInt(b.t_ns) ? -1 : 0))
      .slice(0, limit)
      .map(event => ({
        id: `event:${event.id}`,
        title: `Seek to event ${event.title}`,
        hint: relativeEventTime(event.t_ns, cursorNs),
        keywords: `${event.title} ${event.category} ${event.channel ?? ''}`,
        run: () => run(event.t_ns)
      }));
}

export type TransportOptions = {
  mode: Mode;
  status: string;
  onConnect: () => void;
  onDemo: () => void;
  onLive: () => void;
  onPlayPause: () => void;
};
export function transportCommands(options: TransportOptions): Command[] {
  const playing = options.mode === 'REPLAY_PLAYING' ||
      options.mode === 'LIVE_STREAMING';
  return [
    {id: 'connect', title: 'Connect', hint: options.status, run: options.onConnect},
    {id: 'demo', title: 'Demo data', run: options.onDemo},
    {id: 'live', title: 'Go live', run: options.onLive},
    {id: 'play-pause', title: playing ? 'Pause' : 'Play', run: options.onPlayPause}
  ];
}

export function workspaceCommands(onImport: () => void,
                                  onExport: () => void): Command[] {
  return [
    {id: 'workspace:import', title: 'Import workspace', run: onImport},
    {id: 'workspace:export', title: 'Export workspace', run: onExport}
  ];
}
