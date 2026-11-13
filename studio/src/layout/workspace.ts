// Workspace persistence: localStorage plus JSON import/export.
//
// The tree stays pure JSON, so persistence is nothing but a serialize/parse
// round trip guarded the way the old tab persistence was: a blocked or full
// storage, or JSON that does not parse, means "start from the default" rather
// than "fail to start".

import {defaultWorkspace} from './presets';
import {parseWorkspace, serialize} from './tree';
import type {Workspace} from './types';

export const WORKSPACE_KEY = 'talos-studio-workspace';

// The single-page/tab and graph-list keys from the old shell. Removed once on
// first load so a returning user does not carry dead state, and so a fresh
// profile does not half-restore an old single tab over the new workspace.
const OBSOLETE_KEYS = ['talos-studio-tab', 'talos-studio-graphs'];

function removeObsoleteKeys() {
  try {
    for (const key of OBSOLETE_KEYS) localStorage.removeItem(key);
  } catch {
    // Storage blocked: nothing to remove, nothing lost.
  }
}

export function loadWorkspace(): Workspace {
  removeObsoleteKeys();
  try {
    const raw = localStorage.getItem(WORKSPACE_KEY);
    if (!raw) return defaultWorkspace();
    const parsed = parseWorkspace(JSON.parse(raw));
    // A valid-but-empty document (all pages malformed) is not a workspace the
    // user can be stranded in; fall back to the default rather than an empty
    // shell with no way to add anything.
    return parsed.pages.length ? parsed : defaultWorkspace();
  } catch {
    return defaultWorkspace();
  }
}

export function saveWorkspace(workspace: Workspace): void {
  try {
    localStorage.setItem(WORKSPACE_KEY, serialize(workspace));
  } catch {
    // Storage full or blocked. The workspace still works for this session;
    // only remembering it across a reload is lost.
  }
}

// Import/export use the same validation as persistence, but an invalid import
// is rejected instead of replacing the user's current workspace with defaults.
export function exportWorkspace(workspace: Workspace): string {
  return serialize(workspace);
}

export function importWorkspace(text: string): Workspace {
  const parsed = parseWorkspace(JSON.parse(text));
  if (!parsed.pages.length) throw new Error('Invalid workspace document');
  return parsed;
}
