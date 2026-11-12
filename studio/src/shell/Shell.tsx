// The Studio shell: workspace state, page management, layout actions, and the
// header / status strip / page tabs / workspace / timeline composition. Panels
// are wired only through the data hooks and the navigation context; the shell
// owns the layout and persistence.

import {useEffect, useMemo, useRef, useState} from 'react';

import {useActions, useEvents, usePlayback, useRecorded} from '../data/hooks';
import {useStudio} from '../data/studio';
import {PanelNavigationProvider} from '../layout/navigation';
import {buildPresetPage, buildPresetRoot, defaultWorkspace, PRESETS, presetByName} from '../layout/presets';
import {
  close,
  findPanel,
  firstPanel,
  makePage,
  newInstanceId,
  panelNode,
  replacePanel,
  resize,
  setConfig,
  split,
} from '../layout/tree';
import type {LayoutNode, Page, Workspace} from '../layout/types';
import {exportWorkspace, importWorkspace, loadWorkspace, saveWorkspace} from '../layout/workspace';
import {PANELS} from '../panels/registry';
import {
  eventCommands,
  pageCommands,
  panelCommands,
  presetCommands,
  transportCommands,
  workspaceCommands,
  type Command,
} from './commands';
import {CommandPalette} from './CommandPalette';
import {Header} from './Header';
import {PageTabs} from './PageTabs';
import {RecordedTransport} from './RecordedTransport';
import {ReviewPicker} from './ReviewPicker';
import {ShortcutsOverlay} from './Shortcuts';
import {StatusStrip} from './StatusStrip';
import {Timeline} from './Timeline';
import {WorkspaceArea, type WorkspaceCallbacks} from './WorkspaceArea';

function defaultUrl(): string {
  if (typeof location === 'undefined') return '';
  return `${location.protocol === 'https:' ? 'wss' : 'ws'}://${
      location.hostname || 'localhost'}:${
      location.port && location.port !== '5173' ? location.port :
                                                  '5800'}/telemetry`;
}

function nextPageNumber(pages: Page[]): number {
  let highest = 0;
  for (const page of pages) {
    const match = /^Page (\d+)$/.exec(page.title);
    if (match) highest = Math.max(highest, Number(match[1]));
  }
  return highest + 1;
}

// Apply a pure tree op to the active page's root.
function withActiveRoot(workspace: Workspace,
                        fn: (root: LayoutNode) => LayoutNode): Workspace {
  const page = workspace.pages.find(p => p.id === workspace.activePage);
  if (!page) return workspace;
  return {
    ...workspace,
    pages: workspace.pages.map(
        p => p.id === page.id ? {...p, root: fn(p.root)} : p)
  };
}

export function Shell() {
  const [workspace, setWorkspace] = useState<Workspace>(() => loadWorkspace());
  const [maximized, setMaximized] = useState<string|null>(null);
  const [url, setUrl] = useState(defaultUrl);
  const [stepNs, setStepNs] = useState('20000000');
  const [transportError, setTransportError] = useState('');
  const [renamingPage, setRenamingPage] = useState<string|null>(null);
  const [paletteOpen, setPaletteOpen] = useState(false);
  const [shortcutsOpen, setShortcutsOpen] = useState(false);
  const [reviewOpen, setReviewOpen] = useState(false);
  const [focusedPanelId, setFocusedPanelId] = useState<string|null>(null);

  const studio = useStudio();
  const recorded = useRecorded();
  const actions = useActions();
  const playback = usePlayback();
  const events = useEvents();
  const importFileRef = useRef<HTMLInputElement>(null);

  useEffect(() => saveWorkspace(workspace), [workspace]);

  const activePage = workspace.pages.find(p => p.id === workspace.activePage) ??
      workspace.pages[0] ?? null;

  // --- pages ---------------------------------------------------------------

  const setActivePage = (id: string) =>
      setWorkspace(w => ({...w, activePage: id}));

  const addPage = () => setWorkspace(w => {
    const page = makePage(
        `Page ${nextPageNumber(w.pages)}`,
        panelNode('health', newInstanceId(), undefined));
    return {...w, pages: [...w.pages, page], activePage: page.id};
  });

  const renamePage = (id: string, title: string) => {
    setRenamingPage(null);
    const trimmed = title.trim().slice(0, 40);
    if (!trimmed) return;
    setWorkspace(w => ({
      ...w,
      pages: w.pages.map(p => p.id === id ? {...p, title: trimmed} : p)
    }));
  };

  const closePage = (id: string) => setWorkspace(w => {
    if (w.pages.length <= 1) return w;
    const remaining = w.pages.filter(p => p.id !== id);
    return {
      ...w,
      pages: remaining,
      activePage: w.activePage === id ? (remaining[0]?.id ?? '') : w.activePage
    };
  });

  const resetPage = (id: string, presetName: string) => {
    const preset = presetByName(presetName);
    if (!preset) return;
    setWorkspace(w => ({
      ...w,
      pages: w.pages.map(
          p => p.id === id ? {...p, root: buildPresetRoot(preset)} : p)
    }));
  };

  // --- panel/layout actions -------------------------------------------------

  const callbacks: WorkspaceCallbacks = {
    onSplit: (instanceId, direction) => {
      setMaximized(null);
      setWorkspace(w => withActiveRoot(w, root => {
        const panel = findPanel(root, instanceId);
        if (!panel) return root;
        return split(root, instanceId, direction, panel.panelId,
                     newInstanceId(), panel.config);
      }));
    },
    onClose: (instanceId) => setWorkspace(w => withActiveRoot(w, root => {
      const next = close(root, instanceId);
      // A page is never left empty: closing its last panel leaves a fresh
      // health panel the user can switch to anything.
      return next ?? panelNode('health', newInstanceId(), undefined);
    })),
    onSwitch: (instanceId, panelId) => setWorkspace(w => withActiveRoot(
        w, root => replacePanel(root, instanceId, panelId))),
    onConfig: (instanceId, config) => setWorkspace(w => withActiveRoot(
        w, root => setConfig(root, instanceId, config))),
    onResize: (path, sizes) => setWorkspace(w => withActiveRoot(
        w, root => resize(root, path, sizes))),
    onMaximize: (instanceId) => setMaximized(instanceId),
    onFocusPanel: (instanceId) => setFocusedPanelId(instanceId)
  };

  // Navigation: "open panel X" adds a panel of that type to the active page.
  // Memoised so the context value is stable: the shell re-renders on every
  // playback tick, and a new navigation object would re-render every panel.
  const navigation = useMemo(() => ({
    openPanel: (panelId: string) => {
      setMaximized(null);
      setWorkspace(w => withActiveRoot(w, root => {
        const target = firstPanel(root);
        return target ?
            split(root, target.instanceId, 'row', panelId, newInstanceId(),
                  undefined) :
            panelNode(panelId, newInstanceId(), undefined);
      }));
    }
  }), []);

  // "Add panel" splits the last-interacted panel on the active page (falling
  // back to the first panel), or replaces an empty root outright.
  const addPanel = (panelId: string) => {
    setMaximized(null);
    setWorkspace(w => withActiveRoot(w, root => {
      const target = (focusedPanelId && findPanel(root, focusedPanelId)) ??
          firstPanel(root);
      return target ?
          split(root, target.instanceId, 'row', panelId, newInstanceId(),
                undefined) :
          panelNode(panelId, newInstanceId(), undefined);
    }));
  };

  // "Open preset page" appends a fresh page built from the named preset.
  const openPresetPage = (name: string) => {
    const preset = presetByName(name);
    if (!preset) return;
    setWorkspace(w => {
      const page = buildPresetPage(preset);
      return {...w, pages: [...w.pages, page], activePage: page.id};
    });
  };

  const onImportClick = () => importFileRef.current?.click();

  // --- transport / workspace ------------------------------------------------

  const onConnect = () => {
    setTransportError('');
    actions.connect(url);
  };
  const onDemo = () => {
    setTransportError('');
    actions.demo();
  };

  const onImport = (text: string) => {
    try {
      setWorkspace(importWorkspace(text));
      setMaximized(null);
    } catch (err) {
      setTransportError(
          err instanceof Error ? err.message : 'Could not import workspace');
    }
  };

  const onExport = () => {
    const blob = new Blob([exportWorkspace(workspace)], {type: 'application/json'});
    const href = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = href;
    a.download = 'talos-studio-workspace.json';
    a.click();
    setTimeout(() => URL.revokeObjectURL(href), 1000);
  };

  const onReset = () => {
    const fresh = defaultWorkspace();
    setWorkspace(fresh);
    setMaximized(null);
    saveWorkspace(fresh);
  };

  // --- command palette ------------------------------------------------------

  const playing = playback.mode === 'REPLAY_PLAYING' ||
      playback.mode === 'LIVE_STREAMING';
  const commands: Command[] = paletteOpen ? [
    ...pageCommands(workspace.pages, workspace.activePage, setActivePage),
    ...panelCommands(PANELS, addPanel),
    ...presetCommands(PRESETS, openPresetPage),
    ...eventCommands(events, timestamp => actions.seek(timestamp),
                     playback.cursor_ns),
    ...transportCommands({
      mode: playback.mode,
      status: playback.status,
      onConnect,
      onDemo,
      onLive: actions.live,
      onPlayPause: () => actions.setMode(playing ? 'PAUSED' : 'REPLAY_PLAYING')
    }),
    ...workspaceCommands(onImportClick, onExport)
  ] :
                                             [];

  // --- keyboard -------------------------------------------------------------

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.defaultPrevented) return;
      // ⌘/Ctrl+K toggles the palette from anywhere.
      if ((e.metaKey || e.ctrlKey) && !e.shiftKey && !e.altKey &&
          e.key.toLowerCase() === 'k') {
        e.preventDefault();
        setShortcutsOpen(false);
        setPaletteOpen(open => !open);
        return;
      }

      if (e.metaKey || e.ctrlKey || e.altKey) return;
      const target = e.target as HTMLElement|null;
      if (target?.closest(
          'button, input, textarea, select, [contenteditable="true"], [role="menu"], [role="dialog"], [role="separator"], [role="slider"]'))
        return;

      // '?' toggles the shortcuts overlay.
      if (e.key === '?') {
        e.preventDefault();
        setPaletteOpen(false);
        setShortcutsOpen(open => !open);
        return;
      }

      // A modal owns its own keys; the transport shortcuts below stay quiet.
      if (paletteOpen || shortcutsOpen || reviewOpen) return;

      const playing = playback.mode === 'REPLAY_PLAYING' ||
          playback.mode === 'LIVE_STREAMING';
      if (e.key === ' ') {
        e.preventDefault();
        actions.setMode(playing ? 'PAUSED' : 'REPLAY_PLAYING');
      } else if (e.key === 'ArrowLeft') {
        if (/^[1-9]\d{0,17}$/.test(stepNs)) actions.step('-' + stepNs);
      } else if (e.key === 'ArrowRight') {
        if (/^[1-9]\d{0,17}$/.test(stepNs)) actions.step(stepNs);
      } else if (e.key.toLowerCase() === 'l') {
        actions.live();
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [actions, playback.mode, stepNs, paletteOpen, shortcutsOpen, reviewOpen]);

  return <PanelNavigationProvider value={navigation}>
    <div className="app">
      <Header url={url} onUrl={setUrl} onConnect={onConnect} onDemo={onDemo}
              onImportClick={onImportClick} onExport={onExport}
              onReset={onReset} onReview={() => setReviewOpen(true)}/>
      <StatusStrip/>
      <PageTabs pages={workspace.pages} activePageId={workspace.activePage}
                renamingId={renamingPage} onSelect={setActivePage}
                onStartRename={setRenamingPage} onRename={renamePage}
                onClose={closePage} onAdd={addPage} onReset={resetPage}/>
      <main>
        {activePage ?
             <WorkspaceArea key={activePage.id} root={activePage.root} maximized={maximized}
                            callbacks={callbacks}/> :
             <div className="empty-state">No pages. Add a page to begin.</div>}
      </main>
      {recorded ?
           <RecordedTransport source={recorded} stepNs={stepNs} onStepNs={setStepNs}
                              onClose={studio.closeRecorded}/> :
           <Timeline stepNs={stepNs} onStepNs={setStepNs}
                     transportError={transportError}
                     onTransportError={setTransportError}/>}
      <input ref={importFileRef} type="file" accept=".json" hidden
             aria-label="Import workspace JSON"
             onChange={async e => {
               const file = e.target.files?.[0];
               if (file) onImport(await file.text());
               e.target.value = '';
             }}/>
      {paletteOpen &&
       <CommandPalette commands={commands} onClose={() => setPaletteOpen(false)}/>}
      {reviewOpen && <ReviewPicker onClose={() => setReviewOpen(false)}/>}
      {shortcutsOpen &&
       <ShortcutsOverlay onClose={() => setShortcutsOpen(false)}/>}
    </div>
  </PanelNavigationProvider>;
}
