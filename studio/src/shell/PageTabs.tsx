// The page tabs: one per workspace page, with add, inline rename, close, and a
// per-page "reset to preset" menu.

import {useState} from 'react';

import {PRESETS} from '../layout/presets';
import type {Page} from '../layout/types';

export function PageTabs({pages, activePageId, renamingId, onSelect, onStartRename, onRename, onClose, onAdd, onReset}: {
  pages: Page[];
  activePageId: string;
  renamingId: string|null;
  onSelect: (id: string) => void;
  onStartRename: (id: string) => void;
  onRename: (id: string, title: string) => void;
  onClose: (id: string) => void;
  onAdd: () => void;
  onReset: (id: string, presetName: string) => void;
}) {
  const [resetMenu, setResetMenu] = useState<string|null>(null);

  return <nav className="page-tabs" role="tablist" aria-label="Workspace pages">
    {pages.map(page => <div key={page.id}
                            className={'page-tab' + (page.id === activePageId ? ' active' : '')}>
      <button role="tab" aria-selected={page.id === activePageId}
              onClick={() => onSelect(page.id)}
              onDoubleClick={() => onStartRename(page.id)}>
        {renamingId === page.id ?
             <input className="tab-rename" autoFocus defaultValue={page.title}
                    aria-label="Rename page"
                    onClick={e => e.stopPropagation()}
                    onBlur={e => onRename(page.id, e.target.value)}
                    onKeyDown={e => {
                      if (e.key === 'Enter')
                        onRename(page.id, e.currentTarget.value);
                      if (e.key === 'Escape') onRename(page.id, page.title);
                    }}/> :
             page.title}
      </button>
      {page.id === activePageId && <button className="page-reset" title="Reset page to a preset"
              aria-label="Reset page to a preset" aria-haspopup="menu"
              onClick={() => setResetMenu(m => m === page.id ? null : page.id)}>
        ▾
      </button>}
      <button className="page-close" aria-label={`Close ${page.title}`}
              title="Close page" onClick={() => onClose(page.id)}>×</button>
      {resetMenu === page.id && <>
        <div className="menu page-reset-menu" role="menu">
          {PRESETS.map(preset => <button key={preset.name} role="menuitem"
              onClick={() => { onReset(page.id, preset.name); setResetMenu(null); }}>
            <b>{preset.name}</b>
            <span>{preset.description}</span>
          </button>)}
        </div>
        <div className="menu-backdrop" onClick={() => setResetMenu(null)}/>
      </>}
    </div>)}
    <button className="page-add" onClick={onAdd} aria-label="Add a page"
            title="Add a page">+</button>
  </nav>;
}
