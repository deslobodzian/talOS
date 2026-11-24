// The page tabs: one per workspace page, with add, inline rename, close, and a
// per-page "reset to preset" menu.

import {useEffect, useRef, useState} from 'react';

import {PRESETS} from '../layout/presets';
import type {Page} from '../layout/types';

function RenameInput({initialTitle, onCommit, onCancel}: {
  initialTitle: string;
  onCommit: (title: string) => void;
  onCancel: () => void;
}) {
  const doneRef = useRef(false);
  const finish = (commit: boolean, value: string) => {
    if (doneRef.current) return;
    doneRef.current = true;
    if (commit) onCommit(value);
    else onCancel();
  };
  return <input className="tab-rename" autoFocus defaultValue={initialTitle}
                aria-label="Rename page"
                onClick={e => e.stopPropagation()}
                onBlur={e => finish(true, e.target.value)}
                onKeyDown={e => {
                  if (e.key === 'Enter') {
                    e.preventDefault();
                    e.stopPropagation();
                    finish(true, e.currentTarget.value);
                  } else if (e.key === 'Escape') {
                    e.preventDefault();
                    e.stopPropagation();
                    finish(false, initialTitle);
                  }
                }}/>;
}

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
  const openResetMenu = resetMenu === activePageId ? resetMenu : null;

  useEffect(() => {
    setResetMenu(null);
  }, [activePageId]);

  useEffect(() => {
    if (openResetMenu === null) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        e.preventDefault();
        setResetMenu(null);
      }
    };
    document.addEventListener('keydown', onKey);
    return () => document.removeEventListener('keydown', onKey);
  }, [openResetMenu]);

  return <nav className="page-tabs" role="tablist" aria-label="Workspace pages">
    {pages.map(page => <div key={page.id}
                            className={'page-tab' + (page.id === activePageId ? ' active' : '')}>
      {renamingId === page.id ?
           <RenameInput key={page.id} initialTitle={page.title}
                        onCommit={title => onRename(page.id, title)}
                        onCancel={() => onRename(page.id, page.title)}/> :
           <button role="tab" aria-selected={page.id === activePageId}
                   onClick={() => onSelect(page.id)}
                   onDoubleClick={() => onStartRename(page.id)}>
             {page.title}
           </button>}
      {page.id === activePageId && <button className="page-reset" title="Reset page to a preset"
              aria-label="Reset page to a preset" aria-haspopup="menu"
              aria-expanded={openResetMenu === page.id}
              onClick={() => setResetMenu(m => m === page.id ? null : page.id)}>
        ▾
      </button>}
      <button className="page-close" aria-label={`Close ${page.title}`}
              title="Close page" onClick={() => onClose(page.id)}>×</button>
      {openResetMenu === page.id && <>
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
