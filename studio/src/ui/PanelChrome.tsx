// The chrome around every panel: a header with the panel title (which opens the
// panel-type switcher), the split/maximize/close actions, and the panel's
// body. The switcher lists panels grouped by category with descriptions.

import {useEffect, useState} from 'react';

import {panelsByCategory} from '../panels/registry';
import type {PanelCategory} from '../panels/types';
import type {SplitDirection} from '../layout/types';

export function PanelChrome({panelId, title, category, subtitle, onSwitch, onSplit, onMaximize, onClose, maximized, onInteract, children}: {
  panelId: string;
  title: string;
  category: PanelCategory;
  subtitle?: string;
  onSwitch: (panelId: string) => void;
  onSplit: (direction: SplitDirection) => void;
  onMaximize: () => void;
  onClose: () => void;
  maximized: boolean;
  onInteract?: () => void;
  children: React.ReactNode;
}) {
  const [open, setOpen] = useState(false);

  useEffect(() => {
    if (!open) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') setOpen(false);
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [open]);

  return <section className="panel-shell" onPointerDownCapture={onInteract}
                  onFocusCapture={onInteract}>
    <div className="panel-heading">
      <button className="panel-title" aria-haspopup="menu"
              aria-expanded={open} onClick={() => setOpen(o => !o)}
              title="Switch panel type">
        <h2>{title}</h2>
        <span className="panel-category">{category}</span>
        {subtitle && <span className="panel-subtitle">{subtitle}</span>}
      </button>
      <div className="panel-actions">
        <button aria-label="Split right" title="Split right"
                onClick={() => onSplit('row')}>⫘</button>
        <button aria-label="Split down" title="Split down"
                onClick={() => onSplit('column')}>⫛</button>
        <button aria-label={maximized ? 'Restore' : 'Maximize'}
                title={maximized ? 'Restore' : 'Maximize'} onClick={onMaximize}>
          {maximized ? '⤡' : '⤢'}
        </button>
        <button aria-label="Close panel" title="Close panel" onClick={onClose}>
          ×
        </button>
      </div>
    </div>

    {open && <div className="panel-switcher" role="menu">
      <div className="panel-switcher-list">
        {[...panelsByCategory().entries()].map(([cat, panels]) =>
          <div className="panel-switcher-group" key={cat}>
            <h4>{cat}</h4>
            {panels.map(panel =>
              <button key={panel.id} role="menuitem"
                      className={panel.id === panelId ? 'current' : ''}
                      onClick={() => { onSwitch(panel.id); setOpen(false); }}>
                <b>{panel.title}</b>
                <span>{panel.description}</span>
              </button>)}
          </div>)}
      </div>
      <button className="panel-switcher-backdrop"
              aria-label="Close panel picker"
              onClick={() => setOpen(false)}/>
    </div>}

    <div className="panel-body">{children}</div>
  </section>;
}
