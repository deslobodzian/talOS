// The command palette: a keyboard-first modal over every command the shell
// knows. Typing filters with fuzzy-ish substring scoring; Up/Down move,
// Enter runs, Escape closes. It traps focus while open and restores focus to
// whatever had it when it closed.

import {useEffect, useMemo, useRef, useState} from 'react';

import {filterCommands, type Command} from './commands';

export function CommandPalette({commands, onClose}: {
  commands: readonly Command[];
  onClose: () => void;
}) {
  const [query, setQuery] = useState('');
  const [active, setActive] = useState(0);
  const inputRef = useRef<HTMLInputElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);

  const filtered = useMemo(() => filterCommands(commands, query), [commands, query]);
  const activeIndex = filtered.length ? Math.min(active, filtered.length - 1) : -1;

  // Autofocus and remember where focus was so close restores it.
  useEffect(() => {
    const previous = document.activeElement as HTMLElement|null;
    inputRef.current?.focus();
    return () => {
      previous?.focus?.();
    };
  }, []);

  const run = (command: Command) => {
    command.run();
    onClose();
  };

  // One document-level listener covers navigation wherever focus is, so the
  // palette stays keyboard-first even after clicking a result.
  useEffect(() => {
    const onKey = (event: KeyboardEvent) => {
      if (event.key === 'ArrowDown') {
        event.preventDefault();
        setActive(a => (filtered.length ? Math.min(a + 1, filtered.length - 1) : 0));
      } else if (event.key === 'ArrowUp') {
        event.preventDefault();
        setActive(a => Math.max(a - 1, 0));
      } else if (event.key === 'Enter') {
        event.preventDefault();
        const command = filtered[Math.min(active, Math.max(0, filtered.length - 1))];
        if (command) run(command);
      } else if (event.key === 'Escape') {
        event.preventDefault();
        onClose();
      } else if (event.key === 'Tab') {
        const focusables = containerRef.current?.querySelectorAll<HTMLElement>(
            'input, button');
        if (!focusables || focusables.length < 2) return;
        const first = focusables[0], last = focusables[focusables.length - 1];
        if (event.shiftKey && document.activeElement === first) {
          event.preventDefault();
          last.focus();
        } else if (!event.shiftKey && document.activeElement === last) {
          event.preventDefault();
          first.focus();
        }
      }
    };
    document.addEventListener('keydown', onKey);
    return () => document.removeEventListener('keydown', onKey);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [filtered, active, onClose]);

  return <div className="palette-backdrop"
              onMouseDown={e => { if (e.target === e.currentTarget) onClose(); }}>
    <div className="palette" role="dialog" aria-modal="true"
         aria-label="Command palette" ref={containerRef}>
      <input ref={inputRef} value={query} role="combobox"
             aria-label="Search commands" aria-expanded="true"
             aria-controls="palette-list"
             aria-activedescendant={activeIndex >= 0 ? filtered[activeIndex].id :
                                                       undefined}
             placeholder="Type a command…"
             onChange={e => { setQuery(e.target.value); setActive(0); }}/>
      <div className="palette-list" id="palette-list" role="listbox">
        {filtered.map((command, i) =>
          <button key={command.id} id={command.id} role="option"
                  aria-selected={i === activeIndex}
                  className={'palette-item' + (i === activeIndex ? ' active' : '')}
                  onMouseEnter={() => setActive(i)} onClick={() => run(command)}>
            <span className="palette-item-title">{command.title}</span>
            {command.hint && <span className="palette-item-hint">{command.hint}</span>}
          </button>)}
        {!filtered.length && <div className="palette-empty">No commands match.</div>}
      </div>
    </div>
  </div>;
}
