// The keyboard shortcuts overlay. '?' toggles it; Escape or a click outside
// closes it.

import {useEffect} from 'react';

const SHORTCUTS: {keys: string; label: string}[] = [
  {keys: 'Space', label: 'Play / pause'},
  {keys: '← / →', label: 'Step back / forward'},
  {keys: 'L', label: 'Go live'},
  {keys: '⌘ / Ctrl + K', label: 'Command palette'},
  {keys: '?', label: 'This overlay'},
  {keys: 'Esc', label: 'Close'}
];

export function ShortcutsOverlay({onClose}: {onClose: () => void}) {
  useEffect(() => {
    const onKey = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        event.preventDefault();
        onClose();
      }
    };
    document.addEventListener('keydown', onKey);
    return () => document.removeEventListener('keydown', onKey);
  }, [onClose]);

  return <div className="palette-backdrop"
              onMouseDown={e => { if (e.target === e.currentTarget) onClose(); }}>
    <div className="shortcuts" role="dialog" aria-modal="true"
         aria-label="Keyboard shortcuts">
      <h3>Keyboard shortcuts</h3>
      <ul>
        {SHORTCUTS.map(shortcut => <li key={shortcut.keys}>
          <kbd>{shortcut.keys}</kbd>
          <span>{shortcut.label}</span>
        </li>)}
      </ul>
    </div>
  </div>;
}
