// The keyboard shortcuts overlay. '?' toggles it; Escape or a click outside
// closes it.

import {useEffect, useRef} from 'react';

const SHORTCUTS: {keys: string; label: string}[] = [
  {keys: 'Space', label: 'Play / pause'},
  {keys: '← / →', label: 'Step back / forward'},
  {keys: 'L', label: 'Go live'},
  {keys: '⌘ / Ctrl + K', label: 'Command palette'},
  {keys: '?', label: 'This overlay'},
  {keys: 'Esc', label: 'Close'}
];

// Escape anywhere on the page calls `onClose` while the overlay is mounted.
export function useEscape(onClose: () => void) {
  const onCloseRef = useRef(onClose);
  onCloseRef.current = onClose;
  useEffect(() => {
    const onKey = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        event.preventDefault();
        onCloseRef.current();
      }
    };
    document.addEventListener('keydown', onKey);
    return () => document.removeEventListener('keydown', onKey);
  }, []);
}

export function ShortcutsOverlay({onClose}: {onClose: () => void}) {
  useEscape(onClose);

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
