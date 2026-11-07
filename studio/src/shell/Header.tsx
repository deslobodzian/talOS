// The shell header: brand, connection control, and the workspace menu. The
// connection status reads usePlayback(); connect/demo forward to useActions()
// via the callbacks the shell passes down. The import file input lives in the
// shell so the command palette can trigger the same picker; this header just
// calls onImportClick.

import {useState} from 'react';

import {usePlayback} from '../data/hooks';
import {StatusDot} from '../ui/primitives';

export function Header({url, onUrl, onConnect, onDemo, onImportClick, onExport, onReset}: {
  url: string;
  onUrl: (url: string) => void;
  onConnect: () => void;
  onDemo: () => void;
  onImportClick: () => void;
  onExport: () => void;
  onReset: () => void;
}) {
  const playback = usePlayback();
  const [menuOpen, setMenuOpen] = useState(false);

  return <header>
    <div className="brand">
      <div className="mark">t</div>
      <div>
        talOS <strong>STUDIO</strong>
        <small>DETERMINISTIC TELEMETRY</small>
      </div>
    </div>
    <div className="connection">
      <StatusDot active={playback.connection === 'live' ||
                         playback.connection === 'demo'}/>
      <span>{playback.status}</span>
      <input aria-label="WebSocket URL" value={url}
             onChange={e => onUrl(e.target.value)}/>
      <button className="primary" onClick={onConnect}>Connect</button>
      <button onClick={onDemo}>Demo data</button>
    </div>
    <div className="workspace-menu">
      <button onClick={() => setMenuOpen(o => !o)} aria-haspopup="menu"
              aria-expanded={menuOpen}>Workspace ▾</button>
      {menuOpen && <>
        <div className="menu" role="menu">
          <button role="menuitem" onClick={() => {
            onImportClick();
            setMenuOpen(false);
          }}>Import workspace…</button>
          <button role="menuitem" onClick={() => {
            onExport();
            setMenuOpen(false);
          }}>Export workspace</button>
          <button role="menuitem" className="danger" onClick={() => {
            onReset();
            setMenuOpen(false);
          }}>Reset workspace</button>
        </div>
        <div className="menu-backdrop" onClick={() => setMenuOpen(false)}/>
      </>}
    </div>
  </header>;
}
