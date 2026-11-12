// The session picker: lists the review server's sessions (studio_review,
// default http://127.0.0.1:5805) and opens one as the Studio's source. The
// server address is remembered per browser; every failure is shown here.
// When studio_review served this Studio, `server` is its origin and the
// picker has no address field.

import {useCallback, useEffect, useState} from 'react';

import {RecordedSource} from '../data/recorded';
import {
  DEFAULT_REVIEW_URL,
  errorMessage,
  listSessions,
  type FetchLike,
  type ReviewSession
} from '../data/review';
import {useStudio} from '../data/studio';
import {duration, wallDateTime} from '../ui/formatters';

const SERVER_KEY = 'talos-studio-review-server';

function loadServer(): string {
  try {
    return localStorage.getItem(SERVER_KEY) || DEFAULT_REVIEW_URL;
  } catch {
    return DEFAULT_REVIEW_URL;
  }
}

function saveServer(server: string) {
  try {
    localStorage.setItem(SERVER_KEY, server);
  } catch {
    // A private window: the default is used next time.
  }
}

export function ReviewPicker({onClose, fetchFn, server: fixed}: {
  onClose: () => void;
  fetchFn?: FetchLike;
  server?: string;
}) {
  const studio = useStudio();
  const [server, setServer] = useState(() => fixed ?? loadServer());
  const [sessions, setSessions] = useState<ReviewSession[]|null>(null);
  const [error, setError] = useState('');
  const [busy, setBusy] = useState(false);
  const fetcher = fetchFn ?? ((url, init) => fetch(url, init));

  const list = useCallback(async (address: string) => {
    setBusy(true);
    setError('');
    try {
      setSessions(await listSessions(address, fetcher));
      if (!fixed) saveServer(address);
    } catch (e) {
      setSessions(null);
      setError(errorMessage(e));
    } finally {
      setBusy(false);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useEffect(() => {
    void list(server);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

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

  const open = async (id: string) => {
    setBusy(true);
    setError('');
    try {
      studio.openRecorded(await RecordedSource.open(server, id, {fetch: fetcher}));
      onClose();
    } catch (e) {
      setError(errorMessage(e));
    } finally {
      setBusy(false);
    }
  };

  return <div className="palette-backdrop"
              onMouseDown={e => { if (e.target === e.currentTarget) onClose(); }}>
    <div className="review-picker" role="dialog" aria-modal="true"
         aria-label="Open a recorded session">
      <h3>Open a recorded session</h3>
      {!fixed && <form onSubmit={e => {
        e.preventDefault();
        void list(server);
      }}>
        <input aria-label="Review server address" value={server}
               onChange={e => setServer(e.target.value)}/>
        <button disabled={busy}>List sessions</button>
      </form>}
      {error && <div className="alert-error">{error}</div>}
      {sessions && !sessions.length && <p className="review-empty">No sessions.</p>}
      {sessions && sessions.length > 0 && <table className="review-sessions">
        <thead><tr><th>Started</th><th>Length</th><th>Class</th><th>Session</th><th/></tr></thead>
        <tbody>
          {sessions.map(session => <tr key={session.id}>
            <td>{wallDateTime(session.start_wall_ns)}</td>
            <td>{BigInt(session.end_wall_ns) > BigInt(session.start_wall_ns) ?
                     duration(Number(BigInt(session.end_wall_ns) -
                                     BigInt(session.start_wall_ns))) : '—'}</td>
            <td>{session.class}</td>
            <td><code>{session.id}</code></td>
            <td><button disabled={busy} onClick={() => void open(session.id)}>Open</button></td>
          </tr>)}
        </tbody>
      </table>}
    </div>
  </div>;
}
