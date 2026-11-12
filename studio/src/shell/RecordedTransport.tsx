// The footer while a recorded session is open, in place of the live Timeline:
// step, play/pause at 1x/2x/4x, a scrub slider over the session, the playhead
// as session time and wall time, the review error if any, and Close.

import {useSyncExternalStore} from 'react';

import {usePlayback} from '../data/hooks';
import {PLAY_RATES, type PlayRate, type RecordedSource} from '../data/recorded';
import {wallTimeNs} from '../data/review';
import {sessionTime, timestamp, wallDateTime} from '../ui/formatters';

export function RecordedTransport({source, stepNs, onStepNs, onClose}: {
  source: RecordedSource;
  stepNs: string;
  onStepNs: (value: string) => void;
  onClose: () => void;
}) {
  const playback = usePlayback();
  const rate = useSyncExternalStore(source.subscribe, () => source.rate, () => source.rate);
  const error = useSyncExternalStore(source.subscribe, () => source.error, () => source.error);

  const start = BigInt(source.bounds.start_ns), end = BigInt(source.bounds.end_ns);
  const cursor = playback.cursor_ns === null ? start : BigInt(playback.cursor_ns);
  const stepOk = /^[1-9]\d{0,17}$/.test(stepNs);
  const playing = playback.mode === 'REPLAY_PLAYING';
  const spanMs = Number((end - start) / 1000000n);

  return <footer className="timeline recorded">
    <div className="playback">
      <button aria-label="Step backward" disabled={!stepOk}
              onClick={() => source.step('-' + stepNs)}>▏◀</button>
      <button className="play" aria-label="Play or pause"
              onClick={() => playing ? source.pause() : source.play()}>
        {playing ? 'Ⅱ' : '▶'}
      </button>
      <button aria-label="Step forward" disabled={!stepOk}
              onClick={() => source.step(stepNs)}>▶▕</button>
      <select aria-label="Playback rate" value={rate}
              onChange={e => source.setRate(Number(e.target.value) as PlayRate)}>
        {PLAY_RATES.map(r => <option key={r} value={r}>{r}x</option>)}
      </select>
    </div>
    <div className="scrub-wrap">
      <div className="scrub-labels">
        <span>RECORDED · {source.id}</span>
        <code>{sessionTime(cursor - start)} · {wallDateTime(
            wallTimeNs(source.bounds, cursor.toString()))}</code>
      </div>
      <input type="range" className="scrub-range" aria-label="Recorded session time"
             min={0} max={Math.max(1, spanMs)} step={1}
             value={Number((cursor - start) / 1000000n)}
             onChange={e => source.seek(
                 (start + BigInt(Math.round(Number(e.target.value))) * 1000000n)
                     .toString())}/>
      <div className="scrub-bounds">
        <span>{sessionTime(0n)}</span>
        <span>{timestamp(playback.cursor_ns)}</span>
        <span>{sessionTime(end - start)}</span>
      </div>
    </div>
    <div className="precision">
      <label>
        Step (ns)
        <input aria-label="Frame step duration in nanoseconds"
               inputMode="numeric" value={stepNs}
               onChange={e => onStepNs(e.target.value)}/>
      </label>
    </div>
    <button onClick={onClose}>Close recording</button>
    {error && <div className="alert-error">
      {error} <button onClick={() => source.retry()}>Retry</button>
    </div>}
  </footer>;
}
