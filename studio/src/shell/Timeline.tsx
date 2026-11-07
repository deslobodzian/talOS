// The footer timeline: playback controls, a scrub track that draws Driver
// Station mode bands and severity-colored event ticks, exact-timestamp seek,
// and log import/export. All transport goes through useActions().

import {useRef, useState} from 'react';

import {useActions, useEvents, useModeBands, usePlayback} from '../data/hooks';
import {timestamp} from '../ui/formatters';

function ScrubTrack() {
  const playback = usePlayback();
  const actions = useActions();
  const bands = useModeBands();
  const events = useEvents();
  const svgRef = useRef<SVGSVGElement>(null);

  const bounds = playback.bounds;
  if (!bounds) return <div className="scrub-empty">No data yet</div>;

  const start = BigInt(bounds.start), end = BigInt(bounds.end);
  const span = Number(end - start) || 1;
  const x = (t: string) => Number(BigInt(t) - start) / span * 1000;

  const onTrackClick = (e: React.MouseEvent) => {
    const rect = svgRef.current?.getBoundingClientRect();
    if (!rect) return;
    const frac = Math.max(0, Math.min(1, (e.clientX - rect.left) / rect.width));
    const t = start + BigInt(Math.round(Number(end - start) * frac));
    actions.seek(t.toString());
  };

  const cursorX = playback.cursor_ns ? x(playback.cursor_ns) : 0;

  return <svg ref={svgRef} className="scrub" viewBox="0 0 1000 32"
              preserveAspectRatio="none" role="slider"
              aria-label="Replay timeline" aria-valuemin={0} aria-valuemax={1000}
              aria-valuenow={Math.round(cursorX)} onClick={onTrackClick}>
    {bands.map((band, i) => <rect key={i} x={x(band.start_ns)} y={7}
        width={Math.max(1, x(band.end_ns) - x(band.start_ns))} height={12}
        className={'band ' + band.mode}/>)}
    {events.map(event => <line key={event.id} x1={x(event.t_ns)} y1={2}
        x2={x(event.t_ns)} y2={19} className={'tick ' + event.severity}>
      <title>{event.title}</title>
    </line>)}
    <line x1={cursorX} y1={0} x2={cursorX} y2={32} className="scrub-cursor"/>
  </svg>;
}

export function Timeline({stepNs, onStepNs, transportError, onTransportError}: {
  stepNs: string;
  onStepNs: (value: string) => void;
  transportError: string;
  onTransportError: (value: string) => void;
}) {
  const playback = usePlayback();
  const actions = useActions();
  const [exactTimestamp, setExactTimestamp] = useState('');

  const stepOk = /^[1-9]\d{0,17}$/.test(stepNs);
  const playing = playback.mode === 'REPLAY_PLAYING' ||
      playback.mode === 'LIVE_STREAMING';

  const seekExact = () => {
    if (/^\d{1,20}$/.test(exactTimestamp))
      actions.seek(exactTimestamp);
    else
      onTransportError('Enter a timestamp as integer nanoseconds');
  };

  return <footer className="timeline">
    <div className="playback">
      <button aria-label="Step backward" disabled={!stepOk}
              onClick={() => actions.step('-' + stepNs)}>▏◀</button>
      <button className="play" aria-label="Play or pause"
              onClick={() => actions.setMode(playing ? 'PAUSED' :
                                                      'REPLAY_PLAYING')}>
        {playback.mode === 'PAUSED' ? '▶' : 'Ⅱ'}
      </button>
      <button aria-label="Step forward" disabled={!stepOk}
              onClick={() => actions.step(stepNs)}>▶▕</button>
      <button className={playback.mode === 'LIVE_STREAMING' ? 'live selected' : 'live'}
              onClick={actions.live}>● LIVE</button>
    </div>
    <div className="scrub-wrap">
      <div className="scrub-labels">
        <span>{playback.mode.replaceAll('_', ' ')}</span>
        <code>{timestamp(playback.cursor_ns)}</code>
      </div>
      <ScrubTrack/>
      <div className="scrub-bounds">
        <span>{playback.bounds?.start ?? '—'}</span>
        <span>{playback.bounds?.end ?? '—'}</span>
      </div>
    </div>
    <div className="precision">
      <form onSubmit={e => {
        e.preventDefault();
        seekExact();
      }}>
        <input aria-label="Exact replay timestamp in nanoseconds"
               placeholder="Seek timestamp (ns)" value={exactTimestamp}
               onChange={e => setExactTimestamp(e.target.value)}
               inputMode="numeric"/>
        <button disabled={!playback.bounds}>Seek</button>
      </form>
      <label>
        Step (ns)
        <input aria-label="Frame step duration in nanoseconds"
               inputMode="numeric" value={stepNs}
               onChange={e => onStepNs(e.target.value)}/>
      </label>
    </div>
    <label className="file button">
      Import log
      <input aria-label="Import telemetry log" type="file" accept=".tlog"
             onChange={async e => {
               const f = e.target.files?.[0];
               if (f) actions.importLog(f);
               e.target.value = '';
             }}/>
    </label>
    <button onClick={actions.exportLog}>Export log ↗</button>
    <div className="stream-stats">
      <div><small>DROPPED / LATE</small>
        <b>{playback.stats.dropped} <span>/ {playback.stats.late}</span></b></div>
      <div><small>INVALID</small><b>{playback.stats.invalid}</b></div>
    </div>
    {transportError && <div className="alert-error">{transportError}</div>}
  </footer>;
}
