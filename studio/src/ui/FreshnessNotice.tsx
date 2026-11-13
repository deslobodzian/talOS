import {useMemo, type ReactNode} from 'react';
import {
  useChannelStale,
  useDictionary,
  usePlayback,
  useSource,
  useSourceVersion,
  useSystem
} from '../data/hooks';
import {readingClock, sampleStale} from '../data/freshness';
import {Stat} from './primitives';

export function ChannelStat({channel, label, tone, children}: {
  channel?: string; label: string; tone?: string; children: ReactNode;
}) {
  const stale = useChannelStale(channel ?? '');
  return <Stat label={label} tone={channel && stale ? 'warn' : tone}>
    {children}{channel && stale && <em className="text-warn"> · STALE</em>}
  </Stat>;
}

export function ChannelReading({channel, children}: {channel: string; children: ReactNode}) {
  const stale = useChannelStale(channel);
  return <span className={stale ? 'text-warn' : undefined}>
    {children}{stale && <em> · STALE</em>}
  </span>;
}

// List the affected readings explicitly, rather than turning a live socket
// into a green health claim. Each channel keeps its own last sample time.
export function FreshnessNotice({prefix = ''}: {prefix?: string}) {
  const source = useSource();
  const dictionary = useDictionary();
  const playback = usePlayback();
  const version = useSourceVersion();
  const stale = useMemo(() => dictionary.filter(channel => channel.name.startsWith(prefix))
      .flatMap(channel => {
        const sample = source.channelSample(channel.name, playback.cursor_ns);
        return sampleStale(sample?.timestamp_ns ?? null, readingClock(playback)) ?
            [`${channel.name}: ${sample?.value ?? '—'}`] : [];
      }), [dictionary, prefix, version, playback.clock_ns, playback.cursor_ns, source]);
  if (!stale.length) return null;
  return <details className="text-warn" aria-label="Stale telemetry">
    <summary>STALE · {stale.length} channels (not current)</summary>
    {stale.map(value => <div key={value}><code>{value}</code> · stale</div>)}
  </details>;
}

// The system graph's counterpart: a stale graph is still shown, as last-known
// evidence, but nothing it says about which nodes are alive is current.
export function GraphFreshnessNotice() {
  const system = useSystem();
  if (!system.graph || !system.stale) return null;
  return <p className="text-warn" role="status" aria-label="Stale system graph">
    SYSTEM GRAPH STALE · node liveness unknown; showing the last-known graph
  </p>;
}
