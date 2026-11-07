import {useMemo, useSyncExternalStore, type ReactNode} from 'react';
import {useChannelStale, useDictionary, usePlayback} from '../data/hooks';
import {useStudio} from '../data/studio';
import {sampleStale} from '../data/freshness';
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
  const studio = useStudio();
  const dictionary = useDictionary();
  const playback = usePlayback();
  const version = useSyncExternalStore(studio.subscribe, () => studio.store.version, () => studio.store.version);
  const stale = useMemo(() => dictionary.filter(channel => channel.name.startsWith(prefix))
      .flatMap(channel => {
        const sample = studio.store.channelSample(channel.name, playback.cursor_ns);
        return sampleStale(sample?.timestamp_ns ?? null, playback.clock_ns ?? playback.cursor_ns) ?
            [`${channel.name}: ${sample?.value ?? '—'}`] : [];
      }), [dictionary, prefix, version, playback.clock_ns, playback.cursor_ns, studio.store]);
  if (!stale.length) return null;
  return <details className="text-warn" aria-label="Stale telemetry">
    <summary>STALE · {stale.length} channels (not current)</summary>
    {stale.map(value => <div key={value}><code>{value}</code> · stale</div>)}
  </details>;
}
