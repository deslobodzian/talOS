// One node's wiring: what flows in, what flows out, and which node is on the
// far end of each. The question this answers is the one a topic table cannot:
// not "what topics exist" but "who is this process actually talking to".

import {nodeLiveness} from '../../data/freshness';
import {drivenByLine, isSubscriber, outputsHeld, rateKey, type SystemGraph, type SystemNode,
        type SystemSource} from '../../system';
import {hertz, integer} from '../../ui/formatters';
import {EmptyState, StatusDot} from '../../ui/primitives';

const KIND_LABEL = {
  SENDER: 'publishes',
  WATCHER: 'watches',
  FETCHER: 'fetches',
  TIMER: 'timer'
} as const;

const OWNER_PUBLISHED_ROLES = new Set(['state', 'status', 'event', 'telemetry']);
const observerReads = (topic: string): boolean =>
    OWNER_PUBLISHED_ROLES.has(topic.split('/')[2] ?? '');

export function Wiring(
    {graph, node, stale, rates}: {
      graph: SystemGraph; node: SystemNode; stale: boolean;
      rates: Record<string, number>;
    }) {
  const incoming = node.sources.filter(s => isSubscriber(s.kind));
  const outgoing = node.sources.filter(s => s.kind === 'SENDER');
  const live = nodeLiveness(node, {stale}) === 'live';
  const driver = drivenByLine(node, live);

  // A missing peer is only dangling when nobody said it would be missing. The
  // node holding this end declared whether its far end lives outside talOS or
  // is allowed not to exist yet, and drawing either as a broken wire would
  // contradict what the topic table says about the same topic.
  const edge = (source: SystemSource, direction: 'in'|'out') => {
    const topicRow = graph.topics.find(t => t.name === source.name);
    const directPeers =
        (topicRow?.[direction === 'in' ? 'publishers' : 'subscribers'] ?? [])
            .filter(n => n !== node.name);
    const observers = direction === 'out' ?
        (topicRow?.observers ?? []).filter(n => n !== node.name) : [];
    const peerLabels = [
      ...directPeers,
      ...observers.map(n => `${n} (observer)`)
    ];
    const connected = directPeers.length > 0 ||
        (direction === 'out' && observers.length > 0 && observerReads(source.name));
    const rate = rates[rateKey(node.name, source.id)];
    const declared = source.external || source.optional || source.observer;
    const kindLabel = source.observer ? 'OBSERVER' : KIND_LABEL[source.kind];
    return <li key={source.kind + source.id}
               className={connected || declared ? '' : 'dangling'}>
      <div className="edge-topic">
        <code>{source.name}</code>
        <span className="edge-rate">{hertz(rate)}</span>
      </div>
      <div className="edge-meta">
        <em>{kindLabel}</em>
        <span>{source.observer ? (peerLabels.length ? peerLabels.join(', ') : 'no publisher') :
               peerLabels.length ? peerLabels.join(', ') :
               source.external ? 'outside talOS — no shared-memory peer' :
               source.optional ? 'optional — no peer yet' :
               direction === 'in' ? 'no publisher' :
                                    'no subscriber'}</span>
        {source.dropped !== '0' &&
         <span className="text-warn">{integer(source.dropped)} dropped</span>}
      </div>
    </li>;
  };

  return <div className="wiring">
    <section>
      <h4>Subscribes <small>{incoming.length}</small></h4>
      {incoming.length ? <ul>{incoming.map(s => edge(s, 'in'))}</ul> :
                         <EmptyState>Reads nothing.</EmptyState>}
    </section>
    <div className="wiring-node">
      <StatusDot active={live}/>
      <b>{node.name}</b>
      <small>{node.sources.filter(s => s.kind === 'TIMER').length} timers</small>
      {driver !== null &&
       <small className={'driven-by' + (outputsHeld(node, live) ? ' text-warn' : '')}>
         {`driven by ${driver}`}
       </small>}
    </div>
    <section>
      <h4>Publishes <small>{outgoing.length}</small></h4>
      {outgoing.length ? <ul>{outgoing.map(s => edge(s, 'out'))}</ul> :
                         <EmptyState>Publishes nothing.</EmptyState>}
    </section>
  </div>;
}
