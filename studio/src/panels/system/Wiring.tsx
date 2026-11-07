// One node's wiring: what flows in, what flows out, and which node is on the
// far end of each. The question this answers is the one a topic table cannot:
// not "what topics exist" but "who is this process actually talking to".

import {drivenBy, isSubscriber, rateKey, type SystemGraph, type SystemNode, type SystemSource} from '../../system';
import {hertz, integer} from '../../ui/formatters';
import {EmptyState, StatusDot} from '../../ui/primitives';

const KIND_LABEL = {
  SENDER: 'publishes',
  WATCHER: 'watches',
  FETCHER: 'fetches',
  TIMER: 'timer'
} as const;

export function Wiring(
    {graph, node, rates}:
        {graph: SystemGraph; node: SystemNode; rates: Record<string, number>}) {
  const peersOf = (topic: string, want: 'publishers'|'subscribers') =>
      graph.topics.find(t => t.name === topic)?.[want].filter(
          n => n !== node.name) ??
      [];

  const incoming = node.sources.filter(s => isSubscriber(s.kind));
  const outgoing = node.sources.filter(s => s.kind === 'SENDER');
  const driver = drivenBy(node);

  // A missing peer is only dangling when nobody said it would be missing. The
  // node holding this end declared whether its far end lives outside talOS or
  // is allowed not to exist yet, and drawing either as a broken wire would
  // contradict what the topic table says about the same topic.
  const edge = (source: SystemSource, direction: 'in'|'out') => {
    const peers = peersOf(source.name, direction === 'in' ? 'publishers' :
                                                            'subscribers');
    const rate = rates[rateKey(node.name, source.id)];
    const declared = source.external || source.optional || source.observer;
    const kindLabel = source.observer ? 'OBSERVER' : KIND_LABEL[source.kind];
    return <li key={source.kind + source.id}
               className={peers.length || declared ? '' : 'dangling'}>
      <div className="edge-topic">
        <code>{source.name}</code>
        <span className="edge-rate">{hertz(rate)}</span>
      </div>
      <div className="edge-meta">
        <em>{kindLabel}</em>
        <span>{source.observer ? (peers.length ? peers.join(', ') : 'no publisher') :
               peers.length      ? peers.join(', ') :
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
      <StatusDot active={node.alive}/>
      <b>{node.name}</b>
      <small>{node.sources.filter(s => s.kind === 'TIMER').length} timers</small>
      {driver !== null &&
       <small className="driven-by">{`driven by ${driver}`}</small>}
    </div>
    <section>
      <h4>Publishes <small>{outgoing.length}</small></h4>
      {outgoing.length ? <ul>{outgoing.map(s => edge(s, 'out'))}</ul> :
                         <EmptyState>Publishes nothing.</EmptyState>}
    </section>
  </div>;
}
