// Every topic in the graph, classified by health, with the rate of whoever
// publishes it and the counters a person checks for loss.

import {useMemo, useState} from 'react';

import {
  rateKey,
  type SystemGraph,
  type SystemTopic,
  TOPIC_HEALTH,
  topicHealthOf,
  type TopicHealth,
} from '../../system';
import {hertz, integer} from '../../ui/formatters';
import {EmptyState} from '../../ui/primitives';

const HEALTH_NOTE: Record<TopicHealth, string> = {
  orphaned: 'subscribed but nothing publishes it',
  unread: 'published but nobody reads it',
  lossy: 'messages were lapped before a reader saw them',
  idle: 'wired up, nothing published yet',
  ok: 'publishing and being read',
  bridged: 'one end is outside talOS; no shared-memory peer will appear',
  unconnected: 'declared optional; no peer yet, and that is the design'
};

export function TopicTable(
    {graph, rates, filter, onPickNode}: {
      graph: SystemGraph; rates: Record<string, number>; filter: string;
      onPickNode: (name: string) => void;
    }) {
  const [chosen, setChosen] = useState<TopicHealth|'all'>('all');

  // Endpoint attributes come from the ends, not from the topic row, so the
  // classifier gathers them once per graph rather than per row.
  const health = useMemo(() => topicHealthOf(graph), [graph]);
  const rows = useMemo(() => {
    const needle = filter.trim().toLowerCase();
    return graph.topics
        .map(topic => ({topic, health: health(topic)}))
        .filter(
            row => (chosen === 'all' || row.health === chosen) &&
                (!needle || row.topic.name.toLowerCase().includes(needle) ||
                 row.topic.publishers.some(
                     p => p.toLowerCase().includes(needle)) ||
                 row.topic.subscribers.some(
                     s => s.toLowerCase().includes(needle)) ||
                 row.topic.observers.some(
                     s => s.toLowerCase().includes(needle))));
  }, [graph.topics, health, filter, chosen]);

  const counts = useMemo(() => {
    const tally = new Map<TopicHealth, number>();
    for (const topic of graph.topics) {
      const state = health(topic);
      tally.set(state, (tally.get(state) ?? 0) + 1);
    }
    return tally;
  }, [graph.topics, health]);

  // Publisher-side rate: a topic moves at the rate of whoever writes it, and
  // summing both ends would double-count every healthy topic.
  const topicRate = (topic: SystemTopic) => {
    let total = 0;
    let measured = false;
    for (const node of graph.nodes) {
      for (const source of node.sources) {
        if (source.name !== topic.name || source.kind !== 'SENDER') continue;
        const rate = rates[rateKey(node.name, source.id)];
        if (rate !== undefined) {
          total += rate;
          measured = true;
        }
      }
    }
    return measured ? total : undefined;
  };

  return <>
    <div className="chips">
      {(['all', ...TOPIC_HEALTH] as const)
           .map(state => <button
                    key={state}
                    className={'chip' + (chosen === state ? ' selected' : '') +
                               (state === 'all' ? '' : ' ' + state)}
                    onClick={() => setChosen(state)}>
                 {state}
                 <em>{state === 'all' ? graph.topics.length :
                                        counts.get(state) ?? 0}</em>
               </button>)}
    </div>
    <div className="table-scroll">
      <table className="grid">
        <thead>
          <tr>
            <th>Topic</th>
            <th>Bytes</th>
            <th>Publishers</th>
            <th>Subscribers</th>
            <th className="right">Rate</th>
            <th className="right">Published</th>
            <th className="right">Received</th>
            <th className="right">Dropped</th>
          </tr>
        </thead>
        <tbody>
          {rows.map(({topic, health: state}) => <tr key={topic.name}
                                                    className={state}>
             <td>
               <code>{topic.name}</code>
               <span className={'health ' + state} title={HEALTH_NOTE[state]}>
                 {state}
               </span>
             </td>
             <td>{topic.message_bytes}</td>
             <td>{topic.publishers.length ?
                      topic.publishers.map(
                          name => <button key={name} className="link"
                                          onClick={() => onPickNode(name)}>
                            {name}
                          </button>) :
                      <span className="text-warn">none</span>}</td>
             <td>{topic.subscribers.length || topic.observers.length ?
                      <>
                        {topic.subscribers.map(
                            name => <button key={name} className="link"
                                            onClick={() => onPickNode(name)}>
                              {name}
                            </button>)}
                        {topic.observers.map(
                            name => <button key={`observer:${name}`} className="link"
                                            onClick={() => onPickNode(name)}>
                              {name} (observer)
                            </button>)}
                      </> :
                      <span className="text-warn">none</span>}</td>
             <td className="right">{hertz(topicRate(topic))}</td>
             <td className="right">{integer(topic.published)}</td>
             <td className="right">{integer(topic.received)}</td>
             <td className={'right' + (topic.dropped === '0' ? '' : ' text-warn')}>
               {integer(topic.dropped)}
             </td>
           </tr>)}
        </tbody>
      </table>
      {!rows.length && <EmptyState>No topic matches that filter.</EmptyState>}
    </div>
  </>;
}
