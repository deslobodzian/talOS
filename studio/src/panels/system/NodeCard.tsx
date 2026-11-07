// A single node card: liveness, cadence, dispatch count, and who drives its
// subsystem. Pure at render (data arrives as props), so react-dom/server can
// render it in tests.

import {drivenBy, isSubscriber, rateKey, type SystemGraph, type SystemNode} from '../../system';
import {duration, hertz, integer, staleness} from '../../ui/formatters';
import {Badge, StatusDot} from '../../ui/primitives';

export function NodeCard(
    {graph, node, rates, selected, onSelect}: {
      graph: SystemGraph; node: SystemNode; rates: Record<string, number>;
      selected: boolean;
      onSelect: () => void;
    }) {
  const publishes = node.sources.filter(s => s.kind === 'SENDER');
  const subscribes = node.sources.filter(s => isSubscriber(s.kind));
  const timers = node.sources.filter(s => s.kind === 'TIMER');

  // The node's own cadence. A loop node's timers are the honest answer; the
  // hardware node has none -- it drives its own tick -- so fall back to the
  // fastest thing it publishes rather than reporting no rate for the one
  // process whose rate matters most.
  const busiest = (timers.length ? timers : node.sources)
                      .map(s => rates[rateKey(node.name, s.id)])
                      .filter((r): r is number => r !== undefined)
                      .sort((a, b) => b - a)[0];
  const driver = drivenBy(node);

  return <button
      className={'node-card' + (selected ? ' selected' : '') +
                 (node.alive ? '' : ' dead')}
      onClick={onSelect} aria-pressed={selected}>
    <div className="node-card-head">
      <StatusDot active={node.alive}/>
      <b>{node.name}</b>
      {node.simulation && <Badge tone="sim">SIM</Badge>}
      {node.replay && <Badge>REPLAY</Badge>}
      {!node.alive && <Badge tone="dead">STALE</Badge>}
    </div>
    <code>{node.target || '—'}</code>
    <dl>
      <div><dt>pid</dt><dd>{node.pid}</dd></div>
      <div><dt>pub / sub</dt><dd>{publishes.length} / {subscribes.length}</dd></div>
      <div><dt>dispatches</dt><dd>{integer(node.dispatch_count)}</dd></div>
      <div><dt>loop</dt><dd>{hertz(busiest)}</dd></div>
      <div><dt>seen</dt><dd>{
          staleness(graph, node) <= 0 ? 'now' :
                                        duration(staleness(graph, node)) + ' ago'
      }</dd></div>
      {driver !== null &&
       <div className="driven-by"><dt>driven by</dt><dd className={
           node.selections.some(s => s.selected !== null) ? 'driving' : 'idle'
       }>{driver}</dd></div>}
    </dl>
    {node.declared_sources > node.sources.length &&
     <small className="warn">
       {node.declared_sources - node.sources.length} more sources than the
       registry slot holds
     </small>}
  </button>;
}
