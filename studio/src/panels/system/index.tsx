// The system panel: every node beside one node's wiring, with every topic
// underneath. The old SystemTab reading its data through useSystem().

import {useMemo, useState} from 'react';

import {useSystem} from '../../data/hooks';
import {endpointAttributesKnown} from '../../system';
import {EmptyState} from '../../ui/primitives';
import type {PanelDefinition} from '../types';
import './system.css';
import {DeclaredPanel} from './DeclaredPanel';
import {NodeCard} from './NodeCard';
import {TopicTable} from './TopicTable';
import {Wiring} from './Wiring';

export type SystemConfig = Record<string, never>;

const defaultConfig: SystemConfig = {};
const parseConfig = (_raw: unknown): SystemConfig => ({});

function NoSystem({error}: {error: string}) {
  return <EmptyState>
    {error ? error :
             'No system graph yet. Connect to a robot bridge, or start demo data to explore the viewer.'}
  </EmptyState>;
}

function SystemPanel() {
  const system = useSystem();
  const [filter, setFilter] = useState('');
  const [picked, setPicked] = useState<string|null>(null);
  const graph = system.graph;

  const nodes = useMemo(() => {
    if (!graph) return [];
    const needle = filter.trim().toLowerCase();
    return graph.nodes.filter(
        node => !needle || node.name.toLowerCase().includes(needle) ||
            node.target.toLowerCase().includes(needle) ||
            node.sources.some(s => s.name.toLowerCase().includes(needle)));
  }, [graph, filter]);

  if (!graph) return <div className="system"><NoSystem error={system.error}/></div>;

  const selected = (picked ? nodes.find(n => n.name === picked) : null) ??
      nodes[0] ?? null;

  return <div className="system">
    <div className="system-toolbar">
      <input aria-label="Filter nodes and topics" className="search"
             placeholder="Filter nodes, targets, topics…" value={filter}
             onChange={e => setFilter(e.target.value)}/>
      <div className="system-counts">
        <span>{graph.nodes.filter(n => n.alive).length}/{graph.nodes.length} nodes live</span>
        {system.declared &&
         <span>{system.declared.nodes.length} declared</span>}
        <span>{graph.topics.length} topics</span>
        <span>slots {graph.nodes.length}/{graph.registry.node_capacity}</span>
        {!graph.registry.available &&
         <span className="text-warn">registry unavailable — no node has started</span>}
        {!endpointAttributesKnown(graph) &&
         <span className="text-warn">
           bridge sends version {graph.version} graphs, which carry no endpoint
           attributes: a topic whose far end is outside talOS reads as a fault
         </span>}
        {system.error && <span className="text-warn">{system.error}</span>}
        {system.declaredError &&
         <span className="text-warn">{system.declaredError}</span>}
      </div>
    </div>

    {system.declared &&
     <DeclaredPanel declared={system.declared} graph={graph}/>}

    <section className="panel nodes">
      <div className="panel-heading">
        <h2>Nodes</h2><span>{nodes.length} SHOWN</span>
      </div>
      <div className="node-grid">
        {nodes.map(node => <NodeCard key={node.name + node.generation}
                                     graph={graph} node={node}
                                     rates={system.rates}
                                     selected={selected?.name === node.name}
                                     onSelect={() => setPicked(node.name)}/>)}
        {!nodes.length && <EmptyState>No node matches that filter.</EmptyState>}
      </div>
    </section>

    <section className="panel wiring-panel">
      <div className="panel-heading">
        <h2>Wiring{selected && <span className="handle"> · {selected.name}</span>}</h2>
        <span>PUBLISHERS AND SUBSCRIBERS</span>
      </div>
      {selected ? <Wiring graph={graph} node={selected} rates={system.rates}/> :
                  <EmptyState>Select a node.</EmptyState>}
    </section>

    <section className="panel topics">
      <div className="panel-heading">
        <h2>Topics</h2><span>EVERY END, EVERY NODE</span>
      </div>
      <TopicTable graph={graph} rates={system.rates} filter={filter}
                  onPickNode={setPicked}/>
    </section>
  </div>;
}

export const systemPanel: PanelDefinition<SystemConfig> = {
  id: 'system',
  title: 'System',
  category: 'System',
  description: 'Nodes, wiring and topic health from the live registry.',
  defaultConfig,
  parseConfig,
  component: SystemPanel
};

export {NodeCard, Wiring, TopicTable, DeclaredPanel};
