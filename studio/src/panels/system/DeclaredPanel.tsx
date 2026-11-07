// Declared against actual.
//
// The live registry holds what did start, so a name's absence from it says
// nothing on its own: a node that crashed on the way up and a node that is not
// part of this session look the same from there. This panel is the difference:
// which node was supposed to be here, and then what the launcher itself said
// about the graph.

import {declaredStatus, type DeclaredGraph, type SystemGraph} from '../../system';
import {EmptyState} from '../../ui/primitives';

export function DeclaredPanel(
    {declared, graph}: {declared: DeclaredGraph; graph: SystemGraph|null}) {
  const status = declaredStatus(declared, graph);
  return <section className="panel declared">
    <div className="panel-heading">
      <h2>Declared graph</h2>
      <span>PROBED WITH --describe BEFORE ANYTHING SPAWNED</span>
    </div>
    <div className="system-counts declared-counts">
      <span>{declared.nodes.length} nodes declared</span>
      <span>{status ? `${status.registered} registered` :
                      'no registry yet — nothing to compare against'}</span>
      {!!status?.missing.length &&
       <span className="text-warn">{status.missing.length} never appeared</span>}
      {!!status?.undeclared.length &&
       <span>{status.undeclared.length} not declared</span>}
    </div>
    <div className="declared-body">
      {!!status?.missing.length && <ul className="declared-nodes">
        {status.missing.map(node => <li key={node.name} className="warn">
          <b>{node.name}</b>
          <code>{node.target || '—'}</code>
          <span>declared, never registered — it crashed at startup, or its
            binary is missing</span>
        </li>)}
      </ul>}
      {!!status?.undeclared.length && <ul className="declared-nodes">
        {status.undeclared.map(node => <li key={node.name}>
          <b>{node.name}</b>
          <code>{node.target || '—'}</code>
          <span>running but not declared — started outside the launcher</span>
        </li>)}
      </ul>}
      {declared.diagnostics.length ?
           <ul className="diagnostics">{declared.diagnostics.map(
               (diagnostic, i) => <li key={i} className={diagnostic.severity}>
                 <em>{diagnostic.severity}</em>
                 <code>{diagnostic.subject}</code>
                 <span>{diagnostic.message}</span>
               </li>)}</ul> :
           <EmptyState>
             The launcher's lint found nothing to report about this graph.
           </EmptyState>}
    </div>
  </section>;
}
