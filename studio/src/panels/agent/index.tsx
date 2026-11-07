// The agent panel: the request an agent would send and the bytes it would get
// back, through the same dispatch the headless server uses. The old AgentTab,
// its `call` now the worker RPC from useActions().

import {useState} from 'react';

import {RPC_METHODS} from '../../../agent/rpc';
import {useActions} from '../../data/hooks';
import type {PanelDefinition} from '../types';
import './agent.css';

export type AgentConfig = Record<string, never>;

const defaultConfig: AgentConfig = {};
const parseConfig = (_raw: unknown): AgentConfig => ({});

const METHOD_HELP: Record<string, string> = {
  get_latest: 'Newest retained sample with units, labels, sequence and age.',
  get_schema_tree: 'Leaf topics observed in retained telemetry, with types.',
  query_state_at: 'A snapshot at an exact timestamp, or the one before it.',
  scan_channel_events: 'Sample ranges where a channel satisfies a condition.',
  get_system_graph: 'Every node, every topic, both ends, with rates.',
  describe_topic: 'One topic: who writes it, who reads it, how fast.',
  get_declared_graph:
      'What the launcher declared before spawning, against what registered.'
};

const METHOD_TEMPLATE: Record<string, string> = {
  get_latest: '{}',
  get_schema_tree: '{}',
  query_state_at: '{\n  "timestamp_ns": "0",\n  "mode": "at_or_before"\n}',
  scan_channel_events:
      '{\n  "topic": "velocity",\n  "condition": {"op": "gt", "value": 1}\n}',
  get_system_graph: '{}',
  describe_topic: '{\n  "topic": "/pose_estimator/state"\n}',
  get_declared_graph: '{}'
};

function AgentPanel() {
  const {rpc} = useActions();
  const [method, setMethod] = useState<string>('get_system_graph');
  const [params, setParams] = useState<string>(
      METHOD_TEMPLATE['get_system_graph']);
  const [result, setResult] = useState('');
  const [busy, setBusy] = useState(false);

  const run = async () => {
    setBusy(true);
    try {
      const parsed = params.trim() ? JSON.parse(params) : {};
      const response = await rpc(method, parsed);
      setResult(JSON.stringify(response, null, 2));
    } catch (e) {
      setResult(String(e));
    } finally {
      setBusy(false);
    }
  };

  return <div className="agent">
    <section className="panel">
      <div className="panel-heading"><h2>Query</h2><span>JSON-RPC 2.0</span></div>
      <div className="agent-form">
        <label>
          Method
          <select value={method} onChange={e => {
            setMethod(e.target.value);
            setParams(METHOD_TEMPLATE[e.target.value] ?? '{}');
          }}>
            {RPC_METHODS.map(name => <option key={name} value={name}>{name}</option>)}
          </select>
        </label>
        <small>{METHOD_HELP[method]}</small>
        <label>
          Params
          <textarea aria-label="JSON-RPC params" spellCheck={false}
                    value={params} rows={7}
                    onChange={e => setParams(e.target.value)}/>
        </label>
        <button className="primary" disabled={busy} onClick={run}>
          {busy ? 'Running…' : 'Send'}
        </button>
      </div>
      <div className="panel-footer">
        SAME CODE PATH AS <code>bazel run //studio:agent</code>
        <span>http://127.0.0.1:5804/rpc</span>
      </div>
    </section>
    <section className="panel">
      <div className="panel-heading"><h2>Response</h2><span>VERBATIM</span></div>
      <pre className="agent-result">{
          result || 'Send a request to see exactly what an agent receives.'}</pre>
    </section>
  </div>;
}

export const agentPanel: PanelDefinition<AgentConfig> = {
  id: 'agent',
  title: 'Agent',
  category: 'Tools',
  description: 'JSON-RPC queries against the same code path as the headless agent.',
  defaultConfig,
  parseConfig,
  component: AgentPanel
};
