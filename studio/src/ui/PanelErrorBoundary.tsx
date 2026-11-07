// A per-panel error boundary: a throwing panel shows its own error and a reset
// button, and every other panel keeps working. Reset clears the boundary and
// remounts the panel so a transient error recovers without reloading Studio.

import {Component, Fragment, type ReactNode} from 'react';

type Props = {children: ReactNode};
type State = {error: Error|null; generation: number};

export class PanelErrorBoundary extends Component<Props, State> {
  state: State = {error: null, generation: 0};

  static getDerivedStateFromError(error: Error): Partial<State> {
    return {error};
  }

  reset = () =>
      this.setState(s => ({error: null, generation: s.generation + 1}));

  render() {
    if (this.state.error)
      return <div className="panel-error" role="alert">
        <h3>This panel crashed</h3>
        <pre>{this.state.error.message || String(this.state.error)}</pre>
        <button onClick={this.reset}>Reset panel</button>
      </div>;
    return <Fragment key={this.state.generation}>{this.props.children}</Fragment>;
  }
}
