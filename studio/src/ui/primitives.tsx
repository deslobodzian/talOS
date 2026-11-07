// Small shared atoms used across panels. Each is renderable by
// react-dom/server (no DOM access at render), which keeps the view tests able
// to assert on what a person reads without a browser.

import type {ReactNode} from 'react';

export function EmptyState({children}: {children: ReactNode}) {
  return <div className="empty-state">{children}</div>;
}

export function Badge({children, tone}: {children: ReactNode; tone?: string}) {
  return <em className={'ui-badge' + (tone ? ' ' + tone : '')}>{children}</em>;
}

export function StatusDot({active}: {active: boolean}) {
  return <span className={'status-dot' + (active ? ' active' : '')}/>;
}

// A label/value row in a .stat-list block. `tone` adds a state class to the
// value (e.g. 'warn', styled by the scoped .stat-list dd.warn rule).
export function Stat({label, children, tone}: {
  label: string; children: ReactNode; tone?: string;
}) {
  return <div>
    <dt>{label}</dt>
    <dd className={tone}>{children}</dd>
  </div>;
}

export function StatList({children}: {children: ReactNode}) {
  return <dl className="stat-list">{children}</dl>;
}
