// A minimal typed data table. The system, timing and inspector panels share
// its header/row plumbing; each supplies its own columns and row key. Nothing
// here touches the DOM, so react-dom/server can render it.

import type {ReactNode} from 'react';

export type Column<T> = {
  key: string;
  header: ReactNode;
  right?: boolean;
  sortable?: boolean;
  sorted?: boolean;
  descending?: boolean;
  onSort?: () => void;
  render: (row: T) => ReactNode;
};

export function DataTable<T>({columns, rows, rowKey, empty}: {
  columns: Column<T>[];
  rows: T[];
  rowKey: (row: T) => string;
  empty?: ReactNode;
}) {
  return <div className="table-scroll">
    <table className="grid">
      <thead>
        <tr>
          {columns.map(col => <th key={col.key}
              className={(col.right ? 'right ' : '') +
                         (col.sortable ? 'sortable' : '') +
                         (col.sorted ? ' sorted' : '')}
              onClick={col.sortable ? col.onSort : undefined}>
            {col.header}
            {col.sorted && <i>{col.descending ? '▾' : '▴'}</i>}
          </th>)}
        </tr>
      </thead>
      <tbody>
        {rows.map(row => <tr key={rowKey(row)}>
          {columns.map(col => <td key={col.key}
              className={col.right ? 'right' : ''}>{col.render(row)}</td>)}
        </tr>)}
      </tbody>
    </table>
    {!rows.length && empty != null && empty}
  </div>;
}
