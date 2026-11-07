// A searchable channel picker with units, used by the plot panel (and any
// panel that lets the user choose channels). Pure at render: the search and
// selection are React state.

import {useMemo, useState} from 'react';

import type {ChannelInfo} from '../data/types';

export function ChannelPicker({channels, selected, onToggle, label = 'Channels'}: {
  channels: ChannelInfo[];
  selected: string[];
  onToggle: (name: string) => void;
  label?: string;
}) {
  const [query, setQuery] = useState('');
  const needle = query.trim().toLowerCase();
  const rows = useMemo(
      () => needle ?
          channels.filter(
              c => c.name.toLowerCase().includes(needle) ||
                  c.unit.toLowerCase().includes(needle)) :
          channels,
      [channels, needle]);

  return <div className="channel-picker">
    <input aria-label={`Search ${label.toLowerCase()}`} className="search"
           placeholder={`Search ${label.toLowerCase()}…`} value={query}
           onChange={e => setQuery(e.target.value)}/>
    <div className="channel-picker-list" role="listbox" aria-label={label}>
      {rows.map(channel => <label key={channel.name} className={
                                      selected.includes(channel.name) ?
                                          'selected' :
                                          ''}>
        <input type="checkbox" checked={selected.includes(channel.name)}
               onChange={() => onToggle(channel.name)}/>
        <span className="channel-name">{channel.name}</span>
        {channel.unit && <em>{channel.unit}</em>}
      </label>)}
      {!rows.length && <span className="channel-picker-empty">
        No channels match.
      </span>}
    </div>
  </div>;
}
