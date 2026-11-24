// The placeholder for a panel whose type is not in the registry -- an older
// plugin was removed, or a workspace was hand-edited to a panel id Studio does
// not know. Its config is preserved verbatim (shown, not thrown away), so a
// later upgrade that re-registers the type picks the config straight back up.

function formatConfig(config: unknown): string|null {
  if (config === undefined || config === null) return null;
  try {
    const compact = JSON.stringify(config);
    if (compact === undefined || compact === '{}') return null;
    return JSON.stringify(config, null, 2);
  } catch {
    return String(config);
  }
}

export function UnknownPanel({panelId, config}: {
  panelId: string;
  config: unknown;
}) {
  const formatted = formatConfig(config);
  return <div className="unknown-panel">
    <h3>Unknown panel</h3>
    <p><code>{panelId}</code> is not a registered panel type.</p>
    {formatted !== null ?
         <>
           <p>Its configuration is preserved for when the panel is available
             again:</p>
           <pre className="agent-result">{formatted}</pre>
         </> :
         <p>No configuration to preserve.</p>}
  </div>;
}
