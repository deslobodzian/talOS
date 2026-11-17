// The bridge sends a system graph of another version than this page reads
// (system.ts, GraphVersionMismatch): the page holds no graph, so which nodes
// run is unknown, and this says why instead of an empty view. The system,
// health and session panels show it.

import {useSystem} from '../../data/hooks';
import {graphVersionMismatch} from '../../system';

export function GraphVersionNotice() {
  const system = useSystem();
  const versions = system.graph ? null : graphVersionMismatch(system.error);
  if (!versions) return null;
  return <p className="text-warn" role="status"
            aria-label="System graph version mismatch"
            data-bridge-version={versions.bridge}
            data-page-version={versions.page}>
    SYSTEM GRAPH VERSION MISMATCH · the bridge sends version {versions.bridge},
    this page reads version {versions.page}; node liveness unknown. Load Studio
    from this bridge, or build both from one revision.
  </p>;
}
