// Groups telemetry channels into the robots that made them: subsystems,
// cameras and estimators. Pure functions over the channel dictionary plus the
// live system graph, so panels and tests can both call them with no worker.
//
// A channel name is `<owner>/<role>/<leaf...>` (telemetry strips the leading
// slash and joins nested fields with slashes). `owner` is the first segment,
// `role` is the second (`state` or `status`). Owners in the reserved set are
// infrastructure, not subsystems, and cameras/estimators are recognised by
// their channel shape rather than by name so Studio stays robot-agnostic.

import type {SystemGraph, SystemSelection} from '../system';
import type {ChannelInfo} from './types';

export type Subsystem = {
  name: string;
  stateChannels: string[];
  statusChannels: string[];
  // The node's talos::Target selection, or null when the graph has no node of
  // this name (or the bridge predates the field).
  selection: SystemSelection|null;
};

export type Camera = {
  name: string;
  stateChannels: string[];
  statusChannels: string[];
};

export type Estimator = {name: string; channels: string[]};

const RESERVED = new Set(['hw', 'sim', 'talos', 'pose', 'derived', 'test', 'replay']);

function byOwner(channels: ChannelInfo[]): Map<string, {state: string[]; status: string[]}> {
  const owners = new Map<string, {state: string[]; status: string[]}>();
  for (const c of channels) {
    const slash = c.name.indexOf('/');
    if (slash <= 0) continue;  // no owner
    const owner = c.name.slice(0, slash);
    if (RESERVED.has(owner)) continue;
    const roleEnd = c.name.indexOf('/', slash + 1);
    const role = c.name.slice(slash + 1, roleEnd === -1 ? undefined : roleEnd);
    if (role !== 'state' && role !== 'status') continue;
    let group = owners.get(owner);
    if (!group) {
      group = {state: [], status: []};
      owners.set(owner, group);
    }
    (role === 'state' ? group.state : group.status).push(c.name);
  }
  for (const group of owners.values()) {
    group.state.sort();
    group.status.sort();
  }
  return owners;
}

// A camera is recognised by the PoseObservation shape:
// `<owner>/state/pose/x_m` + `<owner>/state/x_std_m` + `<owner>/state/tag_count`.
export function discoverCameras(channels: ChannelInfo[]): Camera[] {
  const names = new Set(channels.map(c => c.name));
  const out: Camera[] = [];
  for (const [owner, group] of byOwner(channels)) {
    if (names.has(`${owner}/state/pose/x_m`) && names.has(`${owner}/state/x_std_m`) &&
        names.has(`${owner}/state/tag_count`)) {
      out.push({name: owner, stateChannels: group.state, statusChannels: group.status});
    }
  }
  return out.sort((a, b) => a.name < b.name ? -1 : a.name > b.name ? 1 : 0);
}

// An estimator is recognised by `<owner>/state/observation_accepted_count`.
export function discoverEstimators(channels: ChannelInfo[]): Estimator[] {
  const names = new Set(channels.map(c => c.name));
  const out: Estimator[] = [];
  for (const [owner, group] of byOwner(channels)) {
    if (names.has(`${owner}/state/observation_accepted_count`)) {
      out.push({name: owner, channels: [...group.state, ...group.status].sort()});
    }
  }
  return out.sort((a, b) => a.name < b.name ? -1 : a.name > b.name ? 1 : 0);
}

export function discoverSubsystems(channels: ChannelInfo[], graph: SystemGraph|null): Subsystem[] {
  const cameras = new Set(discoverCameras(channels).map(c => c.name));
  const estimators = new Set(discoverEstimators(channels).map(e => e.name));
  const out: Subsystem[] = [];
  for (const [owner, group] of byOwner(channels)) {
    if (cameras.has(owner) || estimators.has(owner)) continue;
    const node = graph?.nodes.find(n => n.name === owner);
    out.push({
      name: owner,
      stateChannels: group.state,
      statusChannels: group.status,
      selection: node?.selections[0] ?? null
    });
  }
  return out.sort((a, b) => a.name < b.name ? -1 : a.name > b.name ? 1 : 0);
}

export type EstimateChannels = {
  x: string; y: string; yaw: string; vx: string|null; vy: string|null;
};

// The estimate pose (and, when present, velocity) channels. The estimator
// publishes the estimated pose; discover it by its accepted-observation
// counter rather than by name. Falls back to the old `/pose/x_m` suffix rule
// for recordings whose estimator predates that shape (the demo).
export function discoverEstimate(dictionary: ChannelInfo[]): EstimateChannels|null {
  const names = new Set(dictionary.map(c => c.name));
  for (const est of discoverEstimators(dictionary)) {
    const x = `${est.name}/state/pose/x_m`;
    if (!names.has(x)) continue;
    const vx = `${est.name}/state/velocity/vx_mps`;
    const vy = `${est.name}/state/velocity/vy_mps`;
    return {
      x,
      y: `${est.name}/state/pose/y_m`,
      yaw: `${est.name}/state/pose/yaw_rad`,
      vx: names.has(vx) ? vx : null,
      vy: names.has(vy) ? vy : null
    };
  }
  const x = dictionary.find(c => c.name.endsWith('/pose/x_m'));
  if (!x) return null;
  const prefix = x.name.slice(0, -'pose/x_m'.length);
  return {
    x: x.name, y: prefix + 'pose/y_m', yaw: prefix + 'pose/yaw_rad',
    vx: null, vy: null
  };
}
