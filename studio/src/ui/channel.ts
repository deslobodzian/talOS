// Channel-name helpers shared by panels that display telemetry channel names.
//
// Telemetry names a channel without its leading slash and joins nested fields
// with slashes: `<owner>/<role>/<leaf...>` (see data/discovery.ts). Two views
// of one name keep the same convention everywhere instead of each panel
// re-deriving it.

// The leaf: the last `/`-separated segment.
// `shooter/state/target_velocity_rps` -> `target_velocity_rps`.
export function channelLeaf(name: string): string {
  const slash = name.lastIndexOf('/');
  return slash === -1 ? name : name.slice(slash + 1);
}

// The display name: everything after the `owner/role` prefix.
// `shooter/state/flywheel_velocity_rps` -> `flywheel_velocity_rps`;
// `drivetrain/state/module0/distance_m` -> `module0/distance_m`.
export function channelDisplayName(name: string): string {
  const first = name.indexOf('/');
  if (first === -1) return name;
  const second = name.indexOf('/', first + 1);
  return second === -1 ? name.slice(first + 1) : name.slice(second + 1);
}
