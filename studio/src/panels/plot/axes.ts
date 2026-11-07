// Pure helpers that decide how the selected channels are grouped onto the
// plot's at-most-two y-axes (left/right). Kept pure and exported so the unit
// tests can pin the policy without a browser or a worker.
//
// Policy:
//   * enum channels with identical label lists own the left axis and render
//     as steps with those labels as ticks; conflicting lists use numeric axes.
//   * numeric channels are grouped by unit; with no enums the first unit takes
//     the left axis and every other unit shares the right axis; with enums all
//     numeric channels share the right axis.
//   * so there are never more than two axes, and each axis is labelled with the
//     unit(s) it carries.

export type PlotChannel = {
  name: string;    // full channel name, e.g. "drivetrain/state/vx_mps"
  unit: string;    // declared unit; '' when none
  labels: string[];  // non-empty for an enum channel
};

export type AxisGroup = {
  unit: string;       // '' when the channel declares none
  channels: string[]; // first-seen order
};

export type AxisAssignment = {
  scale: 'y' | 'y2';
  side: 'left' | 'right';
  label: string;           // the unit(s) carried, '' when unitless
  units: string[];
  labels: string[] | null; // enum tick labels, or null for a numeric axis
  channels: string[];
};

// Group channel names by unit, preserving first-seen channel order and sorting
// the groups so a named unit owns the left axis (unitless last).
export function groupByUnit(
    channels: string[], unitOf: (name: string) => string): AxisGroup[] {
  const order: string[] = [];
  const byUnit = new Map<string, string[]>();
  for (const name of channels) {
    const unit = unitOf(name) ?? '';
    const group = byUnit.get(unit);
    if (group) group.push(name);
    else {
      byUnit.set(unit, [name]);
      order.push(unit);
    }
  }
  return order
      .sort((a, b) => a === '' ? 1 : b === '' ? -1 : a < b ? -1 : a > b ? 1 : 0)
      .map(unit => ({unit, channels: byUnit.get(unit)!}));
}

function numericAxis(scale: 'y'|'y2', side: 'left'|'right',
                     groups: AxisGroup[]): AxisAssignment {
  const units = groups.map(g => g.unit);
  const named = units.filter(u => u !== '');
  return {
    scale,
    side,
    label: named.join(' / '),
    units,
    labels: null,
    channels: groups.flatMap(g => g.channels)
  };
}

export function assignYAxes(channels: PlotChannel[]): AxisAssignment[] {
  // Conflicting enum vocabularies cannot honestly share labelled ticks. Keep
  // the two-axis policy by displaying their values on ordinary numeric axes.
  const declaredEnums = channels.filter(c => c.labels.length > 0);
  const compatible = declaredEnums.every(c =>
    c.labels.length === declaredEnums[0].labels.length &&
    c.labels.every((label, i) => label === declaredEnums[0].labels[i]));
  const enums = compatible ? declaredEnums : [];
  const numerics = compatible ? channels.filter(c => c.labels.length === 0) : channels;
  const groups = groupByUnit(
      numerics.map(c => c.name),
      name => numerics.find(c => c.name === name)!.unit);

  const axes: AxisAssignment[] = [];

  if (enums.length) {
    // Compatible enums own the left axis and share their declared ticks.
    axes.push({
      scale: 'y',
      side: 'left',
      label: enums[0].unit,
      units: [...new Set(enums.map(c => c.unit))],
      labels: enums[0].labels,
      channels: enums.map(c => c.name)
    });
    if (groups.length) axes.push(numericAxis('y2', 'right', groups));
    return axes;
  }

  if (groups.length === 0) return axes;
  axes.push(numericAxis('y', 'left', [groups[0]]));
  if (groups.length > 1)
    axes.push(numericAxis('y2', 'right', groups.slice(1)));
  return axes;
}
