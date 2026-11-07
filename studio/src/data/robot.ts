// Driver Station, power, link, estimator and node status derived from one
// snapshot and the current system graph. Every field is a fact read off the
// data: a channel the snapshot does not carry is null, never a guessed zero
// or false, and "not known yet" is never "nothing".

import type {Snapshot} from '../core';
import type {ChannelInfo, DriverStationMode, SystemState} from './types';
import {discoverEstimators} from './discovery';

export type RobotStatus = {
  ds: {
    mode: DriverStationMode|null;
    enabled: boolean|null;
    matchTime_s: number|null;
    fms: boolean|null;
    alliance: string|null;
  };
  power: {
    battery_v: number|null;
    brownout: boolean|null;
    can_frac: number|null;
    healthValid: boolean|null;
  };
  link: {
    stateAge_us: number|null;
    configured: boolean|null;
    faultMask: number|null;
    timedOutMask: number|null;
  };
  estimator: {valid: boolean|null; accepted: number|null; rejected: number|null}|null;
  nodes: {alive: number; total: number}|null;
};

const DS = 'hw/state/driver_station/';

// Precedence: e-stop outranks a lost link, which outranks a disabled robot,
// which outranks the chosen mode. Null when the DS channels are absent from
// the snapshot (which is a different statement from "disconnected").
export function driverStationMode(snapshot: Snapshot|null): DriverStationMode|null {
  if (!snapshot) return null;
  const ch = snapshot.channels;
  const read = (name: string) => ch[DS + name];
  const connected = read('connected'), enabled = read('enabled'),
        autonomous = read('autonomous'), teleop = read('teleop'),
        test = read('test'), estop = read('estop');
  if (connected === undefined && enabled === undefined && autonomous === undefined &&
      teleop === undefined && test === undefined && estop === undefined)
    return null;
  if (estop === 1) return 'estop';
  if (connected === 0) return 'disconnected';
  if (enabled === 0) return 'disabled';
  if (autonomous === 1) return 'autonomous';
  if (teleop === 1) return 'teleop';
  if (test === 1) return 'test';
  return null;
}

// The channel dictionary as the snapshot declares it, enough for estimator
// discovery without a second source of truth.
function snapshotChannels(snapshot: Snapshot|null): ChannelInfo[] {
  if (!snapshot) return [];
  return Object.keys(snapshot.channels).map(name => ({
    name,
    unit: snapshot.units[name] ?? '',
    labels: snapshot.labels[name] ?? []
  }));
}

// `alliance` is an enum channel: its value is an index into its labels. It is
// optional, so an absent channel or an out-of-range value is null, not a name
// guessed from nothing.
function alliance(snapshot: Snapshot|null): string|null {
  const name = DS + 'alliance';
  const value = snapshot?.channels[name];
  if (value === undefined) return null;
  const labels = snapshot?.labels[name];
  if (labels && Number.isInteger(value) && value >= 0 && value < labels.length)
    return labels[value];
  return null;
}

export function robotStatus(snapshot: Snapshot|null, system: SystemState): RobotStatus {
  const ch = snapshot?.channels ?? {};
  const num = (name: string): number|null => {
    const v = ch[name];
    return v === undefined ? null : v;
  };
  const flag = (name: string): boolean|null => {
    const v = ch[name];
    return v === undefined ? null : v !== 0;
  };

  const estimators = discoverEstimators(snapshotChannels(snapshot));
  let estimator: RobotStatus['estimator'] = null;
  if (estimators.length) {
    const owner = estimators[0].name;
    estimator = {
      valid: flag(`${owner}/state/valid`),
      accepted: num(`${owner}/state/observation_accepted_count`),
      rejected: num(`${owner}/state/observation_rejected_count`)
    };
  }

  const graph = system.graph;
  const nodes = graph ?
      {alive: graph.nodes.filter(n => n.alive).length, total: graph.nodes.length} :
      null;

  return {
    ds: {
      mode: driverStationMode(snapshot),
      enabled: flag(DS + 'enabled'),
      matchTime_s: num(DS + 'match_time_s'),
      fms: flag(DS + 'fms_attached'),
      alliance: alliance(snapshot)
    },
    power: {
      battery_v: num('hw/status/battery_v'),
      brownout: flag('hw/status/brownout'),
      can_frac: num('hw/status/can_utilization_frac'),
      healthValid: flag('hw/status/health_valid')
    },
    link: {
      stateAge_us: num('hw/status/state_age_us'),
      configured: flag('hw/status/configured'),
      faultMask: num('hw/status/motor_fault_mask'),
      timedOutMask: num('hw/status/timed_out_mask')
    },
    estimator,
    nodes
  };
}
