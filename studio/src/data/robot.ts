// Driver Station, power, link, estimator and node status derived from one
// snapshot and the current system graph. Every field is a fact read off the
// data: a channel the snapshot does not carry is null, never a guessed zero
// or false, and "not known yet" is never "nothing".

import type {Snapshot} from '../core';
import type {ChannelInfo, DriverStationMode, SystemState} from './types';
import {discoverEstimators} from './discovery';
import {liveNodeCount} from './freshness';

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
    // One row per CAN bus /hw/status lists (can_bus_count); empty when the
    // snapshot carries no count.
    canBuses: CanBus[];
    healthValid: boolean|null;
  };
  link: {
    stateAge_us: number|null;
    configured: boolean|null;
    faultMask: number|null;
    timedOutMask: number|null;
  };
  // `channel` names the validity channel, so the reading can be aged.
  estimator: {valid: boolean|null; accepted: number|null; rejected: number|null;
              channel: string}|null;
  // `alive` is null when the graph is stale: `total` is last-known, the live
  // count is not knowable.
  nodes: {alive: number|null; total: number}|null;
};

const DS = 'hw/state/driver_station/';

// How a CAN bus's last status read stands (bridge.CanBusReadState, by value).
export type CanBusReadState = 'none'|'ok'|'pending'|'failed'|'stale'|'simulated';
const CAN_BUS_READ_STATES: CanBusReadState[] =
    ['none', 'ok', 'pending', 'failed', 'stale', 'simulated'];

// A read_state channel's value as its state; null for an absent or unknown
// value.
export const canBusReadState = (value: number|undefined): CanBusReadState|null =>
    value === undefined ? null : CAN_BUS_READ_STATES[value] ?? null;

// One /hw/status can_buses slot. In any state but 'ok' /hw/status carries the
// last good read's figures (0 before one), so `frac` is the utilization only
// while the last read is current and null otherwise: a failed, stale or
// unread bus is shown by its state, never by a figure. The counters are the
// last good read's.
export type CanBus = {
  index: number;
  label: string;
  state: CanBusReadState|null;
  frac: number|null;
  busOff: number|null;
  transmitBufferFull: number|null;
  receiveErrors: number|null;
  transmitErrors: number|null;
  readFailures: number|null;
  channel: string;  // the slot's read_state channel
};

// Slot 0 is the RoboRIO's own bus; the others are the configuration's
// further buses in byte order. When the declared graph carries the slot names
// (hardware::CanBuses), use them; otherwise fall back to "rio" and "CAN bus N".
export const canBusLabel = (
    index: number, busNames?: readonly string[]): string => {
  const configured = busNames?.[index];
  return configured ? configured : index === 0 ? 'rio' : `CAN bus ${index}`;
};

export function canBuses(
    channels: Record<string, number>,
    busNames?: readonly string[]): CanBus[] {
  const count = channels['hw/status/can_bus_count'];
  if (count === undefined || !Number.isFinite(count)) return [];
  const out: CanBus[] = [];
  for (let index = 0; index < Math.min(count, 4); index++) {
    const prefix = `hw/status/can_buses/${index}/`;
    const read = (field: string): number|null => {
      const v = channels[prefix + field];
      return v === undefined ? null : v;
    };
    const state = canBusReadState(channels[prefix + 'read_state']);
    out.push({
      index,
      label: canBusLabel(index, busNames),
      state,
      frac: state === 'ok' ? read('utilization_frac') : null,
      busOff: read('bus_off_count'),
      transmitBufferFull: read('transmit_buffer_full_count'),
      receiveErrors: read('receive_error_count'),
      transmitErrors: read('transmit_error_count'),
      readFailures: read('read_failure_count'),
      channel: prefix + 'read_state'
    });
  }
  return out;
}

// What a bus's utilization reads as: a percentage only while its last read is
// current ('ok'); otherwise the state itself ('not read' before any report),
// so a bus that stopped or never started reading never looks idle.
export function canBusReading(bus: CanBus): string {
  if (bus.state === null) return '—';
  if (bus.state === 'ok')
    return bus.frac === null ? '—' : `${(bus.frac * 100).toFixed(0)}%`;
  if (bus.state === 'none') return 'not read';
  return bus.state;
}

export const canBusUnread = (bus: CanBus): boolean =>
    bus.state === 'failed' || bus.state === 'stale';

// Precedence: e-stop outranks a lost link, which outranks a disabled robot,
// which outranks the chosen mode. Null when the DS channels are absent from
// the snapshot (which is a different statement from "disconnected"), and when
// `enabled` is: a mode flag names the selected mode, not that the robot is
// enabled in it.
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
  if (enabled === undefined) return null;
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
      rejected: num(`${owner}/state/observation_rejected_count`),
      channel: `${owner}/state/valid`
    };
  }

  const graph = system.graph;
  const nodes = graph ? {alive: liveNodeCount(system), total: graph.nodes.length} : null;

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
      canBuses: canBuses(ch, system.declared?.can_buses),
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
