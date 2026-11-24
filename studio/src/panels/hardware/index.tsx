// The hardware panel: everything about the main-processor <-> RoboRIO link and
// power, read off /hw/status. Four views over the same snapshot:
//
//   Power   — battery voltage with brownout bands, and each CAN bus's
//             utilization, read state and error counters.
//   Link    — state age and the actuator-owning subsystem timeouts.
//   Clock   — the gateway<->node clock mapping.
//   Counters and the motor fault mask — the bridge and gateway's tallies,
//             total and rate, with a plain-English note per counter.
//
// Every channel is discovered, never assumed: an absent channel renders as
// "—" (no data), and a counter that was never declared is not listed.

import {useMemo} from 'react';

import {useDictionary, useRecorded, useSeries, useSnapshot, useSystem} from '../../data/hooks';
import {canBuses, canBusReading, canBusUnread, type CanBus} from '../../data/robot';
import {durationUs, integer} from '../../ui/formatters';
import {DataTable, type Column} from '../../ui/DataTable';
import {EmptyState, StatList} from '../../ui/primitives';
import {TimeSeriesChart} from '../../ui/TimeSeriesChart';
import type {PanelDefinition} from '../types';
import {
  bitHistory,
  canBusLines,
  canBusSeriesChannels,
  counterRate,
  flagBands,
  type BitHistory
} from './logic';
import {ChannelReading, ChannelStat as Stat, FreshnessNotice} from '../../ui/FreshnessNotice';
import './hardware.css';

export type HardwareConfig = Record<string, never>;

const defaultConfig: HardwareConfig = {};
const parseConfig = (_raw: unknown): HardwareConfig => ({});

const HW = 'hw/status/';
const EMPTY = new Float64Array(0);

// The counter tallies the bridge and gateway publish on /hw/status, in the
// order a person cares about them. Explanations come from the bridge_status
// FlatBuffer schema and the gateway/merge comments (talOS/bridge/node.cc,
// talOS/hardware/gateway.cc); each is a fact read off the data, not an
// inference.
type CounterSpec = {name: string; label: string; note: string};

const COUNTERS: CounterSpec[] = [
  {
    name: 'states_received_count',
    label: 'States received',
    note: 'RoboRIO→bridge /hw/state frames decoded since the bridge started.'
  },
  {
    name: 'commands_sent_count',
    label: 'Commands sent',
    note: 'bridge→RoboRIO /hw/command frames sent since the bridge started.'
  },
  {
    name: 'identity_rejected_count',
    label: 'Identity rejected',
    note: 'Subsystem requests dropped because their config/boot/epoch did not ' +
        'match the live gateway — an obsolete request never merges.'
  },
  {
    name: 'ownership_rejected_count',
    label: 'Ownership rejected',
    note: 'Subsystem requests dropped because they addressed devices the ' +
        'subsystem does not own.'
  },
  {
    name: 'driver_station_packet_count',
    label: 'DS packets',
    note: 'Driver Station packets relayed from the RoboRIO to the nodes.'
  },
  {
    name: 'gateway_rejected_command_count',
    label: 'Gateway rejected',
    note: 'Commands the gateway refused: structural or stale, sent while ' +
        'it is faulted or unhealthy, or asking for output while the robot ' +
        'is not enabled, configured or commissioned. Current neutral ' +
        'commands are not counted while the robot is not enabled, ' +
        'configured or commissioned.'
  },
  {
    name: 'gateway_neutralized_motor_count',
    label: 'Gateway neutralized',
    note: 'Motors neutralized within an otherwise-accepted command for range ' +
        'or feedback errors.'
  },
  {
    name: 'gateway_apply_failure_count',
    label: 'Apply failures',
    note: 'Backend apply failures (total write loss); three in a row latches ' +
        'a fault.'
  },
  {
    name: 'gateway_device_reset_count',
    label: 'Device resets',
    note: 'Device resets detected since the bridge started; the gateway ' +
        'neutralizes and re-configures after each.'
  },
  {
    name: 'socket_send_failures_count',
    label: 'Socket send failures',
    note: 'Failed UDP sends on the bridge→RoboRIO socket since the bridge ' +
        'started.'
  },
  {
    name: 'socket_reopens_count',
    label: 'Socket reopens',
    note: 'Times the bridge closed and reopened its UDP socket after a ' +
        'permanent send or receive error.'
  },
  {
    name: 'state_sequence_gaps_count',
    label: 'State sequence gaps',
    note: 'Missed UDP frame sequence numbers detected on incoming gateway ' +
        'datagrams.'
  },
  {
    name: 'state_decode_failures_count',
    label: 'State decode failures',
    note: 'Incoming UDP frames that failed frame decoding (corrupt or ' +
        'mismatched protocol).'
  },
  {
    name: 'socket_send_would_block_count',
    label: 'Socket send would-block',
    note: 'Outbound UDP sends dropped because the nonblocking socket send ' +
        'buffer was full.'
  }
];

type CounterRow = CounterSpec & {total: string; rate: string};

const rateString = (rate: number|null): string =>
    rate === null || !Number.isFinite(rate) ? '—' : `${rate >= 100 ? rate.toFixed(0) : rate.toFixed(1)}/s`;

const volts = (v: number|undefined) =>
    v === undefined || !Number.isFinite(v) ? '—' : `${v.toFixed(2)} V`;
const micros = (us: number|undefined) =>
    us === undefined || !Number.isFinite(us) ? '—' : durationUs(us);
// A uint64 counter, with the absent case shown as "no data".
const count = (value: number|undefined) =>
    value === undefined || !Number.isFinite(value) ? '—' : integer(value);

// When a bit last rose, as a human offset from the end of the retained view.
const ago = (lastSetAt: number|null, endSec: number): string => {
  if (lastSetAt === null || !Number.isFinite(lastSetAt) || !Number.isFinite(endSec))
    return '—';
  return `${Math.max(0, endSec - lastSetAt).toFixed(1)}s ago`;
};

function BitList({title, bits, prefix, empty, endSec, channel}: {
  title: string; channel: string;
  bits: BitHistory[];
  prefix: string;
  empty: string;
  endSec: number;
}) {
  const meaningful = bits.filter(b => b.set || b.setCount > 0);
  return <div className="hw-block">
    <h3><ChannelReading channel={channel}>{title}</ChannelReading></h3>
    {meaningful.length ?
        <ul className="bit-list">
          {meaningful.map(b =>
            <li key={b.bit} className={b.set ? 'set' : 'clear'}>
              <span className="bit-name">{prefix} {b.bit}</span>
              <span className="bit-state">{b.set ? 'set' : 'clear'}</span>
              <span className="bit-last">
                {b.set && b.lastSetAt === null ?
                    'set since before view' :
                    `last set ${ago(b.lastSetAt, endSec)}`}
                {b.setCount > 1 ? ` · ${b.setCount}×` : ''}
              </span>
            </li>)}
        </ul> :
        <EmptyState>{bits.some(b => b.set !== null) ? empty : '—'}</EmptyState>}
  </div>;
}

const BUS_COLORS = ['var(--info)', 'var(--accent)', 'var(--warn)', 'var(--ok)'];

const counter = (value: number|null) => value === null ? '—' : integer(value);

// One row per CAN bus /hw/status lists: its utilization while its last read
// is current, else its read state (a failed or stale bus is never a "0%"),
// and its controller's error counters and the failed reads since the
// configuration.
function CanBusTable({buses}: {buses: CanBus[]}) {
  const columns: Column<CanBus>[] = [
    {key: 'label', header: 'CAN bus', render: bus => <b>{bus.label}</b>},
    {key: 'state', header: 'Utilization', right: true, render: bus =>
      <ChannelReading channel={bus.channel}>
        <span className={canBusUnread(bus) ? 'can-unread' : undefined}>{canBusReading(bus)}</span>
      </ChannelReading>},
    {key: 'busOff', header: 'Bus off', right: true, render: bus => counter(bus.busOff)},
    {key: 'transmitBufferFull', header: 'TX full', right: true,
     render: bus => counter(bus.transmitBufferFull)},
    {key: 'receiveErrors', header: 'REC', right: true, render: bus => counter(bus.receiveErrors)},
    {key: 'transmitErrors', header: 'TEC', right: true, render: bus => counter(bus.transmitErrors)},
    {key: 'readFailures', header: 'Failed reads', right: true,
     render: bus => counter(bus.readFailures)}
  ];
  return <div className="hw-block">
    <h3>CAN buses</h3>
    {buses.length ?
        <DataTable rows={buses} columns={columns} rowKey={bus => String(bus.index)}/> :
        <EmptyState>No CAN bus reported.</EmptyState>}
  </div>;
}

function HardwarePanel() {
  const snapshot = useSnapshot();
  const dictionary = useDictionary();
  const system = useSystem();
  const recorded = useRecorded();
  const busNames = recorded ? recorded.declared?.can_buses : system.declared?.can_buses;

  const ch = snapshot?.channels ?? {};
  const present = useMemo(() => new Set(dictionary.map(c => c.name)), [dictionary]);
  const num = (name: string): number|undefined => {
    const v = ch[`${HW}${name}`];
    return v === undefined || !Number.isFinite(v) ? undefined : v;
  };
  const flag = (name: string): boolean|undefined => {
    const v = num(name);
    return v === undefined || !Number.isFinite(v) ? undefined : v !== 0;
  };

  const buses = canBuses(ch, busNames);
  const busCount = num('can_bus_count');
  const busChannels = useMemo(() => canBusSeriesChannels(busCount), [busCount]);
  const powerChannels = useMemo(
      () => [`${HW}battery_v`, `${HW}brownout`, ...busChannels]
          .filter(name => present.has(name)),
      [busChannels, present]);
  const power = useSeries(powerChannels, {seconds: 120});
  const ageChannels = useMemo(
      () => [`${HW}state_age_us`].filter(name => present.has(name)),
      [present]);
  const age = useSeries(ageChannels, {seconds: 120});
  const maskChannels = useMemo(
      () => [`${HW}timed_out_mask`, `${HW}motor_fault_mask`,
             `${HW}motor_forward_limit_mask`, `${HW}motor_reverse_limit_mask`]
          .filter(name => present.has(name)),
      [present]);
  const masks = useSeries(maskChannels, 'all');

  const counterNames = useMemo(
      () => COUNTERS.filter(c => present.has(`${HW}${c.name}`))
          .map(c => `${HW}${c.name}`),
      [present]);
  const counterSeries = useSeries(counterNames, {seconds: 10});

  const endSec = masks.times.length ? masks.times[masks.times.length - 1] : 0;

  const batteryV = num('battery_v');
  const brownout = flag('brownout');
  const healthValid = flag('health_valid');
  const stateAgeUs = num('state_age_us');
  const configured = flag('configured');
  const subsystemCount = num('subsystem_count');
  const protocolVersion = num('protocol_version_id');
  const mismatchedProtocol = num('mismatched_protocol_version_id');
  const protocolMismatch = flag('alert_error_protocol_mismatch') === true ||
      (mismatchedProtocol !== undefined && Number.isFinite(mismatchedProtocol) &&
       mismatchedProtocol !== 0);
  const timeoutUs = num('command_timeout_us');  // not published by the 2026 bridge

  const counterRows: CounterRow[] = useMemo(() => {
    const out: CounterRow[] = [];
    for (const spec of COUNTERS) {
      if (!present.has(`${HW}${spec.name}`)) continue;
      out.push({
        ...spec,
        total: count(ch[`${HW}${spec.name}`]),
        rate: rateString(counterRate(
            counterSeries.times,
            counterSeries.values[`${HW}${spec.name}`] ?? EMPTY))
      });
    }
    return out;
  }, [present, ch, counterSeries]);

  const columns: Column<CounterRow>[] = [
    {key: 'label', header: 'Counter', render: row =>
      <div className="counter-cell"><b>{row.label}</b><span>{row.note}</span></div>},
    {key: 'total', header: 'Total', right: true, render: row => <ChannelReading channel={`${HW}${row.name}`}>{row.total}</ChannelReading>},
    {key: 'rate', header: 'Rate (10 s)', right: true, render: row => row.rate}
  ];

  const anyHardware = present.has(`${HW}battery_v`) ||
      present.has(`${HW}state_age_us`) || counterRows.length > 0 ||
      present.has(`${HW}clock_offset_us`) ||
      present.has(`${HW}protocol_version_id`) ||
      present.has(`${HW}mismatched_protocol_version_id`);

  if (!anyHardware) {
    return <div className="hardware">
      <EmptyState>
        {recorded?.bounds.hardware_not_decoded ?
            recorded.bounds.hardware_not_decoded :
         recorded ?
            'No /hw/status telemetry in this recorded session.' :
            'No /hw/status telemetry. Connect to a robot bridge, or start demo data to explore the link and power views.'}
      </EmptyState>
    </div>;
  }

  const brownoutBands =
      flagBands(power.times, power.values[`${HW}brownout`] ?? EMPTY);
  const busLines = canBusLines(busCount, power, busNames);
  const timedOutBits = bitHistory(
      masks.times, masks.values[`${HW}timed_out_mask`] ?? EMPTY,
      subsystemCount ?? 32);
  const faultBits = bitHistory(
      masks.times, masks.values[`${HW}motor_fault_mask`] ?? EMPTY);
  const forwardLimitBits = bitHistory(
      masks.times, masks.values[`${HW}motor_forward_limit_mask`] ?? EMPTY);
  const reverseLimitBits = bitHistory(
      masks.times, masks.values[`${HW}motor_reverse_limit_mask`] ?? EMPTY);

  return <div className="hardware">
    <FreshnessNotice prefix={HW}/>
    <section className="card">
      <div className="card-heading"><h2>Power</h2><span>HW/STATUS</span></div>
      <StatList>
        <Stat label="Battery" channel={`${HW}battery_v`}>{volts(batteryV)}</Stat>
        <Stat label="Brownout" channel={`${HW}brownout`} tone={brownout ? 'warn' : undefined}>
          {brownout === undefined ? '—' : brownout ? 'yes' : 'no'}
        </Stat>
        <Stat label="Health valid" channel={`${HW}health_valid`} tone={healthValid === false ? 'warn' : undefined}>
          {healthValid === undefined ? '—' : healthValid ? 'yes' : 'no'}
        </Stat>
      </StatList>
      <div className="hw-block">
        <h3>Battery voltage · 2 min</h3>
        <TimeSeriesChart
            lines={[{name: 'battery_v', color: 'var(--accent)',
                     times: power.times,
                     values: power.values[`${HW}battery_v`] ?? EMPTY}]}
            bands={brownoutBands} height={96} legend={false}/>
        <p className="chart-note">Shaded spans are brownout ({
            brownout === undefined ? 'unknown' : brownout ? 'active' : 'clear'
          }).</p>
      </div>
      <CanBusTable buses={buses}/>
      {busLines.length > 0 &&
        <div className="hw-block">
          <h3>CAN utilization · 2 min</h3>
          <TimeSeriesChart
              lines={busLines.map(line => ({
                name: line.name,
                color: BUS_COLORS[line.index % BUS_COLORS.length],
                times: power.times, values: line.values}))}
              height={72} legend={busLines.length > 1}/>
          <p className="chart-note">A bus's line breaks wherever its read was not current (failed, stale or not yet read); its row above says which.</p>
        </div>}
    </section>

    <section className="card">
      <div className="card-heading"><h2>Link</h2><span>HW/STATUS</span></div>
      <StatList>
        <Stat label="Config acknowledged" channel={`${HW}configured`} tone={configured === false ? 'warn' : undefined}>
          {configured === undefined ? '—' : configured ? 'yes' : 'no'}
        </Stat>
        <Stat label="Subsystems" channel={`${HW}subsystem_count`}>
          {subsystemCount === undefined ? '—' : integer(subsystemCount)}
        </Stat>
        <Stat label="Protocol"
              channel={protocolMismatch && present.has(`${HW}mismatched_protocol_version_id`) ?
                  `${HW}mismatched_protocol_version_id` : `${HW}protocol_version_id`}
              tone={protocolMismatch ? 'warn' : undefined}>
          {protocolMismatch ?
              (mismatchedProtocol !== undefined && Number.isFinite(mismatchedProtocol) &&
               mismatchedProtocol !== 0 ?
                  `mismatch (expected ${count(protocolVersion)}, gateway ${integer(mismatchedProtocol)})` :
                  `mismatch (expected ${count(protocolVersion)})`) :
              count(protocolVersion)}
        </Stat>
        <Stat label="State age" channel={`${HW}state_age_us`} tone={stateAgeUs !== undefined && stateAgeUs > 20000 ? 'warn' : undefined}>
          {micros(stateAgeUs)}
        </Stat>
      </StatList>
      <div className="hw-block">
        <h3>State age · 2 min</h3>
        <TimeSeriesChart
            lines={[{name: 'state_age_us', color: 'var(--accent)',
                     times: age.times,
                     values: age.values[`${HW}state_age_us`] ?? EMPTY}]}
            height={96} threshold={timeoutUs} legend={false}/>
        <p className="chart-note">
          {timeoutUs === undefined ?
              'Command timeout not reported by this bridge.' :
              `Command timeout ${micros(timeoutUs)}.`}
        </p>
      </div>
      <BitList title="Subsystem timeouts (timed_out_mask)" channel={`${HW}timed_out_mask`}
               bits={timedOutBits} prefix="subsystem"
               empty="No subsystem has timed out."
               endSec={endSec}/>
    </section>

    <section className="card">
      <div className="card-heading"><h2>Clock</h2><span>GATEWAY ↔ NODE</span></div>
      <StatList>
        <Stat label="Offset" channel={`${HW}clock_offset_us`}>{micros(num('clock_offset_us'))}</Stat>
        <Stat label="Spread" channel={`${HW}clock_offset_spread_us`}>{micros(num('clock_offset_spread_us'))}</Stat>
        <Stat label="Valid" channel={`${HW}clock_offset_valid`} tone={flag('clock_offset_valid') === false ? 'warn' : undefined}>
          {flag('clock_offset_valid') === undefined ? '—' :
              flag('clock_offset_valid') ? 'yes' : 'no'}
        </Stat>
        <Stat label="Resets" channel={`${HW}clock_resets_count`}>{count(num('clock_resets_count'))}</Stat>
        <Stat label="Window" channel={`${HW}clock_window_count`}>{count(num('clock_window_count'))}</Stat>
        <Stat label="Ignored" channel={`${HW}clock_ignored_count`}>{count(num('clock_ignored_count'))}</Stat>
      </StatList>
      <p className="chart-note">
        offset maps the gateway boot clock onto the node clock; spread is the
        one-way-delay spread in the window, valid once the window is full.
      </p>
    </section>

    <section className="card wide">
      <div className="card-heading"><h2>Counters</h2><span>BRIDGE + GATEWAY</span></div>
      {counterRows.length ?
          <DataTable columns={columns} rows={counterRows}
                     rowKey={row => row.name}/> :
          <EmptyState>No counter channels in this session.</EmptyState>}
    </section>

    <section className="card wide">
      <div className="card-heading"><h2>Motor faults</h2><span>MOTOR_FAULT_MASK</span></div>
      <BitList title="Motor fault mask (config order)" channel={`${HW}motor_fault_mask`}
               bits={faultBits} prefix="motor"
               empty="No motor fault flags are set."
               endSec={endSec}/>
    </section>

    <section className="card wide">
      <div className="card-heading"><h2>Motor limit switches</h2><span>ASSERTED</span></div>
      <BitList title="Forward limit (config order)" channel={`${HW}motor_forward_limit_mask`}
               bits={forwardLimitBits} prefix="motor"
               empty="No forward limit switch is asserted."
               endSec={endSec}/>
      <BitList title="Reverse limit (config order)" channel={`${HW}motor_reverse_limit_mask`}
               bits={reverseLimitBits} prefix="motor"
               empty="No reverse limit switch is asserted."
               endSec={endSec}/>
      <p className="chart-note">
        A switch asserted at the end of travel is normal; one that stays
        asserted away from it is a normally closed switch with a cut wire.
      </p>
    </section>
  </div>;
}

export const hardwarePanel: PanelDefinition<HardwareConfig> = {
  id: 'hardware',
  title: 'Hardware link',
  category: 'System',
  description: 'Main-processor ↔ RoboRIO link, power, clock and counters.',
  defaultConfig,
  parseConfig,
  component: HardwarePanel,
  recorded: true
};
