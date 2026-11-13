// The hardware panel: everything about the main-processor <-> RoboRIO link and
// power, read off /hw/status. Four views over the same snapshot:
//
//   Power   — battery voltage and CAN utilization, with brownout bands.
//   Link    — state age and the actuator-owning subsystem timeouts.
//   Clock   — the gateway<->node clock mapping.
//   Counters and the motor fault mask — the bridge and gateway's tallies,
//             total and rate, with a plain-English note per counter.
//
// Every channel is discovered, never assumed: an absent channel renders as
// "—" (no data), and a counter that was never declared is not listed.

import {useMemo} from 'react';

import {useDictionary, useSeries, useSnapshot} from '../../data/hooks';
import {durationUs, integer} from '../../ui/formatters';
import {DataTable, type Column} from '../../ui/DataTable';
import {EmptyState, StatList} from '../../ui/primitives';
import {TimeSeriesChart} from '../../ui/TimeSeriesChart';
import type {PanelDefinition} from '../types';
import {bitHistory, counterRate, flagBands, type BitHistory} from './logic';
import {ChannelReading, ChannelStat as Stat, FreshnessNotice} from '../../ui/FreshnessNotice';
import './hardware.css';

export type HardwareConfig = Record<string, never>;

const defaultConfig: HardwareConfig = {};
const parseConfig = (_raw: unknown): HardwareConfig => ({});

const HW = 'hw/status/';

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
  }
];

type CounterRow = CounterSpec & {total: string; rate: string};

const rateString = (rate: number|null): string =>
    rate === null ? '—' : `${rate >= 100 ? rate.toFixed(0) : rate.toFixed(1)}/s`;

const volts = (v: number|undefined) =>
    v === undefined ? '—' : `${v.toFixed(2)} V`;
const percent = (f: number|undefined) =>
    f === undefined ? '—' : `${(f * 100).toFixed(0)}%`;
const micros = (us: number|undefined) =>
    us === undefined ? '—' : durationUs(us);
// A uint64 counter, with the absent case shown as "no data".
const count = (value: number|undefined) =>
    value === undefined ? '—' : integer(value);

// When a bit last rose, as a human offset from the end of the retained view.
const ago = (lastSetAt: number|null, endSec: number): string => {
  if (lastSetAt === null) return '—';
  return `${(endSec - lastSetAt).toFixed(1)}s ago`;
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

function HardwarePanel() {
  const snapshot = useSnapshot();
  const dictionary = useDictionary();

  const ch = snapshot?.channels ?? {};
  const num = (name: string): number|undefined => ch[`${HW}${name}`];
  const flag = (name: string): boolean|undefined => {
    const v = num(name);
    return v === undefined ? undefined : v !== 0;
  };

  const power = useSeries(
      [`${HW}battery_v`, `${HW}brownout`, `${HW}can_utilization_frac`],
      {seconds: 120});
  const age = useSeries([`${HW}state_age_us`], {seconds: 120});
  const masks = useSeries(
      [`${HW}timed_out_mask`, `${HW}motor_fault_mask`,
       `${HW}motor_forward_limit_mask`, `${HW}motor_reverse_limit_mask`],
      'all');

  const present = useMemo(() => new Set(dictionary.map(c => c.name)), [dictionary]);
  const counterNames = useMemo(
      () => COUNTERS.filter(c => present.has(`${HW}${c.name}`))
          .map(c => `${HW}${c.name}`),
      [present]);
  const counterSeries = useSeries(counterNames, {seconds: 10});

  const endSec = masks.times.length ? masks.times[masks.times.length - 1] : 0;

  const batteryV = num('battery_v');
  const brownout = flag('brownout');
  const canFrac = num('can_utilization_frac');
  const healthValid = flag('health_valid');
  const stateAgeUs = num('state_age_us');
  const configured = flag('configured');
  const subsystemCount = num('subsystem_count');
  const timeoutUs = num('command_timeout_us');  // not published by the 2026 bridge

  const counterRows: CounterRow[] = useMemo(() => {
    const out: CounterRow[] = [];
    for (const spec of COUNTERS) {
      if (!present.has(`${HW}${spec.name}`)) continue;
      out.push({
        ...spec,
        total: count(ch[`${HW}${spec.name}`]),
        rate: rateString(counterRate(
            counterSeries.times, counterSeries.values[`${HW}${spec.name}`]))
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
      present.has(`${HW}clock_offset_us`);

  if (!anyHardware) {
    return <div className="hardware">
      <EmptyState>
        No /hw/status telemetry. Connect to a robot bridge, or start demo data
        to explore the link and power views.
      </EmptyState>
    </div>;
  }

  const brownoutBands = flagBands(power.times, power.values[`${HW}brownout`]);
  const timedOutBits = bitHistory(
      masks.times, masks.values[`${HW}timed_out_mask`], subsystemCount ?? 32);
  const faultBits = bitHistory(masks.times, masks.values[`${HW}motor_fault_mask`]);
  const forwardLimitBits = bitHistory(
      masks.times, masks.values[`${HW}motor_forward_limit_mask`]);
  const reverseLimitBits = bitHistory(
      masks.times, masks.values[`${HW}motor_reverse_limit_mask`]);

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
        <Stat label="CAN utilization" channel={`${HW}can_utilization_frac`}>{percent(canFrac)}</Stat>
      </StatList>
      <div className="hw-block">
        <h3>Battery voltage · 2 min</h3>
        <TimeSeriesChart
            lines={[{name: 'battery_v', color: 'var(--accent)',
                     times: power.times, values: power.values[`${HW}battery_v`]}]}
            bands={brownoutBands} height={96} legend={false}/>
        <p className="chart-note">Shaded spans are brownout ({
            brownout === undefined ? 'unknown' : brownout ? 'active' : 'clear'
          }).</p>
      </div>
      <div className="hw-block">
        <h3>CAN utilization · 2 min</h3>
        <TimeSeriesChart
            lines={[{name: 'can_utilization_frac', color: 'var(--info)',
                     times: power.times,
                     values: power.values[`${HW}can_utilization_frac`]}]}
            height={72} legend={false}/>
      </div>
    </section>

    <section className="card">
      <div className="card-heading"><h2>Link</h2><span>HW/STATUS</span></div>
      <StatList>
        <Stat label="Configured" channel={`${HW}configured`} tone={configured === false ? 'warn' : undefined}>
          {configured === undefined ? '—' : configured ? 'yes' : 'no'}
        </Stat>
        <Stat label="Subsystems" channel={`${HW}subsystem_count`}>
          {subsystemCount === undefined ? '—' : integer(subsystemCount)}
        </Stat>
        <Stat label="State age" channel={`${HW}state_age_us`} tone={stateAgeUs !== undefined && stateAgeUs > 20000 ? 'warn' : undefined}>
          {micros(stateAgeUs)}
        </Stat>
      </StatList>
      <div className="hw-block">
        <h3>State age · 2 min</h3>
        <TimeSeriesChart
            lines={[{name: 'state_age_us', color: 'var(--accent)',
                     times: age.times, values: age.values[`${HW}state_age_us`]}]}
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
  component: HardwarePanel
};
