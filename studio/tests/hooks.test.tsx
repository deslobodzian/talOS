// Renders the data hooks with react-dom/server (no DOM) against a static
// studio and asserts what a panel would actually read. useSyncExternalStore
// reads the initial snapshot during server render, so these prove the hooks
// wire through the Studio object without a browser.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import {renderToStaticMarkup} from 'react-dom/server';

import type {Snapshot} from '../src/core';
import {createStaticStudio, StudioProvider} from '../src/data/studio';
import {
  useActions,
  useChannelValue,
  useDictionary,
  useEvents,
  useModeBands,
  usePlayback,
  useRobotStatus,
  useSeries,
  useSnapshot,
  useSystem
} from '../src/data/hooks';
import {StatusStrip} from '../src/shell/StatusStrip';
import {Sparkline} from '../src/ui/Sparkline';

const DS = 'hw/state/driver_station/';

function snap(seq: number, ts: string, channels: Record<string, number>): Snapshot {
  return {
    stream_id: 'a',
    sequence_id: String(seq),
    timestamp_ns: ts,
    dictionary_id: 1,
    chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
    channels,
    units: {},
    labels: {}
  };
}

function Probe() {
  const snapshot = useSnapshot();
  const velocity = useChannelValue('drivetrain/state/velocity');
  const absent = useChannelValue('nope/not/here');
  const status = useRobotStatus();
  const playback = usePlayback();
  const system = useSystem();
  const events = useEvents();
  const dictionary = useDictionary();
  const bands = useModeBands();
  const series = useSeries(['drivetrain/state/velocity'], 'all');
  const actions = useActions();
  return (
    <div>
      <span id="snap">{snapshot?.timestamp_ns ?? 'null'}</span>
      <span id="vel">{velocity}</span>
      <span id="absent">{absent === undefined ? 'undefined' : 'set'}</span>
      <span id="mode">{status.ds.mode ?? 'null'}</span>
      <span id="nodes">{status.nodes ? `${status.nodes.alive}/${status.nodes.total}` : 'null'}</span>
      <span id="pb">{playback.mode}</span>
      <span id="sys">{system.graph ? system.graph.nodes.length : 'null'}</span>
      <span id="events">{events.length}</span>
      <span id="dict">{dictionary.length}</span>
      <span id="bands">{bands.length}</span>
      <span id="series">{series.times.length}</span>
      <span id="actions">{typeof actions.rpc}</span>
    </div>
  );
}

test('hooks render from a static studio', () => {
  const studio = createStaticStudio({
    frames: [
      snap(0, '1000000000', {
        'drivetrain/state/velocity': 1.5,
        [DS + 'connected']: 1,
        [DS + 'enabled']: 0
      }),
      snap(1, '2000000000', {
        'drivetrain/state/velocity': 2.5,
        [DS + 'connected']: 1,
        [DS + 'enabled']: 1,
        [DS + 'teleop']: 1
      })
    ]
  });

  const html =
      renderToStaticMarkup(<StudioProvider studio={studio}><Probe/></StudioProvider>);
  assert.match(html, /id="snap">2000000000/);
  assert.match(html, /id="vel">2\.5/);
  assert.match(html, /id="absent">undefined/);
  assert.match(html, /id="mode">teleop/);
  assert.match(html, /id="nodes">null/);
  assert.match(html, /id="pb">PAUSED/);
  assert.match(html, /id="sys">null/);
  assert.match(html, /id="events">1/);
  assert.match(html, /id="dict">4/);
  assert.match(html, /id="series">2/);
  assert.match(html, /id="actions">function/);
});

test('static studio rpc rejects with a clear error', async () => {
  const studio = createStaticStudio({});
  await assert.rejects(() => studio.actions.rpc('get_latest', {}), /No worker/);
});

test('status strip keeps unknown enabled state free of fault or healthy tones', () => {
  for (const enabled of [null, false, true]) {
    const studio = createStaticStudio({frames: [
      snap(0, '1', enabled === null ? {} : {[DS + 'enabled']: Number(enabled)})
    ]});
    const html = renderToStaticMarkup(
      <StudioProvider studio={studio}><StatusStrip/></StudioProvider>);
    const classes = /class="([^"]*)"><small>Enabled<\/small>/.exec(html)![1].split(' ');
    assert.equal(classes.includes('unknown'), enabled === null);
    assert.equal(classes.includes('warn'), enabled === false);
    assert.equal(classes.includes('ok'), enabled === true);
  }
});

test('sparkline leaves non-finite gaps disconnected between finite runs', () => {
  const html = renderToStaticMarkup(
    <Sparkline values={[0, 1, NaN, 2, 3, Infinity, 4, 5]} width={74} height={24}/>);
  const runs = [...html.matchAll(/points="([^"]*)"/g)].map(match =>
    match[1].split(' ').map(point => Number(point.split(',')[0])));
  assert.deepEqual(runs, [[2, 12], [32, 42], [62, 72]]);
  assert.equal((renderToStaticMarkup(<Sparkline values={[NaN, Infinity]}/>)
    .match(/<polyline/g) ?? []).length, 0);
});
