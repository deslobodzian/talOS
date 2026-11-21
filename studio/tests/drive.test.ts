// The driver dashboard page (src/drive): its feed model on an injected clock
// and a fake socket, and its rendering of the 2026 layout. The layout fixture
// is what the bridge serves at /drive/layout.json for the 2026 robot's
// dashboard.toml, in that robot's repository (DashboardLayoutJson,
// the shape //talOS/telemetry:dashboard_layout_test pins); the telemetry is
// real TLM3 pages, decoded by the same TelemetryDecoder Studio uses.

import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {test} from 'node:test';

import {AlertLevel, ChannelKind, type ChannelSpec, dictionaryId, encodeDictionaryPages, encodeValuePages, rawFromDouble} from '../src/encode';
import {CONNECTING_GRACE_MS, DEAD_AFTER_MS, DECODE_RETRY_MS, DriveModel, type DriveSocket, type Link, RECONNECT_MS, RENDER_MS, REPLACED_CODE, SILENCE_MS, STALE_MS} from '../src/drive/feed';
import {parseLayout} from '../src/drive/layout';
import {DrawSchedule, esc, GLYPH_PX, listColumns, listRowsFit, METRICS, nameCharsShown, NODE_STATE, outOfView, outOfViewText, Painter, renderDashboard, renderStatus, STALE_DRAW_LAG_MS, type Surface, type Timers} from '../src/drive/view';

const LAYOUT = parseLayout(JSON.parse(
    readFileSync(new URL('./drive_layout_2026.json', import.meta.url), 'utf8')));

const LINK_LOST = 'hw/status/alert_error_link_lost';
const OWNER_TIMED_OUT = 'hw/status/alert_warning_owner_timed_out';
const ROUTINES = ['preload', 'none', 'shoot_only', 'taxi'];

// The 2026 dictionary as far as the feed carries it: every layout channel
// (the link-lost alert among them: the bridge always lists it) plus the
// /hw/status alert channels the alerts widget adds.
const LABELS: Record<string, string[]> = {
  'hw/state/driver_station/alliance': ['unknown', 'red', 'blue'],
  'hw/state/driver_station/match_type': ['none', 'practice', 'qualification', 'elimination'],
  'autonomous/state/routine_index': ROUTINES,
  'autonomous/state/phase': ['IDLE', 'DRIVING', 'SHOOTING', 'DONE'],
  'intake/state/inhibit': ['NONE', 'SHOOTER_FULL']
};
const UINT = new Set(['hw/state/driver_station/match_number', 'hw/state/driver_station/station',
                      ...Object.keys(LABELS)]);
const DOUBLE = new Set(['hw/state/driver_station/match_time_s', 'hw/status/battery_v',
                        'pose_estimator/state/pose/x_m', 'pose_estimator/state/pose/y_m',
                        'pose_estimator/state/pose/yaw_rad',
                        'shooter/state/flywheel_velocity_rps', 'shooter/state/target_velocity_rps']);
const unitOf = (name: string) => name.endsWith('_rps') ? 'rps' : name.endsWith('_v') ? 'V' :
    name.endsWith('_s') ? 's' : name.endsWith('_m') ? 'm' : name.endsWith('_rad') ? 'rad' : '';
const CHANNELS: ChannelSpec[] = [
  ...LAYOUT.channels.filter(name => name !== LINK_LOST).map(name => ({
    name, unit: unitOf(name), labels: LABELS[name],
    kind: DOUBLE.has(name) ? ChannelKind.Double : UINT.has(name) ? ChannelKind.UInt :
                             ChannelKind.Bool
  })),
  {name: LINK_LOST, unit: '', kind: ChannelKind.Bool, alert: AlertLevel.Error},
  {name: OWNER_TIMED_OUT, unit: '', kind: ChannelKind.Bool, alert: AlertLevel.Warning}
];
const INDEX = new Map(CHANNELS.map((c, i) => [c.name, i]));
const DICTIONARY_ID = dictionaryId(CHANNELS);
const header = (seq: number) => ({
  streamId: 7n, sequenceId: BigInt(seq), timestampNs: BigInt(seq) * 50_000_000n,
  dictionaryId: DICTIONARY_ID, channelCount: CHANNELS.length
});
const POSE = {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0};

// A healthy robot in autonomous, routine "shoot_only".
const NOMINAL: Record<string, number> = {
  'hw/state/driver_station/connected': 1, 'hw/state/driver_station/enabled': 1,
  'hw/state/driver_station/autonomous': 1, 'hw/state/driver_station/teleop': 0,
  'hw/state/driver_station/test': 0, 'hw/state/driver_station/estop': 0,
  'hw/state/driver_station/fms_attached': 1, 'hw/state/driver_station/alliance': 1,
  'hw/state/driver_station/match_time_s': 12.4, 'hw/state/driver_station/match_number': 12,
  'hw/state/driver_station/match_type': 2, 'hw/state/driver_station/station': 2,
  'hw/status/battery_v': 12.61, 'hw/status/brownout': 0, 'hw/status/health_valid': 1,
  'hw/status/configured': 1, 'hw/status/commissioned': 0,
  'autonomous/state/routine_index': 2, 'autonomous/state/phase': 1,
  'drivetrain/state/modules_valid': 1, 'drivetrain/state/yaw_valid': 1,
  'intake/state/beam_broken': 0, 'intake/state/inhibit': 1,
  'pose_estimator/state/pose/x_m': 3.2, 'pose_estimator/state/pose/y_m': 4.1,
  'pose_estimator/state/pose/yaw_rad': 0.5, 'pose_estimator/state/valid': 1,
  'pose_estimator/state/localized': 1,
  'shooter/state/flywheel_velocity_rps': 52.34, 'shooter/state/target_velocity_rps': 55,
  'shooter/state/at_speed': 0, 'shooter/state/beam_broken': 1,
  [LINK_LOST]: 0, [OWNER_TIMED_OUT]: 0
};

// The same with /hw/status not published: its alert channels absent.
const WITHOUT_HW_STATUS_ALERTS = Object.fromEntries(
    Object.entries(NOMINAL).filter(([name]) => name !== LINK_LOST && name !== OWNER_TIMED_OUT));

const raw = (name: string, value: number) =>
    CHANNELS[INDEX.get(name)!].kind === ChannelKind.Double ? rawFromDouble(value) : BigInt(value);
// One update's pages: a keyframe of `values` (every other channel absent), or
// a change-only update of them (an empty one is the bridge's heartbeat).
function update(seq: number, values: Record<string, number>, keyframe: boolean) {
  const samples = Object.entries(values).map(
      ([name, v]) => [INDEX.get(name)!, raw(name, v)] as [number, bigint]);
  return encodeValuePages(header(seq), POSE, keyframe, samples);
}

class FakeSocket implements DriveSocket {
  binaryType = '';
  onopen: (() => void)|null = null;
  onmessage: ((event: {data: unknown}) => void)|null = null;
  onclose: ((event: {code: number; reason: string}) => void)|null = null;
  onerror: (() => void)|null = null;
  closed = false;
  close() {
    this.closed = true;
  }
  // What the bridge does: each page or text is one WebSocket message.
  send(...messages: (Uint8Array|string)[]) {
    for (const data of messages)
      this.onmessage?.({data: typeof data === 'string' ? data : data.slice().buffer});
  }
}

// A model on a clock the test sets; `sockets` is every socket it opened.
// `carried` is what /drive/layout.json says the feed carries.
function harness(carried: {channels: readonly string[]; alerts: boolean} = LAYOUT) {
  const sockets: FakeSocket[] = [];
  const tracked = ['shooter/state/flywheel_velocity_rps', 'shooter/state/target_velocity_rps',
                   'pose_estimator/state/pose/x_m', 'pose_estimator/state/pose/y_m'];
  const model = new DriveModel('ws://robot/drive', () => {
    // Never two sockets: every earlier one is closed before a new one opens.
    assert.ok(sockets.every(s => s.closed), 'a second socket opened while one was live');
    const s = new FakeSocket();
    sockets.push(s);
    return s;
  }, tracked, carried);
  const clock = {now: 0};
  model.clock = () => clock.now;
  const at = (ms: number) => {
    clock.now = ms;
    model.tick(ms);
  };
  const socket = () => sockets[sockets.length - 1];
  // Opens a socket at `ms` and receives what the bridge sends on connect: the
  // dictionary and a keyframe of the tick it holds, `seq`.
  const join = (ms: number, values: Record<string, number>, seq: number) => {
    at(ms);
    model.start(ms);
    socket().onopen!();
    socket().send(...encodeDictionaryPages(header(seq), CHANNELS), ...update(seq, values, true));
  };
  // Connects to a live producer at `ms`: the connect-time keyframe of tick
  // `seq - 1`, then an update of tick `seq` (nothing changed), which is what
  // makes the values current.
  const connect = (ms: number, values = NOMINAL, seq = 1) => {
    join(ms, values, seq - 1);
    socket().send(...update(seq, {}, false));
  };
  return {model, sockets, at, socket, join, connect, clock};
}

const render = (h: ReturnType<typeof harness>) => renderDashboard(LAYOUT, h.model, h.clock.now);
// The HTML of the widget whose heading is `title`.
function widgetHtml(html: string, title: string): string {
  const start = html.indexOf(`<h2>${title}</h2>`);
  assert.ok(start >= 0, `no widget titled ${title} in ${html}`);
  const open = html.lastIndexOf('<section', start);
  return html.slice(open, html.indexOf('</section>', start));
}

// REQ-OPS-39: every widget of the 2026 layout is drawn in its own cell of
// the layout's grid, with the values the feed carries.
test('renders every widget of the 2026 layout in its cell', () => {
  const h = harness();
  h.connect(0);
  h.socket().send(JSON.stringify({kind: 'talos.drive.summary', session: '42', supervised: true,
                                  registry: true, nodes: [], omitted: 0}));
  const html = render(h);
  assert.match(html, /grid-template-columns:repeat\(12,minmax\(0,1fr\)\);grid-template-rows:repeat\(8,/);
  for (const w of LAYOUT.widgets) {
    const {column, row, width, height} = w.at;
    assert.ok(html.includes(`class="w w-${w.kind}" style="grid-column:${column + 1} / span ${width};` +
                            `grid-row:${row + 1} / span ${height}"`),
              `widget ${w.kind} at ${JSON.stringify(w.at)} not drawn current in its cell`);
  }
  assert.equal(html.match(/<section /g)?.length, LAYOUT.widgets.length,
               'a widget is dropped or drawn twice');
  assert.ok(!html.includes('Unsupported'), 'a 2026 widget kind is unsupported');
  assert.match(widgetHtml(html, 'Match'), /AUTO/);
  assert.match(widgetHtml(html, 'Match'), /0:12/);
  assert.match(widgetHtml(html, 'Match'), /RED 2/);
  assert.match(widgetHtml(html, 'Match'), /qualification 12/);
  assert.match(widgetHtml(html, 'Battery'), /12\.61 <span class="unit">V<\/span>/);
  assert.match(widgetHtml(html, 'Flywheel'), /52\.3 <span class="unit">rps<\/span>/);
  assert.match(widgetHtml(html, 'At speed'), /flag bad">NO/);
  assert.match(widgetHtml(html, 'Shooter beam'), /flag good">YES/);
  assert.match(widgetHtml(html, 'Intake inhibit'), /SHOOTER_FULL/);
  assert.match(widgetHtml(html, 'Field'), /translate\(3\.200 3\.969\)/);
  // Battery 12.61 >= 12.5 passes; commissioned is false and fails.
  const checks = widgetHtml(html, 'Pre-match');
  assert.equal(checks.match(/tag-good">OK/g)?.length, 5, checks);
  assert.match(checks, /tag-bad">FAIL<\/span><span class="name"[^>]*>Commissioned/);
});

// REQ-OPS-49: a widget this page cannot draw is never a silent blank: a kind
// it does not draw is a visible cell, and one with no cell it can read (a
// layout from another version) is counted on the status line.
test('a widget the page cannot draw is shown, never a silent blank', () => {
  const layout = parseLayout({columns: 4, rows: 2, widgets: [
    {kind: 'camera', at: [0, 0, 2, 1], title: '', channels: {}},
    {kind: 'number', at: [2, 0, 2, 1], title: 'Speed', channels: {}},
    {kind: 'boolean', at: [0, 1], title: 'Ready', channels: {}},
  ]});
  const h = harness();
  const html = renderDashboard(layout, h.model, 0);
  assert.match(html, /grid-column:1 \/ span 2;grid-row:1 \/ span 1[^>]*><header><h2>Unsupported widget<\/h2>/);
  assert.match(html, /Unsupported widget "camera"/);
  assert.match(html, /<h2>Speed<\/h2>.*no data/);
  const status = html.slice(html.indexOf('role="status"'), html.indexOf('</div>', html.indexOf('role="status"')));
  assert.match(status, /1 layout widget not drawn/, 'a widget with no readable cell vanished silently');
});

// Timers on a test clock: `run(t)` fires, in order, every timeout due by t.
function fakeTimers(clock: {now: number; wall: number}) {
  let pending: {at: number; fn: () => void; handle: number}[] = [];
  let next = 0;
  const timers: Timers = {
    now: () => clock.now,
    wall: () => clock.wall,
    setTimeout: (fn, ms) => {
      pending.push({at: clock.now + ms, fn, handle: ++next});
      return next;
    },
    clearTimeout: handle => {
      pending = pending.filter(p => p.handle !== handle);
    }
  };
  const run = (t: number) => {
    for (;;) {
      const due = pending.filter(p => p.at <= t).sort((a, b) => a.at - b.at)[0];
      if (!due) return;
      pending = pending.filter(p => p !== due);
      clock.now = due.at;
      due.fn();
    }
  };
  return {timers, run};
}

// REQ-OPS-36: a value is shown stale within 500 ms of its last update,
// whatever else arrives (the bridge's empty update re-sends the held tick;
// only a sequence_id that advances keeps values current). The bridge may
// deliver the producer's last tick up to its 100 ms heartbeat after the
// tick, so the page's own share must be STALE_MS and not STALE_MS plus a
// render period. Whatever the periodic draw phase, the first stale draw
// comes STALE_MS (+ STALE_DRAW_LAG_MS) after the last new tick's receipt,
// keeping the last value shown and marked, and a newer tick clears STALE.
test('the page draws a stopped value stale at its deadline at any draw phase', () => {
  for (let phase = 0; phase < RENDER_MS; phase += 7) {
    const h = harness();
    const clock = {now: 0, wall: 1e12};
    h.model.clock = () => clock.now;
    const {timers, run} = fakeTimers(clock);
    let shownStale = Infinity;
    let staleHtml = '';
    const schedule = new DrawSchedule(h.model, now => {
      h.model.tick(now);
      h.clock.now = now;
      const html = widgetHtml(render(h), 'Flywheel');
      if (shownStale === Infinity && html.includes('STALE')) {
        shownStale = now;
        staleHtml = html;
      }
    }, timers);
    h.connect(0);  // the last new tick, received at 0
    schedule.frame();
    assert.ok(!widgetHtml(render(h), 'Flywheel').includes('STALE'), 'stale while current');
    for (let t = 1; t <= 1000 && shownStale === Infinity; t++) {
      run(t);
      clock.now = clock.wall = t;
      clock.wall += 1e12;
      if (t % 100 === 0) h.socket().send(...update(1, {}, false));
      if (t % RENDER_MS === phase) schedule.frame();
    }
    assert.ok(shownStale >= STALE_MS && shownStale <= STALE_MS + STALE_DRAW_LAG_MS,
              `draw phase ${phase} ms: first stale draw at ${shownStale} ms, deadline ` +
              `${STALE_MS} ms + ${STALE_DRAW_LAG_MS} ms (REQ-OPS-36: 500 ms with the bridge's 100 ms heartbeat)`);
    if (phase === 0) {
      assert.match(staleHtml, /class="w w-number stale"/);
      assert.match(staleHtml, /tag-stale">STALE/);
      assert.match(staleHtml, /52\.3/, 'a stale value is not shown');
      h.at(600);
      h.socket().send(...update(2, {}, false));
      assert.ok(!widgetHtml(render(h), 'Flywheel').includes('STALE'),
                'a newer tick did not clear STALE');
    }
  }
});

// REQ-OPS-36: performance.now() may stand still while the laptop sleeps. A
// draw that finds the wall clock moved more than STALE_MS beyond it shows the
// values stale until a new tick arrives.
test('a draw after the page clock stood still across a sleep shows values stale', () => {
  const h = harness();
  const clock = {now: 0, wall: 0};
  const {timers} = fakeTimers(clock);
  const stale = () => widgetHtml(render(h), 'Flywheel').includes('STALE');
  const schedule = new DrawSchedule(h.model, now => h.model.tick(now), timers);
  h.connect(0);
  schedule.frame();
  assert.ok(!stale());
  clock.wall = 60_000;  // a minute asleep; the monotonic clock still reads 0
  schedule.frame();
  assert.ok(stale(), 'values read current after the page clock missed a sleep');
  h.socket().send(...update(2, {}, false));
  assert.ok(!stale(), 'a new tick after the sleep does not make the values current');
});

// REQ-OPS-36: on connect the bridge sends a keyframe of the newest tick it
// holds, however old. Scenario: the telemetry node died minutes ago and the
// coach reloads /drive; the keyframe's values must read stale, never current.
// With a live producer the next message, an update of a newer tick, makes
// them current.
test('a connect-time keyframe shows its values stale until a newer tick arrives', () => {
  const h = harness();
  h.join(0, NOMINAL, 100);
  const flywheel = () => widgetHtml(render(h), 'Flywheel');
  assert.match(flywheel(), /tag-stale">STALE/, 'the connect-time keyframe is shown current');
  assert.match(flywheel(), /52\.3/, 'the keyframe values are not shown');
  assert.equal(h.model.status(h.clock.now).link, 'connecting');
  // The producer is dead: the bridge restamps tick 100 every 100 ms.
  for (let t = 100; t <= 1000; t += 100) {
    h.at(t);
    h.socket().send(...update(100, {}, false));
    assert.match(flywheel(), /STALE/, `shown current at ${t} ms on the repeated keyframe tick`);
  }
  assert.equal(h.model.status(h.clock.now).link, 'telemetry');

  const live = harness();
  live.join(0, NOMINAL, 100);
  live.at(100);
  live.socket().send(...update(101, {}, false));
  assert.doesNotMatch(widgetHtml(render(live), 'Flywheel'), /STALE/,
                      'a newer tick after the keyframe is not current');
  assert.equal(live.model.status(live.clock.now).link, 'ok');
});

// REQ-OPS-36: Wi-Fi drops at T, the producer runs on to T+3 s and dies, the
// page reconnects at T+6 s. The connect-time keyframe is newer than the tick
// the page held but 3 s old: it must not read as current either.
test('a reconnect does not show the connect-time keyframe as current', () => {
  const h = harness();
  h.connect(0, NOMINAL, 2);
  h.socket().onclose!({code: 1006, reason: ''});
  for (let t = 100; h.sockets.length < 2; t += 100) h.at(t);
  const reconnectedAt = h.clock.now;
  h.socket().onopen!();
  h.socket().send(...encodeDictionaryPages(header(60), CHANNELS),
                  ...update(60, {...NOMINAL, 'shooter/state/flywheel_velocity_rps': 70}, true));
  const held = widgetHtml(render(h), 'Flywheel');
  assert.match(held, /70\.0/, 'the keyframe did not replace the held values');
  assert.match(held, /STALE/, 'the connect-time keyframe after a reconnect is shown current');
  h.at(reconnectedAt + 100);
  h.socket().send(...update(61, {}, false));
  assert.doesNotMatch(widgetHtml(render(h), 'Flywheel'), /STALE/);
});

// REQ-OPS-36: a value never received is absent, not zero.
test('a channel never received is shown as no data, not zero', () => {
  const h = harness();
  const values = {...NOMINAL};
  delete values['shooter/state/flywheel_velocity_rps'];
  delete values['shooter/state/at_speed'];
  h.connect(0, values);
  const html = render(h);
  assert.match(widgetHtml(html, 'Flywheel'), /class="big">.*no data/);
  assert.doesNotMatch(widgetHtml(html, 'Flywheel'), /0\.0/);
  assert.match(widgetHtml(html, 'At speed'), /no data/);
  assert.doesNotMatch(widgetHtml(html, 'At speed'), /NO</);
});

// REQ-OPS-36: the status line names the link that failed, upstream first,
// and the bucket refill after connecting reads as connecting.
test('the status line names the failed link', () => {
  const cases: {name: string; arrange: (h: ReturnType<typeof harness>) => void; link: Link;
                carried?: {channels: readonly string[]; alerts: boolean};
                textMatch?: RegExp}[] = [
    {name: 'all links up', link: 'ok', arrange: h => h.connect(0)},
    {name: 'socket closed', link: 'bridge', arrange: h => {
      h.connect(0);
      h.socket().onclose!({code: 1006, reason: ''});
    }},
    {name: `no message for more than ${SILENCE_MS} ms`, link: 'bridge', arrange: h => {
      h.connect(0);
      h.at(SILENCE_MS + 1);
    }},
    {name: 'heartbeats only, refilling bucket', link: 'connecting', arrange: h => {
      h.model.start(0);
      h.socket().onopen!();
      for (let t = 100; t <= CONNECTING_GRACE_MS; t += 100) {
        h.at(t);
        h.socket().send('{"kind":"talos.drive.heartbeat"}');
      }
    }},
    {name: 'heartbeats only, past the refill bound', link: 'telemetry', arrange: h => {
      h.model.start(0);
      h.socket().onopen!();
      for (let t = 100; t <= CONNECTING_GRACE_MS + 100; t += 100) {
        h.at(t);
        h.socket().send('{"kind":"talos.drive.heartbeat"}');
      }
    }},
    {name: 'updates repeat the held tick', link: 'telemetry', arrange: h => {
      h.connect(0);
      for (let t = 100; t <= 400; t += 100) {
        h.at(t);
        h.socket().send(...update(1, {}, false));
      }
    }},
    {name: 'RoboRIO link lost', link: 'roborio', arrange: h => {
      h.connect(0);
      h.at(50);
      h.socket().send(...update(2, {[LINK_LOST]: 1}, false));
    }},
    {name: 'RoboRIO link lost, named by a layout without an alerts widget', link: 'roborio',
     carried: {channels: LAYOUT.channels, alerts: false}, arrange: h => {
      h.connect(0);
      h.at(50);
      h.socket().send(...update(2, {[LINK_LOST]: 1}, false));
    }},
    {name: 'no hw/status: the hardware bridge is not publishing', link: 'hardware', arrange: h => {
      h.connect(0, WITHOUT_HW_STATUS_ALERTS);
    }},
    // The feed never carries the channel for this layout, so its absence
    // says nothing about the RoboRIO link — and the OK line says it is not
    // judged rather than claiming a verified link.
    {name: 'layout without the link channel', link: 'ok',
     carried: {channels: LAYOUT.channels.filter(n => n !== LINK_LOST), alerts: false},
     textMatch: /RoboRIO link not judged\)$/,
     arrange: h => {
      h.connect(0, WITHOUT_HW_STATUS_ALERTS);
    }},
  ];
  for (const c of cases) {
    const h = harness(c.carried);
    c.arrange(h);
    const status = h.model.status(h.clock.now);
    assert.equal(status.link, c.link, `${c.name}: ${status.text}`);
    if (c.textMatch) assert.match(status.text, c.textMatch, `${c.name}: ${status.text}`);
    assert.match(renderStatus(h.model, h.clock.now), new RegExp(`link-${c.link}`), c.name);
  }
});

// REQ-OPS-36: a session with no hardware bridge has no /hw/status in its
// dictionary, so the OK line states that the RoboRIO link is not judged
// without a hardware bridge. When a layout does carry hw/status/state_age_us,
// renderStatus surfaces the State age alongside the status line.
test('OK line names an absent hardware bridge, and surfaces state age when carried', () => {
  const noHwChannels: ChannelSpec[] = [
    {name: 'shooter/state/flywheel_velocity_rps', unit: 'rps', kind: ChannelKind.Double},
  ];
  const noHwHead = (seq: number) => ({
    streamId: 11n, sequenceId: BigInt(seq), timestampNs: BigInt(seq) * 50_000_000n,
    dictionaryId: dictionaryId(noHwChannels), channelCount: noHwChannels.length
  });
  const h = harness();
  h.model.start(0);
  h.socket().onopen!();
  h.socket().send(
      ...encodeDictionaryPages(noHwHead(0), noHwChannels),
      ...encodeValuePages(noHwHead(0), POSE, true, [[0, rawFromDouble(10)]]),
      ...encodeValuePages(noHwHead(1), POSE, false, []));
  const status = h.model.status(0);
  assert.equal(status.link, 'ok');
  assert.match(status.text, /RoboRIO link not judged: no hardware bridge/);
  assert.match(renderStatus(h.model, 0), /RoboRIO link not judged: no hardware bridge/);

  const ageChannels: ChannelSpec[] = [
    {name: LINK_LOST, unit: '', kind: ChannelKind.Bool, alert: AlertLevel.Error},
    {name: 'hw/status/state_age_us', unit: 'us', kind: ChannelKind.UInt},
  ];
  const ageHead = (seq: number) => ({
    streamId: 12n, sequenceId: BigInt(seq), timestampNs: BigInt(seq) * 50_000_000n,
    dictionaryId: dictionaryId(ageChannels), channelCount: ageChannels.length
  });
  const withAge = harness({channels: [LINK_LOST, 'hw/status/state_age_us'], alerts: true});
  withAge.model.start(0);
  withAge.socket().onopen!();
  withAge.socket().send(
      ...encodeDictionaryPages(ageHead(0), ageChannels),
      ...encodeValuePages(ageHead(0), POSE, true, [[0, 0n], [1, 4200n]]),
      ...encodeValuePages(ageHead(1), POSE, false, []));
  assert.match(renderStatus(withAge.model, 0), /state age 4 ms/);
  withAge.socket().send(...encodeValuePages(ageHead(2), POSE, false, [[0, 1n], [1, 0n]]));
  assert.match(renderStatus(withAge.model, 0), /link-roborio.*no state received/);
  withAge.socket().send(...encodeValuePages(ageHead(3), POSE, false, [[0, 1n], [1, 350_000n]]));
  assert.match(renderStatus(withAge.model, 0), /link-roborio.*state age 350 ms/);
});

// REQ-OPS-36: on a close or an error the page reconnects every 1 s, one
// socket at a time: each attempt starts 1 s after the one before it started.
test('reconnects every second after a close or an error, one socket at a time', () => {
  const h = harness();
  h.connect(0);
  h.at(200);
  h.socket().onclose!({code: 1006, reason: ''});
  h.at(RECONNECT_MS - 1);
  assert.equal(h.sockets.length, 1, 'reconnected within 1 s of the last attempt');
  h.at(RECONNECT_MS);
  assert.equal(h.sockets.length, 2, 'no reconnect 1 s after the last attempt');
  // The attempt fails at once (the bridge is down): the next goes 1 s later.
  h.sockets[1].onerror!();
  assert.ok(h.sockets[1].closed, 'an errored socket is not closed');
  h.at(2 * RECONNECT_MS - 1);
  assert.equal(h.sockets.length, 2, 'reconnected within 1 s of the last attempt');
  h.at(2 * RECONNECT_MS);
  assert.equal(h.sockets.length, 3);
  h.socket().onopen!();
  h.socket().send(...encodeDictionaryPages(header(5), CHANNELS), ...update(5, NOMINAL, true),
                  ...update(6, {}, false));
  assert.equal(h.model.status(h.clock.now).link, 'ok');
});

// REQ-OPS-36: after close 4000 the page shows it was replaced and reconnects
// only on the user's action, so two pages never take the slot back and forth.
test('close 4000 stops reconnecting until the user presses Reconnect', () => {
  const h = harness();
  h.connect(0);
  h.socket().onclose!({code: REPLACED_CODE, reason: 'replaced by a newer driver dashboard'});
  for (let t = 100; t <= 30_000; t += 100) h.at(t);
  assert.equal(h.sockets.length, 1, 'reconnected by itself after 4000');
  const status = renderStatus(h.model, h.clock.now);
  assert.match(status, /REPLACED/);
  assert.match(status, /data-action="reconnect"/);
  h.model.reconnect(h.clock.now);
  assert.equal(h.sockets.length, 2);
  assert.equal(h.model.connection, 'connecting');
});

// REQ-OPS-37: active alerts error first with the level as a word, and an
// activation of 100 ms, carried by one update, stays in the match's latch.
test('alerts: active by level as a word, and a 100 ms activation latched', () => {
  const h = harness();
  h.connect(0);
  h.at(50);
  h.socket().send(...update(2, {[LINK_LOST]: 1, [OWNER_TIMED_OUT]: 1}, false));
  const active = widgetHtml(render(h), 'Alerts');
  const error = active.indexOf('<span class="level">ERROR</span> <span class="name" title="hw/status/alert_error_link_lost">');
  const warning = active.indexOf('<span class="level">WARNING</span> <span class="name" title="hw/status/alert_warning_owner_timed_out">');
  assert.ok(error >= 0 && warning > error, active);
  // Both clear 100 ms later.
  h.at(150);
  h.socket().send(...update(3, {[LINK_LOST]: 0, [OWNER_TIMED_OUT]: 0}, false));
  const after = widgetHtml(render(h), 'Alerts');
  assert.match(after, /<h3>Active<\/h3><p class="none">none<\/p>/);
  assert.match(after, /<b>hw<\/b> link lost<\/span> <span class="meta">×1<\/span>/);
});

// REQ-OPS-38: the routine widget names the routine from the dictionary's
// labels, and shows the number when the index is out of range.
test('the routine widget shows the label for the index, the number out of range', () => {
  const h = harness();
  h.connect(0);
  assert.match(widgetHtml(render(h), 'Auto routine'), /class="big routine">shoot_only</);
  h.at(50);
  h.socket().send(...update(2, {'autonomous/state/routine_index': 7}, false));
  assert.match(widgetHtml(render(h), 'Auto routine'), /class="big routine">routine #7</);
});

// REQ-OPS-49: the subsystems widget shows what the drive summary says, one
// state word per row (the most severe: a quarantined node is not running, so
// FAILING and not DOWN beside it, which left no width for the name), unknown
// as unknown, and the cut-off list as "+N more".
test('the subsystems widget lists the summary, one state word per row', () => {
  const node = (name: string, extra: object) => ({name, alive: true, critical: true, restarts: 0,
                                                  quarantined: false, drives: null, ...extra});
  const h = harness();
  h.connect(0);
  h.socket().send(JSON.stringify({kind: 'talos.drive.summary', session: '42', supervised: true,
    registry: true, omitted: 3, nodes: [
      node('hardware_node', {}), node('drivetrain', {restarts: 2, drives: 'teleop'}),
      node('shooter', {alive: false, quarantined: true}),
      node('studio_bridge', {critical: false, restarts: 3})]}));
  const html = widgetHtml(render(h), 'Subsystems');
  assert.match(html, /drivetrain<\/span><span class="tag tag-good">UP<\/span><span class="meta">restarts 2 · drives teleop/);
  assert.match(html, /shooter<\/span><span class="tag tag-bad">FAILING<\/span>/);
  assert.match(html, /studio_bridge<\/span><span class="tag tag-good">UP<\/span><span class="meta">restarts 3/);
  for (const row of html.match(/<li>.*?<\/li>/g) ?? [])
    assert.ok((row.match(/class="tag /g) ?? []).length <= 1, `two state words in ${row}`);
  assert.match(html, /\+3 more/);
  h.socket().send(JSON.stringify({kind: 'talos.drive.summary', session: '42', supervised: true,
    registry: false, omitted: 0, nodes: [node('shooter', {alive: false})]}));
  const unknown = widgetHtml(render(h), 'Subsystems');
  assert.match(unknown, /shooter<\/span><span class="tag tag-neutral">unknown/);
  assert.doesNotMatch(unknown, /DOWN/);
});

// REQ-OPS-36: before any value page of a socket decoded (nothing it brought
// has changed a value) and within CONNECTING_GRACE_MS of its opening, a
// binary message that does not decode is counted and ignored, and so is a
// text message that does not decode at any time; the page keeps decoding
// what follows.
test('garbage binary before the first tick and garbage text are counted and ignored', () => {
  const h = harness();
  h.at(0);
  h.model.start(0);
  h.socket().onopen!();
  const valid = update(2, {'shooter/state/flywheel_velocity_rps': 60}, false)[0];
  h.socket().send(new Uint8Array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]),
                  valid.slice(0, valid.length - 5), new Uint8Array(0), valid);
  h.socket().onmessage!({data: 42});
  h.socket().send(...encodeDictionaryPages(header(0), CHANNELS), ...update(0, NOMINAL, true),
                  ...update(1, {}, false));
  h.at(50);
  h.socket().send('{not json', '[]', '{"kind":"talos.drive.summary","registry":true,"supervised":true,"nodes":5}',
                  '{"kind":"talos.drive.something_new"}');
  assert.deepEqual(h.model.counters,
                   {connects: 1, malformed_binary: 5, malformed_text: 3, ignored_text: 1});
  assert.equal(h.model.connection, 'open');
  h.socket().send(valid);
  assert.match(widgetHtml(render(h), 'Flywheel'), /60\.0/);
  assert.doesNotMatch(widgetHtml(render(h), 'Flywheel'), /STALE/);
  assert.match(renderStatus(h.model, h.clock.now), /8 malformed messages counted/);
});

// REQ-OPS-36: after a socket's first tick the updates carry only changes, so
// a page that does not decode loses what it changed, and later updates would
// advance the sequence over a wrong value. Scenario: the flywheel goes to 60
// in a page that arrives corrupt; the page must close the socket (counted),
// reconnect, and read nothing current until the new socket's keyframe and
// the tick after it.
test('a binary message that does not decode after the first tick reconnects', () => {
  const h = harness();
  h.connect(0);
  h.at(50);
  const lost = update(2, {'shooter/state/flywheel_velocity_rps': 60}, false)[0];
  const first = h.socket();
  first.send(lost.slice(0, lost.length - 5));
  assert.equal(h.model.counters.malformed_binary, 1);
  assert.ok(first.closed, 'the socket is kept after a lost change-only update');
  first.onmessage?.({data: update(3, {}, false)[0].slice().buffer});
  assert.match(widgetHtml(render(h), 'Flywheel'), /STALE/, 'current after a lost update');
  for (let t = 100; h.sockets.length < 2; t += 100) h.at(t);
  h.socket().onopen!();
  h.socket().send(...encodeDictionaryPages(header(3), CHANNELS),
                  ...update(3, {...NOMINAL, 'shooter/state/flywheel_velocity_rps': 60}, true));
  assert.match(widgetHtml(render(h), 'Flywheel'), /STALE/, 'the keyframe is shown current');
  h.at(h.clock.now + 100);
  h.socket().send(...update(4, {}, false));
  const flywheel = widgetHtml(render(h), 'Flywheel');
  assert.doesNotMatch(flywheel, /STALE/);
  assert.match(flywheel, /60\.0/);
});

// REQ-OPS-36: a feed the page cannot decode is not a lost link. Scenario: a
// bridge sends a page this page's decoder refuses after each connection's
// first ticks. The status line names the telemetry feed, not the
// dashboard-to-bridge link (which sends the drive team to power and
// Ethernet), and after 3 such closes in a row the page retries every
// DECODE_RETRY_MS instead of every second, so it does not spend the slot's
// byte budget on dictionaries.
test('a decode failure that repeats names the feed and backs off its retries', () => {
  const h = harness();
  const garbage = new Uint8Array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]);
  const starts: number[] = [];
  h.connect(0);
  h.socket().send(garbage);
  assert.ok(h.model.status(h.clock.now).undecodable,
            `a decode failure reads as ${h.model.status(h.clock.now).text}`);
  for (let t = 100; t <= 12_000; t += 100) {
    const before = h.sockets.length;
    h.at(t);
    if (h.sockets.length === before) continue;
    starts.push(t);
    h.socket().onopen!();
    h.socket().send(...encodeDictionaryPages(header(t), CHANNELS), ...update(t, NOMINAL, true),
                    ...update(t + 1, {}, false), garbage);
    assert.ok(h.model.status(h.clock.now).undecodable, `${t} ms: ${h.model.status(h.clock.now).text}`);
  }
  assert.deepEqual(starts, [RECONNECT_MS, 2 * RECONNECT_MS, 2 * RECONNECT_MS + DECODE_RETRY_MS,
                            2 * RECONNECT_MS + 2 * DECODE_RETRY_MS]);
});

// REQ-OPS-36: the decode failure is named only until a socket shows the feed
// decodes, or shows nothing undecodable. Scenario: one decode close, the
// reconnect opens, and the telemetry node has stopped. Either the bridge
// sends its held tick's dictionary and keyframe (they decode: the line names
// the stalled telemetry, not the decoder), or it sends heartbeats only (past
// CONNECTING_GRACE_MS with no undecodable page, the line says no telemetry).
// "reload the page" must not stick while the socket stays open.
test('a clean reconnect after a decode failure ends the decode status', () => {
  const garbage = new Uint8Array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]);
  for (const keyframe of [true, false]) {
    const h = harness();
    h.connect(0);
    h.socket().send(garbage);
    assert.ok(h.model.status(h.clock.now).undecodable);
    h.at(RECONNECT_MS);
    h.socket().onopen!();
    if (keyframe)
      h.socket().send(...encodeDictionaryPages(header(1), CHANNELS), ...update(1, NOMINAL, true));
    for (let t = RECONNECT_MS + 100; t <= RECONNECT_MS + 3000; t += 100) {
      h.at(t);
      h.socket().send(keyframe ? update(1, {}, false)[0] : '{"kind":"talos.drive.heartbeat"}');
      const status = h.model.status(t);
      const label = `${keyframe ? 'keyframe' : 'heartbeats only'}, ${t} ms: ${status.text}`;
      if (keyframe || t - RECONNECT_MS > CONNECTING_GRACE_MS) {
        assert.ok(!status.undecodable, `${label}: still names a decode failure`);
        if (t - RECONNECT_MS > CONNECTING_GRACE_MS) assert.equal(status.link, 'telemetry', label);
      }
    }
  }
});

// REQ-OPS-36: a page from another bridge version, whose every page (the
// dictionary included) fails verification, is named as a feed that could not
// be decoded, not as a telemetry node publishing nothing (which sends the
// team to the wrong node), and backed off like any undecodable feed.
// Scenario: the bridge sends a TLM2 dictionary and keyframe on each connect
// and an update every 100 ms. A corrupt message before the dictionary, within
// CONNECTING_GRACE_MS of the socket opening, is still only counted (the
// garbage test above); one after it, with no value page decoded, closes the
// socket for a decode failure.
test('a feed none of whose pages decode is named and backed off', () => {
  const h = harness();
  const foreign = (pages: Uint8Array[]) => pages.map(page => {
    const copy = page.slice();
    copy.set([...'TLM2'].map(c => c.charCodeAt(0)), 8);
    return copy;
  });
  const starts: number[] = [];
  let named: number|null = null;
  for (let t = 0; t <= 14_000; t += 100) {
    const before = h.sockets.length;
    h.at(t);
    if (t === 0) h.model.start(0);
    if (h.sockets.length !== before) {
      starts.push(t);
      h.socket().onopen!();
      h.socket().send(...foreign([...encodeDictionaryPages(header(t), CHANNELS),
                                  ...update(t, NOMINAL, true)]));
    } else if (h.model.connection === 'open') {
      h.socket().send(...foreign(update(t + 1, {}, false)));
    }
    const status = h.model.status(t);
    if (named === null && status.undecodable) named = t;
    if (named !== null)
      assert.ok(status.undecodable, `${t} ms: the undecodable feed reads as ${status.text}`);
  }
  assert.equal(named, CONNECTING_GRACE_MS + 100,
               `an undecodable feed is named at ${named} ms, ${h.model.status(h.clock.now).text}`);
  assert.deepEqual(starts, [0, 1600, 3200, 3200 + DECODE_RETRY_MS, 3200 + 2 * DECODE_RETRY_MS]);
});

// REQ-OPS-36: once named, a feed that stays undecodable stays named on every
// reconnect. Scenario: as above, but the line is read every 50 ms, between
// the bridge's 100 ms updates. On each reconnect an update arrives exactly
// CONNECTING_GRACE_MS after the socket opened (only counted, being within
// the grace) and the next one, 100 ms later, closes it: in between, the line
// must not tell the team "no telemetry" (look at the telemetry node) instead
// of "reload the page".
test('an undecodable feed stays named across the grace of each reconnect', () => {
  const h = harness();
  const foreign = (pages: Uint8Array[]) => pages.map(page => {
    const copy = page.slice();
    copy.set([...'TLM2'].map(c => c.charCodeAt(0)), 8);
    return copy;
  });
  let opened = 0;
  let named = false;
  let reconnects = 0;
  for (let t = 0; t <= 14_000; t += 50) {
    const before = h.sockets.length;
    h.at(t);
    if (t === 0) h.model.start(0);
    if (h.sockets.length !== before) {
      opened = t;
      if (named) reconnects++;
      h.socket().onopen!();
      h.socket().send(...foreign([...encodeDictionaryPages(header(t), CHANNELS),
                                  ...update(t, NOMINAL, true)]));
    } else if (h.model.connection === 'open' && (t - opened) % 100 === 0) {
      h.socket().send(...foreign(update(t + 1, {}, false)));
    }
    const status = h.model.status(t);
    if (status.undecodable) named = true;
    else if (named) assert.fail(`${t} ms (${t - opened} ms after open): the named feed reads as ${status.text}`);
  }
  assert.ok(reconnects >= 3, `${reconnects} reconnects after the feed was named`);
});

// REQ-OPS-36: a corrupt binary page within CONNECTING_GRACE_MS followed only by
// heartbeats (no valid value page ever decodes on the socket) fails once the
// grace expires, reports the undecodable feed, and backs off reconnects after
// DECODE_RETRIES consecutive failures. If a valid dictionary and value page
// arrive before grace expires, the socket recovers and stays open past grace.
test('a corrupt binary page within grace followed only by heartbeats fails after grace', () => {
  const dictPage = encodeDictionaryPages(header(0), CHANNELS)[0];
  const corruptDict = dictPage.slice(0, dictPage.length - 5);
  const heartbeat = '{"kind":"talos.drive.heartbeat"}';

  const h = harness();
  const starts: number[] = [];
  for (let t = 0; t <= 9000; t += 100) {
    const before = h.sockets.length;
    h.at(t);
    if (t === 0) h.model.start(0);
    if (h.sockets.length !== before) {
      starts.push(t);
      h.socket().onopen!();
      h.socket().send(corruptDict);
    } else if (h.model.connection === 'open') {
      h.socket().send(heartbeat);
    }
    if (t <= CONNECTING_GRACE_MS) {
      assert.equal(h.sockets.length, 1, `closed early at ${t} ms within grace`);
      assert.equal(h.sockets[0].closed, false, `socket closed early at ${t} ms`);
    } else {
      const status = h.model.status(t);
      assert.ok(status.undecodable, `${t} ms: expected undecodable status, got "${status.text}"`);
      assert.match(status.text, /Telemetry feed could not be decoded/);
    }
  }
  assert.ok(h.sockets[0].closed, 'first socket stayed open past grace after corrupt dictionary');
  assert.deepEqual(starts, [0, 1600, 3200, 3200 + DECODE_RETRY_MS]);

  // Recovery within grace: a corrupt page followed by a valid dictionary and
  // value page before grace ends keeps the socket open past CONNECTING_GRACE_MS.
  const recovered = harness();
  recovered.at(0);
  recovered.model.start(0);
  recovered.socket().onopen!();
  recovered.socket().send(corruptDict);
  recovered.at(100);
  recovered.socket().send(...encodeDictionaryPages(header(0), CHANNELS),
                          ...update(0, NOMINAL, true), ...update(1, {}, false));
  for (let t = 200; t <= CONNECTING_GRACE_MS + 500; t += 100) {
    recovered.at(t);
    recovered.socket().send(...update(t / 100, {}, false));
  }
  assert.equal(recovered.sockets.length, 1, 'recovered socket reconnected after grace');
  assert.equal(recovered.sockets[0].closed, false, 'recovered socket closed after grace');
  assert.equal(recovered.model.status(recovered.clock.now).link, 'ok');
});

// REQ-OPS-36: a keyframe of several pages applies each as it arrives, the
// first clearing every channel. A later page that does not decode, before
// the socket's first tick completes, would leave the channels of that page
// absent (a false HARDWARE BRIDGE, "no data"): the socket is failed and
// reconnects.
test('a keyframe page that does not decode after an earlier one applied fails the socket', () => {
  const h = harness();
  h.model.start(0);
  h.socket().onopen!();
  const flywheel = INDEX.get('shooter/state/flywheel_velocity_rps')!;
  const padding = Array.from({length: 2000}, () => [flywheel, rawFromDouble(1)] as [number, bigint]);
  const pages = encodeValuePages(header(5), POSE, true, padding);
  assert.ok(pages.length >= 2, 'the keyframe fits one page');
  h.socket().send(...encodeDictionaryPages(header(5), CHANNELS), pages[0],
                  pages[1].slice(0, pages[1].length - 5));
  assert.equal(h.model.counters.malformed_binary, 1);
  assert.ok(h.sockets[0].closed, 'the socket is kept with a keyframe half applied');
});

// REQ-OPS-36: with no TCP close (Wi-Fi drop, robot off) attempts still start
// every 1 s: a connect that has not opened by then is abandoned for the
// next, one socket at a time, and the status line names the bridge link once
// the first connect's grace is over.
test('a connect that never opens is abandoned and retried every second', () => {
  const h = harness();
  const starts: number[] = [];
  h.model.start(0);
  for (let t = 0; t <= 2500; t += 100) {
    h.at(t);
    while (starts.length < h.sockets.length) starts.push(t);
  }
  assert.deepEqual(starts, [0, RECONNECT_MS, 2 * RECONNECT_MS]);
  assert.ok(h.sockets.slice(0, -1).every(s => s.closed), 'an abandoned connect is not closed');
  assert.equal(h.model.status(h.clock.now).link, 'bridge');
});

// REQ-OPS-36: an open socket that hears nothing for DEAD_AFTER_MS (the
// bridge sends something every 100 ms) is a dead path: it is closed and a
// new attempt starts, without waiting for the OS to time it out.
test('an open socket silent for 2 s is torn down and replaced', () => {
  const h = harness();
  h.connect(0);
  h.at(DEAD_AFTER_MS - 1);
  assert.equal(h.sockets[0].closed, false, 'closed while within the silence bound');
  h.at(DEAD_AFTER_MS);
  assert.ok(h.sockets[0].closed, 'a silent socket is kept open');
  assert.equal(h.sockets.length, 2, 'no new attempt after a silent socket');
});

// REQ-OPS-36: the summary is shown current only while it was sent on the
// socket that is now heard; after a reconnect the old one reads STALE until
// the bridge sends one on the new socket.
test('a summary from a previous socket reads stale until a new one arrives', () => {
  const summary = JSON.stringify({kind: 'talos.drive.summary', session: '42', supervised: true,
                                  registry: true, omitted: 0, nodes: []});
  const h = harness();
  h.connect(0);
  h.socket().send(summary);
  assert.doesNotMatch(widgetHtml(render(h), 'Subsystems'), /STALE/);
  h.socket().onclose!({code: 1006, reason: ''});
  for (let t = 100; h.sockets.length < 2; t += 100) h.at(t);
  h.socket().onopen!();
  h.socket().send(...encodeDictionaryPages(header(3), CHANNELS), ...update(3, NOMINAL, true),
                  ...update(4, {}, false));
  assert.match(widgetHtml(render(h), 'Subsystems'), /STALE/,
               'a summary from the previous socket is shown current');
  h.socket().send(summary);
  assert.doesNotMatch(widgetHtml(render(h), 'Subsystems'), /STALE/);
});

// REQ-OPS-49: every string from the layout, the dictionary or the summary is
// escaped where it is drawn: a name with markup in it is shown as text and never
// becomes markup on the page the drive team reads.
test('markup in every drawn string is escaped', () => {
  const markers = {
    title: '<x-title>', check: '<x-check>', enum: '<x-enum>', alert: '<x-alert>',
    owner: '<x-owner>', session: '<x-session>', node: '<x-node>', drives: '<x-drives>'
  };
  const mode = 'e/state/mode';
  const alarm = `${markers.owner}/status/alert_error_${markers.alert}`;
  const channels: ChannelSpec[] = [
    {name: mode, unit: '', kind: ChannelKind.UInt, labels: [markers.enum]},
    {name: alarm, unit: '', kind: ChannelKind.Bool, alert: AlertLevel.Error}
  ];
  const head = {streamId: 9n, sequenceId: 1n, timestampNs: 1n,
                dictionaryId: dictionaryId(channels), channelCount: channels.length};
  const layout = parseLayout({columns: 4, rows: 1, widgets: [
    {kind: 'text', at: [0, 0, 1, 1], title: markers.title, channels: {channel: mode}},
    {kind: 'checks', at: [1, 0, 1, 1], title: '', channels: {},
     items: [{label: markers.check, channel: mode}]},
    {kind: 'alerts', at: [2, 0, 1, 1], title: '', channels: {}},
    {kind: 'subsystems', at: [3, 0, 1, 1], title: '', channels: {}, nodes: []},
  ]});
  const h = harness();
  h.model.start(0);
  h.socket().onopen!();
  h.socket().send(...encodeDictionaryPages(head, channels),
                  ...encodeValuePages(head, POSE, true, [[0, 0n], [1, 1n]]),
                  ...encodeValuePages({...head, sequenceId: 2n}, POSE, false, []),
                  JSON.stringify({kind: 'talos.drive.summary', session: markers.session,
                                  supervised: true, registry: true, omitted: 0,
                                  nodes: [{name: markers.node, alive: true, critical: true,
                                           restarts: 0, quarantined: false,
                                           drives: markers.drives}]}));
  const html = renderDashboard(layout, h.model, 0);
  for (const [site, marker] of Object.entries(markers)) {
    assert.ok(!html.includes(marker), `${site}: ${marker} drawn as markup`);
    assert.ok(html.includes(esc(marker)), `${site}: ${marker} not drawn as text`);
  }
});

// REQ-OPS-49: a subsystems widget that names its nodes shows each of them. A
// named node the summary does not list (cut off by `omitted`, or not in the
// session) reads unknown, never vanishes, and "+N more" never counts nodes
// the widget would not show.
test('a subsystems widget with a node filter shows every named node', () => {
  const layout = parseLayout({columns: 1, rows: 1, widgets: [
    {kind: 'subsystems', at: [0, 0, 1, 1], title: '', channels: {}, nodes: ['shooter', 'intake']}]});
  const node = (name: string) => ({name, alive: true, critical: true, restarts: 0,
                                   quarantined: false, drives: null});
  const h = harness();
  h.connect(0);
  h.socket().send(JSON.stringify({kind: 'talos.drive.summary', session: '42', supervised: true,
                                  registry: true, omitted: 3,
                                  nodes: [node('drivetrain'), node('shooter')]}));
  const html = renderDashboard(layout, h.model, 0);
  assert.match(html, /shooter<\/span><span class="tag tag-good">UP/);
  assert.match(html, /intake<\/span><span class="tag tag-neutral">unknown/,
               'a named node missing from the summary is not shown as unknown');
  assert.doesNotMatch(html, /drivetrain/, 'a node the filter does not name is shown');
  assert.doesNotMatch(html, /more</, '"+N more" counts nodes the filter does not admit');
});

// REQ-OPS-49: at 1366x768 (a Driver Station laptop) every row of the 2026
// layout's lists is visible without scrolling: the subsystems cell holds 14
// nodes (the 13 of a 2026 session and one spare) and the pre-match cell all
// its checks. At 1280x720 (a smaller laptop) the subsystems cell is 294 px
// wide: with columns of at least 150 px it fell to one column of 9 rows, so
// the minimum is low enough for two there too, and every row still fits.
// Computed from the page's box sizes, which the page writes as the custom
// properties drive.css lays out with; a browser screenshot verifies it.
test('the 2026 lists fit their cells at 1366x768 and 1280x720', () => {
  const subsystems = LAYOUT.widgets.find(w => w.kind === 'subsystems')!;
  const checks = LAYOUT.widgets.find(w => w.kind === 'checks')!;
  assert.equal(checks.kind, 'checks');
  for (const page of [{width: 1366, height: 768}, {width: 1280, height: 720}]) {
    const size = `${page.width}x${page.height}`;
    const nodeRows = listRowsFit(LAYOUT, subsystems.at, page, METRICS.node_column);
    assert.ok(nodeRows >= 14,
              `${size}: subsystems at ${JSON.stringify(subsystems.at)} shows ${nodeRows} rows, needs 14`);
    const checkRows = listRowsFit(LAYOUT, checks.at, page, METRICS.check_column);
    assert.ok(checkRows >= checks.items.length,
              `${size}: checks at ${JSON.stringify(checks.at)} shows ${checkRows} rows, needs ${checks.items.length}`);
  }
  // The stylesheet lays out with exactly these sizes: it reads every one and
  // no other --drive-* property.
  const css = readFileSync(new URL('../src/drive/drive.css', import.meta.url), 'utf8');
  const read = new Set([...css.matchAll(/var\(--drive-([a-z-]+)\)/g)].map(m => m[1]));
  assert.deepEqual([...read].sort(),
                   Object.keys(METRICS).map(k => k.replace(/_/g, '-')).sort());
});

// REQ-OPS-49: each row keeps its name readable beside its state. At 1366x768
// and 1280x720 the 2026 subsystems cell gives two columns; the worst-case row
// (each state word, with its note of restarts and drives) still shows the
// name's floor (so the row's edge, which clips and does not ellipsize, never
// reaches the word), and enough of each 2026 node name to tell them apart
// (front_left_camera from front_right_camera). A model: glyph widths are
// GLYPH_PX, not a browser's; a browser screenshot verifies it.
test('a 2026 subsystems row keeps a readable name beside its widest state word', () => {
  const names = ['hardware_node', 'autonomous', 'drivetrain', 'intake', 'driver', 'pose_estimator',
                 'shooter', 'telemetry', 'teleop', 'front_left_camera', 'front_right_camera',
                 'rear_left_camera', 'rear_right_camera'];
  // The shortest prefix length that tells every name apart.
  let distinct = 1;
  while (new Set(names.map(n => n.slice(0, distinct))).size < names.length) distinct++;
  const floor = METRICS.name_min / GLYPH_PX.name;  // the name's floor, in characters
  assert.ok(floor - 1 >= distinct, 'the name floor cannot tell the 2026 nodes apart');
  const subsystems = LAYOUT.widgets.find(w => w.kind === 'subsystems')!;
  for (const page of [{width: 1366, height: 768}, {width: 1280, height: 720}]) {
    const column = listColumns(LAYOUT, subsystems.at, page, METRICS.node_column);
    for (const word of Object.values(NODE_STATE)) {
      const at = `${page.width}x${page.height}, ${word}, a ${column.width.toFixed(1)} px column`;
      const shown = nameCharsShown(column.width, word, 1);
      assert.ok(shown + 1 >= floor,
                `${at}: the name keeps ${shown} characters, under its floor of ` +
                `${floor}, so the row cuts the word`);
      assert.ok(shown >= distinct, `${at}: the name shows ${shown} characters, ${distinct} ` +
                                   'tell the 2026 nodes apart');
    }
  }
});

// REQ-OPS-49: the page never hides a row's state word. The stylesheet lets a
// list row's name and notes give up width (ellipsized, the tooltip has the
// rest) but never its state word: a flex item that may shrink takes a share
// of any overflow in proportion to its size, so a word allowed to shrink at
// all ellipsized ("U…" for UP beside front_right_camera at 1280x720, a
// 29.19 px box for a 29.22 px word). That the row's edge does not cut the
// word either is the readable-name test above: the name's floor, read here
// from the same property, leaves the widest word its width.
test('a list row never shrinks its state word', () => {
  const css = readFileSync(new URL('../src/drive/drive.css', import.meta.url), 'utf8')
                  .replace(/\/\*[\s\S]*?\*\//g, '');
  // Every rule that can reach a list row's state word: one naming the word
  // in a list, every child of a list row, or every word.
  const reaches = (s: string) => s.trim() === '.tag' ||
      /\.(nodes|checks|alerts)\b.*(\.tag\b|li\s*>\s*\*)/.test(s);
  const wordRules = [...css.matchAll(/([^{}]+)\{([^}]*)\}/g)]
                        .filter(([, sel]) => sel.split(',').some(reaches));
  assert.ok(wordRules.length >= 2, 'no rule reaches a list row\'s state word: the selectors moved');
  for (const [, sel, body] of wordRules) {
    const rule = `"${sel.trim().replace(/\s+/g, ' ')}"`;
    const flex = /(?:^|;)\s*flex\s*:\s*([^;]+)/.exec(body)?.[1].trim();
    const shrink = /flex-shrink\s*:\s*([^;]+)/.exec(body)?.[1].trim();
    assert.ok(flex === undefined || flex === 'none' || /^\S+\s+0(\s|$)/.test(flex),
              `${rule} lets the state word shrink (flex: ${flex})`);
    assert.ok(shrink === undefined || shrink === '0', `${rule} sets flex-shrink: ${shrink}`);
    assert.doesNotMatch(body, /text-overflow\s*:\s*ellipsis/, `${rule} ellipsizes the state word`);
    assert.doesNotMatch(body, /overflow\s*:\s*hidden/, `${rule} hides the state word's overflow`);
  }
});

// REQ-OPS-49: when the subsystems list is longer than its cell, the rows
// that matter come first: failed nodes (critical before not), then one
// driving from a target lane, then critical ones, then the rest.
test('subsystems rows put failed and driving nodes first', () => {
  const node = (name: string, extra: object) => ({name, alive: true, critical: false, restarts: 0,
                                                  quarantined: false, drives: null, ...extra});
  const h = harness();
  h.connect(0);
  h.socket().send(JSON.stringify({kind: 'talos.drive.summary', session: '42', supervised: true,
    registry: true, omitted: 0, nodes: [
      node('camera', {}), node('telemetry', {critical: true}),
      node('drivetrain', {critical: true, drives: 'teleop'}), node('vision', {alive: false}),
      node('shooter', {critical: true, alive: false})]}));
  const html = widgetHtml(render(h), 'Subsystems');
  const order = [...html.matchAll(/<span class="name"[^>]*>([^<]*)</g)].map(m => m[1]);
  assert.deepEqual(order, ['shooter', 'vision', 'drivetrain', 'telemetry', 'camera']);
});

// REQ-SAFE-41: a live subsystem whose devices flush is holding outputs with no
// live target (`outputs_held: true`) shows HELD (`tag-warn`) with one state
// word per row, names "outputs held: no live target" in its note and title,
// yields to FAILING / unknown / DOWN when those apply, and ranks right after
// failed nodes ahead of driving and idle subsystems.
test('subsystems widget renders outputs_held as HELD, ranked right after failed nodes', () => {
  const node = (name: string, extra: object) => ({name, alive: true, critical: true, restarts: 0,
                                                  quarantined: false, drives: null, ...extra});
  const h = harness();
  h.connect(0);
  h.socket().send(JSON.stringify({kind: 'talos.drive.summary', session: '42', supervised: true,
    registry: true, omitted: 0, nodes: [
      node('camera', {critical: false}),
      node('drivetrain', {drives: 'teleop'}),
      node('intake', {outputs_held: true, restarts: 1}),
      node('vision', {critical: false, alive: false}),
      node('shooter', {alive: false, quarantined: true, outputs_held: true}),
      node('elevator', {alive: false, outputs_held: true})]}));
  const html = widgetHtml(render(h), 'Subsystems');
  assert.match(html,
               /title="intake · outputs held: no live target · restarts 1">intake<\/span><span class="tag tag-warn">HELD<\/span><span class="meta">outputs held: no live target · restarts 1<\/span>/);
  assert.match(html, /shooter<\/span><span class="tag tag-bad">FAILING<\/span>/);
  assert.match(html, /elevator<\/span><span class="tag tag-bad">DOWN<\/span>/);
  assert.match(html, /drivetrain<\/span><span class="tag tag-good">UP<\/span>/);
  for (const row of html.match(/<li>.*?<\/li>/g) ?? [])
    assert.ok((row.match(/class="tag /g) ?? []).length <= 1, `two state words in ${row}`);
  const order = [...html.matchAll(/<span class="name"[^>]*>([^<]*)</g)].map(m => m[1]);
  assert.deepEqual(order, ['shooter', 'elevator', 'vision', 'intake', 'drivetrain', 'camera']);

  // Without a readable registry, unknown wins over outputs_held and no held note shows.
  h.socket().send(JSON.stringify({kind: 'talos.drive.summary', session: '42', supervised: true,
    registry: false, omitted: 0, nodes: [node('intake', {outputs_held: true})]}));
  const unread = widgetHtml(render(h), 'Subsystems');
  assert.match(unread, /intake<\/span><span class="tag tag-neutral">unknown<\/span>/);
  assert.doesNotMatch(unread, /HELD|outputs held/);
});

// REQ-OPS-49: listRowsFit's box model assumes drive.css keeps zero margin and
// padding on lists and their items, uses --drive-list-gap only between columns
// (no row-gap), and sets each list item's height to --drive-row and item gap
// to --drive-item-gap. Any extra spacing on these selectors would silently
// overflow the calculated row count.
test('list selectors keep the margin, padding, gap and height listRowsFit models', () => {
  const css = readFileSync(new URL('../src/drive/drive.css', import.meta.url), 'utf8')
                  .replace(/\/\*[\s\S]*?\*\//g, '');
  const rules = [...css.matchAll(/([^{}@]+)\{([^}]*)\}/g)].map(([, sel, body]) => ({
    selectors: sel.split(',').map(s => s.trim().replace(/\s+/g, ' ')),
    body
  }));
  const declsFor = (selector: string) => {
    const out = new Map<string, string>();
    for (const r of rules) {
      if (!r.selectors.includes(selector)) continue;
      for (const part of r.body.split(';')) {
        const colon = part.indexOf(':');
        if (colon < 0) continue;
        out.set(part.slice(0, colon).trim(), part.slice(colon + 1).trim());
      }
    }
    return out;
  };
  const ul = declsFor('ul');
  assert.equal(ul.get('margin'), '0', 'ul must have margin: 0');
  assert.equal(ul.get('padding'), '0', 'ul must have padding: 0');

  for (const list of ['.nodes', '.checks']) {
    const d = declsFor(list);
    assert.equal(d.get('column-gap'), 'var(--drive-list-gap)', `${list} column-gap`);
    assert.equal(d.get('row-gap'), undefined, `${list} must not set row-gap`);
    assert.equal(d.get('gap'), undefined, `${list} must not set gap (adds row-gap)`);
    assert.equal(d.get('margin'), undefined, `${list} must not add margin`);
    assert.equal(d.get('padding'), undefined, `${list} must not add padding`);
  }

  for (const item of ['.alerts li', '.nodes li', '.checks li']) {
    const d = declsFor(item);
    assert.equal(d.get('height'), 'var(--drive-row)', `${item} height`);
    assert.equal(d.get('gap'), 'var(--drive-item-gap)', `${item} gap`);
    assert.equal(d.get('margin'), undefined, `${item} must not add margin`);
    assert.equal(d.get('padding'), undefined, `${item} must not add padding`);
  }
});

// A Surface of plain objects that lays out like a browser: `grid` holds the
// element at each index, so a test can tell a kept element from a replaced
// one by identity. Each element's list body holds its `<li>` rows, ROW px
// each, in a cell that leaves it ROOM px. A detached element has no layout
// box: its scroll offset reads 0 and setting one does nothing. A new body
// fills its cell (ROOM px) until fit() snaps it to whole rows, and its
// scroll offset is clamped to what it can scroll at that moment.
const ROW = METRICS.row, ROOM = 100;
type FakeElement = {html: string; top: number; attached: boolean; height: number; content: number};
function fakeSurface() {
  const page = {status: '', drawError: '', errored: false, grid: [] as FakeElement[], puts: 0};
  const surface: Surface<FakeElement> = {
    status: html => {
      page.status = html;
    },
    drawError: html => {
      page.drawError = html;
    },
    create: html => ({html, top: 0, attached: false, height: ROOM,
                      content: (html.match(/<li>/g)?.length ?? 0) * ROW}),
    put: (index, next, previous) => {
      if (previous) previous.attached = false;
      next.attached = true;
      page.grid[index] = next;
      page.puts++;
    },
    fit: e => {
      if (e.attached) e.height = Math.floor(ROOM / ROW) * ROW;
    },
    scrollTop: e => e.attached ? e.top : 0,
    setScrollTop: (e, top) => {
      if (e.attached) e.top = Math.max(0, Math.min(top, e.content - e.height));
    },
    errored: on => {
      page.errored = on;
    }
  };
  return {page, surface};
}
const rows = (n: number) => `<section><ul>${'<li>row</li>'.repeat(n)}</ul></section>`;

// REQ-OPS-49: the page redraws every RENDER_MS and with a live producer some
// widget changes nearly every frame. A widget whose markup did not change
// keeps its element (a list's scroll position and a row's tooltip survive);
// one whose rows changed is replaced and keeps its list's scroll position.
// Scenario: the driver scrolls the subsystems list down and a restart count
// changes; the new list must not start at the top. Scrolled to its bottom,
// it must stay at its bottom (its last row whole), not lose the part of a
// row its body is shorter by once snapped to whole rows.
test('the painter replaces only a changed widget and keeps a list scrolled', () => {
  const {page, surface} = fakeSurface();
  const painter = new Painter(surface);
  // As the page does: each paint, then every list snapped to whole rows.
  const paint = (graph: string, nodes: string) => {
    painter.paint(() => '<div>OK</div>', () => [graph, nodes]);
    page.grid.forEach(e => surface.fit(e));
  };
  paint('<section>graph 1</section>', rows(10));
  const [graph, list] = page.grid;
  assert.ok(graph && list, 'the first paint did not draw every widget');
  surface.setScrollTop(list, 2 * ROW);  // the driver scrolled the list
  paint('<section>graph 2</section>', rows(10));
  assert.notEqual(page.grid[0], graph, 'a changed widget is not repainted');
  assert.equal(page.grid[1], list, 'an unchanged list is replaced, losing its scroll and hover');
  paint('<section>graph 2</section>', rows(11));
  assert.notEqual(page.grid[1], list, 'a list whose rows changed is not repainted');
  assert.equal(surface.scrollTop(page.grid[1]), 2 * ROW, 'a repainted list returns to its top');
  // Scrolled to the bottom: 11 rows in a body of 4 whole rows.
  const bottom = 11 * ROW - Math.floor(ROOM / ROW) * ROW;
  surface.setScrollTop(page.grid[1], bottom);
  assert.equal(surface.scrollTop(page.grid[1]), bottom);
  paint('<section>graph 2</section>', `${rows(11)} `);
  assert.equal(surface.scrollTop(page.grid[1]), bottom,
               'a list scrolled to its bottom no longer is after a repaint (its last row cut)');
});

// REQ-OPS-49: a list that scrolls says how many rows are out of view. Rows
// 22 px tall in two columns, 14 rows on 7 lines, a body 100 px tall scrolled
// to 30: lines at 0 and 22 are above (cut by the top edge counts), 44 to 88
// show, 110 and 132 are below.
test('the out-of-view count of a scrolled list', () => {
  const tops = [0, 1, 2, 3, 4, 5, 6].flatMap(line => [line * 22, line * 22]);
  assert.deepEqual(outOfView(tops, 22, 100, 0), {above: 0, below: 6});
  assert.deepEqual(outOfView(tops, 22, 100, 30), {above: 4, below: 4});
  assert.deepEqual(outOfView(tops, 22, 154, 0), {above: 0, below: 0});
  assert.equal(outOfViewText({above: 4, below: 4}), '4 above · +4 below');
  assert.equal(outOfViewText({above: 0, below: 0}), '');
});

// REQ-OPS-49, REQ-OPS-36: a widget drawing fault keeps the top status line
// visible (and live as link status changes), shows a DRAWING ERROR row
// telling the operator to reload, and marks the grid while keeping its last
// elements. If drawing the status line itself throws, the status slot shows
// DRAWING ERROR; the next good draw clears the mark and repaints cleanly.
test('a drawing error keeps the status line, says to reload, and clears on recovery', () => {
  const {page, surface} = fakeSurface();
  const painter = new Painter(surface);
  const okStatus = '<div class="status link-ok"><b>OK</b> Connected</div>';
  const lostStatus = '<div class="status link-roborio"><b>ROBORIO LINK</b> RoboRIO link lost</div>';
  const goodWidgets = () => ['<section>grid</section>'];

  painter.paint(() => okStatus, goodWidgets);
  const shown = page.grid[0];
  // A widget throws while the RoboRIO link also drops: the status line
  // updates and stays visible, while the DRAWING ERROR row tells the driver
  // to reload.
  painter.paint(() => lostStatus, () => {
    throw new Error('<x-fault>');
  });
  assert.equal(page.errored, true, 'a drawing error leaves the grid unmarked');
  assert.equal(page.grid[0], shown);
  assert.equal(page.status, lostStatus, 'a widget drawing error overwrote the top status line');
  assert.match(page.drawError, /link-error/);
  assert.match(page.drawError, /DRAWING ERROR/);
  assert.match(page.drawError, /reload the page/i);
  assert.ok(page.drawError.includes(esc('<x-fault>')), page.drawError);

  // Recovery clears the DRAWING ERROR row and grid mark.
  painter.paint(() => okStatus, goodWidgets);
  assert.equal(page.errored, false);
  assert.equal(page.drawError, '', 'the DRAWING ERROR row was not cleared on recovery');
  assert.equal(page.status, okStatus);

  // If drawing the status line itself throws, the status slot falls back to
  // DRAWING ERROR telling the operator to reload, and recovers on the next
  // good draw even when the status text matches the pre-fault status.
  painter.paint(() => {
    throw new Error('<x-status-fault>');
  }, goodWidgets);
  assert.equal(page.errored, true);
  assert.equal(page.drawError, '');
  const fallbackStatus: string = page.status;
  assert.match(fallbackStatus, /link-error.*DRAWING ERROR.*reload the page/i);
  assert.ok(fallbackStatus.includes(esc('<x-status-fault>')), fallbackStatus);
  painter.paint(() => okStatus, goodWidgets);
  assert.equal(page.errored, false);
  assert.equal(page.status, okStatus, 'the status line was not repainted after a status drawing error');
});

// REQ-OPS-34, REQ-OPS-36, TODO.md DEPLOY-12: when the bridge runs in refusal
// mode (`--refusal-file`) and sends a `talos.drive.summary` with
// `launch_refused: true` and `refusal_reason`, `/drive` renders a clear
// `LAUNCH REFUSED` banner with the exact escaped refusal reason instead of
// blaming `BRIDGE LINK` or `TELEMETRY`, even with no telemetry ticks.
test('a refused launch renders LAUNCH REFUSED with its reason instead of BRIDGE LINK', () => {
  const minimalLayout = parseLayout({
    title: 'Drive',
    columns: 1,
    rows: 1,
    update_hz: 5,
    alerts: false,
    channels: [],
    widgets: []
  });
  const h = harness(minimalLayout);
  h.model.start(0);
  h.socket().onopen!();
  const reason = 'error: motor_py: unread parameter key <unread.value_m>';
  h.socket().send(JSON.stringify({
    kind: 'talos.drive.summary',
    session: '',
    supervised: false,
    registry: false,
    launch_refused: true,
    refusal_reason: reason,
    nodes: [],
    omitted: 0
  }));

  // Immediate status and past CONNECTING_GRACE_MS with only heartbeats: stays
  // LAUNCH REFUSED, never BRIDGE LINK or TELEMETRY.
  for (let t = 0; t <= CONNECTING_GRACE_MS + 500; t += 100) {
    h.at(t);
    if (t > 0) h.socket().send('{"kind":"talos.drive.heartbeat"}');
    const status = h.model.status(t);
    assert.equal(status.link, 'refused', `${t} ms: got ${status.link} (${status.text})`);
    assert.equal(status.text, reason);
  }

  const statusHtml = renderStatus(h.model, h.clock.now);
  assert.match(statusHtml, /class="status link-refused"/);
  assert.match(statusHtml, /<b>LAUNCH REFUSED<\/b>/);
  assert.ok(statusHtml.includes(esc(reason)), statusHtml);
  assert.doesNotMatch(statusHtml, /BRIDGE LINK/);
  assert.doesNotMatch(statusHtml, /TELEMETRY/);

  const pageHtml = renderDashboard(minimalLayout, h.model, h.clock.now);
  assert.match(pageHtml, /class="w refusal-banner"/);
  assert.match(pageHtml, /LAUNCH REFUSED/);
  assert.ok(pageHtml.includes(esc(reason)), pageHtml);
  assert.doesNotMatch(pageHtml, /BRIDGE LINK/);

  // After a reconnect to a healthy session, LAUNCH REFUSED clears.
  h.socket().onclose!({code: 1006, reason: ''});
  h.at(h.clock.now + RECONNECT_MS);
  h.socket().onopen!();
  h.socket().send(
      ...encodeDictionaryPages(header(1), CHANNELS),
      ...update(1, NOMINAL, true),
      ...update(2, {}, false));
  assert.equal(h.model.status(h.clock.now).link, 'ok');
});
