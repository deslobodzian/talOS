// The driver dashboard page (src/drive): its feed model on an injected clock
// and a fake socket, and its rendering of the 2026 layout. The layout fixture
// is what the bridge serves at /drive/layout.json for
// 2026-robot/main_processor/configuration/dashboard.toml (DashboardLayoutJson,
// the shape //talOS/telemetry:dashboard_layout_test pins); the telemetry is
// real TLM3 pages, decoded by the same TelemetryDecoder Studio uses.

import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {test} from 'node:test';

import {AlertLevel, ChannelKind, type ChannelSpec, dictionaryId, encodeDictionaryPages, encodeValuePages, rawFromDouble} from '../src/encode';
import {CONNECTING_GRACE_MS, DriveModel, type DriveSocket, type Link, RECONNECT_MS, RENDER_MS, REPLACED_CODE, SILENCE_MS, STALE_MS} from '../src/drive/feed';
import {parseLayout} from '../src/drive/layout';
import {renderDashboard, renderStatus} from '../src/drive/view';

const LAYOUT = parseLayout(JSON.parse(
    readFileSync(new URL('./drive_layout_2026.json', import.meta.url), 'utf8')));

const LINK_LOST = 'hw/status/alert_error_link_lost';
const OWNER_TIMED_OUT = 'hw/status/alert_warning_owner_timed_out';
const ROUTINES = ['preload', 'none', 'shoot_only', 'taxi'];

// The 2026 dictionary as far as the feed carries it: every layout channel
// plus the /hw/status alert channels the alerts widget adds.
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
  ...LAYOUT.channels.map(name => ({
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
function harness() {
  const sockets: FakeSocket[] = [];
  const tracked = ['shooter/state/flywheel_velocity_rps', 'shooter/state/target_velocity_rps',
                   'pose_estimator/state/pose/x_m', 'pose_estimator/state/pose/y_m'];
  const model = new DriveModel('ws://robot/drive', () => {
    // Never two sockets: every earlier one is closed before a new one opens.
    assert.ok(sockets.every(s => s.closed), 'a second socket opened while one was live');
    const s = new FakeSocket();
    sockets.push(s);
    return s;
  }, tracked);
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
  assert.equal(LAYOUT.widgets.length, 13);
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
  assert.match(checks, /tag-bad">FAIL<\/span><span class="name">Commissioned/);
});

// REQ-OPS-39: a kind this page does not draw is a visible cell, not a blank.
test('an unknown widget kind renders a visible unsupported cell', () => {
  const layout = parseLayout({columns: 4, rows: 2, widgets: [
    {kind: 'camera', at: [0, 0, 2, 1], title: '', channels: {}},
    {kind: 'number', at: [2, 0, 2, 1], title: 'Speed', channels: {}},
  ]});
  const h = harness();
  const html = renderDashboard(layout, h.model, 0);
  assert.match(html, /grid-column:1 \/ span 2;grid-row:1 \/ span 1[^>]*><header><h2>Unsupported widget<\/h2>/);
  assert.match(html, /Unsupported widget "camera"/);
  assert.match(html, /<h2>Speed<\/h2>.*no data/);
});

// REQ-OPS-36: a value is shown stale within 500 ms of its last update. The
// bridge's empty update re-sends the held tick; only a sequence_id that
// advances keeps the values current.
test('values go stale within 500 ms of the last new tick, whatever else arrives', () => {
  const h = harness();
  h.connect(0);
  // Heartbeat updates of the same tick every 100 ms: the producer stopped.
  for (let t = 100; t <= STALE_MS; t += 100) {
    h.at(t);
    h.socket().send(...update(1, {}, false));
  }
  assert.ok(!widgetHtml(render(h), 'Flywheel').includes('STALE'), 'stale while current');
  // The page redraws every RENDER_MS: the latest draw that can still show
  // the value current is STALE_MS after it, the first stale one within 500.
  assert.ok(STALE_MS + RENDER_MS <= 500, `${STALE_MS} + ${RENDER_MS} ms exceeds 500 ms`);
  h.at(STALE_MS + 1);
  const stale = widgetHtml(render(h), 'Flywheel');
  assert.match(stale, /class="w w-number stale"/);
  assert.match(stale, /tag-stale">STALE/);
  assert.match(stale, /52\.3/, 'a stale value is still shown, marked');
  // A heartbeat of a newer tick makes it current again.
  h.at(600);
  h.socket().send(...update(2, {}, false));
  assert.ok(!widgetHtml(render(h), 'Flywheel').includes('STALE'));
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
  assert.match(flywheel(), /52\.3/, 'the keyframe values are shown, marked');
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
  assert.match(held, /70\.0/, 'the keyframe does not replace the held values');
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
  const cases: {name: string; arrange: (h: ReturnType<typeof harness>) => void; link: Link}[] = [
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
  ];
  for (const c of cases) {
    const h = harness();
    c.arrange(h);
    const status = h.model.status(h.clock.now);
    assert.equal(status.link, c.link, `${c.name}: ${status.text}`);
    assert.match(renderStatus(h.model, h.clock.now), new RegExp(`link-${c.link}`), c.name);
  }
});

// REQ-OPS-36: on a close or an error the page reconnects every 1 s, one
// socket at a time.
test('reconnects every second after a close or an error, one socket at a time', () => {
  const h = harness();
  h.connect(0);
  h.at(200);
  h.socket().onclose!({code: 1006, reason: ''});
  h.at(200 + RECONNECT_MS - 1);
  assert.equal(h.sockets.length, 1, 'reconnected before 1 s');
  h.at(200 + RECONNECT_MS);
  assert.equal(h.sockets.length, 2, 'no reconnect at 1 s');
  // The attempt fails at once (the bridge is down): the next goes 1 s later.
  h.sockets[1].onerror!();
  assert.ok(h.sockets[1].closed, 'an errored socket is closed');
  h.at(1200 + RECONNECT_MS);
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

// The subsystems widget: what the drive summary says, with unknown shown as
// unknown and the cut-off list as "+N more".
test('the subsystems widget lists the summary, unknown as unknown', () => {
  const node = (name: string, extra: object) => ({name, alive: true, critical: true, restarts: 0,
                                                  quarantined: false, drives: null, ...extra});
  const h = harness();
  h.connect(0);
  h.socket().send(JSON.stringify({kind: 'talos.drive.summary', session: '42', supervised: true,
    registry: true, omitted: 3, nodes: [
      node('hardware_node', {}), node('drivetrain', {restarts: 2, drives: 'teleop'}),
      node('shooter', {alive: false, quarantined: true})]}));
  const html = widgetHtml(render(h), 'Subsystems');
  assert.match(html, /drivetrain<\/span><span class="tag tag-good">UP<\/span><span class="meta">restarts 2<\/span><span class="meta">drives teleop/);
  assert.match(html, /shooter<\/span><span class="tag tag-bad">DOWN<\/span><span class="tag tag-bad">QUARANTINED/);
  assert.match(html, /\+3 more/);
  h.socket().send(JSON.stringify({kind: 'talos.drive.summary', session: '42', supervised: true,
    registry: false, omitted: 0, nodes: [node('shooter', {alive: false})]}));
  const unknown = widgetHtml(render(h), 'Subsystems');
  assert.match(unknown, /shooter<\/span><span class="tag tag-neutral">unknown/);
  assert.doesNotMatch(unknown, /DOWN/);
});

// REQ-OPS-36: a malformed frame or message is counted and ignored; the page
// keeps decoding what follows.
test('garbage binary and text are counted and ignored', () => {
  const h = harness();
  h.connect(0);
  h.at(50);
  const valid = update(2, {'shooter/state/flywheel_velocity_rps': 60}, false)[0];
  h.socket().send(new Uint8Array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]),
                  valid.slice(0, valid.length - 5), new Uint8Array(0));
  h.socket().onmessage!({data: 42});
  h.socket().send('{not json', '[]', '{"kind":"talos.drive.summary","registry":true,"supervised":true,"nodes":5}',
                  '{"kind":"talos.drive.something_new"}');
  assert.deepEqual(h.model.counters,
                   {connects: 1, malformed_binary: 4, malformed_text: 3, ignored_text: 1});
  assert.equal(h.model.connection, 'open');
  h.socket().send(valid);
  assert.match(widgetHtml(render(h), 'Flywheel'), /60\.0/);
  assert.match(renderStatus(h.model, h.clock.now), /7 malformed messages ignored/);
});
