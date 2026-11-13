// REQ-OPS-37: the alert model (data/alerts.ts). The active list by level, the
// latch of every alert raised since the current match began until the next,
// and the 100 ms activation it must show, on sample times the test controls.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import {activeAlerts, AlertLevel, type AlertSample, AlertTracker} from '../src/data/alerts';

const ERROR = 'hw/status/alert_error_link_lost';
const WARNING = 'drivetrain/status/alert_warning_module_offset';
const INFO = 'autonomous/status/alert_info_routine_not_selected';
const OTHER_ERROR = 'arm/status/alert_error_encoder';
const LEVELS = {
  [ERROR]: AlertLevel.Error,
  [WARNING]: AlertLevel.Warning,
  [INFO]: AlertLevel.Info,
  [OTHER_ERROR]: AlertLevel.Error
};
const DS = 'hw/state/driver_station/';

type Ds = {enabled?: boolean; auto?: boolean; number?: number; type?: number}|null;

// One sample at `ms`: `on` alerts true, every other alert false, `absent`
// alerts with no reading; `ds` null for no Driver Station reading.
function sample(ms: number, on: string[], ds: Ds = {enabled: false},
                absent: string[] = []): AlertSample {
  const channels: Record<string, number> = {};
  for (const name of Object.keys(LEVELS))
    if (!absent.includes(name)) channels[name] = on.includes(name) ? 1 : 0;
  if (ds) {
    channels[DS + 'enabled'] = ds.enabled ? 1 : 0;
    channels[DS + 'autonomous'] = ds.auto ? 1 : 0;
    channels[DS + 'teleop'] = ds.enabled && !ds.auto ? 1 : 0;
    channels[DS + 'match_number'] = ds.number ?? 0;
    channels[DS + 'match_type'] = ds.type ?? 0;
  }
  return {timestamp_ns: String(BigInt(ms) * 1000000n), channels, alerts: LEVELS};
}

const latched = (tracker: AlertTracker) =>
    tracker.latch().alerts.map(a => `${a.channel} x${a.count} @${a.first_ns}`);

test('active alerts are ordered error, warning, info, then by name; absent is not active', () => {
  const active = activeAlerts(sample(0, [INFO, WARNING, ERROR, OTHER_ERROR]));
  assert.deepEqual(active.map(a => a.channel), [OTHER_ERROR, ERROR, WARNING, INFO]);
  assert.deepEqual(active.map(a => [a.owner, a.text]),
                   [['arm', 'encoder'], ['hw', 'link lost'],
                    ['drivetrain', 'module offset'], ['autonomous', 'routine not selected']]);
  // A stale alert topic has no reading: neither active nor clear.
  assert.deepEqual(activeAlerts(sample(0, [WARNING], null, [ERROR])).map(a => a.channel),
                   [WARNING]);
});

// The slowest update spacing that still carries every 100 ms activation is
// 100 ms. At that spacing, for every phase of the activation against the
// samples (1 ms steps), the activation is in the latch at the push of the
// first sample that carries it, less than 100 ms after it began: inside the
// 1 s REQ-OPS-37 allows. A debounce of even two samples would miss it.
test('a 100 ms activation sampled every 100 ms is shown at its first sample, at any phase', () => {
  const periodMs = 100, activationMs = 100;
  for (let onsetMs = 1000; onsetMs < 1000 + periodMs; onsetMs++) {
    const tracker = new AlertTracker();
    let shownAtMs: number|null = null;
    for (let t = 0; t <= 2000; t += periodMs) {
      const on = t >= onsetMs && t < onsetMs + activationMs;
      tracker.push(sample(t, on ? [ERROR] : []));
      if (shownAtMs === null && tracker.latch().alerts.some(a => a.channel === ERROR))
        shownAtMs = t;
      if (on && shownAtMs !== null && tracker.active().every(a => a.channel !== ERROR))
        assert.fail(`onset ${onsetMs} ms: latched but not active at ${t} ms`);
    }
    assert.ok(shownAtMs !== null, `onset ${onsetMs} ms: never shown`);
    assert.ok(shownAtMs - onsetMs < periodMs,
              `onset ${onsetMs} ms: shown at ${shownAtMs} ms, bound ${periodMs} ms`);
    assert.equal(tracker.latch().alerts[0].count, 1, `onset ${onsetMs} ms`);
  }
});

test('the latch keeps a match\'s alerts through its disabled gaps and clears at the next match number', () => {
  const tracker = new AlertTracker();
  const q1 = (enabled: boolean, auto = false) => ({enabled, auto, number: 1, type: 2});
  tracker.push(sample(0, [], q1(false)));
  tracker.push(sample(100, [WARNING], q1(true, true)));
  tracker.push(sample(200, [], q1(true, true)));
  tracker.push(sample(300, [WARNING], q1(false)));   // raised again, disabled
  tracker.push(sample(400, [], q1(true)));
  tracker.push(sample(500, [ERROR], q1(false)));
  tracker.push(sample(600, [], null, [ERROR]));     // Driver Station link lost
  tracker.push(sample(700, [ERROR], q1(false)));    // same match, still raised once
  assert.equal(tracker.latch().matchStart_ns, '0');
  assert.deepEqual(latched(tracker), [`${ERROR} x1 @500000000`, `${WARNING} x2 @100000000`]);

  // The FMS assigns match 2 while the robot is disabled: a new latch, holding
  // only the error still active then, as raised at the match start.
  tracker.push(sample(800, [ERROR], {enabled: false, number: 2, type: 2}));
  assert.equal(tracker.latch().matchStart_ns, '800000000');
  assert.equal(tracker.latch().matchKey, '2:2');
  assert.deepEqual(latched(tracker), [`${ERROR} x1 @800000000`]);
});

test('without a match number, auto then teleop is one match and the next enable begins another', () => {
  const tracker = new AlertTracker();
  tracker.push(sample(0, [INFO], {enabled: false}));
  assert.equal(tracker.latch().matchStart_ns, null);   // before any match
  tracker.push(sample(100, [], {enabled: true, auto: true}));
  tracker.push(sample(200, [WARNING], {enabled: false}));
  tracker.push(sample(300, [], {enabled: true}));       // teleop after auto
  assert.equal(tracker.latch().matchStart_ns, '100000000');
  assert.deepEqual(latched(tracker), [`${WARNING} x1 @200000000`]);

  tracker.push(sample(400, [], {enabled: false}));
  tracker.push(sample(500, [], {enabled: true}));       // a new enable
  assert.equal(tracker.latch().matchStart_ns, '500000000');
  assert.deepEqual(latched(tracker), []);
});
