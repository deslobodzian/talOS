// Vision panel tests: pure derivation helpers and the rendered panel's choice
// of reference pose (sim truth only when it has finite data).

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import {createStaticStudio, StudioProvider} from '../src/data/studio';
import {visionPanel} from '../src/panels/vision';
import {
  acceptancePct,
  counterDelta,
  fpsFromPeriod,
  fpsSeries,
  latestValidPose,
  observationErrors,
  validObservationRate
} from '../src/panels/vision/logic';

// --- pure helpers -----------------------------------------------------------

test('validObservationRate counts new observations, not repeated samples', () => {
  // The first sample is only a baseline; transitions at 20, 30, 40 count.
  assert.equal(validObservationRate([10, 10, 20, 30, 30, 40], [1, 1, 0, 1, 1, 1]), 2 / 3);
  // No new observation at all is unknown, not a 0 % or 100 % rate.
  assert.equal(validObservationRate([], []), null);
  assert.equal(validObservationRate([NaN, NaN], [NaN, NaN]), null);
  assert.equal(validObservationRate([10, 10, 10], [1, 1, 1]), null);
  assert.equal(validObservationRate([10, 10, 10], [0, 0, 0]), null);
  assert.equal(validObservationRate([10], [1]), null);
});

test('counterDelta sums increments and ignores a reset', () => {
  assert.equal(counterDelta([100, 101, NaN, 103, 110]), 10);
  assert.equal(counterDelta([NaN, NaN]), null);
  // 100 -> 105, reset, 2 -> 104: 5 + 102 observed, never 104 - 100 = 4 nor a
  // negative drop.
  assert.equal(counterDelta([100, 105, 2, 104]), 107);
  assert.equal(counterDelta([100, 50]), 0);
});

// A camera that has not timed a frame publishes a zero period; dividing by it
// would show an infinite (or, for a negative or NaN period, a meaningless)
// frame rate as a reading.
test('fpsFromPeriod is the frame rate, or unknown for a period that is not one', () => {
  assert.equal(fpsFromPeriod(20000), 50);
  for (const period of [0, -20000, NaN, Infinity, null, undefined]) {
    assert.equal(fpsFromPeriod(period), null, String(period));
  }
});

// The open camera's fps sparkline: a zero, negative or missing period is a
// gap, not a point at Infinity that flattens the rest of the line.
test('fpsSeries leaves a gap for a period that is not one', () => {
  const fps = Array.from(fpsSeries([20000, 0, -20000, NaN, 10000]));
  assert.deepEqual(fps.map(v => (Number.isNaN(v) ? 'gap' : v)),
                   [50, 'gap', 'gap', 'gap', 100]);
});

test('latestValidPose walks back to the most recent current accepted pose', () => {
  const t = [0, 0.25, 0.5, 0.75], x = [1, 2, 3, 4], y = [10, 20, 30, 40];
  assert.deepEqual(latestValidPose(t, x, y, [1, 0, 1, 0]), {x: 3, y: 30});
  // The only accepted observation is a full second older than the cursor:
  // the robot has moved since, so there is nothing to compare.
  assert.equal(latestValidPose([0, 0.5, 1], x, y, [1, 0, 0]), null);
  assert.equal(latestValidPose([], [], [], []), null);
});

test('acceptancePct divides accepted by the total, or null', () => {
  assert.equal(acceptancePct(8, 2), 0.8);
  assert.equal(acceptancePct(0, 0), null);
  assert.equal(acceptancePct(null, 2), null);
});

test('observationErrors compares only accepted observations', () => {
  const t = [0, 1, 2, 3];
  const cx = [1, 2, 3, 4], cy = [10, 20, 30, 40];
  const v = [1, 0, 1, 1];
  const rx = [1, 1, 2, 5], ry = [10, 10, 25, 40];
  assert.deepEqual(observationErrors(t, cx, cy, v, rx, ry), [
    {t: 0, x: 0, y: 0},
    {t: 2, x: 1, y: 5},
    {t: 3, x: -1, y: 0}
  ]);

  // Separate decimated recorded time axes: matches each valid observation to
  // the nearest finite reference pose within 0.5 s and skips stale gaps.
  const camTimes = [1.02, 2.04, 5.0];
  const refTimes = [1.0, 2.0, 2.05, 8.0];
  assert.deepEqual(
      observationErrors(camTimes, [1.1, 2.2, 5.0], [10.3, 20.4, 50.0], [1, 1, 1],
                        [1.0, 2.0, 2.1, 8.0], [10.0, 20.0, 20.1, 80.0], refTimes)
          .map(p => ({t: p.t, x: Math.round(p.x * 100) / 100, y: Math.round(p.y * 100) / 100})),
      [
        {t: 1.02, x: 0.1, y: 0.3},
        {t: 2.04, x: 0.1, y: 0.3}
      ]);
});

// --- rendering --------------------------------------------------------------

test('vision compares against sim truth only when sim has finite data', () => {
  for (const [simX, reference] of [
    [undefined, 'vs estimator'], [NaN, 'vs estimator'], [1, 'vs sim truth']
  ] as const) {
    const channels: Record<string, number> = {
      'pose_estimator/state/pose/x_m': 1,
      'pose_estimator/state/pose/y_m': 2,
      'pose_estimator/state/valid': 1,
      'pose_estimator/state/observation_accepted_count': 0
    };
    if (simX !== undefined) {
      channels['sim/state/pose/x_m'] = simX;
      channels['sim/state/pose/y_m'] = 2;
    }
    // Sim pose is declared in the dictionary in every case.
    const units: Record<string, string> = {
      'pose_estimator/state/pose/x_m': 'm',
      'pose_estimator/state/pose/y_m': 'm',
      'pose_estimator/state/valid': '',
      'pose_estimator/state/observation_accepted_count': '',
      'sim/state/pose/x_m': 'm',
      'sim/state/pose/y_m': 'm'
    };
    const studio = createStaticStudio({
      frames: [{
        stream_id: '1', sequence_id: '0', timestamp_ns: '0', dictionary_id: 0,
        chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
        channels, units, labels: {}
      }],
      playback: {mode: 'PAUSED'}
    });
    const html = renderToStaticMarkup(
        <StudioProvider studio={studio}>
          <visionPanel.component config={visionPanel.defaultConfig}
                                 setConfig={() => {}} instanceId="test"/>
        </StudioProvider>);
    assert.ok(html.includes(reference), `sim x ${simX}`);
  }
});

// REQ-OPS-36, REQ-OPS-37: non-finite camera health and estimator validity
// samples (NaN, +/-Infinity) must render as unknown badges, never HEALTHY/VALID.
test('vision renders non-finite camera health and estimator validity as unknown', () => {
  for (const bad of [NaN, Infinity, -Infinity]) {
    const channels: Record<string, number> = {
      'front_camera/state/pose/x_m': 1,
      'front_camera/state/pose/y_m': 2,
      'front_camera/state/x_std_m': 0.05,
      'front_camera/state/tag_count': 2,
      'front_camera/status/healthy': bad,
      'pose_estimator/state/pose/x_m': 1,
      'pose_estimator/state/pose/y_m': 2,
      'pose_estimator/state/valid': bad,
      'pose_estimator/state/observation_accepted_count': 5
    };
    const studio = createStaticStudio({
      frames: [{
        stream_id: '1', sequence_id: '0', timestamp_ns: '1000000000', dictionary_id: 0,
        chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
        channels,
        units: Object.fromEntries(Object.keys(channels).map(k => [k, ''])),
        labels: {}
      }],
      playback: {mode: 'PAUSED', cursor_ns: '1000000000'}
    });
    const html = renderToStaticMarkup(
        <StudioProvider studio={studio}>
          <visionPanel.component config={visionPanel.defaultConfig}
                                 setConfig={() => {}} instanceId="test"/>
        </StudioProvider>);
    assert.doesNotMatch(html, />HEALTHY</, `non-finite healthy rendered HEALTHY for ${bad}`);
    assert.doesNotMatch(html, />UNHEALTHY</, `non-finite healthy rendered UNHEALTHY for ${bad}`);
    assert.doesNotMatch(html, />VALID</, `non-finite valid rendered VALID for ${bad}`);
    assert.doesNotMatch(html, />INVALID</, `non-finite valid rendered INVALID for ${bad}`);
    assert.equal((html.match(/>unknown</g) ?? []).length, 2,
                 `expected two unknown badges for ${bad}: ${html}`);
  }
});
