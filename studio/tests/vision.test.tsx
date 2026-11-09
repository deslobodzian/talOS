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

test('counterDelta takes the last finite reading minus the first', () => {
  assert.equal(counterDelta([100, 101, NaN, 103, 110]), 10);
  assert.equal(counterDelta([NaN, NaN]), null);
  assert.equal(counterDelta([100, 50]), null);  // a reset, not a negative drop
});

test('fpsFromPeriod converts microseconds to Hz', () => {
  assert.equal(fpsFromPeriod(20000), 50);
  assert.equal(fpsFromPeriod(null), null);
  assert.equal(fpsFromPeriod(0), null);
  assert.equal(fpsFromPeriod(NaN), null);
});

test('latestValidPose walks back to the most recent accepted pose', () => {
  const x = [1, 2, 3, 4], y = [10, 20, 30, 40], v = [1, 0, 1, 0];
  assert.deepEqual(latestValidPose(x, y, v), {x: 3, y: 30});
  assert.equal(latestValidPose([], [], []), null);
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
