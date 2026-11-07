// Vision panel tests: pure derivation helpers and SSR rendering for an empty
// studio and a populated one shaped like the 2026 camera/estimator dictionary.

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import type {Snapshot} from '../src/core';
import {createStaticStudio, type Studio, StudioProvider} from '../src/data/studio';
import {visionPanel} from '../src/panels/vision';
import {
  acceptancePct,
  counterDelta,
  disagreement,
  fpsFromPeriod,
  latestValidPose,
  observationErrors,
  validObservationRate
} from '../src/panels/vision/logic';

// --- pure helpers -----------------------------------------------------------

test('validObservationRate counts new observations, not repeated samples', () => {
  const t = [10, 10, 20, 30, 30, 40];
  const v = [1, 1, 0, 1, 1, 1];
  // The first sample is only a baseline; transitions at 20, 30, 40 count.
  assert.equal(validObservationRate(t, v), 2 / 3);
});

test('validObservationRate is null when there are no observations', () => {
  assert.equal(validObservationRate([], []), null);
  assert.equal(validObservationRate([NaN, NaN], [NaN, NaN]), null);
  assert.equal(validObservationRate([10, 10, 10], [1, 1, 1]), null);
  assert.equal(validObservationRate([10, 10, 10], [0, 0, 0]), null);
  assert.equal(validObservationRate([10], [1]), null);
});

test('disagreement is the planar distance', () => {
  assert.equal(disagreement({x: 0, y: 0}, {x: 3, y: 4}), 5);
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

const renderPanel = (studio: Studio): string =>
    renderToStaticMarkup(
        <StudioProvider studio={studio}>
          <visionPanel.component config={visionPanel.defaultConfig}
                                 setConfig={() => {}} instanceId="test"/>
        </StudioProvider>);

const UNITS: Record<string, string> = {};
for (const owner of ['front_left_camera', 'rear_right_camera']) {
  Object.assign(UNITS, {
    [`${owner}/state/sample_time_ns`]: 'ns',
    [`${owner}/state/pose/x_m`]: 'm',
    [`${owner}/state/pose/y_m`]: 'm',
    [`${owner}/state/pose/yaw_rad`]: 'rad',
    [`${owner}/state/x_std_m`]: 'm',
    [`${owner}/state/y_std_m`]: 'm',
    [`${owner}/state/yaw_std_rad`]: 'rad',
    [`${owner}/state/tag_count`]: '',
    [`${owner}/state/valid`]: '',
    [`${owner}/status/frames_dropped_count`]: '',
    [`${owner}/status/frame_period_us`]: 'us',
    [`${owner}/status/last_latency_us`]: 'us',
    [`${owner}/status/visible_tag_count`]: '',
    [`${owner}/status/healthy`]: ''
  });
}
Object.assign(UNITS, {
  'pose_estimator/state/pose/x_m': 'm',
  'pose_estimator/state/pose/y_m': 'm',
  'pose_estimator/state/pose/yaw_rad': 'rad',
  'pose_estimator/state/valid': '',
  'pose_estimator/state/observation_accepted_count': '',
  'pose_estimator/state/observation_rejected_count': '',
  'sim/state/pose/x_m': 'm',
  'sim/state/pose/y_m': 'm',
  'sim/state/pose/yaw_rad': 'rad'
});

function makeFrame(seq: number, ms: number, channels: Record<string, number>):
    Snapshot {
  return {
    stream_id: '1',
    sequence_id: String(seq),
    timestamp_ns: String(BigInt(ms) * 1000000n),
    dictionary_id: 0,
    chassis: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
    channels,
    units: {...UNITS},
    labels: {}
  };
}

function populatedStudio(): Studio {
  const frames: Snapshot[] = [];
  for (let i = 0; i < 5; i++) {
    frames.push(makeFrame(i, i * 200, {
      'front_left_camera/state/sample_time_ns': 1000 + i * 1000,
      'front_left_camera/state/pose/x_m': 1.0,
      'front_left_camera/state/pose/y_m': 2.0,
      'front_left_camera/state/pose/yaw_rad': 0.1,
      'front_left_camera/state/x_std_m': 0.010,
      'front_left_camera/state/y_std_m': 0.012,
      'front_left_camera/state/yaw_std_rad': 0.005,
      'front_left_camera/state/tag_count': 6,
      'front_left_camera/state/valid': i === 2 ? 0 : 1,
      'front_left_camera/status/frames_dropped_count':
          [100, 101, 103, 106, 110][i],
      'front_left_camera/status/frame_period_us': 33333,
      'front_left_camera/status/last_latency_us': 2500,
      'front_left_camera/status/visible_tag_count': 6,
      'front_left_camera/status/healthy': 1,

      'rear_right_camera/state/sample_time_ns': 2000 + i * 1000,
      'rear_right_camera/state/pose/x_m': -3.0,
      'rear_right_camera/state/pose/y_m': -1.0,
      'rear_right_camera/state/pose/yaw_rad': 0.2,
      'rear_right_camera/state/x_std_m': 0.020,
      'rear_right_camera/state/y_std_m': 0.021,
      'rear_right_camera/state/yaw_std_rad': 0.006,
      'rear_right_camera/state/tag_count': 3,
      'rear_right_camera/state/valid': 1,
      'rear_right_camera/status/frames_dropped_count': 10 + i,
      'rear_right_camera/status/frame_period_us': 40000,
      'rear_right_camera/status/last_latency_us': 3000,
      'rear_right_camera/status/visible_tag_count': 3,
      'rear_right_camera/status/healthy': 0,

      'pose_estimator/state/pose/x_m': 1.3,
      'pose_estimator/state/pose/y_m': 2.4,
      'pose_estimator/state/pose/yaw_rad': 0.09,
      'pose_estimator/state/valid': 1,
      'pose_estimator/state/observation_accepted_count':
          [50, 52, 55, 57, 60][i],
      'pose_estimator/state/observation_rejected_count': [2, 2, 3, 3, 4][i],

      'sim/state/pose/x_m': 1.0,
      'sim/state/pose/y_m': 2.0,
      'sim/state/pose/yaw_rad': 0.1
    }));
  }
  return createStaticStudio({frames, playback: {mode: 'PAUSED'}});
}

test('vision panel renders an empty studio without throwing', () => {
  const html = renderPanel(createStaticStudio({}));
  assert.ok(html.includes('No cameras discovered'), html);
  assert.ok(html.includes('No estimator discovered'), html);
});

test('vision panel renders camera rows and the estimator summary', () => {
  const html = renderPanel(populatedStudio());
  // Cameras, by name, with health shown as colour AND text.
  assert.ok(html.includes('front_left_camera'), html);
  assert.ok(html.includes('rear_right_camera'), html);
  assert.ok(html.includes('>HEALTHY<'), html);
  assert.ok(html.includes('>UNHEALTHY<'), html);
  // A derived cell: fps from frame_period_us, valid rate and disagreement.
  assert.ok(html.includes('30.0'), html);
  assert.ok(html.includes('75%'), html);
  assert.ok(html.includes('0.500'), html);
  // Estimator summary: name, state pill and the 10 s counters/acceptance.
  assert.ok(html.includes('POSE_ESTIMATOR'), html);
  assert.ok(html.includes('>VALID<'), html);
  assert.ok(html.includes('83%'), html);
  // The chart is labelled against sim truth when sim pose is present.
  assert.ok(html.includes('vs sim truth'), html);
});
