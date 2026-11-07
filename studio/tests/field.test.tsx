// Field panel tests: the pure geometry helpers (ellipse from stds, staleness
// fade, follow viewport) and the rendered panel against an empty and a
// realistically populated static studio (frames shaped like the 2026
// dictionary: a pose estimator with velocity, two cameras with stds, and sim
// truth).

import assert from 'node:assert/strict';
import {test} from 'node:test';

import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

import type {Snapshot} from '../src/core';
import {createStaticStudio, type Studio, StudioProvider} from '../src/data/studio';
import {fieldPanel} from '../src/panels/field';
import {
  FIELD_HEIGHT_PX,
  FIELD_WIDTH_PX,
  PX_PER_M,
  followViewport,
  stalenessFade,
  uncertaintyEllipse,
  wholeFieldViewport,
  zoomViewport
} from '../src/panels/field/geometry';

// --- pure geometry ----------------------------------------------------------

test('uncertaintyEllipse scales standard deviations to radii', () => {
  assert.deepEqual(uncertaintyEllipse(0.1, 0.2), {rx: 0.1, ry: 0.2});
  assert.deepEqual(uncertaintyEllipse(0.1, 0.2, 2), {rx: 0.2, ry: 0.4});
  // Non-finite or negative stds collapse to a zero-radius dot.
  assert.deepEqual(uncertaintyEllipse(NaN, -1), {rx: 0, ry: 0});
  assert.deepEqual(uncertaintyEllipse(Infinity, 0.5), {rx: 0, ry: 0.5});
});

test('stalenessFade is 1 fresh, 0 at max age, clamps future skew', () => {
  assert.equal(stalenessFade(1000, 1000), 1);
  assert.equal(stalenessFade(0, 1e9), 0);
  assert.equal(stalenessFade(0, 5e8), 0.5);
  assert.equal(stalenessFade(0, 1.5e9), 0);
  // A future sample (clock skew) reads fresh, not over-faded.
  assert.equal(stalenessFade(2000, 1000), 1);
  // Decimal-string timestamps are accepted.
  assert.equal(stalenessFade('1000000000', '1500000000'), 0.5);
});

test('followViewport is a 4 m window centred on the robot, clamped', () => {
  const vp = followViewport({x: 8, y: 4}, 4);
  assert.equal(vp.width, 4 * PX_PER_M);
  assert.equal(vp.height, 4 * PX_PER_M);
  // Centre maps to the robot: midpoint of the viewport is the robot's pixel.
  assert.equal(vp.x + vp.width / 2, 8 * PX_PER_M);
  assert.equal(vp.y + vp.height / 2, FIELD_HEIGHT_PX - 4 * PX_PER_M);
  // At the blue-origin corner the window is clamped inside the field.
  const corner = followViewport({x: 0, y: 0}, 4);
  assert.ok(corner.x >= 0);
  assert.ok(corner.y >= 0);
  assert.ok(corner.x + corner.width <= FIELD_WIDTH_PX);
  assert.ok(corner.y + corner.height <= FIELD_HEIGHT_PX);
});

test('wholeFieldViewport and zoomViewport', () => {
  const whole = wholeFieldViewport();
  assert.equal(whole.width, FIELD_WIDTH_PX + 50);
  assert.equal(whole.height, FIELD_HEIGHT_PX + 50);
  const zoomed = zoomViewport(whole, 2, 0, 0);
  assert.equal(zoomed.width, whole.width * 2);
  assert.equal(zoomed.height, whole.height * 2);
  // Non-positive factors are a no-op.
  assert.deepEqual(zoomViewport(whole, 0, 1, 1), whole);
});

// --- config -----------------------------------------------------------------

test('field config parsing falls back field by field', () => {
  assert.deepEqual(fieldPanel.parseConfig(null), fieldPanel.defaultConfig);
  assert.deepEqual(fieldPanel.parseConfig('nope'), fieldPanel.defaultConfig);
  assert.deepEqual(
      fieldPanel.parseConfig(fieldPanel.defaultConfig), fieldPanel.defaultConfig);
  const parsed = fieldPanel.parseConfig({
    layers: {trail: false, truth: true, velocity: 'x'},
    trailSeconds: -3,
    follow: true
  });
  assert.equal(parsed.layers.trail, false);
  assert.equal(parsed.layers.truth, true);
  assert.equal(parsed.layers.cameras, true);   // untouched fallback
  assert.equal(parsed.layers.velocity, true);  // non-bool falls back
  assert.equal(parsed.trailSeconds, 0.5);      // clamped to a positive floor
  assert.equal(parsed.follow, true);
});

// --- rendering --------------------------------------------------------------

const POSE_EST = 'pose_estimator/state';
const CAM = (name: string) => `${name}/state`;
const T0 = 1000000000000000000n;

function frame(seq: number): Snapshot {
  const tNs = T0 + BigInt(seq) * 20000000n;
  const t = Number(tNs);
  const channels: Record<string, number> = {
    [`${POSE_EST}/pose/x_m`]: 2 + seq * 0.1,
    [`${POSE_EST}/pose/y_m`]: 3 + seq * 0.05,
    [`${POSE_EST}/pose/yaw_rad`]: 0.2 + seq * 0.02,
    [`${POSE_EST}/velocity/vx_mps`]: 1.0,
    [`${POSE_EST}/velocity/vy_mps`]: 0.5,
    [`${POSE_EST}/velocity/omega_radps`]: 0.1,
    [`${POSE_EST}/valid`]: 1,
    [`${POSE_EST}/observation_accepted_count`]: 10 + seq,
    [`${POSE_EST}/observation_rejected_count`]: 1,
    [`${POSE_EST}/sample_time_ns`]: t,
    'sim/state/pose/x_m': 2 + seq * 0.1 + 0.05,
    'sim/state/pose/y_m': 3 + seq * 0.05 - 0.03,
    'sim/state/pose/yaw_rad': 0.2 + seq * 0.02 + 0.01,
    'derived/pose_error_m': 0.06,
    'derived/heading_error_deg': 0.5,
    [`${CAM('front_left_camera')}/pose/x_m`]: 2.1,
    [`${CAM('front_left_camera')}/pose/y_m`]: 3.05,
    [`${CAM('front_left_camera')}/pose/yaw_rad`]: 0.22,
    [`${CAM('front_left_camera')}/x_std_m`]: 0.08,
    [`${CAM('front_left_camera')}/y_std_m`]: 0.1,
    [`${CAM('front_left_camera')}/yaw_std_rad`]: 0.05,
    [`${CAM('front_left_camera')}/tag_count`]: 4,
    [`${CAM('front_left_camera')}/valid`]: 1,
    [`${CAM('front_left_camera')}/sample_time_ns`]: t,
    [`${CAM('front_right_camera')}/pose/x_m`]: 2.0,
    [`${CAM('front_right_camera')}/pose/y_m`]: 2.95,
    [`${CAM('front_right_camera')}/pose/yaw_rad`]: 0.2,
    [`${CAM('front_right_camera')}/x_std_m`]: 0.05,
    [`${CAM('front_right_camera')}/y_std_m`]: 0.07,
    [`${CAM('front_right_camera')}/yaw_std_rad`]: 0.04,
    [`${CAM('front_right_camera')}/tag_count`]: 2,
    [`${CAM('front_right_camera')}/valid`]: 1,
    // Half a second stale: still visible, at half opacity.
    [`${CAM('front_right_camera')}/sample_time_ns`]: t - 0.5e9
  };
  return {
    stream_id: 's1',
    sequence_id: String(seq),
    timestamp_ns: tNs.toString(),
    dictionary_id: 1,
    chassis: {x: 2 + seq * 0.1, y: 3 + seq * 0.05, z: 0, roll: 0, pitch: 0,
              yaw: 0.2 + seq * 0.02},
    channels,
    units: Object.fromEntries(Object.keys(channels).map(k => [k, ''])),
    labels: {}
  };
}

function populatedStudio(): Studio {
  const frames = [0, 1, 2].map(frame);
  return createStaticStudio({
    frames,
    playback: {
      mode: 'PAUSED',
      cursor_ns: frames[2].timestamp_ns,
      bounds: {start: frames[0].timestamp_ns, end: frames[2].timestamp_ns},
      stats: {dropped: 0, late: 0, invalid: 0},
      status: 'Static', connection: 'replay'
    }
  });
}

function renderField(studio: Studio): string {
  return renderToStaticMarkup(
      <StudioProvider studio={studio}>
        <fieldPanel.component config={fieldPanel.defaultConfig}
                              setConfig={() => {}} instanceId="test"/>
      </StudioProvider>);
}

test('the field panel waits for telemetry when there is none', () => {
  const html = renderField(createStaticStudio({}));
  assert.ok(html.includes('Awaiting telemetry'));
  assert.ok(html.includes('BLUE ORIGIN'));
});

test('the field panel renders every layer that has data', () => {
  const html = renderField(populatedStudio());
  assert.ok(html.includes('BLUE ORIGIN'));
  assert.ok(html.includes('Estimate'), 'estimate robot legend');
  assert.ok(html.includes('Trail'), 'estimate trail legend');
  assert.ok(html.includes('Sim truth'), 'truth legend');
  assert.ok(html.includes('Velocity'), 'velocity legend');
  assert.ok(html.includes('front_left_camera'), 'first camera legend');
  assert.ok(html.includes('front_right_camera'), 'second camera legend');
  assert.ok(html.includes('Position error'), 'error readout');
  assert.ok(html.includes('m'), 'position unit');
  assert.ok(html.includes('°'), 'heading unit');
  assert.ok(!html.includes('Awaiting telemetry'));
  // The camera layer draws an uncertainty ellipse and a heading tick.
  assert.ok(html.includes('ellipse'), 'camera uncertainty ellipse rendered');
});

test('camera position survives missing uncertainty and heading without inventing them', () => {
  const renderCamera = (omit: string[]) => {
    const sample = frame(0);
    for (const name of Object.keys(sample.channels)) {
      if (name.startsWith('front_right_camera/')) {
        delete sample.channels[name];
        delete sample.units[name];
      } else if (omit.some(leaf => name === `front_left_camera/state/${leaf}`)) {
        sample.channels[name] = NaN;
      }
    }
    return renderField(createStaticStudio({
      frames: [sample], playback: {mode: 'PAUSED', cursor_ns: sample.timestamp_ns}
    }));
  };
  // These SVG primitives are the actual uncertainty / direction visualisations,
  // not styling hooks. The observation's position marker must remain.
  const complete = renderCamera([]);
  const missing = renderCamera(['x_std_m', 'y_std_m', 'pose/yaw_rad']);
  assert.equal((complete.match(/<ellipse\b/g) ?? []).length, 1);
  assert.equal((missing.match(/<ellipse\b/g) ?? []).length, 0);
  assert.equal((complete.match(/<path\b/g) ?? []).length -
               (missing.match(/<path\b/g) ?? []).length, 1);
  assert.equal((complete.match(/<circle\b/g) ?? []).length,
               (missing.match(/<circle\b/g) ?? []).length);
});
