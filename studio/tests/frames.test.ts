// Pure-logic tests for the recorded-frame archive: pairing PNG with JSON,
// ordering by capture_ns, refusing malformed metadata, and filtering.
//
// frames.ts never touches a DOM API, so these run under plain node and prove
// the contract validation rather than just that the code bundles.

import assert from 'node:assert/strict';
import {test} from 'node:test';
import type {KeyboardEvent} from 'react';
import {FramesKeyScope} from '../src/panels/frames';

import {
  filterFrames,
  framePoseError,
  loadFrameFiles,
  parseFrameMetadata,
  type FrameReason,
  type LocalFrameFile,
  type RecordedFrame,
} from '../src/frames';

test('frame arrow keys advance only the receiving panel and ignore editors', () => {
  const positions = [2, 7];
  const roots = positions.map((_, i) => FramesKeyScope({
    onStep: delta => { positions[i] += delta; }
  }));
  const press = (root: number, key: string, tagName = 'DIV', editable = false) => {
    roots[root].props.onKeyDown({
      key, target: {tagName, isContentEditable: editable},
      preventDefault() {}, stopPropagation() {}
    } as unknown as KeyboardEvent<HTMLDivElement>);
  };
  press(0, 'ArrowRight');
  assert.deepEqual(positions, [3, 7]);
  press(1, 'ArrowLeft');
  assert.deepEqual(positions, [3, 6]);
  for (const tag of ['INPUT', 'SELECT', 'TEXTAREA']) press(0, 'ArrowRight', tag);
  press(0, 'ArrowRight', 'DIV', true);
  press(0, 'Enter');
  assert.deepEqual(positions, [3, 6]);
});

// --- fixtures ---------------------------------------------------------------

const det = () => ({
  tag_id: 1,
  hamming: 0,
  decision_margin: 100,
  corners_px: [10, 10, 110, 10, 110, 110, 10, 110],
  valid: true,
  reprojection_error_px: 0.5,
  ambiguity: 0,
  reject_reason: null
});

const obs = () => ({
  x_m: 1, y_m: 2, yaw_rad: 0.1, valid: true,
  x_std_m: 0.01, y_std_m: 0.01, yaw_std_rad: 0.02, tag_count: 1
});

const truth = () => ({x_m: 1.1, y_m: 2.2, yaw_rad: 0.15});

const metadata = (overrides: Record<string, unknown> = {}) => ({
  format_version: 1,
  camera: 'front',
  capture_ns: 123456789,
  reason: 'sampled',
  width: 1280,
  height: 960,
  intrinsics:
      {fx: 800, fy: 800, cx: 640, cy: 480, k1: 0, k2: 0, k3: 0, p1: 0, p2: 0},
  mounting: {
    x_m: 0, y_m: 0, z_m: 0.5, roll_rad: 0, pitch_rad: 0, yaw_rad: 0
  },
  detector: {
    backend: 'apriltag', decimate: 2, threads: 4, sigma: 0,
    refine_edges: true, hamming_threshold: 3, min_decision_margin: 50
  },
  detections: [det()],
  observation: obs(),
  truth: truth(),
  ...overrides
});

const json = (overrides: Record<string, unknown> = {}) =>
    JSON.stringify(metadata(overrides));

// The on-disk stem: capture_ns zero-padded to 20 digits, then the reason.
const stem = (ns: number, reason: string) =>
    String(ns).padStart(20, '0') + '_' + reason;

const file = (name: string, text = '', size = 100,
              relativePath?: string): LocalFrameFile => ({
                name,
                webkitRelativePath: relativePath,
                size,
                text: async () => text
              });

const dummy: LocalFrameFile = {name: '', size: 0, text: async () => ''};

const recorded = (camera: string, reason: FrameReason,
                  ns: number): RecordedFrame<LocalFrameFile> => ({
                    id: `${camera}-${reason}-${ns}`,
                    metadata:
                        parseFrameMetadata(json({camera, reason, capture_ns: ns})),
                    image: dummy
                  });

// --- parsing ----------------------------------------------------------------

test('parseFrameMetadata reads a complete record', () => {
  const parsed = parseFrameMetadata(json());
  assert.equal(parsed.format_version, 1);
  assert.equal(parsed.camera, 'front');
  assert.equal(parsed.capture_ns, '123456789');
  assert.equal(parsed.reason, 'sampled');
  assert.equal(parsed.width, 1280);
  assert.equal(parsed.height, 960);
  assert.equal(parsed.intrinsics.fx, 800);
  assert.equal(parsed.mounting.yaw_rad, 0);
  assert.equal(parsed.detector.backend, 'apriltag');
  assert.equal(parsed.detections.length, 1);
  assert.equal(parsed.detections[0].tag_id, 1);
  assert.equal(parsed.observation.valid, true);
  assert.equal(parsed.observation.tag_count, 1);
  assert.equal(parsed.truth?.yaw_rad, 0.15);
});

test('parseFrameMetadata refuses missing fields, wrong types and unknown versions',
     () => {
       assert.throws(() => parseFrameMetadata(json({camera: undefined})),
                     /camera/);
       assert.throws(() => parseFrameMetadata(json({width: 'wide'})), /width/);
       assert.throws(() => parseFrameMetadata(json({format_version: 2})),
                     /format_version/);
       assert.throws(() => parseFrameMetadata(json({reason: 'other'})),
                     /reason/);
       assert.throws(() => parseFrameMetadata(json({capture_ns: '123'})),
                     /capture_ns/);
       assert.throws(() => parseFrameMetadata(json({capture_ns: 1.5})),
                     /capture_ns/);
       assert.throws(
           () => parseFrameMetadata(json({observation: {...obs(), valid: 'yes'}})),
           /valid/);
       assert.throws(() => parseFrameMetadata('not json'), /metadata/);
     });

test('parseFrameMetadata bounds collections and validates detection shapes',
     () => {
       const withDet = (value: unknown) =>
           parseFrameMetadata(json({detections: value}));
       // corners_px must carry exactly eight finite numbers.
       assert.throws(
           () => withDet([{...det(), corners_px: [0, 0, 1, 1]}]), /corners_px/);
       // reject_reason is bounded like every other string.
       assert.throws(
           () => withDet([{...det(), reject_reason: 'x'.repeat(513)}]),
           /reject_reason/);
       // detections must be an array.
       assert.throws(() => withDet('not-an-array'), /detections/);
       // At most 1024 detections.
       assert.throws(() => withDet(Array.from({length: 1025}, det)),
                     /detections/);
     });

// --- pairing, ordering, unpaired files --------------------------------------

test('pairs png with json and sorts by capture_ns regardless of file order',
     async () => {
       const archive = await loadFrameFiles([
         file(`${stem(200, 'sampled')}.png`),
         file(`${stem(100, 'sampled')}.json`, json({capture_ns: 100})),
         file(`${stem(200, 'sampled')}.json`, json({capture_ns: 200})),
         file(`${stem(100, 'sampled')}.png`)
       ]);
       assert.equal(archive.errors.length, 0);
       assert.equal(archive.unpaired, 0);
       assert.equal(archive.frames.length, 2);
       assert.equal(archive.frames[0].metadata.capture_ns, '100');
       assert.equal(archive.frames[1].metadata.capture_ns, '200');
     });

test('ignores unpaired files and files that are not frames', async () => {
  const archive = await loadFrameFiles([
    file(`${stem(1, 'sampled')}.png`),
    file(`${stem(2, 'sampled')}.json`, json({capture_ns: 2})),
    file('notes.txt', 'hello'),
    file('README.md', '# readme'),
    file(`${stem(3, 'sampled')}.png`),
    file(`${stem(3, 'sampled')}.json`, json({capture_ns: 3}))
  ]);
  assert.equal(archive.unpaired, 2);
  assert.equal(archive.frames.length, 1);
  assert.equal(archive.errors.length, 0);
});

test('frames from different camera directories keep distinct ids', async () => {
  const name = `${stem(1, 'sampled')}`;
  const archive = await loadFrameFiles([
    file(`${name}.png`, '', 100, `camA/${name}.png`),
    file(`${name}.json`, json({capture_ns: 1, camera: 'camA'}), 100,
         `camA/${name}.json`),
    file(`${name}.png`, '', 100, `camB/${name}.png`),
    file(`${name}.json`, json({capture_ns: 1, camera: 'camB'}), 100,
         `camB/${name}.json`)
  ]);
  assert.equal(archive.errors.length, 0);
  assert.equal(archive.frames.length, 2);
  assert.deepEqual(
      archive.frames.map(f => f.metadata.camera).sort(), ['camA', 'camB']);
});

test('a stem that disagrees with its capture_ns is an error, not a frame',
     async () => {
       const archive = await loadFrameFiles([
         file(`${stem(999, 'sampled')}.png`),
         file(`${stem(999, 'sampled')}.json`, json({capture_ns: 5}))
       ]);
       assert.equal(archive.frames.length, 0);
       assert.equal(archive.errors.length, 1);
       assert.match(archive.errors[0].message, /does not match/);
     });

test('a malformed metadata file is reported rather than fatal', async () => {
  const archive = await loadFrameFiles([
    file(`${stem(1, 'sampled')}.png`),
    file(`${stem(1, 'sampled')}.json`,
         JSON.stringify({format_version: 2, capture_ns: 1, reason: 'sampled'}))
  ]);
  assert.equal(archive.frames.length, 0);
  assert.equal(archive.errors.length, 1);
  assert.match(archive.errors[0].message, /format_version/);
});

test('duplicate stems are reported rather than silently paired', async () => {
  const archive = await loadFrameFiles([
    file(`${stem(1, 'sampled')}.png`),
    file(`${stem(1, 'sampled')}.png`),
    file(`${stem(1, 'sampled')}.json`, json({capture_ns: 1}))
  ]);
  assert.equal(archive.frames.length, 0);
  assert.ok(archive.errors.some(e => /Duplicate/.test(e.message)));
});

// --- error computation ------------------------------------------------------

test('framePoseError computes only with truth and a valid observation', () => {
  const withTruth = parseFrameMetadata(json());
  const error = framePoseError(withTruth);
  assert.ok(error);
  // estimate (1, 2) against truth (1.1, 2.2).
  assert.ok(Math.abs(error.position_m - Math.hypot(0.1, 0.2)) < 1e-9);
  assert.ok(Math.abs(error.heading_deg - (0.1 - 0.15) * 180 / Math.PI) < 1e-9);

  const noTruth = parseFrameMetadata(json({truth: null}));
  assert.equal(framePoseError(noTruth), null);

  const invalid =
      parseFrameMetadata(json({observation: {...obs(), valid: false}}));
  assert.equal(framePoseError(invalid), null);
});

// --- filtering --------------------------------------------------------------

test('filterFrames filters by camera and reason independently', () => {
  const frames = [
    recorded('front', 'sampled', 1),
    recorded('front', 'rejected', 2),
    recorded('rear', 'sampled', 3),
    recorded('rear', 'rejected', 4)
  ];
  assert.equal(filterFrames(frames).length, 4);
  assert.equal(filterFrames(frames, 'front').length, 2);
  assert.equal(filterFrames(frames, '', 'rejected').length, 2);
  assert.equal(filterFrames(frames, 'rear', 'sampled').length, 1);
  assert.equal(filterFrames(frames, 'nowhere').length, 0);
});
