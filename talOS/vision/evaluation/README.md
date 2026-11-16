# Recording evaluation

`recording_eval` runs the camera node's detection and localization (the same
`AprilTagPipeline` and `MultiTagPoseSolver`, with `vision.toml`'s shared
detector settings unless overridden) over a [recording](../playback/README.md)
and reports how well it detects and tracks tags. It is run by hand on
footage that cannot be checked in (third-party licences, size); tests use
footage the repo renders.

```sh
bazel build //talOS/vision/evaluation:recording_eval
bazel-bin/talOS/vision/evaluation/recording_eval --frames DIR \
  (--intrinsics FX,FY,CX,CY[,K1,K2,P1,P2,K3] | --photonvision-calibration JSON | --hfov-deg DEG) \
  (--layout WPILIB_LAYOUT.json | --aprilgrid ROWS,COLS,SIZE_M,SPACING | --detect-only) \
  [--out RESULT_DIR] [options]
```

| Option | Meaning |
| --- | --- |
| `--fps F` | frame rate of a recording without `timestamps.csv` |
| `--photonvision-calibration` | a PhotonVision calibration JSON; a rational (8-coefficient) model is rectified with OpenCV first, since the pipeline models Brown-Conrady only |
| `--opencv-distortion K1,K2,P1,P2[,K3[,K4,K5,K6...]]` | rectify with OpenCV's model before detection |
| `--equidistant K1,K2,K3,K4` | rectify an equidistant (Kalibr `pinhole-equi`) fisheye before detection |
| `--hfov-deg` | assumed intrinsics: every metric number is then meaningless and the summary says so |
| `--tag-size-m` | tag edge for `--layout` (default 0.1651) |
| `--aprilgrid` | a Kalibr aprilgrid as the layout; `--grid-variant 0..7` picks row direction and in-plane tag rotation |
| `--detect-only` | no layout: every tag36h11 id counts as known; only detection, margin, hamming, id and pixel numbers mean anything |
| `--mount X,Y,Z,ROLL,PITCH,YAW` | camera mount (metres, radians, node conventions; positive pitch looks down) |
| `--floor-prior on\|off` | the solver's floor prior (default on with `--layout`, off for a handheld aprilgrid) |
| `--decimate`, `--threads`, `--sigma`, `--refine-edges on\|off`, `--hamming`, `--min-margin`, `--pixel-std-px` | detector and solver settings |
| `--mocap CSV --t-cam-imu 12 or 16 numbers` | EuRoC-format mocap of the IMU and the camera-from-IMU transform: reports pose error with the layout held fixed in the mocap world (not yet exercised on footage that decodes) |
| `--max-frames N`, `--label TEXT` | limit and name the run |

It prints a summary (also `summary.txt` in `--out`) and writes
`detections.csv` (every decode with its single-tag observation and, where a
multi-tag pose exists, the angle of each planar-ambiguity branch from it) and
`frames.csv` (the observation the node would publish per frame).

What it measures, and what each number needs:

- **Detection**: frames with a valid layout tag; decodes rejected by reason;
  decision margin and hamming of layout ids and of ids outside the layout
  (false positives when the layout is complete).
- **Expected-in-view rate and dropouts**: a tag is expected in a frame when
  the frame's multi-tag pose (two or more tags) projects it fully inside the
  image, facing the camera, at least 12 px a side; the dropout is the longest
  run of frames without a tag seen before and after it. Needs intrinsics and
  a layout.
- **Id switches**: decodes in consecutive frames whose centres lie within half
  a tag side of each other but whose ids differ. Pixel-only.
- **Jitter**: the distance of each sample from the midpoint of its neighbours
  (removes constant velocity), as a per-axis sigma; for corners (pixels,
  intrinsics not needed) and the multi-tag camera pose. Real acceleration
  also contributes, so it bounds the noise from above on moving footage.
- **Ambiguity**: per decode, whether the single-tag primary branch is more
  than 15 degrees from the multi-tag pose while the secondary is within it
  (a flip), and whether the node would publish that single tag alone; the
  single-tag versus multi-tag robot position and yaw.
- **Reprojection**: the published joint reprojection error; each decode's
  worst corner against the multi-tag projection (more than a quarter side is
  counted as misplaced).
- **Time**: load (and rectify), detection with tag pose, and localization per
  frame, p50/p95/p99/max. A busy host or one without the realtime grant gives
  an upper bound, not a budget.
