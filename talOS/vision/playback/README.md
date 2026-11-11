# Recorded footage playback

Plays a recording of a camera into the camera node, or into
[`recording_eval`](../evaluation/README.md), through the `CameraSource`
interface, so the node's unmodified pipeline (AprilTag detection, tag pose,
multi-tag localization, `PoseObservation`) runs on recorded footage.

## A recording

A directory of image files (`.png`, `.jpg`, `.jpeg`, `.pgm`, `.bmp`, decoded
as 8-bit grayscale; 16-bit images are reduced to 8 bits) and its timeline:

- with `timestamps.csv` in the directory, one `<file name>,<pts_ns>` line per
  frame, in strictly increasing presentation time (a header line is
  skipped); the times are rebased so the first frame is at 0, and any clock
  works (an absolute capture clock, say);
- without it, every image file in file-name order, at the node's
  `camera.fps` (the tool's `--fps`).

A camera node's frame recorder output (`<session>/frames/<camera>/`) is not
such a directory; replay it with `frame_replay`
([camera node README](../../../2026-robot/main_processor/vision/README.md#frame-recording)).

## Video files

The repo's OpenCV videoio has no FFmpeg backend on Linux (`cap_images`,
MJPEG AVI and V4L2 only), so a video is decoded first, with the system
`ffmpeg` and `ffprobe`, into a recording whose `timestamps.csv` carries each
frame's container presentation time (variable frame rates keep their
timeline):

```sh
talOS/vision/playback/decode_video.sh VIDEO OUT_DIR [ffmpeg -vf filter]
```

`OUT_DIR` must be new. Decoding is lossless after the codec (PNG), so a
recording of 1280×800 frames takes about 0.5 MB per frame.

## Playing a recording in the camera node

Set the camera's `camera.device = "recording:<dir>"`. `CreateCameraSource`
(the node's default source factory) then makes a `RecordedCameraSource`
instead of opening a camera:

- `Start()` (on the node's capture worker) indexes the recording and decodes
  its first frame; a missing or malformed recording, or frames of a size other
  than `camera.width` × `camera.height`, fail the start with that reason.
- A playback thread decodes each frame ahead of its time and delivers it at
  `start + pts` on the host monotonic clock (the realtime node clock: Linux
  `CLOCK_MONOTONIC`, macOS `CLOCK_MONOTONIC_RAW`), stamped with that instant,
  `TimestampKind::kPresentation` (no exposure correction; the camera's
  `latency_offset_us` still applies). A frame whose time passed by more than
  one mean frame period before its decode started is skipped and counted in
  the camera's `frames_dropped_count`. Decoding never runs on the node loop.
- One mean frame period after the last frame the source reports a
  disconnect; the worker reopens it after its retry backoff, which plays the
  recording again.
- Camera controls are accepted and ignored.

Under `testing::Sim`, construct the source with `Pacing::kManual` through the
node's source factory and call `DeliverUntil(sim.now())` between steps: no
thread, no wall clock, and the same frames at the same simulated instants on
every run (`2026-robot/main_processor/vision/recorded_footage_test.cc`).
