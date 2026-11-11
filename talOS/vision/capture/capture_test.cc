#include <gtest/gtest.h>

#include <cstdint>
#include <utility>

#include "talOS/vision/capture/avfoundation_source.h"
#include "talOS/vision/capture/v4l2_source.h"

namespace talos::vision {
namespace {
TEST(Capture, TimestampKindsConvertAtOneBoundary) {
  // exposure 200, readout 50, latency offset 300: each kind's geometry, then
  // the camera's measured offset, which applies to every kind.
  EXPECT_EQ(
      CaptureMidpoint(1000, TimestampKind::kStartOfExposure, 200, 50, 300),
      800);
  EXPECT_EQ(CaptureMidpoint(1000, TimestampKind::kEndOfFrame, 200, 50, 300),
            550);
  for (auto kind : {TimestampKind::kExposure, TimestampKind::kDelivery,
                    TimestampKind::kPresentation}) {
    EXPECT_EQ(CaptureMidpoint(1000, kind, 200, 50, 300), 700);
    EXPECT_FALSE(CaptureMidpoint(299, kind, 200, 50, 300));
  }
  EXPECT_FALSE(
      CaptureMidpoint(199, TimestampKind::kStartOfExposure, 200, 50, 300));
  EXPECT_FALSE(
      CaptureMidpoint(UINT64_MAX, TimestampKind::kStartOfExposure, 200, 0, 0));
  EXPECT_FALSE(CaptureMidpoint(449, TimestampKind::kEndOfFrame, 200, 50, 300));
}

#if defined(__linux__)
TEST(Capture, LinuxTimestampFlagsRetainSourceMeaning) {
  EXPECT_EQ(V4L2TimestampKind(V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
                              V4L2_BUF_FLAG_TSTAMP_SRC_SOE),
            TimestampKind::kStartOfExposure);
  EXPECT_EQ(V4L2TimestampKind(V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
                              V4L2_BUF_FLAG_TSTAMP_SRC_EOF),
            TimestampKind::kEndOfFrame);
  EXPECT_EQ(V4L2TimestampKind(V4L2_BUF_FLAG_TSTAMP_SRC_SOE),
            TimestampKind::kDelivery);
  EXPECT_EQ(V4L2TimestampKind(V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
                                  V4L2_BUF_FLAG_TSTAMP_SRC_SOE,
                              "uvcvideo"),
            TimestampKind::kDelivery);
}

// FIELD-4 (no REQ row covers live capture modes yet): a live camera opens
// only the configured size and rate. Scenario: an Arducam OV9281 (uvcvideo)
// offers YUYV 1280x800 at 20/15 fps and 640x480 at 30/20 fps; asked for
// 640x480 at 25 fps the source opened, reported 25 fps and delivered 30, and
// asked for 1280x800 at 120 fps it reported 120 and delivered nothing.
TEST(Capture, LinuxOpensOnlyAnOfferedSizeAndRate) {
  // As uvcvideo enumerates them: intervals in 100 ns units.
  const auto mode = [](uint32_t format, uint32_t w, uint32_t h,
                       uint32_t interval_100ns) {
    return V4L2Mode{format, w, h, interval_100ns, 10000000};
  };
  const V4L2Mode offered[] = {
      mode(V4L2_PIX_FMT_YUYV, 1280, 800, 500000),
      mode(V4L2_PIX_FMT_YUYV, 1280, 800, 666666),
      mode(V4L2_PIX_FMT_YUYV, 640, 480, 333333),
      mode(V4L2_PIX_FMT_YUYV, 640, 480, 500000),
      mode(V4L2_PIX_FMT_GREY, 640, 480, 500000),
  };
  const struct {
    uint32_t width, height;
    double fps;
    std::optional<V4L2Mode> expected;
  } cases[] = {
      {640, 480, 30, offered[2]},
      // GREY needs no conversion: preferred over YUYV at the same mode.
      {640, 480, 20, offered[4]},
      {1280, 800, 20, offered[0]},
      {1280, 800, 15, offered[1]},
      {640, 480, 25, std::nullopt},
      {1280, 800, 30, std::nullopt},
      {1280, 720, 20, std::nullopt},
      {640, 480, 30.01, std::nullopt},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(testing::Message()
                 << c.width << "x" << c.height << "@" << c.fps);
    EXPECT_EQ(SelectV4L2Mode(c.width, c.height, c.fps, offered), c.expected);
  }
  // What the driver grants after S_FMT/S_PARM must be what was selected.
  const V4L2Mode selected = offered[2];
  EXPECT_TRUE(V4L2Granted(selected, selected));
  EXPECT_TRUE(V4L2Granted(selected, {V4L2_PIX_FMT_YUYV, 640, 480, 1, 30}));
  EXPECT_FALSE(V4L2Granted(selected, offered[3]));
  EXPECT_FALSE(V4L2Granted(selected, {V4L2_PIX_FMT_GREY, 640, 480, 1, 30}));
  EXPECT_FALSE(V4L2Granted(selected, {V4L2_PIX_FMT_YUYV, 640, 400, 1, 30}));
}

TEST(Capture, LinuxExposureUsesDeviceUnitsAndReportsFailures) {
  std::vector<std::pair<uint32_t, int32_t>> writes;
  CameraControls controls;
  controls.exposure_ms = 4.5;
  controls.gain = 0;
  const auto write = [&](uint32_t id, int32_t value) {
    writes.emplace_back(id, value);
    return true;
  };
  EXPECT_TRUE(ApplyV4L2Controls(controls, write));
  ASSERT_EQ(writes.size(), 2);
  EXPECT_EQ(writes[0], (std::pair<uint32_t, int32_t>{V4L2_CID_EXPOSURE_AUTO,
                                                     V4L2_EXPOSURE_MANUAL}));
  EXPECT_EQ(writes[1],
            (std::pair<uint32_t, int32_t>{V4L2_CID_EXPOSURE_ABSOLUTE, 45}));
  for (uint32_t failed : {V4L2_CID_EXPOSURE_AUTO, V4L2_CID_EXPOSURE_ABSOLUTE}) {
    EXPECT_FALSE(ApplyV4L2Controls(
        controls, [failed](uint32_t id, int32_t) { return id != failed; }));
  }
  controls.auto_exposure = true;
  writes.clear();
  EXPECT_TRUE(ApplyV4L2Controls(controls, write));
  ASSERT_EQ(writes.size(), 1);
  EXPECT_EQ(writes[0].second, V4L2_EXPOSURE_AUTO);
}
#endif
TEST(Capture, FrameDurationIsOneTheRangeAccepts) {
  // An Arducam OV9281 (UVC) reports single-rate ranges through AVFoundation,
  // measured: 1 / fps itself (1000 / 60000) is refused with an exception.
  struct Range {
    double min_fps, max_fps;
    FrameDuration shortest, longest;
  };
  constexpr FrameDuration k30{1000000, 30000030}, k60{1000000, 60000240},
      k120{1000000, 120000480}, k100{1000000, 100000000};
  const Range uvc30{30.000030000030002, 30.000030000030002, k30, k30};
  const Range uvc60{60.000240000960005, 60.000240000960005, k60, k60};
  const Range uvc120{120.00048000192001, 120.00048000192001, k120, k120};
  const Range uvc100{100, 100, k100, k100};
  // A continuous range; its slow end is 3.75 fps as a UVC interval.
  const Range wide{3.7500009375002343, 30, {1, 30}, {2666666, 10000000}};
  const struct {
    double fps;
    Range range;
    std::optional<FrameDuration> expected;
  } cases[] = {
      {30, uvc30, k30},
      {60, uvc60, k60},
      {120, uvc120, k120},
      {100, uvc100, k100},
      {60, uvc30, std::nullopt},
      {59.9, uvc60, std::nullopt},
      {120.1, uvc120, std::nullopt},
      {15, wide, FrameDuration{1000, 15000}},
      {30, wide, FrameDuration{1, 30}},
      {3.75, wide, FrameDuration{2666666, 10000000}},
      // 1 / fps would be 1000 / 3750, longer than the range allows.
      {3.7504, wide, FrameDuration{2666666, 10000000}},
      {3.7, wide, std::nullopt},
      {31, wide, std::nullopt},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(testing::Message() << c.fps << " fps in [" << c.range.min_fps
                                    << ", " << c.range.max_fps << "]");
    EXPECT_EQ(FrameDurationFor(c.fps, c.range.min_fps, c.range.max_fps,
                               c.range.shortest, c.range.longest),
              c.expected);
  }
}
TEST(Capture, MissingDeviceFailsAndStopIsIdempotent) {
  CameraSourceConfig config;
  config.device_path = "/talos-nonexistent-camera-device";
  auto source = CreatePlatformCameraSource(config);
  if (!source) return;  // Unsupported platforms have no capture implementation.
  EXPECT_FALSE(source->IsRunning());
  source->Stop();
  EXPECT_FALSE(source->Start());
  EXPECT_FALSE(source->IsRunning());
  source->Stop();
  source->Stop();
  EXPECT_FALSE(source->Start());
  source->Stop();
}
}  // namespace
}  // namespace talos::vision
