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
