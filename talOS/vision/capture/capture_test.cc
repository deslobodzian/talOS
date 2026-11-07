#include <gtest/gtest.h>

#include <cstdint>
#include <thread>
#include <utility>

#include "talOS/vision/capture/avfoundation_source.h"
#include "talOS/vision/capture/v4l2_source.h"

namespace talos::vision {
namespace {
TEST(Capture, ConfigurationDefaults) {
  const CameraSourceConfig config;
  EXPECT_EQ(config.camera_id, 0u);
  EXPECT_TRUE(config.device_path.empty());
  EXPECT_EQ(config.width, 1280u);
  EXPECT_EQ(config.height, 720u);
  EXPECT_DOUBLE_EQ(config.fps, 60.0);
  EXPECT_EQ(config.format, PixelFormat::kGray8);
  EXPECT_DOUBLE_EQ(config.exposure_ms, 0.0);
  EXPECT_DOUBLE_EQ(config.gain, 0.0);
}

TEST(Capture, TimestampKindsConvertAtOneBoundary) {
  EXPECT_EQ(
      CaptureMidpoint(1000, TimestampKind::kStartOfExposure, 200, 50, 300),
      1100);
  EXPECT_EQ(CaptureMidpoint(1000, TimestampKind::kEndOfFrame, 200, 50, 300),
            850);
  EXPECT_EQ(CaptureMidpoint(1000, TimestampKind::kExposure, 200, 50, 300),
            1000);
  for (auto kind : {TimestampKind::kDelivery, TimestampKind::kPresentation}) {
    EXPECT_EQ(CaptureMidpoint(1000, kind, 200, 50, 300), 700);
    EXPECT_FALSE(CaptureMidpoint(299, kind, 200, 50, 300));
  }
  EXPECT_FALSE(
      CaptureMidpoint(UINT64_MAX, TimestampKind::kStartOfExposure, 200, 0, 0));
  EXPECT_FALSE(CaptureMidpoint(149, TimestampKind::kEndOfFrame, 200, 50, 0));
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
  EXPECT_EQ(source->camera_id(), config.camera_id);
  EXPECT_FALSE(source->IsRunning());
  source->Stop();
  EXPECT_FALSE(source->Start());
  EXPECT_FALSE(source->IsRunning());
  source->Stop();
  source->Stop();
  EXPECT_FALSE(source->Start());
  source->Stop();
}
TEST(Capture, LatestSlotReplacesAndConsumesPendingFrame) {
  LatestFrameSlot slot;
  EXPECT_FALSE(slot.TakeLatest().has_value());
  Frame first;
  first.set_sequence_number(1);
  Frame second;
  second.set_sequence_number(2);
  slot.Publish(std::move(first));
  slot.Publish(std::move(second));
  const auto latest = slot.TakeLatest();
  if (!latest.has_value()) {
    FAIL() << "Expected the latest published frame";
  }
  const Frame& frame = *latest;
  EXPECT_EQ(frame.sequence_number(), 2u);
  EXPECT_FALSE(slot.TakeLatest().has_value());
}
TEST(Capture, LatestSlotSupportsConcurrentPublication) {
  LatestFrameSlot slot;
  std::jthread producer([&] {
    for (uint64_t i = 1; i <= 1000; ++i) {
      Frame frame;
      frame.set_sequence_number(i);
      slot.Publish(std::move(frame));
    }
  });
  producer.join();
  const auto latest = slot.TakeLatest();
  if (!latest.has_value()) {
    FAIL() << "Expected a frame after concurrent publication";
  }
  const Frame& frame = *latest;
  EXPECT_EQ(frame.sequence_number(), 1000u);
}
}  // namespace
}  // namespace talos::vision
