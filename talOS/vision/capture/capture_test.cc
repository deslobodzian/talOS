#include <gtest/gtest.h>

#include <cstdint>
#include <thread>
#include <utility>

#include "talOS/vision/capture/avfoundation_source.h"

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
