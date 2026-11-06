#include "talOS/vision/frame.h"

#include <gtest/gtest.h>

#include <utility>
#include <vector>

namespace talos::vision {
namespace {

TEST(FrameTest, PixelFormatHelpers) {
  EXPECT_EQ(BytesPerPixel(PixelFormat::kGray8), 1);
  EXPECT_EQ(BytesPerPixel(PixelFormat::kRgb8), 3);
  EXPECT_EQ(BytesPerPixel(PixelFormat::kBgr8), 3);
  EXPECT_EQ(BytesPerPixel(PixelFormat::kYuyv), 2);
  EXPECT_EQ(BytesPerPixel(PixelFormat::kBgra8), 4);

  EXPECT_EQ(PixelFormatToString(PixelFormat::kGray8), "Gray8");
  EXPECT_EQ(PixelFormatToString(PixelFormat::kRgb8), "Rgb8");
  EXPECT_EQ(PixelFormatToString(PixelFormat::kBgr8), "Bgr8");
  EXPECT_EQ(PixelFormatToString(PixelFormat::kYuyv), "Yuyv");
  EXPECT_EQ(PixelFormatToString(PixelFormat::kBgra8), "Bgra8");
}

TEST(FrameTest, DefaultConstruction) {
  Frame f;
  EXPECT_EQ(f.camera_id(), 0);
  EXPECT_EQ(f.session_id(), 0);
  EXPECT_EQ(f.sequence_number(), 0);
  EXPECT_EQ(f.width(), 0);
  EXPECT_EQ(f.height(), 0);
  EXPECT_EQ(f.stride(), 0);
  EXPECT_TRUE(f.empty());
  EXPECT_EQ(f.size(), 0);
  EXPECT_TRUE(f.is_exposure_timestamp());
}

TEST(FrameTest, AllocatedConstruction) {
  const auto t0 = talos::event::MonotonicTime::from_nanos(123456789);
  Frame f(1, 42, 100, t0, 640, 480, PixelFormat::kRgb8);

  EXPECT_EQ(f.camera_id(), 1);
  EXPECT_EQ(f.session_id(), 42);
  EXPECT_EQ(f.sequence_number(), 100);
  EXPECT_EQ(f.capture_time().nanos(), 123456789);
  EXPECT_EQ(f.capture_timestamp_ns(), 123456789);
  EXPECT_EQ(f.raw_timestamp_ns(), 123456789);
  EXPECT_EQ(f.width(), 640);
  EXPECT_EQ(f.height(), 480);
  EXPECT_EQ(f.stride(), 640 * 3);
  EXPECT_EQ(f.format(), PixelFormat::kRgb8);
  EXPECT_EQ(f.bytes_per_pixel(), 3);
  EXPECT_EQ(f.size(), 640 * 3 * 480);
  EXPECT_FALSE(f.empty());
  EXPECT_NE(f.data(), nullptr);
  EXPECT_NE(f.mutable_data(), nullptr);
}

TEST(FrameTest, CustomStrideAndOwnedBuffer) {
  std::vector<uint8_t> buffer(1000, 0xAB);
  const auto t0 = talos::event::MonotonicTime::from_nanos(500);
  Frame f(2, 1, 5, t0, 30, 20, 50, PixelFormat::kGray8, std::move(buffer));

  EXPECT_EQ(f.camera_id(), 2);
  EXPECT_EQ(f.width(), 30);
  EXPECT_EQ(f.height(), 20);
  EXPECT_EQ(f.stride(), 50);
  EXPECT_EQ(f.size(), 1000);
  EXPECT_EQ(f.data()[0], 0xAB);
  EXPECT_FALSE(f.is_shared());
}

TEST(FrameTest, MetadataAndClockDomain) {
  Frame f;
  f.set_clock_domain(ClockDomain::kDeviceMonotonic);
  EXPECT_EQ(f.clock_domain(), ClockDomain::kDeviceMonotonic);

  f.set_timestamp_kind(TimestampKind::kDelivery);
  EXPECT_EQ(f.timestamp_kind(), TimestampKind::kDelivery);
  EXPECT_FALSE(f.is_exposure_timestamp());

  f.set_mapping_quality(TimestampMappingQuality::kInterpolated);
  EXPECT_EQ(f.mapping_quality(), TimestampMappingQuality::kInterpolated);
}

TEST(FrameTest, CloneAndShareSemantics) {
  Frame orig(0, 1, 10, talos::event::MonotonicTime::from_nanos(100), 10, 10,
             PixelFormat::kGray8);
  orig.mutable_data()[0] = 42;
  orig.mutable_data()[99] = 99;

  // Share buffer
  Frame shared = orig.Share();
  EXPECT_TRUE(shared.is_shared());
  EXPECT_EQ(shared.data()[0], 42);
  EXPECT_EQ(shared.data()[99], 99);
  EXPECT_EQ(shared.data(), orig.data());

  // Mutating shared frame causes copy-on-write
  shared.mutable_data()[0] = 100;
  EXPECT_FALSE(shared.is_shared());
  EXPECT_EQ(shared.data()[0], 100);
  EXPECT_EQ(orig.data()[0], 42);  // Original unchanged

  // Clone frame creates independent copy
  Frame cloned = orig.Clone();
  EXPECT_FALSE(cloned.is_shared());
  EXPECT_EQ(cloned.data()[0], 42);
  EXPECT_NE(cloned.data(), orig.data());
  cloned.mutable_data()[0] = 77;
  EXPECT_EQ(cloned.data()[0], 77);
  EXPECT_EQ(orig.data()[0], 42);
}

}  // namespace
}  // namespace talos::vision
