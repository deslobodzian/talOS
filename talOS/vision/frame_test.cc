#include "talOS/vision/frame.h"

#include <gtest/gtest.h>

namespace talos::vision {
namespace {

TEST(FrameTest, AllocatesPackedImageStorage) {
  const auto t0 = talos::event::MonotonicTime::from_nanos(123456789);
  Frame f(1, 42, 100, t0, 640, 480, PixelFormat::kRgb8);

  EXPECT_EQ(f.stride(), 640 * 3);
  EXPECT_EQ(f.size(), 640 * 3 * 480);
  EXPECT_FALSE(f.empty());
  EXPECT_NE(f.data(), nullptr);
  EXPECT_NE(f.mutable_data(), nullptr);
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
