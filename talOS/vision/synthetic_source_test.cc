#include "talOS/vision/synthetic_source.h"

#include <gtest/gtest.h>

#include <atomic>
#include <cfenv>
#include <chrono>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

namespace talos::vision {
namespace {

TEST(SyntheticCameraSourceTest, RejectsNonFiniteOrNegativeFpsAndClampsPeriod) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  SyntheticCameraSource source(0, "synth", {64, 48, 30.0, PixelFormat::kGray8});
  EXPECT_FALSE(source.SetMode({64, 48, nan, PixelFormat::kGray8}));
  EXPECT_FALSE(source.SetMode({64, 48, inf, PixelFormat::kGray8}));
  EXPECT_FALSE(source.SetMode({64, 48, -inf, PixelFormat::kGray8}));
  EXPECT_FALSE(source.SetMode({64, 48, -1.0, PixelFormat::kGray8}));
  EXPECT_DOUBLE_EQ(source.CurrentMode().fps, 30.0);

  // Constructed directly with NaN/Inf/negative fps: Start() refuses instead of
  // entering duration_cast<nanoseconds>(1.0 / NaN) UB.
  for (const double bad_fps : {nan, inf, -inf, -5.0}) {
    SyntheticCameraSource bad_source(0, "bad",
                                     {64, 48, bad_fps, PixelFormat::kGray8});
    EXPECT_FALSE(bad_source.Start());
    EXPECT_FALSE(bad_source.IsRunning());
    bad_source.Stop();
  }

  // Tiny positive fps (would overflow int64 nanoseconds if unclamped) and huge
  // fps both start, deliver a frame, and stop immediately without hanging.
  for (const double extreme_fps : {1e-300, 1e12}) {
    SyntheticCameraSource extreme_source(
        0, "extreme", {64, 48, extreme_fps, PixelFormat::kGray8});
    std::mutex mu;
    std::condition_variable cv;
    bool got_frame = false;
    extreme_source.SetFrameCallback([&](const Frame&) {
      {
        std::lock_guard lock(mu);
        got_frame = true;
      }
      cv.notify_one();
    });
    ASSERT_TRUE(extreme_source.Start());
    {
      std::unique_lock lock(mu);
      ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds{10},
                              [&] { return got_frame; }));
    }
    extreme_source.Stop();
    EXPECT_FALSE(extreme_source.IsRunning());
  }
}

TEST(SyntheticCameraSourceTest, IgnoresNonFiniteTagDescriptorsWithoutUb) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  SyntheticCameraSource source(0, "synth", {64, 48, 0.0, PixelFormat::kGray8});
  source.AddTag({.tag_id = 0,
                 .center_x = nan,
                 .center_y = 24.0,
                 .size_pixels = 20.0,
                 .rotation_rad = 0.0});
  source.AddTag({.tag_id = 1,
                 .center_x = 32.0,
                 .center_y = inf,
                 .size_pixels = 20.0,
                 .rotation_rad = 0.0});
  source.AddTag({.tag_id = 2,
                 .center_x = 32.0,
                 .center_y = 24.0,
                 .size_pixels = nan,
                 .rotation_rad = 0.0});
  source.AddTag({.tag_id = 3,
                 .center_x = 32.0,
                 .center_y = 24.0,
                 .size_pixels = 20.0,
                 .rotation_rad = nan});
  source.AddTag({.tag_id = 4,
                 .center_x = 1e20,
                 .center_y = 24.0,
                 .size_pixels = 20.0,
                 .rotation_rad = 0.0});
  source.AddTag({.tag_id = 5,
                 .center_x = 32.0,
                 .center_y = 24.0,
                 .size_pixels = 1e20,
                 .rotation_rad = 0.0});

  std::feclearexcept(FE_ALL_EXCEPT);
  const Frame frame = source.GenerateFrame();
  EXPECT_EQ(std::fetestexcept(FE_INVALID), 0);
  ASSERT_EQ(frame.size(), 64u * 48u);
  // Every invalid tag was skipped, leaving the white background untouched.
  for (size_t i = 0; i < frame.size(); ++i) {
    ASSERT_EQ(frame.data()[i], 255u) << "pixel " << i;
  }
}

TEST(SyntheticCameraSourceTest, ConcurrentStartStopNeverTerminatesOrHangs) {
  SyntheticCameraSource source(0, "race",
                               {64, 48, 1000.0, PixelFormat::kGray8});
  std::atomic<int> frames{0};
  source.SetFrameCallback([&](const Frame&) { frames.fetch_add(1); });

  constexpr int kIterations = 40;
  std::thread starter([&] {
    for (int i = 0; i < kIterations; ++i) {
      source.Start();
    }
  });
  std::thread stopper([&] {
    for (int i = 0; i < kIterations; ++i) {
      source.Stop();
    }
  });
  starter.join();
  stopper.join();
  source.Stop();
  EXPECT_FALSE(source.IsRunning());
}

TEST(SyntheticCameraSourceTest, StopFromFrameCallbackDoesNotSelfJoin) {
  SyntheticCameraSource source(0, "self_stop",
                               {64, 48, 1000.0, PixelFormat::kGray8});
  std::mutex mu;
  std::condition_variable cv;
  bool stopped = false;
  source.SetFrameCallback([&](const Frame&) {
    source.Stop();
    {
      std::lock_guard lock(mu);
      stopped = true;
    }
    cv.notify_one();
  });
  ASSERT_TRUE(source.Start());
  {
    std::unique_lock lock(mu);
    ASSERT_TRUE(
        cv.wait_for(lock, std::chrono::seconds{10}, [&] { return stopped; }));
  }
  source.Stop();
  EXPECT_FALSE(source.IsRunning());
}

}  // namespace
}  // namespace talos::vision
