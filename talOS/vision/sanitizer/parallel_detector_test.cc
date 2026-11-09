#include <gtest/gtest.h>

#include <cstdlib>
#include <cstring>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#include "talOS/vision/compute_backend.h"
#include "talOS/vision/frame.h"

namespace talos::vision {
namespace {

cv::Mat Fixture() {
  const char* root = std::getenv("TEST_SRCDIR");
  const char* workspace = std::getenv("TEST_WORKSPACE");
  return cv::imread(std::string(root ? root : ".") + "/" +
                        (workspace ? workspace : "_main") +
                        "/talOS/vision/testdata/scene_single_tag0.png",
                    cv::IMREAD_GRAYSCALE);
}

// Production Linux settings: CPU, two AprilTag workers, decimation two,
// sigma zero, edge refinement enabled. Exercise repeated worker-pool handoffs.
TEST(ParallelDetector, AprilTagRowBands) {
  const cv::Mat gray = Fixture();
  ASSERT_FALSE(gray.empty());
  CpuAprilTagBackend backend(2, 2.0, 0.0, true);
  Frame frame(0, 1, 1, event::MonotonicTime::from_nanos(1), gray.cols,
              gray.rows, PixelFormat::kGray8);
  std::memcpy(frame.mutable_data(), gray.data, gray.total());
  for (int iteration = 0; iteration < 30; ++iteration) {
    const auto result = backend.Execute(frame);
    ASSERT_TRUE(result.success);
    ASSERT_FALSE(result.detections.empty());
    EXPECT_EQ(result.detections.front().id, 0);
  }
}

// This is the actual backend's cvtColor path, not an unrelated parallel_for_.
// Force four workers so this regression remains parallel on small CI hosts.
TEST(ParallelDetector, OpenCvColorConversion) {
  cv::setNumThreads(4);
  ASSERT_EQ(cv::getNumThreads(), 4);
  const cv::Mat gray = Fixture();
  ASSERT_FALSE(gray.empty());
  cv::Mat large;
  cv::resize(gray, large, cv::Size(1920, 1080), 0, 0, cv::INTER_NEAREST);
  cv::Mat bgr;
  cv::cvtColor(large, bgr, cv::COLOR_GRAY2BGR);
  CpuAprilTagBackend backend(2, 2.0, 0.0, true);
  Frame frame(0, 1, 1, event::MonotonicTime::from_nanos(1), bgr.cols, bgr.rows,
              PixelFormat::kBgr8);
  std::memcpy(frame.mutable_data(), bgr.data, bgr.total() * bgr.elemSize());
  for (int iteration = 0; iteration < 30; ++iteration) {
    const auto result = backend.Execute(frame);
    ASSERT_TRUE(result.success);
    ASSERT_FALSE(result.detections.empty());
  }
}

}  // namespace
}  // namespace talos::vision
