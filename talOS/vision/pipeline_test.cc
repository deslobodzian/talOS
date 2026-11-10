#include "talOS/vision/pipeline.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "talOS/vision/metal/metal_apriltag_backend.h"
#include "talOS/vision/synthetic_source.h"

namespace talos::vision {
namespace {

PipelineConfig Config() {
  PipelineConfig config;
  config.intrinsics.fx = 500;
  config.intrinsics.fy = 500;
  config.intrinsics.cx = 320;
  config.intrinsics.cy = 240;
  config.intrinsics.image_width = 640;
  config.intrinsics.image_height = 480;
  config.mounting = CameraMounting::Identity();
  config.layout.set_default_tag_size_m(0.20);
  config.layout.SetTagPose(
      0, talos::geometry::SE3<double>(talos::geometry::SO3<double>::Identity(),
                                      Eigen::Vector3d(3, 0, 0.5)));
  config.validation_options.max_ambiguity = 1;
  return config;
}

Frame TagFrame() {
  SyntheticCameraSource source(0, "synthetic",
                               {640, 480, 0, PixelFormat::kGray8});
  source.AddTag(
      {.tag_id = 0, .center_x = 320, .center_y = 240, .size_pixels = 160});
  return source.GenerateFrame();
}

TEST(PipelineTest, DetectionAndCameraRelativePose) {
  AprilTagPipeline pipeline(Config());
  auto result = pipeline.ProcessFrame(TagFrame());
  ASSERT_TRUE(result.success) << result.error_message;
  ASSERT_EQ(result.detections.size(), 1);
  const auto& det = result.detections.front();
  EXPECT_EQ(det.tag_id, 0);
  EXPECT_EQ(det.hamming, 0);
  EXPECT_NEAR(det.center_x, 320, 5);
  EXPECT_NEAR(det.center_y, 240, 5);
  EXPECT_GE(det.decision_margin, 30);
  EXPECT_TRUE(det.valid) << "margin=" << det.decision_margin;
  EXPECT_GE(det.secondary_reprojection_error_px, 0);
  EXPECT_TRUE(det.secondary_T_camera_tag.matrix().allFinite());
}

TEST(PipelineTest, BackendRejectsFramesLargerThanTheirStorage) {
  // A truncated buffer, or a stride shorter than one row, declares pixels the
  // frame does not store: detection fails instead of reading past them.
  const auto time = talos::event::MonotonicTime::from_nanos(1);
  constexpr std::size_t kHalfImageBytes = std::size_t{640} * 480 / 2;
  const Frame truncated(0, 1, 0, time, 640, 480, 640, PixelFormat::kGray8,
                        std::vector<std::uint8_t>(kHalfImageBytes));
  const Frame short_stride(0, 1, 0, time, 640, 480, 320, PixelFormat::kGray8,
                           std::vector<std::uint8_t>(kHalfImageBytes));
  CpuAprilTagBackend backend;
  EXPECT_FALSE(backend.Execute(truncated).success);
  EXPECT_FALSE(backend.Execute(short_stride).success);
}

// Keep the real decoded geometry, varying only the decode quality to exercise
// the pipeline's post-decode validity gate independently of decoder pruning.
class DecodeQualityBackend : public ComputeBackend {
 public:
  float margin = 30;
  int hamming = 1;
  std::string name() const override { return "cpu"; }
  std::string device_info() const override { return "test decode quality"; }
  bool supports_gpu() const override { return false; }
  BackendExecutionResult Execute(const Frame& frame) override {
    auto result = cpu_.Execute(frame);
    for (auto& detection : result.detections) {
      detection.decision_margin = margin;
      detection.hamming = hamming;
    }
    return result;
  }

 private:
  CpuAprilTagBackend cpu_;
};

TEST(PipelineTest, DecodeQualityLimitsInvalidateAndExcludeFromLocalization) {
  auto backend = std::make_shared<DecodeQualityBackend>();
  const auto config = Config();
  AprilTagPipeline pipeline(config, backend);
  const auto frame = TagFrame();
  auto result = pipeline.ProcessFrame(frame);
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.detections.size(), 1);
  ASSERT_TRUE(result.detections.front().valid);
  backend->margin = 29;
  for (int test = 0; test < 2; ++test) {
    if (test == 1) {
      backend->margin = 30;
      backend->hamming = 2;
    }
    result = pipeline.ProcessFrame(frame);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.detections.size(), 1);
    EXPECT_FALSE(result.detections.front().valid);
    const auto pose = SolveMultiTagRobotPose(
        result.detections, 0, config.mounting, config.layout, config.intrinsics,
        config.validation_options);
    EXPECT_FALSE(pose.validity.valid);
    EXPECT_EQ(pose.tag_count, 0);
  }
  auto relaxed = config;
  relaxed.detector_options.max_hamming = 2;
  AprilTagPipeline relaxed_pipeline(relaxed, backend);
  EXPECT_TRUE(relaxed_pipeline.ProcessFrame(frame).detections.front().valid);
}

class FixedCornersBackend : public ComputeBackend {
 public:
  RawTagDetection detection;
  std::string name() const override { return "cpu"; }
  std::string device_info() const override { return "pinned pose fixture"; }
  bool supports_gpu() const override { return false; }
  BackendExecutionResult Execute(const Frame&) override {
    BackendExecutionResult result;
    result.success = true;
    result.effective_backend = "cpu";
    result.detections.push_back(detection);
    return result;
  }
};

TEST(PipelineTest, DoesNotInventASecondPoseWhenAmbiguitySearchFails) {
  auto config = Config();
  config.intrinsics.fx = config.intrinsics.fy = 700;
  config.intrinsics.cx = 640;
  config.intrinsics.cy = 360;
  config.layout.set_default_tag_size_m(0.1651);
  auto backend = std::make_shared<FixedCornersBackend>();
  backend->detection.id = 0;
  backend->detection.decision_margin = 100;
  backend->detection.corners = {488.490112f, 242.771851f, 497.506744f,
                                250.814423f, 504.358826f, 243.886002f,
                                498.08548f,  236.256561f};
  AprilTagPipeline pipeline(config, backend);
  const auto result = pipeline.ProcessFrame({});
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.detections.size(), 1);
  EXPECT_DOUBLE_EQ(result.detections[0].ambiguity, 1);
  EXPECT_TRUE(std::isinf(result.detections[0].secondary_reprojection_error_px));
}

void ExpectInvalid(const PipelineConfig& config, const char* field) {
  SCOPED_TRACE(field);
  EXPECT_THROW(AprilTagPipeline{config}, std::invalid_argument);
}

TEST(PipelineTest, RejectsInvalidTypedSettings) {
  for (const float value :
       {0.0f, 9.0f, std::numeric_limits<float>::quiet_NaN()}) {
    auto config = Config();
    config.detector_options.quad_decimate = value;
    ExpectInvalid(config, "detector.decimate");
  }
  for (const int value : {0, 17}) {
    auto config = Config();
    config.detector_options.threads = value;
    ExpectInvalid(config, "detector.threads");
  }
  for (const float value :
       {-1.0f, 5.0f, std::numeric_limits<float>::infinity()}) {
    auto config = Config();
    config.detector_options.quad_sigma = value;
    ExpectInvalid(config, "detector.sigma");
  }
  for (const int value : {-1, 3}) {
    auto config = Config();
    config.detector_options.max_hamming = value;
    ExpectInvalid(config, "detector.hamming_threshold");
  }
  for (const double value :
       {-1.0, 1001.0, std::numeric_limits<double>::quiet_NaN()}) {
    auto config = Config();
    config.min_decision_margin = value;
    ExpectInvalid(config, "detector.min_decision_margin");
  }
  auto config = Config();
  config.backend_name = "invalid";
  ExpectInvalid(config, "backend");
}

TEST(PipelineTest, AutoFallbackHonorsSettingsAndForcedMetalRejectsThem) {
  auto config = Config();
  config.backend_name = "auto";
  config.detector_options.quad_sigma = 1;
  config.detector_options.quad_decimate = 1.5;
  const auto frame = TagFrame();
  metal::MetalAprilTagBackend backend(config.detector_options);
  const auto backend_result = backend.Execute(frame);
  ASSERT_TRUE(backend_result.success) << backend_result.error_message;
  EXPECT_EQ(backend_result.effective_backend, "cpu");
  AprilTagPipeline pipeline(config);
  const auto result = pipeline.ProcessFrame(frame);
  ASSERT_TRUE(result.success) << result.error_message;
  ASSERT_EQ(result.detections.size(), 1);
  EXPECT_EQ(result.detections.front().tag_id, 0);
  config.backend_name = "metal";
  ExpectInvalid(config, "detector.sigma");
  config.detector_options.quad_sigma = 0;
  ExpectInvalid(config, "detector.decimate");
}

}  // namespace
}  // namespace talos::vision
