// Replay applies detector overrides to recorded frames rendered without a robot
// dependency, preserving valid observations unless the quality gate rejects them.

#include "talOS/vision/recording/replay.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "talOS/vision/field_sim/field_layout_loader.h"
#include "talOS/vision/field_sim/sim_camera.h"
#include "talOS/vision/field_sim/tag_patterns.h"
#include "talOS/vision/field_sim/tag_rasterizer.h"
#include "talOS/vision/localization.h"
#include "talOS/vision/pipeline.h"
#include "talOS/vision/recording/recorder.h"
#include "talOS/vision/recording/recording.h"

namespace talos::vision::recording {
namespace {

using talos::event::MonotonicTime;

constexpr double kTagSize = 0.1651;
constexpr int kTagId = 7;
constexpr int kWidth = 640;
constexpr int kHeight = 480;

field_sim::FieldLayoutData MakeLayout() {
  field_sim::FieldLayoutData data;
  data.layout.set_default_tag_size_m(kTagSize);
  data.dims.length_m = 16.0;
  data.dims.width_m = 8.0;
  Eigen::Matrix3d upright;
  upright << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
  data.layout.SetTagPose(kTagId, talos::geometry::SE3<double>(
                                     talos::geometry::SO3<double>(upright),
                                     Eigen::Vector3d(3.5, 2.0, 0.5)));
  return data;
}

field_sim::SimCameraView MakeCamera() {
  field_sim::SimCameraView cam;
  cam.intrinsics.fx = 500.0;
  cam.intrinsics.fy = 500.0;
  cam.intrinsics.cx = 320.0;
  cam.intrinsics.cy = 240.0;
  cam.intrinsics.image_width = kWidth;
  cam.intrinsics.image_height = kHeight;
  cam.mounting = field_sim::MountFromXyzYpr(0.0, 0.0, 0.5, 0.0, 0.0, 0.0);
  cam.max_range_m = 8.0;
  return cam;
}

// Render and record a clear tag with a valid observation.
void RecordTagFrame(FrameRecorder& recorder,
                    std::map<int, field_sim::TagPattern>* patterns) {
  const auto layout = MakeLayout();
  const auto camera = MakeCamera();
  const auto pose = field_sim::RobotPoseOnField(2.0, 2.0, 0.0);

  std::vector<std::uint8_t> gray(static_cast<std::size_t>(kWidth) * kHeight);
  field_sim::RenderScratch scratch;
  const int painted = field_sim::RenderInstantaneousFieldView(
      layout.layout, *patterns, camera, pose, kTagSize,
      field_sim::SimRenderOptions{}, &gray, &scratch.detections);
  ASSERT_GT(painted, 0);

  PipelineConfig config;
  config.backend_name = "cpu";
  config.intrinsics = camera.intrinsics;
  config.mounting = camera.mounting;
  config.layout = layout.layout;
  AprilTagPipeline pipeline{config};

  Frame frame{0,
              0,
              0,
              MonotonicTime::from_nanos(0),
              kWidth,
              kHeight,
              kWidth,
              PixelFormat::kGray8,
              std::move(gray)};
  PipelineResult result = pipeline.ProcessFrame(frame);
  std::vector<TagDetection3D> detections = result.detections;
  AnnotateDetectionReasons(detections, layout.layout, 8.0,
                           config.validation_options);

  FrameRecord record;
  record.capture_ns = static_cast<std::uint64_t>(1'000'000);
  record.observation.valid = true;
  record.observation.x_m = 2.0;
  record.observation.y_m = 2.0;
  record.observation.yaw_rad = 0.0;
  record.observation.x_std_m = 0.02;
  record.observation.y_std_m = 0.02;
  record.observation.yaw_std_rad = 0.03;
  record.observation.tag_count = static_cast<int>(detections.size());
  for (const auto& d : detections) {
    DetectionRecord dr;
    dr.tag_id = d.tag_id;
    dr.hamming = d.hamming;
    dr.decision_margin = d.decision_margin;
    dr.corners_px = d.corners;
    dr.valid = d.valid && d.reject_reason.empty();
    dr.reprojection_error_px = d.reprojection_error;
    dr.ambiguity = d.ambiguity;
    dr.reject_reason = d.reject_reason.empty()
                           ? std::optional<std::string>{}
                           : std::optional<std::string>{d.reject_reason};
    record.detections.push_back(std::move(dr));
  }
  record.truth = TruthRecord{2.0, 2.0, 0.0};
  record.reason = std::string{FrameReason(record)};
  recorder.Record(frame, record);
}

class ReplayTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = std::filesystem::path{::testing::TempDir()} /
           ("talos_replay_" + std::to_string(::getpid()));
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
    ASSERT_TRUE(field_sim::LoadTag36h11Patterns(&patterns_, &error_)) << error_;
  }
  void TearDown() override { std::filesystem::remove_all(dir_); }

  std::filesystem::path dir_;
  std::map<int, field_sim::TagPattern> patterns_;
  std::string error_;
};


TEST_F(ReplayTest, RaisingMinDecisionMarginReducesValidDetections) {
  FrameRecorder::Options opts;
  opts.output_dir = dir_;
  opts.sample_hz = 1e12;
  opts.rejected_hz = 1e12;
  opts.context.camera = "test_camera";
  opts.context.width = kWidth;
  opts.context.height = kHeight;
  opts.context.intrinsics = MakeCamera().intrinsics;
  opts.context.mounting = {0.0, 0.0, 0.5, 0.0, 0.0, 0.0};
  opts.context.detector = {.backend = "cpu",
                           .decimate = 1.0,
                           .threads = 1,
                           .sigma = 0.0,
                           .refine_edges = true,
                           .hamming_threshold = 1,
                           .min_decision_margin = 30.0};
  {
    FrameRecorder recorder{std::move(opts)};
    RecordTagFrame(recorder, &patterns_);
  }

  const auto layout = MakeLayout();
  ReplayStats default_stats;
  std::string error;
  ASSERT_TRUE(ReplayFrames(dir_.string(), layout, DetectorOverrides{},
                           &default_stats, &error))
      << error;
  EXPECT_EQ(default_stats.frames, 1u);
  EXPECT_EQ(default_stats.valid_before, 1u);
  EXPECT_EQ(default_stats.valid_after, 1u);
  EXPECT_GT(default_stats.rejections_after["(valid)"], 0u);

  DetectorOverrides high;
  high.min_decision_margin = 1000.0;  // maximum allowed, rejects any real tag
  ReplayStats high_stats;
  ASSERT_TRUE(ReplayFrames(dir_.string(), layout, high, &high_stats, &error))
      << error;
  EXPECT_EQ(high_stats.rejections_after["(valid)"], 0u);
  EXPECT_GT(high_stats.rejections_after["decision_margin"], 0u);
}

}  // namespace
}  // namespace talos::vision::recording
