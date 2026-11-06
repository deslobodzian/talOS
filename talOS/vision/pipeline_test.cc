#include "talOS/vision/pipeline.h"

#include <gtest/gtest.h>

#include <memory>

#include "talOS/vision/synthetic_source.h"

namespace talos::vision {
namespace {

TEST(PipelineTest, AprilTagPipelineDetectionAndLocalization) {
  // Setup synthetic camera rendering Tag 0 at center (320, 240)
  SyntheticCameraSource source(0, "synth_cam_0",
                               {640, 480, 0.0, PixelFormat::kGray8});
  SyntheticTagDescriptor tag_desc{
      .tag_id = 0,
      .center_x = 320.0,
      .center_y = 240.0,
      .size_pixels = 160.0,
      .rotation_rad = 0.0,
  };
  source.AddTag(tag_desc);
  Frame frame = source.GenerateFrame();

  // Setup pipeline config
  PipelineConfig config;
  config.kind = "apriltag";
  config.backend_name = "cpu";
  config.revision = 3;

  config.intrinsics.fx = 500.0;
  config.intrinsics.fy = 500.0;
  config.intrinsics.cx = 320.0;
  config.intrinsics.cy = 240.0;
  config.intrinsics.image_width = 640;
  config.intrinsics.image_height = 480;

  config.mounting = CameraMounting::Identity();
  config.layout.set_default_tag_size_m(0.20);
  config.layout.SetTagPose(
      0, talos::geometry::SE3<double>(talos::geometry::SO3<double>::Identity(),
                                      Eigen::Vector3d(3.0, 0.0, 0.5)));

  config.parameters["decimate"] = "1.0";
  config.parameters["threads"] = "1";
  config.parameters["refine_edges"] = "true";
  // A perfectly frontal planar tag has intrinsically ambiguous dual poses.
  config.validation_options.max_ambiguity = 1.0;

  auto pipeline = std::make_unique<AprilTagPipeline>(config);
  ASSERT_NE(pipeline, nullptr);
  EXPECT_EQ(pipeline->kind(), "apriltag");
  EXPECT_EQ(pipeline->current_revision(), 3);

  PipelineResult res = pipeline->ProcessFrame(frame);
  EXPECT_TRUE(res.success);
  EXPECT_EQ(res.revision, 3);
  EXPECT_EQ(res.camera_id, 0);

  // Check stage timings
  EXPECT_GT(res.timings.total_us, 0);

  // Verify detection
  ASSERT_FALSE(res.detections.empty());
  const auto& det = res.detections[0];
  EXPECT_EQ(det.tag_id, 0);
  EXPECT_EQ(det.hamming, 0);
  EXPECT_NEAR(det.center_x, 320.0, 5.0);
  EXPECT_NEAR(det.center_y, 240.0, 5.0);
  EXPECT_TRUE(det.valid);
  EXPECT_GE(det.secondary_reprojection_error_px, 0);
  EXPECT_TRUE(det.secondary_T_camera_tag.matrix().allFinite());

  // Verify field pose
  ASSERT_FALSE(res.field_poses.empty());
  const auto& fp = res.field_poses[0];
  EXPECT_EQ(fp.tag_id, 0);
  EXPECT_EQ(fp.camera_id, 0);
  EXPECT_EQ(fp.revision, 3);
  EXPECT_TRUE(fp.valid);
}

TEST(PipelineTest, ConfigurationUpdateAndInvalidPreservation) {
  PipelineConfig detector;
  detector.backend_name = "cpu";
  AprilTagPipeline apriltag(detector);
  detector.revision = 2;
  EXPECT_TRUE(apriltag.ApplyConfig(detector));
  EXPECT_EQ(apriltag.current_revision(), 2);
  detector.backend_name = "invalid";
  EXPECT_FALSE(apriltag.ApplyConfig(detector));
  EXPECT_EQ(apriltag.current_config().backend_name, "cpu");
}

TEST(PipelineTest, AutoFallbackHonorsSettingsAndForcedMetalRejectsThem) {
  SyntheticCameraSource source(0, "fallback",
                               {320, 240, 0, PixelFormat::kGray8});
  source.AddTag(
      {.tag_id = 0, .center_x = 160, .center_y = 120, .size_pixels = 100});
  PipelineConfig config;
  config.backend_name = "auto";
  config.parameters = {{"sigma", "1"}, {"decimate", "1.5"}};
  AprilTagPipeline pipeline(config);
  auto result = pipeline.ProcessFrame(source.GenerateFrame());
  ASSERT_TRUE(result.success) << result.error_message;
  EXPECT_EQ(result.effective_backend, "cpu");
  EXPECT_FALSE(result.fallback_reason.empty());
  ASSERT_EQ(result.detections.size(), 1);
  EXPECT_EQ(result.detections[0].tag_id, 0);
  config.backend_name = "metal";
  config.revision = 2;
  EXPECT_FALSE(pipeline.ApplyConfig(config));
  EXPECT_EQ(pipeline.current_revision(), 1);
  config.backend_name = "cpu";
  config.parameters["threads"] = "1.5";
  EXPECT_FALSE(pipeline.ApplyConfig(config));
  config.parameters["threads"] = "2";
  EXPECT_TRUE(pipeline.ApplyConfig(config));
  EXPECT_EQ(pipeline.current_revision(), 2);
}

}  // namespace
}  // namespace talos::vision
