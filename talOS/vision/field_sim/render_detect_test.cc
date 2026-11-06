// Render-to-detect round trip: rasterize a synthetic tag image, run the REAL
// AprilTag detector pipeline on it, and prove the detection matches the
// projection. This is the test that makes the image-level sim honest — if
// the rasterizer's patterns, polarity, or row orientation ever disagree with
// what the detector decodes, this fails instead of the estimator.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "talOS/vision/field_sim/field_layout_loader.h"
#include "talOS/vision/field_sim/sim_camera.h"
#include "talOS/vision/field_sim/tag_patterns.h"
#include "talOS/vision/field_sim/tag_rasterizer.h"
#include "talOS/vision/localization.h"
#include "talOS/vision/pipeline.h"

namespace talos::vision::field_sim {
namespace {

constexpr double kTagSize = 0.1651;  // Real 36h11 FRC tag.
constexpr int kTagId = 7;

SimCameraView RoundTripCamera() {
  SimCameraView cam;
  cam.intrinsics.fx = 500.0;
  cam.intrinsics.fy = 500.0;
  cam.intrinsics.cx = 320.0;
  cam.intrinsics.cy = 240.0;
  cam.intrinsics.image_width = 640;
  cam.intrinsics.image_height = 480;
  cam.mounting = MountFromXyzYpr(0.0, 0.0, 0.5, 0.0, 0.0, 0.0);
  cam.max_range_m = 10.0;
  return cam;
}

// Upright tag 1.5 m ahead at lens height, face toward the robot
// (tag +X normal maps to field -X, tag +Z up maps to field +Z).
FieldLayout RoundTripLayout() {
  FieldLayout layout(kTagSize);
  Eigen::Matrix3d upright;
  upright << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
  layout.SetTagPose(kTagId, talos::geometry::SE3<double>(
                                talos::geometry::SO3<double>(upright),
                                Eigen::Vector3d(1.5, 0.0, 0.5)));
  return layout;
}

TEST(RenderDetect, PatternsLoadForAllFieldTags) {
  std::map<int, TagPattern> patterns;
  std::string error;
  ASSERT_TRUE(LoadTag36h11Patterns(&patterns, &error)) << error;
  EXPECT_GE(patterns.size(), 33u);
  for (int id = 1; id <= 32; ++id) {
    const auto it = patterns.find(id);
    ASSERT_TRUE(it != patterns.end()) << "missing pattern " << id;
    int white = 0;
    for (const auto& row : it->second.bits) {
      for (bool b : row) white += b ? 1 : 0;
    }
    // An all-white or all-black grid cannot be a valid code.
    EXPECT_GT(white, 0) << "tag " << id;
    EXPECT_LT(white, 36) << "tag " << id;
  }
}

TEST(RenderDetect, RenderedTagDecodesThroughTheRealPipeline) {
  std::map<int, TagPattern> patterns;
  std::string error;
  ASSERT_TRUE(LoadTag36h11Patterns(&patterns, &error)) << error;

  const FieldLayout layout = RoundTripLayout();
  const SimCameraView cam = RoundTripCamera();
  const auto T_field_robot = RobotPoseOnField(0.0, 0.0, 0.0);

  std::vector<uint8_t> gray;
  const SimRenderOptions render_opts;
  const int painted = RenderFieldView(layout, patterns, cam, T_field_robot,
                                      kTagSize, render_opts, &gray);
  ASSERT_EQ(painted, 1);
  ASSERT_EQ(gray.size(), 640u * 480u);

  PipelineConfig config;
  config.kind = "apriltag";
  config.backend_name = "cpu";
  config.revision = 1;
  config.intrinsics = cam.intrinsics;
  config.mounting = cam.mounting;
  config.layout = layout;
  config.parameters["decimate"] = "1.0";
  config.parameters["threads"] = "1";
  AprilTagPipeline pipeline(config);

  Frame frame(0, /*session_id=*/1, /*sequence_number=*/1,
              talos::event::MonotonicTime::from_nanos(1000), 640, 480,
              /*stride=*/640, PixelFormat::kGray8, std::move(gray));
  PipelineResult res = pipeline.ProcessFrame(frame);
  ASSERT_TRUE(res.success) << res.error_message;
  ASSERT_EQ(res.detections.size(), 1u) << "expected exactly the rendered tag";
  const DetectionResult& det = res.detections[0];
  EXPECT_EQ(det.tag_id, kTagId);
  EXPECT_TRUE(det.valid);
  // Corners land where the projector put them (detector refines edges, so
  // allow a couple of pixels of honest disagreement). Projector and
  // detector share the model-corner convention, but match up to rotation
  // and winding direction (dihedral group) so the test stays green if the
  // detector's start corner ever differs.
  // (Verified against apriltag-imgs tag36h11-0.png: the real detector
  // reports the same corner SET for physical truth and for our render.)
  const std::vector<SimTagDetection> projected =
      SimulateVisibleTags(layout, cam, T_field_robot, kTagSize);
  ASSERT_EQ(projected.size(), 1u);
  // Rendering and exact projection must share the estimator's optical frame.
  const auto exact_pose = ComputeRobotFieldPose(
      projected[0].tag_id, 0, projected[0].T_camera_tag, cam.mounting, layout);
  ASSERT_TRUE(exact_pose.validity.valid);
  EXPECT_LT((exact_pose.T_field_robot.matrix() - T_field_robot.matrix()).norm(),
            1e-12);
  auto near = [&](int i, int j) {
    const auto corner_index = 2 * static_cast<size_t>(i);
    return std::abs(det.corners[corner_index] - projected[0].corners[j].x()) <=
               2.5 &&
           std::abs(det.corners[corner_index + 1] -
                    projected[0].corners[j].y()) <= 2.5;
  };
  bool quad_matched = false;
  for (int dir = 0; dir < 2 && !quad_matched; ++dir) {
    for (int rot = 0; rot < 4 && !quad_matched; ++rot) {
      quad_matched = true;
      for (int i = 0; i < 4; ++i) {
        const int j = (dir == 0) ? (i + rot) % 4 : (rot - i + 8) % 4;
        if (!near(i, j)) {
          quad_matched = false;
          break;
        }
      }
    }
  }
  EXPECT_TRUE(quad_matched) << "detected quad does not match projection";
  // The estimated pose is close to ground truth: translation within 5 cm.
  const Eigen::Vector3d t = det.T_camera_tag.translation();
  EXPECT_NEAR(t.x(), 0.0, 0.05);
  EXPECT_NEAR(t.y(), 0.0, 0.05);
  EXPECT_NEAR(t.z(), 1.5, 0.05);
  EXPECT_GT(det.decision_margin, 0.0f);
  // End-to-end field composition: the pipeline's own robot field pose must
  // recover the true robot pose (origin). This proves the sim mounting, the
  // backend camera frame, and the layout tag frame all agree with each
  // other — the same composition live code runs on real images.
  ASSERT_EQ(res.field_poses.size(), 1u);
  EXPECT_TRUE(res.field_poses[0].valid);
  const Eigen::Vector3d fr = res.field_poses[0].T_field_robot.translation();
  EXPECT_NEAR(fr.x(), 0.0, 0.05);
  EXPECT_NEAR(fr.y(), 0.0, 0.05);
  EXPECT_NEAR(fr.z(), 0.0, 0.05);
  // Orientation too: the recovered robot rotation is identity. Translation
  // alone cannot arbitrate frame fixes (tag-frame relabelings leave the
  // origin in place); a flipped pattern or a wrong frame conversion shows
  // up here as tens of degrees.
  const Eigen::Matrix3d fR =
      res.field_poses[0].T_field_robot.rotation().rotationMatrix();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      EXPECT_NEAR(fR(r, c), (r == c) ? 1.0 : 0.0, 0.03)
          << "r=" << r << " c=" << c;
    }
  }
}

TEST(RenderDetect, RasterizationDoesNotUseDetectorRangeOrSizeGates) {
  std::map<int, TagPattern> patterns;
  std::string error;
  ASSERT_TRUE(LoadTag36h11Patterns(&patterns, &error)) << error;
  auto camera = RoundTripCamera();
  camera.min_tag_px = 1000;
  camera.max_range_m = .1;
  const auto layout = RoundTripLayout();
  const auto robot = RobotPoseOnField(0, 0, 0);
  EXPECT_TRUE(SimulateVisibleTags(layout, camera, robot, kTagSize).empty());
  EXPECT_EQ(SimulateVisibleTags(layout, camera, robot, kTagSize, true).size(),
            1);
  std::vector<uint8_t> pixels;
  EXPECT_EQ(
      RenderFieldView(layout, patterns, camera, robot, kTagSize, {}, &pixels),
      1);
  EXPECT_NE(std::find(pixels.begin(), pixels.end(), uint8_t{0}), pixels.end());
}

TEST(RenderDetect, PartiallyClippedTagsStillContributePixels) {
  std::map<int, TagPattern> patterns;
  std::string error;
  ASSERT_TRUE(LoadTag36h11Patterns(&patterns, &error)) << error;
  const auto camera = RoundTripCamera();
  auto layout = RoundTripLayout();
  const auto robot = RobotPoseOnField(0, .94, 0);
  EXPECT_TRUE(SimulateVisibleTags(layout, camera, robot, kTagSize).empty());
  const auto expected =
      SimulateVisibleTags(layout, camera, robot, kTagSize, true);
  ASSERT_EQ(expected.size(), 1);
  EXPECT_GT(expected[0].center.x(), 600);
  std::vector<uint8_t> pixels;
  EXPECT_EQ(
      RenderFieldView(layout, patterns, camera, robot, kTagSize, {}, &pixels),
      1);
  EXPECT_NE(std::find(pixels.begin(), pixels.end(), uint8_t{0}), pixels.end());
}

}  // namespace
}  // namespace talos::vision::field_sim
