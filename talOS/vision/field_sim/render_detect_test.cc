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
#include <random>
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

TEST(RenderDetect, SensorNoiseHasConfiguredStatisticsAndAdvancesReproducibly) {
  auto first = RoundTripCamera();
  first.sensor_noise_std_gray = 7.0;
  std::mt19937_64 first_rng(12345), replay_rng(12345);
  auto replay = first;
  RenderScratch scratch;
  const FieldLayout empty(kTagSize);
  SimRenderOptions options;
  options.background_gray = 128;
  std::vector<uint8_t> a, b, next;
  const auto pose = RobotPoseOnField(0, 0, 0);
  RenderFieldView(empty, {}, first, pose, kTagSize, options, &a,
                  Eigen::Vector3d::Zero(), 0.0, &first_rng, &scratch);
  RenderFieldView(empty, {}, replay, pose, kTagSize, options, &b,
                  Eigen::Vector3d::Zero(), 0.0, &replay_rng, &scratch);
  EXPECT_EQ(a, b);
  RenderFieldView(empty, {}, first, pose, kTagSize, options, &next,
                  Eigen::Vector3d::Zero(), 0.0, &first_rng, &scratch);
  EXPECT_NE(a, next);
  RenderFieldView(empty, {}, replay, pose, kTagSize, options, &b,
                  Eigen::Vector3d::Zero(), 0.0, &replay_rng, &scratch);
  EXPECT_EQ(next, b);
  double mean = 0.0;
  double square = 0.0;
  for (uint8_t value : a) {
    const double residual = static_cast<double>(value) - 128.0;
    mean += residual;
    square += residual * residual;
  }
  mean /= static_cast<double>(a.size());
  const double variance = square / static_cast<double>(a.size()) - mean * mean;
  EXPECT_NEAR(mean, 0.0, 0.06);
  EXPECT_NEAR(std::sqrt(variance), 7.0, 0.06);
}

TEST(RenderDetect, ExposureGainIsFrameCoherentAndHasConfiguredVariance) {
  auto camera = RoundTripCamera();
  camera.intrinsics.image_width = 8;
  camera.intrinsics.image_height = 8;
  camera.exposure_gain_std = 0.1;
  std::mt19937_64 rng(84);
  RenderScratch scratch;
  const FieldLayout empty(kTagSize);
  SimRenderOptions options;
  options.background_gray = 100;
  std::vector<uint8_t> pixels;
  double sum = 0.0;
  double square = 0.0;
  constexpr int frames = 2000;
  for (int i = 0; i < frames; ++i) {
    RenderFieldView(empty, {}, camera, RobotPoseOnField(0, 0, 0), kTagSize,
                    options, &pixels, Eigen::Vector3d::Zero(), 0.0, &rng,
                    &scratch);
    ASSERT_FALSE(pixels.empty());
    EXPECT_TRUE(std::all_of(pixels.begin(), pixels.end(),
                            [&](uint8_t p) { return p == pixels.front(); }));
    sum += pixels.front();
    square += static_cast<double>(pixels.front()) * pixels.front();
  }
  EXPECT_NEAR(sum / frames, 100.0, 0.8);
  EXPECT_NEAR(std::sqrt(square / frames - std::pow(sum / frames, 2)), 10.0,
              0.8);
}

TEST(RenderDetect, CornerNoiseNeverChangesRenderedPixels) {
  auto camera = RoundTripCamera();
  const auto layout = RoundTripLayout();
  std::map<int, TagPattern> patterns;
  std::string error;
  ASSERT_TRUE(LoadTag36h11Patterns(&patterns, &error)) << error;
  std::vector<uint8_t> exact, noisy;
  std::mt19937_64 rng(12);
  RenderScratch scratch;
  RenderFieldView(layout, patterns, camera, RobotPoseOnField(0, 0, 0), kTagSize,
                  {}, &exact, Eigen::Vector3d::Zero(), 0.0, &rng, &scratch);
  camera.pixel_noise_std = 100.0;
  RenderFieldView(layout, patterns, camera, RobotPoseOnField(0, 0, 0), kTagSize,
                  {}, &noisy, Eigen::Vector3d::Zero(), 0.0, &rng, &scratch);
  EXPECT_EQ(exact, noisy);
}

TEST(RenderDetect, NullRngDisablesSensorRandomness) {
  auto camera = RoundTripCamera();
  camera.sensor_noise_std_gray = 20.0;
  camera.exposure_gain_std = 0.5;
  const FieldLayout empty(kTagSize);
  SimRenderOptions options;
  options.background_gray = 128;
  RenderScratch scratch;
  std::vector<uint8_t> pixels;
  RenderFieldView(empty, {}, camera, RobotPoseOnField(0, 0, 0), kTagSize,
                  options, &pixels, Eigen::Vector3d::Zero(), 0.0, nullptr,
                  &scratch);
  EXPECT_TRUE(std::all_of(pixels.begin(), pixels.end(),
                          [](uint8_t value) { return value == 128; }));
}

TEST(RenderDetect, SubpixelMotionUsesSingleRenderAndReusesProjectionStorage) {
  auto camera = RoundTripCamera();
  camera.exposure_ms = 10.0;
  const auto layout = RoundTripLayout();
  const auto pose = RobotPoseOnField(0, 0, 0);
  std::map<int, TagPattern> patterns;
  patterns.emplace(kTagId, TagPattern{kTagId, {}});
  RenderScratch scratch;
  std::vector<uint8_t> moving, stationary;
  ASSERT_EQ(
      RenderFieldView(layout, patterns, camera, pose, kTagSize, {}, &moving,
                      Eigen::Vector3d(0, 0.01, 0), 0.0, nullptr, &scratch),
      1);
  EXPECT_TRUE(scratch.instant.empty());
  EXPECT_TRUE(scratch.accumulated.empty());
  const auto* detections = scratch.detections.data();
  const auto* start = scratch.start.data();
  const auto* end = scratch.end.data();
  RenderFieldView(layout, patterns, camera, pose, kTagSize, {}, &moving,
                  Eigen::Vector3d(0, 0.01, 0), 0.0, nullptr, &scratch);
  EXPECT_EQ(detections, scratch.detections.data());
  EXPECT_EQ(start, scratch.start.data());
  EXPECT_EQ(end, scratch.end.data());
  RenderFieldView(layout, patterns, camera, pose, kTagSize, {}, &stationary,
                  Eigen::Vector3d::Zero(), 0.0, nullptr, &scratch);
  EXPECT_EQ(moving, stationary);
}

TEST(RenderDetect, TranslationBlurMatchesAnalyticalPixelBoxExposure) {
  auto camera = RoundTripCamera();
  camera.exposure_ms = 30.0;
  const auto layout = RoundTripLayout();
  const auto pose = RobotPoseOnField(0, 0, 0);
  // All-black payload makes an isolated square and a code-independent step.
  std::map<int, TagPattern> patterns;
  patterns.emplace(kTagId, TagPattern{kTagId, {}});
  RenderScratch scratch;
  auto edge_width = [&](double speed) {
    std::vector<uint8_t> pixels;
    RenderFieldView(layout, patterns, camera, pose, kTagSize, {}, &pixels,
                    Eigen::Vector3d(0, speed, 0), 0.0, nullptr, &scratch);
    auto crossing = [&](double level) {
      for (int x = 240; x < 320; ++x) {
        const double left = pixels[240 * 640 + x];
        const double right = pixels[240 * 640 + x + 1];
        if (left >= level && right < level) {
          return x + (left - level) / (left - right);
        }
      }
      return -1.0;
    };
    const double high = crossing(0.9 * 255);
    const double low = crossing(0.1 * 255);
    EXPECT_GT(high, 240);
    EXPECT_GT(low, high);
    return low - high;
  };
  // Box exposure has 10–90% step width .8*v*T*fx/Z. Pixel rounding
  // and finite temporal samples contribute at most approximately one pixel.
  const double slow = edge_width(1);
  const double fast = edge_width(2);
  EXPECT_NEAR(slow, 0.8 * 1 * .03 * 500 / 1.5, 1.1);
  EXPECT_NEAR(fast, 0.8 * 2 * .03 * 500 / 1.5, 1.1);
  EXPECT_GT(fast, slow);
}

TEST(RenderDetect, YawBlurIncludesCameraMountLeverArm) {
  auto camera = RoundTripCamera();
  camera.mounting = MountFromXyzYpr(0.5, 0, 0.5, 0, 0, 0);
  camera.exposure_ms = 20.0;
  const auto layout = RoundTripLayout();
  std::map<int, TagPattern> patterns;
  std::string error;
  ASSERT_TRUE(LoadTag36h11Patterns(&patterns, &error)) << error;
  std::vector<uint8_t> blurred, instantaneous;
  RenderScratch scratch;
  RenderFieldView(layout, patterns, camera, RobotPoseOnField(0, 0, 0), kTagSize,
                  {}, &blurred, Eigen::Vector3d::Zero(), 1.0, nullptr,
                  &scratch);
  // Independent pose integration rotates the robot about its origin.
  // The camera center thus moves laterally by .5*sin(yaw), in addition
  // to the optical-axis rotation; an in-place camera blur misses this.
  constexpr int samples = 200;
  std::vector<double> reference(blurred.size(), 0.0);
  camera.exposure_ms = 0.0;
  for (int i = 0; i < samples; ++i) {
    const double yaw = ((i + 0.5) / samples - 0.5) * 0.02;
    RenderFieldView(layout, patterns, camera, RobotPoseOnField(0, 0, yaw),
                    kTagSize, {}, &instantaneous, Eigen::Vector3d::Zero(), 0.0,
                    nullptr, &scratch);
    for (size_t p = 0; p < reference.size(); ++p) {
      reference[p] += instantaneous[p] / static_cast<double>(samples);
    }
  }
  double error_sum = 0.0;
  for (int y = 180; y < 300; ++y) {
    for (int x = 250; x < 390; ++x) {
      const size_t p = y * 640 + x;
      error_sum += std::abs(blurred[p] - reference[p]);
    }
  }
  EXPECT_LT(error_sum / (120 * 140), 1.5);
}

TEST(RenderDetect, ModerateSensorNoiseAndMotionStillDecode) {
  auto camera = RoundTripCamera();
  camera.sensor_noise_std_gray = 2.0;
  camera.exposure_gain_std = 0.02;
  camera.exposure_ms = 5.0;
  std::mt19937_64 rng(32);
  RenderScratch scratch;
  const auto layout = RoundTripLayout();
  std::map<int, TagPattern> patterns;
  std::string error;
  ASSERT_TRUE(LoadTag36h11Patterns(&patterns, &error)) << error;
  std::vector<uint8_t> pixels;
  ASSERT_EQ(RenderFieldView(layout, patterns, camera, RobotPoseOnField(0, 0, 0),
                            kTagSize, {}, &pixels, Eigen::Vector3d(0, 0.1, 0),
                            0.01, &rng, &scratch),
            1);
  PipelineConfig config;
  config.kind = "apriltag";
  config.backend_name = "cpu";
  config.revision = 1;
  config.intrinsics = camera.intrinsics;
  config.mounting = camera.mounting;
  config.layout = layout;
  config.parameters["decimate"] = "1.0";
  config.parameters["threads"] = "1";
  AprilTagPipeline pipeline(config);
  Frame frame(0, 1, 1, talos::event::MonotonicTime::from_nanos(1000), 640, 480,
              640, PixelFormat::kGray8, std::move(pixels));
  const auto result = pipeline.ProcessFrame(frame);
  ASSERT_TRUE(result.success) << result.error_message;
  ASSERT_EQ(result.detections.size(), 1u);
  EXPECT_EQ(result.detections.front().tag_id, kTagId);
}

}  // namespace
}  // namespace talos::vision::field_sim
