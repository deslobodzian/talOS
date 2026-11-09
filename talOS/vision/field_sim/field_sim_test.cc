// Tests for the field-sim projection library: JSON parser, WPILib layout
// loader (both schema generations plus the checked-in official 2026 file),
// and the pinhole tag projector.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include "talOS/vision/field_sim/field_layout_loader.h"
#include "talOS/vision/field_sim/json_parser.h"
#include "talOS/vision/field_sim/sim_camera.h"
#include "talOS/vision/localization.h"

namespace talos::vision::field_sim {
namespace {

constexpr double kTagSize = 0.2;
constexpr double kPi = 3.14159265358979323846;

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return "";
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// The official layout is supplied as a framework-owned Bazel runfile.
std::string OfficialLayoutText() {
  if (const char* path = std::getenv("FIELD_SIM_TESTDATA")) {
    return ReadFile(path);
  }
  return ReadFile("talOS/vision/testdata/2026-rebuilt-welded.json");
}

const char* kClassicJson = R"({
  "tags": [
    {"ID": 7, "pose": {"translation": {"x": 2.0, "y": 0.0, "z": 0.0},
     "rotation": {"quaternion": {"W": 0.7071068, "X": 0.0, "Y": -0.7071068, "Z": 0.0}}}}
  ],
  "field": {"length": 16.5, "width": 8.0}
})";

const char* kNewSchemaJson = R"({
  "name": "tiny",
  "field-dimensions": {"length": 10.0, "width": 5.0},
  "field-tags": [
    {"ID": 3, "pose": {"translation": {"x": 1.0, "y": 2.0, "z": 0.5},
     "rotation": {"quaternion": {"W": 1.0, "X": 0.0, "Y": 0.0, "Z": 0.0}}}}
  ]
})";

TEST(FieldLayoutLoader, RejectsInvalidAndDuplicateTagIds) {
  for (const auto* id : {"1.5", "2147483648", "-2147483649", "1e999"}) {
    std::string text = kClassicJson;
    const auto pos = text.find("\"ID\": 7");
    ASSERT_NE(pos, std::string::npos);
    text.replace(pos, 7, std::string("\"ID\": ") + id);
    FieldLayoutData data;
    std::string error;
    EXPECT_FALSE(LoadFieldLayoutFromJson(text, kTagSize, &data, &error)) << id;
  }
  std::string text = kClassicJson;
  const auto start = text.find("{\"ID\"");
  const auto end = text.find("\n  ]", start);
  ASSERT_NE(start, std::string::npos);
  ASSERT_NE(end, std::string::npos);
  const auto tag = text.substr(start, end - start);
  text.insert(end, "," + tag);
  FieldLayoutData data;
  std::string error;
  EXPECT_FALSE(LoadFieldLayoutFromJson(text, kTagSize, &data, &error));
}

TEST(JsonParser, ParsesNestedValuesAndEscapes) {
  JsonValue v;
  std::string err;
  ASSERT_TRUE(ParseJson(
      R"({"a": 1.5, "b": true, "c": null, "d": "x\ty", "e": [1, {"f": -2e3}]})",
      &v, &err))
      << err;
  EXPECT_DOUBLE_EQ(v.Find("a")->AsDouble(0.0), 1.5);
  EXPECT_TRUE(v.Find("b")->AsBool(false));
  EXPECT_EQ(v.Find("c")->type, JsonValue::Type::kNull);
  EXPECT_EQ(v.Find("d")->AsString(""), "x\ty");
  EXPECT_DOUBLE_EQ(v.Find("e")->arr[1].Find("f")->AsDouble(0.0), -2000.0);
  EXPECT_EQ(v.Find("missing"), nullptr);
}

TEST(JsonParser, RejectsMalformed) {
  const std::string deep(200, '[');
  const char* bad_cases[] = {
      "{",        "[1,]",          R"({"a": })", R"("unterminated)",
      "{'a': 1}", R"({"a": tru})", deep.c_str()};
  for (const char* bad : bad_cases) {
    JsonValue v;
    std::string err;
    EXPECT_FALSE(ParseJson(bad, &v, &err)) << "accepted: " << bad;
  }
}

TEST(FieldLayoutLoader, ParsesClassicSchema) {
  FieldLayoutData data;
  std::string err;
  ASSERT_TRUE(LoadFieldLayoutFromJson(kClassicJson, kTagSize, &data, &err))
      << err;
  EXPECT_DOUBLE_EQ(data.dims.length_m, 16.5);
  EXPECT_DOUBLE_EQ(data.dims.width_m, 8.0);
  const auto pose = data.layout.GetTagPose(7);
  if (!pose.has_value()) {
    FAIL() << "Missing tag 7 pose";
  }
  const auto& tag_pose = *pose;
  // Quaternion (W,X,Y,Z)=(0.7071,0,-0.7071,0) maps tag +Z to field -X.
  const Eigen::Vector3d normal = tag_pose.rotation().rotationMatrix().col(2);
  EXPECT_NEAR(normal.x(), -1.0, 1e-6);
  EXPECT_NEAR(tag_pose.translation().x(), 2.0, 1e-9);
  EXPECT_DOUBLE_EQ(data.layout.GetTagSize(7), kTagSize);
}

TEST(FieldLayoutLoader, ParsesNewSchema) {
  FieldLayoutData data;
  std::string err;
  ASSERT_TRUE(LoadFieldLayoutFromJson(kNewSchemaJson, kTagSize, &data, &err))
      << err;
  EXPECT_EQ(data.name, "tiny");
  EXPECT_DOUBLE_EQ(data.dims.length_m, 10.0);
  const auto pose = data.layout.GetTagPose(3);
  if (!pose.has_value()) {
    FAIL() << "Missing tag 3 pose";
  }
  const auto& tag_pose = *pose;
  EXPECT_NEAR(tag_pose.translation().y(), 2.0, 1e-9);
}

TEST(FieldLayoutLoader, RejectsBadDocuments) {
  const char* cases[] = {
      R"({"field": {"length": 1, "width": 1}})",  // no tags
      R"({"tags": [], "field": {"length": 1, "width": 1}})",
      R"({"tags": [{"ID": 1}], "field": {"length": 1, "width": 1}})",
      R"({"tags": [{"ID": 1, "pose": {"translation": {"x": 0, "y": 0, "z": 0},
          "rotation": {"quaternion": {"W": 0, "X": 0, "Y": 0, "Z": 0}}}}],
          "field": {"length": 1, "width": 1}})",  // zero-norm quat
      R"({"tags": [{"ID": 1, "pose": {"translation": {"x": 0, "y": 0, "z": 0},
          "rotation": {"quaternion": {"W": 1, "X": 0, "Y": 0, "Z": 0}}}}]})",
  };
  for (const char* text : cases) {
    FieldLayoutData data;
    std::string err;
    EXPECT_FALSE(LoadFieldLayoutFromJson(text, kTagSize, &data, &err)) << text;
  }
}

TEST(FieldLayoutLoader, ParsesOfficial2026Layout) {
  const std::string text = OfficialLayoutText();
  ASSERT_FALSE(text.empty()) << "official layout runfile not found";
  FieldLayoutData data;
  std::string err;
  ASSERT_TRUE(LoadFieldLayoutFromJson(text, 0.1651, &data, &err)) << err;
  EXPECT_DOUBLE_EQ(data.dims.length_m, 16.541);
  EXPECT_DOUBLE_EQ(data.dims.width_m, 8.069);
  EXPECT_EQ(data.layout.size(), 32u);
  for (int id = 1; id <= 32; ++id) EXPECT_TRUE(data.layout.HasTag(id));
  // Spot-check tag 1 against the published WPILib numbers.
  const auto pose = data.layout.GetTagPose(1);
  if (!pose.has_value()) {
    FAIL() << "Missing tag 1 pose";
  }
  const auto& tag_pose = *pose;
  EXPECT_NEAR(tag_pose.translation().x(), 11.8779798, 1e-6);
  EXPECT_NEAR(tag_pose.translation().y(), 7.4247756, 1e-6);
  EXPECT_NEAR(tag_pose.translation().z(), 0.889, 1e-9);
}

SimCameraView TestCamera() {
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

FieldLayout SingleTagLayout() {
  FieldLayout layout(kTagSize);
  // Upright tag 2 m ahead of the origin at lens height, face toward -X
  // (toward the robot): tag +X (face normal) maps to field -X, tag +Z (up)
  // maps to field +Z, tag +Y maps to field -Y.
  Eigen::Matrix3d upright;
  upright << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
  layout.SetTagPose(
      1, talos::geometry::SE3<double>(talos::geometry::SO3<double>(upright),
                                      Eigen::Vector3d(2.0, 0.0, 0.5)));
  return layout;
}

TEST(SimCamera, CenteredTagProjectsToImageCenter) {
  const auto dets =
      SimulateVisibleTags(SingleTagLayout(), TestCamera(),
                          RobotPoseOnField(0.0, 0.0, 0.0), kTagSize);
  ASSERT_EQ(dets.size(), 1u);
  EXPECT_EQ(dets[0].tag_id, 1);
  // 0.2 m tag at 2 m with fx=500: 50 px across, centered at (320, 240).
  EXPECT_NEAR(dets[0].center.x(), 320.0, 1e-6);
  EXPECT_NEAR(dets[0].center.y(), 240.0, 1e-6);
  // Model-corner order matches the detector (bottom-left first).
  EXPECT_NEAR(dets[0].corners[0].x(), 295.0, 1e-6);
  EXPECT_NEAR(dets[0].corners[0].y(), 265.0, 1e-6);
  EXPECT_NEAR(dets[0].corners[2].x(), 345.0, 1e-6);
  EXPECT_NEAR(dets[0].corners[2].y(), 215.0, 1e-6);
  EXPECT_LT(dets[0].ambiguity, 0.1);
}

TEST(SimCamera, CulledWhenBehindOutOfRangeOrEdgeOn) {
  FieldLayout layout = SingleTagLayout();
  // Behind the robot.
  EXPECT_TRUE(SimulateVisibleTags(layout, TestCamera(),
                                  RobotPoseOnField(4.0, 0.0, 0.0), kTagSize)
                  .empty());
  // Beyond max range.
  SimCameraView near = TestCamera();
  near.max_range_m = 1.0;
  EXPECT_TRUE(SimulateVisibleTags(layout, near, RobotPoseOnField(0.0, 0.0, 0.0),
                                  kTagSize)
                  .empty());
  // Tag turned edge-on: normal perpendicular to the view axis.
  FieldLayout edge(kTagSize);
  edge.SetTagPose(
      1, talos::geometry::SE3<double>(talos::geometry::SO3<double>::Identity(),
                                      Eigen::Vector3d(2.0, 0.0, 0.0)));
  EXPECT_TRUE(SimulateVisibleTags(edge, TestCamera(),
                                  RobotPoseOnField(0.0, 0.0, 0.0), kTagSize)
                  .empty());
}

TEST(SimCamera, MountYawTurnsTheView) {
  SimCameraView back = TestCamera();
  back.mounting = MountFromXyzYpr(0.0, 0.0, 0.0, 180.0, 0.0, 0.0);
  // Tag ahead is invisible to the rear camera...
  EXPECT_TRUE(SimulateVisibleTags(SingleTagLayout(), back,
                                  RobotPoseOnField(0.0, 0.0, 0.0), kTagSize)
                  .empty());
  // ...but visible once the robot itself turns around.
  const auto dets = SimulateVisibleTags(
      SingleTagLayout(), back, RobotPoseOnField(0.0, 0.0, kPi), kTagSize);
  ASSERT_EQ(dets.size(), 1u);
  EXPECT_NEAR(dets[0].center.x(), 320.0, 1e-6);
}

TEST(SimCamera, NoiseAdvancesAndReplaysWithZeroMeanAndConfiguredVariance) {
  SimCameraView noisy = TestCamera();
  noisy.pixel_noise_std = 1.5;
  noisy.rng_seed = 42;
  std::mt19937_64 rng(noisy.rng_seed), replay_rng(noisy.rng_seed);
  std::mt19937_64 different_rng(43);
  const auto layout = SingleTagLayout();
  const auto pose = RobotPoseOnField(0.0, 0.0, 0.0);
  double sum = 0, squares = 0, previous = 0;
  constexpr int kFrames = 4000;
  for (int i = 0; i < kFrames; ++i) {
    const auto observed =
        SimulateVisibleTags(layout, noisy, pose, kTagSize, false, &rng);
    const auto repeated =
        SimulateVisibleTags(layout, noisy, pose, kTagSize, false, &replay_rng);
    ASSERT_EQ(observed.size(), 1u);
    ASSERT_EQ(repeated.size(), 1u);
    const double x = observed[0].corners[0].x();
    EXPECT_DOUBLE_EQ(x, repeated[0].corners[0].x());
    if (i == 0) {
      const auto other = SimulateVisibleTags(layout, noisy, pose, kTagSize,
                                             false, &different_rng);
      ASSERT_EQ(other.size(), 1u);
      EXPECT_NE(x, other[0].corners[0].x());
    } else {
      EXPECT_NE(x, previous);
    }
    previous = x;
    // Analytic pinhole oracle: fx=500, tag half-width=.1, range=2.
    const double error = x - 295.0;
    sum += error;
    squares += error * error;
  }
  const double mean = sum / kFrames;
  EXPECT_NEAR(mean, 0.0, 0.08);
  EXPECT_NEAR(std::sqrt(squares / kFrames - mean * mean), 1.5, 0.08);
}

TEST(SimCamera, GeometryProjectionDoesNotConsumeOrApplyObservationNoise) {
  auto camera = TestCamera();
  camera.pixel_noise_std = 20;
  camera.rng_seed = 31;
  std::mt19937_64 rng(camera.rng_seed), replay_rng(camera.rng_seed);
  const auto layout = SingleTagLayout();
  const auto pose = RobotPoseOnField(0, 0, 0);
  const auto geometry =
      SimulateVisibleTags(layout, camera, pose, kTagSize, true, &rng);
  ASSERT_EQ(geometry.size(), 1u);
  EXPECT_DOUBLE_EQ(geometry[0].corners[0].x(), 295);
  const auto a =
      SimulateVisibleTags(layout, camera, pose, kTagSize, false, &rng);
  const auto b =
      SimulateVisibleTags(layout, camera, pose, kTagSize, false, &replay_rng);
  ASSERT_EQ(a.size(), 1u);
  ASSERT_EQ(b.size(), 1u);
  EXPECT_DOUBLE_EQ(a[0].corners[0].x(), b[0].corners[0].x());
  const auto noiseless = SimulateVisibleTags(layout, camera, pose, kTagSize);
  ASSERT_EQ(noiseless.size(), 1u);
  EXPECT_DOUBLE_EQ(noiseless[0].corners[0].x(), 295);
}


TEST(SimCamera, ReusesCallerStorageAndClearsOldDetections) {
  std::vector<SimTagDetection> detections;
  detections.reserve(32);
  const auto* storage = detections.data();
  const auto layout = SingleTagLayout();
  const auto camera = TestCamera();
  SimulateVisibleTags(layout, camera, RobotPoseOnField(0, 0, 0), kTagSize,
                      &detections);
  ASSERT_EQ(detections.size(), 1u);
  EXPECT_EQ(detections.data(), storage);
  SimulateVisibleTags(layout, camera, RobotPoseOnField(4, 0, 0), kTagSize,
                      &detections);
  EXPECT_TRUE(detections.empty());
  EXPECT_EQ(detections.data(), storage);
}

TEST(SimCamera, OpticalMountRoundTripRecoversFlatRobotPose) {
  const auto layout = SingleTagLayout();
  auto camera = TestCamera();
  camera.mounting =
      CameraMounting::FromExtrinsics(0.12, -0.08, 0.43, 0.03, -0.02, 0.04);
  const auto expected = RobotPoseOnField(0.2, -0.1, 0.08);
  const auto detections =
      SimulateVisibleTags(layout, camera, expected, kTagSize);
  ASSERT_EQ(detections.size(), 1u);
  const auto pose =
      ComputeRobotFieldPose(detections[0].tag_id, 0, detections[0].T_camera_tag,
                            camera.mounting, layout);
  ASSERT_TRUE(pose.validity.valid);
  EXPECT_LT((pose.T_field_robot.matrix() - expected.matrix()).norm(), 1e-12);
  EXPECT_NEAR(pose.T_field_robot.translation().z(), 0.0, 1e-12);
  EXPECT_LT((pose.T_field_robot.rotation().rotationMatrix().col(2) -
             Eigen::Vector3d::UnitZ())
                .norm(),
            1e-12);
}

}  // namespace
}  // namespace talos::vision::field_sim
