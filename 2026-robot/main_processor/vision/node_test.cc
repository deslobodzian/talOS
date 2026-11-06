#include <gtest/gtest.h>
#include <sys/wait.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/node/testing.h"
#include "vision/vision.h"

namespace vision {
namespace {
using namespace std::chrono_literals;
constexpr std::array<const char*, 4> kCameras{
    "front_left_camera", "front_right_camera", "rear_left_camera",
    "rear_right_camera"};

std::string Runfile(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> files(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* value = std::getenv(env);
  if (!value) {
    throw std::runtime_error(std::string("missing test environment: ") + env);
  }
  if (!files) throw std::runtime_error(error);
  return files->Rlocation(value);
}

talos::testing::Sim MakeSim(bool rasterize) {
  // Include the production manifest and override only the simulation settings.
  // The config loader expands its relative subsystem paths before merging.
  std::ostringstream text;
  text << "include = "
       << std::quoted(
              std::filesystem::absolute(Runfile("TALOS_TEST_CONFIG")).string())
       << '\n';
  for (const auto* name : kCameras) {
    text << "[subsystems." << name << "]\n"
         << "rasterize = " << (rasterize ? "true" : "false") << '\n'
         << "backend = \"cpu\"\n";
  }
  return talos::testing::Sim::FromToml(text.str());
}

void CheckPose(bool rasterize) {
  auto sim = MakeSim(rasterize);
  for (const auto* name : kCameras) sim.add<Vision>(name);
  // No driver station, targets, estimator or drivetrain: sensing runs disabled
  // at the configured field start pose, without requiring fresh actuation.
  sim.step(100ms);
  int valid_cameras = 0;
  bool rear_wall_observed = false;
  for (const auto* name : kCameras) {
    SCOPED_TRACE(name);
    auto observation = sim.latest<geometry::PoseObservation>(std::string("/") +
                                                             name + "/state");
    if (!observation.has_value()) {
      FAIL() << "missing camera observation";
    }
    auto status = sim.latest<VisionStatus>(std::string("/") + name + "/status");
    if (!status.has_value()) {
      FAIL() << "missing camera status";
    }
    const auto& camera_status = *status;
    const auto& observed = *observation;
    EXPECT_TRUE(camera_status.healthy());
    EXPECT_EQ(camera_status.last_latency_us(), 0u);
    EXPECT_EQ(camera_status.frame_period_us(), 50000u);
    if (!observed.valid()) continue;
    ++valid_cameras;
    EXPECT_NEAR(observed.pose().x_m(), 2.0, rasterize ? 0.15 : 0.05);
    EXPECT_NEAR(observed.pose().y_m(), 2.0, rasterize ? 0.15 : 0.05);
    EXPECT_NEAR(observed.pose().yaw_rad(), 0.0, rasterize ? 0.06 : 0.02);
    EXPECT_TRUE(std::isfinite(observed.x_std_m()));
    EXPECT_GT(observed.x_std_m(), 0);
    EXPECT_GT(observed.y_std_m(), 0);
    EXPECT_GT(observed.yaw_std_rad(), 0);
    // The range gate guarantees d <= 8 m; accepted ambiguity can reach 1.
    // sigma_xy = .08*d^2/sqrt(n)*(1+2*a), hence this conservative bound.
    const double bound = 0.08 * 8 * 8 * 3 / std::sqrt(observed.tag_count());
    EXPECT_LE(observed.x_std_m(), bound);
    ::testing::Test::RecordProperty(std::string(name) + "_tags",
                                    observed.tag_count());
    ::testing::Test::RecordProperty(std::string(name) + "_std_m",
                                    std::to_string(observed.x_std_m()));
    if (std::string_view(name) == "rear_right_camera") {
      rear_wall_observed = true;
      // This lens sees blue-wall tags 29/30, not the raised hub tags.
      // From camera (1.75,1.8,.45), their ranges are below 2.2 m.
      // Even ambiguity 1 gives sigma <= .08*2.2^2*3/sqrt(2) < .83 m.
      EXPECT_EQ(observed.tag_count(), 2u);
      EXPECT_LT(observed.x_std_m(), 0.08 * 2.2 * 2.2 * 3 / std::sqrt(2.0));
    }
  }
  EXPECT_GT(valid_cameras, 0);
  EXPECT_TRUE(rear_wall_observed);
}

TEST(VisionTest, FastProjectionLocalizesWhileDisabled) { CheckPose(false); }
TEST(VisionTest, RasterDetectorAndLocalizationRunWhileDisabled) {
  CheckPose(true);
}

drivetrain::DrivetrainState Drive(std::uint64_t stamp, double vx,
                                  double yaw = 0) {
  return {stamp, false, true, true, vx, 0, 0, yaw, {}, {}, {}, {}};
}

TEST(VisionTest, FacingAwayPublishesInvalidInsteadOfHoldingLastPose) {
  auto sim = MakeSim(false);
  sim.add<Vision>("rear_right_camera");
  sim.step(100ms);
  bool found_empty_view = false;
  for (int i = 0; i < 16; ++i) {
    // A stationary robot can rotate its camera away from the field's tags.
    sim.publish(drivetrain::topics::kState,
                Drive(100000000 + i * 50000000, 0, i * std::numbers::pi / 8));
    sim.step(50ms);
    const auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
    if (!status.has_value()) {
      FAIL() << "missing camera status";
    }
    if (status->visible_tag_count() != 0) continue;
    const auto observation =
        sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
    if (!observation.has_value()) {
      FAIL() << "missing camera observation";
    }
    const auto& observed = *observation;
    EXPECT_FALSE(observed.valid());
    found_empty_view = true;
    break;
  }
  EXPECT_TRUE(found_empty_view);
}

TEST(VisionTest, DrivetrainTruthMovesIndependentlyOfStaleEstimator) {
  auto sim = MakeSim(false);
  for (const auto* name : kCameras) sim.add<Vision>(name);
  sim.publish("/pose_estimator/state",
              geometry::PoseEstimate{0, geometry::Pose2d{10, 5, 1},
                                     geometry::Twist2d{}, true, 0, 0});
  sim.publish(drivetrain::topics::kState, Drive(1000000, 1));
  sim.step(10ms);
  for (int i = 1; i <= 5; ++i) {
    sim.publish(drivetrain::topics::kState, Drive(1000000 + i * 20000000, 1));
    sim.step(20ms);
  }
  int valid = 0;
  for (const auto* name : kCameras) {
    const auto observation = sim.latest<geometry::PoseObservation>(
        std::string("/") + name + "/state");
    if (!observation.has_value()) {
      FAIL() << "missing camera observation";
    }
    const auto& observed = *observation;
    if (!observed.valid()) continue;
    ++valid;
    EXPECT_NEAR(observed.pose().x_m(), 2.1, 0.06);
    EXPECT_NEAR(observed.pose().y_m(), 2, 0.06);
  }
  EXPECT_GT(valid, 0);
}

TEST(VisionTest, SimTruthRejectsDuplicateInvalidAndDiscontinuousSamples) {
  SimTruth truth{2, 2, std::numbers::pi / 2};
  truth.Observe(Drive(1000000, 1));
  truth.Observe(Drive(51000000, 1));
  truth.Observe(Drive(51000000, 20));
  EXPECT_NEAR(truth.pose().translation().x(), 2, 1e-9);
  EXPECT_NEAR(truth.pose().translation().y(), 2.05, 1e-9);
  truth.Observe(Drive(1000000000, 1));
  EXPECT_NEAR(truth.pose().translation().y(), 2.05, 1e-9);
}

TEST(VisionTest, RejectsOutOfBoundsHighErrorAndNonfiniteEstimates) {
  auto sim = MakeSim(false);
  auto& node = sim.add<Vision>("front_left_camera");
  talos::vision::RobotPoseEstimate estimate;
  estimate.validity.valid = true;
  estimate.T_field_robot = talos::vision::field_sim::RobotPoseOnField(2, 2, 0);
  EXPECT_TRUE(node.AcceptEstimate(estimate));
  estimate.reprojection_error_px = 20;
  EXPECT_FALSE(node.AcceptEstimate(estimate));
  estimate.reprojection_error_px = 0;
  estimate.T_field_robot = talos::vision::field_sim::RobotPoseOnField(-1, 2, 0);
  EXPECT_FALSE(node.AcceptEstimate(estimate));
  estimate.reprojection_error_px = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(node.AcceptEstimate(estimate));
}

TEST(VisionTest, InvalidDetectionsDoNotCountAsVisibleOrRangeRejected) {
  std::vector<talos::vision::TagDetection3D> detections(4);
  detections[0].valid = false;
  detections[1].valid = true;
  detections[1].tag_id = 17;
  detections[1].T_camera_tag = talos::geometry::SE3<double>{
      talos::geometry::SO3<double>::Identity(), Eigen::Vector3d{0, 0, 2}};
  detections[2].valid = false;
  detections[2].T_camera_tag = talos::geometry::SE3<double>{
      talos::geometry::SO3<double>::Identity(), Eigen::Vector3d{0, 0, 20}};
  detections[3].valid = true;
  detections[3].T_camera_tag = detections[2].T_camera_tag;
  const auto capacity = detections.capacity();
  EXPECT_EQ(Vision::FilterDetections(detections, 8), 1u);
  ASSERT_EQ(detections.size(), 1u);
  EXPECT_EQ(detections.front().tag_id, 17);
  EXPECT_EQ(detections.capacity(), capacity);
}

TEST(VisionBinary, DescribeDoesNotOpenLiveCameraAndSimulationReplays) {
  auto run = [](const std::string& arguments) {
    const std::string command = "'" + Runfile("TALOS_TEST_NODE") + "' " +
                                arguments + " --config '" +
                                Runfile("TALOS_TEST_CONFIG") + "' 2>&1";
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) throw std::runtime_error("popen failed");
    std::string output;
    char buffer[4096];
    while (const auto n = std::fread(buffer, 1, sizeof buffer, pipe)) {
      output.append(buffer, n);
    }
    const int status = pclose(pipe);
    return std::pair{WIFEXITED(status) ? WEXITSTATUS(status) : -1, output};
  };
  const auto [code, description] = run("--describe");
  ASSERT_EQ(code, 0) << description;
  EXPECT_NE(description.find("/front_left_camera/status"), std::string::npos);
  EXPECT_EQ(description.find("AVCapture"), std::string::npos);
  EXPECT_EQ(description.find("/pose_estimator/state"), std::string::npos);
  const char* temporary_directory = std::getenv("TEST_TMPDIR");
  ASSERT_NE(temporary_directory, nullptr)
      << "TEST_TMPDIR must identify Bazel's test directory";
  const auto log = std::string(temporary_directory) + "/camera.tlog";
  const auto [sim_code, sim_output] =
      run("--sim --duration-s 1 --log '" + log + "'");
  ASSERT_EQ(sim_code, 0) << sim_output;
  const auto [replay_code, replay_output] = run("--sim --replay '" + log + "'");
  EXPECT_EQ(replay_code, 0) << replay_output;
  EXPECT_NE(replay_output.find("diverged=0 clean_exit=1"), std::string::npos)
      << replay_output;
}
}  // namespace
}  // namespace vision
