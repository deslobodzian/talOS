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
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <numbers>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/node/testing.h"
#include "talOS/vision/recording/recording.h"
#include "talOS/vision/recording/replay.h"
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

talos::testing::Sim MakeSim(bool rasterize, bool noiseless_projection = false,
                            std::string_view camera_overrides = {},
                            std::string_view node_overrides = {}) {
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
         << "backend = \"cpu\"\n"
         << node_overrides;
    if (noiseless_projection || !camera_overrides.empty()) {
      text << "[subsystems." << name << ".camera]\n";
      if (noiseless_projection) text << "pixel_noise_std = 0.0\n";
      text << camera_overrides;
    }
  }
  return talos::testing::Sim::FromToml(text.str());
}

geometry::OdometrySample Truth(const talos::testing::Sim& sim,
                               geometry::Pose2d pose = geometry::Pose2d{2, 2,
                                                                        0}) {
  return {static_cast<std::uint64_t>(sim.now().nanos()), pose,
          geometry::Twist2d{}};
}

struct CameraObserver {
  struct Frame {
    geometry::PoseObservation observation;
    std::uint64_t delivery_ns;
  };

  explicit CameraObserver(talos::Node& n)
      : node(n),
        state(n.subscribe<geometry::PoseObservation>(
            "/rear_right_camera/state",
            [this](const geometry::PoseObservation& observation) {
              frames.push_back({observation, static_cast<std::uint64_t>(
                                                 node.now().nanos())});
            })) {}

  talos::Node& node;
  std::vector<Frame> frames;
  talos::Subscription<geometry::PoseObservation> state;
};

void CheckPose(bool rasterize) {
  // Isolate direct-projection geometry from corner noise; the raster path
  // retains production sensor/exposure noise.
  auto sim = MakeSim(rasterize, !rasterize);
  for (const auto* name : kCameras) sim.add<Vision>(name);
  // No driver station, targets, estimator or drivetrain: sensing runs disabled
  // from gateway truth, without requiring fresh actuation.
  sim.step(1ms);
  sim.publish(sim::topics::kState, Truth(sim));
  sim.step(50ms);
  const auto truth = Truth(sim);
  sim.publish(sim::topics::kState, truth);
  sim.step(50ms);
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
    EXPECT_EQ(observed.sample_time_ns(), truth.sample_time_ns());
    EXPECT_EQ(camera_status.last_latency_us(), 9000u);
    EXPECT_EQ(camera_status.frame_period_us(), 50000u);
    if (!observed.valid()) continue;
    ++valid_cameras;
    EXPECT_NEAR(observed.pose().x_m(), 2.0, rasterize ? 0.15 : 0.05);
    EXPECT_NEAR(observed.pose().y_m(), 2.0, rasterize ? 0.15 : 0.05);
    EXPECT_NEAR(observed.pose().yaw_rad(), 0.0, rasterize ? 0.06 : 0.02);
    EXPECT_TRUE(std::isfinite(observed.x_std_m()));
    EXPECT_TRUE(std::isfinite(observed.y_std_m()));
    EXPECT_TRUE(std::isfinite(observed.yaw_std_rad()));
    EXPECT_GE(observed.x_std_m(), 0.02);
    EXPECT_GE(observed.y_std_m(), 0.02);
    EXPECT_GE(observed.yaw_std_rad(),
              talos::vision::PoseSolverOptions{}.min_yaw_std_rad);
    ::testing::Test::RecordProperty(std::string(name) + "_tags",
                                    static_cast<int>(observed.tag_count()));
    ::testing::Test::RecordProperty(std::string(name) + "_std_m",
                                    std::to_string(observed.x_std_m()));
    if (std::string_view(name) == "rear_right_camera") {
      rear_wall_observed = true;
      // This lens sees blue-wall tags 29/30, not the raised hub tags.
      EXPECT_EQ(observed.tag_count(), 2u);
    }
  }
  EXPECT_GT(valid_cameras, 0);
  EXPECT_TRUE(rear_wall_observed);
}

TEST(VisionTest, FastProjectionLocalizesWhileDisabled) { CheckPose(false); }
TEST(VisionTest, RasterDetectorAndLocalizationRunWhileDisabled) {
  CheckPose(true);
}

TEST(VisionTest, CaptureTimestampCorrectionIsSharedWithLiveFrames) {
  EXPECT_EQ(Vision::CorrectCaptureTimestamp(1'000'000, 250'000), 750'000u);
  EXPECT_EQ(Vision::CorrectCaptureTimestamp(1'000'000, 0, 40'000'000),
            41'000'000u);
  EXPECT_EQ(Vision::CorrectCaptureTimestamp(1'000'000, 40'000'000, 40'000'000),
            1'000'000u);
  EXPECT_EQ(Vision::CorrectCaptureTimestamp(1'000, 1'000), 0u);
  EXPECT_FALSE(Vision::CorrectCaptureTimestamp(1'000, 1'001));
  EXPECT_FALSE(Vision::CorrectCaptureTimestamp(
      std::numeric_limits<std::uint64_t>::max(), 0, 1));
}

TEST(VisionTest, SimLatencyOffsetAndLagCorrectStampAndReportedLatency) {
  auto sample = [](std::string_view overrides) {
    auto sim = MakeSim(false, true, overrides);
    sim.add<Vision>("rear_right_camera");
    sim.step(101ms);
    const auto truth = Truth(sim);
    sim.publish(sim::topics::kState, truth);
    sim.step(50ms);
    const auto observation =
        sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
    const auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
    EXPECT_TRUE(observation.has_value());
    EXPECT_TRUE(status.has_value());
    if (observation && status) {
      EXPECT_TRUE(observation->valid());
      return std::pair{*observation, status->last_latency_us()};
    }
    return std::pair{geometry::PoseObservation{}, std::uint64_t{0}};
  };
  const auto [baseline, baseline_latency] = sample("");
  const auto [offset, offset_latency] = sample("latency_offset_us = 2000\n");
  const auto [lagged, lagged_latency] =
      sample("sim_timestamp_lag_us = 40000\n");
  const auto [corrected, corrected_latency] =
      sample("sim_timestamp_lag_us = 40000\nlatency_offset_us = 40000\n");
  EXPECT_EQ(offset.sample_time_ns(), baseline.sample_time_ns() - 2'000'000);
  EXPECT_EQ(offset_latency, baseline_latency + 2000);
  EXPECT_EQ(lagged.sample_time_ns(), baseline.sample_time_ns() + 40'000'000);
  EXPECT_EQ(corrected.sample_time_ns(), baseline.sample_time_ns());
  EXPECT_EQ(corrected_latency, lagged_latency + 40000);
  EXPECT_DOUBLE_EQ(corrected.pose().x_m(), baseline.pose().x_m());
  EXPECT_DOUBLE_EQ(corrected.pose().y_m(), baseline.pose().y_m());
  EXPECT_DOUBLE_EQ(corrected.pose().yaw_rad(), baseline.pose().yaw_rad());
}

TEST(VisionTest, SimLagDelaysDeliveryRatherThanPublishingFutureObservations) {
  auto sim = MakeSim(false, true, "sim_timestamp_lag_us = 40000\n");
  sim.add<Vision>("rear_right_camera");
  const auto truth = Truth(sim);
  sim.publish(sim::topics::kState, truth);
  sim.step(30ms);
  EXPECT_FALSE(
      sim.latest<geometry::PoseObservation>("/rear_right_camera/state"));
  sim.step(20ms);
  const auto observation =
      sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
  if (!observation.has_value()) {
    FAIL() << "missing camera observation";
  }
  EXPECT_TRUE(observation->valid());
  EXPECT_EQ(observation->sample_time_ns(), truth.sample_time_ns() + 40'000'000);
  EXPECT_LE(observation->sample_time_ns(),
            static_cast<std::uint64_t>(sim.now().nanos()));
  sim.step(
      100ms);  // Stale truth still invalidates a previously delivered pose.
  const auto stale =
      sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
  if (!stale.has_value()) {
    FAIL() << "missing camera observation";
  }
  EXPECT_FALSE(stale->valid());
}

TEST(VisionTest, SteadySixtyFpsFortyMillisecondLagDeliversEveryCapturedFrame) {
  auto sim = MakeSim(false, true,
                     "fps = 60\nsim_timestamp_lag_us = 40000\n"
                     "latency_offset_us = 40000\n",
                     "period_us = 1000\n");
  sim.add<Vision>("rear_right_camera");
  auto& observer = sim.add<CameraObserver>("pose_estimator");
  for (int ms = 0; ms <= 1000; ++ms) {
    sim.publish(sim::topics::kState, Truth(sim, {2 + ms * 0.0001, 2, 0}));
    sim.step(1ms);
  }
  auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
  if (!status.has_value()) {
    FAIL() << "missing camera status";
  }
  // The 1 ms truth grid alternates 16/17 ms intervals without phase drift.
  constexpr std::size_t kCaptured = 61;
  EXPECT_EQ(status->frames_captured_count(), kCaptured);
  EXPECT_EQ(status->frames_processed_count(), kCaptured);
  EXPECT_EQ(status->frames_dropped_count(), 0u);
  EXPECT_EQ(status->frame_period_us(), 16666u);
  EXPECT_EQ(status->last_latency_us(), 40000u);
  // The final three captures remain in flight at 1001 ms.
  EXPECT_EQ(observer.frames.size(), kCaptured - 3);

  // Keep the same still-fresh truth: drain, without fabricating new frames.
  sim.step(40ms);
  status = sim.latest<VisionStatus>("/rear_right_camera/status");
  if (!status.has_value()) {
    FAIL() << "missing camera status";
  }
  EXPECT_EQ(status->frames_captured_count(), kCaptured);
  EXPECT_EQ(status->frames_processed_count(), kCaptured);
  EXPECT_EQ(status->frames_dropped_count(), 0u);
  EXPECT_EQ(status->frame_period_us(), 16666u);
  EXPECT_EQ(status->last_latency_us(), 40000u);
  ASSERT_EQ(observer.frames.size(), kCaptured);
  for (std::size_t i = 0; i < observer.frames.size(); ++i) {
    SCOPED_TRACE(i);
    const auto& frame = observer.frames[i];
    const auto capture_ms = (i * 1000 + 59) / 60;
    const auto capture_ns = capture_ms * 1'000'000u;
    EXPECT_TRUE(frame.observation.valid());
    EXPECT_EQ(frame.observation.sample_time_ns(), capture_ns);
    EXPECT_GE(frame.delivery_ns, capture_ns + 40'000'000u);
    EXPECT_LE(frame.delivery_ns, capture_ns + 41'000'000u);
    EXPECT_NEAR(frame.observation.pose().x_m(),
                2 + static_cast<double>(capture_ms) * 0.0001, 0.05);
    EXPECT_NEAR(frame.observation.pose().y_m(), 2, 0.05);
  }
}

TEST(VisionTest, CatchUpTickPublishesEveryDueObservationInCaptureOrder) {
  auto sim = MakeSim(false, true,
                     "fps = 60\nsim_timestamp_lag_us = 100000\n"
                     "latency_offset_us = 100000\n",
                     "period_us = 30000\n");
  sim.add<Vision>("rear_right_camera");
  auto& observer = sim.add<CameraObserver>("pose_estimator");
  // Truth advances more slowly than node time. Two already-queued frames
  // become due together at 120 ms; both must survive the catch-up tick.
  for (int i = 0; i < 4; ++i) {
    sim.publish(sim::topics::kState,
                geometry::OdometrySample{
                    static_cast<std::uint64_t>(i) * 17'000'000u,
                    geometry::Pose2d{2 + i * 0.01, 2, 0}, geometry::Twist2d{}});
    sim.step(30ms);
    if (i < 3) EXPECT_TRUE(observer.frames.empty());
  }
  sim.step(1ms);  // Deliver both publications to the callback observer.
  ASSERT_EQ(observer.frames.size(), 2u);
  EXPECT_EQ(observer.frames[0].observation.sample_time_ns(), 0u);
  EXPECT_EQ(observer.frames[1].observation.sample_time_ns(), 17'000'000u);
  EXPECT_EQ(observer.frames[0].delivery_ns, observer.frames[1].delivery_ns);
  auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
  if (!status.has_value()) {
    FAIL() << "missing camera status";
  }
  EXPECT_EQ(status->frames_captured_count(), 4u);
  EXPECT_EQ(status->frames_processed_count(), 4u);
  EXPECT_EQ(status->frames_dropped_count(), 0u);
  EXPECT_EQ(status->frame_period_us(), 17000u);
  EXPECT_EQ(status->last_latency_us(), 103000u);

  sim.step(60ms);
  ASSERT_EQ(observer.frames.size(), 4u);
  for (std::size_t i = 0; i < observer.frames.size(); ++i) {
    EXPECT_TRUE(observer.frames[i].observation.valid());
    EXPECT_EQ(observer.frames[i].observation.sample_time_ns(), i * 17'000'000u);
  }
  status = sim.latest<VisionStatus>("/rear_right_camera/status");
  if (!status.has_value()) {
    FAIL() << "missing camera status";
  }
  EXPECT_EQ(status->frames_captured_count(), 4u);
  EXPECT_EQ(status->frames_dropped_count(), 0u);
}

TEST(VisionTest, FullDelayedQueueDropsAndCountsOnlyItsOldestCapture) {
  auto sim = MakeSim(false, true,
                     "fps = 60\nsim_timestamp_lag_us = 40000\n"
                     "latency_offset_us = 40000\n",
                     "period_us = 1000\n");
  sim.add<Vision>("rear_right_camera");
  auto& observer = sim.add<CameraObserver>("pose_estimator");
  // ceil(40 ms * 60 fps) + 2 = 5 slots. Deliberately future-dated truth
  // fills them before any delivery, forcing one genuine oldest-frame loss.
  for (int ms = 0; ms < 240; ++ms) {
    const int capture_index = std::min(ms, 5);
    sim.publish(sim::topics::kState,
                geometry::OdometrySample{
                    (100u + static_cast<std::uint64_t>(capture_index) * 17u) *
                        1'000'000u,
                    geometry::Pose2d{2 + capture_index * 0.01, 2, 0},
                    geometry::Twist2d{}});
    sim.step(1ms);
    if (ms == 5) {
      EXPECT_TRUE(observer.frames.empty());
      const auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
      if (!status.has_value()) {
        FAIL() << "missing camera status";
      }
      EXPECT_EQ(status->frames_captured_count(), 6u);
      EXPECT_EQ(status->frames_dropped_count(), 1u);
    }
  }
  const auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
  if (!status.has_value()) {
    FAIL() << "missing camera status";
  }
  EXPECT_EQ(status->frames_captured_count(), 6u);
  EXPECT_EQ(status->frames_processed_count(), 6u);
  EXPECT_EQ(status->frames_dropped_count(), 1u);
  EXPECT_EQ(status->frame_period_us(), 17000u);
  EXPECT_EQ(status->last_latency_us(), 40000u);
  ASSERT_EQ(observer.frames.size(), 5u);
  for (std::size_t i = 0; i < observer.frames.size(); ++i) {
    EXPECT_TRUE(observer.frames[i].observation.valid());
    EXPECT_EQ(observer.frames[i].observation.sample_time_ns(),
              (117u + i * 17u) * 1'000'000u);
  }
}

TEST(VisionTest, LatencyOffsetUnderflowPublishesInvalid) {
  auto sim = MakeSim(false, true, "latency_offset_us = 40000\n");
  sim.add<Vision>("rear_right_camera");
  sim.publish(sim::topics::kState, Truth(sim));
  sim.step(10ms);
  const auto observation =
      sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
  if (!observation.has_value()) {
    FAIL() << "missing camera observation";
  }
  EXPECT_FALSE(observation->valid());
  EXPECT_EQ(observation->sample_time_ns(), 0u);
}

TEST(VisionTest, SimMountErrorZeroIsUnchangedAndYawErrorBiasesObservations) {
  auto sample = [](std::string_view overrides) {
    auto sim = MakeSim(false, true, overrides);
    for (const auto* name : kCameras) sim.add<Vision>(name);
    sim.publish(sim::topics::kState, Truth(sim, {4, 3, 0}));
    sim.step(50ms);
    std::array<geometry::PoseObservation, kCameras.size()> result{};
    for (size_t i = 0; i < kCameras.size(); ++i) {
      const auto observation = sim.latest<geometry::PoseObservation>(
          std::string("/") + kCameras[i] + "/state");
      EXPECT_TRUE(observation.has_value());
      if (observation) result[i] = *observation;
    }
    return result;
  };
  const auto implicit_zero = sample("pitch_rad = 0.0\n");
  const auto explicit_zero = sample(
      "pitch_rad = 0.0\nsim_mount_error_x_m = 0.0\n"
      "sim_mount_error_y_m = 0.0\nsim_mount_error_z_m = 0.0\n"
      "sim_mount_error_roll_rad = 0.0\nsim_mount_error_pitch_rad = 0.0\n"
      "sim_mount_error_yaw_rad = 0.0\n");
  const auto yaw_error =
      sample("pitch_rad = 0.0\nsim_mount_error_yaw_rad = 0.02\n");
  int compared = 0;
  for (size_t i = 0; i < kCameras.size(); ++i) {
    SCOPED_TRACE(kCameras[i]);
    EXPECT_EQ(implicit_zero[i].valid(), explicit_zero[i].valid());
    if (implicit_zero[i].valid()) {
      EXPECT_DOUBLE_EQ(implicit_zero[i].pose().x_m(),
                       explicit_zero[i].pose().x_m());
      EXPECT_DOUBLE_EQ(implicit_zero[i].pose().y_m(),
                       explicit_zero[i].pose().y_m());
      EXPECT_DOUBLE_EQ(implicit_zero[i].pose().yaw_rad(),
                       explicit_zero[i].pose().yaw_rad());
      if (yaw_error[i].valid()) {
        ++compared;
        EXPECT_NEAR(
            yaw_error[i].pose().yaw_rad() - implicit_zero[i].pose().yaw_rad(),
            0.02, 1e-4);
      }
    }
  }
  EXPECT_GT(compared, 0);
}

TEST(VisionTest, SingleAndMultiTagObservationsPublishConfiguredSolverStdDevs) {
  namespace fs = talos::vision::field_sim;
  // Level the fixture so its single tag clears the production ambiguity gate.
  auto fixture = MakeSim(false, true, "pitch_rad = 0.0\n");
  fixture.add<Vision>("rear_right_camera");
  auto& node = fixture.node("rear_right_camera");
  auto number = [&](const char* key) {
    return node.param<double>(std::string("camera.") + key, 0.0);
  };
  fs::SimCameraView camera;
  camera.mounting = talos::vision::CameraMounting::FromExtrinsics(
      number("x_m"), number("y_m"), number("z_m"), number("roll_rad"),
      number("pitch_rad"), number("yaw_rad"));
  camera.intrinsics.fx = number("fx");
  camera.intrinsics.fy = number("fy");
  camera.intrinsics.cx = number("cx");
  camera.intrinsics.cy = number("cy");
  camera.intrinsics.image_width = node.param<int>("camera.width");
  camera.intrinsics.image_height = node.param<int>("camera.height");
  camera.max_range_m = number("max_range_m");
  camera.min_tag_px = number("min_tag_px");
  camera.facing_min = number("facing_min");
  camera.pixel_noise_std = 0;
  const double tag_size = node.param<double>("tag_size_m");
  fs::FieldLayoutData layout;
  std::string error;
  ASSERT_TRUE(
      fs::LoadFieldLayoutFromJson(kFieldLayoutJson, tag_size, &layout, &error))
      << error;
  std::vector<fs::SimTagDetection> tags;
  std::mt19937_64 rng(0);
  fs::SimulateVisibleTags(layout.layout, camera, fs::RobotPoseOnField(2, 2, 0),
                          tag_size, &tags, false, &rng);
  ASSERT_GE(tags.size(), 2u);
  std::vector<double> ranges;
  ranges.reserve(tags.size());
  for (const auto& tag : tags) {
    ranges.push_back(tag.T_camera_tag.translation().norm());
  }
  std::sort(ranges.begin(), ranges.end());
  const double nearest = ranges[0];
  const double next = ranges[1];
  ASSERT_LT(nearest, next);
  const talos::vision::PoseSolverOptions options{.pixel_std_px = 1.7,
                                                 .floor_z_std_m = 0.08,
                                                 .floor_tilt_std_rad = 0.03,
                                                 .huber_delta_px = 1.4,
                                                 .min_xy_std_m = 0.04,
                                                 .min_yaw_std_rad = 0.15};
  for (bool single_tag : {true, false}) {
    SCOPED_TRACE(single_tag);
    const double range = single_tag ? (nearest + next) / 2 : camera.max_range_m;
    std::vector<talos::vision::TagDetection3D> detections;
    for (const auto& tag : tags) {
      if (tag.T_camera_tag.translation().norm() > range) continue;
      talos::vision::TagDetection3D detection;
      detection.valid = true;
      detection.tag_id = tag.tag_id;
      detection.T_camera_tag = tag.T_camera_tag;
      detection.ambiguity = tag.ambiguity;
      for (std::size_t c = 0; c < 4; ++c) {
        detection.corners[2 * c] = static_cast<float>(tag.corners[c].x());
        detection.corners[2 * c + 1] = static_cast<float>(tag.corners[c].y());
      }
      detections.push_back(detection);
    }
    const auto expected = talos::vision::SolveMultiTagRobotPose(
        detections, node.param<int>("camera.id"), camera.mounting,
        layout.layout, camera.intrinsics, {}, options);
    ASSERT_TRUE(expected.validity.valid) << expected.validity.message;
    std::ostringstream text;
    text << std::setprecision(17) << "include = "
         << std::quoted(std::filesystem::absolute(Runfile("TALOS_TEST_CONFIG"))
                            .string())
         << "\n[subsystems.rear_right_camera]\n"
            "rasterize = false\nbackend = \"cpu\"\n"
         << "pixel_std_px = " << options.pixel_std_px << '\n'
         << "floor_z_std_m = " << options.floor_z_std_m << '\n'
         << "floor_tilt_std_rad = " << options.floor_tilt_std_rad << '\n'
         << "huber_delta_px = " << options.huber_delta_px << '\n'
         << "min_xy_std_m = " << options.min_xy_std_m << '\n'
         << "min_yaw_std_rad = " << options.min_yaw_std_rad << '\n'
         << "[subsystems.rear_right_camera.camera]\n"
            "pixel_noise_std = 0.0\npitch_rad = 0.0\n"
         << "max_range_m = " << range << '\n';
    auto sim = talos::testing::Sim::FromToml(text.str());
    sim.add<Vision>("rear_right_camera");
    sim.publish(sim::topics::kState, Truth(sim));
    sim.step(50ms);
    const auto observation =
        sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
    if (!observation.has_value()) {
      FAIL() << "missing camera observation";
    }
    ASSERT_TRUE(observation->valid());
    EXPECT_EQ(observation->tag_count(), single_tag ? 1u : tags.size());
    EXPECT_NEAR(observation->x_std_m(), expected.std_devs.x_std_m, 1e-8);
    EXPECT_NEAR(observation->y_std_m(), expected.std_devs.y_std_m, 1e-8);
    EXPECT_NEAR(observation->yaw_std_rad(), expected.std_devs.yaw_std_rad,
                1e-8);
  }
}

TEST(VisionTest, MissingTruthPublishesInvalidAndOneDiagnosticPerCamera) {
  auto sim = MakeSim(false);
  for (const auto* name : kCameras) sim.add<Vision>(name);
  ::testing::internal::CaptureStderr();
  sim.step(250ms);
  const auto diagnostic = ::testing::internal::GetCapturedStderr();
  for (const auto* name : kCameras) {
    SCOPED_TRACE(name);
    const auto observation = sim.latest<geometry::PoseObservation>(
        std::string("/") + name + "/state");
    if (!observation.has_value()) {
      FAIL() << "missing camera observation";
    }
    EXPECT_FALSE(observation->valid());
    const auto status =
        sim.latest<VisionStatus>(std::string("/") + name + "/status");
    if (!status.has_value()) {
      FAIL() << "missing camera status";
    }
    EXPECT_FALSE(status->healthy());
    const auto message =
        std::string(name) +
        ": no fresh /sim/state; simulated cameras need "
        "sim_gateway --physics (WPILib sim publishes no truth)\n";
    const auto first = diagnostic.find(message);
    ASSERT_NE(first, std::string::npos) << diagnostic;
    EXPECT_EQ(diagnostic.find(message, first + message.size()),
              std::string::npos);
  }
}

TEST(VisionTest, StaleTruthIsInvalidAndRecoveryAllowsANewOutageDiagnostic) {
  auto sim = MakeSim(false);
  sim.add<Vision>("rear_right_camera");
  sim.publish(sim::topics::kState, Truth(sim));
  sim.step(50ms);
  auto observation =
      sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
  if (!observation.has_value()) {
    FAIL() << "missing camera observation";
  }
  ASSERT_TRUE(observation->valid());
  ::testing::internal::CaptureStderr();
  sim.step(250ms);  // Beyond command_timeout_us, even from the first drain.
  const auto first_diagnostic = ::testing::internal::GetCapturedStderr();
  observation =
      sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
  if (!observation.has_value()) {
    FAIL() << "missing camera observation";
  }
  EXPECT_FALSE(observation->valid());
  auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
  if (!status.has_value()) {
    FAIL() << "missing camera status";
  }
  EXPECT_FALSE(status->healthy());
  EXPECT_EQ(status->visible_tag_count(), 0u);
  EXPECT_NE(first_diagnostic.find("sim_gateway --physics"), std::string::npos);
  EXPECT_EQ(std::count(first_diagnostic.begin(), first_diagnostic.end(), '\n'),
            1);
  sim.publish(sim::topics::kState, Truth(sim));
  sim.step(50ms);
  observation =
      sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
  if (!observation.has_value()) {
    FAIL() << "missing camera observation";
  }
  EXPECT_TRUE(observation->valid());
  status = sim.latest<VisionStatus>("/rear_right_camera/status");
  if (!status.has_value()) {
    FAIL() << "missing camera status";
  }
  EXPECT_TRUE(status->healthy());
  ::testing::internal::CaptureStderr();
  sim.step(250ms);
  const auto second_diagnostic = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(second_diagnostic, first_diagnostic);
}

TEST(VisionTest, FacingAwayPublishesInvalidInsteadOfHoldingLastPose) {
  auto sim = MakeSim(false);
  sim.add<Vision>("rear_right_camera");
  sim.publish(sim::topics::kState, Truth(sim));
  sim.step(50ms);
  bool found_empty_view = false;
  for (int i = 0; i < 16; ++i) {
    // A stationary robot can rotate its camera away from the field's tags.
    sim.publish(sim::topics::kState,
                Truth(sim, {2, 2, i * std::numbers::pi / 8}));
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

TEST(VisionTest, GatewayTruthMovesIndependentlyOfStaleEstimator) {
  // This checks truth routing, not the random corner-noise distribution.
  auto sim = MakeSim(false, true);
  for (const auto* name : kCameras) sim.add<Vision>(name);
  sim.publish("/pose_estimator/state",
              geometry::PoseEstimate{0, geometry::Pose2d{10, 5, 1},
                                     geometry::Twist2d{}, true, 0, 0});
  sim.publish(sim::topics::kState, Truth(sim));
  sim.step(50ms);
  const auto truth = Truth(sim, {2.1, 2, 0});
  sim.publish(sim::topics::kState, truth);
  sim.step(50ms);
  // Even a new estimator pose cannot move the world during a truth pause.
  sim.publish("/pose_estimator/state",
              geometry::PoseEstimate{100000000, geometry::Pose2d{8, 4, -1},
                                     geometry::Twist2d{}, true, 0, 0});
  sim.step(50ms);
  int valid = 0;
  for (const auto* name : kCameras) {
    const auto observation = sim.latest<geometry::PoseObservation>(
        std::string("/") + name + "/state");
    if (!observation.has_value()) {
      FAIL() << "missing camera observation";
    }
    if (!observation->valid()) continue;
    ++valid;
    EXPECT_EQ(observation->sample_time_ns(), truth.sample_time_ns());
    EXPECT_NEAR(observation->pose().x_m(), 2.1, 0.06);
    EXPECT_NEAR(observation->pose().y_m(), 2, 0.06);
  }
  EXPECT_GT(valid, 0);
}

TEST(VisionTest, AbsoluteTruthJumpsAndCameraRestartsNeverReseedTheWorld) {
  const geometry::Pose2d new_pose{3, 2.5, 0.15};
  // Isolate absolute-pose/restart behavior from random corner measurement
  // noise.
  auto check = [&](const talos::testing::Sim& sim,
                   const geometry::OdometrySample& truth) {
    const auto observation =
        sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
    if (!observation.has_value()) {
      FAIL() << "missing camera observation";
    }
    ASSERT_TRUE(observation->valid());
    EXPECT_EQ(observation->sample_time_ns(), truth.sample_time_ns());
    EXPECT_NEAR(observation->pose().x_m(), new_pose.x_m(), 0.06);
    EXPECT_NEAR(observation->pose().y_m(), new_pose.y_m(), 0.06);
    EXPECT_NEAR(observation->pose().yaw_rad(), new_pose.yaw_rad(), 0.02);
  };
  {
    auto sim = MakeSim(false, true);
    sim.add<Vision>("rear_right_camera");
    sim.publish(sim::topics::kState, Truth(sim));
    sim.step(50ms);
    const auto truth = Truth(sim, new_pose);
    sim.publish(sim::topics::kState, truth);
    sim.step(50ms);
    check(sim, truth);
  }
  // A newly launched camera receives the gateway's current absolute pose,
  // not an initial pose or motion that needs integrating from camera startup.
  auto restarted = MakeSim(false, true);
  restarted.add<Vision>("rear_right_camera");
  const auto truth = Truth(restarted, new_pose);
  restarted.publish(sim::topics::kState, truth);
  restarted.step(50ms);
  check(restarted, truth);
}

TEST(VisionTest, RejectsInvalidSyntheticExposureAndSensorConfiguration) {
  for (const auto* key :
       {"sensor_noise_std_gray", "exposure_gain_std", "exposure_ms"}) {
    for (const auto* value : {"-1.0", "nan", "inf"}) {
      SCOPED_TRACE(std::string(key) + "=" + value);
      std::ostringstream text;
      text << "include = "
           << std::quoted(
                  std::filesystem::absolute(Runfile("TALOS_TEST_CONFIG"))
                      .string())
           << "\n[subsystems.front_left_camera.camera]\n"
           << key << " = " << value << '\n';
      auto sim = talos::testing::Sim::FromToml(text.str());
      EXPECT_THROW(sim.add<Vision>("front_left_camera"), std::runtime_error);
    }
  }
}

talos::testing::Sim WithCameraOverride(const char* section, const char* key,
                                       const char* value) {
  std::ostringstream text;
  text << "include = "
       << std::quoted(
              std::filesystem::absolute(Runfile("TALOS_TEST_CONFIG")).string())
       << "\n[subsystems.front_left_camera" << section << "]\n"
       << key << " = " << value << '\n';
  return talos::testing::Sim::FromToml(text.str());
}

TEST(VisionTest, SimMountErrorsMustBeFinite) {
  for (const auto* key :
       {"sim_mount_error_x_m", "sim_mount_error_y_m", "sim_mount_error_z_m",
        "sim_mount_error_roll_rad", "sim_mount_error_pitch_rad",
        "sim_mount_error_yaw_rad"}) {
    for (const auto* value : {"nan", "inf", "-inf"}) {
      auto sim = WithCameraOverride(".camera", key, value);
      EXPECT_THROW(sim.add<Vision>("front_left_camera"), std::runtime_error);
    }
  }
}

TEST(VisionTest, InvalidTimingSettingsNameNodeAndKey) {
  for (const auto* key : {"latency_offset_us", "sim_timestamp_lag_us"}) {
    for (const auto* value : {"-1.0", "nan", "inf", "-inf", "1000000.0"}) {
      SCOPED_TRACE(std::string(key) + "=" + value);
      auto sim = WithCameraOverride(".camera", key, value);
      try {
        sim.add<Vision>("front_left_camera");
        FAIL() << "accepted invalid " << key;
      } catch (const std::exception& error) {
        EXPECT_NE(std::string(error.what()).find("front_left_camera"),
                  std::string::npos);
        EXPECT_NE(std::string(error.what()).find(std::string("camera.") + key),
                  std::string::npos);
      }
    }
  }
}

TEST(VisionTest, InvalidDetectorAndCaptureSettingsNameNodeAndKey) {
  struct Case {
    const char* section;
    const char* key;
    const char* value;
  };
  for (const auto& test :
       {Case{".detector", "decimate", "0.5"},
        Case{".detector", "decimate", "nan"},
        Case{".detector", "threads", "17"}, Case{".detector", "threads", "1.5"},
        Case{".detector", "sigma", "5.0"},
        Case{".detector", "refine_edges", "\"yes\""},
        Case{".detector", "hamming_threshold", "3"},
        Case{".detector", "min_decision_margin", "-1.0"},
        Case{".detector", "min_decision_margin", "inf"},
        Case{"", "backend", "\"invalid\""}, Case{".camera", "fps", "0.0"},
        Case{".camera", "fps", "-1.0"}, Case{".camera", "fps", "nan"},
        Case{".camera", "fps", "inf"}, Case{".camera", "width", "0"},
        Case{".camera", "height", "0"}}) {
    const std::string key =
        std::string(test.section[0] ? test.section + 1 : "") +
        (test.section[0] ? "." : "") + test.key;
    SCOPED_TRACE(key);
    auto sim = WithCameraOverride(test.section, test.key, test.value);
    try {
      sim.add<Vision>("front_left_camera");
      FAIL() << "accepted invalid " << key;
    } catch (const std::exception& error) {
      EXPECT_NE(std::string(error.what()).find("front_left_camera"),
                std::string::npos)
          << error.what();
      EXPECT_NE(std::string(error.what()).find(key), std::string::npos)
          << error.what();
    }
  }
}

TEST(VisionTest, ConfiguredCaptureFpsReachesSourceSettingsWithoutHardware) {
  auto defaults = MakeSim(false);
  EXPECT_EQ(defaults.add<Vision>("front_left_camera").capture_config().fps, 60);
  auto configured = WithCameraOverride(".camera", "fps", "59.94");
  const auto& capture =
      configured.add<Vision>("front_left_camera").capture_config();
  EXPECT_DOUBLE_EQ(capture.fps, 59.94);
  EXPECT_EQ(capture.width, 1280u);
  EXPECT_EQ(capture.height, 720u);
}

TEST(VisionTest, SimulatedCaptureFollowsTruthTimeWithoutPhaseDrift) {
  auto sim = MakeSim(false, true, "fps = 60\n", "period_us = 10000\n");
  sim.add<Vision>("rear_right_camera");
  auto& observer = sim.add<CameraObserver>("pose_estimator");
  sim.publish(sim::topics::kState, Truth(sim));
  sim.step(10ms);
  for (int i = 0; i < 100; ++i) {
    sim.publish(sim::topics::kState, Truth(sim));
    sim.step(10ms);
  }
  sim.step(1ms);  // Deliver the final captured observation to the callback.
  auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
  if (!status.has_value()) {
    FAIL() << "missing camera status";
  }
  EXPECT_EQ(status->frames_captured_count(), 61u);
  EXPECT_EQ(status->frames_processed_count(), 61u);
  EXPECT_EQ(status->frames_dropped_count(), 0u);
  EXPECT_EQ(status->frame_period_us(), 16666u);
  ASSERT_EQ(observer.frames.size(), 61u);
  std::size_t short_intervals = 0;
  std::size_t long_intervals = 0;
  for (std::size_t i = 0; i < observer.frames.size(); ++i) {
    SCOPED_TRACE(i);
    const auto stamp = observer.frames[i].observation.sample_time_ns();
    // Capture the first real truth sample at or past each 1/60 s deadline.
    EXPECT_EQ(stamp, ((i * 5 + 2) / 3) * 10'000'000u);
    EXPECT_TRUE(observer.frames[i].observation.valid());
    if (i == 0) continue;
    const auto interval =
        stamp - observer.frames[i - 1].observation.sample_time_ns();
    if (interval == 10'000'000u) ++short_intervals;
    if (interval == 20'000'000u) ++long_intervals;
    EXPECT_TRUE(interval == 10'000'000u || interval == 20'000'000u);
  }
  EXPECT_EQ(short_intervals, 20u);
  EXPECT_EQ(long_intervals, 40u);
  const auto elapsed = observer.frames.back().observation.sample_time_ns() -
                       observer.frames.front().observation.sample_time_ns();
  EXPECT_NEAR(static_cast<double>(elapsed) /
                  static_cast<double>(observer.frames.size() - 1),
              1.0e9 / 60, 1.0);
  // Advancing node time without advancing truth must not invent captures.
  sim.step(20ms);
  const auto current_status =
      sim.latest<VisionStatus>("/rear_right_camera/status");
  if (!current_status.has_value()) {
    FAIL() << "missing camera status";
  }
  EXPECT_EQ(current_status->frames_captured_count(), 61u);
  EXPECT_EQ(current_status->frames_dropped_count(), 0u);
  EXPECT_EQ(observer.frames.size(), 61u);
}

TEST(VisionTest, SlowTicksCountSkippedDeadlinesWithoutSynthesizingCaptures) {
  auto sim = MakeSim(false, true, "fps = 60\n", "period_us = 50000\n");
  sim.add<Vision>("rear_right_camera");
  auto& observer = sim.add<CameraObserver>("pose_estimator");
  for (int i = 0; i < 4; ++i) {
    sim.publish(sim::topics::kState, Truth(sim));
    sim.step(50ms);
  }
  sim.step(1ms);
  const auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
  if (!status.has_value()) {
    FAIL() << "missing camera status";
  }
  EXPECT_EQ(status->frames_captured_count(), 4u);
  EXPECT_EQ(status->frames_processed_count(), 4u);
  EXPECT_EQ(status->frames_dropped_count(), 6u);
  EXPECT_EQ(status->frame_period_us(), 50000u);
  ASSERT_EQ(observer.frames.size(), 4u);
  for (std::size_t i = 0; i < observer.frames.size(); ++i) {
    EXPECT_TRUE(observer.frames[i].observation.valid());
    EXPECT_EQ(observer.frames[i].observation.sample_time_ns(), i * 50'000'000u);
  }
}

TEST(VisionTest, ConfiguredMarginExcludesWeakDecodesFromLocalization) {
  auto sim = WithCameraOverride(".detector", "min_decision_margin", "1000.0");
  sim.add<Vision>("front_left_camera");
  sim.publish(sim::topics::kState, Truth(sim));
  sim.step(50ms);
  const auto status = sim.latest<VisionStatus>("/front_left_camera/status");
  if (!status.has_value()) {
    FAIL() << "missing camera status";
  }
  EXPECT_TRUE(status->healthy());
  EXPECT_EQ(status->visible_tag_count(), 0u);
  const auto observation =
      sim.latest<geometry::PoseObservation>("/front_left_camera/state");
  if (!observation.has_value()) {
    FAIL() << "missing camera observation";
  }
  EXPECT_FALSE(observation->valid());
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
  auto run = [](const std::string& arguments,
                const std::string& config = Runfile("TALOS_TEST_CONFIG")) {
    const std::string command = "'" + Runfile("TALOS_TEST_NODE") + "' " +
                                arguments + " --config '" + config + "' 2>&1";
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
  const auto live_config =
      std::string(temporary_directory) + "/ignored_sim_mount_errors.toml";
  {
    std::ofstream config(live_config);
    config
        << "include = "
        << std::quoted(
               std::filesystem::absolute(Runfile("TALOS_TEST_CONFIG")).string())
        << "\n[subsystems.front_left_camera.camera]\n";
    for (const auto* key :
         {"sim_mount_error_x_m", "sim_mount_error_y_m", "sim_mount_error_z_m",
          "sim_mount_error_roll_rad", "sim_mount_error_pitch_rad",
          "sim_mount_error_yaw_rad"}) {
      config << key << " = nan\n";
    }
  }
  const auto [live_code, live_output] = run("--describe", live_config);
  EXPECT_EQ(live_code, 0) << live_output;
  const auto log = std::string(temporary_directory) + "/camera.tlog";
  const auto [sim_code, sim_output] =
      run("--sim --duration-s 1 --log '" + log + "'");
  ASSERT_EQ(sim_code, 0) << sim_output;
  const auto [replay_code, replay_output] = run("--sim --replay '" + log + "'");
  EXPECT_EQ(replay_code, 0) << replay_output;
  EXPECT_NE(replay_output.find("diverged=0 clean_exit=1"), std::string::npos)
      << replay_output;
}

std::filesystem::path RecordingTempDir(const std::string& tag) {
  const auto dir = std::filesystem::path{::testing::TempDir()} /
                   ("talos_vision_" + tag + "_" + std::to_string(::getpid()));
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

std::vector<talos::vision::recording::FrameRecord> ReadRecordedFrames(
    const std::filesystem::path& frames_dir) {
  std::vector<talos::vision::recording::FrameRecord> out;
  if (!std::filesystem::exists(frames_dir)) return out;
  for (const auto& entry : std::filesystem::directory_iterator(frames_dir)) {
    const std::string name = entry.path().filename().string();
    if (name.size() < 5 || name.substr(name.size() - 5) != ".json") continue;
    std::ifstream in(entry.path());
    const std::string json{std::istreambuf_iterator<char>{in}, {}};
    talos::vision::recording::CameraContext context;
    talos::vision::recording::FrameRecord record;
    std::string error;
    if (talos::vision::recording::ParseRecord(json, &context, &record,
                                              &error)) {
      out.push_back(std::move(record));
    }
  }
  return out;
}

TEST(VisionTest, RecorderConstructionFailureLeavesCameraRunning) {
  const auto session = RecordingTempDir("uncreatable");
  std::ofstream{session / "frames"} << "not a directory";
  {
    auto sim = MakeSim(false, true);
    sim.set_session_dir(session);
    testing::internal::CaptureStderr();
    EXPECT_NO_THROW(sim.add<Vision>("rear_right_camera"));
    const auto diagnostic = testing::internal::GetCapturedStderr();
    EXPECT_NE(diagnostic.find("rear_right_camera: recording disabled:"),
              std::string::npos);
    EXPECT_EQ(std::count(diagnostic.begin(), diagnostic.end(), '\n'), 1);
    // Sensing must still work while disabled, with no actuation targets.
    sim.step(1ms);
    sim.publish(sim::topics::kState, Truth(sim));
    sim.step(50ms);
    const auto observation =
        sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
    if (!observation.has_value()) {
      FAIL() << "missing camera observation";
    }
    EXPECT_TRUE(observation->valid());
  }
  std::filesystem::remove_all(session);
}

TEST(VisionTest, RecordsSampledFramesWithTruthWhenSessionDirIsSet) {
  const auto session = RecordingTempDir("sampled");
  {
    auto sim = MakeSim(true, true);
    sim.set_session_dir(session);
    sim.add<Vision>("rear_right_camera");
    sim.step(1ms);
    sim.publish(sim::topics::kState, Truth(sim));
    sim.step(50ms);
    sim.publish(sim::topics::kState, Truth(sim));
    sim.step(50ms);
  }
  const auto frames =
      ReadRecordedFrames(session / "frames" / "rear_right_camera");
  ASSERT_FALSE(frames.empty());
  bool found_sampled = false;
  for (const auto& frame : frames) {
    if (frame.reason == "sampled" && frame.observation.valid) {
      found_sampled = true;
      ASSERT_TRUE(frame.truth.has_value());
    }
  }
  EXPECT_TRUE(found_sampled);
}

TEST(VisionTest, RecordsRejectedFramesWhenAllTagsAreRangeRejected) {
  const auto session = RecordingTempDir("rejected");
  {
    auto sim = MakeSim(true, true, "max_range_m = 0.1\n");
    sim.set_session_dir(session);
    sim.add<Vision>("rear_right_camera");
    sim.step(1ms);
    sim.publish(sim::topics::kState, Truth(sim));
    sim.step(50ms);
    sim.publish(sim::topics::kState, Truth(sim));
    sim.step(50ms);
  }
  const auto frames =
      ReadRecordedFrames(session / "frames" / "rear_right_camera");
  ASSERT_FALSE(frames.empty());
  bool found_rejected = false;
  for (const auto& frame : frames) {
    if (frame.reason == "rejected" && !frame.observation.valid) {
      found_rejected = true;
      ASSERT_TRUE(frame.truth.has_value());
    }
  }
  EXPECT_TRUE(found_rejected);
}

TEST(VisionTest, RecorderAttributesAmbiguityOnlyToSingleTagRejections) {
  // At the auto end pose the rear camera sees two grazing (high-ambiguity)
  // tags. The joint solve resolves the planar ambiguity, so neither detection
  // may be marked "ambiguity" in the sidecar: that reason is reserved for a
  // single-tag observation the gate actually rejected.
  const auto session = RecordingTempDir("ambiguity_rule");
  {
    auto sim = MakeSim(true, true);
    sim.set_session_dir(session);
    sim.add<Vision>("rear_right_camera");
    sim.step(1ms);
    for (int i = 0; i < 8; ++i) {
      sim.publish(sim::topics::kState, Truth(sim, {6.08, 4.06, 0}));
      sim.step(50ms);
    }
  }
  const auto frames =
      ReadRecordedFrames(session / "frames" / "rear_right_camera");
  ASSERT_FALSE(frames.empty());
  bool found_ambiguous_multi_tag = false;
  for (const auto& frame : frames) {
    if (frame.detections.size() < 2) continue;
    bool any_ambiguous = false;
    for (const auto& d : frame.detections) {
      if (d.ambiguity > 0.2) any_ambiguous = true;
      ASSERT_FALSE(d.reject_reason.has_value() &&
                   *d.reject_reason == "ambiguity")
          << "multi-tag detection tagged 'ambiguity'";
    }
    if (any_ambiguous) found_ambiguous_multi_tag = true;
  }
  EXPECT_TRUE(found_ambiguous_multi_tag)
      << "expected a recorded two-tag frame with a tag ambiguity above 0.2";
}

TEST(VisionTest, RecordingDisabledOrNoSessionDirWritesNothing) {
  const auto no_session = RecordingTempDir("no_session");
  {
    auto sim = MakeSim(true, true);
    sim.add<Vision>("rear_right_camera");
    sim.step(1ms);
    sim.publish(sim::topics::kState, Truth(sim));
    sim.step(50ms);
  }
  EXPECT_FALSE(std::filesystem::exists(no_session / "frames"));

  const auto disabled = RecordingTempDir("disabled");
  {
    auto sim = MakeSim(true, true, {}, "recording.enabled = false\n");
    sim.set_session_dir(disabled);
    sim.add<Vision>("rear_right_camera");
    sim.step(1ms);
    sim.publish(sim::topics::kState, Truth(sim));
    sim.step(50ms);
  }
  EXPECT_FALSE(std::filesystem::exists(disabled / "frames"));
}

TEST(VisionTest, InvalidRecordingParamsFailConstruction) {
  for (const char* bad :
       {"recording.max_bytes = -1\n", "recording.sample_hz = -1.0\n",
        "recording.rejected_hz = nan\n"}) {
    SCOPED_TRACE(bad);
    const auto session = RecordingTempDir("bad");
    auto sim = MakeSim(true, true, {}, bad);
    sim.set_session_dir(session);
    EXPECT_THROW(sim.add<Vision>("rear_right_camera"), std::invalid_argument);
  }
}

TEST(VisionTest, InvalidMountUncertaintyParamsFailConstruction) {
  // Mount calibration uncertainty must be finite and nonnegative.
  for (const char* bad :
       {"mount_angle_std_rad = -1\n", "mount_angle_std_rad = nan\n",
        "mount_translation_std_m = -0.1\n",
        "mount_translation_std_m = inf\n"}) {
    SCOPED_TRACE(bad);
    auto sim = MakeSim(true, true, bad);
    EXPECT_THROW(sim.add<Vision>("rear_right_camera"), std::runtime_error);
  }
}

TEST(VisionTest, ShutdownWithRecordingActiveCompletesPromptly) {
  const auto session = RecordingTempDir("shutdown");
  const auto start = std::chrono::steady_clock::now();
  {
    auto sim = MakeSim(true, true);
    sim.set_session_dir(session);
    sim.add<Vision>("rear_right_camera");
    sim.step(1ms);
    sim.publish(sim::topics::kState, Truth(sim));
    sim.step(50ms);
    sim.publish(sim::topics::kState, Truth(sim));
    sim.step(50ms);
  }  // Vision destructor joins the worker, then the bounded recorder
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(elapsed, std::chrono::seconds{5})
      << "shutdown with recording active took "
      << std::chrono::duration<double>(elapsed).count() << " s";
}
}  // namespace
}  // namespace vision
