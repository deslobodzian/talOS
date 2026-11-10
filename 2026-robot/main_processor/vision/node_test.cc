#include <gtest/gtest.h>
#include <sys/wait.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/node/testing.h"
#include "talOS/vision/recording/recording.h"
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

// A raster sim camera renders and detects on its worker thread, whose wall
// time (long under sanitizers) must not become simulated time: production
// drops a result older than frame_timeout_us. Step 1 ms at a time until
// `ready`, republishing `truth` so it stays fresh without becoming a new
// capture, and after each step wait in wall time for every camera's worker to
// store its result, which the next step collects. So each result reaches
// update() at most 1 ms of simulated time after its request, however slow the
// worker. Both bounds only end a failing run.
template <typename Ready>
bool AdvanceSimUntil(talos::testing::Sim& sim,
                     const std::vector<Vision*>& cameras,
                     const geometry::OdometrySample& truth, Ready ready) {
  for (int i = 0; i < 1000 && !ready(); ++i) {
    sim.publish(sim::topics::kState, truth);
    sim.step(1ms);
    for (auto* camera : cameras) {
      if (!camera->WaitForSimWorker(60s)) return false;
    }
  }
  return ready();
}

// True once `camera` has published the observation of `truth`'s capture.
bool Observed(const talos::testing::Sim& sim, std::string_view camera,
              const geometry::OdometrySample& truth) {
  const auto observation = sim.latest<geometry::PoseObservation>(
      "/" + std::string(camera) + "/state");
  return observation && observation->sample_time_ns() == truth.sample_time_ns();
}

void CheckPose(bool rasterize) {
  // Isolate direct-projection geometry from corner noise; the raster path
  // retains production sensor/exposure noise.
  auto sim = MakeSim(rasterize, !rasterize);
  std::vector<Vision*> cameras;
  for (const auto* name : kCameras) cameras.push_back(&sim.add<Vision>(name));
  // No driver station, targets, estimator or drivetrain: sensing runs disabled
  // from gateway truth, without requiring fresh actuation.
  sim.step(1ms);
  sim.publish(sim::topics::kState, Truth(sim));
  sim.step(50ms);
  const auto truth = Truth(sim);
  sim.publish(sim::topics::kState, truth);
  sim.step(50ms);
  if (rasterize) {
    ASSERT_TRUE(AdvanceSimUntil(sim, cameras, truth, [&] {
      return std::all_of(kCameras.begin(), kCameras.end(),
                         [&](auto name) { return Observed(sim, name, truth); });
    }));
  }
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
    if (i < 3) {
      EXPECT_TRUE(observer.frames.empty());
    }
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

TEST(VisionTest, SimMountYawErrorBiasesObservations) {
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
  const auto yaw_error =
      sample("pitch_rad = 0.0\nsim_mount_error_yaw_rad = 0.02\n");
  int compared = 0;
  for (size_t i = 0; i < kCameras.size(); ++i) {
    SCOPED_TRACE(kCameras[i]);
    if (implicit_zero[i].valid()) {
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
    const auto first = diagnostic.find(name);
    ASSERT_NE(first, std::string::npos) << diagnostic;
    EXPECT_EQ(diagnostic.find(name, first + std::string_view(name).size()),
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
  EXPECT_EQ(
      std::count(second_diagnostic.begin(), second_diagnostic.end(), '\n'), 1);
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
                                     geometry::Twist2d{}, true, true, 0, 0});
  sim.publish(sim::topics::kState, Truth(sim));
  sim.step(50ms);
  const auto truth = Truth(sim, {2.1, 2, 0});
  sim.publish(sim::topics::kState, truth);
  sim.step(50ms);
  // Even a new estimator pose cannot move the world during a truth pause.
  sim.publish("/pose_estimator/state",
              geometry::PoseEstimate{100000000, geometry::Pose2d{8, 4, -1},
                                     geometry::Twist2d{}, true, true, 0, 0});
  sim.step(35ms);
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

TEST(VisionTest, RejectsInvalidTimingSettings) {
  for (const auto* key : {"latency_offset_us", "sim_timestamp_lag_us"}) {
    for (const auto* value : {"-1.0", "nan", "inf", "-inf", "1000000.0"}) {
      SCOPED_TRACE(std::string(key) + "=" + value);
      auto sim = WithCameraOverride(".camera", key, value);
      EXPECT_THROW(sim.add<Vision>("front_left_camera"), std::exception);
    }
  }
}

TEST(VisionTest, RejectsInvalidDetectorAndCaptureSettings) {
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
    EXPECT_THROW(sim.add<Vision>("front_left_camera"), std::exception);
  }
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
  auto& camera = sim.add<Vision>("front_left_camera");
  const auto truth = Truth(sim);
  sim.publish(sim::topics::kState, truth);
  sim.step(50ms);
  ASSERT_TRUE(AdvanceSimUntil(sim, {&camera}, truth, [&] {
    return Observed(sim, "front_left_camera", truth);
  }));
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

// A CPU detector that, while held, blocks each Execute() for at most 5 s (as
// a slow or stuck detector would), recording the calling thread and the frame.
class GatedBackend final : public talos::vision::ComputeBackend {
 public:
  std::string name() const override { return "gated"; }
  std::string device_info() const override { return "gated cpu"; }
  bool supports_gpu() const override { return false; }
  talos::vision::BackendExecutionResult Execute(
      const talos::vision::Frame& frame) override {
    {
      std::unique_lock lock(mutex_);
      threads_.push_back(std::this_thread::get_id());
      frames_.push_back(frame.capture_timestamp_ns());
      // A cap that expires releases the gate, so a failing run stays short.
      if (!changed_.wait_for(lock, 5s, [this] { return !held_; })) {
        held_ = false;
      }
    }
    return cpu_.Execute(frame);
  }
  void Hold(bool held) {
    {
      std::lock_guard lock(mutex_);
      held_ = held;
    }
    changed_.notify_all();
  }
  std::vector<std::thread::id> threads() {
    std::lock_guard lock(mutex_);
    return threads_;
  }
  std::vector<std::uint64_t> frames() {
    std::lock_guard lock(mutex_);
    return frames_;
  }

 private:
  talos::vision::CpuAprilTagBackend cpu_{2, 2.0};
  std::mutex mutex_;
  std::condition_variable changed_;
  bool held_ = false;
  std::vector<std::thread::id> threads_;
  std::vector<std::uint64_t> frames_;
};

struct GatedSimVision : Vision {
  inline static std::shared_ptr<GatedBackend> backend;
  explicit GatedSimVision(talos::Node& node) : Vision(node, {}, backend) {}
};

// A raster sim frame's render, detection and solve run off the loop thread:
// a detector slower than the 10 ms period never holds update(), status keeps
// publishing, captures due meanwhile replace each other and are counted, and
// only the newest reaches the detector once it frees up.
TEST(VisionTest, SlowSimDetectorNeverHoldsTheLoop) {
  auto backend = std::make_shared<GatedBackend>();
  GatedSimVision::backend = backend;
  auto sim = MakeSim(true, true, {}, "period_us = 10000\n");
  auto& camera = sim.add<GatedSimVision>("rear_right_camera");
  auto& observer = sim.add<CameraObserver>("pose_estimator");
  // Declared after sim, so a failed assertion releases the worker before the
  // node joins it.
  struct Release {
    GatedBackend& backend;
    ~Release() { backend.Hold(false); }
  } release{*backend};
  backend->Hold(true);
  const auto loop_thread = std::this_thread::get_id();
  geometry::OdometrySample truth;
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 20; ++i) {
    truth = Truth(sim);
    sim.publish(sim::topics::kState, truth);
    sim.step(10ms);
  }
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(elapsed, 2s) << std::chrono::duration<double>(elapsed).count();
  for (const auto& thread : backend->threads()) {
    EXPECT_NE(thread, loop_thread);
  }
  const auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
  ASSERT_TRUE(status);
  // Truth at 0..190 ms on the 60 fps grid: 12 captures, the last at 190 ms.
  const auto captured = status->frames_captured_count();
  EXPECT_EQ(captured, 12u);
  // At most the frame in the detector and the newest request are not lost.
  EXPECT_GE(status->frames_dropped_count(), captured - 2);
  EXPECT_TRUE(observer.frames.empty());
  backend->Hold(false);
  ASSERT_TRUE(AdvanceSimUntil(sim, {&camera}, truth, [&] {
    return !observer.frames.empty() &&
           observer.frames.back().observation.sample_time_ns() ==
               truth.sample_time_ns();
  }));
  // Only the frame already in the detector and the newest were detected; no
  // replaced capture reached the detector or an observation.
  const auto detected = backend->frames();
  ASSERT_LE(detected.size(), 2u);
  EXPECT_EQ(detected.back(), truth.sample_time_ns());
  for (const auto& frame : observer.frames) {
    EXPECT_NE(std::find(detected.begin(), detected.end(),
                        frame.observation.sample_time_ns()),
              detected.end());
  }
  EXPECT_TRUE(observer.frames.back().observation.valid());
}

// A raster result the worker returns more than frame_timeout_us after its
// truth instant is dropped and counted, never published valid with its old
// stamp; the next result, inside the timeout, still publishes.
TEST(VisionTest, SimResultOlderThanFrameTimeoutIsDropped) {
  auto backend = std::make_shared<GatedBackend>();
  GatedSimVision::backend = backend;
  auto sim =
      MakeSim(true, true, "frame_timeout_us = 300000\n", "period_us = 10000\n");
  auto& camera = sim.add<GatedSimVision>("rear_right_camera");
  auto& observer = sim.add<CameraObserver>("pose_estimator");
  struct Release {
    GatedBackend& backend;
    ~Release() { backend.Hold(false); }
  } release{*backend};
  backend->Hold(true);
  // One capture enters the detector; republishing the same truth keeps it
  // fresh without making another capture due.
  const auto truth = Truth(sim);
  for (int i = 0; i < 35; ++i) {
    sim.publish(sim::topics::kState, truth);
    sim.step(10ms);
    // The capture must be in the detector before its hold counts.
    for (int w = 0; i == 0 && w < 2000 && backend->frames().empty(); ++w) {
      std::this_thread::sleep_for(1ms);
    }
  }
  ASSERT_EQ(backend->frames(),
            std::vector<std::uint64_t>{truth.sample_time_ns()});
  auto dropped = [&] {
    return sim.latest<VisionStatus>("/rear_right_camera/status")
        ->frames_dropped_count();
  };
  const auto dropped_before = dropped();
  backend->Hold(false);
  // 350 ms after its truth instant the result is collected: dropped, not
  // published.
  ASSERT_TRUE(AdvanceSimUntil(sim, {&camera}, truth, [&] {
    return !observer.frames.empty() || dropped() > dropped_before;
  }));
  EXPECT_TRUE(observer.frames.empty());
  EXPECT_EQ(dropped(), dropped_before + 1);
  // A result inside the timeout still publishes valid.
  const auto next = Truth(sim);
  ASSERT_TRUE(AdvanceSimUntil(sim, {&camera}, next, [&] {
    return !observer.frames.empty() &&
           observer.frames.back().observation.sample_time_ns() ==
               next.sample_time_ns();
  }));
  EXPECT_TRUE(observer.frames.back().observation.valid());
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
  estimate.T_field_robot = talos::vision::field_sim::RobotPoseOnField(2, 2, 0);
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
    auto& camera = sim.add<Vision>("rear_right_camera");
    sim.step(1ms);
    sim.publish(sim::topics::kState, Truth(sim));
    sim.step(50ms);
    const auto truth = Truth(sim);
    sim.publish(sim::topics::kState, truth);
    sim.step(50ms);
    ASSERT_TRUE(AdvanceSimUntil(sim, {&camera}, truth, [&] {
      return Observed(sim, "rear_right_camera", truth);
    }));
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
    auto& camera = sim.add<Vision>("rear_right_camera");
    sim.step(1ms);
    sim.publish(sim::topics::kState, Truth(sim));
    sim.step(50ms);
    const auto truth = Truth(sim);
    sim.publish(sim::topics::kState, truth);
    sim.step(50ms);
    ASSERT_TRUE(AdvanceSimUntil(sim, {&camera}, truth, [&] {
      return Observed(sim, "rear_right_camera", truth);
    }));
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
    auto& camera = sim.add<Vision>("rear_right_camera");
    sim.step(1ms);
    geometry::OdometrySample truth;
    for (int i = 0; i < 8; ++i) {
      truth = Truth(sim, {6.08, 4.06, 0});
      sim.publish(sim::topics::kState, truth);
      sim.step(50ms);
    }
    ASSERT_TRUE(AdvanceSimUntil(sim, {&camera}, truth, [&] {
      return Observed(sim, "rear_right_camera", truth);
    }));
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

TEST(VisionTest, RecordingDisabledWritesNothing) {
  const auto disabled = RecordingTempDir("disabled");
  {
    auto sim = MakeSim(true, true, {}, "recording.enabled = false\n");
    sim.set_session_dir(disabled);
    auto& camera = sim.add<Vision>("rear_right_camera");
    sim.step(1ms);
    const auto truth = Truth(sim);
    sim.publish(sim::topics::kState, truth);
    sim.step(50ms);
    ASSERT_TRUE(AdvanceSimUntil(sim, {&camera}, truth, [&] {
      return Observed(sim, "rear_right_camera", truth);
    }));
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

std::size_t CountLines(const std::string& text, std::string_view needle) {
  std::size_t count = 0;
  std::istringstream lines(text);
  for (std::string line; std::getline(lines, line);) {
    if (line.find(needle) != std::string::npos) ++count;
  }
  return count;
}

struct FakeCameraDevice {
  std::mutex mutex;
  talos::vision::FrameCallback callback;
  talos::vision::ConnectionCallback connection;
  talos::vision::CameraSourceConfig config;
  std::atomic<int> starts{0}, stops{0};
  std::atomic<bool> allow_start{true}, apply_controls{true};
  // Reason a refused Start() reports through the connection callback.
  std::string start_failure;
  // Gray8 image emitted at the configured size; blank when unset.
  std::shared_ptr<const std::vector<std::uint8_t>> pixels;

  // While held, the worker blocks in its next IsRunning() call, as it would
  // behind a detection slower than the frame period.
  void Hold(bool hold) {
    {
      std::lock_guard lock(gate_mutex);
      held = hold;
    }
    gate_changed.notify_all();
  }
  void WaitGate() {
    std::unique_lock lock(gate_mutex);
    blocked = held;
    gate_changed.notify_all();
    gate_changed.wait(lock, [this] { return !held; });
    blocked = false;
  }
  bool WaitBlocked() {
    std::unique_lock lock(gate_mutex);
    return gate_changed.wait_for(lock, 5s, [this] { return blocked; });
  }
  std::mutex gate_mutex;
  std::condition_variable gate_changed;
  bool held = false, blocked = false;

  void Emit(std::int64_t stamp, talos::vision::TimestampKind kind) {
    std::lock_guard lock(mutex);
    if (!callback) return;
    const auto time = talos::Time::from_nanos(stamp);
    talos::vision::Frame frame =
        pixels ? talos::vision::Frame(
                     config.camera_id, 1, 0, time, config.width, config.height,
                     config.width, talos::vision::PixelFormat::kGray8, pixels)
               : talos::vision::Frame(config.camera_id, 1, 0, time,
                                      config.width, config.height);
    frame.set_timestamp_kind(kind);
    callback(frame);
  }
};

class FakeCameraSource final : public talos::vision::CameraSource {
 public:
  FakeCameraSource(std::shared_ptr<FakeCameraDevice> device,
                   const talos::vision::CameraSourceConfig& config)
      : device_(std::move(device)) {
    std::lock_guard lock(device_->mutex);
    device_->config = config;
  }
  uint32_t camera_id() const override { return 0; }
  std::string device_id() const override { return "fake"; }
  std::string name() const override { return "fake"; }
  std::vector<talos::vision::CameraMode> SupportedModes() const override {
    return {};
  }
  talos::vision::CameraMode CurrentMode() const override { return {}; }
  bool SetMode(const talos::vision::CameraMode&) override { return true; }
  talos::vision::CameraControls GetControls() const override { return {}; }
  bool SetControls(const talos::vision::CameraControls&) override {
    return true;
  }
  void SetFrameCallback(talos::vision::FrameCallback callback) override {
    std::lock_guard lock(device_->mutex);
    device_->callback = std::move(callback);
  }
  void SetConnectionCallback(
      talos::vision::ConnectionCallback callback) override {
    std::lock_guard lock(device_->mutex);
    device_->connection = std::move(callback);
  }
  bool Start() override {
    ++device_->starts;
    running_ = device_->allow_start;
    talos::vision::ConnectionCallback connection;
    std::string failure;
    {
      std::lock_guard lock(device_->mutex);
      connection = device_->connection;
      failure = device_->start_failure;
    }
    if (!running_ && connection && !failure.empty()) {
      connection(talos::vision::ConnectionState::kError, failure);
    }
    return running_;
  }
  void Stop() override {
    ++device_->stops;
    running_ = false;
    std::lock_guard lock(device_->mutex);
    device_->callback = {};
  }
  bool IsRunning() const override {
    device_->WaitGate();
    return running_;
  }
  bool IsConnected() const override { return running_; }
  bool controls_applied() const override { return device_->apply_controls; }

 private:
  std::shared_ptr<FakeCameraDevice> device_;
  bool running_ = false;
};

struct LiveTestVision : Vision {
  inline static std::shared_ptr<FakeCameraDevice> device;
  explicit LiveTestVision(talos::Node& node)
      : Vision(node, [camera = device](const auto& config) {
          return std::make_unique<FakeCameraSource>(camera, config);
        }) {}
};

// The acquisition worker is a real thread, whose wall time (long under load
// or sanitizers) must not become simulated time: stall detection and retry
// backoff run on the node clock it is handed. Step 1 ms at a time, and after
// each step wait in wall time until the worker has serviced that instant and
// every frame delivered to it, so the next step collects its result. The wall
// bound only ends a failing run.
void AdvanceLive(talos::testing::Sim& sim, Vision& camera, int milliseconds) {
  for (int i = 0; i < milliseconds; ++i) {
    sim.step(1ms);
    ASSERT_TRUE(camera.WaitForLiveWorker(60s));
  }
}

// Step until the observable result exists; bounded so a real failure fails.
template <typename Ready>
bool AdvanceLiveUntil(talos::testing::Sim& sim, Vision& camera, Ready ready,
                      int max_milliseconds = 500) {
  for (int i = 0; i < max_milliseconds && !ready(); ++i) {
    sim.step(1ms);
    if (!camera.WaitForLiveWorker(60s)) return false;
  }
  return ready();
}

TEST(VisionTest, LiveControlsReachBackendAndFailuresAppearInStatus) {
  for (bool applied : {true, false}) {
    auto device = std::make_shared<FakeCameraDevice>();
    device->apply_controls = applied;
    LiveTestVision::device = device;
    auto sim = MakeSim(false, true,
                       "device = \"fake\"\nwidth = 160\nheight = 120\n"
                       "exposure_ms = 4.5\ngain = 12\n"
                       "auto_exposure = false\nauto_gain = false\n",
                       "period_us = 1000\n");
    auto& camera = sim.add<LiveTestVision>("rear_right_camera");
    ASSERT_TRUE(AdvanceLiveUntil(sim, camera, [&] {
      const auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
      return device->starts == 1 && status &&
             status->control_failure_count() == (applied ? 0u : 1u);
    }));
    ASSERT_EQ(device->starts, 1);
    {
      std::lock_guard lock(device->mutex);
      EXPECT_DOUBLE_EQ(device->config.exposure_ms, 4.5);
      EXPECT_DOUBLE_EQ(device->config.gain, 12);
      EXPECT_FALSE(device->config.auto_exposure);
      EXPECT_FALSE(device->config.auto_gain);
    }
    const auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
    ASSERT_TRUE(status);
    EXPECT_EQ(status->controls_applied(), applied);
    EXPECT_EQ(status->control_failure_count(), applied ? 0 : 1);
    const auto stamp = sim.now().nanos() - 10000000;
    device->Emit(stamp, talos::vision::TimestampKind::kEndOfFrame);
    ASSERT_TRUE(AdvanceLiveUntil(sim, camera, [&] {
      return sim.latest<geometry::PoseObservation>("/rear_right_camera/state")
          .has_value();
    }));
    const auto observation =
        sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
    ASSERT_TRUE(observation);
    EXPECT_EQ(observation->sample_time_ns(), stamp - (applied ? 2250000 : 0));
  }
}

TEST(VisionTest, SilentLiveCameraReopensWithBackoffAndCountsRecovery) {
  auto device = std::make_shared<FakeCameraDevice>();
  LiveTestVision::device = device;
  auto sim =
      MakeSim(false, true, "device = \"fake\"\nframe_timeout_us = 50000\n",
              "period_us = 1000\n");
  auto& camera = sim.add<LiveTestVision>("rear_right_camera");
  AdvanceLive(sim, camera, 30);
  ASSERT_EQ(device->starts, 1);
  device->allow_start = false;
  AdvanceLive(sim, camera, 50);
  auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
  ASSERT_TRUE(status);
  EXPECT_EQ(status->stall_count(), 1);
  EXPECT_EQ(device->starts, 1);  // Initial 100 ms retry deadline.
  AdvanceLive(sim, camera, 50);
  EXPECT_EQ(device->starts, 2);
  AdvanceLive(sim, camera, 120);
  EXPECT_EQ(device->starts, 2);  // Failed reopen doubled backoff to 200 ms.
  device->allow_start = true;
  AdvanceLive(sim, camera, 90);
  ASSERT_EQ(device->starts, 3);
  status = sim.latest<VisionStatus>("/rear_right_camera/status");
  ASSERT_TRUE(status);
  EXPECT_EQ(status->reconnect_count(), 1);
  EXPECT_EQ(status->stall_count(), 1);
  EXPECT_GE(device->stops, 2);
}

TEST(VisionTest, FlowingLiveFramesDoNotReconnectAndUseSourceTiming) {
  using talos::vision::TimestampKind;
  auto device = std::make_shared<FakeCameraDevice>();
  LiveTestVision::device = device;
  // Frames arrive only when the test emits them; the 10 s stall timeout keeps
  // the gaps between them from reading as a stall. Stall recovery has its own
  // test above.
  auto sim = MakeSim(false, true,
                     "device = \"fake\"\nwidth = 160\nheight = 120\n"
                     "frame_timeout_us = 10000000\nexposure_ms = 4\n"
                     "readout_us = 1000\nlatency_offset_us = 10000\n",
                     "period_us = 1000\n");
  auto& camera = sim.add<LiveTestVision>("rear_right_camera");
  ASSERT_TRUE(
      AdvanceLiveUntil(sim, camera, [&] { return device->starts == 1; }));
  // Each frame is stamped 10 ms before now and corrected by up to 13 ms more.
  // A fast worker starts within a few simulated milliseconds, which made those
  // stamps negative: CaptureMidpoint then has no valid time, the observation
  // carries 0 and the wait below could never match. Start from 30 ms.
  AdvanceLive(sim, camera, 30);
  // No DS, no targets: camera sensing/recovery also runs while disabled and
  // cannot depend on a fresh actuation target.
  std::uint64_t processed = 0;
  for (auto kind : {TimestampKind::kStartOfExposure, TimestampKind::kEndOfFrame,
                    TimestampKind::kDelivery, TimestampKind::kPresentation}) {
    const auto stamp = sim.now().nanos() - 10000000;
    // Half the 4 ms exposure and the 1 ms readout, then the 10 ms latency
    // offset, which applies to every kind.
    const auto correction = kind == TimestampKind::kStartOfExposure ? 8000000
                            : kind == TimestampKind::kEndOfFrame    ? 13000000
                                                                    : 10000000;
    ASSERT_GT(stamp - correction, 0);
    device->Emit(stamp, kind);
    // Wait for this frame's own result: the status that counts it is sent in
    // the same update as its observation. Matching the timestamp alone could
    // accept the previous observation, whose corrected time can coincide
    // (a later frame minus a larger correction can equal an earlier one).
    ASSERT_TRUE(AdvanceLiveUntil(
        sim, camera,
        [&] {
          const auto status =
              sim.latest<VisionStatus>("/rear_right_camera/status");
          return status && status->frames_processed_count() == processed + 1;
        },
        9500))
        << static_cast<int>(kind);
    ++processed;
    const auto observation =
        sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
    ASSERT_TRUE(observation);
    EXPECT_EQ(observation->sample_time_ns(),
              static_cast<std::uint64_t>(stamp - correction))
        << static_cast<int>(kind);
  }
  const auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
  ASSERT_TRUE(status);
  EXPECT_EQ(status->frames_captured_count(), 4);
  EXPECT_EQ(status->frames_processed_count(), 4);
  EXPECT_EQ(status->stall_count(), 0);
  EXPECT_EQ(status->reconnect_count(), 0);
  EXPECT_EQ(device->starts, 1);
}

// The configured camera's noiseless view from `pose`, so a live fake frame
// carries real tags through detection and localization.
std::shared_ptr<const std::vector<std::uint8_t>> RenderCameraView(
    const talos::Node& node, const geometry::Pose2d& pose) {
  namespace fs = talos::vision::field_sim;
  auto number = [&](const char* key) {
    return node.param(std::string("camera.") + key, 0.0);
  };
  fs::SimCameraView view;
  view.intrinsics.fx = number("fx");
  view.intrinsics.fy = number("fy");
  view.intrinsics.cx = number("cx");
  view.intrinsics.cy = number("cy");
  view.intrinsics.image_width =
      static_cast<std::uint32_t>(node.param<int>("camera.width", 1280));
  view.intrinsics.image_height =
      static_cast<std::uint32_t>(node.param<int>("camera.height", 720));
  view.mounting = talos::vision::CameraMounting::FromExtrinsics(
      number("x_m"), number("y_m"), number("z_m"), number("roll_rad"),
      number("pitch_rad"), number("yaw_rad"));
  const double tag_size = node.param("tag_size_m", 0.1651);
  fs::FieldLayoutData layout;
  std::map<int, fs::TagPattern> patterns;
  auto pixels = std::make_shared<std::vector<std::uint8_t>>();
  EXPECT_TRUE(fs::LoadFieldLayoutFromJson(kFieldLayoutJson, tag_size, &layout));
  EXPECT_TRUE(fs::LoadTag36h11Patterns(&patterns));
  EXPECT_GT(fs::RenderFieldView(
                layout.layout, patterns, view,
                fs::RobotPoseOnField(pose.x_m(), pose.y_m(), pose.yaw_rad()),
                tag_size, {}, pixels.get()),
            0);
  return pixels;
}

TEST(VisionTest, LiveStampOlderThanFrameTimeoutPublishesInvalid) {
  auto device = std::make_shared<FakeCameraDevice>();
  LiveTestVision::device = device;
  // The maximum 10 s timeout, which the 10.5 s-old stamps below exceed.
  auto sim =
      MakeSim(false, true, "device = \"fake\"\nframe_timeout_us = 10000000\n",
              "period_us = 1000\n");
  auto& camera = sim.add<LiveTestVision>("rear_right_camera");
  // The pose CheckPose localizes from (blue-wall tags 29/30).
  device->pixels = RenderCameraView(sim.node("rear_right_camera"),
                                    geometry::Pose2d{2, 2, 0});
  // Hold the node, and so the worker's clock, until a stamp older than the
  // timeout exists: the camera opens on its first tick after the pause.
  sim.pause("rear_right_camera", 11s);
  sim.step(11s);
  ASSERT_TRUE(
      AdvanceLiveUntil(sim, camera, [&] { return device->starts == 1; }));
  ::testing::internal::CaptureStderr();
  std::vector<bool> valid;
  std::uint64_t processed = 0;
  for (const auto age : {10500ms, 10500ms, 10ms}) {
    device->Emit(sim.now().nanos() - std::chrono::nanoseconds(age).count(),
                 talos::vision::TimestampKind::kExposure);
    if (!AdvanceLiveUntil(
            sim, camera,
            [&] {
              const auto status =
                  sim.latest<VisionStatus>("/rear_right_camera/status");
              return status &&
                     status->frames_processed_count() == processed + 1;
            },
            9500)) {
      break;
    }
    ++processed;
    const auto observation =
        sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
    valid.push_back(observation && observation->valid());
  }
  const auto diagnostics = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(valid, (std::vector<bool>{false, false, true})) << diagnostics;
  // One diagnostic when stamps turn stale and one when they recover, not one
  // per stale frame.
  EXPECT_EQ(CountLines(diagnostics, "capture stamp"), 2u) << diagnostics;
}

// Detection slower than the camera: frames delivered while the worker is busy
// replace each other, the worker then processes the newest, and every frame it
// never published is counted in frames_dropped_count.
TEST(VisionTest, SlowWorkerProcessesNewestLiveFrameAndCountsDrops) {
  auto device = std::make_shared<FakeCameraDevice>();
  LiveTestVision::device = device;
  auto sim = MakeSim(false, true,
                     "device = \"fake\"\nwidth = 160\nheight = 120\n"
                     "frame_timeout_us = 10000000\n",
                     "period_us = 1000\n");
  auto& camera = sim.add<LiveTestVision>("rear_right_camera");
  // Declared after sim, so a failed assertion releases the worker before the
  // node joins it.
  struct Release {
    FakeCameraDevice& device;
    ~Release() { device.Hold(false); }
  } release{*device};
  ASSERT_TRUE(
      AdvanceLiveUntil(sim, camera, [&] { return device->starts == 1; }));
  AdvanceLive(sim, camera, 30);
  constexpr std::uint64_t kFrames = 8;
  const std::int64_t first = sim.now().nanos() - 20000000;
  device->Hold(true);
  for (std::uint64_t i = 0; i < kFrames; ++i) {
    device->Emit(first + static_cast<std::int64_t>(i) * 1000000,
                 talos::vision::TimestampKind::kExposure);
    // The first frame wakes the worker, which then stays busy.
    if (i == 0) ASSERT_TRUE(device->WaitBlocked());
  }
  device->Hold(false);
  const auto newest =
      static_cast<std::uint64_t>(first) + (kFrames - 1) * 1000000;
  ASSERT_TRUE(AdvanceLiveUntil(sim, camera, [&] {
    const auto observation =
        sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
    const auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
    return observation && observation->sample_time_ns() == newest && status &&
           status->frames_captured_count() == kFrames;
  }));
  const auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
  ASSERT_TRUE(status);
  // At most the frame already in hand when the worker stalled, and the newest,
  // reach a result; the frames in between are counted, not silently lost.
  EXPECT_GE(status->frames_dropped_count(), kFrames - 2);
  EXPECT_LE(status->frames_processed_count(), 2u);
}

// A camera that keeps failing to open logs its reason once, not on every
// retry; reconnecting logs once more. Retry backoff stops growing near 1 s.
TEST(VisionTest, FailingOpenLogsOncePerTransitionAndBackoffIsCapped) {
  auto device = std::make_shared<FakeCameraDevice>();
  device->allow_start = false;
  device->start_failure = "fake camera unplugged";
  LiveTestVision::device = device;
  ::testing::internal::CaptureStderr();
  auto sim = MakeSim(false, true,
                     "device = \"fake\"\nwidth = 160\nheight = 120\n"
                     "frame_timeout_us = 10000000\n",
                     "period_us = 1000\n");
  auto& camera = sim.add<LiveTestVision>("rear_right_camera");
  // Attempts at about 0, 0.1, 0.3, 0.7, 1.5 and 2.5 s: the last gap is the
  // 1 s cap, where doubling would have waited 1.6 s.
  std::vector<std::int64_t> attempts_ns;
  int seen = 0;
  for (int i = 0; i < 3500 && seen < 6; ++i) {
    AdvanceLive(sim, camera, 1);
    if (device->starts > seen) {
      seen = device->starts;
      attempts_ns.push_back(sim.now().nanos());
    }
  }
  device->allow_start = true;
  const bool reopened =
      AdvanceLiveUntil(sim, camera, [&] { return device->starts == 7; }, 1500);
  device->Emit(sim.now().nanos(), talos::vision::TimestampKind::kExposure);
  // Processing follows the connected report, so the log is complete.
  const bool processed = AdvanceLiveUntil(sim, camera, [&] {
    const auto status = sim.latest<VisionStatus>("/rear_right_camera/status");
    return status && status->frames_processed_count() == 1;
  });
  const auto diagnostics = ::testing::internal::GetCapturedStderr();
  ASSERT_EQ(attempts_ns.size(), 6u) << diagnostics;
  ASSERT_TRUE(reopened) << diagnostics;
  ASSERT_TRUE(processed) << diagnostics;
  const auto capped_gap = attempts_ns[5] - attempts_ns[4];
  EXPECT_GE(capped_gap, 900000000) << capped_gap;
  // Below the uncapped doubling (1.6 s).
  EXPECT_LT(capped_gap, 1600000000) << capped_gap;
  EXPECT_EQ(CountLines(diagnostics, "camera Error: fake camera unplugged"), 1u)
      << diagnostics;
  EXPECT_EQ(CountLines(diagnostics, "camera Error"), 1u) << diagnostics;
  EXPECT_EQ(CountLines(diagnostics, "camera Connected"), 1u) << diagnostics;
}
}  // namespace
}  // namespace vision
