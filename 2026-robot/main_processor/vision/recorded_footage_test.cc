// The camera node localizes from recorded footage: a recording played through
// talos::vision::playback reaches the node's live capture path unmodified and
// publishes observations stamped on the recording's own timeline. The
// footage is rendered by the repo's field renderer from a known robot pose.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/node/testing.h"
#include "talOS/vision/field_sim/field_layout_loader.h"
#include "talOS/vision/field_sim/sim_camera.h"
#include "talOS/vision/field_sim/tag_patterns.h"
#include "talOS/vision/field_sim/tag_rasterizer.h"
#include "talOS/vision/playback/recorded_source.h"
#include "vision/data/field_layout_2026.h"
#include "vision/vision.h"

namespace vision {
namespace {
using namespace std::chrono_literals;
using talos::vision::playback::RecordedCameraSource;

// rear_right_camera's calibration, restated so the footage and the node agree
// whatever the robot's config later holds.
constexpr int kWidth = 1280, kHeight = 720;
constexpr double kFx = 910.5, kCx = 640.0, kCy = 360.0;
constexpr double kMountX = -0.25, kMountY = -0.2, kMountZ = 0.45;
constexpr double kMountPitch = 0.2617993877991494;
constexpr double kMountYaw = -2.356194490192345;
constexpr double kTagSize = 0.1651;

std::string Runfile(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> files(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* value = std::getenv(env);
  if (!value) throw std::runtime_error(std::string("missing ") + env);
  if (!files) throw std::runtime_error(error);
  return files->Rlocation(value);
}

// Renders the field as rear_right_camera sees it from each robot pose into
// a recording at `dir`, with the given presentation times.
void WriteRecording(const std::filesystem::path& dir,
                    const std::vector<std::array<double, 3>>& poses,
                    const std::vector<std::int64_t>& pts_ns) {
  namespace fs = talos::vision::field_sim;
  fs::FieldLayoutData layout;
  std::string error;
  ASSERT_TRUE(
      fs::LoadFieldLayoutFromJson(kFieldLayoutJson, kTagSize, &layout, &error))
      << error;
  std::map<int, fs::TagPattern> patterns;
  ASSERT_TRUE(fs::LoadTag36h11Patterns(&patterns, &error)) << error;
  fs::SimCameraView camera;
  camera.intrinsics.fx = camera.intrinsics.fy = kFx;
  camera.intrinsics.cx = kCx;
  camera.intrinsics.cy = kCy;
  camera.intrinsics.image_width = kWidth;
  camera.intrinsics.image_height = kHeight;
  camera.mounting = talos::vision::CameraMounting::FromExtrinsics(
      kMountX, kMountY, kMountZ, 0.0, kMountPitch, kMountYaw);
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  std::ofstream timeline(dir / "timestamps.csv");
  timeline << "file,pts_ns\n";
  for (std::size_t i = 0; i < poses.size(); ++i) {
    std::vector<std::uint8_t> gray;
    std::vector<fs::SimTagDetection> visible;
    ASSERT_GE(fs::RenderInstantaneousFieldView(
                  layout.layout, patterns, camera,
                  fs::RobotPoseOnField(poses[i][0], poses[i][1], poses[i][2]),
                  kTagSize, {}, &gray, &visible),
              2);
    const auto name = std::to_string(i) + ".png";
    ASSERT_TRUE(cv::imwrite((dir / name).string(),
                            cv::Mat(kHeight, kWidth, CV_8UC1, gray.data())));
    timeline << name << ',' << pts_ns[i] << '\n';
  }
}

// The node's source factory, with the test holding the source it made so
// the test thread can deliver its frames at simulated instants.
struct RecordedVision : Vision {
  inline static std::atomic<RecordedCameraSource*> source{nullptr};
  explicit RecordedVision(talos::Node& node)
      : Vision(node, [](const talos::vision::CameraSourceConfig& config) {
          auto made = std::make_unique<RecordedCameraSource>(
              config, RecordedCameraSource::Pacing::kManual);
          source = made.get();
          return made;
        }) {}
};

// No catalogue requirement yet: a DEV row for recorded footage driving the
// camera node is proposed, owner decision pending (TODO.md SCOPE-1).
// Mutation: dropping the frame-period grace before the end-of-recording
// disconnect loses the last frame (the worker discards it), so frame 1 is
// published invalid.
TEST(RecordedFootageTest, CameraNodeLocalizesFromARecordingOnItsTimeline) {
  const auto dir =
      std::filesystem::path(std::getenv("TEST_TMPDIR")) / "recording";
  // The robot drives 0.3 m in +x over two frames 40 ms apart (25 fps).
  const std::vector<std::array<double, 3>> poses{{2.0, 2.0, 0.0},
                                                 {2.3, 2.0, 0.0}};
  // A recording clock far from the node's: playback restamps it.
  WriteRecording(dir, poses, {7'000'000'000'000, 7'000'040'000'000});
  std::ostringstream text;
  text << "include = "
       << std::quoted(
              std::filesystem::absolute(Runfile("TALOS_TEST_CONFIG")).string())
       << "\n[subsystems.rear_right_camera]\nbackend = \"cpu\"\n"
       << "period_us = 1000\n[subsystems.rear_right_camera.camera]\n"
       << "device = \"recording:" << dir.string() << "\"\n"
       << "frame_timeout_us = 10000000\nwidth = " << kWidth
       << "\nheight = " << kHeight << "\nfx = " << kFx << "\nfy = " << kFx
       << "\ncx = " << kCx << "\ncy = " << kCy << "\nx_m = " << kMountX
       << "\ny_m = " << kMountY << "\nz_m = " << kMountZ
       << "\npitch_rad = " << kMountPitch << "\nyaw_rad = " << kMountYaw
       << "\nroll_rad = 0.0\nk1 = 0.0\nk2 = 0.0\nk3 = 0.0\np1 = 0.0\n"
       << "p2 = 0.0\nlatency_offset_us = 0\n";
  auto sim = talos::testing::Sim::FromToml(text.str());
  auto& camera = sim.add<RecordedVision>("rear_right_camera");
  std::vector<geometry::PoseObservation> observations;
  std::int64_t start_ns = -1;
  // Step 1 ms at a time; deliver what the recording has due at each instant
  // and wait (wall time, bounding only a hang) for the worker to process it.
  for (int ms = 0; ms < 200 && observations.size() < poses.size(); ++ms) {
    sim.step(1ms);
    ASSERT_TRUE(camera.WaitForLiveWorker(60s));
    auto* source = RecordedVision::source.load();
    if (source == nullptr || !source->IsConnected()) continue;
    if (start_ns < 0) start_ns = sim.now().nanos();
    if (source->DeliverUntil(sim.now().nanos()) == 0) continue;
    ASSERT_TRUE(camera.WaitForLiveWorker(60s));
    for (int settle = 0; settle < 5; ++settle) {
      sim.step(1ms);
      ASSERT_TRUE(camera.WaitForLiveWorker(60s));
    }
    const auto latest =
        sim.latest<geometry::PoseObservation>("/rear_right_camera/state");
    ASSERT_TRUE(latest);
    observations.push_back(*latest);
  }
  ASSERT_EQ(observations.size(), poses.size());
  for (std::size_t i = 0; i < poses.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(observations[i].sample_time_ns(),
              static_cast<std::uint64_t>(start_ns + 40'000'000 * i));
    ASSERT_TRUE(observations[i].valid());
    EXPECT_GE(observations[i].tag_count(), 2u);
    EXPECT_NEAR(observations[i].pose().x_m(), poses[i][0], 0.1);
    EXPECT_NEAR(observations[i].pose().y_m(), poses[i][1], 0.1);
    EXPECT_NEAR(observations[i].pose().yaw_rad(), poses[i][2], 0.05);
  }
}

}  // namespace
}  // namespace vision
