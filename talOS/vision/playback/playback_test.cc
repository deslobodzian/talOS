// A recording (image files plus their timeline) plays through the
// CameraSource interface into the unmodified AprilTag pipeline, stamped on the
// recording's own timeline. Frames are rendered by the repo's field-sim
// renderer and written to TEST_TMPDIR; no third-party footage is checked in.

#include <gtest/gtest.h>

#include <cfenv>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "talOS/process/process.h"
#include "talOS/vision/field_sim/sim_camera.h"
#include "talOS/vision/field_sim/tag_patterns.h"
#include "talOS/vision/field_sim/tag_rasterizer.h"
#include "talOS/vision/pipeline.h"
#include "talOS/vision/playback/image_sequence.h"
#include "talOS/vision/playback/recorded_source.h"

namespace talos::vision::playback {
namespace {

constexpr double kTagSize = 0.1651;
constexpr int kTagId = 7;
constexpr int kWidth = 640;
constexpr int kHeight = 480;

FieldLayout MakeLayout() {
  FieldLayout layout(kTagSize);
  Eigen::Matrix3d facing_robot;
  facing_robot << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
  layout.SetTagPose(kTagId, talos::geometry::SE3<double>(
                                talos::geometry::SO3<double>(facing_robot),
                                Eigen::Vector3d(3.5, 2.0, 0.5)));
  return layout;
}

field_sim::SimCameraView MakeCamera() {
  field_sim::SimCameraView camera;
  camera.intrinsics.fx = 500.0;
  camera.intrinsics.fy = 500.0;
  camera.intrinsics.cx = 320.0;
  camera.intrinsics.cy = 240.0;
  camera.intrinsics.image_width = kWidth;
  camera.intrinsics.image_height = kHeight;
  camera.mounting = field_sim::MountFromXyzYpr(0.0, 0.0, 0.5, 0.0, 0.0, 0.0);
  return camera;
}

// A fresh recording directory under TEST_TMPDIR.
std::filesystem::path NewDirectory(const std::string& name) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  const auto dir = std::filesystem::path(tmp ? tmp : "/tmp") / name;
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

// Writes `file` in `dir`: the tag as seen from robot x = `robot_x_m`.
void RenderFrame(const std::filesystem::path& dir, const std::string& file,
                 double robot_x_m) {
  std::map<int, field_sim::TagPattern> patterns;
  std::string error;
  ASSERT_TRUE(field_sim::LoadTag36h11Patterns(&patterns, &error)) << error;
  std::vector<std::uint8_t> gray;
  std::vector<field_sim::SimTagDetection> visible;
  ASSERT_EQ(field_sim::RenderInstantaneousFieldView(
                MakeLayout(), patterns, MakeCamera(),
                field_sim::RobotPoseOnField(robot_x_m, 2.0, 0.0), kTagSize, {},
                &gray, &visible),
            1);
  const cv::Mat image(kHeight, kWidth, CV_8UC1, gray.data());
  ASSERT_TRUE(cv::imwrite((dir / file).string(), image));
}

void WriteTimeline(const std::filesystem::path& dir, const std::string& text) {
  std::ofstream(dir / kTimestampsFile) << text;
}

TEST(ImageSequenceTest, TimelineComesFromTimestampsFileElseFromFps) {
  const auto dir = NewDirectory("timeline");
  RenderFrame(dir, "b.png", 1.0);
  RenderFrame(dir, "a.png", 2.0);
  std::string error;
  {
    SCOPED_TRACE("fps, file-name order");
    const auto sequence = ImageSequence::Open(dir, 20.0, &error);
    ASSERT_TRUE(sequence) << error;
    ASSERT_EQ(sequence->size(), 2u);
    EXPECT_EQ((*sequence)[0].path.filename(), "a.png");
    EXPECT_EQ((*sequence)[1].pts_ns, 50'000'000);
    EXPECT_FALSE(ImageSequence::Open(dir, 0.0, &error));
  }
  {
    SCOPED_TRACE("timestamps.csv: its order, its clock, rebased to 0");
    WriteTimeline(dir,
                  "file,pts_ns\nb.png,1520527958463478167\n"
                  "a.png,1520527958530000000\n");
    const auto sequence = ImageSequence::Open(dir, 0.0, &error);
    ASSERT_TRUE(sequence) << error;
    ASSERT_EQ(sequence->size(), 2u);
    EXPECT_EQ((*sequence)[0].path.filename(), "b.png");
    EXPECT_EQ((*sequence)[0].pts_ns, 0);
    EXPECT_EQ((*sequence)[1].pts_ns, 66'521'833);
    EXPECT_EQ(sequence->origin_ns(), 1520527958463478167);
  }
}

TEST(ImageSequenceTest, RejectsATimelineThatGoesBackOrLeavesTheDirectory) {
  const auto dir = NewDirectory("bad_timeline");
  RenderFrame(dir, "a.png", 2.0);
  RenderFrame(dir, "b.png", 2.0);
  for (const auto& [name, text] :
       std::vector<std::pair<std::string, std::string>>{
           {"not increasing", "a.png,10\nb.png,10\n"},
           {"path out of the directory", "a.png,0\n../b.png,10\n"},
           {"missing file", "a.png,0\nc.png,10\n"}}) {
    SCOPED_TRACE(name);
    WriteTimeline(dir, text);
    std::string error;
    EXPECT_FALSE(ImageSequence::Open(dir, 30.0, &error));
    EXPECT_NE(error.find(kTimestampsFile), std::string::npos) << error;
  }
}

// The frames reach a consumer in order, at their recorded times (offset by
// the instant playback starts), and the unmodified pipeline decodes them.
TEST(RecordedCameraSourceTest,
     ManualPlaybackDeliversEachFrameAtItsRecordedTime) {
  const auto dir = NewDirectory("manual");
  RenderFrame(dir, "0.png", 1.5);
  RenderFrame(dir, "1.png", 2.0);
  RenderFrame(dir, "2.png", 2.5);
  // Uneven gaps: 30 ms, then 45 ms.
  WriteTimeline(dir, "0.png,1000000000\n1.png,1030000000\n2.png,1075000000\n");
  CameraSourceConfig config;
  config.camera_id = 3;
  config.device_path = std::string(kRecordingDevicePrefix) + dir.string();
  config.width = kWidth;
  config.height = kHeight;
  RecordedCameraSource source(config, RecordedCameraSource::Pacing::kManual);

  PipelineConfig pipeline_config;
  pipeline_config.intrinsics = MakeCamera().intrinsics;
  pipeline_config.mounting = MakeCamera().mounting;
  pipeline_config.layout = MakeLayout();
  AprilTagPipeline pipeline(pipeline_config);
  struct Delivered {
    std::int64_t stamp_ns;
    double range_m;
  };
  std::vector<Delivered> delivered;
  source.SetFrameCallback([&](const Frame& frame) {
    EXPECT_EQ(frame.timestamp_kind(), TimestampKind::kPresentation);
    const auto result = pipeline.ProcessFrame(frame);
    ASSERT_EQ(result.detections.size(), 1u);
    ASSERT_EQ(result.detections[0].tag_id, kTagId);
    ASSERT_TRUE(result.detections[0].valid);
    delivered.push_back({frame.capture_timestamp_ns(),
                         result.detections[0].T_camera_tag.translation().z()});
  });
  ASSERT_TRUE(source.Start());

  constexpr std::int64_t kStart = 5'000'000'000;
  EXPECT_EQ(source.DeliverUntil(kStart), 1u);
  EXPECT_EQ(source.DeliverUntil(kStart + 29'999'999), 0u);
  EXPECT_EQ(source.DeliverUntil(kStart + 30'000'000), 1u);
  EXPECT_EQ(source.DeliverUntil(kStart + 75'000'000), 1u);
  ASSERT_EQ(delivered.size(), 3u);
  EXPECT_EQ(delivered[0].stamp_ns, kStart);
  EXPECT_EQ(delivered[1].stamp_ns, kStart + 30'000'000);
  EXPECT_EQ(delivered[2].stamp_ns, kStart + 75'000'000);
  // Recorded order: the robot approached the tag, 2.0 m, 1.5 m, 1.0 m away.
  EXPECT_NEAR(delivered[0].range_m, 2.0, 0.05);
  EXPECT_NEAR(delivered[2].range_m, 1.0, 0.05);
  // The camera goes away one mean frame period (37.5 ms) after the last
  // frame, not with it: a consumer that sees the disconnect discards the
  // frame it has not processed yet, so the last frame would be lost.
  EXPECT_TRUE(source.IsConnected());
  EXPECT_EQ(source.DeliverUntil(kStart + 112'499'999), 0u);
  EXPECT_TRUE(source.IsConnected());
  EXPECT_EQ(source.DeliverUntil(kStart + 112'500'000), 0u);
  EXPECT_FALSE(source.IsConnected());
}

TEST(RecordedCameraSourceTest, RecordingOfAnotherSizeFailsStartWithItsSize) {
  const auto dir = NewDirectory("size");
  RenderFrame(dir, "0.png", 2.0);
  CameraSourceConfig config;
  config.device_path = dir.string();
  config.width = 1280;
  config.height = 720;
  config.fps = 30;
  RecordedCameraSource source(config, RecordedCameraSource::Pacing::kManual);
  std::vector<std::pair<ConnectionState, std::string>> reports;
  source.SetConnectionCallback(
      [&](ConnectionState state, std::string_view reason) {
        reports.emplace_back(state, std::string(reason));
      });
  EXPECT_FALSE(source.Start());
  EXPECT_FALSE(source.IsConnected());
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_EQ(reports[0].first, ConnectionState::kError);
  EXPECT_NE(reports[0].second.find("640x480"), std::string::npos)
      << reports[0].second;
}

TEST(ImageSequenceTest, RejectsNegativeAndOverflowingTimestamps) {
  const auto dir = NewDirectory("overflow_timeline");
  RenderFrame(dir, "a.png", 2.0);
  RenderFrame(dir, "b.png", 2.0);
  for (const auto& [name, text] :
       std::vector<std::pair<std::string, std::string>>{
           {"negative pts", "a.png,-1\nb.png,10\n"},
           {"int64_min pts", "a.png,-9223372036854775808\nb.png,10\n"},
           {"overflowing pts on first line",
            "a.png,99999999999999999999999\nb.png,10\n"},
           {"overflowing pts on second line",
            "a.png,0\nb.png,99999999999999999999999\n"}}) {
    SCOPED_TRACE(name);
    WriteTimeline(dir, text);
    std::string error;
    EXPECT_FALSE(ImageSequence::Open(dir, 30.0, &error));
    EXPECT_FALSE(error.empty());
  }
  std::filesystem::remove(dir / kTimestampsFile);
  std::string error;
  std::feclearexcept(FE_ALL_EXCEPT);
  EXPECT_FALSE(ImageSequence::Open(dir, 1e-12, &error));
  EXPECT_EQ(std::fetestexcept(FE_INVALID), 0);
  EXPECT_FALSE(error.empty());
}

TEST(RecordedCameraSourceTest, FrameCallbackCanMutateCallbacksWithoutDeadlock) {
  const auto dir = NewDirectory("reentrant_cb");
  RenderFrame(dir, "0.png", 1.5);
  RenderFrame(dir, "1.png", 2.0);
  WriteTimeline(dir, "0.png,0\n1.png,30000000\n");
  CameraSourceConfig config;
  config.device_path = dir.string();
  config.width = kWidth;
  config.height = kHeight;
  RecordedCameraSource source(config, RecordedCameraSource::Pacing::kManual);

  int first_calls = 0;
  int second_calls = 0;
  source.SetFrameCallback([&](const Frame&) {
    ++first_calls;
    source.SetConnectionCallback([](ConnectionState, std::string_view) {});
    source.SetFrameCallback([&](const Frame&) { ++second_calls; });
  });
  ASSERT_TRUE(source.Start());
  EXPECT_EQ(source.DeliverUntil(1'000'000'000), 1u);
  EXPECT_EQ(source.DeliverUntil(1'030'000'000), 1u);
  EXPECT_EQ(first_calls, 1);
  EXPECT_EQ(second_calls, 1);
}

// REQ-RT-10: the recorded camera source's realtime playback thread runs at
// background priority below every loop thread.
TEST(RecordedCameraSourceTest, RealtimePlaybackThreadRunsAtBackgroundPriority) {
  const auto dir = NewDirectory("rt_bg");
  RenderFrame(dir, "0.png", 2.0);
  WriteTimeline(dir, "0.png,0\n");
  CameraSourceConfig config;
  config.device_path = dir.string();
  config.width = kWidth;
  config.height = kHeight;
  config.fps = 30.0;
  RecordedCameraSource source(config, RecordedCameraSource::Pacing::kRealtime);

  std::mutex mu;
  std::condition_variable cv;
  bool delivered = false;
  bool is_background = false;
  source.SetFrameCallback([&](const Frame&) {
    const bool bg = talos::process::ThreadIsBackground();
    {
      std::lock_guard lock(mu);
      is_background = bg;
      delivered = true;
    }
    cv.notify_one();
  });
  ASSERT_TRUE(source.Start());
  {
    std::unique_lock lock(mu);
    // 10 s hang bound (docs/TESTING.md §6.3).
    ASSERT_TRUE(
        cv.wait_for(lock, std::chrono::seconds{10}, [&] { return delivered; }));
  }
  source.Stop();
  EXPECT_TRUE(is_background);
}

}  // namespace
}  // namespace talos::vision::playback
