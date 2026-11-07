#include "talOS/vision/recording/recorder.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "talOS/vision/frame.h"
#include "talOS/vision/recording/recording.h"

namespace talos::vision::recording {
namespace {

using talos::event::MonotonicTime;

CameraContext MakeContext() {
  CameraContext ctx;
  ctx.camera = "test_camera";
  ctx.width = 4;
  ctx.height = 2;
  ctx.intrinsics = {.fx = 500.0,
                    .fy = 501.0,
                    .cx = 320.0,
                    .cy = 240.0,
                    .k1 = 0.1,
                    .k2 = 0.2,
                    .k3 = 0.0,
                    .p1 = 0.01,
                    .p2 = 0.02};
  ctx.mounting = {.x_m = 1.0,
                  .y_m = 2.0,
                  .z_m = 3.0,
                  .roll_rad = 0.1,
                  .pitch_rad = 0.2,
                  .yaw_rad = 0.3};
  ctx.detector = {.backend = "cpu",
                  .decimate = 2.0,
                  .threads = 2,
                  .sigma = 0.0,
                  .refine_edges = true,
                  .hamming_threshold = 1,
                  .min_decision_margin = 30.0};
  return ctx;
}

FrameRecord MakeRecord(std::uint64_t capture_ns, std::string_view reason) {
  FrameRecord r;
  r.capture_ns = capture_ns;
  r.reason = std::string{reason};
  DetectionRecord d;
  d.tag_id = 7;
  d.hamming = 0;
  d.decision_margin = 55.0;
  d.corners_px = {0, 0, 1, 0, 1, 1, 0, 1};
  d.valid = true;
  d.reprojection_error_px = 0.25;
  d.ambiguity = 0.1;
  r.detections.push_back(d);
  r.observation = {.valid = true,
                   .x_m = 1.0,
                   .y_m = 2.0,
                   .yaw_rad = 0.1,
                   .x_std_m = 0.02,
                   .y_std_m = 0.02,
                   .yaw_std_rad = 0.03,
                   .tag_count = 1};
  r.truth = TruthRecord{1.0, 2.0, 0.1};
  return r;
}

Frame MakeGrayFrame() {
  std::vector<std::uint8_t> data(8, 128);
  return Frame{0,
               0,
               0,
               MonotonicTime::from_nanos(0),
               4,
               2,
               4,
               PixelFormat::kGray8,
               std::move(data)};
}

class RecorderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = std::filesystem::path{::testing::TempDir()} /
           ("talos_recorder_" + std::to_string(::getpid()));
    std::filesystem::remove_all(dir_);
  }
  void TearDown() override { std::filesystem::remove_all(dir_); }
  std::filesystem::path dir_;
};

TEST_F(RecorderTest, RejectsMismatchedSizeAndNullDataWithoutWriting) {
  FrameRecorder::Options opts;
  opts.output_dir = dir_;
  opts.context = MakeContext();
  FrameRecorder rec{std::move(opts)};
  const Frame wrong_size{0, 0, 0, MonotonicTime::from_nanos(0), 8, 4};
  const Frame null_data{0,
                        0,
                        0,
                        MonotonicTime::from_nanos(0),
                        4,
                        2,
                        4,
                        PixelFormat::kGray8,
                        std::shared_ptr<const std::vector<std::uint8_t>>{}};
  ASSERT_EQ(null_data.data(), nullptr);
  EXPECT_FALSE(rec.Record(wrong_size, MakeRecord(0, "sampled")));
  EXPECT_FALSE(rec.Record(null_data, MakeRecord(0, "sampled")));
  EXPECT_EQ(rec.dropped(), 2u);
  EXPECT_EQ(rec.recorded(), 0u);
  EXPECT_TRUE(std::filesystem::is_empty(dir_));
}

TEST_F(RecorderTest, ConstructorReportsUncreatableDirectory) {
  std::filesystem::create_directories(dir_);
  const auto blocker = dir_ / "file";
  std::ofstream{blocker} << "not a directory";
  FrameRecorder::Options opts;
  opts.output_dir = blocker / "frames";
  opts.context = MakeContext();
  EXPECT_THROW(FrameRecorder{std::move(opts)},
               std::filesystem::filesystem_error);
}

TEST_F(RecorderTest, WriterFailureDropsPairsAndStopsRecording) {
  FrameRecorder::Options opts;
  opts.output_dir = dir_ / "frames";
  opts.context = MakeContext();
  opts.queue_slots = 4;
  FrameRecorder rec{std::move(opts)};
  // Replace the output directory after construction with a regular file:
  // creates fail deterministically, including when the test runs as root.
  std::filesystem::remove(dir_ / "frames");
  std::ofstream{dir_ / "frames"} << "not a directory";
  const Frame frame = MakeGrayFrame();
  EXPECT_TRUE(rec.Record(frame, MakeRecord(0, "sampled")));
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds{5};
  while (rec.dropped() < rec.recorded() &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  EXPECT_EQ(rec.dropped(), rec.recorded());
  const auto dropped = rec.dropped();
  // Same timestamp would otherwise be rate-limited: a stopped recorder must
  // still count it as dropped.
  EXPECT_FALSE(rec.Record(frame, MakeRecord(0, "sampled")));
  EXPECT_EQ(rec.dropped(), dropped + 1);
}

TEST_F(RecorderTest, RecordStemZeroPadsToTwentyDigits) {
  EXPECT_EQ(RecordStem(7, "sampled"), "00000000000000000007_sampled");
  EXPECT_EQ(RecordStem(1600000000000000000ULL, "rejected"),
            "01600000000000000000_rejected");
}

TEST_F(RecorderTest, SerializeRecordEmitsTheStudioContract) {
  const CameraContext ctx = MakeContext();
  FrameRecord r = MakeRecord(1234567890, "sampled");
  // One invalid detection, with a reason, plus the valid one.
  DetectionRecord bad;
  bad.tag_id = 8;
  bad.hamming = 2;
  bad.decision_margin = 5.0;
  bad.valid = false;
  bad.reject_reason = "hamming";
  r.detections.push_back(bad);

  const std::string json = SerializeRecord(ctx, r);
  // capture_ns is a bare integer literal, never quoted and never a float.
  EXPECT_NE(json.find("\"capture_ns\":1234567890"), std::string::npos);
  EXPECT_EQ(json.find("\"capture_ns\":\"1234567890\""), std::string::npos);
  EXPECT_EQ(json.find("\"capture_ns\":1234567890.0"), std::string::npos);
  // Integer fields are integer literals.
  EXPECT_NE(json.find("\"format_version\":1"), std::string::npos);
  EXPECT_NE(json.find("\"threads\":2"), std::string::npos);
  EXPECT_NE(json.find("\"hamming_threshold\":1"), std::string::npos);
  EXPECT_NE(json.find("\"tag_id\":7"), std::string::npos);
  EXPECT_NE(json.find("\"width\":4"), std::string::npos);
  EXPECT_NE(json.find("\"height\":2"), std::string::npos);
  // Every detection carries a reject_reason: null when valid, a string
  // otherwise.
  EXPECT_NE(json.find("\"reject_reason\":null"), std::string::npos);
  EXPECT_NE(json.find("\"reject_reason\":\"hamming\""), std::string::npos);
  // Truth is an object here, not null.
  EXPECT_NE(json.find("\"truth\":{\"x_m\":1"), std::string::npos);

  // Parses back with all contract fields intact.
  CameraContext parsed_ctx;
  FrameRecord parsed;
  std::string error;
  ASSERT_TRUE(ParseRecord(json, &parsed_ctx, &parsed, &error)) << error;
  EXPECT_EQ(parsed_ctx.camera, "test_camera");
  EXPECT_EQ(parsed_ctx.width, 4);
  EXPECT_EQ(parsed_ctx.height, 2);
  EXPECT_DOUBLE_EQ(parsed_ctx.intrinsics.fx, 500.0);
  EXPECT_DOUBLE_EQ(parsed_ctx.mounting.yaw_rad, 0.3);
  EXPECT_EQ(parsed_ctx.detector.threads, 2);
  EXPECT_DOUBLE_EQ(parsed_ctx.detector.decimate, 2.0);
  EXPECT_EQ(parsed.capture_ns, 1234567890u);
  EXPECT_EQ(parsed.reason, "sampled");
  ASSERT_EQ(parsed.detections.size(), 2u);
  EXPECT_TRUE(parsed.detections[0].valid);
  EXPECT_FALSE(parsed.detections[0].reject_reason.has_value());
  EXPECT_FALSE(parsed.detections[1].valid);
  ASSERT_TRUE(parsed.detections[1].reject_reason.has_value());
  EXPECT_EQ(*parsed.detections[1].reject_reason, "hamming");
  EXPECT_TRUE(parsed.observation.valid);
  EXPECT_EQ(parsed.observation.tag_count, 1);
  ASSERT_TRUE(parsed.truth.has_value());
  EXPECT_DOUBLE_EQ(parsed.truth->x_m, 1.0);
}

TEST_F(RecorderTest, TruthSerializesAsNullWhenAbsent) {
  const CameraContext ctx = MakeContext();
  FrameRecord r = MakeRecord(1, "sampled");
  r.truth.reset();
  const std::string json = SerializeRecord(ctx, r);
  EXPECT_NE(json.find("\"truth\":null"), std::string::npos);
}

TEST_F(RecorderTest, RateLimitsSampledAndRejectedIndependently) {
  FrameRecorder::Options opts;
  opts.output_dir = dir_;
  opts.context = MakeContext();
  opts.sample_hz = 1.0;  // 1 Hz = 1 s interval
  opts.rejected_hz = 1.0;
  opts.queue_slots = 8;
  FrameRecorder rec{std::move(opts)};

  const Frame frame = MakeGrayFrame();
  // Sampled at 0 s and 1 s pass; at 0.5 s it is skipped.
  EXPECT_TRUE(rec.Record(frame, MakeRecord(0, "sampled")));
  EXPECT_FALSE(rec.Record(frame, MakeRecord(500'000'000, "sampled")));
  EXPECT_TRUE(rec.Record(frame, MakeRecord(1'000'000'000, "sampled")));
  // Rejected has its own budget: the first one always passes.
  EXPECT_TRUE(rec.Record(frame, MakeRecord(0, "rejected")));
  EXPECT_FALSE(rec.Record(frame, MakeRecord(400'000'000, "rejected")));
  EXPECT_TRUE(rec.Record(frame, MakeRecord(1'000'000'000, "rejected")));

  EXPECT_EQ(rec.recorded(), 4u);
  EXPECT_EQ(rec.dropped(), 0u);
}

TEST_F(RecorderTest, ZeroRatesRecordNothing) {
  FrameRecorder::Options opts;
  opts.output_dir = dir_;
  opts.context = MakeContext();
  opts.sample_hz = 0.0;
  opts.rejected_hz = 0.0;
  FrameRecorder rec{std::move(opts)};

  const Frame frame = MakeGrayFrame();
  EXPECT_FALSE(rec.Record(frame, MakeRecord(0, "sampled")));
  EXPECT_FALSE(rec.Record(frame, MakeRecord(0, "rejected")));
  EXPECT_EQ(rec.recorded(), 0u);
}

TEST_F(RecorderTest, QueueFullDropsAreCountedWithoutBlocking) {
  FrameRecorder::Options opts;
  opts.output_dir = dir_;
  opts.context = MakeContext();
  opts.sample_hz = 1e12;  // effectively no rate limiting
  opts.rejected_hz = 1e12;
  opts.queue_slots = 2;
  FrameRecorder rec{std::move(opts)};

  const Frame frame = MakeGrayFrame();
  constexpr int kFrames = 200;
  for (int i = 0; i < kFrames; ++i) {
    rec.Record(frame, MakeRecord(static_cast<std::uint64_t>(i), "sampled"));
  }
  const std::uint64_t recorded = rec.recorded();
  const std::uint64_t dropped = rec.dropped();
  EXPECT_EQ(recorded + dropped, static_cast<std::uint64_t>(kFrames));
  EXPECT_GT(dropped, 0u);
}

TEST_F(RecorderTest, WritesPairsAtomicallyAndDeletesOldestUnderMaxBytes) {
  FrameRecorder::Options opts;
  opts.output_dir = dir_;
  opts.context = MakeContext();
  opts.sample_hz = 1e12;
  opts.rejected_hz = 1e12;
  opts.queue_slots = 16;  // enough that none are dropped by the queue
  opts.max_bytes = 2048;  // small: only the newest pairs fit
  {
    FrameRecorder rec{std::move(opts)};
    const Frame frame = MakeGrayFrame();
    for (int i = 0; i < 6; ++i) {
      rec.Record(frame, MakeRecord(static_cast<std::uint64_t>(i), "sampled"));
    }
    // Destruction drops queued pairs; wait for the newest JSON (committed
    // after its PNG), rather than assuming a fixed disk/encoder speed.
    const auto newest = dir_ / "00000000000000000005_sampled.json";
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds{5};
    while (!std::filesystem::exists(newest) &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    ASSERT_TRUE(std::filesystem::exists(newest));
  }  // destructor

  std::uint64_t total = 0;
  std::size_t json_count = 0;
  for (const auto& entry : std::filesystem::directory_iterator(dir_)) {
    const std::string name = entry.path().filename().string();
    if (name.size() >= 5 && name.substr(name.size() - 5) == ".json") {
      ++json_count;
      const std::string stem = name.substr(0, name.size() - 5);
      EXPECT_TRUE(std::filesystem::exists(dir_ / (stem + ".png")))
          << "JSON without its PNG: " << name;
      total += std::filesystem::file_size(entry.path());
      total += std::filesystem::file_size(dir_ / (stem + ".png"));
    }
    // Atomic writes leave no temporary files behind.
    EXPECT_EQ(name.find(".tmp"), std::string::npos);
  }
  EXPECT_GT(json_count, 0u);
  EXPECT_LE(total, static_cast<std::uint64_t>(2048));
  // Oldest pairs are deleted; the newest survives.
  EXPECT_FALSE(
      std::filesystem::exists(dir_ / "00000000000000000000_sampled.json"));
  EXPECT_TRUE(
      std::filesystem::exists(dir_ / "00000000000000000005_sampled.json"));
}

// The recorder's destructor must be bounded: on stop it finishes at most the
// pair in progress and drops the rest, so a slow disk never makes shutdown
// block while draining a full queue. A full queue plus a 100 ms per-pair
// writer would otherwise take ~800 ms; the fixed destructor finishes one pair.
TEST_F(RecorderTest, DestructorIsBoundedWithAFullQueueAndSlowWriter) {
  FrameRecorder::Options opts;
  opts.output_dir = dir_;
  opts.context = MakeContext();
  opts.sample_hz = 1e12;
  opts.rejected_hz = 1e12;
  opts.queue_slots = 8;
  opts.test_write_delay = std::chrono::milliseconds{100};

  const Frame frame = MakeGrayFrame();
  std::chrono::steady_clock::time_point before;
  {
    FrameRecorder rec{std::move(opts)};
    for (int i = 0; i < 64; ++i) {
      rec.Record(frame, MakeRecord(static_cast<std::uint64_t>(i), "sampled"));
    }
    before = std::chrono::steady_clock::now();
  }  // destructor: finish at most one 100 ms pair, drop the rest
  const auto elapsed = std::chrono::steady_clock::now() - before;
  EXPECT_LT(elapsed, std::chrono::milliseconds{500})
      << "destructor took " << elapsed.count() / 1e6 << " ms";
}

}  // namespace
}  // namespace talos::vision::recording
