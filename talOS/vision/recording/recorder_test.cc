#include "talOS/vision/recording/recorder.h"

#include <gtest/gtest.h>
#if defined(__linux__)
#include <poll.h>
#include <sched.h>
#include <sys/inotify.h>
#include <sys/resource.h>
#include <unistd.h>
#endif

#include <atomic>
#include <cerrno>
#include <cfenv>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "talOS/process/process.h"
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
                    .p1 = 0.01,
                    .p2 = 0.02,
                    .k3 = 0.0};
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
      std::chrono::steady_clock::now() + std::chrono::seconds{10};
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
  const auto& reject_reason = parsed.detections[1].reject_reason;
  if (!reject_reason.has_value()) {
    FAIL() << "missing rejection reason";
  }
  EXPECT_EQ(*reject_reason, "hamming");
  EXPECT_TRUE(parsed.observation.valid);
  EXPECT_EQ(parsed.observation.tag_count, 1);
  if (!parsed.truth.has_value()) {
    FAIL() << "missing truth";
  }
  EXPECT_DOUBLE_EQ(parsed.truth->x_m, 1.0);

  // Absent truth is an explicit null, not a missing key.
  r.truth.reset();
  EXPECT_NE(SerializeRecord(ctx, r).find("\"truth\":null"), std::string::npos);
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

TEST_F(RecorderTest, QueueOverflowAccountsForEverySubmission) {
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
        std::chrono::steady_clock::now() + std::chrono::seconds{10};
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
// block while draining a full queue. The writer's first pair is held until
// the destructor has asked it to stop, so that pair is by construction the
// one in progress at stop and the queue is still full behind it. Counting
// the pairs the writer starts holds in every thread schedule, where a
// wall-clock bound on the destructor cannot tell a drained queue from one
// slow write.
TEST_F(RecorderTest, DestructorFinishesOnlyThePairInProgress) {
  FrameRecorder::Options opts;
  opts.output_dir = dir_;
  opts.context = MakeContext();
  opts.sample_hz = 1e12;
  opts.rejected_hz = 1e12;
  opts.queue_slots = 8;
  std::atomic<int> pairs_started{0};
  opts.test_before_pair =
      [&pairs_started](const std::function<bool()>& stop_requested) {
        if (pairs_started.fetch_add(1) == 0) {
          while (!stop_requested()) {
            std::this_thread::yield();
          }
        }
      };

  const Frame frame = MakeGrayFrame();
  {
    FrameRecorder rec{std::move(opts)};
    for (int i = 0; i < 64; ++i) {
      rec.Record(frame, MakeRecord(static_cast<std::uint64_t>(i), "sampled"));
    }
  }  // destructor: finish the pair in progress, drop the full queue behind it

  EXPECT_EQ(pairs_started.load(), 1)
      << "the writer started queued pairs after stop";
  std::size_t json_count = 0;
  std::size_t png_count = 0;
  for (const auto& entry : std::filesystem::directory_iterator(dir_)) {
    const std::string ext = entry.path().extension().string();
    json_count += ext == ".json" ? 1 : 0;
    png_count += ext == ".png" ? 1 : 0;
  }
  EXPECT_EQ(json_count, 1u);
  EXPECT_EQ(png_count, 1u);
}

TEST_F(RecorderTest, BackwardTimestampDoesNotUnderflowRateLimit) {
  FrameRecorder::Options opts;
  opts.output_dir = dir_;
  opts.context = MakeContext();
  opts.sample_hz = 1.0;  // 1 s interval
  opts.rejected_hz = 1.0;
  opts.queue_slots = 8;
  FrameRecorder rec{std::move(opts)};

  const Frame frame = MakeGrayFrame();
  EXPECT_TRUE(rec.Record(frame, MakeRecord(1'000'000'000, "sampled")));
  // Out-of-order timestamp before the last accepted frame must not underflow
  // uint64_t subtraction and bypass the rate limit.
  EXPECT_FALSE(rec.Record(frame, MakeRecord(500'000'000, "sampled")));
  EXPECT_TRUE(rec.Record(frame, MakeRecord(2'000'000'000, "sampled")));
  EXPECT_EQ(rec.recorded(), 2u);
  EXPECT_EQ(rec.dropped(), 0u);
}

TEST_F(RecorderTest, RejectsFramesFailingHoldsImageCheck) {
  FrameRecorder::Options opts;
  opts.output_dir = dir_;
  opts.context = MakeContext();
  opts.sample_hz = 1e12;
  opts.queue_slots = 8;
  FrameRecorder rec{std::move(opts)};

  // 4x2 RGB8 needs stride >= 12; stride == 4 fails holds_image().
  const Frame bad_stride_rgb{0,
                             0,
                             0,
                             MonotonicTime::from_nanos(0),
                             4,
                             2,
                             4,
                             PixelFormat::kRgb8,
                             std::vector<std::uint8_t>(24, 100)};
  ASSERT_FALSE(bad_stride_rgb.holds_image());

  // 4x2 Gray8 with stride == 4 needs 8 bytes; 4 bytes fails holds_image().
  const Frame short_gray{0,
                         0,
                         0,
                         MonotonicTime::from_nanos(0),
                         4,
                         2,
                         4,
                         PixelFormat::kGray8,
                         std::vector<std::uint8_t>(4, 100)};
  ASSERT_FALSE(short_gray.holds_image());

  EXPECT_FALSE(rec.Record(bad_stride_rgb, MakeRecord(0, "sampled")));
  EXPECT_FALSE(rec.Record(short_gray, MakeRecord(1, "sampled")));
  EXPECT_EQ(rec.recorded(), 0u);
  EXPECT_EQ(rec.dropped(), 2u);

  // Valid RGB8 frame with proper stride still succeeds.
  const Frame valid_rgb{0,
                        0,
                        0,
                        MonotonicTime::from_nanos(0),
                        4,
                        2,
                        12,
                        PixelFormat::kRgb8,
                        std::vector<std::uint8_t>(24, 100)};
  ASSERT_TRUE(valid_rgb.holds_image());
  EXPECT_TRUE(rec.Record(valid_rgb, MakeRecord(2, "sampled")));
  EXPECT_EQ(rec.recorded(), 1u);
}

#if defined(__linux__)
// REQ-RT-10: the frame recorder's writer thread runs at background priority
// (non-realtime policy, nice >= kBackgroundNice) below every node loop.
TEST_F(RecorderTest, WriterThreadRunsAtBackgroundPriority) {
  const auto list_tids = [] {
    std::set<int> tids;
    for (const auto& entry :
         std::filesystem::directory_iterator("/proc/self/task")) {
      tids.insert(std::stoi(entry.path().filename().string()));
    }
    return tids;
  };

  std::filesystem::create_directories(dir_);
  const int inotify_fd = ::inotify_init1(IN_CLOEXEC | IN_NONBLOCK);
  ASSERT_GE(inotify_fd, 0);
  const int wd = ::inotify_add_watch(inotify_fd, dir_.c_str(), IN_MOVED_TO);
  ASSERT_GE(wd, 0);

  const std::set<int> before = list_tids();
  FrameRecorder::Options opts;
  opts.output_dir = dir_;
  opts.context = MakeContext();
  opts.sample_hz = 1e12;
  opts.queue_slots = 4;
  FrameRecorder rec{std::move(opts)};

  std::vector<int> spawned;
  for (const int tid : list_tids()) {
    if (!before.contains(tid)) spawned.push_back(tid);
  }
  ASSERT_EQ(spawned.size(), 1u);
  const int writer_tid = spawned.front();

  // Wait for one written JSON pair (moved atomically into dir_) so the writer
  // thread is guaranteed to have passed ConfigureBackgroundThread().
  ASSERT_TRUE(rec.Record(MakeGrayFrame(), MakeRecord(0, "sampled")));
  const auto committed = dir_ / "00000000000000000000_sampled.json";
  while (!std::filesystem::exists(committed)) {
    pollfd pfd{.fd = inotify_fd, .events = POLLIN, .revents = 0};
    // 10 s hang bound (docs/TESTING.md §6.3).
    const int ready = ::poll(&pfd, 1, 10000);
    ASSERT_GT(ready, 0) << "timed out waiting for recorder writer";
    char buf[4096];
    while (::read(inotify_fd, buf, sizeof buf) > 0) {
    }
  }
  ::close(inotify_fd);

  const int raw_policy = ::sched_getscheduler(writer_tid);
  ASSERT_GE(raw_policy, 0);
  const int policy = raw_policy & ~SCHED_RESET_ON_FORK;
  EXPECT_NE(policy, SCHED_FIFO);
  EXPECT_NE(policy, SCHED_RR);
  errno = 0;
  const int nice = ::getpriority(PRIO_PROCESS, static_cast<id_t>(writer_tid));
  ASSERT_EQ(errno, 0);
  EXPECT_GE(nice, talos::process::kBackgroundNice);
}
#endif

TEST_F(RecorderTest,
       ParseRecordRejectsDeepNestingUnquotedKeysAndOverflowingNumbers) {
  CameraContext ctx;
  FrameRecord rec;
  std::string error;

  // Nesting deeper than kMaxDepth (64) must fail instead of overflowing stack.
  const std::string deep =
      "{\"a\":" + std::string(100, '[') + std::string(100, ']') + "}";
  EXPECT_FALSE(ParseRecord(deep, &ctx, &rec, &error));
  EXPECT_FALSE(error.empty());

  // Unquoted object keys and truncated object keys must fail (no OOB ++pos_).
  for (const char* bad : {"{", "{foo:1}", "{x\"width\":1}", "{xwidth\":1}"}) {
    SCOPED_TRACE(bad);
    error.clear();
    EXPECT_FALSE(ParseRecord(bad, &ctx, &rec, &error));
    EXPECT_FALSE(error.empty());
  }

  // Overflowing integers and non-finite/overflowing floats must fail.
  for (const char* bad : {"{\"capture_ns\":99999999999999999999999999999}",
                          "{\"observation\":{\"x_m\":1e400}}",
                          "{\"observation\":{\"x_m\":-1e400}}"}) {
    SCOPED_TRACE(bad);
    error.clear();
    EXPECT_FALSE(ParseRecord(bad, &ctx, &rec, &error));
    EXPECT_FALSE(error.empty());
  }

  // Out-of-int64-range finite doubles read via AsInt() must return fallback
  // without undefined float-to-int conversion (verified by ASan/UBSan; do not
  // assert FE_INVALID because ARM64 clang speculatively hoists fcvtzs before
  // csel without FENV_ACCESS).
  ASSERT_TRUE(
      ParseRecord("{\"format_version\":1,\"width\":1e30,\"height\":-1e30,"
                  "\"capture_ns\":9223372036854775808.0}",
                  &ctx, &rec, &error))
      << error;
  EXPECT_EQ(ctx.width, 0);
  EXPECT_EQ(ctx.height, 0);
  EXPECT_EQ(rec.capture_ns, 0u);
}

TEST_F(RecorderTest, TinyRateClampsIntervalWithoutUbAndRateLimits) {
  FrameRecorder::Options opts;
  opts.output_dir = dir_;
  opts.context = MakeContext();
  opts.sample_hz = 1e-20;
  opts.rejected_hz = 1e-20;
  opts.queue_slots = 4;
  FrameRecorder rec{std::move(opts)};

  const Frame frame = MakeGrayFrame();
  std::feclearexcept(FE_ALL_EXCEPT);
  EXPECT_TRUE(rec.Record(frame, MakeRecord(0, "sampled")));
  EXPECT_EQ(std::fetestexcept(FE_INVALID), 0);
  EXPECT_FALSE(rec.Record(frame, MakeRecord(1'000'000'000, "sampled")));
  EXPECT_EQ(rec.recorded(), 1u);
}

TEST_F(RecorderTest, DuplicateStemsDisambiguateAndTrimKeepsNewerFile) {
  const auto dup_dir = dir_ / "dup";
  const Frame frame = MakeGrayFrame();
  {
    FrameRecorder::Options opts;
    opts.output_dir = dup_dir;
    opts.context = MakeContext();
    opts.sample_hz = 1e12;
    opts.queue_slots = 4;
    FrameRecorder rec{std::move(opts)};
    FrameRecord first = MakeRecord(42, "sampled");
    first.detections[0].tag_id = 1;
    ASSERT_TRUE(rec.Record(frame, first));
  }  // single queued pair finishes on destruction
  const auto first_json = dup_dir / "00000000000000000042_sampled.json";
  const auto first_png = dup_dir / "00000000000000000042_sampled.png";
  ASSERT_TRUE(std::filesystem::exists(first_json));
  ASSERT_TRUE(std::filesystem::exists(first_png));
  const std::uint64_t pair_bytes = std::filesystem::file_size(first_json) +
                                   std::filesystem::file_size(first_png);

  // Budget fits 1 pair, not 2. Writing a second pair with the same capture_ns
  // and reason must disambiguate its stem so TrimToBudget deletes only the
  // older pair instead of double-counting and deleting the newer file.
  {
    FrameRecorder::Options opts;
    opts.output_dir = dup_dir;
    opts.context = MakeContext();
    opts.sample_hz = 1e12;
    opts.queue_slots = 4;
    opts.max_bytes = pair_bytes + pair_bytes / 2;
    FrameRecorder rec{std::move(opts)};
    FrameRecord second = MakeRecord(42, "sampled");
    second.detections[0].tag_id = 2;
    ASSERT_TRUE(rec.Record(frame, second));
  }

  std::vector<int> surviving_tag_ids;
  for (const auto& entry : std::filesystem::directory_iterator(dup_dir)) {
    const std::string name = entry.path().filename().string();
    if (name.size() < 5 || name.substr(name.size() - 5) != ".json") continue;
    const std::string stem = name.substr(0, name.size() - 5);
    EXPECT_TRUE(std::filesystem::exists(dup_dir / (stem + ".png")));
    std::ifstream in(entry.path());
    std::string text((std::istreambuf_iterator<char>(in)), {});
    CameraContext parsed_ctx;
    FrameRecord parsed;
    std::string err;
    ASSERT_TRUE(ParseRecord(text, &parsed_ctx, &parsed, &err)) << err;
    ASSERT_EQ(parsed.detections.size(), 1u);
    surviving_tag_ids.push_back(parsed.detections[0].tag_id);
  }
  EXPECT_EQ(surviving_tag_ids, (std::vector<int>{2}));
}

TEST_F(RecorderTest, QueueFullDropDoesNotAdvanceRateLimit) {
  std::mutex gate_mu;
  std::condition_variable gate_cv;
  int pairs_popped = 0;
  int allowed_pairs = 0;

  const Frame frame = MakeGrayFrame();
  {
    FrameRecorder::Options opts;
    opts.output_dir = dir_;
    opts.context = MakeContext();
    opts.sample_hz = 1.0;  // 1 s interval
    opts.rejected_hz = 1.0;
    opts.queue_slots = 1;
    opts.test_before_pair = [&](const std::function<bool()>& stop_requested) {
      std::unique_lock lock(gate_mu);
      ++pairs_popped;
      gate_cv.notify_all();
      gate_cv.wait(lock, [&] {
        return pairs_popped <= allowed_pairs || stop_requested();
      });
    };
    FrameRecorder rec{std::move(opts)};
    // Always unblock the writer before ~FrameRecorder() joins it, even on
    // early ASSERT failure.
    struct ReleaseWriterOnExit {
      std::mutex& mu;
      std::condition_variable& cv;
      int& allowed;
      ~ReleaseWriterOnExit() {
        {
          std::lock_guard lock(mu);
          allowed = 100;
        }
        cv.notify_all();
      }
    } release_on_exit{gate_mu, gate_cv, allowed_pairs};

    // Enqueue the first sampled frame at t = 0.0 s and hold the writer inside
    // test_before_pair so the queue stays predictable.
    ASSERT_TRUE(rec.Record(frame, MakeRecord(0, "sampled")));
    {
      std::unique_lock lock(gate_mu);
      ASSERT_TRUE(gate_cv.wait_for(lock, std::chrono::seconds{10},
                                   [&] { return pairs_popped == 1; }));
    }

    // Fill the single queue slot with a rejected frame at t = 0.0 s so the
    // sampled rate-limit timestamp remains at 0.0 s.
    ASSERT_TRUE(rec.Record(frame, MakeRecord(0, "rejected")));

    // Queue is full: sampled frame at t = 1.0 s is dropped and must NOT
    // advance the sampled rate-limit timestamp.
    EXPECT_FALSE(rec.Record(frame, MakeRecord(1'000'000'000, "sampled")));
    EXPECT_EQ(rec.dropped(), 1u);

    // Release pair 1 so the writer pops pair 2 and frees the queue slot.
    {
      std::lock_guard lock(gate_mu);
      allowed_pairs = 1;
    }
    gate_cv.notify_all();
    {
      std::unique_lock lock(gate_mu);
      ASSERT_TRUE(gate_cv.wait_for(lock, std::chrono::seconds{10},
                                   [&] { return pairs_popped == 2; }));
    }

    // At t = 1.05 s, the slot is free and >= 1.0 s has elapsed since the last
    // actually enqueued sampled frame (t = 0.0 s): this frame must be queued.
    ASSERT_TRUE(rec.Record(frame, MakeRecord(1'050'000'000, "sampled")));
    EXPECT_EQ(rec.recorded(), 3u);
    EXPECT_EQ(rec.dropped(), 1u);

    // Rate-limit timestamp is now 1.05 s, so t = 1.50 s is rate-limited
    // (returns false without incrementing dropped).
    EXPECT_FALSE(rec.Record(frame, MakeRecord(1'500'000'000, "sampled")));
    EXPECT_EQ(rec.recorded(), 3u);
    EXPECT_EQ(rec.dropped(), 1u);

    // Let the writer pop pair 3 (t = 1.05 s) and finish writing it on
    // destruction.
    {
      std::lock_guard lock(gate_mu);
      allowed_pairs = 3;
    }
    gate_cv.notify_all();
    {
      std::unique_lock lock(gate_mu);
      ASSERT_TRUE(gate_cv.wait_for(lock, std::chrono::seconds{10},
                                   [&] { return pairs_popped == 3; }));
    }
  }

  EXPECT_FALSE(std::filesystem::exists(
      dir_ / (RecordStem(1'000'000'000, "sampled") + ".json")));
  EXPECT_TRUE(std::filesystem::exists(
      dir_ / (RecordStem(1'050'000'000, "sampled") + ".json")));
}

TEST_F(RecorderTest, ParseRecordRoundTripsSubnormalDoublesAndValidatesSchema) {
  CameraContext ctx = MakeContext();
  ctx.intrinsics.k1 = 1e-320;
  FrameRecord r = MakeRecord((std::uint64_t{1} << 63) + 42u, "sampled");
  r.observation.x_m = 1e-320;
  r.truth->x_m = -1e-320;

  const std::string json = SerializeRecord(ctx, r);
  EXPECT_EQ(json.find("\"capture_ns\":-"), std::string::npos) << json;

  CameraContext parsed_ctx;
  FrameRecord parsed;
  std::string error;
  ASSERT_TRUE(ParseRecord(json, &parsed_ctx, &parsed, &error)) << error;
  EXPECT_DOUBLE_EQ(parsed_ctx.intrinsics.k1, 1e-320);
  EXPECT_DOUBLE_EQ(parsed.observation.x_m, 1e-320);
  ASSERT_TRUE(parsed.truth.has_value());
  EXPECT_DOUBLE_EQ(parsed.truth->x_m, -1e-320);
  EXPECT_EQ(parsed.capture_ns, r.capture_ns);

  // Missing, non-integer, or unsupported format_version must fail.
  for (const char* bad : {
           "{}",
           "{\"format_version\":0}",
           "{\"format_version\":2}",
           "{\"format_version\":\"1\"}",
           "{\"format_version\":1.5}",
       }) {
    SCOPED_TRACE(bad);
    error.clear();
    EXPECT_FALSE(ParseRecord(bad, &parsed_ctx, &parsed, &error));
    EXPECT_FALSE(error.empty());
  }

  // Wrong nested JSON object/array/string types must fail.
  for (
      const char* bad : {
          "{\"format_version\":1,\"camera\":123}",
          "{\"format_version\":1,\"reason\":123}",
          "{\"format_version\":1,\"intrinsics\":123}",
          "{\"format_version\":1,\"mounting\":[]}",
          "{\"format_version\":1,\"detector\":\"cpu\"}",
          "{\"format_version\":1,\"detections\":{}}",
          "{\"format_version\":1,\"detections\":[42]}",
          R"({"format_version":1,"detections":[{"tag_id":1,"corners_px":"bad"}]})",
          R"({"format_version":1,"detections":[{"tag_id":1,"reject_reason":123}]})",
          "{\"format_version\":1,\"observation\":[]}",
          "{\"format_version\":1,\"truth\":123}",
      }) {
    SCOPED_TRACE(bad);
    error.clear();
    EXPECT_FALSE(ParseRecord(bad, &parsed_ctx, &parsed, &error));
    EXPECT_FALSE(error.empty());
  }
}

TEST_F(RecorderTest, JsonWriteFailureRemovesOrphanedPng) {
  std::filesystem::create_directories(dir_);
  // Block only the JSON temp file with a non-empty directory so the PNG writes
  // and renames cleanly, then the JSON write fails deterministically.
  const auto json_tmp_dir = dir_ / "00000000000000000000_sampled.json.tmp";
  std::filesystem::create_directories(json_tmp_dir);
  std::ofstream{json_tmp_dir / "keep"} << "block";

  {
    FrameRecorder::Options opts;
    opts.output_dir = dir_;
    opts.context = MakeContext();
    opts.queue_slots = 4;
    FrameRecorder rec{std::move(opts)};
    ASSERT_TRUE(rec.Record(MakeGrayFrame(), MakeRecord(0, "sampled")));
  }  // destructor finishes the single queued pair and joins the writer thread
  EXPECT_FALSE(
      std::filesystem::exists(dir_ / "00000000000000000000_sampled.png"));
  for (const auto& entry : std::filesystem::directory_iterator(dir_)) {
    if (!entry.is_regular_file()) continue;
    EXPECT_NE(entry.path().extension(), ".png") << entry.path();
  }
}

TEST_F(RecorderTest, RejectsPathTraversalReasonAndSanitizesStem) {
  for (const std::string_view bad : {"../escape", "a/b\\c", ".."}) {
    const std::string stem = RecordStem(42, bad);
    EXPECT_EQ(stem.find('/'), std::string::npos) << stem;
    EXPECT_EQ(stem.find('\\'), std::string::npos) << stem;
    EXPECT_EQ(stem.find(".."), std::string::npos) << stem;
  }

  const auto frames_dir = dir_ / "frames";
  {
    FrameRecorder::Options opts;
    opts.output_dir = frames_dir;
    opts.context = MakeContext();
    opts.sample_hz = 1e12;
    opts.rejected_hz = 1e12;
    opts.queue_slots = 8;
    FrameRecorder rec{std::move(opts)};
    const Frame frame = MakeGrayFrame();

    EXPECT_FALSE(rec.Record(frame, MakeRecord(0, "../escape")));
    EXPECT_FALSE(rec.Record(frame, MakeRecord(1, "sub/dir")));
    EXPECT_FALSE(rec.Record(frame, MakeRecord(2, "a\\b")));
    EXPECT_FALSE(rec.Record(frame, MakeRecord(3, "..")));
    EXPECT_EQ(rec.recorded(), 0u);
    EXPECT_EQ(rec.dropped(), 4u);

    // The recorder must stay alive and accept subsequent valid records.
    EXPECT_TRUE(rec.Record(frame, MakeRecord(4, "sampled")));
    EXPECT_EQ(rec.recorded(), 1u);
  }

  EXPECT_TRUE(
      std::filesystem::exists(frames_dir / "00000000000000000004_sampled.png"));
  EXPECT_TRUE(std::filesystem::exists(frames_dir /
                                      "00000000000000000004_sampled.json"));
  // Nothing escaped into parent dir_.
  for (const auto& entry : std::filesystem::directory_iterator(dir_)) {
    EXPECT_EQ(entry.path().filename(), "frames") << entry.path();
  }
}

}  // namespace
}  // namespace talos::vision::recording
