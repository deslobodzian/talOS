// tlog_to_wpilog end to end: logs written with LogWriter (as a node's loop
// writes them), converted by the binary, and read back with the minimal
// WPILOG reader below. The mapping under test is the one
// talOS/launcher/README.md documents ("Converting a session to WPILOG").

#include <gtest/gtest.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <bit>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/reflection.h"
#include "geometry/geometry_generated.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/events/context.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/manifest.h"
#include "talOS/schema/identity.h"
#include "wpilog_test/wpilog_message_descriptor.h"
#include "wpilog_test/wpilog_message_generated.h"

extern char** environ;  // POSIX; <unistd.h> declares it only on some hosts

namespace {

using Bytes = std::vector<std::uint8_t>;

// ---- A minimal WPILOG (DataLog 1.0) reader -------------------------------

struct Entry {
  std::string name;
  std::string type;
  std::string metadata;
  int starts{0};
};

struct Value {
  std::uint64_t time_us;
  Bytes payload;
};

struct Wpilog {
  std::string extra_header;
  std::map<std::uint32_t, Entry> entries;
  std::map<std::string, std::uint32_t> ids;
  std::map<std::uint32_t, std::vector<Value>> values;
  std::vector<std::uint64_t> times;  // every data record, in file order

  const Entry* entry(const std::string& name) const {
    const auto id = ids.find(name);
    return id == ids.end() ? nullptr : &entries.at(id->second);
  }
  std::vector<Value> of(const std::string& name) const {
    const auto id = ids.find(name);
    if (id == ids.end()) return {};
    const auto found = values.find(id->second);
    return found == values.end() ? std::vector<Value>{} : found->second;
  }
};

class Cursor {
 public:
  explicit Cursor(const std::string& bytes) : bytes_{bytes} {}
  bool done() const { return at_ == bytes_.size(); }
  std::uint64_t integer(std::size_t width) {
    need(width);
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < width; ++i) {
      value |=
          static_cast<std::uint64_t>(static_cast<std::uint8_t>(bytes_[at_ + i]))
          << (8U * i);
    }
    at_ += width;
    return value;
  }
  std::string text(std::size_t size) {
    need(size);
    std::string out = bytes_.substr(at_, size);
    at_ += size;
    return out;
  }
  std::string string() { return text(integer(4)); }

 private:
  void need(std::size_t size) const {
    if (bytes_.size() - at_ < size) throw std::runtime_error("short WPILOG");
  }
  const std::string& bytes_;
  std::size_t at_{0};
};

Wpilog ParseWpilog(const std::string& bytes) {
  Cursor in{bytes};
  if (in.text(6) != "WPILOG") throw std::runtime_error("bad WPILOG magic");
  if (in.integer(2) != 0x0100) throw std::runtime_error("bad WPILOG version");
  Wpilog log;
  log.extra_header = in.string();
  while (!in.done()) {
    const auto header = static_cast<unsigned>(in.integer(1));
    const auto id = static_cast<std::uint32_t>(in.integer((header & 3U) + 1));
    const std::size_t size = in.integer(((header >> 2U) & 3U) + 1);
    const std::uint64_t time_us = in.integer(((header >> 4U) & 7U) + 1);
    const std::string payload = in.text(size);
    if (id != 0) {
      if (!log.entries.contains(id)) throw std::runtime_error("unstarted id");
      log.values[id].push_back(
          {time_us, Bytes{payload.begin(), payload.end()}});
      log.times.push_back(time_us);
      continue;
    }
    Cursor control{payload};
    const std::uint64_t type = control.integer(1);
    const auto entry = static_cast<std::uint32_t>(control.integer(4));
    if (type == 0) {
      Entry& started = log.entries[entry];
      started.name = control.string();
      started.type = control.string();
      started.metadata = control.string();
      ++started.starts;
      log.ids[started.name] = entry;
    } else if (type == 2) {
      log.entries.at(entry).metadata = control.string();
    } else if (type != 1) {
      throw std::runtime_error("unknown control record");
    }
  }
  return log;
}

std::uint64_t U64(const Bytes& bytes, std::size_t at = 0) {
  std::uint64_t value = 0;
  for (std::size_t i = 0; i < 8; ++i) {
    value |= static_cast<std::uint64_t>(bytes.at(at + i)) << (8U * i);
  }
  return value;
}
double AsDouble(const Bytes& bytes, std::size_t index = 0) {
  return std::bit_cast<double>(U64(bytes, 8 * index));
}
std::int64_t AsInt64(const Bytes& bytes, std::size_t index = 0) {
  return static_cast<std::int64_t>(U64(bytes, 8 * index));
}
float AsFloat(const Bytes& bytes, std::size_t index = 0) {
  std::uint32_t bits = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    bits |= static_cast<std::uint32_t>(bytes.at(4 * index + i)) << (8U * i);
  }
  return std::bit_cast<float>(bits);
}

// ---- Logs as nodes write them ------------------------------------------

constexpr std::int64_t kStartNs = 1'000'000'000;
constexpr std::int64_t kTotalCount = -1'099'511'627'776;  // -2^40

std::int64_t AtMs(double ms) {
  return kStartNs + static_cast<std::int64_t>(ms * 1e6);
}

template <class M>
talos::event::Registration Topic(std::uint16_t id,
                                 talos::event::SourceKind kind,
                                 const std::string& name) {
  talos::event::Registration source;
  source.id = id;
  source.kind = kind;
  source.name = name;
  source.message_bytes = sizeof(M);
  source.alignment = 8;
  source.schema = talos::schema::Of<M>();
  return source;
}

// A target lane of M: its envelope's size and identity.
template <class M>
talos::event::Registration Lane(std::uint16_t id, const std::string& name) {
  auto lane = Topic<M>(id, talos::event::SourceKind::SENDER, name);
  lane.message_bytes += talos::schema::kEnvelopeHeaderBytes;
  lane.schema = talos::schema::Envelope(lane.schema);
  return lane;
}

template <class M>
std::span<const std::byte> BytesOf(const M& message) {
  return std::as_bytes(std::span{&message, 1});
}

wpilog_test::Sample MakeSample(double position_m) {
  wpilog_test::Sample sample;
  sample.mutate_position_m(position_m);
  sample.mutate_ratio_frac(0.25F);
  sample.mutate_offset_count(-3);
  sample.mutate_error_count(-70000);
  sample.mutate_sample_time_ns(std::numeric_limits<std::uint64_t>::max());
  sample.mutate_total_count(kTotalCount);
  sample.mutate_ready(true);
  sample.mutate_phase(wpilog_test::Phase::RUNNING);
  for (int i = 0; i < 3; ++i) sample.mutable_samples_m()->Mutate(i, i + 1.0);
  sample.mutable_gains_frac()->Mutate(0, 0.5F);
  sample.mutable_gains_frac()->Mutate(1, -0.75F);
  sample.mutable_ticks_count()->Mutate(0, 10);
  sample.mutable_ticks_count()->Mutate(1, -20);
  sample.mutable_flags()->Mutate(0, true);
  sample.mutable_flags()->Mutate(1, false);
  *sample.mutable_wheels()->GetMutablePointer(0) =
      wpilog_test::Wheel(2.5F, true);
  *sample.mutable_wheels()->GetMutablePointer(1) =
      wpilog_test::Wheel(-1.0F, false);
  sample.mutable_pose() = geometry::Pose2d(1.0, 2.0, 0.5);
  return sample;
}

// A TargetEnvelope<M> (talOS/devices/target.h): mode_since_us, active,
// seven zero bytes, then the message.
template <class M>
std::vector<std::byte> Envelope(std::uint64_t mode_since_us, bool active,
                                const M& message) {
  std::vector<std::byte> out(talos::schema::kEnvelopeHeaderBytes +
                             sizeof message);
  for (std::size_t i = 0; i < 8; ++i) {
    out[i] = static_cast<std::byte>(mode_since_us >> (8U * i));
  }
  out[talos::schema::kEnvelopeActiveOffset] =
      active ? std::byte{1} : std::byte{0};
  std::memcpy(out.data() + talos::schema::kEnvelopeHeaderBytes, &message,
              sizeof message);
  return out;
}

talos::event::Context At(talos::event::EventKind kind, std::uint16_t source,
                         std::uint64_t index, double event_ms, double now_ms,
                         double publish_ms, std::uint64_t sequence = 0,
                         std::uint64_t dropped = 0) {
  talos::event::Context ctx{};
  ctx.kind = kind;
  ctx.source_id = source;
  ctx.dispatch_index = index;
  ctx.event_time = talos::event::MonotonicTime::from_nanos(AtMs(event_ms));
  ctx.now = talos::event::MonotonicTime::from_nanos(AtMs(now_ms));
  ctx.publish_time = talos::event::MonotonicTime::from_nanos(AtMs(publish_ms));
  ctx.sequence = sequence;
  ctx.dropped = dropped;
  return ctx;
}

class TlogToWpilogTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = testing::TempDir() + "/tlog_to_wpilog_" + std::to_string(getpid());
    session_ = root_ + "/session";
    std::filesystem::create_directories(session_);
    std::string error;
    std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
        rules_cc::cc::runfiles::Runfiles::CreateForTest(
            BAZEL_CURRENT_REPOSITORY, &error));
    ASSERT_NE(runfiles, nullptr) << error;
    const char* const binary = std::getenv("TALOS_TLOG_TO_WPILOG");
    ASSERT_NE(binary, nullptr);
    binary_ = runfiles->Rlocation(binary);
    WriteSession();
  }

  void TearDown() override { std::filesystem::remove_all(root_); }

  void WriteSession() {
    using talos::event::EventKind;
    using talos::event::SourceKind;
    namespace log = talos::event::log;
    const log::LogWriterOptions options{std::size_t{1} << 16U, 1, false};

    // The writer: a timer whose ticks publish /robot/state, two target
    // lanes, a byte topic and a topic whose schema bytes are garbage.
    {
      talos::event::Registration timer;
      timer.id = 1;
      timer.kind = SourceKind::TIMER;
      timer.name = "update";
      talos::event::Registration bytes;
      bytes.id = 4;
      bytes.kind = SourceKind::SENDER;
      bytes.name = "/robot/telemetry";
      bytes.message_bytes = 8;
      bytes.alignment = 8;
      bytes.schema = talos::schema::Bytes();
      auto garbled =
          Topic<wpilog_test::Wheel>(5, SourceKind::SENDER, "/robot/garbled");
      garbled.schema.bfbs = "not a reflection schema";
      log::LogWriter writer{session_ + "/robot.tlog", "robot", options, 42};
      writer.start(
          {timer,
           Topic<wpilog_test::Sample>(2, SourceKind::SENDER, "/robot/state"),
           Lane<wpilog_test::Wheel>(3, "/arm/target/teleop"), bytes, garbled,
           Lane<wpilog_test::Path>(6, "/path/target/auto")},
          talos::event::MonotonicTime::from_nanos(kStartNs));
      const auto first = At(EventKind::TIMER, 1, 1, 10, 10.5, 10.5, 1);
      writer.dispatch(first, {});
      writer.send(first, 2, BytesOf(MakeSample(1.5)), 0, 1);
      writer.send(first, 3,
                  Envelope(123456, true, wpilog_test::Wheel(3.5F, false)), 0,
                  1);
      const std::uint64_t raw = 0x0102030405060708;
      writer.send(first, 4, BytesOf(raw), 0, 1);
      writer.send(first, 5, BytesOf(wpilog_test::Wheel(1.0F, true)), 0, 1);
      wpilog_test::Path path;
      *path.mutable_poses()->GetMutablePointer(0) =
          geometry::Pose2d(4.0, 5.0, 0.25);
      *path.mutable_poses()->GetMutablePointer(1) =
          geometry::Pose2d(6.0, 7.0, -0.5);
      writer.send(first, 6, Envelope(123456, true, path), 0, 1);
      const auto second = At(EventKind::TIMER, 1, 2, 30, 30.25, 30.25, 1);
      writer.dispatch(second, {});
      writer.send(second, 2, BytesOf(MakeSample(-4.0)), 0, 2);
      // SendStatus::DROPPED: never published, so never converted.
      writer.send(second, 2, BytesOf(MakeSample(99.0)), 1, 0);
      writer.flush();
      ASSERT_FALSE(writer.failed()) << writer.error();
    }
    // Two readers of /odometry/state, whose writer is not in the session,
    // and of /robot/state, whose writer is.
    for (const std::string process : {"viewer", "zeta"}) {
      const double skew = process == "zeta" ? 0.5 : 0.0;
      log::LogWriter writer{session_ + "/" + process + ".tlog", process,
                            options, 42};
      writer.start(
          {Topic<wpilog_test::Sample>(1, SourceKind::WATCHER, "/robot/state"),
           Topic<geometry::Pose2d>(2, SourceKind::WATCHER, "/odometry/state")},
          talos::event::MonotonicTime::from_nanos(kStartNs + 5'000'000));
      writer.dispatch(At(EventKind::MESSAGE, 1, 1, 11, 11.2, 10.5, 1),
                      BytesOf(MakeSample(1.5)));
      writer.dispatch(At(EventKind::MESSAGE, 2, 2, 12, 12 + skew, 11.75, 7,
                         skew > 0 ? 0 : 2),
                      BytesOf(geometry::Pose2d(1.0, 2.0, 3.0)));
      writer.dispatch(
          At(EventKind::MESSAGE, 1, 3, 31, 31.2, 30.25, 2, skew > 0 ? 0 : 1),
          BytesOf(MakeSample(-4.0)));
      writer.dispatch(At(EventKind::MESSAGE, 2, 4, 32, 32 + skew, 31.75, 9,
                         skew > 0 ? 0 : 1),
                      BytesOf(geometry::Pose2d(-1.0, -2.0, -3.0)));
      writer.flush();
      ASSERT_FALSE(writer.failed()) << writer.error();
    }
  }

  struct Result {
    int exit_code{-1};
    std::string output;  // stdout and stderr
  };

  // Runs the converter. With `file_size_bytes`, a write past that size
  // fails (RLIMIT_FSIZE, SIGXFSZ ignored): a disk that fills mid-write.
  // The limit binds every file the process writes, and ThreadSanitizer's
  // runtime writes one at startup: a 512 KiB file in $TMPDIR (else
  // $TEST_TMPDIR, else /tmp) that it maps as read-only data's shadow
  // without checking the write's length, so a cut file faults (SIGBUS) on
  // the first read of a string literal. Under the limit TMPDIR names a
  // directory that does not exist, where the runtime skips that file; the
  // converter itself reads no TMPDIR.
  Result Run(std::vector<std::string> args,
             std::optional<rlim_t> file_size_bytes = std::nullopt) {
    args.insert(args.begin(), binary_);
    const std::string output = root_ + "/output.txt";
    std::vector<std::string> environment;
    for (char** variable = environ; *variable != nullptr; ++variable) {
      if (!std::string_view{*variable}.starts_with("TMPDIR=")) {
        environment.emplace_back(*variable);
      }
    }
    if (file_size_bytes) environment.push_back("TMPDIR=" + root_ + "/absent");
    std::vector<char*> envp;
    envp.reserve(environment.size() + 1);
    for (auto& variable : environment) envp.push_back(variable.data());
    envp.push_back(nullptr);
    std::vector<char*> argv;
    argv.reserve(args.size() + 1);
    for (auto& arg : args) argv.push_back(arg.data());
    argv.push_back(nullptr);
    const pid_t child = fork();
    if (child == 0) {
      if (freopen(output.c_str(), "w", stdout) == nullptr ||
          dup2(fileno(stdout), STDERR_FILENO) < 0) {
        _exit(126);
      }
      if (file_size_bytes) {
        const rlimit limit{*file_size_bytes, *file_size_bytes};
        if (std::signal(SIGXFSZ, SIG_IGN) == SIG_ERR ||
            setrlimit(RLIMIT_FSIZE, &limit) != 0) {
          _exit(125);
        }
      }
      execve(binary_.c_str(), argv.data(), envp.data());
      _exit(127);
    }
    if (child < 0) throw std::runtime_error("fork failed");
    int status = 0;
    EXPECT_EQ(waitpid(child, &status, 0), child);
    // Never a crash: the converter exits, whatever it was given.
    EXPECT_TRUE(WIFEXITED(status)) << "status " << status;
    std::ifstream stream{output};
    return {WIFEXITED(status) ? WEXITSTATUS(status) : -1,
            {std::istreambuf_iterator<char>{stream}, {}}};
  }

  // A log at `path` written by `process` from `manifest`, its loop starting
  // at kStartNs; `fill` writes its records.
  template <class Fill>
  void WriteLog(const std::string& path, const std::string& process,
                const talos::event::Manifest& manifest, Fill fill) {
    std::filesystem::create_directories(
        std::filesystem::path{path}.parent_path());
    talos::event::log::LogWriter writer{
        path, process,
        talos::event::log::LogWriterOptions{std::size_t{1} << 16U, 1, false},
        42};
    writer.start(manifest, talos::event::MonotonicTime::from_nanos(kStartNs));
    fill(writer);
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }

  Wpilog Read(const std::string& path) {
    std::ifstream stream{path, std::ios::binary};
    return ParseWpilog({std::istreambuf_iterator<char>{stream}, {}});
  }

  Wpilog Convert(const std::vector<std::string>& inputs) {
    const std::string out = root_ + "/out.wpilog";
    std::vector<std::string> args = inputs;
    args.insert(args.end(), {"-o", out});
    const Result result = Run(args);
    EXPECT_EQ(result.exit_code, 0) << result.output;
    std::ifstream stream{out, std::ios::binary};
    return ParseWpilog({std::istreambuf_iterator<char>{stream}, {}});
  }

  void Copy(const std::string& from, const std::string& to,
            std::size_t keep_bytes) {
    std::ifstream in{from, std::ios::binary};
    std::string bytes{std::istreambuf_iterator<char>{in}, {}};
    bytes.resize(std::min(bytes.size(), keep_bytes));
    std::filesystem::create_directories(
        std::filesystem::path{to}.parent_path());
    std::ofstream{to, std::ios::binary} << bytes;
  }

  std::string root_;
  std::string session_;
  std::string binary_;
};

// Guards: every scalar field becomes `<topic>/<field path>`, typed from its
// FlatBuffers base type, with its unit suffix kept and the unit in the
// metadata; arrays of scalars become WPILOG arrays, arrays of structs
// per-index entries; a ulong keeps its bits as int64.
// REQ-OPS-26
TEST_F(TlogToWpilogTest, FieldsBecomeTypedEntries) {
  const Wpilog log = Convert({session_});
  EXPECT_EQ(log.extra_header, "talos tlog_to_wpilog session_id=42");
  const std::map<std::string, std::string> types = {
      {"/robot/state/position_m", "double"},
      {"/robot/state/ratio_frac", "float"},
      {"/robot/state/offset_count", "int64"},
      {"/robot/state/error_count", "int64"},
      {"/robot/state/sample_time_ns", "int64"},
      {"/robot/state/total_count", "int64"},
      {"/robot/state/ready", "boolean"},
      {"/robot/state/phase", "int64"},
      {"/robot/state/samples_m", "double[]"},
      {"/robot/state/gains_frac", "float[]"},
      {"/robot/state/ticks_count", "int64[]"},
      {"/robot/state/flags", "boolean[]"},
      {"/robot/state/wheels/0/speed_mps", "float"},
      {"/robot/state/wheels/0/slipping", "boolean"},
      {"/robot/state/wheels/1/speed_mps", "float"},
      {"/robot/state/wheels/1/slipping", "boolean"},
      {"/robot/state/pose/x_m", "double"},
      {"/robot/state/pose/y_m", "double"},
      {"/robot/state/pose/yaw_rad", "double"},
      {"/robot/state/pose", "double[]"},
  };
  for (const auto& [name, type] : types) {
    const Entry* entry = log.entry(name);
    ASSERT_NE(entry, nullptr) << name;
    EXPECT_EQ(entry->type, type) << name;
    EXPECT_EQ(entry->starts, 1) << name;
    EXPECT_NE(entry->metadata.find("\"source\":\"talos.tlog\""),
              std::string::npos);
  }
  EXPECT_NE(log.entry("/robot/state/position_m")
                ->metadata.find(R"("schema":"wpilog_test.Sample","unit":"m")"),
            std::string::npos);
  EXPECT_NE(log.entry("/robot/state/wheels/0/speed_mps")
                ->metadata.find(R"("unit":"m/s")"),
            std::string::npos);

  const auto first = [&](const std::string& name) {
    const auto values = log.of(name);
    EXPECT_FALSE(values.empty()) << name;
    return values.empty() ? Bytes{} : values.front().payload;
  };
  EXPECT_EQ(AsDouble(first("/robot/state/position_m")), 1.5);
  EXPECT_EQ(AsFloat(first("/robot/state/ratio_frac")), 0.25F);
  EXPECT_EQ(AsInt64(first("/robot/state/offset_count")), -3);
  EXPECT_EQ(AsInt64(first("/robot/state/error_count")), -70000);
  EXPECT_EQ(AsInt64(first("/robot/state/sample_time_ns")), -1);
  EXPECT_EQ(AsInt64(first("/robot/state/total_count")), kTotalCount);
  EXPECT_EQ(first("/robot/state/ready"), Bytes{1});
  EXPECT_EQ(AsInt64(first("/robot/state/phase")), 4);
  const Bytes samples = first("/robot/state/samples_m");
  ASSERT_EQ(samples.size(), 24u);
  EXPECT_EQ(AsDouble(samples, 0), 1.0);
  EXPECT_EQ(AsDouble(samples, 2), 3.0);
  const Bytes gains = first("/robot/state/gains_frac");
  ASSERT_EQ(gains.size(), 8u);
  EXPECT_EQ(AsFloat(gains, 1), -0.75F);
  const Bytes ticks = first("/robot/state/ticks_count");
  ASSERT_EQ(ticks.size(), 16u);
  EXPECT_EQ(AsInt64(ticks, 1), -20);
  EXPECT_EQ(first("/robot/state/flags"), (Bytes{1, 0}));
  EXPECT_EQ(AsFloat(first("/robot/state/wheels/1/speed_mps")), -1.0F);
  EXPECT_EQ(first("/robot/state/wheels/0/slipping"), Bytes{1});
  EXPECT_EQ(AsDouble(first("/robot/state/pose/y_m")), 2.0);
}

// Guards: the one pose mapping row, geometry.Pose2d as AdvantageScope's
// legacy [x, y, heading_rad] double[], nested (at its field path) or as the
// whole message (`<topic>/pose`).
// REQ-OPS-26
TEST_F(TlogToWpilogTest, Pose2dAlsoBecomesAdvantageScopePoseArray) {
  const Wpilog log = Convert({session_});
  const auto nested = log.of("/robot/state/pose");
  ASSERT_EQ(nested.size(), 2u);
  ASSERT_EQ(nested[0].payload.size(), 24u);
  EXPECT_EQ(AsDouble(nested[0].payload, 0), 1.0);
  EXPECT_EQ(AsDouble(nested[0].payload, 1), 2.0);
  EXPECT_EQ(AsDouble(nested[0].payload, 2), 0.5);
  const auto whole = log.of("/odometry/state/pose");
  ASSERT_EQ(whole.size(), 2u);
  EXPECT_EQ(AsDouble(whole[1].payload, 0), -1.0);
  EXPECT_EQ(AsDouble(whole[1].payload, 2), -3.0);
  EXPECT_EQ(log.entry("/odometry/state/yaw_rad")->type, "double");
}

// Guards: timestamps are the message's publish time (a SEND's dispatch
// `now`, a MESSAGE's writer time), in microseconds from the earliest log
// start; a topic's values come from its writer's sends
// when the writer was logged (not again from each reader), a refused send
// is not a value, and an unlogged writer's topic comes from one reader.
// REQ-OPS-26
TEST_F(TlogToWpilogTest, OneValuePerPublicationAtItsPublishTime) {
  const Wpilog log = Convert({session_});
  const auto position = log.of("/robot/state/position_m");
  ASSERT_EQ(position.size(), 2u);
  EXPECT_EQ(position[0].time_us, 10500u);
  EXPECT_EQ(position[1].time_us, 30250u);
  EXPECT_EQ(AsDouble(position[1].payload), -4.0);
  const auto x = log.of("/odometry/state/x_m");
  ASSERT_EQ(x.size(), 2u);
  EXPECT_EQ(x[0].time_us, 11750u);  // published, not received at 12 ms
  EXPECT_EQ(x[1].time_us, 31750u);
  for (std::size_t i = 1; i < log.times.size(); ++i) {
    EXPECT_LE(log.times[i - 1], log.times[i]) << "record " << i;
  }
}

// Guards: a target lane's envelope header and message, as DecodeToText
// names them.
// REQ-OPS-26
TEST_F(TlogToWpilogTest, TargetEnvelopeHeaderAndMessage) {
  const Wpilog log = Convert({session_});
  const auto mode = log.of("/arm/target/teleop/mode_since_us");
  ASSERT_EQ(mode.size(), 1u);
  EXPECT_EQ(log.entry("/arm/target/teleop/mode_since_us")->type, "int64");
  EXPECT_EQ(AsInt64(mode[0].payload), 123456);
  EXPECT_EQ(log.of("/arm/target/teleop/active")[0].payload, Bytes{1});
  EXPECT_EQ(AsFloat(log.of("/arm/target/teleop/message/speed_mps")[0].payload),
            3.5F);
  EXPECT_EQ(log.of("/arm/target/teleop/message/slipping")[0].payload, Bytes{0});
}

// Guards: each reader's dropped counts become a cumulative int64 entry;
// topics without a usable schema are skipped and named, not fatal.
TEST_F(TlogToWpilogTest, DroppedCountersAndSkippedTopics) {
  const std::string out = root_ + "/explicit.wpilog";
  const Result result =
      Run({session_ + "/robot.tlog", session_ + "/viewer.tlog",
           session_ + "/zeta.tlog", "-o", out});
  ASSERT_EQ(result.exit_code, 0) << result.output;
  EXPECT_NE(result.output.find("skipped /robot/telemetry: talos.Bytes"),
            std::string::npos)
      << result.output;
  EXPECT_NE(result.output.find("skipped /robot/garbled"), std::string::npos);
  std::ifstream stream{out, std::ios::binary};
  const Wpilog log = ParseWpilog({std::istreambuf_iterator<char>{stream}, {}});
  const auto odometry = log.of("/talos_log/dropped/viewer/odometry/state");
  ASSERT_EQ(odometry.size(), 2u);
  EXPECT_EQ(odometry[0].time_us, 11750u);
  EXPECT_EQ(AsInt64(odometry[0].payload), 2);
  EXPECT_EQ(AsInt64(odometry[1].payload), 3);
  const auto state = log.of("/talos_log/dropped/viewer/robot/state");
  ASSERT_EQ(state.size(), 1u);
  EXPECT_EQ(state[0].time_us, 30250u);
  EXPECT_EQ(log.entry("/talos_log/dropped/zeta/odometry/state"), nullptr);
  for (const auto& [name, id] : log.ids) {
    EXPECT_FALSE(name.starts_with("/robot/telemetry")) << name;
    EXPECT_FALSE(name.starts_with("/robot/garbled")) << name;
  }
}

// Guards: bad input is a clear error and exit status 1, never a crash or a
// silently empty output.
// REQ-OPS-26
TEST_F(TlogToWpilogTest, MalformedInputFailsCleanly) {
  const std::string out = root_ + "/bad.wpilog";
  const std::string robot = session_ + "/robot.tlog";
  const auto size = std::filesystem::file_size(robot);

  // Usage errors: no output, an unknown flag (named), -o without a value.
  auto result = Run({robot});
  EXPECT_EQ(result.exit_code, 1);
  result = Run({"--bogus", robot, "-o", out});
  EXPECT_EQ(result.exit_code, 1);
  EXPECT_NE(result.output.find("--bogus"), std::string::npos);
  result = Run({robot, "-o"});
  EXPECT_EQ(result.exit_code, 1);

  std::filesystem::create_directories(root_ + "/empty");
  result = Run({root_ + "/empty", "-o", out});
  EXPECT_EQ(result.exit_code, 1);

  result = Run({root_ + "/missing.tlog", "-o", out});
  EXPECT_EQ(result.exit_code, 1);
  EXPECT_NE(result.output.find("missing.tlog"), std::string::npos);

  // A file that is not a log, and a log whose manifest fails its checksum:
  // each named.
  std::ofstream{root_ + "/text.tlog"} << "not a talos dispatch log at all";
  result = Run({root_ + "/text.tlog", "-o", out});
  EXPECT_EQ(result.exit_code, 1);
  EXPECT_NE(result.output.find("text.tlog"), std::string::npos)
      << result.output;
  Copy(robot, root_ + "/flipped/robot.tlog", size);
  {
    std::fstream file{root_ + "/flipped/robot.tlog",
                      std::ios::binary | std::ios::in | std::ios::out};
    // Inside the first manifest entry's name (format.h: a 120-byte header).
    file.seekp(120 + 20);
    file.put('\x5a');
  }
  result = Run({root_ + "/flipped", "-o", out});
  EXPECT_EQ(result.exit_code, 1);
  EXPECT_NE(result.output.find("flipped/robot.tlog"), std::string::npos)
      << result.output;

  // A log cut inside its header holds no records: alone, nothing to convert.
  Copy(robot, root_ + "/cut/robot.tlog", 60);
  result = Run({root_ + "/cut", "-o", out});
  EXPECT_EQ(result.exit_code, 1) << result.output;
  EXPECT_FALSE(std::filesystem::exists(out));
}

// Guards: a log cut mid-record (power lost while writing) converts its
// intact records, as log_dump and replay read it, with a warning naming it.
// REQ-OPS-26
TEST_F(TlogToWpilogTest, CutLogConvertsItsIntactRecords) {
  const std::string robot = session_ + "/robot.tlog";
  Copy(robot, root_ + "/tail/robot.tlog",
       std::filesystem::file_size(robot) - 10);
  const std::string out = root_ + "/tail.wpilog";
  const Result result = Run({root_ + "/tail", "-o", out});
  ASSERT_EQ(result.exit_code, 0) << result.output;
  EXPECT_NE(result.output.find("tail/robot.tlog"), std::string::npos)
      << result.output;
  std::ifstream stream{out, std::ios::binary};
  const Wpilog log = ParseWpilog({std::istreambuf_iterator<char>{stream}, {}});
  EXPECT_EQ(log.of("/robot/state/position_m").size(), 2u);
}

// Guards: a fixed array of structs inside a target-lane envelope is named
// per index below `message`, and a Pose2d element there is also a pose.
// REQ-OPS-26
TEST_F(TlogToWpilogTest, PoseArrayInsideEnvelopeIsNamedPerIndex) {
  const Wpilog log = Convert({session_});
  const Entry* x = log.entry("/path/target/auto/message/poses/1/x_m");
  ASSERT_NE(x, nullptr);
  EXPECT_EQ(x->type, "double");
  EXPECT_EQ(
      AsDouble(log.of("/path/target/auto/message/poses/1/x_m")[0].payload),
      6.0);
  const auto pose = log.of("/path/target/auto/message/poses/1");
  ASSERT_EQ(pose.size(), 1u);
  EXPECT_EQ(log.entry("/path/target/auto/message/poses/1")->type, "double[]");
  EXPECT_EQ(AsDouble(pose[0].payload, 0), 6.0);
  EXPECT_EQ(AsDouble(pose[0].payload, 1), 7.0);
  EXPECT_EQ(AsDouble(pose[0].payload, 2), -0.5);
  EXPECT_EQ(AsDouble(log.of("/path/target/auto/message/poses/0")[0].payload, 0),
            4.0);
}

// Guards: values are deduplicated per publication (topic, RTMS sequence,
// publish time), not per topic. A reader's copy fills what the writer's log
// lacks (here its head, as a byte cap deletes it), and a publication both
// logs hold is one value, with the writer's bytes.
// REQ-OPS-26
TEST_F(TlogToWpilogTest, ReaderCopyFillsWhatTheWriterLogLacks) {
  using talos::event::EventKind;
  using talos::event::SourceKind;
  const std::string dir = root_ + "/gap";
  WriteLog(dir + "/robot.tlog", "robot",
           {Topic<wpilog_test::Sample>(2, SourceKind::SENDER, "/robot/state")},
           [](talos::event::log::LogWriter& writer) {
             const auto second =
                 At(EventKind::TIMER, 1, 2, 30, 30.25, 30.25, 1);
             writer.send(second, 2, BytesOf(MakeSample(-4.0)), 0, 2);
           });
  WriteLog(dir + "/viewer.tlog", "viewer",
           {Topic<wpilog_test::Sample>(1, SourceKind::WATCHER, "/robot/state")},
           [](talos::event::log::LogWriter& writer) {
             writer.dispatch(At(EventKind::MESSAGE, 1, 1, 11, 11.2, 10.5, 1),
                             BytesOf(MakeSample(1.5)));
             // Bytes unlike the SEND's, to show which copy is kept.
             writer.dispatch(At(EventKind::MESSAGE, 1, 2, 31, 31.2, 30.25, 2),
                             BytesOf(MakeSample(-5.0)));
           });
  const Wpilog log = Convert({dir});
  const auto position = log.of("/robot/state/position_m");
  ASSERT_EQ(position.size(), 2u);
  EXPECT_EQ(position[0].time_us, 10500u);
  EXPECT_EQ(AsDouble(position[0].payload), 1.5);
  EXPECT_EQ(position[1].time_us, 30250u);
  EXPECT_EQ(AsDouble(position[1].payload), -4.0);
}

// A reflection schema whose struct h.s00 has eight fields, all at offset 0,
// of struct h.s01, which has eight of h.s02, ... down to a fieldless h.s16:
// 8^16 struct instances to walk, in one byte.
std::string FanOutSchema() {
  constexpr int kLevels = 17;
  constexpr int kFanOut = 8;
  flatbuffers::FlatBufferBuilder fbb;
  std::vector<flatbuffers::Offset<reflection::Object>> objects;
  for (int level = 0; level < kLevels; ++level) {
    std::vector<flatbuffers::Offset<reflection::Field>> fields;
    for (int k = 0; level + 1 < kLevels && k < kFanOut; ++k) {
      const auto type =
          reflection::CreateType(fbb, reflection::BaseType::Obj,
                                 reflection::BaseType::None, level + 1, 0, 1);
      const std::string name = "f" + std::to_string(k);
      fields.push_back(reflection::CreateFieldDirect(
          fbb, name.c_str(), type, static_cast<std::uint16_t>(k), 0));
    }
    char name[8];
    std::snprintf(name, sizeof name, "h.s%02d", level);
    objects.push_back(
        reflection::CreateObjectDirect(fbb, name, &fields, true, 1, 1));
  }
  std::vector<flatbuffers::Offset<reflection::Enum>> enums;
  reflection::FinishSchemaBuffer(
      fbb, reflection::CreateSchemaDirect(fbb, &objects, &enums));
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

// Guards: a hostile schema whose structs fan out to each other is refused
// by its struct-instance count (skipped and named), not walked for ever.
TEST_F(TlogToWpilogTest, FanOutSchemaIsRefusedNotWalked) {
  using talos::event::EventKind;
  talos::event::Registration hostile;
  hostile.id = 1;
  hostile.kind = talos::event::SourceKind::SENDER;
  hostile.name = "/hostile/state";
  hostile.message_bytes = 1;
  hostile.alignment = 1;
  hostile.schema = {"h.s00", 1, FanOutSchema()};
  WriteLog(root_ + "/hostile/hostile.tlog", "hostile", {hostile},
           [](talos::event::log::LogWriter& writer) {
             const std::uint8_t value = 1;
             writer.send(At(EventKind::TIMER, 1, 1, 10, 10, 10, 1), 1,
                         BytesOf(value), 0, 1);
           });
  const std::string out = root_ + "/hostile.wpilog";
  const Result result = Run({root_ + "/hostile", "-o", out});
  ASSERT_EQ(result.exit_code, 0) << result.output;
  EXPECT_NE(result.output.find("skipped /hostile/state"), std::string::npos)
      << result.output;
  for (const auto& [name, id] : Read(out).ids) {
    EXPECT_FALSE(name.starts_with("/hostile")) << name;
  }
}

// Guards: two topics whose field paths land on one entry name do not share
// it: the first to publish keeps it, the other's entry is `<topic>:<path>`.
TEST_F(TlogToWpilogTest, CollidingEntryNamesAreSeparated) {
  using talos::event::EventKind;
  using talos::event::SourceKind;
  WriteLog(root_ + "/collide/arm.tlog", "arm",
           {Topic<wpilog_test::Arm>(1, SourceKind::SENDER, "/arm/state"),
            Topic<wpilog_test::Side>(2, SourceKind::SENDER, "/arm/state/left")},
           [](talos::event::log::LogWriter& writer) {
             const auto first = At(EventKind::TIMER, 1, 1, 10, 10, 10, 1);
             writer.send(first, 1,
                         BytesOf(wpilog_test::Arm(wpilog_test::Side(1.0))), 0,
                         1);
             const auto second = At(EventKind::TIMER, 1, 2, 20, 20, 20, 1);
             writer.send(second, 2, BytesOf(wpilog_test::Side(2.0)), 0, 1);
           });
  const Wpilog log = Convert({root_ + "/collide"});
  const auto shared = log.of("/arm/state/left/angle_rad");
  ASSERT_EQ(shared.size(), 1u);
  EXPECT_EQ(AsDouble(shared[0].payload), 1.0);
  const auto renamed = log.of("/arm/state/left:angle_rad");
  ASSERT_EQ(renamed.size(), 1u);
  EXPECT_EQ(AsDouble(renamed[0].payload), 2.0);
}

// Guards: a log named twice (its session directory and the file itself) is
// read once, so a reader's dropped counts are not counted twice.
TEST_F(TlogToWpilogTest, LogNamedTwiceIsReadOnce) {
  const Wpilog log = Convert({session_, session_ + "/viewer.tlog"});
  const auto dropped = log.of("/talos_log/dropped/viewer/odometry/state");
  ASSERT_EQ(dropped.size(), 2u);
  EXPECT_EQ(AsInt64(dropped[1].payload), 3);
}

// The names in `directory`, sorted.
std::vector<std::string> Listing(const std::string& directory) {
  std::vector<std::string> names;
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    names.push_back(entry.path().filename().string());
  }
  std::sort(names.begin(), names.end());
  return names;
}

// Guards: a write that fails part way (the file size limit standing in for
// a full disk) exits 1, leaves a previous output byte-identical and no
// temporary file, and creates nothing where there was no output; a
// successful run replaces the previous output.
// REQ-OPS-26
TEST_F(TlogToWpilogTest, FailedWriteKeepsThePreviousOutput) {
  const std::string directory = root_ + "/outputs";
  std::filesystem::create_directories(directory);
  const std::string out = directory + "/match.wpilog";
  const std::string previous = "a previous conversion";
  std::ofstream{out, std::ios::binary} << previous;

  const Result failed = Run({session_, "-o", out}, rlim_t{1024});
  EXPECT_EQ(failed.exit_code, 1) << failed.output;
  {
    std::ifstream stream{out, std::ios::binary};
    EXPECT_EQ(std::string(std::istreambuf_iterator<char>{stream}, {}),
              previous);
  }
  EXPECT_EQ(Listing(directory), std::vector<std::string>{"match.wpilog"});
  const std::string fresh = directory + "/fresh.wpilog";
  EXPECT_EQ(Run({session_, "-o", fresh}, rlim_t{1024}).exit_code, 1);
  EXPECT_EQ(Listing(directory), std::vector<std::string>{"match.wpilog"});

  // The whole conversion is larger than the limit, so the write did fail.
  ASSERT_EQ(Run({session_, "-o", out}).exit_code, 0);
  EXPECT_GT(std::filesystem::file_size(out), 1024u);
  EXPECT_FALSE(Read(out).entries.empty());
  EXPECT_EQ(Listing(directory), std::vector<std::string>{"match.wpilog"});
}

// Guards: an output whose directory takes no new file fails before any
// input is read, naming the output, and leaves an existing output there
// unchanged.
// REQ-OPS-26
TEST_F(TlogToWpilogTest, UnwritableOutputDirectoryFailsFirst) {
  const std::string directory = root_ + "/read_only";
  std::filesystem::create_directories(directory);
  const std::string out = directory + "/match.wpilog";
  std::ofstream{out, std::ios::binary} << "kept";
  std::filesystem::permissions(
      directory,
      std::filesystem::perms::owner_read | std::filesystem::perms::owner_exec);
  if (access(directory.c_str(), W_OK) == 0) {
    std::filesystem::permissions(directory, std::filesystem::perms::owner_all);
    GTEST_SKIP() << "permissions do not bind this user (root)";
  }
  // An unreadable input too: the output is what fails, before it is read.
  const Result result = Run({root_ + "/missing.tlog", "-o", out});
  std::filesystem::permissions(directory, std::filesystem::perms::owner_all);
  EXPECT_EQ(result.exit_code, 1);
  EXPECT_NE(result.output.find("cannot write " + out), std::string::npos)
      << result.output;
  EXPECT_EQ(result.output.find("missing.tlog"), std::string::npos)
      << result.output;
  std::ifstream stream{out, std::ios::binary};
  EXPECT_EQ(std::string(std::istreambuf_iterator<char>{stream}, {}), "kept");
}

}  // namespace
