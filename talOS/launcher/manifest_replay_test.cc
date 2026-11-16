// The session manifest the launcher writes (session.cc
// SessionManifest::ToJson) is the one input replay reads its members' marks
// from (talOS/events/session_manifest.h): nothing here writes a manifest by
// hand, so a key the writer renames shows as a refusal that went missing.

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "replay_inputs_test/replay_message_generated.h"
#include "talOS/events/context.h"
#include "talOS/events/handles.h"
#include "talOS/events/input_replay.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/events/time.h"
#include "talOS/launcher/session.h"

namespace talos::launcher {
namespace {

using namespace std::chrono_literals;
using event::MonotonicTime;
using Loop = event::SimulatedEventLoop<event::log::LogWriter>;
using replay_inputs_test::Reading;

constexpr std::string_view kTopic = "/replay_source/state";
constexpr MonotonicTime kOrigin = MonotonicTime::from_nanos(5'000'000'000);
constexpr int kReadings = 10;

std::string TempDir(const std::string& name) {
  const std::filesystem::path dir =
      std::filesystem::path{::testing::TempDir()} /
      ("talos_manifest_replay_" + name + "_" + std::to_string(::getpid()));
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir.string();
}

// The session's writer of kTopic: one reading every millisecond.
class Source {
 public:
  explicit Source(Loop& loop)
      : state_{event::make_sender<Reading>(loop, kTopic)},
        tick_{event::make_timer<&Source::on_tick>(loop, "tick", this)} {
    tick_.setup_periodic(kOrigin + 1ms, 1ms);
  }

 private:
  void on_tick(const event::Context&) {
    ++count_;
    state_.send(Reading{count_, 0.5F * static_cast<float>(count_)});
  }

  event::Sender<Loop, Reading> state_;
  event::Timer<Loop> tick_;
  std::uint32_t count_{0};
};

// Counts what it is fed.
class Reader {
 public:
  explicit Reader(event::SimulatedEventLoop<>& loop) {
    event::watch<Reading, &Reader::on_reading>(loop, kTopic, this);
  }
  int readings{0};

 private:
  void on_reading(const event::Context&, const Reading&) { ++readings; }
};

// Records the source's run into `<dir>/source.tlog`; without its EXIT
// record when `cut_short`, as a run its process's end or a failed recorder
// cut short leaves it. Returns the log's path.
std::string RecordSource(const std::string& dir, bool cut_short) {
  const std::string path = dir + "/source.tlog";
  {
    event::SimulationEnvironment environment;
    Loop loop{environment, event::log::LogWriter{path, "source"},
              Loop::Options{.start = kOrigin}};
    const Source source{loop};
    loop.run_until(kOrigin + std::chrono::milliseconds{kReadings});
    loop.finish();
  }
  if (cut_short) {
    event::log::LogReader reader{path};
    event::log::LogReader::Record record;
    std::uint64_t exit_offset = 0;
    while (reader.next(record)) {
      if (record.header.kind ==
          static_cast<std::uint16_t>(event::EventKind::EXIT)) {
        exit_offset = record.offset;
      }
    }
    EXPECT_NE(exit_offset, 0u) << path << " has no EXIT record";
    std::filesystem::resize_file(path, exit_offset);
  }
  return path;
}

// A member's log that may have lost records nothing in it marks is refused,
// before anything is fed, when the launcher's own manifest marks its member
// log_gap or recorder_failed and the run's log lacks its EXIT record; with
// its EXIT, a log_gap member's log holds a GAP record for every gap and is
// fed. The manifest is SessionManifest::ToJson's, the launcher's writer.
// REQ-OPS-14
TEST(ManifestReplay, InputReplayRefusesAMemberTheLaunchersManifestMarks) {
  struct Row {
    const char* name;
    bool cut_short;
    bool log_gap;
    bool recorder_failed;
    const char* sign;  // in the refusal; empty: fed in full
  };
  const Row rows[] = {
      {"log_gap, its run cut short", true, true, false, "log_gap"},
      {"recorder_failed, its run cut short", true, false, true,
       "recorder_failed"},
      {"log_gap, its run with its EXIT", false, true, false, ""},
  };
  for (const Row& row : rows) {
    SCOPED_TRACE(row.name);
    const std::string dir = TempDir("session");
    const std::string log = RecordSource(dir, row.cut_short);

    SessionManifest manifest;
    manifest.session_id = event::log::LogReader{log}.session_id();
    NodeProcessInfo member;
    member.name = "source";
    member.log_path = log;
    member.log_paths = {log};
    if (row.log_gap) member.RecordLogGap(log);
    if (row.recorder_failed) member.failed = true;
    manifest.nodes.push_back(member);
    std::string error;
    ASSERT_TRUE(WriteFileAtomically(dir + "/manifest.json", manifest.ToJson(),
                                    {}, error))
        << error;

    const event::RecordedSession session{dir};
    event::SimulationEnvironment environment;
    event::SimulatedEventLoop<> loop{
        environment,
        event::SimulatedEventLoop<>::Options{.start = session.start()}};
    Reader reader{loop};
    const std::string sign{row.sign};
    try {
      const event::InputReplayReport report =
          event::ReplayInputs(loop, environment, session);
      EXPECT_TRUE(sign.empty()) << "fed from " << log << ", marked " << sign;
      EXPECT_FALSE(report.stopped_at_gap.has_value());
    } catch (const event::ReplayError& e) {
      const std::string what = e.what();
      EXPECT_FALSE(sign.empty()) << what;
      EXPECT_NE(what.find(log), std::string::npos) << what;
      EXPECT_NE(what.find(sign), std::string::npos) << what;
    }
    EXPECT_EQ(reader.readings, sign.empty() ? kReadings : 0);
    std::filesystem::remove_all(dir);
  }
}

}  // namespace
}  // namespace talos::launcher
