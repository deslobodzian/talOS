// launcher --replay-inputs: a session recorded by the launcher with real
// node processes on real shared memory, then replayed into a node that was
// not in it (testdata/replay_consumer.cc), twice, into one that was
// (testdata/replay_sampler.cc), into one that reads the recorded topic as
// another type (testdata/replay_impostor.cc), and into one that reads the
// declared graph and keeps a record under its session directory
// (testdata/replay_graph_reader.cc).

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "replay_inputs_test/replay_message_generated.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/events/context.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/manifest.h"
#include "talOS/launcher/launcher.h"

namespace talos::launcher {
namespace {

using replay_inputs_test::Reading;
using replay_inputs_test::Summary;

constexpr std::uint32_t kReadings = 100;  // ReplaySource::kReadings

std::string Runfile(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!runfiles || location == nullptr) {
    throw std::runtime_error(std::string{"runfile unavailable: "} + env + " " +
                             error);
  }
  return runfiles->Rlocation(location);
}

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_replay_inputs_" +
         std::to_string(::getpid()) + "_" + std::string{name};
}

// Calls `visit(header, payload)` for each `kind` record of `log_path`'s
// source on `topic` (a watcher by MESSAGE, a sender by SEND), in log order.
template <class Visit>
void ForEachRecord(const std::string& log_path, std::string_view topic,
                   event::EventKind kind, Visit visit) {
  event::log::LogReader reader{log_path};
  std::uint16_t id = 0xFFFF;
  for (const auto& registration : reader.manifest()) {
    const bool sender = registration.kind == event::SourceKind::SENDER;
    if (registration.name == topic &&
        sender == (kind == event::EventKind::SEND)) {
      id = registration.id;
    }
  }
  EXPECT_NE(id, 0xFFFF) << log_path << " has no " << topic;
  event::log::LogReader::Record record;
  while (reader.next(record)) {
    if (record.header.kind == static_cast<std::uint16_t>(kind) &&
        record.header.source_id == id) {
      visit(record.header, std::vector<std::byte>{record.payload.begin(),
                                                  record.payload.end()});
    }
  }
}

struct Sent {
  std::int64_t now_ns;
  std::vector<std::byte> payload;
  bool operator==(const Sent&) const = default;
};

// What `log_path`'s node sent on `topic`, with when.
std::vector<Sent> Sends(const std::string& log_path, std::string_view topic) {
  std::vector<Sent> sent;
  ForEachRecord(log_path, topic, event::EventKind::SEND,
                [&](const auto& header, std::vector<std::byte> payload) {
                  sent.push_back({header.now_ns, std::move(payload)});
                });
  return sent;
}

// The same, by the deadline of the dispatch that sent it.
std::map<std::int64_t, std::vector<std::byte>> SendsByDeadline(
    const std::string& log_path, std::string_view topic) {
  std::map<std::int64_t, std::vector<std::byte>> sent;
  ForEachRecord(log_path, topic, event::EventKind::SEND,
                [&](const auto& header, std::vector<std::byte> payload) {
                  sent[header.event_time_ns] = std::move(payload);
                });
  return sent;
}

struct Delivery {
  std::int64_t event_ns;    // the drain that delivered it
  std::int64_t publish_ns;  // when it was published
  std::vector<std::byte> payload;
  bool operator==(const Delivery&) const = default;
};

// What `log_path`'s node received on `topic`, in the order it received it.
std::vector<Delivery> Deliveries(const std::string& log_path,
                                 std::string_view topic) {
  std::vector<Delivery> received;
  ForEachRecord(
      log_path, topic, event::EventKind::MESSAGE,
      [&](const auto& header, std::vector<std::byte> payload) {
        received.push_back(
            {header.event_time_ns, header.publish_time_ns, std::move(payload)});
      });
  return received;
}

template <class M>
M As(const Sent& sent) {
  M message{};
  EXPECT_EQ(sent.payload.size(), sizeof message);
  std::memcpy(&message, sent.payload.data(), sizeof message);
  return message;
}

class ReplayInputsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::filesystem::remove_all(session_dir_);
    std::ofstream{config_} << R"([robot]
name = "replay_inputs"
period_us = 5000

[subsystems.replay_source]
node = "//talOS/launcher:replay_source"

[subsystems.replay_source.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.replay_consumer]
node = "//talOS/launcher:replay_consumer"

[subsystems.replay_sampler]
node = "//talOS/launcher:replay_sampler"

[subsystems.replay_impostor]
node = "//talOS/launcher:replay_impostor"

[subsystems.replay_graph_reader]
node = "//talOS/launcher:replay_graph_reader"
)";
    // The recorded session runs the source, the sampler and the graph
    // reader.
    std::ofstream{record_config_} << R"([robot]
name = "replay_inputs"
period_us = 5000

[subsystems.replay_source]
node = "//talOS/launcher:replay_source"

[subsystems.replay_source.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.replay_sampler]
node = "//talOS/launcher:replay_sampler"

[subsystems.replay_graph_reader]
node = "//talOS/launcher:replay_graph_reader"
)";
    introspect::Description hardware;
    hardware.name = "hardware_node";
    hardware.target = "//talOS/bridge:hardware_node";
    const std::string stub = record_config_ + ".hardware.sh";
    std::ofstream{stub} << "#!/bin/sh\n"
                        << "for arg in \"$@\"; do\n"
                        << "  if [ \"$arg\" = \"--describe\" ]; then\n"
                        << "    cat <<'TALOS_DESCRIBE'\n"
                        << introspect::DescribeToJson(hardware)
                        << "TALOS_DESCRIBE\n"
                        << "    exit 0\n"
                        << "  fi\n"
                        << "done\n"
                        // Alive until the session ends: an exited stub
                        // restarts, and the end's SIGINT landing before the
                        // restart's exec makes the session status nonzero.
                        << "trap 'exit 0' INT TERM\n"
                        << "while :; do sleep 0.05; done\n";
    std::filesystem::permissions(stub, std::filesystem::perms::owner_all);
  }

  void TearDown() override {
    std::filesystem::remove_all(session_dir_);
    std::filesystem::remove(config_);
    std::filesystem::remove(record_config_);
    std::filesystem::remove(record_config_ + ".hardware.sh");
  }

  LauncherOptions Options() const {
    LauncherOptions options;
    options.session_id = 1;
    options.simulation = true;
    options.describe_timeout_ms = 4000;
    options.binary_overrides["//talOS/bridge:hardware_node"] =
        record_config_ + ".hardware.sh";
    options.binary_overrides["//talOS/launcher:replay_source"] =
        Runfile("TALOS_SOURCE_NODE");
    options.binary_overrides["//talOS/launcher:replay_consumer"] =
        Runfile("TALOS_CONSUMER_NODE");
    options.binary_overrides["//talOS/launcher:replay_impostor"] =
        Runfile("TALOS_IMPOSTOR_NODE");
    options.binary_overrides["//talOS/launcher:replay_graph_reader"] =
        Runfile("TALOS_GRAPH_READER_NODE");
    options.binary_overrides["//talOS/launcher:replay_sampler"] =
        Runfile("TALOS_SAMPLER_NODE");
    return options;
  }

  void Record() {
    LauncherOptions options = Options();
    options.config_path = record_config_;
    options.output_dir =
        std::filesystem::path{session_dir_}.parent_path().string();
    options.duration_s = 2;
    ASSERT_EQ(Launcher{options}.Run(), 0);
  }

  // `node` on the recorded session's topics, its log in `out_dir`.
  int Replay(const std::string& node, const std::string& out_dir) {
    LauncherOptions options = Options();
    options.config_path = config_;
    options.output_dir = out_dir;
    options.replay_inputs = session_dir_;
    options.only = node;
    return Launcher{options}.Run();
  }

  const std::string session_dir_ = TempPath("session") + "/1";
  const std::string config_ = TempPath("robot.toml");
  const std::string record_config_ = TempPath("record.toml");
};

TEST_F(ReplayInputsTest, ANodeThatWasNotInTheSessionRunsOnItsTopics) {
  Record();
  const std::vector<Sent> readings =
      Sends(session_dir_ + "/replay_source.tlog", "/replay_source/state");
  ASSERT_EQ(readings.size(), kReadings);

  const std::string first = TempPath("first");
  const std::string second = TempPath("second");
  std::filesystem::remove_all(first);
  std::filesystem::remove_all(second);
  ASSERT_EQ(Replay("replay_consumer", first), 0);
  ASSERT_EQ(Replay("replay_consumer", second), 0);

  const std::vector<Sent> summaries =
      Sends(first + "/1/replay_consumer.tlog", "/replay_consumer/state");
  ASSERT_FALSE(summaries.empty());
  // On the recording's clock: nothing before the first reading arrived
  // counted it, and every reading was counted by the end.
  float expected_total_m = 0.0F;
  for (const Sent& reading : readings) {
    expected_total_m += As<Reading>(reading).position_m();
  }
  const Summary last = As<Summary>(summaries.back());
  EXPECT_EQ(last.reading_count(), kReadings);
  EXPECT_EQ(last.total_m(), expected_total_m);
  for (const Sent& sent : summaries) {
    if (sent.now_ns < readings.front().now_ns) {
      EXPECT_EQ(As<Summary>(sent).reading_count(), 0u);
    }
    std::uint32_t published_by_then = 0;
    for (const Sent& reading : readings) {
      published_by_then += reading.now_ns <= sent.now_ns ? 1 : 0;
    }
    EXPECT_LE(As<Summary>(sent).reading_count(), published_by_then);
  }
  // Reproducible: the second replay sent the same bytes at the same times.
  EXPECT_EQ(Sends(second + "/1/replay_consumer.tlog", "/replay_consumer/state"),
            summaries);

  std::filesystem::remove_all(first);
  std::filesystem::remove_all(second);
}

// REQ-OPS-14: a node the session ran replays on its own recorded timeline.
// Each process's log has its own origin (the sampler was spawned after the
// source, and its loop began running after its construction), and the
// sampler saw each reading at the tick that drained it live. Replayed, it
// sees the same readings at the same ticks, in the same order, with their
// recorded publication times; a replay counted from the session's earliest
// origin lands each reading published between the two timelines' ticks on a
// neighbouring tick. The replayed loop drains at each deadline, without the
// live loop's wake-up latency, so a reading the live drain read although it
// was published during that latency is stamped with the drain's deadline.
TEST_F(ReplayInputsTest, ANodeThatWasInTheSessionSeesItsInputsWhenItDid) {
  Record();
  const std::string recorded_log = session_dir_ + "/replay_sampler.tlog";
  ASSERT_NE(
      event::log::LogReader{recorded_log}.start_time(),
      event::log::LogReader{session_dir_ + "/replay_source.tlog"}.start_time());
  std::vector<Delivery> expected =
      Deliveries(recorded_log, "/replay_source/state");
  ASSERT_FALSE(expected.empty());
  for (Delivery& delivery : expected) {
    delivery.publish_ns = std::min(delivery.publish_ns, delivery.event_ns);
  }

  const std::string out = TempPath("sampler");
  std::filesystem::remove_all(out);
  ASSERT_EQ(Replay("replay_sampler", out), 0);
  const std::vector<Delivery> replayed =
      Deliveries(out + "/1/replay_sampler.tlog", "/replay_source/state");
  EXPECT_EQ(replayed.size(), expected.size());
  const auto [want, got] = std::mismatch(expected.begin(), expected.end(),
                                         replayed.begin(), replayed.end());
  if (want != expected.end() && got != replayed.end()) {
    ADD_FAILURE() << "delivery " << (want - expected.begin())
                  << ": recorded at " << want->event_ns << " ns, published "
                  << want->publish_ns << " ns; replayed at " << got->event_ns
                  << " ns, published " << got->publish_ns << " ns"
                  << (want->payload == got->payload ? "" : ", another reading");
  }
  // So it sends what it sent, at each deadline it sent at live (the replay
  // also ticks at any deadline a late live tick coalesced).
  const auto replayed_sends =
      SendsByDeadline(out + "/1/replay_sampler.tlog", "/replay_sampler/state");
  const auto recorded_sends =
      SendsByDeadline(recorded_log, "/replay_sampler/state");
  ASSERT_FALSE(recorded_sends.empty());
  for (const auto& [deadline_ns, payload] : recorded_sends) {
    const auto replayed_send = replayed_sends.find(deadline_ns);
    if (replayed_send == replayed_sends.end() ||
        replayed_send->second != payload) {
      ADD_FAILURE() << "the tick at " << deadline_ns << " ns sent "
                    << (replayed_send == replayed_sends.end()
                            ? "nothing"
                            : "another summary")
                    << " in the replay";
      break;
    }
  }

  std::filesystem::remove_all(out);
}

TEST_F(ReplayInputsTest, AReaderOfAnotherSchemaIsRefused) {
  Record();
  const std::string out = TempPath("impostor");
  std::filesystem::remove_all(out);
  EXPECT_NE(Replay("replay_impostor", out), 0);

  std::filesystem::remove_all(out);
}

// REQ-OPS-14: a node whose endpoints come from the declared graph (teleop,
// telemetry) replays on the graph its session was launched with, read from
// the replayed session's declared_graph.json: here two state topics besides
// its own, the source's and the sampler's. A session without that file is
// refused, not replayed on an empty graph.
TEST_F(ReplayInputsTest, AGraphReaderReplaysOnTheSessionsDeclaredGraph) {
  Record();
  const std::string out = TempPath("graph_reader");
  std::filesystem::remove_all(out);
  ASSERT_EQ(Replay("replay_graph_reader", out), 0);
  const std::vector<Sent> summaries =
      Sends(out + "/1/replay_graph_reader.tlog", "/replay_graph_reader/state");
  ASSERT_FALSE(summaries.empty());
  EXPECT_EQ(As<Summary>(summaries.back()).reading_count(), 2u);

  std::filesystem::remove_all(out);
  ASSERT_TRUE(std::filesystem::remove(session_dir_ + "/declared_graph.json"));
  EXPECT_NE(Replay("replay_graph_reader", out), 0);

  std::filesystem::remove_all(out);
}

// REQ-OPS-14: a replayed node has no session directory, although the launcher
// passes it --log, so state a node keeps per session (the 2026 autonomous
// node's routine selection, and its progress object, which outlives the
// session in shared memory) is neither read nor written by a replay. The
// recording shows the same node keeps its record under a live session.
TEST_F(ReplayInputsTest, AReplayedNodeKeepsNoSessionState) {
  Record();
  ASSERT_TRUE(
      std::filesystem::exists(session_dir_ + "/replay_graph_reader.record"));
  const std::string out = TempPath("session_state");
  std::filesystem::remove_all(out);
  ASSERT_EQ(Replay("replay_graph_reader", out), 0);
  EXPECT_FALSE(std::filesystem::exists(out + "/1/replay_graph_reader.record"));

  std::filesystem::remove_all(out);
}

TEST_F(ReplayInputsTest, RefusesToWriteIntoTheSessionItReads) {
  Record();
  EXPECT_NE(Replay("replay_consumer", session_dir_), 0);
  EXPECT_FALSE(std::filesystem::exists(session_dir_ + "/replay_consumer.tlog"));
}

}  // namespace
}  // namespace talos::launcher
