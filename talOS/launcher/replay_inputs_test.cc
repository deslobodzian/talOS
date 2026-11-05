// launcher --replay-inputs: a session recorded by the launcher with real
// node processes on real shared memory, then replayed into a node that was
// not in it (testdata/replay_consumer.cc), twice, and into one that reads the
// recorded topic as another type (testdata/replay_impostor.cc).

#include <gtest/gtest.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "replay_inputs_test/replay_message_generated.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/events/context.h"
#include "talOS/events/log/log_reader.h"
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

struct Sent {
  std::int64_t now_ns;
  std::vector<std::byte> payload;
  bool operator==(const Sent&) const = default;
};

// What `log_path`'s node sent on `topic`, with when.
std::vector<Sent> Sends(const std::string& log_path, std::string_view topic) {
  event::log::LogReader reader{log_path};
  std::uint16_t id = 0xFFFF;
  for (const auto& registration : reader.manifest()) {
    if (registration.name == topic) id = registration.id;
  }
  EXPECT_NE(id, 0xFFFF) << log_path << " has no " << topic;
  std::vector<Sent> sent;
  event::log::LogReader::Record record;
  while (reader.next(record)) {
    if (record.header.kind ==
            static_cast<std::uint16_t>(event::EventKind::SEND) &&
        record.header.source_id == id) {
      sent.push_back({record.header.now_ns,
                      {record.payload.begin(), record.payload.end()}});
    }
  }
  return sent;
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

[subsystems.replay_impostor]
node = "//talOS/launcher:replay_impostor"
)";
    // The recorded session runs the source alone.
    std::ofstream{record_config_} << R"([robot]
name = "replay_inputs"
period_us = 5000

[subsystems.replay_source]
node = "//talOS/launcher:replay_source"

[subsystems.replay_source.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
)";
  }

  void TearDown() override {
    std::filesystem::remove_all(session_dir_);
    std::filesystem::remove(config_);
    std::filesystem::remove(record_config_);
  }

  LauncherOptions Options() const {
    LauncherOptions options;
    options.session_id = 1;
    options.simulation = true;
    options.describe_timeout_ms = 4000;
    options.binary_overrides["//talOS/bridge:hardware_node"] = "/usr/bin/true";
    options.binary_overrides["//talOS/launcher:replay_source"] =
        Runfile("TALOS_SOURCE_NODE");
    options.binary_overrides["//talOS/launcher:replay_consumer"] =
        Runfile("TALOS_CONSUMER_NODE");
    options.binary_overrides["//talOS/launcher:replay_impostor"] =
        Runfile("TALOS_IMPOSTOR_NODE");
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

TEST_F(ReplayInputsTest, AReaderOfAnotherSchemaIsRefusedNamingBothTypes) {
  Record();
  const std::string out = TempPath("impostor");
  std::filesystem::remove_all(out);
  EXPECT_NE(Replay("replay_impostor", out), 0);

  // The node's own words, which the launcher passes through.
  const std::string command = Runfile("TALOS_IMPOSTOR_NODE") +
                              " --sim --replay-inputs '" + session_dir_ +
                              "' --config '" + config_ + "' --log '" + out +
                              "/direct.tlog' 2>&1";
  std::string output;
  FILE* pipe = ::popen(command.c_str(), "r");
  ASSERT_NE(pipe, nullptr);
  std::array<char, 512> buffer{};
  while (std::fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
    output += buffer.data();
  }
  EXPECT_NE(::pclose(pipe), 0) << output;
  EXPECT_NE(output.find("'/replay_source/state'"), std::string::npos) << output;
  EXPECT_NE(output.find("replay_inputs_test.Reading ("), std::string::npos)
      << output;
  EXPECT_NE(output.find("replay_inputs_test.Impostor ("), std::string::npos)
      << output;
  std::filesystem::remove_all(out);
}

TEST_F(ReplayInputsTest, RefusesToWriteIntoTheSessionItReads) {
  Record();
  EXPECT_NE(Replay("replay_consumer", session_dir_), 0);
  EXPECT_FALSE(std::filesystem::exists(session_dir_ + "/replay_consumer.tlog"));
}

}  // namespace
}  // namespace talos::launcher
