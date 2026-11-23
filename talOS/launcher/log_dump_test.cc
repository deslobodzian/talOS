#include <gtest/gtest.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "alert_test/alert_message_descriptor.h"
#include "alert_test/alert_message_generated.h"
#include "continuity_test/continuity_message_descriptor.h"
#include "continuity_test/continuity_message_generated.h"
#include "flatbuffers/reflection.h"
#include "flatbuffers/reflection_generated.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "summary_test/summary_message_descriptor.h"
#include "summary_test/summary_message_generated.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/context.h"
#include "talOS/events/log/gap_testing.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/protocol/types.h"

namespace {

class LogDumpTest : public testing::Test {
 protected:
  void SetUp() override {
    directory_ = testing::TempDir() + "/log_dump_" + std::to_string(getpid());
    std::filesystem::create_directories(directory_);
    std::string error;
    std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
        rules_cc::cc::runfiles::Runfiles::CreateForTest(
            BAZEL_CURRENT_REPOSITORY, &error));
    ASSERT_NE(runfiles, nullptr) << error;
    const char* log_dump = std::getenv("TALOS_LOG_DUMP");
    ASSERT_NE(log_dump, nullptr);
    binary_ = runfiles->Rlocation(log_dump);
    WriteLog("alpha", "/alpha/state", {0, 20});
    WriteLog("beta", "/beta/quote,\"\n", {10, 30});
  }

  void TearDown() override { std::filesystem::remove_all(directory_); }

  // One record per time, each {7, 9} unless `values` gives them.
  void WriteLog(const std::string& process, const std::string& topic,
                const std::vector<int>& times,
                const std::vector<continuity_test::Count>& values = {}) {
    std::vector<std::vector<std::byte>> payloads;
    for (std::size_t i = 0; i < times.size(); ++i) {
      const continuity_test::Count value =
          values.empty() ? continuity_test::Count{7, 9} : values.at(i);
      const auto bytes = std::as_bytes(std::span{&value, 1});
      payloads.emplace_back(bytes.begin(), bytes.end());
    }
    WriteRecords<continuity_test::Count>(process, topic, times, payloads);
  }

  // One timer record of message M per time, its payload as given (of any
  // size), each published 3 ms before its event.
  template <class M>
  void WriteRecords(const std::string& process, const std::string& topic,
                    const std::vector<int>& times,
                    const std::vector<std::vector<std::byte>>& payloads) {
    using D = talos::Descriptor<M>;
    talos::event::Registration source;
    source.id = 1;
    source.kind = talos::event::SourceKind::TIMER;
    source.name = topic;
    source.message_bytes = sizeof(M);
    source.schema = {std::string{D::kName}, D::kHash,
                     std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                                 D::kBfbs.size()}};
    talos::event::log::LogWriter writer{
        directory_ + "/" + process + ".tlog", process,
        talos::event::log::LogWriterOptions{4096, 1, false}, 42};
    writer.start({source},
                 talos::event::MonotonicTime::from_nanos(1'000'000'000));
    for (std::size_t i = 0; i < times.size(); ++i) {
      talos::event::Context ctx{};
      ctx.kind = talos::event::EventKind::TIMER;
      ctx.source_id = 1;
      ctx.dispatch_index = i + 1;
      ctx.event_time = talos::event::MonotonicTime::from_nanos(
          1'000'000'000 + times[i] * 1'000'000);
      ctx.now = ctx.event_time;
      ctx.publish_time = ctx.event_time - std::chrono::milliseconds{3};
      ctx.sequence = i + 1;
      writer.dispatch(ctx, payloads.at(i));
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }

  static std::vector<std::string> SplitLines(const std::string& text) {
    std::vector<std::string> out;
    std::istringstream stream{text};
    for (std::string line; std::getline(stream, line);) out.push_back(line);
    return out;
  }

  static bool HasLine(const std::vector<std::string>& lines,
                      const std::string& line) {
    return std::find(lines.begin(), lines.end(), line) != lines.end();
  }

  struct RunResult {
    std::string out;
    std::string err;
    std::uint64_t peak_rss_bytes{0};
  };

  // Dumps the session directory, or only the paths in `args` when
  // `session` is false, keeping stdout and stderr separate.
  RunResult RunSplit(std::vector<std::string> args, bool success = true,
                     bool session = true) {
    args.insert(args.begin(), binary_);
    if (session) args.push_back(directory_);
    const std::string out_path = directory_ + "/stdout.txt";
    const std::string err_path = directory_ + "/stderr.txt";
    const pid_t child = fork();
    if (child == 0) {
      if (freopen(out_path.c_str(), "w", stdout) == nullptr ||
          freopen(err_path.c_str(), "w", stderr) == nullptr) {
        _exit(126);
      }
      std::vector<char*> argv;
      argv.reserve(args.size() + 1);
      for (auto& arg : args) argv.push_back(arg.data());
      argv.push_back(nullptr);
      execv(binary_.c_str(), argv.data());
      _exit(127);
    }
    if (child < 0) throw std::runtime_error("fork failed");
    int status = 0;
    struct rusage usage {};
    EXPECT_EQ(::wait4(child, &status, 0, &usage), child);
    EXPECT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status) == 0, success);
#if defined(__APPLE__)
    const auto peak_rss_bytes = static_cast<std::uint64_t>(usage.ru_maxrss);
#else
    const auto peak_rss_bytes =
        static_cast<std::uint64_t>(usage.ru_maxrss) * 1024u;
#endif
    std::ifstream out_stream{out_path};
    std::ifstream err_stream{err_path};
    return {{std::istreambuf_iterator<char>{out_stream}, {}},
            {std::istreambuf_iterator<char>{err_stream}, {}},
            peak_rss_bytes};
  }

  // Dumps the session directory, or only the paths in `args` when
  // `session` is false.
  std::string Run(std::vector<std::string> args, bool success = true,
                  bool session = true) {
    args.insert(args.begin(), binary_);
    if (session) args.push_back(directory_);
    const std::string output = directory_ + "/output.txt";
    const pid_t child = fork();
    if (child == 0) {
      if (freopen(output.c_str(), "w", stdout) == nullptr ||
          dup2(fileno(stdout), STDERR_FILENO) < 0) {
        _exit(126);
      }
      std::vector<char*> argv;
      argv.reserve(args.size() + 1);
      for (auto& arg : args) argv.push_back(arg.data());
      argv.push_back(nullptr);
      execv(binary_.c_str(), argv.data());
      _exit(127);
    }
    if (child < 0) throw std::runtime_error("fork failed");
    int status = 0;
    EXPECT_EQ(waitpid(child, &status, 0), child);
    EXPECT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status) == 0, success);
    std::ifstream stream{output};
    return {std::istreambuf_iterator<char>{stream}, {}};
  }

  std::string directory_;
  std::string binary_;
};

TEST_F(LogDumpTest, NodeFiltersSelectOnlyRequestedProcesses) {
  for (const auto& selection : std::vector<std::vector<std::string>>{
           {"--node", "alpha,beta"}, {"--node", "alpha", "--node", "beta"}}) {
    auto args = selection;
    args.insert(args.end(), {"--format", "jsonl"});
    const auto records = Run(args);
    EXPECT_EQ(std::count(records.begin(), records.end(), '\n'), 4);
  }
  const auto records = Run({"--format", "jsonl", "--node", "alpha"});
  EXPECT_EQ(std::count(records.begin(), records.end(), '\n'), 2);
  EXPECT_EQ(records.find("\"process\":\"beta\""), std::string::npos);
  EXPECT_NE(records.find("\"process\":\"alpha\""), std::string::npos);
}

TEST_F(LogDumpTest, TopicFiltersAndInclusiveWindow) {
  for (const std::string topic : {"/alpha/state", "/beta/*"}) {
    const auto records = Run({"--format", "jsonl", "--topic", topic});
    EXPECT_EQ(std::count(records.begin(), records.end(), '\n'), 2);
    EXPECT_EQ(records.find(topic == "/alpha/state" ? "\"process\":\"beta\""
                                                   : "\"process\":\"alpha\""),
              std::string::npos);
  }
  const auto records =
      Run({"--format", "jsonl", "--since-ms", "10", "--until-ms", "20"});
  EXPECT_EQ(std::count(records.begin(), records.end(), '\n'), 2);
  EXPECT_NE(records.find("\"event_ms\":10,"), std::string::npos);
  EXPECT_NE(records.find("\"event_ms\":20,"), std::string::npos);
}

TEST_F(LogDumpTest, MachineReadableFormatsEscapeAndDecode) {
  const auto json =
      Run({"--format", "jsonl", "--node", "beta", "--until-ms", "10"});
  EXPECT_EQ(json.front(), '{');
  EXPECT_NE(
      json.find("\"process\":\"beta\",\"dispatch_index\":1,\"kind\":\"TIMER\""),
      std::string::npos);
  EXPECT_NE(json.find("\"topic\":\"/beta/quote,\\\"\\u000a\""),
            std::string::npos);
  EXPECT_NE(json.find("\"event_ms\":10,\"now_ms\":10,\"publish_ms\":7,"
                      "\"sequence\":1,\"aux\":0,\"payload_bytes\":8"),
            std::string::npos);
  EXPECT_NE(json.find("\"schema\":\"continuity_test.Count\","
                      "\"payload_hex\":\"0700000009000000\","
                      "\"decoded\":\"{run_index: 7, sequence_count: 9}\""),
            std::string::npos);
  EXPECT_EQ(json.find("summary"), std::string::npos);
  const auto csv =
      Run({"--format", "csv", "--node", "beta", "--until-ms", "10"});
  EXPECT_TRUE(
      csv.starts_with("process,dispatch_index,kind,source_id,topic,event_ms,"
                      "now_ms,publish_ms,sequence,aux,payload_bytes,schema,"
                      "decoded,hardware\r\n"));
  EXPECT_NE(csv.find("beta,1,TIMER,1,\"/beta/"
                     "quote,\"\"\n\",10,10,7,1,0,8,continuity_test.Count,"),
            std::string::npos);
  EXPECT_NE(csv.find("\"{run_index: 7, sequence_count: 9}\",\r\n"),
            std::string::npos);
  EXPECT_EQ(csv.find("summary"), std::string::npos);
}

TEST_F(LogDumpTest, JsonExportsCompletePayloadBeyondDisplayHexLimit) {
  talos::event::Registration source;
  source.id = 1;
  source.kind = talos::event::SourceKind::SENDER;
  source.name = "/raw";
  source.message_bytes = 96;
  talos::event::log::LogWriter writer{
      directory_ + "/raw.tlog", "raw",
      talos::event::log::LogWriterOptions{4096, 1, false}, 42};
  writer.start({source},
               talos::event::MonotonicTime::from_nanos(1'000'000'000));
  std::vector<std::byte> payload(96, std::byte{0xab});
  payload.back() = std::byte{0};  // Preserve trailing zeros too.
  talos::event::Context ctx{};
  ctx.kind = talos::event::EventKind::SEND;
  ctx.source_id = 1;
  ctx.dispatch_index = 1;
  ctx.event_time = talos::event::MonotonicTime::from_nanos(1'000'000'000);
  ctx.now = ctx.event_time;
  writer.dispatch(ctx, payload);
  writer.flush();
  ASSERT_FALSE(writer.failed()) << writer.error();
  std::string expected{"\"payload_hex\":\""};
  for (int i = 0; i < 95; ++i) expected += "ab";
  expected += "00\"";
  EXPECT_NE(Run({"--format", "jsonl", "--node", "raw"}).find(expected),
            std::string::npos);
}

// A recorded request reads by device name from the config the session's
// manifest names, with no flag; the bytes stay in payload_hex. A relative
// config_path (as manifests written before the launcher recorded it absolute
// hold) is relative to where `bazel run` was run, not to log_dump's runfiles
// directory.
TEST_F(LogDumpTest, HardwarePacketsDecodeWithTheSessionConfig) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.shooter]
period_us = 10000

[subsystems.shooter.motors.flywheel]
type = "TalonFX"
can_id = 2
)";
  std::filesystem::create_directories(directory_ + "/configuration");
  std::ofstream{directory_ + "/configuration/robot.toml"} << kToml;
  std::ofstream{directory_ + "/manifest.json"}
      << "{\n  \"session_id\": 42,\n  \"config_path\": "
         "\"configuration/robot.toml\",\n  \"nodes\": []\n}\n";
  ASSERT_EQ(setenv("BUILD_WORKING_DIRECTORY", directory_.c_str(), 1), 0);
  const auto config = talos::config::ParseRobotConfigString(kToml).hardware;

  using D = talos::Descriptor<talos::hardware::Packet>;
  talos::event::Registration source;
  source.id = 1;
  source.kind = talos::event::SourceKind::SENDER;
  source.name = talos::hardware::RequestTopic("shooter");
  source.message_bytes = sizeof(talos::hardware::Packet);
  source.schema = {std::string{D::kName}, D::kHash, {}};
  talos::event::log::LogWriter writer{
      directory_ + "/shooter.tlog", "shooter",
      talos::event::log::LogWriterOptions{8192, 1, false}, 42};
  writer.start({source},
               talos::event::MonotonicTime::from_nanos(1'000'000'000));
  talos::hardware::Command command;
  command.config_id = talos::hardware::ConfigurationId(config);
  command.count = 1;
  command.motors[0] = {config.motors.at(0).id, talos::hardware::Mode::kVoltage,
                       0, 6.0, 0.0};
  talos::hardware::Packet packet{};
  packet.size =
      static_cast<std::uint32_t>(talos::hardware::Encode(command, packet.data));
  talos::event::Context ctx{};
  ctx.kind = talos::event::EventKind::SEND;
  ctx.source_id = 1;
  ctx.dispatch_index = 1;
  ctx.event_time = talos::event::MonotonicTime::from_nanos(1'000'000'000);
  ctx.now = ctx.event_time;
  writer.dispatch(ctx, std::as_bytes(std::span{&packet, 1}));
  writer.flush();
  ASSERT_FALSE(writer.failed()) << writer.error();

  const auto json = Run({"--format", "jsonl", "--node", "shooter"});
  // The summary reads the packet by name only: the hex its schema-less
  // layout decodes to is no `latest` beside the decoded `hardware`.
  const auto summary =
      SplitLines(Run({"--format", "summary", "--node", "shooter"}));
  unsetenv("BUILD_WORKING_DIRECTORY");
  EXPECT_TRUE(std::any_of(summary.begin(), summary.end(),
                          [](const std::string& line) {
                            return line.starts_with(
                                "  hardware {\"type\":\"command\"");
                          }))
      << ::testing::PrintToString(summary);
  EXPECT_FALSE(std::any_of(
      summary.begin(), summary.end(),
      [](const std::string& line) { return line.starts_with("  latest "); }))
      << ::testing::PrintToString(summary);
  EXPECT_NE(json.find("\"payload_hex\":\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"hardware\":{\"type\":\"command\""), std::string::npos)
      << json;
  EXPECT_NE(json.find("\"name\":\"flywheel\",\"mode\":\"voltage\",\"slot\":0,"
                      "\"demand\":6,"),
            std::string::npos)
      << json;
}

// Guards: a session recorded under another hardware protocol (here a State
// of protocol 7's length, 13 bytes shorter than this build's) is said so in
// one line naming this build's version, and none of its hardware packets is
// decoded with this build's codecs, the Driver Station packet that happens
// to decode included: no per-record error, no `hardware`, no `ds_enabled`.
// Topics with a schema decode as before, and the dump exits 0.
// REQ-OPS-15
TEST_F(LogDumpTest, HardwarePacketsOfAnotherProtocolAreNamedOnceNotDecoded) {
  namespace hw = talos::hardware;
  using D = talos::Descriptor<hw::Packet>;
  const auto topic = [](std::uint16_t id, const char* name) {
    talos::event::Registration source;
    source.id = id;
    source.kind = talos::event::SourceKind::SENDER;
    source.name = name;
    source.message_bytes = sizeof(hw::Packet);
    source.schema = {std::string{D::kName}, D::kHash, {}};
    return source;
  };
  talos::event::log::LogWriter writer{
      directory_ + "/hardware_node.tlog", "hardware_node",
      talos::event::log::LogWriterOptions{1 << 16, 1, false}, 42};
  writer.start({topic(1, hw::kStateTopic), topic(2, hw::kDriverStationTopic)},
               talos::event::MonotonicTime::from_nanos(1'000'000'000));
  const auto send = [&](std::uint16_t source, int ms, const hw::Packet& p) {
    talos::event::Context ctx{};
    ctx.kind = talos::event::EventKind::SEND;
    ctx.source_id = source;
    ctx.dispatch_index = static_cast<std::uint64_t>(ms) + 1;
    ctx.event_time =
        talos::event::MonotonicTime::from_nanos(1'000'000'000 + ms * 1'000'000);
    ctx.now = ctx.event_time;
    ctx.publish_time = ctx.event_time;
    writer.dispatch(ctx, std::as_bytes(std::span{&p, 1}));
  };
  talos::driver_station::DriverStationData enabled;
  enabled.fms.flags =
      talos::driver_station::kDsAttached | talos::driver_station::kEnabled;
  hw::Packet ds{};
  ds.size = static_cast<std::uint32_t>(
      talos::driver_station::Encode(enabled, ds.data));
  send(2, 1, ds);
  for (const int ms : {2, 7, 12}) {
    // Protocol 8 grew the State by 13 bytes (one CAN bus sample per State).
    hw::Packet state{};
    state.size =
        static_cast<std::uint32_t>(hw::Encode(hw::State{}, state.data)) - 13;
    send(1, ms, state);
  }
  writer.flush();
  ASSERT_FALSE(writer.failed()) << writer.error();

  const auto output = Run({"--format", "jsonl"});
  std::vector<std::string> lines;
  std::istringstream stream{output};
  for (std::string line; std::getline(stream, line);) lines.push_back(line);
  const auto named = std::count_if(lines.begin(), lines.end(), [](auto& l) {
    return l.find("hardware packets not decoded") != std::string::npos;
  });
  ASSERT_EQ(named, 1) << output;
  const auto line = *std::find_if(lines.begin(), lines.end(), [](auto& l) {
    return l.find("hardware packets not decoded") != std::string::npos;
  });
  EXPECT_NE(line.find("protocol " +
                      std::to_string(talos::protocol::kProtocolVersion)),
            std::string::npos)
      << line;
  EXPECT_NE(line.find(hw::kStateTopic), std::string::npos) << line;
  EXPECT_EQ(output.find("\"error\""), std::string::npos) << output;
  EXPECT_EQ(output.find("\"hardware\""), std::string::npos) << output;
  EXPECT_EQ(output.find("\"ds_enabled\""), std::string::npos) << output;
  EXPECT_NE(output.find("\"process\":\"alpha\",\"dispatch_index\":2,"),
            std::string::npos)
      << output;
  EXPECT_NE(output.find("\"decoded\":\"{run_index: 7, sequence_count: 9}\""),
            std::string::npos)
      << output;
}

// Guards: log_dump reads the protocol version the session's manifest
// records: a session recorded under another one is not decoded though its
// packets decode under this build, and the line names both versions.
// REQ-OPS-15
TEST_F(LogDumpTest, ManifestOfAnotherProtocolRefusesPacketsThatDecode) {
  const std::uint16_t other = talos::protocol::kProtocolVersion - 1;
  std::ofstream{directory_ + "/manifest.json"}
      << "{\n  \"session_id\": 42,\n  \"hardware_protocol_version\": " << other
      << ",\n  \"nodes\": []\n}\n";
  using D = talos::Descriptor<talos::hardware::Packet>;
  talos::event::Registration source;
  source.id = 1;
  source.kind = talos::event::SourceKind::SENDER;
  source.name = talos::hardware::RequestTopic("shooter");
  source.message_bytes = sizeof(talos::hardware::Packet);
  source.schema = {std::string{D::kName}, D::kHash, {}};
  talos::event::log::LogWriter writer{
      directory_ + "/shooter.tlog", "shooter",
      talos::event::log::LogWriterOptions{8192, 1, false}, 42};
  writer.start({source},
               talos::event::MonotonicTime::from_nanos(1'000'000'000));
  talos::hardware::Packet packet{};
  packet.size = static_cast<std::uint32_t>(
      talos::hardware::Encode(talos::hardware::Command{}, packet.data));
  talos::event::Context ctx{};
  ctx.kind = talos::event::EventKind::SEND;
  ctx.source_id = 1;
  ctx.dispatch_index = 1;
  ctx.event_time = talos::event::MonotonicTime::from_nanos(1'000'000'000);
  ctx.now = ctx.event_time;
  writer.dispatch(ctx, std::as_bytes(std::span{&packet, 1}));
  writer.flush();
  ASSERT_FALSE(writer.failed()) << writer.error();

  const auto output = Run({"--format", "jsonl", "--node", "shooter"});
  EXPECT_EQ(output.find("\"hardware\""), std::string::npos) << output;
  EXPECT_NE(output.find("protocol " + std::to_string(other)), std::string::npos)
      << output;
  // Undecoded, the summary keeps the bytes as the packet's `latest`.
  const auto summary = Run({"--format", "summary", "--node", "shooter"});
  EXPECT_NE(summary.find("\n  latest bytes="), std::string::npos) << summary;
  EXPECT_EQ(summary.find("\n  hardware "), std::string::npos) << summary;
}

// Guards: a log named more than once (as itself, twice, and through its
// session directory) is merged once, so each record prints once.
// REQ-OPS-15
TEST_F(LogDumpTest, LogNamedTwiceIsReadOnce) {
  const std::string alpha = directory_ + "/alpha.tlog";
  const auto records = Run({"--format", "jsonl", alpha, alpha});
  EXPECT_EQ(std::count(records.begin(), records.end(), '\n'), 4) << records;
}

// Guards: a log file dumped alone is timed from its own first record, not
// the session's, and the text dump names that baseline so a reader can
// tell the two apart (beta's first record is 10 ms into the session).
// REQ-OPS-15
TEST_F(LogDumpTest, TextDumpNamesTheBaselineItsTimesCountFrom) {
  const auto alone = Run({directory_ + "/beta.tlog"}, true, false);
  EXPECT_NE(alone.find("\ntimes: ms after event time 1010000000 ns,"),
            std::string::npos)
      << alone;
  const auto session = Run({});
  EXPECT_NE(session.find("\ntimes: ms after event time 1000000000 ns,"),
            std::string::npos)
      << session;
}

// Guards: the summary a student reads to see whether a mechanism moved. Per
// topic it counts the records, keeps the latest value, and ranges each
// numeric field over the session numerically (11 above 2, though the
// latest is 4); --topic and --node select its blocks.
// REQ-OPS-15
TEST_F(LogDumpTest, SummaryGivesCountsLatestValueAndRangePerTopic) {
  WriteLog("gamma", "/gamma/state", {5, 15, 25}, {{2, 9}, {11, 9}, {4, 9}});
  const auto lines = [](const std::string& text) {
    std::vector<std::string> out;
    std::istringstream stream{text};
    for (std::string line; std::getline(stream, line);) out.push_back(line);
    return out;
  };
  const auto has = [](const std::vector<std::string>& block,
                      const std::string& line) {
    return std::find(block.begin(), block.end(), line) != block.end();
  };

  const auto gamma = lines(Run({"--format", "summary", "--topic", "/gamma/*"}));
  EXPECT_EQ(std::count_if(gamma.begin(), gamma.end(),
                          [](const std::string& line) {
                            return line.starts_with("topic ");
                          }),
            1);
  EXPECT_TRUE(has(gamma, "topic /gamma/state continuity_test.Count"));
  EXPECT_TRUE(has(gamma, "  records gamma TIMER 3"));
  EXPECT_TRUE(has(gamma, "  latest {run_index: 4, sequence_count: 9}"));
  EXPECT_TRUE(has(gamma, "  range run_index 2 11"));
  EXPECT_TRUE(has(gamma, "  range sequence_count 9 9"));
  // Publication times: each record was published 3 ms before its event.
  EXPECT_TRUE(has(gamma, "  first_ms 2.000"));
  EXPECT_TRUE(has(gamma, "  last_ms 22.000"));

  const auto alpha = lines(Run({"--format", "summary", "--node", "alpha"}));
  EXPECT_TRUE(has(alpha, "topic /alpha/state continuity_test.Count"));
  EXPECT_TRUE(has(alpha, "  records alpha TIMER 2"));
  EXPECT_FALSE(has(alpha, "topic /gamma/state continuity_test.Count"));
}

// Guards: the summary of a message with an array and an enum. Each array
// element gets its own range by index, and the enum, which has no range,
// lists every value it took in the order first seen, so a mechanism that
// went HOMED then FAILED and back says so though `latest` reads HOMED.
// REQ-OPS-15
TEST_F(LogDumpTest, SummaryRangesArrayElementsAndListsEnumValues) {
  using summary_test::Phase;
  const auto arm = [](float first, float second, Phase phase) {
    std::vector<std::byte> bytes(sizeof(summary_test::Arm));
    std::memcpy(bytes.data(), &first, sizeof first);
    std::memcpy(bytes.data() + 4, &second, sizeof second);
    bytes[8] = static_cast<std::byte>(phase);
    return bytes;
  };
  WriteRecords<summary_test::Arm>(
      "arm", "/arm/state", {5, 15, 25, 35},
      {arm(0.5F, -1, Phase::IDLE), arm(2, 3, Phase::HOMED),
       arm(1, -4, Phase::FAILED), arm(1.5F, 0, Phase::HOMED)});

  const auto arm_summary =
      SplitLines(Run({"--format", "summary", "--topic", "/arm/state"}));
  const auto print = ::testing::PrintToString(arm_summary);
  EXPECT_TRUE(HasLine(arm_summary, "topic /arm/state summary_test.Arm"))
      << print;
  EXPECT_TRUE(
      HasLine(arm_summary, "  latest {position_m: [1.5, 0], phase: HOMED}"))
      << print;
  EXPECT_TRUE(HasLine(arm_summary, "  range position_m[0] 0.5 2")) << print;
  EXPECT_TRUE(HasLine(arm_summary, "  range position_m[1] -4 3")) << print;
  EXPECT_TRUE(HasLine(arm_summary, "  values phase IDLE HOMED FAILED"))
      << print;
  EXPECT_FALSE(std::any_of(arm_summary.begin(), arm_summary.end(),
                           [](const std::string& line) {
                             return line.starts_with("  range phase");
                           }))
      << print;
}

// Guards: a record its schema cannot decode (here 4 bytes of an 8-byte
// message) is counted and shown as `latest` with why, but is no field's
// value: the ranges keep the decoded records' values and no range or value
// line is made of the error text.
// REQ-OPS-15
TEST_F(LogDumpTest, SummaryCountsAnUndecodableRecordButRangesNoneOfIt) {
  const continuity_test::Count low{2, 9};
  const continuity_test::Count high{11, 9};
  const auto bytes = [](const continuity_test::Count& value) {
    const auto span = std::as_bytes(std::span{&value, 1});
    return std::vector<std::byte>(span.begin(), span.end());
  };
  WriteRecords<continuity_test::Count>(
      "delta", "/delta/state", {5, 15, 25},
      {bytes(low), bytes(high), std::vector<std::byte>(4, std::byte{0x7f})});

  const auto delta =
      SplitLines(Run({"--format", "summary", "--topic", "/delta/state"}));
  const auto print = ::testing::PrintToString(delta);
  EXPECT_TRUE(HasLine(delta, "  records delta TIMER 3")) << print;
  EXPECT_TRUE(HasLine(delta, "  latest <4 bytes, continuity_test.Count is 8>"))
      << print;
  EXPECT_TRUE(HasLine(delta, "  range run_index 2 11")) << print;
  EXPECT_TRUE(HasLine(delta, "  range sequence_count 9 9")) << print;
  EXPECT_EQ(std::count_if(delta.begin(), delta.end(),
                          [](const std::string& line) {
                            return line.starts_with("  range ") ||
                                   line.starts_with("  values ");
                          }),
            2)
      << print;
}

// Guards: each record of a node's `status` topic carries every alert field's
// level and state, nested ones by path, so a session read offline shows when
// each alert was raised; an `alert_` field that names no level, the same
// message on a topic of another role, and a record with no alert field get
// none.
// REQ-OPS-37
TEST_F(LogDumpTest, StatusRecordsCarryTheirAlerts) {
  using D = talos::Descriptor<alert_test::Status>;
  talos::event::log::LogWriter writer{
      directory_ + "/watchdog.tlog", "watchdog",
      talos::event::log::LogWriterOptions{4096, 1, false}, 42};
  std::vector<talos::event::Registration> sources;
  for (const std::uint16_t id : {1, 2}) {
    talos::event::Registration source;
    source.id = id;
    source.kind = talos::event::SourceKind::SENDER;
    source.name = id == 1 ? "/watchdog/status" : "/watchdog/state";
    source.message_bytes = sizeof(alert_test::Status);
    source.schema = {std::string{D::kName}, D::kHash,
                     std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                                 D::kBfbs.size()}};
    sources.push_back(source);
  }
  writer.start(sources, talos::event::MonotonicTime::from_nanos(1'000'000'000));
  const alert_test::Status status{true, true, alert_test::Health{false}, 12.5F};
  for (const std::uint16_t id : {1, 2}) {
    talos::event::Context ctx{};
    ctx.kind = talos::event::EventKind::SEND;
    ctx.source_id = id;
    ctx.dispatch_index = id;
    ctx.event_time = talos::event::MonotonicTime::from_nanos(1'000'000'000);
    ctx.now = ctx.event_time;
    ctx.publish_time = ctx.event_time;
    writer.send(ctx, id, std::as_bytes(std::span{&status, 1}), 0, id);
  }
  writer.flush();
  ASSERT_FALSE(writer.failed()) << writer.error();

  const auto status_json =
      Run({"--format", "jsonl", "--topic", "/watchdog/status"});
  EXPECT_NE(status_json.find(
                ",\"alerts\":[{\"name\":\"alert_error_link_lost\",\"level\":"
                "\"error\",\"active\":true},{\"name\":\"health.alert_warning_"
                "hot\",\"level\":\"warning\",\"active\":false}]}"),
            std::string::npos)
      << status_json;
  for (const std::string topic : {"/watchdog/state", "/alpha/state"}) {
    const auto json = Run({"--format", "jsonl", "--topic", topic});
    EXPECT_NE(json.find("\"topic\":\"" + topic + "\""), std::string::npos)
        << json;
    EXPECT_EQ(json.find("\"alerts\""), std::string::npos) << json;
  }
}

// A session whose `gamma` log lost dispatches 2-3 (records at 15 and 25 ms,
// capture resumed at 35 ms) and dispatches 5-6 (45 and 55 ms, closed by the
// stop), beside the fixture's alpha and beta: two GAP records in one file.
class LogDumpGapTest : public LogDumpTest {
 protected:
  void SetUp() override {
    LogDumpTest::SetUp();
    if (HasFatalFailure()) return;
    namespace log = talos::event::log;
    gamma_ = directory_ + "/gamma.tlog";
    WriteLog("gamma", "/gamma/state", {5, 15, 25, 35, 45, 55});
    const std::string staged = directory_ + "/gamma.staged";
    first_ = log::CopyWithGap(gamma_, staged, 1, 3);
    // Records now: 5, GAP, 35, 45, 55.
    last_ = log::CopyWithGap(staged, gamma_, 3, 5);
    std::filesystem::remove(staged);
  }

  static std::vector<std::string> Lines(const std::string& text) {
    std::vector<std::string> out;
    std::istringstream stream{text};
    for (std::string line; std::getline(stream, line);) out.push_back(line);
    return out;
  }

  // The output lines holding `text`.
  static std::size_t CountLines(const std::string& output,
                                const std::string& text) {
    const auto lines = Lines(output);
    return static_cast<std::size_t>(
        std::count_if(lines.begin(), lines.end(), [&](const std::string& l) {
          return l.find(text) != std::string::npos;
        }));
  }

  std::string gamma_;
  talos::event::log::LogGap first_;
  talos::event::log::LogGap last_;
};

// Guards: each GAP record reaches every export with the facts a reader of
// the session needs (where the hole is, what it lost, why, whether it
// closed at the stop): text prints the gap's sentence under its record,
// JSONL a `gap` object, CSV the sentence in `decoded`. Times are on the
// session baseline (alpha's first record).
// REQ-OPS-15
TEST_F(LogDumpGapTest, EachGapIsPrintedAndExported) {
  using talos::event::log::DescribeGap;
  const auto json = Run({"--format", "jsonl", "--node", "gamma"});
  EXPECT_NE(json.find("{\"process\":\"gamma\",\"dispatch_index\":2,\"kind\":"
                      "\"GAP\",\"source_id\":65535,\"topic\":\"?\","
                      "\"event_ms\":15,\"now_ms\":35,"),
            std::string::npos)
      << json;
  EXPECT_NE(
      json.find(",\"gap\":{\"start_ms\":15,\"end_ms\":35,\"first_lost_ms\":15,"
                "\"last_lost_ms\":25,\"first_dispatch_index\":2,"
                "\"last_dispatch_index\":3,\"records\":2,\"bytes\":144,"
                "\"dispatches\":2,\"sends\":0,\"cause\":\"pool_full\","
                "\"splits_dispatch\":false,\"closed_at_flush\":false}}\n"),
      std::string::npos)
      << json;
  EXPECT_NE(
      json.find(",\"gap\":{\"start_ms\":45,\"end_ms\":55,\"first_lost_ms\":45,"
                "\"last_lost_ms\":55,\"first_dispatch_index\":5,"
                "\"last_dispatch_index\":6,\"records\":2,\"bytes\":144,"
                "\"dispatches\":2,\"sends\":0,\"cause\":\"pool_full\","
                "\"splits_dispatch\":false,\"closed_at_flush\":true}}\n"),
      std::string::npos)
      << json;

  const auto csv = Run({"--format", "csv", "--node", "gamma"});
  EXPECT_EQ(CountLines(csv, ",GAP,65535,?,"), 2U) << csv;
  for (const auto& gap : {first_, last_}) {
    // `decoded` holds the gap's sentence, quoted for its commas.
    EXPECT_NE(csv.find("\"" + DescribeGap(gap) + "\",\r\n"), std::string::npos)
        << csv;
  }
  EXPECT_NE(csv.find("gamma,2,GAP,65535,?,15,35,"), std::string::npos) << csv;

  const auto text = Lines(Run({"--node", "gamma"}));
  for (const auto& gap : {first_, last_}) {
    const auto at =
        std::find(text.begin(), text.end(), "      gap: " + DescribeGap(gap));
    ASSERT_NE(at, text.end()) << DescribeGap(gap);
    ASSERT_NE(at, text.begin());
    EXPECT_NE((at - 1)->find("[gamma       ] #" +
                             std::to_string(gap.first_dispatch_index)),
              std::string::npos)
        << *(at - 1);
    EXPECT_NE((at - 1)->find(" GAP "), std::string::npos) << *(at - 1);
  }
}

// Guards: a gap concerns every topic of its log, so no --topic filter hides
// it; --node selects it by its process; the time window selects it by its
// whole span, so a window the gap only overlaps shows it and one wholly
// outside it does not.
// REQ-OPS-15
TEST_F(LogDumpGapTest, FiltersSelectAGapByItsProcessAndSpan) {
  const std::string kGap = "\"kind\":\"GAP\"";
  const auto topic = Run({"--format", "jsonl", "--topic", "/alpha/state"});
  EXPECT_EQ(CountLines(topic, kGap), 2U) << topic;
  EXPECT_EQ(CountLines(topic, "\"process\":\"beta\""), 0U) << topic;

  const auto other = Run({"--format", "jsonl", "--node", "alpha,beta"});
  EXPECT_EQ(CountLines(other, kGap), 0U) << other;

  // [20, 30] lies inside the first gap's [15, 35] and before the second's.
  const auto inside =
      Run({"--format", "jsonl", "--since-ms", "20", "--until-ms", "30"});
  EXPECT_EQ(CountLines(inside, kGap), 1U) << inside;
  EXPECT_NE(inside.find("\"dispatch_index\":2,\"kind\":\"GAP\""),
            std::string::npos)
      << inside;
  // [50, 60] overlaps only the second's [45, 55].
  const auto tail = Run({"--format", "jsonl", "--since-ms", "50"});
  EXPECT_EQ(CountLines(tail, kGap), 1U) << tail;
  EXPECT_NE(tail.find("\"dispatch_index\":5,\"kind\":\"GAP\""),
            std::string::npos)
      << tail;
  // A window ending exactly at a gap's start still meets it.
  const auto edge = Run({"--format", "jsonl", "--until-ms", "15"});
  EXPECT_EQ(CountLines(edge, kGap), 1U) << edge;
  // [36, 44] lies between them.
  const auto between =
      Run({"--format", "jsonl", "--since-ms", "36", "--until-ms", "44"});
  EXPECT_EQ(CountLines(between, kGap), 0U) << between;
}

// Guards: --format summary lists each process's gaps after the topics, in
// numbers a reader adds up (how many, records, dispatches and sends lost,
// earliest start, latest end), and never as a topic; a process with none,
// or filtered out, has no block.
// REQ-OPS-15
TEST_F(LogDumpGapTest, SummaryListsGapsPerProcess) {
  const auto summary = Lines(Run({"--format", "summary"}));
  const auto block = std::find(summary.begin(), summary.end(), "gaps gamma");
  ASSERT_NE(block, summary.end()) << ::testing::PrintToString(summary);
  const std::vector<std::string> expected{
      "gaps gamma", "  count 2",         "  records 4",      "  dispatches 4",
      "  sends 0",  "  first_ms 15.000", "  last_ms 55.000",
  };
  ASSERT_GE(summary.end() - block,
            static_cast<std::ptrdiff_t>(expected.size()));
  EXPECT_EQ(std::vector<std::string>(block, block + expected.size()), expected);
  // After every topic block.
  EXPECT_EQ(std::find_if(block, summary.end(),
                         [](const std::string& line) {
                           return line.starts_with("topic ");
                         }),
            summary.end());
  EXPECT_EQ(CountLines(Run({"--format", "summary", "--topic", "/alpha/state"}),
                       "gaps gamma"),
            1U);
  const auto alpha = Run({"--format", "summary", "--node", "alpha"});
  EXPECT_EQ(CountLines(alpha, "gaps "), 0U) << alpha;
  EXPECT_EQ(CountLines(alpha, "topic ?"), 0U) << alpha;
}

// Guards: a log with gaps is named once on stderr, whatever the format and
// however many gaps it holds, so a reader of a whole session sees which
// file has a hole without one line per gap.
// REQ-OPS-15
TEST_F(LogDumpGapTest, ALogWithGapsIsWarnedOnce) {
  for (const std::string format : {"text", "jsonl", "csv", "summary"}) {
    const auto output = Run({"--format", format});
    EXPECT_EQ(CountLines(output, "warning: log gap in "), 1U) << format << "\n"
                                                              << output;
    EXPECT_EQ(CountLines(output, "warning: log gap in " + gamma_ + ": "), 1U)
        << format << "\n"
        << output;
  }
}

// Verified reflection.Schema .bfbs for:
//   namespace bridge;
//   struct BridgeStatus { protocol_version_id:uint16; }
// Generated with flatc --binary --schema so log_dump's protocol-version reader
// resolves `protocol_version_id` by field name without linking bridge messages.
constexpr unsigned char kMinimalBridgeStatusBfbs[] = {
    0x1c, 0x00, 0x00, 0x00, 0x42, 0x46, 0x42, 0x53, 0x14, 0x00, 0x1c, 0x00,
    0x04, 0x00, 0x08, 0x00, 0x0c, 0x00, 0x10, 0x00, 0x00, 0x00, 0x14, 0x00,
    0x00, 0x00, 0x18, 0x00, 0x14, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00,
    0x28, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
    0x08, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x30, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
    0x94, 0xff, 0xff, 0xff, 0x3c, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x14, 0x00, 0x1c, 0x00, 0x08, 0x00, 0x0c, 0x00,
    0x07, 0x00, 0x10, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00,
    0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x28, 0x00, 0x00, 0x00,
    0x1c, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
    0x04, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x2f, 0x2f, 0x73, 0x2e,
    0x66, 0x62, 0x73, 0x00, 0x01, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00,
    0x13, 0x00, 0x00, 0x00, 0x62, 0x72, 0x69, 0x64, 0x67, 0x65, 0x2e, 0x42,
    0x72, 0x69, 0x64, 0x67, 0x65, 0x53, 0x74, 0x61, 0x74, 0x75, 0x73, 0x00,
    0x08, 0x00, 0x0c, 0x00, 0x04, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00,
    0x28, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x10, 0x00, 0x10, 0x00,
    0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x0c, 0x00,
    0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x02, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x70, 0x72, 0x6f, 0x74,
    0x6f, 0x63, 0x6f, 0x6c, 0x5f, 0x76, 0x65, 0x72, 0x73, 0x69, 0x6f, 0x6e,
    0x5f, 0x69, 0x64, 0x00,
};

// REQ-OPS-12, REQ-OPS-15: a log cut mid-record (power loss) exports every
// intact record before the tail and warns on stderr naming the log and the
// tail byte count.
TEST_F(LogDumpTest, DamagedTailWarnsNamingLogAndTailBytes) {
  WriteLog("gamma", "/gamma/state", {5, 15, 25});
  const std::string gamma = directory_ + "/gamma.tlog";
  std::vector<std::uint64_t> offsets;
  {
    talos::event::log::LogReader reader{gamma};
    talos::event::log::LogReader::Record record;
    while (reader.next(record)) offsets.push_back(record.offset);
  }
  ASSERT_EQ(offsets.size(), 3u);
  constexpr std::uint64_t kTailBytes = 20;
  std::filesystem::resize_file(gamma, offsets[2] + kTailBytes);

  const RunResult res = RunSplit({"--format", "jsonl"});
  EXPECT_EQ(std::count(res.out.begin(), res.out.end(), '\n'), 6) << res.out;
  EXPECT_NE(res.out.find("\"process\":\"gamma\",\"dispatch_index\":1,"),
            std::string::npos)
      << res.out;
  EXPECT_NE(res.out.find("\"process\":\"gamma\",\"dispatch_index\":2,"),
            std::string::npos)
      << res.out;
  EXPECT_EQ(res.out.find("\"process\":\"gamma\",\"dispatch_index\":3,"),
            std::string::npos)
      << res.out;
  const std::string expected_warning =
      "warning: damaged log " + gamma + ": no intact record in the last " +
      std::to_string(kTailBytes) + " bytes (from byte " +
      std::to_string(offsets[2]) + ")";
  const auto err_lines = SplitLines(res.err);
  EXPECT_EQ(std::count(err_lines.begin(), err_lines.end(), expected_warning), 1)
      << res.err;
}

// REQ-OPS-12, REQ-OPS-15: log_dump continues past mid-log corruption (exporting
// records before and after the damaged span alongside intact sibling logs) and
// skips a sibling whose header was cut short, exiting 0 with warnings on
// stderr.
TEST_F(LogDumpTest, MidLogCorruptionAndCutHeaderSiblingExportIntactRecords) {
  WriteLog("gamma", "/gamma/state", {5, 15, 25, 35});
  const std::string gamma = directory_ + "/gamma.tlog";
  std::vector<std::uint64_t> offsets;
  {
    talos::event::log::LogReader reader{gamma};
    talos::event::log::LogReader::Record record;
    while (reader.next(record)) offsets.push_back(record.offset);
  }
  ASSERT_EQ(offsets.size(), 4u);
  {
    std::fstream file{gamma, std::ios::binary | std::ios::in | std::ios::out};
    ASSERT_TRUE(file);
    file.seekg(static_cast<std::streamoff>(offsets[1] + 8));
    char byte = 0;
    ASSERT_TRUE(file.get(byte));
    file.seekp(static_cast<std::streamoff>(offsets[1] + 8));
    file.put(static_cast<char>(byte ^ 0xFF));
  }
  // A sibling log truncated mid-FileHeader holds no records and is skipped.
  const std::string cut = directory_ + "/cut.tlog";
  {
    std::ifstream src{directory_ + "/alpha.tlog", std::ios::binary};
    char prefix[16]{};
    ASSERT_TRUE(src.read(prefix, sizeof prefix));
    std::ofstream dst{cut, std::ios::binary};
    dst.write(prefix, sizeof prefix);
  }

  const RunResult res = RunSplit({"--format", "jsonl"});
  EXPECT_EQ(std::count(res.out.begin(), res.out.end(), '\n'), 7) << res.out;
  // Record before corruption and both intact records after it are exported.
  EXPECT_NE(res.out.find("\"process\":\"gamma\",\"dispatch_index\":1,"),
            std::string::npos)
      << res.out;
  EXPECT_EQ(res.out.find("\"process\":\"gamma\",\"dispatch_index\":2,"),
            std::string::npos)
      << res.out;
  EXPECT_NE(res.out.find("\"process\":\"gamma\",\"dispatch_index\":3,"),
            std::string::npos)
      << res.out;
  EXPECT_NE(res.out.find("\"process\":\"gamma\",\"dispatch_index\":4,"),
            std::string::npos)
      << res.out;
  // Sibling logs still export.
  EXPECT_NE(res.out.find("\"process\":\"alpha\","), std::string::npos)
      << res.out;
  EXPECT_NE(res.out.find("\"process\":\"beta\","), std::string::npos)
      << res.out;

  const auto err_lines = SplitLines(res.err);
  const auto damaged_prefix =
      "warning: damaged log " + gamma + ": 1 damaged span(s) skipped (";
  const auto first_at = "first at byte " + std::to_string(offsets[1]) + ")";
  EXPECT_EQ(std::count_if(err_lines.begin(), err_lines.end(),
                          [&](const std::string& l) {
                            return l.starts_with(damaged_prefix) &&
                                   l.find(first_at) != std::string::npos;
                          }),
            1)
      << res.err;
  EXPECT_EQ(
      std::count_if(err_lines.begin(), err_lines.end(),
                    [&](const std::string& l) {
                      return l.starts_with("warning: skipping " + cut + ": ");
                    }),
      1)
      << res.err;
}

// REQ-OPS-51: when the bridge's /hw/status protocol_version_id and
// manifest.json's hardware_protocol_version disagree, log_dump prints one
// warning naming both versions and reads the session under the bridge's
// version (decoding when bridge == build, refusing when bridge != build).
TEST_F(LogDumpTest,
       BridgeAndManifestProtocolDisagreementWarnsOnceAndUsesBridgesVersion) {
  namespace hw = talos::hardware;
  using D = talos::Descriptor<hw::Packet>;
  const std::uint16_t kBuild = talos::protocol::kProtocolVersion;
  const std::uint16_t kOther = static_cast<std::uint16_t>(kBuild - 1);

  const auto write_bridge = [&](std::uint16_t bridge_version) {
    talos::event::Registration status_reg;
    status_reg.id = 1;
    status_reg.kind = talos::event::SourceKind::SENDER;
    status_reg.name = "/hw/status";
    status_reg.message_bytes = sizeof(std::uint16_t);
    status_reg.schema = {
        "bridge.BridgeStatus", 1,
        std::string{reinterpret_cast<const char*>(kMinimalBridgeStatusBfbs),
                    sizeof(kMinimalBridgeStatusBfbs)}};
    talos::event::Registration state_reg;
    state_reg.id = 2;
    state_reg.kind = talos::event::SourceKind::SENDER;
    state_reg.name = hw::kStateTopic;
    state_reg.message_bytes = sizeof(hw::Packet);
    state_reg.schema = {std::string{D::kName}, D::kHash, {}};

    talos::event::log::LogWriter writer{
        directory_ + "/hardware_node.tlog", "hardware_node",
        talos::event::log::LogWriterOptions{1 << 14, 1, false}, 42};
    writer.start({status_reg, state_reg},
                 talos::event::MonotonicTime::from_nanos(1'000'000'000));
    talos::event::Context ctx{};
    ctx.kind = talos::event::EventKind::SEND;
    ctx.event_time = talos::event::MonotonicTime::from_nanos(1'000'000'000);
    ctx.now = ctx.event_time;
    ctx.publish_time = ctx.event_time;
    writer.send(ctx, 1, std::as_bytes(std::span{&bridge_version, 1}), 0, 1);
    hw::Packet state{};
    state.size =
        static_cast<std::uint32_t>(hw::Encode(hw::State{}, state.data));
    ctx.dispatch_index = 2;
    writer.send(ctx, 2, std::as_bytes(std::span{&state, 1}), 0, 1);
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  };

  // Bridge matches this build; manifest names another: warns once naming both
  // and decodes under the bridge's version.
  {
    std::ofstream{directory_ + "/manifest.json"}
        << "{\n  \"session_id\": 42,\n  \"hardware_protocol_version\": "
        << kOther << ",\n  \"nodes\": []\n}\n";
    write_bridge(kBuild);
    const RunResult res =
        RunSplit({"--format", "jsonl", "--node", "hardware_node"});
    const auto err_lines = SplitLines(res.err);
    const std::string expected_disagreement =
        "log_dump: warning: the session's bridge recorded hardware protocol " +
        std::to_string(kBuild) +
        " (/hw/status protocol_version_id), its launcher " +
        std::to_string(kOther) +
        " (manifest.json hardware_protocol_version): read under the bridge's";
    EXPECT_EQ(
        std::count(err_lines.begin(), err_lines.end(), expected_disagreement),
        1)
        << res.err;
    EXPECT_EQ(res.err.find("hardware packets not decoded"), std::string::npos)
        << res.err;
    EXPECT_NE(res.out.find("\"hardware\":{\"type\":\"state\""),
              std::string::npos)
        << res.out;
  }

  // Bridge names another version; manifest matches this build: warns once
  // naming both and refuses hardware packets under the bridge's version.
  {
    std::ofstream{directory_ + "/manifest.json"}
        << "{\n  \"session_id\": 42,\n  \"hardware_protocol_version\": "
        << kBuild << ",\n  \"nodes\": []\n}\n";
    write_bridge(kOther);
    const RunResult res =
        RunSplit({"--format", "jsonl", "--node", "hardware_node"});
    const auto err_lines = SplitLines(res.err);
    const std::string expected_disagreement =
        "log_dump: warning: the session's bridge recorded hardware protocol " +
        std::to_string(kOther) +
        " (/hw/status protocol_version_id), its launcher " +
        std::to_string(kBuild) +
        " (manifest.json hardware_protocol_version): read under the bridge's";
    EXPECT_EQ(
        std::count(err_lines.begin(), err_lines.end(), expected_disagreement),
        1)
        << res.err;
    EXPECT_NE(res.err.find("hardware packets not decoded"), std::string::npos)
        << res.err;
    EXPECT_EQ(res.out.find("\"hardware\":"), std::string::npos) << res.out;
  }
}

// REQ-OPS-15: --format summary streams records in bounded memory proportional
// to topics and fields, not record count: a 50x larger log on the same topics
// folds every record with peak RSS staying within a small constant slack of
// the 1k-record run.
TEST_F(LogDumpTest, SummaryMemoryIsBoundedAcrossRecordCounts) {
  const auto write_scaled = [&](const std::string& dir, std::uint32_t count) {
    std::filesystem::create_directories(dir);
    using D = talos::Descriptor<continuity_test::Count>;
    talos::event::Registration source;
    source.id = 1;
    source.kind = talos::event::SourceKind::TIMER;
    source.name = "/alpha/state";
    source.message_bytes = sizeof(continuity_test::Count);
    source.schema = {std::string{D::kName}, D::kHash,
                     std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                                 D::kBfbs.size()}};
    talos::event::log::LogWriter writer{
        dir + "/alpha.tlog", "alpha",
        talos::event::log::LogWriterOptions{1u << 20, 4, false}, 42};
    writer.start({source},
                 talos::event::MonotonicTime::from_nanos(1'000'000'000));
    for (std::uint32_t i = 0; i < count; ++i) {
      const continuity_test::Count value{i, 1};
      talos::event::Context ctx{};
      ctx.kind = talos::event::EventKind::TIMER;
      ctx.source_id = 1;
      ctx.dispatch_index = static_cast<std::uint64_t>(i) + 1;
      ctx.event_time = talos::event::MonotonicTime::from_nanos(
          1'000'000'000 + static_cast<std::int64_t>(i) * 1'000'000);
      ctx.now = ctx.event_time;
      ctx.publish_time = ctx.event_time;
      ctx.sequence = ctx.dispatch_index;
      writer.dispatch(ctx, std::as_bytes(std::span{&value, 1}));
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
    ASSERT_FALSE(writer.incomplete());
  };

  constexpr std::uint32_t kSmall = 1'000;
  constexpr std::uint32_t kLarge = 50'000;
  const std::string small_dir = directory_ + "/summary_1k";
  const std::string large_dir = directory_ + "/summary_50k";
  write_scaled(small_dir, kSmall);
  write_scaled(large_dir, kLarge);

  const RunResult small_res =
      RunSplit({"--format", "summary", small_dir}, true, false);
  const RunResult large_res =
      RunSplit({"--format", "summary", large_dir}, true, false);
  const auto large_lines = SplitLines(large_res.out);
  EXPECT_TRUE(HasLine(large_lines, "  records alpha TIMER 50000"))
      << large_res.out;
  EXPECT_TRUE(HasLine(large_lines, "  range run_index 0 49999"))
      << large_res.out;
  EXPECT_TRUE(
      HasLine(large_lines, "  latest {run_index: 49999, sequence_count: 1}"))
      << large_res.out;
  EXPECT_TRUE(large_res.err.empty()) << large_res.err;

  // Under ASan/TSan, deferred heap quarantine and shadow pages grow with
  // transient allocations even when live memory is O(1).
#if !defined(__SANITIZE_ADDRESS__) && !defined(__SANITIZE_THREAD__)
  bool sanitized = false;
#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || \
    __has_feature(memory_sanitizer)
  sanitized = true;
#endif
#endif
  if (!sanitized) {
    // 50k records is ~3.6 MiB on disk (and >10 MiB if buffered decoded);
    // streaming TopicSummary keeps peak RSS growth well under 2 MiB.
    constexpr std::uint64_t kMaxGrowthBytes = 2u << 20;
    EXPECT_GT(small_res.peak_rss_bytes, 0u);
    EXPECT_LE(large_res.peak_rss_bytes,
              small_res.peak_rss_bytes + kMaxGrowthBytes)
        << "1k RSS=" << small_res.peak_rss_bytes
        << " B, 50k RSS=" << large_res.peak_rss_bytes << " B";
  }
#endif
}

TEST_F(LogDumpTest, RejectsInvalidOptions) {
  for (const auto& args :
       std::vector<std::vector<std::string>>{{"--unknown"},
                                             {"--format", "xml"},
                                             {"--since-ms", "nan"},
                                             {"--until-ms", "inf"},
                                             {"--since-ms", "bad"},
                                             {"--until-ms", "10oops"}}) {
    Run(args, false);
  }
}

// REQ-OPS-15, REQ-OPS-37: hostile .tlog files with extreme int64 timestamps,
// unclosed/deeply-nested braces in schema enum names, and non-UTF-8 bytes in
// process/topic names must not crash, trigger signed overflow UB, or emit
// invalid UTF-8 JSONL.
TEST_F(LogDumpTest, HandlesHostileTimestampsUnclosedBracesAndNonUtf8) {
  const std::string hostile_dir = directory_ + "/hostile";
  std::filesystem::create_directories(hostile_dir);

  // Build a verified .bfbs struct with an enum whose value name holds unclosed
  // deeply nested braces ("{{{..."), exercising DecodedFields recursion and
  // missing closing brace handling.
  flatbuffers::FlatBufferBuilder builder;
  const std::string hostile_enum_val(256, '{');
  const auto ev_name = builder.CreateString(hostile_enum_val);
  reflection::EnumValBuilder evb(builder);
  evb.add_name(ev_name);
  evb.add_value(0);
  std::vector<flatbuffers::Offset<reflection::EnumVal>> evs{evb.Finish()};
  const auto evs_vec = builder.CreateVectorOfSortedTables(&evs);
  reflection::TypeBuilder etb(builder);
  etb.add_base_type(reflection::BaseType::UByte);
  const auto enum_underlying = etb.Finish();
  const auto enum_name = builder.CreateString("hostile.Braces");
  reflection::EnumBuilder eb(builder);
  eb.add_name(enum_name);
  eb.add_values(evs_vec);
  eb.add_underlying_type(enum_underlying);
  std::vector<flatbuffers::Offset<reflection::Enum>> enums{eb.Finish()};
  const auto enums_vec = builder.CreateVectorOfSortedTables(&enums);

  const auto field_name = builder.CreateString("mode");
  reflection::TypeBuilder ftb(builder);
  ftb.add_base_type(reflection::BaseType::UByte);
  ftb.add_index(0);
  const auto field_type = ftb.Finish();
  reflection::FieldBuilder fb(builder);
  fb.add_name(field_name);
  fb.add_type(field_type);
  fb.add_id(0);
  fb.add_offset(0);
  std::vector<flatbuffers::Offset<reflection::Field>> fields{fb.Finish()};
  const auto fields_vec = builder.CreateVectorOfSortedTables(&fields);

  const auto obj_name = builder.CreateString("hostile.Status");
  reflection::ObjectBuilder ob(builder);
  ob.add_name(obj_name);
  ob.add_fields(fields_vec);
  ob.add_is_struct(true);
  ob.add_bytesize(1);
  std::vector<flatbuffers::Offset<reflection::Object>> objs{ob.Finish()};
  const auto objs_vec = builder.CreateVectorOfSortedTables(&objs);

  reflection::SchemaBuilder sb(builder);
  sb.add_objects(objs_vec);
  sb.add_enums(enums_vec);
  reflection::FinishSchemaBuffer(builder, sb.Finish());
  const std::string bfbs{
      reinterpret_cast<const char*>(builder.GetBufferPointer()),
      builder.GetSize()};

  talos::event::Registration source;
  source.id = 1;
  source.kind = talos::event::SourceKind::SENDER;
  // Role is "status" so AlertsJson runs DecodedFields::Walk in jsonl mode, and
  // contains an invalid UTF-8 byte (0xff) to verify JSON UTF-8 escaping.
  source.name = std::string{"/bad\xff/status"};
  source.message_bytes = 1;
  source.schema = {"hostile.Status", 0x123456789abcdef0ULL, bfbs};

  talos::event::log::LogWriter writer{
      hostile_dir + "/hostile.tlog", std::string{"proc\xfe"},
      talos::event::log::LogWriterOptions{1 << 16, 1, false}, 99};
  writer.start({source}, talos::event::MonotonicTime::from_nanos(0));
  const std::array<std::byte, 1> payload{std::byte{0}};
  talos::event::Context ctx{};
  ctx.kind = talos::event::EventKind::SEND;
  ctx.source_id = 1;
  ctx.dispatch_index = 1;
  // Extreme int64 timestamps: INT64_MIN event_time vs 0 now/publish overflows
  // signed int64 subtraction if not widened to unsigned difference.
  ctx.event_time = talos::event::MonotonicTime::from_nanos(
      std::numeric_limits<std::int64_t>::min());
  ctx.now = talos::event::MonotonicTime::from_nanos(0);
  ctx.publish_time = talos::event::MonotonicTime::from_nanos(
      std::numeric_limits<std::int64_t>::max());
  ctx.sequence = 1;
  writer.dispatch(ctx, payload);
  writer.flush();
  ASSERT_FALSE(writer.failed()) << writer.error();

  const RunResult jsonl =
      RunSplit({"--format", "jsonl", hostile_dir}, true, false);
  EXPECT_NE(jsonl.out.find("\"process\":\"proc\\u00fe\""), std::string::npos)
      << jsonl.out;
  EXPECT_NE(jsonl.out.find("\"topic\":\"/bad\\u00ff/status\""),
            std::string::npos)
      << jsonl.out;
  EXPECT_EQ(jsonl.out.find('\xff'), std::string::npos);
  EXPECT_EQ(jsonl.out.find('\xfe'), std::string::npos);

  const RunResult summary =
      RunSplit({"--format", "summary", hostile_dir}, true, false);
  EXPECT_NE(summary.out.find("records proc"), std::string::npos) << summary.out;
}

}  // namespace
