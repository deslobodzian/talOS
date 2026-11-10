#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "continuity_test/continuity_message_descriptor.h"
#include "continuity_test/continuity_message_generated.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/context.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"

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

  void WriteLog(const std::string& process, const std::string& topic,
                const std::vector<int>& times) {
    using D = talos::Descriptor<continuity_test::Count>;
    talos::event::Registration source;
    source.id = 1;
    source.kind = talos::event::SourceKind::TIMER;
    source.name = topic;
    source.message_bytes = sizeof(continuity_test::Count);
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
      const continuity_test::Count value{7, 9};
      writer.dispatch(ctx, std::as_bytes(std::span{&value, 1}));
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }

  std::string Run(std::vector<std::string> args, bool success = true) {
    args.insert(args.begin(), binary_);
    args.push_back(directory_);
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
  unsetenv("BUILD_WORKING_DIRECTORY");
  EXPECT_NE(json.find("\"payload_hex\":\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"hardware\":{\"type\":\"command\""), std::string::npos)
      << json;
  EXPECT_NE(json.find("\"name\":\"flywheel\",\"mode\":\"voltage\",\"slot\":0,"
                      "\"demand\":6,"),
            std::string::npos)
      << json;
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

}  // namespace
