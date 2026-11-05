#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

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
#include "talOS/events/context.h"
#include "talOS/events/log/log_writer.h"

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
    ASSERT_NE(std::getenv("TALOS_LOG_DUMP"), nullptr);
    binary_ = runfiles->Rlocation(std::getenv("TALOS_LOG_DUMP"));
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

TEST_F(LogDumpTest, DefaultTextAndNodeFilters) {
  const auto text = Run({});
  EXPECT_NE(text.find("merged log dump: 2 files, 4 total records"),
            std::string::npos);
  EXPECT_NE(text.find("summary: 4 records across 2 processes"),
            std::string::npos);
  EXPECT_NE(text.find("continuity_test.Count"), std::string::npos);
  const auto alpha = Run({"--node", "alpha"});
  EXPECT_NE(alpha.find("summary: 2 records across 1 processes"),
            std::string::npos);
  EXPECT_EQ(alpha.find("[beta"), std::string::npos);
  EXPECT_NE(Run({"--node", "alpha,beta"}).find("summary: 4 records"),
            std::string::npos);
  EXPECT_NE(
      Run({"--node", "alpha", "--node", "beta"}).find("summary: 4 records"),
      std::string::npos);
}

TEST_F(LogDumpTest, TopicFiltersAndStableInclusiveWindow) {
  EXPECT_NE(Run({"--topic", "/alpha/state"}).find("summary: 2 records"),
            std::string::npos);
  EXPECT_NE(Run({"--topic", "/beta/*"}).find("summary: 2 records"),
            std::string::npos);
  EXPECT_NE(Run({"--topic", "/alpha/state,/beta/*"}).find("summary: 4 records"),
            std::string::npos);
  EXPECT_NE(Run({"--topic", "/alpha/state", "--topic", "/beta/*"})
                .find("summary: 4 records"),
            std::string::npos);
  const auto window =
      Run({"--node", "beta", "--since-ms", "10", "--until-ms", "10"});
  EXPECT_NE(window.find("summary: 1 records"), std::string::npos);
  EXPECT_NE(window.find("10.000ms"), std::string::npos);
  EXPECT_NE(
      Run({"--since-ms", "20", "--until-ms", "30"}).find("summary: 2 records"),
      std::string::npos);
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
  EXPECT_NE(json.find("\"event_ms\":10,\"now_ms\":10,\"sequence\":1,\"aux\":0,"
                      "\"payload_bytes\":8"),
            std::string::npos);
  EXPECT_NE(json.find("\"schema\":\"continuity_test.Count\",\"decoded\":\"{run_"
                      "index: 7, sequence_count: 9}\""),
            std::string::npos);
  EXPECT_EQ(json.find("summary"), std::string::npos);
  const auto csv =
      Run({"--format", "csv", "--node", "beta", "--until-ms", "10"});
  EXPECT_TRUE(
      csv.starts_with("process,dispatch_index,kind,source_id,topic,event_ms,"
                      "now_ms,sequence,aux,payload_bytes,schema,decoded\r\n"));
  EXPECT_NE(csv.find("beta,1,TIMER,1,\"/beta/"
                     "quote,\"\"\n\",10,10,1,0,8,continuity_test.Count,"),
            std::string::npos);
  EXPECT_NE(csv.find("\"{run_index: 7, sequence_count: 9}\"\r\n"),
            std::string::npos);
  EXPECT_EQ(csv.find("summary"), std::string::npos);
}

TEST_F(LogDumpTest, RejectsInvalidOptions) {
  for (const auto& args :
       std::vector<std::vector<std::string>>{{"--unknown"},
                                             {"--format", "xml"},
                                             {"--since-ms", "nan"},
                                             {"--until-ms", "inf"},
                                             {"--since-ms", "bad"},
                                             {"--until-ms", "10oops"}}) {
    const auto output = Run(args, false);
    EXPECT_NE(output.find("log_dump:"), std::string::npos);
    EXPECT_NE(output.find("usage:"), std::string::npos);
  }
}

}  // namespace
