// The launcher owns every declared topic (launcher.h CreateSessionSegments):
// a publisher killed with SIGKILL is restarted (max_restarts) and its
// subscriber keeps receiving on the same segment, and a publisher's clean
// exit leaves the topic in place for whoever publishes next. Real nodes, real
// shared memory: continuity_source and continuity_sink in testdata/.

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

#include "continuity_test/continuity_message_generated.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/events/context.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/launcher/launcher.h"
#include "talOS/memory/shared_memory_ptr.h"
#include "talOS/rtms/rtms.h"

namespace talos::launcher {
namespace {

using continuity_test::Count;
using namespace std::chrono_literals;

constexpr std::string_view kTopic = "/continuity_source/state";
// talos::kMessageAlignment: every node topic's slot alignment.
constexpr std::size_t kNodeAlignment = 8;

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
  return ::testing::TempDir() + "/talos_continuity_" +
         std::to_string(::getpid()) + "_" + std::string{name};
}

bool SegmentExists(std::string_view topic) {
  const int fd = ::shm_open(ShmObjectName(topic).c_str(), O_RDONLY, 0);
  if (fd < 0) return false;
  ::close(fd);
  return true;
}

// True once `path` is a complete log: its run ended and wrote EXIT.
bool LogFinished(const std::string& path) {
  if (!std::filesystem::exists(path)) return false;
  try {
    event::log::LogReader reader{path};
    event::log::LogReader::Record record;
    while (reader.next(record)) {
      if (record.header.kind ==
          static_cast<std::uint16_t>(event::EventKind::EXIT)) {
        return true;
      }
    }
  } catch (const std::exception&) {
  }
  return false;
}

// The run_index of every kTopic message `path`'s node was handed.
std::multiset<std::uint32_t> RunsReceived(const std::string& path) {
  event::log::LogReader reader{path};
  std::uint16_t topic_id = 0xFFFF;
  for (const auto& registration : reader.manifest()) {
    if (registration.name == kTopic) topic_id = registration.id;
  }
  EXPECT_NE(topic_id, 0xFFFF) << path << " has no " << kTopic;
  std::multiset<std::uint32_t> runs;
  event::log::LogReader::Record record;
  while (reader.next(record)) {
    if (record.header.kind !=
            static_cast<std::uint16_t>(event::EventKind::MESSAGE) ||
        record.header.source_id != topic_id) {
      continue;
    }
    Count count{};
    EXPECT_EQ(record.payload.size(), sizeof count);
    std::memcpy(&count, record.payload.data(), sizeof count);
    runs.insert(count.run_index());
  }
  return runs;
}

// One node's object in manifest.json, from its name to the closing brace.
std::string ManifestEntry(const std::string& manifest, std::string_view name) {
  const std::size_t start =
      manifest.find("\"name\": \"" + std::string{name} + "\"");
  if (start == std::string::npos) return {};
  return manifest.substr(start, manifest.find('}', start) - start);
}

TEST(SessionSegments, SubscribersOutliveAKilledAndACleanlyExitedPublisher) {
  const std::string out_dir = TempPath("out");
  std::filesystem::remove_all(out_dir);
  const std::string marker = TempPath("marker");
  std::filesystem::remove(marker);
  const std::string config = TempPath("robot.toml");
  std::ofstream{config} << R"([robot]
name = "continuity"
period_us = 5000
max_restarts = 2

[subsystems.continuity_source]
node = "//talOS/launcher:continuity_source"
marker = ")" << marker << R"("

[subsystems.continuity_source.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.continuity_sink]
node = "//talOS/launcher:continuity_sink"

[subsystems.continuity_sink.motors.m2]
type = "TalonFX"
bus = "rio"
can_id = 2
)";

  LauncherOptions options;
  options.config_path = config;
  options.output_dir = out_dir;
  options.simulation = true;
  options.duration_s = 4;
  options.describe_timeout_ms = 4000;
  options.binary_overrides["//talOS/bridge:hardware_node"] = "/usr/bin/true";
  options.binary_overrides["//talOS/launcher:continuity_source"] =
      Runfile("TALOS_SOURCE_NODE");
  options.binary_overrides["//talOS/launcher:continuity_sink"] =
      Runfile("TALOS_SINK_NODE");

  int status = -1;
  std::thread session{[&] { status = Launcher{options}.Run(); }};

  // The restart's clean exit: its log is complete.
  const std::string restart_log = out_dir + "/continuity_source.restart1.tlog";
  const auto give_up = std::chrono::steady_clock::now() + 3s;
  while (!LogFinished(restart_log) &&
         std::chrono::steady_clock::now() < give_up) {
    std::this_thread::sleep_for(20ms);
  }
  if (!LogFinished(restart_log)) {
    session.join();
    FAIL() << "the restarted source never exited";
  }

  // The source exited cleanly; the topic is still there, and the sink still
  // reads it: a standalone publisher (this test) reaches it.
  EXPECT_TRUE(SegmentExists(kTopic));
  {
    RTMSQueue publisher{kTopic,
                        sizeof(Count),
                        kNodeAlignment,
                        schema::Of<Count>(),
                        MAX_SLOTS,
                        {.overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
                         .read_mode = ReadMode::SEQUENCE,
                         .publisher = true}};
    for (std::uint32_t i = 0; i < 10; ++i) {
      const Count count{2, i};
      publisher.write(std::as_bytes(std::span<const Count, 1>{&count, 1}));
      std::this_thread::sleep_for(20ms);
    }
  }
  session.join();

  EXPECT_EQ(status, 0);
  // The session's end removes what it created.
  EXPECT_FALSE(SegmentExists(kTopic));

  const std::string manifest = [&] {
    std::ifstream in{out_dir + "/manifest.json"};
    return std::string{std::istreambuf_iterator<char>(in),
                       std::istreambuf_iterator<char>()};
  }();
  const std::string source = ManifestEntry(manifest, "continuity_source");
  EXPECT_NE(source.find("\"restart_count\": 1"), std::string::npos) << source;
  EXPECT_NE(source.find("\"exit_code\": 0"), std::string::npos) << source;
  const std::string sink = ManifestEntry(manifest, "continuity_sink");
  EXPECT_NE(sink.find("\"restart_count\": 0"), std::string::npos) << sink;

  // One sink process, one log, every publisher's messages in it: the first
  // run's, the restart's after a SIGKILL, and this test's after a clean exit.
  const std::multiset<std::uint32_t> runs =
      RunsReceived(out_dir + "/continuity_sink.tlog");
  EXPECT_GT(runs.count(0), 0u);
  EXPECT_GT(runs.count(1), 0u);
  EXPECT_GT(runs.count(2), 0u);

  std::filesystem::remove_all(out_dir);
  std::filesystem::remove(marker);
  std::filesystem::remove(config);
}

}  // namespace
}  // namespace talos::launcher
