#include "talOS/launcher/launcher.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "talOS/events/log/log_writer.h"

namespace talos::launcher {
namespace {

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_launcher_test_" +
         std::to_string(::getpid()) + "_" + std::string{name};
}

event::Manifest MakeTestManifest(std::string_view name) {
  event::Manifest manifest;
  event::Registration reg;
  reg.id = 1;
  reg.kind = event::SourceKind::TIMER;
  reg.name = std::string{name};
  reg.message_bytes = 8;
  reg.period_ns = 1'000'000;
  manifest.push_back(reg);
  return manifest;
}

void WriteTestLog(const std::string& path, const std::string& process_name,
                  uint64_t session_id,
                  const std::vector<int64_t>& event_times_ns) {
  event::log::LogWriter writer{
      path, process_name,
      event::log::LogWriterOptions{4096, 1, /*background=*/false}, session_id};
  const auto manifest = MakeTestManifest(process_name + "_src");
  writer.start(manifest, event::MonotonicTime::from_nanos(event_times_ns.empty() ? 0 : event_times_ns.front()));

  for (std::size_t i = 0; i < event_times_ns.size(); ++i) {
    event::Context ctx{};
    ctx.kind = event::EventKind::TIMER;
    ctx.source_id = 1;
    ctx.dispatch_index = i + 1;
    ctx.event_time = event::MonotonicTime::from_nanos(event_times_ns[i]);
    ctx.now = event::MonotonicTime::from_nanos(event_times_ns[i] + 100);
    ctx.sequence = i + 1;

    uint64_t payload_val = i;
    std::span<const std::byte> payload{reinterpret_cast<const std::byte*>(&payload_val), sizeof(payload_val)};
    writer.dispatch(ctx, payload);
  }

  writer.flush();
  ASSERT_FALSE(writer.failed()) << writer.error();
}

TEST(LauncherTest, SessionIdGenerationIsUniqueAndNonZero) {
  uint64_t id1 = GenerateSessionId();
  uint64_t id2 = GenerateSessionId();
  uint64_t id3 = GenerateSessionId();

  EXPECT_NE(id1, 0u);
  EXPECT_NE(id2, 0u);
  EXPECT_NE(id3, 0u);

  EXPECT_NE(id1, id2);
  EXPECT_NE(id2, id3);
  EXPECT_NE(id1, id3);
}

TEST(LauncherTest, DiscoverNodesFromRobotToml) {
  const auto robot_cfg = config::ParseRobotConfig("talOS/configuration/robot.toml");
  LauncherOptions options;
  auto nodes = DiscoverNodes(robot_cfg, options);

  ASSERT_GE(nodes.size(), 2u);

  bool found_hw = false;
  bool found_drivetrain = false;
  bool found_shooter = false;
  bool found_driver_station = false;

  for (const auto& n : nodes) {
    if (n.name == "hardware_node") {
      found_hw = true;
      EXPECT_EQ(n.target, "//talOS/hardware:hardware_node");
    } else if (n.name == "drivetrain") {
      found_drivetrain = true;
      EXPECT_EQ(n.target, "//talOS/drivetrain:node");
    } else if (n.name == "shooter") {
      found_shooter = true;
      EXPECT_EQ(n.target, "//talOS/shooter:node");
    } else if (n.name == "driver_station") {
      found_driver_station = true;
      EXPECT_EQ(n.target, "//talOS/driver_station:node");
    }
  }

  EXPECT_TRUE(found_hw);
  EXPECT_TRUE(found_drivetrain);
  EXPECT_TRUE(found_shooter);
  EXPECT_TRUE(found_driver_station);
}

TEST(LauncherTest, MergedLogsMonotonicOrder) {
  const std::string path_a = TempPath("log_a.tlog");
  const std::string path_b = TempPath("log_b.tlog");
  const uint64_t session_id = 0xCAFEBABE12345678ULL;

  // Proc A writes events at 1ms, 3ms, 5ms
  WriteTestLog(path_a, "proc_a", session_id, {1'000'000, 3'000'000, 5'000'000});

  // Proc B writes events at 2ms, 4ms, 6ms
  WriteTestLog(path_b, "proc_b", session_id, {2'000'000, 4'000'000, 6'000'000});

  std::vector<MergedRecord> merged;
  std::string error;
  ASSERT_TRUE(MergeLogs({path_a, path_b}, merged, error)) << error;

  ASSERT_EQ(merged.size(), 6u);

  // Check monotonic order and interleaving
  EXPECT_EQ(merged[0].process_name, "proc_a");
  EXPECT_EQ(merged[0].header.event_time_ns, 1'000'000);

  EXPECT_EQ(merged[1].process_name, "proc_b");
  EXPECT_EQ(merged[1].header.event_time_ns, 2'000'000);

  EXPECT_EQ(merged[2].process_name, "proc_a");
  EXPECT_EQ(merged[2].header.event_time_ns, 3'000'000);

  EXPECT_EQ(merged[3].process_name, "proc_b");
  EXPECT_EQ(merged[3].header.event_time_ns, 4'000'000);

  EXPECT_EQ(merged[4].process_name, "proc_a");
  EXPECT_EQ(merged[4].header.event_time_ns, 5'000'000);

  EXPECT_EQ(merged[5].process_name, "proc_b");
  EXPECT_EQ(merged[5].header.event_time_ns, 6'000'000);

  std::remove(path_a.c_str());
  std::remove(path_b.c_str());
}

TEST(LauncherTest, MergedLogsRejectsSessionIdMismatch) {
  const std::string path_a = TempPath("mismatch_a.tlog");
  const std::string path_b = TempPath("mismatch_b.tlog");

  WriteTestLog(path_a, "proc_a", 1001ULL, {1'000'000});
  WriteTestLog(path_b, "proc_b", 2002ULL, {2'000'000});

  std::vector<MergedRecord> merged;
  std::string error;
  bool ok = MergeLogs({path_a, path_b}, merged, error);

  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("session_id mismatch"), std::string::npos);

  std::remove(path_a.c_str());
  std::remove(path_b.c_str());
}

TEST(LauncherTest, SessionManifestJsonFormatting) {
  SessionManifest manifest;
  manifest.session_id = 987654321ULL;
  manifest.config_path = "talOS/configuration/robot.toml";
  manifest.output_dir = "/tmp/test_dir";
  manifest.simulation = true;
  manifest.start_wall_ns = 1'000'000'000LL;
  manifest.end_wall_ns = 2'000'000'000LL;

  NodeProcessInfo node1;
  node1.name = "drivetrain";
  node1.target = "//talOS/drivetrain:node";
  node1.binary_path = "bazel-bin/talOS/drivetrain/node";
  node1.log_path = "/tmp/test_dir/drivetrain.tlog";
  node1.pid = 1234;
  node1.exit_code = 0;
  node1.record_count = 50;
  node1.failed = false;

  manifest.nodes.push_back(node1);

  std::string json = manifest.ToJson();
  EXPECT_NE(json.find("\"session_id\": 987654321"), std::string::npos);
  EXPECT_NE(json.find("\"name\": \"drivetrain\""), std::string::npos);
  EXPECT_NE(json.find("\"pid\": 1234"), std::string::npos);
  EXPECT_NE(json.find("\"exit_code\": 0"), std::string::npos);
  EXPECT_NE(json.find("\"record_count\": 50"), std::string::npos);
}

TEST(LauncherTest, RunsChildProcessesAndWritesManifest) {
  const std::string out_dir = TempPath("launcher_run_out");
  std::filesystem::create_directories(out_dir);

  LauncherOptions options;
  options.output_dir = out_dir;
  options.session_id = 0x55555555ULL;
  options.duration_s = 1;

  // Use /usr/bin/true (or /bin/echo) for all nodes so the test executes instantly and cleanly
  std::string mock_bin = ::access("/usr/bin/true", X_OK) == 0 ? "/usr/bin/true" : "/bin/echo";
  options.binary_overrides["//talOS/hardware:hardware_node"] = mock_bin;
  options.binary_overrides["//talOS/drivetrain:node"] = mock_bin;
  options.binary_overrides["//talOS/shooter:node"] = mock_bin;
  options.binary_overrides["//talOS/driver_station:node"] = mock_bin;

  Launcher launcher{options};
  int code = launcher.Run();
  EXPECT_EQ(code, 0);

  std::string manifest_path = out_dir + "/manifest.json";
  EXPECT_TRUE(std::filesystem::exists(manifest_path));

  std::ifstream mf(manifest_path);
  std::string content((std::istreambuf_iterator<char>(mf)),
                      std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("\"session_id\":"), std::string::npos);
  EXPECT_NE(content.find("\"nodes\":"), std::string::npos);

  std::filesystem::remove_all(out_dir);
}

}  // namespace
}  // namespace talos::launcher
