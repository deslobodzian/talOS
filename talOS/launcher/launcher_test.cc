#include "talOS/launcher/launcher.h"

#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "talOS/events/log/log_writer.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"

namespace talos::launcher {
namespace {

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_launcher_test_" +
         std::to_string(::getpid()) + "_" + std::string{name};
}

// A framework-owned fixture: the launcher's discovery logic is robot-agnostic,
// so its tests must not read any particular robot's configuration.
constexpr std::string_view kFixtureToml = R"(
[robot]
name = "launcher_fixture"
period_us = 5000
sim_gateway = "//example/sim:gateway"

[subsystems.alpha]
node = "//example/alpha:node"
period_us = 5000

[subsystems.alpha.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.beta]
node = "//example/beta:node"
period_us = 10000

[subsystems.beta.motors.m2]
type = "TalonFX"
bus = "rio"
can_id = 2
)";

inline std::string WriteFixtureToml() {
  const std::string path = TempPath("launcher_fixture_toml") + ".toml";
  std::ofstream out{path};
  out << kFixtureToml;
  return path;
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
  writer.start(manifest,
               event::MonotonicTime::from_nanos(
                   event_times_ns.empty() ? 0 : event_times_ns.front()));

  for (std::size_t i = 0; i < event_times_ns.size(); ++i) {
    event::Context ctx{};
    ctx.kind = event::EventKind::TIMER;
    ctx.source_id = 1;
    ctx.dispatch_index = i + 1;
    ctx.event_time = event::MonotonicTime::from_nanos(event_times_ns[i]);
    ctx.now = event::MonotonicTime::from_nanos(event_times_ns[i] + 100);
    ctx.sequence = i + 1;

    uint64_t payload_val = i;
    std::span<const std::byte> payload{
        reinterpret_cast<const std::byte*>(&payload_val), sizeof(payload_val)};
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
  const auto robot_cfg = config::ParseRobotConfigString(kFixtureToml);
  LauncherOptions options;
  auto nodes = DiscoverNodes(robot_cfg, options);

  ASSERT_GE(nodes.size(), 2u);

  bool found_hw = false;
  bool found_alpha = false;
  bool found_beta = false;

  for (const auto& n : nodes) {
    if (n.name == "hardware_node") {
      found_hw = true;
      EXPECT_EQ(n.target, "//talOS/bridge:hardware_node");
    } else if (n.name == "alpha") {
      found_alpha = true;
      EXPECT_EQ(n.target, "//example/alpha:node");
    } else if (n.name == "beta") {
      found_beta = true;
      EXPECT_EQ(n.target, "//example/beta:node");
    }
  }

  EXPECT_TRUE(found_hw);
  EXPECT_TRUE(found_alpha);
  EXPECT_TRUE(found_beta);
}

TEST(LauncherTest, SimGatewayComesFromRobotConfigNotTheFramework) {
  LauncherOptions options;
  options.simulation = true;
  options.start_sim_gateway = true;

  auto with_gw =
      DiscoverNodes(config::ParseRobotConfigString(kFixtureToml), options);
  EXPECT_EQ(
      std::count_if(with_gw.begin(), with_gw.end(),
                    [](const NodeSpec& n) { return n.name == "sim_gateway"; }),
      1);

  // No [robot] sim_gateway key: the launcher must not invent a robot target.
  constexpr std::string_view kNoGateway = R"(
[robot]
name = "no_gateway"
period_us = 5000

[subsystems.alpha]
node = "//example/alpha:node"

[subsystems.alpha.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
)";
  auto without_gw =
      DiscoverNodes(config::ParseRobotConfigString(kNoGateway), options);
  EXPECT_EQ(
      std::count_if(without_gw.begin(), without_gw.end(),
                    [](const NodeSpec& n) { return n.name == "sim_gateway"; }),
      0);
}

TEST(LauncherTest, StudioBridgeNeedsFlagAndSim) {
  const auto has_bridge = [](const std::vector<NodeSpec>& nodes) {
    return std::count_if(nodes.begin(), nodes.end(), [](const NodeSpec& n) {
      return n.name == "studio_bridge";
    });
  };
  // Neither flag nor sim: the bridge stays outside the session, as before.
  EXPECT_EQ(
      has_bridge(DiscoverNodes(config::ParseRobotConfigString(kFixtureToml),
                               LauncherOptions{})),
      0);
  // Flag without sim (a deployed session): no webroot, no browser, no bridge.
  LauncherOptions flag_only;
  flag_only.start_studio_bridge = true;
  EXPECT_EQ(has_bridge(DiscoverNodes(
                config::ParseRobotConfigString(kFixtureToml), flag_only)),
            0);
  // Sim without the flag (--no-studio): the session runs headless.
  LauncherOptions sim_only;
  sim_only.simulation = true;
  EXPECT_EQ(has_bridge(DiscoverNodes(
                config::ParseRobotConfigString(kFixtureToml), sim_only)),
            0);
  // Both: declared with the framework default target.
  LauncherOptions both;
  both.simulation = true;
  both.start_studio_bridge = true;
  auto nodes =
      DiscoverNodes(config::ParseRobotConfigString(kFixtureToml), both);
  ASSERT_EQ(has_bridge(nodes), 1);
  for (const auto& n : nodes) {
    if (n.name == "studio_bridge") {
      EXPECT_EQ(n.target, "//studio/bridge:studio_bridge");
    }
  }
}

TEST(LauncherTest, StudioBridgeTargetFromStudioTable) {
  constexpr std::string_view kCustomStudio = R"(
[robot]
name = "custom_studio"
period_us = 5000

[studio]
bridge = "//example/studio:bridge"
http_port = 8080

[subsystems.alpha]
node = "//example/alpha:node"

[subsystems.alpha.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
)";
  LauncherOptions options;
  options.simulation = true;
  options.start_studio_bridge = true;
  auto nodes =
      DiscoverNodes(config::ParseRobotConfigString(kCustomStudio), options);
  ASSERT_EQ(std::count_if(
                nodes.begin(), nodes.end(),
                [](const NodeSpec& n) { return n.name == "studio_bridge"; }),
            1);
  for (const auto& n : nodes) {
    if (n.name == "studio_bridge") {
      EXPECT_EQ(n.target, "//example/studio:bridge");
    }
  }
  EXPECT_EQ(EffectiveStudioHttpPort(
                config::ParseRobotConfigString(kCustomStudio), options),
            8080);
  LauncherOptions override;
  override.studio_http_port = 9090;
  EXPECT_EQ(EffectiveStudioHttpPort(
                config::ParseRobotConfigString(kCustomStudio), override),
            9090);
}

TEST(LauncherTest, StudioBridgeWatchTopicFollowsDescribe) {
  // No description: the config's telemetry topic, else the default.
  config::RobotConfig robot;
  EXPECT_EQ(StudioBridgeWatchTopic({}, robot), "/talos/telemetry");
  robot.toml_data =
      toml::parse("[subsystems.telemetry]\ntopic = \"/config/telemetry\"\n");
  EXPECT_EQ(StudioBridgeWatchTopic({}, robot), "/config/telemetry");

  // The bridge's own WATCHER wins, tracking a topic override.
  NodeDescription bridge;
  bridge.roster_name = "studio_bridge";
  bridge.answered = true;
  bridge.declared.name = "studio_bridge";
  introspect::naming::SourceShape watcher{event::SourceKind::WATCHER,
                                          "/custom/telemetry", 48, 0};
  bridge.declared.sources.push_back(watcher);
  EXPECT_EQ(StudioBridgeWatchTopic({bridge}, robot), "/custom/telemetry");

  // Other nodes never steer the bridge's subscription.
  NodeDescription alpha;
  alpha.roster_name = "alpha";
  alpha.answered = true;
  alpha.declared.name = "alpha";
  introspect::naming::SourceShape sender{event::SourceKind::SENDER,
                                         "/alpha/state", 48, 0};
  alpha.declared.sources.push_back(sender);
  EXPECT_EQ(StudioBridgeWatchTopic({alpha}, robot), "/config/telemetry");
}

TEST(LauncherTest, ShouldRetryStudioBridgePredicate) {
  using clock = std::chrono::steady_clock;
  const auto now = clock::now();
  const auto first = now - std::chrono::seconds(5);
  const auto last = now - std::chrono::seconds(2);
  // Budget remains and the last spawn is paced: retry.
  EXPECT_TRUE(ShouldRetryStudioBridge(1, first, last, now, false));
  // Pacing: a spawn less than one interval ago is not retried yet.
  EXPECT_FALSE(ShouldRetryStudioBridge(1, first, now, now, false));
  // Attempts exhausted.
  EXPECT_TRUE(kStudioBridgeMaxAttempts > 1);
  EXPECT_FALSE(ShouldRetryStudioBridge(kStudioBridgeMaxAttempts, first, last,
                                       now, false));
  // Window exhausted.
  EXPECT_FALSE(ShouldRetryStudioBridge(
      1, now - kStudioBridgeStartupWindow - std::chrono::seconds(1), last, now,
      false));
  // A stopping session (signal or deadline) ends the retry, never extends it.
  EXPECT_FALSE(ShouldRetryStudioBridge(1, first, last, now, true));
}

// Forward declarations for helpers defined with the graph tests below.
std::string WriteStubNode(std::string_view name, std::string_view script);
std::string DescribeJson(
    std::string_view node, std::string_view target,
    const std::vector<introspect::naming::SourceShape>& sources);
introspect::naming::SourceShape Sender(std::string topic,
                                       std::uint32_t bytes = 48);
introspect::naming::SourceShape Watcher(std::string topic,
                                        std::uint32_t bytes = 48);
std::string ReadFile(const std::string& path);

// A bridge stub for supervision tests: answers --describe like a viewer, then
// either fails (telemetry never arrives) or sleeps (viewer stays up).
std::string BridgeDescribeThen(std::string_view runtime) {
  return std::string{"#!/bin/sh\n"} +
         "for arg in \"$@\"; do\n"
         "  if [ \"$arg\" = \"--describe\" ]; then\n"
         "    cat <<'TALOS_DESCRIBE'\n" +
         DescribeJson("studio_bridge", "//studio/bridge:studio_bridge",
                      {Watcher("/alpha/state")}) +
         "TALOS_DESCRIBE\n"
         "    exit 0\n"
         "  fi\n"
         "done\n" +
         std::string{runtime};
}

std::string NodeDescribeThenSleep(
    std::string_view node, std::string_view target,
    const std::vector<introspect::naming::SourceShape>& sources, int sleep_s) {
  return std::string{"#!/bin/sh\n"} +
         "for arg in \"$@\"; do\n"
         "  if [ \"$arg\" = \"--describe\" ]; then\n"
         "    cat <<'TALOS_DESCRIBE'\n" +
         DescribeJson(node, target, sources) +
         "TALOS_DESCRIBE\n"
         "    exit 0\n"
         "  fi\n"
         "done\n"
         "exec sleep " +
         std::to_string(sleep_s) + "\n";
}

// A failing bridge must not delay the session: the first attempt spawns with
// the nodes and retries happen inside the supervision loop while the session
// runs. The counter proves a retry fired; the elapsed bound proves nothing
// blocked for the old ~60s startup wait.
TEST(LauncherTest, FailingBridgeRetriesInLoopWithoutBlockingSession) {
  const std::string tag = "bridge_nonblock";
  const std::string out_dir = TempPath(tag + "_out");
  std::filesystem::create_directories(out_dir);
  const std::string counter = TempPath(tag + "_counter");
  ::remove(counter.c_str());

  LauncherOptions options;
  options.output_dir = out_dir;
  options.session_id = 0xB81D9EULL;
  options.simulation = true;
  options.start_studio_bridge = true;
  options.describe_timeout_ms = 4000;
  options.config_path = WriteFixtureToml();
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      ::access("/usr/bin/true", X_OK) == 0 ? "/usr/bin/true" : "/bin/echo";
  options.binary_overrides["//example/alpha:node"] = WriteStubNode(
      tag + "_alpha", NodeDescribeThenSleep("alpha", "//example/alpha:node",
                                            {Sender("/alpha/state")}, 3));
  options.binary_overrides["//example/beta:node"] = WriteStubNode(
      tag + "_beta", NodeDescribeThenSleep("beta", "//example/beta:node",
                                           {Watcher("/alpha/state")}, 3));
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStubNode(tag + "_bridge",
                    BridgeDescribeThen("echo x >> " + counter + "\nexit 1\n"));

  const auto started = std::chrono::steady_clock::now();
  Launcher launcher{options};
  launcher.Run();
  const auto elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_LT(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(),
            20)
      << "a failing bridge must not cost the old ~60s startup wait";
  EXPECT_TRUE(std::filesystem::exists(out_dir + "/manifest.json"));
  const std::string manifest = ReadFile(out_dir + "/manifest.json");
  EXPECT_NE(manifest.find("\"name\": \"studio_bridge\""), std::string::npos)
      << manifest;
  // Exhaustion is degraded, not broken: the bridge is failed, the session ran.
  EXPECT_NE(manifest.find("\"recorder_failed\": true"), std::string::npos)
      << manifest;

  std::ifstream count_in{counter};
  std::size_t attempts = 0;
  std::string line;
  while (std::getline(count_in, line)) ++attempts;
  EXPECT_GE(attempts, 2u) << "retries must fire inside the supervision loop";

  std::filesystem::remove_all(out_dir);
  ::remove(counter.c_str());
}

// The duration deadline starts with the session, not after the viewer: a
// failing bridge plus long-lived nodes must still end near --duration-s.
TEST(LauncherTest, DurationDeadlineIsNotDelayedByFailingBridge) {
  const std::string tag = "bridge_deadline";
  const std::string out_dir = TempPath(tag + "_out");
  std::filesystem::create_directories(out_dir);

  LauncherOptions options;
  options.output_dir = out_dir;
  options.session_id = 0xDEAD1100ULL;
  options.simulation = true;
  options.start_studio_bridge = true;
  options.describe_timeout_ms = 4000;
  options.duration_s = 2;
  options.config_path = WriteFixtureToml();
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      ::access("/usr/bin/true", X_OK) == 0 ? "/usr/bin/true" : "/bin/echo";
  options.binary_overrides["//example/alpha:node"] = WriteStubNode(
      tag + "_alpha", NodeDescribeThenSleep("alpha", "//example/alpha:node",
                                            {Sender("/alpha/state")}, 30));
  options.binary_overrides["//example/beta:node"] = WriteStubNode(
      tag + "_beta", NodeDescribeThenSleep("beta", "//example/beta:node",
                                           {Watcher("/alpha/state")}, 30));
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStubNode(tag + "_bridge", BridgeDescribeThen("exit 1\n"));

  const auto started = std::chrono::steady_clock::now();
  Launcher launcher{options};
  launcher.Run();
  const auto elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_LT(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(),
            20)
      << "deadline must fire without waiting out the bridge window";
  EXPECT_TRUE(std::filesystem::exists(out_dir + "/manifest.json"));

  std::filesystem::remove_all(out_dir);
}

// A bridge that survives its grace second joins the session: it is not failed
// and the session runs to the deadline with it.
TEST(LauncherTest, StableBridgeJoinsSessionUntilDeadline) {
  const std::string tag = "bridge_stable";
  const std::string out_dir = TempPath(tag + "_out");
  std::filesystem::create_directories(out_dir);

  LauncherOptions options;
  options.output_dir = out_dir;
  options.session_id = 0x57AB1EULL;
  options.simulation = true;
  options.start_studio_bridge = true;
  options.describe_timeout_ms = 4000;
  options.duration_s = 3;
  options.config_path = WriteFixtureToml();
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      ::access("/usr/bin/true", X_OK) == 0 ? "/usr/bin/true" : "/bin/echo";
  options.binary_overrides["//example/alpha:node"] = WriteStubNode(
      tag + "_alpha", NodeDescribeThenSleep("alpha", "//example/alpha:node",
                                            {Sender("/alpha/state")}, 30));
  options.binary_overrides["//example/beta:node"] = WriteStubNode(
      tag + "_beta", NodeDescribeThenSleep("beta", "//example/beta:node",
                                           {Watcher("/alpha/state")}, 30));
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStubNode(tag + "_bridge", BridgeDescribeThen("exec sleep 30\n"));

  const auto started = std::chrono::steady_clock::now();
  Launcher launcher{options};
  launcher.Run();
  const auto elapsed = std::chrono::steady_clock::now() - started;
  const auto elapsed_s =
      std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

  EXPECT_GE(elapsed_s, 2) << "session must run to the deadline with Studio";
  EXPECT_LT(elapsed_s, 20) << "session must end at the deadline";
  ASSERT_TRUE(std::filesystem::exists(out_dir + "/manifest.json"));
  const std::string manifest = ReadFile(out_dir + "/manifest.json");
  EXPECT_NE(manifest.find("\"name\": \"studio_bridge\""), std::string::npos)
      << manifest;
  // A stable viewer is a session member, not a failure.
  EXPECT_EQ(manifest.find("\"recorder_failed\": true"), std::string::npos)
      << manifest;

  std::filesystem::remove_all(out_dir);
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
  manifest.config_path = "fixture/robot.toml";
  manifest.output_dir = "/tmp/test_dir";
  manifest.simulation = true;
  manifest.start_wall_ns = 1'000'000'000LL;
  manifest.end_wall_ns = 2'000'000'000LL;

  NodeProcessInfo node1;
  node1.name = "drivetrain";
  node1.target = "//2026-robot/main_processor/drivetrain:node";
  node1.binary_path = "bazel-bin/2026-robot/main_processor/drivetrain/node";
  node1.log_path = "/tmp/test_dir/drivetrain.tlog";
  node1.pid = 1234;
  node1.exit_code = 0;
  node1.record_count = 50;
  node1.failed = false;

  manifest.config_sha256 = "deadbeef";
  manifest.nodes.push_back(node1);

  std::string json = manifest.ToJson();
  EXPECT_NE(json.find("\"session_id\": 987654321"), std::string::npos);
  EXPECT_NE(json.find("\"name\": \"drivetrain\""), std::string::npos);
  EXPECT_NE(json.find("\"pid\": 1234"), std::string::npos);
  EXPECT_NE(json.find("\"exit_code\": 0"), std::string::npos);
  EXPECT_NE(json.find("\"record_count\": 50"), std::string::npos);
  EXPECT_NE(json.find("\"config_sha256\": \"deadbeef\""), std::string::npos);
}

TEST(LauncherTest, RunsChildProcessesAndWritesManifest) {
  const std::string out_dir = TempPath("launcher_run_out");
  std::filesystem::create_directories(out_dir);

  LauncherOptions options;
  options.output_dir = out_dir;
  options.session_id = 0x55555555ULL;
  options.duration_s = 1;

  // Use /usr/bin/true (or /bin/echo) for all nodes so the test executes
  // instantly and cleanly
  std::string mock_bin =
      ::access("/usr/bin/true", X_OK) == 0 ? "/usr/bin/true" : "/bin/echo";
  options.config_path = WriteFixtureToml();
  options.binary_overrides["//talOS/bridge:hardware_node"] = mock_bin;
  options.binary_overrides["//example/alpha:node"] = mock_bin;
  options.binary_overrides["//example/beta:node"] = mock_bin;

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

// A stand-in for a node binary.
//
// What the launcher wants from a binary is what it prints when asked what it
// is, so a script that prints it is a complete node for this purpose -- and the
// graph under test stays in the test instead of in some other package's source,
// where it would drift.
std::string WriteStubNode(std::string_view name, std::string_view script) {
  const std::string path = TempPath(name);
  {
    std::ofstream out{path};
    out << script;
  }
  ::chmod(path.c_str(), 0755);
  return path;
}

// The JSON a real node prints, written by the real writer: a stub that answered
// with hand-typed JSON would pass the day describe.h changed shape.
std::string DescribeJson(
    std::string_view node, std::string_view target,
    const std::vector<introspect::naming::SourceShape>& sources) {
  introspect::Description description;
  description.name = std::string{node};
  description.target = std::string{target};
  description.sources = sources;
  return introspect::DescribeToJson(description);
}

std::string DescribingStub(
    std::string_view node, std::string_view target,
    const std::vector<introspect::naming::SourceShape>& sources) {
  return std::string{"#!/bin/sh\n"} +
         "for arg in \"$@\"; do\n"
         "  if [ \"$arg\" = \"--describe\" ]; then\n"
         "    cat <<'TALOS_DESCRIBE'\n" +
         DescribeJson(node, target, sources) +
         "TALOS_DESCRIBE\n"
         "    exit 0\n"
         "  fi\n"
         "done\n"
         "exit 0\n";
}

// A node written before --describe existed, or one halfway through being
// written: it rejects the flag the way every node used to.
constexpr std::string_view kStubWithoutDescribe =
    "#!/bin/sh\n"
    "for arg in \"$@\"; do\n"
    "  if [ \"$arg\" = \"--describe\" ]; then\n"
    "    echo \"usage: node [--sim] [--log PATH]\" >&2\n"
    "    exit 1\n"
    "  fi\n"
    "done\n"
    "exit 0\n";

// exec so the pid the launcher kills is the one that is sleeping; a shell that
// forked the sleep would report the kill and leave the sleep behind.
constexpr std::string_view kStubThatHangsOnDescribe =
    "#!/bin/sh\n"
    "for arg in \"$@\"; do\n"
    "  if [ \"$arg\" = \"--describe\" ]; then\n"
    "    exec sleep 30\n"
    "  fi\n"
    "done\n"
    "exit 0\n";

introspect::naming::SourceShape Sender(std::string topic, std::uint32_t bytes) {
  return {event::SourceKind::SENDER, std::move(topic), bytes, 0};
}
introspect::naming::SourceShape Watcher(std::string topic,
                                        std::uint32_t bytes) {
  return {event::SourceKind::WATCHER, std::move(topic), bytes, 0};
}

std::string ReadFile(const std::string& path) {
  std::ifstream in{path};
  return std::string{std::istreambuf_iterator<char>(in),
                     std::istreambuf_iterator<char>()};
}

// Two stubs and a hardware node that says nothing, wired to the fixture config.
struct GraphFixture {
  LauncherOptions options;
  std::string out_dir;

  GraphFixture(std::string_view tag, std::string_view alpha_script,
               std::string_view beta_script) {
    out_dir = TempPath(std::string{tag} + "_out");
    std::filesystem::create_directories(out_dir);
    options.output_dir = out_dir;
    options.session_id = 0x0123456789ABCDEFULL;
    options.config_path = WriteFixtureToml();
    options.describe_timeout_ms = 4000;
    options.binary_overrides["//talOS/bridge:hardware_node"] =
        ::access("/usr/bin/true", X_OK) == 0 ? "/usr/bin/true" : "/bin/echo";
    options.binary_overrides["//example/alpha:node"] =
        WriteStubNode(std::string{tag} + "_alpha", alpha_script);
    options.binary_overrides["//example/beta:node"] =
        WriteStubNode(std::string{tag} + "_beta", beta_script);
  }

  ~GraphFixture() { std::filesystem::remove_all(out_dir); }

  std::string graph_json() const { return ReadFile(out_dir + "/graph.json"); }
  bool has_manifest() const {
    return std::filesystem::exists(out_dir + "/manifest.json");
  }
};

TEST(LauncherGraphTest, ProbesEveryBinaryAndPutsWhatItSaysInTheGraph) {
  GraphFixture fixture{
      "meets",
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")}),
      DescribingStub("beta", "//example/beta:node", {Watcher("/alpha/state")})};

  Launcher launcher{fixture.options};
  EXPECT_EQ(launcher.Run(), 0);

  const std::string graph = fixture.graph_json();
  EXPECT_NE(graph.find("\"session_id\": \"81985529216486895\""),
            std::string::npos)
      << graph;
  EXPECT_NE(graph.find("\"name\": \"alpha\""), std::string::npos) << graph;
  EXPECT_NE(graph.find("\"kind\": \"SENDER\", \"name\": \"/alpha/state\""),
            std::string::npos)
      << graph;
  EXPECT_NE(graph.find("\"kind\": \"WATCHER\", \"name\": \"/alpha/state\""),
            std::string::npos)
      << graph;
  // A topic with both ends declared is the case nothing should be said about.
  EXPECT_EQ(graph.find("\"severity\": \"error\""), std::string::npos) << graph;

  ASSERT_TRUE(fixture.has_manifest());
  const std::string manifest = ReadFile(fixture.out_dir + "/manifest.json");
  EXPECT_NE(manifest.find("\"described\": true"), std::string::npos)
      << manifest;
}

TEST(LauncherGraphTest, RefusesToSpawnWhenTheTwoEndsAreSpelledDifferently) {
  GraphFixture fixture{
      "differs",
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")}),
      DescribingStub("beta", "//example/beta:node",
                     {Watcher("/alpha/status")})};

  Launcher launcher{fixture.options};
  EXPECT_NE(launcher.Run(), 0);

  const std::string graph = fixture.graph_json();
  EXPECT_NE(graph.find("nothing publishes it"), std::string::npos) << graph;
  EXPECT_NE(graph.find("\"severity\": \"error\""), std::string::npos) << graph;

  // Nothing was started, so there is no session: the manifest is the evidence.
  EXPECT_FALSE(fixture.has_manifest());
}

TEST(LauncherGraphTest, AllowGraphErrorsLaunchesTheBrokenGraphAnyway) {
  GraphFixture fixture{
      "allowed",
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")}),
      DescribingStub("beta", "//example/beta:node",
                     {Watcher("/alpha/status")})};
  fixture.options.allow_graph_errors = true;

  Launcher launcher{fixture.options};
  EXPECT_EQ(launcher.Run(), 0);
  EXPECT_TRUE(fixture.has_manifest());
  EXPECT_NE(fixture.graph_json().find("\"severity\": \"error\""),
            std::string::npos);
}

TEST(LauncherGraphTest, ABinaryThatIgnoresDescribeIsAWarningAndStillLaunches) {
  GraphFixture fixture{"silent", kStubWithoutDescribe, kStubWithoutDescribe};

  Launcher launcher{fixture.options};
  EXPECT_EQ(launcher.Run(), 0);
  EXPECT_TRUE(fixture.has_manifest());

  const std::string graph = fixture.graph_json();
  EXPECT_NE(graph.find("\"described\": false"), std::string::npos) << graph;
  EXPECT_NE(graph.find("did not answer --describe"), std::string::npos)
      << graph;
  EXPECT_NE(graph.find("\"severity\": \"warning\""), std::string::npos)
      << graph;
  EXPECT_EQ(graph.find("\"severity\": \"error\""), std::string::npos) << graph;
}

TEST(LauncherGraphTest, ANodeThatHangsOnDescribeDoesNotHangTheLaunch) {
  GraphFixture fixture{
      "hangs", kStubThatHangsOnDescribe,
      DescribingStub("beta", "//example/beta:node", {Sender("/beta/state")})};
  fixture.options.describe_timeout_ms = 250;

  const auto started = std::chrono::steady_clock::now();
  Launcher launcher{fixture.options};
  EXPECT_EQ(launcher.Run(), 0);
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_LT(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(),
            10);

  EXPECT_NE(fixture.graph_json().find("no answer within 250ms"),
            std::string::npos)
      << fixture.graph_json();
}

TEST(LauncherGraphTest, DescribeOnlyLintsWithoutStartingAnything) {
  GraphFixture fixture{
      "preflight",
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")}),
      DescribingStub("beta", "//example/beta:node", {Watcher("/alpha/state")})};
  fixture.options.describe_only = true;

  Launcher launcher{fixture.options};
  EXPECT_EQ(launcher.Run(), 0);
  EXPECT_FALSE(fixture.has_manifest());
  EXPECT_NE(fixture.graph_json().find("\"name\": \"beta\""), std::string::npos);
}

TEST(LauncherGraphTest, DescribeOnlyReportsABrokenGraphAsAFailure) {
  GraphFixture fixture{
      "preflight_broken",
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")}),
      DescribingStub("beta", "//example/beta:node",
                     {Watcher("/alpha/status")})};
  fixture.options.describe_only = true;

  Launcher launcher{fixture.options};
  EXPECT_NE(launcher.Run(), 0);
  EXPECT_FALSE(fixture.has_manifest());
}

TEST(LauncherGraphTest, WarnsWhenABinaryDisagreesWithTheConfigAboutItsName) {
  NodeSpec spec;
  spec.name = "alpha";
  spec.target = "//example/alpha:node";
  spec.binary_path =
      WriteStubNode("renamed", DescribingStub("gamma", "//example/gamma:node",
                                              {Sender("/gamma/state")}));

  LauncherOptions options;
  options.describe_timeout_ms = 4000;
  std::vector<NodeDescription> described;
  const auto diagnostics = DescribeGraph({spec}, options, described);

  ASSERT_EQ(described.size(), 1u);
  ASSERT_TRUE(described[0].answered) << described[0].error;
  EXPECT_EQ(described[0].declared.name, "gamma");
  EXPECT_EQ(described[0].roster_name, "alpha");

  const bool warned = std::any_of(
      diagnostics.begin(), diagnostics.end(),
      [](const introspect::naming::Diagnostic& d) {
        return d.severity == introspect::naming::Severity::WARNING &&
               d.message.find("describes itself as 'gamma'") !=
                   std::string::npos;
      });
  EXPECT_TRUE(warned) << "diagnostics did not mention the disagreement";

  std::remove(spec.binary_path.c_str());
}

TEST(LauncherGraphTest, ProbeReportsABinaryThatIsNotThere) {
  introspect::Description out;
  std::string error;
  EXPECT_FALSE(ProbeDescribe("/nonexistent/talos_node",
                             {"/nonexistent/talos_node", "--describe"},
                             std::chrono::milliseconds{500}, out, error));
  EXPECT_NE(error.find("no executable"), std::string::npos) << error;
}

TEST(LauncherBundleTest, ResolveBinaryPrefersOverrideThenBazelBin) {
  std::map<std::string, std::string> overrides;
  overrides["//example/alpha:node"] = "/bin/alpha";
  EXPECT_EQ(ResolveBinary("//example/alpha:node", overrides), "/bin/alpha");
  EXPECT_EQ(ResolveBinary("//example/beta:node", overrides),
            "bazel-bin/example/beta/node");
  EXPECT_EQ(ResolveBinary("/opt/talos/bin/beta"), "/opt/talos/bin/beta");
}

TEST(LauncherBundleTest, LoadBundleParsesManifestAndOverrides) {
  const std::string bundle_dir = TempPath("test_bundle_dir");
  std::filesystem::create_directories(bundle_dir + "/config");
  std::filesystem::create_directories(bundle_dir + "/bin");

  const std::string cfg_path = bundle_dir + "/config/robot.toml";
  {
    std::ofstream cfg(cfg_path);
    cfg << "[robot]\nname = \"bundle_bot\"\nperiod_us = 5000\n";
  }

  {
    std::ofstream mf(bundle_dir + "/bundle_manifest.toml");
    mf << "bundle_version = 1\n"
       << "config_path = \"config/robot.toml\"\n"
       << "config_sha256 = \"abc123\"\n"
       << "\n[[nodes]]\n"
       << "name = \"alpha\"\n"
       << "targets = [\"//example/alpha:node\", \"//example/alpha:alias\"]\n"
       << "binary = \"bin/alpha\"\n"
       << "language = \"cpp\"\n";
  }

  {
    std::ofstream b(bundle_dir + "/bin/alpha");
    b << "#!/bin/sh\nexit 0\n";
  }
  ::chmod((bundle_dir + "/bin/alpha").c_str(), 0755);

  LauncherOptions options;
  options.bundle_dir = bundle_dir;

  std::string error;
  EXPECT_TRUE(LoadBundle(options, error)) << error;
  EXPECT_EQ(options.config_path, cfg_path);
  EXPECT_EQ(options.binary_overrides["//example/alpha:node"],
            bundle_dir + "/bin/alpha");
  EXPECT_EQ(options.binary_overrides["alpha"], bundle_dir + "/bin/alpha");
  EXPECT_EQ(options.binary_overrides["//example/alpha:alias"],
            bundle_dir + "/bin/alpha");
  EXPECT_EQ(options.bundle_config_sha256, "abc123");

  // A manifest that is not TOML is an error, not a silently empty bundle.
  { std::ofstream{bundle_dir + "/bundle_manifest.toml"} << "nodes = [\n"; }
  LauncherOptions broken;
  broken.bundle_dir = bundle_dir;
  EXPECT_FALSE(LoadBundle(broken, error));
  EXPECT_NE(error.find("bundle_manifest.toml"), std::string::npos) << error;

  std::filesystem::remove_all(bundle_dir);
}

TEST(LauncherBundleTest, LauncherRunsFromBundleDirectoryWithDescribeOnly) {
  const std::string bundle_dir = TempPath("test_bundle_run");
  std::filesystem::create_directories(bundle_dir + "/config");
  std::filesystem::create_directories(bundle_dir + "/bin");

  const std::string cfg_path = bundle_dir + "/config/robot.toml";
  {
    std::ofstream cfg(cfg_path);
    cfg << "[robot]\nname = \"bundle_robot\"\nperiod_us = 5000\n"
        << "[subsystems.alpha]\nnode = \"//bundle/alpha:node\"\nperiod_us = "
           "5000\n"
        << "[subsystems.alpha.motors.m1]\ntype = \"TalonFX\"\nbus = "
           "\"rio\"\ncan_id = 1\n";
  }

  const std::string alpha_bin = bundle_dir + "/bin/alpha";
  {
    std::ofstream out{alpha_bin};
    out << DescribingStub("alpha", "//bundle/alpha:node",
                          {Sender("/alpha/state")});
  }
  ::chmod(alpha_bin.c_str(), 0755);

  const std::string hw_bin = bundle_dir + "/bin/hardware_node";
  {
    std::ofstream out{hw_bin};
    out << DescribingStub("hardware_node", "//talOS/bridge:hardware_node",
                          {Watcher("/alpha/state")});
  }
  ::chmod(hw_bin.c_str(), 0755);

  {
    std::ofstream mf(bundle_dir + "/bundle_manifest.toml");
    mf << "bundle_version = 1\n"
       << "config_path = \"config/robot.toml\"\n"
       << "\n[[nodes]]\nname = \"hardware_node\"\n"
       << "targets = [\"//talOS/bridge:hardware_node\"]\n"
       << "binary = \"bin/hardware_node\"\nlanguage = \"cpp\"\n"
       << "\n[[nodes]]\nname = \"alpha\"\n"
       << "targets = [\"//bundle/alpha:node\"]\n"
       << "binary = \"bin/alpha\"\nlanguage = \"cpp\"\n";
  }

  LauncherOptions options;
  options.bundle_dir = bundle_dir;
  options.describe_only = true;
  options.output_dir = bundle_dir + "/logs";

  Launcher launcher{options};
  EXPECT_EQ(launcher.Run(), 0);
  EXPECT_TRUE(std::filesystem::exists(bundle_dir + "/logs/graph.json"));

  std::filesystem::remove_all(bundle_dir);
}

TEST(LauncherRemoteTest, EffectiveAddressComesFromDeploymentConfig) {
  constexpr std::string_view kToml = R"(
[robot]
name = "remote_fixture"
period_us = 5000
controller_address = "10.56.87.2"

[subsystems.alpha]
node = "//example/alpha:node"

[subsystems.alpha.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
)";
  const auto robot_cfg = config::ParseRobotConfigString(kToml);

  LauncherOptions from_config;
  EXPECT_EQ(EffectiveRemoteAddress(robot_cfg, from_config), "10.56.87.2");
  EXPECT_EQ(EffectiveRemotePort(robot_cfg, from_config), 5802);
  EXPECT_EQ(EffectiveLocalPort(robot_cfg, from_config), 5803);

  LauncherOptions cli_override;
  cli_override.remote = "10.0.0.9";
  cli_override.remote_port = 5900;
  cli_override.local_port = 5901;
  EXPECT_EQ(EffectiveRemoteAddress(robot_cfg, cli_override), "10.0.0.9");
  EXPECT_EQ(EffectiveRemotePort(robot_cfg, cli_override), 5900);
  EXPECT_EQ(EffectiveLocalPort(robot_cfg, cli_override), 5901);
}

TEST(LauncherRemoteTest, EffectiveAddressDefaultsToLoopback) {
  const auto robot_cfg = config::ParseRobotConfigString(kFixtureToml);
  LauncherOptions options;
  EXPECT_EQ(EffectiveRemoteAddress(robot_cfg, options), "127.0.0.1");
  EXPECT_EQ(EffectiveRemotePort(robot_cfg, options), 5802);
  EXPECT_EQ(EffectiveLocalPort(robot_cfg, options), 5803);
}

}  // namespace
}  // namespace talos::launcher
