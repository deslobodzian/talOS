#include "talOS/launcher/launcher.h"

#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
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


TEST(LauncherTest, StudioBridgeWatchTopicFollowsDescribe) {
  // No description: the config's studio topic, else the default.
  config::RobotConfig robot;
  EXPECT_EQ(StudioBridgeWatchTopic({}, robot), "/talos/telemetry");
  robot.studio.topic = "/config/telemetry";
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

// METHODOLOGY §6 for parameters: a numeric key without a unit suffix is
// named, per subsystem; strings, bools and nested device tables are not
// parameters with units.
TEST(LauncherTest, NumericParametersWithoutAUnitSuffixAreWarned) {
  config::RobotConfig robot;
  robot.toml_data = toml::parse(R"(
[subsystems.shooter]
period_us = 5000
gain = 0.5
tolerance_rot = 0.1
gains = [1, 2.5]
offsets_m = [0.1]
label = "x"
enabled = true
names = ["a"]
[subsystems.shooter.motors.flywheel]
can_id = 3
inverted_count = 1
wobble = 2

[subsystems.intake]
speed_rps = 40.0
)");
  const auto findings = LintParameterUnits(robot);
  ASSERT_EQ(findings.size(), 1u);
  EXPECT_EQ(findings[0].severity, introspect::naming::Severity::WARNING);
  EXPECT_EQ(findings[0].subject, "shooter");
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
std::string DescribingStub(
    std::string_view node, std::string_view target,
    const std::vector<introspect::naming::SourceShape>& sources);
introspect::naming::SourceShape Sender(std::string topic,
                                       std::uint32_t bytes = 48);
introspect::naming::SourceShape Watcher(std::string topic,
                                        std::uint32_t bytes = 48);
std::string ReadFile(const std::string& path);

TEST(LauncherTest, DescribeDeadlineAppliesAfterStdoutCloses) {
  const auto binary = WriteStubNode("describe_closed_stdout",
                                    "#!/bin/sh\nexec 1>&-\nexec sleep 30\n");
  introspect::Description description;
  std::string error;
  const auto started = std::chrono::steady_clock::now();
  EXPECT_FALSE(ProbeDescribe(binary, {binary, "--describe"},
                             std::chrono::milliseconds{100}, description,
                             error));
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_LT(elapsed, std::chrono::seconds{3});
  std::filesystem::remove(binary);
}

TEST(LauncherTest, DescribeOutputIsBounded) {
  const auto binary = WriteStubNode(
      "describe_excess_output",
      "#!/bin/sh\nexec dd if=/dev/zero bs=4096 count=1025 2>/dev/null\n");
  introspect::Description description;
  std::string error;
  EXPECT_FALSE(ProbeDescribe(binary, {binary, "--describe"},
                             std::chrono::seconds{5}, description, error));
  std::filesystem::remove(binary);
}

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
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
      tag + "_hardware",
      DescribingStub("hardware_node", "//talOS/bridge:hardware_node", {}));
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
  const std::string session =
      out_dir + "/" + std::to_string(options.session_id);
  EXPECT_TRUE(std::filesystem::exists(session + "/manifest.json"));
  const std::string manifest = ReadFile(session + "/manifest.json");
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

TEST(LauncherTest, RestartSharesByteBudgetAndManifestEnospcDoesNotStopIt) {
  const auto root = TempPath("budget_restart");
  const auto attempts = root + "/attempts";
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  LauncherOptions options;
  options.output_dir = root;
  options.session_id = 12345;
  options.simulation = true;
  options.duration_s = 2;
  options.config_path = root + "/robot.toml";
  std::string config{kFixtureToml};
  config.insert(config.find("name ="), "log_max_bytes = 1200\n");
  std::ofstream{options.config_path} << config;
  const auto session = root + "/12345";
  std::string script = NodeDescribeThenSleep("alpha", "//example/alpha:node",
                                             {Sender("/alpha/state")}, 0);
  script.erase(script.rfind("exec sleep"));
  script +=
      "log=''\ncap=''\n"
      "while [ $# -gt 0 ]; do\n"
      " case \"$1\" in\n"
      " --log) shift; log=\"$1\" ;;\n"
      " --log-max-bytes) shift; cap=\"$1\" ;;\n"
      " esac\n shift\n done\n"
      "echo \"$cap\" >> '" +
      attempts +
      "'\n"
      "[ -n \"$cap\" ] || exit 2\n"
      "if [ ! -f '" +
      root +
      "/once' ]; then\n"
      " touch '" +
      root +
      "/once'\n"
      " dd if=/dev/zero of=\"$log\" bs=1 count=100 2>/dev/null\n"
      // The launcher writes manifest.json.tmp itself and renames it; wait
      // until its own temporary file is gone before planting /dev/full.
      " until ln -s /dev/full '" +
      session +
      "/manifest.json.tmp' 2>/dev/null; do sleep 0.01; done\n"
      " exit 1\nfi\n"
      "touch \"$log.capped\"\nexec sleep 5\n";
  options.binary_overrides["//example/alpha:node"] =
      WriteStubNode("budget_alpha", script);
  options.binary_overrides["//example/beta:node"] = WriteStubNode(
      "budget_beta", NodeDescribeThenSleep("beta", "//example/beta:node",
                                           {Watcher("/alpha/state")}, 5));
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
      "budget_hw", NodeDescribeThenSleep(
                       "hardware_node", "//talOS/bridge:hardware_node", {}, 5));
  Launcher launcher{options};
  launcher.Run();
  EXPECT_EQ(ReadFile(attempts), "200\n100\n");
  const auto manifest = ReadFile(session + "/manifest.json");
  EXPECT_NE(manifest.find("\"restart_count\": 1"), std::string::npos);
  EXPECT_NE(manifest.find("\"log_capped\": true"), std::string::npos);
  std::filesystem::remove_all(root);
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
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
      tag + "_hardware",
      DescribingStub("hardware_node", "//talOS/bridge:hardware_node", {}));
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
  EXPECT_TRUE(std::filesystem::exists(
      out_dir + "/" + std::to_string(options.session_id) + "/manifest.json"));

  std::filesystem::remove_all(out_dir);
}

TEST(LauncherTest, ShutdownKillsChildrenIgnoringInterrupt) {
  const std::string tag = "ignore_interrupt";
  const std::string out_dir = TempPath(tag + "_out");
  std::filesystem::create_directories(out_dir);
  LauncherOptions options;
  options.output_dir = out_dir;
  options.session_id = 0xDEAD2200ULL;
  options.simulation = true;
  options.describe_timeout_ms = 4000;
  options.duration_s = 1;
  options.config_path = WriteFixtureToml();
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
      tag + "_hardware",
      DescribingStub("hardware_node", "//talOS/bridge:hardware_node", {}));
  auto stubborn = NodeDescribeThenSleep("alpha", "//example/alpha:node",
                                        {Sender("/alpha/state")}, 30);
  stubborn.insert(stubborn.find("exec sleep"), "trap '' INT\n");
  const auto alpha = WriteStubNode(tag + "_alpha", stubborn);
  const auto beta = WriteStubNode(
      tag + "_beta", NodeDescribeThenSleep("beta", "//example/beta:node",
                                           {Watcher("/alpha/state")}, 30));
  options.binary_overrides["//example/alpha:node"] = alpha;
  options.binary_overrides["//example/beta:node"] = beta;
  const auto started = std::chrono::steady_clock::now();
  Launcher launcher{options};
  EXPECT_EQ(launcher.Run(), -1);
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_GE(elapsed, std::chrono::seconds{3});
  EXPECT_LT(elapsed, std::chrono::seconds{10});
  std::filesystem::remove(alpha);
  std::filesystem::remove(beta);
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
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
      tag + "_hardware",
      DescribingStub("hardware_node", "//talOS/bridge:hardware_node", {}));
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
  const std::string session =
      out_dir + "/" + std::to_string(options.session_id);
  ASSERT_TRUE(std::filesystem::exists(session + "/manifest.json"));
  const std::string manifest = ReadFile(session + "/manifest.json");
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

  std::remove(path_a.c_str());
  std::remove(path_b.c_str());
}


TEST(LauncherTest, RestartBackoffDoublesFrom50msCappedAt1s) {
  using std::chrono::milliseconds;
  EXPECT_EQ(RestartBackoff(1), milliseconds{50});
  EXPECT_EQ(RestartBackoff(2), milliseconds{100});
  EXPECT_EQ(RestartBackoff(3), milliseconds{200});
  EXPECT_EQ(RestartBackoff(4), milliseconds{400});
  EXPECT_EQ(RestartBackoff(5), milliseconds{800});
  EXPECT_EQ(RestartBackoff(6), milliseconds{1000});
  EXPECT_EQ(RestartBackoff(1000), milliseconds{1000});
}


TEST(LauncherTest, ShouldRestartNodePredicate) {
  // A crash restarts a critical and a non-critical member alike.
  EXPECT_TRUE(ShouldRestartNode(false, true, 5, false));
  EXPECT_TRUE(ShouldRestartNode(false, false, 5, false));
  // A critical member exiting 0 while its session runs on is lost, not done.
  EXPECT_TRUE(ShouldRestartNode(true, true, 5, false));
  // A non-critical member exiting 0 finished.
  EXPECT_FALSE(ShouldRestartNode(true, false, 5, false));
  // Zero is a deliberate permanent stop, not a quarantine.
  EXPECT_FALSE(ShouldRestartNode(false, true, 0, false));
  EXPECT_FALSE(ShouldRestartNode(true, true, 0, false));
  // A stopping session (signal, deadline, shutdown) restarts nothing.
  EXPECT_FALSE(ShouldRestartNode(false, true, 5, true));
  EXPECT_FALSE(ShouldRestartNode(true, true, 5, true));
}

TEST(LauncherTest, MaxRestartsComesFromFlagThenConfigThenDefault) {
  const auto with_robot_keys = [](std::string_view keys) {
    std::string toml{kFixtureToml};
    const std::string header = "[robot]\n";
    toml.insert(toml.find(header) + header.size(), keys);
    return config::ParseRobotConfigString(toml);
  };
  int max = -1;
  std::string error;

  EXPECT_TRUE(ResolveMaxRestarts(with_robot_keys(""), {}, max, error));
  EXPECT_EQ(max, kDefaultMaxRestarts);
  EXPECT_EQ(kDefaultMaxRestarts, 5);

  EXPECT_TRUE(ResolveMaxRestarts(with_robot_keys("max_restarts = 2\n"), {}, max,
                                 error));
  EXPECT_EQ(max, 2);

  LauncherOptions flag;
  flag.max_restarts = 0;
  EXPECT_TRUE(ResolveMaxRestarts(with_robot_keys("max_restarts = 2\n"), flag,
                                 max, error));
  EXPECT_EQ(max, 0);

  EXPECT_FALSE(ResolveMaxRestarts(with_robot_keys("max_restarts = -1\n"), {},
                                  max, error));
  EXPECT_FALSE(ResolveMaxRestarts(with_robot_keys("max_restarts = 1.5\n"), {},
                                  max, error));
  EXPECT_FALSE(ResolveMaxRestarts(with_robot_keys("max_restarts = \"3\"\n"), {},
                                  max, error));
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

// A topic endpoint as a node describes it. The schema follows the size, so
// two endpoints of one size agree on the topic and of two sizes do not.
introspect::naming::SourceShape Endpoint(event::SourceKind kind,
                                         std::string topic,
                                         std::uint32_t bytes) {
  return {kind,
          std::move(topic),
          bytes,
          0,
          "launcher_test.Message" + std::to_string(bytes),
          0x1000 + bytes,
          8,
          1024};
}
introspect::naming::SourceShape Sender(std::string topic, std::uint32_t bytes) {
  return Endpoint(event::SourceKind::SENDER, std::move(topic), bytes);
}
introspect::naming::SourceShape Watcher(std::string topic,
                                        std::uint32_t bytes) {
  return Endpoint(event::SourceKind::WATCHER, std::move(topic), bytes);
}

std::string ReadFile(const std::string& path) {
  std::ifstream in{path};
  return std::string{std::istreambuf_iterator<char>(in),
                     std::istreambuf_iterator<char>()};
}

// Two stubs and an empty hardware node, wired to the fixture config.
struct GraphFixture {
  LauncherOptions options;
  std::string out_dir;

  GraphFixture(std::string_view tag, std::string_view alpha_script,
               std::string_view beta_script) {
    out_dir = TempPath(std::string{tag} + "_out");
    std::filesystem::create_directories(out_dir);
    options.output_dir = out_dir;
    options.session_id = 0x0123456789ABCDEFULL;
    out_dir += "/" + std::to_string(options.session_id);
    options.config_path = WriteFixtureToml();
    options.describe_timeout_ms = 4000;
    options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
        std::string{tag} + "_hardware",
        DescribingStub("hardware_node", "//talOS/bridge:hardware_node", {}));
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

TEST(LauncherGraphTest, ABinaryThatIgnoresDescribeRefusesLaunch) {
  GraphFixture fixture{"silent", kStubWithoutDescribe, kStubWithoutDescribe};

  Launcher launcher{fixture.options};
  EXPECT_EQ(launcher.Run(), 1);
  EXPECT_FALSE(fixture.has_manifest());

  const std::string graph = fixture.graph_json();
  EXPECT_NE(graph.find("\"described\": false"), std::string::npos) << graph;
  EXPECT_NE(graph.find("\"severity\": \"error\""), std::string::npos) << graph;
}

TEST(LauncherGraphTest, AllowGraphErrorsPermitsFailedDescribe) {
  GraphFixture fixture{"silent_allowed", kStubWithoutDescribe,
                       kStubWithoutDescribe};
  fixture.options.allow_graph_errors = true;
  EXPECT_EQ(Launcher{fixture.options}.Run(), 0);
  EXPECT_TRUE(fixture.has_manifest());
  EXPECT_NE(fixture.graph_json().find("\"severity\": \"error\""),
            std::string::npos);
}

TEST(LauncherGraphTest, ANodeThatHangsOnDescribeDoesNotHangTheLaunch) {
  GraphFixture fixture{
      "hangs", kStubThatHangsOnDescribe,
      DescribingStub("beta", "//example/beta:node", {Sender("/beta/state")})};
  fixture.options.describe_timeout_ms = 250;

  const auto started = std::chrono::steady_clock::now();
  Launcher launcher{fixture.options};
  EXPECT_EQ(launcher.Run(), 1);
  EXPECT_FALSE(fixture.has_manifest());
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_LT(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(),
            10);

}

TEST(LauncherGraphTest, NonCriticalTargetProducerIsAnError) {
  GraphFixture fixture{"optional_target",
                       DescribingStub("alpha", "//example/alpha:node",
                                      {Sender("/beta/target/teleop")}),
                       DescribingStub("beta", "//example/beta:node",
                                      {Watcher("/beta/target/teleop")})};
  auto specs = DiscoverNodes(
      config::ParseRobotConfig(fixture.options.config_path), fixture.options);
  for (auto& spec : specs) {
    if (spec.name == "alpha") {
      spec.critical = false;
      spec.owns_actuators = false;
    }
  }
  std::vector<NodeDescription> described;
  EXPECT_TRUE(naming::HasError(DescribeGraph(specs, fixture.options, described,
                                             {{"beta", {"teleop"}}})));
}

TEST(LauncherGraphTest,
     NonCriticalActuatorOwnersAreErrorsAfterSuccessfulProbe) {
  GraphFixture fixture{"optional_actuator",
                       DescribingStub("alpha", "//example/alpha:node", {}),
                       DescribingStub("beta", "//example/beta:node", {})};
  for (const auto* section : {"motors", "digital_outputs", "pwm_outputs"}) {
    auto config = toml::parse_file(fixture.options.config_path);
    auto* alpha = config["subsystems"]["alpha"].as_table();
    alpha->erase("motors");
    alpha->insert_or_assign("critical", false);
    toml::table device;
    device.insert(section == std::string_view{"motors"}            ? "can_id"
                  : section == std::string_view{"digital_outputs"} ? "dio"
                                                                   : "channel",
                  1);
    alpha->insert_or_assign(section,
                            toml::table{{"output", std::move(device)}});
    const auto parsed = config::ParseRobotConfigTable(std::move(config));
    std::vector<NodeDescription> described;
    EXPECT_TRUE(naming::HasError(DescribeGraph(
        DiscoverNodes(parsed, fixture.options), fixture.options, described)))
        << section;
    ASSERT_TRUE(described.at(1).answered);
  }
}

TEST(LauncherGraphTest, NonCriticalDescribeTimeoutAllowsPeersToLaunch) {
  GraphFixture fixture{
      "optional_timeout", kStubThatHangsOnDescribe,
      DescribingStub("beta", "//example/beta:node", {Sender("/beta/state")})};
  auto config = toml::parse_file(fixture.options.config_path);
  auto* alpha = config["subsystems"]["alpha"].as_table();
  alpha->erase("motors");
  alpha->insert_or_assign("critical", false);
  std::ofstream{fixture.options.config_path} << config;
  fixture.options.describe_timeout_ms = 100;
  fixture.options.max_restarts = 0;
  EXPECT_EQ(Launcher{fixture.options}.Run(), 0);
  EXPECT_TRUE(fixture.has_manifest());
  EXPECT_EQ(fixture.graph_json().find("\"severity\": \"error\""),
            std::string::npos);
}

TEST(LauncherGraphTest, NonCriticalDoesNotRelaxNamingRules) {
  GraphFixture fixture{"optional_invalid",
                       DescribingStub("alpha", "//example/alpha:node",
                                      {Sender("/invalid/topic")}),
                       DescribingStub("beta", "//example/beta:node", {})};
  auto config = toml::parse_file(fixture.options.config_path);
  auto* alpha = config["subsystems"]["alpha"].as_table();
  alpha->erase("motors");
  alpha->insert_or_assign("critical", false);
  std::ofstream{fixture.options.config_path} << config;
  EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
  EXPECT_FALSE(fixture.has_manifest());
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

TEST(LauncherGraphTest, DescribeOnlyEnforcesOptInPerModeTargetCoverage) {
  for (const bool required : {false, true}) {
    auto teleop_lane = Watcher("/alpha/target/teleop");
    auto auto_lane = Watcher("/alpha/target/auto");
    teleop_lane.flags = introspect::naming::kSourceFlagOptional;
    auto_lane.flags = introspect::naming::kSourceFlagOptional;
    GraphFixture fixture{
        required ? "required_mode_coverage" : "optional_mode_coverage",
        DescribingStub("alpha", "//example/alpha:node",
                       {teleop_lane, auto_lane}),
        DescribingStub("beta", "//example/beta:node",
                       {Sender("/alpha/target/teleop")})};
    std::string toml{kFixtureToml};
    const std::string header = "[robot]\n";
    toml.insert(toml.find(header) + header.size(),
                std::string{"require_target_coverage = "} +
                    (required ? "true\n" : "false\n"));
    {
      std::ofstream config{fixture.options.config_path};
      config << toml;
    }
    fixture.options.describe_only = true;
    Launcher launcher{fixture.options};
    EXPECT_EQ(launcher.Run(), required ? 1 : 0);
    EXPECT_FALSE(fixture.has_manifest());
    const auto graph = fixture.graph_json();
    EXPECT_NE(graph.find("/alpha/target/auto"), std::string::npos);
    EXPECT_EQ(graph.find("\"severity\": \"error\"") != std::string::npos,
              required);
  }
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
               d.subject == "alpha";
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
}

// A framework node that reads the declared graph, as a stub: described
// without --graph it registers nothing but says it reads the graph; with
// --graph it observes /alpha/state and keeps a copy of the graph it was
// given. Every run appends its arguments to `<record>/args`.
std::string GraphReaderStub(std::string_view node, std::string_view target,
                            const std::string& record) {
  introspect::Description first;
  first.name = std::string{node};
  first.target = std::string{target};
  first.reads_graph = true;
  introspect::Description second = first;
  auto observed = Watcher("/alpha/state");
  observed.flags = introspect::naming::kSourceFlagObserver;
  second.sources = {observed};
  return std::string{"#!/bin/sh\n"} + "echo \"$@\" >> '" + record +
         "/args'\n"
         "describe=0\n"
         "graph=''\n"
         "while [ $# -gt 0 ]; do\n"
         "  case \"$1\" in\n"
         "    --describe) describe=1 ;;\n"
         "    --graph) shift; graph=\"$1\" ;;\n"
         "  esac\n"
         "  shift\n"
         "done\n"
         "[ $describe = 1 ] || exit 0\n"
         "if [ -z \"$graph\" ]; then\n"
         "  cat <<'TALOS_DESCRIBE'\n" +
         introspect::DescribeToJson(first) +
         "TALOS_DESCRIBE\n"
         "  exit 0\n"
         "fi\n"
         "cp \"$graph\" '" +
         record +
         "/graph'\n"
         "cat <<'TALOS_DESCRIBE'\n" +
         introspect::DescribeToJson(second) + "TALOS_DESCRIBE\n";
}

TEST(LauncherGraphTest, AGraphReaderIsDescribedAgainWithTheOtherNodesGraph) {
  const std::string record = TempPath("graph_reader_record");
  std::filesystem::create_directories(record);
  NodeSpec alpha{"alpha", "//example/alpha:node",
                 WriteStubNode("graph_alpha",
                               DescribingStub("alpha", "//example/alpha:node",
                                              {Sender("/alpha/state")}))};
  NodeSpec gamma{
      "gamma", "//example/gamma:node",
      WriteStubNode("graph_gamma",
                    GraphReaderStub("gamma", "//example/gamma:node", record))};
  LauncherOptions options;
  options.describe_timeout_ms = 4000;
  options.output_dir = TempPath("graph_reader_out");
  std::vector<NodeDescription> described;
  const auto diagnostics = DescribeGraph({gamma, alpha}, options, described);

  ASSERT_EQ(described.size(), 2u);
  ASSERT_TRUE(described[0].answered) << described[0].error;
  EXPECT_TRUE(described[0].declared.reads_graph);
  ASSERT_EQ(described[0].declared.sources.size(), 1u);
  EXPECT_TRUE(described[0].declared.sources[0].observer());

  // The graph it was handed: alpha, every byte of its schema, and not gamma.
  introspect::DeclaredGraph graph;
  std::string error;
  ASSERT_TRUE(introspect::DescribeReader::ParseGraph(
      ReadFile(record + "/graph"), graph, error))
      << error;
  ASSERT_EQ(graph.size(), 1u);
  EXPECT_EQ(graph[0].name, "alpha");
  EXPECT_EQ(graph[0].sources.at(0).schema, "launcher_test.Message48");

  // An observer is not a consumer: alpha's state still has nobody reading it.
  const bool unread = std::any_of(
      diagnostics.begin(), diagnostics.end(),
      [](const introspect::naming::Diagnostic& d) {
        return d.subject == "/alpha/state" &&
               d.severity == introspect::naming::Severity::WARNING;
      });
  EXPECT_TRUE(unread);

  std::filesystem::remove_all(record);
  std::filesystem::remove_all(options.output_dir);
  std::remove(alpha.binary_path.c_str());
  std::remove(gamma.binary_path.c_str());
}


TEST(LauncherBundleTest, ResolveBinaryPrefersOverrideThenBazelBin) {
  std::map<std::string, std::string> overrides;
  overrides["//example/alpha:node"] = "/bin/alpha";
  overrides["@override_mod//:node"] = "/bin/override_mod";
  EXPECT_EQ(ResolveBinary("//example/alpha:node", overrides), "/bin/alpha");
  EXPECT_EQ(ResolveBinary("@override_mod//:node", overrides),
            "/bin/override_mod");
  EXPECT_EQ(ResolveBinary("//example/beta:node", overrides),
            "bazel-bin/example/beta/node");
  EXPECT_EQ(ResolveBinary("@wrist//:node", overrides),
            "bazel-bin/external/wrist+/node");
  EXPECT_EQ(ResolveBinary("@wrist//pkg:node", overrides),
            "bazel-bin/external/wrist+/pkg/node");
  EXPECT_EQ(ResolveBinary("@@wrist+//:node", overrides),
            "bazel-bin/external/wrist+/node");
  EXPECT_EQ(ResolveBinary("@@wrist+//pkg:node", overrides),
            "bazel-bin/external/wrist+/pkg/node");
  // The bundle manifest spells the baked (canonical) label while the robot
  // config spells the apparent one; the override still wins either way.
  overrides["@@wrist+//:node"] = "/opt/talos/bundles/current/bin/wrist";
  EXPECT_EQ(ResolveBinary("@wrist//:node", overrides),
            "/opt/talos/bundles/current/bin/wrist");
  EXPECT_EQ(ResolveBinary("@@wrist+//:node", overrides),
            "/opt/talos/bundles/current/bin/wrist");
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
  options.session_id = 1;

  Launcher launcher{options};
  EXPECT_EQ(launcher.Run(), 0);
  EXPECT_TRUE(std::filesystem::exists(bundle_dir + "/logs/1/graph.json"));

  std::filesystem::remove_all(bundle_dir);
}


LauncherOptions SessionFixture(std::string_view tag) {
  LauncherOptions options;
  options.config_path = WriteFixtureToml();
  options.output_dir = TempPath(tag);
  options.session_id = 1;
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
      std::string{tag} + "_hardware",
      DescribingStub("hardware_node", "//talOS/bridge:hardware_node", {}));
  options.binary_overrides["//example/alpha:node"] =
      WriteStubNode(std::string{tag} + "_alpha",
                    DescribingStub("alpha", "//example/alpha:node", {}));
  options.binary_overrides["//example/beta:node"] =
      WriteStubNode(std::string{tag} + "_beta",
                    DescribingStub("beta", "//example/beta:node", {}));
  return options;
}

TEST(LauncherSessions, TwoLaunchesPreserveBothSessions) {
  auto options = SessionFixture("two_sessions");
  std::filesystem::remove_all(options.output_dir);
  ASSERT_EQ(Launcher{options}.Run(), 0);
  const std::string first = ReadFile(options.output_dir + "/1/manifest.json");
  options.session_id = 2;
  ASSERT_EQ(Launcher{options}.Run(), 0);
  EXPECT_EQ(ReadFile(options.output_dir + "/1/manifest.json"), first);
  EXPECT_TRUE(std::filesystem::exists(options.output_dir + "/2/manifest.json"));
  std::filesystem::remove_all(options.output_dir);
}

TEST(LauncherSessions, RetentionDeletesOnlyOldSessions) {
  auto options = SessionFixture("retention");
  std::filesystem::remove_all(options.output_dir);
  std::string configuration{kFixtureToml};
  configuration.insert(configuration.find("period_us"),
                       "log_keep_sessions = 2\n");
  std::ofstream{options.config_path} << configuration;
  ASSERT_EQ(Launcher{options}.Run(), 0);
  options.session_id = 2;
  ASSERT_EQ(Launcher{options}.Run(), 0);
  const auto old =
      std::filesystem::file_time_type::clock::now() - std::chrono::hours{1};
  std::filesystem::last_write_time(options.output_dir + "/1", old);
  std::filesystem::create_directories(options.output_dir + "/unrelated");
  std::ofstream{options.output_dir + "/unrelated/keep"} << "untouched";
  options.session_id = 3;
  ASSERT_EQ(Launcher{options}.Run(), 0);
  EXPECT_FALSE(std::filesystem::exists(options.output_dir + "/1"));
  EXPECT_TRUE(std::filesystem::exists(options.output_dir + "/2/manifest.json"));
  EXPECT_TRUE(std::filesystem::exists(options.output_dir + "/3/manifest.json"));
  EXPECT_EQ(ReadFile(options.output_dir + "/unrelated/keep"), "untouched");
  std::filesystem::remove_all(options.output_dir);
}

TEST(LauncherSessions, KilledLauncherLeavesCompleteManifest) {
  auto options = SessionFixture("killed_manifest");
  std::filesystem::remove_all(options.output_dir);
  options.binary_overrides["//example/alpha:node"] = WriteStubNode(
      "killed_alpha", NodeDescribeThenSleep("alpha", "//example/alpha:node",
                                            {Sender("/alpha/state")}, 2));
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) ::_exit(Launcher{options}.Run());
  const std::string path = options.output_dir + "/1/manifest.json";
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds{5};
  std::string before;
  do {
    before = ReadFile(path);
    if (before.find("\"name\": \"alpha\"") != std::string::npos &&
        before.find("\"pid\": -1") == std::string::npos) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  } while (std::chrono::steady_clock::now() < deadline);
  ::kill(child, SIGKILL);
  int status = 0;
  ::waitpid(child, &status, 0);
  const std::string manifest = ReadFile(path);
  ASSERT_FALSE(manifest.empty());
  const std::string parse =
      "python3 -c 'import json,sys; json.load(open(sys.argv[1]))' '" + path +
      "'";
  EXPECT_EQ(std::system(parse.c_str()), 0);
  EXPECT_NE(manifest.find("\"end_wall_ns\": 0"), std::string::npos);
  EXPECT_NE(manifest.find("\"name\": \"alpha\""), std::string::npos);
  std::filesystem::remove_all(options.output_dir);
}

TEST(LauncherRemoteTest, SimulationRefusesPhysicalEffectiveRemote) {
  auto options = SessionFixture("sim_guard");
  options.simulation = true;
  options.remote = "10.12.34.2";
  EXPECT_EQ(Launcher{options}.Run(), 1);
  std::filesystem::remove_all(options.output_dir);
}

}  // namespace
}  // namespace talos::launcher
