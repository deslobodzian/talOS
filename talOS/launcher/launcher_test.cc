#include "talOS/launcher/launcher.h"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "talOS/driver_station/driver_station.h"
#include "talOS/events/input_replay.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/hardware/packet.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/launcher/file_digest.h"
#include "talOS/launcher/log_budget.h"
#include "talOS/launcher/log_merge.h"
#include "talOS/launcher/testdata/first_exec.h"
#include "talOS/launcher/testdata/supervision_events.h"

namespace talos::launcher {
namespace {

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_launcher_test_" +
         std::to_string(::getpid()) + "_" + std::string{name};
}

// A wait for an event gives up after this, and a session that ends on an
// event sets it as its duration: it only bounds a hang (docs/TESTING.md
// §6.3).
constexpr std::chrono::seconds kHang{20};

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

// REQ-OPS-30: the flag alone decides; --sim does not.
TEST(LauncherTest, StudioBridgeNeedsOnlyTheFlag) {
  const auto has_bridge = [](const std::vector<NodeSpec>& nodes) {
    return std::count_if(nodes.begin(), nodes.end(), [](const NodeSpec& n) {
      return n.name == "studio_bridge";
    });
  };
  // Without the flag (sim.sh --no-studio, or a deployed session that did
  // not ask), the session runs headless, sim or not.
  for (const bool simulation : {false, true}) {
    LauncherOptions no_flag;
    no_flag.simulation = simulation;
    EXPECT_EQ(has_bridge(DiscoverNodes(
                  config::ParseRobotConfigString(kFixtureToml), no_flag)),
              0)
        << "simulation=" << simulation;
  }
  // With it, declared with the framework default target, sim or not.
  for (const bool simulation : {false, true}) {
    LauncherOptions flag;
    flag.simulation = simulation;
    flag.start_studio_bridge = true;
    const auto nodes =
        DiscoverNodes(config::ParseRobotConfigString(kFixtureToml), flag);
    ASSERT_EQ(has_bridge(nodes), 1) << "simulation=" << simulation;
    for (const auto& n : nodes) {
      if (n.name == "studio_bridge") {
        EXPECT_EQ(n.target, "//studio/bridge:studio_bridge");
      }
    }
  }
}

TEST(LauncherTest, StudioBridgeWatchTopicFollowsDescribe) {
  // A description without a watcher: the config's studio topic, else the
  // default.
  config::RobotConfig robot;
  NodeDescription bridge;
  bridge.roster_name = "studio_bridge";
  bridge.answered = true;
  bridge.declared.name = "studio_bridge";
  EXPECT_EQ(StudioBridgeWatchTopic(bridge, robot), "/talos/telemetry");
  robot.studio.topic = "/config/telemetry";
  EXPECT_EQ(StudioBridgeWatchTopic(bridge, robot), "/config/telemetry");

  // The bridge's own WATCHER wins, tracking a topic override.
  introspect::naming::SourceShape watcher{event::SourceKind::WATCHER,
                                          "/custom/telemetry", 48, 0};
  bridge.declared.sources.push_back(watcher);
  EXPECT_EQ(StudioBridgeWatchTopic(bridge, robot), "/custom/telemetry");
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

// ARCH-3: a misspelled parameter used to load and leave the node on its
// fallback. A key no node's describe reports reading is an error naming its
// file and the nearest read key; framework sections, an array read by
// element, an empty table read inside, nested and array-of-table keys and
// another node's param_of all count as read, but a read under a scalar
// (`recording.enabled` where `recording = false`, `count[1]` where
// `count = 5`) does not.
TEST(LauncherTest, ASubsystemKeyNoNodeReadsIsAnErrorNamingTheNearestReadKey) {
  constexpr std::string_view kToml = R"(
[subsystems.elevator]
node = "//e:elevator"
period_us = 10000
homing_vv = -3.0
tolerance_rot = 0.1
names = ["a", "b"]
recording = false
count = 5
[subsystems.elevator.detector]
sigma = 0.0
[subsystems.elevator.routine.none]
[subsystems.elevator.motors.lift]
can_id = 2
[[subsystems.elevator.geometry.modules]]
x_m = 1.0

[subsystems.drivetrain]
node = "//d:drivetrain"
offset_m = 0.5
)";
  const std::string path = TempPath("parameter_keys.toml");
  std::ofstream{path} << kToml;
  config::RobotConfig robot;
  robot.toml_data = toml::parse(kToml);

  NodeDescription elevator{.roster_name = "elevator", .answered = true};
  elevator.declared.params = {{"elevator", "homing_v"},
                              {"elevator", "tolerance_rot"},
                              {"elevator", "names[1]"},
                              {"elevator", "recording.enabled"},
                              {"elevator", "count[1]"},
                              {"elevator", "detector.sigma"},
                              {"elevator", "routine.none.speed_rps"},
                              {"elevator", "geometry.modules[0].x_m"},
                              {"drivetrain", "offset_m"}};
  NodeDescription drivetrain{.roster_name = "drivetrain", .answered = true};
  drivetrain.declared.params.emplace();
  // Not a subsystem, and its describe has no params: ignored.
  const NodeDescription bridge{.roster_name = std::string{kHardwareNodeName},
                               .answered = true};

  const auto subjects = [](const std::vector<naming::Diagnostic>& findings) {
    std::vector<std::string> keys;
    for (const auto& finding : findings) {
      const auto begin = finding.message.find('\'') + 1;
      keys.push_back(finding.message.substr(
          begin, finding.message.find('\'', begin) - begin));
    }
    return keys;
  };
  const auto findings =
      LintParameterKeys(robot, {elevator, drivetrain, bridge}, path);
  EXPECT_EQ(subjects(findings),
            (std::vector<std::string>{"count", "homing_vv", "recording"}));
  for (const auto& finding : findings) {
    EXPECT_EQ(finding.severity, naming::Severity::ERROR) << finding.message;
    EXPECT_EQ(finding.subject, "elevator");
  }
  ASSERT_EQ(findings.size(), 3u);
  const std::string& typo = findings[1].message;  // Sorted by key.
  EXPECT_NE(typo.find("'homing_vv' (" + path + ")"), std::string::npos) << typo;
  EXPECT_NE(typo.find("did you mean 'homing_v'?"), std::string::npos) << typo;

  // A failed probe leaves only its own table unchecked: a key nobody else
  // reads stays an error, naming the node that might read it.
  drivetrain.answered = false;
  drivetrain.declared.params.reset();
  const auto failed = LintParameterKeys(robot, {elevator, drivetrain}, path);
  ASSERT_EQ(subjects(failed),
            (std::vector<std::string>{"count", "homing_vv", "recording"}));
  EXPECT_EQ(failed[0].severity, naming::Severity::ERROR);
  EXPECT_NE(failed[0].message.find("drivetrain did not answer"),
            std::string::npos)
      << failed[0].message;

  // A node that answered without a params field (an older binary): its own
  // table goes unchecked, and since it may read another table with
  // param_of, a key nobody else reads is only a warning.
  drivetrain.answered = true;
  const auto unsure = LintParameterKeys(robot, {elevator, drivetrain}, path);
  EXPECT_EQ(unsure.size(), 4u);
  EXPECT_FALSE(naming::HasError(unsure));
  std::filesystem::remove(path);
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
introspect::naming::SourceShape Endpoint(event::SourceKind kind,
                                         std::string topic,
                                         std::uint32_t bytes);
std::string ReadFile(const std::string& path);

TEST(LauncherTest, DescribeDeadlineAppliesAfterStdoutCloses) {
  const auto binary = WriteStubNode("describe_closed_stdout",
                                    "#!/bin/sh\nexec 1>&-\nexec sleep 30\n");
  introspect::Description description;
  std::string error;
  const auto started = std::chrono::steady_clock::now();
  EXPECT_FALSE(ProbeDescribe({binary, "--describe"},
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
  EXPECT_FALSE(ProbeDescribe({binary, "--describe"}, std::chrono::seconds{5},
                             description, error));
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
// blocked for the old ~60s startup wait. The test moves the supervision
// clock one retry interval ahead once the launcher has reaped the first
// attempt, and the nodes exit once the second has failed. Moved on the
// attempt's report alone, the clock could pass the interval while the
// attempt was still exiting, and a pass would take it for a bridge that
// came up.
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
  // The stubs never take a registry row; nothing here waits for one.
  options.heartbeat_startup_grace_ms = 60000;
  testing::SteppedClock clock;
  options.supervision_clock = clock.clock();
  options.config_path = WriteFixtureToml();
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
      tag + "_hardware",
      DescribingStub("hardware_node", "//talOS/bridge:hardware_node", {}));
  const auto until_retried =
      [&](std::string_view node, std::string_view target,
          const introspect::naming::SourceShape& source) {
        std::string script = NodeDescribeThenSleep(node, target, {source}, 0);
        script.erase(script.rfind("exec sleep"));
        return script + "tries=0\nwhile [ \"$(wc -l < '" + counter +
               "' 2>/dev/null || echo 0)\" -lt 2 ] && [ $tries -lt 2000 ]; do\n"
               "  tries=$((tries + 1))\n  sleep 0.01\ndone\nexit 0\n";
      };
  options.binary_overrides["//example/alpha:node"] = WriteStubNode(
      tag + "_alpha",
      until_retried("alpha", "//example/alpha:node", Sender("/alpha/state")));
  options.binary_overrides["//example/beta:node"] = WriteStubNode(
      tag + "_beta",
      until_retried("beta", "//example/beta:node", Watcher("/alpha/state")));
  options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStubNode(tag + "_bridge",
                    BridgeDescribeThen("echo $$ >> " + counter + "\nexit 1\n"));

  std::jthread advance{[&](const std::stop_token& stop) {
    const auto give_up = std::chrono::steady_clock::now() + kHang;
    const auto wait = [&](const std::function<bool()>& done) {
      while (!stop.stop_requested() && !done() &&
             std::chrono::steady_clock::now() < give_up)
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    };
    pid_t first = 0;
    wait([&] {
      std::istringstream{ReadFile(counter)} >> first;
      return first > 0;
    });
    // Gone once reaped: its exit was handled as a starting attempt's.
    wait([&] { return ::kill(first, 0) != 0 && errno == ESRCH; });
    clock.Advance(kStudioBridgeRetryInterval);
  }};
  const auto started = std::chrono::steady_clock::now();
  Launcher launcher{options};
  launcher.Run();
  const auto elapsed = std::chrono::steady_clock::now() - started;
  advance.request_stop();

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
  options.duration_s = static_cast<int>(kHang.count());
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
      "runs=$(wc -l < '" +
      attempts +
      "')\n"
      "if [ \"$runs\" -eq 1 ]; then\n"
      " dd if=/dev/zero of=\"$log\" bs=1 count=100 2>/dev/null\n"
      // The launcher writes manifest.json.tmp itself and renames it; wait
      // until its own temporary file is gone before planting /dev/full.
      " until ln -s /dev/full '" +
      session +
      "/manifest.json.tmp' 2>/dev/null; do sleep 0.01; done\n"
      " exit 1\nfi\n"
      // The second run fills what it was given.
      "if [ \"$runs\" -eq 2 ]; then\n"
      " dd if=/dev/zero of=\"$log\" bs=1 count=\"$cap\" 2>/dev/null\n"
      " exit 1\nfi\n"
      "exec sleep 30\n";
  options.binary_overrides["//example/alpha:node"] =
      WriteStubNode("budget_alpha", script);
  options.binary_overrides["//example/beta:node"] = WriteStubNode(
      "budget_beta", NodeDescribeThenSleep("beta", "//example/beta:node",
                                           {Watcher("/alpha/state")}, 30));
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
      "budget_hw",
      NodeDescribeThenSleep("hardware_node", "//talOS/bridge:hardware_node", {},
                            30));
  // Stopped (SIGTERM, as an operator) once the third run has its cap.
  std::jthread stopper{[&](const std::stop_token& stop) {
    while (!stop.stop_requested()) {
      const std::string caps = ReadFile(attempts);
      if (std::count(caps.begin(), caps.end(), '\n') >= 3) {
        ::kill(::getpid(), SIGTERM);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
  }};
  Launcher launcher{options};
  launcher.Run();
  stopper.request_stop();
  // The first restart gets alpha's rate-weighted share less the 100 bytes
  // its first run wrote. The second, with the share used up, gets half of
  // it: the first run's log is deleted (and marked capped) to make room,
  // the second run's, the newest, is kept, and half is the floor.
  std::istringstream caps{ReadFile(attempts)};
  uint64_t first = 0;
  uint64_t second = 0;
  uint64_t third = 0;
  ASSERT_TRUE(caps >> first >> second >> third);
  EXPECT_GT(first, 200u);
  EXPECT_EQ(second, first - 100);
  EXPECT_EQ(third, first / 2);
  EXPECT_FALSE(std::filesystem::exists(session + "/alpha.tlog"));
  EXPECT_TRUE(std::filesystem::exists(session + "/alpha.tlog.capped"));
  EXPECT_TRUE(std::filesystem::exists(session + "/alpha.restart1.tlog"));
  const auto manifest = ReadFile(session + "/manifest.json");
  EXPECT_NE(manifest.find("\"restart_count\": 2"), std::string::npos);
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
  options.duration_s = 1;
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

// A member that ignores SIGINT is killed (SIGKILL, run reason stop_timeout)
// at the stop timeout (kStopTimeout), neither before nor after. The test
// freezes the supervision clock at the stop, holds it 100 ms short of the
// timeout for four passes after its peer died of the stop's SIGINT (alpha
// must outlive them), then at the timeout itself (alpha must be killed, and
// the session end, within four passes).
TEST(LauncherTest, ShutdownKillsChildrenIgnoringInterrupt) {
  const std::string tag = "ignore_interrupt";
  const std::string out_dir = TempPath(tag + "_out");
  std::filesystem::create_directories(out_dir);
  const std::string trapped = TempPath(tag + "_trapped");
  ::remove(trapped.c_str());
  LauncherOptions options;
  options.output_dir = out_dir;
  options.session_id = 0xDEAD2200ULL;
  options.simulation = true;
  options.describe_timeout_ms = 4000;
  options.duration_s = static_cast<int>(kHang.count());
  // The stubs never take a registry row; nothing here waits for one.
  options.heartbeat_startup_grace_ms = 60000;
  testing::SteppedClock clock;
  options.supervision_clock = clock.clock();
  options.config_path = WriteFixtureToml();
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
      tag + "_hardware",
      NodeDescribeThenSleep("hardware_node", "//talOS/bridge:hardware_node", {},
                            30));
  auto stubborn = NodeDescribeThenSleep("alpha", "//example/alpha:node",
                                        {Sender("/alpha/state")}, 30);
  stubborn.insert(stubborn.find("exec sleep"),
                  "trap '' INT\n: > '" + trapped + "'\n");
  const auto alpha = WriteStubNode(tag + "_alpha", stubborn);
  const auto beta = WriteStubNode(
      tag + "_beta", NodeDescribeThenSleep("beta", "//example/beta:node",
                                           {Watcher("/alpha/state")}, 30));
  options.binary_overrides["//example/alpha:node"] = alpha;
  options.binary_overrides["//example/beta:node"] = beta;
  const std::string live =
      out_dir + "/" + std::to_string(options.session_id) + "/manifest.json";
  // `name`'s runs as `manifest` lists them.
  const auto runs_of = [](const std::string& manifest, std::string_view name) {
    const auto at = manifest.find("\"name\": \"" + std::string{name} + "\"");
    if (at == std::string::npos) return std::string{};
    const auto runs = manifest.find("\"runs\": [", at);
    if (runs == std::string::npos) return std::string{};
    return manifest.substr(runs, manifest.find(']', runs) - runs);
  };
  testing::SupervisionPasses passes;
  bool beta_stopped = false;  // The stop's SIGINT ended alpha's peers.
  // Four passes ran with the clock 100 ms short of the timeout and the
  // session still on: a kill in any of the first three would have been
  // reaped and ended the session (alpha is its last member) before the
  // fourth. And when that was: alpha's run must end after it.
  bool held_short = false;
  int64_t short_of_timeout_ns = 0;
  // Four passes ran with the clock at the timeout and the session still on:
  // one may have read the clock before it moved, one kills alpha, one reaps
  // it and ends the session, and a fourth means it was not killed.
  bool late = false;
  std::jthread stopper{[&](const std::stop_token& stop) {
    // Every exit, a give-up included, lets the launcher's own clock run on,
    // so its duration ends the session and the test fails with a message.
    const testing::SteppedClock::ReleaseOnExit release{clock};
    const auto wait = [&](const std::function<bool()>& done) {
      const auto give_up = std::chrono::steady_clock::now() + kHang;
      while (!stop.stop_requested() && !done() &&
             std::chrono::steady_clock::now() < give_up)
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
      return !stop.stop_requested() && done();
    };
    const auto passes_from_now = [&](uint64_t count) {
      const uint64_t until = passes.Started() + count;
      return wait([&] { return passes.Started() >= until; });
    };
    if (!wait([&] { return std::filesystem::exists(trapped); })) return;
    // The stop's pass reads exactly `stopped_at`.
    const SupervisionTime stopped_at = clock.Freeze();
    ::kill(::getpid(), SIGTERM);
    beta_stopped = wait([&] {
      const std::string manifest = ReadFile(live);
      return runs_of(manifest, "beta").find("\"reason\": \"stop\"") !=
                 std::string::npos &&
             runs_of(manifest, "hardware_node").find("\"reason\": \"stop\"") !=
                 std::string::npos;
    });
    if (!beta_stopped) return;
    clock.HoldAt(stopped_at + kStopTimeout - std::chrono::milliseconds{100});
    held_short = passes_from_now(4);
    if (!held_short) return;
    short_of_timeout_ns = WallClockNanos();
    clock.HoldAt(stopped_at + kStopTimeout);
    late = passes_from_now(4);
  }};
  const auto started = std::chrono::steady_clock::now();
  Launcher launcher{options};
  EXPECT_EQ(launcher.Run(), -1);
  const auto elapsed = std::chrono::steady_clock::now() - started;
  stopper.request_stop();
  stopper.join();
  EXPECT_LT(elapsed, std::chrono::seconds{10});
  ASSERT_TRUE(beta_stopped) << "the stop's SIGINT ended no peer";
  ASSERT_TRUE(held_short) << "alpha was killed before the stop timeout: the "
                             "session ended while the clock was held short "
                             "of it";
  EXPECT_FALSE(late)
      << "alpha was not killed once the clock reached the stop timeout";
  const auto run = runs_of(ReadFile(live), "alpha");
  EXPECT_NE(run.find("\"exit_signal\": 9, \"reason\": \"stop_timeout\""),
            std::string::npos)
      << run;
  const std::string end_key = "\"end_wall_ns\": ";
  const auto end_at = run.find(end_key);
  ASSERT_NE(end_at, std::string::npos) << run;
  const int64_t ended_ns = std::stoll(run.substr(end_at + end_key.size()));
  EXPECT_GE(ended_ns, short_of_timeout_ns)
      << "alpha was killed before the stop timeout: " << run;
  std::filesystem::remove(alpha);
  std::filesystem::remove(beta);
  std::filesystem::remove_all(out_dir);
  ::remove(trapped.c_str());
}

// A bridge that survives its grace second joins the session: it is not failed
// and the session runs to the deadline with it. A session against the real
// controller (no --sim) starts it too, so a deployed robot is viewable in
// Studio (REQ-OPS-30). Once the bridge has spawned, the test moves the
// supervision clock past the 3 s deadline, a whole retry interval after the
// spawn: the deadline's pass finds the bridge up that long, so stable.
TEST(LauncherTest, StableBridgeJoinsDeployedSessionUntilDeadline) {
  const std::string tag = "bridge_stable";
  const std::string out_dir = TempPath(tag + "_out");
  std::filesystem::create_directories(out_dir);

  LauncherOptions options;
  options.output_dir = out_dir;
  options.session_id = 0x57AB1EULL;
  options.start_studio_bridge = true;
  options.describe_timeout_ms = 4000;
  options.duration_s = 3;
  // The stubs never take a registry row; nothing here waits for one.
  options.heartbeat_startup_grace_ms = 60000;
  testing::SteppedClock clock;
  options.supervision_clock = clock.clock();
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
  const std::string session =
      out_dir + "/" + std::to_string(options.session_id);
  // The bridge's first run in the running session's manifest: spawned.
  const auto bridge_spawned = [](const std::string& manifest) {
    const auto bridge = manifest.find("\"name\": \"studio_bridge\"");
    const auto runs = manifest.find("\"runs\": [", bridge);
    return bridge != std::string::npos && runs != std::string::npos &&
           manifest.find("\n  \"end_wall_ns\": 0,") != std::string::npos &&
           manifest.find("{\"pid\": ", runs) < manifest.find(']', runs) &&
           manifest.find("{\"pid\": -1", runs) > manifest.find(']', runs);
  };
  bool advanced = false;
  std::jthread advance{[&](const std::stop_token& stop) {
    const auto give_up = std::chrono::steady_clock::now() + kHang;
    while (!stop.stop_requested() &&
           !bridge_spawned(ReadFile(session + "/manifest.json"))) {
      if (std::chrono::steady_clock::now() >= give_up) return;
      std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    if (stop.stop_requested()) return;
    advanced = true;
    clock.Advance(std::chrono::seconds{options.duration_s});
  }};

  const auto started = std::chrono::steady_clock::now();
  Launcher launcher{options};
  launcher.Run();
  const auto elapsed = std::chrono::steady_clock::now() - started;
  advance.request_stop();
  advance.join();

  EXPECT_TRUE(advanced) << "the bridge never spawned";
  EXPECT_LT(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(),
            20)
      << "session must end at the deadline";
  ASSERT_TRUE(std::filesystem::exists(session + "/manifest.json"));
  const std::string manifest = ReadFile(session + "/manifest.json");
  EXPECT_NE(manifest.find("\"simulation\": false"), std::string::npos)
      << manifest;
  const std::size_t bridge = manifest.find("\"name\": \"studio_bridge\"");
  ASSERT_NE(bridge, std::string::npos) << manifest;
  // Its first run was a spawned process (a run never spawned has pid -1).
  const std::size_t runs = manifest.find("\"runs\": [", bridge);
  ASSERT_NE(runs, std::string::npos) << manifest;
  EXPECT_NE(manifest.find("{\"pid\": ", runs),
            manifest.find("{\"pid\": -1", runs))
      << manifest;
  // A stable viewer is a session member, not a failure.
  EXPECT_EQ(manifest.find("\"recorder_failed\": true"), std::string::npos)
      << manifest;

  std::filesystem::remove_all(out_dir);
}

// A log cut while it was being created (a short or zero-filled header) is
// skipped by every session reader, not taken for corruption of the session.
TEST(LauncherTest, SessionReadersSkipALogCutBeforeItsHeader) {
  const std::string dir = TempPath("cut_header");
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  const uint64_t session_id = 77;
  WriteTestLog(dir + "/whole.tlog", "whole", session_id, {1'000'000});
  WriteTestLog(dir + "/short.tlog", "short", session_id, {2'000'000});
  std::filesystem::resize_file(dir + "/short.tlog",
                               sizeof(event::log::FileHeader) / 2);
  std::ofstream{dir + "/zeros.tlog", std::ios::binary}
      << std::string(sizeof(event::log::FileHeader) + 64, '\0');

  std::vector<std::string> merged;
  std::string error;
  ASSERT_TRUE(MergeLogs(
      event::log::FindLogFiles(dir),
      [&](const MergedRecord& r) {
        merged.push_back(r.process_name());
        return true;
      },
      error))
      << error;
  EXPECT_EQ(merged, std::vector<std::string>{"whole"});

  const event::RecordedSession session{dir};
  EXPECT_EQ(session.session_id(), session_id);

  EXPECT_THROW(event::log::SummarizeLog(dir + "/zeros.tlog"),
               event::log::IncompleteLogHeader);
  std::filesystem::remove_all(dir);
}

// REQ-OPS-32: a member's expected log bytes count what its log stores, at
// the rate it stores it: /hw/state once per gateway tick and trimmed to its
// encoded State (not the 1436 B Packet), for a 10 ms member too; a
// latest-only watcher at most once per tick; the bridge only the State and
// commands it logs, not the ends it declares and never logs.
TEST(LauncherTest, ExpectedLogRateCountsWhatEachLogStoresAtItsRate) {
  const auto robot = config::ParseRobotConfigString(kFixtureToml);
  using event::SourceKind;
  using introspect::naming::SourceShape;
  const auto packet = [](SourceKind kind, std::string topic) {
    return Endpoint(kind, std::move(topic), sizeof(hardware::Packet));
  };
  SourceShape latest = Watcher("/alpha/state", 48);
  latest.flags = introspect::naming::kSourceFlagLatestOnly;
  NodeDescription alpha{.roster_name = "alpha", .answered = true};
  alpha.declared = {"alpha", "", {Sender("/alpha/state", 48)}};
  NodeDescription beta{.roster_name = "beta", .answered = true};
  beta.declared = {
      "beta",
      "",
      {packet(SourceKind::WATCHER, hardware::kStateTopic), latest,
       packet(SourceKind::SENDER, hardware::RequestTopic("beta"))}};
  NodeDescription bridge{.roster_name = "hardware_node", .answered = true};
  bridge.declared = {
      "hardware_node",
      "",
      {packet(SourceKind::SENDER, hardware::kStateTopic),
       packet(SourceKind::SENDER, hardware::kCommandTopic),
       packet(SourceKind::SENDER, hardware::kDriverStationTopic),
       packet(SourceKind::FETCHER, hardware::RequestTopic("alpha")),
       packet(SourceKind::FETCHER, hardware::RequestTopic("beta"))}};
  const TopicLoads loads = DeclaredTopicLoads({alpha, beta, bridge}, robot);

  // Two motors: 84 + 2 x 45 + 34 encoded bytes after the Packet's size word.
  const TopicLoad state = loads.at(hardware::kStateTopic);
  EXPECT_DOUBLE_EQ(state.per_second, 200);
  EXPECT_DOUBLE_EQ(state.stored_bytes, 4 + 84 + 2 * 45 + 34);
  // A request carries its owner's one motor: 38 + 20.
  EXPECT_DOUBLE_EQ(loads.at(hardware::RequestTopic("beta")).stored_bytes,
                   4 + 38 + 20);
  // At most one command per owner request: here only beta's (alpha's
  // describe sends none).
  const TopicLoad command = loads.at(hardware::kCommandTopic);
  EXPECT_DOUBLE_EQ(command.per_second, 100);
  EXPECT_DOUBLE_EQ(command.stored_bytes, 4 + 38 + 20);

  // beta at 100 Hz: every State (200/s), the newest /alpha/state per tick
  // (100 of 200/s), and its request per tick.
  EXPECT_DOUBLE_EQ(
      ExpectedLogRate(beta.declared, 10000, false, loads),
      200 * (64 + state.stored_bytes) + 100 * (64 + 48) + 100 * (64 + 62));
  // The bridge: the States it publishes and the commands it sends, nothing
  // for the Driver Station packets or the requests it only relays.
  EXPECT_DOUBLE_EQ(ExpectedLogRate(bridge.declared, 5000, true, loads),
                   200 * (64 + state.stored_bytes) + 100 * (64 + 62));
}

// A control loop the OS kept at timeshare is warned about and recorded in the
// manifest session_report --strict judges. A later run that got its grant
// does not clear it: the session still ran that loop without one. Only the
// actuation path is warned about, and --sim excuses a loop that never asked.
TEST(LauncherTest, ARefusedRealtimeGrantIsWarnedAndOutlivesALaterFullRun) {
  using process::LoopTier;
  const process::RealtimeOutcome refused{.tier = LoopTier::kControl,
                                         .requested = true,
                                         .refused = process::kStepPolicy};
  const process::RealtimeOutcome full{.tier = LoopTier::kControl,
                                      .requested = true};
  const process::RealtimeOutcome unasked{.tier = LoopTier::kControl};

  EXPECT_NE(RealtimeWarning("shooter", 42, refused, true).find("shooter"),
            std::string::npos);
  EXPECT_FALSE(RealtimeWarning("shooter", 42, unasked, false).empty());
  EXPECT_TRUE(RealtimeWarning("shooter", 42, unasked, true).empty());
  EXPECT_TRUE(RealtimeWarning("shooter", 42, full, false).empty());
  EXPECT_TRUE(RealtimeWarning("telemetry", 42,
                              {.tier = LoopTier::kObserver,
                               .requested = true,
                               .refused = process::kStepPolicy},
                              false)
                  .empty());

  NodeProcessInfo shooter;
  shooter.name = "shooter";
  EXPECT_TRUE(shooter.RecordRealtime(refused));
  EXPECT_FALSE(shooter.RecordRealtime(full));
  SessionManifest manifest;
  manifest.nodes.push_back(shooter);
  const std::string json = manifest.ToJson();
  EXPECT_NE(json.find("\"loop_tier\": \"control\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"realtime\": \"refused\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"realtime_refused\": \"policy\""), std::string::npos)
      << json;
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
  // A config cannot turn off critical members' restarts; only the flag can.
  EXPECT_FALSE(ResolveMaxRestarts(with_robot_keys("max_restarts = 0\n"), {},
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
//
// Stubs live in a directory of their own, so no TempPath a test means to be
// missing (a never-built binary) or to hold output can name one.
std::string WriteStubNode(std::string_view name, std::string_view script) {
  const std::string dir = ::testing::TempDir() + "/talos_launcher_stubs_" +
                          std::to_string(::getpid());
  std::filesystem::create_directories(dir);
  return testing::WriteWarmStub(dir + "/" + std::string{name}, script);
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

// DescribingStub that first appends "<node> <its arguments>" to `record`, on
// every invocation, --describe or not.
std::string RecordingStub(
    std::string_view node, std::string_view target, const std::string& record,
    const std::vector<introspect::naming::SourceShape>& sources = {}) {
  const std::string stub = DescribingStub(node, target, sources);
  const std::string shebang = "#!/bin/sh\n";
  return shebang + "echo \"" + std::string{node} + " $*\" >> '" + record +
         "'\n" + stub.substr(shebang.size());
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
  fixture.options.simulation = true;

  Launcher launcher{fixture.options};
  EXPECT_EQ(launcher.Run(), 0);
  EXPECT_TRUE(fixture.has_manifest());
  EXPECT_NE(fixture.graph_json().find("\"severity\": \"error\""),
            std::string::npos);
}

// REQ-CONTRACT-09: --allow-graph-errors overrides every graph refusal, the
// safety ones included (REQ-SAFE-27/28), so it is accepted only with --sim,
// whose loopback remote keeps it off a real controller. Without --sim the
// launch is refused before any member is probed or spawned and before any
// session directory exists, --describe-only included; every member's stub
// records each invocation, --describe or not.
TEST(LauncherGraphTest, AllowGraphErrorsWithoutSimIsRefusedBeforeAnyProbe) {
  for (const bool describe_only : {false, true}) {
    SCOPED_TRACE(describe_only ? "--describe-only" : "launch");
    const std::string invoked = TempPath("allow_without_sim_invoked");
    ::remove(invoked.c_str());
    GraphFixture fixture{
        "allow_without_sim",
        RecordingStub("alpha", "//example/alpha:node", invoked),
        RecordingStub("beta", "//example/beta:node", invoked)};
    fixture.options.binary_overrides["//talOS/bridge:hardware_node"] =
        WriteStubNode("allow_without_sim_hardware",
                      RecordingStub("hardware_node",
                                    "//talOS/bridge:hardware_node", invoked));
    fixture.options.allow_graph_errors = true;
    fixture.options.describe_only = describe_only;

    EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
    EXPECT_FALSE(std::filesystem::exists(invoked)) << ReadFile(invoked);
    EXPECT_FALSE(std::filesystem::exists(fixture.out_dir)) << fixture.out_dir;
    ::remove(invoked.c_str());
  }
}

// TODO.md SIM-4: a config whose `[robot] simulation_only = true` (here set by
// a simulation config that includes the deployable fixture) runs only with
// --sim. Without it the launch is refused before any member is probed or
// spawned and before any session directory exists; with --sim it runs, and
// --describe-only, which touches no hardware, lints it.
// REQ-SAFE-40
TEST(LauncherGraphTest, ASimulationOnlyConfigIsRefusedWithoutSim) {
  struct Row {
    const char* name;
    bool simulation;
    bool describe_only;
    bool refused;
  };
  for (const Row row :
       {Row{"launch", false, false, true}, Row{"--sim", true, false, false},
        Row{"--describe-only", false, true, false}}) {
    SCOPED_TRACE(row.name);
    const std::string invoked = TempPath("simulation_only_invoked");
    ::remove(invoked.c_str());
    GraphFixture fixture{
        "simulation_only",
        RecordingStub("alpha", "//example/alpha:node", invoked),
        RecordingStub("beta", "//example/beta:node", invoked)};
    fixture.options.binary_overrides["//talOS/bridge:hardware_node"] =
        WriteStubNode("simulation_only_hardware",
                      RecordingStub("hardware_node",
                                    "//talOS/bridge:hardware_node", invoked));
    const std::string sim_config = TempPath("simulation_only") + ".toml";
    std::ofstream{sim_config} << "include = \"" << fixture.options.config_path
                              << "\"\n[robot]\nsimulation_only = true\n";
    fixture.options.config_path = sim_config;
    fixture.options.simulation = row.simulation;
    fixture.options.describe_only = row.describe_only;

    EXPECT_EQ(Launcher{fixture.options}.Run(), row.refused ? 1 : 0);
    EXPECT_EQ(std::filesystem::exists(invoked), !row.refused)
        << ReadFile(invoked);
    EXPECT_EQ(std::filesystem::exists(fixture.out_dir), !row.refused)
        << fixture.out_dir;
    ::remove(invoked.c_str());
    ::remove(sim_config.c_str());
  }
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
  fixture.options.simulation = true;
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

// The error diagnostics graph.json records, one per line.
std::vector<std::string> GraphErrors(const std::string& graph) {
  std::vector<std::string> errors;
  std::istringstream lines{graph};
  for (std::string line; std::getline(lines, line);) {
    if (line.find("\"severity\": \"error\"") != std::string::npos) {
      errors.push_back(line);
    }
  }
  return errors;
}

// TOOLS-3: a launch before the build finds no binaries. That is one refusal
// carrying the build command for every missing node, not one "did not
// answer" per node; a binary that exists but does not answer keeps its own.
// REQ-OPS-01
TEST(LauncherGraphTest, MissingBinariesAreOneRefusalWithTheBuildCommand) {
  GraphFixture fixture{"not_built", kStubWithoutDescribe, kStubWithoutDescribe};
  fixture.options.describe_only = true;
  fixture.options.binary_overrides["//example/alpha:node"] =
      TempPath("never_built_alpha");
  fixture.options.binary_overrides["//example/beta:node"] =
      TempPath("never_built_beta");

  EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
  EXPECT_FALSE(fixture.has_manifest());
  const auto errors = GraphErrors(fixture.graph_json());
  ASSERT_EQ(errors.size(), 1u) << fixture.graph_json();
  EXPECT_NE(errors[0].find("//example/alpha:node"), std::string::npos)
      << errors[0];
  EXPECT_NE(errors[0].find("//example/beta:node"), std::string::npos)
      << errors[0];
  EXPECT_EQ(errors[0].find("//talOS/bridge:hardware_node"), std::string::npos)
      << "the built bridge is not to be rebuilt: " << errors[0];
}

// REQ-OPS-01
TEST(LauncherGraphTest, MissingBinaryDoesNotHideOneThatDoesNotAnswer) {
  GraphFixture fixture{"half_built", kStubWithoutDescribe,
                       kStubWithoutDescribe};
  fixture.options.describe_only = true;
  fixture.options.binary_overrides["//example/alpha:node"] =
      TempPath("never_built_alpha");

  EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
  const auto errors = GraphErrors(fixture.graph_json());
  ASSERT_EQ(errors.size(), 2u) << fixture.graph_json();
  const auto missing =
      std::find_if(errors.begin(), errors.end(), [](const std::string& error) {
        return error.find("//example/alpha:node") != std::string::npos;
      });
  ASSERT_NE(missing, errors.end()) << fixture.graph_json();
  EXPECT_EQ(missing->find("//example/beta:node"), std::string::npos)
      << *missing;
  EXPECT_TRUE(std::any_of(errors.begin(), errors.end(),
                          [](const std::string& error) {
                            return error.find("\"subject\": \"beta\"") !=
                                   std::string::npos;
                          }))
      << fixture.graph_json();
}

// The diagnostics of `severity` graph.json records about `subject`, one per
// line.
std::vector<std::string> GraphFindings(const std::string& graph,
                                       std::string_view severity,
                                       std::string_view subject) {
  std::vector<std::string> found;
  std::istringstream lines{graph};
  for (std::string line; std::getline(lines, line);) {
    if (line.find("\"severity\": \"" + std::string{severity} + "\"") !=
            std::string::npos &&
        line.find("\"subject\": \"" + std::string{subject} + "\"") !=
            std::string::npos) {
      found.push_back(line);
    }
  }
  return found;
}

// REQ-OPS-30, DEPLOY-4: --start-studio-bridge from a checkout that has not
// built the bridge runs the session without Studio, with one warning
// carrying the build command, not a refused graph: the bridge is an
// observer, critical for no other member. A missing critical node is still
// a refusal beside it.
TEST(LauncherGraphTest, AnUnbuiltStudioBridgeIsAWarningAndLeavesTheSession) {
  GraphFixture fixture{
      "studio_unbuilt",
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")}),
      DescribingStub("beta", "//example/beta:node", {Watcher("/alpha/state")})};
  fixture.options.start_studio_bridge = true;
  fixture.options.binary_overrides["//studio/bridge:studio_bridge"] =
      TempPath("never_built_studio_bridge");

  EXPECT_EQ(Launcher{fixture.options}.Run(), 0);
  const std::string graph = fixture.graph_json();
  EXPECT_EQ(GraphErrors(graph), std::vector<std::string>{}) << graph;
  const auto warnings = GraphFindings(graph, "warning", "studio_bridge");
  ASSERT_EQ(warnings.size(), 1u) << graph;
  EXPECT_NE(warnings[0].find("bazel build //studio/bridge:studio_bridge"),
            std::string::npos)
      << warnings[0];
  ASSERT_TRUE(fixture.has_manifest());
  const std::string manifest = ReadFile(fixture.out_dir + "/manifest.json");
  EXPECT_NE(manifest.find("\"name\": \"alpha\""), std::string::npos)
      << manifest;
  EXPECT_EQ(manifest.find("\"name\": \"studio_bridge\""), std::string::npos)
      << manifest;

  fixture.options.describe_only = true;
  fixture.options.binary_overrides["//example/alpha:node"] =
      TempPath("never_built_alpha");
  EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
  const std::string refused = fixture.graph_json();
  EXPECT_EQ(GraphFindings(refused, "error", "alpha").size(), 1u) << refused;
  EXPECT_EQ(GraphFindings(refused, "error", "studio_bridge").size(), 0u)
      << refused;
}

// The roster is probed in parallel: each of three probes answers only once
// all three have started, so a launcher that probed one at a time would see
// its first probe miss the describe deadline.
TEST(LauncherGraphTest, ProbesRunInParallel) {
  const std::string started = TempPath("parallel_started");
  std::filesystem::remove_all(started);
  std::filesystem::create_directories(started);
  const auto together = [&](std::string_view node, std::string_view target) {
    const std::string stub = DescribingStub(node, target, {});
    const std::string shebang = "#!/bin/sh\n";
    return shebang + ": > '" + started + "/" + std::string{node} +
           "'\nwhile [ \"$(ls '" + started +
           "' | wc -l)\" -lt 3 ]; do sleep 0.01; done\n" +
           stub.substr(shebang.size());
  };
  GraphFixture fixture{"parallel", together("alpha", "//example/alpha:node"),
                       together("beta", "//example/beta:node")};
  fixture.options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStubNode("parallel_hardware",
                    together("hardware_node", "//talOS/bridge:hardware_node"));
  const auto specs = DiscoverNodes(
      config::ParseRobotConfig(fixture.options.config_path), fixture.options);
  ASSERT_EQ(specs.size(), 3u);
  std::vector<NodeDescription> described;
  DescribeGraph(specs, fixture.options, described);
  for (const auto& node : described)
    EXPECT_TRUE(node.answered) << node.roster_name << ": " << node.error;
  std::filesystem::remove_all(started);
}

// REQ-SAFE-28: a non-critical node that sends a target lane is a graph error.
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
  // Only alpha must miss the deadline. A freshly written stub's first exec
  // alone can take about 100 ms (macOS, Bazel's darwin sandbox), so the
  // answering peers need a deadline well above that.
  fixture.options.describe_timeout_ms = 1000;
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

// REQ-OPS-01: a graph error makes --describe-only exit non-zero, no manifest
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

// REQ-CONTRACT-09: alpha reads its teleop and auto lanes and only teleop has
// a writer. Coverage is required by `require_target_coverage = true` and by
// `commissioned = true` (the config file's, whatever --sim does in memory):
// either refuses the graph with one error naming alpha, the mode and its
// lanes. An uncommissioned config without coverage required keeps the
// unwritten lane a warning.
TEST(LauncherGraphTest, DescribeOnlyEnforcesTargetCoverageWhenRequired) {
  struct Row {
    const char* name;
    const char* robot_keys;
    bool refused;
  };
  for (const Row row : {
           Row{"optional_mode_coverage", "require_target_coverage = false\n",
               false},
           Row{"required_mode_coverage", "require_target_coverage = true\n",
               true},
           Row{"commissioned_mode_coverage", "commissioned = true\n", true},
       }) {
    SCOPED_TRACE(row.name);
    auto teleop_lane = Watcher("/alpha/target/teleop");
    auto auto_lane = Watcher("/alpha/target/auto");
    teleop_lane.flags = introspect::naming::kSourceFlagOptional;
    auto_lane.flags = introspect::naming::kSourceFlagOptional;
    GraphFixture fixture{row.name,
                         DescribingStub("alpha", "//example/alpha:node",
                                        {teleop_lane, auto_lane}),
                         DescribingStub("beta", "//example/beta:node",
                                        {Sender("/alpha/target/teleop")})};
    std::string toml{kFixtureToml};
    const std::string header = "[robot]\n";
    toml.insert(toml.find(header) + header.size(), row.robot_keys);
    {
      std::ofstream config{fixture.options.config_path};
      config << toml;
    }
    fixture.options.describe_only = true;
    Launcher launcher{fixture.options};
    EXPECT_EQ(launcher.Run(), row.refused ? 1 : 0);
    EXPECT_FALSE(fixture.has_manifest());
    const auto graph = fixture.graph_json();
    const auto uncovered = GraphFindings(graph, "error", "alpha");
    if (row.refused) {
      ASSERT_EQ(uncovered.size(), 1u) << graph;
      EXPECT_NE(uncovered[0].find("autonomous"), std::string::npos)
          << uncovered[0];
      EXPECT_NE(uncovered[0].find("/alpha/target/auto"), std::string::npos)
          << uncovered[0];
    } else {
      EXPECT_EQ(GraphErrors(graph), std::vector<std::string>{}) << graph;
      EXPECT_EQ(GraphFindings(graph, "warning", "/alpha/target/auto").size(),
                1u)
          << graph;
    }
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

// A binary that cannot be run is reported with the reason its exec failed,
// not as a node that exited 127.
TEST(LauncherGraphTest, ProbeReportsWhyABinaryCannotRun) {
  introspect::Description out;
  std::string error;
  EXPECT_FALSE(ProbeDescribe({"/nonexistent/talos_node", "--describe"},
                             std::chrono::milliseconds{500}, out, error));
  EXPECT_NE(error.find("/nonexistent/talos_node"), std::string::npos) << error;
  EXPECT_NE(error.find(std::strerror(ENOENT)), std::string::npos) << error;
}

// A spawned child whose setpgid(0, 0) is refused but which already leads its
// own group (as it would if the launcher's setpgid(pid, pid) had moved it, an
// unconfirmed cause) has the group the launcher signals, so it must run, not
// exit 127 as "cannot spawn" (TODO.md INC-5); the report names its ids. A
// session leader is the one caller the kernel refuses (EPERM) while it leads
// its own group.
TEST(LauncherTest, ARefusedSetpgidOfAGroupLeaderStillEntersItsGroup) {
  int results[2];
  ASSERT_EQ(::pipe(results), 0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(results[0]);
    ChildReport report;
    report.fatal = ::setsid() < 0 || !EnterOwnProcessGroup(report);
    [[maybe_unused]] const ssize_t written =
        ::write(results[1], &report, sizeof report);
    ::_exit(0);
  }
  ::close(results[1]);
  ChildReport report;
  const ssize_t got = ::read(results[0], &report, sizeof report);
  ::close(results[0]);
  ::waitpid(child, nullptr, 0);
  ASSERT_EQ(got, static_cast<ssize_t>(sizeof report));
  EXPECT_FALSE(report.fatal);
  EXPECT_EQ(report.code, EPERM);
  EXPECT_EQ(report.attempts, 1);
  EXPECT_FALSE(report.new_session);
  EXPECT_EQ(report.pid, child);
  EXPECT_EQ(report.pgrp, child);
  EXPECT_EQ(report.sid, child);
}

// The launcher reads a child's error pipe past a recovered setpgid refusal: to
// EOF, where the spawn succeeds and the note names the child's ids and the
// launcher's own setpgid result (the evidence INC-5 waits for), or to a later
// fatal report, which is the step the spawn failed at.
TEST(LauncherTest, ChildReportsReadPastARecoveredRefusal) {
  ChildReport refusal;
  refusal.step = ChildStep::kProcessGroup;
  refusal.code = EPERM;
  refusal.attempts = 3;
  refusal.new_session = true;
  refusal.pid = 812;
  refusal.pgrp = 640;
  refusal.sid = 101;
  refusal.ppid = 640;
  ChildReport exec_failure;
  exec_failure.code = ENOENT;
  exec_failure.fatal = true;
  for (const bool then_fails : {false, true}) {
    int pipe_ends[2];
    ASSERT_EQ(::pipe(pipe_ends), 0);
    ASSERT_EQ(::write(pipe_ends[1], &refusal, sizeof refusal),
              static_cast<ssize_t>(sizeof refusal));
    if (then_fails) {
      ASSERT_EQ(::write(pipe_ends[1], &exec_failure, sizeof exec_failure),
                static_cast<ssize_t>(sizeof exec_failure));
    }
    ::close(pipe_ends[1]);
    std::ostringstream notes;
    const std::optional<ChildReport> fatal =
        ReadChildReports(pipe_ends[0], "/bin/alpha", 812, EACCES, notes);
    ::close(pipe_ends[0]);
    EXPECT_NE(notes.str().find("pgrp 640, sid 101"), std::string::npos)
        << notes.str();
    EXPECT_NE(notes.str().find(std::strerror(EACCES)), std::string::npos)
        << notes.str();
    ASSERT_EQ(fatal.has_value(), then_fails);
    if (then_fails) {
      EXPECT_EQ(fatal->step, ChildStep::kExec);
      EXPECT_EQ(fatal->code, ENOENT);
    }
  }
}

// A probe is asked once per launch, so a fork refused for want of a process
// slot (EAGAIN) must be retried, not reported as a binary that did not answer:
// that refuses a critical node's launch (TODO.md INC-5).
TEST(LauncherGraphTest, TransientProbeSpawnFailureIsRetried) {
  NodeSpec spec;
  spec.name = "alpha";
  spec.target = "//example/alpha:node";
  spec.binary_path = WriteStubNode(
      "transient_alpha", DescribingStub("alpha", "//example/alpha:node",
                                        {Sender("/alpha/state")}));

  LauncherOptions options;
  options.describe_timeout_ms = 4000;
  options.probe_spawn_failures["alpha"] = 2;
  std::vector<NodeDescription> described;
  const auto diagnostics = DescribeGraph({spec}, options, described);

  ASSERT_EQ(described.size(), 1u);
  EXPECT_TRUE(described[0].answered) << described[0].error;
  for (const auto& diagnostic : diagnostics)
    EXPECT_NE(diagnostic.subject, "alpha") << diagnostic.message;

  std::remove(spec.binary_path.c_str());
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
  const bool unread =
      std::any_of(diagnostics.begin(), diagnostics.end(),
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

// A graph file the launch cannot write -- the declared graph its graph
// readers are described and started with, or graph.json -- refuses the launch
// before anything starts.
TEST(LauncherGraphTest, AGraphFileThatCannotBeWrittenRefusesTheLaunch) {
  const std::string record = TempPath("unwritable_graph_record");
  std::filesystem::create_directories(record);
  for (const auto& [tag, file] :
       {std::pair{"unwritable_declared_graph", "declared_graph.json"},
        std::pair{"unwritable_graph", "graph.json"}}) {
    GraphFixture fixture{
        tag,
        DescribingStub("alpha", "//example/alpha:node",
                       {Sender("/alpha/state")}),
        GraphReaderStub("beta", "//example/beta:node", record)};
    // A directory where the file belongs: no write replaces it, even as root.
    std::filesystem::create_directories(fixture.out_dir + "/" + file);
    EXPECT_EQ(Launcher{fixture.options}.Run(), 1) << file;
    EXPECT_FALSE(fixture.has_manifest()) << file;
  }
  std::filesystem::remove_all(record);
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

  testing::WriteWarmStub(bundle_dir + "/bin/alpha", "#!/bin/sh\nexit 0\n");

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

// REQ-OPS-30: a deployed bundle that carries Studio starts its own bridge and
// serves its own webroot, through the [studio] keys a checkout uses; one that
// carries none runs its session without a bridge rather than failing the
// graph on a probe of a binary it lacks.
TEST(LauncherBundleTest, BundledStudioResolvesThroughTheStudioKeys) {
  const std::string bundle_dir = TempPath("test_bundle_studio");
  std::filesystem::create_directories(bundle_dir + "/config");
  std::filesystem::create_directories(bundle_dir + "/studio/webroot");
  { std::ofstream{bundle_dir + "/config/robot.toml"} << "[robot]\n"; }
  const auto write_manifest = [&](bool studio) {
    std::ofstream mf(bundle_dir + "/bundle_manifest.toml");
    mf << "bundle_version = 1\nconfig_path = \"config/robot.toml\"\n";
    if (studio) {
      mf << "\n[studio]\nbridge = \"studio/studio_bridge\"\n"
         << "targets = [\"//viewer:bridge\"]\n"
         << "webroot = \"studio/webroot\"\n";
    }
  };
  testing::WriteWarmStub(bundle_dir + "/studio/studio_bridge",
                         "#!/bin/sh\nexit 0\n");
  const std::string bridge = bundle_dir + "/studio/studio_bridge";

  write_manifest(true);
  LauncherOptions options;
  options.bundle_dir = bundle_dir;
  options.start_studio_bridge = true;
  std::string error;
  ASSERT_TRUE(LoadBundle(options, error)) << error;

  config::RobotConfig robot;  // webroot: the checkout's relative default
  robot.studio.bridge = "//viewer:bridge";
  EXPECT_EQ(MakeNodeSpec(kStudioBridgeName, robot.studio.bridge, options,
                         /*studio_bridge=*/true)
                .binary_path,
            bridge);
  // The framework's default label, unset in the config: found by name.
  EXPECT_EQ(
      MakeNodeSpec(kStudioBridgeName, "//studio/bridge:studio_bridge", options,
                   /*studio_bridge=*/true)
          .binary_path,
      bridge);
  EXPECT_EQ(std::filesystem::path{EffectiveStudioWebroot(robot, options)},
            std::filesystem::absolute(bundle_dir + "/studio/webroot")
                .lexically_normal());
  robot.studio.webroot = "/srv/studio";
  EXPECT_EQ(EffectiveStudioWebroot(robot, options), "/srv/studio");

  write_manifest(false);
  LauncherOptions bare;
  bare.bundle_dir = bundle_dir;
  bare.start_studio_bridge = true;
  ASSERT_TRUE(LoadBundle(bare, error)) << error;
  const auto nodes =
      DiscoverNodes(config::ParseRobotConfigString(kFixtureToml), bare);
  EXPECT_EQ(std::count_if(
                nodes.begin(), nodes.end(),
                [](const NodeSpec& n) { return n.name == kStudioBridgeName; }),
            0);

  std::filesystem::remove_all(bundle_dir);
}

// A binary's digest is the SHA-256 sha256sum and the bundle builder's
// hashlib write, across the padding boundary and FileSha256's read chunks.
TEST(LauncherBundleTest, FileSha256MatchesTheStandardDigest) {
  const auto digest = [](std::string_view bytes) {
    const std::string path = TempPath("sha256_input");
    std::ofstream{path, std::ios::binary} << bytes;
    std::string hex = FileSha256(path);
    std::filesystem::remove(path);
    return hex;
  };
  EXPECT_EQ(digest(""),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(digest("abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(digest("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  EXPECT_EQ(digest(std::string(1000000, 'a')),
            "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
  EXPECT_EQ(FileSha256(TempPath("sha256_missing")), "");
}

// A member's run history is bounded: it keeps the first run, which says how
// the trouble started, and the newest, and counts what it dropped between.
TEST(LauncherTest, RunHistoryKeepsFirstAndNewestRunsAndCountsTheRest) {
  NodeProcessInfo proc;
  const int total = static_cast<int>(kMaxRecordedRuns) + 8;
  for (int pid = 1; pid <= total; ++pid) {
    proc.BeginRun(pid);
    proc.exit_signal = SIGKILL;
    proc.EndRun(kRunStalled);
  }
  ASSERT_EQ(proc.runs.size(), kMaxRecordedRuns);
  EXPECT_EQ(proc.runs_dropped, 8u);
  EXPECT_EQ(proc.runs.front().pid, 1);
  EXPECT_EQ(proc.runs[1].pid, 10);
  EXPECT_EQ(proc.runs.back().pid, total);
  EXPECT_EQ(proc.runs.back().reason, kRunStalled);
  EXPECT_EQ(proc.runs.back().exit_signal, SIGKILL);
}

// A launcher kill names its run whatever the status; a stop names only the
// endings a stop asks for, so a crash while stopping stays a crash.
TEST(LauncherTest, RunEndReasonKeepsACrashDuringAStop) {
  const auto exited = [](int code) { return code << 8; };  // WIFEXITED
  EXPECT_EQ(RunEndReason({}, false, exited(0)), kRunExit);
  EXPECT_EQ(RunEndReason({}, false, SIGSEGV), kRunCrash);
  EXPECT_EQ(RunEndReason(kRunStalled, false, SIGKILL), kRunStalled);
  EXPECT_EQ(RunEndReason(kRunEndpointMismatch, true, exited(0)),
            kRunEndpointMismatch);
  EXPECT_EQ(RunEndReason(kRunStop, true, exited(0)), kRunStop);
  EXPECT_EQ(RunEndReason({}, true, SIGTERM), kRunStop);
  EXPECT_EQ(RunEndReason(kRunStop, true, SIGABRT), kRunCrash);
  EXPECT_EQ(RunEndReason(kRunStop, true, exited(1)), kRunCrash);
  EXPECT_EQ(RunEndReason(kRunStopTimeout, true, SIGKILL), kRunStopTimeout);
  EXPECT_EQ(RunEndReason(kRunStopTimeout, true, SIGINT), kRunStop);
}

// A bundle's roster, Studio's bridge included, runs from its own directory.
// REQ-OPS-30: a bundled bridge that cannot run on this machine (here it
// fails its --describe, as a missing library or a crash would) is one
// warning, and the robot's session runs without Studio.
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
  testing::WriteWarmStub(
      alpha_bin,
      DescribingStub("alpha", "//bundle/alpha:node", {Sender("/alpha/state")}));

  const std::string hw_bin = bundle_dir + "/bin/hardware_node";
  testing::WriteWarmStub(
      hw_bin, DescribingStub("hardware_node", "//talOS/bridge:hardware_node",
                             {Watcher("/alpha/state")}));

  {
    std::ofstream mf(bundle_dir + "/bundle_manifest.toml");
    mf << "bundle_version = 1\n"
       << "config_path = \"config/robot.toml\"\n"
       << "\n[[nodes]]\nname = \"hardware_node\"\n"
       << "targets = [\"//talOS/bridge:hardware_node\"]\n"
       << "binary = \"bin/hardware_node\"\nlanguage = \"cpp\"\n"
       << "\n[[nodes]]\nname = \"alpha\"\n"
       << "targets = [\"//bundle/alpha:node\"]\n"
       << "binary = \"bin/alpha\"\nlanguage = \"cpp\"\n"
       << "\n[studio]\nbridge = \"studio/studio_bridge\"\n"
       << "targets = [\"//studio/bridge:studio_bridge\"]\n"
       << "webroot = \"studio/webroot\"\n";
  }
  std::filesystem::create_directories(bundle_dir + "/studio/webroot");
  testing::WriteWarmStub(
      bundle_dir + "/studio/studio_bridge",
      "#!/bin/sh\necho 'cannot load libstudio' >&2\nexit 127\n");

  LauncherOptions options;
  options.bundle_dir = bundle_dir;
  options.describe_only = true;
  options.start_studio_bridge = true;
  options.output_dir = bundle_dir + "/logs";
  options.session_id = 1;

  Launcher launcher{options};
  EXPECT_EQ(launcher.Run(), 0);
  const std::string graph = ReadFile(bundle_dir + "/logs/1/graph.json");
  EXPECT_EQ(GraphErrors(graph), std::vector<std::string>{}) << graph;
  EXPECT_EQ(GraphFindings(graph, "warning", "studio_bridge").size(), 1u)
      << graph;

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
  // Whatever the test machine's disk holds, retention sees room to spare.
  options.available_bytes = [](const std::filesystem::path&) {
    return std::optional{std::numeric_limits<uint64_t>::max()};
  };
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

// Retention passes over an old session it cannot read or cannot remove and
// prunes the rest; neither stops the launch.
TEST(LauncherSessions, RetentionPassesOverLockedSessions) {
  if (::geteuid() == 0) GTEST_SKIP() << "root reads and removes any directory";
  auto options = SessionFixture("locked_sessions");
  std::filesystem::remove_all(options.output_dir);
  std::string configuration{kFixtureToml};
  configuration.insert(configuration.find("period_us"),
                       "log_keep_sessions = 1\n");
  std::ofstream{options.config_path} << configuration;
  // Oldest first: unreadable, unremovable, then removable.
  const auto now = std::filesystem::file_time_type::clock::now();
  for (const int id : {1, 2, 3}) {
    const std::string dir = options.output_dir + "/" + std::to_string(id);
    std::filesystem::create_directories(dir);
    std::ofstream{dir + "/manifest.json"} << "{}\n";
    std::filesystem::last_write_time(dir, now - std::chrono::hours{4 - id});
  }
  const std::string unreadable = options.output_dir + "/1";
  const std::string unremovable = options.output_dir + "/2";
  ASSERT_EQ(::chmod(unreadable.c_str(), 0), 0);
  ASSERT_EQ(::chmod(unremovable.c_str(), 0555), 0);
  options.session_id = 4;
  const int result = Launcher{options}.Run();
  ::chmod(unreadable.c_str(), 0755);
  ::chmod(unremovable.c_str(), 0755);
  EXPECT_EQ(result, 0);
  EXPECT_FALSE(std::filesystem::exists(options.output_dir + "/3"));
  EXPECT_TRUE(std::filesystem::exists(options.output_dir + "/4/manifest.json"));
  std::filesystem::remove_all(options.output_dir);
}

// A directory named by a session id holding that session's event log, as
// --replay-inputs leaves, is a session: counted, and pruned oldest first. One
// holding nothing, or only what a launch writes before it starts its nodes
// (graph.json too: --describe-only, a refused graph), is an abandoned launch:
// never counted, and removed once another session directory is newer, so the
// newest stays. Another session's log or any other file leaves a directory
// alone, and so does a retention record, which a launch that died before its
// manifest leaves only when it removed something. Only session directories
// decide the newest. Only a launch that holds the session lock removes
// anything: a replay (or --describe-only) runs beside a live session that may
// look like any of these.
TEST(LauncherSessions, RetentionPrunesReplayOutputAndAbandonedLaunchesOnly) {
  auto options = SessionFixture("leftovers");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  std::string configuration{kFixtureToml};
  configuration.insert(configuration.find("period_us"),
                       "log_keep_sessions = 2\n");
  std::ofstream{options.config_path} << configuration;
  const auto dir = [&root](int id) { return root + "/" + std::to_string(id); };
  for (const int id : {5, 6, 7, 8, 9, 10, 11, 12, 13, 2026}) {
    std::filesystem::create_directories(dir(id));
  }
  std::ofstream{dir(5) + "/" + std::string{kRetentionRecordFile}}
      << "retention removes session\n";
  WriteTestLog(dir(6) + "/consumer.tlog", "consumer", 6, {1'000'000});
  std::ofstream{dir(7) + "/manifest.json"} << "{}\n";
  std::ofstream{dir(9) + "/declared_graph.json"} << "[]\n";
  // A run stopped before writing its log's header.
  std::ofstream{dir(9) + "/consumer.tlog"}.close();
  std::ofstream{dir(10) + "/manifest.json"} << "{}\n";
  WriteTestLog(dir(12) + "/other.tlog", "other", 99, {1'000'000});
  std::ofstream{dir(13) + "/graph.json"} << "{}\n";
  std::ofstream{dir(13) + "/declared_graph.json"} << "[]\n";
  std::ofstream{dir(2026) + "/photo.jpg"} << "not a session";
  // Oldest first, after the files: each write moves its directory's time.
  const auto now = std::filesystem::file_time_type::clock::now();
  for (const auto& [id, age] :
       {std::pair{5, 10}, std::pair{12, 9}, std::pair{6, 8}, std::pair{7, 7},
        std::pair{8, 6}, std::pair{9, 5}, std::pair{10, 4}, std::pair{13, 3},
        std::pair{11, 2}, std::pair{2026, 1}}) {
    std::filesystem::last_write_time(dir(id), now - std::chrono::hours{age});
  }
  // A replay of the older recorded session, into the same root.
  options.session_id = 14;
  options.replay_inputs = dir(7);
  options.only = "alpha";
  ASSERT_EQ(Launcher{options}.Run(), 0);
  for (const int id : {6, 7, 8, 9, 10, 11, 12, 13}) {
    EXPECT_TRUE(std::filesystem::exists(dir(id))) << id;
  }
  options.session_id = 15;
  options.replay_inputs.clear();
  options.only.clear();
  ASSERT_EQ(Launcher{options}.Run(), 0);
  // 15 and 10 fill log_keep_sessions = 2, so the older 6 and 7 go; the
  // abandoned 8, 9, 11 and 13 count for nothing and are older than the
  // replay's directory 14.
  for (const int id : {6, 7, 8, 9, 11, 13}) {
    EXPECT_FALSE(std::filesystem::exists(dir(id))) << id;
  }
  for (const int id : {5, 10, 12, 14, 15}) {
    EXPECT_TRUE(std::filesystem::exists(dir(id))) << id;
  }
  EXPECT_EQ(ReadFile(dir(2026) + "/photo.jpg"), "not a session");
  std::filesystem::remove_all(root);
}

// The manifest a launch at `start_s` on the wall clock leaves in `dir`.
void WriteSessionManifest(const std::string& dir, uint64_t id, int64_t start_s,
                          int64_t end_s = 0) {
  std::filesystem::create_directories(dir);
  SessionManifest manifest;
  manifest.session_id = id;
  manifest.output_dir = dir;
  manifest.start_wall_ns = start_s * 1'000'000'000;
  manifest.end_wall_ns = end_s * 1'000'000'000;
  std::ofstream{dir + "/manifest.json"} << manifest.ToJson();
}

// `node`'s event log in `dir`: one received Driver Station packet per entry
// of `fms`, FMS attached or not, and `pad_bytes` of other recording.
void WriteDriverStationLog(const std::string& dir, const std::string& node,
                           uint64_t id, const std::vector<bool>& fms,
                           std::size_t pad_bytes = 0) {
  using D = talos::Descriptor<hardware::Packet>;
  event::Registration source;
  source.id = 1;
  source.kind = event::SourceKind::WATCHER;
  source.name = hardware::kDriverStationTopic;
  source.message_bytes = sizeof(hardware::Packet);
  source.schema = {std::string{D::kName}, D::kHash, {}};
  event::Registration timer;
  timer.id = 2;
  timer.kind = event::SourceKind::TIMER;
  timer.name = "pad";
  timer.message_bytes = 64;
  timer.period_ns = 1'000'000;
  event::log::LogWriter writer{
      dir + "/" + node + ".tlog", node,
      event::log::LogWriterOptions{8192, 1, /*background=*/false}, id};
  writer.start({source, timer},
               event::MonotonicTime::from_nanos(1'000'000'000));
  uint64_t index = 0;
  for (const bool attached : fms) {
    driver_station::DriverStationData ds;
    ds.sample_time_us = index + 1;
    ds.mode_since_us = 1;
    ds.fms.flags = driver_station::kDsAttached | driver_station::kEnabled |
                   driver_station::kTeleop |
                   (attached ? driver_station::kFmsAttached : 0u);
    hardware::Packet packet{};
    packet.size =
        static_cast<uint32_t>(driver_station::Encode(ds, packet.data));
    event::Context ctx{};
    ctx.kind = event::EventKind::MESSAGE;
    ctx.source_id = 1;
    ctx.dispatch_index = ++index;
    ctx.event_time =
        event::MonotonicTime::from_nanos(1'000'000'000 + int64_t(index));
    ctx.now = ctx.event_time;
    writer.dispatch(ctx, std::as_bytes(std::span{&packet, 1}));
  }
  // Timer records stand in for the rest of the recording.
  const std::vector<uint8_t> pad(64, 0xA5);
  for (std::size_t bytes = 0; bytes < pad_bytes; bytes += 128) {
    event::Context ctx{};
    ctx.kind = event::EventKind::TIMER;
    ctx.source_id = 2;
    ctx.dispatch_index = ++index;
    ctx.event_time =
        event::MonotonicTime::from_nanos(1'000'000'000 + int64_t(index));
    ctx.now = ctx.event_time;
    writer.dispatch(ctx, std::as_bytes(std::span{pad}));
  }
  writer.flush();
  ASSERT_FALSE(writer.failed()) << writer.error();
}

// A session whose FMS-attached Driver Station packet sits in a log of a
// format version this build rejects.
void WriteOldFormatSession(const std::string& dir, uint64_t id,
                           int64_t start_s) {
  WriteSessionManifest(dir, id, start_s);
  WriteDriverStationLog(dir, "late", id, {true});
  std::fstream log{dir + "/late.tlog",
                   std::ios::binary | std::ios::in | std::ios::out};
  const uint32_t old_version = event::log::FORMAT_VERSION - 1;
  log.seekp(offsetof(event::log::FileHeader, version));
  log.write(reinterpret_cast<const char*>(&old_version), sizeof old_version);
}

// OBS-1: sessions that ran no match are removed before older ones that did,
// or that this build cannot classify, oldest first by the start time their
// manifests record, whatever the directories' modification times say (a
// tool reading or touching one moves them).
TEST(LauncherSessions, RetentionRemovesPitSessionsBeforeAnOlderMatch) {
  auto options = SessionFixture("match_first");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  std::string configuration{kFixtureToml};
  configuration.insert(configuration.find("period_us"),
                       "log_keep_sessions = 4\n");
  std::ofstream{options.config_path} << configuration;
  const auto dir = [&root](int id) { return root + "/" + std::to_string(id); };
  // 1, a match: the node that received the fewest bytes died before the
  // field link and saw only an unattached Driver Station.
  WriteSessionManifest(dir(1), 1, 1001);
  WriteDriverStationLog(dir(1), "early", 1, {false});
  WriteDriverStationLog(dir(1), "late", 1, {false, true}, 4096);
  // 2, unclassifiable: its log is of a format version this build rejects.
  WriteOldFormatSession(dir(2), 2, 1002);
  for (const int id : {3, 4, 5}) {
    WriteSessionManifest(dir(id), id, 1000 + id);
    WriteDriverStationLog(dir(id), "late", id, {false});
  }
  // Modification times disagree with the start times: 3 was touched last.
  const auto now = std::filesystem::file_time_type::clock::now();
  for (const auto& [id, age] :
       {std::pair{5, 5}, std::pair{1, 4}, std::pair{2, 3}, std::pair{4, 2},
        std::pair{3, 1}}) {
    std::filesystem::last_write_time(dir(id), now - std::chrono::hours{age});
  }
  options.session_id = 6;
  ASSERT_EQ(Launcher{options}.Run(), 0);
  // log_keep_sessions = 4 keeps the new 6 and three others: the match 1,
  // the unclassifiable 2 and the newest pit session 5.
  for (const int id : {3, 4}) {
    EXPECT_FALSE(std::filesystem::exists(dir(id))) << id;
  }
  for (const int id : {1, 2, 5, 6}) {
    EXPECT_TRUE(std::filesystem::exists(dir(id) + "/manifest.json")) << id;
  }
  std::filesystem::remove_all(root);
}

// Only the newest kProtectedMatches unclassifiable sessions are protected:
// after a log format change makes every older session unclassifiable, the
// rest go oldest first with the pit sessions, not the newest pit session.
TEST(LauncherSessions, RetentionProtectsOnlyTheNewestUnclassifiableSessions) {
  auto options = SessionFixture("unknown_floor");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  const int unknown = kProtectedMatches + 1;
  std::string configuration{kFixtureToml};
  configuration.insert(
      configuration.find("period_us"),
      "log_keep_sessions = " + std::to_string(unknown + 1) + "\n");
  std::ofstream{options.config_path} << configuration;
  const auto dir = [&root](int id) { return root + "/" + std::to_string(id); };
  for (int id = 1; id <= unknown; ++id) {
    WriteOldFormatSession(dir(id), id, 1000 + id);
  }
  const int pit = unknown + 1;
  WriteSessionManifest(dir(pit), pit, 1000 + pit);
  WriteDriverStationLog(dir(pit), "late", pit, {false});
  options.session_id = pit + 1;
  ASSERT_EQ(Launcher{options}.Run(), 0);
  EXPECT_FALSE(std::filesystem::exists(dir(1)));
  for (const int id : {2, unknown, pit, pit + 1}) {
    EXPECT_TRUE(std::filesystem::exists(dir(id) + "/manifest.json")) << id;
  }
  std::filesystem::remove_all(root);
}

// Bytes in the regular files under `dir`.
uintmax_t DirectoryBytes(const std::string& dir) {
  uintmax_t bytes = 0;
  for (const auto& file : std::filesystem::recursive_directory_iterator{dir}) {
    if (file.is_regular_file()) bytes += file.file_size();
  }
  return bytes;
}

// The newest matches outlast log_keep_sessions and the byte budget, which
// counts only the sessions retention may remove for it: a pit reboot after a
// match larger than the half older sessions get keeps that match, and keeps
// pit sessions within that half beside it.
TEST(LauncherSessions, RetentionKeepsTheNewestMatchesBeyondTheBudgets) {
  auto options = SessionFixture("match_floor");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  const auto configure = [&options](std::string_view keys) {
    std::string configuration{kFixtureToml};
    configuration.insert(configuration.find("period_us"), keys);
    std::ofstream{options.config_path} << configuration;
  };
  const auto dir = [&root](int id) { return root + "/" + std::to_string(id); };
  for (const int id : {1, 2}) {
    WriteSessionManifest(dir(id), id, 1000 + id);
    WriteDriverStationLog(dir(id), "late", id, {true}, 1'000'000);
  }
  WriteSessionManifest(dir(3), 3, 1003);
  WriteDriverStationLog(dir(3), "late", 3, {false});

  configure("log_keep_sessions = 1\n");
  options.session_id = 4;
  ASSERT_EQ(Launcher{options}.Run(), 0);
  EXPECT_FALSE(std::filesystem::exists(dir(3)));
  EXPECT_TRUE(std::filesystem::exists(dir(1)));
  EXPECT_TRUE(std::filesystem::exists(dir(2)));

  // Older sessions get 1.5 MB: the matches alone hold more, and two pit
  // sessions (5, 6) push the unprotected past it.
  for (const int id : {5, 6}) {
    WriteSessionManifest(dir(id), id, 1000 + id);
    WriteDriverStationLog(dir(id), "late", id, {false}, 1'000'000);
  }
  ASSERT_GT(DirectoryBytes(dir(1)) + DirectoryBytes(dir(2)), 1'500'000U);
  configure("log_max_bytes = 3000000\n");
  options.session_id = 7;
  ASSERT_EQ(Launcher{options}.Run(), 0);
  EXPECT_FALSE(std::filesystem::exists(dir(5)));
  for (const int id : {1, 2, 4, 6, 7}) {
    EXPECT_TRUE(std::filesystem::exists(dir(id) + "/manifest.json")) << id;
  }
  std::filesystem::remove_all(root);
}

// When the log root's filesystem lacks room for the new session (log_max_bytes
// and 64 MiB of output log per roster member and for talos.log), sessions go in
// retention's order, a pit session before older matches and the protected
// matches oldest first, only until removing them frees enough; the launch
// records each removal in its own session directory.
TEST(LauncherSessions, RetentionRemovesProtectedMatchesLastForFreeSpace) {
  auto options = SessionFixture("free_floor");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  std::string configuration{kFixtureToml};
  // Older sessions get 5 MB, more than they hold: only free space removes.
  configuration.insert(configuration.find("period_us"),
                       "log_max_bytes = 10000000\n");
  std::ofstream{options.config_path} << configuration;
  const auto dir = [&root](int id) { return root + "/" + std::to_string(id); };
  for (const int id : {1, 2, 4}) {
    WriteSessionManifest(dir(id), id, 1000 + id);
    WriteDriverStationLog(dir(id), "late", id, {true}, 1'000'000);
  }
  // The pit session 3 is the newest.
  WriteSessionManifest(dir(3), 3, 1005);
  WriteDriverStationLog(dir(3), "late", 3, {false}, 1'000'000);
  // An output log for each of the fixture's roster (hardware_node, alpha,
  // beta) and talos.log.
  const uint64_t floor = 10'000'000 + (3 + 1) * kChildOutputLogMaxBytes;
  // One byte short of the floor once the pit session 3 and the oldest match
  // 1 are gone.
  const uintmax_t needed = DirectoryBytes(dir(3)) + DirectoryBytes(dir(1));
  options.available_bytes = [available = floor - needed +
                                         1](const std::filesystem::path&) {
    return std::optional<uint64_t>{available};
  };
  options.session_id = 6;
  ASSERT_EQ(Launcher{options}.Run(), 0);
  for (const int id : {1, 3}) {
    EXPECT_FALSE(std::filesystem::exists(dir(id))) << id;
  }
  for (const int id : {2, 4}) {
    EXPECT_TRUE(std::filesystem::exists(dir(id) + "/manifest.json")) << id;
  }
  const std::string record =
      ReadFile(dir(6) + "/" + std::string{kRetentionRecordFile});
  for (const int id : {1, 3}) {
    EXPECT_NE(record.find(dir(id) + " ("), std::string::npos) << record;
  }
  std::filesystem::remove_all(root);
}

// On a full log disk, where not even the new session's directory can be
// created, retention runs first and frees the room; the launch then creates
// its session and records the removal there. When nothing it removes makes
// room, the launch is refused instead of throwing.
TEST(LauncherSessions, FullDiskLaunchFreesRoomBeforeCreatingItsSession) {
  auto options = SessionFixture("full_disk");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  std::string configuration{kFixtureToml};
  configuration.insert(configuration.find("period_us"),
                       "log_max_bytes = 10000000\n");
  std::ofstream{options.config_path} << configuration;
  const auto dir = [&root](int id) { return root + "/" + std::to_string(id); };
  WriteSessionManifest(dir(1), 1, 1001);
  WriteDriverStationLog(dir(1), "late", 1, {false}, 1'000'000);
  // An output log for each of the fixture's roster (hardware_node, alpha,
  // beta) and talos.log.
  const uint64_t floor = 10'000'000 + (3 + 1) * kChildOutputLogMaxBytes;
  options.available_bytes = [available =
                                 floor - 1](const std::filesystem::path&) {
    return std::optional<uint64_t>{available};
  };
  // The disk is full until session 1 is gone.
  const std::string old_session = dir(1);
  options.create_session_directory =
      [old_session](const std::filesystem::path& path, std::error_code& error) {
        if (std::filesystem::exists(old_session)) {
          error = std::make_error_code(std::errc::no_space_on_device);
          return;
        }
        std::filesystem::create_directories(path, error);
      };
  options.session_id = 2;
  ASSERT_EQ(Launcher{options}.Run(), 0);
  EXPECT_FALSE(std::filesystem::exists(dir(1)));
  EXPECT_TRUE(std::filesystem::exists(dir(2) + "/manifest.json"));
  EXPECT_NE(ReadFile(dir(2) + "/" + std::string{kRetentionRecordFile})
                .find(dir(1) + " ("),
            std::string::npos);

  options.create_session_directory = [](const std::filesystem::path&,
                                        std::error_code& error) {
    error = std::make_error_code(std::errc::no_space_on_device);
  };
  options.session_id = 3;
  EXPECT_EQ(Launcher{options}.Run(), 1);
  EXPECT_FALSE(std::filesystem::exists(dir(3)));
  std::filesystem::remove_all(root);
}

// A session lock file that is absent and has no room to be created on the log
// root's filesystem (macOS's /tmp after a reboot on a full disk) is held by
// no launcher, so retention runs without it, then the lock is taken. Another
// launch in the same state may take it meanwhile and start recording, so
// that retention removes only sessions whose launcher shut down. Any other
// lock failure refuses the launch before retention, and a lock still not
// taken after it refuses the launch with the record on stderr.
TEST(LauncherSessions, LockFileWithoutRoomPrunesOnlyFinishedSessions) {
  auto options = SessionFixture("lock_no_room");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  std::string configuration{kFixtureToml};
  configuration.insert(configuration.find("period_us"),
                       "log_max_bytes = 10000000\n");
  std::ofstream{options.config_path} << configuration;
  const auto dir = [&root](int id) { return root + "/" + std::to_string(id); };
  const auto write_finished = [&dir] {
    WriteSessionManifest(dir(1), 1, 1001, 1101);
    WriteDriverStationLog(dir(1), "late", 1, {false}, 1'000'000);
  };
  write_finished();
  // Unfinished: a launch that lost power, or one recording now.
  WriteSessionManifest(dir(2), 2, 1002);
  WriteDriverStationLog(dir(2), "late", 2, {false}, 1'000'000);
  const uint64_t floor = 10'000'000 + (3 + 1) * kChildOutputLogMaxBytes;
  // Short of the floor until both are gone.
  const uintmax_t both = DirectoryBytes(dir(1)) + DirectoryBytes(dir(2));
  options.available_bytes = [available = floor - both +
                                         1](const std::filesystem::path&) {
    return std::optional<uint64_t>{available};
  };
  // The first `no_room` calls find no room; later ones take the lock.
  const auto lock_after = [](int no_room_calls) {
    return
        [calls = 0, no_room_calls](std::string& error, bool* no_room) mutable {
          if (calls++ < no_room_calls) {
            error = "cannot open the session lock: No space left on device";
            *no_room = true;
            return std::optional<SessionLock>{};
          }
          return SessionLock::Acquire(error, no_room);
        };
  };

  options.acquire_session_lock = [](std::string& error, bool* no_room) {
    error = "cannot open the session lock: Permission denied";
    *no_room = false;
    return std::optional<SessionLock>{};
  };
  options.session_id = 3;
  EXPECT_EQ(Launcher{options}.Run(), 1);
  EXPECT_TRUE(std::filesystem::exists(dir(1)));
  EXPECT_FALSE(std::filesystem::exists(dir(3)));

  options.acquire_session_lock = lock_after(3);
  ::testing::internal::CaptureStderr();
  const int refused = Launcher{options}.Run();
  const std::string stderr_text = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(refused, 1);
  EXPECT_FALSE(std::filesystem::exists(dir(1)));
  EXPECT_FALSE(std::filesystem::exists(dir(3)));
  EXPECT_NE(stderr_text.find(dir(1) + " ("), std::string::npos) << stderr_text;

  write_finished();
  options.acquire_session_lock = lock_after(2);
  options.session_id = 4;
  ASSERT_EQ(Launcher{options}.Run(), 0);
  EXPECT_FALSE(std::filesystem::exists(dir(1)));
  EXPECT_TRUE(std::filesystem::exists(dir(2) + "/manifest.json"));
  EXPECT_TRUE(std::filesystem::exists(dir(4) + "/manifest.json"));
  std::filesystem::remove_all(root);
}

// Every regular file under `dir`, by path relative to it, with its bytes.
std::map<std::string, std::string> DirectoryContents(const std::string& dir) {
  std::map<std::string, std::string> contents;
  for (const auto& file : std::filesystem::recursive_directory_iterator{dir}) {
    if (!file.is_regular_file()) continue;
    contents[std::filesystem::relative(file.path(), dir).string()] =
        ReadFile(file.path().string());
  }
  return contents;
}

// REQ-OPS-11: a launch in another shared-memory namespace records into the
// same log root (the default /tmp/talos_logs: a simulation beside a bench
// session) and holds its own namespace's session lock, not this one's. Its
// running session's manifest has no end_wall_ns, as a launch that lost power
// leaves, and log_keep_sessions = 2 makes it old enough to remove. A launch
// here, holding this namespace's session lock, must leave every byte of it
// in place and say on stderr that it passed it over; trusting its own session
// lock, it removed the session while it was being written. A session of the
// other namespace whose launcher died is still removed.
TEST(LauncherSessions, RetentionKeepsAnotherNamespacesRunningSession) {
  auto options = SessionFixture("other_namespace");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  std::string configuration{kFixtureToml};
  configuration.insert(configuration.find("period_us"),
                       "log_keep_sessions = 2\n");
  std::ofstream{options.config_path} << configuration;
  const auto dir = [&root](int id) { return root + "/" + std::to_string(id); };
  // Close-on-exec: no node the launch below starts holds them.
  int ready[2];
  int release[2];
  for (int* fds : {ready, release}) {
    ASSERT_EQ(::pipe(fds), 0);
    for (int end = 0; end < 2; ++end) {
      ASSERT_EQ(::fcntl(fds[end], F_SETFD, FD_CLOEXEC), 0);
    }
  }
  // A launch of session `id` in the other namespace, as the launcher starts
  // one: its namespace's session lock, then its session directory, then an
  // unfinished manifest and an event log. A running one reports on `ready`,
  // then runs until `release` closes; the other dies at once, as a killed
  // launcher does.
  const std::string other_namespace =
      "talos_launcher_test_other_" + std::to_string(::getpid());
  const auto launch = [&](int id, bool runs) {
    const pid_t child = ::fork();
    if (child != 0) return child;
    ::close(ready[0]);
    ::close(release[1]);
    ::setenv("TALOS_SHM_NAMESPACE", other_namespace.c_str(), 1);
    std::string error;
    const auto namespace_lock = SessionLock::Acquire(error);
    const std::string lock_path =
        namespace_lock ? namespace_lock->path() : std::string{};
    std::error_code created;
    const auto session = SessionDirectoryLock::Create(
        dir(id),
        [](const std::filesystem::path& path, std::error_code& failed) {
          std::filesystem::create_directories(path, failed);
        },
        created);
    if (!namespace_lock || !session) ::_exit(1);
    WriteSessionManifest(dir(id), id, 1000 + id);
    WriteDriverStationLog(dir(id), "late", id, {false}, 4096);
    // Lock files outside TEST_TMPDIR are not removed with the test.
    ::unlink(lock_path.c_str());
    if (runs) {
      const char byte = 1;
      if (::write(ready[1], &byte, 1) != 1) ::_exit(1);
      char unused = 0;
      while (::read(release[0], &unused, 1) < 0 && errno == EINTR) {
      }
    }
    ::_exit(0);
  };
  const pid_t dead = launch(1, false);
  ASSERT_GT(dead, 0);
  int status = 0;
  ASSERT_EQ(::waitpid(dead, &status, 0), dead);
  ASSERT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0) << status;
  const pid_t running = launch(2, true);
  ASSERT_GT(running, 0);
  ::close(ready[1]);
  ::close(release[0]);
  char byte = 0;
  ASSERT_EQ(::read(ready[0], &byte, 1), 1) << "the running launch failed";
  ::close(ready[0]);
  // This namespace's own session, newer and finished.
  WriteSessionManifest(dir(3), 3, 1003, 1103);
  WriteDriverStationLog(dir(3), "late", 3, {false});
  const auto before = DirectoryContents(dir(2));
  ASSERT_TRUE(before.contains("manifest.json"));

  options.session_id = 4;
  ::testing::internal::CaptureStderr();
  const int result = Launcher{options}.Run();
  const std::string stderr_text = ::testing::internal::GetCapturedStderr();
  const bool survived = std::filesystem::exists(dir(2));
  const auto after = survived ? DirectoryContents(dir(2)) : decltype(before){};
  ::close(release[1]);
  ASSERT_EQ(::waitpid(running, &status, 0), running);
  EXPECT_EQ(result, 0);
  EXPECT_TRUE(survived) << "retention removed the running session " << dir(2);
  std::vector<std::string> changed;
  for (const auto& [file, bytes] : before) {
    const auto now = after.find(file);
    if (now == after.end() || now->second != bytes) changed.push_back(file);
  }
  for (const auto& [file, bytes] : after) {
    if (!before.contains(file)) changed.push_back(file);
  }
  EXPECT_EQ(changed, std::vector<std::string>{}) << dir(2);
  EXPECT_FALSE(std::filesystem::exists(dir(1)));
  EXPECT_TRUE(std::filesystem::exists(dir(3) + "/manifest.json"));
  EXPECT_TRUE(std::filesystem::exists(dir(4) + "/manifest.json"));
  EXPECT_NE(stderr_text.find("passes over running session " + dir(2)),
            std::string::npos)
      << stderr_text;
  EXPECT_EQ(
      ReadFile(dir(4) + "/" + std::string{kRetentionRecordFile}).find(dir(2)),
      std::string::npos)
      << "retention.txt records removals only";
  std::filesystem::remove_all(root);
}

// LOG-9: a launch that locks a session directory after retention listed it
// as idle, and before retention removes it (one given that --session-id, or
// a running one whose refresh locked its removed session.lock again,
// SessionDirectoryLock::Refresh), keeps it whole: retention removes a
// directory only while it holds that lock itself, so it passes it over and
// says so. The launch is injected through the free space read, which
// retention makes after its listing and before the removals the free space
// floor asks for; the injected lock is another open file description in this
// process, which flock treats as another holder.
// REQ-OPS-11
TEST(LauncherSessions, RetentionKeepsASessionLockedAfterItsListing) {
  auto options = SessionFixture("locked_after_listing");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  std::string configuration{kFixtureToml};
  configuration.insert(configuration.find("period_us"),
                       "log_max_bytes = 10000000\n");
  std::ofstream{options.config_path} << configuration;
  const std::string listed = root + "/1";
  WriteSessionManifest(listed, 1, 1001, 1101);
  WriteDriverStationLog(listed, "late", 1, {false}, 4096);
  // What its own launch left.
  std::ofstream{listed + "/" + std::string{kSessionDirectoryLockFile}}.close();
  const auto before = DirectoryContents(listed);
  // An output log for each of the fixture's roster (hardware_node, alpha,
  // beta) and talos.log: one byte short, so retention removes what it can.
  const uint64_t floor = 10'000'000 + (3 + 1) * kChildOutputLogMaxBytes;
  const auto launch = std::make_shared<std::optional<SessionDirectoryLock>>();
  options.available_bytes = [launch, listed](const std::filesystem::path&) {
    if (!*launch) {
      std::error_code error;
      *launch = SessionDirectoryLock::Create(
          listed,
          [](const std::filesystem::path& path, std::error_code& failed) {
            std::filesystem::create_directories(path, failed);
          },
          error);
    }
    return std::optional<uint64_t>{floor - 1};
  };
  options.session_id = 2;
  ::testing::internal::CaptureStderr();
  const int result = Launcher{options}.Run();
  const std::string stderr_text = ::testing::internal::GetCapturedStderr();
  ASSERT_EQ(result, 0) << stderr_text;
  ASSERT_TRUE(launch->has_value())
      << "the injected launch could not lock " << listed;
  auto after = std::filesystem::exists(listed) ? DirectoryContents(listed)
                                               : decltype(before){};
  // The verdict retention's read of it leaves (InspectSession).
  after.erase("session_class");
  EXPECT_TRUE(after == before)
      << "retention removed or changed the session a launch locked";
  EXPECT_NE(stderr_text.find("passes over running session " + listed),
            std::string::npos)
      << stderr_text;
  launch->reset();
  std::filesystem::remove_all(root);
}

// A launch that stops before its manifest (here a refused graph; also a
// missing binary, a failed probe, a service restarting it in a loop) leaves
// an abandoned directory, which a later launch removes once a newer one
// exists, so a crash loop leaves its last attempt's and the one before it,
// not one per attempt. That
// holds while another launch runs in the same log root: retention passing
// that session over must not mark each attempt's directory as a record
// (retention.txt) to keep.
// REQ-OPS-11
TEST(LauncherSessions, ACrashLoopBesideARunningSessionLeavesOneDirectory) {
  auto options = SessionFixture("crash_loop");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  const auto dir = [&root](int id) { return root + "/" + std::to_string(id); };
  WriteSessionManifest(dir(1), 1, 1001);
  std::error_code error;
  const auto running = SessionDirectoryLock::Create(
      dir(1),
      [](const std::filesystem::path& path, std::error_code& failed) {
        std::filesystem::create_directories(path, failed);
      },
      error);
  ASSERT_TRUE(running.has_value()) << error.message();
  options.binary_overrides["//example/alpha:node"] =
      WriteStubNode("crash_loop_silent_alpha", kStubWithoutDescribe);
  const auto start = std::filesystem::file_time_type::clock::now();
  constexpr int kAttempts = 5;
  for (int id = 2; id < 2 + kAttempts; ++id) {
    options.session_id = static_cast<uint64_t>(id);
    ASSERT_EQ(Launcher{options}.Run(), 1) << "attempt " << id;
    // Each attempt's directory is newer than the last's.
    std::filesystem::last_write_time(dir(id), start + std::chrono::minutes{id});
  }
  int left = 0;
  for (int id = 2; id < 2 + kAttempts; ++id) {
    left += std::filesystem::exists(dir(id)) ? 1 : 0;
  }
  EXPECT_LE(left, 2) << "attempts' directories left behind";
  EXPECT_TRUE(std::filesystem::exists(dir(2 + kAttempts - 1)));
  EXPECT_TRUE(std::filesystem::exists(dir(1) + "/manifest.json"));
  std::filesystem::remove_all(root);
}

// A log root on a filesystem without advisory locks (NFS without a lock
// daemon: ENOLCK; macOS msdos and exFAT volumes: ENOTSUP) cannot hold a
// session directory's lock. The robot still starts: one warning, and the
// session runs without it, protected as before launches locked their
// directories (by the namespace's session lock only); retention removes
// as it did then.
// REQ-OPS-11
TEST(LauncherSessions, ALogRootWithoutLocksStillLaunches) {
  auto options = SessionFixture("no_locks");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  std::string configuration{kFixtureToml};
  configuration.insert(configuration.find("period_us"),
                       "log_keep_sessions = 1\n");
  std::ofstream{options.config_path} << configuration;
  WriteSessionManifest(root + "/1", 1, 1001, 1101);
  options.lock_session_directory = [](int, int) {
    errno = ENOLCK;
    return -1;
  };
  options.session_id = 2;
  ::testing::internal::CaptureStderr();
  const int result = Launcher{options}.Run();
  const std::string stderr_text = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(result, 0) << stderr_text;
  EXPECT_TRUE(std::filesystem::exists(root + "/2/manifest.json"));
  EXPECT_FALSE(std::filesystem::exists(root + "/1"));
  EXPECT_NE(stderr_text.find("WARNING: cannot lock " + root + "/2/"),
            std::string::npos)
      << stderr_text;
  std::filesystem::remove_all(root);
}

// One lock-daemon hiccup that fails both the session's flock and the root's
// probe leaves the launch running unlocked (as on a root without locks). Its
// next refresh, the daemon answering again, takes the lock, so retention in
// another namespace, whose probe now locks, finds the session held and
// passes it over instead of locking the unheld file and removing it.
// REQ-OPS-11
TEST(LauncherSessions, AnUnlockedSessionIsLockedAtItsNextRefresh) {
  const std::string root = TempPath("lock_recovers");
  std::filesystem::remove_all(root);
  const std::filesystem::path dir = root + "/1";
  bool answering = false;
  const FlockFunction lock = [&answering](int fd, int operation) {
    if (answering) return ::flock(fd, operation);
    errno = ENOLCK;
    return -1;
  };
  std::error_code error;
  auto session = SessionDirectoryLock::Create(
      dir,
      [](const std::filesystem::path& path, std::error_code& made) {
        std::filesystem::create_directories(path, made);
      },
      error, lock);
  ASSERT_TRUE(session.has_value()) << error.message();
  ASSERT_FALSE(session->held());
  EXPECT_EQ(session->Refresh(lock), SessionDirectoryLock::Refreshed::kNone);
  answering = true;
  EXPECT_EQ(session->Refresh(lock), SessionDirectoryLock::Refreshed::kTaken);
  EXPECT_TRUE(session->held());
  EXPECT_TRUE(SessionDirectoryLock::Held(dir, /*root_takes_no_locks=*/false));
  bool held = false;
  EXPECT_FALSE(SessionDirectoryLock::TakeIdle(
                   dir, /*root_takes_no_locks=*/false, held, error)
                   .has_value());
  EXPECT_TRUE(held) << error.message();
  session.reset();
  std::filesystem::remove_all(root);
}

// The flock of the launch's own session.lock failing as on a root without
// locks (ENOLCK) while the root's probe locks: a call's refusal, not the
// root's. The launch tries again; one whose lock keeps failing (all 100
// attempts) is refused and starts nothing, rather than running unlocked
// on a root where retention trusts locks.
// REQ-OPS-11
TEST(LauncherSessions,
     AnOwnLockThatKeepsFailingOnALockingRootRefusesTheLaunch) {
  auto options = SessionFixture("own_lock_fails");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  const std::string lock_file =
      root + "/1/" + std::string{kSessionDirectoryLockFile};
  options.lock_session_directory = [lock_file](int fd, int operation) {
    struct stat locked {};
    struct stat named {};
    if (::fstat(fd, &locked) == 0 && ::stat(lock_file.c_str(), &named) == 0 &&
        locked.st_ino == named.st_ino && locked.st_dev == named.st_dev) {
      errno = ENOLCK;
      return -1;
    }
    return ::flock(fd, operation);
  };
  ::testing::internal::CaptureStderr();
  const int result = Launcher{options}.Run();
  const std::string stderr_text = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(result, 1) << stderr_text;
  EXPECT_FALSE(std::filesystem::exists(root + "/1/manifest.json"))
      << stderr_text;
  EXPECT_NE(stderr_text.find(root + "/1"), std::string::npos) << stderr_text;
  std::filesystem::remove_all(root);
}

// The same failure once, then the lock: the launch holds it.
// REQ-OPS-11
TEST(LauncherSessions, AnOwnLockThatFailsOnceIsTakenOnTheNextAttempt) {
  const std::string root = TempPath("own_lock_fails_once");
  std::filesystem::remove_all(root);
  const std::filesystem::path dir = root + "/1";
  int refusals = 0;
  const FlockFunction lock = [&refusals](int fd, int operation) {
    if ((operation & LOCK_EX) != 0 && refusals == 0) {
      ++refusals;
      errno = ENOLCK;
      return -1;
    }
    return ::flock(fd, operation);
  };
  std::error_code error;
  auto session = SessionDirectoryLock::Create(
      dir,
      [](const std::filesystem::path& path, std::error_code& made) {
        std::filesystem::create_directories(path, made);
      },
      error, lock);
  ASSERT_TRUE(session.has_value()) << error.message();
  EXPECT_EQ(refusals, 1);
  EXPECT_TRUE(session->held());
  EXPECT_TRUE(SessionDirectoryLock::Held(dir, /*root_takes_no_locks=*/false));
  session.reset();
  std::filesystem::remove_all(root);
}

// A flock failing as on a filesystem without advisory locks (ENOLCK: an NFS
// lock daemon that stops answering) on a log root whose locks work says
// nothing about whether a launch holds that session: retention passes the
// session over rather than removing it unlocked. Two running sessions, both
// old enough to go for the free space floor: 1 is locked from the start
// and every flock of its session.lock fails (retention's listing check),
// 2 is locked after the listing, as in
// RetentionKeepsASessionLockedAfterItsListing, and retention's exclusive
// flock of its session.lock fails (the removal's own lock). Every other
// flock, the root's probe included, is real.
// REQ-OPS-11
TEST(LauncherSessions, AOneOffLockFailureLeavesARunningSession) {
  auto options = SessionFixture("lock_hiccup");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  std::string configuration{kFixtureToml};
  configuration.insert(configuration.find("period_us"),
                       "log_max_bytes = 10000000\n");
  std::ofstream{options.config_path} << configuration;
  const auto dir = [&root](int id) { return root + "/" + std::to_string(id); };
  const auto create = [](const std::filesystem::path& path,
                         std::error_code& failed) {
    std::filesystem::create_directories(path, failed);
  };
  WriteSessionManifest(dir(1), 1, 1001);
  WriteDriverStationLog(dir(1), "late", 1, {false}, 4096);
  std::error_code error;
  const auto first = SessionDirectoryLock::Create(dir(1), create, error);
  ASSERT_TRUE(first.has_value() && first->held()) << error.message();
  WriteSessionManifest(dir(2), 2, 1002);
  WriteDriverStationLog(dir(2), "late", 2, {false}, 4096);
  std::ofstream{dir(2) + "/" + std::string{kSessionDirectoryLockFile}}.close();
  const auto before_first = DirectoryContents(dir(1));
  const auto before_second = DirectoryContents(dir(2));
  const auto inode = [](const std::string& path) {
    struct stat status {};
    return ::stat(path.c_str(), &status) == 0 ? status.st_ino : ino_t{0};
  };
  const ino_t first_lock =
      inode(dir(1) + "/" + std::string{kSessionDirectoryLockFile});
  const ino_t second_lock =
      inode(dir(2) + "/" + std::string{kSessionDirectoryLockFile});
  ASSERT_NE(first_lock, ino_t{0});
  ASSERT_NE(second_lock, ino_t{0});
  options.lock_session_directory = [first_lock, second_lock](int fd,
                                                             int operation) {
    struct stat status {};
    if (::fstat(fd, &status) == 0 &&
        (status.st_ino == first_lock ||
         (status.st_ino == second_lock && (operation & LOCK_EX) != 0))) {
      errno = ENOLCK;
      return -1;
    }
    return ::flock(fd, operation);
  };
  const auto second = std::make_shared<std::optional<SessionDirectoryLock>>();
  const uint64_t floor = 10'000'000 + (3 + 1) * kChildOutputLogMaxBytes;
  const std::string listed = dir(2);
  options.available_bytes = [second, listed,
                             floor](const std::filesystem::path&) {
    if (!*second) {
      std::error_code failed;
      *second = SessionDirectoryLock::Create(
          listed,
          [](const std::filesystem::path& path, std::error_code& made) {
            std::filesystem::create_directories(path, made);
          },
          failed);
    }
    // Short by more than both sessions hold, so each is a removal.
    return std::optional<uint64_t>{floor / 2};
  };
  options.session_id = 3;
  ::testing::internal::CaptureStderr();
  const int result = Launcher{options}.Run();
  const std::string stderr_text = ::testing::internal::GetCapturedStderr();
  ASSERT_EQ(result, 0) << stderr_text;
  ASSERT_TRUE(second->has_value() && (*second)->held())
      << "the injected launch could not lock " << listed;
  auto after_first = std::filesystem::exists(dir(1)) ? DirectoryContents(dir(1))
                                                     : decltype(before_first){};
  auto after_second = std::filesystem::exists(dir(2))
                          ? DirectoryContents(dir(2))
                          : decltype(before_second){};
  // The verdict retention's read of a listed session leaves (InspectSession).
  after_second.erase("session_class");
  EXPECT_TRUE(after_first == before_first)
      << "retention removed or changed running session " << dir(1) << "\n"
      << stderr_text;
  EXPECT_TRUE(after_second == before_second)
      << "retention removed or changed running session " << dir(2) << "\n"
      << stderr_text;
  second->reset();
  std::filesystem::remove_all(root);
}

// `studio_bridge` is a valid subsystem name. A robot's own subsystem of
// that name is an ordinary node: critical, its failed probe is an error that
// refuses the launch, not the Studio bridge's warning that leaves it out of
// the session. With --start-studio-bridge, the launcher's bridge would be a
// second roster member of that name: the launch is refused naming the key.
// REQ-OPS-30, REQ-OPS-01
TEST(LauncherSessions, ASubsystemNamedStudioBridgeIsAnOrdinaryNode) {
  auto options = SessionFixture("user_studio_bridge");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  std::ofstream{options.config_path}
      << kFixtureToml
      << "\n[subsystems.studio_bridge]\nnode = \"//example/studio:node\"\n"
         "period_us = 5000\n";
  options.binary_overrides["//example/studio:node"] =
      WriteStubNode("user_studio_bridge_node", kStubWithoutDescribe);
  options.describe_only = true;
  ::testing::internal::CaptureStderr();
  EXPECT_EQ(Launcher{options}.Run(), 1);
  const std::string refused = ::testing::internal::GetCapturedStderr();
  const std::string graph = ReadFile(root + "/1/graph.json");
  const auto errors = GraphErrors(graph);
  EXPECT_TRUE(std::any_of(errors.begin(), errors.end(),
                          [](const std::string& line) {
                            return line.find("studio_bridge") !=
                                   std::string::npos;
                          }))
      << graph << refused;

  options.session_id = 2;
  options.start_studio_bridge = true;
  ::testing::internal::CaptureStderr();
  EXPECT_EQ(Launcher{options}.Run(), 1);
  const std::string twice = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(twice.find("[subsystems.studio_bridge]"), std::string::npos)
      << twice;
  std::filesystem::remove_all(root);
}

// A bundle registers its Studio bridge under the name `studio_bridge` too,
// so the launcher's own bridge is found when [studio] names no bundled
// label. A robot's subsystem of that name whose node is not built is
// missing, never that bridge run in its place; the launcher's bridge still
// finds it by name.
// REQ-OPS-30
TEST(LauncherSessions, ASubsystemNamedStudioBridgeNeverRunsTheBundledBridge) {
  LauncherOptions options;
  options.binary_overrides[std::string{kStudioBridgeName}] = WriteStubNode(
      "bundled_studio_bridge",
      DescribingStub("studio_bridge", "//studio/bridge:studio_bridge", {}));
  options.start_studio_bridge = true;
  const auto specs = DiscoverNodes(
      config::ParseRobotConfigString(
          "[robot]\nname = \"bot\"\nperiod_us = 5000\n"
          "[subsystems.studio_bridge]\nnode = \"//example/studio:unbuilt\"\n"),
      options);
  const auto spec = [&](bool bridge) {
    const auto found =
        std::find_if(specs.begin(), specs.end(), [&](const NodeSpec& s) {
          return s.name == kStudioBridgeName && s.studio_bridge == bridge;
        });
    return found == specs.end() ? std::string{"(none)"} : found->binary_path;
  };
  EXPECT_EQ(spec(/*bridge=*/false), "bazel-bin/example/studio/unbuilt");
  EXPECT_EQ(spec(/*bridge=*/true),
            options.binary_overrides[std::string{kStudioBridgeName}]);
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
        before.find("\n      \"pid\": -1") == std::string::npos) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  } while (std::chrono::steady_clock::now() < deadline);
  ::kill(child, SIGKILL);
  int status = 0;
  ::waitpid(child, &status, 0);
  // The killed launcher never unlinked its session's one topic segment.
  ::shm_unlink(ShmObjectName("/alpha/state").c_str());
  const std::string manifest = ReadFile(path);
  ASSERT_FALSE(manifest.empty());
  const std::string parse =
      "python3 -c 'import json,sys; json.load(open(sys.argv[1]))' '" + path +
      "'";
  EXPECT_EQ(std::system(parse.c_str()), 0);
  EXPECT_NE(manifest.find("\n  \"end_wall_ns\": 0"), std::string::npos);
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
