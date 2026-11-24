#include "talOS/launcher/launcher.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <condition_variable>
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
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "schema_test/shapes_generated.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/input_replay.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/session_manifest.h"
#include "talOS/hardware/packet.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/launcher/file_digest.h"
#include "talOS/launcher/log_budget.h"
#include "talOS/launcher/log_merge.h"
#include "talOS/launcher/session_lock.h"
#include "talOS/launcher/session_retention.h"
#include "talOS/launcher/testdata/first_exec.h"
#include "talOS/launcher/testdata/supervision_events.h"
#include "talOS/protocol/types.h"
#include "talOS/schema/identity.h"

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

// Polls `done()` until it holds, `stop` is requested, or `kHang` elapses.
bool WaitFor(const std::stop_token& stop, const std::function<bool()>& done) {
  const auto give_up = std::chrono::steady_clock::now() + kHang;
  while (!stop.stop_requested() && !done() &&
         std::chrono::steady_clock::now() < give_up)
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
  return !stop.stop_requested() && done();
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
// REQ-CONTRACT-10
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
  const auto closed = TempPath("describe_closed_stdout_marker");
  ::remove(closed.c_str());
  const auto binary = WriteStubNode(
      "describe_closed_stdout",
      "#!/bin/sh\nexec 1>&-\n: > '" + closed + "'\nexec sleep 60\n");
  introspect::Description description;
  std::string error;
  const auto started = std::chrono::steady_clock::now();
  EXPECT_FALSE(ProbeDescribe({binary, "--describe"},
                             std::chrono::milliseconds{1000}, description,
                             error));
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_TRUE(std::filesystem::exists(closed))
      << "probe timed out before child closed stdout";
  EXPECT_LT(elapsed, kHang);
  std::filesystem::remove(binary);
  ::remove(closed.c_str());
}

TEST(LauncherTest, DescribeOutputIsBounded) {
  const auto binary = WriteStubNode(
      "describe_excess_output",
      "#!/bin/sh\nexec dd if=/dev/zero bs=4096 count=1025 2>/dev/null\n");
  introspect::Description description;
  std::string error;
  // Refused for its size, not cut off by the deadline, which only bounds a
  // hang: a refusal by deadline would pass this test without the bound.
  EXPECT_FALSE(
      ProbeDescribe({binary, "--describe"}, kHang, description, error));
  EXPECT_NE(error.find("exceeds"), std::string::npos) << error;
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

// REQ-OPS-09
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
// failing bridge plus long-lived nodes must still end at --duration-s on the
// supervision clock. A bridge none of whose runs came up by then never served
// Studio, and its manifest entry says so (recorder_failed), though its restarts
// were not yet used up. REQ-OPS-30
TEST(LauncherTest, DurationDeadlineIsNotDelayedByFailingBridge) {
  const std::string tag = "bridge_deadline";
  const std::string out_dir = TempPath(tag + "_out");
  std::filesystem::remove_all(out_dir);
  std::filesystem::create_directories(out_dir);

  LauncherOptions options;
  options.output_dir = out_dir;
  options.session_id = 0xDEAD1100ULL;
  options.simulation = true;
  options.start_studio_bridge = true;
  options.duration_s = 1;
  // The stubs never take a registry row; nothing here waits for one.
  options.heartbeat_startup_grace_ms = 60000;
  testing::SteppedClock clock;
  const SupervisionTime start = clock.Freeze();
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
      WriteStubNode(tag + "_bridge", BridgeDescribeThen("exit 1\n"));

  const std::string live =
      out_dir + "/" + std::to_string(options.session_id) + "/manifest.json";
  bool advanced = false;
  std::jthread driver{[&](const std::stop_token& stop) {
    const testing::SteppedClock::ReleaseOnExit release{clock};
    if (!WaitFor(stop, [&] {
          const std::string manifest = ReadFile(live);
          const std::size_t bridge =
              manifest.find("\"name\": \"studio_bridge\"");
          if (bridge == std::string::npos ||
              manifest.find("\n  \"end_wall_ns\": 0,") == std::string::npos)
            return false;
          const std::string entry = manifest.substr(
              bridge, manifest.find("\"runs\": [", bridge) - bridge);
          return entry.find("\"exit_code\": 1") != std::string::npos;
        })) {
      return;
    }
    advanced = true;
    clock.HoldAt(start + std::chrono::seconds{options.duration_s});
  }};

  const auto started = std::chrono::steady_clock::now();
  Launcher launcher{options};
  launcher.Run();
  const auto elapsed = std::chrono::steady_clock::now() - started;
  driver.request_stop();
  driver.join();

  EXPECT_TRUE(advanced) << "studio_bridge never recorded its failing run";
  EXPECT_LT(elapsed, kHang)
      << "deadline must fire without waiting out the bridge window";
  const std::string manifest = ReadFile(live);
  const std::size_t bridge = manifest.find("\"name\": \"studio_bridge\"");
  ASSERT_NE(bridge, std::string::npos) << manifest;
  const std::string entry =
      manifest.substr(bridge, manifest.find("\"runs\": [", bridge) - bridge);
  EXPECT_NE(entry.find("\"recorder_failed\": true"), std::string::npos)
      << "a bridge that never came up is not marked failed: " << entry;

  std::filesystem::remove_all(out_dir);
}

// A run whose event log holds a GAP record is marked at shutdown: log_gap,
// and recorder_failed with it (a gapped log is incomplete evidence), with one
// line naming the member and that run's log. The stub leaves the log a
// writer makes when it drops an oversized record and resumes at the next
// dispatch, in its first run, which crashes, and in its restart: each run's
// log is named once. A member whose log has none is marked neither.
// REQ-OPS-52
TEST(LauncherTest, ALogWithAGapMarksTheMemberAtShutdown) {
  const std::string tag = "log_gap";
  const std::string out_dir = TempPath(tag + "_out");
  std::filesystem::remove_all(out_dir);
  std::filesystem::create_directories(out_dir);
  const std::string gapped = TempPath(tag + "_source.tlog");
  const std::string restarted = TempPath(tag + "_restarted");
  ::remove(restarted.c_str());
  {
    event::log::LogWriter writer{
        gapped, "alpha",
        event::log::LogWriterOptions{512, 1, /*background=*/false}};
    writer.start({}, {});
    const std::vector<std::byte> small(16, std::byte{1});
    const std::vector<std::byte> oversized(1024, std::byte{1});
    for (uint64_t d = 1; d <= 3; ++d) {
      event::Context context{};
      context.kind = event::EventKind::TIMER;
      context.dispatch_index = d;
      writer.dispatch(context, d == 2 ? oversized : small);
    }
    writer.flush();
    ASSERT_EQ(writer.gaps(), 1u);
  }

  LauncherOptions options;
  options.output_dir = out_dir;
  options.session_id = 0x6A9;
  options.simulation = true;
  options.duration_s = static_cast<int>(kHang.count());
  // The stubs never take a registry row; nothing here waits for one.
  options.heartbeat_startup_grace_ms = 60000;
  options.config_path = WriteFixtureToml();
  std::string alpha = NodeDescribeThenSleep("alpha", "//example/alpha:node",
                                            {Sender("/alpha/state")}, 30);
  alpha.erase(alpha.rfind("exec sleep"));
  alpha +=
      "while [ $# -gt 0 ]; do\n"
      " [ \"$1\" = --log ] && { shift; log=\"$1\"; cp '" +
      gapped +
      "' \"$1\"; }\n"
      " shift\n done\n"
      "case \"$log\" in *.restart*) : > '" +
      restarted +
      "'; exec sleep 30 ;; esac\n"
      "exit 1\n";
  options.binary_overrides["//example/alpha:node"] =
      WriteStubNode(tag + "_alpha", alpha);
  options.binary_overrides["//example/beta:node"] = WriteStubNode(
      tag + "_beta", NodeDescribeThenSleep("beta", "//example/beta:node",
                                           {Watcher("/alpha/state")}, 30));
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
      tag + "_hardware",
      NodeDescribeThenSleep("hardware_node", "//talOS/bridge:hardware_node", {},
                            30));

  bool saw_restart = false;
  std::jthread stopper{[&](const std::stop_token& stop) {
    saw_restart =
        WaitFor(stop, [&] { return std::filesystem::exists(restarted); });
    if (saw_restart) ::kill(::getpid(), SIGTERM);
  }};
  ::testing::internal::CaptureStderr();
  Launcher{options}.Run();
  const std::string stderr_text = ::testing::internal::GetCapturedStderr();
  stopper.request_stop();
  stopper.join();
  EXPECT_TRUE(saw_restart) << "alpha's restart never wrote its gapped log";

  const std::string manifest = ReadFile(
      out_dir + "/" + std::to_string(options.session_id) + "/manifest.json");
  const auto entry = [&manifest](std::string_view name) {
    const std::size_t at =
        manifest.find("\"name\": \"" + std::string{name} + "\"");
    if (at == std::string::npos) return std::string{};
    return manifest.substr(at, manifest.find("\"runs\": [", at) - at);
  };
  const std::string gapped_entry = entry("alpha");
  EXPECT_NE(gapped_entry.find("\"log_gap\": true"), std::string::npos)
      << gapped_entry;
  EXPECT_NE(gapped_entry.find("\"recorder_failed\": true"), std::string::npos)
      << gapped_entry;
  const std::string whole_entry = entry("beta");
  EXPECT_NE(whole_entry.find("\"log_gap\": false"), std::string::npos)
      << whole_entry;
  // One line per run, naming the member and that run's log.
  std::vector<std::string> named;
  std::istringstream text{stderr_text};
  for (std::string line; std::getline(text, line);) {
    if (line.find("alpha event log") != std::string::npos &&
        line.find("lost records") != std::string::npos) {
      const std::size_t at = line.find("/alpha.");
      named.push_back(at == std::string::npos
                          ? line
                          : line.substr(at + 1, line.find(' ', at) - at - 1));
    }
  }
  std::sort(named.begin(), named.end());
  EXPECT_EQ(named,
            (std::vector<std::string>{"alpha.restart1.tlog", "alpha.tlog"}))
      << stderr_text;
  std::filesystem::remove_all(out_dir);
  std::filesystem::remove(gapped);
  ::remove(restarted.c_str());
}

// REQ-OPS-52: a GAP record in a rotated segment (`alpha.segment0.tlog`) marks
// the member `log_gap: true` and `recorder_failed: true` at shutdown even when
// the active `alpha.tlog` is clean, and stderr names `alpha` and `alpha.tlog`.
TEST(LauncherTest, ARotatedSegmentWithAGapMarksTheMemberAtShutdown) {
  const std::string tag = "rotated_log_gap";
  const std::string out_dir = TempPath(tag + "_out");
  std::filesystem::remove_all(out_dir);
  std::filesystem::create_directories(out_dir);
  const std::string gapped = TempPath(tag + "_gapped.tlog");
  const std::string clean = TempPath(tag + "_clean.tlog");
  {
    event::log::LogWriter writer{
        gapped, "alpha",
        event::log::LogWriterOptions{512, 1, /*background=*/false}};
    writer.start({}, {});
    const std::vector<std::byte> small(16, std::byte{1});
    const std::vector<std::byte> oversized(1024, std::byte{1});
    for (uint64_t d = 1; d <= 3; ++d) {
      event::Context context{};
      context.kind = event::EventKind::TIMER;
      context.dispatch_index = d;
      writer.dispatch(context, d == 2 ? oversized : small);
    }
    writer.flush();
    ASSERT_EQ(writer.gaps(), 1u);
  }
  {
    event::log::LogWriter writer{
        clean, "alpha",
        event::log::LogWriterOptions{4096, 1, /*background=*/false}};
    writer.start({}, {});
    const std::vector<std::byte> small(16, std::byte{1});
    for (uint64_t d = 1; d <= 2; ++d) {
      event::Context context{};
      context.kind = event::EventKind::TIMER;
      context.dispatch_index = d;
      writer.dispatch(context, small);
    }
    writer.flush();
    ASSERT_EQ(writer.gaps(), 0u);
  }

  LauncherOptions options;
  options.output_dir = out_dir;
  options.session_id = 0x6AA;
  options.simulation = true;
  options.max_restarts = 0;
  options.heartbeat_startup_grace_ms = 60000;
  options.config_path = WriteFixtureToml();
  std::string alpha =
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")});
  alpha.erase(alpha.rfind("exit 0"));
  alpha +=
      "while [ $# -gt 0 ]; do\n"
      "  if [ \"$1\" = --log ]; then\n"
      "    shift\n"
      "    cp '" +
      gapped +
      "' \"${1%.tlog}.segment0.tlog\"\n"
      "    cp '" +
      clean +
      "' \"$1\"\n"
      "  fi\n"
      "  shift\n"
      "done\n"
      "exit 0\n";
  options.binary_overrides["//example/alpha:node"] =
      WriteStubNode(tag + "_alpha", alpha);
  options.binary_overrides["//example/beta:node"] = WriteStubNode(
      tag + "_beta",
      DescribingStub("beta", "//example/beta:node", {Watcher("/alpha/state")}));
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
      tag + "_hardware",
      DescribingStub("hardware_node", "//talOS/bridge:hardware_node", {}));

  ::testing::internal::CaptureStderr();
  EXPECT_EQ(Launcher{options}.Run(), 0);
  const std::string stderr_text = ::testing::internal::GetCapturedStderr();

  const std::string session_dir =
      out_dir + "/" + std::to_string(options.session_id);
  ASSERT_TRUE(std::filesystem::exists(session_dir + "/alpha.segment0.tlog"));
  ASSERT_TRUE(std::filesystem::exists(session_dir + "/alpha.tlog"));
  const std::string manifest = ReadFile(session_dir + "/manifest.json");
  const auto entry = [&manifest](std::string_view name) {
    const std::size_t at =
        manifest.find("\"name\": \"" + std::string{name} + "\"");
    if (at == std::string::npos) return std::string{};
    return manifest.substr(at, manifest.find("\"runs\": [", at) - at);
  };
  const std::string gapped_entry = entry("alpha");
  EXPECT_NE(gapped_entry.find("\"log_gap\": true"), std::string::npos)
      << gapped_entry;
  EXPECT_NE(gapped_entry.find("\"recorder_failed\": true"), std::string::npos)
      << gapped_entry;
  const std::string whole_entry = entry("beta");
  EXPECT_NE(whole_entry.find("\"log_gap\": false"), std::string::npos)
      << whole_entry;
  bool found_stderr_line = false;
  std::istringstream lines{stderr_text};
  for (std::string line; std::getline(lines, line);) {
    if (line.find("alpha") != std::string::npos &&
        line.find("alpha.tlog") != std::string::npos &&
        line.find("lost records") != std::string::npos) {
      found_stderr_line = true;
      break;
    }
  }
  EXPECT_TRUE(found_stderr_line) << stderr_text;

  std::filesystem::remove_all(out_dir);
  std::filesystem::remove(gapped);
  std::filesystem::remove(clean);
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
      return WaitFor(stop, done);
    };
    const auto passes_from_now = [&](uint64_t count) {
      const uint64_t until = passes.Started() + count;
      return wait([&] { return passes.Started() >= until; });
    };
    // beta, spawned after alpha, must have started too: a stop never starts
    // a member it finds unstarted.
    if (!wait([&] {
          return std::filesystem::exists(trapped) &&
                 runs_of(ReadFile(live), "beta")
                         .find("\"reason\": \"running\"") != std::string::npos;
        }))
      return;
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
  EXPECT_LT(elapsed, kHang);
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

// A bridge that survives its settle interval joins the session: it is not
// failed and the session runs to the deadline with it. A session against the
// real controller (no --sim) starts it too, so a deployed robot is viewable
// in Studio (REQ-OPS-30). Once the bridge has spawned, the test moves the
// supervision clock past the 3 s deadline, a whole settle interval after the
// spawn: the deadline's pass finds the bridge up that long, so come up.
// The manifest records the config's [robot] period_us and command_timeout_us,
// which session_report judges the /hw/state cadence against (REQ-OPS-16).
TEST(LauncherTest, StableBridgeJoinsDeployedSessionUntilDeadline) {
  const std::string tag = "bridge_stable";
  const std::string out_dir = TempPath(tag + "_out");
  std::filesystem::create_directories(out_dir);

  LauncherOptions options;
  options.output_dir = out_dir;
  options.session_id = 0x57AB1EULL;
  options.start_studio_bridge = true;
  options.duration_s = 3;
  // The stubs never take a registry row; nothing here waits for one.
  options.heartbeat_startup_grace_ms = 60000;
  testing::SteppedClock clock;
  options.supervision_clock = clock.clock();
  // A [robot] period and timeout other than their defaults.
  std::string toml{kFixtureToml};
  const std::string_view robot_period = "period_us = 5000\nsim_gateway";
  const std::size_t pos = toml.find(robot_period);
  ASSERT_NE(pos, std::string::npos);
  toml.replace(pos, robot_period.size(),
               "period_us = 4000\ncommand_timeout_us = 120000\nsim_gateway");
  options.config_path = TempPath(tag + "_toml") + ".toml";
  std::ofstream{options.config_path} << toml;
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
  EXPECT_NE(manifest.find("\"simulation\": false,\n"
                          "  \"period_us\": 4000,\n"
                          "  \"command_timeout_us\": 120000,\n"),
            std::string::npos)
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
  std::filesystem::remove(options.config_path);
}

// REQ-OPS-30: the Studio bridge is an observer, so how it ends never sets the
// launcher's exit status, while the manifest still records it. Row
// "ignores the stop": a bridge still starting (the supervision clock is held
// inside its first settle interval) ignores the stop's SIGINT and is
// SIGKILLed when the rest of the roster has stopped (StudioBridge::Finish).
// Row "crashed": a bridge that has come up (the clock moved one settle
// interval past its spawn, then two passes) exits 3 mid-session, and so does
// each of its restarts. Either way the operator's stop ends the session with
// 0, and the bridge's entry shows the SIGKILL or the exit code. The other
// members die of the stop's own SIGINT, which leaves the status 0
// (REQ-OPS-06).
TEST(LauncherTest, StudioBridgeEndingNeverSetsTheSessionStatus) {
  struct Row {
    const char* name;
    bool crashes;
    const char* recorded;
  };
  for (const Row row : {Row{"ignores the stop", false, "\"exit_signal\": 9,"},
                        Row{"crashed", true, "\"exit_code\": 3,"}}) {
    SCOPED_TRACE(row.name);
    const std::string tag = "bridge_status";
    const std::string out_dir = TempPath(tag + "_out");
    std::filesystem::remove_all(out_dir);
    std::filesystem::create_directories(out_dir);
    const std::string ready = TempPath(tag + "_ready");
    const std::string crash = TempPath(tag + "_crash");
    ::remove(ready.c_str());
    ::remove(crash.c_str());

    LauncherOptions options;
    options.output_dir = out_dir;
    options.session_id = 0xB51A7ULL;
    options.start_studio_bridge = true;
    options.duration_s = static_cast<int>(kHang.count());
    // The stubs never take a registry row; nothing here waits for one.
    options.heartbeat_startup_grace_ms = 60000;
    testing::SteppedClock clock;
    options.supervision_clock = clock.clock();
    options.config_path = WriteFixtureToml();
    options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
        tag + "_hardware",
        NodeDescribeThenSleep("hardware_node", "//talOS/bridge:hardware_node",
                              {}, 30));
    options.binary_overrides["//example/alpha:node"] = WriteStubNode(
        tag + "_alpha", NodeDescribeThenSleep("alpha", "//example/alpha:node",
                                              {Sender("/alpha/state")}, 30));
    options.binary_overrides["//example/beta:node"] = WriteStubNode(
        tag + "_beta", NodeDescribeThenSleep("beta", "//example/beta:node",
                                             {Watcher("/alpha/state")}, 30));
    options.binary_overrides["//studio/bridge:studio_bridge"] = WriteStubNode(
        tag + "_bridge",
        BridgeDescribeThen(
            row.crashes ? ": > '" + ready + "'\nwhile [ ! -e '" + crash +
                              "' ]; do sleep 0.01; done\nexit 3\n"
                        : "trap '' INT\n: > '" + ready + "'\nexec sleep 30\n"));
    const std::string live =
        out_dir + "/" + std::to_string(options.session_id) + "/manifest.json";
    // The bridge's entry in `manifest`, up to the next member's.
    const auto bridge_entry = [](const std::string& manifest) {
      const auto at = manifest.find("\"name\": \"studio_bridge\"");
      if (at == std::string::npos) return std::string{};
      return manifest.substr(at, manifest.find("\"name\": ", at + 1) - at);
    };
    testing::SupervisionPasses passes;
    bool stopped = false;
    std::jthread stopper{[&](const std::stop_token& stop) {
      const testing::SteppedClock::ReleaseOnExit release{clock};
      const auto wait = [&](const std::function<bool()>& done) {
        const auto give_up = std::chrono::steady_clock::now() + kHang;
        while (!stop.stop_requested() && !done() &&
               std::chrono::steady_clock::now() < give_up)
          std::this_thread::sleep_for(std::chrono::milliseconds{2});
        return !stop.stop_requested() && done();
      };
      if (!wait([&] { return std::filesystem::exists(ready); })) return;
      if (row.crashes) {
        clock.Advance(kStudioBridgeSettleInterval);
        const uint64_t until = passes.Started() + 2;
        if (!wait([&] { return passes.Started() >= until; })) return;
        std::ofstream{crash}.close();
        if (!wait([&] {
              return bridge_entry(ReadFile(live)).find(row.recorded) !=
                     std::string::npos;
            }))
          return;
      } else {
        // Held until the session has ended, so the bridge is still
        // starting when the stop finds it.
        clock.Freeze();
      }
      stopped = true;
      ::kill(::getpid(), SIGTERM);
      wait([] { return false; });
    }};
    const int status = Launcher{options}.Run();
    stopper.request_stop();
    stopper.join();

    ASSERT_TRUE(stopped) << "the bridge never came up";
    EXPECT_EQ(status, 0);
    const std::string entry = bridge_entry(ReadFile(live));
    EXPECT_NE(entry.find(row.recorded), std::string::npos) << entry;
    std::filesystem::remove_all(out_dir);
    ::remove(ready.c_str());
    ::remove(crash.c_str());
  }
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
// latest-only watcher at most once per tick; the bridge only the State,
// commands and statuses it logs (the statuses at 50 Hz, whatever its tick),
// not the ends it declares and never logs.
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
       Endpoint(SourceKind::SENDER, ::bridge::topics::kStatus,
                sizeof(::bridge::BridgeStatus)),
       packet(SourceKind::FETCHER, hardware::RequestTopic("alpha")),
       packet(SourceKind::FETCHER, hardware::RequestTopic("beta"))}};
  const TopicLoads loads = DeclaredTopicLoads({alpha, beta, bridge}, robot);

  // Two motors: 84 + 2 x 45 + 47 encoded bytes after the Packet's size word.
  const TopicLoad state = loads.at(hardware::kStateTopic);
  EXPECT_DOUBLE_EQ(state.per_second, 200);
  EXPECT_DOUBLE_EQ(state.stored_bytes, 4 + 84 + 2 * 45 + 47);
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
  // The bridge: the States it publishes, the commands it sends and its
  // statuses, nothing for the Driver Station packets or the requests it only
  // relays.
  EXPECT_DOUBLE_EQ(ExpectedLogRate(bridge.declared, 5000, true, loads),
                   200 * (64 + state.stored_bytes) + 100 * (64 + 62) +
                       50 * (64 + sizeof(::bridge::BridgeStatus)));
}

// REQ-RT-09: a control loop the OS kept at timeshare is warned about and
// recorded in the manifest session_report --strict judges. A later run that got
// its grant does not clear it: the session still ran that loop without one.
// Only the actuation path is warned about, and --sim excuses a loop that never
// asked.
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

// REQ-OPS-11, REQ-OPS-30: SessionManifest::ToJson carries lock-loss/retake
// flags and per-member `critical` and `studio_bridge`, and DiscoverNodes marks
// the launcher-added Studio bridge `studio_bridge = true, critical = false`.
TEST(LauncherTest, ManifestSerializesLockFlagsCriticalAndStudioBridge) {
  SessionManifest defaults;
  NodeProcessInfo critical_node;
  critical_node.name = "drivetrain";
  NodeProcessInfo bridge;
  bridge.name = "studio_bridge";
  bridge.critical = false;
  bridge.studio_bridge = true;
  defaults.nodes = {critical_node, bridge};

  const std::string default_json = defaults.ToJson();
  EXPECT_NE(default_json.find("\"session_lock_lost\": false"),
            std::string::npos)
      << default_json;
  EXPECT_NE(default_json.find("\"directory_lock_lost\": false"),
            std::string::npos)
      << default_json;
  EXPECT_NE(default_json.find("\"directory_lock_retaken\": false"),
            std::string::npos)
      << default_json;
  const std::size_t dt_pos = default_json.find("\"name\": \"drivetrain\"");
  const std::size_t br_pos = default_json.find("\"name\": \"studio_bridge\"");
  ASSERT_NE(dt_pos, std::string::npos);
  ASSERT_NE(br_pos, std::string::npos);
  const std::string dt_slice = default_json.substr(dt_pos, br_pos - dt_pos);
  const std::string br_slice = default_json.substr(br_pos);
  EXPECT_NE(dt_slice.find("\"critical\": true"), std::string::npos) << dt_slice;
  EXPECT_NE(dt_slice.find("\"studio_bridge\": false"), std::string::npos)
      << dt_slice;
  EXPECT_NE(br_slice.find("\"critical\": false"), std::string::npos)
      << br_slice;
  EXPECT_NE(br_slice.find("\"studio_bridge\": true"), std::string::npos)
      << br_slice;

  SessionManifest anomalous;
  anomalous.session_lock_lost = true;
  anomalous.directory_lock_lost = true;
  anomalous.directory_lock_retaken = true;
  const std::string anomalous_json = anomalous.ToJson();
  EXPECT_NE(anomalous_json.find("\"session_lock_lost\": true"),
            std::string::npos)
      << anomalous_json;
  EXPECT_NE(anomalous_json.find("\"directory_lock_lost\": true"),
            std::string::npos)
      << anomalous_json;
  EXPECT_NE(anomalous_json.find("\"directory_lock_retaken\": true"),
            std::string::npos)
      << anomalous_json;

  // DiscoverNodes marks the launcher-added Studio bridge non-critical.
  LauncherOptions options;
  options.start_studio_bridge = true;
  const auto specs =
      DiscoverNodes(config::ParseRobotConfigString(kFixtureToml), options);
  const auto it = std::find_if(specs.begin(), specs.end(), [](const auto& s) {
    return s.name == kStudioBridgeName;
  });
  ASSERT_NE(it, specs.end());
  EXPECT_TRUE(it->studio_bridge);
  EXPECT_FALSE(it->critical);
}

TEST(LauncherTest, MaxRestartsComesFromFlagThenConfigThenDefault) {
  const auto toml_with_robot_keys = [](std::string_view keys) {
    std::string toml{kFixtureToml};
    const std::string header = "[robot]\n";
    toml.insert(toml.find(header) + header.size(), keys);
    return toml;
  };
  const auto with_robot_keys = [&](std::string_view keys) {
    return config::ParseRobotConfigString(toml_with_robot_keys(keys));
  };
  const auto raw_with_robot_keys = [&](std::string_view keys) {
    config::RobotConfig cfg;
    cfg.toml_data = toml::parse(toml_with_robot_keys(keys));
    return cfg;
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

  // Config load rejects invalid [robot] max_restarts (REQ-CONTRACT-17), and
  // ResolveMaxRestarts also rejects them in depth if given raw toml_data.
  for (const std::string_view bad :
       {"max_restarts = -1\n", "max_restarts = 0\n", "max_restarts = 1.5\n",
        "max_restarts = \"3\"\n"}) {
    SCOPED_TRACE(bad);
    EXPECT_THROW((void)with_robot_keys(bad), std::invalid_argument);
    EXPECT_FALSE(ResolveMaxRestarts(raw_with_robot_keys(bad), {}, max, error));
  }
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

// first_exec.h's store of paid scripts goes with the process that made it,
// so a test binary run outside Bazel (no TEST_TMPDIR to remove) does not
// leave a directory per run; the stubs handed out stay runnable, and a
// forked child's exit removes only its own store. No REQ row: test
// infrastructure.
TEST(FirstExecStubs, StoreIsRemovedWhenItsProcessExits) {
  const std::string parent_stub = WriteStubNode("store_parent", "#!/bin/sh\n");
  const std::string parent_store =
      ::testing::TempDir() + "/talos_first_exec_" + std::to_string(::getpid());
  ASSERT_TRUE(std::filesystem::is_directory(parent_store)) << parent_store;
  const std::string child_tmp = TempPath("first_exec_child_tmp");
  std::filesystem::remove_all(child_tmp);
  std::filesystem::create_directories(child_tmp);
  const std::string child_stub = child_tmp + "/stub";
  std::fflush(nullptr);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0) << std::strerror(errno);
  if (child == 0) {
    ::setenv("TEST_TMPDIR", child_tmp.c_str(), 1);
    testing::WriteWarmStub(child_stub, "#!/bin/sh\nexit 7\n");
    std::exit(0);  // Runs static destructors, as a test binary's exit does.
  }
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0) << status;
  EXPECT_FALSE(std::filesystem::exists(child_tmp + "/talos_first_exec_" +
                                       std::to_string(child)));
  EXPECT_EQ(std::system(child_stub.c_str()), 7 << 8);
  EXPECT_TRUE(std::filesystem::is_directory(parent_store)) << parent_store;
  EXPECT_TRUE(std::filesystem::exists(parent_stub));
  std::filesystem::remove_all(child_tmp);
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
    "    exec sleep 60\n"
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
  EXPECT_NE(graph.find("\"can_buses\": [\"rio\"]"), std::string::npos) << graph;
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

// REQ-OPS-23: graph.json records the robot configuration's CAN bus slot table
// (hardware::CanBuses: slot 0 "rio", then each extra bus in byte order) so
// Studio labels /hw/status can_buses slots by bus name rather than slot index.
TEST(LauncherGraphTest, WritesConfiguredCanBusesInSlotOrderToGraphJson) {
  GraphFixture fixture{
      "can_buses",
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")}),
      DescribingStub("beta", "//example/beta:node", {Watcher("/alpha/state")})};
  const std::string config_path = TempPath("can_buses_fixture") + ".toml";
  {
    std::ofstream out{config_path};
    out << R"(
[robot]
name = "can_buses_fixture"
period_us = 5000
sim_gateway = "//example/sim:gateway"

[subsystems.alpha]
node = "//example/alpha:node"
period_us = 5000

[subsystems.alpha.motors.m1]
type = "TalonFX"
bus = "Shooter"
can_id = 1

[subsystems.beta]
node = "//example/beta:node"
period_us = 10000

[subsystems.beta.motors.m2]
type = "TalonFX"
bus = "Drive"
can_id = 2
)";
  }
  fixture.options.config_path = config_path;
  fixture.options.describe_only = true;

  Launcher launcher{fixture.options};
  EXPECT_EQ(launcher.Run(), 0);

  const std::string graph = fixture.graph_json();
  EXPECT_NE(graph.find("\"can_buses\": [\"rio\", \"Drive\", \"Shooter\"]"),
            std::string::npos)
      << graph;
  std::filesystem::remove(config_path);
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

// REQ-DEV-31: the launcher passes the simulator the bridge's ports, so a
// port written by hand as well is refused before anything is probed or a
// session directory exists, rather than left to disagree with the bridge's.
TEST(LauncherGraphTest, ASimulatorPortWrittenByHandIsRefusedBeforeAnyProbe) {
  for (const std::string_view flag : {"--port", "--remote-port"}) {
    SCOPED_TRACE(flag);
    const std::string invoked = TempPath("simulator_port_invoked");
    ::remove(invoked.c_str());
    GraphFixture fixture{
        "simulator_port",
        RecordingStub("alpha", "//example/alpha:node", invoked),
        RecordingStub("beta", "//example/beta:node", invoked)};
    fixture.options.binary_overrides["//talOS/bridge:hardware_node"] =
        WriteStubNode("simulator_port_hardware",
                      RecordingStub("hardware_node",
                                    "//talOS/bridge:hardware_node", invoked));
    fixture.options.simulation = true;
    fixture.options.start_sim_gateway = true;
    fixture.options.sim_gateway_args = {"--physics", std::string{flag}, "5900"};

    EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
    EXPECT_FALSE(std::filesystem::exists(invoked)) << ReadFile(invoked);
    EXPECT_FALSE(std::filesystem::exists(fixture.out_dir)) << fixture.out_dir;
    ::remove(invoked.c_str());
  }
}

// A config whose `[robot] simulation_only = true` (here set by
// a simulation config that includes the deployable fixture) runs only with
// --sim. Without it the launch is refused before any member is probed or
// spawned and before any session directory exists, naming both
// `simulation_only` and the config path; with --sim it runs, and
// --describe-only and --replay-inputs, which touch no hardware, are allowed.
// --describe-as-launch makes --describe-only refuse it as the launch would
// (deploy.sh's install check, which judges what the boot service will start),
// and is refused without --describe-only, where it would start the roster.
// REQ-SAFE-40, REQ-OPS-50
TEST(LauncherGraphTest, ASimulationOnlyConfigIsRefusedWithoutSim) {
  struct Row {
    const char* name;
    bool simulation;
    bool describe_only;
    bool as_launch;
    bool replay_inputs;
    bool refused;
    bool names_simulation_only;
  };
  for (const Row row :
       {Row{"launch", false, false, false, false, true, true},
        Row{"--sim", true, false, false, false, false, false},
        Row{"--describe-only", false, true, false, false, false, false},
        Row{"--replay-inputs", false, false, false, true, false, false},
        Row{"--describe-only --describe-as-launch", false, true, true, false,
            true, true},
        Row{"--sim --describe-only --describe-as-launch", true, true, true,
            false, false, false},
        Row{"--sim --describe-as-launch", true, false, true, false, true,
            false}}) {
    SCOPED_TRACE(row.name);
    const std::string invoked = TempPath("simulation_only_invoked");
    ::remove(invoked.c_str());
    const std::string recorded = TempPath("simulation_only_recorded");
    std::filesystem::remove_all(recorded);
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
    fixture.options.describe_as_launch = row.as_launch;
    if (row.replay_inputs) {
      std::filesystem::create_directories(recorded);
      std::ofstream{recorded + "/manifest.json"} << "{}\n";
      fixture.options.replay_inputs = recorded;
      fixture.options.only = "alpha";
    }

    ::testing::internal::CaptureStderr();
    const int status = Launcher{fixture.options}.Run();
    const std::string err = ::testing::internal::GetCapturedStderr();
    EXPECT_EQ(status, row.refused ? 1 : 0) << err;
    EXPECT_EQ(std::filesystem::exists(invoked), !row.refused)
        << ReadFile(invoked);
    EXPECT_EQ(std::filesystem::exists(fixture.out_dir), !row.refused)
        << fixture.out_dir;
    if (row.names_simulation_only) {
      EXPECT_NE(err.find("simulation_only"), std::string::npos) << err;
      EXPECT_NE(
          err.find(std::filesystem::weakly_canonical(sim_config).string()),
          std::string::npos)
          << err;
    }
    ::remove(invoked.c_str());
    ::remove(sim_config.c_str());
    std::filesystem::remove_all(recorded);
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
  fixture.options.describe_timeout_ms = 3000;

  const auto started = std::chrono::steady_clock::now();
  Launcher launcher{fixture.options};
  EXPECT_EQ(launcher.Run(), 1);
  EXPECT_FALSE(fixture.has_manifest());
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_LT(elapsed, kHang);
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

// A node that calls talos::AllowUntargetedOutputs only when it is not
// describing gives the lint a probe without the opt-out, so the run's live
// hardware request, which carries it, differs from the probe's. The same
// opt-out described openly is no mismatch.
// REQ-SAFE-42, REQ-CONTRACT-11
TEST(LauncherGraphTest, AnOptOutTheProbeDidNotDescribeIsAnEndpointMismatch) {
  auto described = Sender("/hw/request/alpha", 64);
  described.flags = introspect::naming::kSourceFlagHardwareRequest;
  auto opted_out = described;
  opted_out.flags |= introspect::naming::kSourceFlagUntargetedOutputs;

  const std::string hidden = EndpointMismatch({described}, {opted_out});
  EXPECT_NE(hidden.find("/hw/request/alpha"), std::string::npos) << hidden;
  EXPECT_NE(hidden.find("untargeted_outputs"), std::string::npos) << hidden;
  EXPECT_EQ(EndpointMismatch({opted_out}, {opted_out}), "");
}

// REQ-SAFE-14, REQ-CONTRACT-11: EndpointMismatch catches a live node that
// drops operator_arming on a target lane or drops internal_driver_station on
// /hw/state/driver_station compared to its --describe probe.
TEST(LauncherGraphTest,
     HiddenUnarmedTeleopLaneOrDriverStationIsAnEndpointMismatch) {
  auto armed = Sender("/alpha/target/teleop", 24);
  armed.flags = introspect::naming::kSourceFlagOperatorArming;
  auto unarmed = Sender("/alpha/target/teleop", 24);
  const std::string dropped_arming = EndpointMismatch({armed}, {unarmed});
  EXPECT_NE(dropped_arming.find("/alpha/target/teleop"), std::string::npos)
      << dropped_arming;
  EXPECT_NE(dropped_arming.find("operator_arming"), std::string::npos)
      << dropped_arming;
  EXPECT_EQ(EndpointMismatch({armed}, {armed}), "");

  auto internal_ds = Watcher("/hw/state/driver_station", 1436);
  internal_ds.flags = introspect::naming::kSourceFlagInternalDriverStation;
  auto explicit_ds = Watcher("/hw/state/driver_station", 1436);
  const std::string dropped_internal =
      EndpointMismatch({internal_ds}, {explicit_ds});
  EXPECT_NE(dropped_internal.find("/hw/state/driver_station"),
            std::string::npos)
      << dropped_internal;
  EXPECT_NE(dropped_internal.find("internal_driver_station"), std::string::npos)
      << dropped_internal;
  EXPECT_EQ(EndpointMismatch({internal_ds}, {internal_ds}), "");
}

// SPEC-10 / REQ-SAFE-42, REQ-CONTRACT-11: the launcher passes
// --targeted-outputs ONLY when the member's --describe probe answered,
// --allow-graph-errors is not set, and the probe declared a hardware_request
// endpoint without untargeted_outputs. Non-actuator nodes, nodes declaring
// untargeted_outputs, hardware_node, unanswered probes, and
// --allow-graph-errors launches do not receive it.
TEST(LauncherGraphTest, PassesTargetedOutputsOnlyToTargetedActuatorMembers) {
  const std::string config_path = TempPath("targeted_outputs_cfg") + ".toml";
  {
    std::ofstream out{config_path};
    out << R"(
[robot]
name = "targeted_outputs_fixture"
period_us = 5000

[subsystems.alpha]
node = "//example/alpha:node"

[subsystems.alpha.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.beta]
node = "//example/beta:node"

[subsystems.beta.motors.m2]
type = "TalonFX"
bus = "rio"
can_id = 2

[subsystems.gamma]
node = "//example/gamma:node"
critical = false
)";
  }

  auto alpha_req = Sender("/hw/request/alpha", 64);
  alpha_req.flags = introspect::naming::kSourceFlagHardwareRequest;
  auto beta_req = Sender("/hw/request/beta", 64);
  beta_req.flags = introspect::naming::kSourceFlagHardwareRequest |
                   introspect::naming::kSourceFlagUntargetedOutputs;
  const auto hw_sources = std::vector<introspect::naming::SourceShape>{
      Endpoint(event::SourceKind::FETCHER, "/hw/request/alpha", 64),
      Endpoint(event::SourceKind::FETCHER, "/hw/request/beta", 64)};

  const auto live_lines_of = [](const std::string& invoked,
                                std::string_view member) {
    std::vector<std::string> lines;
    std::istringstream in{invoked};
    const std::string prefix = std::string{member} + " ";
    for (std::string line; std::getline(in, line);) {
      if (line.starts_with(prefix) &&
          line.find("--describe") == std::string::npos) {
        lines.push_back(std::move(line));
      }
    }
    return lines;
  };
  const auto describe_lines_of = [](const std::string& invoked) {
    std::vector<std::string> lines;
    std::istringstream in{invoked};
    for (std::string line; std::getline(in, line);) {
      if (line.find("--describe") != std::string::npos) {
        lines.push_back(std::move(line));
      }
    }
    return lines;
  };

  {
    SCOPED_TRACE("normal launch gates only targeted actuator member");
    const std::string invoked = TempPath("targeted_outputs_normal_invoked");
    ::remove(invoked.c_str());
    const std::string out_dir = TempPath("targeted_outputs_normal_out");
    std::filesystem::remove_all(out_dir);
    std::filesystem::create_directories(out_dir);
    LauncherOptions options;
    options.output_dir = out_dir;
    options.session_id = GenerateSessionId();
    options.config_path = config_path;
    options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
        "to_norm_hw",
        RecordingStub("hardware_node", "//talOS/bridge:hardware_node", invoked,
                      hw_sources));
    options.binary_overrides["//example/alpha:node"] = WriteStubNode(
        "to_norm_alpha",
        RecordingStub("alpha", "//example/alpha:node", invoked, {alpha_req}));
    options.binary_overrides["//example/beta:node"] = WriteStubNode(
        "to_norm_beta",
        RecordingStub("beta", "//example/beta:node", invoked, {beta_req}));
    options.binary_overrides["//example/gamma:node"] = WriteStubNode(
        "to_norm_gamma", RecordingStub("gamma", "//example/gamma:node", invoked,
                                       {Sender("/gamma/state")}));

    EXPECT_EQ(Launcher{options}.Run(), 0);
    const std::string recorded = ReadFile(invoked);
    for (const auto& line : describe_lines_of(recorded)) {
      EXPECT_EQ(line.find("--targeted-outputs"), std::string::npos) << line;
    }
    const auto alpha_live = live_lines_of(recorded, "alpha");
    ASSERT_EQ(alpha_live.size(), 1u) << recorded;
    EXPECT_NE(alpha_live[0].find("--targeted-outputs"), std::string::npos)
        << alpha_live[0];
    for (const std::string_view other : {"beta", "gamma", "hardware_node"}) {
      const auto lines = live_lines_of(recorded, other);
      ASSERT_EQ(lines.size(), 1u) << other << "\n" << recorded;
      EXPECT_EQ(lines[0].find("--targeted-outputs"), std::string::npos)
          << other << ": " << lines[0];
    }
    ::remove(invoked.c_str());
    std::filesystem::remove_all(out_dir);
  }

  {
    SCOPED_TRACE("--allow-graph-errors omits --targeted-outputs");
    const std::string invoked = TempPath("targeted_outputs_allow_invoked");
    ::remove(invoked.c_str());
    const std::string out_dir = TempPath("targeted_outputs_allow_out");
    std::filesystem::remove_all(out_dir);
    std::filesystem::create_directories(out_dir);
    LauncherOptions options;
    options.output_dir = out_dir;
    options.session_id = GenerateSessionId();
    options.config_path = config_path;
    options.simulation = true;
    options.allow_graph_errors = true;
    options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
        "to_allow_hw",
        RecordingStub("hardware_node", "//talOS/bridge:hardware_node", invoked,
                      hw_sources));
    options.binary_overrides["//example/alpha:node"] = WriteStubNode(
        "to_allow_alpha",
        RecordingStub("alpha", "//example/alpha:node", invoked, {alpha_req}));
    options.binary_overrides["//example/beta:node"] = WriteStubNode(
        "to_allow_beta",
        RecordingStub("beta", "//example/beta:node", invoked, {beta_req}));
    options.binary_overrides["//example/gamma:node"] = WriteStubNode(
        "to_allow_gamma", RecordingStub("gamma", "//example/gamma:node",
                                        invoked, {Sender("/gamma/state")}));

    EXPECT_EQ(Launcher{options}.Run(), 0);
    const std::string recorded = ReadFile(invoked);
    const auto alpha_live = live_lines_of(recorded, "alpha");
    ASSERT_EQ(alpha_live.size(), 1u) << recorded;
    EXPECT_EQ(alpha_live[0].find("--targeted-outputs"), std::string::npos)
        << alpha_live[0];
    ::remove(invoked.c_str());
    std::filesystem::remove_all(out_dir);
  }

  {
    SCOPED_TRACE("unanswered non-critical probe omits --targeted-outputs");
    const std::string invoked = TempPath("targeted_outputs_unanswered_invoked");
    ::remove(invoked.c_str());
    const std::string out_dir = TempPath("targeted_outputs_unanswered_out");
    std::filesystem::remove_all(out_dir);
    std::filesystem::create_directories(out_dir);
    const std::string silent_gamma =
        std::string{"#!/bin/sh\necho \"gamma $*\" >> '"} + invoked + "'\n" +
        std::string{kStubWithoutDescribe}.substr(
            std::string{"#!/bin/sh\n"}.size());
    LauncherOptions options;
    options.output_dir = out_dir;
    options.session_id = GenerateSessionId();
    options.config_path = config_path;
    options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
        "to_unans_hw",
        RecordingStub("hardware_node", "//talOS/bridge:hardware_node", invoked,
                      hw_sources));
    options.binary_overrides["//example/alpha:node"] = WriteStubNode(
        "to_unans_alpha",
        RecordingStub("alpha", "//example/alpha:node", invoked, {alpha_req}));
    options.binary_overrides["//example/beta:node"] = WriteStubNode(
        "to_unans_beta",
        RecordingStub("beta", "//example/beta:node", invoked, {beta_req}));
    options.binary_overrides["//example/gamma:node"] =
        WriteStubNode("to_unans_gamma", silent_gamma);

    EXPECT_EQ(Launcher{options}.Run(), 0);
    const std::string recorded = ReadFile(invoked);
    const auto gamma_live = live_lines_of(recorded, "gamma");
    ASSERT_EQ(gamma_live.size(), 1u) << recorded;
    EXPECT_EQ(gamma_live[0].find("--targeted-outputs"), std::string::npos)
        << gamma_live[0];
    ::remove(invoked.c_str());
    std::filesystem::remove_all(out_dir);
  }

  ::remove(config_path.c_str());
}

// SPEC-10 / REQ-SAFE-42, REQ-CONTRACT-11: a critical actuator member whose
// --describe probe declared hardware_request without untargeted_outputs
// receives --targeted-outputs on BOTH run 0 and every restart (Restart reuses
// launch.args), so a member hiding AllowUntargetedOutputs behind
// !Node::describing() fails construction on both run 0 and restart instead of
// ticking untargeted.
TEST(LauncherGraphTest, HiddenOptOutRefusesBothFirstRunAndRestart) {
  const std::string out_dir = TempPath("targeted_restart_out");
  const std::string runs_file = TempPath("targeted_restart_runs");
  const std::string ticked_file = TempPath("targeted_restart_ticked");
  const std::string done_fifo = TempPath("targeted_restart_fifo");
  std::filesystem::remove_all(out_dir);
  std::filesystem::create_directories(out_dir);
  ::remove(runs_file.c_str());
  ::remove(ticked_file.c_str());
  ::unlink(done_fifo.c_str());
  ASSERT_EQ(::mkfifo(done_fifo.c_str(), 0600), 0) << std::strerror(errno);
  const int fifo_fd = ::open(done_fifo.c_str(), O_RDWR | O_CLOEXEC);
  ASSERT_GE(fifo_fd, 0) << std::strerror(errno);

  auto alpha_req = Sender("/hw/request/alpha", 64);
  alpha_req.flags = introspect::naming::kSourceFlagHardwareRequest;
  const std::string alpha_script =
      std::string{"#!/bin/sh\n"} +
      "for arg in \"$@\"; do\n"
      "  if [ \"$arg\" = \"--describe\" ]; then\n"
      "    cat <<'TALOS_DESCRIBE'\n" +
      DescribeJson("alpha", "//example/alpha:node", {alpha_req}) +
      "TALOS_DESCRIBE\n"
      "    exit 0\n"
      "  fi\n"
      "done\n"
      "echo \"$*\" >> '" +
      runs_file +
      "'\n"
      "has_flag=0\n"
      "for arg in \"$@\"; do\n"
      "  if [ \"$arg\" = \"--targeted-outputs\" ]; then\n"
      "    has_flag=1\n"
      "  fi\n"
      "done\n"
      "if [ \"$has_flag\" -eq 0 ]; then\n"
      "  echo untargeted >> '" +
      ticked_file +
      "'\n"
      "  exit 0\n"
      "fi\n"
      "count=$(wc -l < '" +
      runs_file +
      "')\n"
      "if [ \"$count\" -ge 2 ]; then\n"
      "  printf 'd' > '" +
      done_fifo +
      "'\n"
      "fi\n"
      "echo 'alpha: AllowUntargetedOutputs refused under --targeted-outputs' "
      ">&2\n"
      "exit 1\n";

  LauncherOptions options;
  options.output_dir = out_dir;
  options.session_id = GenerateSessionId();
  options.config_path = WriteFixtureToml();
  options.duration_s = static_cast<int>(kHang.count());
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
      "to_restart_hw",
      NodeDescribeThenSleep(
          "hardware_node", "//talOS/bridge:hardware_node",
          {Endpoint(event::SourceKind::FETCHER, "/hw/request/alpha", 64)}, 30));
  options.binary_overrides["//example/alpha:node"] =
      WriteStubNode("to_restart_alpha", alpha_script);
  options.binary_overrides["//example/beta:node"] = WriteStubNode(
      "to_restart_beta",
      NodeDescribeThenSleep("beta", "//example/beta:node", {}, 30));

  std::jthread stopper{[fifo_fd](const std::stop_token& stop) {
    char byte = 0;
    if (::read(fifo_fd, &byte, 1) == 1 && byte == 'd' &&
        !stop.stop_requested()) {
      ::kill(::getpid(), SIGTERM);
    }
  }};

  Launcher{options}.Run();
  stopper.request_stop();
  const char wake = 'x';
  (void)::write(fifo_fd, &wake, 1);
  stopper.join();
  ::close(fifo_fd);
  ::unlink(done_fifo.c_str());

  EXPECT_FALSE(std::filesystem::exists(ticked_file))
      << "member ticked untargeted: " << ReadFile(ticked_file);
  std::vector<std::string> run_lines;
  {
    std::istringstream in{ReadFile(runs_file)};
    for (std::string line; std::getline(in, line);) {
      if (!line.empty()) run_lines.push_back(std::move(line));
    }
  }
  ASSERT_GE(run_lines.size(), 2u) << "expected both run 0 and restart 1; got:\n"
                                  << ReadFile(runs_file);
  EXPECT_NE(run_lines[0].find("alpha.tlog"), std::string::npos) << run_lines[0];
  EXPECT_NE(run_lines[1].find("alpha.restart1.tlog"), std::string::npos)
      << run_lines[1];
  for (std::size_t i = 0; i < run_lines.size(); ++i) {
    EXPECT_NE(run_lines[i].find("--targeted-outputs"), std::string::npos)
        << "run " << i << ": " << run_lines[i];
  }

  ::remove(runs_file.c_str());
  ::remove(ticked_file.c_str());
  std::filesystem::remove_all(out_dir);
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
  EXPECT_NE(errors[0].find("bazel build"), std::string::npos) << errors[0];
  EXPECT_NE(errors[0].find("//example/alpha:node"), std::string::npos)
      << errors[0];
  EXPECT_NE(errors[0].find("//example/beta:node"), std::string::npos)
      << errors[0];
  EXPECT_EQ(errors[0].find("//talOS/bridge:hardware_node"), std::string::npos)
      << "the built bridge is not to be rebuilt: " << errors[0];
}

// A bundle's reader is at the robot after deploy.sh install, with no Bazel
// and no source tree: the refusal names the bundle and the missing binary,
// and the cure is to rebuild and deploy it, never a `bazel build` there.
// REQ-OPS-01
TEST(LauncherGraphTest, MissingBundleBinaryNamesTheBundleNotABazelBuild) {
  const NodeSpec missing{.name = "alpha",
                         .target = "//example/alpha:node",
                         .binary_path = "/opt/robot/bin/alpha",
                         .owns_actuators = true};
  const auto finding = MissingBinariesFinding({missing}, "/opt/robot");
  ASSERT_TRUE(finding.has_value());
  EXPECT_EQ(finding->severity, naming::Severity::ERROR);
  EXPECT_NE(finding->message.find("/opt/robot/bin/alpha"), std::string::npos)
      << finding->message;
  EXPECT_NE(finding->message.find("bundle /opt/robot"), std::string::npos)
      << finding->message;
  EXPECT_NE(finding->message.find("deploy"), std::string::npos)
      << finding->message;
  EXPECT_EQ(finding->message.find("bazel"), std::string::npos)
      << "a robot has no Bazel: " << finding->message;
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

// REQ-OPS-01: when every missing binary is non-critical (`critical = false`),
// the single missing-binaries finding is Severity::WARNING with `bazel build`,
// and launch proceeds for the remaining peers.
TEST(LauncherGraphTest, MissingNonCriticalBinariesWarnAndLaunchProceeds) {
  const NodeSpec missing{.name = "alpha",
                         .target = "//example/alpha:node",
                         .binary_path = TempPath("never_built_optional_alpha"),
                         .critical = false,
                         .owns_actuators = false};
  const auto finding = MissingBinariesFinding({missing}, "");
  ASSERT_TRUE(finding.has_value());
  EXPECT_EQ(finding->severity, naming::Severity::WARNING);
  EXPECT_NE(finding->message.find("bazel build"), std::string::npos)
      << finding->message;
  EXPECT_NE(finding->message.find("//example/alpha:node"), std::string::npos)
      << finding->message;

  const std::string beta_runs = TempPath("missing_noncrit_beta_runs");
  ::remove(beta_runs.c_str());
  GraphFixture fixture{"missing_noncrit", kStubWithoutDescribe,
                       RecordingStub("beta", "//example/beta:node", beta_runs,
                                     {Sender("/beta/state")})};
  auto config = toml::parse_file(fixture.options.config_path);
  auto* alpha = config["subsystems"]["alpha"].as_table();
  alpha->erase("motors");
  alpha->insert_or_assign("critical", false);
  std::ofstream{fixture.options.config_path} << config;
  fixture.options.binary_overrides["//example/alpha:node"] =
      missing.binary_path;
  fixture.options.max_restarts = 0;

  Launcher{fixture.options}.Run();
  const std::string graph = fixture.graph_json();
  EXPECT_EQ(GraphErrors(graph), std::vector<std::string>{}) << graph;
  const auto warnings = GraphFindings(graph, "warning", "alpha");
  ASSERT_EQ(warnings.size(), 1u) << graph;
  EXPECT_NE(warnings[0].find("bazel build"), std::string::npos) << warnings[0];
  EXPECT_NE(warnings[0].find("//example/alpha:node"), std::string::npos)
      << warnings[0];
  EXPECT_TRUE(fixture.has_manifest());
  bool beta_launched = false;
  std::istringstream lines{ReadFile(beta_runs)};
  for (std::string line; std::getline(lines, line);) {
    if (line.find("--describe") == std::string::npos &&
        line.find("beta ") == 0) {
      beta_launched = true;
    }
  }
  EXPECT_TRUE(beta_launched) << ReadFile(beta_runs);
  ::remove(beta_runs.c_str());
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

// REQ-SAFE-28: the hardware bridge is always critical. `critical` is read only
// from a subsystem's table, so a config that replaces the bridge's binary and
// marks every subsystem critical = false still lists the bridge critical.
TEST(LauncherTest, TheHardwareBridgeIsAlwaysCritical) {
  constexpr std::string_view kEveryNodeOptional = R"(
[robot]
name = "optional_everything"
period_us = 5000
hardware_node = "//example/bridge:node"

[subsystems.alpha]
node = "//example/alpha:node"
critical = false
)";
  const auto specs = DiscoverNodes(
      config::ParseRobotConfigString(kEveryNodeOptional), LauncherOptions{});
  const auto bridge = std::find_if(
      specs.begin(), specs.end(),
      [](const NodeSpec& spec) { return spec.name == kHardwareNodeName; });
  ASSERT_NE(bridge, specs.end());
  EXPECT_EQ(bridge->target, "//example/bridge:node");
  EXPECT_TRUE(bridge->critical);
  const auto alpha =
      std::find_if(specs.begin(), specs.end(),
                   [](const NodeSpec& spec) { return spec.name == "alpha"; });
  ASSERT_NE(alpha, specs.end());
  EXPECT_FALSE(alpha->critical);
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

// REQ-SAFE-28
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

// REQ-SAFE-28: full Launcher::Run() refuses to launch (exits non-zero, writes
// no manifest, spawns nothing after --describe) when a critical = false
// subsystem owns a motor, digital output, or PWM output, or describes a
// target-lane sender.
TEST(LauncherGraphTest, NonCriticalActuatorOrTargetProducerRefusesLaunch) {
  const auto check_only_described = [](const std::string& record) {
    std::istringstream in{ReadFile(record)};
    std::string line;
    bool saw_any = false;
    while (std::getline(in, line)) {
      if (line.empty()) continue;
      saw_any = true;
      EXPECT_NE(line.find("--describe"), std::string::npos)
          << "node was spawned after --describe: " << line;
    }
    EXPECT_TRUE(saw_any) << "expected --describe invocations";
  };

  for (const auto* section : {"motors", "digital_outputs", "pwm_outputs"}) {
    SCOPED_TRACE(section);
    const std::string record =
        TempPath(std::string{"optional_actuator_"} + section + "_runs");
    ::remove(record.c_str());
    GraphFixture fixture{"optional_actuator_run",
                         RecordingStub("alpha", "//example/alpha:node", record),
                         RecordingStub("beta", "//example/beta:node", record)};
    fixture.options.binary_overrides["//talOS/bridge:hardware_node"] =
        WriteStubNode(std::string{"optional_actuator_"} + section + "_hw",
                      RecordingStub("hardware_node",
                                    "//talOS/bridge:hardware_node", record));
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
    std::ofstream{fixture.options.config_path} << config;

    EXPECT_EQ(Launcher{fixture.options}.Run(), 1) << section;
    EXPECT_FALSE(fixture.has_manifest()) << section;
    const std::string graph = fixture.graph_json();
    const std::size_t err_pos = graph.find("\"severity\": \"error\"");
    ASSERT_NE(err_pos, std::string::npos) << graph;
    const std::string diag = graph.substr(err_pos);
    EXPECT_NE(diag.find("alpha"), std::string::npos) << diag;
    EXPECT_NE(diag.find("critical"), std::string::npos) << diag;
    check_only_described(record);
    ::remove(record.c_str());
  }

  {
    SCOPED_TRACE("non_critical_target_producer");
    const std::string record = TempPath("optional_target_runs");
    ::remove(record.c_str());
    GraphFixture fixture{"optional_target_run",
                         RecordingStub("alpha", "//example/alpha:node", record,
                                       {Sender("/beta/target/teleop")}),
                         RecordingStub("beta", "//example/beta:node", record,
                                       {Watcher("/beta/target/teleop")})};
    fixture.options.binary_overrides["//talOS/bridge:hardware_node"] =
        WriteStubNode("optional_target_hw",
                      RecordingStub("hardware_node",
                                    "//talOS/bridge:hardware_node", record));
    auto config = toml::parse_file(fixture.options.config_path);
    auto* alpha = config["subsystems"]["alpha"].as_table();
    alpha->erase("motors");
    alpha->insert_or_assign("critical", false);
    auto* beta = config["subsystems"]["beta"].as_table();
    toml::table targets;
    targets.insert("teleop", toml::array{"teleop"});
    beta->insert_or_assign("targets", std::move(targets));
    std::ofstream{fixture.options.config_path} << config;

    EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
    EXPECT_FALSE(fixture.has_manifest());
    const std::string graph = fixture.graph_json();
    const std::size_t err_pos = graph.find("\"severity\": \"error\"");
    ASSERT_NE(err_pos, std::string::npos) << graph;
    const std::string diag = graph.substr(err_pos);
    EXPECT_NE(diag.find("alpha"), std::string::npos) << diag;
    EXPECT_NE(diag.find("critical"), std::string::npos) << diag;
    check_only_described(record);
    ::remove(record.c_str());
  }
}

// REQ-OPS-01
TEST(LauncherGraphTest, NonCriticalDescribeTimeoutAllowsPeersToLaunch) {
  GraphFixture fixture{
      "optional_timeout", kStubThatHangsOnDescribe,
      DescribingStub("beta", "//example/beta:node", {Sender("/beta/state")})};
  auto config = toml::parse_file(fixture.options.config_path);
  auto* alpha = config["subsystems"]["alpha"].as_table();
  alpha->erase("motors");
  alpha->insert_or_assign("critical", false);
  std::ofstream{fixture.options.config_path} << config;
  // Only alpha (which hangs for 60 s) must miss the deadline; give critical
  // peers generous headroom even under heavy host load.
  fixture.options.describe_timeout_ms = 5000;
  fixture.options.max_restarts = 0;
  EXPECT_EQ(Launcher{fixture.options}.Run(), 0);
  EXPECT_TRUE(fixture.has_manifest());
  const std::string graph = fixture.graph_json();
  EXPECT_EQ(graph.find("\"severity\": \"error\""), std::string::npos) << graph;
  EXPECT_EQ(GraphFindings(graph, "warning", "alpha").size(), 1u) << graph;
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

// REQ-DEV-04
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

// REQ-DEV-04, REQ-OPS-01: a graph error makes --describe-only exit non-zero,
// no manifest
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
// REQ-OPS-01
TEST(LauncherGraphTest, ProbeReportsWhyABinaryCannotRun) {
  introspect::Description out;
  std::string error;
  EXPECT_FALSE(ProbeDescribe({"/nonexistent/talos_node", "--describe"}, kHang,
                             out, error));
  EXPECT_NE(error.find("execv"), std::string::npos) << error;
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

  // The lint judged the second description: gamma's observer records
  // alpha's state, so it is read and draws no "nothing subscribes" warning,
  // which the first description (no sources) would.
  const bool unread =
      std::any_of(diagnostics.begin(), diagnostics.end(),
                  [](const introspect::naming::Diagnostic& d) {
                    return d.subject == "/alpha/state" &&
                           d.severity == introspect::naming::Severity::WARNING;
                  });
  EXPECT_FALSE(unread);

  std::filesystem::remove_all(record);
  std::filesystem::remove_all(options.output_dir);
  std::remove(alpha.binary_path.c_str());
  std::remove(gamma.binary_path.c_str());
}

// REQ-OPS-55: a graph-reading node's Pass-1 static non-telemetry SENDER (such
// as teleop's /teleop/status) is exported to declared_graph.json so another
// graph reader (telemetry) can observe it, while telemetry's own
// telemetry-role SENDER (/talos/telemetry) and Pass-1 watchers are omitted so
// telemetry never sees itself.
TEST(LauncherGraphTest,
     TelemetrySeesOtherGraphReadersPass1StaticSenderAndNotItsOwn) {
  const std::string record = TempPath("graph_pass1_static_record");
  std::filesystem::create_directories(record);

  auto two_pass_stub = [&](const introspect::Description& pass1,
                           const introspect::Description& pass2,
                           const std::string& save_as) {
    return std::string{"#!/bin/sh\n"} +
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
           introspect::DescribeToJson(pass1) +
           "TALOS_DESCRIBE\n"
           "  exit 0\n"
           "fi\n" +
           (save_as.empty()
                ? std::string{}
                : "cp \"$graph\" '" + record + "/" + save_as + "'\n") +
           "cat <<'TALOS_DESCRIBE'\n" + introspect::DescribeToJson(pass2) +
           "TALOS_DESCRIBE\n";
  };

  introspect::Description teleop_p1;
  teleop_p1.name = "teleop";
  teleop_p1.target = "//example/teleop:node";
  teleop_p1.reads_graph = true;
  teleop_p1.sources = {Sender("/teleop/status"),
                       Watcher("/hw/state/driver_station")};
  introspect::Description teleop_p2 = teleop_p1;
  teleop_p2.sources.push_back(Sender("/alpha/target/teleop"));

  introspect::Description telemetry_p1;
  telemetry_p1.name = "telemetry";
  telemetry_p1.target = "//example/telemetry:node";
  telemetry_p1.reads_graph = true;
  telemetry_p1.sources = {Sender("/talos/telemetry")};
  introspect::Description telemetry_p2 = telemetry_p1;
  auto obs_alpha = Watcher("/alpha/state");
  obs_alpha.flags = introspect::naming::kSourceFlagObserver;
  auto obs_teleop = Watcher("/teleop/status");
  obs_teleop.flags = introspect::naming::kSourceFlagObserver;
  telemetry_p2.sources.push_back(obs_alpha);
  telemetry_p2.sources.push_back(obs_teleop);

  NodeSpec alpha{"alpha", "//example/alpha:node",
                 WriteStubNode("graph_pass1_alpha",
                               DescribingStub("alpha", "//example/alpha:node",
                                              {Sender("/alpha/state")}))};
  NodeSpec teleop{"teleop", "//example/teleop:node",
                  WriteStubNode("graph_pass1_teleop",
                                two_pass_stub(teleop_p1, teleop_p2, ""))};
  NodeSpec telemetry{
      "telemetry", "//example/telemetry:node",
      WriteStubNode("graph_pass1_telemetry",
                    two_pass_stub(telemetry_p1, telemetry_p2, "graph"))};

  LauncherOptions options;
  options.output_dir = TempPath("graph_pass1_static_out");
  std::vector<NodeDescription> described;
  DescribeGraph({telemetry, teleop, alpha}, options, described);

  ASSERT_EQ(described.size(), 3u);
  EXPECT_TRUE(described[0].answered) << described[0].error;
  EXPECT_TRUE(described[1].answered) << described[1].error;
  EXPECT_TRUE(described[2].answered) << described[2].error;

  introspect::DeclaredGraph graph;
  std::string error;
  ASSERT_TRUE(introspect::DescribeReader::ParseGraph(
      ReadFile(record + "/graph"), graph, error))
      << error;
  ASSERT_EQ(graph.size(), 2u);

  const auto find_node = [&](std::string_view name) {
    return std::find_if(
        graph.begin(), graph.end(),
        [&](const introspect::Description& d) { return d.name == name; });
  };
  EXPECT_NE(find_node("alpha"), graph.end());
  EXPECT_EQ(find_node("telemetry"), graph.end());
  const auto teleop_it = find_node("teleop");
  ASSERT_NE(teleop_it, graph.end());
  ASSERT_EQ(teleop_it->sources.size(), 1u);
  EXPECT_EQ(teleop_it->sources[0].kind, event::SourceKind::SENDER);
  EXPECT_EQ(teleop_it->sources[0].name, "/teleop/status");

  std::filesystem::remove_all(record);
  std::filesystem::remove_all(options.output_dir);
  std::remove(alpha.binary_path.c_str());
  std::remove(teleop.binary_path.c_str());
  std::remove(telemetry.binary_path.c_str());
}

// A graph file the launch cannot write -- the declared graph its graph
// readers are described and started with, or graph.json -- refuses the launch
// before anything starts.
TEST(LauncherGraphTest, AGraphFileThatCannotBeWrittenRefusesTheLaunch) {
  const std::string record = TempPath("unwritable_graph_record");
  std::filesystem::create_directories(record);
  for (const auto& [tag, file] :
       // kSessionQuietFiles' names, so the hourly refresh touches the files
       // a launch writes, renamed or not.
       {std::pair{"unwritable_declared_graph", kSessionQuietFiles[2]},
        std::pair{"unwritable_graph", kSessionQuietFiles[1]}}) {
    GraphFixture fixture{
        tag,
        DescribingStub("alpha", "//example/alpha:node",
                       {Sender("/alpha/state")}),
        GraphReaderStub("beta", "//example/beta:node", record)};
    // A directory where the file belongs: no write replaces it, even as root.
    std::filesystem::create_directories(fixture.out_dir + "/" +
                                        std::string{file});
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

// REQ-DEV-30
TEST(LauncherBundleTest, RepoMappingResolvesCustomApparentAndCanonicalNames) {
  const RepoMapping mapping = ParseRepoMapping(
      ",my_talos,talos+\n"
      ",legacy_talos,talos~\n"
      ",robot,_main\n"
      "talos+,rules_cc,rules_cc+\n"
      "malformed_line\n");
  ASSERT_EQ(mapping.size(), 3U);
  EXPECT_EQ(mapping.at("my_talos"), "talos+");
  EXPECT_EQ(mapping.at("legacy_talos"), "talos~");
  EXPECT_EQ(mapping.at("robot"), "_main");

  // Apparent names resolve through `_repo_mapping` for both Bazel 8 (+) and
  // Bazel 7 (~), and canonical names are used directly without appending `+`.
  const std::map<std::string, std::string> empty_overrides;
  EXPECT_EQ(
      ResolveBinary("@my_talos//talOS/teleop:node", empty_overrides, mapping),
      "bazel-bin/external/talos+/talOS/teleop/node");
  EXPECT_EQ(ResolveBinary("@legacy_talos//talOS/teleop:node", empty_overrides,
                          mapping),
            "bazel-bin/external/talos~/talOS/teleop/node");
  EXPECT_EQ(
      ResolveBinary("@@talos~//talOS/teleop:node", empty_overrides, mapping),
      "bazel-bin/external/talos~/talOS/teleop/node");
  EXPECT_EQ(ResolveBinary("@@plain_canon//pkg:node", empty_overrides, mapping),
            "bazel-bin/external/plain_canon/pkg/node");
  EXPECT_EQ(ResolveBinary("@robot//gate:node", empty_overrides, mapping),
            "bazel-bin/gate/node");
  EXPECT_EQ(ResolveBinary("@@//gate:node", empty_overrides, mapping),
            "bazel-bin/gate/node");
  EXPECT_EQ(ResolveBinary("@@_main//gate:node", empty_overrides, mapping),
            "bazel-bin/gate/node");

  // NormalizeTargetKey aligns apparent and canonical spellings via the mapping
  // even when repo_name != module_name, without colliding unrelated repos.
  EXPECT_EQ(NormalizeTargetKey("@my_talos//talOS/teleop:node", mapping),
            NormalizeTargetKey("@@talos+//talOS/teleop:node", mapping));
  EXPECT_EQ(NormalizeTargetKey("@legacy_talos//talOS/teleop:node", mapping),
            NormalizeTargetKey("@@talos~//talOS/teleop:node", mapping));
  EXPECT_NE(NormalizeTargetKey("@talos//talOS/teleop:node", mapping),
            NormalizeTargetKey("@@talos+//talOS/teleop:node", mapping));
  EXPECT_EQ(NormalizeTargetKey("@robot//gate:node", mapping), "//gate:node");
  EXPECT_EQ(NormalizeTargetKey("@@//gate:node", mapping), "//gate:node");
  EXPECT_EQ(NormalizeTargetKey("@@_main//gate:node", mapping), "//gate:node");

  std::map<std::string, std::string> overrides;
  overrides["@@talos+//talOS/teleop:node"] = "/bundle/bin/teleop";
  overrides["@@talos~//talOS/telemetry:node"] = "/bundle/bin/telemetry";
  EXPECT_EQ(ResolveBinary("@my_talos//talOS/teleop:node", overrides, mapping),
            "/bundle/bin/teleop");
  EXPECT_EQ(
      ResolveBinary("@legacy_talos//talOS/telemetry:node", overrides, mapping),
      "/bundle/bin/telemetry");

  // LoadRepoMapping reads `<argv0>.runfiles/_repo_mapping`.
  const std::string fake_root = TempPath("repo_mapping_test");
  const std::string fake_bin = fake_root + "/launcher";
  std::filesystem::create_directories(fake_bin + ".runfiles");
  {
    std::ofstream f(fake_bin + ".runfiles/_repo_mapping");
    f << ",custom_talos,talos+\n";
  }
  const RepoMapping loaded = LoadRepoMapping(fake_bin);
  ASSERT_EQ(loaded.size(), 1U);
  EXPECT_EQ(loaded.at("custom_talos"), "talos+");
  std::filesystem::remove_all(fake_root);
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

// REQ-OPS-10 (REPO-4): LoadBundle reads dual-revision stamps from
// bundle_manifest.toml, and SessionManifest::ToJson emits talos_git_revision /
// talos_git_dirty only when talos_git_revision is non-empty.
TEST(LauncherBundleTest, LoadBundleAndManifestCarryTalosRevisionWhenPresent) {
  const std::string bundle_dir = TempPath("test_bundle_talos_rev");
  std::filesystem::create_directories(bundle_dir + "/config");
  { std::ofstream{bundle_dir + "/config/robot.toml"} << "[robot]\n"; }

  {
    std::ofstream mf(bundle_dir + "/bundle_manifest.toml");
    mf << "bundle_version = 1\n"
       << "config_path = \"config/robot.toml\"\n"
       << "git_revision = \"1111aaaa\"\n"
       << "git_dirty = false\n"
       << "talos_git_revision = \"2222bbbb\"\n"
       << "talos_git_dirty = true\n";
  }

  LauncherOptions options;
  options.bundle_dir = bundle_dir;
  std::string error;
  ASSERT_TRUE(LoadBundle(options, error)) << error;
  EXPECT_EQ(options.bundle_git_revision, "1111aaaa");
  EXPECT_FALSE(options.bundle_git_dirty);
  EXPECT_EQ(options.bundle_talos_git_revision, "2222bbbb");
  EXPECT_TRUE(options.bundle_talos_git_dirty);

  SessionManifest dual;
  dual.git_revision = options.bundle_git_revision;
  dual.git_dirty = options.bundle_git_dirty;
  dual.talos_git_revision = options.bundle_talos_git_revision;
  dual.talos_git_dirty = options.bundle_talos_git_dirty;
  const std::string dual_json = dual.ToJson();
  EXPECT_NE(dual_json.find("\"talos_git_revision\": \"2222bbbb\""),
            std::string::npos);
  EXPECT_NE(dual_json.find("\"talos_git_dirty\": true"), std::string::npos);

  // Single-repo or unstamped manifests omit the talos_git_* keys completely.
  SessionManifest single;
  single.git_revision = "1111aaaa";
  const std::string single_json = single.ToJson();
  EXPECT_EQ(single_json.find("talos_git_revision"), std::string::npos);
  EXPECT_EQ(single_json.find("talos_git_dirty"), std::string::npos);

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
  const uint32_t old_version = event::log::OLDEST_READABLE_VERSION - 1;
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

// Writes a bridge event log (`hardware_node.tlog`) in `dir` that publishes one
// `/hw/status` record carrying `protocol_version_id = protocol_version`.
void WriteBridgeStatusLog(const std::string& dir, uint64_t id,
                          uint16_t protocol_version) {
  event::Registration status_source;
  status_source.id = 1;
  status_source.kind = event::SourceKind::SENDER;
  status_source.name = std::string{::bridge::topics::kStatus};
  status_source.message_bytes = sizeof(::bridge::BridgeStatus);
  status_source.schema = schema::Of<::bridge::BridgeStatus>();
  event::log::LogWriter writer{
      dir + "/hardware_node.tlog", "hardware_node",
      event::log::LogWriterOptions{1 << 14, 1, /*background=*/false}, id};
  writer.start({status_source},
               event::MonotonicTime::from_nanos(1'000'000'000));
  ::bridge::BridgeStatus status{};
  status.mutate_protocol_version_id(protocol_version);
  event::Context ctx{};
  ctx.kind = event::EventKind::SEND;
  ctx.event_time = event::MonotonicTime::from_nanos(1'020'000'000);
  ctx.now = ctx.event_time;
  ctx.publish_time = ctx.event_time;
  writer.send(ctx, 1, std::as_bytes(std::span{&status, 1}), 0, 1);
  writer.flush();
  ASSERT_FALSE(writer.failed()) << writer.error();
}

// REQ-OPS-51: retention reads a session's Driver Station packets only under
// the hardware protocol they were recorded in: the bridge's `/hw/status`
// `protocol_version_id` first, else `manifest.json`. A session whose manifest
// or bridge `/hw/status` records another protocol is unknown (kept like a
// match), not `other` (removed first) from packets decoded in this build's
// layout, even when `manifest.json` records this build's version or omits it;
// conversely, a bridge `/hw/status` matching this build overrides a mismatched
// manifest. An `other` verdict a build of another protocol cached is read
// again, not trusted: here a session the cached line calls `other` whose log
// shows the FMS.
TEST(LauncherSessions, ClassifiesOnlyUnderTheRecordedHardwareProtocol) {
  const std::string root = TempPath("protocol_class");
  std::filesystem::remove_all(root);
  constexpr uint16_t kCurrent = protocol::kProtocolVersion;
  constexpr uint16_t kOther = kCurrent - 1;

  // Manifest-only mismatch (no bridge /hw/status log).
  {
    SessionManifest manifest;
    manifest.session_id = 1;
    manifest.start_wall_ns = 1'001'000'000'000;
    manifest.hardware_protocol_version = kOther;
    const std::string other = root + "/1";
    std::filesystem::create_directories(other);
    std::ofstream{other + "/manifest.json"} << manifest.ToJson();
    WriteDriverStationLog(other, "late", 1, {false});
    EXPECT_EQ(InspectSession(other).verdict, MatchVerdict::kUnknown);
  }

  // Cached `other` verdict without this build's reader identity is ignored.
  {
    const std::string cached = root + "/2";
    WriteSessionManifest(cached, 2, 1002);
    WriteDriverStationLog(cached, "late", 2, {true});
    std::ofstream{cached + "/" + std::string{kSessionClassFile}}
        << "other " << std::filesystem::file_size(cached + "/late.tlog")
        << "\n";
    EXPECT_EQ(InspectSession(cached).verdict, MatchVerdict::kMatch);
  }

  // Bridge `/hw/status` records a mismatched protocol while `manifest.json`
  // records this build's current version: retention must inspect `/hw/status`
  // (not manifest-only) and classify as unknown.
  {
    const std::string bridge_mismatch = root + "/3";
    WriteSessionManifest(bridge_mismatch, 3, 1003);
    WriteDriverStationLog(bridge_mismatch, "late", 3, {false});
    WriteBridgeStatusLog(bridge_mismatch, 3, kOther);
    EXPECT_EQ(InspectSession(bridge_mismatch).verdict, MatchVerdict::kUnknown);
  }

  // Bridge `/hw/status` records a mismatched protocol while `manifest.json`
  // omits `hardware_protocol_version`: still classified as unknown.
  {
    SessionManifest manifest;
    manifest.session_id = 4;
    manifest.start_wall_ns = 1'004'000'000'000;
    manifest.hardware_protocol_version.reset();
    const std::string bridge_mismatch_omitted = root + "/4";
    std::filesystem::create_directories(bridge_mismatch_omitted);
    std::ofstream{bridge_mismatch_omitted + "/manifest.json"}
        << manifest.ToJson();
    WriteDriverStationLog(bridge_mismatch_omitted, "late", 4, {false});
    WriteBridgeStatusLog(bridge_mismatch_omitted, 4, kOther);
    EXPECT_EQ(InspectSession(bridge_mismatch_omitted).verdict,
              MatchVerdict::kUnknown);
  }

  // Positive control: bridge `/hw/status` records this build's protocol while
  // `manifest.json` records another: bridge `/hw/status` takes precedence so
  // Driver Station packets are decoded as `other`.
  {
    SessionManifest manifest;
    manifest.session_id = 5;
    manifest.start_wall_ns = 1'005'000'000'000;
    manifest.hardware_protocol_version = kOther;
    const std::string bridge_current = root + "/5";
    std::filesystem::create_directories(bridge_current);
    std::ofstream{bridge_current + "/manifest.json"} << manifest.ToJson();
    WriteDriverStationLog(bridge_current, "late", 5, {false});
    WriteBridgeStatusLog(bridge_current, 5, kCurrent);
    EXPECT_EQ(InspectSession(bridge_current).verdict, MatchVerdict::kOther);
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

// REQ-DEV-04: --describe-only lints without touching shared memory or running
// session retention, even when the log root already exceeds log_keep_sessions.
TEST(LauncherSessions, DescribeOnlyLeavesSessionsAndSharedMemoryUntouched) {
  auto options = SessionFixture("describe_only_no_side_effects");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  options.config_path = TempPath("describe_only_no_side_effects") + ".toml";
  std::string configuration{kFixtureToml};
  configuration.insert(configuration.find("period_us"),
                       "log_keep_sessions = 1\n");
  std::ofstream{options.config_path} << configuration;
  options.binary_overrides["//example/alpha:node"] =
      WriteStubNode("describe_only_no_side_effects_alpha",
                    DescribingStub("alpha", "//example/alpha:node",
                                   {Sender("/alpha/state")}));
  options.binary_overrides["//example/beta:node"] = WriteStubNode(
      "describe_only_no_side_effects_beta",
      DescribingStub("beta", "//example/beta:node", {Watcher("/alpha/state")}));

  const auto dir = [&root](int id) { return root + "/" + std::to_string(id); };
  const auto now = std::filesystem::file_time_type::clock::now();
  std::map<int, std::map<std::string, std::string>> before;
  for (const int id : {1, 2, 3}) {
    WriteSessionManifest(dir(id), static_cast<uint64_t>(id), 1000 + id,
                         1100 + id);
    WriteTestLog(dir(id) + "/alpha.tlog", "alpha", static_cast<uint64_t>(id),
                 {1'000'000});
  }
  std::filesystem::create_directories(dir(4));
  std::ofstream{dir(4) + "/graph.json"} << "{}\n";
  for (const auto& [id, age] :
       {std::pair{1, 4}, std::pair{2, 3}, std::pair{3, 2}, std::pair{4, 1}}) {
    std::filesystem::last_write_time(dir(id), now - std::chrono::hours{age});
    before[id] = DirectoryContents(dir(id));
  }

  // Isolate into a fresh TALOS_SHM_NAMESPACE when not already under Bazel's
  // per-shard TEST_TMPDIR, and clear any prior segments in the active
  // namespace before running.
  struct ScopedNamespace {
    bool set = false;
    ScopedNamespace() {
      if (std::getenv("TEST_TMPDIR") == nullptr &&
          std::getenv("TALOS_SHM_NAMESPACE") == nullptr) {
        const std::string ns =
            "talos_describe_only_" + std::to_string(::getpid());
        ::setenv("TALOS_SHM_NAMESPACE", ns.c_str(), 1);
        set = true;
      }
    }
    ~ScopedNamespace() {
      if (set) ::unsetenv("TALOS_SHM_NAMESPACE");
    }
  } scoped_ns;

  const std::vector<std::string> shm_topics{"/alpha/state",
                                            "/talos_registry.6"};
  for (const std::string& topic : shm_topics) {
    const std::string shm = ShmObjectName(topic);
    ::shm_unlink(shm.c_str());
    errno = 0;
    const int fd = ::shm_open(shm.c_str(), O_RDONLY, 0);
    ASSERT_EQ(fd, -1) << topic << " (" << shm << ") still exists";
    ASSERT_EQ(errno, ENOENT) << topic << " (" << shm << ")";
  }

  options.session_id = 5;
  options.describe_only = true;
  EXPECT_EQ(Launcher{options}.Run(), 0);
  EXPECT_FALSE(std::filesystem::exists(dir(5) + "/manifest.json"));
  EXPECT_TRUE(std::filesystem::exists(dir(5) + "/graph.json"));

  // Every pre-existing session directory remains intact.
  for (const auto& [id, expected] : before) {
    ASSERT_TRUE(std::filesystem::exists(dir(id)))
        << "session dir " << id << " was removed by --describe-only";
    EXPECT_EQ(DirectoryContents(dir(id)), expected) << "session dir " << id;
  }

  // Neither declared topics nor the registry were created in shared memory.
  for (const std::string& topic : shm_topics) {
    const std::string shm = ShmObjectName(topic);
    errno = 0;
    const int fd = ::shm_open(shm.c_str(), O_RDONLY, 0);
    if (fd >= 0) {
      ::close(fd);
      ::shm_unlink(shm.c_str());
    }
    EXPECT_EQ(fd, -1) << topic << " (" << shm << ") created by --describe-only";
    EXPECT_EQ(errno, ENOENT) << topic << " (" << shm << ")";
  }

  std::filesystem::remove_all(root);
  std::filesystem::remove(options.config_path);
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

// A log root on a filesystem without advisory locks (macOS msdos and exFAT
// volumes: ENOTSUP/EOPNOTSUPP; ENOSYS) cannot hold a session directory's lock.
// The robot still starts: one warning, and the session runs without it,
// protected as before launches locked their directories (by the namespace's
// session lock only); retention removes as it did then. (ENOLCK is tested
// separately in launcher_retention_test.cc: retention refuses all deletions.)
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
    errno = EOPNOTSUPP;
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

// The files a launch writes once and then leaves (manifest.json in a quiet
// session, the graphs, the retention record) age past a /tmp cleaner's limit
// under a long session while session.lock stays fresh; the cleaner would
// delete them, and the session would read as abandoned or unreplayable.
// Each refresh touches them with the lock, whether or not the lock is held;
// one that does not exist is not created.
// REQ-OPS-11
TEST(LauncherSessions, ARefreshTouchesTheSessionsQuietFiles) {
  const std::string root = TempPath("quiet_files");
  std::filesystem::remove_all(root);
  const std::filesystem::path dir = root + "/1";
  std::error_code error;
  auto session = SessionDirectoryLock::Create(
      dir,
      [](const std::filesystem::path& path, std::error_code& made) {
        std::filesystem::create_directories(path, made);
      },
      error);
  ASSERT_TRUE(session.has_value()) << error.message();
  constexpr time_t kAged = 1'000'000;  // 1970: older than any cleaner's age.
  // The writers' names: the manifest's, graph.json (held to launcher.cc by
  // AGraphFileThatCannotBeWrittenRefusesTheLaunch), the retention record's.
  const std::vector<std::string> quiet = {
      std::string{event::session_manifest::kFileName},
      std::string{kSessionQuietFiles[1]}, std::string{kRetentionRecordFile}};
  for (const auto& name : quiet) {
    std::ofstream{dir / name} << "{}";
    const timespec times[2] = {{kAged, 0}, {kAged, 0}};
    ASSERT_EQ(::utimensat(AT_FDCWD, (dir / name).c_str(), times, 0), 0);
  }
  EXPECT_EQ(session->Refresh(), SessionDirectoryLock::Refreshed::kHeld);
  for (const auto& name : quiet) {
    struct stat status {};
    ASSERT_EQ(::stat((dir / name).c_str(), &status), 0) << name;
    EXPECT_NE(status.st_mtime, kAged) << name << " was not touched";
  }
  EXPECT_FALSE(std::filesystem::exists(dir / kSessionQuietFiles[2]));
  session.reset();
  std::filesystem::remove_all(root);
}

// A filesystem that refuses an exclusive flock on a descriptor not open for
// writing, with EBADF, as NFSv4 is reported to (not reproduced here: no NFS
// on the test hosts). Every exclusive lock of a session.lock still takes it:
// the launch's own (Create), its refresh after the file was removed
// (Refresh), and a retention's (TakeIdle).
// REQ-OPS-11
TEST(LauncherSessions, ExclusiveLocksHoldWhereReadOnlyDescriptorsCannot) {
  const std::string root = TempPath("lock_write_access");
  std::filesystem::remove_all(root);
  const std::filesystem::path dir = root + "/1";
  const FlockFunction lock = [](int fd, int operation) {
    if ((operation & LOCK_EX) != 0 &&
        (::fcntl(fd, F_GETFL) & O_ACCMODE) == O_RDONLY) {
      errno = EBADF;
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
  EXPECT_TRUE(session->held());
  std::filesystem::remove(session->path());
  EXPECT_EQ(session->Refresh(lock), SessionDirectoryLock::Refreshed::kRetaken);
  EXPECT_TRUE(SessionDirectoryLock::Held(dir, /*root_takes_no_locks=*/false));
  session.reset();
  bool held = false;
  EXPECT_TRUE(SessionDirectoryLock::TakeIdle(dir, /*root_takes_no_locks=*/false,
                                             held, error, lock)
                  .has_value())
      << error.message();
  EXPECT_FALSE(held);
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
  options.available_bytes = [second, listed](const std::filesystem::path&) {
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

// Fixture TOML with `log_keep_sessions = 1` so pre-seeded finished sessions
// exceed the retention cap and would be pruned if ApplyRetention ran.
std::string FixtureTomlWithRetentionCap() {
  std::string configuration{kFixtureToml};
  configuration.insert(configuration.find("period_us"),
                       "log_keep_sessions = 1\n");
  return configuration;
}

// Seeds two older finished sessions under `root` above `log_keep_sessions = 1`
// and returns each directory's path and file contents.
std::map<std::string, std::map<std::string, std::string>>
SeedSessionsAboveRetentionCap(const std::string& root) {
  std::map<std::string, std::map<std::string, std::string>> seeded;
  const auto now = std::filesystem::file_time_type::clock::now();
  for (const auto& [id, age] : {std::pair{100, 3}, std::pair{101, 2}}) {
    const std::string dir = root + "/" + std::to_string(id);
    WriteSessionManifest(dir, static_cast<uint64_t>(id), 1000 + id, 1100 + id);
    WriteTestLog(dir + "/alpha.tlog", "alpha", static_cast<uint64_t>(id),
                 {1'000'000});
    std::filesystem::last_write_time(dir, now - std::chrono::hours{age});
    seeded[dir] = DirectoryContents(dir);
  }
  return seeded;
}

// `studio_bridge` is a valid subsystem name. A robot's own subsystem of
// that name is an ordinary node: critical, its failed probe is an error that
// refuses the launch, not the Studio bridge's warning that leaves it out of
// the session. With --start-studio-bridge, the launcher's bridge would be a
// second roster member of that name: the launch is refused before retention,
// probes, and spawn naming the key, even with --sim --allow-graph-errors.
// REQ-OPS-30, REQ-OPS-01, REQ-CONTRACT-42
TEST(LauncherSessions, ASubsystemNamedStudioBridgeIsAnOrdinaryNode) {
  auto options = SessionFixture("user_studio_bridge");
  const std::string root = options.output_dir;
  std::filesystem::remove_all(root);
  std::ofstream{options.config_path}
      << FixtureTomlWithRetentionCap()
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

  // Non-describe launch with --sim --allow-graph-errors is still refused
  // before ApplyRetention, probes, and spawn.
  const std::string invoked = root + "_invoked";
  std::filesystem::remove(invoked);
  options.binary_overrides["//example/studio:node"] = WriteStubNode(
      "user_studio_bridge_recorded",
      RecordingStub("studio_bridge", "//example/studio:node", invoked));
  const auto seeded = SeedSessionsAboveRetentionCap(root);
  options.session_id = 3;
  options.describe_only = false;
  options.simulation = true;
  options.remote = "127.0.0.1";
  options.allow_graph_errors = true;
  ::testing::internal::CaptureStderr();
  EXPECT_EQ(Launcher{options}.Run(), 1);
  const std::string allowed_refused = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(allowed_refused.find("[subsystems.studio_bridge]"),
            std::string::npos)
      << allowed_refused;
  EXPECT_EQ(ReadFile(invoked), "") << "the subsystem's binary was run";
  EXPECT_FALSE(std::filesystem::exists(root + "/3"));
  for (const auto& [dir, contents] : seeded) {
    EXPECT_TRUE(std::filesystem::exists(dir)) << "retention pruned " << dir;
    EXPECT_EQ(DirectoryContents(dir), contents) << "retention altered " << dir;
  }
  std::filesystem::remove_all(root);
  std::filesystem::remove(invoked);
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

// The launcher knows the hardware bridge by its roster name: the lint's `hw`
// writer exemption, "always critical" and the bridge's log share all follow
// it. A subsystem keyed `hardware_node` (non-critical, its own binary
// claiming the bridge's name and `/hw/state`) would be a second member of that
// name; it is refused before retention, probes, or spawn, naming the key,
// even with --sim --allow-graph-errors, which would override the lint's `hw`
// writer error.
// REQ-SAFE-28, REQ-CONTRACT-09, REQ-CONTRACT-42
TEST(LauncherSessions, ASubsystemCannotTakeTheHardwareBridgesName) {
  auto options = SessionFixture("subsystem_hardware_node");
  std::filesystem::remove_all(options.output_dir);
  const std::string invoked = options.output_dir + "_invoked";
  std::filesystem::remove(invoked);
  std::ofstream{options.config_path} << FixtureTomlWithRetentionCap()
                                     << "\n[subsystems.hardware_node]\nnode = "
                                        "\"//example/hardware_node:node\"\n"
                                        "period_us = 5000\ncritical = false\n";
  options.binary_overrides["//example/hardware_node:node"] = WriteStubNode(
      "subsystem_hardware_node",
      RecordingStub("hardware_node", "//example/hardware_node:node", invoked,
                    {Sender("/hw/state")}));
  const auto seeded = SeedSessionsAboveRetentionCap(options.output_dir);
  options.simulation = true;
  options.remote = "127.0.0.1";
  options.allow_graph_errors = true;
  ::testing::internal::CaptureStderr();
  EXPECT_EQ(Launcher{options}.Run(), 1);
  const std::string refused = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(refused.find("[subsystems.hardware_node]"), std::string::npos)
      << refused;
  EXPECT_EQ(ReadFile(invoked), "") << "the subsystem's binary was run";
  EXPECT_FALSE(std::filesystem::exists(options.output_dir + "/1"));
  for (const auto& [dir, contents] : seeded) {
    EXPECT_TRUE(std::filesystem::exists(dir)) << "retention pruned " << dir;
    EXPECT_EQ(DirectoryContents(dir), contents) << "retention altered " << dir;
  }
  std::filesystem::remove_all(options.output_dir);
  std::filesystem::remove(invoked);
}

// The sim gateway is known by the roster flag --start-sim-gateway sets, not by
// its name. A robot's subsystem named `sim_gateway`, in a session where the
// launcher adds no gateway, is an ordinary node: started under --sim with
// --log (its event log in the manifest) and --sim, and replayable. The
// launcher's own gateway is still started with --config and no --log, and
// with it such a subsystem is refused before retention, probes, and spawn,
// naming the key, whether or not --allow-graph-errors is passed.
// REQ-OPS-09, REQ-OPS-14, REQ-CONTRACT-42
TEST(LauncherSessions, ASubsystemNamedSimGatewayIsAnOrdinaryNode) {
  struct Outcome {
    int status{-1};
    std::string err;
    std::string record;    // every invocation, --describe included
    std::string launched;  // the last one that was not --describe
    std::string manifest;
    int replay_status{-1};
    std::string replay_err;
    std::string replay_line;  // the invocation --replay-inputs made
    bool retention_untouched{true};
    bool session_dir_created{false};
  };
  const auto run = [](std::string_view tag, bool subsystem, bool gateway,
                      bool allow_graph_errors = false) {
    Outcome out;
    auto options = SessionFixture(tag);
    std::filesystem::remove_all(options.output_dir);
    const std::string invoked = options.output_dir + "_invoked";
    std::filesystem::remove(invoked);
    std::ofstream config{options.config_path};
    config << (subsystem && gateway ? FixtureTomlWithRetentionCap()
                                    : std::string{kFixtureToml});
    if (subsystem) {
      config << "\n[subsystems.sim_gateway]\nnode = "
                "\"//example/sim_gateway:node\"\nperiod_us = 5000\n";
    }
    config.close();
    std::map<std::string, std::map<std::string, std::string>> seeded;
    if (subsystem && gateway) {
      seeded = SeedSessionsAboveRetentionCap(options.output_dir);
    }
    options.binary_overrides["//example/sim_gateway:node"] = WriteStubNode(
        std::string{tag} + "_subsystem",
        RecordingStub("sim_gateway", "//example/sim_gateway:node", invoked));
    // The stock gateway describes its own label, whose rule is its name.
    options.binary_overrides["//example/sim:gateway"] =
        WriteStubNode(std::string{tag} + "_gateway",
                      RecordingStub("sim_gateway",
                                    "//talOS/simulation:sim_gateway", invoked));
    options.simulation = true;
    options.remote = "127.0.0.1";
    options.start_sim_gateway = gateway;
    options.allow_graph_errors = allow_graph_errors;
    ::testing::internal::CaptureStderr();
    out.status = Launcher{options}.Run();
    out.err = ::testing::internal::GetCapturedStderr();
    out.record = ReadFile(invoked);
    for (const auto& [dir, contents] : seeded) {
      if (!std::filesystem::exists(dir) || DirectoryContents(dir) != contents) {
        out.retention_untouched = false;
      }
    }
    out.session_dir_created =
        std::filesystem::exists(options.output_dir + "/1");
    std::istringstream lines{out.record};
    for (std::string line; std::getline(lines, line);) {
      if (line.find("--describe") == std::string::npos) out.launched = line;
    }
    const std::string session = options.output_dir;
    out.manifest = ReadFile(session + "/1/manifest.json");
    if (out.status == 0 && subsystem) {
      options.replay_inputs = session + "/1";
      options.only = "sim_gateway";
      options.output_dir = session + "_replay";
      ::testing::internal::CaptureStderr();
      out.replay_status = Launcher{options}.Run();
      out.replay_err = ::testing::internal::GetCapturedStderr();
      const std::string record = ReadFile(invoked);
      const std::string replayed =
          "sim_gateway --replay-inputs " + options.replay_inputs + " ";
      if (const auto at = record.find(replayed); at != std::string::npos) {
        out.replay_line = record.substr(at, record.find('\n', at) - at);
      }
      std::filesystem::remove_all(options.output_dir);
    }
    std::filesystem::remove_all(session);
    std::filesystem::remove(invoked);
    return out;
  };

  const Outcome ordinary = run("subsystem_sim_gateway", true, false);
  EXPECT_EQ(ordinary.status, 0) << ordinary.err;
  EXPECT_NE(ordinary.launched.find(" --log "), std::string::npos)
      << ordinary.launched;
  EXPECT_NE(ordinary.launched.find(" --sim"), std::string::npos)
      << ordinary.launched;
  EXPECT_NE(ordinary.manifest.find("/sim_gateway.tlog\""), std::string::npos)
      << ordinary.manifest;
  // Replayable: --only sim_gateway runs it on the recorded session.
  EXPECT_EQ(ordinary.replay_status, 0) << ordinary.replay_err;
  EXPECT_NE(ordinary.replay_line.find(" --log "), std::string::npos)
      << "no replay run of sim_gateway:\n"
      << ordinary.replay_err;

  const Outcome own = run("own_sim_gateway", false, true);
  EXPECT_EQ(own.status, 0) << own.err;
  EXPECT_EQ(own.launched.rfind("sim_gateway --config ", 0), 0u) << own.launched;
  EXPECT_EQ(own.launched.find("--log"), std::string::npos) << own.launched;
  EXPECT_EQ(own.manifest.find("sim_gateway.tlog"), std::string::npos)
      << own.manifest;

  for (const bool allow : {false, true}) {
    const Outcome both =
        run(allow ? "both_sim_gateway_allowed" : "both_sim_gateway", true, true,
            allow);
    EXPECT_EQ(both.status, 1) << "allow_graph_errors=" << allow;
    EXPECT_NE(both.err.find("[subsystems.sim_gateway]"), std::string::npos)
        << both.err;
    EXPECT_EQ(both.record, "") << "a member was run or probed";
    EXPECT_TRUE(both.retention_untouched)
        << "retention pruned or altered pre-seeded sessions";
    EXPECT_FALSE(both.session_dir_created);
  }
}

// REQ-OPS-08
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
  const auto deadline = std::chrono::steady_clock::now() + kHang;
  bool running = false;
  do {
    const std::string before = ReadFile(path);
    running = before.find("\"name\": \"alpha\"") != std::string::npos &&
              before.find("\n      \"pid\": -1") == std::string::npos;
    if (running) break;
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  } while (std::chrono::steady_clock::now() < deadline);
  ::kill(child, SIGKILL);
  int status = 0;
  ::waitpid(child, &status, 0);
  // Killed while alpha ran, not before it started: the deadline only bounds a
  // hang.
  EXPECT_TRUE(running) << "the manifest never showed alpha running";
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

// Launcher::Run in a child of the test process, which starts with SIGINT at
// its default action as a launcher process does: a stop the launcher does not
// handle then ends the child by the signal, not the test.
pid_t ForkLauncher(const LauncherOptions& options) {
  const pid_t child = ::fork();
  if (child == 0) {
    ::signal(SIGINT, SIG_DFL);
    ::_exit(Launcher{options}.Run());
  }
  return child;
}

// Whether the segment of `topic` is still in this test's namespace; unlinks
// it when it is, so a failing case leaves nothing behind.
bool SegmentLeft(const std::string& topic) {
  const std::string name = ShmObjectName(topic);
  const int fd = ::shm_open(name.c_str(), O_RDONLY, 0);
  if (fd < 0) return false;
  ::close(fd);
  ::shm_unlink(name.c_str());
  return true;
}

// A stub that answers --describe with `sources` and whose every run appends
// `node` to `record`.
std::string DescribeThenRecord(
    std::string_view node, std::string_view target,
    const std::vector<introspect::naming::SourceShape>& sources,
    const std::string& record) {
  std::string stub = DescribingStub(node, target, sources);
  const std::string_view ending = "exit 0\n";
  stub.resize(stub.size() - ending.size());
  return stub + "echo " + std::string{node} + " >> '" + record + "'\nexit 0\n";
}

// A stop (SIGINT, as Ctrl-C sends it) that arrives once the launcher has
// created the session's topic segments and before it has spawned a member
// ends the session in order: no member is spawned, the segments are
// unlinked, the manifest records the session's end and that the stop came
// before every member started, and the launcher exits 0 as for any requested
// stop. The SIGINT is raised on the launcher's thread at the seam, so it is
// handled before the seam returns. Seen on the Mac mini (2026-11-13, SIGINT
// 0.3 s after starting the 2026 simulation): the stop handlers were installed
// only after the manifest, and the signal's default action ended the
// launcher there, its manifest unended and its segments left behind.
// REQ-OPS-03, REQ-OPS-06
TEST(LauncherSessions, AStopOnceSegmentsExistEndsTheSessionBeforeAnySpawn) {
  struct SegmentSnapshot {
    std::uint64_t lifetime{0};
    std::uint64_t schema_hash{0};
    std::uint64_t message_bytes{0};
    std::uint64_t message_alignment{0};
    std::uint64_t slots{0};
    char schema_name[talos::schema::kMaxSchemaName + 1]{};
    bool had_record_before_spawn{false};
  };
  const auto expected = Sender("/alpha/state");
  const auto snapshot_segment = [&](const std::string& rec_path) {
    SegmentSnapshot snap{};
    snap.had_record_before_spawn = !ReadFile(rec_path).empty();
    SharedMemoryPtr segment{"/alpha/state", sizeof(RTMSHeader)};
    const auto* header = static_cast<const RTMSHeader*>(segment.ptr());
    snap.lifetime = header->lifetime.load(std::memory_order_acquire);
    snap.schema_hash = header->schema_hash;
    snap.message_bytes = header->message_bytes;
    snap.message_alignment = header->message_alignment;
    snap.slots = header->slots;
    std::memcpy(snap.schema_name, header->schema_name,
                sizeof(snap.schema_name));
    return snap;
  };
  const auto check_snapshot = [&](const SegmentSnapshot& snap) {
    EXPECT_FALSE(snap.had_record_before_spawn)
        << "a member ran before session_segments_created";
    EXPECT_NE(snap.lifetime & kLifetimeSessionOwned, 0u);
    EXPECT_EQ(std::string_view{snap.schema_name}, expected.schema);
    EXPECT_EQ(snap.schema_hash, expected.schema_hash);
    EXPECT_EQ(snap.message_bytes, expected.message_bytes);
    EXPECT_EQ(snap.message_alignment, expected.alignment);
    EXPECT_EQ(snap.slots, expected.slots);
  };

  // Case 1: stop arriving once segments exist (before any spawn) inspects the
  // session-owned segment header and unlinks it on stop.
  {
    const std::string tag = "stop_after_segments";
    auto options = SessionFixture(tag);
    std::filesystem::remove_all(options.output_dir);
    const std::string record = TempPath(tag + "_runs");
    ::remove(record.c_str());
    // Only bounds a launch that missed the stop.
    options.duration_s = static_cast<int>(kHang.count());
    options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
        tag + "_hardware",
        DescribeThenRecord("hardware_node", "//talOS/bridge:hardware_node", {},
                           record));
    options.binary_overrides["//example/alpha:node"] = WriteStubNode(
        tag + "_alpha", DescribeThenRecord("alpha", "//example/alpha:node",
                                           {expected}, record));
    options.binary_overrides["//example/beta:node"] = WriteStubNode(
        tag + "_beta", DescribeThenRecord("beta", "//example/beta:node",
                                          {Watcher("/alpha/state")}, record));
    int snap_pipe[2]{-1, -1};
    ASSERT_EQ(::pipe(snap_pipe), 0);
    options.session_segments_created = [&, write_fd = snap_pipe[1]] {
      const SegmentSnapshot snap = snapshot_segment(record);
      [[maybe_unused]] const ssize_t n = ::write(write_fd, &snap, sizeof(snap));
      ::close(write_fd);
      ::raise(SIGINT);
    };
    const pid_t launcher = ForkLauncher(options);
    ASSERT_GE(launcher, 0);
    ::close(snap_pipe[1]);
    SegmentSnapshot snap{};
    const ssize_t got = ::read(snap_pipe[0], &snap, sizeof(snap));
    ::close(snap_pipe[0]);
    int status = 0;
    ASSERT_EQ(::waitpid(launcher, &status, 0), launcher);
    const bool left = SegmentLeft("/alpha/state");
    ASSERT_EQ(got, static_cast<ssize_t>(sizeof(snap)))
        << "session_segments_created did not run";
    check_snapshot(snap);
    EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
        << "the launcher ended with " << ExitReason(status);
    EXPECT_EQ(ReadFile(record), "") << "members spawned after the stop";
    EXPECT_FALSE(left) << "the session's /alpha/state segment was not unlinked";
    const std::string manifest =
        ReadFile(options.output_dir + "/1/manifest.json");
    ASSERT_FALSE(manifest.empty()) << "no manifest.json";
    EXPECT_EQ(manifest.find("\n  \"end_wall_ns\": 0,"), std::string::npos)
        << manifest;
    EXPECT_NE(manifest.find("\n  \"stopped_before_start\": true,"),
              std::string::npos)
        << manifest;
    std::filesystem::remove_all(options.output_dir);
    ::remove(record.c_str());
  }

  // Case 2: normal session end also creates /alpha/state matching the probe
  // before any spawn and unlinks it once the session returns cleanly.
  {
    const std::string tag = "normal_end_segments";
    auto options = SessionFixture(tag);
    std::filesystem::remove_all(options.output_dir);
    const std::string record = TempPath(tag + "_runs");
    ::remove(record.c_str());
    options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
        tag + "_hardware",
        DescribeThenRecord("hardware_node", "//talOS/bridge:hardware_node", {},
                           record));
    options.binary_overrides["//example/alpha:node"] = WriteStubNode(
        tag + "_alpha", DescribeThenRecord("alpha", "//example/alpha:node",
                                           {expected}, record));
    options.binary_overrides["//example/beta:node"] = WriteStubNode(
        tag + "_beta", DescribeThenRecord("beta", "//example/beta:node",
                                          {Watcher("/alpha/state")}, record));
    bool saw_segments = false;
    SegmentSnapshot snap{};
    options.session_segments_created = [&] {
      snap = snapshot_segment(record);
      saw_segments = true;
    };
    EXPECT_EQ(Launcher{options}.Run(), 0);
    const bool left = SegmentLeft("/alpha/state");
    ASSERT_TRUE(saw_segments);
    check_snapshot(snap);
    EXPECT_FALSE(left)
        << "the session's /alpha/state segment survived normal end";
    EXPECT_NE(ReadFile(record).find("alpha"), std::string::npos);
    std::filesystem::remove_all(options.output_dir);
    ::remove(record.c_str());
  }
}

// A stop (SIGINT) that arrives between two spawns: the members already
// started are stopped as every stop stops them (the stop's SIGINT, which
// ends these sleeping stubs: run reason `stop`, exit_signal 2), no member
// after them is spawned, the manifest ends with `stopped_before_start` and
// no run for the unstarted, the segments are unlinked, and the launcher
// exits 0: neither the stopped runs nor the unstarted members count against
// the status. The SIGINT is raised on the launcher's thread right after the
// second spawn, so it is handled before the third member's turn. Each stub
// sleeps by exec, so the stop's signal never finds a shell waiting on a
// child.
// REQ-OPS-06
TEST(LauncherSessions, AStopBetweenTwoSpawnsStopsTheStartedAndStartsNoOther) {
  const std::string tag = "stop_between_spawns";
  auto options = SessionFixture(tag);
  std::filesystem::remove_all(options.output_dir);
  const std::string spawned = TempPath(tag + "_spawned");
  ::remove(spawned.c_str());
  // Only bounds a launch that missed the stop; each member outsleeps it.
  options.duration_s = static_cast<int>(kHang.count());
  const int sleep_s = 3 * options.duration_s;
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
      tag + "_hardware",
      NodeDescribeThenSleep("hardware_node", "//talOS/bridge:hardware_node", {},
                            sleep_s));
  options.binary_overrides["//example/alpha:node"] = WriteStubNode(
      tag + "_alpha", NodeDescribeThenSleep("alpha", "//example/alpha:node",
                                            {Sender("/alpha/state")}, sleep_s));
  options.binary_overrides["//example/beta:node"] = WriteStubNode(
      tag + "_beta", NodeDescribeThenSleep("beta", "//example/beta:node",
                                           {Watcher("/alpha/state")}, sleep_s));
  options.member_spawned = [spawned,
                            count = 0](const std::string& member) mutable {
    std::ofstream{spawned, std::ios::app} << member << "\n";
    if (++count == 2) ::raise(SIGINT);
  };
  const pid_t launcher = ForkLauncher(options);
  ASSERT_GE(launcher, 0);
  int status = 0;
  ASSERT_EQ(::waitpid(launcher, &status, 0), launcher);
  const bool left = SegmentLeft("/alpha/state");
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << "the launcher ended with " << ExitReason(status);
  EXPECT_FALSE(left) << "the session's /alpha/state segment was not unlinked";
  std::vector<std::string> started;
  {
    std::istringstream lines{ReadFile(spawned)};
    for (std::string line; std::getline(lines, line);) started.push_back(line);
  }
  EXPECT_EQ(started.size(), 2u) << "members spawned: " << ReadFile(spawned);
  const std::string manifest =
      ReadFile(options.output_dir + "/1/manifest.json");
  ASSERT_FALSE(manifest.empty()) << "no manifest.json";
  EXPECT_EQ(manifest.find("\n  \"end_wall_ns\": 0,"), std::string::npos)
      << manifest;
  EXPECT_NE(manifest.find("\n  \"stopped_before_start\": true,"),
            std::string::npos)
      << manifest;
  // `name`'s runs: the text between its "runs": [ and the closing ].
  const auto runs_of = [&manifest](const std::string& name) {
    const auto at = manifest.find("\"name\": \"" + name + "\"");
    const auto runs = manifest.find("\"runs\": [", at);
    if (at == std::string::npos || runs == std::string::npos)
      return std::string{"(no entry)"};
    return manifest.substr(runs, manifest.find(']', runs) + 1 - runs);
  };
  int unstarted = 0;
  for (const std::string name : {"hardware_node", "alpha", "beta"}) {
    const std::string runs = runs_of(name);
    if (std::find(started.begin(), started.end(), name) == started.end()) {
      ++unstarted;
      EXPECT_EQ(runs, "\"runs\": []") << name << " was never started";
      continue;
    }
    // One run, ended by the stop's own SIGINT.
    EXPECT_EQ(runs.find("{\"pid\": "), runs.rfind("{\"pid\": ")) << runs;
    EXPECT_NE(runs.find("\"exit_signal\": " + std::to_string(SIGINT) +
                        ", \"reason\": \"stop\""),
              std::string::npos)
        << name << ": " << runs;
  }
  EXPECT_EQ(unstarted, 1) << manifest;
  std::filesystem::remove_all(options.output_dir);
  ::remove(spawned.c_str());
}

// A launch that starts the roster takes SIGINT, SIGTERM and SIGHUP for its
// stop; once Run has returned each is what it was before, so a later
// --describe-only or --replay-inputs run in the same process gets its
// caller's action (README "Two phases"), not a handler that only sets a flag
// nobody reads. Each starts with a handler of the test's own, which a reset
// to the default action would not put back. No signal is raised: the test
// process's default action would end it.
// REQ-OPS-06
TEST(LauncherSessions, ARosterRunPutsBackTheStopSignalDispositions) {
  auto options = SessionFixture("signal_restore");
  std::filesystem::remove_all(options.output_dir);
  constexpr std::array kStopSignals{SIGINT, SIGTERM, SIGHUP};
  struct sigaction own {};
  own.sa_handler = [](int) {};
  sigemptyset(&own.sa_mask);
  std::array<struct sigaction, kStopSignals.size()> before{};
  for (std::size_t i = 0; i < kStopSignals.size(); ++i)
    ASSERT_EQ(::sigaction(kStopSignals[i], &own, &before[i]), 0);
  const int status = Launcher{options}.Run();
  std::array<struct sigaction, kStopSignals.size()> after{};
  for (std::size_t i = 0; i < kStopSignals.size(); ++i)
    ::sigaction(kStopSignals[i], &before[i], &after[i]);
  EXPECT_EQ(status, 0);
  for (std::size_t i = 0; i < kStopSignals.size(); ++i) {
    EXPECT_EQ(after[i].sa_handler, own.sa_handler)
        << "signal " << SignalText(kStopSignals[i])
        << " kept the launcher's handler after Run returned";
  }
  std::filesystem::remove_all(options.output_dir);
}

// A stop that arrives while the roster is being described ends the launch
// without waiting for a probe that has not answered: the probe is killed
// rather than given the rest of its describe deadline, nothing of a session
// is written after it (no graph.json, no manifest, no topic segments), both
// session locks are released, and the launcher exits 0.
// REQ-OPS-06
TEST(LauncherSessions, AStopWhileProbingKillsTheProbeAndStartsNothing) {
  const std::string tag = "stop_while_probing";
  auto options = SessionFixture(tag);
  std::filesystem::remove_all(options.output_dir);
  // Ensure neither topic segment exists before the launch starts.
  for (const std::string topic : {"/alpha/state", "/beta/state"}) {
    ::shm_unlink(ShmObjectName(topic).c_str());
  }
  // The deadline a stop must not wait out.
  const std::chrono::milliseconds describe_deadline{std::chrono::seconds{30}};
  options.describe_timeout_ms = static_cast<int>(describe_deadline.count());
  const std::string probe = TempPath(tag + "_probe_pid");
  ::remove(probe.c_str());
  // exec keeps the pid it wrote: the probe the launcher must kill.
  options.binary_overrides["//example/alpha:node"] = WriteStubNode(
      tag + "_alpha", "#!/bin/sh\necho $$ > '" + probe + ".tmp' && mv '" +
                          probe + ".tmp' '" + probe + "'\nexec sleep 60\n");
  // Beta completes its probe with a real topic; a stop before segment creation
  // must still create no segment for it.
  options.binary_overrides["//example/beta:node"] = WriteStubNode(
      tag + "_beta",
      DescribingStub("beta", "//example/beta:node", {Sender("/beta/state")}));
  const pid_t launcher = ForkLauncher(options);
  ASSERT_GE(launcher, 0);
  pid_t probe_pid = 0;
  for (const auto give_up = std::chrono::steady_clock::now() + kHang;
       probe_pid <= 0 && std::chrono::steady_clock::now() < give_up;
       std::this_thread::sleep_for(std::chrono::milliseconds{2})) {
    const std::string text = ReadFile(probe);
    pid_t read = 0;
    if (std::from_chars(text.data(), text.data() + text.size(), read).ec ==
        std::errc{})
      probe_pid = read;
  }
  const auto stopped_at = std::chrono::steady_clock::now();
  ::kill(launcher, SIGINT);
  int status = 0;
  ASSERT_EQ(::waitpid(launcher, &status, 0), launcher);
  const auto stopping = std::chrono::steady_clock::now() - stopped_at;
  ASSERT_GT(probe_pid, 0) << "alpha's probe never started";
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << "the launcher ended with " << ExitReason(status);
  // Well inside the deadline: without the stop the launcher ends about one
  // deadline after the probe's spawn, which is a little before stopped_at.
  EXPECT_LT(stopping, describe_deadline / 2)
      << "the stop waited for the probe's describe deadline";
  errno = 0;
  EXPECT_TRUE(::kill(probe_pid, 0) != 0 && errno == ESRCH)
      << "alpha's probe (pid " << probe_pid << ") outlived the launcher";
  const std::string session = options.output_dir + "/1";
  EXPECT_FALSE(std::filesystem::exists(session + "/graph.json"));
  EXPECT_FALSE(std::filesystem::exists(session + "/manifest.json"));
  for (const std::string topic : {"/alpha/state", "/beta/state"}) {
    const std::string shm = ShmObjectName(topic);
    errno = 0;
    const int fd = ::shm_open(shm.c_str(), O_RDONLY, 0);
    if (fd >= 0) {
      ::close(fd);
      ::shm_unlink(shm.c_str());
    }
    EXPECT_LT(fd, 0) << topic << " segment was created before stop";
    EXPECT_EQ(errno, ENOENT) << topic << ": " << std::strerror(errno);
  }
  EXPECT_FALSE(SessionDirectoryLock::Held(session, false))
      << "session.lock remained held after stop while probing";
  const std::string namespace_lock =
      introspect::LockFilePath(ShmObjectName(kSessionLockName));
  const bool namespaced =
      ShmObjectName(introspect::kRegistryPath).starts_with("/~");
  if (namespaced) {
    EXPECT_FALSE(std::filesystem::exists(namespace_lock))
        << "namespace session lock file " << namespace_lock
        << " survived stop while probing";
  }
  std::string lock_error;
  {
    const auto reacquired = SessionLock::Acquire(lock_error);
    EXPECT_TRUE(reacquired.has_value())
        << "namespace session lock remained held: " << lock_error;
  }
  if (namespaced) ::unlink(namespace_lock.c_str());
  std::filesystem::remove_all(options.output_dir);
  ::remove(probe.c_str());
}

// --sim commissions in memory only with a loopback remote, so a physical
// effective remote, from --remote or from the config's controller_address
// when no --remote is given, is refused before any member is probed or
// spawned and before any session directory exists; every member's stub
// records each invocation, --describe or not.
// REQ-SAFE-20
TEST(LauncherRemoteTest, SimulationRefusesPhysicalEffectiveRemote) {
  struct Row {
    const char* name;
    const char* remote_flag;
    const char* controller_address;
  };
  for (const Row row : {Row{"--remote", "10.12.34.2", nullptr},
                        Row{"controller_address", "", "10.56.87.2"}}) {
    SCOPED_TRACE(row.name);
    const std::string invoked = TempPath("sim_guard_invoked");
    ::remove(invoked.c_str());
    GraphFixture fixture{
        "sim_guard", RecordingStub("alpha", "//example/alpha:node", invoked),
        RecordingStub("beta", "//example/beta:node", invoked)};
    fixture.options.binary_overrides["//talOS/bridge:hardware_node"] =
        WriteStubNode("sim_guard_hardware",
                      RecordingStub("hardware_node",
                                    "//talOS/bridge:hardware_node", invoked));
    if (row.controller_address != nullptr) {
      std::string toml{kFixtureToml};
      const std::string header = "[robot]\n";
      toml.insert(toml.find(header) + header.size(),
                  std::string{"controller_address = \""} +
                      row.controller_address + "\"\n");
      fixture.options.config_path = TempPath("sim_guard_config") + ".toml";
      std::ofstream{fixture.options.config_path} << toml;
    }
    fixture.options.simulation = true;
    fixture.options.remote = row.remote_flag;

    EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
    EXPECT_FALSE(std::filesystem::exists(invoked)) << ReadFile(invoked);
    EXPECT_FALSE(std::filesystem::exists(fixture.out_dir)) << fixture.out_dir;
    ::remove(invoked.c_str());
    if (row.controller_address != nullptr) {
      ::remove(fixture.options.config_path.c_str());
    }
  }
}

// REQ-OPS-25, TODO.md DEPLOY-7: `--sim --remote 127.0.0.1` on a config whose
// controller_address is the RoboRIO's. hardware_node refuses --sim without
// the loopback remote, so its --describe probe must get the effective
// --remote the launch gives it, not fall back to the config's address. The
// stub refuses as the bridge does: before the fix it was probed without
// --remote, did not answer, and the critical bridge refused the graph.
TEST(LauncherRemoteTest, BridgeProbeGetsTheEffectiveRemote) {
  auto options = SessionFixture("probe_remote");
  {
    std::string toml{kFixtureToml};
    const std::string header = "[robot]\n";
    toml.insert(toml.find(header) + header.size(),
                "controller_address = \"10.56.87.2\"\n");
    std::ofstream{options.config_path} << toml;
  }
  const std::string stub =
      DescribingStub("hardware_node", "//talOS/bridge:hardware_node", {});
  const std::string shebang = "#!/bin/sh\n";
  options.binary_overrides["//talOS/bridge:hardware_node"] = WriteStubNode(
      "probe_remote_hardware",
      shebang +
          "remote=\"\"\nprev=\"\"\n"
          "for arg in \"$@\"; do\n"
          "  if [ \"$prev\" = \"--remote\" ]; then remote=\"$arg\"; fi\n"
          "  prev=\"$arg\"\n"
          "done\n"
          "if [ \"$remote\" != \"127.0.0.1\" ]; then\n"
          "  echo \"--sim requires --remote 127.0.0.1, got '$remote'\" >&2\n"
          "  exit 1\n"
          "fi\n" +
          stub.substr(shebang.size()));
  options.simulation = true;
  options.remote = "127.0.0.1";
  options.describe_only = true;
  EXPECT_EQ(Launcher{options}.Run(), 0);
  const std::string graph =
      ReadFile(options.output_dir + "/" + std::to_string(options.session_id) +
               "/graph.json");
  EXPECT_EQ(graph.find("\"described\": false"), std::string::npos) << graph;
  std::filesystem::remove_all(options.output_dir);
}

// REQ-OPS-25, REQ-RT-22: `--local-address` resolves over `[robot]
// local_address` (empty keeps wildcard `0.0.0.0`), malformed IPv4 and
// non-loopback `--sim` local addresses are refused before probing or spawning,
// and the effective address is forwarded to `hardware_node` as
// `--local-address <ip>` and logged on startup.
TEST(LauncherRemoteTest, LocalAddressResolvesAndForwardsToBridge) {
  {
    const auto unset = config::ParseRobotConfigString(kFixtureToml);
    LauncherOptions options;
    EXPECT_EQ(EffectiveLocalAddress(unset, options), "");
    const auto unset_args = ControllerArgs(unset, options);
    EXPECT_EQ(
        std::find(unset_args.begin(), unset_args.end(), "--local-address"),
        unset_args.end());

    std::string with_local_toml{kFixtureToml};
    const std::string header = "[robot]\n";
    with_local_toml.insert(with_local_toml.find(header) + header.size(),
                           "local_address = \"10.56.87.5\"\n");
    const auto configured = config::ParseRobotConfigString(with_local_toml);
    EXPECT_EQ(EffectiveLocalAddress(configured, options), "10.56.87.5");
    const auto config_args = ControllerArgs(configured, options);
    const auto config_flag =
        std::find(config_args.begin(), config_args.end(), "--local-address");
    ASSERT_NE(config_flag, config_args.end());
    ASSERT_NE(std::next(config_flag), config_args.end());
    EXPECT_EQ(*std::next(config_flag), "10.56.87.5");

    options.local_address = "10.0.0.37";
    EXPECT_EQ(EffectiveLocalAddress(configured, options), "10.0.0.37");
    const auto cli_args = ControllerArgs(configured, options);
    const auto cli_flag =
        std::find(cli_args.begin(), cli_args.end(), "--local-address");
    ASSERT_NE(cli_flag, cli_args.end());
    ASSERT_NE(std::next(cli_flag), cli_args.end());
    EXPECT_EQ(*std::next(cli_flag), "10.0.0.37");
  }

  // Invalid `--local-address` values are refused before any probe or spawn.
  for (const char* bad : {"not-an-ip", "10.0.0.999", "010.0.0.1"}) {
    SCOPED_TRACE(bad);
    const std::string invoked = TempPath("bad_local_addr_invoked");
    ::remove(invoked.c_str());
    GraphFixture fixture{
        "bad_local_addr",
        RecordingStub("alpha", "//example/alpha:node", invoked),
        RecordingStub("beta", "//example/beta:node", invoked)};
    fixture.options.binary_overrides["//talOS/bridge:hardware_node"] =
        WriteStubNode("bad_local_addr_hardware",
                      RecordingStub("hardware_node",
                                    "//talOS/bridge:hardware_node", invoked));
    fixture.options.local_address = bad;
    EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
    EXPECT_FALSE(std::filesystem::exists(invoked)) << ReadFile(invoked);
    EXPECT_FALSE(std::filesystem::exists(fixture.out_dir)) << fixture.out_dir;
    ::remove(invoked.c_str());
  }

  // `--sim` refuses a non-loopback local address (from CLI or config) before
  // any probe or spawn.
  struct SimLocalRow {
    const char* name;
    const char* cli_local;
    const char* config_local;
  };
  for (const SimLocalRow row :
       {SimLocalRow{"--local-address", "10.56.87.5", nullptr},
        SimLocalRow{"config local_address", "", "10.56.87.5"}}) {
    SCOPED_TRACE(row.name);
    const std::string invoked = TempPath("sim_local_guard_invoked");
    ::remove(invoked.c_str());
    GraphFixture fixture{
        "sim_local_guard",
        RecordingStub("alpha", "//example/alpha:node", invoked),
        RecordingStub("beta", "//example/beta:node", invoked)};
    fixture.options.binary_overrides["//talOS/bridge:hardware_node"] =
        WriteStubNode("sim_local_guard_hardware",
                      RecordingStub("hardware_node",
                                    "//talOS/bridge:hardware_node", invoked));
    if (row.config_local != nullptr) {
      std::string toml{kFixtureToml};
      const std::string header = "[robot]\n";
      toml.insert(
          toml.find(header) + header.size(),
          std::string{"local_address = \""} + row.config_local + "\"\n");
      fixture.options.config_path =
          TempPath("sim_local_guard_config") + ".toml";
      std::ofstream{fixture.options.config_path} << toml;
    }
    fixture.options.simulation = true;
    fixture.options.local_address = row.cli_local;
    EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
    EXPECT_FALSE(std::filesystem::exists(invoked)) << ReadFile(invoked);
    EXPECT_FALSE(std::filesystem::exists(fixture.out_dir)) << fixture.out_dir;
    ::remove(invoked.c_str());
    if (row.config_local != nullptr) {
      ::remove(fixture.options.config_path.c_str());
    }
  }

  // End-to-end launch: verify both probe and runtime receive the effective
  // `--remote` and `--local-address` (CLI overriding `[robot]
  // controller_address` and `[robot] local_address`, including allowing `--sim`
  // with `--remote 127.0.0.1 --local-address 127.0.0.1`) and that `talos.log`
  // prints `(local <ip>:<port>)` when bound vs `(local port <port>)` when
  // unset.
  const std::string args_file = TempPath("local_addr_bridge_args");
  const auto make_recording_bridge = [&](const std::string& stub_name) {
    const std::string base =
        DescribingStub("hardware_node", "//talOS/bridge:hardware_node", {});
    const std::string shebang = "#!/bin/sh\n";
    return WriteStubNode(stub_name, shebang + "printf '%s\\n' \"$*\" >> '" +
                                        args_file + "'\n" +
                                        base.substr(shebang.size()));
  };
  const auto split_invocations = [](const std::string& recorded,
                                    std::vector<std::string>& probe_lines,
                                    std::vector<std::string>& runtime_lines) {
    probe_lines.clear();
    runtime_lines.clear();
    std::istringstream lines{recorded};
    for (std::string line; std::getline(lines, line);) {
      if (line.empty()) continue;
      if (line.find("--describe") != std::string::npos) {
        probe_lines.push_back(std::move(line));
      } else {
        runtime_lines.push_back(std::move(line));
      }
    }
  };

  {
    ::remove(args_file.c_str());
    auto options = SessionFixture("local_addr_bound");
    std::filesystem::remove_all(options.output_dir);
    {
      std::string toml{kFixtureToml};
      const std::string header = "[robot]\n";
      toml.insert(toml.find(header) + header.size(),
                  "controller_address = \"10.56.87.2\"\n"
                  "local_address = \"10.56.87.5\"\n");
      options.config_path = TempPath("local_addr_bound_config") + ".toml";
      std::ofstream{options.config_path} << toml;
    }
    options.binary_overrides["//talOS/bridge:hardware_node"] =
        make_recording_bridge("local_addr_bound_hw");
    options.simulation = true;
    options.remote = "127.0.0.1";
    options.local_address = "127.0.0.1";
    ASSERT_EQ(Launcher{options}.Run(), 0);
    const std::string recorded = ReadFile(args_file);
    std::vector<std::string> probe_lines;
    std::vector<std::string> runtime_lines;
    split_invocations(recorded, probe_lines, runtime_lines);
    ASSERT_EQ(probe_lines.size(), 1u) << recorded;
    ASSERT_EQ(runtime_lines.size(), 1u) << recorded;
    for (const std::string& invocation : {probe_lines[0], runtime_lines[0]}) {
      EXPECT_NE(invocation.find("--remote 127.0.0.1"), std::string::npos)
          << invocation;
      EXPECT_NE(invocation.find("--local-address 127.0.0.1"), std::string::npos)
          << invocation;
      EXPECT_EQ(invocation.find("10.56.87.2"), std::string::npos) << invocation;
      EXPECT_EQ(invocation.find("10.56.87.5"), std::string::npos) << invocation;
    }
    const std::string talos_log = ReadFile(options.output_dir + "/1/talos.log");
    EXPECT_NE(
        talos_log.find(
            "[launcher] Controller 127.0.0.1:5802 (local 127.0.0.1:5803)"),
        std::string::npos)
        << talos_log;
    std::filesystem::remove_all(options.output_dir);
    ::remove(options.config_path.c_str());
    ::remove(args_file.c_str());
  }

  {
    ::remove(args_file.c_str());
    auto options = SessionFixture("local_addr_unset");
    std::filesystem::remove_all(options.output_dir);
    options.binary_overrides["//talOS/bridge:hardware_node"] =
        make_recording_bridge("local_addr_unset_hw");
    ASSERT_EQ(Launcher{options}.Run(), 0);
    const std::string recorded = ReadFile(args_file);
    std::vector<std::string> probe_lines;
    std::vector<std::string> runtime_lines;
    split_invocations(recorded, probe_lines, runtime_lines);
    ASSERT_EQ(probe_lines.size(), 1u) << recorded;
    ASSERT_EQ(runtime_lines.size(), 1u) << recorded;
    for (const std::string& invocation : {probe_lines[0], runtime_lines[0]}) {
      EXPECT_NE(invocation.find("--remote 127.0.0.1"), std::string::npos)
          << invocation;
      EXPECT_EQ(invocation.find("--local-address"), std::string::npos)
          << invocation;
    }
    const std::string talos_log = ReadFile(options.output_dir + "/1/talos.log");
    EXPECT_NE(talos_log.find(
                  "[launcher] Controller 127.0.0.1:5802 (local port 5803)"),
              std::string::npos)
        << talos_log;
    std::filesystem::remove_all(options.output_dir);
    ::remove(args_file.c_str());
  }
}

// REQ-OPS-25: `--describe-only` emits a non-fatal warning when `local_address`
// is unset on a non-loopback, non-`--sim` target, and emits none when
// `local_address` is set (config or CLI), on `127.0.0.1`, or under `--sim`.
TEST(LauncherRemoteTest, DescribeOnlyWarnsWhenLocalAddressUnsetOnNonLoopback) {
  struct Row {
    const char* name;
    const char* controller_address;
    const char* config_local_address;
    const char* cli_remote;
    const char* cli_local_address;
    bool simulation;
    bool expect_warning;
  };
  for (const Row row : {
           Row{"non_loopback_config_unset", "10.56.87.2", nullptr, "", "",
               false, true},
           Row{"non_loopback_cli_unset", "127.0.0.1", nullptr, "10.56.87.2", "",
               false, true},
           Row{"non_loopback_config_local_set", "10.56.87.2", "10.56.87.5", "",
               "", false, false},
           Row{"non_loopback_cli_local_set", "10.56.87.2", nullptr, "",
               "10.56.87.5", false, false},
           Row{"loopback_unset", "127.0.0.1", nullptr, "", "", false, false},
           Row{"sim_override_unset", "10.56.87.2", nullptr, "127.0.0.1", "",
               true, false},
       }) {
    SCOPED_TRACE(row.name);
    auto options = SessionFixture(std::string{"local_warn_"} + row.name);
    std::filesystem::remove_all(options.output_dir);
    std::string toml{kFixtureToml};
    const std::string header = "[robot]\n";
    std::string extra = std::string{"controller_address = \""} +
                        row.controller_address + "\"\n";
    if (row.config_local_address != nullptr) {
      extra +=
          std::string{"local_address = \""} + row.config_local_address + "\"\n";
    }
    toml.insert(toml.find(header) + header.size(), extra);
    options.config_path =
        TempPath(std::string{"local_warn_cfg_"} + row.name) + ".toml";
    std::ofstream{options.config_path} << toml;
    options.remote = row.cli_remote;
    options.local_address = row.cli_local_address;
    options.simulation = row.simulation;
    options.describe_only = true;

    EXPECT_EQ(Launcher{options}.Run(), 0);
    const std::string graph =
        ReadFile(options.output_dir + "/" + std::to_string(options.session_id) +
                 "/graph.json");
    const bool has_warning =
        graph.find(
            "\"severity\": \"warning\", \"subject\": \"hardware_node\", "
            "\"message\": \"[robot] local_address is unset") !=
        std::string::npos;
    EXPECT_EQ(has_warning, row.expect_warning) << graph;
    std::filesystem::remove_all(options.output_dir);
    ::remove(options.config_path.c_str());
  }
}

// Fetches one HTTP GET path on 127.0.0.1:`port` and returns the raw response.
std::string HttpGetLoopback(uint16_t port, std::string_view path) {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) return {};
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::connect(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) !=
      0) {
    ::close(fd);
    return {};
  }
  const std::string request = "GET " + std::string{path} +
                              " HTTP/1.1\r\nHost: localhost\r\n"
                              "Connection: close\r\n\r\n";
  (void)::send(fd, request.data(), request.size(), 0);
  std::string response;
  char buf[1024];
  for (;;) {
    const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
    if (n <= 0) break;
    response.append(buf, static_cast<std::size_t>(n));
  }
  ::close(fd);
  return response;
}

// Stub studio_bridge that answers --describe and, when given --refusal-file,
// binds port 0, prints `listening <port>`, notifies `ready_fifo` with the port,
// serves GET /drive/summary.json from --refusal-file, and exits cleanly on
// SIGINT/SIGTERM. Starts with `#!/bin/sh` because WriteStubNode prepends a
// shell --talos-first-exec guard on line 2.
std::string RefusalBridgeStub(const std::string& ready_fifo) {
  return R"(#!/bin/sh
exec python3 - "$@" <<'PYEOF'
import http.server, json, os, signal, sys

if "--describe" in sys.argv:
    print(json.dumps({
        "describe_version": 1,
        "name": "studio_bridge",
        "target": "//studio/bridge:studio_bridge",
        "sources": [{"kind": "WATCHER", "name": "/talos/telemetry",
                     "message_bytes": 1200, "alignment": 8,
                     "schema": "bytes", "schema_hash": 0, "slots": 64}]
    }))
    sys.exit(0)

refusal_file = ""
for i, arg in enumerate(sys.argv):
    if arg == "--refusal-file" and i + 1 < len(sys.argv):
        refusal_file = sys.argv[i + 1]

with open(refusal_file, encoding="utf-8") as f:
    rec = json.load(f)
body = json.dumps({
    "kind": "talos.drive.summary",
    "session": "",
    "supervised": False,
    "registry": False,
    "launch_refused": True,
    "refusal_reason": rec.get("reason", ""),
    "nodes": [],
    "omitted": 0,
}).encode("utf-8")

class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/drive/summary.json":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()
    def log_message(self, *args):
        pass

server = http.server.HTTPServer(("127.0.0.1", 0), Handler)
port = server.server_address[1]
print(f"listening {port}", flush=True)

def on_stop(signum, frame):
    os._exit(0)

signal.signal(signal.SIGINT, on_stop)
signal.signal(signal.SIGTERM, on_stop)

with open(")" +
         ready_fifo + R"(", "w", encoding="utf-8") as fifo:
    fifo.write(f"{port}\n")
    fifo.flush()

server.serve_forever()
PYEOF
)";
}

// Captures sd_notify datagrams sent to NOTIFY_SOCKET while in scope.
class NotifyCapture {
 public:
  NotifyCapture() {
    static int serial = 0;
    path_ = "/tmp/talos_notify_cap_" + std::to_string(::getpid()) + "_" +
            std::to_string(serial++);
    ::unlink(path_.c_str());
    fd_ = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::memcpy(addr.sun_path, path_.c_str(), path_.size() + 1);
    const auto len = static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) +
                                            path_.size() + 1);
    if (fd_ >= 0 &&
        ::bind(fd_, reinterpret_cast<const sockaddr*>(&addr), len) == 0) {
      ::fcntl(fd_, F_SETFL, O_NONBLOCK);
      ::setenv("NOTIFY_SOCKET", path_.c_str(), 1);
    }
  }
  ~NotifyCapture() {
    ::unsetenv("NOTIFY_SOCKET");
    if (fd_ >= 0) ::close(fd_);
    ::unlink(path_.c_str());
  }
  NotifyCapture(const NotifyCapture&) = delete;
  NotifyCapture& operator=(const NotifyCapture&) = delete;

  std::vector<std::string> Drain() const {
    std::vector<std::string> out;
    if (fd_ < 0) return out;
    char buf[128];
    for (;;) {
      const ssize_t n = ::recv(fd_, buf, sizeof(buf), 0);
      if (n <= 0) break;
      out.emplace_back(buf, static_cast<std::size_t>(n));
    }
    return out;
  }

  bool WaitFor(std::string_view expected, std::chrono::seconds timeout) const {
    if (fd_ < 0) return false;
    const int timeout_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    for (int attempt = 0; attempt < 64; ++attempt) {
      for (const auto& msg : Drain()) {
        if (msg == expected) return true;
      }
      pollfd pfd{.fd = fd_, .events = POLLIN, .revents = 0};
      const int ready = ::poll(&pfd, 1, timeout_ms);
      if (ready <= 0) return false;
    }
    return false;
  }

 private:
  std::string path_;
  int fd_{-1};
};

// REQ-OPS-34, REQ-OPS-36, TODO.md DEPLOY-12: with --serve-refusal and
// --start-studio-bridge, a phase-1 graph refusal writes last_refusal.json in
// the log root, starts the Studio bridge with --refusal-file serving
// /drive/summary.json with launch_refused: true and the refusal reason,
// sends READY=1 once the bridge announces its listening port, stops
// the bridge cleanly on SIGINT, and still returns 1.
TEST(LauncherGraphTest, ServeRefusalStartsBridgeAndServesSummaryUntilStopped) {
  NotifyCapture notify;
  const std::string ready_fifo = TempPath("serve_refusal_ready.fifo");
  ::unlink(ready_fifo.c_str());
  ASSERT_EQ(::mkfifo(ready_fifo.c_str(), 0600), 0);

  // Two writers on /alpha/state triggers a phase-1 graph refusal.
  const std::string writer_alpha =
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")});
  const std::string writer_beta =
      DescribingStub("beta", "//example/beta:node", {Sender("/alpha/state")});
  GraphFixture fixture{"serve_refusal", writer_alpha, writer_beta};
  const std::string log_root = fixture.options.output_dir;
  fixture.options.start_studio_bridge = true;
  fixture.options.serve_refusal = true;
  fixture.options.studio_http_port = 0;
  fixture.options.duration_s = static_cast<int>(kHang.count());
  fixture.options.binary_overrides["//studio/bridge:studio_bridge"] =
      WriteStubNode("serve_refusal_bridge", RefusalBridgeStub(ready_fifo));

  int exit_code = -1;
  std::thread runner([&] { exit_code = Launcher{fixture.options}.Run(); });

  // Blocks until the refusal bridge binds and announces its port.
  uint16_t bound_port = 0;
  {
    std::ifstream in{ready_fifo};
    int parsed = 0;
    ASSERT_TRUE(in >> parsed) << "refusal bridge did not report a port";
    bound_port = static_cast<uint16_t>(parsed);
  }
  ASSERT_GT(bound_port, 0u);
  EXPECT_TRUE(notify.WaitFor("READY=1", kHang))
      << "launcher did not send READY=1 after refusal bridge announced port";

  const std::string response =
      HttpGetLoopback(bound_port, "/drive/summary.json");
  EXPECT_NE(response.find("200 OK"), std::string::npos) << response;
  EXPECT_NE(response.find("\"launch_refused\": true"), std::string::npos)
      << response;
  EXPECT_NE(response.find("/alpha/state"), std::string::npos) << response;

  const std::string refusal_json =
      ReadFile(log_root + "/" + std::string{kLastRefusalFile});
  EXPECT_NE(refusal_json.find("\"refused\": true"), std::string::npos)
      << refusal_json;
  EXPECT_NE(refusal_json.find("/alpha/state"), std::string::npos)
      << refusal_json;

  ASSERT_EQ(::kill(::getpid(), SIGINT), 0);
  runner.join();

  EXPECT_EQ(exit_code, 1);
  EXPECT_FALSE(fixture.has_manifest());
  ::unlink(ready_fifo.c_str());
  std::filesystem::remove_all(log_root);
}

// REQ-OPS-34, REQ-OPS-50, TODO.md DEPLOY-12:
// 1. Without --serve-refusal, a phase-1 refusal still writes last_refusal.json
//    and returns 1 immediately without spawning the Studio bridge in refusal
//    mode (preserving CLI tests like RefusedGraphIsNamedAtOnce).
// 2. With --describe-only --serve-refusal, neither last_refusal.json nor the
//    refusal bridge runs.
// 3. When the refusal bridge exits immediately on its own (e.g. EADDRINUSE
//    because a live session holds port 5800), --serve-refusal returns 1
//    immediately without waiting for duration_s and without sending READY=1.
TEST(LauncherGraphTest,
     RefusalWithoutServeFlagOrWithExitingBridgeReturnsOneImmediately) {
  const std::string writer_alpha =
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")});
  const std::string writer_beta =
      DescribingStub("beta", "//example/beta:node", {Sender("/alpha/state")});

  // Case 1: serve_refusal = false -> refusal bridge is never spawned.
  {
    const std::string spawned = TempPath("no_serve_refusal_spawned");
    ::unlink(spawned.c_str());
    GraphFixture fixture{"no_serve_refusal", writer_alpha, writer_beta};
    const std::string log_root = fixture.options.output_dir;
    fixture.options.start_studio_bridge = true;
    fixture.options.serve_refusal = false;
    fixture.options.binary_overrides["//studio/bridge:studio_bridge"] =
        WriteStubNode(
            "no_serve_refusal_bridge",
            "#!/bin/sh\n"
            "for a in \"$@\"; do\n"
            "  if [ \"$a\" = \"--describe\" ]; then\n"
            "    printf '{\"name\":\"studio_bridge\","
            "\"target\":\"//studio/bridge:studio_bridge\",\"sources\":[]}\\n'\n"
            "    exit 0\n"
            "  fi\n"
            "done\n"
            "echo spawned > \"" +
                spawned + "\"\nexit 0\n");

    EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
    EXPECT_FALSE(std::filesystem::exists(spawned))
        << "refusal bridge ran without --serve-refusal";
    const std::string refusal_json =
        ReadFile(log_root + "/" + std::string{kLastRefusalFile});
    EXPECT_NE(refusal_json.find("/alpha/state"), std::string::npos)
        << refusal_json;
    ::unlink(spawned.c_str());
    std::filesystem::remove_all(log_root);
  }

  // Case 2: --describe-only with --serve-refusal writes no last_refusal.json.
  {
    GraphFixture fixture{"describe_only_serve_refusal", writer_alpha,
                         writer_beta};
    const std::string log_root = fixture.options.output_dir;
    fixture.options.describe_only = true;
    fixture.options.start_studio_bridge = true;
    fixture.options.serve_refusal = true;
    EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
    EXPECT_FALSE(
        std::filesystem::exists(log_root + "/" + std::string{kLastRefusalFile}))
        << "--describe-only must not write last_refusal.json";
    std::filesystem::remove_all(log_root);
  }

  // Case 3: refusal bridge exits immediately (e.g. EADDRINUSE) -> returns 1
  // without waiting for duration_s and without reporting READY=1 to systemd.
  {
    NotifyCapture notify;
    GraphFixture fixture{"eaddrinuse_serve_refusal", writer_alpha, writer_beta};
    const std::string log_root = fixture.options.output_dir;
    testing::SteppedClock clock;
    fixture.options.supervision_clock = clock.clock();
    fixture.options.start_studio_bridge = true;
    fixture.options.serve_refusal = true;
    fixture.options.duration_s = static_cast<int>(kHang.count());
    fixture.options
        .binary_overrides["//studio/bridge:studio_bridge"] = WriteStubNode(
        "eaddrinuse_refusal_bridge",
        "#!/bin/sh\n"
        "for a in \"$@\"; do\n"
        "  if [ \"$a\" = \"--describe\" ]; then\n"
        "    printf '{\"name\":\"studio_bridge\","
        "\"target\":\"//studio/bridge:studio_bridge\",\"sources\":[]}\\n'\n"
        "    exit 0\n"
        "  fi\n"
        "done\n"
        "echo 'cannot listen on port 5800 (another process holds it?)' >&2\n"
        "exit 1\n");

    EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
    const auto notifications = notify.Drain();
    EXPECT_EQ(std::find(notifications.begin(), notifications.end(), "READY=1"),
              notifications.end())
        << "READY=1 must not be sent when refusal bridge exits before "
           "listening";
    std::filesystem::remove_all(log_root);
  }
}

// REQ-OPS-34, REQ-OPS-50, TODO.md DEPLOY-12: a launcher that does not own the
// session lock (refused because another launcher holds it, or refused on a
// pre-lock CLI error) must exit 1 immediately without overwriting an existing
// <log_root>/last_refusal.json or spawning a refusal studio_bridge (which
// would collide with a live session running --no-studio or a custom port).
TEST(LauncherGraphTest,
     RefusalWithoutSessionLockPreservesLastRefusalAndSkipsBridge) {
  const std::string sentinel =
      "{\"refused\":true,\"reason\":\"existing live refusal\"}\n";
  const std::string writer_alpha =
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")});
  const std::string writer_beta =
      DescribingStub("beta", "//example/beta:node", {Sender("/alpha/state")});

  // Case 1: held session lock (!take_lock()).
  {
    const std::string spawned = TempPath("held_lock_refusal_spawned");
    ::unlink(spawned.c_str());
    GraphFixture fixture{"held_lock_refusal", writer_alpha, writer_beta};
    const std::string log_root = fixture.options.output_dir;
    std::filesystem::create_directories(log_root);
    const std::string refusal_path =
        log_root + "/" + std::string{kLastRefusalFile};
    {
      std::ofstream out{refusal_path, std::ios::binary};
      out << sentinel;
    }
    testing::SteppedClock clock;
    fixture.options.supervision_clock = clock.clock();
    fixture.options.start_studio_bridge = true;
    fixture.options.serve_refusal = true;
    fixture.options.duration_s = static_cast<int>(kHang.count());
    fixture.options.acquire_session_lock = [](std::string& error,
                                              bool* no_room) {
      error = "another launcher (pid 99999) is running a session";
      if (no_room != nullptr) *no_room = false;
      return std::optional<SessionLock>{};
    };
    fixture.options.binary_overrides["//studio/bridge:studio_bridge"] =
        WriteStubNode(
            "held_lock_refusal_bridge",
            "#!/bin/sh\necho spawned > \"" + spawned + "\"\nexit 0\n");

    EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
    EXPECT_FALSE(std::filesystem::exists(spawned))
        << "refusal bridge must not spawn when session_lock is held";
    EXPECT_EQ(ReadFile(refusal_path), sentinel)
        << "existing last_refusal.json must not be clobbered on held lock";
    ::unlink(spawned.c_str());
    std::filesystem::remove_all(log_root);
  }

  // Case 2: pre-lock refusal (--allow-graph-errors without --sim).
  {
    const std::string spawned = TempPath("pre_lock_refusal_spawned");
    ::unlink(spawned.c_str());
    GraphFixture fixture{"pre_lock_refusal", writer_alpha, writer_beta};
    const std::string log_root = fixture.options.output_dir;
    std::filesystem::create_directories(log_root);
    const std::string refusal_path =
        log_root + "/" + std::string{kLastRefusalFile};
    {
      std::ofstream out{refusal_path, std::ios::binary};
      out << sentinel;
    }
    testing::SteppedClock clock;
    fixture.options.supervision_clock = clock.clock();
    fixture.options.allow_graph_errors = true;
    fixture.options.simulation = false;
    fixture.options.start_studio_bridge = true;
    fixture.options.serve_refusal = true;
    fixture.options.duration_s = static_cast<int>(kHang.count());
    fixture.options.binary_overrides["//studio/bridge:studio_bridge"] =
        WriteStubNode(
            "pre_lock_refusal_bridge",
            "#!/bin/sh\necho spawned > \"" + spawned + "\"\nexit 0\n");

    EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
    EXPECT_FALSE(std::filesystem::exists(spawned))
        << "refusal bridge must not spawn on pre-lock refusal";
    EXPECT_EQ(ReadFile(refusal_path), sentinel)
        << "existing last_refusal.json must not be clobbered on pre-lock "
           "refusal";
    ::unlink(spawned.c_str());
    std::filesystem::remove_all(log_root);
  }
}

// REQ-OPS-34, TODO.md DEPLOY-12: StopSignals::Install() must be idempotent so
// RefuseLaunch() never clears a stop signal that arrived after RunSession()
// installed stop handlers; a pending stop must return 1 immediately without
// spawning the refusal studio_bridge.
TEST(LauncherGraphTest,
     RefuseLaunchHonorsPendingStopSignalWithoutSpawningBridge) {
  const std::string spawned = TempPath("pending_stop_refusal_spawned");
  ::unlink(spawned.c_str());
  auto options = SessionFixture("pending_stop_refusal");
  const std::string log_root = options.output_dir;
  // Invalid TOML forces ParseRobotConfig to fail and enter RefuseLaunch right
  // after RunSession installed stop_signals and acquired session_lock.
  std::ofstream{options.config_path} << "[robot\n";
  testing::SteppedClock clock;
  options.supervision_clock = clock.clock();
  options.start_studio_bridge = true;
  options.serve_refusal = true;
  options.duration_s = static_cast<int>(kHang.count());
  options.acquire_session_lock = [](std::string& error, bool* no_room) {
    auto lock = SessionLock::Acquire(error, no_room);
    if (lock) ::raise(SIGINT);
    return lock;
  };
  options.binary_overrides["//studio/bridge:studio_bridge"] = WriteStubNode(
      "pending_stop_refusal_bridge",
      "#!/bin/sh\necho spawned > \"" + spawned + "\"\nsleep 60\n");

  EXPECT_EQ(Launcher{options}.Run(), 1);
  EXPECT_FALSE(std::filesystem::exists(spawned))
      << "refusal bridge must not spawn when a stop signal is already pending";
  ::unlink(spawned.c_str());
  std::filesystem::remove_all(log_root);
}

// TOOLS-7 / REQ-OPS-20: --check-session-lock checks the namespace SessionLock
// without requiring --config or --bundle, exits 0 (and releases the lock) when
// free, and exits 1 naming the holding process on stderr when held.
TEST(LauncherTest, CheckSessionLockReportsFreeAndHeldWithoutConfig) {
  const std::string log_root = TempPath("check_session_lock_out");
  std::filesystem::remove_all(log_root);

  LauncherOptions options;
  options.check_session_lock = true;
  options.output_dir = log_root;

  // Free: exits 0 without creating any session directory or leaving the lock
  // held.
  EXPECT_EQ(Launcher{options}.Run(), 0);
  EXPECT_FALSE(std::filesystem::exists(log_root));
  {
    std::string lock_error;
    EXPECT_TRUE(SessionLock::Acquire(lock_error).has_value()) << lock_error;
  }

  // Held by another process: exits 1 and names the holder's pid on stderr.
  int ready_pipe[2]{-1, -1};
  int hold_pipe[2]{-1, -1};
  ASSERT_EQ(::pipe(ready_pipe), 0);
  ASSERT_EQ(::pipe(hold_pipe), 0);
  const pid_t holder = ::fork();
  ASSERT_GE(holder, 0);
  if (holder == 0) {
    ::close(ready_pipe[0]);
    ::close(hold_pipe[1]);
    std::string err;
    auto lock = SessionLock::Acquire(err);
    const char ok = lock ? '1' : '0';
    static_cast<void>(::write(ready_pipe[1], &ok, 1));
    ::close(ready_pipe[1]);
    char sink = 0;
    static_cast<void>(::read(hold_pipe[0], &sink, 1));
    ::close(hold_pipe[0]);
    _exit(lock ? 0 : 2);
  }
  ::close(ready_pipe[1]);
  ::close(hold_pipe[0]);
  char ok = 0;
  ASSERT_EQ(::read(ready_pipe[0], &ok, 1), 1);
  ::close(ready_pipe[0]);
  ASSERT_EQ(ok, '1');

  ::testing::internal::CaptureStderr();
  const int held_status = Launcher{options}.Run();
  const std::string stderr_text = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(held_status, 1) << stderr_text;
  EXPECT_NE(stderr_text.find("pid " + std::to_string(holder)),
            std::string::npos)
      << stderr_text;
  EXPECT_FALSE(std::filesystem::exists(log_root));

  ::close(hold_pipe[1]);
  int wait_status = 0;
  ASSERT_EQ(::waitpid(holder, &wait_status, 0), holder);
  EXPECT_EQ(Launcher{options}.Run(), 0);
}

// VIS-1 (REQ-OPS-34, REQ-OPS-38, REQ-OPS-39, REQ-CONTRACT-33): even when
// telemetry is non-critical (`critical = false`), an invalid
// `[[subsystem.labels]]` entry or invalid `[studio] dashboard` layout is a
// hard error in `--describe-only` and normal launch rather than a warning,
// while valid configs pass and `--sim --allow-graph-errors` overrides refusal.
TEST(LauncherGraphTest,
     DescribeOnlyRefusesInvalidTelemetryLabelsAndDashboardLayout) {
  std::string rf_error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &rf_error));
  const char* telemetry_loc = std::getenv("TALOS_TELEMETRY");
  ASSERT_NE(runfiles, nullptr) << rf_error;
  ASSERT_NE(telemetry_loc, nullptr);
  const std::string telemetry_bin = runfiles->Rlocation(telemetry_loc);
  ASSERT_FALSE(telemetry_bin.empty());

  const schema::Identity alerts_id = schema::Of<schema_test::Alerts>();
  introspect::naming::SourceShape shooter_status;
  shooter_status.kind = event::SourceKind::SENDER;
  shooter_status.name = "/shooter/status";
  shooter_status.message_bytes = sizeof(schema_test::Alerts);
  shooter_status.alignment = alignof(schema_test::Alerts);
  shooter_status.schema = alerts_id.name;
  shooter_status.schema_hash = alerts_id.hash;
  shooter_status.bfbs.assign(alerts_id.bfbs.begin(), alerts_id.bfbs.end());

  const std::string hw_stub = WriteStubNode(
      "vis1_hw",
      DescribingStub("hardware_node", "//talOS/bridge:hardware_node", {}));
  const std::string shooter_stub = WriteStubNode(
      "vis1_shooter",
      DescribingStub("shooter", "//example/shooter:node", {shooter_status}));
  const std::string auto_stub = WriteStubNode(
      "vis1_auto",
      DescribingStub("autonomous", "//example/autonomous:node", {}));

  const std::string dir = TempPath("vis1_telemetry_lint");
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  const auto run_case =
      [&](const std::string& tag, const std::string& studio_extra,
          const std::string& telemetry_extra, const std::string& dashboard_toml,
          bool describe_only, bool allow_graph_errors,
          std::string& out_graph) -> int {
    const std::string case_dir = dir + "/" + tag;
    std::filesystem::create_directories(case_dir);
    if (!dashboard_toml.empty()) {
      std::ofstream(case_dir + "/dashboard.toml") << dashboard_toml;
    }
    const std::string config_path = case_dir + "/robot.toml";
    std::ofstream(config_path)
        << "[robot]\nperiod_us = 10000\n"
        << "[studio]\ntopic = \"/talos/telemetry\"\n"
        << studio_extra
        << "[subsystems.shooter]\nnode = \"//example/shooter:node\"\n"
        << "[subsystems.autonomous]\nnode = \"//example/autonomous:node\"\n"
        << "routines = [\"preload\", \"none\", \"taxi\"]\n"
        << "[subsystems.telemetry]\nnode = \"//talOS/telemetry:node\"\n"
        << "critical = false\n"
        << telemetry_extra;

    LauncherOptions opts;
    opts.config_path = config_path;
    opts.output_dir = case_dir + "/out";
    opts.session_id = 7001;
    opts.describe_only = describe_only;
    opts.simulation = allow_graph_errors;
    opts.allow_graph_errors = allow_graph_errors;
    opts.binary_overrides["//talOS/bridge:hardware_node"] = hw_stub;
    opts.binary_overrides["//example/shooter:node"] = shooter_stub;
    opts.binary_overrides["//example/autonomous:node"] = auto_stub;
    opts.binary_overrides["//talOS/telemetry:node"] = telemetry_bin;
    const int rc = Launcher{opts}.Run();
    out_graph = ReadFile(opts.output_dir + "/7001/graph.json");
    return rc;
  };

  const std::string valid_dashboard = R"([dashboard]
title = "Drive"
[[widgets]]
kind = "number"
at = [0, 0, 2, 1]
channel = "shooter/status/supply_v"
)";
  const std::string valid_labels = R"([[subsystems.telemetry.labels]]
channel = "shooter/status/routine_index"
from = "autonomous.routines"
)";

  // 1. Valid labels + valid dashboard with critical = false telemetry passes.
  {
    std::string graph;
    EXPECT_EQ(run_case("valid", "dashboard = \"dashboard.toml\"\n",
                       valid_labels, valid_dashboard, /*describe_only=*/true,
                       /*allow_graph_errors=*/false, graph),
              0)
        << graph;
    EXPECT_TRUE(GraphErrors(graph).empty()) << graph;
  }

  // 2. Invalid [[subsystem.labels]] (unknown dictionary channel) is an error
  // in --describe-only and normal launch.
  for (const bool describe_only : {true, false}) {
    SCOPED_TRACE(describe_only ? "bad_label_describe" : "bad_label_launch");
    std::string graph;
    const std::string bad_labels = R"([[subsystems.telemetry.labels]]
channel = "shooter/status/nonexistent"
from = "autonomous.routines"
)";
    EXPECT_EQ(run_case(describe_only ? "bad_label_d" : "bad_label_l", "",
                       bad_labels, "", describe_only,
                       /*allow_graph_errors=*/false, graph),
              1)
        << graph;
    const auto errors = GraphFindings(graph, "error", "telemetry");
    ASSERT_FALSE(errors.empty()) << graph;
    EXPECT_NE(errors[0].find("labels[0].channel"), std::string::npos)
        << errors[0];
  }

  // 3. Invalid dashboard layout (file-level syntax/schema error) is an error.
  {
    std::string graph;
    const std::string bad_layout = R"([dashboard]
title = "Drive"
[[widgets]]
kind = "not_a_widget"
at = [0, 0, 2, 1]
)";
    EXPECT_EQ(run_case("bad_layout_kind", "dashboard = \"dashboard.toml\"\n",
                       "", bad_layout, /*describe_only=*/true,
                       /*allow_graph_errors=*/false, graph),
              1)
        << graph;
    const auto errors = GraphFindings(graph, "error", "studio");
    ASSERT_FALSE(errors.empty()) << graph;
    EXPECT_NE(errors[0].find("widgets[0].kind"), std::string::npos)
        << errors[0];
  }

  // 4. Invalid dashboard layout (channel absent from dictionary) is an error.
  {
    std::string graph;
    const std::string missing_ch_layout = R"([dashboard]
title = "Drive"
[[widgets]]
kind = "number"
at = [0, 0, 2, 1]
channel = "shooter/status/nope"
)";
    EXPECT_EQ(run_case("bad_layout_channel", "dashboard = \"dashboard.toml\"\n",
                       "", missing_ch_layout, /*describe_only=*/true,
                       /*allow_graph_errors=*/false, graph),
              1)
        << graph;
    const auto errors = GraphFindings(graph, "error", "studio");
    ASSERT_FALSE(errors.empty()) << graph;
    EXPECT_NE(errors[0].find("widgets[0].channel"), std::string::npos)
        << errors[0];
  }

  // 5. --sim --allow-graph-errors overrides the refusal while keeping the
  // error recorded in graph.json.
  {
    std::string graph;
    const std::string bad_layout = R"([dashboard]
[[widgets]]
kind = "number"
at = [0, 0, 2, 1]
channel = "shooter/status/nope"
)";
    EXPECT_EQ(run_case("allow_errors", "dashboard = \"dashboard.toml\"\n", "",
                       bad_layout, /*describe_only=*/true,
                       /*allow_graph_errors=*/true, graph),
              0)
        << graph;
    EXPECT_FALSE(GraphFindings(graph, "error", "studio").empty()) << graph;
  }

  std::filesystem::remove_all(dir);
  std::remove(hw_stub.c_str());
  std::remove(shooter_stub.c_str());
  std::remove(auto_stub.c_str());
}

// REQ-RT-23: `--describe-only` enforces the 20 ms minimum period for Python
// actuator owners (`abi_update: true`): a Python actuator owner at 10 ms fails
// with an error naming the subsystem, its period and the 20000 us minimum,
// while a C++ actuator owner (`abi_update: false`) at 5 ms passes.
TEST(LauncherGraphTest, DescribeOnlyEnforcesPythonActuatorMinimumPeriod) {
  const auto stub_with_abi = [](std::string_view node, std::string_view target,
                                bool abi_update) {
    introspect::Description description;
    description.name = std::string{node};
    description.target = std::string{target};
    description.abi_update = abi_update;
    return std::string{"#!/bin/sh\n"} +
           "for arg in \"$@\"; do\n"
           "  if [ \"$arg\" = \"--describe\" ]; then\n"
           "    cat <<'TALOS_DESCRIBE'\n" +
           introspect::DescribeToJson(description) +
           "TALOS_DESCRIBE\n"
           "    exit 0\n"
           "  fi\n"
           "done\n"
           "exit 0\n";
  };

  // Case 1: beta (10 ms motor owner) describes abi_update: true -> refused;
  // alpha (5 ms motor owner) describes abi_update: false -> no error for alpha.
  {
    GraphFixture fixture{
        "py_period_refused",
        stub_with_abi("alpha", "//example/alpha:node", /*abi_update=*/false),
        stub_with_abi("beta", "//example/beta:node", /*abi_update=*/true)};
    fixture.options.describe_only = true;
    EXPECT_EQ(Launcher{fixture.options}.Run(), 1);
    EXPECT_FALSE(fixture.has_manifest());
    const std::string graph = fixture.graph_json();
    const auto beta_errors = GraphFindings(graph, "error", "beta");
    ASSERT_EQ(beta_errors.size(), 1u) << graph;
    EXPECT_NE(beta_errors[0].find("10000"), std::string::npos)
        << beta_errors[0];
    EXPECT_NE(beta_errors[0].find(std::to_string(kMinPythonActuatorPeriodUs)),
              std::string::npos)
        << beta_errors[0];
    EXPECT_TRUE(GraphFindings(graph, "error", "alpha").empty()) << graph;
  }

  // Case 2: alpha (5 ms motor owner) and beta (10 ms motor owner) both
  // describe abi_update: false -> passes --describe-only cleanly.
  {
    GraphFixture fixture{
        "py_period_cpp_ok",
        stub_with_abi("alpha", "//example/alpha:node", /*abi_update=*/false),
        stub_with_abi("beta", "//example/beta:node", /*abi_update=*/false)};
    fixture.options.describe_only = true;
    EXPECT_EQ(Launcher{fixture.options}.Run(), 0);
    EXPECT_FALSE(fixture.has_manifest());
    const std::string graph = fixture.graph_json();
    EXPECT_TRUE(GraphErrors(graph).empty()) << graph;
  }
}

// REQ-OPS-12: WriteFileAtomically syncs the flushed temporary file before
// renaming over the target, syncs the parent directory after renaming, and
// leaves the prior file intact (and cleans up the temp file) if the temp-fd
// sync fails.
TEST(LauncherTest, WriteFileAtomicallySyncsTempBeforeRenameAndDirAfterRename) {
  const std::string dir = TempPath("atomic_write_sync_order");
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  const std::string target = dir + "/manifest.json";
  const std::string temp = target + ".tmp";
  {
    std::ofstream out{target, std::ios::binary};
    out << "prior-version";
  }

  struct stat dir_stat {};
  ASSERT_EQ(::stat(dir.c_str(), &dir_stat), 0);

  // Success path: temp-fd synced before rename, dir-fd synced after rename.
  {
    std::vector<std::string> events;
    std::string error;
    const SyncFunction record_sync = [&](int fd) {
      struct stat st {};
      if (::fstat(fd, &st) != 0) {
        events.push_back("fstat_failed");
        errno = EIO;
        return -1;
      }
      if (S_ISREG(st.st_mode)) {
        struct stat tmp_st {};
        const bool matches_tmp = ::stat(temp.c_str(), &tmp_st) == 0 &&
                                 st.st_dev == tmp_st.st_dev &&
                                 st.st_ino == tmp_st.st_ino;
        const bool target_still_prior = ReadFile(target) == "prior-version";
        const bool tmp_has_new = ReadFile(temp) == "new-version";
        events.push_back(matches_tmp && target_still_prior && tmp_has_new
                             ? "sync_temp_before_rename"
                             : "sync_temp_wrong_state");
      } else if (S_ISDIR(st.st_mode)) {
        const bool matches_dir =
            st.st_dev == dir_stat.st_dev && st.st_ino == dir_stat.st_ino;
        const bool target_has_new = ReadFile(target) == "new-version";
        const bool tmp_gone = !std::filesystem::exists(temp);
        events.push_back(matches_dir && target_has_new && tmp_gone
                             ? "sync_dir_after_rename"
                             : "sync_dir_wrong_state");
      } else {
        events.push_back("sync_unexpected_kind");
      }
      return 0;
    };
    EXPECT_TRUE(WriteFileAtomically(target, "new-version", record_sync, error))
        << error;
    EXPECT_EQ(events, (std::vector<std::string>{"sync_temp_before_rename",
                                                "sync_dir_after_rename"}));
    EXPECT_EQ(ReadFile(target), "new-version");
    EXPECT_FALSE(std::filesystem::exists(temp));
  }

  // Failure path: temp-fd sync failure aborts before rename, leaves the prior
  // file intact, removes the temporary file, and does not sync the directory.
  {
    int sync_calls = 0;
    std::string error;
    const SyncFunction fail_temp_sync = [&](int) {
      ++sync_calls;
      errno = EIO;
      return -1;
    };
    EXPECT_FALSE(
        WriteFileAtomically(target, "corrupt-attempt", fail_temp_sync, error));
    EXPECT_FALSE(error.empty());
    EXPECT_EQ(sync_calls, 1);
    EXPECT_EQ(ReadFile(target), "new-version");
    EXPECT_FALSE(std::filesystem::exists(temp));
  }

  std::filesystem::remove_all(dir);
}

// REQ-OPS-08, REQ-OPS-33, REQ-OPS-11: when an exception escapes RunSession()
// while ManifestWriter's background thread is active, ManifestWriter must stop
// and join before SessionDirectoryLock is released and before LauncherOutput is
// torn down (preventing UAF/race on std::cerr and unlocked directory writes).
TEST(LauncherTest, ManifestWriterStopsBeforeTeardownOnException) {
  GraphFixture fixture{
      "manifest_unwind",
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")}),
      DescribingStub("beta", "//example/beta:node", {Watcher("/alpha/state")})};
  const std::string session_dir = fixture.out_dir;
  const std::thread::id main_thread = std::this_thread::get_id();

  std::mutex mutex;
  std::condition_variable cv;
  bool worker_in_sync = false;
  bool worker_synced = false;
  bool lock_lost_while_writing = false;

  fixture.options.sync_file = [&](int) {
    if (std::this_thread::get_id() == main_thread) return 0;
    {
      const std::lock_guard lock{mutex};
      worker_in_sync = true;
      worker_synced = true;
    }
    cv.notify_all();
    // Hold briefly while the main thread throws and unwinds: if ~Launcher()
    // releases session_directory_ before joining manifest_writer_, Held() flips
    // to false here.
    std::unique_lock lock{mutex};
    for (int i = 0; i < 30; ++i) {
      if (!SessionDirectoryLock::Held(session_dir, false)) {
        lock_lost_while_writing = true;
        break;
      }
      cv.wait_for(lock, std::chrono::milliseconds{5});
    }
    // Fail the sync so ManifestWriter::Persist logs to std::cerr while
    // LauncherOutput must still be alive.
    errno = EIO;
    return -1;
  };

  bool threw_once = false;
  fixture.options.member_spawned = [&](const std::string&) {
    if (threw_once) return;
    threw_once = true;
    std::unique_lock lock{mutex};
    cv.wait_for(lock, kHang, [&] { return worker_in_sync; });
    throw std::runtime_error("injected unwind");
  };

  EXPECT_THROW(static_cast<void>(Launcher{fixture.options}.Run()),
               std::runtime_error);
  EXPECT_TRUE(worker_synced);
  EXPECT_FALSE(lock_lost_while_writing)
      << "SessionDirectoryLock was released while ManifestWriter was still "
         "running";
  EXPECT_FALSE(SessionDirectoryLock::Held(session_dir, false));
  const std::string talos_log =
      ReadFile(session_dir + "/" + std::string{kLauncherOutputFile});
  EXPECT_NE(talos_log.find("cannot persist"), std::string::npos) << talos_log;
}

// REQ-RT-08, REQ-OPS-30: the launcher forwards --sim to studio_bridge if and
// only if the session runs under --sim, so studio_bridge stays timeshare in
// simulation (unless TALOS_REALTIME=1) and requests observer realtime on the
// robot.
TEST(LauncherTest, StudioBridgeReceivesSimFlagIffSessionIsSimulation) {
  for (const bool simulation : {false, true}) {
    SCOPED_TRACE(simulation ? "simulation" : "robot");
    const std::string tag =
        simulation ? "bridge_sim_flag_on" : "bridge_sim_flag_off";
    auto options = SessionFixture(tag);
    std::filesystem::remove_all(options.output_dir);
    options.simulation = simulation;
    if (simulation) options.remote = "127.0.0.1";
    options.start_studio_bridge = true;
    options.max_restarts = 0;
    const std::string invoked = TempPath(tag + "_invoked");
    const std::string done_fifo = TempPath(tag + "_fifo");
    std::filesystem::remove(invoked);
    std::filesystem::remove(done_fifo);
    ASSERT_EQ(::mkfifo(done_fifo.c_str(), 0600), 0);

    // Hold alpha open until studio_bridge records its launch args and signals
    // the FIFO, so StudioBridge::Finish never SIGKILLs studio_bridge before
    // exec completes.
    options.binary_overrides["//example/alpha:node"] =
        WriteStubNode(tag + "_alpha",
                      "#!/bin/sh\n"
                      "for arg in \"$@\"; do\n"
                      "  if [ \"$arg\" = \"--describe\" ]; then\n"
                      "    printf '%s\\n' '" +
                          DescribeJson("alpha", "//example/alpha:node",
                                       {Sender("/alpha/state")}) +
                          "'\n"
                          "    exit 0\n"
                          "  fi\n"
                          "done\n"
                          "read _ < '" +
                          done_fifo +
                          "'\n"
                          "exit 0\n");
    options.binary_overrides["//studio/bridge:studio_bridge"] = WriteStubNode(
        tag + "_bridge",
        BridgeDescribeThen("echo \"studio_bridge $*\" >> '" + invoked +
                           "'\necho ok > '" + done_fifo + "'\nexit 0\n"));

    ASSERT_EQ(Launcher{options}.Run(), 0);
    const std::string launched = ReadFile(invoked);
    ASSERT_FALSE(launched.empty()) << "studio_bridge did not record args";
    const bool has_sim = launched.find(" --sim ") != std::string::npos ||
                         launched.find(" --sim\n") != std::string::npos;
    EXPECT_EQ(has_sim, simulation) << launched;

    std::filesystem::remove_all(options.output_dir);
    std::filesystem::remove(invoked);
    std::filesystem::remove(done_fifo);
  }
}

}  // namespace
}  // namespace talos::launcher
