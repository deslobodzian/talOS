// Launcher supervision: clear failures, isolated sessions, and exit-code
// propagation from critical nodes.
//
// Every session runs in a pid-unique temp directory with shell-stub nodes,
// so no two tests share state, ports, or shared memory. Stubs describe an
// empty graph unless a test explicitly exercises a failed probe.

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/launcher/launcher.h"

namespace talos::launcher {
namespace {

using namespace std::chrono_literals;

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_supervision_" +
         std::to_string(::getpid()) + "_" + std::string{name};
}

// Framework-owned fixture config: robot-agnostic, two stub subsystems.
constexpr std::string_view kFixtureToml = R"(
[robot]
name = "delivery_fixture"
period_us = 5000

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

// `robot_keys` lands inside the fixture's [robot] table.
std::string WriteFixtureToml(std::string_view tag,
                             std::string_view robot_keys = "") {
  const std::string path = TempPath(tag) + ".toml";
  std::string toml{kFixtureToml};
  const std::string header = "[robot]\n";
  toml.insert(toml.find(header) + header.size(), robot_keys);
  std::ofstream out{path};
  out << toml;
  return path;
}

std::string WriteStub(std::string_view tag, std::string_view body,
                      std::string_view node = "",
                      bool answers_describe = true) {
  const std::string path = TempPath(tag) + ".sh";
  if (node.empty()) node = tag.ends_with("_beta") ? "beta" : "alpha";
  introspect::Description description;
  description.name = std::string{node};
  description.target = node == "hardware_node"
                           ? "//talOS/bridge:hardware_node"
                           : "//example/" + std::string{node} + ":node";
  std::ofstream out{path};
  out << "#!/bin/sh\n"
      << "for arg in \"$@\"; do\n"
      << "  if [ \"$arg\" = \"--describe\" ]; then\n";
  if (answers_describe) {
    out << "    cat <<'TALOS_DESCRIBE'\n"
        << introspect::DescribeToJson(description) << "TALOS_DESCRIBE\n"
        << "    exit 0\n";
  } else {
    out << "    exit 1\n";
  }
  out << "  fi\n"
      << "done\n"
      << body;
  out.close();
  ::chmod(path.c_str(), 0755);
  return path;
}

std::string ReadFile(const std::string& path) {
  std::ifstream in{path};
  return std::string((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());
}

LauncherOptions BaseOptions(std::string_view tag,
                            std::string_view robot_keys = "") {
  LauncherOptions options;
  options.session_id = GenerateSessionId();
  options.output_dir = TempPath(tag);
  options.describe_timeout_ms = 4000;
  options.config_path = WriteFixtureToml(tag, robot_keys);
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub(std::string{tag} + "_hardware", "exit 0\n", "hardware_node");
  return options;
}

void MakeAlphaNonCritical(const LauncherOptions& options) {
  auto config = toml::parse_file(options.config_path);
  auto* alpha = config["subsystems"]["alpha"].as_table();
  alpha->erase("motors");
  alpha->insert_or_assign("critical", false);
  std::ofstream{options.config_path} << config;
}

int RunSession(LauncherOptions& options) {
  const int result = Launcher{options}.Run();
  options.output_dir += "/" + std::to_string(options.session_id);
  return result;
}

// A stub body that appends its --log argument to `record` (one line per run),
// then ends with `ending`.
std::string RecordLogArgThen(const std::string& record,
                             std::string_view ending) {
  return "prev=\"\"\n"
         "log=\"\"\n"
         "for arg in \"$@\"; do\n"
         "  if [ \"$prev\" = \"--log\" ]; then log=\"$arg\"; fi\n"
         "  prev=\"$arg\"\n"
         "done\n"
         "echo \"$log\" >> " +
         record + "\n" + std::string{ending};
}

std::vector<std::string> ReadLines(const std::string& path) {
  std::ifstream in{path};
  std::vector<std::string> lines;
  for (std::string line; std::getline(in, line);) lines.push_back(line);
  return lines;
}

// One node's object in manifest.json, from its name to the closing brace.
std::string ManifestEntry(const std::string& manifest, std::string_view name) {
  const std::size_t start =
      manifest.find("\"name\": \"" + std::string{name} + "\"");
  if (start == std::string::npos) return {};
  return manifest.substr(start, manifest.find('}', start) - start);
}

LauncherOptions ConstructionOptions(std::string_view tag, int delay_ms) {
  auto options = BaseOptions(tag);
  options.max_restarts = 0;
  auto config = toml::parse_file(options.config_path);
  config["subsystems"]["alpha"].as_table()->insert_or_assign("construction_ms",
                                                             delay_ms);
  std::ofstream{options.config_path} << config;
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles{
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error)};
  if (!runfiles) throw std::runtime_error(error);
  const char* construction_node = std::getenv("TALOS_CONSTRUCTION_NODE");
  if (construction_node == nullptr)
    throw std::runtime_error("missing construction node runfile");
  options.binary_overrides["//example/alpha:node"] =
      runfiles->Rlocation(construction_node);
  options.binary_overrides["//example/beta:node"] =
      WriteStub(std::string{tag} + "_beta", "exit 0\n");
  return options;
}

TEST(Supervision, SlowConstructorIsObservableAndBecomesActiveAfterOldGrace) {
  auto options = ConstructionOptions("slow_construction", 6000);
  options.duration_s = 9;
  std::atomic<bool> constructing{false};
  std::atomic<bool> active{false};
  std::jthread observer{[&](std::stop_token stop) {
    while (!stop.stop_requested()) {
      if (auto reader = introspect::RegistryReader::open()) {
        for (const auto& node : reader->nodes()) {
          if (node.session_id != options.session_id || node.name != "alpha")
            continue;
          if ((node.flags & introspect::kFlagConstructing) != 0) {
            constructing = true;
            EXPECT_FALSE(node.alive);
          } else if (node.alive && node.dispatch_count != 0) {
            active = true;
          }
        }
      }
      std::this_thread::sleep_for(10ms);
    }
  }};
  EXPECT_EQ(RunSession(options), 0);
  observer.request_stop();
  observer.join();
  EXPECT_TRUE(constructing);
  EXPECT_TRUE(active);
  const auto entry =
      ManifestEntry(ReadFile(options.output_dir + "/manifest.json"), "alpha");
  EXPECT_NE(entry.find("\"restart_count\": 0"), std::string::npos) << entry;
  std::filesystem::remove_all(options.output_dir);
}

// The construction deadline, not the session's end, stops a constructor that
// keeps heartbeating: the run ends well before the 10 s session would.
TEST(Supervision, HeartbeatingConstructorStillHasAFiniteDeadline) {
  auto options = ConstructionOptions("hung_construction", 60000);
  EXPECT_EQ(options.construction_timeout, 30s);
  options.construction_timeout = 150ms;
  options.duration_s = 10;
  const auto started = std::chrono::steady_clock::now();
  EXPECT_NE(RunSession(options), 0);
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_GE(elapsed, options.construction_timeout);
  EXPECT_LT(elapsed, 5s);
  std::filesystem::remove_all(options.output_dir);
}

TEST(Supervision, ConstructorFailureExitsNonzeroAndIsReported) {
  auto options = ConstructionOptions("failed_construction", -1);
  options.duration_s = 3;
  EXPECT_NE(RunSession(options), 0);
  const auto entry =
      ManifestEntry(ReadFile(options.output_dir + "/manifest.json"), "alpha");
  EXPECT_NE(entry.find("\"exit_code\": 1"), std::string::npos) << entry;
  std::filesystem::remove_all(options.output_dir);
}

// A run whose event log stops early is in the live manifest while it still
// runs, not first at shutdown: its reporter flags the registry row and the
// launcher records it. A 2-byte log_max_bytes leaves this session 1 byte, so
// each logging member gets the 1-byte minimum share and the real node's log
// is capped before its header; a cap is log_capped, not recorder_failed.
TEST(Supervision, StoppedEventLogReachesTheLiveManifest) {
  auto options = ConstructionOptions("stopped_log", 0);
  auto config = toml::parse_file(options.config_path);
  config["robot"].as_table()->insert_or_assign("log_max_bytes", 2);
  std::ofstream{options.config_path} << config;
  options.duration_s = 3;
  // RunSession rewrites options.output_dir on return; don't read it
  // concurrently.
  const auto manifest = options.output_dir + "/" +
                        std::to_string(options.session_id) + "/manifest.json";
  std::string live;  // alpha's entry from a manifest of the running session
  std::jthread observer{[&](const std::stop_token& stop) {
    while (!stop.stop_requested()) {
      const std::string text = ReadFile(manifest);
      auto entry = ManifestEntry(text, "alpha");
      if (text.find("\"end_wall_ns\": 0,") != std::string::npos &&
          entry.find("\"log_capped\": true") != std::string::npos) {
        live = std::move(entry);
        return;
      }
      std::this_thread::sleep_for(2ms);
    }
  }};
  EXPECT_EQ(RunSession(options), 0);
  observer.request_stop();
  observer.join();
  ASSERT_FALSE(live.empty()) << "no live manifest recorded the capped log";
  EXPECT_NE(live.find("\"recorder_failed\": false"), std::string::npos) << live;
  std::filesystem::remove_all(options.output_dir);
}

TEST(Supervision,
     NonCriticalDescribeFailureWarnsAndRestartsWithoutBlockingPeers) {
  auto options = BaseOptions("optional_probe");
  MakeAlphaNonCritical(options);
  const auto runs = TempPath("optional_probe_runs");
  ::remove(runs.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "optional_alpha",
      RecordLogArgThen(runs,
                       "case \"$log\" in *restart1.tlog) exit 0;; esac\n"
                       "exit 3\n"),
      "alpha", false);
  options.binary_overrides["//example/beta:node"] =
      WriteStub("optional_beta", "exit 0\n");
  const auto config = config::ParseRobotConfig(options.config_path);
  std::vector<NodeDescription> described;
  const auto findings =
      DescribeGraph(DiscoverNodes(config, options), options, described);
  EXPECT_FALSE(naming::HasError(findings));
  EXPECT_TRUE(std::any_of(findings.begin(), findings.end(), [](const auto& d) {
    return d.subject == "alpha" && d.severity == naming::Severity::WARNING;
  }));
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_EQ(ReadLines(runs).size(), 2u);
  const auto manifest = ReadFile(options.output_dir + "/manifest.json");
  EXPECT_NE(ManifestEntry(manifest, "beta").find("\"exit_code\": 0"),
            std::string::npos);
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

TEST(Supervision, ActuatorDescribeFailureRefusesEvenWhenMarkedNonCritical) {
  auto options = BaseOptions("actuator_probe");
  auto config = toml::parse_file(options.config_path);
  config["subsystems"]["alpha"].as_table()->insert_or_assign("critical", false);
  std::ofstream{options.config_path} << config;
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("actuator_alpha", "exit 0\n", "alpha", false);
  options.binary_overrides["//example/beta:node"] =
      WriteStub("actuator_beta", "exit 0\n");
  EXPECT_EQ(RunSession(options), 1);
  EXPECT_FALSE(std::filesystem::exists(options.output_dir + "/manifest.json"));
  std::filesystem::remove_all(options.output_dir);
}

// A critical member is never quarantined: past its budget it keeps restarting
// on the capped backoff. Seven runs inside the 6 s session prove it, because a
// quarantine would wait out the 60 s restart window.
TEST(Supervision, CriticalCrashLoopIsNeverQuarantined) {
  auto options = BaseOptions("critical_loop", "max_restarts = 1\n");
  options.duration_s = 6;
  const auto runs = TempPath("critical_loop_runs");
  ::remove(runs.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "critical_loop_alpha",
      RecordLogArgThen(runs,
                       "case \"$log\" in *restart6.tlog) exit 0;; esac\n"
                       "exit 3\n"));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("critical_loop_beta", "exit 0\n");
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_EQ(ReadLines(runs).size(), 7u);
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// A long-lived stub that exits 0 when the launcher stops it.
constexpr std::string_view kRunUntilStopped =
    "trap 'exit 0' INT TERM\nwhile :; do sleep 0.05; done\n";

// A critical node that exits 0 while its peers run on is lost, not finished:
// it restarts. Its restart then exits 0 at the deadline and is not restarted.
TEST(Supervision, CriticalCleanExitRestartsButDeadlineStopDoesNot) {
  auto options = BaseOptions("clean_exit");
  options.duration_s = 2;
  const auto runs = TempPath("clean_exit_runs");
  ::remove(runs.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "clean_exit_alpha",
      RecordLogArgThen(runs, "case \"$log\" in *alpha.tlog) exit 0;; esac\n" +
                                 std::string{kRunUntilStopped}));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("clean_exit_beta", kRunUntilStopped);
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_EQ(ReadLines(runs), (std::vector<std::string>{
                                 options.output_dir + "/alpha.tlog",
                                 options.output_dir + "/alpha.restart1.tlog"}));
  const auto alpha =
      ManifestEntry(ReadFile(options.output_dir + "/manifest.json"), "alpha");
  EXPECT_NE(alpha.find("\"restart_count\": 1"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"exit_code\": 0"), std::string::npos) << alpha;
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// A run that cannot be spawned -- fork or pipe failing, as under EAGAIN -- is
// retried on the restart backoff instead of abandoned: a critical member whose
// first spawn and first retry both fail still runs, as its first run.
TEST(Supervision, FailedSpawnIsRetriedNotAbandoned) {
  auto options = BaseOptions("spawn_failure");
  options.duration_s = 2;
  options.spawn_failures["alpha"] = 2;
  const auto runs = TempPath("spawn_failure_runs");
  ::remove(runs.c_str());
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("spawn_failure_hardware", kRunUntilStopped, "hardware_node");
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "spawn_failure_alpha", RecordLogArgThen(runs, kRunUntilStopped));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("spawn_failure_beta", kRunUntilStopped);
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_EQ(ReadLines(runs),
            std::vector<std::string>{options.output_dir + "/alpha.tlog"});
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// A run whose exec fails is a spawn failure that names its cause, as the
// child reports it, rather than a run that exited 127: alpha's first run takes
// its binary's execute permission away, so its restart cannot exec, and that
// restart is retried, not counted as a run, and marks alpha recorder_failed.
TEST(Supervision, ExecFailureIsASpawnFailureThatNamesItsCause) {
  auto options = BaseOptions("exec_failure", "max_restarts = 1\n");
  MakeAlphaNonCritical(options);
  const std::string alpha =
      WriteStub("exec_failure_alpha", "chmod a-x \"$0\"\nexit 3\n");
  options.binary_overrides["//example/alpha:node"] = alpha;
  options.binary_overrides["//example/beta:node"] =
      WriteStub("exec_failure_beta", kRunUntilStopped);
  const auto log = TempPath("exec_failure_stderr");
  const pid_t launcher = ::fork();
  ASSERT_GE(launcher, 0);
  if (launcher == 0) {
    const int fd = ::open(log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fd < 0 || ::dup2(fd, STDERR_FILENO) < 0) ::_exit(100);
    ::_exit(Launcher{options}.Run());
  }
  // The launcher's line for the failed restart: alpha's binary and the errno.
  const std::string cause = std::strerror(EACCES);
  const auto report = [&] {
    std::istringstream lines{ReadFile(log)};
    for (std::string line; std::getline(lines, line);) {
      if (line.find(alpha) != std::string::npos &&
          line.find(cause) != std::string::npos)
        return line;
    }
    return std::string{};
  };
  std::string reported = report();
  for (const auto give_up = std::chrono::steady_clock::now() + 20s;
       reported.empty() && std::chrono::steady_clock::now() < give_up;
       reported = report())
    std::this_thread::sleep_for(10ms);
  ::kill(launcher, SIGTERM);
  while (::waitpid(launcher, nullptr, 0) < 0 && errno == EINTR) {
  }
  EXPECT_FALSE(reported.empty()) << ReadFile(log);
  const auto entry = ManifestEntry(
      ReadFile(options.output_dir + "/" + std::to_string(options.session_id) +
               "/manifest.json"),
      "alpha");
  EXPECT_NE(entry.find("\"restart_count\": 0,"), std::string::npos) << entry;
  EXPECT_NE(entry.find("\"recorder_failed\": true"), std::string::npos)
      << entry;
  std::filesystem::remove_all(options.output_dir);
  ::remove(log.c_str());
}

// A requested stop (SIGTERM to the launcher) restarts none of the members
// that exit 0 while stopping.
TEST(Supervision, RequestedStopDoesNotRestartCleanExits) {
  auto options = BaseOptions("clean_stop");
  const auto runs = TempPath("clean_stop_runs");
  ::remove(runs.c_str());
  // Every roster member must stay alive until shutdown. An exit-0 hardware
  // stub otherwise restarts and can receive SIGINT before exec, where the
  // default signal disposition correctly produces a nonzero session status.
  for (const std::string name : {"hardware_node", "alpha", "beta"}) {
    const std::string target = name == "hardware_node"
                                   ? "//talOS/bridge:hardware_node"
                                   : "//example/" + name + ":node";
    options.binary_overrides[target] =
        WriteStub("clean_stop_" + name,
                  "trap 'exit 0' INT TERM\n" +
                      RecordLogArgThen(runs, "while :; do sleep 0.05; done\n"),
                  name);
  }
  const pid_t launcher = ::fork();
  ASSERT_GE(launcher, 0);
  if (launcher == 0) ::_exit(Launcher{options}.Run());
  const auto give_up = std::chrono::steady_clock::now() + 10s;
  // A recorded log argument acknowledges an installed stop handler, not just
  // a fork. Wait for all members rather than guessing with a fixed sleep.
  while (ReadLines(runs).size() < 3 &&
         std::chrono::steady_clock::now() < give_up)
    std::this_thread::sleep_for(10ms);
  const bool ready = ReadLines(runs).size() == 3;
  ::kill(launcher, SIGTERM);
  int status = 0;
  while (::waitpid(launcher, &status, 0) < 0 && errno == EINTR) {
  }
  EXPECT_TRUE(ready) << "not every node installed its stop handler";
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  const std::string dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  auto recorded = ReadLines(runs);
  std::sort(recorded.begin(), recorded.end());
  EXPECT_EQ(recorded,
            (std::vector<std::string>{dir + "/alpha.tlog", dir + "/beta.tlog",
                                      dir + "/hardware_node.tlog"}));
  const auto manifest = ReadFile(dir + "/manifest.json");
  for (const auto* name : {"hardware_node", "alpha", "beta"}) {
    const auto entry = ManifestEntry(manifest, name);
    EXPECT_NE(entry.find("\"restart_count\": 0"), std::string::npos) << entry;
    EXPECT_NE(entry.find("\"exit_code\": 0"), std::string::npos) << entry;
  }
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

TEST(Supervision, MissingConfigFailsFastWithClearError) {
  LauncherOptions options;
  options.output_dir = TempPath("missing_cfg_out");
  options.config_path = TempPath("does_not_exist") + ".toml";
  Launcher launcher{options};
  EXPECT_EQ(launcher.Run(), 1);
  EXPECT_FALSE(std::filesystem::exists(options.output_dir + "/graph.json"));
}

// Registry rows held by live processes outside the session are never swept, so
// a roster that cannot get a row per node is refused before anything starts
// rather than run until its unregistered nodes are killed.
TEST(Supervision, RosterThatDoesNotFitTheRegistryIsRefused) {
  auto options = BaseOptions("registry_full");
  const auto runs = TempPath("registry_full_runs");
  ::remove(runs.c_str());
  for (const std::string name : {"alpha", "beta"}) {
    options.binary_overrides["//example/" + name + ":node"] = WriteStub(
        "registry_full_" + name, RecordLogArgThen(runs, "exit 0\n"), name);
  }
  // The roster registers three nodes (hardware_node, alpha, beta): one too
  // many once 30 of the 32 rows are held.
  std::vector<std::unique_ptr<introspect::NodeRegistration>> held;
  for (int i = 0; i < 30; ++i) {
    held.push_back(std::make_unique<introspect::NodeRegistration>(
        introspect::NodeRegistration::Identity{.name = "held" +
                                                       std::to_string(i)}));
  }
  EXPECT_EQ(RunSession(options), 1);
  EXPECT_TRUE(ReadLines(runs).empty());
  held.pop_back();
  options.output_dir = TempPath("registry_full");
  options.session_id = GenerateSessionId();
  options.duration_s = 2;
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_GE(ReadLines(runs).size(), 2u);
  std::filesystem::remove_all(TempPath("registry_full"));
  ::remove(runs.c_str());
}

TEST(Supervision, SessionsAreIsolatedAndCleanOnlyOwnedDirs) {
  const std::string sentinel = TempPath("sentinel");
  {
    std::ofstream out{sentinel};
    out << "do not touch\n";
  }

  LauncherOptions a = BaseOptions("iso_a_out");
  a.binary_overrides["//example/alpha:node"] =
      WriteStub("iso_a_alpha", "exit 0\n");
  a.binary_overrides["//example/beta:node"] =
      WriteStub("iso_a_beta", "exit 0\n");
  LauncherOptions b = BaseOptions("iso_b_out");
  b.binary_overrides["//example/alpha:node"] =
      WriteStub("iso_b_alpha", "exit 0\n");
  b.binary_overrides["//example/beta:node"] =
      WriteStub("iso_b_beta", "exit 0\n");

  EXPECT_EQ(RunSession(a), 0);
  EXPECT_EQ(RunSession(b), 0);
  EXPECT_TRUE(std::filesystem::exists(a.output_dir + "/manifest.json"));
  EXPECT_TRUE(std::filesystem::exists(b.output_dir + "/manifest.json"));
  // Distinct sessions: manifests name their own output dir, never each
  // other's, and neither run touched the sibling sentinel file.
  EXPECT_NE(ReadFile(a.output_dir + "/manifest.json").find(a.output_dir),
            std::string::npos);
  EXPECT_EQ(ReadFile(a.output_dir + "/manifest.json").find(b.output_dir),
            std::string::npos);
  EXPECT_EQ(ReadFile(sentinel), "do not touch\n");

  std::filesystem::remove_all(a.output_dir);
  std::filesystem::remove_all(b.output_dir);
  ::remove(sentinel.c_str());
}

// A persistent fault is quarantined at the rolling limit, not retried in a
// tight loop. Stopping the session cancels its one pending retry. The session
// stops once alpha has stayed quarantined for 500 ms, not at a deadline, which
// a stalled host can reach before alpha's third run. The stubs never register,
// so their startup grace outlasts the 20 s give-up.
TEST(Supervision, CrashingNodeQuarantinesUntilTheSessionStops) {
  LauncherOptions options = BaseOptions("restart_out", "max_restarts = 2\n");
  MakeAlphaNonCritical(options);
  options.heartbeat_startup_grace_ms = 60'000;
  const std::string alpha_runs = TempPath("restart_alpha_runs");
  const std::string beta_runs = TempPath("restart_beta_runs");
  ::remove(alpha_runs.c_str());
  ::remove(beta_runs.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "restart_alpha", RecordLogArgThen(alpha_runs, "kill -KILL $$\n"));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("restart_beta", RecordLogArgThen(beta_runs, kRunUntilStopped));

  // RunSession rewrites options.output_dir on return; don't read it
  // concurrently.
  const auto live_manifest = options.output_dir + "/" +
                             std::to_string(options.session_id) +
                             "/manifest.json";
  bool quarantined = false;
  std::jthread stopper{[&](const std::stop_token& stop) {
    const auto give_up = std::chrono::steady_clock::now() + 20s;
    while (!stop.stop_requested() &&
           std::chrono::steady_clock::now() < give_up) {
      if (ManifestEntry(ReadFile(live_manifest), "alpha")
              .find("\"quarantined\": true") != std::string::npos) {
        quarantined = true;
        break;
      }
      std::this_thread::sleep_for(2ms);
    }
    if (quarantined) std::this_thread::sleep_for(500ms);
    if (!stop.stop_requested()) ::kill(::getpid(), SIGTERM);
  }};
  EXPECT_NE(RunSession(options), 0);
  stopper.request_stop();
  stopper.join();
  EXPECT_TRUE(quarantined);

  const std::string dir = options.output_dir;
  const std::vector<std::string> expected_logs{dir + "/alpha.tlog",
                                               dir + "/alpha.restart1.tlog",
                                               dir + "/alpha.restart2.tlog"};
  EXPECT_EQ(ReadLines(alpha_runs), expected_logs);
  EXPECT_EQ(ReadLines(beta_runs), std::vector<std::string>{dir + "/beta.tlog"});

  const std::string manifest = ReadFile(dir + "/manifest.json");
  const std::string alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"restart_count\": 2"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"log_path\": \"" + expected_logs[0] + "\""),
            std::string::npos)
      << alpha;
  EXPECT_NE(alpha.find("\"log_paths\": [\"" + expected_logs[0] + "\", \"" +
                       expected_logs[1] + "\", \"" + expected_logs[2] + "\"]"),
            std::string::npos)
      << alpha;
  const std::string beta = ManifestEntry(manifest, "beta");
  EXPECT_NE(beta.find("\"restart_count\": 0"), std::string::npos) << beta;
  EXPECT_NE(beta.find("\"log_paths\": [\"" + dir + "/beta.tlog\"]"),
            std::string::npos)
      << beta;

  std::filesystem::remove_all(dir);
  ::remove(alpha_runs.c_str());
  ::remove(beta_runs.c_str());
}

// --max-restarts 0 wins over the config and disables restarts.
TEST(Supervision, MaxRestartsZeroDisablesRestarts) {
  LauncherOptions options = BaseOptions("norestart_out", "max_restarts = 4\n");
  options.max_restarts = 0;
  const std::string alpha_runs = TempPath("norestart_alpha_runs");
  ::remove(alpha_runs.c_str());
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("norestart_alpha", RecordLogArgThen(alpha_runs, "exit 3\n"));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("norestart_beta", "exit 0\n");

  EXPECT_EQ(RunSession(options), 3);
  EXPECT_EQ(ReadLines(alpha_runs),
            std::vector<std::string>{options.output_dir + "/alpha.tlog"});
  const std::string alpha =
      ManifestEntry(ReadFile(options.output_dir + "/manifest.json"), "alpha");
  EXPECT_NE(alpha.find("\"restart_count\": 0"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"exit_code\": 3"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"quarantined\": false"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"retry_at_monotonic_ns\": 0"), std::string::npos)
      << alpha;

  std::filesystem::remove_all(options.output_dir);
  ::remove(alpha_runs.c_str());
}

// A run a signal ended says which: exit_code -1 alone cannot tell a crash
// from a stop the node never handled, or from the launcher's own SIGKILL.
TEST(Supervision, SignalDeathRecordsTheSignal) {
  LauncherOptions options = BaseOptions("signal_death");
  options.max_restarts = 0;
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("signal_death_alpha", "kill -KILL $$\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("signal_death_beta", "exit 0\n");

  EXPECT_NE(RunSession(options), 0);
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  const std::string alpha = ManifestEntry(manifest, "alpha");
  EXPECT_NE(alpha.find("\"exit_code\": -1"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"exit_signal\": " + std::to_string(SIGKILL)),
            std::string::npos)
      << alpha;
  const std::string beta = ManifestEntry(manifest, "beta");
  EXPECT_NE(beta.find("\"exit_signal\": 0"), std::string::npos) << beta;
  std::filesystem::remove_all(options.output_dir);
}

// A node's output never waits for the launcher's stdout: alpha writes 2 MiB
// to stdout while the launcher's stdout is a pipe nobody reads, then records
// that it got past the write. Inherited, that stdout would block it at 64 KiB.
// Its output lands in alpha.log after the line naming the run, all of it or
// with what was dropped marked.
TEST(Supervision, NodeOutputNeverWaitsForTheLaunchersStdout) {
  auto options = BaseOptions("output_drain");
  const std::string written = TempPath("output_drain_written");
  ::remove(written.c_str());
  constexpr int kBytes = 2 * 1024 * 1024;
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("output_drain_hardware", kRunUntilStopped, "hardware_node");
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "output_drain_alpha", "head -c " + std::to_string(kBytes) +
                                " /dev/zero | tr '\\0' x\necho done > " +
                                written + "\n" + std::string{kRunUntilStopped});
  options.binary_overrides["//example/beta:node"] =
      WriteStub("output_drain_beta", kRunUntilStopped);
  int launcher_stdout[2];
  ASSERT_EQ(::pipe(launcher_stdout), 0);
  const pid_t launcher = ::fork();
  ASSERT_GE(launcher, 0);
  if (launcher == 0) {
    ::close(launcher_stdout[0]);
    if (::dup2(launcher_stdout[1], STDOUT_FILENO) < 0) ::_exit(100);
    ::close(launcher_stdout[1]);
    ::_exit(Launcher{options}.Run());
  }
  ::close(launcher_stdout[1]);
  bool got_past = false;
  for (const auto give_up = std::chrono::steady_clock::now() + 20s;
       !got_past && std::chrono::steady_clock::now() < give_up;) {
    std::this_thread::sleep_for(20ms);
    got_past = std::filesystem::exists(written);
  }
  // Now let the launcher's own lines out, so that it can stop.
  std::thread drain{[fd = launcher_stdout[0]] {
    char bytes[4096];
    while (::read(fd, bytes, sizeof bytes) > 0) {
    }
  }};
  ::kill(launcher, SIGTERM);
  while (::waitpid(launcher, nullptr, 0) < 0 && errno == EINTR) {
  }
  drain.join();
  ::close(launcher_stdout[0]);
  EXPECT_TRUE(got_past) << "alpha blocked on its output";
  const std::string log =
      ReadFile(options.output_dir + "/" + std::to_string(options.session_id) +
               "/alpha.log");
  EXPECT_EQ(log.rfind("[launcher] alpha run 0, pid ", 0), 0u)
      << log.substr(0, 200);
  auto accounted = std::count(log.begin(), log.end(), 'x');
  const std::regex dropped{"\\[launcher\\] ([0-9]+) bytes of output dropped"};
  for (std::sregex_iterator it{log.begin(), log.end(), dropped}, end; it != end;
       ++it)
    accounted += std::stol((*it)[1].str());
  EXPECT_EQ(accounted, kBytes);
  std::filesystem::remove_all(options.output_dir);
  ::remove(written.c_str());
}

// A run's last output stays under its own header when the next run starts
// at once: the old pipe still holds "last words" when the restart's pipe is
// added, and the reader moves them out before writing the new header.
TEST(Supervision, CrashOutputIsNotCreditedToTheRestart) {
  const std::string path = TempPath("output_restart.log");
  ::remove(path.c_str());
  int first[2];
  int second[2];
  ASSERT_EQ(::pipe(first), 0);
  ASSERT_EQ(::pipe(second), 0);
  const std::string last_words = "last words\n";
  ASSERT_EQ(::write(first[1], last_words.data(), last_words.size()),
            static_cast<ssize_t>(last_words.size()));
  {
    ChildOutput output;
    output.Add(first[0], path, "alpha", 100, false);
    output.Add(second[0], path, "alpha", 101, false);
    ::close(first[1]);
    ::close(second[1]);
    output.Stop();
  }
  EXPECT_EQ(ReadFile(path),
            "[launcher] alpha run 0, pid 100\nlast words\n"
            "[launcher] alpha run 1, pid 101\n");
  ::remove(path.c_str());
}

// A negative [robot] max_restarts is a config error, caught before launch.
TEST(Supervision, NegativeMaxRestartsRefusesToLaunch) {
  LauncherOptions options =
      BaseOptions("badrestart_out", "max_restarts = -1\n");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("badrestart_alpha", "exit 0\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("badrestart_beta", "exit 0\n");
  EXPECT_EQ(RunSession(options), 1);
  EXPECT_FALSE(std::filesystem::exists(options.output_dir + "/manifest.json"));
  std::filesystem::remove_all(options.output_dir);
}

TEST(Supervision, RestartsOutsideRollingWindowDoNotExhaustBudget) {
  auto options = BaseOptions("window_out", "max_restarts = 1\n");
  MakeAlphaNonCritical(options);
  options.restart_window_ms = 100;
  const auto runs = TempPath("window_runs");
  ::remove(runs.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "window_alpha",
      RecordLogArgThen(runs,
                       "case \"$log\" in *restart2.tlog) exit 0;; esac\n"
                       "sleep 0.2\nexit 3\n"));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("window_beta", "exit 0\n");
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_EQ(ReadLines(runs).size(), 3u);
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// Quarantine needs both crashes inside one window, and a restart that has left
// the window no longer counts, so the window holds two restart-to-crash cycles
// of a stalled launcher (under TSan on CI one cycle can exceed 300 ms).
TEST(Supervision, ExhaustedBudgetRetriesOnceAfterTheWindowAndRecovers) {
  auto options = BaseOptions("quarantine_out", "max_restarts = 1\n");
  MakeAlphaNonCritical(options);
  options.restart_window_ms = 2000;
  options.duration_s = 6;
  const auto runs = TempPath("quarantine_runs");
  const auto cleared = TempPath("quarantine_cleared");
  ::remove(runs.c_str());
  ::remove(cleared.c_str());
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("quarantine_alpha",
                RecordLogArgThen(runs, "if [ -f " + cleared +
                                           " ]; then exit 0; fi\nexit 3\n"));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("quarantine_beta", "exit 0\n");
  std::string quarantined;
  std::size_t runs_at_quarantine = 0;
  int64_t retry_at = 0;
  int64_t recovered_at = 0;
  // RunSession rewrites options.output_dir on return; don't read it
  // concurrently.
  const auto manifest = options.output_dir + "/" +
                        std::to_string(options.session_id) + "/manifest.json";
  std::jthread observer{[&](const std::stop_token& stop) {
    while (!stop.stop_requested()) {
      auto entry = ManifestEntry(ReadFile(manifest), "alpha");
      if (entry.find("\"quarantined\": true") != std::string::npos) {
        quarantined = std::move(entry);
        runs_at_quarantine = ReadLines(runs).size();
        const std::string key = "\"retry_at_monotonic_ns\": ";
        const auto at = quarantined.find(key);
        if (at != std::string::npos)
          retry_at = std::stoll(quarantined.substr(at + key.size()));
        std::ofstream{cleared} << "fault cleared\n";
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    for (;;) {
      if (ReadLines(runs).size() >= 3) {
        recovered_at = introspect::monotonic_now_ns();
        return;
      }
      if (stop.stop_requested()) return;
      std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
  }};
  const auto session_started_at = introspect::monotonic_now_ns();
  EXPECT_EQ(RunSession(options), 0);
  observer.request_stop();
  observer.join();
  EXPECT_FALSE(quarantined.empty());
  EXPECT_EQ(runs_at_quarantine, 2u);
  EXPECT_GT(retry_at, 0);
  // Initial 50 ms backoff, the 2 s rolling window, then normal 100 ms
  // backoff for the second restart; quarantine cannot turn into a quick retry.
  EXPECT_GE(retry_at, session_started_at + 2'150'000'000);
  EXPECT_GE(recovered_at, retry_at);
  EXPECT_EQ(ReadLines(runs).size(), 3u);
  const auto alpha =
      ManifestEntry(ReadFile(options.output_dir + "/manifest.json"), "alpha");
  EXPECT_NE(alpha.find("\"restart_count\": 2"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"quarantined\": false"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"retry_at_monotonic_ns\": 0"), std::string::npos)
      << alpha;
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
  ::remove(cleared.c_str());
}

TEST(Supervision, StalledNodeGroupKillStopsItsHelper) {
  auto options = BaseOptions("helper_out");
  options.max_restarts = 0;
  options.heartbeat_startup_grace_ms = 0;
  options.duration_s = 3;
  const auto helper_pid = TempPath("helper_pid");
  int lifetime[2];
  ASSERT_EQ(::pipe(lifetime), 0);
  // The helper inherits this writer across exec. EOF proves it stopped, even
  // on hosts where an orphaned helper briefly remains as a zombie.
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "helper_alpha", "sleep 30 &\necho $! > " + helper_pid + "\nwait\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("helper_beta", "exit 0\n");
  EXPECT_NE(RunSession(options), 0);
  ::close(lifetime[1]);
  pollfd ready{lifetime[0], POLLIN, 0};
  const int result = ::poll(&ready, 1, 1000);
  char byte = 0;
  EXPECT_GT(result, 0);
  if (result > 0) EXPECT_EQ(::read(lifetime[0], &byte, 1), 0);
  ::close(lifetime[0]);
  pid_t helper = 0;
  std::ifstream{helper_pid} >> helper;
  EXPECT_GT(helper, 0);
  // Also clean up on a regression, rather than leaking the fixture's helper.
  if (helper > 0) ::kill(helper, SIGKILL);
  std::filesystem::remove_all(options.output_dir);
  ::remove(helper_pid.c_str());
}

TEST(Supervision, KillingLauncherKillsNodeAndItsHelper) {
  int output[2];
  ASSERT_EQ(::pipe(output), 0);
  const pid_t launcher = ::fork();
  ASSERT_GE(launcher, 0);
  if (launcher == 0) {
    ::close(output[0]);
    std::string error;
    if (::dup2(output[1], STDOUT_FILENO) < 0) ::_exit(1);
    ::close(output[1]);
    const auto node =
        SpawnProcess({"/bin/sh", "-c", "sleep 30 & echo $$; wait"}, error);
    if (node < 0) ::_exit(1);
    for (;;) ::pause();
  }
  ::close(output[1]);
  pollfd ready{output[0], POLLIN, 0};
  const int started = ::poll(&ready, 1, 3000);
  char text[64]{};
  ssize_t size = 0;
  if (started > 0) size = ::read(output[0], text, sizeof(text) - 1);
  ::kill(launcher, SIGKILL);
  int status = 0;
  while (::waitpid(launcher, &status, 0) < 0 && errno == EINTR) {
  }
  const int stopped = ::poll(&ready, 1, 3000);
  char byte = 0;
  EXPECT_GT(started, 0);
  EXPECT_GT(size, 0);
  EXPECT_GT(stopped, 0);
  if (stopped > 0) EXPECT_EQ(::read(output[0], &byte, 1), 0);
  if (size > 0) {
    SignalProcessGroup(static_cast<pid_t>(std::strtol(text, nullptr, 10)),
                       SIGKILL);
  }
  ::close(output[0]);
}

// A guardian never execs, so a descriptor its launcher's process held at the
// spawn would stay open in it for as long as the node runs, close-on-exec or
// not: another thread's registry lock (close-on-exec, like this one) held that
// way would block every later registration.
TEST(Supervision, GuardianKeepsNoLockHeldAtSpawn) {
  const std::string path = TempPath("guardian_lock");
  const int held = ::open(path.c_str(), O_CREAT | O_RDONLY | O_CLOEXEC, 0600);
  const int probe = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  ASSERT_GE(held, 0);
  ASSERT_GE(probe, 0);
  ASSERT_EQ(::flock(held, LOCK_EX), 0);
  std::string error;
  const pid_t node = SpawnProcess({"/bin/sh", "-c", "exec sleep 30"}, error);
  ::close(held);
  ASSERT_GT(node, 0) << error;
  // The node's own copy closes at its exec; then nothing may hold the lock.
  bool released = ::flock(probe, LOCK_EX | LOCK_NB) == 0;
  for (const auto give_up = std::chrono::steady_clock::now() + 5s;
       !released && std::chrono::steady_clock::now() < give_up;) {
    std::this_thread::sleep_for(10ms);
    released = ::flock(probe, LOCK_EX | LOCK_NB) == 0;
  }
  EXPECT_TRUE(released) << "the node's guardian still holds the lock";
  ::close(probe);
  SignalProcessGroup(node, SIGKILL);
  while (::waitpid(node, nullptr, 0) < 0 && errno == EINTR) {
  }
  FinishProcessGroup(node);
  ::remove(path.c_str());
}

// The real supervision loop, not a watchdog thread, must deliver READY before
// its first WATCHDOG. A hung loop consequently cannot keep the unit healthy.
TEST(Supervision, SendsReadyThenWatchdogDatagrams) {
  const std::string path = "/tmp/tn_" + std::to_string(::getpid());
  ::unlink(path.c_str());
  const int socket = ::socket(AF_UNIX, SOCK_DGRAM, 0);
  ASSERT_GE(socket, 0);
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
  ASSERT_EQ(::bind(socket, reinterpret_cast<const sockaddr*>(&address),
                   static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) +
                                          path.size() + 1)),
            0);
  auto options = BaseOptions("notify_out");
  options.max_restarts = 0;
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("notify_alpha", "sleep 0.1\nexit 0\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("notify_beta", "exit 0\n");
  const pid_t launcher = ::fork();
  ASSERT_GE(launcher, 0);
  if (launcher == 0) {
    ::close(socket);
    ::setenv("NOTIFY_SOCKET", path.c_str(), 1);
    ::_exit(Launcher{options}.Run());
  }
  std::vector<std::string> messages;
  for (int i = 0; i < 2; ++i) {
    pollfd ready{socket, POLLIN, 0};
    if (::poll(&ready, 1, 10000) <= 0) break;
    char buffer[64];
    const auto size = ::recv(socket, buffer, sizeof(buffer), 0);
    if (size > 0) messages.emplace_back(buffer, static_cast<std::size_t>(size));
  }
  int status = 0;
  while (::waitpid(launcher, &status, 0) < 0 && errno == EINTR) {
  }
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  EXPECT_EQ(messages, (std::vector<std::string>{"READY=1", "WATCHDOG=1"}));
  ::close(socket);
  ::unlink(path.c_str());
  std::filesystem::remove_all(options.output_dir);
}

#ifdef __linux__
TEST(Supervision, NotifierSupportsAbstractSocketNames) {
  // Abstract names live in the network namespace, which concurrent sandboxed
  // runs of this test share while their pids repeat across PID namespaces.
  const std::string path = "@tn_" + std::to_string(GenerateSessionId());
  const int socket = ::socket(AF_UNIX, SOCK_DGRAM, 0);
  ASSERT_GE(socket, 0);
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.data(), path.size());
  address.sun_path[0] = '\0';
  ASSERT_EQ(::bind(socket, reinterpret_cast<const sockaddr*>(&address),
                   static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) +
                                          path.size())),
            0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(socket);
    ::setenv("NOTIFY_SOCKET", path.c_str(), 1);
    ServiceNotifier notifier;
    notifier.Send("WATCHDOG=1");
    ::_exit(0);
  }
  pollfd ready{socket, POLLIN, 0};
  const int received = ::poll(&ready, 1, 1000);
  char buffer[64]{};
  ssize_t size = 0;
  if (received > 0) size = ::recv(socket, buffer, sizeof(buffer), 0);
  int status = 0;
  ::waitpid(child, &status, 0);
  EXPECT_EQ(
      std::string(buffer, static_cast<std::size_t>(std::max<ssize_t>(size, 0))),
      "WATCHDOG=1");
  ::close(socket);
}
#endif

// The registry lock is a node's while it creates or claims in the registry,
// and a node can hold it for good: stopped or stuck mid-registration. The
// launcher's loop, which feeds the service watchdog, must not wait on it.
// With the lock held all session, every member starts and the session still
// ends at its deadline.
TEST(Supervision, HeldRegistryLockDoesNotStallSupervision) {
  auto options = BaseOptions("held_lock");
  options.duration_s = 2;
  const auto runs = TempPath("held_lock_runs");
  ::remove(runs.c_str());
  for (const std::string name : {"hardware_node", "alpha", "beta"}) {
    const std::string target = name == "hardware_node"
                                   ? "//talOS/bridge:hardware_node"
                                   : "//example/" + name + ":node";
    options.binary_overrides[target] =
        WriteStub("held_lock_" + name,
                  "trap 'exit 0' INT TERM\n" +
                      RecordLogArgThen(runs, "while :; do sleep 0.05; done\n"),
                  name);
  }
  // The launcher waits on this lock at startup only to create the registry
  // (PrepareRegistry); here it exists, as after any earlier session.
  introspect::PrepareRegistry();
  const introspect::RegistryLock held{ShmObjectName(introspect::kRegistryPath)};
  const pid_t launcher = ::fork();
  ASSERT_GE(launcher, 0);
  if (launcher == 0) ::_exit(Launcher{options}.Run());
  int status = 0;
  pid_t reaped = 0;
  const auto give_up = std::chrono::steady_clock::now() + 20s;
  while ((reaped = ::waitpid(launcher, &status, WNOHANG)) == 0 &&
         std::chrono::steady_clock::now() < give_up)
    std::this_thread::sleep_for(10ms);
  if (reaped != launcher) {
    ::kill(launcher, SIGKILL);
    while (::waitpid(launcher, &status, 0) < 0 && errno == EINTR) {
    }
  }
  EXPECT_EQ(reaped, launcher) << "supervision stalled on the registry lock";
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  EXPECT_EQ(ReadLines(runs).size(), 3u);
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

// One session per shared-memory namespace: a second launcher would unlink the
// running session's topics. It is refused before it writes or starts
// anything, naming the holder; --describe-only, which starts nothing, still
// runs; and the lock dies with its holder.
TEST(Supervision, SecondLaunchInTheNamespaceIsRefused) {
  int ready[2];
  ASSERT_EQ(::pipe(ready), 0);
  const pid_t holder = ::fork();
  ASSERT_GE(holder, 0);
  if (holder == 0) {
    ::close(ready[0]);
    std::string error;
    const auto lock = SessionLock::Acquire(error);
    const char held = lock ? 1 : 0;
    if (::write(ready[1], &held, 1) != 1 || !lock) ::_exit(1);
    for (;;) ::pause();
  }
  ::close(ready[1]);
  char held = 0;
  const bool reported = ::read(ready[0], &held, 1) == 1;
  ::close(ready[0]);
  const auto with_stubs = [](std::string_view tag, bool describe_only) {
    auto options = BaseOptions(tag);
    options.describe_only = describe_only;
    options.binary_overrides["//example/alpha:node"] =
        WriteStub(std::string{tag} + "_alpha", "exit 0\n");
    options.binary_overrides["//example/beta:node"] =
        WriteStub(std::string{tag} + "_beta", "exit 0\n");
    return options;
  };
  auto second = with_stubs("second_launch", false);
  auto describe = with_stubs("second_describe", true);
  ::testing::internal::CaptureStderr();
  const int refused = RunSession(second);
  const std::string error = ::testing::internal::GetCapturedStderr();
  const int described = RunSession(describe);
  ::kill(holder, SIGKILL);
  while (::waitpid(holder, nullptr, 0) < 0 && errno == EINTR) {
  }
  ASSERT_TRUE(reported && held == 1);
  EXPECT_EQ(refused, 1);
  EXPECT_NE(error.find("pid " + std::to_string(holder)), std::string::npos)
      << error;
  EXPECT_FALSE(std::filesystem::exists(second.output_dir));
  EXPECT_EQ(described, 0);
  auto after = with_stubs("after_holder", false);
  EXPECT_EQ(RunSession(after), 0);
  for (const auto* options : {&second, &describe, &after}) {
    std::filesystem::remove_all(options->output_dir);
  }
}

// The loop never waits for the disk, yet every mid-session manifest is
// synced. With every sync stuck from alpha's first run on, the critical
// alpha's crash is still restarted at once; once the disk recovers, a synced
// manifest of the running session records the restart (PersistManifest,
// ManifestWriter).
TEST(Supervision, StuckManifestSyncDelaysNoRestartAndSyncsItLater) {
  auto options = BaseOptions("stuck_sync");
  // The stubs never register; no heartbeat kill may restart them meanwhile.
  options.heartbeat_startup_grace_ms = 60'000;
  const auto runs = TempPath("stuck_sync_runs");
  ::remove(runs.c_str());
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      WriteStub("stuck_sync_hardware", kRunUntilStopped, "hardware_node");
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "stuck_sync_alpha",
      RecordLogArgThen(runs, "case \"$log\" in *alpha.tlog) exit 3;; esac\n" +
                                 std::string{kRunUntilStopped}));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("stuck_sync_beta", kRunUntilStopped);
  // RunSession rewrites options.output_dir on return; don't read it
  // concurrently.
  const auto live_manifest = options.output_dir + "/" +
                             std::to_string(options.session_id) +
                             "/manifest.json";
  std::mutex mutex;
  std::condition_variable release;
  bool released = false;
  std::atomic<bool> synced_restart{false};
  options.sync_file = [&](int fd) {
    if (!ReadLines(runs).empty()) {
      std::unique_lock lock{mutex};
      release.wait(lock, [&] { return released; });
    }
    const int result = ::fsync(fd);
    // The directory is synced after the rename: manifest.json is the
    // version just made durable.
    struct stat status{};
    if (::fstat(fd, &status) == 0 && S_ISDIR(status.st_mode)) {
      const std::string text = ReadFile(live_manifest);
      if (ManifestEntry(text, "alpha").find("\"restart_count\": 1,") !=
              std::string::npos &&
          text.find("\"end_wall_ns\": 0,") != std::string::npos)
        synced_restart = true;
    }
    return result;
  };
  bool restarted = false;
  std::jthread stopper{[&](const std::stop_token& stop) {
    const auto give_up = std::chrono::steady_clock::now() + 20s;
    while (!stop.stop_requested() &&
           std::chrono::steady_clock::now() < give_up) {
      if (ReadLines(runs).size() >= 2) {
        restarted = true;
        break;
      }
      std::this_thread::sleep_for(2ms);
    }
    {
      const std::lock_guard lock{mutex};
      released = true;
    }
    release.notify_all();
    while (!stop.stop_requested() && !synced_restart &&
           std::chrono::steady_clock::now() < give_up) {
      std::this_thread::sleep_for(2ms);
    }
    if (!stop.stop_requested()) ::kill(::getpid(), SIGTERM);
  }};
  RunSession(options);
  stopper.request_stop();
  stopper.join();
  EXPECT_TRUE(restarted) << "the restart waited for a manifest sync";
  EXPECT_TRUE(synced_restart)
      << "no manifest synced mid-session recorded the restart";
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

TEST(Supervision, StalledRegistryHeartbeatIsKilledAndRestarted) {
  auto options = BaseOptions("stalled_out", "max_restarts = 1\n");
  options.heartbeat_startup_grace_ms = 0;
  options.duration_s = 3;
  const auto pid_file = TempPath("stalled_pid");
  const auto runs = TempPath("stalled_runs");
  ::remove(pid_file.c_str());
  ::remove(runs.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "stalled_alpha",
      RecordLogArgThen(runs,
                       "case \"$log\" in *restart1.tlog) exit 0;; esac\n"
                       "echo $$ > " +
                           pid_file + "\nexec sleep 10\n"));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("stalled_beta", "exit 0\n");
  // Supply the stub's registry row from the fixture, without giving the
  // launcher a writable mapping or requiring a real framework node binary.
  std::jthread publisher{[&](const std::stop_token& stop) {
    pid_t pid = 0;
    while (!stop.stop_requested()) {
      std::ifstream input{pid_file};
      if (input >> pid) break;
      std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }
    if (pid <= 0) return;
    introspect::NodeRegistration registration{
        {"alpha", "//example/alpha:node", options.session_id, 0}};
    introspect::RegistryMapping mapping;
    for (auto& row : mapping.header()->nodes) {
      if (row.state.load(std::memory_order_acquire) !=
          static_cast<uint64_t>(introspect::SlotState::ACTIVE)) {
        continue;
      }
      if (row.session_id == options.session_id &&
          row.pid == static_cast<uint64_t>(::getpid())) {
        row.state.store(static_cast<uint64_t>(introspect::SlotState::CLAIMING),
                        std::memory_order_release);
        row.pid = static_cast<uint64_t>(pid);
        row.pid_start = 0;  // The stub's start is unknown: compare its pid.
        row.heartbeat_monotonic_ns.store(introspect::monotonic_now_ns());
        row.dispatch_count.store(1);
        row.state.store(static_cast<uint64_t>(introspect::SlotState::ACTIVE),
                        std::memory_order_release);
        break;
      }
    }
    while (!stop.stop_requested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }
  }};
  EXPECT_EQ(RunSession(options), 0);
  publisher.request_stop();
  publisher.join();
  EXPECT_EQ(ReadLines(runs).size(), 2u);
  const auto alpha =
      ManifestEntry(ReadFile(options.output_dir + "/manifest.json"), "alpha");
  EXPECT_NE(alpha.find("\"restart_count\": 1"), std::string::npos);
  std::filesystem::remove_all(options.output_dir);
  ::remove(pid_file.c_str());
  ::remove(runs.c_str());
}

TEST(Supervision, NeverRegisteredNodeIsKilledAndRestarted) {
  auto options = BaseOptions("unregistered_out", "max_restarts = 1\n");
  options.heartbeat_startup_grace_ms = 0;
  options.duration_s = 3;
  const auto runs = TempPath("unregistered_runs");
  ::remove(runs.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "unregistered_alpha",
      RecordLogArgThen(runs,
                       "case \"$log\" in *restart1.tlog) exit 0;; esac\n"
                       "exec sleep 10\n"));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("unregistered_beta", "exit 0\n");
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_EQ(ReadLines(runs), (std::vector<std::string>{
                                 options.output_dir + "/alpha.tlog",
                                 options.output_dir + "/alpha.restart1.tlog"}));
  const auto alpha =
      ManifestEntry(ReadFile(options.output_dir + "/manifest.json"), "alpha");
  EXPECT_NE(alpha.find("\"restart_count\": 1"), std::string::npos) << alpha;
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

TEST(Supervision, UndescribedNodeAllowedExplicitlyStillMustRegister) {
  auto options = BaseOptions("undescribed_out", "max_restarts = 1\n");
  options.allow_graph_errors = true;
  options.heartbeat_startup_grace_ms = 0;
  options.duration_s = 3;
  const auto runs = TempPath("undescribed_runs");
  ::remove(runs.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "undescribed_alpha",
      RecordLogArgThen(runs,
                       "case \"$log\" in *restart1.tlog) exit 0;; esac\n"
                       "exec sleep 10\n"),
      "alpha", false);
  options.binary_overrides["//example/beta:node"] =
      WriteStub("undescribed_beta", "exit 0\n");
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_EQ(ReadLines(runs).size(), 2u);
  const auto alpha =
      ManifestEntry(ReadFile(options.output_dir + "/manifest.json"), "alpha");
  EXPECT_NE(alpha.find("\"described\": false"), std::string::npos) << alpha;
  EXPECT_NE(alpha.find("\"restart_count\": 1"), std::string::npos) << alpha;
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

TEST(Supervision, SimGatewayDoesNotRequireFrameworkRegistration) {
  auto options = BaseOptions(
      "gateway_out",
      "max_restarts = 1\nsim_gateway = \"//example/sim_gateway:node\"\n");
  options.simulation = true;
  options.start_sim_gateway = true;
  options.heartbeat_startup_grace_ms = 0;
  options.duration_s = 3;
  const auto runs = TempPath("gateway_runs");
  ::remove(runs.c_str());
  options.binary_overrides["//example/sim_gateway:node"] = WriteStub(
      "gateway", "echo run >> " + runs + "\nexec sleep 1\n", "sim_gateway");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("gateway_alpha", "exit 0\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("gateway_beta", "exit 0\n");
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_EQ(ReadLines(runs).size(), 1u);
  const auto gateway = ManifestEntry(
      ReadFile(options.output_dir + "/manifest.json"), "sim_gateway");
  EXPECT_NE(gateway.find("\"described\": true"), std::string::npos) << gateway;
  EXPECT_NE(gateway.find("\"restart_count\": 0"), std::string::npos) << gateway;
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

}  // namespace
}  // namespace talos::launcher
