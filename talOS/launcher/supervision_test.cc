// Launcher supervision: clear failures, isolated sessions, and exit-code
// propagation from critical nodes.
//
// Every session runs in a pid-unique temp directory with shell-stub nodes,
// so no two tests share state, ports, or shared memory. Stubs describe an
// empty graph unless a test explicitly exercises a failed probe.

#include <gtest/gtest.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

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

TEST(Supervision, CriticalCrashLoopExceedsBudgetWithBackoffCappedAtOneSecond) {
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
  const auto started = std::chrono::steady_clock::now();
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_EQ(ReadLines(runs).size(), 7u);
  EXPECT_LT(std::chrono::steady_clock::now() - started,
            std::chrono::seconds{5});
  for (int restart = 1; restart <= 100; ++restart) {
    EXPECT_LE(RestartBackoff(restart), std::chrono::milliseconds{1000});
  }
  EXPECT_EQ(RestartBackoff(6), std::chrono::milliseconds{1000});
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

// A requested stop (SIGTERM to the launcher) restarts none of the members
// that exit 0 while stopping.
TEST(Supervision, RequestedStopDoesNotRestartCleanExits) {
  auto options = BaseOptions("clean_stop");
  const auto runs = TempPath("clean_stop_runs");
  ::remove(runs.c_str());
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("clean_stop_alpha", RecordLogArgThen(runs, kRunUntilStopped));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("clean_stop_beta", kRunUntilStopped);
  const pid_t launcher = ::fork();
  ASSERT_GE(launcher, 0);
  if (launcher == 0) ::_exit(Launcher{options}.Run());
  const auto give_up = std::chrono::steady_clock::now() + 10s;
  while (ReadLines(runs).empty() && std::chrono::steady_clock::now() < give_up)
    std::this_thread::sleep_for(10ms);
  std::this_thread::sleep_for(200ms);
  ::kill(launcher, SIGTERM);
  int status = 0;
  while (::waitpid(launcher, &status, 0) < 0 && errno == EINTR) {
  }
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  const std::string dir =
      options.output_dir + "/" + std::to_string(options.session_id);
  EXPECT_EQ(ReadLines(runs), std::vector<std::string>{dir + "/alpha.tlog"});
  const auto alpha = ManifestEntry(ReadFile(dir + "/manifest.json"), "alpha");
  EXPECT_NE(alpha.find("\"restart_count\": 0"), std::string::npos) << alpha;
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

  // Cleanup removes only what the test owned.
  std::filesystem::remove_all(a.output_dir);
  std::filesystem::remove_all(b.output_dir);
  EXPECT_TRUE(std::filesystem::exists(sentinel));
  EXPECT_FALSE(std::filesystem::exists(a.output_dir));
  ::remove(sentinel.c_str());
}

TEST(Supervision, CriticalFailurePropagatesExitCode) {
  LauncherOptions options = BaseOptions("crit_out");
  options.max_restarts = 0;
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("crit_alpha", "exit 3\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("crit_beta", "exit 0\n");
  EXPECT_EQ(RunSession(options), 3);
  const std::string manifest = ReadFile(options.output_dir + "/manifest.json");
  EXPECT_NE(manifest.find("\"name\": \"alpha\""), std::string::npos);
  EXPECT_NE(manifest.find("\"exit_code\": 3"), std::string::npos) << manifest;
  std::filesystem::remove_all(options.output_dir);
}

// A persistent fault is quarantined at the rolling limit, not retried in a
// tight loop. Stopping the session cancels its one pending retry.
TEST(Supervision, CrashingNodeQuarantinesUntilTheSessionStops) {
  LauncherOptions options = BaseOptions("restart_out", "max_restarts = 2\n");
  MakeAlphaNonCritical(options);
  options.duration_s = 1;
  const std::string alpha_runs = TempPath("restart_alpha_runs");
  const std::string beta_runs = TempPath("restart_beta_runs");
  ::remove(alpha_runs.c_str());
  ::remove(beta_runs.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "restart_alpha", RecordLogArgThen(alpha_runs, "kill -KILL $$\n"));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("restart_beta", RecordLogArgThen(beta_runs, kRunUntilStopped));

  EXPECT_NE(RunSession(options), 0);

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

TEST(Supervision, ExhaustedBudgetRetriesOnceAfterTheWindowAndRecovers) {
  auto options = BaseOptions("quarantine_out", "max_restarts = 1\n");
  MakeAlphaNonCritical(options);
  options.restart_window_ms = 300;
  options.duration_s = 2;
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
  // Initial 50 ms backoff, the 300 ms rolling window, then normal 100 ms
  // backoff for the second restart; quarantine cannot turn into a quick retry.
  EXPECT_GE(retry_at, session_started_at + 450'000'000);
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
    const auto node = SpawnProcess(
        {"/bin/sh", "-c", "sleep 30 & echo $$; wait"}, "", output[1]);
    ::close(output[1]);
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
  const std::string path = "@tn_" + std::to_string(::getpid());
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
