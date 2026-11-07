// Launcher supervision: clear failures, isolated sessions, and exit-code
// propagation from critical nodes.
//
// Every session runs in a pid-unique temp directory with shell-stub nodes,
// so no two tests share state, ports, or shared memory. Stubs describe an
// empty graph unless a test explicitly exercises a failed probe.

#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/launcher/launcher.h"

namespace talos::launcher {
namespace {

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
        << introspect::DescribeToJson(description)
        << "TALOS_DESCRIBE\n"
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

// A node that keeps crashing is restarted up to [robot] max_restarts, each run
// into its own log, and the manifest says so. Its clean-exiting sibling is not
// restarted.
TEST(Supervision, CrashingNodeIsRestartedUpToTheLimit) {
  LauncherOptions options = BaseOptions("restart_out", "max_restarts = 2\n");
  const std::string alpha_runs = TempPath("restart_alpha_runs");
  const std::string beta_runs = TempPath("restart_beta_runs");
  ::remove(alpha_runs.c_str());
  ::remove(beta_runs.c_str());
  options.binary_overrides["//example/alpha:node"] = WriteStub(
      "restart_alpha", RecordLogArgThen(alpha_runs, "kill -KILL $$\n"));
  options.binary_overrides["//example/beta:node"] =
      WriteStub("restart_beta", RecordLogArgThen(beta_runs, "exit 0\n"));

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
        row.heartbeat_wall_ns.store(introspect::wall_now_ns());
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
  EXPECT_EQ(ReadLines(runs),
            (std::vector<std::string>{options.output_dir + "/alpha.tlog",
                                      options.output_dir +
                                          "/alpha.restart1.tlog"}));
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
  options.binary_overrides["//example/sim_gateway:node"] =
      WriteStub("gateway", "echo run >> " + runs + "\nexec sleep 1\n",
                "sim_gateway");
  options.binary_overrides["//example/alpha:node"] =
      WriteStub("gateway_alpha", "exit 0\n");
  options.binary_overrides["//example/beta:node"] =
      WriteStub("gateway_beta", "exit 0\n");
  EXPECT_EQ(RunSession(options), 0);
  EXPECT_EQ(ReadLines(runs).size(), 1u);
  const auto gateway =
      ManifestEntry(ReadFile(options.output_dir + "/manifest.json"),
                    "sim_gateway");
  EXPECT_NE(gateway.find("\"described\": true"), std::string::npos) << gateway;
  EXPECT_NE(gateway.find("\"restart_count\": 0"), std::string::npos) << gateway;
  std::filesystem::remove_all(options.output_dir);
  ::remove(runs.c_str());
}

}  // namespace
}  // namespace talos::launcher
