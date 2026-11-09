#pragma once

#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#ifdef __linux__
#include <sys/prctl.h>
#endif

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/events/context.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/introspection/registry.h"
#include "talOS/launcher/service_notifier.h"
#include "talOS/rtms/rtms.h"
#include "talOS/schema/unit_suffix.h"
#include "toml.hpp"

namespace talos::launcher {

// The launcher works in the vocabulary of the naming protocol, not its own.
namespace naming = introspect::naming;

inline uint64_t GenerateSessionId() {
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
  std::random_device rd;
  const uint64_t r = (static_cast<uint64_t>(rd()) << 32) | rd();
  const uint64_t id = static_cast<uint64_t>(now) ^ r;
  return id == 0 ? 1 : id;
}

inline std::string Quoted(std::string_view value) {
  std::string out;
  introspect::AppendJsonString(out, value);
  return out;
}

struct NodeSpec {
  std::string name;
  std::string target;  // e.g. "//2026-robot/main_processor/drivetrain:node"
  std::string binary_path;
  // Forwarded unchanged to both --describe and runtime.
  std::vector<std::string> extra_args;
  bool critical{true};
  bool owns_actuators{false};
};

struct NodeProcessInfo {
  std::string name;
  std::string target;
  std::string binary_path;
  std::string log_path;  // First run's event log; empty for none.
  // Every event log the node wrote, first run first: log_path, then
  // `<name>.restart<k>.tlog` for restart k. Empty when log_path is.
  std::vector<std::string> log_paths;
  int restart_count{0};
  pid_t pid{-1};
  int exit_code{-1};
  bool exited{false};
  bool quarantined{false};
  int64_t retry_at_monotonic_ns{0};  // Zero when no restart is pending.
  bool failed{false};
  // A run's event log reached its share of the session log budget and stopped
  // early (`<log>.capped`): the recording is incomplete, not corrupt.
  bool log_capped{false};
  uint64_t record_count{0};
  // Whether this binary answered --describe: a session whose graph was never
  // checked otherwise reads exactly like one whose graph passed.
  bool described{false};
};

struct SessionManifest {
  uint64_t session_id{0};
  std::string config_path;
  // sha256 over every staged file as config-relative path + bytes, from the
  // bundle manifest. Empty for non-bundle launches.
  std::string config_sha256;
  std::string output_dir;
  bool simulation{false};
  int64_t start_wall_ns{0};
  int64_t end_wall_ns{0};
  std::size_t declared_topics{0};
  std::vector<NodeProcessInfo> nodes;

  std::string ToJson() const {
    std::ostringstream ss;
    ss << "{\n";
    ss << "  \"session_id\": " << session_id << ",\n";
    ss << "  \"config_path\": " << Quoted(config_path) << ",\n";
    ss << "  \"config_sha256\": " << Quoted(config_sha256) << ",\n";
    ss << "  \"output_dir\": " << Quoted(output_dir) << ",\n";
    ss << "  \"simulation\": " << (simulation ? "true" : "false") << ",\n";
    ss << "  \"start_wall_ns\": " << start_wall_ns << ",\n";
    ss << "  \"end_wall_ns\": " << end_wall_ns << ",\n";
    ss << "  \"nodes\": [\n";
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      const auto& n = nodes[i];
      ss << "    {\n";
      ss << "      \"name\": " << Quoted(n.name) << ",\n";
      ss << "      \"target\": " << Quoted(n.target) << ",\n";
      ss << "      \"binary_path\": " << Quoted(n.binary_path) << ",\n";
      ss << "      \"log_path\": " << Quoted(n.log_path) << ",\n";
      ss << "      \"log_paths\": [";
      for (std::size_t j = 0; j < n.log_paths.size(); ++j) {
        ss << (j ? ", " : "") << Quoted(n.log_paths[j]);
      }
      ss << "],\n";
      ss << "      \"pid\": " << n.pid << ",\n";
      ss << "      \"exit_code\": " << n.exit_code << ",\n";
      ss << "      \"restart_count\": " << n.restart_count << ",\n";
      ss << "      \"quarantined\": " << (n.quarantined ? "true" : "false")
         << ",\n";
      ss << "      \"retry_at_monotonic_ns\": " << n.retry_at_monotonic_ns
         << ",\n";
      ss << "      \"recorder_failed\": " << (n.failed ? "true" : "false")
         << ",\n";
      ss << "      \"log_capped\": " << (n.log_capped ? "true" : "false")
         << ",\n";
      ss << "      \"described\": " << (n.described ? "true" : "false")
         << ",\n";
      ss << "      \"record_count\": " << n.record_count << "\n";
      ss << "    }" << (i + 1 < nodes.size() ? "," : "") << "\n";
    }
    ss << "  ]\n";
    ss << "}\n";
    return ss.str();
  }
};

struct LauncherOptions {
  // No default: the framework names no robot, so the caller must say which
  // config to launch. Empty means missing.
  std::string config_path;
  std::string output_dir;
  uint64_t session_id{0};
  bool simulation{false};
  int duration_s{0};
  bool start_sim_gateway{false};
  std::vector<std::string> sim_gateway_args;
  // Sim sessions only: run the Studio viewer bridge as a supervised member of
  // the session (see "Studio bridge" below and [studio] in the robot config).
  bool start_studio_bridge{false};

  // Controller addressing. Empty/zero means "take it from [robot]".
  std::string remote;
  uint16_t remote_port{0};
  uint16_t local_port{0};

  // Studio HTTP port override. Zero means "take [studio] http_port".
  uint16_t studio_http_port{0};

  // Restart budget per node in restart_window_ms; unset takes
  // robot.max_restarts.
  std::optional<int> max_restarts;
  int restart_window_ms{60000};
  int heartbeat_periods{10};
  int heartbeat_startup_grace_ms{5000};
  // Internal supervision seam; production uses the fixed registry deadline.
  std::chrono::milliseconds construction_timeout{
      introspect::kConstructionTimeout};

  // Launch a graph the linter called broken anyway. The default is refusal:
  // two spellings of one topic are silent at runtime.
  bool allow_graph_errors{false};

  // Probe and lint without spawning anything, as a pre-flight check.
  bool describe_only{false};

  // Input replay: run only `only`, fed the topics the session in this
  // directory logged (talOS/events/input_replay.h), and nothing else.
  std::string replay_inputs;
  std::string only;

  // How long one binary gets to answer --describe. A Python node importing
  // native wheels (robotpy-wpimath) needs several seconds on its first run
  // from a freshly copied bundle, while macOS checks every new library.
  int describe_timeout_ms{15000};

  // Bundle directory assembled by talos_bundle.
  std::string bundle_dir;

  // The config identity hash from the bundle manifest, carried into the
  // session manifest.
  std::string bundle_config_sha256;

  // Target or node name -> binary path. Filled from the bundle manifest, or
  // directly by tests.
  std::map<std::string, std::string> binary_overrides;
};

// ---------------------------------------------------------------------------
// Bundles
// ---------------------------------------------------------------------------

inline constexpr std::string_view kBundleManifest = "bundle_manifest.toml";

// Reads `bundle_manifest.toml` (written by talOS/build/bundle_builder.py) from
// options.bundle_dir, or from the working directory when that holds one.
inline bool LoadBundle(LauncherOptions& options, std::string& error) {
  if (options.bundle_dir.empty()) {
    if (!std::filesystem::exists(kBundleManifest)) return true;
    options.bundle_dir = ".";
  }

  const std::filesystem::path root{options.bundle_dir};
  if (!std::filesystem::exists(root)) {
    error = "bundle directory does not exist: " + options.bundle_dir;
    return false;
  }

  const std::filesystem::path manifest_path = root / kBundleManifest;
  if (std::filesystem::exists(manifest_path)) {
    toml::table manifest;
    try {
      manifest = toml::parse_file(manifest_path.string());
    } catch (const toml::parse_error& e) {
      error = "cannot read " + manifest_path.string() + ": " +
              std::string{e.description()};
      return false;
    }

    if (options.config_path.empty()) {
      if (auto config = manifest["config_path"].value<std::string>()) {
        options.config_path = (root / *config).string();
      }
    }
    options.bundle_config_sha256 =
        manifest["config_sha256"].value_or(std::string{});

    if (const auto* nodes = manifest["nodes"].as_array()) {
      for (const auto& entry : *nodes) {
        const auto* node = entry.as_table();
        if (node == nullptr) continue;
        const auto name = (*node)["name"].value<std::string>();
        if (!name) continue;
        std::vector<std::string> keys{*name};
        if (const auto* targets = (*node)["targets"].as_array()) {
          for (const auto& target : *targets) {
            if (auto label = target.value<std::string>())
              keys.push_back(*label);
          }
        }
        if (const auto binary = (*node)["binary"].value<std::string>()) {
          const std::string path = (root / *binary).string();
          for (const auto& key : keys) {
            options.binary_overrides[key] = path;
          }
        }
      }
    }
  }

  if (options.config_path.empty()) {
    const std::string default_config =
        (root / "config" / "robot.toml").string();
    if (std::filesystem::exists(default_config)) {
      options.config_path = default_config;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Roster
// ---------------------------------------------------------------------------

inline constexpr std::string_view kHardwareNodeName = "hardware_node";
inline constexpr std::string_view kSimGatewayName = "sim_gateway";
inline constexpr std::string_view kStudioBridgeName = "studio_bridge";

// Resolves a node target to an executable path: an override (bundle manifest
// or test) wins, then the target's bazel-bin output relative to the
// workspace. Anything that is not a `//pkg:target` or `@repo//pkg:target`
// label is taken as a path.
//
// One node has two label spellings: apparent `@repo//pkg:rule` (as a robot
// config spells it) and canonical `@@repo+//pkg:rule` (as a baked
// TALOS_NODE_TARGET and the bundle manifest spell it). Module names never
// contain '+', so for an override lookup both spellings normalize the repo
// to the part before the first '+': `@wrist`, `@@wrist+` and `@@wrist`
// compare equal, while `//pkg:rule`, paths and node names compare literally.
inline std::string NormalizeTargetKey(std::string_view target) {
  std::string_view rest = target;
  if (rest.starts_with("@@")) {
    rest.remove_prefix(2);
  } else if (rest.starts_with("@")) {
    rest.remove_prefix(1);
  } else {
    return std::string{target};
  }
  const std::size_t slashes = rest.find("//");
  if (slashes == std::string_view::npos) return std::string{target};
  std::string_view repo = rest.substr(0, slashes);
  if (const std::size_t plus = repo.find('+'); plus != std::string_view::npos)
    repo = repo.substr(0, plus);
  if (repo.empty() || repo.find_first_of("@/:") != std::string_view::npos)
    return std::string{target};
  return std::string{repo} + "//" + std::string{rest.substr(slashes + 2)};
}

inline std::string ResolveBinary(
    const std::string& target_or_path,
    const std::map<std::string, std::string>& overrides = {}) {
  const std::string want = NormalizeTargetKey(target_or_path);
  for (const auto& [key, path] : overrides) {
    if (NormalizeTargetKey(key) == want) return path;
  }
  if (target_or_path.starts_with("@")) {
    const bool canonical = target_or_path.starts_with("@@");
    const std::size_t repo_start = canonical ? 2 : 1;
    const std::size_t slashes = target_or_path.find("//", repo_start);
    const std::size_t colon = slashes == std::string::npos
                                  ? std::string::npos
                                  : target_or_path.find(':', slashes + 2);
    if (slashes == std::string::npos || colon == std::string::npos ||
        slashes == repo_start) {
      return target_or_path;
    }
    std::string repo = target_or_path.substr(repo_start, slashes - repo_start);
    // Bazel 8 Bzlmod places apparent @repo under bazel-bin/external/<repo>+/,
    // while canonical @@wrist+ already carries the trailing '+'.
    if (!canonical && !repo.ends_with('+')) repo += '+';
    std::string subpath = target_or_path.substr(slashes + 2);
    if (subpath.starts_with(':')) {
      subpath.erase(0, 1);
    } else {
      std::replace(subpath.begin(), subpath.end(), ':', '/');
    }
    return "bazel-bin/external/" + repo + "/" + subpath;
  }
  if (!target_or_path.starts_with("//")) return target_or_path;
  std::string subpath = target_or_path.substr(2);
  if (subpath.starts_with(':')) {
    subpath.erase(0, 1);
  } else {
    std::replace(subpath.begin(), subpath.end(), ':', '/');
  }
  return "bazel-bin/" + subpath;
}

// ResolveBinary, falling back to an override keyed by the node's name when
// the target does not resolve to an executable.
inline NodeSpec MakeNodeSpec(std::string_view name, const std::string& target,
                             const LauncherOptions& options) {
  NodeSpec spec{std::string{name}, target,
                ResolveBinary(target, options.binary_overrides)};
  if (::access(spec.binary_path.c_str(), X_OK) != 0) {
    auto it = options.binary_overrides.find(spec.name);
    if (it != options.binary_overrides.end()) spec.binary_path = it->second;
  }
  if (name == kSimGatewayName) spec.extra_args = options.sim_gateway_args;
  return spec;
}

inline std::vector<NodeSpec> DiscoverNodes(
    const config::RobotConfig& robot_config, const LauncherOptions& options) {
  std::vector<NodeSpec> specs;
  const auto robot_string = [&](std::string_view key) {
    return robot_config.toml_data["robot"][key].value<std::string>();
  };

  // 1. The hardware bridge.
  specs.push_back(MakeNodeSpec(
      kHardwareNodeName,
      robot_string("hardware_node").value_or("//talOS/bridge:hardware_node"),
      options));

  // 2. Subsystems.
  if (const auto* subsystems =
          robot_config.toml_data["subsystems"].as_table()) {
    for (auto&& [key, value] : *subsystems) {
      if (const auto* table = value.as_table()) {
        if (auto node = (*table)["node"].value<std::string>()) {
          auto spec = MakeNodeSpec(key.str(), *node, options);
          spec.critical = (*table)["critical"].value_or(true);
          if (const auto* devices =
                  robot_config.GetDevices(std::string{key.str()})) {
            spec.owns_actuators = !devices->motors.empty() ||
                                  !devices->digital_outputs.empty() ||
                                  !devices->pwm_outputs.empty();
          }
          specs.push_back(std::move(spec));
        }
      }
    }
  }

  // 3. The sim gateway stands in for a controller processor, so which binary
  // that is belongs to the robot config, not to the framework.
  if (options.start_sim_gateway && options.simulation) {
    if (auto gateway = robot_string("sim_gateway")) {
      specs.push_back(MakeNodeSpec(kSimGatewayName, *gateway, options));
    }
  }

  // 4. The Studio bridge: a host-side viewer named by [studio], sim only.
  if (options.start_studio_bridge && options.simulation) {
    specs.push_back(
        MakeNodeSpec(kStudioBridgeName, robot_config.studio.bridge, options));
  }
  return specs;
}

// Effective endpoints for a launch: a CLI flag wins, otherwise the config.
inline std::string EffectiveRemoteAddress(const config::RobotConfig& config,
                                          const LauncherOptions& options) {
  return config::ResolveControllerAddress(config, options.remote);
}

inline uint16_t EffectiveRemotePort(const config::RobotConfig& config,
                                    const LauncherOptions& options) {
  return config::ResolveControllerPort(config, options.remote_port);
}

inline uint16_t EffectiveLocalPort(const config::RobotConfig& config,
                                   const LauncherOptions& options) {
  return config::ResolveLocalPort(config, options.local_port);
}

inline uint16_t EffectiveStudioHttpPort(const config::RobotConfig& config,
                                        const LauncherOptions& options) {
  return options.studio_http_port != 0 ? options.studio_http_port
                                       : config.studio.http_port;
}

// The launcher alone holds each lifetime pipe's writer. A tiny guardian in the
// child's group kills the group on EOF, including when the launcher is
// SIGKILLed. Descriptors are closed in other children and on exec; no node owns
// a writer.
inline std::map<pid_t, int>& ProcessLifetimes() {
  static std::map<pid_t, int> lifetimes;
  return lifetimes;
}

inline void SignalProcessGroup(pid_t pid, int signal) {
  if (pid > 0) ::kill(-pid, signal);
}

// Call after reaping the leader: even a clean exit must not leave helpers.
inline void FinishProcessGroup(pid_t pid) {
  SignalProcessGroup(pid, SIGKILL);
  auto& lifetimes = ProcessLifetimes();
  const auto found = lifetimes.find(pid);
  if (found != lifetimes.end()) {
    ::close(found->second);
    lifetimes.erase(found);
  }
}

// fork + execv of args[0], in its own process group. stdout_fd is used only by
// --describe; otherwise stdout/stderr can be appended to output_path.
inline pid_t SpawnProcess(std::vector<std::string> args,
                          const std::string& output_path = "",
                          int stdout_fd = -1) {
  int lifetime[2];
  if (::pipe(lifetime) != 0) return -1;
  if (::fcntl(lifetime[0], F_SETFD, FD_CLOEXEC) < 0 ||
      ::fcntl(lifetime[1], F_SETFD, FD_CLOEXEC) < 0) {
    ::close(lifetime[0]);
    ::close(lifetime[1]);
    return -1;
  }
  std::vector<char*> c_args;
  c_args.reserve(args.size() + 1);
  for (auto& arg : args) c_args.push_back(arg.data());
  c_args.push_back(nullptr);
  std::cout.flush();
  std::fflush(nullptr);
  const pid_t parent = ::getpid();
  const pid_t pid = ::fork();
  if (pid != 0) {
    ::close(lifetime[0]);
    if (pid < 0) {
      ::close(lifetime[1]);
    } else {
      // Both sides establish the group, closing the spawn/signal race.
      ::setpgid(pid, pid);
      ProcessLifetimes().emplace(pid, lifetime[1]);
    }
    return pid;
  }
  ::close(lifetime[1]);
  for (const auto& [other_pid, fd] : ProcessLifetimes()) {
    (void)other_pid;
    ::close(fd);
  }
  if (::setpgid(0, 0) != 0) ::_exit(127);
#ifdef __linux__
  if (::prctl(PR_SET_PDEATHSIG, SIGKILL) != 0) ::_exit(127);
#endif
  if (::getppid() != parent) ::_exit(127);
  std::signal(SIGINT, SIG_DFL);
  std::signal(SIGTERM, SIG_DFL);
  const pid_t group = ::getpid();
  const pid_t guardian = ::fork();
  if (guardian < 0) ::_exit(127);
  if (guardian == 0) {
    // Graceful group signals belong to the node; the guardian waits for EOF.
    std::signal(SIGINT, SIG_IGN);
    std::signal(SIGTERM, SIG_IGN);
    for (const int fd : {STDIN_FILENO, STDOUT_FILENO, STDERR_FILENO}) {
      if (fd != lifetime[0]) ::close(fd);
    }
    if (stdout_fd > STDERR_FILENO) ::close(stdout_fd);
    char unused;
    while (::read(lifetime[0], &unused, 1) < 0 && errno == EINTR) {
    }
    SignalProcessGroup(group, SIGKILL);
    ::_exit(0);
  }
  ::close(lifetime[0]);
  if (stdout_fd >= 0) {
    if (::dup2(stdout_fd, STDOUT_FILENO) < 0) ::_exit(127);
    if (stdout_fd != STDOUT_FILENO) ::close(stdout_fd);
  } else if (!output_path.empty()) {
    std::freopen(output_path.c_str(), "a", stdout);
    std::freopen(output_path.c_str(), "a", stderr);
  }
  ::execv(c_args[0], c_args.data());
  std::fprintf(stderr, "[launcher] execv failed for %s: %s\n", c_args[0],
               std::strerror(errno));
  ::_exit(127);
}

// ---------------------------------------------------------------------------
// Declared graph (phase one)
// ---------------------------------------------------------------------------

// What one binary answered when it was asked what it is.
struct NodeDescription {
  std::string roster_name;  // The `[subsystems.<name>]` key from the config.
  std::string target;
  std::string binary_path;
  bool answered{false};
  std::string error;  // Why not, when it did not answer.
  introspect::Description declared;
};

// Runs `binary --describe` and parses the manifest from its stdout, killing it
// after `timeout`: a node that blocks in its constructor costs the launch that
// much and no more. stderr is left alone so a node that throws says so.
inline bool ProbeDescribe(const std::string& binary_path,
                          const std::vector<std::string>& args,
                          std::chrono::milliseconds timeout,
                          introspect::Description& out, std::string& error) {
  error.clear();
  if (::access(binary_path.c_str(), X_OK) != 0) {
    error = "no executable at " + binary_path;
    return false;
  }

  int fds[2];
  if (::pipe(fds) != 0) {
    error = std::string{"pipe() failed: "} + std::strerror(errno);
    return false;
  }

  const pid_t pid = SpawnProcess(args, "", fds[1]);
  if (pid < 0) {
    error = std::string{"fork() failed: "} + std::strerror(errno);
    ::close(fds[0]);
    ::close(fds[1]);
    return false;
  }
  ::close(fds[1]);

  std::string text;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  bool timed_out = false;
  constexpr std::size_t kMaxDescribeBytes = std::size_t{4} * 1024 * 1024;
  for (;;) {
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    if (left.count() <= 0) {
      timed_out = true;
      break;
    }
    ::pollfd waiting{fds[0], POLLIN, 0};
    const int ready = ::poll(&waiting, 1, static_cast<int>(left.count()));
    if (ready < 0) {
      if (errno == EINTR) continue;
      error = std::string{"poll() failed: "} + std::strerror(errno);
      break;
    }
    if (ready == 0) {
      timed_out = true;
      break;
    }
    char buffer[4096];
    const ssize_t got = ::read(fds[0], buffer, sizeof(buffer));
    if (got < 0) {
      if (errno == EINTR) continue;
      error = std::string{"read() failed: "} + std::strerror(errno);
      break;
    }
    if (got == 0) break;  // EOF does not imply that the child has exited.
    if (static_cast<std::size_t>(got) > kMaxDescribeBytes - text.size()) {
      error = "describe output exceeds the 4 MiB limit";
      break;
    }
    text.append(buffer, static_cast<std::size_t>(got));
  }
  ::close(fds[0]);

  int status = 0;
  if (!timed_out && error.empty()) {
    for (;;) {
      const pid_t result = ::waitpid(pid, &status, WNOHANG);
      if (result == pid) break;
      if (result < 0) {
        if (errno == EINTR) continue;
        error = std::string{"waitpid() failed: "} + std::strerror(errno);
        break;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        timed_out = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
  }
  if (timed_out || !error.empty()) {
    SignalProcessGroup(pid, SIGKILL);
    while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
    }
  }
  FinishProcessGroup(pid);

  if (timed_out) {
    error = "no answer within " + std::to_string(timeout.count()) +
            "ms; the probe was killed";
    return false;
  }
  if (!error.empty()) return false;

  std::string parse_error;
  if (!introspect::DescribeReader::Parse(text, out, parse_error)) {
    error = parse_error;
    if (WIFEXITED(status) && WEXITSTATUS(status) != 0) {
      error += "; exit code " + std::to_string(WEXITSTATUS(status));
    } else if (WIFSIGNALED(status)) {
      error += "; killed by signal " + std::to_string(WTERMSIG(status));
    }
    return false;
  }
  return true;
}

// Every subsystem's target sources, for the lane checks in LintGraph.
inline naming::TargetSources TargetSourcesOf(
    const config::RobotConfig& robot_config) {
  naming::TargetSources sources;
  for (const auto& [subsystem, policy] : robot_config.subsystem_targets) {
    sources.emplace(subsystem, policy.Sources());
  }
  return sources;
}

inline naming::TargetModeSources TargetModeSourcesOf(
    const config::RobotConfig& robot_config) {
  naming::TargetModeSources sources;
  for (const auto& [subsystem, policy] : robot_config.subsystem_targets) {
    sources.emplace(subsystem, naming::TargetModeSources::mapped_type{
                                   {"teleop", policy.teleop},
                                   {"autonomous", policy.autonomous},
                                   {"test", policy.test}});
  }
  return sources;
}

// METHODOLOGY §6: every numeric parameter key ends in a unit suffix. One
// warning per subsystem whose [subsystems.<name>] table has numeric keys (a
// number, or an array of numbers) without one, listing them. Nested tables
// (motors, targets) are device and policy config, not parameters.
inline std::vector<naming::Diagnostic> LintParameterUnits(
    const config::RobotConfig& robot_config) {
  std::vector<naming::Diagnostic> findings;
  const toml::table* subsystems =
      robot_config.toml_data["subsystems"].as_table();
  if (subsystems == nullptr) return findings;
  const auto number = [](const toml::node& value) {
    return value.is_integer() || value.is_floating_point();
  };
  const auto numeric = [&number](const toml::node& value) {
    if (number(value)) return true;
    const toml::array* array = value.as_array();
    return array != nullptr && !array->empty() &&
           std::all_of(array->begin(), array->end(), number);
  };
  for (const auto& [subsystem, table] : *subsystems) {
    const toml::table* params = table.as_table();
    if (params == nullptr) continue;
    std::string unitless;
    for (const auto& [key, value] : *params) {
      if (!numeric(value) || schema::has_unit_suffix(key.str())) continue;
      unitless += unitless.empty() ? "" : ", ";
      unitless += key.str();
    }
    if (unitless.empty()) continue;
    findings.push_back(
        {naming::Severity::WARNING, std::string{subsystem.str()},
         "numeric parameters without a unit suffix (docs/METHODOLOGY.md "
         "§6): " +
             unitless});
  }
  return findings;
}

// Where the launcher writes the declared graph that graph-reading nodes get
// as `--graph` (introspect::DeclaredGraph): beside graph.json in the session
// directory.
inline std::string DeclaredGraphPath(const LauncherOptions& options) {
  const std::filesystem::path dir =
      options.output_dir.empty()
          ? std::filesystem::temp_directory_path() /
                ("talos_describe_" + std::to_string(::getpid()))
          : std::filesystem::path{options.output_dir};
  return (dir / "declared_graph.json").string();
}

// Phase one of a launch: ask every binary in the roster what it is, assemble
// the graph they describe, and lint it. The binaries, not the config, are the
// source: a second declaration of the topology is a thing to forget to update.
//
// Two passes. A framework node whose topics follow from the other nodes'
// (telemetry observes every state topic, teleop writes each bound lane with
// its consumer's schema) asks for the declared graph in its constructor and
// says so in its description (reads_graph). Every node is described once
// without a graph; then the descriptions of the nodes that do not read it
// are written to DeclaredGraphPath, and each graph reader is described
// again with `--graph` that file, which it is also started with. A graph
// reader therefore sees every node but the graph readers.
inline std::vector<naming::Diagnostic> DescribeGraph(
    const std::vector<NodeSpec>& specs, const LauncherOptions& options,
    std::vector<NodeDescription>& out,
    const naming::TargetSources& target_sources = {},
    const naming::TargetModeSources& target_mode_sources = {},
    bool require_target_coverage = false) {
  out.clear();
  out.reserve(specs.size());

  std::vector<naming::Diagnostic> probe_findings;
  const auto probe = [&](const NodeSpec& spec, NodeDescription& described,
                         const std::string& graph_path) {
    // A node whose topics depend on its configuration must describe the
    // configuration this session will use.
    std::vector<std::string> args{spec.binary_path, "--describe", "--config",
                                  options.config_path};
    if (options.simulation) args.push_back("--sim");
    if (!graph_path.empty()) args.insert(args.end(), {"--graph", graph_path});
    args.insert(args.end(), spec.extra_args.begin(), spec.extra_args.end());
    described.answered =
        ProbeDescribe(spec.binary_path, args,
                      std::chrono::milliseconds{options.describe_timeout_ms},
                      described.declared, described.error);
  };

  bool any_reader = false;
  for (const auto& spec : specs) {
    NodeDescription described;
    described.roster_name = spec.name;
    described.target = spec.target;
    described.binary_path = spec.binary_path;
    probe(spec, described, "");
    any_reader =
        any_reader || (described.answered && described.declared.reads_graph);
    out.push_back(std::move(described));
  }

  if (any_reader) {
    introspect::DeclaredGraph graph;
    for (const auto& described : out) {
      if (described.answered && !described.declared.reads_graph) {
        graph.push_back(described.declared);
      }
    }
    const std::string graph_path = DeclaredGraphPath(options);
    std::filesystem::create_directories(
        std::filesystem::path{graph_path}.parent_path());
    std::ofstream{graph_path} << introspect::DeclaredGraphToJson(graph);
    for (std::size_t i = 0; i < specs.size(); ++i) {
      if (!out[i].answered || !out[i].declared.reads_graph) continue;
      probe(specs[i], out[i], graph_path);
    }
  }

  std::vector<naming::NodeShape> shapes;
  for (std::size_t i = 0; i < out.size(); ++i) {
    const auto& described = out[i];
    const auto& spec = specs[i];
    const bool writes_targets =
        std::any_of(described.declared.sources.begin(),
                    described.declared.sources.end(), [](const auto& source) {
                      return source.kind == event::SourceKind::SENDER &&
                             source.name.find("/target/") != std::string::npos;
                    });
    if (!spec.critical && (spec.owns_actuators || writes_targets)) {
      probe_findings.push_back(
          {naming::Severity::ERROR, described.roster_name,
           "critical=false is forbidden for an actuator owner or target-lane "
           "producer"});
    }
    if (described.answered) {
      shapes.push_back({described.declared.name, described.declared.target,
                        described.declared.sources});
      if (described.declared.name != described.roster_name) {
        probe_findings.push_back(
            {naming::Severity::WARNING, described.roster_name,
             "the config calls this node '" + described.roster_name +
                 "' but the binary describes itself as '" +
                 described.declared.name + "'"});
      }
    } else {
      probe_findings.push_back(
          {spec.critical || spec.owns_actuators ? naming::Severity::ERROR
                                                : naming::Severity::WARNING,
           described.roster_name,
           "did not answer --describe (" + described.error +
               "), so its topics are missing from the declared graph; " +
               (spec.critical || spec.owns_actuators
                    ? "critical node requires a successful probe"
                    : "NON-CRITICAL NODE: launching under restart "
                      "supervision")});
    }
  }

  auto diagnostics = naming::LintGraph(
      shapes, target_sources, target_mode_sources, require_target_coverage);
  diagnostics.insert(diagnostics.end(), probe_findings.begin(),
                     probe_findings.end());
  return diagnostics;
}

// Distinct topics in a declared graph. Timers are not topics.
inline std::size_t CountDeclaredTopics(
    const std::vector<NodeDescription>& described) {
  std::vector<std::string> topics;
  for (const auto& node : described) {
    for (const auto& source : node.declared.sources) {
      if (source.kind == event::SourceKind::TIMER) continue;
      if (std::find(topics.begin(), topics.end(), source.name) ==
          topics.end()) {
        topics.push_back(source.name);
      }
    }
  }
  return topics.size();
}

// The shape each declared topic's segment is created with: its publisher's
// (the first SENDER), else its first end's. Ends that disagree are a lint
// error the session was told to ignore (--allow-graph-errors); the ends that
// differ from the segment then fail at attach, naming both shapes.
inline std::vector<naming::SourceShape> SessionTopics(
    const std::vector<NodeDescription>& described) {
  std::vector<naming::SourceShape> topics;
  for (const auto& node : described) {
    for (const auto& source : node.declared.sources) {
      if (source.kind == event::SourceKind::TIMER) continue;
      const auto known = std::find_if(
          topics.begin(), topics.end(),
          [&source](const auto& topic) { return topic.name == source.name; });
      if (known == topics.end()) {
        topics.push_back(source);
      } else if (source.kind == event::SourceKind::SENDER &&
                 known->kind != event::SourceKind::SENDER) {
        *known = source;
      }
    }
  }
  return topics;
}

// Phase two's first step: the session owns every declared topic. Each
// segment is created session-owned (rtms.h "Segment lifetime") before any
// node starts, replacing whatever the name held, and unlinked when the
// returned queues are destroyed at the end of the session. Nodes only
// attach, so a node that is killed and restarted, or that exits cleanly,
// leaves its topics -- and the subscribers reading them -- in place. A
// topic of a node that did not answer --describe is not declared, so its
// first process creates it as outside a session. Throws, naming the topic,
// if a segment cannot be created.
inline std::vector<RTMSQueue> CreateSessionSegments(
    const std::vector<NodeDescription>& described) {
  std::vector<RTMSQueue> segments;
  for (const auto& topic : SessionTopics(described)) {
    try {
      segments.emplace_back(
          topic.name, topic.message_bytes, topic.alignment,
          schema::Identity{topic.schema, topic.schema_hash, {}}, topic.slots,
          RTMSOptions{.session_owner = true});
    } catch (const std::exception& e) {
      throw std::runtime_error("cannot create the segment of " + topic.name +
                               ": " + e.what());
    }
  }
  return segments;
}

// The declared graph on disk, beside the session's manifest. uint64 values are
// decimal strings: a JSON number is a double, and an id past 2^53 would change.
inline std::string GraphToJson(
    uint64_t session_id, const std::string& config_path,
    const std::vector<NodeDescription>& described,
    const std::vector<naming::Diagnostic>& diagnostics,
    const std::string& config_sha256 = "") {
  std::ostringstream ss;
  ss << "{\n";
  ss << "  \"describe_version\": " << introspect::kDescribeVersion << ",\n";
  ss << "  \"session_id\": \"" << session_id << "\",\n";
  ss << "  \"config_path\": " << Quoted(config_path) << ",\n";
  ss << "  \"config_sha256\": " << Quoted(config_sha256) << ",\n";
  ss << "  \"nodes\": [\n";
  for (std::size_t i = 0; i < described.size(); ++i) {
    const auto& node = described[i];
    ss << "    {\n";
    ss << "      \"name\": "
       << Quoted(node.answered ? node.declared.name : node.roster_name)
       << ",\n";
    ss << "      \"target\": "
       << Quoted(node.answered ? node.declared.target : node.target) << ",\n";
    ss << "      \"described\": " << (node.answered ? "true" : "false")
       << ",\n";
    if (!node.answered) {
      ss << "      \"describe_error\": " << Quoted(node.error) << ",\n";
    }
    ss << "      \"sources\": [";
    for (std::size_t j = 0; j < node.declared.sources.size(); ++j) {
      const auto& source = node.declared.sources[j];
      ss << (j ? ",\n        {" : "\n        {");
      ss << "\"kind\": " << Quoted(event::to_string(source.kind));
      ss << ", \"name\": " << Quoted(source.name);
      ss << ", \"message_bytes\": " << source.message_bytes;
      ss << ", \"external\": " << (source.external() ? "true" : "false");
      ss << ", \"optional\": " << (source.optional() ? "true" : "false");
      ss << ", \"observer\": " << (source.observer() ? "true" : "false");
      if (source.kind != event::SourceKind::TIMER) {
        // As describe writes them: the hash a hex string, not a double.
        ss << ", \"schema\": " << Quoted(source.schema);
        ss << ", \"schema_hash\": "
           << Quoted(introspect::HexHash(source.schema_hash));
        ss << ", \"alignment\": " << source.alignment;
        ss << ", \"slots\": " << source.slots;
      }
      ss << "}";
    }
    ss << (node.declared.sources.empty() ? "]\n" : "\n      ]\n");
    ss << "    }" << (i + 1 < described.size() ? "," : "") << "\n";
  }
  ss << "  ],\n";
  ss << "  \"diagnostics\": [\n";
  for (std::size_t i = 0; i < diagnostics.size(); ++i) {
    const auto& diagnostic = diagnostics[i];
    ss << "    {\"severity\": "
       << Quoted(diagnostic.severity == naming::Severity::ERROR ? "error"
                                                                : "warning")
       << ", \"subject\": " << Quoted(diagnostic.subject)
       << ", \"message\": " << Quoted(diagnostic.message) << "}"
       << (i + 1 < diagnostics.size() ? "," : "") << "\n";
  }
  ss << "  ]\n";
  ss << "}\n";
  return ss.str();
}

// ---------------------------------------------------------------------------
// Restarts
//
// A roster member other than the Studio bridge that crashes -- exits nonzero
// or by a signal -- while the session is not stopping is respawned after a
// backoff of 50 ms, doubling within a rolling restart window, capped at 1 s.
// max_restarts limits starts within that window, not the session lifetime.
// A critical member that exits cleanly (status 0) while the session is not
// stopping did not finish anything -- its peers still run -- so it restarts
// on the same backoff; a non-critical clean exit is a node that finished.
// When a clean exit's restart falls due and every other live member's latest
// run also exited 0, the whole roster has completed and the restart is
// dropped (see RosterContinues). The backoff is scheduled inside the
// supervision loop, so a restarting node never delays another node or the
// session deadline. Every run writes its own event log.
// ---------------------------------------------------------------------------

inline constexpr int kDefaultMaxRestarts = 5;
inline constexpr std::chrono::milliseconds kRestartInitialBackoff{50};
inline constexpr std::chrono::milliseconds kRestartMaxBackoff{1000};

// The wait before restart `restart` (1 for the first).
inline std::chrono::milliseconds RestartBackoff(int restart) {
  std::chrono::milliseconds backoff = kRestartInitialBackoff;
  for (int i = 1; i < restart && backoff < kRestartMaxBackoff; ++i) {
    backoff *= 2;
  }
  return std::min(backoff, kRestartMaxBackoff);
}

// A node's event log for run `restart` (0 for the first run): `<name>.tlog`,
// then `<name>.restart<k>.tlog`, so a restart never truncates an earlier run.
inline std::string NodeLogPath(const std::string& output_dir,
                               std::string_view name, int restart) {
  std::string path = output_dir + "/" + std::string{name};
  if (restart > 0) path += ".restart" + std::to_string(restart);
  return path + ".tlog";
}

// Requested stop, the session deadline and launcher shutdown (`stopping`)
// never restart; zero disables all retries. Crashes remain retryable after a
// budget quarantine. A clean exit restarts only a critical member.
inline bool ShouldRestartNode(bool clean_exit, bool critical, int max_restarts,
                              bool stopping) {
  if (stopping || max_restarts <= 0) return false;
  return !clean_exit || critical;
}

// `--max-restarts` wins, then `[robot] max_restarts`, then the default.
inline bool ResolveMaxRestarts(const config::RobotConfig& robot_config,
                               const LauncherOptions& options, int& out,
                               std::string& error) {
  if (options.max_restarts) {
    if (*options.max_restarts < 0) {
      error = "--max-restarts must be >= 0";
      return false;
    }
    out = *options.max_restarts;
    return true;
  }
  const auto node = robot_config.toml_data["robot"]["max_restarts"];
  if (!node) {
    out = kDefaultMaxRestarts;
    return true;
  }
  const auto* value = node.as_integer();
  if (value == nullptr || value->get() < 0 ||
      value->get() > std::numeric_limits<int>::max()) {
    error = "[robot] max_restarts must be an integer >= 0";
    return false;
  }
  out = static_cast<int>(value->get());
  return true;
}

// "exit code N" or "signal N", from a waitpid status.
inline std::string ExitReason(int status) {
  if (WIFSIGNALED(status)) return "signal " + std::to_string(WTERMSIG(status));
  return "exit code " +
         std::to_string(WIFEXITED(status) ? WEXITSTATUS(status) : -1);
}

// ---------------------------------------------------------------------------
// Studio bridge
//
// The one roster member supervised differently. It takes positional args
// rather than the node flags, logs plaintext to studio_bridge.log rather than
// a .tlog, and refuses to start before the telemetry publisher exists -- so
// its first attempts usually lose that race. It is retried at most once per
// interval, inside the supervision loop, within a bounded window; while it is
// still starting it neither holds the session open nor reports its exits. A
// bridge that survives one interval is stable and supervised like any node,
// except that a crash is not restarted (see Restarts above).
// One that never comes up is a degraded session, not a broken robot: it is
// marked failed and the session runs on without Studio.
// ---------------------------------------------------------------------------

inline constexpr int kStudioBridgeMaxAttempts = 60;
inline constexpr std::chrono::seconds kStudioBridgeRetryInterval{1};
inline constexpr std::chrono::seconds kStudioBridgeStartupWindow{60};

// True while the bridge still has startup budget, its last spawn is at least
// one interval old, and the session is not stopping.
inline bool ShouldRetryStudioBridge(
    int attempts_made, std::chrono::steady_clock::time_point first_attempt,
    std::chrono::steady_clock::time_point last_spawn,
    std::chrono::steady_clock::time_point now, bool stopping) {
  if (stopping) return false;
  if (attempts_made >= kStudioBridgeMaxAttempts) return false;
  if (now - first_attempt >= kStudioBridgeStartupWindow) return false;
  return now - last_spawn >= kStudioBridgeRetryInterval;
}

// The topic the bridge watches: what it described (its --describe reads the
// `[studio] topic` key the telemetry node publishes on), or that key itself
// when the bridge did not answer.
inline std::string StudioBridgeWatchTopic(
    const std::vector<NodeDescription>& described,
    const config::RobotConfig& robot_config) {
  std::string topic = robot_config.studio.topic;
  for (const auto& node : described) {
    if (!node.answered || node.declared.name != kStudioBridgeName) continue;
    for (const auto& source : node.declared.sources) {
      if (source.kind == event::SourceKind::WATCHER && !source.name.empty()) {
        topic = source.name;
      }
    }
  }
  return topic;
}

class StudioBridge {
 public:
  using Clock = std::chrono::steady_clock;

  // `proc` is the bridge's roster entry; it keeps no .tlog.
  StudioBridge(std::size_t index, const NodeSpec& spec, NodeProcessInfo& proc,
               const config::RobotConfig& robot_config,
               const LauncherOptions& options,
               const std::vector<NodeDescription>& described,
               const std::string& graph_path)
      : index_{index},
        target_{spec.target},
        log_path_{options.output_dir + "/studio_bridge.log"},
        http_port_{EffectiveStudioHttpPort(robot_config, options)},
        args_{spec.binary_path,
              "--drop-newest-publisher",
              StudioBridgeWatchTopic(described, robot_config),
              std::to_string(http_port_),
              robot_config.studio.webroot,
              "--config",
              options.config_path,
              "--declared",
              graph_path,
              "--manifest",
              options.output_dir + "/manifest.json"} {
    proc.log_path.clear();
  }

  std::size_t index() const { return index_; }

  // Still racing telemetry: exits are expected and the session does not wait.
  bool Starting(std::size_t index) const {
    return index == index_ && active_ && !stable_;
  }

  // The first attempt, spawned alongside every other node without waiting.
  void Start(NodeProcessInfo& proc) {
    const std::string& binary = args_.front();
    if (binary.empty() || ::access(binary.c_str(), X_OK) != 0) {
      std::cerr << "[launcher] studio_bridge not executable at '"
                << (binary.empty() ? target_ : binary)
                << "'; session runs without Studio\n";
      proc.failed = true;
      return;
    }
    std::cout << "[launcher] Starting studio_bridge (target " << target_
              << ")...\n";
    const pid_t pid = SpawnProcess(args_, log_path_);
    if (pid < 0) {
      std::cerr << "[launcher] fork() failed for studio_bridge\n";
      proc.failed = true;
      return;
    }
    const auto now = Clock::now();
    proc.pid = pid;
    attempts_ = 1;
    first_attempt_ = now;
    last_spawn_ = now;
    active_ = true;
  }

  // One supervision tick, after the loop has reaped exits: stabilize a
  // survivor, respawn a startup failure, or give up when budget runs out.
  void Supervise(NodeProcessInfo& proc, Clock::time_point now, bool stopping) {
    if (!active_ || stable_) return;
    if (proc.pid > 0 && !proc.exited) {
      if (now - last_spawn_ >= kStudioBridgeRetryInterval) {
        stable_ = true;
        std::cout << "[launcher] Started studio_bridge (pid " << proc.pid
                  << ") on http://localhost:" << http_port_ << "\n";
      }
      return;
    }
    if (ShouldRetryStudioBridge(attempts_, first_attempt_, last_spawn_, now,
                                stopping)) {
      const int last_code = proc.exit_code;
      const pid_t pid = SpawnProcess(args_, log_path_);
      if (pid < 0) {
        std::cerr << "[launcher] fork() failed for studio_bridge\n";
        GiveUp(proc);
        return;
      }
      if (attempts_ == 1 || attempts_ % 15 == 0) {
        std::cout << "[launcher] studio_bridge not up yet (exit " << last_code
                  << "); waiting for the telemetry publisher...\n";
      }
      proc.pid = pid;
      proc.exited = false;
      proc.exit_code = -1;
      ++attempts_;
      last_spawn_ = now;
    } else if (now - last_spawn_ >= kStudioBridgeRetryInterval) {
      active_ = false;
      // Abandoned because the session is stopping: not a bridge failure.
      if (stopping) return;
      std::cerr << "[launcher] studio_bridge never came up; session runs "
                   "without Studio (see "
                << log_path_ << ")\n";
      GiveUp(proc);
    }
  }

  // The session ended while the bridge was still starting: it never served
  // Studio. A stop-driven ending is not a bridge failure.
  void Finish(NodeProcessInfo& proc, bool stop_requested) {
    if (!active_ || stable_ || stop_requested) return;
    if (proc.pid > 0 && !proc.exited) {
      SignalProcessGroup(proc.pid, SIGKILL);
      int status = 0;
      while (::waitpid(proc.pid, &status, 0) < 0 && errno == EINTR) {
      }
      FinishProcessGroup(proc.pid);
      proc.exited = true;
    }
    GiveUp(proc);
  }

 private:
  void GiveUp(NodeProcessInfo& proc) {
    proc.failed = true;
    proc.exit_code = -1;
    active_ = false;
  }

  std::size_t index_;
  std::string target_;
  std::string log_path_;
  uint16_t http_port_;
  std::vector<std::string> args_;
  int attempts_{0};
  Clock::time_point first_attempt_{};
  Clock::time_point last_spawn_{};
  bool active_{false};
  bool stable_{false};
};

// ---------------------------------------------------------------------------
// Launcher
// ---------------------------------------------------------------------------

class Launcher {
 public:
  explicit Launcher(LauncherOptions options) : options_{std::move(options)} {
    if (options_.session_id == 0) options_.session_id = GenerateSessionId();
    const std::filesystem::path root =
        options_.output_dir.empty() ? "/tmp/talos_logs" : options_.output_dir;
    options_.output_dir = (root / std::to_string(options_.session_id)).string();
  }

  // Lints the declared graph, then runs the roster until every node exits,
  // the duration passes or a signal arrives, and writes manifest.json.
  // Returns 0, or the last nonzero exit code of a node. With --replay-inputs
  // it runs only the --only node, on recorded topics (RunOnRecordedInputs).
  int Run() {
    if (!options_.sim_gateway_args.empty() && !options_.start_sim_gateway) {
      std::cerr << "launcher: --sim-gateway-arg requires --start-sim-gateway\n";
      return 1;
    }
    std::string bundle_error;
    if (!LoadBundle(options_, bundle_error)) {
      std::cerr << "launcher: " << bundle_error << "\n";
      return 1;
    }
    if (options_.config_path.empty()) {
      std::cerr << "launcher: --config PATH or --bundle PATH is required\n";
      return 1;
    }
    if (options_.replay_inputs.empty() != options_.only.empty()) {
      std::cerr << "launcher: --replay-inputs SESSION_DIR and --only NODE go "
                   "together\n";
      return 1;
    }
    std::filesystem::create_directories(options_.output_dir);

    config::RobotConfig robot_config;
    try {
      robot_config = config::ParseRobotConfig(options_.config_path);
    } catch (const std::exception& e) {
      std::cerr << "launcher: failed to load config from "
                << options_.config_path << ": " << e.what() << "\n";
      if (std::filesystem::path{options_.config_path}.is_relative()) {
        std::cerr << "launcher: working directory is "
                  << std::filesystem::current_path()
                  << "; run from the repository root or pass an absolute "
                     "--config path\n";
      }
      return 1;
    }
    const std::string effective_remote =
        EffectiveRemoteAddress(robot_config, options_);
    if (options_.simulation && effective_remote != "127.0.0.1") {
      std::cerr
          << "launcher: --sim requires remote 127.0.0.1; effective remote "
          << effective_remote << "\n";
      return 1;
    }
    const uint64_t session_log_budget = ApplyRetention(robot_config);
    std::string restart_error;
    if (!ResolveMaxRestarts(robot_config, options_, max_restarts_,
                            restart_error)) {
      std::cerr << "launcher: " << restart_error << "\n";
      return 1;
    }
    if (options_.restart_window_ms <= 0 || options_.heartbeat_periods <= 0 ||
        options_.heartbeat_startup_grace_ms < 0) {
      std::cerr
          << "launcher: restart window and heartbeat periods must be positive; "
             "startup grace must be nonnegative\n";
      return 1;
    }

    const auto node_specs = DiscoverNodes(robot_config, options_);
    if (node_specs.empty()) {
      std::cerr << "launcher: no nodes declared in config\n";
      return 1;
    }
    if (!options_.replay_inputs.empty()) {
      return RunOnRecordedInputs(node_specs);
    }

    // Phase one: describe and lint before anything starts.
    std::vector<NodeDescription> described;
    auto diagnostics = DescribeGraph(node_specs, options_, described,
                                     TargetSourcesOf(robot_config),
                                     TargetModeSourcesOf(robot_config),
                                     robot_config.require_target_coverage);
    for (auto& finding : LintParameterUnits(robot_config)) {
      diagnostics.push_back(std::move(finding));
    }
    for (auto& finding : naming::CheckHardwareConfig(robot_config.hardware)) {
      diagnostics.push_back(std::move(finding));
    }
    const std::size_t declared_topics = CountDeclaredTopics(described);

    const std::string graph_path = options_.output_dir + "/graph.json";
    std::ofstream{graph_path}
        << GraphToJson(options_.session_id, options_.config_path, described,
                       diagnostics, options_.bundle_config_sha256);
    PrintDeclaredGraph(described, diagnostics, declared_topics, graph_path);

    if (naming::HasError(diagnostics) && !options_.allow_graph_errors) {
      std::cerr << "launcher: the declared graph has errors; refusing to "
                   "start. Fix the names above, or pass --allow-graph-errors "
                   "to launch anyway.\n";
      return 1;
    }
    if (options_.describe_only) {
      std::cout << "[launcher] describe-only: nothing was started\n";
      return 0;
    }

    // Phase two. The session's topics exist before any node does, and until
    // the session ends.
    std::vector<RTMSQueue> segments;
    try {
      segments = CreateSessionSegments(described);
    } catch (const std::exception& e) {
      std::cerr << "launcher: " << e.what() << "\n";
      return 1;
    }
    std::cout << "[launcher] Created " << segments.size()
              << " session-owned topic segments\n";

    SessionManifest manifest;
    manifest.session_id = options_.session_id;
    manifest.config_path = options_.config_path;
    manifest.config_sha256 = options_.bundle_config_sha256;
    manifest.output_dir = options_.output_dir;
    manifest.simulation = options_.simulation;
    manifest.declared_topics = declared_topics;
    manifest.start_wall_ns = WallNanos();
    session_manifest_ = manifest;
    PersistManifest({});

    std::vector<NodeProcessInfo> procs;
    procs.reserve(node_specs.size());
    std::optional<StudioBridge> studio;
    for (std::size_t i = 0; i < node_specs.size(); ++i) {
      const auto& spec = node_specs[i];
      NodeProcessInfo& proc = procs.emplace_back();
      proc.name = spec.name;
      proc.target = spec.target;
      proc.binary_path = spec.binary_path;
      // The gateway is started without --log, so it writes no event log.
      if (spec.name != kSimGatewayName) {
        proc.log_path = NodeLogPath(options_.output_dir, spec.name, 0);
      }
      proc.described = described[i].answered;
      if (spec.name == kStudioBridgeName) {
        studio.emplace(i, spec, proc, robot_config, options_, described,
                       graph_path);
      }
      if (!proc.log_path.empty()) proc.log_paths.push_back(proc.log_path);
    }
    // Each logging member's share of the session's log budget; its restarts
    // draw on what its earlier runs left (Restart).
    const auto loggers = static_cast<uint64_t>(
        std::count_if(procs.begin(), procs.end(),
                      [](const auto& p) { return !p.log_path.empty(); }));
    const uint64_t node_log_budget = std::max<uint64_t>(
        1, session_log_budget / std::max<uint64_t>(1, loggers));
    PersistManifest(procs);

    InstallSignals();

    // The deadline starts with the session, so a bridge that never comes up
    // cannot push the session's end out.
    const auto deadline = options_.duration_s > 0
                              ? std::chrono::steady_clock::now() +
                                    std::chrono::seconds(options_.duration_s)
                              : std::chrono::steady_clock::time_point::max();

    const std::string remote = EffectiveRemoteAddress(robot_config, options_);
    const uint16_t remote_port = EffectiveRemotePort(robot_config, options_);
    const uint16_t local_port = EffectiveLocalPort(robot_config, options_);
    std::cout << "[launcher] Starting session " << options_.session_id
              << " with " << procs.size() << " nodes...\n";
    std::cout << "[launcher] Controller " << remote << ":" << remote_port
              << " (local port " << local_port << ")\n";

    std::vector<NodeLaunch> launches(procs.size());
    for (std::size_t i = 0; i < procs.size(); ++i) {
      if (studio && i == studio->index()) continue;
      launches[i].critical = node_specs[i].critical;
      auto& proc = procs[i];
      auto period_us =
          robot_config.toml_data["subsystems"][proc.name]["period_us"]
              .value_or<int64_t>(robot_config.toml_data["robot"]["period_us"]
                                     .value_or<int64_t>(5000));
      launches[i].heartbeat_timeout = std::chrono::microseconds{
          std::max<int64_t>(500000, period_us * options_.heartbeat_periods)};
      auto& args = launches[i].args;
      args.push_back(node_specs[i].binary_path);
      if (proc.name == kSimGatewayName) {
        // The gateway stamps the config's config_id; unconfigured it stamps
        // 0, which every mechanism rejects.
        args.insert(args.end(),
                    {"--config", options_.config_path, "--session-id",
                     std::to_string(options_.session_id)});
      } else {
        args.insert(args.end(), {"--session-id",
                                 std::to_string(options_.session_id), "--log"});
        launches[i].log_arg = args.size();
        launches[i].log_budget = node_log_budget;
        args.insert(args.end(), {proc.log_path, "--log-max-bytes",
                                 std::to_string(node_log_budget), "--config",
                                 options_.config_path});
        if (options_.simulation) args.push_back("--sim");
        // Started with the graph it was described with (DescribeGraph).
        if (described[i].answered && described[i].declared.reads_graph) {
          args.insert(args.end(), {"--graph", DeclaredGraphPath(options_)});
        }
        if (proc.name == kHardwareNodeName) {
          args.insert(args.end(), {"--remote", remote, "--remote-port",
                                   std::to_string(remote_port), "--local-port",
                                   std::to_string(local_port)});
        }
      }
      args.insert(args.end(), node_specs[i].extra_args.begin(),
                  node_specs[i].extra_args.end());
      if (options_.duration_s > 0) {
        args.insert(args.end(),
                    {"--duration-s", std::to_string(options_.duration_s)});
      }

      const pid_t pid = SpawnProcess(args);
      if (pid < 0) {
        std::cerr << "[launcher] fork() failed for node " << proc.name << "\n";
        proc.failed = true;
        continue;
      }
      proc.pid = pid;
      PersistManifest(procs);
      std::cout << "[launcher] Started " << proc.name << " (pid " << pid
                << ", target " << proc.target << ")\n";
    }
    if (studio) studio->Start(procs[studio->index()]);
    PersistManifest(procs);

    Supervise(procs, launches, studio, deadline);
    manifest.end_wall_ns = WallNanos();

    int exit_status = 0;
    for (auto& proc : procs) {
      // Every run's log: a restarted node's records are the sum of its runs.
      for (const auto& path : proc.log_paths) {
        if (std::filesystem::exists(event::log::CappedMarkerPath(path)))
          proc.log_capped = true;
        if (!std::filesystem::exists(path)) continue;
        // Streamed: shutdown memory does not grow with the logs' size.
        try {
          const auto summary = event::log::SummarizeLog(path);
          proc.record_count += summary.record_count;
          if (summary.truncated) proc.failed = true;
        } catch (...) {
          proc.failed = true;
        }
      }
      if (proc.exit_code != 0) exit_status = proc.exit_code;
    }
    manifest.nodes = std::move(procs);

    session_manifest_ = manifest;
    PersistManifest(manifest.nodes);
    PrintManifest(manifest);
    return exit_status;
  }

 private:
  // Never throws: a full disk (ENOSPC) or an unwritable session directory
  // must not end supervision of a running robot. The last complete manifest
  // stays in place, a partial temporary file is removed so it holds no disk,
  // and the next lifecycle change retries. The first failure is logged once,
  // and so is the recovery.
  void PersistManifest(const std::vector<NodeProcessInfo>& procs) {
    session_manifest_.nodes = procs;
    const std::string path = options_.output_dir + "/manifest.json";
    const std::string temporary = path + ".tmp";
    const auto fail = [&](std::string_view step, int error) {
      std::remove(temporary.c_str());
      if (!manifest_failing_) {
        std::cerr << "[launcher] cannot persist " << path << " (" << step
                  << ": " << std::strerror(error)
                  << "); supervision continues and retries at the next "
                     "change\n";
      }
      manifest_failing_ = true;
    };
    FILE* file = std::fopen(temporary.c_str(), "wb");
    if (file == nullptr) return fail("create", errno);
    const std::string json = session_manifest_.ToJson();
    int error = 0;
    if (std::fwrite(json.data(), 1, json.size(), file) != json.size() ||
        std::fflush(file) != 0 || ::fsync(::fileno(file)) != 0)
      error = errno;
    if (std::fclose(file) != 0 && error == 0) error = errno;
    if (error != 0) return fail("write", error);
    if (std::rename(temporary.c_str(), path.c_str()) != 0)
      return fail("rename", errno);
    const int directory = ::open(options_.output_dir.c_str(), O_RDONLY);
    if (directory < 0) return fail("open session directory", errno);
    const int result = ::fsync(directory);
    error = errno;
    ::close(directory);
    if (result != 0) return fail("sync session directory", error);
    if (manifest_failing_) {
      std::cerr << "[launcher] " << path << " persisted again\n";
      manifest_failing_ = false;
    }
  }

  // Removes the oldest recognized sessions until at most `log_keep_sessions`
  // remain (the new one included) and they hold at most the half of
  // `log_max_bytes` the new session does not get. Returns the new session's
  // log budget: the other half, which its event writers are capped to.
  // Graph/manifest metadata and non-event captures require separate headroom.
  uint64_t ApplyRetention(const config::RobotConfig& config) {
    const auto robot = config.toml_data["robot"];
    const int64_t cap = robot["log_keep_sessions"].value_or(int64_t{50});
    const int64_t budget =
        robot["log_max_bytes"].value_or(int64_t{2LL * 1024 * 1024 * 1024});
    if (cap < 1 || budget < 1)
      throw std::runtime_error(
          "log_keep_sessions and log_max_bytes must be positive");
    const auto session_budget = static_cast<uint64_t>(budget) / 2;
    const uint64_t retained_budget =
        static_cast<uint64_t>(budget) - session_budget;
    struct Session {
      std::filesystem::path path;
      std::filesystem::file_time_type time;
      uintmax_t bytes;
    };
    std::vector<Session> sessions;
    uintmax_t total = 0;
    const auto current = std::filesystem::path{options_.output_dir};
    for (const auto& entry :
         std::filesystem::directory_iterator(current.parent_path())) {
      // A session is recognized by its manifest; unrelated folders and symlinks
      // are never retention candidates.
      if (entry.is_symlink() || !entry.is_directory() ||
          !std::filesystem::is_regular_file(entry.path() / "manifest.json"))
        continue;
      uintmax_t bytes = 0;
      for (const auto& file :
           std::filesystem::recursive_directory_iterator(entry.path())) {
        if (!file.is_symlink() && file.is_regular_file())
          bytes += file.file_size();
      }
      total += bytes;
      sessions.push_back({entry.path(), entry.last_write_time(), bytes});
    }
    std::sort(
        sessions.begin(), sessions.end(),
        [](const Session& a, const Session& b) { return a.time < b.time; });
    // Include the newly created current session even before its manifest
    // exists.
    std::size_t count = sessions.size() + 1;
    for (const auto& session : sessions) {
      if (session.path == current) continue;
      if (count <= static_cast<uint64_t>(cap) && total <= retained_budget)
        break;
      std::filesystem::remove_all(session.path);
      total -= session.bytes;
      --count;
    }
    return session_budget;
  }

  // How one roster member is spawned, and its pending restart.
  struct NodeLaunch {
    std::vector<std::string> args;
    std::size_t log_arg{0};  // Index of the --log value in args; 0 for none.
    // Bytes all of this member's runs may log together (its share of the
    // session budget); a restart gets what earlier runs left.
    uint64_t log_budget{0};
    bool restart_pending{false};
    bool last_exit_clean{false};  // Its latest run exited 0.
    bool critical{true};
    std::chrono::steady_clock::time_point restart_at{};
    std::deque<std::chrono::steady_clock::time_point> restart_times;
    bool heartbeat_killed{false};
    std::chrono::nanoseconds heartbeat_timeout{std::chrono::milliseconds{500}};
    std::chrono::steady_clock::time_point started_at{};
    std::chrono::steady_clock::time_point heartbeat_changed_at{};
    int64_t last_heartbeat{0};
    uint64_t last_dispatch{0};
  };

  static int64_t WallNanos() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }

  // --replay-inputs: runs the one roster member --only names on the topics
  // the session in options_.replay_inputs logged, by starting its binary with
  // --replay-inputs (talOS/node/node.cc, which Python nodes share through
  // the C ABI). Nothing else starts: no other node, no segment, no hardware.
  // Its log is <output_dir>/<name>.tlog. Returns its exit code.
  int RunOnRecordedInputs(const std::vector<NodeSpec>& specs) {
    std::error_code ec;
    if (!std::filesystem::is_directory(options_.replay_inputs, ec)) {
      std::cerr << "launcher: --replay-inputs " << options_.replay_inputs
                << " is not a session directory\n";
      return 1;
    }
    // The new log must not land among the logs being read.
    const auto relative = std::filesystem::weakly_canonical(options_.output_dir)
                              .lexically_relative(std::filesystem::canonical(
                                  options_.replay_inputs));
    if (!relative.empty() && *relative.begin() != "..") {
      std::cerr << "launcher: --output-dir must not be the replayed session's "
                   "directory\n";
      return 1;
    }
    const auto spec = std::find_if(
        specs.begin(), specs.end(),
        [&](const NodeSpec& s) { return s.name == options_.only; });
    if (spec == specs.end()) {
      std::cerr << "launcher: --only " << options_.only
                << " names no roster member of " << options_.config_path
                << "\n";
      return 1;
    }
    if (spec->name == kHardwareNodeName || spec->name == kSimGatewayName ||
        spec->name == kStudioBridgeName) {
      std::cerr << "launcher: " << spec->name
                << " is not a talos node and has no input replay\n";
      return 1;
    }

    const std::string log_path =
        NodeLogPath(options_.output_dir, spec->name, 0);
    std::cout << "[launcher] Replaying session " << options_.replay_inputs
              << " into " << spec->name << " (" << spec->target << "); log "
              << log_path << "\n";
    const pid_t pid = SpawnProcess(
        {spec->binary_path, "--replay-inputs", options_.replay_inputs,
         "--session-id", std::to_string(options_.session_id), "--log", log_path,
         "--config", options_.config_path, "--sim"});
    if (pid < 0) {
      std::cerr << "[launcher] fork() failed for node " << spec->name << "\n";
      return 1;
    }
    int status = 0;
    while (::waitpid(pid, &status, 0) < 0) {
      if (errno != EINTR) return 1;
    }
    FinishProcessGroup(pid);
    const bool clean = WIFEXITED(status) && WEXITSTATUS(status) == 0;
    std::cout << "[launcher] " << spec->name << " finished ("
              << ExitReason(status) << ")\n";
    return clean ? 0 : (WIFEXITED(status) ? WEXITSTATUS(status) : 1);
  }

  static void InstallSignals() {
    stop_requested_ = false;
    std::signal(SIGINT, [](int) { stop_requested_ = true; });
    std::signal(SIGTERM, [](int) { stop_requested_ = true; });
  }

  // Respawns a crashed or unexpectedly exited node into a fresh event log.
  void Restart(NodeProcessInfo& proc, NodeLaunch& launch) {
    launch.restart_pending = false;
    if (proc.quarantined)
      std::cout << "[launcher] " << proc.name << " retrying after quarantine\n";
    proc.quarantined = false;
    proc.retry_at_monotonic_ns = 0;
    const int restart = proc.restart_count + 1;
    if (launch.log_arg != 0) {
      launch.args[launch.log_arg] =
          NodeLogPath(options_.output_dir, proc.name, restart);
      uint64_t used = 0;
      for (const auto& path : proc.log_paths) {
        std::error_code error;
        const auto bytes = std::filesystem::file_size(path, error);
        if (!error) used += bytes;
      }
      // 0 means unbounded to the writer; 1 means no valid header can fit.
      launch.args[launch.log_arg + 2] = std::to_string(
          used >= launch.log_budget ? 1 : launch.log_budget - used);
    }
    const pid_t pid = SpawnProcess(launch.args);
    if (pid < 0) {
      std::cerr << "[launcher] fork() failed restarting node " << proc.name
                << "\n";
      proc.failed = true;
      return;
    }
    if (launch.log_arg != 0)
      proc.log_paths.push_back(launch.args[launch.log_arg]);
    proc.restart_count = restart;
    proc.pid = pid;
    proc.exited = false;
    proc.exit_code = -1;
    launch.restart_times.push_back(std::chrono::steady_clock::now());
    launch.started_at = std::chrono::steady_clock::now();
    launch.heartbeat_changed_at = launch.started_at;
    launch.last_heartbeat = 0;
    launch.last_dispatch = 0;
    launch.heartbeat_killed = false;
  }

  // Whether any member other than `self` (and the Studio bridge, a viewer)
  // is still live -- running or awaiting a restart -- without having exited
  // 0. A clean exit's restart that falls due when none is would only rerun
  // a roster whose every member has already completed, so it is dropped.
  static bool RosterContinues(const std::vector<NodeProcessInfo>& procs,
                              const std::vector<NodeLaunch>& launches,
                              const std::optional<StudioBridge>& studio,
                              std::size_t self) {
    for (std::size_t j = 0; j < procs.size(); ++j) {
      if (j == self || (studio && j == studio->index())) continue;
      if (launches[j].last_exit_clean) continue;
      if (launches[j].restart_pending) return true;
      if (procs[j].pid > 0 && !procs[j].exited) return true;
    }
    return false;
  }

  // Waits until every node has exited, SIGINTing them all once on a signal or
  // at the deadline. A crashed node is restarted per the policy above without
  // blocking the loop; a pending restart holds the session open until it
  // fires or the session stops. A bridge that is still starting does not hold
  // it open.
  void Supervise(std::vector<NodeProcessInfo>& procs,
                 std::vector<NodeLaunch>& launches,
                 std::optional<StudioBridge>& studio,
                 std::chrono::steady_clock::time_point deadline) {
    bool stop_sent = false;
    auto shutdown_deadline = std::chrono::steady_clock::time_point::max();
    std::optional<introspect::RegistryMapping> registry;
    const auto started = std::chrono::steady_clock::now();
    for (auto& launch : launches) {
      launch.started_at = started;
      launch.heartbeat_changed_at = started;
    }
    ServiceNotifier notifier;
    notifier.Send("READY=1");
    for (;;) {
      notifier.Send("WATCHDOG=1");
      const auto now = std::chrono::steady_clock::now();
      const bool past_deadline = now >= deadline;
      const bool stopping = stop_requested_ != 0 || stop_sent || past_deadline;
      if (!registry) {
        try {
          registry.emplace(introspect::kRegistryPath, true);
        } catch (const std::runtime_error&) {
          // Nodes create the registry asynchronously; never create it here.
          // Left empty, the next pass retries the attach.
          registry.reset();
        }
      }
      for (std::size_t i = 0; i < procs.size(); ++i) {
        auto& proc = procs[i];
        auto& launch = launches[i];
        while (!launch.restart_times.empty() &&
               now - launch.restart_times.front() >=
                   std::chrono::milliseconds{options_.restart_window_ms}) {
          launch.restart_times.pop_front();
        }
        if (proc.pid <= 0 || proc.exited) continue;
        if (!stopping && !launch.heartbeat_killed &&
            !(studio && i == studio->index()) && proc.name != kSimGatewayName) {
          bool found = false;
          bool constructing = false;
          if (registry) {
            for (const auto& record : registry->header()->nodes) {
              if (record.state.load(std::memory_order_acquire) !=
                  static_cast<uint64_t>(introspect::SlotState::ACTIVE))
                continue;
              const auto generation =
                  record.generation.load(std::memory_order_acquire);
              if (record.pid != static_cast<uint64_t>(proc.pid) ||
                  record.session_id != options_.session_id)
                continue;
              const auto beat =
                  record.heartbeat_monotonic_ns.load(std::memory_order_relaxed);
              const auto dispatch =
                  record.dispatch_count.load(std::memory_order_relaxed);
              const auto flags = record.flags.load(std::memory_order_acquire);
              if (record.generation.load(std::memory_order_acquire) !=
                      generation ||
                  record.state.load(std::memory_order_acquire) !=
                      static_cast<uint64_t>(introspect::SlotState::ACTIVE))
                continue;
              found = true;
              constructing = (flags & introspect::kFlagConstructing) != 0;
              if (beat != launch.last_heartbeat &&
                  (constructing || dispatch != launch.last_dispatch)) {
                launch.heartbeat_changed_at = now;
                launch.last_heartbeat = beat;
                launch.last_dispatch = dispatch;
              }
              break;
            }
          }
          const bool grace_over =
              now - launch.started_at >=
              std::chrono::milliseconds{options_.heartbeat_startup_grace_ms};
          // Registration is part of startup, not an opt-in to supervision:
          // until a row arrives, the spawn time is the last progress time.
          const bool construction_expired =
              constructing &&
              now - launch.started_at >= options_.construction_timeout;
          if (construction_expired ||
              (grace_over &&
               now - launch.heartbeat_changed_at >= launch.heartbeat_timeout)) {
            std::cerr << "[launcher] " << proc.name
                      << (construction_expired
                              ? " construction deadline exceeded"
                          : found ? " heartbeat/dispatch stalled"
                                  : " heartbeat missing")
                      << "; killing pid " << proc.pid << "\n";
            SignalProcessGroup(proc.pid, SIGKILL);
            launch.heartbeat_killed = true;
          }
        }
        int status = 0;
        if (::waitpid(proc.pid, &status, WNOHANG) != proc.pid) continue;
        FinishProcessGroup(proc.pid);
        proc.exited = true;
        proc.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
        PersistManifest(procs);
        const bool is_bridge = studio && i == studio->index();
        if (is_bridge && studio->Starting(i)) continue;
        const bool clean = WIFEXITED(status) && WEXITSTATUS(status) == 0;
        launch.last_exit_clean = clean;
        if (!is_bridge && ShouldRestartNode(clean, launch.critical,
                                            max_restarts_, stopping)) {
          const int recent = static_cast<int>(launch.restart_times.size());
          const auto backoff = RestartBackoff(recent + 1);
          proc.quarantined = !launch.critical && recent >= max_restarts_;
          launch.restart_pending = true;
          launch.restart_at =
              (proc.quarantined
                   ? launch.restart_times.front() +
                         std::chrono::milliseconds{options_.restart_window_ms}
                   : now) +
              backoff;
          proc.retry_at_monotonic_ns =
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  launch.restart_at.time_since_epoch())
                  .count();
          const auto delay =
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  launch.restart_at - now);
          std::cout << "[launcher] " << proc.name << " exited ("
                    << ExitReason(status) << "); "
                    << (proc.quarantined ? "quarantined; retry" : "restart")
                    << " " << proc.restart_count + 1 << " in " << delay.count()
                    << " ms\n";
          PersistManifest(procs);
          continue;
        }
        std::cout << "[launcher] Process " << proc.name << " (pid " << proc.pid
                  << ") exited with code " << proc.exit_code;
        std::cout << "\n";
      }
      for (std::size_t i = 0; i < procs.size(); ++i) {
        auto& launch = launches[i];
        if (!launch.restart_pending) continue;
        // A stopping session ends with the crash as the node's last word.
        if (stopping) {
          launch.restart_pending = false;
          procs[i].quarantined = false;
          procs[i].retry_at_monotonic_ns = 0;
          PersistManifest(procs);
        } else if (now >= launch.restart_at) {
          if (launch.last_exit_clean &&
              !RosterContinues(procs, launches, studio, i)) {
            launch.restart_pending = false;
            procs[i].retry_at_monotonic_ns = 0;
            std::cout << "[launcher] " << procs[i].name
                      << " finished with the rest of the roster; not "
                         "restarted\n";
          } else {
            Restart(procs[i], launch);
          }
          PersistManifest(procs);
        }
      }
      if (studio) {
        const auto& before = procs[studio->index()];
        const auto pid = before.pid;
        const bool exited = before.exited;
        const bool failed = before.failed;
        studio->Supervise(procs[studio->index()], now, stopping);
        const auto& after = procs[studio->index()];
        if (pid != after.pid || exited != after.exited ||
            failed != after.failed)
          PersistManifest(procs);
      }

      bool all_done = true;
      for (std::size_t i = 0; i < procs.size(); ++i) {
        const auto& proc = procs[i];
        if (launches[i].restart_pending) {
          all_done = false;
          break;
        }
        if (proc.pid <= 0 || proc.exited) continue;
        if (studio && studio->Starting(i)) continue;
        all_done = false;
        break;
      }

      if (all_done) {
        if (studio) {
          studio->Finish(procs[studio->index()], stop_requested_ != 0);
        }
        return;
      }

      if (!stop_sent && (stop_requested_ || past_deadline)) {
        stop_sent = true;
        shutdown_deadline = now + std::chrono::seconds{2};
        std::cout << "[launcher] Stopping children...\n";
        for (const auto& proc : procs) {
          if (proc.pid > 0 && !proc.exited)
            SignalProcessGroup(proc.pid, SIGINT);
        }
      }
      if (stop_sent && now >= shutdown_deadline) {
        for (const auto& proc : procs) {
          if (proc.pid > 0 && !proc.exited)
            SignalProcessGroup(proc.pid, SIGKILL);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  // Phase one's report, warnings included.
  static void PrintDeclaredGraph(
      const std::vector<NodeDescription>& described,
      const std::vector<naming::Diagnostic>& diagnostics,
      std::size_t declared_topics, const std::string& graph_path) {
    std::size_t answered = 0;
    for (const auto& node : described) answered += node.answered ? 1 : 0;

    std::cout << "\n================ Declared Graph ==================\n";
    std::cout << "Nodes described: " << answered << "/" << described.size()
              << ", topics: " << declared_topics << "\n";
    for (const auto& node : described) {
      std::cout << "  " << (node.answered ? "\u2022" : "?") << " "
                << node.roster_name;
      if (node.answered) {
        std::cout << " (" << node.declared.sources.size() << " sources)\n";
      } else {
        std::cout << " (not described)\n";
      }
    }
    for (const auto& diagnostic : diagnostics) {
      auto& out = diagnostic.severity == naming::Severity::ERROR ? std::cerr
                                                                 : std::cout;
      out << "  " << diagnostic.ToString() << "\n";
    }
    std::cout << "Graph written to: " << graph_path << "\n";
    std::cout << "==================================================\n";
  }

  static void PrintManifest(const SessionManifest& manifest) {
    std::size_t described = 0;
    std::string roster;
    for (const auto& n : manifest.nodes) {
      if (!n.described) continue;
      ++described;
      if (!roster.empty()) roster += ", ";
      roster += n.name;
    }

    std::cout << "\n================ Session Manifest ================\n";
    std::cout << "Session ID: " << manifest.session_id << "\n";
    std::cout << "Output Dir: " << manifest.output_dir << "\n";
    std::cout << "Declared: " << described << "/" << manifest.nodes.size()
              << " nodes answered --describe, " << manifest.declared_topics
              << " topics\n";
    std::cout << "Declared nodes: " << (roster.empty() ? "none" : roster)
              << "\n";
    std::cout << "Nodes (" << manifest.nodes.size() << "):\n";
    for (const auto& n : manifest.nodes) {
      std::cout << "  \u2022 " << n.name << " (pid=" << n.pid
                << ", exit=" << n.exit_code << ", records=" << n.record_count
                << ", restarts=" << n.restart_count
                << ", failed=" << (n.failed ? "yes" : "no")
                << ", log=" << n.log_path << ")\n";
    }
    std::cout << "Manifest written to: " << manifest.output_dir
              << "/manifest.json\n";
    std::cout << "==================================================\n";
  }

  LauncherOptions options_;
  SessionManifest session_manifest_;
  bool manifest_failing_{false};
  int max_restarts_{kDefaultMaxRestarts};
  static inline volatile std::sig_atomic_t stop_requested_{false};
};

// ---------------------------------------------------------------------------
// Log merge (log_dump)
// ---------------------------------------------------------------------------

struct MergedRecord {
  std::string process_name;
  std::string source_name;
  // The record's source in its log's manifest, schema included (format.h);
  // null for an EXIT record or an id the manifest lacks.
  std::shared_ptr<const event::Registration> source;
  event::log::RecordHeader header;
  std::vector<std::byte> payload;
};

// `source_id`'s entry in `manifest`, sharing its ownership; null if none.
inline std::shared_ptr<const event::Registration> ResolveSource(
    const std::shared_ptr<const event::Manifest>& manifest,
    std::uint16_t source_id) {
  for (const auto& reg : *manifest) {
    if (reg.id == source_id) return {manifest, &reg};
  }
  return nullptr;
}

// Merges logs of one session by event time, then now, then dispatch index.
inline bool MergeLogs(const std::vector<std::string>& log_paths,
                      std::vector<MergedRecord>& out_records,
                      std::string& error) {
  out_records.clear();
  error.clear();
  if (log_paths.empty()) {
    error = "no log files provided";
    return false;
  }

  struct ReaderState {
    std::unique_ptr<event::log::LogReader> reader;
    std::shared_ptr<const event::Manifest> manifest;
    event::log::LogReader::Record current;
    bool has_record{false};
  };
  std::vector<ReaderState> states;
  states.reserve(log_paths.size());

  for (const auto& path : log_paths) {
    ReaderState state;
    try {
      state.reader = std::make_unique<event::log::LogReader>(path);
    } catch (const std::exception& e) {
      error = "failed to open " + path + ": " + e.what();
      return false;
    }
    if (!states.empty() &&
        state.reader->session_id() != states.front().reader->session_id()) {
      error = "session_id mismatch: file '" + path + "' has session_id " +
              std::to_string(state.reader->session_id()) + ", expected " +
              std::to_string(states.front().reader->session_id());
      return false;
    }
    state.manifest =
        std::make_shared<const event::Manifest>(state.reader->manifest());
    state.has_record = state.reader->next(state.current);
    states.push_back(std::move(state));
  }

  const auto key = [](const ReaderState& state) {
    const auto& header = state.current.header;
    return std::tuple{header.event_time_ns, header.now_ns,
                      header.dispatch_index};
  };
  for (;;) {
    ReaderState* best = nullptr;
    for (auto& state : states) {
      if (state.has_record && (best == nullptr || key(state) < key(*best))) {
        best = &state;
      }
    }
    if (best == nullptr) return true;

    MergedRecord record;
    record.process_name = best->reader->process_name();
    if (best->current.header.kind !=
        static_cast<std::uint16_t>(event::EventKind::EXIT)) {
      record.source =
          ResolveSource(best->manifest, best->current.header.source_id);
    }
    record.source_name = record.source ? record.source->name : "?";
    record.header = best->current.header;
    record.payload.assign(best->current.payload.begin(),
                          best->current.payload.end());
    out_records.push_back(std::move(record));
    best->has_record = best->reader->next(best->current);
  }
}

}  // namespace talos::launcher
