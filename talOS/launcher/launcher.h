#pragma once

#include <poll.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
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
#include "talOS/events/log/log_reader.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
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
  bool failed{false};
  uint64_t record_count{0};
  // Whether this binary answered --describe: a session whose graph was never
  // checked otherwise reads exactly like one whose graph passed.
  bool described{false};
};

struct SessionManifest {
  uint64_t session_id{0};
  std::string config_path;
  // sha256 over the robot config plus every subsystem file, from the bundle
  // manifest. Empty for non-bundle launches.
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
      ss << "      \"recorder_failed\": " << (n.failed ? "true" : "false")
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
  // Sim sessions only: run the Studio viewer bridge as a supervised member of
  // the session (see "Studio bridge" below and [studio] in the robot config).
  bool start_studio_bridge{false};

  // Controller addressing. Empty/zero means "take it from [robot]".
  std::string remote;
  uint16_t remote_port{0};
  uint16_t local_port{0};

  // Studio HTTP port override. Zero means "take [studio] http_port".
  uint16_t studio_http_port{0};

  // Restart budget per crashed node. Unset means "take [robot] max_restarts".
  std::optional<int> max_restarts;

  // Launch a graph the linter called broken anyway. The default is refusal:
  // two spellings of one topic are silent at runtime.
  bool allow_graph_errors{false};

  // Probe and lint without spawning anything, as a pre-flight check.
  bool describe_only{false};

  // How long one binary gets to answer --describe.
  int describe_timeout_ms{5000};

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
          for (const auto& key : keys) {
            options.binary_overrides[key] = (root / *binary).string();
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
          specs.push_back(MakeNodeSpec(key.str(), *node, options));
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

// fork + execv of args[0]. With an output path, the child's stdout and stderr
// are appended there. Returns the child pid, or -1 when fork fails.
inline pid_t SpawnProcess(std::vector<std::string> args,
                          const std::string& output_path = "") {
  // The child inherits unflushed stdio buffers, and freopen below flushes them
  // into the launcher's own log a second time. Empty them before forking.
  std::cout.flush();
  std::fflush(nullptr);
  const pid_t pid = ::fork();
  if (pid != 0) return pid;
  if (!output_path.empty()) {
    std::freopen(output_path.c_str(), "a", stdout);
    std::freopen(output_path.c_str(), "a", stderr);
  }
  std::vector<char*> c_args;
  c_args.reserve(args.size() + 1);
  for (auto& arg : args) c_args.push_back(arg.data());
  c_args.push_back(nullptr);
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

  std::vector<std::string> owned = args;
  std::vector<char*> c_args;
  c_args.reserve(owned.size() + 1);
  for (auto& arg : owned) c_args.push_back(arg.data());
  c_args.push_back(nullptr);

  const pid_t pid = ::fork();
  if (pid < 0) {
    error = std::string{"fork() failed: "} + std::strerror(errno);
    ::close(fds[0]);
    ::close(fds[1]);
    return false;
  }
  if (pid == 0) {
    ::close(fds[0]);
    ::dup2(fds[1], STDOUT_FILENO);
    ::close(fds[1]);
    ::execv(binary_path.c_str(), c_args.data());
    ::_exit(127);
  }
  ::close(fds[1]);

  std::string text;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  bool timed_out = false;
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
    if (got == 0) break;  // EOF: the child closed stdout, so it is finished.
    text.append(buffer, static_cast<std::size_t>(got));
  }
  ::close(fds[0]);

  if (timed_out || !error.empty()) ::kill(pid, SIGKILL);
  int status = 0;
  while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
  }

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

// Phase one of a launch: ask every binary in the roster what it is, assemble
// the graph they describe, and lint it. The binaries, not the config, are the
// source: a second declaration of the topology is a thing to forget to update.
inline std::vector<naming::Diagnostic> DescribeGraph(
    const std::vector<NodeSpec>& specs, const LauncherOptions& options,
    std::vector<NodeDescription>& out) {
  out.clear();
  out.reserve(specs.size());

  std::vector<naming::NodeShape> shapes;
  std::vector<naming::Diagnostic> probe_findings;

  for (const auto& spec : specs) {
    NodeDescription described;
    described.roster_name = spec.name;
    described.target = spec.target;
    described.binary_path = spec.binary_path;

    // A node whose topics depend on its configuration must describe the
    // configuration this session will use.
    std::vector<std::string> args{spec.binary_path, "--describe", "--config",
                                  options.config_path};
    if (options.simulation) args.push_back("--sim");

    described.answered =
        ProbeDescribe(spec.binary_path, args,
                      std::chrono::milliseconds{options.describe_timeout_ms},
                      described.declared, described.error);

    if (described.answered) {
      shapes.push_back({described.declared.name, described.declared.target,
                        described.declared.sources});
      if (described.declared.name != spec.name) {
        probe_findings.push_back({naming::Severity::WARNING, spec.name,
                                  "the config calls this node '" + spec.name +
                                      "' but the binary describes itself as '" +
                                      described.declared.name + "'"});
      }
    } else {
      // A warning: a node someone is halfway through writing stays launchable.
      probe_findings.push_back(
          {naming::Severity::WARNING, spec.name,
           "did not answer --describe (" + described.error +
               "), so its topics are missing from the declared graph"});
    }
    out.push_back(std::move(described));
  }

  auto diagnostics = naming::LintGraph(shapes);
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
// backoff of 50 ms, doubling per restart, capped at 1 s, at most max_restarts
// times. A clean exit is a node that finished and stays finished. The backoff
// is scheduled inside the supervision loop, so a restarting node never delays
// another node or the session deadline. Every run writes its own event log.
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

// True for a crash the budget still covers in a session that is not stopping.
inline bool ShouldRestartNode(bool clean_exit, int restarts_made,
                              int max_restarts, bool stopping) {
  return !clean_exit && !stopping && restarts_made < max_restarts;
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
// same `[subsystems.telemetry] topic` key the telemetry node publishes on), or
// that key itself when the bridge did not answer.
inline std::string StudioBridgeWatchTopic(
    const std::vector<NodeDescription>& described,
    const config::RobotConfig& robot_config) {
  std::string topic =
      robot_config.toml_data["subsystems"]["telemetry"]["topic"].value_or(
          std::string{"/talos/telemetry"});
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
              "--declared",
              graph_path} {
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
      ::kill(proc.pid, SIGINT);
      int status = 0;
      ::waitpid(proc.pid, &status, WNOHANG);
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
    if (options_.output_dir.empty()) {
      options_.output_dir =
          "/tmp/talos_logs/" + std::to_string(options_.session_id);
    }
  }

  // Lints the declared graph, then runs the roster until every node exits,
  // the duration passes or a signal arrives, and writes manifest.json.
  // Returns 0, or the last nonzero exit code of a node.
  int Run() {
    std::string bundle_error;
    if (!LoadBundle(options_, bundle_error)) {
      std::cerr << "launcher: " << bundle_error << "\n";
      return 1;
    }
    if (options_.config_path.empty()) {
      std::cerr << "launcher: --config PATH or --bundle PATH is required\n";
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
    std::string restart_error;
    if (!ResolveMaxRestarts(robot_config, options_, max_restarts_,
                            restart_error)) {
      std::cerr << "launcher: " << restart_error << "\n";
      return 1;
    }

    const auto node_specs = DiscoverNodes(robot_config, options_);
    if (node_specs.empty()) {
      std::cerr << "launcher: no nodes declared in config\n";
      return 1;
    }

    // Phase one: describe and lint before anything starts.
    std::vector<NodeDescription> described;
    const auto diagnostics = DescribeGraph(node_specs, options_, described);
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

    // Phase two.
    SessionManifest manifest;
    manifest.session_id = options_.session_id;
    manifest.config_path = options_.config_path;
    manifest.config_sha256 = options_.bundle_config_sha256;
    manifest.output_dir = options_.output_dir;
    manifest.simulation = options_.simulation;
    manifest.declared_topics = declared_topics;
    manifest.start_wall_ns = WallNanos();

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
      auto& proc = procs[i];
      auto& args = launches[i].args;
      args.push_back(node_specs[i].binary_path);
      if (proc.name == kSimGatewayName) {
        // The gateway stamps the config's config_id; unconfigured it stamps
        // 0, which every mechanism rejects.
        args.insert(args.end(), {"--config", options_.config_path});
      } else {
        args.insert(args.end(), {"--session-id",
                                 std::to_string(options_.session_id), "--log"});
        launches[i].log_arg = args.size();
        args.insert(args.end(),
                    {proc.log_path, "--config", options_.config_path});
        if (options_.simulation) args.push_back("--sim");
        if (proc.name == kHardwareNodeName) {
          args.insert(args.end(), {"--remote", remote, "--remote-port",
                                   std::to_string(remote_port), "--local-port",
                                   std::to_string(local_port)});
        }
      }
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
      std::cout << "[launcher] Started " << proc.name << " (pid " << pid
                << ", target " << proc.target << ")\n";
    }
    if (studio) studio->Start(procs[studio->index()]);

    Supervise(procs, launches, studio, deadline);
    manifest.end_wall_ns = WallNanos();

    int exit_status = 0;
    for (auto& proc : procs) {
      // Every run's log: a restarted node's records are the sum of its runs.
      for (const auto& path : proc.log_paths) {
        if (!std::filesystem::exists(path)) continue;
        try {
          event::log::LogReader reader{path};
          event::log::LogReader::Record record;
          while (reader.next(record)) {
          }
          proc.record_count += reader.record_count();
          if (reader.truncated()) proc.failed = true;
        } catch (...) {
          proc.failed = true;
        }
      }
      if (proc.exit_code != 0) exit_status = proc.exit_code;
    }
    manifest.nodes = std::move(procs);

    std::ofstream{options_.output_dir + "/manifest.json"} << manifest.ToJson();
    PrintManifest(manifest);
    return exit_status;
  }

 private:
  // How one roster member is spawned, and its pending restart.
  struct NodeLaunch {
    std::vector<std::string> args;
    std::size_t log_arg{0};  // Index of the --log value in args; 0 for none.
    bool restart_pending{false};
    std::chrono::steady_clock::time_point restart_at{};
  };

  static int64_t WallNanos() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }

  static void InstallSignals() {
    stop_requested_ = false;
    std::signal(SIGINT, [](int) { stop_requested_ = true; });
    std::signal(SIGTERM, [](int) { stop_requested_ = true; });
  }

  // Respawns a crashed node into a fresh event log.
  void Restart(NodeProcessInfo& proc, NodeLaunch& launch) {
    launch.restart_pending = false;
    const int restart = proc.restart_count + 1;
    if (launch.log_arg != 0) {
      launch.args[launch.log_arg] =
          NodeLogPath(options_.output_dir, proc.name, restart);
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
    for (;;) {
      const auto now = std::chrono::steady_clock::now();
      const bool past_deadline = now >= deadline;
      const bool stopping = stop_requested_ != 0 || stop_sent || past_deadline;
      for (std::size_t i = 0; i < procs.size(); ++i) {
        auto& proc = procs[i];
        if (proc.pid <= 0 || proc.exited) continue;
        int status = 0;
        if (::waitpid(proc.pid, &status, WNOHANG) != proc.pid) continue;
        proc.exited = true;
        proc.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
        const bool is_bridge = studio && i == studio->index();
        if (is_bridge && studio->Starting(i)) continue;
        const bool clean = WIFEXITED(status) && WEXITSTATUS(status) == 0;
        if (!is_bridge && ShouldRestartNode(clean, proc.restart_count,
                                            max_restarts_, stopping)) {
          const int restart = proc.restart_count + 1;
          const auto backoff = RestartBackoff(restart);
          launches[i].restart_pending = true;
          launches[i].restart_at = now + backoff;
          std::cout << "[launcher] " << proc.name << " exited ("
                    << ExitReason(status) << "); restart " << restart << "/"
                    << max_restarts_ << " in " << backoff.count() << " ms\n";
          continue;
        }
        std::cout << "[launcher] Process " << proc.name << " (pid " << proc.pid
                  << ") exited with code " << proc.exit_code;
        if (!is_bridge && !clean && !stopping && max_restarts_ > 0) {
          std::cout << "; restart limit (" << max_restarts_ << ") reached";
        }
        std::cout << "\n";
      }
      for (std::size_t i = 0; i < procs.size(); ++i) {
        auto& launch = launches[i];
        if (!launch.restart_pending) continue;
        // A stopping session ends with the crash as the node's last word.
        if (stopping) {
          launch.restart_pending = false;
        } else if (now >= launch.restart_at) {
          Restart(procs[i], launch);
        }
      }
      if (studio) studio->Supervise(procs[studio->index()], now, stopping);

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
        std::cout << "[launcher] Stopping children...\n";
        for (const auto& proc : procs) {
          if (proc.pid > 0 && !proc.exited) ::kill(proc.pid, SIGINT);
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
  int max_restarts_{kDefaultMaxRestarts};
  static inline volatile std::sig_atomic_t stop_requested_{false};
};

// ---------------------------------------------------------------------------
// Log merge (log_dump)
// ---------------------------------------------------------------------------

struct MergedRecord {
  std::string process_name;
  std::string source_name;
  event::log::RecordHeader header;
  std::vector<std::byte> payload;
};

inline std::string ResolveSourceName(const event::Manifest& manifest,
                                     std::uint16_t source_id) {
  for (const auto& reg : manifest) {
    if (reg.id == source_id) return reg.name;
  }
  return "?";
}

// A session directory's event logs (every `*.tlog` in it, which is where the
// launcher puts each node's log), or a single log file as given.
inline std::vector<std::string> FindLogFiles(const std::string& path_or_dir) {
  std::error_code ec;
  if (!std::filesystem::is_directory(path_or_dir, ec)) return {path_or_dir};
  std::vector<std::string> result;
  for (const auto& entry :
       std::filesystem::directory_iterator(path_or_dir, ec)) {
    if (entry.path().extension() == ".tlog") {
      result.push_back(entry.path().string());
    }
  }
  std::sort(result.begin(), result.end());
  return result;
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
    record.source_name = ResolveSourceName(best->reader->manifest(),
                                           best->current.header.source_id);
    record.header = best->current.header;
    record.payload.assign(best->current.payload.begin(),
                          best->current.payload.end());
    out_records.push_back(std::move(record));
    best->has_record = best->reader->next(best->current);
  }
}

}  // namespace talos::launcher
