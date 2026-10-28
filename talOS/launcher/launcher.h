#pragma once

#include <poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"

namespace talos::launcher {

inline uint64_t GenerateSessionId() {
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
  std::random_device rd;
  const uint64_t r = (static_cast<uint64_t>(rd()) << 32) | rd();
  uint64_t id = static_cast<uint64_t>(now) ^ r;
  return id == 0 ? 1 : id;
}

struct NodeSpec {
  std::string name;
  std::string target;  // e.g. "//2026-robot/main_processor/drivetrain:node"
  std::string binary_path;  // Resolved executable path
  std::vector<std::string> extra_args;
};

struct NodeProcessInfo {
  std::string name;
  std::string target;
  std::string binary_path;
  // Contract label recorded in the bundle manifest (e.g.
  // "//2026-robot/main_processor/drivetrain:contract"). Traceability, not a
  // compatibility proof: a matching label does not establish wire
  // compatibility.
  std::string contract;
  std::string log_path;
  pid_t pid{-1};
  int exit_code{-1};
  bool exited{false};
  bool failed{false};
  uint64_t record_count{0};

  // Whether this binary answered --describe. Recorded because a session whose
  // graph was never checked reads exactly like one whose graph passed.
  bool described{false};
};

struct SessionManifest {
  uint64_t session_id{0};
  std::string config_path;
  // sha256 over the robot config plus every subsystem file, as recorded in
  // the bundle manifest. Empty for non-bundle launches.
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
    ss << "  \"config_path\": \"" << config_path << "\",\n";
    ss << "  \"config_sha256\": \"" << config_sha256 << "\",\n";
    ss << "  \"output_dir\": \"" << output_dir << "\",\n";
    ss << "  \"simulation\": " << (simulation ? "true" : "false") << ",\n";
    ss << "  \"start_wall_ns\": " << start_wall_ns << ",\n";
    ss << "  \"end_wall_ns\": " << end_wall_ns << ",\n";
    ss << "  \"nodes\": [\n";
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      const auto& n = nodes[i];
      ss << "    {\n";
      ss << "      \"name\": \"" << n.name << "\",\n";
      ss << "      \"target\": \"" << n.target << "\",\n";
      ss << "      \"binary_path\": \"" << n.binary_path << "\",\n";
      ss << "      \"contract\": \"" << n.contract << "\",\n";
      ss << "      \"log_path\": \"" << n.log_path << "\",\n";
      ss << "      \"pid\": " << n.pid << ",\n";
      ss << "      \"exit_code\": " << n.exit_code << ",\n";
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
  // No default: the framework names no robot (ARCHITECTURE.md Rule 1), so
  // the caller must say which config to launch. Empty means missing.
  std::string config_path;
  std::string output_dir;
  uint64_t session_id{0};
  bool simulation{false};
  int duration_s{0};
  bool start_sim_gateway{false};
  // Studio viewer bridge (sim sessions only): --start-studio-bridge starts
  // //studio/bridge:studio_bridge under launcher supervision, so the bridge
  // is a declared node in graph.json instead of a process sim.sh runs behind
  // the launcher's back. See [studio] in the robot config for ports/webroot.
  bool start_studio_bridge{false};

  // Controller addressing. Empty/zero means "take it from the robot config":
  // --remote overrides [robot] controller_address, --remote-port overrides
  // [robot] controller_port, --local-port overrides [robot] local_port.
  std::string remote;
  uint16_t remote_port{0};
  uint16_t local_port{0};

  // Studio HTTP port override. Zero means "take [studio] http_port".
  uint16_t studio_http_port{0};

  // Launch a graph the linter called broken anyway. The default is refusal,
  // because what the lint catches -- a publisher and a subscriber on two
  // spellings of one name -- is silent at runtime: both ends work perfectly and
  // never meet, so nothing downstream will ever say so. But a robot in a
  // competition queue must be able to run past a lint failure, so there is a
  // way to say that out loud rather than by editing the launcher.
  bool allow_graph_errors{false};

  // Probe and lint without spawning anything, as a pre-flight check.
  bool describe_only{false};

  // How long one binary gets to answer --describe. A node that hangs there must
  // cost the launch this much and no more.
  int describe_timeout_ms{5000};

  // Path to a bundle directory assembled by talos_bundle
  std::string bundle_dir;

  // Provenance read from bundle_manifest.json by LoadBundle: the config
  // identity hash and the per-node contract labels (keyed by target, with a
  // name fallback). Carried into the session manifest, never trusted as a
  // compatibility proof.
  std::string bundle_config_sha256;
  std::map<std::string, std::string> bundle_contracts;

  // Map of target/name -> binary override, useful for testing
  std::map<std::string, std::string> binary_overrides;
};

inline bool LoadBundle(LauncherOptions& options, std::string& error) {
  if (options.bundle_dir.empty()) {
    if (std::filesystem::exists("bundle_manifest.json")) {
      options.bundle_dir = ".";
    } else {
      return true;
    }
  }

  std::filesystem::path bpath(options.bundle_dir);
  if (!std::filesystem::exists(bpath)) {
    error = "bundle directory does not exist: " + options.bundle_dir;
    return false;
  }

  // Set library search paths so child processes find bundled libraries
  std::string lib_dir = (bpath / "lib").string();
  if (std::filesystem::exists(lib_dir)) {
    const char* old_ld = std::getenv("LD_LIBRARY_PATH");
    std::string new_ld =
        lib_dir + (old_ld && *old_ld ? std::string(":") + old_ld : "");
    ::setenv("LD_LIBRARY_PATH", new_ld.c_str(), 1);

    const char* old_dyld = std::getenv("DYLD_LIBRARY_PATH");
    std::string new_dyld =
        lib_dir + (old_dyld && *old_dyld ? std::string(":") + old_dyld : "");
    ::setenv("DYLD_LIBRARY_PATH", new_dyld.c_str(), 1);

    if (std::getenv("TALOS_NODE_LIB") == nullptr) {
      std::string node_so = lib_dir + "/libtalos_node.so";
      if (std::filesystem::exists(node_so)) {
        ::setenv("TALOS_NODE_LIB", node_so.c_str(), 1);
      }
    }
  }

  // Parse bundle_manifest.json if present
  std::string manifest_file = (bpath / "bundle_manifest.json").string();
  if (std::filesystem::exists(manifest_file)) {
    std::ifstream mf(manifest_file);
    std::string content((std::istreambuf_iterator<char>(mf)),
                        std::istreambuf_iterator<char>());

    if (options.config_path.empty()) {
      auto pos = content.find("\"config_path\": \"");
      if (pos != std::string::npos) {
        auto start = pos + 16;
        auto end = content.find("\"", start);
        if (end != std::string::npos) {
          options.config_path =
              (bpath / content.substr(start, end - start)).string();
        }
      }
    }

    {
      auto pos = content.find("\"config_sha256\": \"");
      if (pos != std::string::npos) {
        auto start = pos + 18;
        auto end = content.find("\"", start);
        if (end != std::string::npos) {
          options.bundle_config_sha256 = content.substr(start, end - start);
        }
      }
    }

    std::size_t search_pos = 0;
    while ((search_pos = content.find("\"name\": \"", search_pos)) !=
           std::string::npos) {
      auto name_start = search_pos + 9;
      auto name_end = content.find("\"", name_start);
      if (name_end == std::string::npos) break;
      std::string node_name =
          content.substr(name_start, name_end - name_start);

      std::string node_target;
      auto target_pos = content.find("\"target\": \"", name_end);
      if (target_pos != std::string::npos &&
          target_pos < content.find("}", name_end)) {
        auto t_start = target_pos + 11;
        auto t_end = content.find("\"", t_start);
        if (t_end != std::string::npos) {
          node_target = content.substr(t_start, t_end - t_start);
        }
      }

      std::string binary_path;
      auto bin_pos = content.find("\"binary_path\": \"", name_end);
      if (bin_pos != std::string::npos &&
          bin_pos < content.find("}", name_end)) {
        auto b_start = bin_pos + 16;
        auto b_end = content.find("\"", b_start);
        if (b_end != std::string::npos) {
          binary_path = content.substr(b_start, b_end - b_start);
        }
      }

      std::string node_contract;
      auto contract_pos = content.find("\"contract\": \"", name_end);
      if (contract_pos != std::string::npos &&
          contract_pos < content.find("}", name_end)) {
        auto c_start = contract_pos + 13;
        auto c_end = content.find("\"", c_start);
        if (c_end != std::string::npos) {
          node_contract = content.substr(c_start, c_end - c_start);
        }
      }
      if (!node_contract.empty()) {
        if (!node_target.empty()) {
          options.bundle_contracts[node_target] = node_contract;
        }
        options.bundle_contracts[node_name] = node_contract;
      }

      if (!binary_path.empty()) {
        std::string full_bin = (bpath / binary_path).string();
        if (!node_target.empty()) {
          options.binary_overrides[node_target] = full_bin;
        }
        options.binary_overrides[node_name] = full_bin;
      }

      search_pos = name_end + 1;
    }
  }

  if (options.config_path.empty()) {
    std::string default_cfg = (bpath / "config" / "robot.toml").string();
    if (std::filesystem::exists(default_cfg)) {
      options.config_path = default_cfg;
    }
  }

  return true;
}

// Resolves a node target to an executable path: an explicit override (bundle
// manifest or test) wins, then the target's bazel-bin output relative to the
// workspace. Anything that is not a `//pkg:target` label is taken as a path.
inline std::string ResolveBinary(
    const std::string& target_or_path,
    const std::map<std::string, std::string>& overrides = {}) {
  if (auto it = overrides.find(target_or_path); it != overrides.end()) {
    return it->second;
  }
  if (!target_or_path.starts_with("//")) return target_or_path;
  std::string subpath = target_or_path.substr(2);
  std::replace(subpath.begin(), subpath.end(), ':', '/');
  return "bazel-bin/" + subpath;
}

// ResolveBinary, falling back to an override keyed by the node's name (a
// bundle manifest records both) when the target does not resolve.
inline NodeSpec MakeNodeSpec(const std::string& name, const std::string& target,
                             const LauncherOptions& options) {
  NodeSpec spec;
  spec.name = name;
  spec.target = target;
  spec.binary_path = ResolveBinary(target, options.binary_overrides);
  if (::access(spec.binary_path.c_str(), X_OK) != 0) {
    auto it = options.binary_overrides.find(name);
    if (it != options.binary_overrides.end()) spec.binary_path = it->second;
  }
  return spec;
}

inline std::vector<NodeSpec> DiscoverNodes(
    const config::RobotConfig& robot_config, const LauncherOptions& options) {
  std::vector<NodeSpec> specs;

  // 1. Hardware node (bridge)
  std::string hw_target = "//talOS/bridge:hardware_node";
  if (auto* robot_tbl = robot_config.toml_data["robot"].as_table()) {
    if (auto node_field = (*robot_tbl)["hardware_node"]) {
      if (auto val = node_field.value<std::string_view>()) {
        hw_target = std::string(*val);
      }
    }
  }
  specs.push_back(MakeNodeSpec("hardware_node", hw_target, options));

  // 2. Subsystems
  if (auto* subs = robot_config.toml_data["subsystems"].as_table()) {
    for (auto&& [sub_key, sub_val] : *subs) {
      if (auto* sub_tbl = sub_val.as_table()) {
        if (auto node_field = (*sub_tbl)["node"]) {
          if (auto node_val = node_field.value<std::string_view>()) {
            specs.push_back(MakeNodeSpec(std::string(sub_key.str()),
                                         std::string(*node_val), options));
          }
        }
      }
    }
  }

  // 3. Optional sim_gateway. The gateway stands in for a controller processor,
  // so which binary that is belongs to the robot, not to the framework.
  if (options.start_sim_gateway && options.simulation) {
    std::string gw_target;
    if (auto* robot_tbl = robot_config.toml_data["robot"].as_table()) {
      if (auto gw_field = (*robot_tbl)["sim_gateway"]) {
        if (auto val = gw_field.value<std::string_view>()) {
          gw_target = std::string(*val);
        }
      }
    }
    if (!gw_target.empty()) {
      specs.push_back(MakeNodeSpec("sim_gateway", gw_target, options));
    }
  }

  // 4. Optional Studio bridge. A host-side viewer, not a robot node, so it is
  // not a [[subsystems]] entry; the [studio] table names the binary (defaulting
  // to the framework's own bridge, so unlike the gateway there is no
  // per-robot target to require). Sim sessions only: in a deployed session
  // there is no build tree serving the webroot and no browser to open.
  if (options.start_studio_bridge && options.simulation) {
    specs.push_back(
        MakeNodeSpec("studio_bridge", robot_config.studio.bridge, options));
  }

  return specs;
}

// Effective controller endpoint for a launch: an explicit CLI flag wins,
// otherwise the deployment configuration in [robot]. A zero CLI port means
// "not given".
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

// Effective Studio HTTP endpoint for a launch: an explicit CLI flag wins,
// otherwise the [studio] deployment configuration. A zero CLI port means
// "not given".
inline uint16_t EffectiveStudioHttpPort(const config::RobotConfig& config,
                                        const LauncherOptions& options) {
  return options.studio_http_port != 0 ? options.studio_http_port
                                       : config.studio.http_port;
}

// Studio bridge supervision budget. The bridge refuses to start before the
// telemetry publisher exists, so it gets a bounded startup window of
// 1s-spaced attempts. The window is paced, not just counted: retries happen
// at most once per second so a bridge that exits instantly still waits out
// telemetry's startup rather than burning all attempts in a tight loop.
inline constexpr int kStudioBridgeMaxAttempts = 60;
inline constexpr std::chrono::seconds kStudioBridgeRetryInterval{1};
inline constexpr std::chrono::seconds kStudioBridgeStartupWindow{60};

inline std::size_t FindStudioBridgeIndex(
    const std::vector<NodeSpec>& specs) {
  for (std::size_t i = 0; i < specs.size(); ++i) {
    if (specs[i].name == "studio_bridge") return i;
  }
  return specs.size();
}

// Pure retry predicate, unit-testable without forking. True while the bridge
// still owns startup budget: attempts and window remain, the last spawn is at
// least one retry interval old (pacing), and the session itself is not
// shutting down (signal or duration deadline).
inline bool ShouldRetryStudioBridge(
    int attempts_made,
    std::chrono::steady_clock::time_point first_attempt,
    std::chrono::steady_clock::time_point last_spawn,
    std::chrono::steady_clock::time_point now, bool stop_requested,
    bool past_deadline) {
  if (stop_requested || past_deadline) return false;
  if (attempts_made >= kStudioBridgeMaxAttempts) return false;
  if (now - first_attempt >= kStudioBridgeStartupWindow) return false;
  if (now - last_spawn < kStudioBridgeRetryInterval) return false;
  return true;
}

// A bridge that is still alive after one retry interval survived the startup
// race the retries exist for; afterwards it is supervised like any other
// node and a later exit is terminal, not a startup failure.
inline bool IsStudioBridgeStable(
    std::chrono::steady_clock::time_point last_spawn,
    std::chrono::steady_clock::time_point now) {
  return now - last_spawn >= kStudioBridgeRetryInterval;
}

// The launcher works in the vocabulary of the naming protocol, not its own.
namespace naming = introspect::naming;

// What one binary answered when it was asked what it is.
struct NodeDescription {
  std::string roster_name;  // The `[subsystems.<name>]` key from the config.
  std::string target;
  std::string binary_path;
  bool answered{false};
  std::string error;  // Why not, when it did not answer.
  introspect::Description declared;
};

// Runs `binary --describe`, captures its stdout and parses the manifest out of
// it.
//
// fork/execv with a pipe rather than popen, to match the spawn path below and
// because popen goes through a shell: a binary path with a space in it, or a
// node that inherits a shell's signal disposition, is not a thing to debug at a
// competition. The timeout is the point of the whole function -- a node that
// blocks in its constructor costs the launch a second, not the session.
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
  for (auto& arg : owned) {
    c_args.push_back(arg.data());
  }
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
    // stderr is left alone on purpose: a node that throws on --describe should
    // say so where the person launching can see it.
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

  // SIGKILL rather than SIGINT: this process was asked a question, not given
  // work, so there is nothing for it to shut down cleanly.
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
// the graph they describe, and lint it.
//
// The descriptions come from the binaries rather than from the config because
// the config would be a second declaration of the topology, and a second
// declaration is a thing to forget to update -- which produces exactly the
// failure the lint exists to catch.
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

    // The same flags the spawn below passes, minus the ones about running:
    // a node whose topics depend on its configuration must describe the
    // configuration this session will actually use.
    std::vector<std::string> args{spec.binary_path, "--describe", "--config",
                                  options.config_path};
    if (options.simulation) args.push_back("--sim");

    described.answered =
        ProbeDescribe(spec.binary_path, args,
                      std::chrono::milliseconds{options.describe_timeout_ms},
                      described.declared, described.error);

    if (described.answered) {
      // Timers are left out of the graph handed to the linter. A timer is an
      // event source but not a topic: nothing can be at the other end of one,
      // and its name is a label -- "swerve" -- rather than a path, so checking
      // it against the topic grammar reports every periodic node as broken.
      // graph.json still carries them, because a viewer wants the whole shape.
      naming::NodeShape shape{
          described.declared.name, described.declared.target, {}};
      for (const auto& source : described.declared.sources) {
        if (source.kind != event::SourceKind::TIMER) {
          shape.sources.push_back(source);
        }
      }
      shapes.push_back(std::move(shape));
      if (described.declared.name != spec.name) {
        // A warning and not an error: the two names disagreeing makes the log
        // file and the registry row hard to line up, which is a confusing
        // session rather than a broken one.
        probe_findings.push_back({naming::Severity::WARNING, spec.name,
                                  "the config calls this node '" + spec.name +
                                      "' but the binary describes itself as '" +
                                      described.declared.name + "'"});
      }
    } else {
      // Also a warning. A node someone is halfway through writing has to stay
      // launchable: a framework that refuses to start what it cannot describe
      // is a framework people work around, and then nothing is described.
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

// Distinct topics in a declared graph. Timers are event sources but not
// topics -- nothing can be at the other end of one -- so they do not count.
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

// What the bridge itself described is what it watches: its --describe reads
// the same [subsystems.telemetry] topic key the telemetry node reads, so this
// follows a topic override with no second spelling of it. The fallback pins
// 2026-robot/main_processor/telemetry/node.h kStudioTopic by value; the graph
// lint pins the copy, failing the launch with "read by studio_bridge but
// nothing publishes it" if it ever drifts.
inline std::string StudioBridgeWatchTopic(
    const std::vector<NodeDescription>& described) {
  std::string topic = "/talos/telemetry";
  for (const auto& node : described) {
    if (node.answered && node.declared.name == "studio_bridge") {
      for (const auto& source : node.declared.sources) {
        if (source.kind == event::SourceKind::WATCHER && !source.name.empty()) {
          topic = source.name;
        }
      }
    }
  }
  return topic;
}

// The declared graph on disk, beside the session's manifest.
//
// uint64 values are written as decimal strings, which is the rule everywhere in
// this repo that emits JSON: a JSON number is a double, so a session id past
// 2^53 comes back a different number, and an id that changes is not an id.
inline std::string GraphToJson(
    uint64_t session_id, const std::string& config_path,
    const std::vector<NodeDescription>& described,
    const std::vector<naming::Diagnostic>& diagnostics,
    const std::string& config_sha256 = "") {
  const auto quoted = [](std::string_view value) {
    std::string out;
    introspect::AppendJsonString(out, value);
    return out;
  };

  std::ostringstream ss;
  ss << "{\n";
  ss << "  \"describe_version\": " << introspect::kDescribeVersion << ",\n";
  ss << "  \"session_id\": \"" << session_id << "\",\n";
  ss << "  \"config_path\": " << quoted(config_path) << ",\n";
  ss << "  \"config_sha256\": " << quoted(config_sha256) << ",\n";
  ss << "  \"nodes\": [\n";
  for (std::size_t i = 0; i < described.size(); ++i) {
    const auto& node = described[i];
    ss << "    {\n";
    ss << "      \"name\": "
       << quoted(node.answered ? node.declared.name : node.roster_name)
       << ",\n";
    ss << "      \"target\": "
       << quoted(node.answered ? node.declared.target : node.target) << ",\n";
    ss << "      \"described\": " << (node.answered ? "true" : "false")
       << ",\n";
    if (!node.answered) {
      ss << "      \"describe_error\": " << quoted(node.error) << ",\n";
    }
    ss << "      \"sources\": [";
    for (std::size_t j = 0; j < node.declared.sources.size(); ++j) {
      const auto& source = node.declared.sources[j];
      ss << (j ? ",\n        {" : "\n        {");
      ss << "\"kind\": " << quoted(event::to_string(source.kind));
      ss << ", \"name\": " << quoted(source.name);
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
       << quoted(diagnostic.severity == naming::Severity::ERROR ? "error"
                                                                : "warning")
       << ", \"subject\": " << quoted(diagnostic.subject)
       << ", \"message\": " << quoted(diagnostic.message) << "}"
       << (i + 1 < diagnostics.size() ? "," : "") << "\n";
  }
  ss << "  ]\n";
  ss << "}\n";
  return ss.str();
}

class Launcher {
 public:
  explicit Launcher(LauncherOptions options) : options_{std::move(options)} {
    std::string bundle_err;
    LoadBundle(options_, bundle_err);
    if (options_.session_id == 0) {
      options_.session_id = GenerateSessionId();
    }
    if (options_.output_dir.empty()) {
      options_.output_dir =
          "/tmp/talos_logs/" + std::to_string(options_.session_id);
    }
  }

  const LauncherOptions& options() const noexcept { return options_; }
  uint64_t session_id() const noexcept { return options_.session_id; }
  const std::string& output_dir() const noexcept { return options_.output_dir; }

  // Runs the declared nodes, waits for completion/signals, and produces
  // manifest. Returns 0 on success, or non-zero if any critical node failed.
  int Run() {
    std::string bundle_err;
    if (!LoadBundle(options_, bundle_err)) {
      std::cerr << "launcher: " << bundle_err << "\n";
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
      // A relative config path is resolved against the working directory, so
      // naming it is the difference between "the file is missing" and "you are
      // standing somewhere else".
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

    auto node_specs = DiscoverNodes(robot_config, options_);
    if (node_specs.empty()) {
      std::cerr << "launcher: no nodes declared in config\n";
      return 1;
    }

    // Controller addressing is config-driven with a CLI override: the same
    // resolution the bridge applies on its own, computed here so the
    // hardware node is always started with an explicit endpoint.
    const std::string effective_remote =
        EffectiveRemoteAddress(robot_config, options_);
    const uint16_t effective_remote_port =
        EffectiveRemotePort(robot_config, options_);
    const uint16_t effective_local_port =
        EffectiveLocalPort(robot_config, options_);

    // Phase one. Every binary is asked what it is before any of them is
    // started, because the launcher is the only place that knows the whole
    // roster before it exists -- and a topic whose two ends are spelled
    // differently cannot be seen from inside either node.
    std::vector<NodeDescription> described;
    const auto diagnostics = DescribeGraph(node_specs, options_, described);
    const std::size_t declared_topics = CountDeclaredTopics(described);

    const std::string graph_path = options_.output_dir + "/graph.json";
    std::ofstream graph_file(graph_path);
    if (graph_file.is_open()) {
      graph_file << GraphToJson(options_.session_id, options_.config_path,
                                described, diagnostics,
                                options_.bundle_config_sha256);
      graph_file.close();
    }
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

    SessionManifest manifest;
    manifest.session_id = options_.session_id;
    manifest.config_path = options_.config_path;
    manifest.config_sha256 = options_.bundle_config_sha256;
    manifest.output_dir = options_.output_dir;
    manifest.simulation = options_.simulation;
    manifest.start_wall_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();

    std::vector<NodeProcessInfo> procs;
    procs.reserve(node_specs.size());

    for (std::size_t i = 0; i < node_specs.size(); ++i) {
      const auto& spec = node_specs[i];
      NodeProcessInfo pinfo;
      pinfo.name = spec.name;
      pinfo.target = spec.target;
      pinfo.binary_path = spec.binary_path;
      auto contract_it = options_.bundle_contracts.find(spec.target);
      if (contract_it == options_.bundle_contracts.end()) {
        contract_it = options_.bundle_contracts.find(spec.name);
      }
      if (contract_it != options_.bundle_contracts.end()) {
        pinfo.contract = contract_it->second;
      }
      pinfo.log_path = options_.output_dir + "/" + spec.name + ".tlog";
      if (spec.name == "studio_bridge") {
        // Plaintext HTTP log, not an event recording: leave the .tlog slot
        // empty so the manifest pass does not try to parse it as one. The
        // bridge's stdout lands in studio_bridge.log (see Run's bridge state).
        pinfo.log_path = "";
      }
      pinfo.described = described[i].answered;
      procs.push_back(pinfo);
    }

    // Install stop handlers
    InstallSignals();

    // The duration deadline is established BEFORE spawning, so a bridge that
    // never comes up cannot push the session's end out: the clock that bounds
    // the session starts with the session, not after the viewer.
    const auto session_start = std::chrono::steady_clock::now();
    const auto deadline = options_.duration_s > 0
                              ? session_start +
                                    std::chrono::seconds(options_.duration_s)
                              : std::chrono::steady_clock::time_point::max();

    std::cout << "[launcher] Starting session " << options_.session_id
              << " with " << procs.size() << " nodes...\n";
    std::cout << "[launcher] Controller " << effective_remote << ":"
              << effective_remote_port << " (local port "
              << effective_local_port << ")\n";

    // Spawn each node process (the Studio bridge starts below, once the
    // declared graph it serves exists on disk).
    for (std::size_t i = 0; i < procs.size(); ++i) {
      const auto& spec = node_specs[i];
      auto& pinfo = procs[i];
      if (spec.name == "studio_bridge") continue;

      std::vector<std::string> args;
      args.push_back(spec.binary_path);
      if (spec.name == "sim_gateway") {
        // The gateway needs the robot manifest to stamp a real config_id:
        // unconfigured it stamps 0, which every mechanism's AcceptState
        // gate rejects, leaving the whole graph motionless but green.
        args.push_back("--config");
        args.push_back(options_.config_path);
      }
      if (spec.name != "sim_gateway") {
        args.push_back("--session-id");
        args.push_back(std::to_string(options_.session_id));
        args.push_back("--log");
        args.push_back(pinfo.log_path);
        args.push_back("--config");
        args.push_back(options_.config_path);
        if (options_.simulation) {
          args.push_back("--sim");
        }
        if (spec.name == "hardware_node") {
          args.push_back("--remote");
          args.push_back(effective_remote);
          args.push_back("--remote-port");
          args.push_back(std::to_string(effective_remote_port));
          args.push_back("--local-port");
          args.push_back(std::to_string(effective_local_port));
        }
      }
      if (options_.duration_s > 0) {
        args.push_back("--duration-s");
        args.push_back(std::to_string(options_.duration_s));
      }
      for (const auto& extra : spec.extra_args) {
        args.push_back(extra);
      }

      pid_t pid = ::fork();
      if (pid < 0) {
        std::cerr << "[launcher] fork() failed for node " << pinfo.name << "\n";
        pinfo.failed = true;
        continue;
      }

      if (pid == 0) {
        // In child process
        std::vector<char*> c_args;
        c_args.reserve(args.size() + 1);
        for (auto& a : args) {
          c_args.push_back(a.data());
        }
        c_args.push_back(nullptr);

        ::execv(spec.binary_path.c_str(), c_args.data());
        // If exec fails:
        std::fprintf(stderr, "[launcher] execv failed for %s (%s): %s\n",
                     pinfo.name.c_str(), spec.binary_path.c_str(),
                     std::strerror(errno));
        ::_exit(127);
      }

      pinfo.pid = pid;
      std::cout << "[launcher] Started " << pinfo.name << " (pid " << pid
                << ", target " << pinfo.target << ")\n";
    }

    // The Studio bridge serves the graph written above and refuses to start
    // before the telemetry publisher exists. Its first attempt spawns here,
    // alongside every other node, without waiting: the bounded retry across
    // telemetry's startup window happens INSIDE the supervision loop below,
    // so a failing viewer never delays child monitoring or the duration
    // deadline established above.
    struct StudioBridgeState {
      std::size_t index = std::numeric_limits<std::size_t>::max();
      std::vector<std::string> args;
      std::string binary;
      std::string log_path;
      uint16_t http_port{0};
      int attempts{0};
      std::chrono::steady_clock::time_point first_attempt{};
      std::chrono::steady_clock::time_point last_spawn{};
      bool active{false};
      bool stable{false};
    };
    StudioBridgeState bridge;
    // Nonblocking spawn: fork/exec only, no sleep, no wait. Returns the child
    // pid, or -1 on fork failure.
    const auto spawn_bridge_once = [&]() -> pid_t {
      const pid_t pid = ::fork();
      if (pid < 0) return -1;
      if (pid == 0) {
        // The bridge has no --log flag; collect its stdout/stderr here so
        // the session log keeps one file per process.
        std::freopen(bridge.log_path.c_str(), "a", stdout);
        std::freopen(bridge.log_path.c_str(), "a", stderr);
        std::vector<char*> c_args;
        c_args.reserve(bridge.args.size() + 1);
        for (auto& a : bridge.args) c_args.push_back(a.data());
        c_args.push_back(nullptr);
        ::execv(bridge.binary.c_str(), c_args.data());
        std::fprintf(stderr,
                     "[launcher] execv failed for studio_bridge (%s): %s\n",
                     bridge.binary.c_str(), std::strerror(errno));
        ::_exit(127);
      }
      return pid;
    };
    {
      const std::size_t bi = FindStudioBridgeIndex(node_specs);
      if (bi != node_specs.size()) {
        bridge.index = bi;
        auto& pinfo = procs[bi];
        const std::string& binary = node_specs[bi].binary_path;
        if (binary.empty() || ::access(binary.c_str(), X_OK) != 0) {
          std::cerr << "[launcher] studio_bridge not executable at '"
                    << (binary.empty() ? node_specs[bi].target : binary)
                    << "'; session runs without Studio\n";
          pinfo.failed = true;
        } else {
          bridge.binary = binary;
          bridge.log_path = options_.output_dir + "/studio_bridge.log";
          bridge.http_port = EffectiveStudioHttpPort(robot_config, options_);
          const std::string topic = StudioBridgeWatchTopic(described);
          bridge.args = {bridge.binary,
                         "--drop-newest-publisher",
                         topic,
                         "127.0.0.1",
                         std::to_string(robot_config.studio.udp_port),
                         std::to_string(bridge.http_port),
                         robot_config.studio.webroot,
                         "--declared",
                         graph_path};
          std::cout << "[launcher] Starting studio_bridge (target "
                    << node_specs[bi].target << ")...\n";
          const pid_t pid = spawn_bridge_once();
          if (pid < 0) {
            std::cerr << "[launcher] fork() failed for studio_bridge\n";
            pinfo.failed = true;
          } else {
            const auto now = std::chrono::steady_clock::now();
            pinfo.pid = pid;
            bridge.attempts = 1;
            bridge.first_attempt = now;
            bridge.last_spawn = now;
            bridge.active = true;
          }
        }
      }
    }

    // Wait loop: wait until all children terminate or duration/signal triggers
    // stop. Bridge retries live here: a bridge that exits inside its grace
    // second lost the telemetry race and is respawned while startup budget
    // (attempts, window) and session time (deadline, no stop) remain. A bridge
    // that survives one interval is stable and supervised like any other node;
    // exhaustion marks it failed and the session runs on without Studio.
    bool stop_sent = false;
    for (;;) {
      const auto now = std::chrono::steady_clock::now();
      const bool past_deadline = (now >= deadline);
      for (auto& pinfo : procs) {
        if (pinfo.pid <= 0) continue;
        if (!pinfo.exited) {
          int status = 0;
          pid_t res = ::waitpid(pinfo.pid, &status, WNOHANG);
          if (res == pinfo.pid) {
            pinfo.exited = true;
            pinfo.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
            const bool is_bridge =
                (bridge.active && !bridge.stable &&
                 &pinfo == &procs[bridge.index]);
            if (!is_bridge) {
              std::cout << "[launcher] Process " << pinfo.name << " (pid "
                        << pinfo.pid << ") exited with code " << pinfo.exit_code
                        << "\n";
            }
          }
        }
      }

      // Bridge supervision: stabilize survivors, respawn startup failures.
      if (bridge.active && !bridge.stable) {
        auto& bpinfo = procs[bridge.index];
        const bool alive = (bpinfo.pid > 0 && !bpinfo.exited);
        if (alive) {
          if (IsStudioBridgeStable(bridge.last_spawn, now)) {
            bridge.stable = true;
            std::cout << "[launcher] Started studio_bridge (pid " << bpinfo.pid
                      << ") on http://localhost:" << bridge.http_port << "\n";
          }
        } else if (ShouldRetryStudioBridge(bridge.attempts,
                                           bridge.first_attempt,
                                           bridge.last_spawn, now,
                                           stop_requested_ != 0,
                                           past_deadline || stop_sent)) {
          const int last_code = bpinfo.exit_code;
          const pid_t pid = spawn_bridge_once();
          if (pid < 0) {
            std::cerr << "[launcher] fork() failed for studio_bridge\n";
            bpinfo.failed = true;
            bpinfo.exit_code = -1;
            bridge.active = false;
          } else {
            if (bridge.attempts == 1 || bridge.attempts % 15 == 0) {
              std::cout << "[launcher] studio_bridge not up yet (exit "
                        << last_code
                        << "); waiting for the telemetry publisher...\n";
            }
            bpinfo.pid = pid;
            bpinfo.exited = false;
            bpinfo.exit_code = -1;
            ++bridge.attempts;
            bridge.last_spawn = now;
          }
        } else if (now - bridge.last_spawn >= kStudioBridgeRetryInterval) {
          // Grace elapsed and no retry budget (or the session is shutting
          // down): give up without blocking the session. A stop-driven
          // abandon is not a bridge failure; exhaustion is.
          bridge.active = false;
          if (stop_requested_ != 0 || stop_sent || past_deadline) {
            // Interrupted startup: leave failed/exit_code for the manifest
            // pass as-is and shut down with the session.
          } else {
            std::cerr << "[launcher] studio_bridge never came up; session runs "
                         "without Studio (see "
                      << bridge.log_path << ")\n";
            bpinfo.failed = true;
            // Discard the last startup-race code the way the blocking retry
            // did: exhaustion is reported as failed, not as that code.
            bpinfo.exit_code = -1;
          }
        }
      }

      // A bridge still in its startup race does not hold the session open:
      // the session ends when the robot nodes (or the deadline) say so, with
      // or without Studio. Only a stable bridge keeps running as a session
      // member.
      bool all_done = true;
      for (std::size_t i = 0; i < procs.size(); ++i) {
        const auto& pinfo = procs[i];
        if (pinfo.pid <= 0) continue;
        if (pinfo.exited) continue;
        if (bridge.index != std::numeric_limits<std::size_t>::max() && i == bridge.index &&
            !bridge.stable) {
          continue;
        }
        all_done = false;
        break;
      }

      if (all_done) {
        // A session that ends while the bridge is still racing never got
        // Studio: record the exhaustion the blocking retry would have, unless
        // the ending is a stop (whose path never marked the bridge failed).
        if (bridge.active && !bridge.stable && stop_requested_ == 0) {
          auto& bpinfo = procs[bridge.index];
          // Reap a just-spawned child that outlived the session so it does
          // not escape supervision.
          if (bpinfo.pid > 0 && !bpinfo.exited) {
            ::kill(bpinfo.pid, SIGINT);
            int status = 0;
            ::waitpid(bpinfo.pid, &status, WNOHANG);
            bpinfo.exited = true;
          }
          bpinfo.failed = true;
          bpinfo.exit_code = -1;
          bridge.active = false;
        }
        break;
      }

      if (!stop_sent && (stop_requested_ || past_deadline)) {
        stop_sent = true;
        std::cout << "[launcher] Stopping children...\n";
        for (const auto& pinfo : procs) {
          if (pinfo.pid > 0 && !pinfo.exited) {
            ::kill(pinfo.pid, SIGINT);
          }
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    manifest.end_wall_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();

    // Inspect logs and populate manifest
    int exit_status = 0;
    for (auto& pinfo : procs) {
      if (!pinfo.log_path.empty() && std::filesystem::exists(pinfo.log_path)) {
        try {
          event::log::LogReader reader{pinfo.log_path};
          event::log::LogReader::Record rec;
          while (reader.next(rec)) {
          }
          pinfo.record_count = reader.record_count();
          pinfo.failed = reader.truncated();
        } catch (...) {
          pinfo.failed = true;
        }
      }
      if (pinfo.exit_code != 0) {
        exit_status = pinfo.exit_code;
      }
    }
    manifest.nodes = procs;
    manifest.declared_topics = declared_topics;

    // Write manifest to disk
    const std::string manifest_path = options_.output_dir + "/manifest.json";
    std::ofstream mf(manifest_path);
    if (mf.is_open()) {
      mf << manifest.ToJson();
      mf.close();
    }

    PrintManifest(manifest);

    return exit_status;
  }

  // Bridge spawn notes (see Run): the bridge takes positional wire args, not
  // the --config/--session-id flags robot nodes take, so it gets its own
  // spawn path; and it refuses to start before the telemetry publisher
  // exists, which needs its own startup window, so the first attempt usually
  // loses that race and a bounded retry inside the supervision loop follows.
  // A viewer that never comes up is a degraded session, not a broken robot:
  // exhaustion marks the node failed and the session runs on without Studio.
  static void RequestStop() { stop_requested_ = true; }

 private:
  void InstallSignals() {
    stop_requested_ = false;
    std::signal(SIGINT, [](int) { stop_requested_ = true; });
    std::signal(SIGTERM, [](int) { stop_requested_ = true; });
  }

  // Phase one's report. Printed in full, warnings included: a diagnostic
  // nobody reads is the same as no diagnostic, and the ones about a node that
  // could not be described are how a stale binary is noticed.
  void PrintDeclaredGraph(const std::vector<NodeDescription>& described,
                          const std::vector<naming::Diagnostic>& diagnostics,
                          std::size_t declared_topics,
                          const std::string& graph_path) {
    std::size_t answered = 0;
    for (const auto& node : described) answered += node.answered ? 1 : 0;

    std::cout << "\n================ Declared Graph ==================\n";
    std::cout << "Nodes described: " << answered << "/" << described.size()
              << ", topics: " << declared_topics << "\n";
    for (const auto& node : described) {
      std::cout << "  " << (node.answered ? "\u2022" : "?") << " "
                << node.roster_name;
      if (node.answered) {
        std::cout << " (" << node.declared.sources.size() << " sources)";
      } else {
        std::cout << " (not described)";
      }
      std::cout << "\n";
    }
    for (const auto& diagnostic : diagnostics) {
      auto& out = diagnostic.severity == naming::Severity::ERROR ? std::cerr
                                                                 : std::cout;
      out << "  " << diagnostic.ToString() << "\n";
    }
    std::cout << "Graph written to: " << graph_path << "\n";
    std::cout << "==================================================\n";
  }

  void PrintManifest(const SessionManifest& manifest) {
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
      std::cout << "  • " << n.name << " (pid=" << n.pid
                << ", exit=" << n.exit_code << ", records=" << n.record_count
                << ", failed=" << (n.failed ? "yes" : "no")
                << ", log=" << n.log_path << ")\n";
    }
    std::cout << "Manifest written to: " << manifest.output_dir
              << "/manifest.json\n";
    std::cout << "==================================================\n";
  }

  LauncherOptions options_;
  static inline volatile std::sig_atomic_t stop_requested_{false};
};

struct MergedRecord {
  std::string process_name;
  std::string source_name;
  event::log::RecordHeader header;
  std::vector<std::byte> payload;
};

inline std::string ResolveSourceName(const event::Manifest& manifest,
                                     std::uint16_t source_id) {
  for (const auto& reg : manifest) {
    if (reg.id == source_id) {
      return reg.name;
    }
  }
  return "?";
}

inline std::vector<std::string> FindLogFiles(const std::string& path_or_dir) {
  std::vector<std::string> result;
  std::error_code ec;
  if (std::filesystem::is_directory(path_or_dir, ec)) {
    // Check if manifest.json exists
    std::string manifest_file = path_or_dir + "/manifest.json";
    if (std::filesystem::exists(manifest_file)) {
      std::ifstream mf(manifest_file);
      std::string line;
      while (std::getline(mf, line)) {
        auto pos = line.find("\"log_path\": \"");
        if (pos != std::string::npos) {
          auto start = pos + 13;
          auto end = line.find("\"", start);
          if (end != std::string::npos) {
            std::string log_p = line.substr(start, end - start);
            if (std::filesystem::exists(log_p)) {
              result.push_back(log_p);
            }
          }
        }
      }
    }
    // If no manifest or no logs found in manifest, scan directory for *.tlog
    if (result.empty()) {
      for (const auto& entry :
           std::filesystem::directory_iterator(path_or_dir, ec)) {
        if (entry.path().extension() == ".tlog") {
          result.push_back(entry.path().string());
        }
      }
      std::sort(result.begin(), result.end());
    }
  } else {
    result.push_back(path_or_dir);
  }
  return result;
}

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
    event::log::LogReader::Record current_record;
    bool has_record{false};
  };

  std::vector<ReaderState> states;
  states.reserve(log_paths.size());

  uint64_t expected_session = 0;
  bool session_set = false;

  for (const auto& path : log_paths) {
    ReaderState s;
    try {
      s.reader = std::make_unique<event::log::LogReader>(path);
    } catch (const std::exception& e) {
      error = "failed to open " + path + ": " + e.what();
      return false;
    }

    if (!session_set) {
      expected_session = s.reader->session_id();
      session_set = true;
    } else if (s.reader->session_id() != expected_session) {
      error = "session_id mismatch: file '" + path + "' has session_id " +
              std::to_string(s.reader->session_id()) + ", expected " +
              std::to_string(expected_session);
      return false;
    }

    s.has_record = s.reader->next(s.current_record);
    states.push_back(std::move(s));
  }

  // Multi-way merge by event_time_ns (with now_ns and dispatch_index
  // tie-breaker)
  for (;;) {
    int best_index = -1;
    for (std::size_t i = 0; i < states.size(); ++i) {
      if (!states[i].has_record) continue;
      if (best_index == -1) {
        best_index = static_cast<int>(i);
      } else {
        const auto& best_hdr = states[best_index].current_record.header;
        const auto& cur_hdr = states[i].current_record.header;
        if (cur_hdr.event_time_ns < best_hdr.event_time_ns ||
            (cur_hdr.event_time_ns == best_hdr.event_time_ns &&
             cur_hdr.now_ns < best_hdr.now_ns) ||
            (cur_hdr.event_time_ns == best_hdr.event_time_ns &&
             cur_hdr.now_ns == best_hdr.now_ns &&
             cur_hdr.dispatch_index < best_hdr.dispatch_index)) {
          best_index = static_cast<int>(i);
        }
      }
    }

    if (best_index == -1) {
      break;  // All streams exhausted
    }

    auto& best = states[best_index];
    MergedRecord mr;
    mr.process_name = best.reader->process_name();
    mr.source_name = ResolveSourceName(best.reader->manifest(),
                                       best.current_record.header.source_id);
    mr.header = best.current_record.header;
    mr.payload.assign(best.current_record.payload.begin(),
                      best.current_record.payload.end());
    out_records.push_back(std::move(mr));

    best.has_record = best.reader->next(best.current_record);
  }

  return true;
}

}  // namespace talos::launcher
