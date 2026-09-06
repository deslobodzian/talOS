#pragma once

#include <poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
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

  // Map of target/name -> binary override, useful for testing
  std::map<std::string, std::string> binary_overrides;
};

inline std::string ResolveBinary(
    const std::string& target_or_path,
    const std::map<std::string, std::string>& overrides = {}) {
  auto it = overrides.find(target_or_path);
  if (it != overrides.end()) {
    return it->second;
  }

  // If already a regular file path (or exists as executable), check it
  if (!target_or_path.starts_with("//")) {
    if (::access(target_or_path.c_str(), X_OK) == 0) {
      return target_or_path;
    }
  }

  // Convert "//2026-robot/main_processor/drivetrain:node" ->
  // "2026-robot/main_processor/drivetrain/node"
  std::string subpath = target_or_path;
  if (subpath.starts_with("//")) {
    subpath = subpath.substr(2);
  }
  for (char& c : subpath) {
    if (c == ':') c = '/';
  }

  // Check possible locations
  std::vector<std::string> candidates;
  if (const char* test_srcdir = std::getenv("TEST_SRCDIR")) {
    candidates.push_back(std::string(test_srcdir) + "/_main/" + subpath);
    candidates.push_back(std::string(test_srcdir) + "/talos/" + subpath);
    candidates.push_back(std::string(test_srcdir) + "/" + subpath);
  }
  if (const char* runfiles = std::getenv("RUNFILES_DIR")) {
    candidates.push_back(std::string(runfiles) + "/_main/" + subpath);
    candidates.push_back(std::string(runfiles) + "/talos/" + subpath);
    candidates.push_back(std::string(runfiles) + "/" + subpath);
  }
  candidates.push_back("bazel-bin/" + subpath);
  candidates.push_back(subpath);
  candidates.push_back("./" + subpath);
  candidates.push_back("../" + subpath);

  for (const auto& path : candidates) {
    if (::access(path.c_str(), X_OK) == 0) {
      return path;
    }
  }

  // If subpath ends with "hardware_node", also check "node"
  if (subpath.ends_with("hardware_node")) {
    std::string alt_subpath = subpath.substr(0, subpath.size() - 13) + "node";
    std::vector<std::string> alt_candidates = {"bazel-bin/" + alt_subpath,
                                               alt_subpath, "./" + alt_subpath,
                                               "../" + alt_subpath};
    for (const auto& path : alt_candidates) {
      if (::access(path.c_str(), X_OK) == 0) {
        return path;
      }
    }
  }

  // Fallback to the subpath itself
  return candidates.front();
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
  NodeSpec hw_spec;
  hw_spec.name = "hardware_node";
  hw_spec.target = hw_target;
  hw_spec.binary_path = ResolveBinary(hw_target, options.binary_overrides);
  specs.push_back(hw_spec);

  // 2. Subsystems
  if (auto* subs = robot_config.toml_data["subsystems"].as_table()) {
    for (auto&& [sub_key, sub_val] : *subs) {
      std::string sub_name = std::string(sub_key.str());
      if (auto* sub_tbl = sub_val.as_table()) {
        if (auto node_field = (*sub_tbl)["node"]) {
          if (auto node_val = node_field.value<std::string_view>()) {
            std::string target = std::string(*node_val);
            NodeSpec spec;
            spec.name = sub_name;
            spec.target = target;
            spec.binary_path = ResolveBinary(target, options.binary_overrides);
            specs.push_back(spec);
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
      NodeSpec gw_spec;
      gw_spec.name = "sim_gateway";
      gw_spec.target = gw_target;
      gw_spec.binary_path = ResolveBinary(gw_target, options.binary_overrides);
      specs.push_back(gw_spec);
    }
  }

  return specs;
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

// The declared graph on disk, beside the session's manifest.
//
// uint64 values are written as decimal strings, which is the rule everywhere in
// this repo that emits JSON: a JSON number is a double, so a session id past
// 2^53 comes back a different number, and an id that changes is not an id.
inline std::string GraphToJson(
    uint64_t session_id, const std::string& config_path,
    const std::vector<NodeDescription>& described,
    const std::vector<naming::Diagnostic>& diagnostics) {
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
    if (options_.config_path.empty()) {
      std::cerr << "launcher: --config PATH is required\n";
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
                                described, diagnostics);
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
      pinfo.log_path = options_.output_dir + "/" + spec.name + ".tlog";
      pinfo.described = described[i].answered;
      procs.push_back(pinfo);
    }

    // Install stop handlers
    InstallSignals();

    std::cout << "[launcher] Starting session " << options_.session_id
              << " with " << procs.size() << " nodes...\n";

    // Spawn each node process
    for (std::size_t i = 0; i < procs.size(); ++i) {
      const auto& spec = node_specs[i];
      auto& pinfo = procs[i];

      std::vector<std::string> args;
      args.push_back(spec.binary_path);
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

    // Wait loop: wait until all children terminate or duration/signal triggers
    // stop
    const auto deadline = options_.duration_s > 0
                              ? std::chrono::steady_clock::now() +
                                    std::chrono::seconds(options_.duration_s)
                              : std::chrono::steady_clock::time_point::max();

    bool stop_sent = false;
    for (;;) {
      bool all_done = true;
      for (auto& pinfo : procs) {
        if (pinfo.pid <= 0) continue;
        if (!pinfo.exited) {
          int status = 0;
          pid_t res = ::waitpid(pinfo.pid, &status, WNOHANG);
          if (res == pinfo.pid) {
            pinfo.exited = true;
            pinfo.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
            std::cout << "[launcher] Process " << pinfo.name << " (pid "
                      << pinfo.pid << ") exited with code " << pinfo.exit_code
                      << "\n";
          } else {
            all_done = false;
          }
        }
      }

      if (all_done) {
        break;
      }

      if (!stop_sent &&
          (stop_requested_ || std::chrono::steady_clock::now() >= deadline)) {
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
      if (std::filesystem::exists(pinfo.log_path)) {
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
