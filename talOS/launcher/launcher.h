#pragma once

#include <chrono>
#include <cstdint>
#include <cstdlib>
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

#include <csignal>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "talOS/configuration/config_parser.h"
#include "talOS/events/log/log_reader.h"

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
  std::string target;       // e.g. "//2026-robot/main_processor/drivetrain:node"
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
};

struct SessionManifest {
  uint64_t session_id{0};
  std::string config_path;
  std::string output_dir;
  bool simulation{false};
  int64_t start_wall_ns{0};
  int64_t end_wall_ns{0};
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
      ss << "      \"recorder_failed\": " << (n.failed ? "true" : "false") << ",\n";
      ss << "      \"record_count\": " << n.record_count << "\n";
      ss << "    }" << (i + 1 < nodes.size() ? "," : "") << "\n";
    }
    ss << "  ]\n";
    ss << "}\n";
    return ss.str();
  }
};

struct LauncherOptions {
  std::string config_path{"2026-robot/main_processor/configuration/robot.toml"};
  std::string output_dir;
  uint64_t session_id{0};
  bool simulation{false};
  int duration_s{0};
  bool start_sim_gateway{false};

  // Map of target/name -> binary override, useful for testing
  std::map<std::string, std::string> binary_overrides;
};

inline std::string ResolveBinary(const std::string& target_or_path,
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

  // Convert "//2026-robot/main_processor/drivetrain:node" -> "2026-robot/main_processor/drivetrain/node"
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
    std::string alt_subpath =
        subpath.substr(0, subpath.size() - 13) + "node";
    std::vector<std::string> alt_candidates = {
        "bazel-bin/" + alt_subpath,
        alt_subpath,
        "./" + alt_subpath,
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

inline std::vector<NodeSpec> DiscoverNodes(const config::RobotConfig& robot_config,
                                          const LauncherOptions& options) {
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
      gw_spec.binary_path =
          ResolveBinary(gw_target, options.binary_overrides);
      specs.push_back(gw_spec);
    }
  }

  return specs;
}

class Launcher {
 public:
  explicit Launcher(LauncherOptions options)
      : options_{std::move(options)} {
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

  // Runs the declared nodes, waits for completion/signals, and produces manifest.
  // Returns 0 on success, or non-zero if any critical node failed.
  int Run() {
    std::filesystem::create_directories(options_.output_dir);

    config::RobotConfig robot_config;
    try {
      robot_config = config::ParseRobotConfig(options_.config_path);
    } catch (const std::exception& e) {
      std::cerr << "launcher: failed to load config from "
                << options_.config_path << ": " << e.what() << "\n";
      return 1;
    }

    auto node_specs = DiscoverNodes(robot_config, options_);
    if (node_specs.empty()) {
      std::cerr << "launcher: no nodes declared in config\n";
      return 1;
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

    for (const auto& spec : node_specs) {
      NodeProcessInfo pinfo;
      pinfo.name = spec.name;
      pinfo.target = spec.target;
      pinfo.binary_path = spec.binary_path;
      pinfo.log_path = options_.output_dir + "/" + spec.name + ".tlog";
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
        std::cerr << "[launcher] fork() failed for node " << pinfo.name
                  << "\n";
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

    // Wait loop: wait until all children terminate or duration/signal triggers stop
    const auto deadline =
        options_.duration_s > 0
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

      if (!stop_sent && (stop_requested_ || std::chrono::steady_clock::now() >= deadline)) {
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
          while (reader.next(rec)) {}
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

  void PrintManifest(const SessionManifest& manifest) {
    std::cout << "\n================ Session Manifest ================\n";
    std::cout << "Session ID: " << manifest.session_id << "\n";
    std::cout << "Output Dir: " << manifest.output_dir << "\n";
    std::cout << "Nodes (" << manifest.nodes.size() << "):\n";
    for (const auto& n : manifest.nodes) {
      std::cout << "  • " << n.name << " (pid=" << n.pid
                << ", exit=" << n.exit_code
                << ", records=" << n.record_count
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

  // Multi-way merge by event_time_ns (with now_ns and dispatch_index tie-breaker)
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
