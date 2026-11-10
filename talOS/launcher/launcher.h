#pragma once

#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#ifdef __linux__
#include <sys/prctl.h>
#include <sys/syscall.h>
#endif

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
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
#include "talOS/launcher/session_lock.h"
#include "talOS/process/durable_sync.h"
#include "talOS/rtms/rtms.h"
#include "talOS/schema/identity.h"
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

// Syncs an open file or directory to disk, as process::DurableSync does
// (production; the seam is LauncherOptions::sync_file). 0, or -1 with errno
// set.
using SyncFunction = std::function<int(int fd)>;

// Replaces `path` with `contents` so that a reader sees the old file or the
// new one, never part of one: written to `<path>.tmp`, flushed, and renamed
// over `path`. Given a `sync`, it also syncs the temporary file before the
// rename and the directory after it, so that a power loss leaves the new file
// too; with an empty one, the new file reaches the disk whenever the OS writes
// it back. A failure before the rename leaves `path` as it was and removes the
// temporary file so it holds no disk; `error` names the failed step and its
// errno.
inline bool WriteFileAtomically(const std::string& path,
                                std::string_view contents,
                                const SyncFunction& sync, std::string& error) {
  const std::string temporary = path + ".tmp";
  const auto fail = [&](std::string_view step, int code) {
    std::remove(temporary.c_str());
    error = std::string{step} + ": " + std::strerror(code);
    return false;
  };
  const int temporary_fd =
      ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (temporary_fd < 0) return fail("create", errno);
  FILE* file = ::fdopen(temporary_fd, "wb");
  if (file == nullptr) {
    const int fdopen_error = errno;
    ::close(temporary_fd);
    return fail("create", fdopen_error);
  }
  int code = 0;
  if (std::fwrite(contents.data(), 1, contents.size(), file) !=
          contents.size() ||
      std::fflush(file) != 0 || (sync && sync(::fileno(file)) != 0))
    code = errno;
  if (std::fclose(file) != 0 && code == 0) code = errno;
  if (code != 0) return fail("write", code);
  if (std::rename(temporary.c_str(), path.c_str()) != 0)
    return fail("rename", errno);
  if (!sync) return true;
  const std::filesystem::path parent =
      std::filesystem::path{path}.parent_path();
  const int directory = ::open(parent.empty() ? "." : parent.c_str(),
                               O_RDONLY | O_CLOEXEC | O_DIRECTORY);
  if (directory < 0) return fail("open directory", errno);
  const int result = sync(directory);
  code = errno;
  ::close(directory);
  if (result != 0) return fail("sync directory", code);
  return true;
}

// Keeps one session file (manifest.json) current and durable without making
// its caller wait for the disk. While started, Post() hands over the newest
// contents and returns; a worker thread writes each with WriteFileAtomically
// and `sync`, so every version it writes survives a power loss, and skips a
// version a newer one replaced while it was busy. Writes and syncs on a disk
// under load take hundreds of milliseconds each, which the supervision loop,
// restarting crashed nodes and feeding the service watchdog, cannot spend.
//
// Write() writes on the calling thread instead, for the versions before the
// worker starts and after it stops. A failed write is reported once on
// stderr, and so is the recovery; the last complete file stays in place and
// the next version retries.
class ManifestWriter {
 public:
  ManifestWriter(std::string path, SyncFunction sync)
      : path_{std::move(path)}, sync_{std::move(sync)} {}
  ~ManifestWriter() { Stop(); }
  ManifestWriter(const ManifestWriter&) = delete;
  ManifestWriter& operator=(const ManifestWriter&) = delete;

  bool running() const { return worker_.joinable(); }

  // Only while the worker is not running.
  void Write(const std::string& contents, bool durable) {
    Persist(contents, durable ? sync_ : SyncFunction{});
  }

  // The worker starts with every signal blocked, so the launcher's stop
  // signals are always handled by the supervision thread.
  void Start() {
    if (running()) return;
    stopping_ = false;
    sigset_t all;
    sigset_t previous;
    sigfillset(&all);
    ::pthread_sigmask(SIG_SETMASK, &all, &previous);
    worker_ = std::thread{[this] { Run(); }};
    ::pthread_sigmask(SIG_SETMASK, &previous, nullptr);
  }

  // Never waits for the disk: the worker holds the mutex only to take the
  // pending version, never across a write.
  void Post(std::string contents) {
    {
      const std::lock_guard lock{mutex_};
      pending_ = std::move(contents);
    }
    wake_.notify_one();
  }

  // Writes the pending version, if any, then stops the worker. Waits for the
  // disk.
  void Stop() {
    if (!running()) return;
    {
      const std::lock_guard lock{mutex_};
      stopping_ = true;
    }
    wake_.notify_one();
    worker_.join();
  }

 private:
  void Run() {
    std::unique_lock lock{mutex_};
    for (;;) {
      wake_.wait(lock, [this] { return pending_ || stopping_; });
      if (!pending_) return;
      const std::string contents = std::move(*pending_);
      pending_.reset();
      lock.unlock();
      Persist(contents, sync_);
      lock.lock();
    }
  }

  void Persist(const std::string& contents, const SyncFunction& sync) {
    std::string error;
    if (!WriteFileAtomically(path_, contents, sync, error)) {
      if (!failing_) {
        std::cerr << "[launcher] cannot persist " << path_ << " (" << error
                  << "); supervision continues and retries at the next "
                     "change\n";
      }
      failing_ = true;
      return;
    }
    if (failing_) {
      std::cerr << "[launcher] " << path_ << " persisted again\n";
      failing_ = false;
    }
  }

  const std::string path_;
  const SyncFunction sync_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::optional<std::string> pending_;  // Under mutex_.
  bool stopping_{false};                // Under mutex_.
  bool failing_{false};  // The worker's, or the caller's while it is stopped.
  std::thread worker_;
};

// A member's expected event-log bytes per second, from what it declared
// (--describe): one record per source per `period_us` tick, each a record
// header plus the source's message. Byte topics (Node::publish_bytes) are
// left out: the loop never logs them. An estimate to weight log shares by,
// not a bound: callback topics can record more often, idle ones less.
inline double ExpectedLogRate(const introspect::Description& declared,
                              int64_t period_us) {
  double per_tick = sizeof(event::log::RecordHeader);
  for (const auto& source : declared.sources) {
    if (source.schema_hash == schema::BytesHash()) continue;
    per_tick += sizeof(event::log::RecordHeader) + source.message_bytes;
  }
  return per_tick * 1e6 / static_cast<double>(std::max<int64_t>(period_us, 1));
}

// Splits a session's log budget among the members that write an event log
// (`logs[i]`): a quarter of it in equal parts, so no member starves, and the
// rest by expected rate (`rates[i]`; negative when unknown, as for a member
// that did not answer --describe, which is weighted as the mean of the
// known). Every logging member gets at least 1 byte; others get 0.
inline std::vector<uint64_t> SplitLogBudget(uint64_t budget,
                                            const std::vector<bool>& logs,
                                            const std::vector<double>& rates) {
  std::vector<uint64_t> shares(logs.size(), 0);
  std::size_t loggers = 0;
  std::size_t known = 0;
  double known_sum = 0;
  for (std::size_t i = 0; i < logs.size(); ++i) {
    if (!logs[i]) continue;
    ++loggers;
    if (rates[i] >= 0) {
      ++known;
      known_sum += rates[i];
    }
  }
  if (loggers == 0) return shares;
  const double mean = known > 0 ? known_sum / static_cast<double>(known) : 1;
  double total = 0;
  std::vector<double> weights(logs.size(), 0);
  for (std::size_t i = 0; i < logs.size(); ++i) {
    if (!logs[i]) continue;
    weights[i] = rates[i] >= 0 ? rates[i] : mean;
    total += weights[i];
  }
  const double equal =
      static_cast<double>(budget) / 4.0 / static_cast<double>(loggers);
  const double weighted =
      static_cast<double>(budget) - equal * static_cast<double>(loggers);
  for (std::size_t i = 0; i < logs.size(); ++i) {
    if (!logs[i]) continue;
    const double share =
        equal + (total > 0 ? weighted * weights[i] / total
                           : weighted / static_cast<double>(loggers));
    shares[i] = std::max<uint64_t>(1, static_cast<uint64_t>(share));
  }
  return shares;
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
  int exit_code{-1};   // -1 while it runs, and after a signal ended it.
  int exit_signal{0};  // The signal that ended the last run, else 0.
  bool exited{false};
  bool quarantined{false};
  int64_t retry_at_monotonic_ns{0};  // Zero when no restart is pending.
  // `recorder_failed`: a run could not be spawned (or the Studio bridge never
  // came up), or a run's event log failed -- seen live in the node's registry
  // row (introspect::kFlagRecorderFailed), or unreadable or truncated at
  // shutdown.
  bool failed{false};
  // A run's event log reached its share of the session log budget and lost
  // records: its oldest segments were deleted to keep the newest, or, with a
  // share too small to rotate, it stopped early (`<log>.capped`, or live,
  // introspect::kFlagLogCapped). The recording is incomplete, not corrupt.
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
      ss << "      \"exit_signal\": " << n.exit_signal << ",\n";
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
  // Internal supervision seam: the next N spawns of the named roster member
  // fail as fork() does under EAGAIN. Production leaves it empty.
  std::map<std::string, int> spawn_failures;
  // Internal supervision seam: syncs an open session file or directory to
  // disk; process::DurableSync in production. The supervision loop never
  // calls it: the manifest writer's thread syncs the versions the loop
  // hands it (PersistManifest).
  SyncFunction sync_file{process::DurableSync};

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

// Empty when `registering` more nodes can each claim a registry row now, else
// why not. Rows held by live processes outside this session (a viewer's test
// node, another namespace user's tools) are not swept, and a node that cannot
// claim a row runs unregistered until supervision kills it as missing, so a
// roster that does not fit is refused before anything starts.
inline std::string RegistryCapacityError(
    const introspect::RegistryReader& reader, std::size_t registering) {
  const auto held = reader.held_rows();
  if (held.size() + registering <= introspect::kMaxNodes) return {};
  std::string holders;
  for (const auto& row : held) {
    if (!holders.empty()) holders += ", ";
    holders += (row.name.empty() ? std::string{"?"} : row.name) + " (pid " +
               std::to_string(row.pid) + ")";
  }
  return "the node registry has " + std::to_string(held.size()) + " of " +
         std::to_string(introspect::kMaxNodes) +
         " rows held by live processes (" + holders + "), and this roster's " +
         std::to_string(registering) +
         " nodes need a row each; a node without one would be killed as "
         "unregistered. Stop those processes first";
}

// The launcher alone holds each lifetime pipe's writer. A tiny guardian in the
// child's group kills the group on EOF, including when the launcher is
// SIGKILLed. Writers are close-on-exec and a guardian keeps no descriptor but
// its own pipe's reader, so no node or guardian owns a writer.
inline std::map<pid_t, int>& ProcessLifetimes() {
  static std::map<pid_t, int> lifetimes;
  return lifetimes;
}

// Held by every thread that spawns (DescribeGraph probes in parallel) from
// the creation of a spawn's pipes to the parent's close of the child's ends,
// and around ProcessLifetimes. A child another thread forks inside that window
// would otherwise inherit this spawn's pipe ends -- close-on-exec is set after
// pipe() on macOS, which has no pipe2 -- and hold a writer open: a probe would
// see no EOF, a drained output pipe no end, until that other child exited.
inline std::mutex& SpawnMutex() {
  static std::mutex mutex;
  return mutex;
}

inline void SignalProcessGroup(pid_t pid, int signal) {
  if (pid > 0) ::kill(-pid, signal);
}

// Call after reaping the leader: even a clean exit must not leave helpers.
inline void FinishProcessGroup(pid_t pid) {
  SignalProcessGroup(pid, SIGKILL);
  const std::lock_guard lock{SpawnMutex()};
  auto& lifetimes = ProcessLifetimes();
  const auto found = lifetimes.find(pid);
  if (found != lifetimes.end()) {
    ::close(found->second);
    lifetimes.erase(found);
  }
}

// Closes every descriptor but `keep`. System calls only, so it is safe in a
// forked child of a multithreaded process.
inline void CloseDescriptorsExcept(int keep) {
#if defined(__linux__) && defined(SYS_close_range)
  // Linux 5.9+: two calls, however high RLIMIT_NOFILE is.
  const auto kept = static_cast<unsigned>(keep);
  if ((keep == 0 || ::syscall(SYS_close_range, 0u, kept - 1, 0u) == 0) &&
      ::syscall(SYS_close_range, kept + 1, ~0u, 0u) == 0)
    return;
#endif
  const int limit = ::getdtablesize();
  for (int fd = 0; fd < limit; ++fd) {
    if (fd != keep) ::close(fd);
  }
}

// A pipe whose ends both close on exec. False with errno set.
inline bool CloseOnExecPipe(int (&fds)[2]) {
  if (::pipe(fds) != 0) return false;
  if (::fcntl(fds[0], F_SETFD, FD_CLOEXEC) == 0 &&
      ::fcntl(fds[1], F_SETFD, FD_CLOEXEC) == 0)
    return true;
  const int error = errno;
  ::close(fds[0]);
  ::close(fds[1]);
  errno = error;
  return false;
}

// The steps by which a forked child becomes its program; each can fail before
// the program runs.
enum class ChildStep : int {
  kProcessGroup,  // setpgid
  kDeathSignal,   // prctl(PR_SET_PDEATHSIG), Linux only
  kGuardian,      // the fork of the group's guardian
  kOutput,        // the stdout (and stderr) redirection
  kExec,          // execv
};

// What a child that cannot become its program writes to its error pipe: one
// write of a few bytes, so the launcher reads all of it or none.
struct ChildFailure {
  ChildStep step;
  int code;  // errno
};

// In a forked child, before its exec: reports `step` and errno to the
// launcher, then exits 127, a shell's status for a command it cannot run.
// write and _exit are async-signal-safe, as every call between a fork and an
// exec must be: the fork copied whatever locks other threads held.
[[noreturn]] inline void ReportChildFailure(int errors, ChildStep step) {
  const ChildFailure failure{step, errno};
  [[maybe_unused]] const ssize_t written =
      ::write(errors, &failure, sizeof failure);
  ::_exit(127);
}

// How SpawnProcess names a child's failed step.
inline std::string ChildStepName(ChildStep step, const std::string& program) {
  switch (step) {
    case ChildStep::kProcessGroup:
      return "setpgid";
    case ChildStep::kDeathSignal:
      return "prctl(PR_SET_PDEATHSIG)";
    case ChildStep::kGuardian:
      return "fork (guardian)";
    case ChildStep::kOutput:
      return "redirect output";
    case ChildStep::kExec:
      return "execv " + program;
  }
  return "child setup";
}

// Where a spawned child's stdout and stderr go.
enum class SpawnOutput {
  kInherit,     // The launcher's own.
  kStdoutPipe,  // stdout into a new pipe (a --describe answer); stderr is the
                // launcher's.
  kPipe,        // Both into one new pipe, for a ChildOutput to drain.
};

// The pipe capacity requested for a drained child's output where the OS
// allows it (Linux F_SETPIPE_SZ; a macOS pipe grows to 64 KiB by itself):
// what the child can write while the drain thread is not scheduled.
inline constexpr int kChildOutputPipeBytes = 256 * 1024;

// fork + execv of args[0], in its own process group, its output as `output`
// says; for a pipe, `*reader` receives the read end (close-on-exec). Returns
// the child's pid once it has exec'd, which the close-on-exec error pipe it
// shares with the launcher shows by closing. A child that fails before or at
// its exec reports the step and errno there instead, exits 127 and is reaped;
// then, as when a pipe or the fork fails here, this returns -1 with `error`
// naming the step and its errno ("execv <path>: Permission denied"), and no
// reader. Safe to call from several threads at once (SpawnMutex).
inline pid_t SpawnProcess(std::vector<std::string> args, std::string& error,
                          SpawnOutput output = SpawnOutput::kInherit,
                          int* reader = nullptr) {
  const auto fail = [&error](std::string_view step, int code) {
    error = std::string{step} + ": " + std::strerror(code);
    return pid_t{-1};
  };
  if (output != SpawnOutput::kInherit && reader == nullptr)
    return fail("output pipe", EINVAL);
  std::vector<char*> c_args;
  c_args.reserve(args.size() + 1);
  for (auto& arg : args) c_args.push_back(arg.data());
  c_args.push_back(nullptr);
  std::cout.flush();
  std::fflush(nullptr);
  const pid_t parent = ::getpid();
  int lifetime[2];
  int errors[2];
  int child_output[2]{-1, -1};
  std::unique_lock spawning{SpawnMutex()};
  if (!CloseOnExecPipe(lifetime)) return fail("pipe", errno);
  if (!CloseOnExecPipe(errors)) {
    const int code = errno;
    ::close(lifetime[0]);
    ::close(lifetime[1]);
    return fail("pipe", code);
  }
  if (output != SpawnOutput::kInherit && !CloseOnExecPipe(child_output)) {
    const int code = errno;
    for (const int fd : {lifetime[0], lifetime[1], errors[0], errors[1]})
      ::close(fd);
    return fail("pipe", code);
  }
#if defined(__linux__) && defined(F_SETPIPE_SZ)
  // Best effort: the default 64 KiB still never blocks a child that writes
  // less than that between two of the drain thread's reads.
  if (output == SpawnOutput::kPipe)
    ::fcntl(child_output[1], F_SETPIPE_SZ, kChildOutputPipeBytes);
#endif
  const pid_t pid = ::fork();
  if (pid < 0) {
    const int code = errno;
    for (const int fd : {lifetime[0], lifetime[1], errors[0], errors[1],
                         child_output[0], child_output[1]})
      if (fd >= 0) ::close(fd);
    return fail("fork", code);
  }
  if (pid > 0) {
    ::close(lifetime[0]);
    ::close(errors[1]);
    if (child_output[1] >= 0) ::close(child_output[1]);
    // Both sides establish the group, closing the spawn/signal race.
    ::setpgid(pid, pid);
    ProcessLifetimes().emplace(pid, lifetime[1]);
    spawning.unlock();
    // EOF once the exec has closed the child's writer and the guardian its
    // copy, or once a child died without a report; a report otherwise.
    ChildFailure failure{};
    ssize_t got = 0;
    while ((got = ::read(errors[0], &failure, sizeof failure)) < 0 &&
           errno == EINTR) {
    }
    ::close(errors[0]);
    if (got != static_cast<ssize_t>(sizeof failure)) {
      if (reader != nullptr) *reader = child_output[0];
      return pid;
    }
    if (child_output[0] >= 0) ::close(child_output[0]);
    while (::waitpid(pid, nullptr, 0) < 0 && errno == EINTR) {
    }
    FinishProcessGroup(pid);
    return fail(ChildStepName(failure.step, args.front()), failure.code);
  }
  // The child: only async-signal-safe calls from here to the exec.
  if (::setpgid(0, 0) != 0)
    ReportChildFailure(errors[1], ChildStep::kProcessGroup);
#ifdef __linux__
  if (::prctl(PR_SET_PDEATHSIG, SIGKILL) != 0)
    ReportChildFailure(errors[1], ChildStep::kDeathSignal);
#endif
  // The launcher died first, so nothing reads a report.
  if (::getppid() != parent) ::_exit(127);
  std::signal(SIGINT, SIG_DFL);
  std::signal(SIGTERM, SIG_DFL);
  const pid_t group = ::getpid();
  const pid_t guardian = ::fork();
  if (guardian < 0) ReportChildFailure(errors[1], ChildStep::kGuardian);
  if (guardian == 0) {
    // The launcher waits for every copy of the error pipe's writer to close.
    ::close(errors[1]);
    // Graceful group signals belong to the node; the guardian waits for EOF.
    std::signal(SIGINT, SIG_IGN);
    std::signal(SIGTERM, SIG_IGN);
    // Unlike the node it never execs, so any other descriptor of the
    // launcher's process, close-on-exec or not, would stay open for as long
    // as the node runs: another thread's registry lock would stay locked.
    CloseDescriptorsExcept(lifetime[0]);
    char unused;
    while (::read(lifetime[0], &unused, 1) < 0 && errno == EINTR) {
    }
    SignalProcessGroup(group, SIGKILL);
    ::_exit(0);
  }
  ::close(lifetime[0]);
  // dup2 clears close-on-exec on the copies; the pipe's own descriptors close
  // at the exec. The writer stays blocking: see ChildOutput for why.
  if (output != SpawnOutput::kInherit) {
    if (::dup2(child_output[1], STDOUT_FILENO) < 0 ||
        (output == SpawnOutput::kPipe &&
         ::dup2(child_output[1], STDERR_FILENO) < 0))
      ReportChildFailure(errors[1], ChildStep::kOutput);
  }
  ::execv(c_args[0], c_args.data());
  ReportChildFailure(errors[1], ChildStep::kExec);
}

// ---------------------------------------------------------------------------
// Child output
//
// Every roster member writes its stdout and stderr into one pipe, which a
// ChildOutput drains into the member's `<output_dir>/<name>.log`, every run
// appended to the one file after a line naming the run and its pid, and,
// except for the Studio bridge, echoes to the launcher's stdout.
//
// A child's write blocks only while its pipe is full, so the pipe is drained
// by a thread that never waits for a disk: it only moves what arrives into a
// bounded buffer in memory. A second thread writes those buffers to the log
// files, and a third the echo to stdout: a one-line write to a file on a
// loaded disk can take hundreds of milliseconds (222 ms at p99 with eight
// disk writers on the Mac mini), and so can stdout when it is a file or a
// pipe read slowly. While a writer is behind, a member's buffer (or the
// echo's) grows to kChildOutputBufferBytes; what arrives beyond it is
// dropped, counted, and marked where it was lost.
//
// The child's end of the pipe stays blocking. A nonblocking one would turn a
// full pipe into EAGAIN inside the child: stdio loses the data and sets the
// stream's error flag, iostreams set badbit, which silences std::cout or
// std::cerr for the rest of the run, and Python raises BlockingIOError out of
// print(). Losing output here is the gentler failure, and the one that can be
// counted; a child still blocks if the drain thread is not scheduled while it
// fills its pipe (64 KiB on macOS, kChildOutputPipeBytes on Linux).
// ---------------------------------------------------------------------------

// What one member's output, or the echo, may hold in memory while the writer
// is behind.
inline constexpr std::size_t kChildOutputBufferBytes = std::size_t{1} << 20;
// What one member's output log may hold in a session; later output is
// dropped, counted, and the log says so.
inline constexpr uint64_t kChildOutputLogMaxBytes = uint64_t{64} << 20;

// The log a member's stdout and stderr are drained into, all runs together.
inline std::string NodeOutputPath(const std::string& output_dir,
                                  std::string_view name) {
  return output_dir + "/" + std::string{name} + ".log";
}

class ChildOutput {
 public:
  using Clock = std::chrono::steady_clock;

  // Starts the reader, writer and echo threads, which block every signal so
  // that the launcher's handlers run on its own threads. Throws
  // std::system_error when a pipe or thread cannot be created.
  ChildOutput() {
    {
      const std::lock_guard lock{SpawnMutex()};
      if (!CloseOnExecPipe(wake_))
        throw std::system_error(errno, std::generic_category(),
                                "child output wake pipe");
    }
    for (const int fd : wake_)
      ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK);
    sigset_t all;
    sigset_t previous;
    sigfillset(&all);
    ::pthread_sigmask(SIG_SETMASK, &all, &previous);
    try {
      reader_ = std::thread{[this] { ReadLoop(); }};
      writer_ = std::thread{[this] { WriteLoop(); }};
      echoer_ = std::thread{[this] { EchoLoop(); }};
    } catch (...) {
      ::pthread_sigmask(SIG_SETMASK, &previous, nullptr);
      Stop();
      throw;
    }
    ::pthread_sigmask(SIG_SETMASK, &previous, nullptr);
  }
  ~ChildOutput() { Stop(); }
  ChildOutput(const ChildOutput&) = delete;
  ChildOutput& operator=(const ChildOutput&) = delete;

  // Drains `reader`, an output pipe SpawnProcess returned for `name`'s run
  // `pid`, into `path`; with `echo`, also to the launcher's stdout. Takes the
  // descriptor.
  void Add(int reader, const std::string& path, std::string_view name,
           pid_t pid, bool echo) {
    ::fcntl(reader, F_SETFL, ::fcntl(reader, F_GETFL) | O_NONBLOCK);
    {
      const std::lock_guard lock{mutex_};
      if (stopping_) {
        ::close(reader);
        return;
      }
      // The reader writes the header once it has drained the previous run's
      // pipe, so that run's last output is not credited to this one.
      Pending& pending = pending_[path];
      added_.push_back({reader, path, echo,
                        "[launcher] " + std::string{name} + " run " +
                            std::to_string(pending.runs++) + ", pid " +
                            std::to_string(pid) + "\n"});
      dirty_ = true;
    }
    arrived_.notify_all();
    Wake();
  }

  // Waits up to `linger` for every pipe to end -- its run has exited, but a
  // descendant that left the process group can hold it open -- writes out
  // everything that arrived, and stops the threads. The echo still waits for
  // a stdout that does not drain, as every launcher line does. Idempotent.
  void Stop(std::chrono::milliseconds linger = std::chrono::milliseconds{500}) {
    {
      const std::lock_guard lock{mutex_};
      if (stopping_ && !reader_.joinable() && !writer_.joinable() &&
          !echoer_.joinable())
        return;
      stopping_ = true;
      linger_ = linger;
    }
    Wake();
    if (reader_.joinable()) reader_.join();
    {
      const std::lock_guard lock{mutex_};
      reader_done_ = true;
    }
    arrived_.notify_all();
    if (writer_.joinable()) writer_.join();
    if (echoer_.joinable()) echoer_.join();
    for (int& fd : wake_) {
      if (fd >= 0) ::close(fd);
      fd = -1;
    }
  }

  // Bytes of the output bound for `path` that were dropped so far: the
  // writer fell behind, the log reached kChildOutputLogMaxBytes, or a write
  // to it failed.
  uint64_t Dropped(const std::string& path) const {
    const std::lock_guard lock{mutex_};
    const auto found = pending_.find(path);
    return found == pending_.end() ? 0 : found->second.dropped_total;
  }

 private:
  struct Stream {
    int fd;
    std::string path;
    bool echo;
    std::string header;  // Written to the log when the reader picks it up.
  };
  // Output bound for one path that the writer has not taken yet.
  struct Pending {
    std::string bytes;
    uint64_t dropped{0};  // Since the writer last took `bytes`.
    uint64_t dropped_total{0};
    int runs{0};
  };
  struct LogFile {
    int fd{-1};
    uint64_t written{0};
    bool capped{false};
    bool reported{false};
  };

  // A full wake pipe already holds a wakeup.
  void Wake() {
    if (wake_[1] < 0) return;
    const char byte = 0;
    [[maybe_unused]] const ssize_t wrote = ::write(wake_[1], &byte, 1);
  }

  // A line marking `count` lost bytes, starting a line of its own.
  static std::string DropNote(const std::string& before, uint64_t count,
                              std::string_view why) {
    std::string note = before.empty() || before.back() == '\n' ? "" : "\n";
    return note + "[launcher] " + std::to_string(count) +
           " bytes of output dropped here: " + std::string{why} + "\n";
  }

  // Moves `size` bytes read from `stream` into its buffers, dropping what
  // they cannot hold.
  void Deliver(const Stream& stream, const char* data, std::size_t size) {
    {
      const std::lock_guard lock{mutex_};
      Pending& pending = pending_[stream.path];
      const std::size_t room =
          kChildOutputBufferBytes -
          std::min(kChildOutputBufferBytes, pending.bytes.size());
      const std::size_t kept = std::min(size, room);
      pending.bytes.append(data, kept);
      pending.dropped += size - kept;
      pending.dropped_total += size - kept;
      if (stream.echo) {
        const std::size_t echo_room =
            kChildOutputBufferBytes -
            std::min(kChildOutputBufferBytes, echo_.size());
        const std::size_t echoed = std::min(size, echo_room);
        echo_.append(data, echoed);
        echo_dropped_ += size - echoed;
      }
      dirty_ = true;
    }
    arrived_.notify_all();
  }

  // Reads every registered pipe until it ends, or, once stopping, until
  // `linger_` has passed. Touches no file.
  void ReadLoop() {
    std::vector<Stream> streams;
    std::vector<Stream> picked;
    std::vector<::pollfd> fds;
    std::vector<char> buffer(std::size_t{64} * 1024);
    std::optional<Clock::time_point> give_up;
    for (;;) {
      {
        const std::lock_guard lock{mutex_};
        picked.swap(added_);
        if (stopping_ && !give_up) give_up = Clock::now() + linger_;
      }
      for (auto& stream : picked) {
        // An earlier run's pipe for the same log may still hold its last
        // output (a restart can follow the exit at once): read what it holds
        // before this run's header. Its descendants may keep it open.
        for (auto& earlier : streams) {
          if (earlier.fd < 0 || earlier.path != stream.path) continue;
          for (;;) {
            const ssize_t got =
                ::read(earlier.fd, buffer.data(), buffer.size());
            if (got > 0) {
              Deliver(earlier, buffer.data(), static_cast<std::size_t>(got));
            } else if (got < 0 && errno == EINTR) {
              continue;
            } else {
              if (got == 0 || errno != EAGAIN) {
                ::close(earlier.fd);
                earlier.fd = -1;
              }
              break;
            }
          }
        }
        {
          const std::lock_guard lock{mutex_};
          pending_[stream.path].bytes += stream.header;
          dirty_ = true;
        }
        arrived_.notify_all();
        stream.header.clear();
        streams.push_back(std::move(stream));
      }
      picked.clear();
      std::erase_if(streams,
                    [](const Stream& stream) { return stream.fd < 0; });
      if (give_up && (streams.empty() || Clock::now() >= *give_up)) break;
      fds.assign(1, ::pollfd{wake_[0], POLLIN, 0});
      for (const auto& stream : streams)
        fds.push_back(::pollfd{stream.fd, POLLIN, 0});
      int timeout = -1;
      if (give_up) {
        timeout = static_cast<int>(std::max<int64_t>(
            0, std::chrono::duration_cast<std::chrono::milliseconds>(
                   *give_up - Clock::now())
                       .count() +
                   1));
      }
      if (::poll(fds.data(), fds.size(), timeout) < 0) {
        // Never give the pipes up while their children may still write.
        if (errno != EINTR)
          std::this_thread::sleep_for(std::chrono::milliseconds{10});
        continue;
      }
      if (fds[0].revents != 0) {
        char drained[64];
        while (::read(wake_[0], drained, sizeof drained) > 0) {
        }
      }
      std::size_t kept = 0;
      for (std::size_t i = 0; i < streams.size(); ++i) {
        bool open = true;
        if (fds[i + 1].revents != 0) {
          const ssize_t got =
              ::read(streams[i].fd, buffer.data(), buffer.size());
          if (got > 0) {
            Deliver(streams[i], buffer.data(), static_cast<std::size_t>(got));
          } else if (got == 0 || (errno != EINTR && errno != EAGAIN)) {
            open = false;
          }
        }
        if (open) {
          // Never a self-move, which may empty the path.
          if (kept != i) streams[kept] = std::move(streams[i]);
          ++kept;
        } else {
          ::close(streams[i].fd);
        }
      }
      streams.resize(kept);
    }
    for (const auto& stream : streams) ::close(stream.fd);
  }

  // Appends `bytes` to `path`'s log up to kChildOutputLogMaxBytes. Returns
  // the bytes it could not write.
  static uint64_t WriteLog(LogFile& file, const std::string& path,
                           std::string bytes) {
    if (file.capped) return bytes.size();
    if (file.fd < 0) {
      file.fd =
          ::open(path.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
      if (file.fd < 0) {
        if (!file.reported)
          std::cerr << "[launcher] cannot open " << path << " ("
                    << std::strerror(errno) << "); its output is dropped\n";
        file.reported = true;
        return bytes.size();
      }
    }
    uint64_t lost = 0;
    const std::string cap_note = "\n[launcher] output log reached " +
                                 std::to_string(kChildOutputLogMaxBytes) +
                                 " bytes; later output is dropped\n";
    if (file.written + bytes.size() > kChildOutputLogMaxBytes) {
      const uint64_t room = kChildOutputLogMaxBytes -
                            std::min<uint64_t>(kChildOutputLogMaxBytes,
                                               file.written + cap_note.size());
      lost = bytes.size() - std::min<uint64_t>(room, bytes.size());
      bytes.resize(bytes.size() - lost);
      bytes += cap_note;
      file.capped = true;
    }
    const std::size_t unwritten = WriteAll(file.fd, bytes);
    file.written += bytes.size() - unwritten;
    if (unwritten != 0 && !file.reported) {
      std::cerr << "[launcher] cannot write " << path << " ("
                << std::strerror(errno) << "); output is dropped\n";
      file.reported = true;
    }
    return lost + unwritten;
  }

  // Writes all of `bytes` to `fd`. Returns the bytes left unwritten, with
  // errno set, after an error.
  static std::size_t WriteAll(int fd, std::string_view bytes) {
    while (!bytes.empty()) {
      const ssize_t wrote = ::write(fd, bytes.data(), bytes.size());
      if (wrote < 0) {
        if (errno == EINTR) continue;
        return bytes.size();
      }
      bytes.remove_prefix(static_cast<std::size_t>(wrote));
    }
    return 0;
  }

  // Takes what the reader buffered and writes it out, until the reader has
  // ended and nothing is left.
  void WriteLoop() {
    std::map<std::string, LogFile> files;
    std::unique_lock lock{mutex_};
    for (;;) {
      arrived_.wait(lock, [this] { return dirty_ || reader_done_; });
      if (!dirty_ && reader_done_) break;
      dirty_ = false;
      std::vector<std::pair<std::string, std::string>> batches;
      for (auto& [path, pending] : pending_) {
        if (pending.bytes.empty() && pending.dropped == 0) continue;
        std::string bytes = std::move(pending.bytes);
        pending.bytes.clear();
        if (pending.dropped != 0) {
          bytes += DropNote(bytes, pending.dropped,
                            "the launcher's log writer fell behind");
          pending.dropped = 0;
        }
        batches.emplace_back(path, std::move(bytes));
      }
      lock.unlock();
      std::vector<std::pair<std::string, uint64_t>> lost;
      for (auto& [path, bytes] : batches) {
        const uint64_t count = WriteLog(files[path], path, std::move(bytes));
        if (count != 0) lost.emplace_back(path, count);
      }
      lock.lock();
      for (const auto& [path, count] : lost)
        pending_[path].dropped_total += count;
    }
    lock.unlock();
    for (auto& [path, file] : files)
      if (file.fd >= 0) ::close(file.fd);
  }

  // Copies echoed output to the launcher's stdout, on its own thread: a
  // stdout that is slow, or a pipe nobody reads, never holds up the logs.
  void EchoLoop() {
    std::unique_lock lock{mutex_};
    for (;;) {
      arrived_.wait(lock, [this] {
        return !echo_.empty() || echo_dropped_ != 0 || reader_done_;
      });
      if (echo_.empty() && echo_dropped_ == 0) break;  // reader_done_
      std::string echo = std::move(echo_);
      echo_.clear();
      if (echo_dropped_ != 0) {
        echo +=
            DropNote(echo, echo_dropped_, "the launcher's stdout fell behind");
        echo_dropped_ = 0;
      }
      lock.unlock();
      // Lost if stdout is closed.
      WriteAll(STDOUT_FILENO, echo);
      lock.lock();
    }
  }

  int wake_[2]{-1, -1};
  mutable std::mutex mutex_;
  std::condition_variable arrived_;
  // Guarded by mutex_.
  std::vector<Stream> added_;
  std::map<std::string, Pending> pending_;
  std::string echo_;
  uint64_t echo_dropped_{0};
  bool dirty_{false};
  bool stopping_{false};
  bool reader_done_{false};
  std::chrono::milliseconds linger_{0};
  std::thread reader_;
  std::thread writer_;
  std::thread echoer_;
};

// While it lives, a SIGCHLD wakes Wait(): the supervision loop reaps an exit
// as it happens rather than at its next pass. The self-pipe is created once
// per process and never closed, so a late signal can never write into a
// descriptor number another file has reused; the previous SIGCHLD action is
// restored on destruction. The handler only writes one byte (SA_RESTART;
// errno preserved), so it is safe on any thread that receives it.
class ChildExitWakeup {
 public:
  ChildExitWakeup() {
    {
      const std::lock_guard lock{SpawnMutex()};
      if (pipe_[0] < 0) {
        int fds[2];
        if (CloseOnExecPipe(fds)) {
          for (const int fd : fds)
            ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK);
          pipe_[0] = fds[0];
          pipe_[1] = fds[1];
        }
      }
    }
    if (pipe_[0] < 0) return;  // Wait() then sleeps, as a poll would.
    struct sigaction action{};
    action.sa_handler = &ChildExitWakeup::OnSignal;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_RESTART | SA_NOCLDSTOP;
    installed_ = ::sigaction(SIGCHLD, &action, &previous_) == 0;
  }
  ~ChildExitWakeup() {
    if (installed_) ::sigaction(SIGCHLD, &previous_, nullptr);
  }
  ChildExitWakeup(const ChildExitWakeup&) = delete;
  ChildExitWakeup& operator=(const ChildExitWakeup&) = delete;

  // Returns after `timeout`, or sooner once a child has exited since the
  // last call.
  void Wait(std::chrono::milliseconds timeout) const {
    if (!installed_) {
      std::this_thread::sleep_for(timeout);
      return;
    }
    ::pollfd ready{pipe_[0], POLLIN, 0};
    if (::poll(&ready, 1, static_cast<int>(timeout.count())) > 0) {
      char drained[64];
      while (::read(pipe_[0], drained, sizeof drained) > 0) {
      }
    }
  }

 private:
  static void OnSignal(int /*signal*/) {
    const int saved = errno;
    const char byte = 0;
    [[maybe_unused]] const ssize_t wrote = ::write(pipe_[1], &byte, 1);
    errno = saved;
  }

  static inline int pipe_[2]{-1, -1};
  struct sigaction previous_{};
  bool installed_{false};
};

// SpawnProcess with the child's stdout and stderr drained into `path` by
// `output` (echoed to the launcher's stdout with `echo`).
inline pid_t SpawnDrained(std::vector<std::string> args, std::string& error,
                          ChildOutput& output, const std::string& path,
                          std::string_view name, bool echo) {
  int reader = -1;
  const pid_t pid =
      SpawnProcess(std::move(args), error, SpawnOutput::kPipe, &reader);
  if (pid > 0) output.Add(reader, path, name, pid, echo);
  return pid;
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

// "9 SIGKILL": a signal's number, which differs between Linux and macOS
// for some signals, and its name where it has a portable one.
inline std::string SignalText(int signal) {
  struct Named {
    int signal;
    const char* name;
  };
  static constexpr Named kNames[] = {
      {SIGHUP, "SIGHUP"},   {SIGINT, "SIGINT"},   {SIGQUIT, "SIGQUIT"},
      {SIGILL, "SIGILL"},   {SIGTRAP, "SIGTRAP"}, {SIGABRT, "SIGABRT"},
      {SIGBUS, "SIGBUS"},   {SIGFPE, "SIGFPE"},   {SIGKILL, "SIGKILL"},
      {SIGUSR1, "SIGUSR1"}, {SIGSEGV, "SIGSEGV"}, {SIGUSR2, "SIGUSR2"},
      {SIGPIPE, "SIGPIPE"}, {SIGALRM, "SIGALRM"}, {SIGTERM, "SIGTERM"},
      {SIGXCPU, "SIGXCPU"}, {SIGXFSZ, "SIGXFSZ"}, {SIGSYS, "SIGSYS"},
  };
  std::string text = std::to_string(signal);
  for (const auto& named : kNames) {
    if (named.signal == signal) return text + " " + named.name;
  }
  return text;
}

// Runs `args` (a binary and --describe) and parses the manifest from its
// stdout, killing it after `timeout`: a node that blocks in its constructor
// costs the launch that much and no more. stderr is left alone so a node that
// throws says so. A binary that cannot be run says why (SpawnProcess).
inline bool ProbeDescribe(const std::vector<std::string>& args,
                          std::chrono::milliseconds timeout,
                          introspect::Description& out, std::string& error) {
  error.clear();
  int answer = -1;
  const pid_t pid =
      SpawnProcess(args, error, SpawnOutput::kStdoutPipe, &answer);
  if (pid < 0) return false;

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
    ::pollfd waiting{answer, POLLIN, 0};
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
    const ssize_t got = ::read(answer, buffer, sizeof(buffer));
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
  ::close(answer);

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
      error += "; killed by signal " + SignalText(WTERMSIG(status));
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

// Calls `run(i)` for every i below `count`, on up to one thread per hardware
// thread, and returns when all have returned. `run` must not throw.
inline void ForEachInParallel(std::size_t count,
                              const std::function<void(std::size_t)>& run) {
  const std::size_t width = std::min<std::size_t>(
      count, std::max(1u, std::thread::hardware_concurrency()));
  std::atomic<std::size_t> next{0};
  const auto work = [&] {
    for (std::size_t i = next++; i < count; i = next++) run(i);
  };
  std::vector<std::thread> threads;
  threads.reserve(width > 0 ? width - 1 : 0);
  for (std::size_t t = 1; t < width; ++t) {
    try {
      threads.emplace_back(work);
    } catch (const std::system_error&) {
      break;  // Fewer threads still run every index.
    }
  }
  work();
  for (auto& thread : threads) thread.join();
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
// reader therefore sees every node but the graph readers. Each pass probes
// its binaries in parallel (ForEachInParallel), so a pass takes about as long
// as its slowest probe rather than the sum of them.
// Throws when that file cannot be written: a graph reader is never described,
// or started, without the graph.
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
    try {
      described.answered = ProbeDescribe(
          args, std::chrono::milliseconds{options.describe_timeout_ms},
          described.declared, described.error);
    } catch (const std::exception& e) {
      described.answered = false;
      described.error = e.what();
    }
  };

  for (const auto& spec : specs) {
    NodeDescription& described = out.emplace_back();
    described.roster_name = spec.name;
    described.target = spec.target;
    described.binary_path = spec.binary_path;
  }
  ForEachInParallel(specs.size(),
                    [&](std::size_t i) { probe(specs[i], out[i], ""); });
  const bool any_reader =
      std::any_of(out.begin(), out.end(), [](const NodeDescription& node) {
        return node.answered && node.declared.reads_graph;
      });

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
    std::string error;
    if (!WriteFileAtomically(graph_path, introspect::DeclaredGraphToJson(graph),
                             options.sync_file, error)) {
      throw std::runtime_error("cannot write the declared graph " + graph_path +
                               " (" + error + ")");
    }
    std::vector<std::size_t> readers;
    for (std::size_t i = 0; i < specs.size(); ++i) {
      if (out[i].answered && out[i].declared.reads_graph) readers.push_back(i);
    }
    ForEachInParallel(readers.size(), [&](std::size_t k) {
      probe(specs[readers[k]], out[readers[k]], graph_path);
    });
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
// A critical member's first crash in that window restarts it at once: the
// robot cannot run without it, and a crash loop still backs off from its
// second restart on.
// max_restarts limits starts within that window, not the session lifetime.
// A critical member that exits cleanly (status 0) while the session is not
// stopping did not finish anything -- its peers still run -- so it restarts,
// always after the backoff; a non-critical clean exit is a node that finished.
// A run that cannot be spawned at all (fork or pipe failing, as under EAGAIN
// or EMFILE) is rescheduled like a crash, against the same budget, so a
// transient shortage never leaves a member down for the rest of a session
// the launcher -- and so its service -- outlives.
// When a clean exit's restart falls due and every other live member's latest
// run also exited 0, the whole roster has completed and the restart is
// dropped (see RosterContinues); the backoff gives such a roster's other
// members the time to exit. The backoff is scheduled inside the
// supervision loop, so a restarting node never delays another node or the
// session deadline. Every run writes its own event log.
// ---------------------------------------------------------------------------

inline constexpr int kDefaultMaxRestarts = 5;
inline constexpr std::chrono::milliseconds kRestartInitialBackoff{50};
inline constexpr std::chrono::milliseconds kRestartMaxBackoff{1000};

// The wait before restart `restart` (1 for the first in the rolling window)
// of a `critical` member or not, after a `clean_exit` or a crash.
inline std::chrono::milliseconds RestartBackoff(int restart, bool clean_exit,
                                                bool critical) {
  if (restart == 1 && critical && !clean_exit) {
    return std::chrono::milliseconds{0};
  }
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
// Only the flag may be 0, which disables every restart, critical members'
// included: a robot config that did so would leave the robot without its
// drivetrain after one crash, so the key must be at least 1.
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
  if (value == nullptr || value->get() < 1 ||
      value->get() > std::numeric_limits<int>::max()) {
    error =
        "[robot] max_restarts must be an integer >= 1 (only --max-restarts 0 "
        "disables restarts)";
    return false;
  }
  out = static_cast<int>(value->get());
  return true;
}

// "exit code N" or "signal N SIGNAME", from a waitpid status.
inline std::string ExitReason(int status) {
  if (WIFSIGNALED(status)) return "signal " + SignalText(WTERMSIG(status));
  return "exit code " +
         std::to_string(WIFEXITED(status) ? WEXITSTATUS(status) : -1);
}

// ---------------------------------------------------------------------------
// Studio bridge
//
// The one roster member supervised differently. It takes positional args
// rather than the node flags, and logs plaintext to studio_bridge.log rather
// than a .tlog. It waits for the telemetry topic itself, so an attempt that
// exits quickly failed for some other reason. It is retried at most once per
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
               const std::string& graph_path, ChildOutput& output)
      : index_{index},
        target_{spec.target},
        log_path_{NodeOutputPath(options.output_dir, kStudioBridgeName)},
        output_{output},
        http_port_{EffectiveStudioHttpPort(robot_config, options)},
        args_{spec.binary_path,
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
    std::string error;
    const pid_t pid = Spawn(error);
    if (pid < 0) {
      std::cerr << "[launcher] cannot spawn studio_bridge (" << error
                << "); session runs without Studio\n";
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
      std::string error;
      const pid_t pid = Spawn(error);
      if (pid < 0) {
        std::cerr << "[launcher] cannot spawn studio_bridge (" << error
                  << "); session runs without Studio\n";
        GiveUp(proc);
        return;
      }
      if (attempts_ == 1 || attempts_ % 15 == 0) {
        std::cout << "[launcher] studio_bridge not up yet (exit " << last_code
                  << "); retrying...\n";
      }
      proc.pid = pid;
      proc.exited = false;
      proc.exit_code = -1;
      proc.exit_signal = 0;
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
      proc.exit_signal = WIFSIGNALED(status) ? WTERMSIG(status) : 0;
    }
    GiveUp(proc);
  }

 private:
  // One attempt, its output drained into log_path_ only, never echoed.
  pid_t Spawn(std::string& error) {
    return SpawnDrained(args_, error, output_, log_path_, kStudioBridgeName,
                        /*echo=*/false);
  }

  void GiveUp(NodeProcessInfo& proc) {
    proc.failed = true;
    proc.exit_code = -1;
    active_ = false;
  }

  std::size_t index_;
  std::string target_;
  std::string log_path_;
  ChildOutput& output_;
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
    // A run that starts the roster owns the namespace's topics and registry
    // for its lifetime. --describe-only probes, and --replay-inputs runs one
    // node on recorded topics, without shared memory (RunOnRecordedInputs).
    const bool starts_roster =
        !options_.describe_only && options_.replay_inputs.empty();
    session_lock_ = nullptr;
    std::optional<SessionLock> session_lock;
    if (starts_roster) {
      std::string lock_error;
      session_lock = SessionLock::Acquire(lock_error);
      if (!session_lock) {
        std::cerr << "launcher: " << lock_error << "\n";
        return 1;
      }
      session_lock_ = &*session_lock;
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
    std::string effective_remote;
    try {
      effective_remote = EffectiveRemoteAddress(robot_config, options_);
    } catch (const std::exception& e) {
      std::cerr << "launcher: " << e.what() << "\n";
      return 1;
    }
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
    // Before any probe, so an unusable registry refuses the launch at once
    // rather than after the roster is described.
    if (starts_roster) {
      try {
        introspect::PrepareRegistry();
        // Every member but Studio's bridge registers a row; the sim gateway
        // registers its own.
        const auto registering = static_cast<std::size_t>(std::count_if(
            node_specs.begin(), node_specs.end(), [](const NodeSpec& spec) {
              return spec.name != kStudioBridgeName;
            }));
        const std::string full =
            RegistryCapacityError(introspect::RegistryReader{}, registering);
        if (!full.empty()) {
          std::cerr << "launcher: " << full << "\n";
          return 1;
        }
      } catch (const std::exception& e) {
        std::cerr << "launcher: " << e.what() << "\n";
        return 1;
      }
    }

    // Phase one: describe and lint before anything starts.
    std::vector<NodeDescription> described;
    std::vector<naming::Diagnostic> diagnostics;
    try {
      diagnostics = DescribeGraph(node_specs, options_, described,
                                  TargetSourcesOf(robot_config),
                                  TargetModeSourcesOf(robot_config),
                                  robot_config.require_target_coverage);
    } catch (const std::exception& e) {
      std::cerr << "launcher: " << e.what() << "\n";
      return 1;
    }
    for (auto& finding : LintParameterUnits(robot_config)) {
      diagnostics.push_back(std::move(finding));
    }
    for (auto& finding : naming::CheckHardwareConfig(robot_config.hardware)) {
      diagnostics.push_back(std::move(finding));
    }
    const std::size_t declared_topics = CountDeclaredTopics(described);

    const std::string graph_path = options_.output_dir + "/graph.json";
    std::string graph_error;
    if (!WriteFileAtomically(
            graph_path,
            GraphToJson(options_.session_id, options_.config_path, described,
                        diagnostics, options_.bundle_config_sha256),
            options_.sync_file, graph_error)) {
      std::cerr << "launcher: cannot write " << graph_path << " ("
                << graph_error << "); nothing was started\n";
      return 1;
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

    // Every member's stdout and stderr, drained into its <name>.log.
    try {
      output_.emplace();
    } catch (const std::exception& e) {
      std::cerr << "launcher: " << e.what() << "; nothing was started\n";
      return 1;
    }

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
                       graph_path, *output_);
      }
      if (!proc.log_path.empty()) proc.log_paths.push_back(proc.log_path);
    }
    // Each logging member's share of the session's log budget, weighted by
    // its expected record rate (SplitLogBudget); its restarts draw on what
    // its earlier runs left (Restart).
    std::vector<uint64_t> log_budgets;
    {
      std::vector<bool> logs(procs.size());
      std::vector<double> rates(procs.size(), -1);
      for (std::size_t i = 0; i < procs.size(); ++i) {
        logs[i] = !procs[i].log_path.empty();
        if (!described[i].answered) continue;
        const auto period_us =
            robot_config.toml_data["subsystems"][procs[i].name]["period_us"]
                .value_or<int64_t>(robot_config.toml_data["robot"]["period_us"]
                                       .value_or<int64_t>(5000));
        rates[i] = ExpectedLogRate(described[i].declared, period_us);
      }
      log_budgets = SplitLogBudget(session_log_budget, logs, rates);
    }
    // Synced before any node starts; from here on every version is synced,
    // off this thread (PersistManifest).
    PersistManifest(procs, /*durable=*/true);
    manifest_writer_->Start();

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
        launches[i].log_budget = log_budgets[i];
        args.insert(args.end(), {proc.log_path, "--log-max-bytes",
                                 std::to_string(log_budgets[i]), "--config",
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

      std::string spawn_error;
      const pid_t pid = SpawnNode(proc, launches[i], spawn_error);
      if (pid < 0) {
        SpawnFailed(proc, launches[i], std::chrono::steady_clock::now(),
                    spawn_error);
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
    // Every run has been reaped and its group killed, so the pipes end.
    output_->Stop();
    for (const auto& proc : procs) {
      const std::string path = NodeOutputPath(options_.output_dir, proc.name);
      if (const uint64_t dropped = output_->Dropped(path); dropped != 0)
        std::cerr << "[launcher] " << proc.name << ": " << dropped
                  << " bytes of output dropped (marked in " << path << ")\n";
    }

    int exit_status = 0;
    for (auto& proc : procs) {
      // Every run's log: a restarted node's records are the sum of its runs.
      for (const auto& path : proc.log_paths) {
        if (std::filesystem::exists(event::log::CappedMarkerPath(path)))
          proc.log_capped = true;
        // The segments a capped log kept, then the newest (LogSegmentFiles).
        for (const auto& file : event::log::LogSegmentFiles(path)) {
          // Streamed: shutdown memory does not grow with the logs' size.
          try {
            const auto summary = event::log::SummarizeLog(file);
            proc.record_count += summary.record_count;
            if (summary.truncated) proc.failed = true;
          } catch (const event::log::IncompleteLogHeader& e) {
            // Cut while it was being created: no records to count.
            std::cerr << "[launcher] skipping " << file << ": " << e.what()
                      << "\n";
          } catch (...) {
            proc.failed = true;
          }
        }
      }
      if (proc.exit_code != 0) exit_status = proc.exit_code;
    }
    manifest.nodes = std::move(procs);

    session_manifest_ = manifest;
    PersistManifest(manifest.nodes, /*durable=*/true);
    PrintManifest(manifest);
    return exit_status;
  }

 private:
  // Replaces manifest.json with the session's state and `procs`. Before the
  // first node starts, and for the final manifest, it is written here and in
  // place when this returns; only a `durable` one is synced (the roster
  // before the first node starts, and the final manifest). Once Run starts
  // the manifest writer, every version is synced, so a power loss mid-match
  // keeps the lifecycle it had reached, and none is written here: the
  // supervision loop restarts crashed nodes and feeds the service watchdog,
  // and a write or sync can take hundreds of milliseconds while other
  // processes write to the same disk (ManifestWriter). A reader (Studio, a
  // test) then sees a version shortly after, not before, what it records.
  //
  // Never throws: a full disk (ENOSPC) or an unwritable session directory
  // must not end supervision of a running robot. The last complete manifest
  // stays in place (WriteFileAtomically) and the next lifecycle change
  // retries. The first failure is logged once, and so is the recovery.
  void PersistManifest(const std::vector<NodeProcessInfo>& procs,
                       bool durable = false) {
    session_manifest_.nodes = procs;
    if (!manifest_writer_) {
      manifest_writer_ = std::make_unique<ManifestWriter>(
          options_.output_dir + "/manifest.json", options_.sync_file);
    }
    std::string json = session_manifest_.ToJson();
    if (manifest_writer_->running() && !durable) {
      manifest_writer_->Post(std::move(json));
      return;
    }
    // A durable version is the last word on every version posted before it.
    manifest_writer_->Stop();
    manifest_writer_->Write(json, durable);
  }

  // What retention makes of a directory under the log root.
  enum class Retained {
    kNo,         // Not a session: left alone.
    kSession,    // A recording: counted against log_keep_sessions and the
                 // byte budget, oldest removed first.
    kAbandoned,  // A launch that started no node and recorded nothing:
                 // never counted, removed once another session directory
                 // is newer.
  };

  // The id a session directory is named by -- std::to_string of a nonzero
  // session id, as the constructor names it -- or nullopt for any other name.
  static std::optional<uint64_t> SessionIdNamed(std::string_view name) {
    uint64_t id = 0;
    const char* const end = name.data() + name.size();
    const auto [stop, error] = std::from_chars(name.data(), end, id);
    if (error != std::errc{} || stop != end || id == 0 ||
        std::to_string(id) != name)
      return std::nullopt;
    return id;
  }

  // The session id in the header of the event log at `path`
  // (talOS/events/log/format.h), or 0 when the file does not start with one.
  // Reads the fixed-size header only.
  static uint64_t LogSessionId(const std::filesystem::path& path) {
    event::log::FileHeader header{};
    std::ifstream in{path, std::ios::binary};
    if (!in.read(reinterpret_cast<char*>(&header), sizeof(header)) ||
        std::memcmp(header.magic, event::log::FILE_MAGIC,
                    sizeof(header.magic)) != 0)
      return 0;
    return header.session_id;
  }

  // Classifies `entry`, one entry of the log root; a symlink is never a
  // session. A session is a directory holding the manifest.json of a launch,
  // or, named by a session id, an event log whose header carries that id
  // (what --replay-inputs leaves). A directory named by a session id that
  // holds nothing, or only what a launch writes before it starts its nodes
  // -- graph.json, the declared graph, an interrupted atomic write's
  // temporary, an empty event log -- is an abandoned launch: --describe-only,
  // a refused graph, or a launch that failed or died before its manifest.
  // Any other content (another session's log, any other file or directory)
  // makes it no session. Sets `error` when it cannot be read.
  static Retained Classify(const std::filesystem::directory_entry& entry,
                           std::error_code& error) {
    namespace fs = std::filesystem;
    if (entry.symlink_status(error).type() != fs::file_type::directory)
      return Retained::kNo;
    const fs::file_type manifest =
        fs::status(entry.path() / "manifest.json", error).type();
    if (manifest == fs::file_type::regular) return Retained::kSession;
    if (manifest != fs::file_type::not_found) return Retained::kNo;
    error.clear();  // Absent is an answer, not an error.
    const auto id = SessionIdNamed(entry.path().filename().string());
    if (!id) return Retained::kNo;
    constexpr std::string_view kLaunchFiles[] = {
        "graph.json", "declared_graph.json", "graph.json.tmp",
        "declared_graph.json.tmp", "manifest.json.tmp"};
    bool abandoned = true;
    for (fs::directory_iterator file{entry.path(), error};
         !error && file != fs::directory_iterator{}; file.increment(error)) {
      const fs::file_type type = file->symlink_status(error).type();
      if (error) break;
      const std::string name = file->path().filename().string();
      const bool regular = type == fs::file_type::regular;
      if (regular && name.ends_with(".tlog")) {
        if (LogSessionId(file->path()) == *id) return Retained::kSession;
        const uintmax_t bytes = file->file_size(error);
        if (error) break;
        abandoned = abandoned && bytes == 0;
      } else if (!regular ||
                 std::ranges::find(kLaunchFiles, std::string_view{name}) ==
                     std::ranges::end(kLaunchFiles)) {
        abandoned = false;
      }
    }
    if (error) return Retained::kNo;
    return abandoned ? Retained::kAbandoned : Retained::kNo;
  }

  // Bytes in the regular files under `dir`, symlinks not followed. Sets
  // `error` when any of the tree cannot be read.
  static uintmax_t TreeBytes(const std::filesystem::path& dir,
                             std::error_code& error) {
    namespace fs = std::filesystem;
    uintmax_t bytes = 0;
    fs::recursive_directory_iterator file{dir, error};
    while (!error && file != fs::recursive_directory_iterator{}) {
      const fs::file_type type = file->symlink_status(error).type();
      if (error) break;
      if (type == fs::file_type::regular) {
        const uintmax_t size = file->file_size(error);
        if (error) break;
        bytes += size;
      }
      file.increment(error);
    }
    return bytes;
  }

  // Removes the oldest sessions (Classify) until at most `log_keep_sessions`
  // remain (the new one included) and they hold at most the half of
  // `log_max_bytes` the new session does not get, and every abandoned launch
  // older than another session directory. Returns the new session's log
  // budget: the other half, which its event writers are capped to.
  // Graph/manifest metadata and non-event captures require separate headroom.
  // A session that cannot be read or removed (another user's, unreadable,
  // removed meanwhile) is reported and passed over, never counted against
  // the others: retention never stops a launch. The session --replay-inputs
  // reads is passed over too: this launch is about to read it.
  uint64_t ApplyRetention(const config::RobotConfig& config) {
    namespace fs = std::filesystem;
    // Config load refuses a wrong type or a nonpositive value.
    const int64_t cap = config.log_keep_sessions;
    const int64_t budget = config.log_max_bytes;
    const auto session_budget = static_cast<uint64_t>(budget) / 2;
    const uint64_t retained_budget =
        static_cast<uint64_t>(budget) - session_budget;
    const auto report = [](std::string_view what, const fs::path& path,
                           const std::error_code& error) {
      std::cerr << "[launcher] retention " << what << " " << path.string()
                << " (" << error.message() << ")\n";
    };
    struct Session {
      fs::path path;
      fs::file_time_type time;
      uintmax_t bytes;
    };
    std::vector<Session> sessions;
    std::vector<Session> abandoned;
    // The newest session directory other than the new one. A launch still
    // describing has no manifest.json yet and reads as abandoned; one is
    // removed only when another session directory is newer, so the newest
    // launch before this one is kept, and a crash loop of launches that stop
    // before their nodes (refused graphs included) leaves one behind rather
    // than one per attempt, and evicts no recording.
    auto newest = fs::file_time_type::min();
    uintmax_t total = 0;
    const fs::path current{options_.output_dir};
    std::error_code error;
    for (fs::directory_iterator entry{current.parent_path(), error};
         !error && entry != fs::directory_iterator{}; entry.increment(error)) {
      if (entry->path() == current) continue;  // Counted below.
      std::error_code ignored;  // An input that is gone is the replay's error.
      if (!options_.replay_inputs.empty() &&
          fs::equivalent(entry->path(), options_.replay_inputs, ignored))
        continue;
      std::error_code unreadable;
      const Retained kind = Classify(*entry, unreadable);
      if (kind == Retained::kNo) {
        if (unreadable) report("skips", entry->path(), unreadable);
        continue;
      }
      const auto time = entry->last_write_time(unreadable);
      const uintmax_t bytes = unreadable || kind == Retained::kAbandoned
                                  ? 0
                                  : TreeBytes(entry->path(), unreadable);
      if (unreadable) {
        report("skips", entry->path(), unreadable);
        continue;
      }
      newest = std::max(newest, time);
      if (kind == Retained::kAbandoned) {
        abandoned.push_back({entry->path(), time, bytes});
        continue;
      }
      total += bytes;
      sessions.push_back({entry->path(), time, bytes});
    }
    if (error) {
      // A partial listing could remove a session newer than one it missed.
      report("removes nothing; cannot list", current.parent_path(), error);
      return session_budget;
    }
    for (const auto& launch : abandoned) {
      if (launch.time >= newest) continue;
      std::error_code removal;
      fs::remove_all(launch.path, removal);
      if (removal) report("cannot remove", launch.path, removal);
    }
    std::sort(
        sessions.begin(), sessions.end(),
        [](const Session& a, const Session& b) { return a.time < b.time; });
    // The new session counts too, before it holds anything.
    std::size_t count = sessions.size() + 1;
    for (const auto& session : sessions) {
      if (count <= static_cast<uint64_t>(cap) && total <= retained_budget)
        break;
      std::error_code removal;
      fs::remove_all(session.path, removal);
      // One left in place is passed over like an unreadable one, rather than
      // costing a newer session its place.
      if (removal) report("cannot remove", session.path, removal);
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
    // Event log flags of its registry rows already reported (NoteRecording).
    uint32_t recording_reported{0};
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
    std::string error;
    const pid_t pid = SpawnProcess(
        {spec->binary_path, "--replay-inputs", options_.replay_inputs,
         "--session-id", std::to_string(options_.session_id), "--log", log_path,
         "--config", options_.config_path, "--sim"},
        error);
    if (pid < 0) {
      std::cerr << "[launcher] cannot spawn " << spec->name << " (" << error
                << ")\n";
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

  // SpawnProcess for one run of a roster member: -1 with `error` naming the
  // failed step when it fails, or while the spawn_failures seam names the
  // member.
  pid_t SpawnNode(const NodeProcessInfo& proc, const NodeLaunch& launch,
                  std::string& error) {
    const auto injected = options_.spawn_failures.find(proc.name);
    if (injected != options_.spawn_failures.end() && injected->second > 0) {
      --injected->second;
      error = std::string{"fork: "} + std::strerror(EAGAIN);
      return -1;
    }
    return SpawnDrained(launch.args, error, *output_,
                        NodeOutputPath(options_.output_dir, proc.name),
                        proc.name, /*echo=*/true);
  }

  // Schedules `proc`'s next start: after a backoff that doubles with each
  // start in the rolling window (none for a critical member's first crash
  // there) or, for a non-critical member that has spent its budget there,
  // quarantined until the oldest of those starts leaves the window. Only for
  // a restart ShouldRestartNode allows. Returns the delay.
  std::chrono::milliseconds ScheduleRestart(
      NodeProcessInfo& proc, NodeLaunch& launch,
      std::chrono::steady_clock::time_point now) {
    const int recent = static_cast<int>(launch.restart_times.size());
    proc.quarantined = !launch.critical && recent >= max_restarts_;
    launch.restart_pending = true;
    launch.restart_at =
        (proc.quarantined
             ? launch.restart_times.front() +
                   std::chrono::milliseconds{options_.restart_window_ms}
             : now) +
        RestartBackoff(recent + 1, launch.last_exit_clean, launch.critical);
    proc.retry_at_monotonic_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            launch.restart_at.time_since_epoch())
            .count();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        launch.restart_at - now);
  }

  // A run that cannot be spawned is rescheduled like a crash (see Restarts
  // above); with restarts disabled it stays down, as a crashed one does.
  // `error` names the failed step and its errno (SpawnProcess).
  void SpawnFailed(NodeProcessInfo& proc, NodeLaunch& launch,
                   std::chrono::steady_clock::time_point now,
                   const std::string& error) {
    proc.failed = true;
    std::cerr << "[launcher] cannot spawn " << proc.name << " (" << error
              << "); ";
    // Never `stopping` here: Supervise cancels a pending restart once the
    // session stops.
    if (!ShouldRestartNode(/*clean_exit=*/false, launch.critical, max_restarts_,
                           /*stopping=*/false)) {
      std::cerr << "restarts are disabled\n";
      return;
    }
    const auto delay = ScheduleRestart(proc, launch, now);
    std::cerr << (proc.quarantined ? "quarantined; retry" : "retry") << " in "
              << delay.count() << " ms\n";
  }

  // Starts `proc`'s pending run: restart `restart_count + 1` into a fresh
  // event log or, when no run has spawned yet, the first. The attempt counts
  // against the rolling budget whether or not it spawns.
  // The byte cap for a member's next run: what its earlier runs left of its
  // share `budget`, but never less than half of it, so a run restarted after
  // one that used the whole share still records the match's tail. To make
  // that room it deletes the earlier runs' oldest files (whole segments,
  // oldest run first), never the newest file of the last run, and marks each
  // run it shortened capped (`<log>.capped`, `log_capped`).
  static uint64_t RestartLogBudget(NodeProcessInfo& proc, uint64_t budget) {
    struct File {
      std::string path;
      std::string run;  // The run's log path, for its cap marker.
      uint64_t bytes;
    };
    std::vector<File> files;
    uint64_t used = 0;
    for (const auto& run : proc.log_paths) {
      for (auto& file : event::log::LogSegmentFiles(run)) {
        std::error_code error;
        const auto bytes = std::filesystem::file_size(file, error);
        if (error) continue;
        used += bytes;
        files.push_back({std::move(file), run, bytes});
      }
    }
    const uint64_t floor = budget / 2;
    // The last file is the newest of the run that just ended: kept.
    for (std::size_t i = 0;
         i + 1 < files.size() && (used >= budget || budget - used < floor);
         ++i) {
      std::error_code error;
      if (!std::filesystem::remove(files[i].path, error) || error) continue;
      used -= files[i].bytes;
      std::ofstream{event::log::CappedMarkerPath(files[i].run)};
      proc.log_capped = true;
    }
    // 0 means unbounded to the writer, so never less than 1.
    return std::max<uint64_t>(
        {1, floor, used >= budget ? uint64_t{0} : budget - used});
  }

  void Restart(NodeProcessInfo& proc, NodeLaunch& launch,
               std::chrono::steady_clock::time_point now) {
    launch.restart_pending = false;
    if (proc.quarantined)
      std::cout << "[launcher] " << proc.name << " retrying after quarantine\n";
    proc.quarantined = false;
    proc.retry_at_monotonic_ns = 0;
    // Only a spawned run sets a pid.
    const int restart = proc.pid > 0 ? proc.restart_count + 1 : 0;
    if (launch.log_arg != 0) {
      launch.args[launch.log_arg] =
          NodeLogPath(options_.output_dir, proc.name, restart);
      launch.args[launch.log_arg + 2] =
          std::to_string(RestartLogBudget(proc, launch.log_budget));
    }
    launch.restart_times.push_back(now);
    std::string error;
    const pid_t pid = SpawnNode(proc, launch, error);
    if (pid < 0) {
      SpawnFailed(proc, launch, now, error);
      return;
    }
    if (launch.log_arg != 0 && restart > 0)
      proc.log_paths.push_back(launch.args[launch.log_arg]);
    proc.restart_count = restart;
    proc.pid = pid;
    proc.exited = false;
    proc.exit_code = -1;
    proc.exit_signal = 0;
    launch.started_at = std::chrono::steady_clock::now();
    launch.heartbeat_changed_at = launch.started_at;
    launch.last_heartbeat = 0;
    launch.last_dispatch = 0;
    launch.heartbeat_killed = false;
  }

  // A run's registry row says its event log stopped early
  // (introspect::kFlagRecorderFailed, kFlagLogCapped): the rest of that run
  // is not recorded. Reported once per member and kind, and recorded as the
  // member's recorder_failed or log_capped while the node still runs -- in
  // the live manifest Studio reads, and so in the final one, where a log
  // that stopped at a record boundary would otherwise read as complete.
  // Returns whether `proc` changed.
  static bool NoteRecording(NodeProcessInfo& proc, NodeLaunch& launch,
                            uint32_t flags) {
    const uint32_t fresh =
        flags & ~launch.recording_reported &
        (introspect::kFlagRecorderFailed | introspect::kFlagLogCapped);
    if (fresh == 0) return false;
    launch.recording_reported |= fresh;
    const std::string& log =
        proc.log_paths.empty() ? proc.log_path : proc.log_paths.back();
    if ((fresh & introspect::kFlagRecorderFailed) != 0) {
      proc.failed = true;
      std::cerr << "[launcher] " << proc.name << " event log " << log
                << " failed; the rest of this run is not recorded\n";
    }
    if ((fresh & introspect::kFlagLogCapped) != 0) {
      proc.log_capped = true;
      std::cerr << "[launcher] " << proc.name << " event log " << log
                << " reached its byte budget; the rest of this run is not "
                   "recorded\n";
    }
    return true;
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
    const ChildExitWakeup exits;  // A pass starts as soon as a child exits.
    notifier.Send("READY=1");
    auto next_lock_refresh = std::chrono::steady_clock::time_point::min();
    bool lock_lost_reported = false;
    for (;;) {
      notifier.Send("WATCHDOG=1");
      const auto now = std::chrono::steady_clock::now();
      const bool past_deadline = now >= deadline;
      const bool stopping = stop_requested_ != 0 || stop_sent || past_deadline;
      // A stop signal can land after `stopping` was sampled: systemctl stop
      // signals the launcher and then every node, so a node killed by it can
      // be reaped in this pass. Restart decisions read the flag again, so
      // once shutdown starts nothing is restarted.
      const auto stop_arrived = [&] {
        return stopping || stop_requested_ != 0;
      };
      if (session_lock_ != nullptr && now >= next_lock_refresh) {
        next_lock_refresh = now + kSessionLockRefresh;
        if (!session_lock_->Refresh() && !lock_lost_reported) {
          lock_lost_reported = true;
          std::cerr << "[launcher] the session lock " << session_lock_->path()
                    << " was removed; a second launcher in this namespace "
                       "would no longer be refused\n";
        }
      }
      // Read-only, the attach takes no lock (RegistryMapping): no node can
      // stall this loop, and with it the service watchdog.
      if (!registry) {
        try {
          registry.emplace(introspect::kRegistryPath, true);
        } catch (const std::runtime_error&) {
          // Run prepared it before any node started (PrepareRegistry); this
          // read-only attach never creates it. Left empty, the next pass
          // retries the attach.
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
        // Every member but Studio's bridge, which takes no registry row, is
        // supervised by its row; the simulation gateway registers like a node.
        if (!stopping && !launch.heartbeat_killed &&
            !(studio && i == studio->index())) {
          bool found = false;
          bool constructing = false;
          uint32_t recording = 0;
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
              recording = flags;
              if (beat != launch.last_heartbeat &&
                  (constructing || dispatch != launch.last_dispatch)) {
                launch.heartbeat_changed_at = now;
                launch.last_heartbeat = beat;
                launch.last_dispatch = dispatch;
              }
              break;
            }
          }
          if (NoteRecording(proc, launch, recording)) PersistManifest(procs);
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
        proc.exit_signal = WIFSIGNALED(status) ? WTERMSIG(status) : 0;
        PersistManifest(procs);
        const bool is_bridge = studio && i == studio->index();
        if (is_bridge && studio->Starting(i)) continue;
        const bool clean = WIFEXITED(status) && WEXITSTATUS(status) == 0;
        launch.last_exit_clean = clean;
        if (!is_bridge && ShouldRestartNode(clean, launch.critical,
                                            max_restarts_, stop_arrived())) {
          const auto delay = ScheduleRestart(proc, launch, now);
          std::cout << "[launcher] " << proc.name << " exited ("
                    << ExitReason(status) << "); "
                    << (proc.quarantined ? "quarantined; retry" : "restart")
                    << " " << proc.restart_count + 1 << " in " << delay.count()
                    << " ms\n";
          PersistManifest(procs);
          continue;
        }
        std::cout << "[launcher] Process " << proc.name << " (pid " << proc.pid
                  << ") exited (" << ExitReason(status) << ")\n";
      }
      for (std::size_t i = 0; i < procs.size(); ++i) {
        auto& launch = launches[i];
        if (!launch.restart_pending) continue;
        // A stopping session ends with the crash as the node's last word.
        if (stop_arrived()) {
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
            Restart(procs[i], launch, now);
          }
          PersistManifest(procs);
        }
      }
      if (studio) {
        const auto& before = procs[studio->index()];
        const auto pid = before.pid;
        const bool exited = before.exited;
        const bool failed = before.failed;
        studio->Supervise(procs[studio->index()], now, stop_arrived());
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
      exits.Wait(std::chrono::milliseconds(20));
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
                << ", exit=" << n.exit_code
                << (n.exit_signal != 0
                        ? " (signal " + SignalText(n.exit_signal) + ")"
                        : std::string{})
                << ", records=" << n.record_count
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
  std::unique_ptr<ManifestWriter> manifest_writer_;  // PersistManifest
  int max_restarts_{kDefaultMaxRestarts};
  // Drains every member's output for the session (Run); empty before it.
  std::optional<ChildOutput> output_;
  // Held by Run for the session, refreshed by Supervise; null without one.
  const SessionLock* session_lock_{nullptr};
  // Far inside macOS clean-tmps' 3 unused days (introspect::TouchLockFile).
  static constexpr std::chrono::hours kSessionLockRefresh{1};
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
    } catch (const event::log::IncompleteLogHeader& e) {
      // A log cut while it was being created holds no records.
      std::cerr << "warning: skipping " << path << ": " << e.what() << "\n";
      continue;
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
