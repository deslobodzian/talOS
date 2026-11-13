#include "talOS/launcher/describe_graph.h"

#include <poll.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <functional>
#include <iostream>
#include <iterator>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <tuple>
#include <utility>

#include "talOS/launcher/session.h"
#include "talOS/launcher/spawn.h"
#include "talOS/schema/identity.h"
#include "talOS/schema/unit_suffix.h"
#include "toml.hpp"

namespace talos::launcher {
namespace {

// One key of a parameter value as Node::param names it, and what the
// config holds there, which decides what a read under it covers.
struct ParameterKey {
  enum class Kind { kValue, kArray, kEmptyTable };
  std::string key;
  Kind kind{Kind::kValue};
};

// The keys of one parameter value: a nested table's keys dotted
// ("detector.sigma"), an array of tables' elements indexed
// ("geometry.modules[0].x_m"). An empty table is one key, so an unknown
// table is refused like an unknown key.
void ParameterKeys(const toml::node& value, const std::string& path,
                   std::vector<ParameterKey>& out) {
  if (const toml::table* table = value.as_table()) {
    if (table->empty()) {
      out.push_back({path, ParameterKey::Kind::kEmptyTable});
      return;
    }
    for (const auto& [key, child] : *table) {
      ParameterKeys(child, path + "." + std::string{key.str()}, out);
    }
    return;
  }
  if (const toml::array* array = value.as_array()) {
    if (!array->empty() && array->is_array_of_tables()) {
      for (std::size_t i = 0; i < array->size(); ++i) {
        ParameterKeys((*array)[i], path + "[" + std::to_string(i) + "]", out);
      }
      return;
    }
    out.push_back({path, ParameterKey::Kind::kArray});
    return;
  }
  out.push_back({path, ParameterKey::Kind::kValue});
}

// Levenshtein distance, for the nearest declared key a finding suggests.
std::size_t EditDistance(std::string_view a, std::string_view b) {
  std::vector<std::size_t> row(b.size() + 1);
  for (std::size_t j = 0; j <= b.size(); ++j) row[j] = j;
  for (std::size_t i = 1; i <= a.size(); ++i) {
    std::size_t diagonal = row[0];
    row[0] = i;
    for (std::size_t j = 1; j <= b.size(); ++j) {
      const std::size_t above = row[j];
      row[j] = std::min({row[j] + 1, row[j - 1] + 1,
                         diagonal + (a[i - 1] == b[j - 1] ? 0 : 1)});
      diagonal = above;
    }
  }
  return row[b.size()];
}

// Calls `run(i)` for every i below `count`, on up to one thread per hardware
// thread, and returns when all have returned. `run` must not throw.
void ForEachInParallel(std::size_t count,
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

// The shape each declared topic's segment is created with: its publisher's
// (the first SENDER), else its first end's. Ends that disagree are a lint
// error the session was told to ignore (--allow-graph-errors); the ends that
// differ from the segment then fail at attach, naming both shapes.
std::vector<naming::SourceShape> SessionTopics(
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

// Waits up to `wait` for `stop_fd` to read ready, and says whether it did:
// a stop was requested. Without one (-1) it only waits.
bool StopWithin(int stop_fd, std::chrono::milliseconds wait) {
  if (stop_fd < 0) {
    if (wait.count() > 0) std::this_thread::sleep_for(wait);
    return false;
  }
  ::pollfd stop{stop_fd, POLLIN, 0};
  // EINTR ends the wait early, which every caller's loop absorbs.
  return ::poll(&stop, 1, static_cast<int>(wait.count())) > 0 &&
         (stop.revents & POLLIN) != 0;
}

}  // namespace

bool ProbeDescribe(const std::vector<std::string>& args,
                   std::chrono::milliseconds timeout,
                   introspect::Description& out, std::string& error,
                   int injected_spawn_failures, int stop_fd) {
  error.clear();
  const std::string stopped_error = "the launch was stopped";
  if (StopWithin(stop_fd, std::chrono::milliseconds{0})) {
    error = stopped_error;
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  int answer = -1;
  pid_t pid = -1;
  auto backoff = kProbeSpawnRetryFirst;
  for (int attempt = 1;; ++attempt) {
    SpawnFailure failure;
    if (injected_spawn_failures > 0) {
      --injected_spawn_failures;
      failure = {std::string{kGuardianForkStep}, EAGAIN};
      error = failure.step + ": " + std::strerror(EAGAIN);
    } else {
      pid = SpawnProcess(args, error, SpawnOutput::kStdoutPipe, &answer,
                         &failure);
      if (pid >= 0) {
        // Absorbed, but said: how often the process limit is reached.
        if (attempt > 1)
          std::cerr << "[launcher] probe of " << args.front() << " spawned on "
                    << "attempt " << attempt << " (after " << error << ")\n";
        error.clear();
        break;
      }
    }
    if (!IsTransientSpawnFailure(failure)) return false;
    if (std::chrono::steady_clock::now() + backoff >= deadline) {
      error += " (" + std::to_string(attempt) + " attempts within " +
               std::to_string(timeout.count()) + "ms)";
      return false;
    }
    if (StopWithin(stop_fd, backoff)) {
      error = stopped_error;
      return false;
    }
    backoff = std::min(backoff * 2, kProbeSpawnRetryMax);
  }

  std::string text;
  bool timed_out = false;
  bool stopped = false;
  constexpr std::size_t kMaxDescribeBytes = std::size_t{4} * 1024 * 1024;
  for (;;) {
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    if (left.count() <= 0) {
      timed_out = true;
      break;
    }
    // A negative stop_fd is ignored.
    std::array<::pollfd, 2> waiting{
        {{answer, POLLIN, 0}, {stop_fd, POLLIN, 0}}};
    const int ready =
        ::poll(waiting.data(), waiting.size(), static_cast<int>(left.count()));
    if (ready < 0) {
      if (errno == EINTR) continue;
      error = std::string{"poll() failed: "} + std::strerror(errno);
      break;
    }
    if (ready == 0) {
      timed_out = true;
      break;
    }
    if ((waiting[1].revents & POLLIN) != 0) {
      stopped = true;
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
  if (!timed_out && !stopped && error.empty()) {
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
      if (StopWithin(stop_fd, std::chrono::milliseconds{1})) {
        stopped = true;
        break;
      }
    }
  }
  if (timed_out || stopped || !error.empty()) {
    SignalProcessGroup(pid, SIGKILL);
    while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
    }
  }
  FinishProcessGroup(pid);

  if (stopped) {
    error = stopped_error;
    return false;
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
      error += "; killed by signal " + SignalText(WTERMSIG(status));
    }
    return false;
  }
  return true;
}

naming::TargetSources TargetSourcesOf(const config::RobotConfig& robot_config) {
  naming::TargetSources sources;
  for (const auto& [subsystem, policy] : robot_config.subsystem_targets) {
    sources.emplace(subsystem, policy.Sources());
  }
  return sources;
}

naming::TargetModeSources TargetModeSourcesOf(
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

std::vector<naming::Diagnostic> LintParameterUnits(
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

std::vector<naming::Diagnostic> LintParameterKeys(
    const config::RobotConfig& robot_config,
    const std::vector<NodeDescription>& described,
    const std::string& config_path) {
  std::vector<naming::Diagnostic> findings;
  const toml::table* subsystems =
      robot_config.toml_data["subsystems"].as_table();
  if (subsystems == nullptr) return findings;

  std::map<std::string, std::vector<std::string>, std::less<>> reads;
  std::vector<std::string> unchecked;  // Subsystems whose node's reads are
                                       // unknown.
  std::string failed;                  // Did not answer --describe.
  std::string silent;                  // Answered, without params.
  const auto append = [](std::string& list, const std::string& name) {
    list += (list.empty() ? "" : ", ") + name;
  };
  for (const auto& node : described) {
    if (!subsystems->contains(node.roster_name)) continue;
    if (!node.answered || !node.declared.params) {
      unchecked.push_back(node.roster_name);
      append(node.answered ? silent : failed, node.roster_name);
      if (node.answered) {
        findings.push_back(
            {naming::Severity::WARNING, node.roster_name,
             "its --describe reports no parameters (a binary built before "
             "they were reported?), so its subsystem keys are not checked"});
      }
      continue;
    }
    for (const auto& read : *node.declared.params) {
      reads[read.subsystem].push_back(read.key);
    }
  }

  static const std::vector<std::string> kNone;
  for (const auto& [name_key, value] : *subsystems) {
    const std::string name{name_key.str()};
    const toml::table* table = value.as_table();
    if (table == nullptr ||
        std::ranges::find(unchecked, name) != unchecked.end()) {
      continue;
    }
    const auto found = reads.find(name);
    const std::vector<std::string>& read =
        found == reads.end() ? kNone : found->second;
    const auto covered = [&read](const ParameterKey& set) {
      const std::string& key = set.key;
      const char under = set.kind == ParameterKey::Kind::kArray        ? '['
                         : set.kind == ParameterKey::Kind::kEmptyTable ? '.'
                                                                       : '\0';
      return std::ranges::any_of(read, [&key, under](const std::string& r) {
        return r == key || (under != '\0' && r.size() > key.size() &&
                            r.starts_with(key) && r[key.size()] == under);
      });
    };
    std::vector<ParameterKey> keys;
    for (const auto& [key, child] : *table) {
      if (config::IsFrameworkSubsystemKey(key.str())) continue;
      ParameterKeys(child, std::string{key.str()}, keys);
    }
    for (const auto& set : keys) {
      if (covered(set)) continue;
      const std::string& key = set.key;
      std::string message = "parameter '" + key + "'";
      if (!config_path.empty()) {
        const auto files = config::SubsystemKeyFiles(config_path, name, key);
        for (std::size_t i = 0; i < files.size(); ++i) {
          message += (i == 0 ? " (" : ", ") + files[i];
        }
        if (!files.empty()) message += ")";
      }
      message += " is read by no node's constructor (Node::param)";
      const std::string* nearest = nullptr;
      std::size_t distance = std::max<std::size_t>(2, key.size() / 3) + 1;
      for (const auto& candidate : read) {
        const std::size_t d = EditDistance(key, candidate);
        if (d < distance) {
          distance = d;
          nearest = &candidate;
        }
      }
      if (nearest != nullptr) message += "; did you mean '" + *nearest + "'?";
      if (!failed.empty()) {
        message += " (" + failed +
                   " did not answer --describe; if it reads this key with "
                   "param_of, fix its probe)";
      }
      if (!silent.empty()) {
        message += " (a warning while " + silent + " reports no parameters)";
      }
      findings.push_back(
          {silent.empty() ? naming::Severity::ERROR : naming::Severity::WARNING,
           name, std::move(message)});
    }
  }
  return findings;
}

std::string DeclaredGraphPath(const LauncherOptions& options) {
  const std::filesystem::path dir =
      options.output_dir.empty()
          ? std::filesystem::temp_directory_path() /
                ("talos_describe_" + std::to_string(::getpid()))
          : std::filesystem::path{options.output_dir};
  return (dir / "declared_graph.json").string();
}

std::vector<naming::Diagnostic> DescribeGraph(
    const std::vector<NodeSpec>& specs, const LauncherOptions& options,
    std::vector<NodeDescription>& out,
    const naming::TargetSources& target_sources,
    const naming::TargetModeSources& target_mode_sources,
    bool require_target_coverage, int stop_fd) {
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
    if (described.missing) return;
    const auto injected = options.probe_spawn_failures.find(spec.name);
    try {
      described.answered = ProbeDescribe(
          args, std::chrono::milliseconds{options.describe_timeout_ms},
          described.declared, described.error,
          injected == options.probe_spawn_failures.end() ? 0 : injected->second,
          stop_fd);
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
    std::error_code ignored;
    described.missing =
        std::filesystem::status(spec.binary_path, ignored).type() ==
        std::filesystem::file_type::not_found;
    if (described.missing) {
      described.error = "no binary at " + spec.binary_path;
    }
  }
  ForEachInParallel(specs.size(),
                    [&](std::size_t i) { probe(specs[i], out[i], ""); });
  const bool any_reader =
      std::any_of(out.begin(), out.end(), [](const NodeDescription& node) {
        return node.answered && node.declared.reads_graph;
      });

  // A stop leaves the readers undescribed rather than writing a graph of
  // the probes it cut short.
  if (any_reader && !StopWithin(stop_fd, std::chrono::milliseconds{0})) {
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
  std::vector<NodeSpec> missing;
  for (std::size_t i = 0; i < out.size(); ++i) {
    const auto& described = out[i];
    const auto& spec = specs[i];
    const bool writes_targets =
        std::any_of(described.declared.sources.begin(),
                    described.declared.sources.end(), [](const auto& source) {
                      return source.kind == event::SourceKind::SENDER &&
                             naming::IsTargetLane(source.name);
                    });
    if (!spec.critical && (spec.owns_actuators || writes_targets)) {
      probe_findings.push_back(
          {naming::Severity::ERROR, described.roster_name,
           "critical=false is forbidden for an actuator owner or target-lane "
           "producer"});
    }
    if (described.answered) {
      // The bridge is known by its roster slot, so a replacement bridge
      // binary that describes itself by another name still owns `hw`.
      shapes.push_back({described.declared.name, described.declared.target,
                        described.declared.sources,
                        spec.name == kHardwareNodeName});
      if (described.declared.name != described.roster_name) {
        probe_findings.push_back(
            {naming::Severity::WARNING, described.roster_name,
             "the config calls this node '" + described.roster_name +
                 "' but the binary describes itself as '" +
                 described.declared.name + "'"});
      }
    } else if (spec.studio_bridge) {
      probe_findings.push_back(StudioBridgeFinding(
          spec, described.missing, described.error, options.bundle_dir));
    } else if (described.missing) {
      missing.push_back(spec);
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

  if (auto finding = MissingBinariesFinding(missing, options.bundle_dir)) {
    probe_findings.push_back(std::move(*finding));
  }

  auto diagnostics = naming::LintGraph(
      shapes, target_sources, target_mode_sources, require_target_coverage);
  diagnostics.insert(diagnostics.end(), probe_findings.begin(),
                     probe_findings.end());
  return diagnostics;
}

namespace {

// Why the nonempty `missing` cannot be probed, and the cure: the bundle to
// rebuild, or outside one, the `bazel build` of their labels.
std::string MissingBinariesMessage(const std::vector<NodeSpec>& missing,
                                   const std::string& bundle_dir) {
  std::string labels;
  for (const auto& spec : missing) {
    if (spec.target.starts_with("//") || spec.target.starts_with("@")) {
      labels += " " + spec.target;
    }
  }
  const std::string& first = missing.front().binary_path;
  std::string where = "no binary at " + first;
  if (missing.size() > 1) {
    where += " and " + std::to_string(missing.size() - 1) + " more";
  }
  if (std::filesystem::path{first}.is_relative()) {
    std::error_code ignored;
    where += ", relative to " + std::filesystem::current_path(ignored).string();
  }
  std::string message;
  if (!bundle_dir.empty()) {
    message = "missing from the bundle " + bundle_dir + " (" + where +
              "); rebuild and deploy the bundle";
  } else {
    message = "not built (" + where + ")";
    if (!labels.empty()) message += "; build them first: bazel build" + labels;
  }
  return message;
}

}  // namespace

std::optional<naming::Diagnostic> MissingBinariesFinding(
    const std::vector<NodeSpec>& missing, const std::string& bundle_dir) {
  if (missing.empty()) return std::nullopt;
  std::string names;
  bool fatal = false;
  for (const auto& spec : missing) {
    names += (names.empty() ? "" : ", ") + spec.name;
    fatal = fatal || spec.critical || spec.owns_actuators;
  }
  std::string message = MissingBinariesMessage(missing, bundle_dir);
  if (!fatal) {
    message += "; NON-CRITICAL: launching the rest under restart supervision";
  }
  return naming::Diagnostic{
      fatal ? naming::Severity::ERROR : naming::Severity::WARNING,
      std::move(names), std::move(message)};
}

naming::Diagnostic StudioBridgeFinding(const NodeSpec& spec, bool missing,
                                       const std::string& error,
                                       const std::string& bundle_dir) {
  return {naming::Severity::WARNING, spec.name,
          (missing ? MissingBinariesMessage({spec}, bundle_dir)
                   : "did not answer --describe (" + error + ")") +
              "; the session runs without Studio"};
}

std::string EndpointMismatch(const std::vector<naming::SourceShape>& declared,
                             const std::vector<naming::SourceShape>& live,
                             bool live_complete) {
  using Key =
      std::tuple<std::string, int, std::uint32_t, std::uint32_t, std::uint32_t>;
  constexpr std::uint32_t kLinted = naming::kSourceFlagExternal |
                                    naming::kSourceFlagOptional |
                                    naming::kSourceFlagObserver;
  const auto keys = [](const std::vector<naming::SourceShape>& sources) {
    std::vector<Key> out;
    for (const auto& source : sources) {
      if (source.kind == event::SourceKind::TIMER) continue;
      out.emplace_back(source.name, static_cast<int>(source.kind),
                       source.message_bytes, source.alignment,
                       source.flags & kLinted);
    }
    std::sort(out.begin(), out.end());
    return out;
  };
  const auto text = [](const std::vector<Key>& ends) {
    std::string out;
    for (const auto& [name, kind, bytes, alignment, flags] : ends) {
      if (!out.empty()) out += ", ";
      out += event::to_string(static_cast<event::SourceKind>(kind));
      out += " " + name + " (" + std::to_string(bytes) + " B, align " +
             std::to_string(alignment);
      if ((flags & naming::kSourceFlagExternal) != 0) out += ", external";
      if ((flags & naming::kSourceFlagOptional) != 0) out += ", optional";
      if ((flags & naming::kSourceFlagObserver) != 0) out += ", observer";
      out += ")";
    }
    return out;
  };
  const auto described = keys(declared);
  const auto running = keys(live);
  std::vector<Key> live_only;
  std::vector<Key> described_only;
  std::set_difference(running.begin(), running.end(), described.begin(),
                      described.end(), std::back_inserter(live_only));
  if (live_complete) {
    std::set_difference(described.begin(), described.end(), running.begin(),
                        running.end(), std::back_inserter(described_only));
  }
  std::string out;
  if (!live_only.empty()) out = "live only: " + text(live_only);
  if (!described_only.empty()) {
    out += (out.empty() ? "" : "; ") + std::string{"described only: "} +
           text(described_only);
  }
  return out;
}

std::size_t CountDeclaredTopics(const std::vector<NodeDescription>& described) {
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

std::vector<RTMSQueue> CreateSessionSegments(
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

std::string GraphToJson(uint64_t session_id, const std::string& config_path,
                        const std::vector<NodeDescription>& described,
                        const std::vector<naming::Diagnostic>& diagnostics,
                        const std::string& config_sha256) {
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

}  // namespace talos::launcher
