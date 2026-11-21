#include "talOS/launcher/launcher.h"

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include "talOS/events/context.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/os/poller.h"
#include "talOS/introspection/lock_file.h"
#include "talOS/introspection/registry.h"
#include "talOS/launcher/console_line.h"
#include "talOS/launcher/hardware_record.h"
#include "talOS/launcher/log_budget.h"
#include "talOS/launcher/session_lock.h"
#include "talOS/memory/shared_memory_ptr.h"
#include "talOS/process/process.h"
#include "talOS/rtms/rtms.h"
#include "toml.hpp"

namespace talos::launcher {
namespace {

// How often Supervise checks whether every non-observer member's loop has
// started while waiting to write kSimScheduleStartFile. Stops once written or
// once sim_gateway exits with no pending restart.
constexpr std::chrono::milliseconds kSimScheduleStartPoll{20};

// The topic ends a registry row published, or nullopt while it has
// published none: a row is visible from its claim, and a writer that never
// sets kFlagConstructing (the hardware bridge, the simulation gateway)
// publishes its sources just after. Sources are written once per claim,
// before the count (NodeRegistration::publish), so the acquire below sees
// them whole. `complete` is false when the row could not hold them all.
std::optional<std::vector<naming::SourceShape>> PublishedEndpoints(
    const introspect::NodeRecord& record, bool& complete) {
  const uint32_t count = record.source_count.load(std::memory_order_acquire);
  if (count == 0) return std::nullopt;
  const auto held = std::min<uint32_t>(
      count, static_cast<uint32_t>(introspect::kMaxSourcesPerNode));
  complete =
      record.declared_source_count.load(std::memory_order_relaxed) <= held;
  std::vector<naming::SourceShape> out;
  out.reserve(held);
  for (uint32_t s = 0; s < held; ++s) {
    const introspect::SourceRecord& source = record.sources[s];
    naming::SourceShape& shape = out.emplace_back();
    shape.kind = static_cast<event::SourceKind>(source.kind);
    shape.name = introspect::read_name(source.name, introspect::kMaxNameBytes);
    shape.message_bytes = source.message_bytes;
    shape.alignment = source.alignment;
    shape.flags = source.flags;
  }
  return out;
}

// What a row says its run lost (introspect::DeliveryLossOf), with the
// bridge's skipped requests split per subsystem, read from its request
// sources only when DeliveryLossOf found any.
DeliveryCounts DeliveryCountsOf(const introspect::NodeRecord& record) {
  const introspect::DeliveryLoss loss = introspect::DeliveryLossOf(record);
  DeliveryCounts counts{.wake_failures = loss.wake_failures,
                        .stalled_arms = loss.stalled_arms,
                        .wake_fallbacks = loss.wake_fallbacks,
                        .requests_skipped = {}};
  if (loss.requests_skipped == 0) return counts;
  const auto held =
      std::min<uint32_t>(record.source_count.load(std::memory_order_acquire),
                         static_cast<uint32_t>(introspect::kMaxSourcesPerNode));
  for (uint32_t s = 0; s < held; ++s) {
    const introspect::SourceRecord& source = record.sources[s];
    std::string name =
        introspect::read_name(source.name, introspect::kMaxNameBytes);
    const uint64_t dropped = source.dropped.load(std::memory_order_relaxed);
    if (dropped == 0 || !introspect::IsRequestRead(
                            static_cast<event::SourceKind>(source.kind), name))
      continue;
    // `/hw/request/<subsystem>`, as the bridge reads it.
    const std::string_view prefix{hardware::kRequestTopicPrefix};
    if (name.starts_with(prefix)) name.erase(0, prefix.size());
    counts.requests_skipped.emplace_back(std::move(name), dropped);
  }
  return counts;
}

// What a reaped run's row says it lost as the run ended, and its `flags`
// (the event log flags among them, set into `flags`). A run that exits
// takes a last sample and then releases its row, which keeps that sample
// until the next claim (NodeRegistration::release); a killed run leaves its
// row as its last sample left it. The supervision pass reads rows only
// while the session runs (WatchRun), so this read, once per reaped run, is
// what carries the counts a run gained while the session stopped (TODO.md
// RT-7). A row is the run's only while it carries the run's pid and
// session and a heartbeat at or after `spawned_ns`, the run's spawn on the
// host monotonic clock: a claim stores a heartbeat, so a row the run
// claimed has one, and a row an earlier run of a reused pid left behind
// does not, even when the run itself claimed none. Of those, the newest
// heartbeat wins; a row claimed again during the read is passed over. A
// released row another process claims before the reap is no longer the
// run's, and the run's counts are then what the supervision pass last read;
// a claim takes a just-released row only when every other row is held or
// was released after it (NodeRegistration::claim; a restart storm).
// Loads from the mapped segment only: no lock, no disk.
std::optional<DeliveryCounts> FinalDeliveryCounts(
    const introspect::RegistryMapping& registry, pid_t pid, uint64_t session_id,
    int64_t spawned_ns, uint32_t& flags) {
  const auto claiming = static_cast<uint64_t>(introspect::SlotState::CLAIMING);
  std::optional<DeliveryCounts> counts;
  int64_t newest = 0;
  for (const auto& record : registry.header()->nodes) {
    const auto generation = record.generation.load(std::memory_order_acquire);
    if (record.state.load(std::memory_order_acquire) == claiming ||
        record.pid != static_cast<uint64_t>(pid) ||
        record.session_id != session_id)
      continue;
    const auto beat =
        record.heartbeat_monotonic_ns.load(std::memory_order_relaxed);
    if (beat < spawned_ns || (counts && beat <= newest)) continue;
    DeliveryCounts read = DeliveryCountsOf(record);
    const uint32_t read_flags = record.flags.load(std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_acquire);
    if (record.generation.load(std::memory_order_relaxed) != generation ||
        record.state.load(std::memory_order_relaxed) == claiming)
      continue;
    counts = std::move(read);
    flags = read_flags;
    newest = beat;
  }
  return counts;
}

// Phase one's report, warnings included.
void PrintDeclaredGraph(const std::vector<NodeDescription>& described,
                        const std::vector<naming::Diagnostic>& diagnostics,
                        std::size_t declared_topics,
                        const std::string& graph_path) {
  std::size_t answered = 0;
  for (const auto& node : described) answered += node.answered ? 1 : 0;

  ConsoleLine(std::cout)
      << "\n================ Declared Graph ==================\n";
  ConsoleLine(std::cout) << "Nodes described: " << answered << "/"
                         << described.size() << ", topics: " << declared_topics
                         << "\n";
  for (const auto& node : described) {
    ConsoleLine line{std::cout};
    line << "  " << (node.answered ? "\u2022" : "?") << " " << node.roster_name;
    if (node.answered) {
      line << " (" << node.declared.sources.size() << " sources)\n";
    } else {
      line << (node.missing ? " (not built)\n" : " (not described)\n");
    }
  }
  for (const auto& diagnostic : diagnostics) {
    auto& out =
        diagnostic.severity == naming::Severity::ERROR ? std::cerr : std::cout;
    ConsoleLine(out) << "  " << diagnostic.ToString() << "\n";
  }
  ConsoleLine(std::cout) << "Graph written to: " << graph_path << "\n";
  ConsoleLine(std::cout)
      << "==================================================\n";
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

  ConsoleLine(std::cout)
      << "\n================ Session Manifest ================\n";
  ConsoleLine(std::cout) << "Session ID: " << manifest.session_id << "\n";
  ConsoleLine(std::cout) << "Output Dir: " << manifest.output_dir << "\n";
  ConsoleLine(std::cout) << "Declared: " << described << "/"
                         << manifest.nodes.size()
                         << " nodes answered --describe, "
                         << manifest.declared_topics << " topics\n";
  ConsoleLine(std::cout) << "Declared nodes: "
                         << (roster.empty() ? "none" : roster) << "\n";
  ConsoleLine(std::cout) << "Nodes (" << manifest.nodes.size() << "):\n";
  for (const auto& n : manifest.nodes) {
    ConsoleLine(std::cout) << "  \u2022 " << n.name << " (pid=" << n.pid
                           << ", exit=" << n.exit_code
                           << (n.exit_signal != 0
                                   ? " (signal " + SignalText(n.exit_signal) +
                                         ")"
                                   : std::string{})
                           << ", records=" << n.record_count
                           << ", restarts=" << n.restart_count
                           << ", failed=" << (n.failed ? "yes" : "no")
                           << ", log=" << n.log_path << ")\n";
  }
  ConsoleLine(std::cout) << "Manifest written to: " << manifest.output_dir
                         << "/manifest.json\n";
  ConsoleLine(std::cout)
      << "==================================================\n";
}

// Each logging member's share of the session's log budget, in proportion to
// the bytes its log is expected to store per second, and at least what
// rotates (log_budget.h), and the window of record time every member's log
// keeps; its restarts draw on what its earlier runs left (Launcher::Restart).
LogSplit LogBudgets(const std::vector<NodeProcessInfo>& procs,
                    const std::vector<NodeDescription>& described,
                    const config::RobotConfig& robot_config,
                    uint64_t session_log_budget) {
  const TopicLoads loads = DeclaredTopicLoads(described, robot_config);
  std::vector<bool> logs(procs.size());
  std::vector<double> rates(procs.size(), -1);
  std::vector<uint64_t> floors(procs.size(), 0);
  for (std::size_t i = 0; i < procs.size(); ++i) {
    logs[i] = !procs[i].log_path.empty();
    if (!described[i].answered) continue;
    rates[i] = ExpectedLogRate(described[i].declared,
                               MemberPeriodUs(robot_config, procs[i].name),
                               procs[i].name == kHardwareNodeName, loads);
    floors[i] = SmallestRotatingShare(described[i].declared);
  }
  return SplitLogBudget(session_log_budget, logs, rates, floors);
}

// The one line each gapped run's log gets (NodeProcessInfo::RecordLogGap).
// It says no more than that records were lost: a stall that lasts until the
// stop leaves nothing recorded after the gap, and a run killed during one
// leaves no GAP record.
void PrintLogGap(const std::string& member, const std::string& log) {
  ConsoleLine(std::cerr) << "[launcher] " << member << " event log " << log
                         << " lost records (a gap)\n";
}

// What a launch that started the roster in a shared-memory namespace
// (TALOS_SHM_NAMESPACE, Bazel's TEST_TMPDIR) leaves of its own besides its
// topic segments, removed when RunSession returns: the registry segment and
// its writer lock file, once no row of the registry is held by a running
// process of another session, and the session lock file. A namespace is
// rarely used twice, so each would otherwise stay until reboot: the segment
// in shared memory, the lock files in introspect::LockDirectory() (but for
// Bazel's TEST_TMPDIR, removed with the test). Not done outside a
// namespace: there the ordinary names are one set per host, which a node or
// tool run without a launcher may share. Skipped while an exception unwinds,
// when members may still run.
class NamespaceNames {
 public:
  NamespaceNames(const std::optional<SessionLock>& session_lock,
                 uint64_t session_id)
      : session_lock_{session_lock}, session_id_{session_id} {}
  NamespaceNames(const NamespaceNames&) = delete;
  NamespaceNames& operator=(const NamespaceNames&) = delete;

  // The registry is removed only once the launch has prepared it.
  void RegistryPrepared() { registry_prepared_ = true; }

  ~NamespaceNames() {
    if (std::uncaught_exceptions() > 0 || !session_lock_) return;
    const std::string registry = ShmObjectName(introspect::kRegistryPath);
    if (!registry.starts_with("/~")) return;
    if (registry_prepared_)
      static_cast<void>(introspect::RemoveRegistryUnlessHeld(session_id_));
    // Still held, so no other launch's: removed before the lock is released
    // (SessionLock's destructor, after this one). Not when the path names
    // another file (Refresh false): that may be another launch's.
    if (session_lock_->Refresh()) ::unlink(session_lock_->path().c_str());
  }

 private:
  const std::optional<SessionLock>& session_lock_;
  uint64_t session_id_;
  bool registry_prepared_{false};
};

// Totals every run's event log into `procs` once the session has ended --
// record counts, capped, gapped and damaged logs -- and returns the session's
// exit status before the endpoint refusal: 0, or the last nonzero exit code of
// a member whose exit counts (`exit_not_counted`, one per member, false: not
// the Studio bridge, and its last run did not end by the stop signal the
// launcher sent it, NodeRun::stopped_by_session).
int SummarizeLogs(std::vector<NodeProcessInfo>& procs,
                  const std::vector<bool>& exit_not_counted) {
  int exit_status = 0;
  for (std::size_t i = 0; i < procs.size(); ++i) {
    auto& proc = procs[i];
    // Every run's log: a restarted node's records are the sum of its runs.
    for (const auto& path : proc.log_paths) {
      if (std::filesystem::exists(event::log::CappedMarkerPath(path)))
        proc.log_capped = true;
      bool gapped = false;
      // The segments a capped log kept, then the newest (LogSegmentFiles).
      for (const auto& file : event::log::LogSegmentFiles(path)) {
        // Streamed: shutdown memory does not grow with the logs' size.
        try {
          const auto summary = event::log::SummarizeLog(file);
          proc.record_count += summary.record_count;
          if (!summary.damage.clean()) proc.RecordRecorderFailure(path);
          if (summary.gaps > 0) gapped = true;
        } catch (const event::log::IncompleteLogHeader& e) {
          // Cut while it was being created: no records to count.
          ConsoleLine(std::cerr)
              << "[launcher] skipping " << file << ": " << e.what() << "\n";
        } catch (...) {
          proc.RecordRecorderFailure(path);
        }
      }
      // A gap the live row did not show (a reporter that never sampled it)
      // is named here, once per run's log.
      if (gapped && proc.RecordLogGap(path)) PrintLogGap(proc.name, path);
    }
    if (proc.exit_code != 0 && !exit_not_counted[i])
      exit_status = proc.exit_code;
  }
  return exit_status;
}

}  // namespace

std::string RegistryCapacityError(const introspect::RegistryReader& reader,
                                  std::size_t registering) {
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

bool ResolveMaxRestarts(const config::RobotConfig& robot_config,
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

std::string_view RunEndReason(std::string_view kill_reason, bool stopping,
                              int status) {
  const bool clean = WIFEXITED(status) && WEXITSTATUS(status) == 0;
  const int signal = WIFSIGNALED(status) ? WTERMSIG(status) : 0;
  if (kill_reason.empty() && !stopping) return clean ? kRunExit : kRunCrash;
  if (!kill_reason.empty() && kill_reason != kRunStop &&
      kill_reason != kRunStopTimeout)
    return kill_reason;
  if (kill_reason == kRunStopTimeout && signal == SIGKILL)
    return kRunStopTimeout;
  return clean || signal == SIGINT || signal == SIGTERM ? kRunStop : kRunCrash;
}

bool EndedByOwnStopSignal(std::string_view kill_reason, int status) {
  return (kill_reason == kRunStop || kill_reason == kRunStopTimeout) &&
         WIFSIGNALED(status) && WTERMSIG(status) == kStopSignal;
}

bool CappedLogStopped(const std::string& log_path) {
  // The kept segments and, unless it is mid-rotation, `log_path` itself.
  return event::log::LogSegmentFiles(log_path).size() < 2;
}

Launcher::Launcher(LauncherOptions options) : options_{std::move(options)} {
  if (options_.session_id == 0) options_.session_id = GenerateSessionId();
  const std::filesystem::path root =
      options_.output_dir.empty() ? "/tmp/talos_logs" : options_.output_dir;
  log_root_ = root.string();
  options_.output_dir = (root / std::to_string(options_.session_id)).string();
}

int Launcher::RefuseLaunch(std::string_view reason,
                           const config::RobotConfig* robot_config,
                           StopSignals& stop_signals) {
  // Without owning `session_lock_`, another session may already be live
  // (possibly with --no-studio or a custom HTTP port) or already serving a
  // real refusal: never overwrite <log_root>/last_refusal.json or bind the
  // Studio HTTP port unless this launcher holds the session lock.
  if (options_.describe_only || !options_.replay_inputs.empty() ||
      session_lock_ == nullptr) {
    return 1;
  }

  const std::string refusal_path =
      (std::filesystem::path{log_root_} / kLastRefusalFile).string();
  std::string json = "{\n  \"refused\": true,\n  \"reason\": ";
  json += Quoted(reason);
  json += ",\n  \"config_path\": ";
  json += Quoted(options_.config_path);
  json += ",\n  \"wall_ns\": ";
  json += std::to_string(WallClockNanos());
  json += "\n}\n";

  std::error_code dir_error;
  std::filesystem::create_directories(log_root_, dir_error);
  std::string write_error;
  const bool refusal_written =
      !dir_error &&
      WriteFileAtomically(refusal_path, json, options_.sync_file, write_error);
  if (!refusal_written) {
    ConsoleLine(std::cerr) << "[launcher] cannot write " << refusal_path << " ("
                           << (dir_error ? dir_error.message() : write_error)
                           << ")\n";
  }

  if (!options_.serve_refusal || !options_.start_studio_bridge ||
      BundleWithoutStudio(options_)) {
    return 1;
  }

  const std::string target = robot_config != nullptr
                                 ? robot_config->studio.bridge
                                 : std::string{"//studio/bridge:studio_bridge"};
  const NodeSpec spec =
      MakeNodeSpec(kStudioBridgeName, target, options_, /*studio_bridge=*/true);
  if (spec.binary_path.empty() ||
      ::access(spec.binary_path.c_str(), X_OK) != 0) {
    ConsoleLine(std::cerr)
        << "[launcher] cannot serve refusal: studio_bridge not executable at '"
        << (spec.binary_path.empty() ? target : spec.binary_path) << "'\n";
    return 1;
  }

  const decltype(config::RobotConfig::studio) defaults{};
  const uint16_t http_port = options_.studio_http_port.value_or(
      robot_config != nullptr ? robot_config->studio.http_port
                              : defaults.http_port);
  const std::string webroot =
      robot_config != nullptr ? EffectiveStudioWebroot(*robot_config, options_)
                              : (options_.bundle_has_studio &&
                                         !options_.bundle_studio_webroot.empty()
                                     ? options_.bundle_studio_webroot
                                     : defaults.webroot);
  const std::string topic =
      robot_config != nullptr ? robot_config->studio.topic : defaults.topic;

  std::vector<std::string> args{spec.binary_path,          topic,
                                std::to_string(http_port), webroot,
                                "--refusal-file",          refusal_path};
  if (!refusal_written) {
    args.emplace_back("--refusal-reason");
    args.emplace_back(reason);
  }

  stop_signals.Install();
  if (process::stop_requested.load()) return 1;

  SupervisionWakeup wakeup;
  int stdout_fd = -1;
  std::string spawn_error;
  const pid_t pid = SpawnProcess(std::move(args), spawn_error,
                                 SpawnOutput::kStdoutPipe, &stdout_fd);
  if (pid < 0) {
    ConsoleLine(std::cerr) << "[launcher] cannot spawn refusal studio_bridge ("
                           << spawn_error << ")\n";
    return 1;
  }
  if (stdout_fd >= 0) {
    const int flags = ::fcntl(stdout_fd, F_GETFL, 0);
    if (flags >= 0) ::fcntl(stdout_fd, F_SETFL, flags | O_NONBLOCK);
  }

  ServiceNotifier notifier;
  const auto watchdog = notifier.watchdog_interval();
  const auto start_time = options_.supervision_clock();
  SupervisionTime next_watchdog = SupervisionTime::max();
  const SupervisionTime deadline =
      options_.duration_s > 0
          ? start_time + std::chrono::seconds(options_.duration_s)
          : SupervisionTime::max();

  bool stop_sent = false;
  bool ready_sent = false;
  SupervisionTime shutdown_deadline = SupervisionTime::max();
  bool port_announced = false;
  std::string stdout_buf;

  const auto drain_stdout = [&] {
    if (stdout_fd < 0) return;
    char chunk[256];
    for (;;) {
      const ssize_t n = ::read(stdout_fd, chunk, sizeof(chunk));
      if (n > 0) {
        if (!port_announced && stdout_buf.size() < 4096) {
          stdout_buf.append(chunk, static_cast<std::size_t>(n));
        }
        continue;
      }
      if (n == 0) {
        ::close(stdout_fd);
        stdout_fd = -1;
      }
      break;
    }
    if (!port_announced) {
      std::size_t pos = 0;
      for (std::size_t nl = stdout_buf.find('\n', pos); nl != std::string::npos;
           nl = stdout_buf.find('\n', pos)) {
        std::string_view line(stdout_buf.data() + pos, nl - pos);
        if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
        constexpr std::string_view kPrefix = "listening ";
        if (line.starts_with(kPrefix)) {
          const std::string_view value = line.substr(kPrefix.size());
          int bound = 0;
          const auto* end = value.data() + value.size();
          const auto [ptr, ec] = std::from_chars(value.data(), end, bound);
          if (!value.empty() && ec == std::errc{} && ptr == end && bound >= 1 &&
              bound <= 65535) {
            port_announced = true;
            ConsoleLine(std::cout)
                << "[launcher] Launch refused; serving refusal on "
                   "http://localhost:"
                << bound << "/drive\n";
            pos = nl + 1;
            break;
          }
        }
        pos = nl + 1;
      }
      if (pos > 0) stdout_buf.erase(0, pos);
    }
  };

  for (;;) {
    drain_stdout();
    int status = 0;
    const pid_t reaped = ::waitpid(pid, &status, WNOHANG);
    if (reaped == pid) {
      FinishProcessGroup(pid);
      drain_stdout();
      if (stdout_fd >= 0) {
        ::close(stdout_fd);
        stdout_fd = -1;
      }
      if (!stop_sent) {
        ConsoleLine(std::cerr) << "[launcher] refusal studio_bridge exited ("
                               << ExitReason(status) << ")\n";
      }
      break;
    }
    const SupervisionTime now = options_.supervision_clock();
    // Notify systemd READY=1 only once the bridge is alive and has announced
    // its listening port, so an immediate EADDRINUSE exit never falsely
    // reports READY=1 before returning 1.
    if (port_announced && !ready_sent) {
      notifier.Send("READY=1");
      ready_sent = true;
      if (watchdog) next_watchdog = now + *watchdog;
    }
    if (ready_sent && watchdog && now >= next_watchdog) {
      notifier.Send("WATCHDOG=1");
      next_watchdog = now + *watchdog;
    }
    if (!stop_sent && (process::stop_requested.load() || now >= deadline)) {
      stop_sent = true;
      shutdown_deadline = now + kStopTimeout;
      SignalProcessGroup(pid, kStopSignal);
    }
    if (stop_sent && now >= shutdown_deadline) {
      SignalProcessGroup(pid, SIGKILL);
    }
    auto wake = SupervisionTime::max();
    const auto due = [&wake](SupervisionTime at) { wake = std::min(wake, at); };
    if (ready_sent && watchdog) due(next_watchdog);
    if (stop_sent) {
      if (now < shutdown_deadline) due(shutdown_deadline);
    } else {
      due(deadline);
      if (!port_announced && stdout_fd >= 0) {
        due(now + std::chrono::milliseconds{10});
      }
    }
    wakeup.Wait(wake == SupervisionTime::max()
                    ? std::chrono::nanoseconds::max()
                    : wake - options_.supervision_clock());
  }
  return 1;
}

int Launcher::Run() {
  // Declared first, so destroyed last: when RunSession returns, a roster
  // launch's stop handlers give way to the dispositions the launcher's
  // caller set only after it has returned, which is after Supervise has
  // reaped every member's run and killed its group (or no member was
  // spawned, and DescribeGraph reaped every probe), and after RunSession's
  // locals have unlinked the session's topic segments and released the
  // session lock; and after the output drains have stopped and the session
  // directory's lock is released, below. Until then a stop is the
  // launcher's orderly one: none is lost, and none takes a default action
  // (ending the launcher) while a member, a probe or a segment of the
  // session exists. Afterwards there is nothing left for it to stop. An
  // exception that escapes RunSession once members are spawned puts the
  // dispositions back while those members still run: main.cc then returns
  // 1, and each member's guardian kills its group as the launcher exits.
  StopSignals stop_signals;
  // A session must not end because whoever reads the launcher's stdout or
  // stderr went away (a `tee` or a log pipe closing): its writes then fail
  // with EPIPE, which LauncherOutput survives (REQ-OPS-33). Children start
  // with the default again (SpawnProcess). Unlike the stop signals it is
  // never put back: LauncherOutput::Stop may leave a writer thread blocked
  // in a write to a stdout or stderr nobody reads (it detaches it after its
  // grace), and that write failing with EPIPE after Run returns must not
  // end the process.
  std::signal(SIGPIPE, SIG_IGN);
  // From here no launcher thread writes to descriptors 1 and 2 itself.
  try {
    launcher_output_.emplace();
  } catch (const std::exception& e) {
    ConsoleLine(std::cerr) << "launcher: " << e.what()
                           << "; nothing was started\n";
    return 1;
  }
  const int status = RunSession(stop_signals);
  // Every member has been reaped, so its pipe has ended; then the
  // launcher's own lines get the descriptors' linger.
  if (output_) output_->Stop();
  output_.reset();
  launcher_output_.reset();
  session_directory_.reset();
  return status;
}

int Launcher::RunSession(StopSignals& stop_signals) {
  session_lock_ = nullptr;
  // The flag overrides every graph refusal, the safety ones included
  // (REQ-SAFE-27/28) and the live-endpoint refusal, so it never reaches a
  // real controller: --sim requires the loopback remote (below). Refused
  // before the config is read, so nothing is probed, written or spawned.
  if (options_.allow_graph_errors && !options_.simulation) {
    constexpr std::string_view kMessage =
        "--allow-graph-errors is accepted only with --sim; nothing was "
        "described or started";
    ConsoleLine(std::cerr) << "launcher: " << kMessage << "\n";
    return RefuseLaunch(kMessage, nullptr, stop_signals);
  }
  // Alone it would start the roster, which whoever passed it meant to check.
  if (options_.describe_as_launch && !options_.describe_only) {
    constexpr std::string_view kMessage =
        "--describe-as-launch requires --describe-only; nothing was described "
        "or started";
    ConsoleLine(std::cerr) << "launcher: " << kMessage << "\n";
    return RefuseLaunch(kMessage, nullptr, stop_signals);
  }
  if (!options_.sim_gateway_args.empty() && !options_.start_sim_gateway) {
    constexpr std::string_view kMessage =
        "--sim-gateway-arg requires --start-sim-gateway";
    ConsoleLine(std::cerr) << "launcher: " << kMessage << "\n";
    return RefuseLaunch(kMessage, nullptr, stop_signals);
  }
  if (options_.allow_capped_replay && options_.replay_inputs.empty()) {
    constexpr std::string_view kMessage =
        "--allow-capped-replay requires --replay-inputs SESSION_DIR";
    ConsoleLine(std::cerr) << "launcher: " << kMessage << "\n";
    return RefuseLaunch(kMessage, nullptr, stop_signals);
  }
  // The launcher gives the simulator the bridge's ports (SimulatorArgs); a
  // port written by hand as well could only disagree with the bridge's.
  for (const std::string& arg : options_.sim_gateway_args) {
    if (IsSimulatorPortArg(arg)) {
      const std::string message =
          "--sim-gateway-arg " + arg +
          " is refused: the launcher passes the simulator --port "
          "and --remote-port from [robot] controller_port and "
          "local_port, or from its --remote-port and --local-port; "
          "set those instead";
      ConsoleLine(std::cerr) << "launcher: " << message << "\n";
      return RefuseLaunch(message, nullptr, stop_signals);
    }
  }
  std::string bundle_error;
  if (!LoadBundle(options_, bundle_error)) {
    ConsoleLine(std::cerr) << "launcher: " << bundle_error << "\n";
    return RefuseLaunch(bundle_error, nullptr, stop_signals);
  }
  if (options_.config_path.empty()) {
    constexpr std::string_view kMessage =
        "--config PATH or --bundle PATH is required";
    ConsoleLine(std::cerr) << "launcher: " << kMessage << "\n";
    return RefuseLaunch(kMessage, nullptr, stop_signals);
  }
  // Recorded absolute (manifest.json, graph.json, every member's --config):
  // a relative path names nothing once the launcher's working directory is
  // forgotten.
  const bool config_was_relative =
      std::filesystem::path{options_.config_path}.is_relative();
  {
    std::error_code cwd_error;
    const auto absolute =
        std::filesystem::absolute(options_.config_path, cwd_error);
    if (!cwd_error) options_.config_path = absolute.lexically_normal().string();
  }
  if (options_.replay_inputs.empty() != options_.only.empty()) {
    ConsoleLine(std::cerr)
        << "launcher: --replay-inputs SESSION_DIR and --only NODE go "
           "together\n";
    return 1;
  }
  // A run that starts the roster owns the namespace's topics and registry
  // for its lifetime. --describe-only probes, and --replay-inputs runs one
  // node on recorded topics, without shared memory (RunOnRecordedInputs).
  const bool starts_roster =
      !options_.describe_only && options_.replay_inputs.empty();
  // Before the first thing such a launch creates that outlives it (the
  // session lock, its session directory, the probes, the topic segments,
  // the manifest): from here a stop request is an orderly stop at whatever
  // step it finds the launch (stopped_before_start below).
  if (starts_roster) stop_signals.Install();
  // The launch stops before its roster starts: only what a launch that
  // recorded nothing holds exists yet (README "Session segments and logs"),
  // and the stop's exit status is 0, as for any requested stop.
  const auto stopped_before_start =
      [&] {
        if (!starts_roster || !process::stop_requested.load()) return false;
        ConsoleLine(std::cout)
            << "[launcher] stop requested before the session started; "
               "nothing was started\n";
        return true;
      };
  session_lock_ = nullptr;
  std::optional<SessionLock> session_lock;
  // Destroyed before the lock, after the topic segments (declared later).
  NamespaceNames namespace_names{session_lock, options_.session_id};
  // The lock file is absent and the log root's full filesystem has no room
  // to create it (SessionLock::Acquire, LockSharesLogFilesystem): retention,
  // which may free that room, runs without the lock (Pruning::kUnlocked),
  // and the lock is taken after it.
  bool lock_needs_room = false;
  std::string lock_error;
  // False when the launch is refused: the lock is neither held nor waiting
  // for room retention can free.
  const auto take_lock = [&] {
    bool no_room = false;
    session_lock = options_.acquire_session_lock(lock_error, &no_room);
    session_lock_ = session_lock ? &*session_lock : nullptr;
    lock_needs_room = !session_lock && no_room && LockSharesLogFilesystem();
    return session_lock.has_value() || lock_needs_room;
  };
  if (starts_roster) {
    if (!take_lock()) {
      ConsoleLine(std::cerr) << "launcher: " << lock_error << "\n";
      return RefuseLaunch(lock_error, nullptr, stop_signals);
    }
    if (lock_needs_room) {
      ConsoleLine(std::cerr)
          << "[launcher] " << lock_error
          << "; retention runs first, then the lock is taken\n";
    }
  }
  // Nothing is written to the log root until retention has run: on a full
  // log disk it is what makes room for the new session's directory.

  config::RobotConfig robot_config;
  try {
    robot_config = config::ParseRobotConfig(options_.config_path);
  } catch (const std::exception& e) {
    const std::string message =
        "failed to load config from " + options_.config_path + ": " + e.what();
    ConsoleLine(std::cerr) << "launcher: " << message << "\n";
    if (config_was_relative) {
      ConsoleLine(std::cerr)
          << "launcher: working directory is "
          << std::filesystem::current_path()
          << "; run from the repository root or pass an absolute "
             "--config path\n";
    }
    return RefuseLaunch(message, nullptr, stop_signals);
  }
  // A simulation config (its gains, bindings and travel) never drives real
  // hardware. --describe-only and --replay-inputs touch none, but
  // --describe-as-launch asks --describe-only for the launch's verdict.
  // Of the launch's other refusals before its probes, this is the only one
  // that needs no lock, shared memory, socket or process: the session lock
  // (held by the session a check runs beside) and the registry's
  // preparation and capacity stay the launch's alone.
  if (robot_config.simulation_only && !options_.simulation &&
      (starts_roster || options_.describe_as_launch)) {
    const std::string message =
        options_.config_path + " sets [robot] simulation_only = true; " +
        (options_.describe_as_launch
             ? "a launch of it without --sim is refused, so it cannot be "
               "installed as the robot's; nothing was started"
             : "run it with --sim (or --describe-only); nothing was started");
    ConsoleLine(std::cerr) << "launcher: " << message << "\n";
    return RefuseLaunch(message, &robot_config, stop_signals);
  }
  std::string effective_remote;
  try {
    effective_remote = EffectiveRemoteAddress(robot_config, options_);
  } catch (const std::exception& e) {
    ConsoleLine(std::cerr) << "launcher: " << e.what() << "\n";
    return RefuseLaunch(e.what(), &robot_config, stop_signals);
  }
  if (options_.simulation && effective_remote != "127.0.0.1") {
    const std::string message =
        "--sim requires remote 127.0.0.1; effective remote " + effective_remote;
    ConsoleLine(std::cerr) << "launcher: " << message << "\n";
    return RefuseLaunch(message, &robot_config, stop_signals);
  }
  std::string effective_local_address;
  try {
    effective_local_address = EffectiveLocalAddress(robot_config, options_);
  } catch (const std::exception& e) {
    ConsoleLine(std::cerr) << "launcher: " << e.what() << "\n";
    return RefuseLaunch(e.what(), &robot_config, stop_signals);
  }
  if (options_.simulation && !effective_local_address.empty() &&
      effective_local_address != "127.0.0.1") {
    const std::string message =
        "--sim requires local_address unset or 127.0.0.1; effective local "
        "address " +
        effective_local_address;
    ConsoleLine(std::cerr) << "launcher: " << message << "\n";
    return RefuseLaunch(message, &robot_config, stop_signals);
  }
  std::string restart_error;
  int max_restarts = kDefaultMaxRestarts;
  if (!ResolveMaxRestarts(robot_config, options_, max_restarts,
                          restart_error)) {
    ConsoleLine(std::cerr) << "launcher: " << restart_error << "\n";
    return RefuseLaunch(restart_error, &robot_config, stop_signals);
  }
  if (options_.restart_window_ms <= 0 || options_.heartbeat_periods <= 0 ||
      options_.heartbeat_startup_grace_ms < 0) {
    constexpr std::string_view kMessage =
        "restart window and heartbeat periods must be positive; startup grace "
        "must be nonnegative";
    std::cerr << "launcher: " << kMessage << "\n";
    return RefuseLaunch(kMessage, &robot_config, stop_signals);
  }
  restart_limits_ = {max_restarts,
                     std::chrono::milliseconds{options_.restart_window_ms}};

  auto node_specs = DiscoverNodes(robot_config, options_);
  if (options_.start_studio_bridge && BundleWithoutStudio(options_)) {
    ConsoleLine(std::cerr)
        << "[launcher] this bundle carries no Studio (its talos_bundle "
           "sets no studio_bridge and studio_webroot); session runs "
           "without Studio\n";
  }
  if (node_specs.empty()) {
    constexpr std::string_view kMessage = "no nodes declared in config";
    ConsoleLine(std::cerr) << "launcher: " << kMessage << "\n";
    return RefuseLaunch(kMessage, &robot_config, stop_signals);
  }
  // A member's name is its file names (<name>.log, <name>.tlog) and its
  // registry row. Subsystem keys are unique, so two members of one name are
  // a subsystem and a member the launcher adds (the hardware bridge, the
  // sim gateway, the Studio bridge): refused, naming the key, since either
  // would take the other's files and the launcher's own would be taken for
  // the robot's node, or the reverse.
  for (std::size_t i = 0; i < node_specs.size(); ++i) {
    for (std::size_t j = i + 1; j < node_specs.size(); ++j) {
      if (node_specs[i].name != node_specs[j].name) continue;
      const NodeSpec& added =
          node_specs[i].studio_bridge ? node_specs[i] : node_specs[j];
      const std::string message =
          "[subsystems." + node_specs[i].name +
          "] has the name of the launcher's own " +
          (added.studio_bridge
               ? "Studio bridge, which --start-studio-bridge adds"
               : "roster member of that name") +
          "; rename the subsystem";
      ConsoleLine(std::cerr) << "launcher: " << message << "\n";
      return RefuseLaunch(message, &robot_config, stop_signals);
    }
  }
  // A member's name is its file names (<name>.log, <name>.tlog). One that
  // breaks the node-name rule is a graph error the lint reports, but
  // --allow-graph-errors overrides the lint, and such a name can take a file
  // that is not its own: `talos` writes the launcher's talos.log, and so does
  // `Talos` on a case-insensitive file system (macOS's default APFS). This
  // refusal is unconditional.
  for (const NodeSpec& spec : node_specs) {
    const auto problem = naming::CheckNodeName(spec.name);
    if (!problem) continue;
    const std::string message =
        "roster member '" + spec.name + "': " + *problem +
        "; refused even with --allow-graph-errors, since the name is its "
        "output and event log file names (the launcher's own is " +
        std::string{kLauncherOutputFile} + ")";
    ConsoleLine(std::cerr) << "launcher: " << message << "\n";
    return RefuseLaunch(message, &robot_config, stop_signals);
  }
  // Checked again right before retention lists the log root: another
  // launch may have taken the lock meanwhile.
  if (lock_needs_room && !take_lock()) {
    ConsoleLine(std::cerr) << "launcher: " << lock_error << "\n";
    return RefuseLaunch(lock_error, &robot_config, stop_signals);
  }
  const Pruning pruning = session_lock      ? Pruning::kLocked
                          : lock_needs_room ? Pruning::kUnlocked
                                            : Pruning::kNone;
  std::string retention_record;
  const uint64_t session_log_budget = ApplyRetention(
      robot_config, node_specs.size(), pruning, retention_record);
  if (lock_needs_room && (!take_lock() || !session_lock)) {
    ConsoleLine(std::cerr) << "launcher: " << lock_error << "\n";
    ReportUnwrittenRetention(retention_record);
    return RefuseLaunch(lock_error, &robot_config, stop_signals);
  }
  if (!CreateSessionDirectory(robot_config, node_specs.size(),
                              pruning != Pruning::kNone, retention_record)) {
    return RefuseLaunch(
        "cannot create or lock session directory " + options_.output_dir,
        &robot_config, stop_signals);
  }
  launcher_output_->Record(options_.output_dir + "/" +
                           std::string{kLauncherOutputFile});
  if (!options_.replay_inputs.empty()) {
    return RunOnRecordedInputs(node_specs);
  }
  // Before any probe, so an unusable registry refuses the launch at once
  // rather than after the roster is described.
  if (starts_roster) {
    namespace_names.RegistryPrepared();
    try {
      introspect::PrepareRegistry();
    } catch (const std::exception& e) {
      ConsoleLine(std::cerr) << "launcher: " << e.what() << "\n";
      return RefuseLaunch(e.what(), &robot_config, stop_signals);
    }
  }

  // Phase one: describe and lint before anything starts.
  std::vector<NodeDescription> described;
  std::vector<naming::Diagnostic> diagnostics;
  try {
    diagnostics = DescribeGraph(
        node_specs, options_, described, TargetSourcesOf(robot_config),
        TargetModeSourcesOf(robot_config), robot_config.require_target_coverage,
        starts_roster ? process::StopFd() : -1);
  } catch (const std::exception& e) {
    ConsoleLine(std::cerr) << "launcher: " << e.what() << "\n";
    return RefuseLaunch(e.what(), &robot_config, stop_signals);
  }
  // A stop kills the probes it finds running (ProbeDescribe), so their
  // descriptions are not the roster's: nothing is linted or recorded.
  if (stopped_before_start()) return 0;
  for (auto& finding : LintParameterUnits(robot_config)) {
    diagnostics.push_back(std::move(finding));
  }
  for (auto& finding :
       LintParameterKeys(robot_config, described, options_.config_path)) {
    diagnostics.push_back(std::move(finding));
  }
  for (auto& finding : LintPythonActuatorPeriods(robot_config, described)) {
    diagnostics.push_back(std::move(finding));
  }
  for (auto& finding : naming::CheckHardwareConfig(robot_config.hardware)) {
    diagnostics.push_back(std::move(finding));
  }
  if (effective_local_address.empty() && !options_.simulation &&
      effective_remote != "127.0.0.1") {
    diagnostics.push_back(
        {naming::Severity::WARNING, std::string{kHardwareNodeName},
         "[robot] local_address is unset; hardware_node binds 0.0.0.0 and may "
         "pick another interface on route changes (set [robot] local_address "
         "or --local-address)"});
  }
  const std::size_t declared_topics = CountDeclaredTopics(described);

  const std::string graph_path = options_.output_dir + "/graph.json";
  std::string graph_error;
  if (!WriteFileAtomically(
          graph_path,
          GraphToJson(options_.session_id, options_.config_path, described,
                      diagnostics, options_.bundle_config_sha256,
                      CanBusNamesOf(robot_config)),
          options_.sync_file, graph_error)) {
    const std::string message = "cannot write " + graph_path + " (" +
                                graph_error + "); nothing was started";
    ConsoleLine(std::cerr) << "launcher: " << message << "\n";
    return RefuseLaunch(message, &robot_config, stop_signals);
  }
  PrintDeclaredGraph(described, diagnostics, declared_topics, graph_path);
  // Its warning is in the graph above; an observer's absence stops nothing.
  DropUnansweredStudioBridge(node_specs, described);

  // Checked after dropping an unanswered Studio bridge so an absent observer
  // never causes an off-by-one capacity refusal.
  if (starts_roster) {
    try {
      // Every remaining member registers a row, including the sim gateway and
      // Studio's bridge when present.
      const std::string full = RegistryCapacityError(
          introspect::RegistryReader{}, node_specs.size());
      if (!full.empty()) {
        ConsoleLine(std::cerr) << "launcher: " << full << "\n";
        return RefuseLaunch(full, &robot_config, stop_signals);
      }
    } catch (const std::exception& e) {
      ConsoleLine(std::cerr) << "launcher: " << e.what() << "\n";
      return RefuseLaunch(e.what(), &robot_config, stop_signals);
    }
  }

  if (naming::HasError(diagnostics) && !options_.allow_graph_errors) {
    ConsoleLine(std::cerr)
        << "launcher: the declared graph has errors; refusing to "
           "start. Fix the errors above, or, in simulation, pass "
           "--allow-graph-errors with --sim to launch anyway.\n";
    std::string details;
    int error_count = 0;
    for (const auto& diagnostic : diagnostics) {
      if (diagnostic.severity != naming::Severity::ERROR) continue;
      if (error_count < 5) {
        if (!details.empty()) details += "; ";
        details += diagnostic.ToString();
      }
      ++error_count;
    }
    if (details.empty()) {
      details = "the declared graph has errors";
    } else if (error_count > 5) {
      details += " (+" + std::to_string(error_count - 5) + " more)";
    }
    return RefuseLaunch(details, &robot_config, stop_signals);
  }
  if (options_.describe_only) {
    ConsoleLine(std::cout) << "[launcher] describe-only: nothing was started\n";
    return 0;
  }
  if (stopped_before_start()) return 0;

  // Phase two. The session's topics exist before any node does, and until
  // the session ends.
  std::vector<RTMSQueue> segments;
  try {
    segments = CreateSessionSegments(described);
  } catch (const std::exception& e) {
    ConsoleLine(std::cerr) << "launcher: " << e.what() << "\n";
    return RefuseLaunch(e.what(), &robot_config, stop_signals);
  }
  ConsoleLine(std::cout) << "[launcher] Created " << segments.size()
                         << " session-owned topic segments\n";
  if (options_.session_segments_created) options_.session_segments_created();

  SessionManifest manifest;
  manifest.session_id = options_.session_id;
  manifest.config_path = options_.config_path;
  manifest.config_sha256 = options_.bundle_config_sha256;
  manifest.git_revision = options_.bundle_git_revision;
  manifest.git_dirty = options_.bundle_git_dirty;
  manifest.output_dir = options_.output_dir;
  // Recorded since the session directory's creation (above).
  manifest.launcher_log =
      options_.output_dir + "/" + std::string{kLauncherOutputFile};
  manifest.simulation = options_.simulation;
  manifest.declared_topics = declared_topics;
  manifest.start_wall_ns = WallClockNanos();
  session_manifest_ = manifest;
  PersistManifest({});

  // Every member's stdout and stderr, drained into its <name>.log.
  try {
    output_.emplace(&*launcher_output_);
  } catch (const std::exception& e) {
    ConsoleLine(std::cerr) << "launcher: " << e.what()
                           << "; nothing was started\n";
    return 1;
  }

  std::vector<NodeProcessInfo> procs;
  procs.reserve(node_specs.size());
  std::optional<StudioBridge> studio;
  std::optional<std::size_t> sim_gateway_index;
  for (std::size_t i = 0; i < node_specs.size(); ++i) {
    const auto& spec = node_specs[i];
    NodeProcessInfo& proc = procs.emplace_back();
    proc.name = spec.name;
    proc.target = spec.target;
    proc.binary_path = spec.binary_path;
    proc.binary_sha256 = FileSha256(spec.binary_path);
    // The launcher's gateway is started without --log, so it writes no
    // event log.
    if (!spec.sim_gateway) {
      proc.log_path = NodeLogPath(options_.output_dir, spec.name, 0);
    } else if (!HasSimulatorScheduleStartArg(options_.sim_gateway_args)) {
      sim_gateway_index = i;
    }
    proc.described = described[i].answered;
    if (spec.studio_bridge) {
      studio.emplace(i, spec, proc, robot_config, options_, restart_limits_,
                     described, graph_path, *output_);
    }
    if (!proc.log_path.empty()) proc.log_paths.push_back(proc.log_path);
  }
  const LogSplit log_split =
      LogBudgets(procs, described, robot_config, session_log_budget);
  ConsoleLine(std::cout) << "[launcher] Event-log window "
                         << (log_split.window.count() > 0
                                 ? std::to_string(log_split.window.count()) +
                                       " us"
                                 : std::string{"none (segments end on bytes)"})
                         << "\n";
  // Synced before any node starts; from here on every version is synced,
  // off this thread (PersistManifest).
  PersistManifest(procs, /*durable=*/true);
  manifest_writer_->Start();

  // The deadline starts with the session, so a bridge that never comes up
  // cannot push the session's end out.
  const auto deadline = options_.duration_s > 0
                            ? options_.supervision_clock() +
                                  std::chrono::seconds(options_.duration_s)
                            : std::chrono::steady_clock::time_point::max();

  const uint16_t remote_port = EffectiveRemotePort(robot_config, options_);
  const uint16_t local_port = EffectiveLocalPort(robot_config, options_);
  ConsoleLine(std::cout) << "[launcher] Starting session "
                         << options_.session_id << " with " << procs.size()
                         << " nodes...\n";
  if (effective_local_address.empty()) {
    ConsoleLine(std::cout) << "[launcher] Controller " << effective_remote
                           << ":" << remote_port << " (local port "
                           << local_port << ")\n";
  } else {
    ConsoleLine(std::cout) << "[launcher] Controller " << effective_remote
                           << ":" << remote_port << " (local "
                           << effective_local_address << ":" << local_port
                           << ")\n";
  }

  // From the segments on, a stop ends the session through supervision and
  // the final manifest, as any stop does: a member it finds not yet started
  // is never started, and has no run.
  std::vector<NodeLaunch> launches(procs.size());
  const auto stop_skips_start = [&] {
    if (!process::stop_requested.load()) return false;
    manifest.stopped_before_start = true;
    session_manifest_.stopped_before_start = true;
    return true;
  };
  for (std::size_t i = 0; i < procs.size(); ++i) {
    if (studio && i == studio->index()) continue;
    if (stop_skips_start()) break;
    auto& proc = procs[i];
    PrepareLaunch(node_specs[i], described[i], proc, robot_config,
                  log_split.shares[i], log_split.window, launches[i]);

    std::string spawn_error;
    launches[i].spawned_monotonic_ns = introspect::monotonic_now_ns();
    const pid_t pid = SpawnNode(proc, launches[i], spawn_error);
    if (pid < 0) {
      SpawnFailed(proc, launches[i], options_.supervision_clock(), spawn_error);
      continue;
    }
    proc.pid = pid;
    proc.BeginRun(pid);
    PersistManifest(procs);
    ConsoleLine(std::cout) << "[launcher] Started " << proc.name << " (pid "
                           << pid << ", target " << proc.target << ")\n";
    if (options_.member_spawned) options_.member_spawned(proc.name);
  }
  if (studio && !stop_skips_start())
    studio->Start(procs[studio->index()], options_.supervision_clock);
  PersistManifest(procs);

  Supervise(procs, launches, studio, sim_gateway_index, deadline);
  manifest.end_wall_ns = WallClockNanos();
  // Every run has been reaped and its group killed, so the pipes end.
  output_->Stop();
  for (const auto& proc : procs) {
    const std::string path = NodeOutputPath(options_.output_dir, proc.name);
    if (const uint64_t dropped = output_->Dropped(path); dropped != 0)
      ConsoleLine(std::cerr)
          << "[launcher] " << proc.name << ": " << dropped
          << " bytes of output dropped (marked in " << path << ")\n";
  }

  // Whose last exit leaves the status alone: a run the stop's own signal
  // ended (its manifest run's stopped_by_session, so the two never
  // disagree), the Studio bridge however it ended (an observer, REQ-OPS-30:
  // killed while still settling, crashed, left, or stopped), which the
  // manifest records all the same, and a member the stop left unstarted (no
  // run).
  std::vector<bool> exit_not_counted;
  exit_not_counted.reserve(procs.size());
  for (std::size_t i = 0; i < procs.size(); ++i) {
    const auto& runs = procs[i].runs;
    exit_not_counted.push_back(runs.empty() || runs.back().stopped_by_session ||
                               (studio && i == studio->index()));
  }
  int exit_status = SummarizeLogs(procs, exit_not_counted);
  if (endpoints_refused_) exit_status = 1;
  manifest.nodes = std::move(procs);

  session_manifest_ = manifest;
  PersistManifest(manifest.nodes, /*durable=*/true);
  PrintManifest(manifest);
  return exit_status;
}

void Launcher::PrepareLaunch(const NodeSpec& spec,
                             const NodeDescription& described,
                             const NodeProcessInfo& proc,
                             const config::RobotConfig& robot_config,
                             uint64_t log_budget,
                             std::chrono::microseconds log_window,
                             NodeLaunch& launch) const {
  launch.restart = RestartSchedule{spec.critical};
  launch.compare_endpoints = described.answered;
  launch.declared = described.declared.sources;
  launch.holds_stop = described.answered && described.declared.holds_stop;
  auto period_us =
      robot_config.toml_data["subsystems"][proc.name]["period_us"]
          .value_or<int64_t>(
              robot_config.toml_data["robot"]["period_us"].value_or<int64_t>(
                  5000));
  launch.limits = {
      HeartbeatTimeout(period_us, options_.heartbeat_periods),
      std::chrono::milliseconds{options_.heartbeat_startup_grace_ms},
      options_.construction_timeout};
  auto& args = launch.args;
  args.push_back(spec.binary_path);
  if (spec.sim_gateway) {
    // The gateway stamps the config's config_id; unconfigured it stamps
    // 0, which every mechanism rejects.
    args.insert(args.end(), {"--config", options_.config_path, "--session-id",
                             std::to_string(options_.session_id)});
    if (!HasSimulatorScheduleStartArg(options_.sim_gateway_args)) {
      args.insert(args.end(), {"--schedule-start-file",
                               options_.output_dir + "/" +
                                   std::string{kSimScheduleStartFile}});
    }
  } else {
    args.insert(args.end(),
                {"--session-id", std::to_string(options_.session_id), "--log"});
    launch.log_arg = args.size();
    launch.log_budget = log_budget;
    args.insert(args.end(),
                {proc.log_path, "--log-max-bytes", std::to_string(log_budget)});
    // The same window for every run, a restart's included (Restart rewrites
    // only the path and the cap); absent, the log ends segments on bytes.
    if (log_window.count() > 0) {
      args.insert(args.end(),
                  {"--log-window-us", std::to_string(log_window.count())});
    }
    args.insert(args.end(), {"--config", options_.config_path});
    if (options_.simulation) args.push_back("--sim");
    // Started with the graph it was described with (DescribeGraph).
    if (described.answered && described.declared.reads_graph) {
      args.insert(args.end(), {"--graph", DeclaredGraphPath(options_)});
    }
    // REQ-SAFE-42: when the probe declared a hardware_request endpoint without
    // untargeted_outputs, pass --targeted-outputs so neither the first run nor
    // any restart (which reuses launch.args) can hide AllowUntargetedOutputs
    // behind !Node::describing(). Only real talos::Run / talos.run actuator
    // nodes declare kSourceFlagHardwareRequest; hardware_node, sim_gateway,
    // studio_bridge and non-actuator nodes never do.
    if (described.answered && !options_.allow_graph_errors) {
      const bool targeted_actuator = std::any_of(
          described.declared.sources.begin(), described.declared.sources.end(),
          [](const naming::SourceShape& source) {
            return naming::IsSender(source.kind) &&
                   (source.flags & naming::kSourceFlagHardwareRequest) != 0 &&
                   (source.flags & naming::kSourceFlagUntargetedOutputs) == 0;
          });
      if (targeted_actuator) args.push_back("--targeted-outputs");
    }
  }
  // The hardware bridge's controller addresses are among them
  // (ControllerArgs), as they were in its probe.
  args.insert(args.end(), spec.extra_args.begin(), spec.extra_args.end());
  // No --duration-s: the session's deadline is the launcher's alone
  // (Supervise), so nothing but its stop ends a member that runs well. A
  // member's own duration would count from its own start on its own clock
  // (on macOS one that counts host sleep), and could run out first: a bridge
  // that heard no State then exits 1, a crash. A member whose launcher dies
  // is killed by its guardian (spawn.h).
}

void Launcher::PersistManifest(const std::vector<NodeProcessInfo>& procs,
                               bool durable) {
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

int Launcher::RunOnRecordedInputs(const std::vector<NodeSpec>& specs) {
  std::error_code ec;
  if (!std::filesystem::is_directory(options_.replay_inputs, ec)) {
    ConsoleLine(std::cerr) << "launcher: --replay-inputs "
                           << options_.replay_inputs
                           << " is not a session directory\n";
    return 1;
  }
  // The new log must not land among the logs being read.
  const auto relative = std::filesystem::weakly_canonical(options_.output_dir)
                            .lexically_relative(std::filesystem::canonical(
                                options_.replay_inputs));
  if (!relative.empty() && *relative.begin() != "..") {
    ConsoleLine(std::cerr)
        << "launcher: --output-dir must not be the replayed session's "
           "directory\n";
    return 1;
  }
  const auto spec =
      std::find_if(specs.begin(), specs.end(),
                   [&](const NodeSpec& s) { return s.name == options_.only; });
  if (spec == specs.end()) {
    ConsoleLine(std::cerr) << "launcher: --only " << options_.only
                           << " names no roster member of "
                           << options_.config_path << "\n";
    return 1;
  }
  if (spec->name == kHardwareNodeName || spec->sim_gateway ||
      spec->studio_bridge) {
    ConsoleLine(std::cerr) << "launcher: " << spec->name
                           << " is not a talos node and has no input replay\n";
    return 1;
  }

  // A replay of another hardware protocol's session would hand the node's
  // devices layer States in another layout, decoded with this build's (one
  // that kept every length decodes, wrongly), and could not match the
  // recording it is compared with. The version is the one the session's
  // bridge recorded, else its manifest's. A session that records none is
  // replayed: its States that this build's codec rejects are no State to
  // the node (hardware_record.h, "Which logs a build can read").
  const RecordedProtocol recorded =
      RecordedProtocolVersion({options_.replay_inputs});
  if (const std::string line = recorded.Disagreement(); !line.empty()) {
    ConsoleLine(std::cerr) << "launcher: --replay-inputs "
                           << options_.replay_inputs << ": " << line << "\n";
  }
  if (recorded.refused()) {
    ConsoleLine(std::cerr) << "launcher: --replay-inputs "
                           << options_.replay_inputs << " was "
                           << recorded.Refusal() << ": not replayed\n";
    return 1;
  }

  const std::string log_path = NodeLogPath(options_.output_dir, spec->name, 0);
  // The node runs as it ran in the session: --sim changes what it accepts
  // (the devices layer treats the config as commissioned, which is part of
  // the config id every recorded State carries). The session's manifest
  // says; without one, this launch's --sim does.
  const bool simulation =
      SessionSimulation(options_.replay_inputs).value_or(options_.simulation);
  ConsoleLine(std::cout) << "[launcher] Replaying session "
                         << options_.replay_inputs << " into " << spec->name
                         << " (" << spec->target << ")"
                         << (simulation ? " under --sim" : "") << "; log "
                         << log_path << "\n";
  std::vector<std::string> args{spec->binary_path,
                                "--replay-inputs",
                                options_.replay_inputs,
                                "--session-id",
                                std::to_string(options_.session_id),
                                "--log",
                                log_path,
                                "--config",
                                options_.config_path};
  if (simulation) args.push_back("--sim");
  if (options_.allow_capped_replay) args.push_back("--allow-capped-replay");
  std::string error;
  const pid_t pid = SpawnProcess(std::move(args), error);
  if (pid < 0) {
    ConsoleLine(std::cerr) << "[launcher] cannot spawn " << spec->name << " ("
                           << error << ")\n";
    return 1;
  }
  int status = 0;
  while (::waitpid(pid, &status, 0) < 0) {
    if (errno != EINTR) return 1;
  }
  FinishProcessGroup(pid);
  const bool clean = WIFEXITED(status) && WEXITSTATUS(status) == 0;
  ConsoleLine(std::cout) << "[launcher] " << spec->name << " finished ("
                         << ExitReason(status) << ")\n";
  return clean ? 0 : (WIFEXITED(status) ? WEXITSTATUS(status) : 1);
}

std::string Launcher::StallReport(const NodeLaunch& launch, bool constructing,
                                  SupervisionTime now) {
  const auto ms = [now](SupervisionTime since) {
    return std::to_string(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - since)
            .count());
  };
  const RunLiveness& run = launch.liveness;
  if (constructing) {
    return " stalled: construction heartbeat unchanged for " +
           ms(run.heartbeat_changed_at()) + " ms";
  }
  return " stalled: dispatch count unchanged for " +
         ms(run.progress_changed_at()) + " ms (heartbeat " +
         (now - run.heartbeat_changed_at() < launch.limits.heartbeat_timeout
              ? "still moving)"
              : "unchanged too, for " + ms(run.heartbeat_changed_at()) +
                    " ms)");
}

void Launcher::StopSignals::Install() {
  if (installed_) return;
  for (std::size_t i = 0; i < kSignals.size(); ++i)
    ::sigaction(kSignals[i], nullptr, &saved_[i]);
  installed_ = true;
  InstallSignals();
}

Launcher::StopSignals::~StopSignals() {
  if (!installed_) return;
  for (std::size_t i = 0; i < kSignals.size(); ++i)
    ::sigaction(kSignals[i], &saved_[i], nullptr);
}

void Launcher::InstallSignals() {
  // The stop pipe exists before a stop can arrive, and a stop an earlier
  // session of this process took is cleared, its byte read out.
  process::StopFd();
  process::stop_requested.store(false);
  process::ConsumeStopWake();
  struct sigaction stop {};
  // The flag and the stop pipe first (process::RequestStop): the wake
  // written next makes the sleeping supervision loop run a pass, which reads
  // the flag (SupervisionWakeup); a probe waits on the pipe (ProbeDescribe).
  stop.sa_handler = [](int received) {
    process::RequestStop(received);
    SupervisionWakeup::Wake();
  };
  sigemptyset(&stop.sa_mask);
  stop.sa_flags = SA_RESTART;
  ::sigaction(SIGINT, &stop, nullptr);
  ::sigaction(SIGTERM, &stop, nullptr);
  // A hangup is a stop too, unless the launcher started with it ignored
  // (`nohup`): Run changes no disposition but SIGPIPE's before this.
  struct sigaction hangup {};
  if (::sigaction(SIGHUP, nullptr, &hangup) == 0 &&
      hangup.sa_handler != SIG_IGN)
    ::sigaction(SIGHUP, &stop, nullptr);
}

pid_t Launcher::SpawnNode(const NodeProcessInfo& proc, const NodeLaunch& launch,
                          std::string& error) {
  const auto injected = options_.spawn_failures.find(proc.name);
  if (injected != options_.spawn_failures.end() && injected->second > 0) {
    --injected->second;
    error = std::string{"fork: "} + std::strerror(EAGAIN);
    return -1;
  }
  // Run starts the drain before the first spawn; without one there is
  // nowhere to send the member's output.
  if (!output_) {
    error = "no output drain";
    return -1;
  }
  return SpawnDrained(launch.args, error, *output_,
                      NodeOutputPath(options_.output_dir, proc.name), proc.name,
                      /*echo=*/true, launch.holds_stop);
}

std::chrono::milliseconds Launcher::ScheduleRestart(NodeProcessInfo& proc,
                                                    NodeLaunch& launch,
                                                    SupervisionTime now) {
  const auto delay = launch.restart.Schedule(now, restart_limits_);
  ShowRestart(proc, launch.restart);
  return delay;
}

void Launcher::NodeLaunch::NewRun(SupervisionTime spawned) {
  liveness.Begin(spawned);
  kill_reason = {};
  heartbeat_killed = false;
  endpoints_checked = false;
  realtime_reported = false;
  loop_started = false;
  run_recorder_failed = false;
}

void Launcher::SpawnFailed(NodeProcessInfo& proc, NodeLaunch& launch,
                           SupervisionTime now, const std::string& error) {
  proc.failed = true;
  proc.RecordSpawnFailure();
  ConsoleLine line{std::cerr};
  line << "[launcher] cannot spawn " << proc.name << " (" << error << "); ";
  // Never `stopping` here: Supervise cancels a pending restart once the
  // session stops.
  if (!ShouldRestartNode(/*clean_exit=*/false, launch.restart.critical(),
                         restart_limits_.max_restarts, /*stopping=*/false)) {
    line << "restarts are disabled\n";
    return;
  }
  const auto delay = ScheduleRestart(proc, launch, now);
  line << (proc.quarantined ? "quarantined; retry" : "retry") << " in "
       << delay.count() << " ms\n";
}

uint64_t Launcher::RestartLogBudget(NodeProcessInfo& proc, uint64_t budget) {
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
       i + 1 < files.size() && (used >= budget || budget - used < floor); ++i) {
    std::error_code error;
    if (!std::filesystem::remove(files[i].path, error) || error) continue;
    used -= files[i].bytes;
    // Creating the marker is the point; the stream closes at once.
    const std::ofstream marker{event::log::CappedMarkerPath(files[i].run)};
    proc.log_capped = true;
  }
  // 0 means unbounded to the writer, so never less than 1.
  return std::max<uint64_t>(
      {1, floor, used >= budget ? uint64_t{0} : budget - used});
}

void Launcher::Restart(NodeProcessInfo& proc, NodeLaunch& launch,
                       SupervisionTime now) {
  if (launch.restart.quarantined())
    ConsoleLine(std::cout) << "[launcher] " << proc.name
                           << " retrying after quarantine\n";
  // The attempt counts against the rolling budget whether or not it spawns.
  launch.restart.Started(now);
  ShowRestart(proc, launch.restart);
  // Only a spawned run sets a pid.
  const int restart = proc.pid > 0 ? proc.restart_count + 1 : 0;
  if (launch.log_arg != 0) {
    launch.args[launch.log_arg] =
        NodeLogPath(options_.output_dir, proc.name, restart);
    launch.args[launch.log_arg + 2] =
        std::to_string(RestartLogBudget(proc, launch.log_budget));
  }
  std::string error;
  launch.spawned_monotonic_ns = introspect::monotonic_now_ns();
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
  proc.BeginRun(pid);
  launch.NewRun(options_.supervision_clock());
  // In the first start's form, so every run's pid is in the launcher's
  // output, not only in the manifest. A first run whose first spawn failed
  // starts here too, as a start.
  ConsoleLine line{std::cout};
  line << "[launcher] " << (restart > 0 ? "Restarted " : "Started ")
       << proc.name << " (pid " << pid << ", target " << proc.target;
  if (restart > 0) line << ", restart " << restart;
  line << ")\n";
}

bool Launcher::NoteRealtime(NodeProcessInfo& proc, NodeLaunch& launch,
                            uint32_t flags) const {
  if (launch.realtime_reported) return false;
  const std::optional<process::RealtimeOutcome> outcome =
      introspect::RealtimeOf(flags);
  if (!outcome) return false;
  launch.realtime_reported = true;
  const std::string warning =
      RealtimeWarning(proc.name, proc.pid, *outcome, options_.simulation);
  if (!warning.empty()) ConsoleLine(std::cerr) << warning;
  return proc.RecordRealtime(*outcome);
}

bool Launcher::NoteRecording(NodeProcessInfo& proc, NodeLaunch& launch,
                             uint32_t flags) {
  const std::string& log =
      proc.log_paths.empty() ? proc.log_path : proc.log_paths.back();
  // Once per run's log: each run that dropped records is named.
  const bool gap =
      (flags & introspect::kFlagLogGap) != 0 && proc.RecordLogGap(log);
  if (gap) PrintLogGap(proc.name, log);
  // Once per run: record this run's log in recorder_failed_logs independently
  // of the per-member stderr-once flags in recording_reported.
  bool changed = gap;
  if ((flags & introspect::kFlagRecorderFailed) != 0 &&
      !launch.run_recorder_failed) {
    launch.run_recorder_failed = true;
    if (proc.RecordRecorderFailure(log)) changed = true;
  }
  const uint32_t fresh =
      flags & ~launch.recording_reported &
      (introspect::kFlagRecorderFailed | introspect::kFlagLogCapped);
  if (fresh == 0) return changed;
  launch.recording_reported |= fresh;
  if ((fresh & introspect::kFlagRecorderFailed) != 0) {
    proc.failed = true;
    ConsoleLine(std::cerr) << "[launcher] " << proc.name << " event log " << log
                           << " failed; the rest of this run is not recorded\n";
  }
  if ((fresh & introspect::kFlagLogCapped) != 0) {
    proc.log_capped = true;
    ConsoleLine(std::cerr)
        << "[launcher] " << proc.name << " event log " << log
        << " reached its byte budget; "
        << (CappedLogStopped(log)
                ? "the rest of this run is not recorded\n"
                : "it keeps recording, deleting its oldest records\n");
  }
  return true;
}

bool Launcher::CheckEndpoints(NodeProcessInfo& proc, NodeLaunch& launch,
                              const std::vector<naming::SourceShape>& live,
                              bool complete) {
  const std::string mismatch =
      EndpointMismatch(launch.declared, live, complete);
  if (mismatch.empty()) return false;
  proc.endpoint_mismatch = true;
  ConsoleLine line{std::cerr};
  line << "[launcher] error: " << proc.name << " (pid " << proc.pid
       << "): live endpoints differ from its --describe probe, the "
          "graph the lint checked: "
       << mismatch << "; ";
  if (options_.allow_graph_errors) {
    line << "running anyway (--allow-graph-errors)\n";
  } else if (!launch.restart.critical()) {
    launch.endpoints_stopped = true;
    launch.heartbeat_killed = true;
    launch.kill_reason = kRunEndpointMismatch;
    SignalProcessGroup(proc.pid, SIGKILL);
    line << "non-critical: killing it, without restarts this "
            "session\n";
  } else if (proc.restart_count == 0) {
    endpoints_refused_ = true;
    launch.kill_reason = kRunEndpointMismatch;
    line << "stopping the session. A node must declare the same "
            "endpoints whether or not Node::describing(); in "
            "simulation, --sim --allow-graph-errors runs anyway.\n";
  } else {
    line << "a critical member's restart: the session keeps running\n";
  }
  return true;
}

bool Launcher::RosterContinues(const std::vector<NodeProcessInfo>& procs,
                               const std::vector<NodeLaunch>& launches,
                               const std::optional<StudioBridge>& studio,
                               std::size_t self) {
  for (std::size_t j = 0; j < procs.size(); ++j) {
    if (j == self || (studio && j == studio->index())) continue;
    if (launches[j].restart.last_exit_clean()) continue;
    if (launches[j].restart.pending()) return true;
    if (procs[j].pid > 0 && !procs[j].exited) return true;
  }
  return false;
}

bool Launcher::WatchRun(NodeProcessInfo& proc, NodeLaunch& launch,
                        const introspect::RegistryMapping* registry) {
  bool changed = false;
  uint32_t recording = 0;
  bool row_loop_started = false;
  std::optional<std::vector<naming::SourceShape>> live;
  bool live_complete = true;
  std::optional<DeliveryCounts> delivery;
  const auto read_row = [&]() -> RowObservation {
    if (registry == nullptr) return {};
    for (const auto& record : registry->header()->nodes) {
      if (record.state.load(std::memory_order_acquire) !=
          static_cast<uint64_t>(introspect::SlotState::ACTIVE))
        continue;
      const auto generation = record.generation.load(std::memory_order_acquire);
      if (record.pid != static_cast<uint64_t>(proc.pid) ||
          record.session_id != options_.session_id)
        continue;
      const auto beat =
          record.heartbeat_monotonic_ns.load(std::memory_order_relaxed);
      // An earlier run's row whose pid this run reuses, left active by its
      // kill until the next claim frees it: every row this run claims beats
      // at or after its spawn (FinalDeliveryCounts reads the same way).
      if (beat < launch.spawned_monotonic_ns) continue;
      const auto dispatch =
          record.dispatch_count.load(std::memory_order_relaxed);
      const auto flags = record.flags.load(std::memory_order_acquire);
      // Once per run, from the pass that first sees its row active
      // with its endpoints published (CheckEndpoints).
      std::optional<std::vector<naming::SourceShape>> published;
      if (launch.compare_endpoints && !launch.endpoints_checked &&
          (flags & introspect::kFlagConstructing) == 0) {
        published = PublishedEndpoints(record, live_complete);
      }
      const bool loop_ready =
          (flags & introspect::kFlagConstructing) == 0 &&
          (introspect::RealtimeOf(flags).has_value() || dispatch > 0);
      DeliveryCounts loss = DeliveryCountsOf(record);
      if (record.generation.load(std::memory_order_acquire) != generation ||
          record.state.load(std::memory_order_acquire) !=
              static_cast<uint64_t>(introspect::SlotState::ACTIVE))
        continue;
      live = std::move(published);
      delivery = std::move(loss);
      recording = flags;
      row_loop_started = loop_ready;
      return {.found = true,
              .constructing = (flags & introspect::kFlagConstructing) != 0,
              .heartbeat = beat,
              .dispatch = dispatch};
    }
    return {};
  };
  // The clock is read around the row read, not taken from the pass's
  // start: the launcher's own stall earlier in the pass must not count
  // against this run (RunLiveness).
  const LivenessCheck check = launch.liveness.Watch(options_.supervision_clock,
                                                    read_row, launch.limits);
  if (row_loop_started) launch.loop_started = true;
  if (NoteRecording(proc, launch, recording)) changed = true;
  if (NoteRealtime(proc, launch, recording)) changed = true;
  // Kept every pass, posted only when a count first leaves 0: a later
  // increase rides the next version the launcher posts anyway (at the
  // latest the one the run's end posts), so a healthy session's counts add
  // no manifest version.
  if (delivery && proc.RecordDeliveryLoss(*delivery)) changed = true;
  if (live) {
    launch.endpoints_checked = true;
    if (CheckEndpoints(proc, launch, *live, live_complete)) changed = true;
  }
  if (!check.kill.empty()) {
    const bool constructing = (recording & introspect::kFlagConstructing) != 0;
    ConsoleLine(std::cerr) << "[launcher] " << proc.name
                           << (check.kill == kRunConstructionDeadline
                                   ? " construction deadline exceeded"
                               : check.kill == kRunHeartbeatMissing
                                   ? " heartbeat missing"
                                   : StallReport(launch, constructing,
                                                 check.judged_at))
                           << "; killing pid " << proc.pid << "\n";
    SignalProcessGroup(proc.pid, SIGKILL);
    launch.heartbeat_killed = true;
    launch.kill_reason = check.kill;
  }
  return changed;
}

void Launcher::Supervise(std::vector<NodeProcessInfo>& procs,
                         std::vector<NodeLaunch>& launches,
                         std::optional<StudioBridge>& studio,
                         std::optional<std::size_t> sim_gateway_index,
                         std::chrono::steady_clock::time_point deadline) {
  bool stop_sent = false;
  bool sim_schedule_done = !sim_gateway_index.has_value();
  auto shutdown_deadline = std::chrono::steady_clock::time_point::max();
  std::optional<introspect::RegistryMapping> registry;
  // Every first run's liveness counts from here, not its spawn.
  const auto started = options_.supervision_clock();
  for (auto& launch : launches) launch.NewRun(started);
  if (studio) studio->BeginSupervision(started);
  ServiceNotifier notifier;
  const auto watchdog = notifier.watchdog_interval();
  // A pass starts as soon as a child exits or a stop signal arrives.
  const SupervisionWakeup wakeup;
  notifier.Send("READY=1");
  auto next_lock_refresh = std::chrono::steady_clock::time_point::min();
  const bool refreshes_locks =
      session_lock_ != nullptr || session_directory_.has_value();
  bool lock_lost_reported = false;
  bool directory_lock_retaken_reported = false;
  bool directory_lock_lost_reported = false;
  for (;;) {
    notifier.Send("WATCHDOG=1");
    const auto now = options_.supervision_clock();
    const bool past_deadline = now >= deadline;
    const bool stopping =
        process::stop_requested.load() || stop_sent || past_deadline;
    // A stop signal can land after `stopping` was sampled: systemctl stop
    // signals the launcher and then every node, so a node killed by it can
    // be reaped in this pass. Restart decisions read the flag again, so
    // once shutdown starts nothing is restarted.
    const auto stop_arrived = [&] {
      return stopping || process::stop_requested.load() || endpoints_refused_;
    };
    // A pass posts at most one manifest version, after its restarts (see
    // the post below).
    bool manifest_changed = false;
    if (refreshes_locks && now >= next_lock_refresh) {
      next_lock_refresh = now + kLockFileRefresh;
      if (session_lock_ != nullptr && !session_lock_->Refresh() &&
          !lock_lost_reported) {
        lock_lost_reported = true;
        ConsoleLine(std::cerr)
            << "[launcher] the session lock " << session_lock_->path()
            << " was removed; a second launcher in this namespace "
               "would no longer be refused\n";
      }
      using Refreshed = SessionDirectoryLock::Refreshed;
      // An unheld session.lock is retried only here, at supervision start
      // and then hourly: between two tries a retention whose probe locks
      // again may remove this session, and on a network log root this flock
      // can block supervision and the watchdog ping (README "Session
      // segments and logs"; TODO.md LOG-10).
      const Refreshed directory =
          session_directory_
              ? session_directory_->Refresh(options_.lock_session_directory)
              : Refreshed::kHeld;
      if (directory == Refreshed::kTaken) {
        // Once: a lock taken stays held.
        ConsoleLine(std::cerr)
            << "[launcher] locked " << session_directory_->path()
            << ", which could not be locked when the session started; "
               "retention passes over this session from now on\n";
      } else if (directory == Refreshed::kRetaken &&
                 !directory_lock_retaken_reported) {
        directory_lock_retaken_reported = true;
        ConsoleLine(std::cerr)
            << "[launcher] " << session_directory_->path()
            << " was removed while this session held it; locked it "
               "again on a new file, so retention passes over this "
               "session again\n";
      } else if (directory == Refreshed::kLost &&
                 !directory_lock_lost_reported) {
        directory_lock_lost_reported = true;
        ConsoleLine(std::cerr)
            << "[launcher] " << session_directory_->path()
            << " was removed and cannot be locked again (retention "
               "holds it, or the session directory was moved away); "
               "retention may remove this running session\n";
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
      if (proc.pid <= 0 || proc.exited) continue;
      // Every member is supervised by its registry row; Studio's bridge runs
      // its own observer liveness and bounded restart policy (StudioBridge).
      if (studio && i == studio->index()) {
        if (studio->Watch(proc, registry ? &*registry : nullptr,
                          options_.supervision_clock, stopping))
          manifest_changed = true;
      } else if (!stopping && !launch.heartbeat_killed) {
        if (WatchRun(proc, launch, registry ? &*registry : nullptr))
          manifest_changed = true;
      }
      int status = 0;
      if (::waitpid(proc.pid, &status, WNOHANG) != proc.pid) continue;
      FinishProcessGroup(proc.pid);
      // The run's counts as it ended, before EndRun closes the run. Studio's
      // bridge reports no delivery-loss counts.
      if (registry && !(studio && i == studio->index())) {
        uint32_t flags = 0;
        if (const auto loss =
                FinalDeliveryCounts(*registry, proc.pid, options_.session_id,
                                    launch.spawned_monotonic_ns, flags)) {
          proc.RecordDeliveryLoss(*loss);
          // A run that ended before a supervision pass saw its row's event
          // log flags (one killed during a gap) is marked from them here.
          NoteRecording(proc, launch, flags);
        }
      }
      proc.exited = true;
      proc.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
      proc.exit_signal = WIFSIGNALED(status) ? WTERMSIG(status) : 0;
      manifest_changed = true;
      const bool clean = WIFEXITED(status) && WEXITSTATUS(status) == 0;
      // Read only from the launcher's own record of what it sent, never
      // from the stop flag: a stop signal from elsewhere that ended the run
      // first is not the session's (EndedByOwnStopSignal). Recorded in the
      // run, which the exit status reads too (exit_not_counted).
      const std::string_view kill_reason =
          (studio && i == studio->index() && !studio->kill_reason().empty())
              ? studio->kill_reason()
              : launch.kill_reason;
      proc.EndRun(RunEndReason(kill_reason, stop_arrived(), status),
                  EndedByOwnStopSignal(kill_reason, status));
      // An observer's restarts are its own, on a non-critical schedule that
      // touches no other member (StudioBridge).
      if (studio && i == studio->index()) {
        studio->Exited(proc, status, now, stop_arrived());
        continue;
      }
      launch.restart.RunEnded(clean);
      if (!launch.endpoints_stopped &&
          ShouldRestartNode(clean, launch.restart.critical(),
                            restart_limits_.max_restarts, stop_arrived())) {
        const auto delay = ScheduleRestart(proc, launch, now);
        ConsoleLine(std::cout)
            << "[launcher] " << proc.name << " exited (" << ExitReason(status)
            << "); " << (proc.quarantined ? "quarantined; retry" : "restart")
            << " " << proc.restart_count + 1 << " in " << delay.count()
            << " ms\n";
        continue;
      }
      ConsoleLine(std::cout)
          << "[launcher] Process " << proc.name << " (pid " << proc.pid
          << ") exited (" << ExitReason(status) << ")\n";
    }
    for (std::size_t i = 0; i < procs.size(); ++i) {
      auto& launch = launches[i];
      if (!launch.restart.pending()) continue;
      // A stopping session ends with the crash as the node's last word.
      if (stop_arrived()) {
        launch.restart.Cancel();
        ShowRestart(procs[i], launch.restart);
        manifest_changed = true;
      } else if (launch.restart.Due(now)) {
        if (launch.restart.last_exit_clean() &&
            !RosterContinues(procs, launches, studio, i)) {
          launch.restart.Cancel();
          ShowRestart(procs[i], launch.restart);
          ConsoleLine(std::cout)
              << "[launcher] " << procs[i].name
              << " finished with the rest of the roster; not "
                 "restarted\n";
        } else {
          Restart(procs[i], launch, now);
        }
        manifest_changed = true;
      }
    }
    bool sim_schedule_polling = false;
    if (!sim_schedule_done && sim_gateway_index && !stop_arrived()) {
      const auto& gw_proc = procs[*sim_gateway_index];
      const auto& gw_launch = launches[*sim_gateway_index];
      const bool gw_running = gw_proc.pid > 0 && !gw_proc.exited;
      if (!gw_running && !gw_launch.restart.pending()) {
        // sim_gateway exited or failed to spawn with no pending restart: stop
        // waiting and stop the 20 ms schedule-start poll for the rest of the
        // session.
        sim_schedule_done = true;
      } else if (gw_running) {
        bool roster_ready = gw_launch.loop_started;
        bool waiting_for_running_loop = !gw_launch.loop_started;
        for (std::size_t i = 0; i < procs.size(); ++i) {
          if (studio && i == studio->index()) continue;
          if (i == *sim_gateway_index) continue;
          const auto& proc = procs[i];
          const auto& launch = launches[i];
          if (proc.pid > 0 && !proc.exited) {
            if (!launch.loop_started) {
              roster_ready = false;
              waiting_for_running_loop = true;
            }
          } else if (launch.restart.pending() &&
                     !launch.restart.quarantined()) {
            // Exited or unspawned: wait only while a non-quarantined restart is
            // pending, so a finished, unrestartable or quarantined member
            // cannot hold the simulator's schedules forever.
            roster_ready = false;
          }
        }
        if (roster_ready) {
          const std::string path =
              options_.output_dir + "/" + std::string{kSimScheduleStartFile};
          const std::string stamp =
              std::to_string(event::Poller::now().nanos()) + "\n";
          std::string write_error;
          if (WriteFileAtomically(path, stamp, SyncFunction{}, write_error)) {
            ConsoleLine(std::cout)
                << "[launcher] Started sim_gateway schedule (" << path << ")\n";
          } else {
            ConsoleLine(std::cerr)
                << "[launcher] cannot write " << path << " (" << write_error
                << "); sim_gateway schedule not started\n";
          }
          sim_schedule_done = true;
        } else {
          sim_schedule_polling = waiting_for_running_loop;
        }
      }
    }
    if (studio) {
      const auto& bridge = procs[studio->index()];
      // Every run it begins, spawned or not, dropped from the history or not.
      const auto runs = [&bridge] {
        return bridge.runs.size() + bridge.runs_dropped;
      };
      const uint64_t began = runs();
      const int64_t retry_at = bridge.retry_at_monotonic_ns;
      const bool failed = bridge.failed;
      studio->Supervise(procs[studio->index()], now, options_.supervision_clock,
                        stop_arrived());
      if (began != runs() || retry_at != bridge.retry_at_monotonic_ns ||
          failed != bridge.failed)
        manifest_changed = true;
    }
    // Posted after the pass's restarts and the bridge's: the manifest
    // writer's create in the session directory can stall a restart's
    // directory reads (RestartLogBudget) and fork for tens to hundreds of
    // milliseconds while nodes write to the same disk, and the version
    // posted here carries every change of the pass anyway.
    if (manifest_changed) PersistManifest(procs);

    bool all_done = true;
    for (std::size_t i = 0; i < procs.size(); ++i) {
      const auto& proc = procs[i];
      if (launches[i].restart.pending()) {
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
        studio->Finish(procs[studio->index()], process::stop_requested.load());
      }
      return;
    }

    if (!stop_sent && (process::stop_requested.load() || past_deadline ||
                       endpoints_refused_)) {
      stop_sent = true;
      shutdown_deadline = now + kStopTimeout;
      ConsoleLine(std::cout) << "[launcher] Stopping children...\n";
      for (std::size_t i = 0; i < procs.size(); ++i) {
        const auto& proc = procs[i];
        if (proc.pid <= 0 || proc.exited) continue;
        // The exit this causes is reaped by a later pass, which reads the
        // kill_reason set here (EndedByOwnStopSignal).
        SignalProcessGroup(proc.pid, kStopSignal);
        if (launches[i].kill_reason.empty()) launches[i].kill_reason = kRunStop;
      }
    }
    if (stop_sent && now >= shutdown_deadline) {
      for (std::size_t i = 0; i < procs.size(); ++i) {
        const auto& proc = procs[i];
        if (proc.pid <= 0 || proc.exited) continue;
        SignalProcessGroup(proc.pid, SIGKILL);
        if (launches[i].kill_reason == kRunStop)
          launches[i].kill_reason = kRunStopTimeout;
      }
    }

    // Sleeps until the earliest time a pass has something to decide without
    // an event; a child's exit and a stop signal wake it at once. Every
    // timed decision above must be here, or it waits for an unrelated wake.
    auto wake = SupervisionTime::max();
    const auto due = [&wake](SupervisionTime at) { wake = std::min(wake, at); };
    // The pass's WATCHDOG=1 went out at about `now`.
    if (watchdog) due(now + *watchdog);
    if (refreshes_locks) due(next_lock_refresh);
    if (stop_sent) {
      // Once passed, its SIGKILLs are sent; the exits they cause wake it.
      if (now < shutdown_deadline) due(shutdown_deadline);
    } else {
      due(deadline);
      if (sim_schedule_polling) due(now + kSimScheduleStartPoll);
      // Each run the next pass watches (WatchRun): its row's next look.
      for (std::size_t i = 0; i < procs.size(); ++i) {
        if (procs[i].pid <= 0 || procs[i].exited ||
            launches[i].heartbeat_killed || (studio && i == studio->index()))
          continue;
        due(launches[i].liveness.NextLook(launches[i].limits));
      }
    }
    for (const auto& launch : launches) {
      if (launch.restart.pending()) due(launch.restart.due_at());
    }
    if (studio) due(studio->NextCheck(stop_sent));
    wakeup.Wait(wake == SupervisionTime::max()
                    ? std::chrono::nanoseconds::max()
                    : wake - options_.supervision_clock());
  }
}

}  // namespace talos::launcher
