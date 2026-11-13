#include "talOS/launcher/launcher.h"

#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <system_error>
#include <utility>

#include "talOS/events/context.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/launcher/log_budget.h"
#include "talOS/process/process.h"
#include "talOS/rtms/rtms.h"
#include "toml.hpp"

namespace talos::launcher {
namespace {

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

// What a reaped run's row says it lost as the run ended. A run that exits
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
// run's, and the run's counts are then what the supervision pass last read.
// Loads from the mapped segment only: no lock, no disk.
std::optional<DeliveryCounts> FinalDeliveryCounts(
    const introspect::RegistryMapping& registry, pid_t pid, uint64_t session_id,
    int64_t spawned_ns) {
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
    std::atomic_thread_fence(std::memory_order_acquire);
    if (record.generation.load(std::memory_order_relaxed) != generation ||
        record.state.load(std::memory_order_relaxed) == claiming)
      continue;
    counts = std::move(read);
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

  std::cout << "\n================ Declared Graph ==================\n";
  std::cout << "Nodes described: " << answered << "/" << described.size()
            << ", topics: " << declared_topics << "\n";
  for (const auto& node : described) {
    std::cout << "  " << (node.answered ? "\u2022" : "?") << " "
              << node.roster_name;
    if (node.answered) {
      std::cout << " (" << node.declared.sources.size() << " sources)\n";
    } else {
      std::cout << (node.missing ? " (not built)\n" : " (not described)\n");
    }
  }
  for (const auto& diagnostic : diagnostics) {
    auto& out =
        diagnostic.severity == naming::Severity::ERROR ? std::cerr : std::cout;
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
  std::cout << "Declared nodes: " << (roster.empty() ? "none" : roster) << "\n";
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

// Totals every run's event log into `procs` once the session has ended --
// record counts, capped and damaged logs -- and returns the session's exit
// status before the endpoint refusal: 0, or the last nonzero exit code of a
// member whose exit counts (`exit_not_counted`, one per member, false: not
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
      // The segments a capped log kept, then the newest (LogSegmentFiles).
      for (const auto& file : event::log::LogSegmentFiles(path)) {
        // Streamed: shutdown memory does not grow with the logs' size.
        try {
          const auto summary = event::log::SummarizeLog(file);
          proc.record_count += summary.record_count;
          if (!summary.damage.clean()) proc.failed = true;
        } catch (const event::log::IncompleteLogHeader& e) {
          // Cut while it was being created: no records to count.
          std::cerr << "[launcher] skipping " << file << ": " << e.what()
                    << "\n";
        } catch (...) {
          proc.failed = true;
        }
      }
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
  options_.output_dir = (root / std::to_string(options_.session_id)).string();
}

int Launcher::Run() {
  // Declared first, so destroyed last, on every return path: a roster
  // launch's stop handlers give way to the dispositions the launcher's
  // caller set only after RunSession has returned, which is after
  // Supervise has reaped every member's run and killed its group (or no
  // member was spawned, and DescribeGraph reaped every probe), and after
  // RunSession's locals have unlinked the session's topic segments and
  // released the session lock; and after the output drains have stopped
  // and the session directory's lock is released, below. Until then a stop
  // is the launcher's orderly one: none is lost, and none takes a default
  // action (ending the launcher) while a member, a probe or a segment of
  // the session exists. Afterwards there is nothing left for it to stop.
  StopSignals stop_signals;
  // A session must not end because whoever reads the launcher's stdout or
  // stderr went away (a `tee` or a log pipe closing): its writes then fail
  // with EPIPE, which LauncherOutput survives (REQ-OPS-33). Children start
  // with the default again (SpawnProcess).
  std::signal(SIGPIPE, SIG_IGN);
  // From here no launcher thread writes to descriptors 1 and 2 itself.
  try {
    launcher_output_.emplace();
  } catch (const std::exception& e) {
    std::cerr << "launcher: " << e.what() << "; nothing was started\n";
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
  // The flag overrides every graph refusal, the safety ones included
  // (REQ-SAFE-27/28) and the live-endpoint refusal, so it never reaches a
  // real controller: --sim requires the loopback remote (below). Refused
  // before the config is read, so nothing is probed, written or spawned.
  if (options_.allow_graph_errors && !options_.simulation) {
    std::cerr << "launcher: --allow-graph-errors is accepted only with --sim; "
                 "nothing was described or started\n";
    return 1;
  }
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
    std::cerr << "launcher: --replay-inputs SESSION_DIR and --only NODE go "
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
  const auto stopped_before_start = [&] {
    if (!starts_roster || !process::stop_requested.load()) return false;
    std::cout << "[launcher] stop requested before the session started; "
                 "nothing was started\n";
    return true;
  };
  session_lock_ = nullptr;
  std::optional<SessionLock> session_lock;
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
    if (session_lock) session_lock_ = &*session_lock;
    lock_needs_room = !session_lock && no_room && LockSharesLogFilesystem();
    return session_lock.has_value() || lock_needs_room;
  };
  if (starts_roster) {
    if (!take_lock()) {
      std::cerr << "launcher: " << lock_error << "\n";
      return 1;
    }
    if (lock_needs_room) {
      std::cerr << "[launcher] " << lock_error
                << "; retention runs first, then the lock is taken\n";
    }
  }
  // Nothing is written to the log root until retention has run: on a full
  // log disk it is what makes room for the new session's directory.

  config::RobotConfig robot_config;
  try {
    robot_config = config::ParseRobotConfig(options_.config_path);
  } catch (const std::exception& e) {
    std::cerr << "launcher: failed to load config from " << options_.config_path
              << ": " << e.what() << "\n";
    if (config_was_relative) {
      std::cerr << "launcher: working directory is "
                << std::filesystem::current_path()
                << "; run from the repository root or pass an absolute "
                   "--config path\n";
    }
    return 1;
  }
  // A simulation config (its gains, bindings and travel) never drives real
  // hardware. --describe-only and --replay-inputs touch none.
  if (robot_config.simulation_only && !options_.simulation && starts_roster) {
    std::cerr << "launcher: " << options_.config_path
              << " sets [robot] simulation_only = true; run it with --sim "
                 "(or --describe-only); nothing was started\n";
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
    std::cerr << "launcher: --sim requires remote 127.0.0.1; effective remote "
              << effective_remote << "\n";
    return 1;
  }
  std::string restart_error;
  int max_restarts = kDefaultMaxRestarts;
  if (!ResolveMaxRestarts(robot_config, options_, max_restarts,
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
  restart_limits_ = {max_restarts,
                     std::chrono::milliseconds{options_.restart_window_ms}};

  auto node_specs = DiscoverNodes(robot_config, options_);
  if (options_.start_studio_bridge && BundleWithoutStudio(options_)) {
    std::cerr << "[launcher] this bundle carries no Studio (its talos_bundle "
                 "sets no studio_bridge and studio_webroot); session runs "
                 "without Studio\n";
  }
  if (node_specs.empty()) {
    std::cerr << "launcher: no nodes declared in config\n";
    return 1;
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
      std::cerr << "launcher: [subsystems." << node_specs[i].name
                << "] has the name of the launcher's own "
                << (added.studio_bridge
                        ? "Studio bridge, which --start-studio-bridge adds"
                        : "roster member of that name")
                << "; rename the subsystem\n";
      return 1;
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
    std::cerr << "launcher: roster member '" << spec.name << "': " << *problem
              << "; refused even with --allow-graph-errors, since the name "
                 "is its output and event log file names (the launcher's own "
                 "is "
              << kLauncherOutputFile << ")\n";
    return 1;
  }
  // Checked again right before retention lists the log root: another
  // launch may have taken the lock meanwhile.
  if (lock_needs_room && !take_lock()) {
    std::cerr << "launcher: " << lock_error << "\n";
    return 1;
  }
  const Pruning pruning = session_lock      ? Pruning::kLocked
                          : lock_needs_room ? Pruning::kUnlocked
                                            : Pruning::kNone;
  std::string retention_record;
  const uint64_t session_log_budget = ApplyRetention(
      robot_config, node_specs.size(), pruning, retention_record);
  if (lock_needs_room && (!take_lock() || !session_lock)) {
    std::cerr << "launcher: " << lock_error << "\n";
    ReportUnwrittenRetention(retention_record);
    return 1;
  }
  if (!CreateSessionDirectory(robot_config, node_specs.size(),
                              pruning != Pruning::kNone, retention_record)) {
    return 1;
  }
  launcher_output_->Record(options_.output_dir + "/" +
                           std::string{kLauncherOutputFile});
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
          node_specs.begin(), node_specs.end(),
          [](const NodeSpec& spec) { return !spec.studio_bridge; }));
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
    diagnostics = DescribeGraph(
        node_specs, options_, described, TargetSourcesOf(robot_config),
        TargetModeSourcesOf(robot_config), robot_config.require_target_coverage,
        starts_roster ? process::StopFd() : -1);
  } catch (const std::exception& e) {
    std::cerr << "launcher: " << e.what() << "\n";
    return 1;
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
    std::cerr << "launcher: cannot write " << graph_path << " (" << graph_error
              << "); nothing was started\n";
    return 1;
  }
  PrintDeclaredGraph(described, diagnostics, declared_topics, graph_path);
  // Its warning is in the graph above; an observer's absence stops nothing.
  DropUnansweredStudioBridge(node_specs, described);

  if (naming::HasError(diagnostics) && !options_.allow_graph_errors) {
    std::cerr << "launcher: the declared graph has errors; refusing to "
                 "start. Fix the errors above, or, in simulation, pass "
                 "--allow-graph-errors with --sim to launch anyway.\n";
    return 1;
  }
  if (options_.describe_only) {
    std::cout << "[launcher] describe-only: nothing was started\n";
    return 0;
  }
  if (stopped_before_start()) return 0;

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
    proc.binary_sha256 = FileSha256(spec.binary_path);
    // The gateway is started without --log, so it writes no event log.
    if (spec.name != kSimGatewayName) {
      proc.log_path = NodeLogPath(options_.output_dir, spec.name, 0);
    }
    proc.described = described[i].answered;
    if (spec.studio_bridge) {
      studio.emplace(i, spec, proc, robot_config, options_, described,
                     graph_path, *output_);
    }
    if (!proc.log_path.empty()) proc.log_paths.push_back(proc.log_path);
  }
  const LogSplit log_split =
      LogBudgets(procs, described, robot_config, session_log_budget);
  std::cout << "[launcher] Event-log window "
            << (log_split.window.count() > 0
                    ? std::to_string(log_split.window.count()) + " us"
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

  const std::string remote = EffectiveRemoteAddress(robot_config, options_);
  const uint16_t remote_port = EffectiveRemotePort(robot_config, options_);
  const uint16_t local_port = EffectiveLocalPort(robot_config, options_);
  std::cout << "[launcher] Starting session " << options_.session_id << " with "
            << procs.size() << " nodes...\n";
  std::cout << "[launcher] Controller " << remote << ":" << remote_port
            << " (local port " << local_port << ")\n";

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
    std::cout << "[launcher] Started " << proc.name << " (pid " << pid
              << ", target " << proc.target << ")\n";
    if (options_.member_spawned) options_.member_spawned(proc.name);
  }
  if (studio && !stop_skips_start())
    studio->Start(procs[studio->index()], options_.supervision_clock);
  PersistManifest(procs);

  Supervise(procs, launches, studio, deadline);
  manifest.end_wall_ns = WallClockNanos();
  // Every run has been reaped and its group killed, so the pipes end.
  output_->Stop();
  for (const auto& proc : procs) {
    const std::string path = NodeOutputPath(options_.output_dir, proc.name);
    if (const uint64_t dropped = output_->Dropped(path); dropped != 0)
      std::cerr << "[launcher] " << proc.name << ": " << dropped
                << " bytes of output dropped (marked in " << path << ")\n";
  }

  // Whose last exit leaves the status alone: a run the stop's own signal
  // ended (its manifest run's stopped_by_session, so the two never
  // disagree), the Studio bridge however it ended (an observer, REQ-OPS-30:
  // killed while still starting, crashed, or stopped), which the manifest
  // records all the same, and a member the stop left unstarted (no run).
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
  if (proc.name == kSimGatewayName) {
    // The gateway stamps the config's config_id; unconfigured it stamps
    // 0, which every mechanism rejects.
    args.insert(args.end(), {"--config", options_.config_path, "--session-id",
                             std::to_string(options_.session_id)});
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
  }
  // The hardware bridge's controller addresses are among them
  // (ControllerArgs), as they were in its probe.
  args.insert(args.end(), spec.extra_args.begin(), spec.extra_args.end());
  if (options_.duration_s > 0) {
    args.insert(args.end(),
                {"--duration-s", std::to_string(options_.duration_s)});
  }
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
  const auto spec =
      std::find_if(specs.begin(), specs.end(),
                   [&](const NodeSpec& s) { return s.name == options_.only; });
  if (spec == specs.end()) {
    std::cerr << "launcher: --only " << options_.only
              << " names no roster member of " << options_.config_path << "\n";
    return 1;
  }
  if (spec->name == kHardwareNodeName || spec->name == kSimGatewayName ||
      spec->studio_bridge) {
    std::cerr << "launcher: " << spec->name
              << " is not a talos node and has no input replay\n";
    return 1;
  }

  const std::string log_path = NodeLogPath(options_.output_dir, spec->name, 0);
  // The node runs as it ran in the session: --sim changes what it accepts
  // (the devices layer treats the config as commissioned, which is part of
  // the config id every recorded State carries). The session's manifest
  // says; without one, this launch's --sim does.
  const bool simulation =
      SessionSimulation(options_.replay_inputs).value_or(options_.simulation);
  std::cout << "[launcher] Replaying session " << options_.replay_inputs
            << " into " << spec->name << " (" << spec->target << ")"
            << (simulation ? " under --sim" : "") << "; log " << log_path
            << "\n";
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
  std::string error;
  const pid_t pid = SpawnProcess(std::move(args), error);
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
  if (!installed_) {
    for (std::size_t i = 0; i < kSignals.size(); ++i)
      ::sigaction(kSignals[i], nullptr, &saved_[i]);
    installed_ = true;
  }
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
                      /*echo=*/true);
}

std::chrono::milliseconds Launcher::ScheduleRestart(NodeProcessInfo& proc,
                                                    NodeLaunch& launch,
                                                    SupervisionTime now) {
  const auto delay = launch.restart.Schedule(now, restart_limits_);
  ShowRestart(proc, launch.restart);
  return delay;
}

void Launcher::ShowRestart(NodeProcessInfo& proc,
                           const RestartSchedule& restart) {
  proc.quarantined = restart.quarantined();
  // steady_clock's epoch, which introspect::monotonic_now_ns() shares.
  proc.retry_at_monotonic_ns =
      restart.pending() ? std::chrono::duration_cast<std::chrono::nanoseconds>(
                              restart.due_at().time_since_epoch())
                              .count()
                        : 0;
}

void Launcher::NodeLaunch::NewRun(SupervisionTime spawned) {
  liveness.Begin(spawned);
  kill_reason = {};
  heartbeat_killed = false;
  endpoints_checked = false;
  realtime_reported = false;
}

void Launcher::SpawnFailed(NodeProcessInfo& proc, NodeLaunch& launch,
                           SupervisionTime now, const std::string& error) {
  proc.failed = true;
  proc.RecordSpawnFailure();
  std::cerr << "[launcher] cannot spawn " << proc.name << " (" << error
            << "); ";
  // Never `stopping` here: Supervise cancels a pending restart once the
  // session stops.
  if (!ShouldRestartNode(/*clean_exit=*/false, launch.restart.critical(),
                         restart_limits_.max_restarts, /*stopping=*/false)) {
    std::cerr << "restarts are disabled\n";
    return;
  }
  const auto delay = ScheduleRestart(proc, launch, now);
  std::cerr << (proc.quarantined ? "quarantined; retry" : "retry") << " in "
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
    std::cout << "[launcher] " << proc.name << " retrying after quarantine\n";
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
  if (!warning.empty()) std::cerr << warning;
  return proc.RecordRealtime(*outcome);
}

bool Launcher::NoteRecording(NodeProcessInfo& proc, NodeLaunch& launch,
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
  std::cerr << "[launcher] error: " << proc.name << " (pid " << proc.pid
            << "): live endpoints differ from its --describe probe, the "
               "graph the lint checked: "
            << mismatch << "; ";
  if (options_.allow_graph_errors) {
    std::cerr << "running anyway (--allow-graph-errors)\n";
  } else if (!launch.restart.critical()) {
    launch.endpoints_stopped = true;
    launch.heartbeat_killed = true;
    launch.kill_reason = kRunEndpointMismatch;
    SignalProcessGroup(proc.pid, SIGKILL);
    std::cerr << "non-critical: killing it, without restarts this "
                 "session\n";
  } else if (proc.restart_count == 0) {
    endpoints_refused_ = true;
    launch.kill_reason = kRunEndpointMismatch;
    std::cerr << "stopping the session. A node must declare the same "
                 "endpoints whether or not Node::describing(); in "
                 "simulation, --sim --allow-graph-errors runs anyway.\n";
  } else {
    std::cerr << "a critical member's restart: the session keeps running\n";
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
      DeliveryCounts loss = DeliveryCountsOf(record);
      if (record.generation.load(std::memory_order_acquire) != generation ||
          record.state.load(std::memory_order_acquire) !=
              static_cast<uint64_t>(introspect::SlotState::ACTIVE))
        continue;
      live = std::move(published);
      delivery = std::move(loss);
      recording = flags;
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
    std::cerr << "[launcher] " << proc.name
              << (check.kill == kRunConstructionDeadline
                      ? " construction deadline exceeded"
                  : check.kill == kRunHeartbeatMissing
                      ? " heartbeat missing"
                      : StallReport(launch, constructing, check.judged_at))
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
                         std::chrono::steady_clock::time_point deadline) {
  bool stop_sent = false;
  auto shutdown_deadline = std::chrono::steady_clock::time_point::max();
  std::optional<introspect::RegistryMapping> registry;
  // Every first run's liveness counts from here, not its spawn.
  const auto started = options_.supervision_clock();
  for (auto& launch : launches) launch.NewRun(started);
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
        std::cerr << "[launcher] the session lock " << session_lock_->path()
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
        std::cerr << "[launcher] locked " << session_directory_->path()
                  << ", which could not be locked when the session started; "
                     "retention passes over this session from now on\n";
      } else if (directory == Refreshed::kRetaken &&
                 !directory_lock_retaken_reported) {
        directory_lock_retaken_reported = true;
        std::cerr << "[launcher] " << session_directory_->path()
                  << " was removed while this session held it; locked it "
                     "again on a new file, so retention passes over this "
                     "session again\n";
      } else if (directory == Refreshed::kLost &&
                 !directory_lock_lost_reported) {
        directory_lock_lost_reported = true;
        std::cerr << "[launcher] " << session_directory_->path()
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
      // Every member but Studio's bridge, which takes no registry row, is
      // supervised by its row; the simulation gateway registers like a node.
      if (!stopping && !launch.heartbeat_killed &&
          !(studio && i == studio->index())) {
        if (WatchRun(proc, launch, registry ? &*registry : nullptr))
          manifest_changed = true;
      }
      int status = 0;
      if (::waitpid(proc.pid, &status, WNOHANG) != proc.pid) continue;
      FinishProcessGroup(proc.pid);
      // The run's counts as it ended, before EndRun closes the run. Studio's
      // bridge takes no row.
      if (registry && !(studio && i == studio->index())) {
        if (const auto loss =
                FinalDeliveryCounts(*registry, proc.pid, options_.session_id,
                                    launch.spawned_monotonic_ns))
          proc.RecordDeliveryLoss(*loss);
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
      proc.EndRun(RunEndReason(launch.kill_reason, stop_arrived(), status),
                  EndedByOwnStopSignal(launch.kill_reason, status));
      const bool is_bridge = studio && i == studio->index();
      if (is_bridge && studio->Starting(i)) continue;
      launch.restart.RunEnded(clean);
      if (!is_bridge && !launch.endpoints_stopped &&
          ShouldRestartNode(clean, launch.restart.critical(),
                            restart_limits_.max_restarts, stop_arrived())) {
        const auto delay = ScheduleRestart(proc, launch, now);
        std::cout << "[launcher] " << proc.name << " exited ("
                  << ExitReason(status) << "); "
                  << (proc.quarantined ? "quarantined; retry" : "restart")
                  << " " << proc.restart_count + 1 << " in " << delay.count()
                  << " ms\n";
        continue;
      }
      std::cout << "[launcher] Process " << proc.name << " (pid " << proc.pid
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
          std::cout << "[launcher] " << procs[i].name
                    << " finished with the rest of the roster; not "
                       "restarted\n";
        } else {
          Restart(procs[i], launch, now);
        }
        manifest_changed = true;
      }
    }
    if (studio) {
      const auto& before = procs[studio->index()];
      const auto pid = before.pid;
      const bool exited = before.exited;
      const bool failed = before.failed;
      studio->Supervise(procs[studio->index()], now, stop_arrived());
      const auto& after = procs[studio->index()];
      if (pid != after.pid || exited != after.exited || failed != after.failed)
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
      std::cout << "[launcher] Stopping children...\n";
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
    if (studio) due(studio->NextCheck());
    wakeup.Wait(wake == SupervisionTime::max()
                    ? std::chrono::nanoseconds::max()
                    : wake - options_.supervision_clock());
  }
}

}  // namespace talos::launcher
