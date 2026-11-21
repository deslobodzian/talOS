#include "talOS/launcher/studio_bridge.h"

#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <csignal>
#include <cstddef>
#include <iostream>
#include <optional>
#include <string_view>

#include "talOS/launcher/console_line.h"
#include "talOS/launcher/spawn.h"

namespace talos::launcher {
namespace {

constexpr std::string_view kStudioListeningPrefix = "listening ";

std::optional<uint16_t> ParseListeningPort(std::string_view line) {
  if (!line.starts_with(kStudioListeningPrefix)) return std::nullopt;
  const std::string_view value = line.substr(kStudioListeningPrefix.size());
  int port = 0;
  const auto* end = value.data() + value.size();
  const auto [ptr, ec] = std::from_chars(value.data(), end, port);
  if (value.empty() || ec != std::errc{} || ptr != end || port < 1 ||
      port > 65535) {
    return std::nullopt;
  }
  return static_cast<uint16_t>(port);
}

}  // namespace

bool DropUnansweredStudioBridge(std::vector<NodeSpec>& specs,
                                std::vector<NodeDescription>& described) {
  for (std::size_t i = 0; i < specs.size() && i < described.size(); ++i) {
    if (!specs[i].studio_bridge || described[i].answered) continue;
    specs.erase(specs.begin() + static_cast<std::ptrdiff_t>(i));
    described.erase(described.begin() + static_cast<std::ptrdiff_t>(i));
    return true;
  }
  return false;
}

std::string StudioBridgeWatchTopic(const NodeDescription& bridge,
                                   const config::RobotConfig& robot_config) {
  std::string topic = robot_config.studio.topic;
  if (!bridge.answered) return topic;
  for (const auto& source : bridge.declared.sources) {
    if (source.kind == event::SourceKind::WATCHER && !source.name.empty()) {
      topic = source.name;
    }
  }
  return topic;
}

StudioBridge::StudioBridge(std::size_t index, const NodeSpec& spec,
                           NodeProcessInfo& proc,
                           const config::RobotConfig& robot_config,
                           const LauncherOptions& options,
                           const RestartLimits& limits,
                           const std::vector<NodeDescription>& described,
                           const std::string& graph_path, ChildOutput& output)
    : index_{index},
      target_{spec.target},
      log_path_{NodeOutputPath(options.output_dir, kStudioBridgeName)},
      output_{output},
      session_id_{options.session_id},
      http_port_{EffectiveStudioHttpPort(robot_config, options)},
      args_{spec.binary_path,
            StudioBridgeWatchTopic(described[index], robot_config),
            std::to_string(http_port_),
            EffectiveStudioWebroot(robot_config, options),
            "--session-id",
            std::to_string(options.session_id),
            "--config",
            options.config_path,
            "--declared",
            graph_path,
            "--manifest",
            options.output_dir + "/manifest.json"},
      limits_{limits},
      liveness_limits_{
          HeartbeatTimeout(
              std::chrono::microseconds{kStudioBridgePeriod}.count(),
              options.heartbeat_periods),
          std::chrono::milliseconds{options.heartbeat_startup_grace_ms},
          options.construction_timeout} {
  proc.log_path.clear();
}

void StudioBridge::NewRun(SupervisionTime spawned) {
  liveness_.Begin(spawned);
  kill_reason_ = {};
  heartbeat_killed_ = false;
  port_observed_ = false;
  started_port_announced_ = false;
}

void StudioBridge::BeginSupervision(SupervisionTime started) {
  if (running_) liveness_.Begin(started);
}

void StudioBridge::Start(NodeProcessInfo& proc, const SupervisionClock& clock) {
  const std::string& binary = args_.front();
  if (binary.empty() || ::access(binary.c_str(), X_OK) != 0) {
    ConsoleLine(std::cerr) << "[launcher] studio_bridge not executable at '"
                           << (binary.empty() ? target_ : binary)
                           << "'; session runs without Studio\n";
    proc.RecordSpawnFailure();
    Leave(proc);
    return;
  }
  ConsoleLine(std::cout) << "[launcher] Starting studio_bridge (target "
                         << target_ << ")...\n";
  std::string error;
  spawned_monotonic_ns_ = introspect::monotonic_now_ns();
  const pid_t pid = Spawn(error);
  const SupervisionTime now = clock();
  if (pid < 0) {
    proc.RecordSpawnFailure();
    Failed(proc, "cannot spawn studio_bridge (" + error + ")", now);
    return;
  }
  proc.pid = pid;
  proc.BeginRun(pid);
  last_spawn_ = now;
  running_ = true;
  settled_ = false;
  NewRun(now);
}

bool StudioBridge::Watch(NodeProcessInfo& proc,
                         const introspect::RegistryMapping* registry,
                         const SupervisionClock& clock, bool stopping) {
  if (!running_ || heartbeat_killed_ || stopping || proc.pid <= 0 ||
      proc.exited) {
    return false;
  }
  uint32_t row_flags = 0;
  const auto read_row = [&]() -> RowObservation {
    if (registry == nullptr) return {};
    for (const auto& record : registry->header()->nodes) {
      if (record.state.load(std::memory_order_acquire) !=
          static_cast<uint64_t>(introspect::SlotState::ACTIVE))
        continue;
      const auto generation = record.generation.load(std::memory_order_acquire);
      if (record.pid != static_cast<uint64_t>(proc.pid) ||
          record.session_id != session_id_)
        continue;
      const auto beat =
          record.heartbeat_monotonic_ns.load(std::memory_order_relaxed);
      if (beat < spawned_monotonic_ns_) continue;
      const auto dispatch =
          record.dispatch_count.load(std::memory_order_relaxed);
      const auto flags = record.flags.load(std::memory_order_acquire);
      if (record.generation.load(std::memory_order_acquire) != generation ||
          record.state.load(std::memory_order_acquire) !=
              static_cast<uint64_t>(introspect::SlotState::ACTIVE))
        continue;
      row_flags = flags;
      return {.found = true,
              .constructing = (flags & introspect::kFlagConstructing) != 0,
              .heartbeat = beat,
              .dispatch = dispatch};
    }
    return {};
  };
  const LivenessCheck check =
      liveness_.Watch(clock, read_row, liveness_limits_);
  bool changed = false;
  if (!port_observed_) {
    if (const auto line = output_.MatchedLine(log_path_, proc.pid)) {
      port_observed_ = true;
      if (const auto port = ParseListeningPort(*line)) {
        http_port_ = *port;
        args_[2] = std::to_string(*port);
        if (proc.http_port != port) {
          proc.http_port = port;
          changed = true;
        }
        if (settled_ && !started_port_announced_) {
          started_port_announced_ = true;
          ConsoleLine(std::cout)
              << "[launcher] Started studio_bridge (pid " << proc.pid
              << ") on http://localhost:" << http_port_ << "\n";
        }
      }
    }
  }
  if (const auto outcome = introspect::RealtimeOf(row_flags)) {
    if (proc.RecordRealtime(*outcome)) changed = true;
  }
  if (!check.kill.empty()) {
    const auto ms = [now = check.judged_at](SupervisionTime since) {
      return std::to_string(
          std::chrono::duration_cast<std::chrono::milliseconds>(now - since)
              .count());
    };
    const std::string detail =
        check.kill == kRunConstructionDeadline
            ? " construction deadline exceeded"
        : check.kill == kRunHeartbeatMissing
            ? " heartbeat missing"
            : " stalled: dispatch count unchanged for " +
                  ms(liveness_.progress_changed_at()) + " ms (heartbeat " +
                  (check.judged_at - liveness_.heartbeat_changed_at() <
                           liveness_limits_.heartbeat_timeout
                       ? "still moving)"
                       : "unchanged too, for " +
                             ms(liveness_.heartbeat_changed_at()) + " ms)");
    ConsoleLine(std::cerr) << "[launcher] studio_bridge" << detail
                           << "; killing pid " << proc.pid << "\n";
    SignalProcessGroup(proc.pid, SIGKILL);
    heartbeat_killed_ = true;
    kill_reason_ = check.kill;
  }
  return changed;
}

void StudioBridge::Exited(NodeProcessInfo& proc, int status,
                          SupervisionTime now, bool stopping) {
  running_ = false;
  settled_ = false;
  heartbeat_killed_ = false;
  port_observed_ = false;
  started_port_announced_ = false;
  kill_reason_ = {};
  if (stopping) {
    ConsoleLine(std::cout) << "[launcher] Process studio_bridge (pid "
                           << proc.pid << ") exited (" << ExitReason(status)
                           << ")\n";
    return;
  }
  Failed(proc, "studio_bridge exited (" + ExitReason(status) + ")", now);
}

void StudioBridge::Supervise(NodeProcessInfo& proc, SupervisionTime now,
                             const SupervisionClock& clock, bool stopping) {
  if (running_ && !settled_ &&
      now - last_spawn_ >= kStudioBridgeSettleInterval) {
    settled_ = true;
    came_up_ = true;
    if (http_port_ != 0 && !started_port_announced_) {
      started_port_announced_ = true;
      ConsoleLine(std::cout)
          << "[launcher] Started studio_bridge (pid " << proc.pid
          << ") on http://localhost:" << http_port_ << "\n";
    }
  }
  if (!restart_.pending()) return;
  // Nothing is started once the stop has begun.
  if (stopping) {
    restart_.Cancel();
    ShowRestart(proc, restart_);
  } else if (restart_.Due(now)) {
    Restart(proc, now, clock);
  }
}

SupervisionTime StudioBridge::NextCheck(bool stopping) const {
  SupervisionTime next = SupervisionTime::max();
  if (running_ && !settled_) next = last_spawn_ + kStudioBridgeSettleInterval;
  if (running_ && !stopping && !heartbeat_killed_)
    next = std::min(next, liveness_.NextLook(liveness_limits_));
  if (restart_.pending()) next = std::min(next, restart_.due_at());
  return next;
}

void StudioBridge::Finish(NodeProcessInfo& proc, bool stop_requested) {
  if (restart_.pending()) {
    restart_.Cancel();
    ShowRestart(proc, restart_);
  }
  if (running_ && !settled_ && proc.pid > 0 && !proc.exited) {
    SignalProcessGroup(proc.pid, SIGKILL);
    int status = 0;
    while (::waitpid(proc.pid, &status, 0) < 0 && errno == EINTR) {
    }
    FinishProcessGroup(proc.pid);
    proc.exited = true;
    proc.exit_signal = WIFSIGNALED(status) ? WTERMSIG(status) : 0;
    proc.EndRun(kRunStop);
    running_ = false;
  }
  // A session that ended on its deadline or its roster's completion before
  // any run came up never served Studio; a requested stop is no failure.
  if (!stop_requested && !came_up_) Leave(proc);
}

pid_t StudioBridge::Spawn(std::string& error) {
  return SpawnDrained(args_, error, output_, log_path_, kStudioBridgeName,
                      /*echo=*/false, /*hold_stop=*/false,
                      std::string{kStudioListeningPrefix});
}

void StudioBridge::Restart(NodeProcessInfo& proc, SupervisionTime now,
                           const SupervisionClock& clock) {
  // The attempt is a restart whether or not it spawns: it counts against the
  // window and in restart_count, so the restart lines, the give-up line's
  // count and the manifest agree even when a spawn fails.
  restart_.Started(now);
  ShowRestart(proc, restart_);
  ++proc.restart_count;
  std::string error;
  spawned_monotonic_ns_ = introspect::monotonic_now_ns();
  const pid_t pid = Spawn(error);
  if (pid < 0) {
    proc.RecordSpawnFailure();
    Failed(proc, "cannot spawn studio_bridge (" + error + ")", now);
    return;
  }
  const SupervisionTime spawned = clock();
  proc.pid = pid;
  proc.exited = false;
  proc.exit_code = -1;
  proc.exit_signal = 0;
  proc.BeginRun(pid);
  last_spawn_ = spawned;
  running_ = true;
  settled_ = false;
  NewRun(spawned);
}

void StudioBridge::Failed(NodeProcessInfo& proc, std::string_view what,
                          SupervisionTime now) {
  // A server that ends before the stop has failed, whatever its status.
  if (!ShouldRestartNode(/*clean_exit=*/false, restart_.critical(),
                         limits_.max_restarts, /*stopping=*/false)) {
    ConsoleLine(std::cerr)
        << "[launcher] " << what
        << "; restarts are disabled (--max-restarts 0): studio_bridge "
           "will not be restarted again; session runs without Studio "
           "(see "
        << log_path_ << ")\n";
    Leave(proc);
    return;
  }
  restart_.RunEnded(/*clean=*/false);
  const auto delay = restart_.Schedule(now, limits_);
  if (restart_.quarantined()) {
    // Where a node would wait out the window, an observer is left.
    restart_.Cancel();
    ShowRestart(proc, restart_);
    ConsoleLine(std::cerr)
        << "[launcher] " << what << "; " << limits_.max_restarts
        << " restarts within " << limits_.window.count()
        << " ms: studio_bridge will not be restarted again; session "
           "runs without Studio (see "
        << log_path_ << ")\n";
    Leave(proc);
    return;
  }
  ShowRestart(proc, restart_);
  ConsoleLine(std::cout) << "[launcher] " << what << "; restart "
                         << proc.restart_count + 1 << " in " << delay.count()
                         << " ms\n";
}

void StudioBridge::Leave(NodeProcessInfo& proc) { proc.failed = true; }

}  // namespace talos::launcher
