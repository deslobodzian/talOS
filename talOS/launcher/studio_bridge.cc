#include "talOS/launcher/studio_bridge.h"

#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstddef>
#include <iostream>

#include "talOS/launcher/spawn.h"

namespace talos::launcher {

bool ShouldRetryStudioBridge(
    int attempts_made, std::chrono::steady_clock::time_point first_attempt,
    std::chrono::steady_clock::time_point last_spawn,
    std::chrono::steady_clock::time_point now, bool stopping) {
  if (stopping) return false;
  if (attempts_made >= kStudioBridgeMaxAttempts) return false;
  if (now - first_attempt >= kStudioBridgeStartupWindow) return false;
  return now - last_spawn >= kStudioBridgeRetryInterval;
}

bool DropUnansweredStudioBridge(std::vector<NodeSpec>& specs,
                                std::vector<NodeDescription>& described) {
  for (std::size_t i = 0; i < specs.size() && i < described.size(); ++i) {
    if (specs[i].name != kStudioBridgeName || described[i].answered) continue;
    specs.erase(specs.begin() + static_cast<std::ptrdiff_t>(i));
    described.erase(described.begin() + static_cast<std::ptrdiff_t>(i));
    return true;
  }
  return false;
}

std::string StudioBridgeWatchTopic(
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

StudioBridge::StudioBridge(std::size_t index, const NodeSpec& spec,
                           NodeProcessInfo& proc,
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
            EffectiveStudioWebroot(robot_config, options),
            "--config",
            options.config_path,
            "--declared",
            graph_path,
            "--manifest",
            options.output_dir + "/manifest.json"} {
  proc.log_path.clear();
}

void StudioBridge::Start(NodeProcessInfo& proc, const SupervisionClock& clock) {
  const std::string& binary = args_.front();
  if (binary.empty() || ::access(binary.c_str(), X_OK) != 0) {
    std::cerr << "[launcher] studio_bridge not executable at '"
              << (binary.empty() ? target_ : binary)
              << "'; session runs without Studio\n";
    proc.failed = true;
    proc.RecordSpawnFailure();
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
    proc.RecordSpawnFailure();
    return;
  }
  const auto now = clock();
  proc.pid = pid;
  proc.BeginRun(pid);
  attempts_ = 1;
  first_attempt_ = now;
  last_spawn_ = now;
  active_ = true;
}

void StudioBridge::Supervise(NodeProcessInfo& proc, Clock::time_point now,
                             bool stopping) {
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
      proc.RecordSpawnFailure();
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
    proc.BeginRun(pid);
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

void StudioBridge::Finish(NodeProcessInfo& proc, bool stop_requested) {
  if (!active_ || stable_) return;
  if (proc.pid > 0 && !proc.exited) {
    SignalProcessGroup(proc.pid, SIGKILL);
    int status = 0;
    while (::waitpid(proc.pid, &status, 0) < 0 && errno == EINTR) {
    }
    FinishProcessGroup(proc.pid);
    proc.exited = true;
    proc.exit_signal = WIFSIGNALED(status) ? WTERMSIG(status) : 0;
    proc.EndRun(kRunStop);
  }
  if (!stop_requested) GiveUp(proc);
}

pid_t StudioBridge::Spawn(std::string& error) {
  return SpawnDrained(args_, error, output_, log_path_, kStudioBridgeName,
                      /*echo=*/false);
}

void StudioBridge::GiveUp(NodeProcessInfo& proc) {
  proc.failed = true;
  proc.exit_code = -1;
  active_ = false;
}

}  // namespace talos::launcher
