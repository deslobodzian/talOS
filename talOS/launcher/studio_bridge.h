#pragma once

#include <sys/types.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/launcher/child_output.h"
#include "talOS/launcher/describe_graph.h"
#include "talOS/launcher/roster.h"
#include "talOS/launcher/session.h"

namespace talos::launcher {

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
// except that a crash is not restarted (see Restarts in supervision_policy.h).
// One that never comes up is a degraded session, not a broken robot: it is
// marked failed and the session runs on without Studio.
// ---------------------------------------------------------------------------

inline constexpr int kStudioBridgeMaxAttempts = 60;
inline constexpr std::chrono::seconds kStudioBridgeRetryInterval{1};
inline constexpr std::chrono::seconds kStudioBridgeStartupWindow{60};

// True while the bridge still has startup budget, its last spawn is at least
// one interval old, and the session is not stopping.
bool ShouldRetryStudioBridge(
    int attempts_made, std::chrono::steady_clock::time_point first_attempt,
    std::chrono::steady_clock::time_point last_spawn,
    std::chrono::steady_clock::time_point now, bool stopping);

// The topic the bridge watches: what it described (its --describe reads the
// `[studio] topic` key the telemetry node publishes on), or that key itself
// when the bridge did not answer.
std::string StudioBridgeWatchTopic(
    const std::vector<NodeDescription>& described,
    const config::RobotConfig& robot_config);

class StudioBridge {
 public:
  using Clock = std::chrono::steady_clock;

  // `proc` is the bridge's roster entry; it keeps no .tlog.
  StudioBridge(std::size_t index, const NodeSpec& spec, NodeProcessInfo& proc,
               const config::RobotConfig& robot_config,
               const LauncherOptions& options,
               const std::vector<NodeDescription>& described,
               const std::string& graph_path, ChildOutput& output);

  std::size_t index() const { return index_; }

  // Still racing telemetry: exits are expected and the session does not wait.
  bool Starting(std::size_t index) const {
    return index == index_ && active_ && !stable_;
  }

  // The first attempt, spawned alongside every other node without waiting.
  void Start(NodeProcessInfo& proc);

  // One supervision tick, after the loop has reaped exits: stabilize a
  // survivor, respawn a startup failure, or give up when budget runs out.
  void Supervise(NodeProcessInfo& proc, Clock::time_point now, bool stopping);

  // The session ended while the bridge was still starting: it never served
  // Studio. An attempt still running is killed and reaped, so its run ends
  // with the session. A stop-driven ending is not a bridge failure.
  void Finish(NodeProcessInfo& proc, bool stop_requested);

 private:
  // One attempt, its output drained into log_path_ only, never echoed.
  pid_t Spawn(std::string& error);

  void GiveUp(NodeProcessInfo& proc);

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

}  // namespace talos::launcher
