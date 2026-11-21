#pragma once

#include <sys/types.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/introspection/registry.h"
#include "talOS/launcher/child_output.h"
#include "talOS/launcher/describe_graph.h"
#include "talOS/launcher/roster.h"
#include "talOS/launcher/session.h"
#include "talOS/launcher/supervision_policy.h"

namespace talos::launcher {

// ---------------------------------------------------------------------------
// Studio bridge
//
// The one roster member supervised differently. It takes positional args
// rather than the node flags (plus --session-id, --config, --declared and
// --manifest) and logs plaintext to studio_bridge.log rather than a .tlog. It
// claims a live registry row (observer tier, advancing heartbeat_monotonic_ns
// and dispatch_count on every loop pass), so the launcher watches it with
// RunLiveness (period kStudioBridgePeriod) and kills and restarts a hung or
// unregistered bridge on the same bounded schedule as an exit. It is an
// observer: nothing it does starts, stops, restarts or delays another
// member, and how it ended never sets the session's exit status.
//
// Every ending of a run while the session is not stopping -- a nonzero
// exit, a signal, a liveness kill (stalled or heartbeat_missing), or exit 0,
// since a server that ends before the stop has failed -- and every spawn that
// fails is restarted on the restart schedule a non-critical member has
// (RestartSchedule: 50 ms, doubling to 1 s, at most max_restarts starts per
// rolling restart window, inside the supervision loop), with the restart line
// a node's restart prints. Where a node would be quarantined, the bridge is
// left: one line says it will not be restarted again, it is marked failed,
// and the session runs on without Studio. --max-restarts 0 leaves it at its
// first ending. A stop cancels a pending restart, so no bridge is spawned
// once the stop has begun.
//
// A run counts as up once it has run for kStudioBridgeSettleInterval: until
// then, and while a restart is pending, the bridge does not hold the session
// open, and a run still settling when the session ends is SIGKILLed.
// ---------------------------------------------------------------------------

inline constexpr std::chrono::seconds kStudioBridgeSettleInterval{1};
// Matches studio_bridge's <= 1 s wake cadence (250 ms graph refresh and 1 s
// attached segment check, even with zero clients and zero telemetry).
inline constexpr std::chrono::seconds kStudioBridgePeriod{1};

// Removes Studio's bridge from `specs` and `described` (index for index)
// when its --describe probe did not answer, missing binary included:
// DescribeGraph warned (StudioBridgeFinding) and the session runs without
// Studio, as REQ-OPS-30 asks of an observer. A bridge that answered stays,
// supervised as StudioBridge says. Returns whether it removed one.
bool DropUnansweredStudioBridge(std::vector<NodeSpec>& specs,
                                std::vector<NodeDescription>& described);

// The topic the bridge watches, from `bridge`, its own description: the
// watcher its --describe declared (it reads the `[studio] topic` key the
// telemetry node publishes on), or that key itself when its description
// declares none. A bridge that did not answer never gets here: Run
// constructs StudioBridge only after DropUnansweredStudioBridge has removed
// one.
std::string StudioBridgeWatchTopic(const NodeDescription& bridge,
                                   const config::RobotConfig& robot_config);

class StudioBridge {
 public:
  // `proc` is the bridge's roster entry, and described[index] its
  // description; it keeps no .tlog. `limits` are the session's restart
  // limits, which the bridge's restarts count against as a node's do.
  StudioBridge(std::size_t index, const NodeSpec& spec, NodeProcessInfo& proc,
               const config::RobotConfig& robot_config,
               const LauncherOptions& options, const RestartLimits& limits,
               const std::vector<NodeDescription>& described,
               const std::string& graph_path, ChildOutput& output);

  std::size_t index() const { return index_; }

  // A run of the bridge is live but not yet up a settle interval: its exit
  // is expected to be possible, and the session does not wait for it.
  bool Starting(std::size_t index) const {
    return index == index_ && running_ && !settled_;
  }

  // The first run, spawned alongside every other node without waiting, at
  // the time `clock` reads after the spawn.
  void Start(NodeProcessInfo& proc, const SupervisionClock& clock);

  // Aligns the first run's liveness origin with Supervise's start time.
  void BeginSupervision(SupervisionTime started);

  // One supervision pass's look at the bridge: captures its bound port from
  // `listening <port>` (updating `proc.http_port` and `args_[2]` so restarts
  // rebind the same port), and inspects its live registry row (read-only,
  // lock-free), recording its realtime outcome (without warning for an
  // observer) and killing a hung or unregistered run with SIGKILL. Returns
  // whether `proc` changed.
  bool Watch(NodeProcessInfo& proc, const introspect::RegistryMapping* registry,
             const SupervisionClock& clock, bool stopping);

  // Why Watch killed the current run (kRunStalled, kRunHeartbeatMissing, ...),
  // or empty when it has not.
  std::string_view kill_reason() const { return kill_reason_; }

  // The supervision loop reaped the current run at `now` (waitpid `status`;
  // the run already ended in `proc`): schedules its restart, or leaves the
  // bridge, unless the session is `stopping`.
  void Exited(NodeProcessInfo& proc, int status, SupervisionTime now,
              bool stopping);

  // One supervision pass, after the loop has reaped exits: marks a run that
  // has been up a settle interval as up, cancels a pending restart once the
  // session is `stopping`, and starts one that is due.
  void Supervise(NodeProcessInfo& proc, SupervisionTime now,
                 const SupervisionClock& clock, bool stopping);

  // When Supervise or Watch next has something to decide without an exit: the
  // settle interval's end of a run still settling, the liveness look deadline
  // while running and not `stopping`, and a pending restart's start;
  // time_point::max() for none.
  SupervisionTime NextCheck(bool stopping = false) const;

  // The session ended. A pending restart is dropped; a run still settling
  // is killed and reaped, so its run ends with the session. A bridge none
  // of whose runs came up is marked failed unless the session ended on a
  // requested stop (`stop_requested`), which is not a bridge failure.
  void Finish(NodeProcessInfo& proc, bool stop_requested);

 private:
  // One run, its output drained into log_path_ only, never echoed.
  pid_t Spawn(std::string& error);

  // Starts liveness tracking for a freshly spawned run.
  void NewRun(SupervisionTime spawned);

  // Starts the pending restart at `now`; one that cannot be spawned fails
  // like a run.
  void Restart(NodeProcessInfo& proc, SupervisionTime now,
               const SupervisionClock& clock);

  // A run ended, or a spawn failed (`what`, for the line), at `now` while
  // the session runs: schedules the next start, or leaves the bridge.
  void Failed(NodeProcessInfo& proc, std::string_view what,
              SupervisionTime now);

  // Not restarted again this session: marked failed.
  void Leave(NodeProcessInfo& proc);

  std::size_t index_;
  std::string target_;
  std::string log_path_;
  ChildOutput& output_;
  uint64_t session_id_;
  uint16_t http_port_;
  std::vector<std::string> args_;
  RestartLimits limits_;
  LivenessLimits liveness_limits_;
  RunLiveness liveness_;
  // Its restarts: one schedule, a non-critical member's.
  RestartSchedule restart_{/*critical=*/false};
  SupervisionTime last_spawn_{};
  int64_t spawned_monotonic_ns_{0};
  std::string_view kill_reason_;
  bool heartbeat_killed_{false};
  bool port_observed_{false};
  bool started_port_announced_{false};
  bool running_{false};  // A run was spawned and is not yet reaped.
  bool settled_{false};  // That run has been up a settle interval.
  bool came_up_{false};  // Some run has.
};

}  // namespace talos::launcher
