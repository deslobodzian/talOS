#pragma once

// Supervision decisions as functions of time: a member's restart backoff,
// rolling budget and quarantine (RestartSchedule), and the stall,
// missing-row and construction kills of one run (RunLiveness). Nothing here
// reads a real clock: each call takes the `now` its supervision pass sampled
// from the supervision clock (SupervisionClock), or, for a run's liveness
// (RunLiveness::Watch), that clock itself.
//
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

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <string_view>

namespace talos::launcher {

using SupervisionTime = std::chrono::steady_clock::time_point;
// The supervisor's one clock (LauncherOptions::supervision_clock):
// steady_clock in production, whose epoch introspect::monotonic_now_ns()
// shares; a scripted one in tests.
using SupervisionClock = std::function<SupervisionTime()>;

inline constexpr int kDefaultMaxRestarts = 5;
inline constexpr std::chrono::milliseconds kRestartInitialBackoff{50};
inline constexpr std::chrono::milliseconds kRestartMaxBackoff{1000};
inline constexpr std::chrono::milliseconds kMinHeartbeatTimeout{500};

// The wait before restart `restart` (1 for the first in the rolling window)
// of a `critical` member or not, after a `clean_exit` or a crash.
std::chrono::milliseconds RestartBackoff(int restart, bool clean_exit,
                                         bool critical);

// Requested stop, the session deadline and launcher shutdown (`stopping`)
// never restart; zero disables all retries. Crashes remain retryable after a
// budget quarantine. A clean exit restarts only a critical member.
bool ShouldRestartNode(bool clean_exit, bool critical, int max_restarts,
                       bool stopping);

// max(kMinHeartbeatTimeout, period_us x heartbeat_periods).
std::chrono::nanoseconds HeartbeatTimeout(int64_t period_us,
                                          int heartbeat_periods);

struct RestartLimits {
  int max_restarts;                  // 0 disables every restart
  std::chrono::milliseconds window;  // the rolling restart window
};

// One roster member's restarts across its runs.
class RestartSchedule {
 public:
  explicit RestartSchedule(bool critical = true) : critical_{critical} {}
  bool critical() const { return critical_; }
  // Its latest run ended, exiting 0 or not (RosterContinues reads it).
  void RunEnded(bool clean) { last_exit_clean_ = clean; }
  bool last_exit_clean() const { return last_exit_clean_; }
  // Only for a restart ShouldRestartNode allows (a crash, a critical clean
  // exit, or a start that could not be spawned): forgets starts that have
  // left the window ending at `now`, then schedules the next start
  // RestartBackoff(starts left + 1, last_exit_clean(), critical()) after
  // `now` or, for a non-critical member whose window already holds
  // max_restarts starts, after the oldest of them plus the window
  // (quarantined). Returns the delay from `now`.
  std::chrono::milliseconds Schedule(SupervisionTime now,
                                     const RestartLimits& limits);
  bool pending() const { return pending_; }
  bool quarantined() const { return quarantined_; }
  SupervisionTime due_at() const { return at_; }
  bool Due(SupervisionTime now) const { return pending_ && now >= at_; }
  // The pending start is attempted at `now`: it counts against the window
  // whether or not it spawns; clears pending and quarantined.
  void Started(SupervisionTime now);
  // A stopping session, or a roster that has completed, drops it.
  void Cancel();

 private:
  bool critical_;
  bool last_exit_clean_{false};
  bool pending_{false};
  bool quarantined_{false};
  SupervisionTime at_{};
  std::deque<SupervisionTime> starts_;
};

// What one pass read from a run's registry row (WatchRun).
struct RowObservation {
  bool found{false};         // an ACTIVE row of this session with the run's pid
  bool constructing{false};  // introspect::kFlagConstructing
  int64_t heartbeat{0};      // heartbeat_monotonic_ns; compared for change only
  uint64_t dispatch{0};      // dispatch_count
};

struct LivenessLimits {
  std::chrono::nanoseconds heartbeat_timeout;  // HeartbeatTimeout()
  std::chrono::milliseconds startup_grace;     // --heartbeat-startup-grace-ms
  std::chrono::milliseconds construction_timeout;  // from spawn
};

// What one pass's look at a run (RunLiveness::Watch) decided.
struct LivenessCheck {
  std::string_view kill;      // as RunLiveness::Judge returns it
  SupervisionTime judged_at;  // the time the decision was taken at
};

// One run's progress, judged once per supervision pass.
//
// A run is killed for a stall only when the launcher itself saw its progress
// unchanged over at least the heartbeat timeout: progress is stamped with
// the time the clock reads after the read that saw it move, and judged at
// the time read before the read that sees it unchanged. The launcher's own
// stall at any point of a pass therefore lengthens no run's measured stall;
// it can only delay a kill.
class RunLiveness {
 public:
  // A run spawned at `spawned`: its grace and construction deadline start.
  void Begin(SupervisionTime spawned);
  // One supervision pass's look at the run: reads `clock`, then the run's
  // row (`read_row`), then `clock` again. The row's progress is recorded at
  // the second time, and the run is judged (Judge) at the first.
  LivenessCheck Watch(const SupervisionClock& clock,
                      const std::function<RowObservation()>& read_row,
                      const LivenessLimits& limits);
  // For a row read at `now`: records its progress at `now` (a running
  // row's dispatch count; a constructing row's heartbeat too) and returns
  // why the run must be killed: kRunConstructionDeadline, kRunStalled or
  // kRunHeartbeatMissing (session.h), or empty.
  std::string_view Judge(const RowObservation& row, SupervisionTime now,
                         const LivenessLimits& limits);
  SupervisionTime progress_changed_at() const { return progress_changed_at_; }
  SupervisionTime heartbeat_changed_at() const { return heartbeat_changed_at_; }

 private:
  // Records `row`'s progress as seen at `read_at`.
  void Record(const RowObservation& row, SupervisionTime read_at);
  // Why the run must be killed at `now`, given what Record kept.
  std::string_view Verdict(const RowObservation& row, SupervisionTime now,
                           const LivenessLimits& limits) const;

  SupervisionTime started_at_{};
  SupervisionTime progress_changed_at_{};
  SupervisionTime heartbeat_changed_at_{};
  int64_t last_heartbeat_{0};
  uint64_t last_dispatch_{0};
};

}  // namespace talos::launcher
