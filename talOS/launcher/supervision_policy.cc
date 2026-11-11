#include "talOS/launcher/supervision_policy.h"

#include <algorithm>

#include "talOS/launcher/session.h"

namespace talos::launcher {

std::chrono::milliseconds RestartBackoff(int restart, bool clean_exit,
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

bool ShouldRestartNode(bool clean_exit, bool critical, int max_restarts,
                       bool stopping) {
  if (stopping || max_restarts <= 0) return false;
  return !clean_exit || critical;
}

std::chrono::nanoseconds HeartbeatTimeout(int64_t period_us,
                                          int heartbeat_periods) {
  return std::chrono::microseconds{
      std::max<int64_t>(std::chrono::microseconds{kMinHeartbeatTimeout}.count(),
                        period_us * heartbeat_periods)};
}

std::chrono::milliseconds RestartSchedule::Schedule(
    SupervisionTime now, const RestartLimits& limits) {
  while (!starts_.empty() && now - starts_.front() >= limits.window) {
    starts_.pop_front();
  }
  const int recent = static_cast<int>(starts_.size());
  quarantined_ = !critical_ && recent > 0 && recent >= limits.max_restarts;
  pending_ = true;
  at_ = (quarantined_ ? starts_.front() + limits.window : now) +
        RestartBackoff(recent + 1, last_exit_clean_, critical_);
  return std::chrono::duration_cast<std::chrono::milliseconds>(at_ - now);
}

void RestartSchedule::Started(SupervisionTime now) {
  starts_.push_back(now);
  pending_ = false;
  quarantined_ = false;
}

void RestartSchedule::Cancel() {
  pending_ = false;
  quarantined_ = false;
}

void RunLiveness::Begin(SupervisionTime spawned) {
  started_at_ = spawned;
  progress_changed_at_ = spawned;
  heartbeat_changed_at_ = spawned;
  last_heartbeat_ = 0;
  last_dispatch_ = 0;
}

std::string_view RunLiveness::Judge(const RowObservation& row,
                                    SupervisionTime now,
                                    const LivenessLimits& limits) {
  if (row.found) {
    // Progress is the loop's dispatch count, which an event-loop node's loop
    // thread stores itself (introspect::Reporter), so a starved reporting
    // thread cannot make a running loop look stalled. Its heartbeat, which
    // that thread writes, counts only while it constructs and has no loop
    // yet.
    const bool beat_moved = row.heartbeat != last_heartbeat_;
    const bool dispatch_moved = row.dispatch != last_dispatch_;
    if (beat_moved) {
      heartbeat_changed_at_ = now;
      last_heartbeat_ = row.heartbeat;
    }
    if (dispatch_moved) last_dispatch_ = row.dispatch;
    if (dispatch_moved || (row.constructing && beat_moved))
      progress_changed_at_ = now;
  }
  const bool grace_over = now - started_at_ >= limits.startup_grace;
  // Registration is part of startup, not an opt-in to supervision:
  // until a row arrives, the spawn time is the last progress time.
  const bool construction_expired =
      row.constructing && now - started_at_ >= limits.construction_timeout;
  if (construction_expired) return kRunConstructionDeadline;
  if (grace_over && now - progress_changed_at_ >= limits.heartbeat_timeout)
    return row.found ? kRunStalled : kRunHeartbeatMissing;
  return {};
}

}  // namespace talos::launcher
