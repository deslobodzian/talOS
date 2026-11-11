#pragma once

// A session's records: its id, manifest.json and the per-member run history
// it holds, the atomic writes that keep them, and the event-log budget.

#include <sys/types.h>

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "talOS/introspection/describe.h"
#include "talOS/process/realtime_outcome.h"

namespace talos::launcher {

uint64_t GenerateSessionId();

std::string Quoted(std::string_view value);

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
bool WriteFileAtomically(const std::string& path, std::string_view contents,
                         const SyncFunction& sync, std::string& error);

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
  void Start();

  // Never waits for the disk: the worker holds the mutex only to take the
  // pending version, never across a write.
  void Post(std::string contents);

  // Writes the pending version, if any, then stops the worker. Waits for the
  // disk.
  void Stop();

 private:
  void Run();
  void Persist(const std::string& contents, const SyncFunction& sync);

  const std::string path_;
  const SyncFunction sync_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::optional<std::string> pending_;  // Under mutex_.
  bool stopping_{false};                // Under mutex_.
  bool failing_{false};  // The worker's, or the caller's while it is stopped.
  std::thread worker_;
};

// Why a run of a roster member ended (NodeRun::reason), as the supervisor
// saw it. The first five are the launcher's own kills.
inline constexpr std::string_view kRunStalled = "stalled";
inline constexpr std::string_view kRunHeartbeatMissing = "heartbeat_missing";
inline constexpr std::string_view kRunConstructionDeadline =
    "construction_deadline";
inline constexpr std::string_view kRunEndpointMismatch = "endpoint_mismatch";
inline constexpr std::string_view kRunStopTimeout = "stop_timeout";
inline constexpr std::string_view kRunStop = "stop";  // The session stopped.
inline constexpr std::string_view kRunExit = "exit";  // Exited 0 on its own.
// Exited nonzero, or a signal the launcher did not send ended it.
inline constexpr std::string_view kRunCrash = "crash";
inline constexpr std::string_view kRunSpawnFailed = "spawn_failed";
inline constexpr std::string_view kRunRunning = "running";

// A member's run history keeps its first run and its newest ones, at most
// this many in all; older ones are counted in runs_dropped.
inline constexpr std::size_t kMaxRecordedRuns = 32;

// One run of a roster member: one spawn attempt, and how it ended.
struct NodeRun {
  pid_t pid{-1};  // -1 for a spawn that failed.
  int64_t start_wall_ns{0};
  int64_t end_wall_ns{0};  // 0 while it runs.
  int exit_code{-1};       // -1 while it runs, and after a signal ended it.
  int exit_signal{0};
  std::string_view reason{kRunRunning};
};

int64_t WallClockNanos();

struct NodeProcessInfo {
  std::string name;
  std::string target;
  std::string binary_path;
  // SHA-256 of binary_path when the session started (FileSha256); empty when
  // it could not be read.
  std::string binary_sha256;
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
  // row (introspect::kFlagRecorderFailed), or unreadable, truncated or
  // damaged (event::log::LogDamage) at shutdown.
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
  // A run's live endpoints, once its registry row published them, differed
  // from what its --describe probe declared (EndpointMismatch): that run ran
  // under a graph the lint never checked. Never cleared by a later run.
  bool endpoint_mismatch{false};
  // The worst realtime outcome any run's registry row reported
  // (introspect::RealtimeOf; process::RealtimeGrant order); empty until one
  // did.
  std::optional<process::RealtimeOutcome> realtime;

  // Keeps `outcome` when it is worse than the one recorded; returns whether
  // it was.
  bool RecordRealtime(const process::RealtimeOutcome& outcome);

  // Every run, first first, capped at kMaxRecordedRuns: the first run and
  // the newest; `runs_dropped` counts the ones between that were dropped.
  std::vector<NodeRun> runs;
  uint64_t runs_dropped{0};

  // Records the start of a run (`pid` -1 for a spawn that failed).
  void BeginRun(pid_t run_pid);

  // Ends the newest run with this process's exit_code and exit_signal.
  void EndRun(std::string_view reason);

  // Records a run that could not be spawned: it has no exit of its own.
  void RecordSpawnFailure();
};

// The launcher's warning line for a run whose loop reported `outcome`, or ""
// when it needs none. A bridge or control loop needs one without its full
// grant: the policy or a memory step refused, or, off --sim, never requested.
std::string RealtimeWarning(std::string_view name, pid_t pid,
                            const process::RealtimeOutcome& outcome,
                            bool simulation);

struct SessionManifest {
  uint64_t session_id{0};
  std::string config_path;
  // sha256 over every staged file as config-relative path + bytes, from the
  // bundle manifest. Empty for non-bundle launches.
  std::string config_sha256;
  // The source revision a --stamp build of the bundle recorded
  // (bundle_manifest.toml git_revision), and whether its tree had
  // uncommitted changes. Empty and false otherwise.
  std::string git_revision;
  bool git_dirty{false};
  std::string output_dir;
  bool simulation{false};
  int64_t start_wall_ns{0};
  int64_t end_wall_ns{0};
  std::size_t declared_topics{0};
  std::vector<NodeProcessInfo> nodes;

  std::string ToJson() const;
};

// A node's event log for run `restart` (0 for the first run): `<name>.tlog`,
// then `<name>.restart<k>.tlog`, so a restart never truncates an earlier run.
std::string NodeLogPath(const std::string& output_dir, std::string_view name,
                        int restart);

}  // namespace talos::launcher
