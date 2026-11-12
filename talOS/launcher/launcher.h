#pragma once

// The supervisor: Launcher runs one session of a robot config's roster
// (describe and lint, session directory and retention, spawn, supervision,
// restarts, manifest). The libraries it is built on have narrower headers:
// roster.h, describe_graph.h, spawn.h, child_output.h, session.h,
// supervision_policy.h (the restart and kill decisions), studio_bridge.h;
// log_merge.h is log_dump's.

#include <sys/types.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/introspection/registry.h"
#include "talOS/launcher/child_output.h"
#include "talOS/launcher/describe_graph.h"
#include "talOS/launcher/file_digest.h"
#include "talOS/launcher/roster.h"
#include "talOS/launcher/service_notifier.h"
#include "talOS/launcher/session.h"
#include "talOS/launcher/session_lock.h"
#include "talOS/launcher/session_retention.h"
#include "talOS/launcher/spawn.h"
#include "talOS/launcher/studio_bridge.h"
#include "talOS/launcher/supervision_policy.h"

namespace talos::launcher {

// Empty when `registering` more nodes can each claim a registry row now, else
// why not. Rows held by live processes outside this session (a viewer's test
// node, another namespace user's tools) are not swept, and a node that cannot
// claim a row runs unregistered until supervision kills it as missing, so a
// roster that does not fit is refused before anything starts.
std::string RegistryCapacityError(const introspect::RegistryReader& reader,
                                  std::size_t registering);

// `--max-restarts` wins, then `[robot] max_restarts`, then the default.
// Only the flag may be 0, which disables every restart, critical members'
// included: a robot config that did so would leave the robot without its
// drivetrain after one crash, so the key must be at least 1.
bool ResolveMaxRestarts(const config::RobotConfig& robot_config,
                        const LauncherOptions& options, int& out,
                        std::string& error);

// Why a run ended, for its NodeRun, from the reason the launcher ended it
// (`kill_reason`, empty when it did not try), whether the session is
// stopping, and its waitpid status. A kill of the launcher's own (stalled,
// endpoint_mismatch, ...) names the run whatever the status says. A stop
// does only for the endings a stop asks for -- exit 0, SIGINT, SIGTERM, or
// stop_timeout's SIGKILL -- so a node that crashes while stopping (a
// nonzero exit, SIGABRT) is a crash.
std::string_view RunEndReason(std::string_view kill_reason, bool stopping,
                              int status);

// ---------------------------------------------------------------------------
// Launcher
// ---------------------------------------------------------------------------

class Launcher {
 public:
  explicit Launcher(LauncherOptions options);

  // Lints the declared graph, then runs the roster until every node exits,
  // the duration passes or a signal arrives, and writes manifest.json.
  // Returns 0, or the last nonzero exit code of a node. With --replay-inputs
  // it runs only the --only node, on recorded topics (RunOnRecordedInputs).
  // Sets the process's SIGPIPE disposition to ignored, for good.
  int Run();

 private:
  // Replaces manifest.json with the session's state and `procs`. Before the
  // first node starts, and for the final manifest, it is written here and in
  // place when this returns; only a `durable` one is synced (the roster
  // before the first node starts, and the final manifest). Once Run starts
  // the manifest writer, every version is synced, so a power loss mid-match
  // keeps the lifecycle it had reached, and none is written here: the
  // supervision loop restarts crashed nodes and feeds the service watchdog,
  // and a write or sync can take hundreds of milliseconds while other
  // processes write to the same disk (ManifestWriter). A reader (Studio, a
  // test) then sees a version shortly after, not before, what it records.
  //
  // Never throws: a full disk (ENOSPC) or an unwritable session directory
  // must not end supervision of a running robot. The last complete manifest
  // stays in place (WriteFileAtomically) and the next lifecycle change
  // retries. The first failure is logged once, and so is the recovery.
  void PersistManifest(const std::vector<NodeProcessInfo>& procs,
                       bool durable = false);

  // Whether retention may remove anything (ApplyRetention).
  enum class Pruning {
    kNone,      // --describe-only, --replay-inputs.
    kLocked,    // Holds the session lock.
    kUnlocked,  // The lock file is absent and has no room to be created.
  };

  // Removes sessions (Classify) until at most `log_keep_sessions` remain (the
  // new one included), or only protected sessions are left, and those not
  // protected hold at most the half of `log_max_bytes` the new session does
  // not get; then until the log root's filesystem has room for the new
  // session of `roster` members (FreeFloor); and every abandoned launch older
  // than another session directory. Returns the new session's log budget:
  // the other half, which its event writers are capped to.
  // Every file under a session counts toward its bytes: event logs, output
  // logs, camera captures, metadata.
  // The newest kProtectedMatches of the sessions that ran a match or that
  // this build cannot classify (InspectSession) are protected: counted
  // toward log_keep_sessions but not the byte budget, and removed for neither
  // limit, only for the free space. Sessions go in order, each group oldest
  // first by the start time its manifest records (the directory's modification
  // time without one): those that ran no match and unprotected unclassifiable
  // ones, then unprotected matches, then the protected. Free space is read once
  // the limits are met, and each later removal is counted as freeing its bytes,
  // so a filesystem that frees nothing at once (a snapshot holding the
  // blocks) does not empty the log root.
  // Every removal, and every session that could not be read or removed, is
  // recorded in the new session's kRetentionRecordFile, written durably;
  // removing a match or an unclassifiable session is also a warning on
  // stderr.
  // A session that cannot be read or removed (another user's, unreadable,
  // removed meanwhile) is passed over, never counted against the others:
  // retention never stops a launch.
  // Only a launch that may prune removes anything. Pruning::kLocked holds
  // the session lock: that proves no other launch in this shared-memory
  // namespace is recording, so a manifest without an end_wall_ns is one
  // whose launcher died (a power loss), not a live one. Pruning::kUnlocked
  // found the lock file absent with no room to create it, so no launcher
  // holds the lock it would take, but one in the same state may take it
  // while this lists: it removes only sessions whose manifest records an
  // end_wall_ns, and no abandoned launch (which may be that launch
  // describing). --describe-only and --replay-inputs (Pruning::kNone) run
  // beside a live session and only get the budget.
  // Runs before the new session's directory exists, so a full disk does not
  // stop it; `record` gets kRetentionRecordFile's lines, which Run writes
  // once that directory exists (CreateSessionDirectory).
  uint64_t ApplyRetention(const config::RobotConfig& config, std::size_t roster,
                          Pruning pruning, std::string& record);

  // Creates the new session's directory, then writes retention's `record`
  // in it (kRetentionRecordFile, synced; none when empty). When the
  // directory cannot be created, says why and returns false: nothing is
  // started. When the disk is full after a launch that `pruned` removed what
  // it could, that names the free space and the floor a new session of
  // `roster` members may need (FreeFloor).
  bool CreateSessionDirectory(const config::RobotConfig& config,
                              std::size_t roster, bool pruned,
                              const std::string& record);

  // Whether the session lock's directory (introspect::LockDirectory) is on
  // the filesystem holding the log root, or the nearest existing directory
  // above it: only then can retention make room to create the lock file.
  // Linux keeps it in /dev/shm, a tmpfs no log retention frees.
  bool LockSharesLogFilesystem() const;

  // Puts on stderr a retention record no session directory holds.
  static void ReportUnwrittenRetention(const std::string& record);

  // How one roster member is spawned, and its pending restart.
  struct NodeLaunch {
    std::vector<std::string> args;
    std::size_t log_arg{0};  // Index of the --log value in args; 0 for none.
    // Bytes all of this member's runs may log together (its share of the
    // session budget); a restart gets what earlier runs left.
    uint64_t log_budget{0};
    // Its restarts across runs: backoff, rolling budget, quarantine.
    RestartSchedule restart;
    // The current run's progress (its row's dispatch count, or, while it
    // constructs, its heartbeat). Begun only by NewRun.
    RunLiveness liveness;
    LivenessLimits limits{};
    bool heartbeat_killed{false};
    // Event log flags of its registry rows already reported (NoteRecording).
    uint32_t recording_reported{0};
    // This run's realtime outcome was seen in its row (NoteRealtime).
    bool realtime_reported{false};
    // The member answered --describe: its runs' live endpoints are compared
    // with `declared` (CheckEndpoints), once per run, after the run's row
    // has published them; `endpoints_checked` says this run's was.
    bool compare_endpoints{false};
    bool endpoints_checked{false};
    std::vector<naming::SourceShape> declared;
    // A non-critical member killed for a mismatch: never restarted.
    bool endpoints_stopped{false};
    // Why the launcher ended the current run (kRunStalled, kRunStop, ...);
    // empty while it has not tried. Its run is recorded with this reason.
    std::string_view kill_reason;

    // A run of this member was spawned at `spawned`, or, for every first
    // run, supervision starts then: starts its liveness (grace,
    // construction deadline, progress) and clears what the launcher noted
    // of the run before. The one place a run's liveness begins, so a path
    // that starts a run without it also keeps the last run's kill reason.
    void NewRun(SupervisionTime spawned);
  };

  // --replay-inputs: runs the one roster member --only names on the topics
  // the session in options_.replay_inputs logged, by starting its binary with
  // --replay-inputs (talOS/node/node.cc, which Python nodes share through
  // the C ABI). Nothing else starts: no other node, no segment, no hardware.
  // Its log is <output_dir>/<name>.tlog. Returns its exit code.
  int RunOnRecordedInputs(const std::vector<NodeSpec>& specs);

  // Which signal of a found row stopped moving, for the stall kill's line:
  // the heartbeat of a constructing run, or the dispatch count of a running
  // one, with whether its reporting thread's heartbeat still moved or was
  // unchanged too.
  static std::string StallReport(const NodeLaunch& launch, bool constructing,
                                 SupervisionTime now);

  static void InstallSignals();

  // SpawnProcess for one run of a roster member: -1 with `error` naming the
  // failed step when it fails, or while the spawn_failures seam names the
  // member.
  pid_t SpawnNode(const NodeProcessInfo& proc, const NodeLaunch& launch,
                  std::string& error);

  // Schedules `proc`'s next start (RestartSchedule::Schedule) and shows it
  // in the manifest. Only for a restart ShouldRestartNode allows. Returns
  // the delay.
  std::chrono::milliseconds ScheduleRestart(NodeProcessInfo& proc,
                                            NodeLaunch& launch,
                                            SupervisionTime now);

  // The manifest's view of `restart`: `quarantined`, and
  // `retry_at_monotonic_ns` (0 with no pending start). Written after every
  // change to it.
  static void ShowRestart(NodeProcessInfo& proc,
                          const RestartSchedule& restart);

  // A run that cannot be spawned is rescheduled like a crash (Restarts in
  // supervision_policy.h); with restarts disabled it stays down, as a
  // crashed one does.
  // `error` names the failed step and its errno (SpawnProcess).
  void SpawnFailed(NodeProcessInfo& proc, NodeLaunch& launch,
                   SupervisionTime now, const std::string& error);

  // The byte cap for a member's next run: what its earlier runs left of its
  // share `budget`, but never less than half of it, so a run restarted after
  // one that used the whole share still records the match's tail. To make
  // that room it deletes the earlier runs' oldest files (whole segments,
  // oldest run first), never the newest file of the last run, and marks each
  // run it shortened capped (`<log>.capped`, `log_capped`).
  static uint64_t RestartLogBudget(NodeProcessInfo& proc, uint64_t budget);

  // Starts `proc`'s pending run: restart `restart_count + 1` into a fresh
  // event log or, when no run has spawned yet, the first. The attempt counts
  // against the rolling budget whether or not it spawns.
  void Restart(NodeProcessInfo& proc, NodeLaunch& launch, SupervisionTime now);

  // A run's registry row reports its loop's realtime outcome once the node
  // asked for it, or decided not to (introspect::RealtimeOf; introspection
  // README "Realtime scheduling"). Warned about once per run
  // (RealtimeWarning); the member keeps the worst outcome any run reported,
  // in the live manifest and so in the final one, which session_report
  // --strict judges. Returns whether `proc` changed.
  bool NoteRealtime(NodeProcessInfo& proc, NodeLaunch& launch,
                    uint32_t flags) const;

  // A run's registry row says its event log stopped early
  // (introspect::kFlagRecorderFailed, kFlagLogCapped): the rest of that run
  // is not recorded. Reported once per member and kind, and recorded as the
  // member's recorder_failed or log_capped while the node still runs -- in
  // the live manifest Studio reads, and so in the final one, where a log
  // that stopped at a record boundary would otherwise read as complete.
  // Returns whether `proc` changed.
  static bool NoteRecording(NodeProcessInfo& proc, NodeLaunch& launch,
                            uint32_t flags);

  // Compares a run's published endpoints with its member's probe. The graph
  // lint checked only what the probe declared (Node::describing() must not
  // change a node's endpoints), so a run that differs runs a graph nobody
  // checked: a second writer, a reader of a topic nobody writes. Always
  // reported on stderr and as the member's endpoint_mismatch. Without
  // --allow-graph-errors it is then refused as the critical/non-critical
  // split already treats a failure (README "Live endpoints"): a non-critical
  // member is killed and not restarted, and the session runs on; a critical
  // member's first run stops the session, as the lint refuses to start one;
  // a critical member's restart, in a session that may be driving, is only
  // reported, since the launcher cannot tell whether the robot is enabled.
  // Returns whether `proc` changed.
  bool CheckEndpoints(NodeProcessInfo& proc, NodeLaunch& launch,
                      const std::vector<naming::SourceShape>& live,
                      bool complete);

  // Whether any member other than `self` (and the Studio bridge, a viewer)
  // is still live -- running or awaiting a restart -- without having exited
  // 0. A clean exit's restart that falls due when none is would only rerun
  // a roster whose every member has already completed, so it is dropped.
  static bool RosterContinues(const std::vector<NodeProcessInfo>& procs,
                              const std::vector<NodeLaunch>& launches,
                              const std::optional<StudioBridge>& studio,
                              std::size_t self);

  // Waits until every node has exited, SIGINTing them all once on a signal or
  // at the deadline. A crashed node is restarted per supervision_policy.h
  // without blocking the loop; a pending restart holds the session open until
  // it fires or the session stops. A bridge that is still starting does not
  // hold it open.
  void Supervise(std::vector<NodeProcessInfo>& procs,
                 std::vector<NodeLaunch>& launches,
                 std::optional<StudioBridge>& studio,
                 std::chrono::steady_clock::time_point deadline);

  // One supervision pass over a running member other than Studio's bridge:
  // finds its run's row in `registry` (null while unattached), records its
  // progress, recording and realtime flags, checks its live endpoints once,
  // and kills a run whose construction deadline passed or whose progress
  // stopped past the startup grace, judged at the times the supervision
  // clock reads around that row read (RunLiveness::Watch). Returns whether
  // `proc` changed.
  bool WatchRun(NodeProcessInfo& proc, NodeLaunch& launch,
                const introspect::RegistryMapping* registry);

  // Fills `launch` for a member's first run: its supervision settings and
  // the command line it is spawned with, its event log capped at
  // `log_budget`; the hardware bridge also gets the controller's addresses.
  void PrepareLaunch(const NodeSpec& spec, const NodeDescription& described,
                     const NodeProcessInfo& proc,
                     const config::RobotConfig& robot_config,
                     uint64_t log_budget, const std::string& remote,
                     uint16_t remote_port, uint16_t local_port,
                     NodeLaunch& launch) const;

  LauncherOptions options_;
  SessionManifest session_manifest_;
  std::unique_ptr<ManifestWriter> manifest_writer_;  // PersistManifest
  // Set by Run from --max-restarts or the config, and the window.
  RestartLimits restart_limits_{};
  // Drains every member's output for the session (Run); empty before it.
  std::optional<ChildOutput> output_;
  // Held by Run for the session, refreshed by Supervise; null without one.
  const SessionLock* session_lock_{nullptr};
  // A critical member's first run's live endpoints differed from its probe
  // and --allow-graph-errors was not given (CheckEndpoints): Supervise stops
  // the session and Run returns 1.
  bool endpoints_refused_{false};
  // Far inside macOS clean-tmps' 3 unused days (introspect::TouchLockFile).
  static constexpr std::chrono::hours kSessionLockRefresh{1};
  // Set by the SIGINT/SIGTERM handler, which may run on any thread, and read
  // by the supervision thread: a lock-free atomic is both async-signal-safe
  // and race-free across threads, which volatile sig_atomic_t is not.
  static inline std::atomic<int> stop_requested_{0};
  static_assert(std::atomic<int>::is_always_lock_free);
};

}  // namespace talos::launcher
