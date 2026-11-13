#pragma once

// Spawning a roster member or probe: fork + exec into a process group of its
// own with a guardian that kills the group if the launcher dies, the child's
// reports of a failed setup step, and the supervision loop's wakeups.

#include <limits.h>
#include <signal.h>
#include <sys/types.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace talos::launcher {

// Held by every thread that spawns (DescribeGraph probes in parallel) from
// the creation of a spawn's pipes to the parent's close of the child's ends,
// and around ProcessLifetimes. A child another thread forks inside that window
// would otherwise inherit this spawn's pipe ends -- close-on-exec is set after
// pipe() on macOS, which has no pipe2 -- and hold a writer open: a probe would
// see no EOF, a drained output pipe no end, until that other child exited.
std::mutex& SpawnMutex();

void SignalProcessGroup(pid_t pid, int signal);

// Call after reaping the leader: even a clean exit must not leave helpers.
void FinishProcessGroup(pid_t pid);

// A pipe whose ends both close on exec. False with errno set.
bool CloseOnExecPipe(int (&fds)[2]);

// Why SpawnProcess returned -1: the step that failed, as its error names it
// ("fork", "fork (guardian)", "execv <path>", ...), and its errno.
struct SpawnFailure {
  std::string step;
  int code{0};
};

// The step names of the two forks a spawn makes: the launcher's own, and the
// child's fork of its group's guardian.
inline constexpr std::string_view kForkStep = "fork";
inline constexpr std::string_view kGuardianForkStep = "fork (guardian)";

// A failure that a later attempt can avoid: a fork refused for want of a
// process slot (EAGAIN, the per-user or system process limit) or memory
// (ENOMEM). Every other step's failure repeats until something is changed.
bool IsTransientSpawnFailure(const SpawnFailure& failure);

// The steps by which a forked child becomes its program; each can fail before
// the program runs.
enum class ChildStep : int {
  kProcessGroup,  // setpgid
  kDeathSignal,   // prctl(PR_SET_PDEATHSIG), Linux only
  kGuardian,      // the fork of the group's guardian
  kOutput,        // the stdout (and stderr) redirection
  kExec,          // execv
};

// What a child writes to its error pipe: a failed step it cannot become its
// program after (`fatal`), or a refused setpgid it recovered from. One write
// of a few dozen bytes, below PIPE_BUF, so the launcher reads a whole report
// or none.
struct ChildReport {
  ChildStep step{ChildStep::kExec};
  int code{0};  // errno of the step's call; 0 when nothing was refused
  bool fatal{false};
  // kProcessGroup only: the setpgid calls made; the errno of the setsid
  // fallback (0 when it succeeded or was not tried); whether the child leads
  // a new session; and its ids when setpgid was last refused.
  int attempts{0};
  int session_code{0};
  bool new_session{false};
  pid_t pid{0};
  pid_t pgrp{0};
  pid_t sid{0};
  pid_t ppid{0};
};
static_assert(sizeof(ChildReport) <= PIPE_BUF,
              "one write of a ChildReport must be atomic");
static_assert(std::is_trivially_copyable_v<ChildReport>,
              "a ChildReport crosses the error pipe as bytes");

// The setpgid(0, 0) calls a child makes before it falls back to setsid.
inline constexpr int kProcessGroupAttempts = 3;

// In a forked child: makes it the leader of a process group of its own (pgid
// = its pid), which is what the launcher signals and its guardian kills.
// setpgid(0, 0) can only be refused with EPERM, which both Linux and XNU
// return for a caller they hold to be its session's leader; a forked child
// that never called setsid is not one, yet macOS CI has seen the refusal
// (TODO.md INC-5, cause not established). So a refused call succeeds anyway
// when the child already leads its own group (as it would if the launcher's
// concurrent setpgid(pid, pid) had moved it; unconfirmed, which is why the
// launcher prints that call's result with the refusal), is retried
// kProcessGroupAttempts times, then falls back to setsid, which also makes it
// the leader of a group of its own, in a new session without a controlling
// terminal; a node in its own group never received the terminal's signals.
// False when none of that worked. `report` records any refusal for the
// launcher (code 0: none). getsid and sched_yield are not on POSIX's
// async-signal-safe list but are bare system calls on Linux and macOS.
bool EnterOwnProcessGroup(ChildReport& report);

// What the launcher adds to a child's refused setpgid, to name its cause: the
// child's last refused attempt and its ids, the setsid fallback's errno when
// it was refused too, and the result of the launcher's own setpgid(pid, pid)
// (`launcher_code`, 0 when it succeeded): "attempt 3 of 3; pid 812, pgrp 640,
// sid 101, ppid 640; launcher's setpgid: ok".
std::string DescribeProcessGroupRefusal(const ChildReport& report,
                                        int launcher_code);

// The launcher's side of a spawned child's error pipe `errors`: reads it until
// EOF, which the child's exec (or its death without a report) leaves once the
// guardian has closed its copy, or until a fatal report, which it returns.
// Each report before that is a refused setpgid the child `program` (pid
// `pid`) recovered from, written to `notes` with DescribeProcessGroupRefusal
// and `launcher_code`. Nullopt at EOF: the child is its program.
std::optional<ChildReport> ReadChildReports(int errors,
                                            const std::string& program,
                                            pid_t pid, int launcher_code,
                                            std::ostream& notes);

// Where a spawned child's stdout and stderr go.
enum class SpawnOutput {
  kInherit,     // The launcher's own.
  kStdoutPipe,  // stdout into a new pipe (a --describe answer); stderr is the
                // launcher's.
  kPipe,        // Both into one new pipe, for a ChildOutput to drain.
};

// The pipe capacity requested for a drained child's output where the OS
// allows it (Linux F_SETPIPE_SZ; a macOS pipe grows to 64 KiB by itself):
// what the child can write while the drain thread is not scheduled.
inline constexpr int kChildOutputPipeBytes = 256 * 1024;

// fork + execv of args[0], in its own process group, its output as `output`
// says, with every catchable signal at its default disposition and none
// blocked, whatever the launcher's dispositions and the calling thread's mask
// are (its guardian alone ignores SIGINT and SIGTERM); with `hold_stop`,
// process::kHeldStopSignals (SIGINT, SIGTERM, SIGHUP) are blocked in the
// child from before its exec, for a member whose --describe says it unblocks
// them once it handles a stop (introspect::Description::holds_stop): a stop
// sent before then stays pending. The mask survives exec, so a script that
// execs its interpreter, and that interpreter's exec of another, keep it;
// the guardian never has them blocked. For a pipe, `*reader`
// receives the read end (close-on-exec). Returns
// the child's pid once it has exec'd, which the close-on-exec error pipe it
// shares with the launcher shows by closing. A child that fails before or at
// its exec reports the step and errno there instead, exits 127 and is reaped;
// then, as when a pipe or the fork fails here, this returns -1 with `error`
// naming the step and its errno ("execv <path>: Permission denied"; a
// refused setpgid adds DescribeProcessGroupRefusal), the same two in
// `*failure` when given, and no reader. A refused setpgid the child recovered
// from is written to stderr. Safe to call from several threads at once
// (SpawnMutex).
pid_t SpawnProcess(std::vector<std::string> args, std::string& error,
                   SpawnOutput output = SpawnOutput::kInherit,
                   int* reader = nullptr, SpawnFailure* failure = nullptr,
                   bool hold_stop = false);

// "9 SIGKILL": a signal's number, which differs between Linux and macOS
// for some signals, and its name where it has a portable one.
std::string SignalText(int signal);

// "exit code N" or "signal N SIGNAME", from a waitpid status.
std::string ExitReason(int status);

// What the supervision loop sleeps on between passes: while it lives, a
// SIGCHLD wakes Wait(), so the loop reaps an exit as it happens, and so does
// Wake(), which the launcher's stop signal handlers call. A self-pipe: the
// wake writes one byte and Wait() polls the read end, so a wake that lands
// after the loop last looked and before it sleeps is not lost. The pipe is
// created once per process and never closed, so a late signal can never
// write into a descriptor number another file has reused; the previous
// SIGCHLD action is restored on destruction. The handler only writes one
// byte (SA_RESTART; errno preserved), so it is safe on any thread that
// receives it. The same on Linux and macOS: no signalfd, pidfd or kqueue.
class SupervisionWakeup {
 public:
  SupervisionWakeup();
  ~SupervisionWakeup();
  SupervisionWakeup(const SupervisionWakeup&) = delete;
  SupervisionWakeup& operator=(const SupervisionWakeup&) = delete;

  // Returns after `timeout` (rounded up to whole milliseconds; a negative
  // one returns at once, and nanoseconds::max() waits for a wake alone), or
  // sooner once a child has exited or Wake() was called since the last call;
  // true when it was woken. Without its pipe or SIGCHLD handler nothing can
  // wake it, so it then returns after at most kUnwokenWait, as a poll.
  bool Wait(std::chrono::nanoseconds timeout) const;

  // Wakes the current or next Wait(), in this process. Async-signal-safe;
  // does nothing before the first SupervisionWakeup made the pipe.
  static void Wake();

  static constexpr std::chrono::milliseconds kUnwokenWait{20};

 private:
  static void OnSignal(int /*signal*/);

  static inline std::atomic<int> read_end_{-1};
  // Read by Wake() in a signal handler: lock-free, so async-signal-safe.
  static inline std::atomic<int> write_end_{-1};
  static_assert(std::atomic<int>::is_always_lock_free);
  struct sigaction previous_ {};
  bool installed_{false};
};

}  // namespace talos::launcher
