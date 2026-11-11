#include "talOS/launcher/spawn.h"

#include <fcntl.h>
#include <poll.h>
#include <sched.h>
#include <sys/wait.h>
#include <unistd.h>
#ifdef __linux__
#include <sys/prctl.h>
#include <sys/syscall.h>
#endif

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <thread>
#include <utility>

namespace talos::launcher {
namespace {

// The launcher alone holds each lifetime pipe's writer. A tiny guardian in the
// child's group kills the group on EOF, including when the launcher is
// SIGKILLed. Writers are close-on-exec and a guardian keeps no descriptor but
// its own pipe's reader, so no node or guardian owns a writer.
std::map<pid_t, int>& ProcessLifetimes() {
  static std::map<pid_t, int> lifetimes;
  return lifetimes;
}

// Closes every descriptor but `keep`. System calls only, so it is safe in a
// forked child of a multithreaded process.
void CloseDescriptorsExcept(int keep) {
#if defined(__linux__) && defined(SYS_close_range)
  // Linux 5.9+: two calls, however high RLIMIT_NOFILE is.
  const auto kept = static_cast<unsigned>(keep);
  if ((keep == 0 || ::syscall(SYS_close_range, 0u, kept - 1, 0u) == 0) &&
      ::syscall(SYS_close_range, kept + 1, ~0u, 0u) == 0)
    return;
#endif
  const int limit = ::getdtablesize();
  for (int fd = 0; fd < limit; ++fd) {
    if (fd != keep) ::close(fd);
  }
}

// In a forked child: one write of `report`, async-signal-safe as every call
// between a fork and an exec must be (the fork copied whatever locks other
// threads held). Nothing can be done if it fails.
void WriteChildReport(int errors, const ChildReport& report) {
  [[maybe_unused]] const ssize_t written =
      ::write(errors, &report, sizeof report);
}

// In a forked child, before its exec: reports `step` and errno to the
// launcher, then exits 127, a shell's status for a command it cannot run.
[[noreturn]] void ReportChildFailure(int errors, ChildStep step) {
  ChildReport report;
  report.step = step;
  report.code = errno;
  report.fatal = true;
  WriteChildReport(errors, report);
  ::_exit(127);
}

// How SpawnProcess names a child's failed step.
std::string ChildStepName(ChildStep step, const std::string& program) {
  switch (step) {
    case ChildStep::kProcessGroup:
      return "setpgid";
    case ChildStep::kDeathSignal:
      return "prctl(PR_SET_PDEATHSIG)";
    case ChildStep::kGuardian:
      return std::string{kGuardianForkStep};
    case ChildStep::kOutput:
      return "redirect output";
    case ChildStep::kExec:
      return "execv " + program;
  }
  return "child setup";
}

}  // namespace

std::mutex& SpawnMutex() {
  static std::mutex mutex;
  return mutex;
}

void SignalProcessGroup(pid_t pid, int signal) {
  if (pid > 0) ::kill(-pid, signal);
}

void FinishProcessGroup(pid_t pid) {
  SignalProcessGroup(pid, SIGKILL);
  const std::lock_guard lock{SpawnMutex()};
  auto& lifetimes = ProcessLifetimes();
  const auto found = lifetimes.find(pid);
  if (found != lifetimes.end()) {
    ::close(found->second);
    lifetimes.erase(found);
  }
}

bool CloseOnExecPipe(int (&fds)[2]) {
  if (::pipe(fds) != 0) return false;
  if (::fcntl(fds[0], F_SETFD, FD_CLOEXEC) == 0 &&
      ::fcntl(fds[1], F_SETFD, FD_CLOEXEC) == 0)
    return true;
  const int error = errno;
  ::close(fds[0]);
  ::close(fds[1]);
  errno = error;
  return false;
}

bool IsTransientSpawnFailure(const SpawnFailure& failure) {
  return (failure.step == kForkStep || failure.step == kGuardianForkStep) &&
         (failure.code == EAGAIN || failure.code == ENOMEM);
}

bool EnterOwnProcessGroup(ChildReport& report) {
  report = ChildReport{};
  report.step = ChildStep::kProcessGroup;
  const pid_t self = ::getpid();
  for (int attempt = 1;; ++attempt) {
    if (::setpgid(0, 0) == 0) return true;
    report.code = errno;
    report.attempts = attempt;
    report.pid = self;
    report.pgrp = ::getpgrp();
    report.sid = ::getsid(0);
    report.ppid = ::getppid();
    if (report.pgrp == self) return true;
    if (report.code != EPERM || attempt == kProcessGroupAttempts) break;
    ::sched_yield();
  }
  if (report.code != EPERM) return false;
  if (::setsid() >= 0) {
    report.new_session = true;
    return true;
  }
  report.session_code = errno;
  return ::getpgrp() == self;
}

std::string DescribeProcessGroupRefusal(const ChildReport& report,
                                        int launcher_code) {
  std::string text = "attempt " + std::to_string(report.attempts) + " of " +
                     std::to_string(kProcessGroupAttempts) + "; pid " +
                     std::to_string(report.pid) + ", pgrp " +
                     std::to_string(report.pgrp) + ", sid " +
                     std::to_string(report.sid) + ", ppid " +
                     std::to_string(report.ppid);
  if (report.session_code != 0)
    text += "; setsid: " + std::string{std::strerror(report.session_code)};
  text += "; launcher's setpgid: ";
  text += launcher_code == 0 ? std::string{"ok"}
                             : std::string{std::strerror(launcher_code)};
  return text;
}

std::optional<ChildReport> ReadChildReports(int errors,
                                            const std::string& program,
                                            pid_t pid, int launcher_code,
                                            std::ostream& notes) {
  for (;;) {
    ChildReport report;
    ssize_t got = 0;
    while ((got = ::read(errors, &report, sizeof report)) < 0 &&
           errno == EINTR) {
    }
    if (got != static_cast<ssize_t>(sizeof report)) return std::nullopt;
    if (report.fatal) return report;
    notes << "[launcher] " << program << " (pid " << pid
          << "): setpgid: " << std::strerror(report.code) << " ("
          << DescribeProcessGroupRefusal(report, launcher_code)
          << (report.new_session ? "); it leads a new session instead\n"
                                 : "); it leads its own group\n");
  }
}

pid_t SpawnProcess(std::vector<std::string> args, std::string& error,
                   SpawnOutput output, int* reader, SpawnFailure* failure) {
  const auto fail = [&error, failure](std::string_view step, int code) {
    error = std::string{step} + ": " + std::strerror(code);
    if (failure != nullptr) *failure = {std::string{step}, code};
    return pid_t{-1};
  };
  if (output != SpawnOutput::kInherit && reader == nullptr)
    return fail("output pipe", EINVAL);
  std::vector<char*> c_args;
  c_args.reserve(args.size() + 1);
  for (auto& arg : args) c_args.push_back(arg.data());
  c_args.push_back(nullptr);
  std::cout.flush();
  std::fflush(nullptr);
  const pid_t parent = ::getpid();
  int lifetime[2];
  int errors[2];
  int child_output[2]{-1, -1};
  std::unique_lock spawning{SpawnMutex()};
  if (!CloseOnExecPipe(lifetime)) return fail("pipe", errno);
  if (!CloseOnExecPipe(errors)) {
    const int code = errno;
    ::close(lifetime[0]);
    ::close(lifetime[1]);
    return fail("pipe", code);
  }
  if (output != SpawnOutput::kInherit && !CloseOnExecPipe(child_output)) {
    const int code = errno;
    for (const int fd : {lifetime[0], lifetime[1], errors[0], errors[1]})
      ::close(fd);
    return fail("pipe", code);
  }
#if defined(__linux__) && defined(F_SETPIPE_SZ)
  // Best effort: the default 64 KiB still never blocks a child that writes
  // less than that between two of the drain thread's reads.
  if (output == SpawnOutput::kPipe)
    ::fcntl(child_output[1], F_SETPIPE_SZ, kChildOutputPipeBytes);
#endif
  const pid_t pid = ::fork();
  if (pid < 0) {
    const int code = errno;
    for (const int fd : {lifetime[0], lifetime[1], errors[0], errors[1],
                         child_output[0], child_output[1]})
      if (fd >= 0) ::close(fd);
    return fail(kForkStep, code);
  }
  if (pid > 0) {
    ::close(lifetime[0]);
    ::close(errors[1]);
    if (child_output[1] >= 0) ::close(child_output[1]);
    // Both sides establish the group. The child's own call is the one that
    // counts: this returns only after the child's exec or death, so nothing
    // signals the group before it exists. This one may let a child whose call
    // is refused find itself its group's leader (EnterOwnProcessGroup); its
    // result goes with any refusal the child reports, since whether it races
    // the child's call into that refusal is not known (TODO.md INC-5).
    const int launcher_group = ::setpgid(pid, pid) == 0 ? 0 : errno;
    ProcessLifetimes().emplace(pid, lifetime[1]);
    spawning.unlock();
    const std::optional<ChildReport> report = ReadChildReports(
        errors[0], args.front(), pid, launcher_group, std::cerr);
    ::close(errors[0]);
    if (!report) {
      if (reader != nullptr) *reader = child_output[0];
      return pid;
    }
    const ChildReport failure = *report;
    if (child_output[0] >= 0) ::close(child_output[0]);
    while (::waitpid(pid, nullptr, 0) < 0 && errno == EINTR) {
    }
    FinishProcessGroup(pid);
    const pid_t failed =
        fail(ChildStepName(failure.step, args.front()), failure.code);
    if (failure.step == ChildStep::kProcessGroup)
      error +=
          " (" + DescribeProcessGroupRefusal(failure, launcher_group) + ")";
    return failed;
  }
  // The child: only async-signal-safe calls from here to the exec.
  ChildReport own_group;
  if (!EnterOwnProcessGroup(own_group)) {
    own_group.fatal = true;
    WriteChildReport(errors[1], own_group);
    ::_exit(127);
  }
  if (own_group.code != 0) WriteChildReport(errors[1], own_group);
#ifdef __linux__
  if (::prctl(PR_SET_PDEATHSIG, SIGKILL) != 0)
    ReportChildFailure(errors[1], ChildStep::kDeathSignal);
#endif
  // The launcher died first, so nothing reads a report.
  if (::getppid() != parent) ::_exit(127);
  std::signal(SIGINT, SIG_DFL);
  std::signal(SIGTERM, SIG_DFL);
  const pid_t group = ::getpid();
  const pid_t guardian = ::fork();
  if (guardian < 0) ReportChildFailure(errors[1], ChildStep::kGuardian);
  if (guardian == 0) {
    // The launcher waits for every copy of the error pipe's writer to close.
    ::close(errors[1]);
    // Graceful group signals belong to the node; the guardian waits for EOF.
    std::signal(SIGINT, SIG_IGN);
    std::signal(SIGTERM, SIG_IGN);
    // Unlike the node it never execs, so any other descriptor of the
    // launcher's process, close-on-exec or not, would stay open for as long
    // as the node runs: another thread's registry lock would stay locked.
    CloseDescriptorsExcept(lifetime[0]);
    char unused;
    while (::read(lifetime[0], &unused, 1) < 0 && errno == EINTR) {
    }
    SignalProcessGroup(group, SIGKILL);
    ::_exit(0);
  }
  ::close(lifetime[0]);
  // dup2 clears close-on-exec on the copies; the pipe's own descriptors close
  // at the exec. The writer stays blocking: see ChildOutput for why.
  if (output != SpawnOutput::kInherit) {
    if (::dup2(child_output[1], STDOUT_FILENO) < 0 ||
        (output == SpawnOutput::kPipe &&
         ::dup2(child_output[1], STDERR_FILENO) < 0))
      ReportChildFailure(errors[1], ChildStep::kOutput);
  }
  ::execv(c_args[0], c_args.data());
  ReportChildFailure(errors[1], ChildStep::kExec);
}

std::string SignalText(int signal) {
  struct Named {
    int signal;
    const char* name;
  };
  static constexpr Named kNames[] = {
      {SIGHUP, "SIGHUP"},   {SIGINT, "SIGINT"},   {SIGQUIT, "SIGQUIT"},
      {SIGILL, "SIGILL"},   {SIGTRAP, "SIGTRAP"}, {SIGABRT, "SIGABRT"},
      {SIGBUS, "SIGBUS"},   {SIGFPE, "SIGFPE"},   {SIGKILL, "SIGKILL"},
      {SIGUSR1, "SIGUSR1"}, {SIGSEGV, "SIGSEGV"}, {SIGUSR2, "SIGUSR2"},
      {SIGPIPE, "SIGPIPE"}, {SIGALRM, "SIGALRM"}, {SIGTERM, "SIGTERM"},
      {SIGXCPU, "SIGXCPU"}, {SIGXFSZ, "SIGXFSZ"}, {SIGSYS, "SIGSYS"},
  };
  std::string text = std::to_string(signal);
  for (const auto& named : kNames) {
    if (named.signal == signal) return text + " " + named.name;
  }
  return text;
}

std::string ExitReason(int status) {
  if (WIFSIGNALED(status)) return "signal " + SignalText(WTERMSIG(status));
  return "exit code " +
         std::to_string(WIFEXITED(status) ? WEXITSTATUS(status) : -1);
}

ChildExitWakeup::ChildExitWakeup() {
  {
    const std::lock_guard lock{SpawnMutex()};
    if (pipe_[0] < 0) {
      int fds[2];
      if (CloseOnExecPipe(fds)) {
        for (const int fd : fds)
          ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK);
        pipe_[0] = fds[0];
        pipe_[1] = fds[1];
      }
    }
  }
  if (pipe_[0] < 0) return;  // Wait() then sleeps, as a poll would.
  struct sigaction action {};
  action.sa_handler = &ChildExitWakeup::OnSignal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_RESTART | SA_NOCLDSTOP;
  installed_ = ::sigaction(SIGCHLD, &action, &previous_) == 0;
}

ChildExitWakeup::~ChildExitWakeup() {
  if (installed_) ::sigaction(SIGCHLD, &previous_, nullptr);
}

void ChildExitWakeup::Wait(std::chrono::milliseconds timeout) const {
  if (!installed_) {
    std::this_thread::sleep_for(timeout);
    return;
  }
  ::pollfd ready{pipe_[0], POLLIN, 0};
  if (::poll(&ready, 1, static_cast<int>(timeout.count())) > 0) {
    char drained[64];
    while (::read(pipe_[0], drained, sizeof drained) > 0) {
    }
  }
}

void ChildExitWakeup::OnSignal(int /*signal*/) {
  const int saved = errno;
  const char byte = 0;
  [[maybe_unused]] const ssize_t wrote = ::write(pipe_[1], &byte, 1);
  errno = saved;
}

}  // namespace talos::launcher
