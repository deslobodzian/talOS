// The launcher binary SIGKILLed while it supervises a roster: every member's
// process group dies with it (REQ-SAFE-30), so no node outlives its
// supervisor.
//
// Each member is a shell stub that forks a grandchild into its own group and
// then execs `sleep`, so its group holds three processes: the member (group
// leader), its guardian and the grandchild. The test snapshots every group's
// members, SIGKILLs the launcher and waits for each of them to exit.
//
// Which mechanism a group member dies by: on Linux the member itself carries
// PR_SET_PDEATHSIG (SIGKILL), which fork does not pass on, so the grandchild
// and the guardian die only by the guardian's kill of the group; Linux
// therefore proves the guardian path through them. On macOS every member
// dies by the guardian alone.
//
// The bound. Nothing between the launcher's death and the group's exit waits
// on a timer: the kernel closes the dead launcher's descriptors (the last
// writer of each lifetime pipe), each guardian's blocked read returns EOF and
// it sends SIGKILL to its group, and each member exits. Every step is a
// wakeup or a system call, so on a quiet host the whole takes milliseconds
// (the measured value is printed). REQ-SAFE-30 states no latency, and an
// orphan cannot command a motor past the RoboRIO's own command timeout
// anyway, so the deadline here is kHangMs, which only bounds a hang
// (docs/TESTING.md §6.3): a missing or polite guardian leaves a member alive
// forever, which it catches.
//
// Exits are watched by an identity the pid cannot be reused under (a pidfd
// on Linux, a kqueue process filter on macOS), registered while the process
// is known to be alive, so a reused pid can neither pass nor fail a case.
// A zombie has exited: it runs nothing.

#include <fcntl.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#ifdef __linux__
#include <sys/prctl.h>
#include <sys/syscall.h>
#else
#include <libproc.h>
#include <sys/event.h>
#endif

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/registry.h"
#include "talOS/launcher/testdata/first_exec.h"

#if defined(__linux__) && !defined(SYS_pidfd_open)
#define SYS_pidfd_open 434  // The same number on every architecture.
#endif

namespace {

using Clock = std::chrono::steady_clock;

// How long a wait for one event may see nothing before the test gives up.
constexpr int kHangMs = 10000;

// The descriptor the forked launcher's members find the test's event pipe
// on. They inherit it across exec; their guardians close it.
constexpr int kEventFd = 9;

// How long a stub's processes would live if nothing killed them: longer
// than any run of this test, short enough that a leak ends by itself.
constexpr int kStubLifetimeS = 300;

// The roster: the hardware bridge's slot and two subsystems.
const std::vector<std::string> kMembers{"hardware_node", "alpha", "beta"};

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_launcher_death_" +
         std::to_string(::getpid()) + "_" + std::string{name};
}

std::string Runfile(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!runfiles || location == nullptr) return {};
  return runfiles->Rlocation(location);
}

std::string ReadFile(const std::string& path) {
  std::ifstream in{path};
  return std::string((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());
}

int RemainingMs(Clock::time_point deadline) {
  const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
  return static_cast<int>(std::max<std::int64_t>(0, left.count()));
}

// A shell stub for `member`: answers --describe (an empty graph), else forks
// a grandchild into its group, reports "<member> <pid> <grandchild>" on
// kEventFd and execs `sleep`. With `ignore_termination`, both start with
// SIGTERM, SIGINT and SIGHUP ignored, which exec and fork keep.
std::string WriteStub(const std::string& member, bool ignore_termination) {
  const std::string path = TempPath(member) + ".sh";
  talos::introspect::Description description;
  description.name = member;
  description.target = member == "hardware_node"
                           ? "//talOS/bridge:hardware_node"
                           : "//example/" + member + ":node";
  const std::string lifetime = std::to_string(kStubLifetimeS);
  std::ostringstream out;
  out << "#!/bin/sh\n"
      << "for arg in \"$@\"; do\n"
      << "  if [ \"$arg\" = \"--describe\" ]; then\n"
      << "    cat <<'TALOS_DESCRIBE'\n"
      << talos::introspect::DescribeToJson(description) << "TALOS_DESCRIBE\n"
      << "    exit 0\n"
      << "  fi\n"
      << "done\n"
      << (ignore_termination ? "trap '' TERM INT HUP\n" : "") << "sleep "
      << lifetime << " &\n"
      << "echo \"" << member << " $$ $!\" >&" << kEventFd << "\n"
      << "exec sleep " << lifetime << "\n";
  return talos::launcher::testing::WriteWarmStub(path, out.str());
}

// Every live process whose process group is `group`, the leader included.
std::vector<pid_t> GroupMembers(pid_t group) {
  std::vector<pid_t> members;
#ifdef __linux__
  for (const auto& entry : std::filesystem::directory_iterator{"/proc"}) {
    const std::string name = entry.path().filename().string();
    if (name.empty() || !std::all_of(name.begin(), name.end(), [](char c) {
          return c >= '0' && c <= '9';
        }))
      continue;
    // "pid (comm) state ppid pgrp ...": comm may hold spaces and parentheses.
    const std::string stat = ReadFile(entry.path().string() + "/stat");
    const std::size_t close = stat.rfind(')');
    if (close == std::string::npos) continue;
    std::istringstream fields{stat.substr(close + 1)};
    std::string state;
    pid_t parent = 0;
    pid_t pgrp = 0;
    if (fields >> state >> parent >> pgrp && pgrp == group)
      members.push_back(static_cast<pid_t>(std::stol(name)));
  }
#else
  std::vector<pid_t> buffer(256);
  const int bytes = ::proc_listpids(
      PROC_PGRP_ONLY, static_cast<uint32_t>(group), buffer.data(),
      static_cast<int>(buffer.size() * sizeof(pid_t)));
  for (int i = 0; i < bytes / static_cast<int>(sizeof(pid_t)); ++i) {
    if (buffer[static_cast<std::size_t>(i)] > 0)
      members.push_back(buffer[static_cast<std::size_t>(i)]);
  }
#endif
  return members;
}

// Waits for the exit of processes registered while alive.
class ExitWatch {
 public:
  ExitWatch() {
#ifndef __linux__
    queue_ = ::kqueue();
#endif
  }
  ~ExitWatch() {
#ifdef __linux__
    for (const auto& [pid, fd] : pidfds_) ::close(fd);
#else
    if (queue_ >= 0) ::close(queue_);
#endif
  }
  ExitWatch(const ExitWatch&) = delete;
  ExitWatch& operator=(const ExitWatch&) = delete;

  // False with errno when `pid` cannot be watched; a pid that has already
  // exited (ESRCH) is watched as exited.
  bool Add(pid_t pid) {
#ifdef __linux__
    const long fd = ::syscall(SYS_pidfd_open, pid, 0);
    if (fd < 0) {
      if (errno != ESRCH) return false;
      exited_.insert(pid);
      return true;
    }
    pidfds_[pid] = static_cast<int>(fd);
    return true;
#else
    struct kevent change {};
    EV_SET(&change, pid, EVFILT_PROC, EV_ADD | EV_ONESHOT, NOTE_EXIT, 0,
           nullptr);
    if (::kevent(queue_, &change, 1, nullptr, 0, nullptr) != 0) {
      if (errno != ESRCH) return false;
      exited_.insert(pid);
      return true;
    }
    watched_.insert(pid);
    return true;
#endif
  }

  // Waits until every added process has exited or `deadline` passes; the
  // ones still running.
  std::vector<pid_t> AwaitAll(Clock::time_point deadline) {
    for (;;) {
      const std::vector<pid_t> running = Running();
      if (running.empty()) return running;
      const int wait_ms = RemainingMs(deadline);
#ifdef __linux__
      std::vector<pollfd> ready;
      ready.reserve(running.size());
      for (const pid_t pid : running)
        ready.push_back({pidfds_[pid], POLLIN, 0});
      const int polled =
          ::poll(ready.data(), static_cast<nfds_t>(ready.size()), wait_ms);
      if (polled < 0 && errno == EINTR) continue;
      if (polled <= 0) return running;
      for (std::size_t i = 0; i < ready.size(); ++i) {
        if (ready[i].revents != 0) exited_.insert(running[i]);
      }
#else
      struct kevent events[16];
      const timespec timeout{wait_ms / 1000, (wait_ms % 1000) * 1000000L};
      const int got = ::kevent(queue_, nullptr, 0, events, 16, &timeout);
      if (got < 0 && errno == EINTR) continue;
      if (got <= 0) return running;
      for (int i = 0; i < got; ++i)
        exited_.insert(static_cast<pid_t>(events[i].ident));
#endif
    }
  }

 private:
  std::vector<pid_t> Running() const {
    std::vector<pid_t> running;
#ifdef __linux__
    for (const auto& [pid, fd] : pidfds_)
      if (!exited_.contains(pid)) running.push_back(pid);
#else
    for (const pid_t pid : watched_)
      if (!exited_.contains(pid)) running.push_back(pid);
#endif
    return running;
  }

#ifdef __linux__
  std::map<pid_t, int> pidfds_;
#else
  int queue_{-1};
  std::set<pid_t> watched_;
#endif
  std::set<pid_t> exited_;
};

// The launcher binary running the stub roster, forked from the test into a
// process group of its own, its output in a log file. Destruction leaves
// nothing running whatever the case got to: it stops a launcher still alive
// (SIGINT, then SIGKILL) and kills every group the case learned of.
class Session {
 public:
  explicit Session(bool ignore_termination) {
    config_ = TempPath("robot") + ".toml";
    output_dir_ = TempPath("sessions");
    log_ = TempPath("launcher") + ".log";
    std::map<std::string, std::string> stubs;
    for (const auto& member : kMembers)
      stubs[member] = WriteStub(member, ignore_termination);
    std::ofstream{config_} << "[robot]\n"
                           << "name = \"launcher_death_fixture\"\n"
                           << "period_us = 5000\n"
                           << "hardware_node = \"" << stubs["hardware_node"]
                           << "\"\n\n"
                           << "[subsystems.alpha]\n"
                           << "node = \"" << stubs["alpha"] << "\"\n"
                           << "period_us = 5000\n\n"
                           // A config needs a device; alpha owns a motor.
                           << "[subsystems.alpha.motors.m1]\n"
                           << "type = \"TalonFX\"\n"
                           << "bus = \"rio\"\n"
                           << "can_id = 1\n\n"
                           << "[subsystems.beta]\n"
                           << "node = \"" << stubs["beta"] << "\"\n"
                           << "period_us = 5000\n";
  }

  ~Session() {
    if (launcher_ > 0 && !launcher_reaped_) {
      ::kill(launcher_, SIGINT);
      ExitWatch watch;
      if (watch.Add(launcher_))
        watch.AwaitAll(std::chrono::steady_clock::now() +
                       std::chrono::milliseconds{kHangMs});
      ::kill(launcher_, SIGKILL);
      while (::waitpid(launcher_, nullptr, 0) < 0 && errno == EINTR) {
      }
    }
    // Once seen to exit, a pid may already name another process, so only
    // the snapshot's processes the watch still sees running are killed, and
    // only the groups whose leader is one of them. A group never
    // snapshotted is left to its guardian, which kills it when the launcher
    // is gone.
    if (!gone_) {
      const std::vector<pid_t> running = watch_.AwaitAll(Clock::now());
      const auto alive = [&running](pid_t pid) {
        return std::find(running.begin(), running.end(), pid) != running.end();
      };
      for (const auto& [member, group] : groups_) {
        if (alive(group)) ::kill(-group, SIGKILL);
      }
      for (const pid_t pid : running) ::kill(pid, SIGKILL);
    }
    if (events_ >= 0) ::close(events_);
    std::filesystem::remove_all(output_dir_);
    // The launcher prepared the registry in this test's namespace, and a
    // registry outlives its sessions by design.
    talos::introspect::RegistryMapping::unlink();
  }
  Session(const Session&) = delete;
  Session& operator=(const Session&) = delete;

  // Forks and execs the launcher; false with `error` set when it cannot.
  bool Start(std::string& error) {
    const std::string launcher = Runfile("TALOS_LAUNCHER");
    if (launcher.empty()) {
      error = "TALOS_LAUNCHER runfile unavailable";
      return false;
    }
    int events[2];
    if (::pipe(events) != 0 || ::fcntl(events[0], F_SETFD, FD_CLOEXEC) != 0 ||
        ::fcntl(events[1], F_SETFD, FD_CLOEXEC) != 0) {
      error = std::string{"event pipe: "} + std::strerror(errno);
      return false;
    }
    std::vector<std::string> args{launcher, "--config", config_, "--output-dir",
                                  output_dir_,
                                  // The stubs never take a registry row.
                                  "--heartbeat-startup-grace-ms", "60000"};
    std::vector<char*> argv;
    argv.reserve(args.size() + 1);
    for (auto& arg : args) argv.push_back(arg.data());
    argv.push_back(nullptr);
    std::cout.flush();
    std::fflush(nullptr);
    const pid_t test = ::getpid();
    const pid_t pid = ::fork();
    if (pid < 0) {
      error = std::string{"fork: "} + std::strerror(errno);
      return false;
    }
    if (pid == 0) {
      // dup2 clears close-on-exec on kEventFd only.
      const int log = ::open(log_.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
      if (::setpgid(0, 0) != 0 || log < 0 || ::dup2(log, STDOUT_FILENO) < 0 ||
          ::dup2(log, STDERR_FILENO) < 0 || ::dup2(events[1], kEventFd) < 0)
        ::_exit(100);
      if (log > STDERR_FILENO) ::close(log);
#ifdef __linux__
      // In a group of its own, the launcher would outlive a test Bazel kills
      // on its timeout, and supervise its stubs on; it dies with the test
      // instead. The death signal survives execv. A test already gone
      // before it was set would never send it.
      if (::prctl(PR_SET_PDEATHSIG, SIGKILL) != 0 || ::getppid() != test)
        ::_exit(102);
#else
      // macOS has no parent-death signal: a test killed on its timeout
      // leaves this launcher running, to be stopped by hand.
      (void)test;
#endif
      ::execv(argv[0], argv.data());
      ::_exit(101);
    }
    launcher_ = pid;
    ::close(events[1]);
    events_ = events[0];
    return true;
  }

  // Reads kEventFd until every member has reported; false, with `error`
  // naming what was missing, when the pipe ends or is silent for kHangMs.
  bool AwaitMembers(std::string& error) {
    std::string seen;
    while (reported_.size() < kMembers.size()) {
      pollfd ready{events_, POLLIN, 0};
      const int polled = ::poll(&ready, 1, kHangMs);
      if (polled < 0 && errno == EINTR) continue;
      char buffer[256];
      const ssize_t got =
          polled > 0 ? ::read(events_, buffer, sizeof buffer) : 0;
      if (got <= 0) {
        error = polled > 0 ? "the event pipe ended"
                           : "no report in " + std::to_string(kHangMs) + " ms";
        error += "; reported:";
        for (const auto& [member, pids] : reported_) error += " " + member;
        return false;
      }
      seen.append(buffer, static_cast<std::size_t>(got));
      for (std::size_t end; (end = seen.find('\n')) != std::string::npos;) {
        std::istringstream line{seen.substr(0, end)};
        seen.erase(0, end + 1);
        std::string member;
        pid_t pid = 0;
        pid_t grandchild = 0;
        if (line >> member >> pid >> grandchild) {
          groups_[member] = pid;
          reported_[member] = {pid, grandchild};
        }
      }
    }
    return true;
  }

  // Every member of every reported group, each watched from now on.
  bool Snapshot(std::string& error) {
    for (const auto& [member, group] : groups_) {
      const std::vector<pid_t> members = GroupMembers(group);
      const auto [leader, grandchild] = reported_[member];
      for (const pid_t wanted : {leader, grandchild}) {
        if (std::find(members.begin(), members.end(), wanted) ==
            members.end()) {
          error = member + "'s group " + std::to_string(group) +
                  " lacks its process " + std::to_string(wanted);
          return false;
        }
      }
      for (const pid_t pid : members) {
        if (!watch_.Add(pid)) {
          error = "cannot watch pid " + std::to_string(pid) + ": " +
                  std::strerror(errno);
          return false;
        }
        snapshot_.push_back(pid);
        // The group's only other process is the guardian the spawn forked.
        owner_[pid] = member + "'s " +
                      (pid == leader       ? "process"
                       : pid == grandchild ? "grandchild"
                                           : "guardian");
      }
    }
    return true;
  }

  // SIGKILL to the launcher alone, and its reaping.
  void KillLauncher() {
    ::kill(launcher_, SIGKILL);
    while (::waitpid(launcher_, nullptr, 0) < 0 && errno == EINTR) {
    }
    launcher_reaped_ = true;
  }

  std::string Describe(const std::vector<pid_t>& pids) {
    std::string text;
    for (const pid_t pid : pids)
      text += " " + std::to_string(pid) + " (" + owner_[pid] + ")";
    return text;
  }

  // Waits until every snapshot process has exited or `deadline` passes;
  // the ones still running.
  std::vector<pid_t> AwaitAll(Clock::time_point deadline) {
    return watch_.AwaitAll(deadline);
  }

  // Every snapshot process has been seen to exit: teardown signals none.
  void Gone() { gone_ = true; }

  std::string Log() const { return ReadFile(log_); }
  std::size_t SnapshotSize() const { return snapshot_.size(); }

 private:
  std::string config_;
  std::string output_dir_;
  std::string log_;
  pid_t launcher_{-1};
  bool launcher_reaped_{false};
  bool gone_{false};
  int events_{-1};
  std::map<std::string, pid_t> groups_;
  std::map<std::string, std::pair<pid_t, pid_t>> reported_;
  std::vector<pid_t> snapshot_;
  std::map<pid_t, std::string> owner_;
  // Watches every snapshot process; teardown signals only what it still
  // sees running.
  ExitWatch watch_;
};

// SIGKILLs the launcher once every member runs, then waits for every member
// of every group to exit.
void ExpectGroupsDieWithTheLauncher(bool ignore_termination) {
  Session session{ignore_termination};
  std::string error;
  ASSERT_TRUE(session.Start(error)) << error;
  ASSERT_TRUE(session.AwaitMembers(error)) << error << "\nlauncher output:\n"
                                           << session.Log();
  ASSERT_TRUE(session.Snapshot(error)) << error;
  // Leader, guardian and grandchild in each group.
  ASSERT_GE(session.SnapshotSize(), 3 * kMembers.size());

  const Clock::time_point killed = std::chrono::steady_clock::now();
  session.KillLauncher();
  const std::vector<pid_t> survivors =
      session.AwaitAll(killed + std::chrono::milliseconds{kHangMs});
  const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now() - killed)
                              .count();
  EXPECT_TRUE(survivors.empty())
      << "still running " << kHangMs
      << " ms (the hang bound) after the launcher's SIGKILL:"
      << session.Describe(survivors) << "\nlauncher output:\n"
      << session.Log();
  if (!survivors.empty()) return;
  session.Gone();
  std::cout << "[launcher_death] " << session.SnapshotSize() << " processes in "
            << kMembers.size() << " groups exited " << elapsed_us
            << " us after the launcher's SIGKILL\n";
}

// REQ-SAFE-30
// The launcher is SIGKILLed while it supervises the hardware bridge's slot and
// two subsystems, each of which has forked a grandchild into its group: every
// leader, guardian and grandchild exits. On Linux the grandchildren and
// guardians have no death signal, so they prove the guardian's kill.
TEST(LauncherDeathTest, EveryGroupDiesWithTheLauncher) {
  ExpectGroupsDieWithTheLauncher(/*ignore_termination=*/false);
}

// REQ-SAFE-30
// The same with every member and grandchild ignoring SIGTERM, SIGINT and
// SIGHUP: the guardian's kill cannot be one a node may refuse.
TEST(LauncherDeathTest, GroupsThatIgnoreTerminationDieWithTheLauncher) {
  ExpectGroupsDieWithTheLauncher(/*ignore_termination=*/true);
}

}  // namespace
