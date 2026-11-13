#include "talOS/rtms/process_liveness.h"

#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <cstdint>
#include <cstdio>
#include <string_view>
#include <system_error>

#if defined(__APPLE__)
#include <sys/types.h>
// kinfo_proc embeds struct extern_proc (p_stat, p_starttime, SZOMB).
#include <sys/proc.h>
#include <sys/sysctl.h>
#endif

namespace talos {
namespace {

// A word fills 62 bits, so a reader state word fits it above two state bits.
static_assert(kProcessNamespaceShift + kProcessNamespaceBits == 62);
// Codes are one-to-one with the inodes they encode: two live namespaces never
// share a code, and an inode outside the encodable range is opaque.
static_assert(PidNamespaceCode(kInitialPidNamespaceInode) == 1);
static_assert(PidNamespaceCode(0xF0000000ULL) == 5);
static_assert(PidNamespaceCode(kInitialPidNamespaceInode +
                               kProcessNamespaceOpaque - 2) ==
              kProcessNamespaceOpaque - 1);
static_assert(PidNamespaceCode(kInitialPidNamespaceInode +
                               kProcessNamespaceOpaque - 1) ==
              kProcessNamespaceOpaque);
static_assert(PidNamespaceCode(kInitialPidNamespaceInode - 1) ==
              kProcessNamespaceOpaque);
static_assert(!ProcessNamespaceComparable(kProcessNamespaceOpaque,
                                          kProcessNamespaceOpaque));

// What the kernel reports about a pid: whether a process with it runs -- a
// zombie has exited and maps nothing -- and when it started, 0 when the
// platform did not report it. A pid that exists but cannot be inspected
// counts as running: only a process known to be gone may lose its claims.
struct ProcessStart {
  bool running{false};
  std::uint64_t start{0};
};

// This process as it records itself (its word) and the namespace code it
// checks other words from.
struct OwnIdentity {
  std::uint64_t word{0};
  std::uint64_t checker{kProcessNamespaceNone};
};

#if defined(__APPLE__)
// The start is in milliseconds since the epoch, fixed when the process forked
// (the unit process_liveness.h chooses for its wrap).
ProcessStart InspectProcess(pid_t pid) {
  int mib[4] = {CTL_KERN, KERN_PROC, KERN_PROC_PID, pid};
  struct kinfo_proc info {};
  std::size_t size = sizeof(info);
  if (::sysctl(mib, 4, &info, &size, nullptr, 0) != 0) {
    return {.running = ::kill(pid, 0) == 0 || errno == EPERM};
  }
  // An absent pid succeeds with nothing copied out.
  if (size == 0 || info.kp_proc.p_stat == SZOMB) return {};
  const struct timeval& start = info.kp_proc.p_starttime;
  return {.running = true,
          .start = static_cast<std::uint64_t>(start.tv_sec) * 1'000U +
                   static_cast<std::uint64_t>(start.tv_usec) / 1'000U};
}

// macOS has no pid namespaces.
OwnIdentity InspectSelf() {
  const auto pid = static_cast<std::uint64_t>(::getpid());
  const std::uint64_t start =
      InspectProcess(static_cast<pid_t>(pid)).start & kProcessStartMask;
  return {.word = (start << kProcessPidBits) | pid,
          .checker = kProcessNamespaceNone};
}
#else
// A /proc/<pid>/stat file holds the state in field 3, the thread count in
// field 20 and the start, in clock ticks since boot, in field 22. The command
// name (field 2) may hold spaces and parentheses, so fields are counted from
// its closing ')'. A file that cannot be read or parsed reports a running
// process of unknown start.
ProcessStart ReadStat(const char* path) {
  const int fd = ::open(path, O_RDONLY | O_CLOEXEC);
  if (fd == -1) return {.running = true};
  std::array<char, 1024> buffer{};
  const ssize_t length = ::read(fd, buffer.data(), buffer.size());
  ::close(fd);
  if (length <= 0) return {.running = true};
  const std::string_view line{buffer.data(), static_cast<std::size_t>(length)};
  const std::size_t name_end = line.rfind(')');
  if (name_end == std::string_view::npos || name_end + 2 >= line.size()) {
    return {.running = true};
  }
  const char state = line[name_end + 2];
  std::size_t at = name_end + 2;
  int index = 3;
  // Parses field `wanted`; a number that runs into the end of the read may
  // have been cut short, so it must end in a space.
  const auto field = [&](int wanted, std::uint64_t& value) {
    for (; index < wanted; ++index) {
      at = line.find(' ', at);
      if (at == std::string_view::npos) return false;
      ++at;
    }
    const char* const end = line.data() + line.size();
    const auto [last, error] = std::from_chars(line.data() + at, end, value);
    return error == std::errc{} && last != end && *last == ' ';
  };
  std::uint64_t threads = 0;
  std::uint64_t start = 0;
  if (!field(20, threads) || !field(22, start)) return {.running = true};
  // A leader that exited while its other threads run (pthread_exit from
  // main) shows as a zombie but its threads still map the segment.
  if ((state == 'Z' && threads <= 1) || state == 'X' || state == 'x') {
    return {};
  }
  return {.running = true, .start = start};
}

// kill() first, so another user's process hidden from /proc (hidepid) still
// counts as running.
ProcessStart InspectProcess(pid_t pid) {
  if (::kill(pid, 0) != 0 && errno == ESRCH) return {};
  std::array<char, 32> path{};
  std::snprintf(path.data(), path.size(), "/proc/%d/stat",
                static_cast<int>(pid));
  return ReadStat(path.data());
}

// Whether the mounted /proc shows this process's own pid namespace. NSpid in
// /proc/self/status lists this process's pid in every namespace from the one
// that /proc belongs to down to its own (Linux 4.1+), so exactly one pid means
// they are the same. A process that entered a new pid namespace without
// mounting /proc for it (unshare --pid without --mount-proc) would otherwise
// read another process's start for every pid it checks; comparing
// /proc/self with getpid() misses it when the two pids happen to be equal.
bool ProcIsOwnNamespace() {
  const int fd = ::open("/proc/self/status", O_RDONLY | O_CLOEXEC);
  if (fd == -1) return false;
  std::array<char, 8192> buffer{};
  std::size_t used = 0;
  while (used < buffer.size()) {
    const ssize_t length =
        ::read(fd, buffer.data() + used, buffer.size() - used);
    if (length < 0 && errno == EINTR) continue;
    if (length <= 0) break;
    used += static_cast<std::size_t>(length);
  }
  ::close(fd);
  const std::string_view status{buffer.data(), used};
  constexpr std::string_view kKey = "\nNSpid:";
  const std::size_t key = status.find(kKey);
  if (key == std::string_view::npos) return false;
  const std::size_t end = status.find('\n', key + kKey.size());
  if (end == std::string_view::npos) return false;
  std::string_view pids =
      status.substr(key + kKey.size(), end - key - kKey.size());
  int count = 0;
  while (!pids.empty()) {
    const std::size_t first = pids.find_first_not_of(" \t");
    if (first == std::string_view::npos) break;
    pids.remove_prefix(first);
    ++count;
    const std::size_t gap = pids.find_first_of(" \t");
    pids.remove_prefix(gap == std::string_view::npos ? pids.size() : gap);
  }
  return count == 1;
}

// Reads itself through /proc/self, which names this process in whichever
// namespace the mounted /proc shows. The checking code is its own
// namespace's only while that /proc is its own namespace's.
OwnIdentity InspectSelf() {
  const auto pid = static_cast<std::uint64_t>(::getpid());
  const std::uint64_t start = ReadStat("/proc/self/stat").start;
  struct stat entry {};
  const std::uint64_t space =
      ::stat("/proc/self/ns/pid", &entry) == 0
          ? PidNamespaceCode(static_cast<std::uint64_t>(entry.st_ino))
          : kProcessNamespaceOpaque;
  return {.word = (space << kProcessNamespaceShift) |
                  ((start & kProcessStartMask) << kProcessPidBits) | pid,
          .checker = ProcIsOwnNamespace() ? space : kProcessNamespaceOpaque};
}
#endif

// Keyed by pid, and cleared in a forked child, so a child -- even one that
// a new pid namespace gave its parent's pid -- records itself.
std::atomic<std::uint64_t> cached_word{0};
// The reset runs in the forked child, where only lock-free operations are
// safe.
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
std::atomic<std::uint64_t> cached_checker{kProcessNamespaceNone};

void ForgetIdentity() { cached_word.store(0, std::memory_order_relaxed); }

OwnIdentity Own() {
  static const bool registered =
      ::pthread_atfork(nullptr, nullptr, &ForgetIdentity) == 0;
  (void)registered;
  const auto pid = static_cast<std::uint64_t>(::getpid());
  const std::uint64_t word = cached_word.load(std::memory_order_acquire);
  if (word != 0 && (word & kProcessPidMask) == pid) {
    return {.word = word,
            .checker = cached_checker.load(std::memory_order_relaxed)};
  }
  const OwnIdentity own = InspectSelf();
  cached_checker.store(own.checker, std::memory_order_relaxed);
  cached_word.store(own.word, std::memory_order_release);
  return own;
}

}  // namespace

std::uint64_t OwnProcessWord() { return Own().word; }

bool ProcessWordAlive(std::uint64_t word) {
  return ProcessWordAliveFrom(word, Own().checker);
}

bool ProcessWordAliveFrom(std::uint64_t word, std::uint64_t checker) {
  const std::uint64_t pid = word & kProcessPidMask;
  if (pid == 0) return false;
  if (!ProcessNamespaceComparable(
          (word >> kProcessNamespaceShift) & kProcessNamespaceMask, checker)) {
    return true;
  }
  const ProcessStart process = InspectProcess(static_cast<pid_t>(pid));
  const std::uint64_t recorded = (word >> kProcessPidBits) & kProcessStartMask;
  return process.running && (recorded == 0 || process.start == 0 ||
                             recorded == (process.start & kProcessStartMask));
}

}  // namespace talos
