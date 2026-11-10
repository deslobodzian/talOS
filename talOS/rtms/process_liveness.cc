#include "talOS/rtms/process_liveness.h"

#include <fcntl.h>
#include <signal.h>
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

// What the kernel reports about a pid: whether a process with it runs -- a
// zombie has exited and maps nothing -- and when it started, 0 when the
// platform did not report it. A pid that exists but cannot be inspected
// counts as running: only a process known to be gone may lose its claims.
struct ProcessStart {
  bool running{false};
  std::uint64_t start{0};
};

#if defined(__APPLE__)
// The start is in microseconds since the epoch, fixed when the process forked.
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
          .start = static_cast<std::uint64_t>(start.tv_sec) * 1'000'000U +
                   static_cast<std::uint64_t>(start.tv_usec)};
}
#else
// /proc/<pid>/stat holds the state in field 3, the thread count in field 20
// and the start, in clock ticks since boot, in field 22. The command name
// (field 2) may hold spaces and parentheses, so fields are counted from its
// closing ')'. kill() first, so another user's process hidden from /proc
// (hidepid) still counts as running.
ProcessStart InspectProcess(pid_t pid) {
  if (::kill(pid, 0) != 0 && errno == ESRCH) return {};
  std::array<char, 32> path{};
  std::snprintf(path.data(), path.size(), "/proc/%d/stat",
                static_cast<int>(pid));
  const int fd = ::open(path.data(), O_RDONLY | O_CLOEXEC);
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
#endif

}  // namespace

std::uint64_t OwnProcessWord() {
  static std::atomic<std::uint64_t> cached{0};
  const auto pid = static_cast<std::uint64_t>(::getpid());
  std::uint64_t word = cached.load(std::memory_order_relaxed);
  if ((word & kProcessPidMask) != pid) {
    const std::uint64_t start =
        InspectProcess(static_cast<pid_t>(pid)).start & kProcessStartMask;
    word = (start << kProcessPidBits) | pid;
    cached.store(word, std::memory_order_relaxed);
  }
  return word;
}

bool ProcessWordAlive(std::uint64_t word) {
  const std::uint64_t pid = word & kProcessPidMask;
  if (pid == 0 || pid > static_cast<std::uint64_t>(INT32_MAX)) return false;
  const ProcessStart process = InspectProcess(static_cast<pid_t>(pid));
  const std::uint64_t recorded = word >> kProcessPidBits;
  return process.running && (recorded == 0 || process.start == 0 ||
                             recorded == (process.start & kProcessStartMask));
}

}  // namespace talos
