#include "talOS/process/process.h"

#include <pthread.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "talOS/memory/residency.h"

#if defined(__APPLE__)
#include <dlfcn.h>
#include <mach-o/loader.h>
#include <mach/mach.h>
#include <mach/mach_error.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <pthread/qos.h>
#else
#include <sched.h>
#include <sys/syscall.h>
#endif

namespace talos::process {

std::atomic<bool> stop_requested{false};

namespace {

// The refusals so far, as one line. Fixed storage: this runs where an
// allocation failure must not turn into an exception.
class Problems {
 public:
  void add(const char* step, const char* reason) {
    if (used_ >= sizeof text_) return;
    const int written =
        std::snprintf(text_ + used_, sizeof text_ - used_, "%s%s (%s)",
                      used_ == 0 ? "" : "; ", step, reason);
    if (written > 0) {
      used_ = std::min(sizeof text_, used_ + static_cast<std::size_t>(written));
    }
  }
  bool empty() const { return used_ == 0; }
  const char* text() const { return text_; }

 private:
  char text_[256]{};
  std::size_t used_ = 0;
};

// How much of the loop thread's stack below ConfigureRealtime's frame is
// prefaulted and locked, so a deeper update() than construction reached does
// not fault. Less when the thread's stack has less room (kStackSpare kept).
constexpr std::size_t kStackResidentBytes = 256 * 1024;
constexpr std::size_t kStackSpare = 64 * 1024;

// The calling thread's stack, [low, high).
bool StackBounds(char*& low, char*& high) {
#if defined(__APPLE__)
  high = static_cast<char*>(::pthread_get_stackaddr_np(::pthread_self()));
  low = high - ::pthread_get_stacksize_np(::pthread_self());
  return true;
#else
  pthread_attr_t attributes{};
  if (::pthread_getattr_np(::pthread_self(), &attributes) != 0) return false;
  void* address = nullptr;
  std::size_t size = 0;
  const int result = ::pthread_attr_getstack(&attributes, &address, &size);
  ::pthread_attr_destroy(&attributes);
  if (result != 0) return false;
  low = static_cast<char*>(address);
  high = low + size;
  return true;
#endif
}

// Prefaults the next kStackResidentBytes of the calling thread's stack by
// writing a frame that size, then locks from there to the stack's top (the
// frames in use, the environment above main's on the main thread).
[[gnu::noinline]] void LockStack(Problems& problems) {
  char* low = nullptr;
  char* high = nullptr;
  if (!StackBounds(low, high)) {
    problems.add("stack lock", "stack bounds unknown");
    return;
  }
  char* const here = static_cast<char*>(__builtin_frame_address(0));
  if (here <= low || here > high ||
      static_cast<std::size_t>(here - low) <= kStackSpare) {
    problems.add("stack lock", "no stack to spare");
    return;
  }
  const std::size_t bytes = std::min(
      kStackResidentBytes, static_cast<std::size_t>(here - low) - kStackSpare);
  auto* window = static_cast<volatile char*>(__builtin_alloca(bytes));
  const auto page = static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
  for (std::size_t offset = 0; offset < bytes; offset += page) {
    window[offset] = 0;
  }
  window[bytes - 1] = 0;
  char* const start = const_cast<char*>(window) -
                      reinterpret_cast<std::uintptr_t>(window) % page;
  if (::mlock(start, static_cast<std::size_t>(high - start)) != 0) {
    problems.add("stack mlock", std::strerror(errno));
  }
}

#if defined(__APPLE__)

// Prefaults the __TEXT segment of the image holding this code: the node
// binary for a C++ node, libtalos_node.so for a Python one, so the event
// loop, RTMS and devices code the loop runs does not fault on its first
// call. Read, not locked: macOS refuses mlock of executable pages (EPERM),
// even unprivileged within the wired limit. System libraries and the Python
// interpreter are left as they are.
void PrefaultText(Problems& problems) {
  Dl_info info{};
  if (::dladdr(reinterpret_cast<const void*>(&ConfigureRealtime), &info) == 0 ||
      info.dli_fbase == nullptr) {
    problems.add("text prefault", "image unknown");
    return;
  }
  const auto* header = static_cast<const mach_header_64*>(info.dli_fbase);
  const auto* command = reinterpret_cast<const load_command*>(header + 1);
  for (std::uint32_t i = 0; i < header->ncmds; ++i) {
    if (command->cmd == LC_SEGMENT_64) {
      const auto* segment =
          reinterpret_cast<const segment_command_64*>(command);
      if (std::strncmp(segment->segname, SEG_TEXT, sizeof segment->segname) ==
          0) {
        // __TEXT starts at the image header, at file offset 0.
        const auto* text = static_cast<const volatile char*>(info.dli_fbase);
        const auto page = static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
        for (std::uint64_t offset = 0; offset < segment->vmsize;
             offset += page) {
          (void)text[offset];
        }
        return;
      }
    }
    command = reinterpret_cast<const load_command*>(
        reinterpret_cast<const char*>(command) + command->cmdsize);
  }
  problems.add("text prefault", "no __TEXT segment");
}

#endif

// What ConfigureRealtime keeps resident. Linux: mlockall(MCL_CURRENT |
// MCL_FUTURE), which prefaults and locks every mapping, present and future.
// macOS implements mlockall as a stub that always fails with ENOSYS, so it
// works range by range instead, starting with the image's text (prefaulted
// only). Both then prefault and lock every shared-memory ring, now and as
// mapped later, and the loop thread's stack: redundant on Linux after
// mlockall, except that a ring mapped later is prefaulted for writing too,
// and that when mlockall is refused (past RLIMIT_MEMLOCK) the rings and
// stack are still locked one by one within the limit.
void LockMemory(Problems& problems) {
#if defined(__APPLE__)
  PrefaultText(problems);
#else
  if (::mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    problems.add("mlockall", std::strerror(errno));
  }
#endif
  const memory::Residency rings = memory::KeepMappingsResident();
  if (rings.failed != 0) {
    char reason[96];
    std::snprintf(reason, sizeof reason, "%zu of %zu: %s", rings.failed,
                  rings.ranges, std::strerror(rings.first_error));
    problems.add("ring mlock", reason);
  }
  LockStack(problems);
}

#if defined(__APPLE__)

void SetThreadPolicy(const RealtimeOptions& options, Problems& problems) {
  mach_timebase_info_data_t timebase{};
  if (::mach_timebase_info(&timebase) != KERN_SUCCESS || timebase.numer == 0) {
    problems.add("time constraint", "no mach timebase");
    return;
  }
  const std::int64_t period_ns =
      options.period.count() > 0 ? options.period.count() : 1'000'000;
  const auto to_abs = [&](std::int64_t ns) {
    return static_cast<std::uint32_t>(
        std::min<std::int64_t>(ns * timebase.denom / timebase.numer,
                               std::numeric_limits<std::uint32_t>::max()));
  };
  // Up to half the period of work, which must finish within the period.
  thread_time_constraint_policy_data_t policy{};
  policy.period = to_abs(period_ns);
  policy.computation = to_abs(period_ns / 2);
  policy.constraint = to_abs(period_ns);
  policy.preemptible = TRUE;
  const kern_return_t result = ::thread_policy_set(
      ::pthread_mach_thread_np(::pthread_self()), THREAD_TIME_CONSTRAINT_POLICY,
      reinterpret_cast<thread_policy_t>(&policy),
      THREAD_TIME_CONSTRAINT_POLICY_COUNT);
  if (result != KERN_SUCCESS) {
    problems.add("time constraint", ::mach_error_string(result));
  }
}

#else

void SetThreadPolicy(const RealtimeOptions& options, Problems& problems) {
  const int low = ::sched_get_priority_min(SCHED_FIFO);
  const int high = ::sched_get_priority_max(SCHED_FIFO);
  sched_param param{};
  param.sched_priority = std::clamp(options.priority, low, high);
  const int result =
      ::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &param);
  if (result != 0) {
    problems.add("SCHED_FIFO", std::strerror(result));
  }
}

#endif

#if defined(__APPLE__)

bool HasTimeConstraint() {
  thread_time_constraint_policy_data_t policy{};
  mach_msg_type_number_t count = THREAD_TIME_CONSTRAINT_POLICY_COUNT;
  boolean_t get_default = FALSE;
  const kern_return_t result = ::thread_policy_get(
      ::pthread_mach_thread_np(::pthread_self()), THREAD_TIME_CONSTRAINT_POLICY,
      reinterpret_cast<thread_policy_t>(&policy), &count, &get_default);
  return result == KERN_SUCCESS && get_default == FALSE;
}

void SetBackground(Problems& problems) {
  const int result = ::pthread_set_qos_class_self_np(QOS_CLASS_UTILITY, 0);
  if (result == 0) return;
  problems.add("QOS_CLASS_UTILITY", std::strerror(result));
  // The kernel refuses a QoS for a thread under an explicit Mach policy;
  // leaving the time-constraint policy at least stops it preempting loops.
  if (!HasTimeConstraint()) return;
  thread_standard_policy_data_t standard{};
  const kern_return_t dropped = ::thread_policy_set(
      ::pthread_mach_thread_np(::pthread_self()), THREAD_STANDARD_POLICY,
      reinterpret_cast<thread_policy_t>(&standard),
      THREAD_STANDARD_POLICY_COUNT);
  if (dropped != KERN_SUCCESS) {
    problems.add("standard policy", ::mach_error_string(dropped));
  }
}

#else

bool IsRealtimePolicy(int policy) {
  return policy == SCHED_FIFO || policy == SCHED_RR;
}

void SetBackground(Problems& problems) {
  int policy = SCHED_OTHER;
  sched_param param{};
  if (::pthread_getschedparam(::pthread_self(), &policy, &param) == 0 &&
      IsRealtimePolicy(policy)) {
    param = sched_param{};
    const int result =
        ::pthread_setschedparam(::pthread_self(), SCHED_OTHER, &param);
    if (result != 0) problems.add("SCHED_OTHER", std::strerror(result));
  }
  // Linux keeps a nice value per thread; PRIO_PROCESS with a thread id sets
  // only that thread's. Never lowered: that would need privilege.
  const auto tid = static_cast<id_t>(::syscall(SYS_gettid));
  errno = 0;
  const int nice = ::getpriority(PRIO_PROCESS, tid);
  if (errno == 0 && nice >= kBackgroundNice) return;
  if (::setpriority(PRIO_PROCESS, tid, kBackgroundNice) != 0) {
    problems.add("nice", std::strerror(errno));
  }
}

#endif

}  // namespace

bool RealtimeRequested(bool simulation) noexcept {
  const char* setting = std::getenv("TALOS_REALTIME");
  if (setting != nullptr && std::strcmp(setting, "0") == 0) return false;
  if (setting != nullptr && std::strcmp(setting, "1") == 0) return true;
  return !simulation;
}

bool ConfigureRealtime(const RealtimeOptions& options) noexcept {
  const char* setting = std::getenv("TALOS_REALTIME");
  if (setting != nullptr && std::strcmp(setting, "0") == 0) return false;

  Problems problems;
  LockMemory(problems);
  SetThreadPolicy(options, problems);
  if (problems.empty()) return true;
  std::fprintf(stderr, "talos: realtime scheduling incomplete: %s\n",
               problems.text());
  return false;
}

bool ConfigureBackgroundThread() noexcept {
  Problems problems;
  SetBackground(problems);
  if (problems.empty()) return true;
  std::fprintf(stderr, "talos: background scheduling incomplete: %s\n",
               problems.text());
  return false;
}

bool ThreadIsBackground() noexcept {
#if defined(__APPLE__)
  const qos_class_t qos = ::qos_class_self();
  return !HasTimeConstraint() && qos != QOS_CLASS_UNSPECIFIED &&
         qos <= QOS_CLASS_UTILITY;
#else
  int policy = SCHED_OTHER;
  sched_param param{};
  if (::pthread_getschedparam(::pthread_self(), &policy, &param) != 0 ||
      IsRealtimePolicy(policy)) {
    return false;
  }
  errno = 0;
  const int nice =
      ::getpriority(PRIO_PROCESS, static_cast<id_t>(::syscall(SYS_gettid)));
  return errno == 0 && nice >= kBackgroundNice;
#endif
}

}  // namespace talos::process
