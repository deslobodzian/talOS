#include "talOS/process/process.h"

#include <pthread.h>
#include <sys/mman.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

#if defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_error.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#else
#include <sched.h>
#endif

namespace talos::process {
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

void LockMemory(Problems& problems) {
#if defined(__APPLE__)
  // macOS implements mlockall as a stub that always fails with ENOSYS, so
  // asking would only make every node on the coprocessor print a warning.
  (void)problems;
#else
  if (::mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    problems.add("mlockall", std::strerror(errno));
  }
#endif
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

}  // namespace

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

}  // namespace talos::process
