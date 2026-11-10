#include "LoopWatchdog.h"

#include <frc/Threads.h>

#ifdef __linux__
#include <pthread.h>
#endif

#include <cstdio>
#include <utility>

namespace talos::rio {

LoopWatchdog::~LoopWatchdog() { Stop(); }

void LoopWatchdog::Beat(uint64_t now_us, uint64_t deadline_us) {
  beat_us_.store(now_us, std::memory_order_relaxed);
  deadline_us_.store(deadline_us, std::memory_order_release);
}

void LoopWatchdog::Poll(uint64_t now_us) {
  const uint64_t deadline = deadline_us_.load(std::memory_order_acquire);
  // Disarmed, or the loop still has time to neutralize the command itself.
  if (deadline == 0 || now_us < deadline) return;
  // Every poll until a beat moves the deadline: the loop may have re-applied
  // outputs since the last poll without beating (a tick on a stale clock
  // reading that stalled again), and a dropped send is retried.
  const bool neutral = outputs_.Neutralize();
  if (deadline == reported_deadline_us_) return;
  reported_deadline_us_ = deadline;
  const uint64_t stall = stalls_.fetch_add(1, std::memory_order_relaxed) + 1;
  // Possibly a newer beat's time than the deadline's; the text only.
  const uint64_t beat_us = beat_us_.load(std::memory_order_relaxed);
  char text[256];
  std::snprintf(
      text, sizeof(text),
      "RoboRIO loop stalled past an expired command: no tick for %llu ms, "
      "%llu ms past its deadline; watchdog %s every output each poll until "
      "it ticks (stall %llu)",
      static_cast<unsigned long long>(
          now_us > beat_us ? (now_us - beat_us) / 1000 : 0),
      static_cast<unsigned long long>((now_us - deadline) / 1000),
      neutral ? "neutralized" : "could not reach (retrying)",
      static_cast<unsigned long long>(stall));
  outputs_.Report(text);
}

void LoopWatchdog::Start(std::function<uint64_t()> clock,
                         int realtime_priority) {
  if (thread_.joinable()) return;
  stop_.store(false, std::memory_order_relaxed);
  thread_ = std::thread([this, clock = std::move(clock), realtime_priority] {
    Run(clock, realtime_priority);
  });
}

void LoopWatchdog::Stop() {
  if (!thread_.joinable()) return;
  stop_.store(true, std::memory_order_relaxed);
  thread_.join();
}

void LoopWatchdog::Run(const std::function<uint64_t()>& clock,
                       int realtime_priority) {
#ifdef __linux__
  // So a thread listing (ps -L) on the RoboRIO can tell it from the loop.
  pthread_setname_np(pthread_self(), "talos_watchdog");
#endif
  if (realtime_priority > 0 &&
      !frc::SetCurrentThreadPriority(true, realtime_priority)) {
    char text[128];
    std::snprintf(text, sizeof(text),
                  "RoboRIO loop watchdog: realtime priority %d refused; it "
                  "runs at the loop's priority and may starve with it",
                  realtime_priority);
    outputs_.Report(text);
  }
  while (!stop_.load(std::memory_order_relaxed)) {
    Poll(clock());
    std::this_thread::sleep_for(kPollPeriod);
  }
}

}  // namespace talos::rio
