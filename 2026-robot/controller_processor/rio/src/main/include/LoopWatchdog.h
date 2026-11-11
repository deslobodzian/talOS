#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <string_view>
#include <thread>

// A backstop for the RoboRIO loop (Robot.cpp), the one thread that enforces
// the command timeout and output neutral (Gateway::Tick). If that loop stops
// ticking (starved of the CPU, or blocked) right after it applied a command,
// nothing else would stop the outputs: TalonFX controls are one-shot and
// their device-side expiry is unmeasured, and DIO and PWM outputs hold their
// last value. So each tick ends with a beat carrying the deadline by which
// the loop's next tick would have neutralized the command in force (its
// expiry plus one period), and a thread of its own, at a realtime priority
// above the loop, neutralizes every output on every poll once that deadline
// has passed, until a beat moves it.
//
// It does not fight a running loop: a loop on time beats a later deadline
// before each one passes. Neutralizing on every poll, not once per stall,
// also covers a loop that resumes on a stale clock reading, re-applies a
// command and stalls again before its next beat.
namespace talos::rio {

// What the watchdog acts on: the RoboRIO's outputs, a recorder in tests.
class WatchdogOutputs {
 public:
  virtual ~WatchdogOutputs() = default;
  // Every motor NeutralOut, every DIO output false, every PWM output
  // disabled; idempotent. False when an output could not be reached (the
  // devices are being rebuilt by a configure, or a send failed); the
  // watchdog retries on its next poll.
  virtual bool Neutralize() = 0;
  // Once per deadline passed, after the first Neutralize attempt.
  virtual void Report(std::string_view text) = 0;
};

class LoopWatchdog {
 public:
  // How often the thread checks the beat: a passed deadline is acted on
  // within this of passing.
  static constexpr std::chrono::microseconds kPollPeriod{5000};

  explicit LoopWatchdog(WatchdogOutputs& outputs) : outputs_{outputs} {}
  ~LoopWatchdog();
  LoopWatchdog(const LoopWatchdog&) = delete;
  LoopWatchdog& operator=(const LoopWatchdog&) = delete;

  // The loop thread, every tick, after the gateway ran: `now_us` is the
  // tick's time and `deadline_us` the time by which the loop would have
  // neutralized the command still in force, or 0 when none is (this tick
  // neutralized every output itself), which disarms the watchdog until a
  // beat arms it. Both on one clock. Never blocks.
  void Beat(uint64_t now_us, uint64_t deadline_us);

  // The watchdog thread, every kPollPeriod; tests call it directly on a
  // watchdog that was not started. `now_us` is on the clock Beat uses.
  void Poll(uint64_t now_us);

  // Starts the thread, which reads `clock` (microseconds, the clock the
  // loop beats with). `realtime_priority` 1-99 asks for SCHED_FIFO at that
  // priority (frc::SetCurrentThreadPriority); 0 keeps the default
  // scheduling. A refused request is reported once and the thread runs on.
  // Returns once the thread has made that request (or after
  // kStartHandshakeTimeout), so held_priority() is then the outcome.
  // Stop() (also the destructor) joins it.
  void Start(std::function<uint64_t()> clock, int realtime_priority);
  void Stop();

  // The SCHED_FIFO priority the watchdog thread holds, read back from the
  // kernel after its request; 0 when it holds none (refused, not asked,
  // not started, or not on Linux). The loop raises itself only below it.
  int held_priority() const {
    return held_priority_.load(std::memory_order_acquire);
  }

  // How long Start waits for the thread's priority request.
  static constexpr std::chrono::milliseconds kStartHandshakeTimeout{1000};

  // Deadlines passed (stalls) since construction.
  uint64_t stall_count() const {
    return stalls_.load(std::memory_order_relaxed);
  }

 private:
  void Run(const std::function<uint64_t()>& clock, int realtime_priority,
           std::promise<void>& requested);

  WatchdogOutputs& outputs_;
  // The latest beat's deadline, 0 while disarmed: one word, so the watchdog
  // never pairs one beat's time with another's arming.
  std::atomic<uint64_t> deadline_us_{0};
  // The latest beat's time, for the report only.
  std::atomic<uint64_t> beat_us_{0};

  // Poll-side state: the deadline last reported.
  uint64_t reported_deadline_us_{0};
  std::atomic<uint64_t> stalls_{0};

  std::atomic<int> held_priority_{0};
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace talos::rio
