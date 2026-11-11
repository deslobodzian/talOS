#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <thread>

// Diagnostics for the RoboRIO program, kept off its loop thread.
//
// That one thread enforces the command timeout and DIO/PWM neutral
// (Gateway::Tick); LoopWatchdog only backstops it once it stops ticking with
// a command in force. A report written from it (std::cerr is unit-buffered:
// one write syscall per <<) can block on the console, and FaultField bits such
// as the current limits toggle at the fault-field rate while the robot
// accelerates hard. So the loop thread only copies a fixed-size
// LogEvent into a preallocated bounded queue, never waiting: a full queue
// drops the event and counts it. A low-priority logger thread formats and
// writes, at most one line per second for each (kind, device): the first
// change prints at once, and the changes after it are coalesced into one line
// a second later with their count (and, for a fault field, the bits that
// toggled), then one such line per second for as long as they continue.
//
// The same thread prints a summary once in each second that saw dropped
// events or loop timing faults: overruns (a tick's work, from wake to sleep,
// longer than the period) and late ticks (one starting more than 1.5 periods
// after the previous, as when the loop is starved of the CPU, which delays
// command-timeout enforcement however short the work), with that window's
// longest work and longest interval. None has a field in State, so all stay
// RIO-local.
//
// Every 10 s it also prints where the backend's part of the ticks went
// (TickPhase), when that window had ticks: per phase the mean and longest
// time per tick, and the device counts. The loop thread adds a tick's phases
// with plain arithmetic and hands a finished window over on the first tick
// after the logger asks for it: relaxed stores between an acquire fence and a
// release store, once per window.
namespace talos::rio {

// Where the backend's share of a tick goes (PhoenixBackend measures it).
enum class TickPhase : uint8_t {
  kRefresh,  // BaseStatusSignal::RefreshAll over every device signal
  kDevices,  // per-device validity, latency and reset (HasResetOccurred) pass
  kControl,  // TalonFX SetControl for every motor (Apply or Neutral)
  kHealth,   // CAN status, battery voltage, brownout
  kOther,    // the rest of Read, Apply, ApplyOutputs and Neutral
  kCount,
};

// "[<seconds>.<milliseconds>] ", the prefix of this program's own console
// lines: `steady_us` on the steady clock, the gateway's clock (State's
// sample_time_us), which on Linux counts from kernel boot.
std::string ConsoleTime(uint64_t steady_us);

enum class LogKind : uint8_t {
  kMotorFault,      // FaultField (faults and limits) changed: value,
                    // previous, motion sample
  kMotorFeedback,   // motion feedback valid/invalid (flag = valid)
  kSensorFeedback,  // CAN sensor feedback valid/invalid (flag = valid)
  // Feedback four status periods old, or younger again (flag = late;
  // value = the spell's oldest age, previous = the threshold, both in us).
  kMotorFeedbackLate,
  kSensorFeedbackLate,
  kMotorFaultMask,  // faulted-motor mask changed: value, previous
  kBrownout,        // flag = active, a = battery volts
  kApplyControl,    // control send failed/recovered (flag = failed)
  kNeutralSend,     // NeutralOut send failing/recovered (flag = failed)
  kDeviceReset,     // device reset latched (flag = sensor)
  kMessage,         // text only; index names the message for rate limiting
  kCount,
};

// Copied by value into the queue: no pointer here may own memory. The
// status strings are the vendor library's static names (StatusCode::GetName)
// or string literals.
struct LogEvent {
  LogKind kind{LogKind::kMessage};
  // Device slot (motor or sensor index); with kind, the rate-limit key.
  uint16_t index{0};
  uint16_t id{0};
  int32_t can_id{0};
  bool flag{false};
  uint32_t value{0};
  uint32_t previous{0};
  uint32_t age_us{0};
  uint32_t age2_us{0};
  const char* status{nullptr};
  const char* status2{nullptr};
  double a{0.0}, b{0.0}, c{0.0}, d{0.0};
  double limit1{0.0}, limit2{0.0};
  char name[32]{};
  char bus[16]{};
  char text[160]{};

  // Bounded copies (truncating), for the loop thread: no allocation.
  void SetName(std::string_view value);
  void SetBus(std::string_view value);
  void SetText(std::string_view value);
};

class RioLog {
 public:
  using Sink = std::function<void(std::string_view line)>;
  static constexpr std::size_t kCapacity = 256;
  static constexpr uint64_t kWindowUs = 1'000'000;

  // Lines go to `sink` (default: stderr, each prefixed with ConsoleTime of
  // its writing, within one 20 ms drain of the event unless the logger
  // thread is starved), only ever from the thread that calls Drain/Flush:
  // the logger thread once Start() has run.
  explicit RioLog(Sink sink = {});
  ~RioLog();
  RioLog(const RioLog&) = delete;
  RioLog& operator=(const RioLog&) = delete;

  // Any thread; never blocks, allocates or writes. False (and counted in
  // dropped()) when the queue is full.
  bool Post(const LogEvent& event);

  // The loop thread, once per tick: `interval_us` from the previous tick's
  // start to this one's (0 for the first), `duration_us` from wake to sleep.
  // Work longer than `period_us` is an overrun; an interval longer than 1.5
  // periods is a late tick. Single writer.
  // It also closes the tick's phases (AddPhase): the tick joins the current
  // phase window, after handing the logger the previous window if it asked.
  void RecordTick(uint64_t interval_us, uint64_t duration_us,
                  uint64_t period_us);

  // The loop thread, during a tick: adds `duration_ns` to this tick's
  // `phase`, and records the device counts the phase line reports. Plain
  // arithmetic on loop-thread state; RecordTick closes the tick.
  void AddPhase(TickPhase phase, uint64_t duration_ns);
  void SetTickDevices(std::size_t motors, std::size_t sensors);
  static constexpr uint64_t kPhaseWindowUs = 10'000'000;

  // Starts the logger thread (nice 19 on Linux, best effort), which
  // drains every 20 ms. Stop() (also the destructor) drains, flushes the
  // coalesced lines and joins it.
  void Start();
  void Stop();

  // Pops every queued event and writes due lines as of `now_us` (a
  // steady-clock time). Called by the logger thread; tests call it directly
  // on a log that was not started.
  void Drain(uint64_t now_us);
  // Drain, then write every pending coalesced line regardless of its window.
  void Flush(uint64_t now_us);

  uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }
  uint64_t overrun_count() const {
    return overruns_.load(std::memory_order_relaxed);
  }
  uint64_t late_tick_count() const {
    return late_ticks_.load(std::memory_order_relaxed);
  }
  // The longest work and interval since the last summary.
  uint64_t max_tick_us() const {
    return max_tick_us_.load(std::memory_order_relaxed);
  }
  uint64_t max_interval_us() const {
    return max_interval_us_.load(std::memory_order_relaxed);
  }

  // Decode a TalonFX FaultField (SPN 0x34c, frame 0xad).
  static std::string DecodeTalonFaultBits(uint32_t fault_flags);
  static std::string Format(const LogEvent& event);

 private:
  // Bounded multi-producer queue (Vyukov): one sequence number per cell, so
  // a producer claims a cell with one compare-exchange and never waits on the
  // consumer. The configure worker posts too, besides the loop thread.
  struct Cell {
    std::atomic<std::size_t> sequence{0};
    LogEvent event{};
  };
  bool Pop(LogEvent& event);
  void Handle(const LogEvent& event, uint64_t now_us);
  void WriteDue(uint64_t now_us, bool force);
  void Run();

  struct Window {
    bool open{false};
    uint64_t start_us{0};
    uint64_t suppressed{0};
    uint32_t toggled{0};
    LogEvent last{};
  };
  void WriteCoalesced(Window& window, uint64_t now_us);
  static constexpr std::size_t kKeysPerKind = 32;

  Sink sink_;
  std::unique_ptr<std::array<Cell, kCapacity>> cells_;
  alignas(64) std::atomic<std::size_t> enqueue_{0};
  alignas(64) std::atomic<std::size_t> dequeue_{0};
  std::atomic<uint64_t> dropped_{0};
  std::atomic<uint64_t> overruns_{0};
  std::atomic<uint64_t> late_ticks_{0};
  std::atomic<uint64_t> max_tick_us_{0};
  std::atomic<uint64_t> max_interval_us_{0};
  std::atomic<uint64_t> period_us_{0};

  // Tick phases. Loop-thread state: this tick's phases and the open window.
  static constexpr std::size_t kPhaseCount =
      static_cast<std::size_t>(TickPhase::kCount);
  struct PhaseWindow {
    uint64_t ticks{0};
    std::array<uint64_t, kPhaseCount> sum_ns{};
    std::array<uint64_t, kPhaseCount> max_ns{};
    uint32_t motors{0};
    uint32_t sensors{0};
  };
  std::array<uint64_t, kPhaseCount> tick_phase_ns_{};
  uint32_t tick_motors_{0};
  uint32_t tick_sensors_{0};
  PhaseWindow phase_window_{};
  uint32_t phase_served_local_{0};
  // The handed-over window: the loop thread stores it, then `phase_served_`
  // (release) once `phase_request_` differs from what it last served.
  std::atomic<uint32_t> phase_request_{0};
  std::atomic<uint32_t> phase_served_{0};
  std::atomic<uint64_t> published_ticks_{0};
  std::array<std::atomic<uint64_t>, kPhaseCount> published_sum_ns_{};
  std::array<std::atomic<uint64_t>, kPhaseCount> published_max_ns_{};
  std::atomic<uint32_t> published_motors_{0};
  std::atomic<uint32_t> published_sensors_{0};
  void WritePhases(uint64_t now_us, bool force);

  // Logger-thread state.
  bool phase_clock_started_{false};
  bool phase_awaiting_{false};
  uint32_t phase_requested_{0};
  uint64_t next_phase_us_{0};
  std::unique_ptr<std::array<std::array<Window, kKeysPerKind>,
                             static_cast<std::size_t>(LogKind::kCount)>>
      windows_;
  uint64_t reported_dropped_{0};
  uint64_t reported_overruns_{0};
  uint64_t reported_late_ticks_{0};
  uint64_t next_summary_us_{0};

  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace talos::rio
