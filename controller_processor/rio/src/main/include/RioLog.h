#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
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
// longest work, that single worst-work tick's 8-phase split (published
// lock-free through an atomic seqlock), and longest interval. None has a field
// in State, so all stay RIO-local.
//
// Every 10 s (and on Flush()/Stop() for the final partial window) it also
// prints where the ticks' work went (TickPhase), when that window had ticks:
// per phase the mean and longest time per tick, and the device counts. The
// loop thread adds a tick's phases with plain arithmetic and hands a finished
// window over on the first tick after the logger asks for it: relaxed stores
// between an acquire load of the request and a release store, once per window.
//
// Beside that line, the same window's latency line (REQ-RT-25): the p50, p99
// and max of each drained command's observation age (accepted and refused
// apart) and of each sent Driver Station snapshot's refresh age, all on the
// RoboRIO's own clock. The loop records at most one of each per tick; the
// tick folds them into its window at RecordTick. Each figure is a fixed
// histogram (kLatencyBuckets of kLatencyBucketUs, an overflow bucket and the
// exact max), double-buffered: the handover only switches the loop to the
// other buffer, which the logger cleared after printing it.
namespace talos::rio {

// Where a tick's work goes. PhoenixBackend measures the backend's phases
// (AddPhase); the loop (Robot.cpp) measures its own between boundaries
// (BeginTick, ClosePhase), less the backend's phases nested inside them, so
// a tick's phases sum to its work, wake to sleep, as RecordTick receives it.
enum class TickPhase : uint8_t {
  kRefresh,  // BaseStatusSignal::RefreshAll, one per CAN network
  kDevices,  // per-device validity, latency and reset (HasResetOccurred) pass
  kControl,  // TalonFX SetControl for every motor (Apply or Neutral)
  kHealth,   // CAN status, battery voltage, brownout
  kOther,    // the rest of the backend's phases in Read, Apply, ApplyOutputs
             // and Neutral
  // The loop's: the socket retry, the endpoint's tick (receive, the
  // gateway's own command and State work, the State send) or, with no socket
  // open, the gateway's tick alone, and the Driver Station packet's send,
  // less the backend phases inside them.
  kLink,
  // The loop's: reading the Driver Station (the snapshot the talos-ds
  // thread last published, DriverStationReader::Read, and the mode
  // generation; the desktop simulation's Driver Station) and copying and
  // encoding that snapshot as its packet. The refresh, the snapshot and the
  // mode report are that thread's, outside every phase.
  kDriverStation,
  // The loop's: the console's checks (LoopConsole), the watchdog beat, the
  // companion report, the config persister's tick.
  kLoop,
  kCount,
};

// "[<seconds>.<milliseconds>] ", the prefix of this program's own console
// lines and Driver Station messages: `steady_us` on the steady clock, the
// gateway's clock (State's sample_time_us), which on Linux counts from kernel
// boot.
std::string ConsoleTime(uint64_t steady_us);

// The same prefix in a caller's buffer, NUL-terminated, for a thread that
// must not allocate (the loop watchdog's).
inline constexpr std::size_t kConsoleTimeSize = 32;
void FormatConsoleTime(uint64_t steady_us, char (&out)[kConsoleTimeSize]);

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
  // The loop thread: the tick's work starts at `now_ns` (steady clock), its
  // first phase boundary.
  void BeginTick(uint64_t now_ns);
  // The loop thread: adds to `phase` the time from the previous boundary
  // (BeginTick or ClosePhase) to `now_ns`, less what AddPhase added since
  // (the backend's phases, nested in the loop's calls), and makes `now_ns`
  // the next boundary. A tick closed up to its last instant this way has
  // phases that sum to its work.
  void ClosePhase(TickPhase phase, uint64_t now_ns);
  void SetTickDevices(std::size_t motors, std::size_t sensors);
  static constexpr uint64_t kPhaseWindowUs = 10'000'000;

  // The loop thread, at most once per tick each (a second call in one tick
  // folds the first into the window at once): the age, on the RoboRIO's
  // clock, of the command the endpoint's tick drained, at the end of that
  // tick, and whether the gateway accepted it; and the age of the sent Driver
  // Station snapshot's refresh at its send. Plain stores; RecordTick adds them
  // to the tick's window. An age of 2^63 us or more is a difference that
  // wrapped (an observation ahead of the clock: one from before an OS
  // reboot, a corrupt stamp) and is not recorded. A previous program run's
  // command is no such case: the clock runs on across a program restart, so
  // its observation lies in the past and it counts with its age.
  void RecordCommandAge(uint64_t age_us, bool accepted);
  void RecordDriverStationAge(uint64_t age_us);
  static constexpr std::size_t kLatencyBuckets = 512;
  static constexpr uint64_t kLatencyBucketUs = 100;

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
  // Writes ConsoleTime(steady_us) + line to `stream` using a stack buffer so
  // the default sink allocates nothing.
  static void WriteConsoleLine(uint64_t steady_us, std::string_view line,
                               std::FILE* stream) noexcept;

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
  // This tick's phases summed, and that sum and the time at the last
  // boundary.
  uint64_t tick_total_ns_{0};
  uint64_t boundary_total_ns_{0};
  uint64_t boundary_ns_{0};
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
  // Worst-work tick of the current 1 s timing summary window. The logger
  // advances timing_epoch_ (release) on each 1 s summary; the loop thread
  // resets local_worst_work_us_ when the epoch changes and publishes the new
  // worst tick's phases via an atomic seqlock (worst_seq_ odd acq_rel ->
  // release atomic stores -> worst_seq_ even release) so Drain() reads a
  // consistent snapshot without waiting for a handover tick.
  struct WorstSnapshot {
    bool valid{false};
    uint64_t epoch{0};
    uint64_t work_us{0};
    std::array<uint64_t, kPhaseCount> phase_ns{};
  };
  WorstSnapshot ReadWorstTick() const;
  std::atomic<uint64_t> timing_epoch_{1};
  uint64_t local_timing_epoch_{0};
  uint64_t local_worst_work_us_{0};
  std::atomic<uint64_t> worst_seq_{0};
  std::atomic<uint64_t> worst_published_epoch_{0};
  std::atomic<uint64_t> worst_published_us_{0};
  std::array<std::atomic<uint64_t>, kPhaseCount> worst_phase_ns_{};
  void WritePhases(uint64_t now_us, bool force);

  // Latency: per figure, counts per kLatencyBucketUs bucket and one
  // overflow bucket (ages from kLatencyBuckets * kLatencyBucketUs), with the
  // sample count and the exact max.
  struct LatencyHistogram {
    std::array<uint32_t, kLatencyBuckets + 1> counts{};
    uint64_t samples{0};
    uint64_t max_us{0};
    void Add(uint64_t age_us);
    // "<p50>/<p99>/<max> us over <n>", or "-/-/- us over 0".
    std::string Summary() const;
  };
  struct LatencyWindow {
    LatencyHistogram accepted;
    LatencyHistogram refused;
    LatencyHistogram driver_station;
  };
  void EmitPhasesAndLatency(const PhaseWindow& window,
                            const LatencyWindow& latency);
  struct PendingAge {
    bool present{false};
    bool accepted{false};
    uint64_t age_us{0};
  };
  void FoldCommandAge();
  void FoldDriverStationAge();
  // This tick's ages, folded at RecordTick after any handover.
  PendingAge tick_command_age_{};
  PendingAge tick_driver_station_age_{};
  // The loop writes latency_[latency_front_]; at a handover it moves to the
  // other buffer and names the one it left in latency_published_, which the
  // logger reads after phase_served_'s acquire and clears before its next
  // request (release), which the loop loads with acquire before it switches
  // back, so that switch finds it empty.
  std::array<LatencyWindow, 2> latency_{};
  uint32_t latency_front_{0};
  uint32_t latency_published_{0};

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
