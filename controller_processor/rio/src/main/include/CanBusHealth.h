#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string_view>
#include <thread>

#include "hardware/config.h"
#include "hardware/messages.h"

// State's CAN health (HealthSample::can_bus): one slot per bus
// hardware::CanBuses lists for the applied configuration, the RoboRIO's own
// bus ("rio") in slot 0 and each CANivore after it, and the gateway asks for
// one slot per State in turn.
//
// "rio" is read with HAL_CAN_GetCANStatus and always has a slot: the power
// distribution and pneumatics modules, which no configuration names, share
// it. Every other bus a motor or sensor names is a CANivore (its name or
// serial number, or "*"), read with Phoenix 6's CANBus::GetStatus, which
// blocks about 1 ms on the bench (Phoenix 26.3 documents "up to 1 ms"). Both
// are vendor library calls the 5 ms loop must not make (the HAL call reaches
// the NetComm library, and frc::RobotController::GetCANStatus throws on a
// failed one), so a reader thread (CanBusSampler) reads one bus per
// kSamplePeriodUs, the configured buses in turn, and publishes each read on
// the configuration's Board; the loop (Poll) only copies the published
// reads, under the Board's mutex, which it try-locks.
//
// Each slot's read state, judged on the loop's clock (Poll):
// - kOk: its last read succeeded (an OK status and a finite, non-negative
//   utilization) and is current;
// - kPending: never read, and less than StaleUs since the loop's first tick
//   under this configuration;
// - kFailed: its last read failed and is current;
// - kStale: its last read started StaleUs (kStaleReads of its own sampling
//   periods: kStaleReads * buses * kSamplePeriodUs) or more before the tick,
//   or it was never read and that long has passed since the first tick, so a
//   hung vendor call or a starved reader never leaves an old figure standing
//   as current.
// In any state but kOk the figures are the last good read's (0 before one);
// read_failure_count counts the failed reads since the configuration was
// applied. PhoenixBackend prints each bus's failing spell, its start and its
// end, from Poll's `on_change`.
//
// Under WPILib simulation no bus is read (Phoenix simulates every device
// whatever its bus) and every slot reads kSimulated with zero figures; a
// configuration with no device at all reads no bus either, and its "rio"
// slot stays kNone. Neither starts a reader thread.
namespace talos::rio {

class CanBusHealth {
 public:
  static constexpr uint64_t kSamplePeriodUs = 100'000;
  static constexpr uint64_t kStaleReads = 3;
  static constexpr std::size_t kMaxBuses = hardware::kMaxCanBuses;
  // A bus name is at most 63 bytes (hardware::Validate).
  static constexpr std::size_t kMaxNameSize = 64;

  // One bus status read: whether its status was OK, the utilization it
  // reported (0 to 1) and its counters as the vendor reports them, and the
  // vendor status name (a static string) for the console.
  struct Sample {
    bool ok{false};
    double utilization_frac{0.0};
    uint32_t bus_off_count{0};
    uint32_t transmit_buffer_full_count{0};
    uint32_t receive_error_count{0};
    uint32_t transmit_error_count{0};
    const char* status{nullptr};
  };

  // The RoboRIO bus's read from what HAL_CAN_GetCANStatus returned: a
  // nonzero `status` is a failed read, never an exception.
  static Sample FromHal(float utilization_frac, uint32_t bus_off_count,
                        uint32_t transmit_buffer_full_count,
                        uint32_t receive_error_count,
                        uint32_t transmit_error_count, int32_t status) {
    if (status != 0) return {.status = "HAL_CAN_GetCANStatus failed"};
    return {true,
            utilization_frac,
            bus_off_count,
            transmit_buffer_full_count,
            receive_error_count,
            transmit_error_count,
            "OK"};
  }

  enum class Event : uint8_t {
    // The bus's slot stopped reading kOk: a failed read, or stale.
    kFailing,
    // It reads kOk again, ending the spell.
    kRecovered,
  };

  // What Poll reports for bus `bus`: the event; the last read's status name
  // (nullptr for a bus never read); whether a kFailing was for staleness
  // rather than a failed read; and the failed reads in the spell so far
  // (kRecovered: all of them).
  struct Change {
    std::size_t bus{0};
    Event event{Event::kFailing};
    const char* status{nullptr};
    bool stale{false};
    uint64_t spell_failures{0};
  };

 private:
  struct Published {
    bool read{false};
    bool ok{false};
    // The last good read's figures; read_state unused.
    hardware::CanBusSample figures{};
    uint64_t started_us{0};
    const char* status{nullptr};
    uint64_t failed_reads{0};
  };

 public:
  // One configuration's buses and what its reader published. Shared
  // (std::shared_ptr) by CanBusHealth, which reads it until a new
  // configuration replaces it, and by that configuration's reader thread,
  // which keeps it alive for as long as a vendor call takes: a reader
  // abandoned in a read (CanBusSampler::Stop) writes only here, where nobody
  // looks any more. The bus names never change once the Board is built.
  class Board {
   public:
    std::size_t bus_count() const { return count_; }
    std::string_view bus(std::size_t index) const {
      return {names_[index].data(), sizes_[index]};
    }

    // The reader: unless stopped, reads the next bus in turn, `now_us`
    // (steady clock) being the read's start, with `read(bus_name)`
    // (returning Sample), and publishes it. The mutex is held only to pick
    // the bus and to publish, never across `read`. False once stopped,
    // without reading: no read starts after StopAndWait. Never allocates.
    template <typename ReadBus>
    bool SampleNext(uint64_t now_us, ReadBus&& read) {
      std::size_t index = 0;
      {
        const std::lock_guard lock{mutex_};
        if (stop_) return false;
        index = next_;
        next_ = (next_ + 1) % count_;
        reading_ = true;
        reading_index_ = index;
      }
      const Sample sample = read(bus(index));
      const bool good = sample.ok && std::isfinite(sample.utilization_frac) &&
                        sample.utilization_frac >= 0.0;
      const std::lock_guard lock{mutex_};
      reading_ = false;
      Published& out = published_[index];
      out.read = true;
      out.ok = good;
      out.started_us = now_us;
      out.status = sample.status;
      if (good) {
        const auto saturate = [](uint32_t count) {
          return static_cast<uint8_t>(std::min<uint32_t>(count, 255));
        };
        out.figures.utilization_frac =
            static_cast<float>(sample.utilization_frac);
        out.figures.bus_off_count = sample.bus_off_count;
        out.figures.transmit_buffer_full_count =
            sample.transmit_buffer_full_count;
        out.figures.receive_error_count = saturate(sample.receive_error_count);
        out.figures.transmit_error_count =
            saturate(sample.transmit_error_count);
      } else {
        ++out.failed_reads;
      }
      return true;
    }

    // The reader thread, between reads: waits `period` or until stopped;
    // false once stopped.
    bool WaitPeriod(std::chrono::microseconds period) {
      std::unique_lock lock{mutex_};
      wake_.wait_for(lock, period, [this] { return stop_; });
      return !stop_;
    }

    // The reader thread, as it ends: marks the reader exited. True when it
    // had been abandoned by then.
    bool Exit() {
      const std::lock_guard lock{mutex_};
      exited_ = true;
      exit_.notify_all();
      return abandoned_;
    }

    bool exited() {
      const std::lock_guard lock{mutex_};
      return exited_;
    }

    // Stops the reader and waits up to `bound` for it to exit: true when it
    // did. Otherwise marks it abandoned and calls `on_abandon(bus)` under the
    // mutex, `bus` the bus whose read is in flight (empty when none is),
    // before the reader can see that it was abandoned, and returns false.
    template <typename OnAbandon>
    bool StopAndWait(std::chrono::microseconds bound, OnAbandon&& on_abandon) {
      std::unique_lock lock{mutex_};
      stop_ = true;
      wake_.notify_all();
      if (exit_.wait_for(lock, bound, [this] { return exited_; })) return true;
      abandoned_ = true;
      on_abandon(reading_ ? bus(reading_index_) : std::string_view{});
      return false;
    }

   private:
    friend class CanBusHealth;
    // Building only, before the Board is shared.
    void Add(std::string_view name) {
      if (count_ == kMaxBuses || name.size() >= kMaxNameSize) return;
      std::memcpy(names_[count_].data(), name.data(), name.size());
      sizes_[count_] = name.size();
      ++count_;
    }

    std::array<std::array<char, kMaxNameSize>, kMaxBuses> names_{};
    std::array<std::size_t, kMaxBuses> sizes_{};
    std::size_t count_{0};

    // Under the mutex.
    std::mutex mutex_;
    std::condition_variable wake_;
    std::condition_variable exit_;
    bool stop_{false};
    bool exited_{false};
    bool abandoned_{false};
    bool reading_{false};
    std::size_t reading_index_{0};
    std::size_t next_{0};
    std::array<Published, kMaxBuses> published_{};
  };

  // Builds a new Board for `config` (validated), its buses in
  // hardware::CanBuses order; none when `simulation` or for a configuration
  // with no device at all. Forgets every read and spell of the previous one,
  // whose reader, if any, writes only into the previous Board. Allocates;
  // never runs beside Poll (Gateway reads no State while it configures).
  void Configure(const hardware::Config& config, bool simulation) {
    board_.reset();
    const hardware::CanBusList buses = hardware::CanBuses(config);
    simulated_buses_ = simulation ? buses.count : 0;
    const bool any_device =
        !config.motors.empty() || !config.sensors.empty() ||
        !config.digital_inputs.empty() || !config.digital_outputs.empty() ||
        !config.analog_inputs.empty() || !config.encoders.empty() ||
        !config.pwm_outputs.empty();
    if (!simulation && any_device) {
      auto board = std::make_shared<Board>();
      for (std::size_t i = 0; i < buses.count; ++i) board->Add(buses.names[i]);
      board_ = std::move(board);
    }
    snapshot_.fill({});
    samples_.fill({});
    spells_.fill({});
    polled_ = false;
    first_poll_us_ = 0;
  }

  // The current Board, for its reader; null when no bus is read.
  std::shared_ptr<Board> board() const { return board_; }

  // Fixed between Configure calls.
  std::size_t bus_count() const { return board_ ? board_->count_ : 0; }
  std::string_view bus(std::size_t index) const { return board_->bus(index); }
  // The age at which a bus's last read no longer counts.
  uint64_t StaleUs() const {
    return kStaleReads * bus_count() * kSamplePeriodUs;
  }

  // Board::SampleNext on the current Board (which the read keeps alive, even
  // across a Configure), for a test to drive a reader step by step.
  template <typename ReadBus>
  void SampleNext(uint64_t now_us, ReadBus&& read) {
    const std::shared_ptr<Board> board = board_;
    if (board) board->SampleNext(now_us, read);
  }

  // The loop thread, each Read at `now_us` (steady clock): copies what the
  // reader published (try-locking the Board's mutex: when the reader holds
  // it, the previous tick's copy serves), judges each bus's slot (Slot),
  // and calls `on_change(Change)` for each bus whose slot stopped or started
  // reading kOk. Never waits, allocates or calls into a vendor library.
  template <typename OnChange>
  void Poll(uint64_t now_us, OnChange&& on_change) {
    if (!board_) return;
    Board& board = *board_;
    const std::size_t count = board.count_;
    {
      const std::unique_lock lock{board.mutex_, std::try_to_lock};
      if (lock.owns_lock())
        for (std::size_t i = 0; i < count; ++i)
          snapshot_[i] = board.published_[i];
    }
    if (!polled_) {
      polled_ = true;
      first_poll_us_ = now_us;
    }
    const uint64_t stale_us = StaleUs();
    for (std::size_t i = 0; i < count; ++i) {
      const Published& p = snapshot_[i];
      // A read stamped after this tick's clock reading is current.
      const uint64_t since = p.read ? p.started_us : first_poll_us_;
      const bool fresh = since > now_us || now_us - since < stale_us;
      hardware::CanBusSample& sample = samples_[i];
      sample = p.figures;
      sample.read_failure_count = static_cast<uint32_t>(std::min<uint64_t>(
          p.failed_reads, std::numeric_limits<uint32_t>::max()));
      if (!fresh) {
        sample.read_state = hardware::CanBusReadState::kStale;
      } else if (!p.read) {
        sample.read_state = hardware::CanBusReadState::kPending;
      } else {
        sample.read_state = p.ok ? hardware::CanBusReadState::kOk
                                 : hardware::CanBusReadState::kFailed;
      }
      const bool ok = sample.read_state == hardware::CanBusReadState::kOk;
      Spell& spell = spells_[i];
      if (ok && spell.failing) {
        spell.failing = false;
        on_change(Change{i, Event::kRecovered, p.status, false,
                         p.failed_reads - spell.failed_reads_before});
      } else if (!ok && !spell.failing && (p.read || !fresh)) {
        spell.failing = true;
        const bool failed_read = fresh && p.read && !p.ok;
        spell.failed_reads_before = p.failed_reads - (failed_read ? 1 : 0);
        on_change(Change{i, Event::kFailing, p.status, !failed_read,
                         failed_read ? 1u : 0u});
      }
    }
  }

  // Slot `index`'s health as the last Poll judged it: kSimulated under
  // simulation, kNone for a slot past the configuration's buses or when no
  // bus is read.
  hardware::CanBusSample Slot(std::size_t index) const {
    if (index < simulated_buses_) {
      return {.read_state = hardware::CanBusReadState::kSimulated};
    }
    if (index >= bus_count()) return {};
    return samples_[index];
  }

 private:
  struct Spell {
    bool failing{false};
    uint64_t failed_reads_before{0};
  };

  std::shared_ptr<Board> board_;
  std::size_t simulated_buses_{0};
  // Loop-thread state.
  std::array<Published, kMaxBuses> snapshot_{};
  std::array<hardware::CanBusSample, kMaxBuses> samples_{};
  std::array<Spell, kMaxBuses> spells_{};
  bool polled_{false};
  uint64_t first_poll_us_{0};
};

// The reader thread that reads the configured buses for CanBusHealth.
//
// Reconfigure, on PhoenixBackend's configure worker and outside its device
// lock, stops the reader, applies the new configuration, and starts a reader
// for its Board: for every configuration with a device ("rio" is always
// read), none under simulation or for a configuration with no device. The
// reader calls Board::SampleNext with `read` (HAL_CAN_GetCANStatus for "rio",
// Phoenix's CANBus::GetStatus for a CANivore), then waits `period_us` or
// until stopped. No read starts once Stop has begun, so no bus the new
// configuration dropped is read after Reconfigure returns, except a read
// already in flight.
//
// Stop (from Reconfigure and the destructor) waits for the reader to exit for
// at most `stop_bound` (kStopBound, 50 ms: fifty times the ~1 ms a read
// takes). A reader that has not exited by then, a vendor call that did not
// return, is abandoned: it is left running, `report` gets one line naming the
// bus in flight, and Stop returns. An abandoned reader owns all it can still
// touch: its Board (whose late result nobody reads), a copy of `read`, the
// clock (a function pointer) and the StuckReaders count, which it decrements
// as it exits. It never touches the sampler, CanBusHealth or the backend, so
// neither a configure, the device lock nor destruction ever waits on it. At
// most kMaxStuckReaders abandoned readers may still be blocked: while that
// many are, Reconfigure starts no new reader (`report` says so once per
// configure), and every bus of that configuration reads stale. An abandoned
// reader that exits is joined at the next Reconfigure or at destruction; one
// still blocked at destruction is detached.
//
// On Linux the reader names itself talos-canbus and puts itself on
// SCHED_OTHER, whatever the thread that started it, so it always runs below
// the loop's SCHED_FIFO; its nice is its starter's (0: the configure worker
// is started by the loop thread, whose SCHED_RESET_ON_FORK gives it the
// default scheduling, or before the loop raises itself). Not nice 19 like the
// logger: a starved reader goes stale, and /hw/status says so. Its own code
// allocates nothing once started. Phoenix's CANBus::GetStatus builds a
// std::string of the bus name (no allocation for a name of 15 bytes or
// fewer, as "Shooter") before an indirect call into the vendor's CAN
// library, whose allocations are not known.
class CanBusSampler {
 public:
  using Read = std::function<CanBusHealth::Sample(std::string_view bus)>;
  // Steady-clock microseconds, the clock of PhoenixBackend's Read.
  using Clock = uint64_t (*)();
  // One console line; called on the configure worker or the destroying
  // thread, never the loop.
  using Report = std::function<void(std::string_view line)>;
  static constexpr std::chrono::microseconds kStopBound{50'000};
  static constexpr int kMaxStuckReaders = 2;

  // The abandoned readers still blocked in a read. Shared with them, so the
  // count outlives the sampler.
  class StuckReaders {
   public:
    int count() {
      const std::lock_guard lock{mutex_};
      return count_;
    }
    // Waits until none is blocked, for at most `bound`; false if one still
    // is. For a test, which must not end with a reader still running.
    bool WaitUntilNone(std::chrono::milliseconds bound) {
      std::unique_lock lock{mutex_};
      return changed_.wait_for(lock, bound, [this] { return count_ <= 0; });
    }

   private:
    friend class CanBusSampler;
    void Add() {
      const std::lock_guard lock{mutex_};
      ++count_;
    }
    void Exited() {
      const std::lock_guard lock{mutex_};
      --count_;
      changed_.notify_all();
    }
    std::mutex mutex_;
    std::condition_variable changed_;
    int count_{0};
  };

  CanBusSampler(
      CanBusHealth& health, Read read, Clock clock, Report report,
      uint64_t period_us = CanBusHealth::kSamplePeriodUs,
      std::chrono::microseconds stop_bound = kStopBound,
      std::shared_ptr<StuckReaders> stuck = std::make_shared<StuckReaders>());
  ~CanBusSampler();
  CanBusSampler(const CanBusSampler&) = delete;
  CanBusSampler& operator=(const CanBusSampler&) = delete;

  // The configure worker. When the reader cannot be created no bus is read,
  // and Poll reports each bus stale once StaleUs has passed: a configure does
  // not fail for want of CAN health.
  void Reconfigure(const hardware::Config& config, bool simulation);
  void Stop();
  bool running() const { return thread_.joinable(); }

 private:
  // The reader thread. Everything it touches it owns, so it can be abandoned
  // in `read` and still finish safely.
  static void Run(const std::shared_ptr<CanBusHealth::Board>& board,
                  const Read& read, Clock clock, uint64_t period_us,
                  const std::shared_ptr<StuckReaders>& stuck);
  // Joins the abandoned readers that have exited, freeing their slots.
  void Reap();

  CanBusHealth& health_;
  const Read read_;
  const Clock clock_;
  const Report report_;
  const uint64_t period_us_;
  const std::chrono::microseconds stop_bound_;
  const std::shared_ptr<StuckReaders> stuck_;
  // The running reader and its Board.
  std::thread thread_;
  std::shared_ptr<CanBusHealth::Board> board_;
  struct Abandoned {
    std::thread thread;
    std::shared_ptr<CanBusHealth::Board> board;
  };
  std::array<Abandoned, kMaxStuckReaders> abandoned_{};
};

}  // namespace talos::rio
