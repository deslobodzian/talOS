#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string_view>
#include <thread>

#include "hardware/config.h"

// State's CAN health (HealthSample::can_utilization_frac, 0 to 1): the
// utilization of the busiest CAN bus still read, among the RoboRIO's own and
// every CANivore the applied configuration has a device on.
//
// The RoboRIO's own bus ("rio") is read on every tick, as before
// (frc::RobotController::GetCANStatus), and always counts: the power
// distribution and pneumatics modules, which no configuration names, share
// it. Every other bus a motor or sensor names is a CANivore (its name or
// serial number, or "*"), read with Phoenix 6's CANBus::GetStatus, which
// blocks about 1 ms on the bench (Phoenix 26.3 documents "up to 1 ms"). The
// loop must not pay that, so a reader thread (CanBusSampler) reads one
// CANivore per kSamplePeriodUs, the configured buses in turn, and publishes
// each read on the configuration's Board; the loop (Poll) only copies the
// published figures, under the Board's mutex, which it try-locks, and takes
// the largest.
//
// A CANivore drops out of the maximum, until a read of it succeeds again,
// when its last read failed (a non-OK status, or a figure that is not a
// finite, non-negative fraction) or when it is stale: its last read started
// kStaleReads of its own sampling periods (StaleUs: kStaleReads * buses *
// kSamplePeriodUs) or more before the loop's tick, judged on the loop's
// clock, or it was never read and that long has passed since the loop's first
// tick under this configuration. So a hung vendor call or a starved reader
// never leaves an old figure standing as current. What the figure does not
// say: it is the busiest of the buses still read, so while a CANivore is out
// it covers only the others, and once every CANivore is out it is the RoboRIO
// bus's figure alone, which is 0 on a RoboRIO bus with nothing on it: then
// `health_valid` true with 0 reads exactly as an idle bus does. State has no
// field to say a bus dropped out; the only sign is the RoboRIO console, where
// PhoenixBackend prints each bus's failing spell, its start and its end, from
// Poll's `on_change`.
//
// Under WPILib simulation no CANivore is read (Phoenix simulates every device
// whatever its bus), so the field is the RoboRIO bus's figure alone, as it is
// for a configuration whose devices are all on "rio"; neither starts a
// reader thread.
namespace talos::rio {

class CanBusHealth {
 public:
  static constexpr uint64_t kSamplePeriodUs = 100'000;
  static constexpr uint64_t kStaleReads = 3;
  // One bus per configured CAN device at most; a bus name is at most 63
  // bytes (hardware::Validate).
  static constexpr std::size_t kMaxBuses =
      hardware::kMaxMotors + hardware::kMaxSensors;
  static constexpr std::size_t kMaxNameSize = 64;

  // One CANivore read: whether its status was OK, the utilization it reported
  // (0 to 1), and the vendor status name (a static string) for the console.
  struct Sample {
    bool ok{false};
    double utilization_frac{0.0};
    const char* status{nullptr};
  };

  enum class Event : uint8_t {
    // The bus dropped out of the maximum: a failed read, or stale.
    kFailing,
    // It reads again, ending the spell.
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
    double frac{0.0};
    uint64_t started_us{0};
    const char* status{nullptr};
    uint64_t failed_reads{0};
  };

 public:
  // One configuration's CANivores and what its reader published. Shared
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
      out.frac = good ? sample.utilization_frac : 0.0;
      out.started_us = now_us;
      out.status = sample.status;
      if (!good) ++out.failed_reads;
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
      if (name == "rio" || name.size() >= kMaxNameSize) return;
      for (std::size_t i = 0; i < count_; ++i)
        if (bus(i) == name) return;
      if (count_ == kMaxBuses) return;
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

  // Builds a new Board for `config`: each distinct bus other than "rio"
  // that a motor or sensor names, in configuration order (motors first); none
  // (no Board) when `simulation` or with no CANivore. Forgets every figure and
  // spell of the previous one, whose reader, if any, writes only into the
  // previous Board. Allocates; never runs beside Poll (Gateway reads no State
  // while it configures).
  void Configure(const hardware::Config& config, bool simulation) {
    board_.reset();
    if (!simulation) {
      auto board = std::make_shared<Board>();
      for (const auto& motor : config.motors) board->Add(motor.bus);
      for (const auto& sensor : config.sensors) board->Add(sensor.bus);
      if (board->count_ > 0) board_ = std::move(board);
    }
    snapshot_.fill({});
    spells_.fill({});
    polled_ = false;
    first_poll_us_ = 0;
  }

  // The current Board, for its reader; null with no CANivore.
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

  // The loop thread, each Read at `now_us` (steady clock), with `rio_frac`
  // the RoboRIO bus's figure this tick: copies what the reader published
  // (try-locking the Board's mutex: when the reader holds it, the previous
  // tick's copy serves), calls `on_change(Change)` for each bus that dropped
  // out of the maximum or came back, and returns the largest of `rio_frac`
  // and every bus still counted. With no CANivore it returns `rio_frac`
  // exactly. Never waits, allocates or calls into a CANivore.
  template <typename OnChange>
  double Poll(uint64_t now_us, double rio_frac, OnChange&& on_change) {
    if (!board_) return rio_frac;
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
    double result = rio_frac;
    for (std::size_t i = 0; i < count; ++i) {
      const Published& p = snapshot_[i];
      // A read stamped after this tick's clock reading is current.
      const uint64_t since = p.read ? p.started_us : first_poll_us_;
      const bool fresh = since > now_us || now_us - since < stale_us;
      const bool counted = p.read && p.ok && fresh;
      if (counted && p.frac > result) result = p.frac;
      Spell& spell = spells_[i];
      if (counted && spell.failing) {
        spell.failing = false;
        on_change(Change{i, Event::kRecovered, p.status, false,
                         p.failed_reads - spell.failed_reads_before});
      } else if (!counted && !spell.failing && (p.read || !fresh)) {
        spell.failing = true;
        const bool failed_read = p.read && !p.ok;
        spell.failed_reads_before = p.failed_reads - (failed_read ? 1 : 0);
        on_change(Change{i, Event::kFailing, p.status, !failed_read,
                         failed_read ? 1u : 0u});
      }
    }
    return result;
  }

 private:
  struct Spell {
    bool failing{false};
    uint64_t failed_reads_before{0};
  };

  std::shared_ptr<Board> board_;
  // Loop-thread state.
  std::array<Published, kMaxBuses> snapshot_{};
  std::array<Spell, kMaxBuses> spells_{};
  bool polled_{false};
  uint64_t first_poll_us_{0};
};

// The reader thread that reads the configured CANivores for CanBusHealth.
//
// Reconfigure, on PhoenixBackend's configure worker and outside its device
// lock, stops the reader, applies the new configuration, and starts a reader
// for its Board only when it has a CANivore: none under simulation or with
// only "rio" devices. The reader calls Board::SampleNext with `read`
// (Phoenix's CANBus::GetStatus), then waits `period_us` or until stopped. No
// read starts once Stop has begun, so no CANivore the new configuration
// dropped is read after Reconfigure returns, except a read already in flight.
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
// configure), and every CANivore of that configuration reads stale. An
// abandoned reader that exits is joined at the next Reconfigure or at
// destruction; one still blocked at destruction is detached.
//
// On Linux the reader names itself talos-canbus and puts itself on
// SCHED_OTHER, whatever the thread that started it, so it always runs below
// the loop's SCHED_FIFO; its nice is its starter's (0: the configure worker
// is started by the loop thread, whose SCHED_RESET_ON_FORK gives it the
// default scheduling, or before the loop raises itself). Not nice 19 like the
// logger: a starved reader goes stale, and its bus drops out of the field.
// Its own code allocates nothing once started. Phoenix's CANBus::GetStatus
// builds a std::string of the bus name (no allocation for a name of 15 bytes
// or fewer, as "Shooter") before an indirect call into the vendor's CAN
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

  // The configure worker. When the reader cannot be created no CANivore is
  // read, and Poll reports each bus stale once StaleUs has passed: a
  // configure does not fail for want of CAN health.
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
