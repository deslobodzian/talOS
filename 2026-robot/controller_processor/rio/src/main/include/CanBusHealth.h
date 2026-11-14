#pragma once

#include <array>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <string_view>
#include <thread>

#include "hardware/config.h"

// State's CAN health (HealthSample::can_utilization_frac, 0 to 1): the
// utilization of the busiest CAN bus, among the RoboRIO's own and every
// CANivore the applied configuration has a device on.
//
// The RoboRIO's own bus ("rio") is read on every tick, as before
// (frc::RobotController::GetCANStatus), and always counts: the power
// distribution and pneumatics modules, which no configuration names, share
// it. Every other bus a motor or sensor names is a CANivore (its name or
// serial number, or "*"), read with Phoenix 6's CANBus::GetStatus, which
// blocks about 1 ms on the bench (Phoenix 26.3 documents "up to 1 ms"). The
// loop must not pay that, so a sampler thread (CanBusSampler) reads one
// CANivore per kSamplePeriodUs, the configured buses in turn, and publishes
// each read; the loop (Poll) only copies the published figures, under a mutex
// it try-locks, and takes the largest.
//
// A bus drops out of the maximum, until a read of it succeeds again, when its
// last read failed (a non-OK status, or a figure that is not a finite,
// non-negative fraction) or when it is stale: its last read started
// kStaleReads of its own sampling periods (StaleUs: kStaleReads * buses *
// kSamplePeriodUs) or more before the loop's tick, judged on the loop's
// clock, or it was never read and that long has passed since the loop's first
// tick under this configuration. So a hung vendor call or a starved sampler
// never leaves an old figure standing as current, and the field is the
// busiest of the buses still read: neither a dropped bus's older figure nor
// a 0 that looks healthy. State has no field to say a bus dropped out; the
// loop reports each bus's failing spell, its start and its end, through Poll's
// `on_change` (PhoenixBackend prints them on the RoboRIO console).
//
// Under WPILib simulation no CANivore is read (Phoenix simulates every device
// whatever its bus), so the field is the RoboRIO bus's figure alone, as it is
// for a configuration whose devices are all on "rio"; neither starts a
// sampler thread.
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

  // Rebuilds the bus set from `config`: each distinct bus other than "rio"
  // that a motor or sensor names, in configuration order (motors first); none
  // when `simulation`. Forgets every published figure and spell; a read still
  // in flight for the old set publishes nothing. Never runs beside Poll
  // (Gateway reads no State while it configures); CanBusSampler::Reconfigure
  // also stops its thread first.
  void Configure(const hardware::Config& config, bool simulation) {
    const std::lock_guard lock{mutex_};
    ++generation_;
    count_ = 0;
    next_ = 0;
    published_.fill({});
    if (!simulation) {
      for (const auto& motor : config.motors) Add(motor.bus);
      for (const auto& sensor : config.sensors) Add(sensor.bus);
    }
    snapshot_.fill({});
    spells_.fill({});
    polled_ = false;
    first_poll_us_ = 0;
  }

  // Fixed between Configure calls.
  std::size_t bus_count() const { return count_; }
  std::string_view bus(std::size_t index) const {
    return {names_[index].data(), sizes_[index]};
  }
  // The age at which a bus's last read no longer counts.
  uint64_t StaleUs() const { return kStaleReads * count_ * kSamplePeriodUs; }

  // The sampler thread: reads the next bus in turn, `now_us` (steady clock)
  // being the read's start, with `read(bus_name)` (returning Sample), and
  // publishes it unless a Configure replaced the bus set meanwhile. The mutex
  // is held only to pick the bus and to publish, never across `read`, which
  // gets a copy of the name. Does nothing with no bus. Never allocates.
  template <typename ReadBus>
  void SampleNext(uint64_t now_us, ReadBus&& read) {
    std::array<char, kMaxNameSize> name{};
    std::size_t size = 0;
    std::size_t index = 0;
    uint64_t generation = 0;
    {
      const std::lock_guard lock{mutex_};
      if (count_ == 0) return;
      index = next_;
      next_ = (next_ + 1) % count_;
      generation = generation_;
      size = sizes_[index];
      std::memcpy(name.data(), names_[index].data(), size);
    }
    const Sample sample = read(std::string_view{name.data(), size});
    const bool good = sample.ok && std::isfinite(sample.utilization_frac) &&
                      sample.utilization_frac >= 0.0;
    const std::lock_guard lock{mutex_};
    if (generation != generation_) return;
    Published& out = published_[index];
    out.read = true;
    out.ok = good;
    out.frac = good ? sample.utilization_frac : 0.0;
    out.started_us = now_us;
    out.status = sample.status;
    if (!good) ++out.failed_reads;
  }

  // The loop thread, each Read at `now_us` (steady clock), with `rio_frac`
  // the RoboRIO bus's figure this tick: copies what the sampler published
  // (try-locking the mutex: when the sampler holds it, the previous tick's
  // copy serves), calls `on_change(Change)` for each bus that dropped out of
  // the maximum or came back, and returns the largest of `rio_frac` and every
  // bus still counted. With no CANivore it returns `rio_frac` exactly. Never
  // waits, allocates or calls into a CANivore.
  template <typename OnChange>
  double Poll(uint64_t now_us, double rio_frac, OnChange&& on_change) {
    if (count_ == 0) return rio_frac;
    {
      const std::unique_lock lock{mutex_, std::try_to_lock};
      if (lock.owns_lock())
        for (std::size_t i = 0; i < count_; ++i) snapshot_[i] = published_[i];
    }
    if (!polled_) {
      polled_ = true;
      first_poll_us_ = now_us;
    }
    const uint64_t stale_us = StaleUs();
    double result = rio_frac;
    for (std::size_t i = 0; i < count_; ++i) {
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
  struct Published {
    bool read{false};
    bool ok{false};
    double frac{0.0};
    uint64_t started_us{0};
    const char* status{nullptr};
    uint64_t failed_reads{0};
  };
  struct Spell {
    bool failing{false};
    uint64_t failed_reads_before{0};
  };

  // Configure only, under the mutex.
  void Add(std::string_view name) {
    if (name == "rio" || name.size() >= kMaxNameSize) return;
    for (std::size_t i = 0; i < count_; ++i)
      if (bus(i) == name) return;
    if (count_ == kMaxBuses) return;
    std::memcpy(names_[count_].data(), name.data(), name.size());
    sizes_[count_] = name.size();
    ++count_;
  }

  // The bus set: written by Configure, under the mutex; read by Poll and
  // bus() without it (Configure never runs beside them) and by SampleNext
  // under it.
  std::array<std::array<char, kMaxNameSize>, kMaxBuses> names_{};
  std::array<std::size_t, kMaxBuses> sizes_{};
  std::size_t count_{0};

  // Shared with the sampler thread, under the mutex.
  std::mutex mutex_;
  uint64_t generation_{0};
  std::size_t next_{0};
  std::array<Published, kMaxBuses> published_{};

  // Loop-thread state.
  std::array<Published, kMaxBuses> snapshot_{};
  std::array<Spell, kMaxBuses> spells_{};
  bool polled_{false};
  uint64_t first_poll_us_{0};
};

// The thread that reads the configured CANivores for CanBusHealth.
//
// Reconfigure, on PhoenixBackend's configure worker, stops and joins the
// thread, applies the new bus set, and starts a new thread only when that set
// has a CANivore: none under simulation or with only "rio" devices. The
// thread calls CanBusHealth::SampleNext with `read` (Phoenix's
// CANBus::GetStatus) and then waits `period_us` or until stopped. So no
// CANivore the new configuration dropped is read after Reconfigure returns,
// and the bus list is never replaced under a read; a read in flight when it
// is called (it cannot be interrupted) delays the join by that read, about
// 1 ms, and publishes nothing for the new set. Stop (also the destructor)
// joins the thread, so it waits for a vendor call that never returns: until a
// configure, the stale bound keeps that bus out of the field; a configure
// then never completes, and the gateway stays unconfigured, driving nothing.
// Leaving the thread unjoined instead would let it write into a bus set or
// backend already gone.
//
// On Linux the thread names itself talos-canbus and puts itself on
// SCHED_OTHER, whatever the thread that started it, so it always runs below
// the loop's SCHED_FIFO; its nice is its starter's (0: the configure worker
// is started by the loop thread, whose SCHED_RESET_ON_FORK gives it the
// default scheduling, or before the loop raises itself). Not nice 19 like the
// logger: a starved sampler goes stale, and its bus drops out of the field.
// Its own code allocates nothing once started; whether Phoenix's GetStatus
// allocates (an allocator lock the loop could then wait on, ConfigStore.h)
// is not known.
class CanBusSampler {
 public:
  using Read = std::function<CanBusHealth::Sample(std::string_view bus)>;
  // Steady-clock microseconds, the clock of PhoenixBackend's Read.
  using Clock = uint64_t (*)();

  CanBusSampler(CanBusHealth& health, Read read, Clock clock,
                uint64_t period_us = CanBusHealth::kSamplePeriodUs);
  ~CanBusSampler();
  CanBusSampler(const CanBusSampler&) = delete;
  CanBusSampler& operator=(const CanBusSampler&) = delete;

  // The configure worker. When the thread cannot be created no CANivore is
  // read, and Poll reports each bus stale once StaleUs has passed: a
  // configure does not fail for want of CAN health.
  void Reconfigure(const hardware::Config& config, bool simulation);
  void Stop();
  bool running() const { return thread_.joinable(); }

 private:
  void Run();

  CanBusHealth& health_;
  const Read read_;
  const Clock clock_;
  const uint64_t period_us_;
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stop_{false};
  std::thread thread_;
};

}  // namespace talos::rio
