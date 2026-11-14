#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "hardware/config.h"

// State's CAN health (HealthSample::can_utilization_frac, 0 to 1): the
// utilization of the busiest CAN bus, among the RoboRIO's own and every
// CANivore the applied configuration has a device on.
//
// The RoboRIO's own bus ("rio") is read on every tick, as before
// (frc::RobotController::GetCANStatus), and always counts: the power
// distribution and pneumatics modules, which no configuration names, share
// it. Every other bus a motor or sensor names is a CANivore (its name or
// serial number, or "*"), read with Phoenix 6's CANBus::GetStatus, which the
// pinned Phoenix 26.3 documents as blocking for up to 1 ms. So the loop reads
// one CANivore per kSamplePeriodUs, the configured buses in turn, and holds
// each bus's last good figure between its reads: with n CANivore buses, each
// figure is at most n * kSamplePeriodUs old (plus the loop's lateness), and
// the added cost is one GetStatus call on one tick in twenty, inside
// PhoenixBackend's health phase.
//
// A read that fails (a non-OK status, or a figure that is not a finite,
// non-negative fraction) drops its bus out of the maximum until a read of it
// succeeds: the field is then the busiest of the buses still read, neither
// the failed bus's older figure nor a 0 that looks healthy. State has no field
// to say a bus dropped out; PhoenixBackend prints each failing spell's start
// and end on the RoboRIO console instead (Event).
//
// Under WPILib simulation no CANivore is read (Phoenix simulates every device
// whatever its bus), so the field is the RoboRIO bus's figure alone, as it is
// for a configuration whose devices are all on "rio".
namespace talos::rio {

class CanBusHealth {
 public:
  static constexpr uint64_t kSamplePeriodUs = 100'000;

  // One CANivore read: whether its status was OK, the utilization it reported
  // (0 to 1), and the vendor status name (a static string) for the console.
  struct Sample {
    bool ok{false};
    double utilization_frac{0.0};
    const char* status{nullptr};
  };

  enum class Event : uint8_t {
    kNone,
    // The bus's first failed read since it last read well (or since the
    // configuration was applied).
    kFailing,
    // A good read ending a failing spell.
    kRecovered,
  };

  struct Result {
    double utilization_frac{0.0};
    // What this call's read changed, for the bus at index `bus`, with the
    // read's status and the spell's failed reads (kRecovered: all of them).
    Event event{Event::kNone};
    std::size_t bus{0};
    const char* status{nullptr};
    uint32_t spell_failures{0};
  };

  // Rebuilds the bus set from `config`: each distinct bus other than "rio"
  // that a motor or sensor names, in configuration order (motors first); none
  // when `simulation`. Forgets every held figure and spell, so the next Poll
  // reads the first bus. Allocates: the configure worker calls it, while the
  // loop does not Poll (Gateway reads no State while it configures).
  void Configure(const hardware::Config& config, bool simulation) {
    buses_.clear();
    next_ = 0;
    read_once_ = false;
    last_read_us_ = 0;
    if (simulation) return;
    buses_.reserve(config.motors.size() + config.sensors.size());
    const auto add = [this](const std::string& name) {
      if (name == "rio") return;
      for (const auto& bus : buses_)
        if (bus.name == name) return;
      Bus bus;
      bus.name = name;
      buses_.push_back(std::move(bus));
    };
    for (const auto& motor : config.motors) add(motor.bus);
    for (const auto& sensor : config.sensors) add(sensor.bus);
  }

  // The loop thread's Read at `now_us` (steady clock), with `rio_frac` the
  // RoboRIO bus's figure this tick. When no read was made since Configure,
  // or kSamplePeriodUs has passed since the last, calls `read(bus_name)`
  // (returning Sample) for the next bus in turn: at most one read per call.
  // Returns the largest of `rio_frac` and every bus's held figure. Never
  // allocates.
  template <typename ReadBus>
  Result Poll(uint64_t now_us, double rio_frac, ReadBus&& read) {
    Result result;
    if (!buses_.empty() &&
        (!read_once_ || now_us - last_read_us_ >= kSamplePeriodUs)) {
      read_once_ = true;
      last_read_us_ = now_us;
      const std::size_t index = next_;
      next_ = (next_ + 1) % buses_.size();
      Bus& bus = buses_[index];
      const Sample sample = read(std::string_view{bus.name});
      const bool good = sample.ok && std::isfinite(sample.utilization_frac) &&
                        sample.utilization_frac >= 0.0;
      result.bus = index;
      result.status = sample.status;
      if (good) {
        bus.held = true;
        bus.frac = sample.utilization_frac;
        if (bus.failing) {
          result.event = Event::kRecovered;
          result.spell_failures = bus.spell_failures;
        }
        bus.failing = false;
      } else {
        bus.held = false;
        if (!bus.failing) {
          bus.failing = true;
          bus.spell_failures = 0;
          result.event = Event::kFailing;
        }
        ++bus.spell_failures;
        result.spell_failures = bus.spell_failures;
      }
    }
    result.utilization_frac = rio_frac;
    for (const auto& bus : buses_)
      if (bus.held && bus.frac > result.utilization_frac)
        result.utilization_frac = bus.frac;
    return result;
  }

  std::size_t bus_count() const { return buses_.size(); }
  std::string_view bus(std::size_t index) const { return buses_[index].name; }

 private:
  struct Bus {
    std::string name;
    bool held{false};
    double frac{0.0};
    bool failing{false};
    uint32_t spell_failures{0};
  };
  std::vector<Bus> buses_;
  std::size_t next_{0};
  bool read_once_{false};
  uint64_t last_read_us_{0};
};

}  // namespace talos::rio
