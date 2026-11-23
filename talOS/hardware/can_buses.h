#pragma once

// The CAN bus table /hw/status carries (BridgeStatus::can_buses), assembled
// from the one bus slot each State reports in turn
// (HealthSample::can_bus). Run by hardware_node and by
// talos::testing::SimHardware when it plays the bridge, so both publish the
// same table from the same States.

#include <array>
#include <cstddef>
#include <cstdint>

#include "bridge/bridge_status_generated.h"
#include "talOS/hardware/config.h"
#include "talOS/hardware/messages.h"

// talos::hardware, as the bridge's other libraries: a talos::bridge would hide
// the BridgeStatus namespace from code inside talos.
namespace talos::hardware {

// /hw/status carries the State's read state as is.
static_assert(static_cast<uint8_t>(::bridge::CanBusReadState::NONE) ==
              static_cast<uint8_t>(CanBusReadState::kNone));
static_assert(static_cast<uint8_t>(::bridge::CanBusReadState::OK) ==
              static_cast<uint8_t>(CanBusReadState::kOk));
static_assert(static_cast<uint8_t>(::bridge::CanBusReadState::PENDING) ==
              static_cast<uint8_t>(CanBusReadState::kPending));
static_assert(static_cast<uint8_t>(::bridge::CanBusReadState::FAILED) ==
              static_cast<uint8_t>(CanBusReadState::kFailed));
static_assert(static_cast<uint8_t>(::bridge::CanBusReadState::STALE) ==
              static_cast<uint8_t>(CanBusReadState::kStale));
static_assert(static_cast<uint8_t>(::bridge::CanBusReadState::SIMULATED) ==
              static_cast<uint8_t>(CanBusReadState::kSimulated));
static_assert(::bridge::CanBusReadState::MAX ==
                  ::bridge::CanBusReadState::SIMULATED &&
              kLastCanBusReadState == CanBusReadState::kSimulated);
static_assert(kMaxCanBuses == 4,
              "BridgeStatus::can_buses holds kMaxCanBuses slots");

class CanBusTable {
 public:
  // `config_id`: the configuration the bridge pushes; only its States fill
  // the table.
  explicit CanBusTable(uint64_t config_id) : config_id_{config_id} {}

  // A State the bridge accepted (in order: the clock mapping's gate). A new
  // gateway boot, or a State of another configuration, empties the table;
  // a State of this configuration then stores the slot it reports, unless
  // that slot reads kNone (the gateway read no bus for it).
  void Observe(const State& state) {
    if (state.boot_id != boot_id_ || state.config_id != config_id_) {
      boot_id_ = state.boot_id;
      slots_.fill({});
      count_ = 0;
    }
    if (state.config_id != config_id_) return;
    const HealthSample& health = state.health;
    count_ = health.can_bus_count;
    if (health.can_bus_index < count_ &&
        health.can_bus.read_state != CanBusReadState::kNone) {
      slots_[health.can_bus_index] = health.can_bus;
    }
  }

  uint8_t count() const { return count_; }
  const CanBusSample& slot(std::size_t index) const { return slots_[index]; }

  // alert_warning_can_bus_unread: a slot below the count reads kFailed or
  // kStale.
  bool unread() const {
    for (std::size_t i = 0; i < count_; ++i) {
      const auto state = slots_[i].read_state;
      if (state == CanBusReadState::kFailed ||
          state == CanBusReadState::kStale) {
        return true;
      }
    }
    return false;
  }

  // Writes the count, the slots and the alert into `status`.
  void Fill(::bridge::BridgeStatus& status) const {
    status.mutate_can_bus_count(count_);
    status.mutate_alert_warning_can_bus_unread(unread());
    auto* out = status.mutable_can_buses();
    for (std::size_t i = 0; i < slots_.size(); ++i) {
      const CanBusSample& s = slots_[i];
      out->Mutate(static_cast<flatbuffers::uoffset_t>(i),
                  ::bridge::CanBusStatus{
                      s.utilization_frac, s.bus_off_count,
                      s.transmit_buffer_full_count, s.read_failure_count,
                      s.receive_error_count, s.transmit_error_count,
                      static_cast<::bridge::CanBusReadState>(s.read_state)});
    }
  }

 private:
  uint64_t config_id_;
  uint64_t boot_id_{0};
  uint8_t count_{0};
  std::array<CanBusSample, kMaxCanBuses> slots_{};
};

}  // namespace talos::hardware
